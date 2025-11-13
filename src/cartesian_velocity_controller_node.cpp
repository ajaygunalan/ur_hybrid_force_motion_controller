#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include <stdexcept>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <rclcpp/parameter_client.hpp>
#include <urdf/model.h>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

using namespace std::chrono_literals;

namespace hfmc {

namespace {

KDL::JntArray ToJntArray(const std::vector<double>& joints) {
  KDL::JntArray array(joints.size());
  for (size_t i = 0; i < joints.size(); ++i) {
    array(i) = joints[i];
  }
  return array;
}

bool BuildKinematics(const urdf::Model& urdf_model,
                     const std::string& base_link,
                     const std::string& tool_link,
                     std::string& error_message,
                     KDL::Tree& tree_out,
                     KDL::Chain& chain_out,
                     size_t& joint_count,
                     KDL::Frame& wrist_to_tool,
                     std::unique_ptr<KDL::ChainFkSolverPos_recursive>& fk_solver,
                     std::unique_ptr<KDL::ChainIkSolverVel_wdls>& ik_solver) {
  if (!kdl_parser::treeFromUrdfModel(urdf_model, tree_out)) {
    error_message = "URDF to KDL tree conversion failed";
    return false;
  }

  if (!tree_out.getChain(base_link, "wrist_3_link", chain_out)) {
    error_message = "KDL chain " + base_link + " -> wrist_3_link not found";
    return false;
  }

  joint_count = chain_out.getNrOfJoints();
  if (joint_count == 0) {
    error_message = "KDL chain has zero joints";
    return false;
  }

  fk_solver = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_out);
  ik_solver = std::make_unique<KDL::ChainIkSolverVel_wdls>(chain_out, 1e-5, 150);
  ik_solver->setLambda(0.1);

  wrist_to_tool = KDL::Frame::Identity();
  KDL::Chain tool_chain;
  if (tree_out.getChain("wrist_3_link", tool_link, tool_chain)) {
    for (unsigned int i = 0; i < tool_chain.getNrOfSegments(); ++i) {
      wrist_to_tool = wrist_to_tool * tool_chain.getSegment(i).getFrameToTip();
    }
  }
  return true;
}

Eigen::Matrix<double, 6, 1> BuildWristTwist(const Eigen::Matrix<double, 6, 1>& tool_twist,
                                            const Eigen::Vector3d& wrist_offset) {
  Eigen::Matrix<double, 6, 1> wrist_twist = tool_twist;
  wrist_twist.head<3>() -= wrist_twist.tail<3>().cross(wrist_offset);
  return wrist_twist;
}

Eigen::Matrix3d KdlRotationToEigen(const KDL::Rotation& rot) {
  double x, y, z, w;
  rot.GetQuaternion(x, y, z, w);
  Eigen::Quaterniond q(w, x, y, z);
  return q.toRotationMatrix();
}

bool ComputeForwardKinematics(const std::vector<double>& joints,
                              KDL::ChainFkSolverPos_recursive* solver,
                              const KDL::Frame& wrist_to_tool,
                              Eigen::Isometry3d& pose_out) {
  if (!solver) return false;
  auto q = ToJntArray(joints);
  KDL::Frame frame;
  if (solver->JntToCart(q, frame) < 0) {
    return false;
  }
  frame = frame * wrist_to_tool;
  pose_out = Eigen::Isometry3d::Identity();
  pose_out.translation() = Eigen::Vector3d(frame.p.x(), frame.p.y(), frame.p.z());
  pose_out.linear() = KdlRotationToEigen(frame.M);
  return true;
}

bool ComputeJointVelocity(const Eigen::Matrix<double, 6, 1>& twist_world,
                          const std::vector<double>& joints,
                          KDL::ChainIkSolverVel_wdls* solver,
                          std::vector<double>& qdot_out) {
  if (!solver) return false;

  KDL::JntArray q = ToJntArray(joints);
  KDL::Twist twist;
  twist.vel = KDL::Vector(twist_world(0), twist_world(1), twist_world(2));
  twist.rot = KDL::Vector(twist_world(3), twist_world(4), twist_world(5));

  KDL::JntArray qdot(joints.size());
  if (solver->CartToJnt(q, twist, qdot) < 0) {
    return false;
  }

  qdot_out.resize(joints.size());
  for (size_t i = 0; i < joints.size(); ++i) {
    double value = qdot(i);
    if (std::isnan(value) || std::isinf(value)) {
      return false;
    }
    qdot_out[i] = value;
  }
  return true;
}

void LimitTwist(Eigen::Matrix<double, 6, 1>& twist,
                double max_linear,
                double max_angular) {
  double lin_norm = twist.head<3>().norm();
  if (max_linear > 0.0 && lin_norm > max_linear && lin_norm > 1e-6) {
    twist.head<3>() *= (max_linear / lin_norm);
  }
  double ang_norm = twist.tail<3>().norm();
  if (max_angular > 0.0 && ang_norm > max_angular && ang_norm > 1e-6) {
    twist.tail<3>() *= (max_angular / ang_norm);
  }
}

void ClampVectorInPlace(std::vector<double>& values,
                        const std::vector<double>& limits) {
  for (size_t i = 0; i < values.size(); ++i) {
    double limit = (i < limits.size()) ? limits[i] : limits.back();
    values[i] = std::clamp(values[i], -limit, limit);
  }
}

}  // namespace

class CartesianVelocityControllerNode : public rclcpp::Node {
public:
  CartesianVelocityControllerNode()
  : Node("cartesian_velocity_controller") {
    DeclareParameters();
    InitializeKinematics();
    SetupInterfaces();
    RCLCPP_INFO(get_logger(), "Cartesian velocity controller ready (control=%.1f Hz)",
                control_frequency_hz_);
  }

private:
  void DeclareParameters() {
    control_frequency_hz_ = declare_parameter("control_frequency_hz", 250.0);
    control_period_s_ = 1.0 / std::max(1.0, control_frequency_hz_);

    base_link_ = declare_parameter<std::string>("base_link", "base_link");
    tool_link_ = declare_parameter<std::string>("tool_link", "p42v_link1");

    joint_names_ = declare_parameter<std::vector<std::string>>(
        "joint_names",
        {"shoulder_pan_joint","shoulder_lift_joint","elbow_joint",
         "wrist_1_joint","wrist_2_joint","wrist_3_joint"});

    linear_limit_ = declare_parameter("velocity_limits.linear_mps", 0.05);
    angular_limit_ = declare_parameter("velocity_limits.angular_radps", 0.5);
    joint_velocity_limits_ = declare_parameter<std::vector<double>>(
        "joint_velocity_limits", std::vector<double>(joint_names_.size(), 0.5));
    if (joint_velocity_limits_.size() != joint_names_.size()) {
      RCLCPP_WARN(get_logger(),
                  "joint_velocity_limits size (%zu) differs from joint_names (%zu); "
                  "using first value for all joints",
                  joint_velocity_limits_.size(), joint_names_.size());
      joint_velocity_limits_.assign(joint_names_.size(),
                                    joint_velocity_limits_.empty()
                                        ? 0.5
                                        : joint_velocity_limits_[0]);
    }

    twist_timeout_s_ = declare_parameter("twist_timeout_s", 0.1);
    twist_topic_ = declare_parameter<std::string>("twist_topic",
                                                  "/hybrid_force_motion_controller/twist_cmd");
  }

  void InitializeKinematics() {
    auto client = std::make_shared<rclcpp::SyncParametersClient>(
        this, "/robot_state_publisher");
    if (!client->wait_for_service(10s)) {
      RCLCPP_FATAL(get_logger(), "robot_state_publisher param service unavailable");
      throw std::runtime_error("parameter service missing");
    }
    auto params = client->get_parameters({"robot_description"});
    if (params.empty() || params[0].get_type() == rclcpp::PARAMETER_NOT_SET) {
      RCLCPP_FATAL(get_logger(), "robot_description not set on /robot_state_publisher");
      throw std::runtime_error("robot_description missing");
    }
    std::string urdf_string = params[0].as_string();
    urdf::Model model;
    if (!model.initString(urdf_string)) {
      RCLCPP_FATAL(get_logger(), "Failed to parse robot_description");
      throw std::runtime_error("urdf parse failed");
    }

    std::string err;
    if (!BuildKinematics(model, base_link_, tool_link_, err,
                         kdl_tree_, kdl_chain_, joint_count_, wrist_to_tool_,
                         fk_solver_, ik_solver_)) {
      RCLCPP_FATAL(get_logger(), "Kinematics init failed: %s", err.c_str());
      throw std::runtime_error("kinematics failure");
    }

    wrist_to_tool_translation_ = Eigen::Vector3d(
        wrist_to_tool_.p.x(), wrist_to_tool_.p.y(), wrist_to_tool_.p.z());
    wrist_to_tool_rotation_ = KdlRotationToEigen(wrist_to_tool_.M);

    joint_positions_.assign(joint_count_, 0.0);
    qdot_buffer_.assign(joint_count_, 0.0);
  }

  void SetupInterfaces() {
    twist_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
        twist_topic_, 10,
        std::bind(&CartesianVelocityControllerNode::OnTwist, this, std::placeholders::_1));

    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 20,
        std::bind(&CartesianVelocityControllerNode::OnJointState, this, std::placeholders::_1));

    velocity_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
        "/forward_velocity_controller/commands", 10);

    control_timer_ = create_wall_timer(
        std::chrono::duration<double>(control_period_s_),
        std::bind(&CartesianVelocityControllerNode::ControlLoop, this));

    velocity_msg_.layout.dim.resize(1);
    velocity_msg_.layout.dim[0].label = "joints";
    velocity_msg_.layout.dim[0].size = joint_count_;
    velocity_msg_.layout.dim[0].stride = joint_count_;
    velocity_msg_.data.assign(joint_count_, 0.0);
  }

  void OnTwist(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    last_twist_msg_ = *msg;
    twist_ready_ = true;
    last_twist_time_ = now();
  }

  void OnJointState(const sensor_msgs::msg::JointState::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!joint_map_ready_) {
      ResolveJointIndices(*msg);
      if (!joint_map_ready_) return;
    }
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      const auto idx = joint_state_index_[i];
      if (idx >= msg->position.size()) continue;
      joint_positions_[i] = msg->position[idx];
    }
    joint_ready_ = true;
  }

  void ControlLoop() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!joint_ready_ || !twist_ready_ || !fk_solver_) {
      PublishZeroVelocity();
      return;
    }

    if ((now() - last_twist_time_).seconds() > twist_timeout_s_) {
      PublishZeroVelocity();
      return;
    }

    UpdateCurrentPose();

    Eigen::Matrix<double, 6, 1> twist;
    twist << last_twist_msg_.twist.linear.x,
             last_twist_msg_.twist.linear.y,
             last_twist_msg_.twist.linear.z,
             last_twist_msg_.twist.angular.x,
             last_twist_msg_.twist.angular.y,
             last_twist_msg_.twist.angular.z;
    LimitTwist(twist, linear_limit_, angular_limit_);

    Eigen::Vector3d wrist_offset = WristOffsetWorld();
    Eigen::Matrix<double, 6, 1> wrist_twist = BuildWristTwist(twist, wrist_offset);

    if (!ComputeJointVelocity(wrist_twist, joint_positions_, ik_solver_.get(), qdot_buffer_)) {
      PublishZeroVelocity();
      return;
    }
    ClampVectorInPlace(qdot_buffer_, joint_velocity_limits_);
    PublishVelocity(qdot_buffer_);
  }

  Eigen::Vector3d WristOffsetWorld() const {
    Eigen::Matrix3d R_tool = current_pose_.linear();
    Eigen::Matrix3d R_wrist = R_tool * wrist_to_tool_rotation_.transpose();
    return R_wrist * wrist_to_tool_translation_;
  }

  void PublishVelocity(const std::vector<double>& qdot) {
    velocity_msg_.data = qdot;
    velocity_pub_->publish(velocity_msg_);
  }

  void PublishZeroVelocity() {
    velocity_msg_.data.assign(joint_count_, 0.0);
    velocity_pub_->publish(velocity_msg_);
  }

  void ResolveJointIndices(const sensor_msgs::msg::JointState& msg) {
    joint_state_index_.assign(joint_names_.size(), 0);
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      const auto& name = joint_names_[i];
      auto it = std::find(msg.name.begin(), msg.name.end(), name);
      if (it == msg.name.end()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "Joint %s missing in /joint_states", name.c_str());
        joint_map_ready_ = false;
        return;
      }
      joint_state_index_[i] = static_cast<size_t>(std::distance(msg.name.begin(), it));
    }
    joint_map_ready_ = true;
  }

  void UpdateCurrentPose() {
    if (!ComputeForwardKinematics(joint_positions_, fk_solver_.get(),
                                  wrist_to_tool_, current_pose_)) {
      current_pose_ = Eigen::Isometry3d::Identity();
    }
  }

  double control_frequency_hz_{250.0};
  double control_period_s_{0.004};
  std::string base_link_{"base_link"};
  std::string tool_link_{"p42v_link1"};

  std::vector<std::string> joint_names_;
  std::vector<double> joint_positions_;
  std::vector<size_t> joint_state_index_;
  std::vector<double> joint_velocity_limits_;
  std::vector<double> qdot_buffer_;
  bool joint_ready_{false};
  bool joint_map_ready_{false};

  double linear_limit_{0.05};
  double angular_limit_{0.5};
  double twist_timeout_s_{0.1};
  std::string twist_topic_{"/hybrid_force_motion_controller/twist_cmd"};

  geometry_msgs::msg::TwistStamped last_twist_msg_;
  bool twist_ready_{false};
  rclcpp::Time last_twist_time_;

  Eigen::Isometry3d current_pose_{Eigen::Isometry3d::Identity()};
  KDL::Tree kdl_tree_;
  KDL::Chain kdl_chain_;
  size_t joint_count_{0};
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
  std::unique_ptr<KDL::ChainIkSolverVel_wdls> ik_solver_;
  KDL::Frame wrist_to_tool_{KDL::Frame::Identity()};
  Eigen::Vector3d wrist_to_tool_translation_{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d wrist_to_tool_rotation_{Eigen::Matrix3d::Identity()};

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  std_msgs::msg::Float64MultiArray velocity_msg_;

  std::mutex data_mutex_;
};

}  // namespace hfmc

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<hfmc::CartesianVelocityControllerNode>();
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("cartesian_velocity_controller"),
                 "Fatal error: %s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
