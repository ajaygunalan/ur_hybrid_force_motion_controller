#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include <iterator>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>

#include "hybrid_force_motion_controller/msg/hybrid_force_motion_state.hpp"

namespace hfmc {

using StateMsg = hybrid_force_motion_controller::msg::HybridForceMotionState;
using namespace std::chrono_literals;

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
  double x, y, z, w;
  frame.M.GetQuaternion(x, y, z, w);
  Eigen::Quaterniond q_frame(w, x, y, z);
  q_frame.normalize();
  pose_out.linear() = q_frame.toRotationMatrix();
  return true;
}

Eigen::Matrix3d KdlRotationToEigen(const KDL::Rotation& rot) {
  double x, y, z, w;
  rot.GetQuaternion(x, y, z, w);
  Eigen::Quaterniond q(w, x, y, z);
  return q.toRotationMatrix();
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

Eigen::Vector3d NormalizeOrFallback(const Eigen::Vector3d& v,
                                    const Eigen::Vector3d& fallback) {
  if (v.norm() < 1e-6) {
    return fallback.normalized();
  }
  return v.normalized();
}

Eigen::Vector3d ProjectOntoPlane(const Eigen::Vector3d& v,
                                 const Eigen::Vector3d& normal) {
  return v - normal * (v.dot(normal));
}

void ClampVectorInPlace(std::vector<double>& values,
                        const std::vector<double>& limits) {
  for (size_t i = 0; i < values.size(); ++i) {
    double limit = (i < limits.size()) ? limits[i] : limits.back();
    values[i] = std::clamp(values[i], -limit, limit);
  }
}

}  // namespace

enum class RunState : uint8_t {
  WAITING_FOR_START = 0,
  READY,
  RUNNING,
  PAUSED,
  COMPLETED,
  ABORTED,
  FAULT
};

enum class Phase : uint8_t {
  SEEK = 0,
  DWELL,
  TANGENTIAL
};

class HybridForceMotionNode : public rclcpp::Node {
public:
  HybridForceMotionNode()
  : Node("hybrid_force_motion_controller"),
    state_(RunState::WAITING_FOR_START),
    phase_(Phase::SEEK) {
    DeclareAndLoadParameters();
    InitializeKinematics();
    SetupInterfaces();
    RCLCPP_INFO(get_logger(), "Hybrid force-motion controller ready (control=%.1f Hz)",
                control_frequency_hz_);
  }

private:
  // ---- parameter handling ----
  void DeclareAndLoadParameters() {
    control_frequency_hz_ = declare_parameter("control_frequency_hz", 250.0);
    control_period_s_ = 1.0 / std::max(1.0, control_frequency_hz_);

    base_link_ = declare_parameter<std::string>("base_link", "base_link");
    tool_link_ = declare_parameter<std::string>("tool_link", "p42v_link1");

    joint_names_ = declare_parameter<std::vector<std::string>>(
        "joint_names",
        {"shoulder_pan_joint","shoulder_lift_joint","elbow_joint",
         "wrist_1_joint","wrist_2_joint","wrist_3_joint"});

    auto search = declare_parameter<std::vector<double>>(
        "search_direction", {0.0, 0.0, -1.0});
    search_direction_ = NormalizeOrFallback(
        Eigen::Vector3d(search[0], search[1], search[2]),
        Eigen::Vector3d(0.0, 0.0, -1.0));
    contact_normal_ = search_direction_;

    auto hint = declare_parameter<std::vector<double>>(
        "tangential_direction_hint", {1.0, 0.0, 0.0});
    direction_hint_ = NormalizeOrFallback(
        Eigen::Vector3d(hint[0], hint[1], hint[2]),
        Eigen::Vector3d(1.0, 0.0, 0.0));
    tangential_dir_ = direction_hint_;

    tangential_target_ = declare_parameter("tangential_distance_m", 0.05);
    tangential_gain_ = declare_parameter("tangential_gain", 1.0);
    tangential_speed_limit_ = declare_parameter("tangential_speed_mps", 0.01);
    dwell_time_s_ = declare_parameter("dwell_time_s", 1.0);

    normal_target_ = declare_parameter("normal.target_N", 5.0);
    normal_tolerance_ = declare_parameter("normal.tolerance_N", 0.2);
    normal_kp_ = declare_parameter("normal.kp", 0.6);
    normal_ki_ = declare_parameter("normal.ki", 4.0);
    normal_max_velocity_ = declare_parameter("normal.max_velocity_mps", 0.01);
    softstart_time_s_ = declare_parameter("normal.softstart_time_s", 0.7);

    disengage_threshold_ = declare_parameter("disengage_force_threshold_N", 2.0);
    disengage_count_limit_ = declare_parameter("disengage_count", 30);

    estimator_enabled_ = declare_parameter("estimator.enable", true);
    estimator_alpha_ = declare_parameter("estimator.alpha", 0.15);
    estimator_min_speed_ = declare_parameter("estimator.min_speed_mps", 0.001);

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

    publish_contact_frame_ = declare_parameter("publish_contact_frame", true);
    contact_frame_id_ = declare_parameter<std::string>("contact_frame_id", "contact_frame");
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
    last_twist_cmd_.setZero();
    RCLCPP_INFO(get_logger(), "Loaded KDL chain %s -> %s (%zu joints)",
                base_link_.c_str(), tool_link_.c_str(), joint_count_);
  }

  void SetupInterfaces() {
    auto sensor_qos = rclcpp::SensorDataQoS();
    wrench_sub_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
        "/netft/proc_probe", sensor_qos,
        std::bind(&HybridForceMotionNode::OnWrench, this, std::placeholders::_1));
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 20,
        std::bind(&HybridForceMotionNode::OnJointState, this, std::placeholders::_1));
    direction_sub_ = create_subscription<geometry_msgs::msg::Vector3>(
        "~/direction", 1,
        std::bind(&HybridForceMotionNode::OnDirectionHint, this, std::placeholders::_1));

    velocity_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
        "/forward_velocity_controller/commands", 10);
    state_pub_ = create_publisher<StateMsg>("~/state", 10);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    set_start_srv_ = create_service<std_srvs::srv::Trigger>(
        "~/set_start_pose",
        std::bind(&HybridForceMotionNode::OnSetStartPose, this,
                  std::placeholders::_1, std::placeholders::_2));
    start_srv_ = create_service<std_srvs::srv::Trigger>(
        "~/start_motion",
        std::bind(&HybridForceMotionNode::OnStartMotion, this,
                  std::placeholders::_1, std::placeholders::_2));
    pause_srv_ = create_service<std_srvs::srv::Trigger>(
        "~/pause_motion",
        std::bind(&HybridForceMotionNode::OnPauseMotion, this,
                  std::placeholders::_1, std::placeholders::_2));
    resume_srv_ = create_service<std_srvs::srv::Trigger>(
        "~/resume_motion",
        std::bind(&HybridForceMotionNode::OnResumeMotion, this,
                  std::placeholders::_1, std::placeholders::_2));
    stop_srv_ = create_service<std_srvs::srv::Trigger>(
        "~/stop_motion",
        std::bind(&HybridForceMotionNode::OnStopMotion, this,
                  std::placeholders::_1, std::placeholders::_2));

    control_timer_ = create_wall_timer(
        std::chrono::duration<double>(control_period_s_),
        std::bind(&HybridForceMotionNode::ControlLoop, this));

    velocity_msg_.layout.dim.resize(1);
    velocity_msg_.layout.dim[0].label = "joints";
    velocity_msg_.layout.dim[0].size = joint_count_;
    velocity_msg_.layout.dim[0].stride = joint_count_;
    velocity_msg_.data.assign(joint_count_, 0.0);
  }

  // ---- callbacks ----
  void OnWrench(const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    force_probe_ = Eigen::Vector3d(msg->wrench.force.x,
                                   msg->wrench.force.y,
                                   msg->wrench.force.z);
    wrench_ready_ = true;
  }

  void OnJointState(const sensor_msgs::msg::JointState::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!joint_map_ready_) {
      ResolveJointIndices(*msg);
      if (!joint_map_ready_) {
        return;
      }
    }

    for (size_t i = 0; i < joint_names_.size(); ++i) {
      const auto idx = joint_state_index_[i];
      if (idx >= msg->position.size()) continue;
      joint_positions_[i] = msg->position[idx];
    }
    joint_ready_ = true;
  }

  void OnDirectionHint(const geometry_msgs::msg::Vector3::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    Eigen::Vector3d candidate(msg->x, msg->y, msg->z);
    if (candidate.norm() < 1e-6) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Received zero direction hint; ignoring");
      return;
    }
    direction_hint_ = candidate.normalized();
    tangential_dir_ = ProjectAndNormalize(direction_hint_, contact_normal_);
    RCLCPP_INFO(get_logger(), "Updated tangential direction hint to [%.2f %.2f %.2f]",
                tangential_dir_.x(), tangential_dir_.y(), tangential_dir_.z());
  }

  // ---- service handlers ----
  void OnSetStartPose(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                      std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!joint_ready_) {
      res->success = false;
      res->message = "Joint states not received yet";
      return;
    }

    Eigen::Isometry3d pose;
    if (!ComputeForwardKinematics(joint_positions_, fk_solver_.get(), wrist_to_tool_, pose)) {
      res->success = false;
      res->message = "FK failed";
      return;
    }

    start_pose_ = pose;
    contact_normal_ = search_direction_;
    tangential_dir_ = ProjectAndNormalize(direction_hint_, contact_normal_);

    tangential_distance_ = 0.0;
    dwell_elapsed_ = 0.0;
    normal_integral_ = 0.0;
    softstart_elapsed_ = 0.0;
    softstart_active_ = true;
    disengage_counter_ = 0;
    fault_reason_.clear();

    state_ = RunState::READY;
    phase_ = Phase::SEEK;

    res->success = true;
    res->message = "Start pose captured";
  }

  void OnStartMotion(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                     std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (state_ != RunState::READY) {
      res->success = false;
      res->message = "Controller not READY";
      return;
    }
    state_ = RunState::RUNNING;
    phase_ = Phase::SEEK;
    normal_integral_ = 0.0;
    softstart_elapsed_ = 0.0;
    softstart_active_ = true;
    dwell_elapsed_ = 0.0;
    tangential_distance_ = 0.0;
    res->success = true;
    res->message = "RUNNING";
  }

  void OnPauseMotion(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                     std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (state_ != RunState::RUNNING) {
      res->success = false;
      res->message = "Controller not RUNNING";
      return;
    }
    state_ = RunState::PAUSED;
    res->success = true;
    res->message = "PAUSED";
  }

  void OnResumeMotion(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                      std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (state_ != RunState::PAUSED) {
      res->success = false;
      res->message = "Controller not PAUSED";
      return;
    }
    state_ = RunState::RUNNING;
    res->success = true;
    res->message = "RUNNING";
  }

  void OnStopMotion(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                    std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (state_ != RunState::RUNNING && state_ != RunState::PAUSED) {
      res->success = false;
      res->message = "Controller not active";
      return;
    }
    state_ = RunState::ABORTED;
    phase_ = Phase::SEEK;
    res->success = true;
    res->message = "ABORTED";
  }

  // ---- control loop ----
  void ControlLoop() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!InputsReady()) {
      PublishZeroVelocity();
      PublishState(0.0);
      return;
    }

    if (!UpdateToolPose()) {
      PublishZeroVelocity();
      PublishState(0.0);
      return;
    }

    Eigen::Vector3d force_world = current_pose_.linear() * force_probe_;
    double normal_force = UpdateContactFrame(force_world);
    UpdatePhaseForForce(normal_force);

    Eigen::Vector3d linear_cmd = Eigen::Vector3d::Zero();
    bool command_active = ComputeLinearCommand(normal_force, linear_cmd);
    Eigen::Matrix<double, 6, 1> twist = Eigen::Matrix<double, 6, 1>::Zero();

    if (command_active) {
      twist.head<3>() = linear_cmd;
      twist.tail<3>().setZero();
      LimitTwist(twist, linear_limit_, angular_limit_);
      last_twist_cmd_ = twist;
      if (!SendJointVelocityCommand(twist)) {
        EnterFault("IK_FAILED");
        PublishZeroVelocity();
      }
    } else {
      last_twist_cmd_.setZero();
      PublishZeroVelocity();
    }

    PublishState(normal_force);
    BroadcastContactFrame();
  }

  bool InputsReady() const {
    return joint_ready_ && wrench_ready_ && static_cast<bool>(fk_solver_);
  }

  bool UpdateToolPose() {
    return ComputeForwardKinematics(joint_positions_, fk_solver_.get(), wrist_to_tool_, current_pose_);
  }

  Eigen::Vector3d WristOffsetWorld() const {
    Eigen::Matrix3d R_tool = current_pose_.linear();
    Eigen::Matrix3d R_wrist = R_tool * wrist_to_tool_rotation_.transpose();
    return R_wrist * wrist_to_tool_translation_;
  }

  bool ComputeLinearCommand(double normal_force, Eigen::Vector3d& linear_cmd) {
    if (state_ != RunState::RUNNING && state_ != RunState::PAUSED) {
      return false;
    }

    double normal_setpoint = GetNormalSetpoint();
    double normal_vel = ComputeNormalVelocity(normal_setpoint, normal_force);
    linear_cmd = normal_vel * contact_normal_;

    if (phase_ == Phase::SEEK) {
      if (IsForceWithinBand(normal_force)) {
        phase_ = Phase::DWELL;
        dwell_elapsed_ = 0.0;
      }
    } else if (phase_ == Phase::DWELL) {
      if (state_ == RunState::RUNNING) {
        dwell_elapsed_ += control_period_s_;
        if (dwell_elapsed_ >= dwell_time_s_) {
          phase_ = Phase::TANGENTIAL;
        }
      }
    } else if (phase_ == Phase::TANGENTIAL && state_ == RunState::RUNNING) {
      double remaining = tangential_target_ - tangential_distance_;
      if (remaining <= 1e-4) {
        state_ = RunState::COMPLETED;
        phase_ = Phase::TANGENTIAL;
        return false;
      }

      double tangential_speed = std::min(tangential_speed_limit_,
                                         tangential_gain_ * remaining);
      linear_cmd += tangential_dir_ * tangential_speed;
      tangential_distance_ += tangential_speed * control_period_s_;
    }

    return true;
  }

  bool SendJointVelocityCommand(const Eigen::Matrix<double, 6, 1>& twist) {
    Eigen::Matrix<double, 6, 1> wrist_twist = twist;
    Eigen::Vector3d offset = WristOffsetWorld();
    wrist_twist.head<3>() = twist.head<3>() - twist.tail<3>().cross(offset);

    if (!ComputeJointVelocity(wrist_twist, joint_positions_, ik_solver_.get(), qdot_buffer_)) {
      return false;
    }
    ClampVectorInPlace(qdot_buffer_, joint_velocity_limits_);
    PublishVelocity(qdot_buffer_);
    return true;
  }

  double UpdateContactFrame(const Eigen::Vector3d& force_world) {
    Eigen::Vector3d normal = contact_normal_;
    if (force_world.norm() > 1e-4) {
      if (estimator_enabled_ &&
          phase_ == Phase::TANGENTIAL &&
          last_twist_cmd_.head<3>().norm() > estimator_min_speed_) {
        Eigen::Vector3d v_hat = last_twist_cmd_.head<3>().normalized();
        double f_parallel = force_world.dot(v_hat);
        Eigen::Vector3d f_v = f_parallel * v_hat;
        Eigen::Vector3d f_perp = force_world - f_v;
        if (f_perp.norm() > 1e-4) {
          double mu = std::clamp(f_v.norm() / f_perp.norm(), 0.0, 5.0);
          mu_filtered_ = estimator_alpha_ * mu + (1.0 - estimator_alpha_) * mu_filtered_;
          Eigen::Vector3d f_tau = -mu_filtered_ * f_perp.norm() * v_hat;
          Eigen::Vector3d f_normal = force_world - f_tau;
          if (f_normal.norm() > 1e-5) {
            normal = f_normal.normalized();
          } else {
            normal = force_world.normalized();
          }
        } else {
          normal = force_world.normalized();
        }
      } else {
        normal = force_world.normalized();
      }
    } else {
      normal = search_direction_;
    }

    if (normal.dot(contact_normal_) < 0.0) {
      normal = -normal;
    }
    contact_normal_ = normal.normalized();
    tangential_dir_ = ProjectAndNormalize(tangential_dir_, contact_normal_);

    return force_world.dot(contact_normal_);
  }

  void UpdatePhaseForForce(double normal_force) {
    if (state_ == RunState::RUNNING &&
        phase_ == Phase::TANGENTIAL &&
        normal_force < (normal_target_ - normal_tolerance_)) {
      phase_ = Phase::SEEK;
      softstart_active_ = false;
    }

    if (state_ == RunState::RUNNING &&
        phase_ == Phase::TANGENTIAL) {
      if (normal_force < disengage_threshold_) {
        if (++disengage_counter_ >= disengage_count_limit_) {
          EnterFault("CONTACT_LOST");
        }
      } else {
        disengage_counter_ = 0;
      }
    } else {
      disengage_counter_ = 0;
    }
  }

  Eigen::Vector3d ProjectAndNormalize(const Eigen::Vector3d& v,
                                      const Eigen::Vector3d& normal) const {
    Eigen::Vector3d projected = ProjectOntoPlane(v, normal);
    if (projected.norm() < 1e-6) {
      projected = normal.unitOrthogonal();
    }
    return projected.normalized();
  }

  double GetNormalSetpoint() {
    if (!softstart_active_) return normal_target_;
    softstart_elapsed_ += control_period_s_;
    double alpha = std::clamp(softstart_elapsed_ / std::max(1e-6, softstart_time_s_),
                              0.0, 1.0);
    if (alpha >= 1.0) {
      softstart_active_ = false;
    }
    return normal_target_ * alpha;
  }

  double ComputeNormalVelocity(double setpoint, double measured) {
    double error = setpoint - measured;
    if (normal_ki_ > 1e-6) {
      double integral_limit = normal_max_velocity_ / normal_ki_;
      normal_integral_ = std::clamp(normal_integral_ + error * control_period_s_,
                                    -integral_limit, integral_limit);
    } else {
      normal_integral_ = 0.0;
    }
    double cmd = normal_kp_ * error + normal_ki_ * normal_integral_;
    return std::clamp(cmd, -normal_max_velocity_, normal_max_velocity_);
  }

  bool IsForceWithinBand(double normal_force) const {
    return std::abs(normal_force - normal_target_) <= normal_tolerance_;
  }

  void PublishVelocity(const std::vector<double>& qdot) {
    velocity_msg_.data = qdot;
    velocity_pub_->publish(velocity_msg_);
  }

  void PublishZeroVelocity() {
    velocity_msg_.data.assign(joint_count_, 0.0);
    velocity_pub_->publish(velocity_msg_);
  }

  void PublishState(double normal_force) {
    if (!state_pub_) return;

    StateMsg msg;
    msg.stamp = now();
    msg.state = static_cast<uint8_t>(StateToMsgValue(state_));
    msg.phase = static_cast<uint8_t>(PhaseToMsgValue(phase_));
    msg.normal_force = static_cast<float>(normal_force);
    msg.normal_force_target = static_cast<float>(normal_target_);
    msg.normal_force_error = static_cast<float>(normal_force - normal_target_);
    msg.tangential_distance = static_cast<float>(tangential_distance_);
    msg.tangential_target = static_cast<float>(tangential_target_);
    msg.dwell_active = (phase_ == Phase::DWELL && state_ == RunState::RUNNING);
    msg.paused = (state_ == RunState::PAUSED);
    msg.fault_active = (state_ == RunState::FAULT);
    msg.fault_reason = fault_reason_;
    state_pub_->publish(msg);
  }

  static uint8_t StateToMsgValue(RunState state) {
    switch (state) {
      case RunState::WAITING_FOR_START: return StateMsg::STATE_WAITING_FOR_START;
      case RunState::READY: return StateMsg::STATE_READY;
      case RunState::RUNNING: return StateMsg::STATE_RUNNING;
      case RunState::PAUSED: return StateMsg::STATE_PAUSED;
      case RunState::COMPLETED: return StateMsg::STATE_COMPLETED;
      case RunState::ABORTED: return StateMsg::STATE_ABORTED;
      case RunState::FAULT: return StateMsg::STATE_FAULT;
    }
    return StateMsg::STATE_WAITING_FOR_START;
  }

  static uint8_t PhaseToMsgValue(Phase phase) {
    switch (phase) {
      case Phase::SEEK: return StateMsg::PHASE_SEEK;
      case Phase::DWELL: return StateMsg::PHASE_DWELL;
      case Phase::TANGENTIAL: return StateMsg::PHASE_TANGENTIAL;
    }
    return StateMsg::PHASE_SEEK;
  }

  void BroadcastContactFrame() {
    if (!publish_contact_frame_ || !tf_broadcaster_) return;
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = now();
    tf_msg.header.frame_id = base_link_;
    tf_msg.child_frame_id = contact_frame_id_;
    tf_msg.transform.translation.x = current_pose_.translation().x();
    tf_msg.transform.translation.y = current_pose_.translation().y();
    tf_msg.transform.translation.z = current_pose_.translation().z();

    Eigen::Vector3d z = contact_normal_;
    Eigen::Vector3d x = tangential_dir_;
    Eigen::Vector3d y = z.cross(x).normalized();
    x = y.cross(z).normalized();

    Eigen::Matrix3d R;
    R.col(0) = x;
    R.col(1) = y;
    R.col(2) = z;
    Eigen::Quaterniond q(R);
    tf_msg.transform.rotation = tf2::toMsg(q);
    tf_broadcaster_->sendTransform(tf_msg);
  }

  void EnterFault(const std::string& reason) {
    state_ = RunState::FAULT;
    fault_reason_ = reason;
    RCLCPP_ERROR(get_logger(), "Fault: %s", reason.c_str());
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

  // data members
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
  bool wrench_ready_{false};

  Eigen::Vector3d force_probe_{Eigen::Vector3d::Zero()};
  Eigen::Isometry3d current_pose_{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d start_pose_{Eigen::Isometry3d::Identity()};

  Eigen::Vector3d search_direction_{0.0, 0.0, -1.0};
  Eigen::Vector3d contact_normal_{0.0, 0.0, -1.0};
  Eigen::Vector3d direction_hint_{1.0, 0.0, 0.0};
  Eigen::Vector3d tangential_dir_{1.0, 0.0, 0.0};

  double tangential_target_{0.05};
  double tangential_distance_{0.0};
  double tangential_gain_{1.0};
  double tangential_speed_limit_{0.01};
  double dwell_time_s_{1.0};
  double dwell_elapsed_{0.0};

  double normal_target_{5.0};
  double normal_tolerance_{0.2};
  double normal_kp_{0.6};
  double normal_ki_{4.0};
  double normal_max_velocity_{0.01};
  double normal_integral_{0.0};
  double softstart_time_s_{0.7};
  double softstart_elapsed_{0.0};
  bool softstart_active_{true};

  double disengage_threshold_{2.0};
  int disengage_count_limit_{30};
  int disengage_counter_{0};

  bool estimator_enabled_{true};
  double estimator_alpha_{0.15};
  double estimator_min_speed_{0.001};
  double mu_filtered_{0.0};

  double linear_limit_{0.05};
  double angular_limit_{0.5};

  bool publish_contact_frame_{true};
  std::string contact_frame_id_{"contact_frame"};

  RunState state_;
  Phase phase_;
  std::string fault_reason_;

  Eigen::Matrix<double, 6, 1> last_twist_cmd_;

  // ROS entities
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr direction_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_pub_;
  rclcpp::Publisher<StateMsg>::SharedPtr state_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr set_start_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pause_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resume_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  std_msgs::msg::Float64MultiArray velocity_msg_;

  // KDL
  KDL::Tree kdl_tree_;
  KDL::Chain kdl_chain_;
  size_t joint_count_{0};
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
  std::unique_ptr<KDL::ChainIkSolverVel_wdls> ik_solver_;
  KDL::Frame wrist_to_tool_{KDL::Frame::Identity()};
  Eigen::Vector3d wrist_to_tool_translation_{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d wrist_to_tool_rotation_{Eigen::Matrix3d::Identity()};

  std::mutex data_mutex_;
};

}  // namespace hfmc

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<hfmc::HybridForceMotionNode>();
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("hybrid_force_motion_controller"),
                 "Fatal error: %s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
