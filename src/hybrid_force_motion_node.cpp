// hybrid_force_motion_optimized.cpp
// Optimized hybrid force/motion controller combining best practices from both implementations:
// - Correct tangential tracking via direction projection (from Code 1)
// - Anti-windup PI + soft-start (from Code 2)
// - Proper angle-axis orientation error (from Code 2)
// - Friction-compensated normal estimation (Nasiri-style, both)
// - Streamlined FSM and reduced code bloat

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace hfmc {

using namespace std::chrono_literals;

// ============================================================================
// Utility Functions
// ============================================================================
template <typename T>
T clamp(T v, T lo, T hi) { return std::min(hi, std::max(lo, v)); }

Eigen::Vector3d safe_normalize(const Eigen::Vector3d& v, const Eigen::Vector3d& fallback) {
  return (v.squaredNorm() > 1e-12) ? v.normalized() : fallback;
}

Eigen::Vector3d project_orthogonal(const Eigen::Vector3d& v, const Eigen::Vector3d& n) {
  Eigen::Vector3d proj = v - n * v.dot(n);
  return (proj.squaredNorm() > 1e-12) ? proj.normalized() : n.unitOrthogonal();
}

std::string to_string(const Eigen::Vector3d& v) {
  char buf[64];
  std::snprintf(buf, sizeof(buf), "[%.2f %.2f %.2f]", v.x(), v.y(), v.z());
  return std::string(buf);
}

// ============================================================================
// State Machine
// ============================================================================
enum class State : uint8_t { IDLE, READY, SEEK, DWELL, SLIDE, PAUSED, DONE, FAULT };

const char* state_name(State s) {
  constexpr const char* names[] = {"IDLE","READY","SEEK","DWELL","SLIDE","PAUSED","DONE","FAULT"};
  return names[static_cast<uint8_t>(s)];
}

// ============================================================================
// PI Controller with Anti-Windup
// ============================================================================
struct PIController {
  // Interpreted as an impedance-style PD on force error:
  // v_n ≈ kp * (F_d - F_s) - kd * xdot_n  (+ optional small integral for bias)
  double kp{0.002}, ki{0.0}, kd{0.1}, max_out{0.02};
  double integral{0.0};
  
  void reset() { integral = 0.0; }
  
  double compute(double error, double v_n_current, double error_band, bool in_contact, double dt) {
    double i_limit = (ki > 1e-6) ? max_out / ki : 1e6;
    if (in_contact && std::abs(error) <= error_band) {
      integral = clamp(integral + error * dt, -i_limit, i_limit);
    } else {
      // Decay integral gently when outside the band or out of contact to avoid windup
      integral *= 0.95;
      integral = clamp(integral, -i_limit, i_limit);
    }
    
    double p_term = kp * error;
    double i_term = ki * integral;
    // v_n_current must be EE velocity along the contact normal (positive into the surface)
    double d_term = -kd * v_n_current;  // Proper damping: opposes penetration velocity
    return clamp(p_term + i_term + d_term, -max_out, max_out);
  }
};

// ============================================================================
// Friction-Compensated Normal Estimator (Nasiri-style, per Eq. 28-37)
// ============================================================================
// Key insight from the paper: raw force direction is biased by arctan(μ) during
// sliding. We must subtract the friction component BEFORE normalizing.
//
// Algorithm (from your guide):
//   1. Ω_v = v̂v̂ᵀ                    (projection matrix onto velocity)
//   2. f_v = Ω_v · f_s               (force component ∥ to velocity)
//   3. f_⊥ = (I - Ω_v) · f_s         (force component ⊥ to velocity)
//   4. μ_k = |f_v| / |f_⊥|           (instantaneous Coulomb estimate)
//   5. μ̄  ← (1-α)μ̄ + α·μ_k          (low-pass filtered estimate)
//   6. f_τ = -μ̄ |f_⊥| v̂             (estimated friction force)
//   7. f̂_n = f_s - f_τ               (friction-compensated normal force)
//   8. n̂ = f̂_n / |f̂_n|              (surface normal estimate)
//
// CRITICAL: When estimation is unreliable (low speed, small f_⊥), we KEEP the
// previous normal estimate. NEVER fall back to raw force direction — that's
// the biased value we're trying to avoid!
// ============================================================================
struct FrictionEstimator {
  double mu{0.0}, alpha{0.1}, min_speed{0.002};
  bool enabled{true};
  
  // IMPORTANT: v_meas must be the MEASURED EE velocity from TF/FK, NOT the commanded velocity.
  // Using v_cmd would correlate the estimator input with controller output, causing drift.
  Eigen::Vector3d estimate_normal(const Eigen::Vector3d& f_s,
                                   const Eigen::Vector3d& v_meas,
                                   const Eigen::Vector3d& current_normal) {
    // Guard: estimator disabled or velocity too low for reliable μ estimate
    if (!enabled || v_meas.squaredNorm() < min_speed * min_speed)
      return current_normal;  // Keep previous estimate, don't use biased f_s
    
    // Step 1-2: Project force onto velocity direction
    // Ω_v = v̂v̂ᵀ, so f_v = (f_s · v̂) v̂
    Eigen::Vector3d v_hat = v_meas.normalized();
    double f_parallel = f_s.dot(v_hat);           // scalar: f_s · v̂
    Eigen::Vector3d f_v = f_parallel * v_hat;     // vector: projection onto v̂
    
    // Step 3: Force perpendicular to velocity
    Eigen::Vector3d f_perp = f_s - f_v;           // f_⊥ = (I - Ω_v) f_s
    double f_perp_norm = f_perp.norm();
    
    // Guard: f_⊥ too small → μ estimate unreliable (division by ~0)
    if (f_perp_norm < 1e-4) 
      return current_normal;  // Keep previous, don't use biased f_s!
    
    // Step 4-5: Estimate and filter Coulomb coefficient
    // μ_k = |f_v| / |f_⊥| = |f_s · v̂| / |f_⊥|
    double mu_inst = clamp(std::abs(f_parallel) / f_perp_norm, 0.0, 2.0);
    mu = (1.0 - alpha) * mu + alpha * mu_inst;    // EMA filter
    
    // Step 6: Construct estimated friction force
    // f_τ = -μ̄ |f_⊥| v̂  (opposes velocity direction)
    Eigen::Vector3d f_tau = -mu * f_perp_norm * v_hat;
    
    // Step 7: Subtract friction to get normal force
    // f̂_n = f_s - f_τ
    Eigen::Vector3d f_n_hat = f_s - f_tau;
    
    // Guard: corrected normal force too small (shouldn't happen in practice)
    if (f_n_hat.squaredNorm() < 1e-8) 
      return current_normal;  // Keep previous, don't use biased f_s!
    
    // Step 8: Normalize to get surface normal direction
    Eigen::Vector3d n_new = f_n_hat.normalized();
    
    // Avoid 180° sign flips (normal should vary smoothly)
    return (n_new.dot(current_normal) >= 0) ? n_new : -n_new;
  }
};

// ============================================================================
// Main Controller Node
// ============================================================================
class HybridForceMotionNode : public rclcpp::Node {
public:
  HybridForceMotionNode()
      : Node("hybrid_force_motion_controller"),
        tf_buffer_(get_clock()), tf_listener_(tf_buffer_) {
    load_params();
    setup_ros();
    RCLCPP_INFO(get_logger(), "Controller ready @ %.0f Hz", 1.0 / dt_);
  }

private:
  // --------------------------------------------------------------------------
  // Parameters
  // --------------------------------------------------------------------------
  void load_params() {
    // Legacy/compatibility parameters (from controller.yaml)
    const double legacy_control_rate = declare_parameter("control_frequency_hz", 500.0);
    const std::string legacy_base = declare_parameter("base_link", "base_link");
    const std::string legacy_tool = declare_parameter("tool_link", "tool0");
    auto legacy_dir_hint = declare_parameter<std::vector<double>>("tangential_direction_hint", {1.0, 0.0, 0.0});
    double legacy_slide_dist = declare_parameter("tangential_distance_m", 0.05);
    double legacy_tan_gain = declare_parameter("tangential_gain", 1.0);
    double legacy_tan_speed = declare_parameter("tangential_speed_mps", 0.03);
    dwell_time_ = declare_parameter("dwell_time_s", 1.0);
    // Nested legacy parameters appear as dotted names
    double legacy_force_target = declare_parameter("normal.target_N", 5.0);
    double legacy_force_band = declare_parameter("normal.tolerance_N", 1.0);
    double legacy_force_kp = declare_parameter("normal.kp", 0.002);
    // Theory-closer default: pure PD force loop; integral only if explicitly enabled in YAML
    double legacy_force_ki = declare_parameter("normal.ki", 0.0);
    double legacy_force_max = declare_parameter("normal.max_velocity_mps", 0.02);
    double legacy_softstart = declare_parameter("normal.softstart_time_s", 0.5);
    declare_parameter("disengage_force_threshold_N", 2.0);  // unused but keep for compatibility
    contact_loss_limit_ = declare_parameter("disengage_count", 25);
    estimator_.enabled = declare_parameter("estimator.enable", true);
    estimator_.alpha = declare_parameter("estimator.alpha", 0.1);
    estimator_.min_speed = declare_parameter("estimator.min_speed_mps", 0.002);
    contact_loss_thresh_ = declare_parameter("contact_force_min_threshold_N", 0.2);
    declare_parameter("publish_contact_frame", true);
    declare_parameter("contact_frame_id", "contact_frame");
    
    // New names (overrides legacy if provided)
    base_frame_ = declare_parameter("base_frame", legacy_base);
    tool_frame_ = declare_parameter("tool_frame", legacy_tool);
    
    force_target_ = declare_parameter("force_target_N", legacy_force_target);
    force_band_ = declare_parameter("force_band_N", legacy_force_band);
    pi_.kp = declare_parameter("force_kp", legacy_force_kp);
    pi_.ki = declare_parameter("force_ki", legacy_force_ki);
    pi_.kd = declare_parameter("force_kd", 0.1);
    pi_.max_out = declare_parameter("v_normal_max", legacy_force_max);
    
    softstart_time_ = declare_parameter("softstart_time_s", legacy_softstart);
    
    slide_dist_ = declare_parameter("slide_distance_m", legacy_slide_dist);
    v_tan_max_ = declare_parameter("v_tangent_max", legacy_tan_speed);
    tan_kp_ = declare_parameter("tangent_kp", legacy_tan_gain);
    
    estimator_.enabled = declare_parameter("friction_estimator_enable", estimator_.enabled);
    estimator_.alpha = declare_parameter("friction_alpha", estimator_.alpha);
    estimator_.min_speed = declare_parameter("friction_min_speed", estimator_.min_speed);
    
    orient_k_ = declare_parameter("orientation_k", 3.0);
    omega_max_ = declare_parameter("omega_max", 0.8);
    
    dt_ = 1.0 / declare_parameter("control_rate_hz", legacy_control_rate);
    
    contact_loss_thresh_ = declare_parameter("contact_loss_threshold", contact_loss_thresh_);
    contact_loss_limit_ = declare_parameter("contact_loss_count", contact_loss_limit_);
    max_force_ = declare_parameter("max_force_N", 20.0);
    
    // Initial directions
    auto sd = declare_parameter<std::vector<double>>("search_direction", {0, 0, -1});
    auto th = declare_parameter<std::vector<double>>("tangent_hint", legacy_dir_hint);
    search_dir_ = safe_normalize({sd[0], sd[1], sd[2]}, {0, 0, -1});
    tan_hint_ = safe_normalize({th[0], th[1], th[2]}, {1, 0, 0});
    
    contact_normal_ = search_dir_;
    tan_dir_ = project_orthogonal(tan_hint_, contact_normal_);
  }

  // --------------------------------------------------------------------------
  // ROS Setup
  // --------------------------------------------------------------------------
  void setup_ros() {
    wrench_sub_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
        "/netft/proc_base", rclcpp::SensorDataQoS(),
        [this](geometry_msgs::msg::WrenchStamped::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lk(mtx_);
          force_ = {msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z};
          have_wrench_ = true;
        });
    
    wrench_probe_sub_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
        "/netft/proc_probe", rclcpp::SensorDataQoS(),
        [this](geometry_msgs::msg::WrenchStamped::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lk(mtx_);
          force_probe_ = {msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z};
          have_wrench_probe_ = true;
        });
    
    wrench_sensor_sub_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
        "/netft/proc_sensor", rclcpp::SensorDataQoS(),
        [this](geometry_msgs::msg::WrenchStamped::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lk(mtx_);
          force_sensor_ = {msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z};
          have_wrench_sensor_ = true;
        });
    
    wrench_raw_sub_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
        "/netft/raw_sensor", rclcpp::SensorDataQoS(),
        [this](geometry_msgs::msg::WrenchStamped::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lk(mtx_);
          force_raw_ = {msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z};
          have_wrench_raw_ = true;
        });
    
    dir_sub_ = create_subscription<geometry_msgs::msg::Vector3>(
        "~/direction", 1, [this](geometry_msgs::msg::Vector3::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lk(mtx_);
          Eigen::Vector3d v(msg->x, msg->y, msg->z);
          if (v.squaredNorm() > 1e-8) {
            tan_hint_ = v.normalized();
            tan_dir_ = project_orthogonal(tan_hint_, contact_normal_);
          }
        });
    
    twist_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("~/twist_cmd", 10);
    
    set_start_srv_ = create_service<std_srvs::srv::Trigger>("~/set_start_pose",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
               std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
          res->success = set_start();
          res->message = state_name(state_);
        });
    start_srv_ = create_service<std_srvs::srv::Trigger>("~/start_motion",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
               std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
          res->success = start();
          res->message = state_name(state_);
        });
    pause_srv_ = create_service<std_srvs::srv::Trigger>("~/pause_motion",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
               std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
          res->success = pause();
          res->message = state_name(state_);
        });
    resume_srv_ = create_service<std_srvs::srv::Trigger>("~/resume_motion",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
               std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
          res->success = resume();
          res->message = state_name(state_);
        });
    stop_srv_ = create_service<std_srvs::srv::Trigger>("~/stop_motion",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
               std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
          res->success = stop();
          res->message = state_name(state_);
        });
    
    timer_ = create_wall_timer(std::chrono::duration<double>(dt_),
        std::bind(&HybridForceMotionNode::control_loop, this));
  }

  // --------------------------------------------------------------------------
  // Service Handlers
  // --------------------------------------------------------------------------
  bool set_start() {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!have_wrench_probe_) return false;
    Eigen::Isometry3d T;
    if (!get_pose(T)) return false;
    
    start_pos_ = last_pos_ = T.translation();
    start_rot_ = T.linear();
    contact_normal_ = search_dir_;
    tan_dir_ = project_orthogonal(tan_hint_, contact_normal_);
    reset_state();
    state_ = State::READY;
    return true;
  }
  
  bool start() {
    std::lock_guard<std::mutex> lk(mtx_);
    if (state_ != State::READY) return false;
    reset_state();
    state_ = State::SEEK;
    return true;
  }
  
  bool pause() {
    std::lock_guard<std::mutex> lk(mtx_);
    if (state_ != State::SLIDE) return false;
    state_ = State::PAUSED;
    return true;
  }
  
  bool resume() {
    std::lock_guard<std::mutex> lk(mtx_);
    if (state_ != State::PAUSED) return false;
    state_ = State::SLIDE;
    return true;
  }
  
  bool stop() {
    std::lock_guard<std::mutex> lk(mtx_);
    state_ = State::DONE;
    return true;
  }
  
  void reset_state() {
    pi_.reset();
    estimator_.mu = 0.0;
    tan_progress_ = 0.0;
    dwell_elapsed_ = 0.0;
    softstart_elapsed_ = 0.0;
    contact_loss_count_ = 0;
    runtime_ = 0.0;
    last_v_n_cmd_ = 0.0;
    have_last_pos_vel_ = false;
  }

  // --------------------------------------------------------------------------
  // Main Control Loop
  // --------------------------------------------------------------------------
  void control_loop() {
    std::lock_guard<std::mutex> lk(mtx_);
    
    Eigen::Vector3d v_cmd = Eigen::Vector3d::Zero();
    Eigen::Vector3d w_cmd = Eigen::Vector3d::Zero();
    
    if (!have_wrench_probe_ || state_ == State::IDLE || state_ == State::DONE || state_ == State::FAULT) {
      publish_twist(v_cmd, w_cmd);
      return;
    }
    
    Eigen::Isometry3d T;
    if (!get_pose(T)) {
      fault("TF_LOOKUP_FAILED");
      publish_twist(v_cmd, w_cmd);
      return;
    }
    
    const Eigen::Vector3d pos = T.translation();
    const Eigen::Matrix3d R = T.linear();
    const Eigen::Vector3d probe_axis = -R.col(2);  // Probe along -Z of tool
    // Measured EE displacement since last cycle; used to estimate normal velocity for damping
    Eigen::Vector3d dp_vel = Eigen::Vector3d::Zero();
    if (have_last_pos_vel_) {
      dp_vel = pos - last_pos_vel_;
    }
    last_pos_vel_ = pos;
    have_last_pos_vel_ = true;
    
    // Map probe-frame wrench into base for consistent math with base-frame velocity/normal
    Eigen::Vector3d force_base = R * force_probe_;

    // Transform contact normal into probe frame to match probe-frame wrench
    Eigen::Vector3d n_probe = R.transpose() * contact_normal_;
    double F_n = force_probe_.dot(n_probe);  // signed force along normal
    
    // Measure normal force magnitude along normal direction
    double F_n_mag = std::abs(F_n);
    
    // Soft-start: ramp force target
    double target = force_target_;
    if (softstart_elapsed_ < softstart_time_) {
      softstart_elapsed_ += dt_;
      target = force_target_ * std::min(1.0, softstart_elapsed_ / softstart_time_);
    }
    
    auto compute_v_n = [&](double F_n_val) {
      double error = target - F_n_val;
      // EE velocity component along the contact normal (positive into the surface)
      double v_n_current = (dt_ > 0.0)
                               ? dp_vel.dot(contact_normal_) / dt_
                               : 0.0;
      double F_n_abs = std::abs(F_n_val);
      bool in_contact = F_n_abs > contact_loss_thresh_ * force_target_;
      double v_n = pi_.compute(error, v_n_current, force_band_, in_contact, dt_);
      last_v_n_cmd_ = v_n;
      return v_n;
    };

    // Always-on normal-force control for all active states:
    // if F < F_target -> move along +normal (into surface),
    // if F > F_target -> move along -normal (out of surface),
    // if within band  -> small or zero v_n.
    double v_n = compute_v_n(F_n);
    v_cmd = v_n * contact_normal_;
    
    switch (state_) {
      case State::READY:
        // READY: force controller already applied above; no tangential motion.
        break;
        
      case State::SEEK: {
        // SEEK: use the same force loop, and when inside the band long enough,
        // transition toward SLIDE via DWELL (or directly if dwell_time_ == 0).
        if (std::abs(F_n - force_target_) <= force_band_) {
          dwell_elapsed_ += dt_;
          if (dwell_elapsed_ >= dwell_time_) {
            state_ = State::DWELL;
            dwell_elapsed_ = 0.0;
          }
        } else {
          dwell_elapsed_ = 0.0;
        }
      } break;
        
      case State::DWELL: {
        // DWELL: same force controller as READY/SEEK, but accumulate time
        // within the band before starting SLIDE.
        dwell_elapsed_ += dt_;
        if (dwell_elapsed_ >= dwell_time_) {
          tan_progress_ = 0.0;
          last_pos_ = pos;
          state_ = State::SLIDE;
          RCLCPP_INFO(get_logger(), "Entering SLIDE phase");
        }
      } break;
        
      case State::SLIDE:
      case State::PAUSED: {
        // Compute measured EE velocity from position difference (base frame)
        // CRITICAL: This must be the ACTUAL velocity, not the commanded velocity,
        // for correct friction estimation. Using v_cmd would cause estimator drift.
        Eigen::Vector3d dp = pos - last_pos_;
        Eigen::Vector3d v_meas = dp / dt_;
        
        // Update normal estimate with friction compensation using MEASURED velocity
        if (F_n > 0.5 && state_ == State::SLIDE) {
          contact_normal_ = estimator_.estimate_normal(force_base, v_meas, contact_normal_);
          tan_dir_ = project_orthogonal(tan_hint_, contact_normal_);
        }
        
        // Recompute normal force with updated normal
        n_probe = R.transpose() * contact_normal_;
        F_n = force_probe_.dot(n_probe);
        F_n_mag = std::abs(F_n);
        
        // Tangential velocity (position control) — CORRECT: project onto tangent
        if (state_ == State::SLIDE) {
          double dp_tan = dp.dot(tan_dir_);
          tan_progress_ += std::max(0.0, dp_tan);  // only forward progress counts
          last_pos_ = pos;  // Update AFTER using dp
          
          double remaining = slide_dist_ - tan_progress_;
          if (remaining <= 1e-4) {
            state_ = State::DONE;
            RCLCPP_INFO(get_logger(), "Slide complete: %.3f m", tan_progress_);
            break;
          }
          
          double v_t = clamp(tan_kp_ * remaining, 0.0, v_tan_max_);
          v_cmd += v_t * tan_dir_;
        } else {
          // PAUSED: still update last_pos_ to avoid velocity spike on resume
          last_pos_ = pos;
        }
        
        // Orientation alignment: probe axis → contact normal
        // Use proper angle-axis method (not small-angle approximation)
        // Error axis = probe × normal, with magnitude sin(θ)
        Eigen::Vector3d axis_err = probe_axis.cross(contact_normal_);
        double sin_angle = axis_err.norm();
        
        if (sin_angle > 1e-6) {
          // Recover actual angle: θ = asin(|axis_err|), but also check dot product
          // for angles > 90° (though shouldn't happen if normal tracking is good)
          double cos_angle = probe_axis.dot(contact_normal_);
          double angle = std::atan2(sin_angle, cos_angle);  // Proper quadrant handling
          
          // ω = k * θ * axis_normalized (P-control on angle, not sin(angle))
          Eigen::Vector3d axis_unit = axis_err / sin_angle;
          w_cmd = orient_k_ * angle * axis_unit;
          
          if (w_cmd.norm() > omega_max_) 
            w_cmd = w_cmd.normalized() * omega_max_;
        }
        
        // Safety checks
        if (F_n_mag < contact_loss_thresh_ * force_target_) {
          if (++contact_loss_count_ >= contact_loss_limit_) {
            fault("CONTACT_LOST");
          }
        } else {
          contact_loss_count_ = 0;
        }
        
        if (F_n_mag > max_force_) {
          fault("MAX_FORCE_EXCEEDED");
        }
      } break;
        
      default:
        break;
    }
    
    // Accumulate runtime for active states
    if (state_ != State::IDLE && state_ != State::DONE && state_ != State::FAULT) {
      runtime_ += dt_;
    }
    
    if (state_ == State::FAULT || state_ == State::DONE) {
      v_cmd.setZero();
      w_cmd.setZero();
    }
    
    publish_twist(v_cmd, w_cmd);
    
    // Throttled status log at ~1 Hz
    const double progress_cm = tan_progress_ * 100.0;
    const double target_cm = slide_dist_ * 100.0;
    double remaining_cm = std::max(0.0, (slide_dist_ - tan_progress_) * 100.0);
    double percent = (slide_dist_ > 1e-9) ? (tan_progress_ / slide_dist_) * 100.0 : 0.0;
    const double F_n_log = std::abs(force_base.dot(-contact_normal_));
    bool in_contact = F_n_log > contact_loss_thresh_ * force_target_;
    double f_err = force_target_ - F_n_log;
    double v_n_cmd = last_v_n_cmd_;
    double v_cmd_norm = v_cmd.norm();
    double w_cmd_norm = w_cmd.norm();
    double ang_err_deg = 0.0;
    if (state_ == State::SLIDE || state_ == State::PAUSED) {
      const Eigen::Vector3d probe_axis = -T.linear().col(2);
      Eigen::Vector3d axis_err = probe_axis.cross(contact_normal_);
      double sin_angle = axis_err.norm();
      if (sin_angle > 1e-6) {
        double cos_angle = probe_axis.dot(contact_normal_);
        ang_err_deg = std::atan2(sin_angle, cos_angle) * 180.0 / M_PI;
      }
    }
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "\nSTATE=%s t=%.2fs prog=%.2fcm (%.0f%% of %.2fcm) remain=%.2fcm"
        "\n  Force: F_n=%.2f/%.2f err=%.2f v_n=%.4f vcmd=%.4f wcmd=%.4f ang_err=%.2fdeg"
        "\n  Frames: n=%s tan=%s z=%.3f pose=%s contact=%c wrench=%c"
        "\n  Wrench: base=%s probe=%s sensor=%s raw=%s",
        state_name(state_), runtime_, progress_cm, percent, target_cm, remaining_cm,
        F_n_log, force_target_, f_err, v_n_cmd, v_cmd_norm, w_cmd_norm, ang_err_deg,
        to_string(contact_normal_).c_str(), to_string(tan_dir_).c_str(), T.translation().z(), "TF",
        in_contact ? 'Y' : 'N', have_wrench_probe_ ? 'Y' : 'N',
        to_string(force_base).c_str(), to_string(force_probe_).c_str(), to_string(force_sensor_).c_str(), to_string(force_raw_).c_str());
  }

  // --------------------------------------------------------------------------
  // Helpers
  // --------------------------------------------------------------------------
  bool get_pose(Eigen::Isometry3d& T) {
    try {
      auto tf = tf_buffer_.lookupTransform(base_frame_, tool_frame_, tf2::TimePointZero);
      T = tf2::transformToEigen(tf);
      return true;
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Pose unavailable (TF): %s", ex.what());
      return false;
    }
  }
  
  void publish_twist(const Eigen::Vector3d& v, const Eigen::Vector3d& w) {
    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = base_frame_;
    msg.twist.linear.x = v.x();
    msg.twist.linear.y = v.y();
    msg.twist.linear.z = v.z();
    msg.twist.angular.x = w.x();
    msg.twist.angular.y = w.y();
    msg.twist.angular.z = w.z();
    twist_pub_->publish(msg);
  }
  
  void fault(const std::string& reason) {
    if (state_ != State::FAULT) {
      RCLCPP_ERROR(get_logger(), "FAULT: %s", reason.c_str());
      state_ = State::FAULT;
    }
  }

  // --------------------------------------------------------------------------
  // Members
  // --------------------------------------------------------------------------
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_probe_sub_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sensor_sub_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_raw_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr dir_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr set_start_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pause_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resume_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;
  
  std::mutex mtx_;
  std::string base_frame_, tool_frame_;
  
  // Sensor data
  Eigen::Vector3d force_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d force_probe_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d force_sensor_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d force_raw_{Eigen::Vector3d::Zero()};
  bool have_wrench_{false};
  bool have_wrench_probe_{false};
  bool have_wrench_sensor_{false};
  bool have_wrench_raw_{false};
  
  // Control parameters
  double force_target_{5.0}, force_band_{1.0};
  double slide_dist_{0.05}, v_tan_max_{0.03}, tan_kp_{1.0};
  double orient_k_{3.0}, omega_max_{0.8};
  double dwell_time_{1.0}, dt_{0.002};
  double softstart_time_{0.5};
  double contact_loss_thresh_{0.2}, max_force_{20.0};
  int contact_loss_limit_{25};
  
  PIController pi_;
  FrictionEstimator estimator_;
  
  // State
  State state_{State::IDLE};
  Eigen::Vector3d search_dir_{0, 0, -1}, tan_hint_{1, 0, 0};
  Eigen::Vector3d contact_normal_{0, 0, -1}, tan_dir_{1, 0, 0};
  Eigen::Vector3d start_pos_, last_pos_, last_pos_vel_{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d start_rot_;
  
  double tan_progress_{0.0}, dwell_elapsed_{0.0}, softstart_elapsed_{0.0};
  int contact_loss_count_{0};
  double runtime_{0.0};
  double last_v_n_cmd_{0.0};
  bool have_last_pos_vel_{false};
};

}  // namespace hfmc

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<hfmc::HybridForceMotionNode>());
  rclcpp::shutdown();
  return 0;
}
