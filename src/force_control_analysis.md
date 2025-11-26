## Hybrid Force/Motion Controller – Normal-Force Loop Analysis

This document distills what we observed in sim, how the current `hybrid_force_motion_controller` is implemented, how it differs from the De Luca hybrid control theory (`reference/16_HybridControl.pdf`), and why the normal-force loop is oscillating instead of settling. It is meant as an engineering design note to guide the eventual fix plan.

All paths and line numbers below refer to:
- Code: `src/hybrid_force_motion_controller/src/hybrid_force_motion_node.cpp`
- Theory: `src/hybrid_force_motion_controller/reference/16_HybridControl.pdf` (De Luca, “Hybrid Force/Motion Control”, slide 19 in particular)

---

## 1. What We See in the Logs

Representative log excerpt (Gazebo sim, SEEK phase):

```text
STATE=SEEK t=16.46s ...
  Force: F_n=260.19/5.00 err=-255.19 v_n=-0.0100 vcmd=0.0100 ...
...
STATE=SEEK t=18.51s ...
  Force: F_n=20.23/5.00 err=-15.23 v_n=-0.0100 vcmd=0.0100 ...
...
STATE=SEEK t=21.59s ...
  Force: F_n=20.24/5.00 err=-15.24 v_n=-0.0100 vcmd=0.0100 ...
...
STATE=SEEK t=29.79s ...
  Force: F_n=5.09/5.00 err=-0.09 v_n=-0.0100 vcmd=0.0100 ...
...
STATE=SEEK t=30.82s ...
  Force: F_n=0.00/5.00 err=5.00 v_n=0.0100 vcmd=0.0100 ...
```

Key patterns:
- In `READY`:
  - `F_n ≈ 50–100 N`, `err ≈ -45 … -100 N`, but `v_n=0.0000`, `vcmd=0.0000`, `state=READY`, `contact=Y`.
  - So we are already in hard contact with the environment with a large preload, but the controller is idle and commands zero velocity.
- In `SEEK` (after start):
  - Before contact: `F_n=0`, `err=5`, `v_n=+0.0100`, `vcmd=+0.0100`, `contact=N`.
  - At contact: `F_n` jumps 0 → 260 N in one reporting interval; `err` becomes ~-255; `v_n` flips to `-0.0100` (max retract).
  - As the robot retracts, `F_n` decays 260 → 73 → 20 → 5 N, but `v_n` remains saturated at `-0.0100` throughout.
  - When `F_n` is finally close to the target (e.g., 5.1 N, error ≈ -0.1 N) we still see `v_n=-0.0100`.
  - Then contact is lost: `F_n=0`, error=+5, and `v_n` saturates back to `+0.0100` and the cycle repeats.

Important: the log **never** shows an intermediate `v_n` such as -0.003 or +0.004 in SEEK. It is essentially always `+0.0100` or `-0.0100`. That indicates that the normal-force controller is operating in saturation almost 100% of the time and behaves like a bang–bang velocity source.

---

## 2. How the Normal-Force Loop Is Implemented

### 2.1 Normal Force Definition

In the main control loop (`control_loop()`, around `hybrid_force_motion_node.cpp:404-422`):

```cpp
const Eigen::Vector3d pos = T.translation();
const Eigen::Matrix3d R = T.linear();
const Eigen::Vector3d probe_axis = -R.col(2);  // Probe along -Z of tool

// Map probe-frame wrench into base frame
Eigen::Vector3d force_base = R * force_probe_;

// Transform contact normal into probe frame to match probe-frame wrench
Eigen::Vector3d n_probe = R.transpose() * contact_normal_;
double F_n = force_probe_.dot(n_probe);  // signed force along normal

double F_n_mag = std::abs(F_n);
```

Interpretation:
- `contact_normal_` points *into* the contact (e.g., [0, 0, -1] if “down” is into the surface).
- `force_probe_` is the probe-frame wrench; `force_probe_.dot(n_probe)` is the component of the measured force along the normal.
- By convention, **positive `F_n` means pushing into the surface**.

### 2.2 Normal Velocity Command

Still in `control_loop()`:

```cpp
// Soft-start target
double target = force_target_;
if (softstart_elapsed_ < softstart_time_) {
  softstart_elapsed_ += dt_;
  target = force_target_ * std::min(1.0, softstart_elapsed_ / softstart_time_);
}

Eigen::Vector3d dp_vel = Eigen::Vector3d::Zero();
if (have_last_pos_vel_) {
  dp_vel = pos - last_pos_vel_;
}
last_pos_vel_ = pos;
have_last_pos_vel_ = true;

auto compute_v_n = [&](double F_n_val) {
  double error = target - F_n_val;  // e_f = F_d - F_s
  double v_n_current = dp_vel.dot(-contact_normal_) / dt_;
  double F_n_abs = std::abs(F_n_val);
  bool in_contact = F_n_abs > contact_loss_thresh_ * force_target_;
  double v_n = pi_.compute(error, v_n_current, force_band_, in_contact, dt_);
  last_v_n_cmd_ = v_n;
  return v_n;
};
```

The normal velocity command is then used as:

```cpp
// In SEEK
double v_n = compute_v_n(F_n);
v_cmd = v_n * contact_normal_;

// In DWELL/SLIDE similarly
```

So:
- `v_n` is a **scalar normal velocity** (signed).
- `v_cmd` is a 3D velocity vector along `contact_normal_` in the base frame.

The log prints:

```cpp
double v_n_cmd   = last_v_n_cmd_;
double v_cmd_norm = v_cmd.norm();

// ... RCLCPP_INFO_THROTTLE(... "v_n=%.4f vcmd=%.4f ..." ...)
```

So:
- `v_n` in the log = signed normal speed command.
- `vcmd` in the log = magnitude of the full 3D twist command.

### 2.3 PI(+D) Controller Implementation

At the top of the file (`hybrid_force_motion_node.cpp:60-80`):

```cpp
struct PIController {
  double kp{0.002}, ki{0.01}, kd{0.1}, max_out{0.02};
  double integral{0.0};
  
  void reset() { integral = 0.0; }
  
  double compute(double error, double v_n_current,
                 double error_band, bool in_contact, double dt) {
    double i_limit = (ki > 1e-6) ? max_out / ki : 1e6;  // ~2.0 if max_out=0.02

    if (in_contact && std::abs(error) <= error_band) {
      integral = clamp(integral + error * dt, -i_limit, i_limit);
    } else {
      // Decay integral gently when outside the band or out of contact
      integral *= 0.95;
      integral = clamp(integral, -i_limit, i_limit);
    }
    
    double p_term = kp * error;
    double i_term = ki * integral;
    double d_term = -kd * v_n_current;  // intended “velocity damping”
    
    return clamp(p_term + i_term + d_term, -max_out, max_out);
  }
};
```

Parameter loading (`load_params()`, `hybrid_force_motion_node.cpp:190-212`):

```cpp
double legacy_force_target = declare_parameter("normal.target_N", 5.0);
double legacy_force_band   = declare_parameter("normal.tolerance_N", 1.0);
double legacy_force_kp     = declare_parameter("normal.kp", 0.002);
double legacy_force_ki     = declare_parameter("normal.ki", 0.01);
double legacy_force_max    = declare_parameter("normal.max_velocity_mps", 0.02);

force_target_ = declare_parameter("force_target_N", legacy_force_target);
force_band_   = declare_parameter("force_band_N", legacy_force_band);
pi_.kp        = declare_parameter("force_kp", legacy_force_kp);
pi_.ki        = declare_parameter("force_ki", legacy_force_ki);
pi_.kd        = declare_parameter("force_kd", 0.1);
pi_.max_out   = declare_parameter("v_normal_max", legacy_force_max);
```

And member defaults (`hybrid_force_motion_node.cpp:646-655`):

```cpp
double force_target_{5.0}, force_band_{1.0};
...
double contact_loss_thresh_{0.2}, max_force_{20.0};
```

So:
- Effective force deadband ≈ ±1 N by default.
- Normal velocity saturation `max_out` ≈ ±0.02 m/s (or ±0.01 m/s if configured as in the logs).
- Gains are tuned such that the **P term alone** saturates for `|error| > max_out / kp ≈ 0.01 / 0.002 = 5 N`.

Result:
- For most of the force range (`|error| > 5 N`), the output of the force controller is **saturated** at ±`v_normal_max`.

---

## 3. Theoretical Force Loop (De Luca, Slide 19)

From `16_HybridControl.pdf`, “Force control via an impedance model” (slide 19):

Let:
- `x` = position along a force-controlled direction,
- `x_c` = environment contact point,
- `k_c > 0` = contact stiffness,
- `f_d > 0` = desired contact force,
- `f_s = k_c (x - x_c)` = measured contact force.

Impedance model:

```text
m_m * ẍ + d_m * ẋ + k_c (x - x_c) = f_d
```

Control law after feedback linearization (`ẍ = a_d`):

```text
a_d = (1/m_m) (f_d - f_s) - (d_m/m_m) ẋ
```

Key properties:
- **P on force error**: `(f_d - f_s)/m_m`
- **Velocity damping**: `-(d_m/m_m) ẋ` always opposes the direction of motion.
- **No integral** on force.
- Before contact (`f_s = 0`), this guarantees a constant approach speed:

  ```text
  ẋ_ss = f_d / d_m
  ```

Assumptions:
- Linear mass–spring–damper plant in the force direction.
- Gains chosen to keep the loop in the linear region (no saturation).

---

## 4. Where Implementation and Theory Diverge

### 4.1 Saturation Dominates → Bang–Bang Behaviour

Given:
- `kp ≈ 0.002`, `max_out ≈ 0.01 m/s` (from the logs),
- Saturation threshold in error is roughly:

  ```text
  |error| > max_out / kp ≈ 0.01 / 0.002 = 5 N
  ```

That means:
- Pre-contact (`F_n = 0`):
  - `error = 5` → `p_term = 0.002 * 5 = 0.01` → immediately at saturation.
- At modest overshoot (`F_n = 20 N`):
  - `error = 5 - 20 = -15 N` → `p_term = -0.03` → clamped to `-0.01`.
- At large overshoot (`F_n = 260 N`):
  - `error = 5 - 260 = -255 N` → `p_term = -0.51` → also clamped to `-0.01`.

Thus, the outer force loop behaves almost everywhere as:

```text
v_n ≈ +v_normal_max,  if error ≫ 0 (no contact)
v_n ≈ -v_normal_max,  if error ≪ 0 (in heavy contact)
```

This is **bang–bang normal velocity control** with only a narrow linear region around the force target (±~5 N), very different from the linear impedance model used for the theory.

The logs confirm this:
- SEEK in free space: `F_n=0`, `err=5`, `v_n=+0.0100`.
- After impact: `F_n` in the tens or hundreds of N, always `v_n=-0.0100`.
- Only when `F_n` is very close to 5 N could the loop be in the linear region – but by then overshoot and history of the integrator matter.

### 4.2 Damping Term Has the Wrong Sign

Theory (slide 19):

```text
a_d = (1/m_m)(f_d - f_s) - (d_m/m_m) ẋ
```

- `x` increases into the surface (penetration).
- `ẋ > 0` means moving deeper into contact.
- The term `-d_m ẋ` always **opposes** motion (dissipative).

In the implementation:

```cpp
double v_n_current = dp_vel.dot(-contact_normal_) / dt_;
double d_term      = -kd * v_n_current;
```

- Let `x` be along `contact_normal_` (positive into the surface).
- Penetration velocity should be:

  ```text
  ẋ ∝ v_base · contact_normal_
  ```

- Instead, we use `v_n_current` along `-contact_normal_`, which is effectively:

  ```text
  v_n_current ≈ -ẋ
  ```

- D term becomes:

  ```text
  d_term = -kd * v_n_current = -kd * (-ẋ) = +kd * ẋ
  ```

So:
- When moving **into** the surface (`ẋ > 0`):
  - `d_term > 0` → increases the approach command instead of damping it.
- When moving **out of** the surface (`ẋ < 0`):
  - `d_term < 0` → increases the retract command instead of damping.

In other words, the current “damping” is actually **anti-damping** with respect to the physical penetration velocity. It injects energy into the motion instead of removing it, which is the opposite of the PDF law.

The logs reflect this:
- After impact (`F_n ≈ 260 N`), `v_n` immediately saturates to `-0.0100` and stays there as `F_n` decays through 73, 20, and down toward 5 N, rather than smoothly reducing as the velocity and error drop.

### 4.3 Integral Term vs. Pure P

The De Luca model uses **no integral** on force; instead, it relies on:
- Known contact stiffness `k_c`, and
- An impedance model tuned for desired dynamics.

In contrast, this implementation includes an integral with gating and decay:

```cpp
if (in_contact && std::abs(error) <= error_band) {
  integral = clamp(integral + error * dt, -i_limit, i_limit);
} else {
  integral *= 0.95;
}
```

Consequences:
- While force error is *small and negative* (i.e., near the target but still in heavy contact), the integrator accumulates negative bias.
- When the robot retracts and `F_n` crosses through the target band (e.g., 20 → 5 N), `v_n` is still saturated at `-v_normal_max`, and the integrator may retain enough negative value to keep the command saturated even when `error` is tiny.
- When contact is lost (`F_n=0`), the integral decays, but not instantly; the loop still has memory of past heavy contact states.

This matches the logs where, near `F_n ≈ 5 N`, the error is only around -0.1 N but `v_n` remains at `-0.0100`.

---

## 5. How All This Produces the Observed Oscillation

Putting it together:

1. **READY in preload**  
   - In `READY`, control is effectively idle:
     - `state_ == State::READY` short-circuits the switch; `v_cmd` remains zero.
   - The log shows `F_n ≈ 50–100 N` in READY, with `v_n=0`.
   - So the robot is *already in hard contact* with the environment before SEEK begins.

2. **SEEK approach – saturated velocity into the surface**  
   - After `start()`, state switches to `SEEK`.
   - With `F_n=0`, `error=5`, `p_term=0.01`, the controller saturates to `v_n=+0.0100`.
   - Because of the stiffness of the environment and underlying Cartesian controller behaviour, this yields a step impact in force when contact is established.

3. **Impact – large overshoot and immediate saturated retract**  
   - First contact frame: `F_n` jumps to ~260 N in less than the log period.
   - Now `error=5 - 260 ≈ -255 N`, `p_term=-0.51`, so the controller saturates to `v_n=-0.0100`.
   - The D term, due to its wrong sign, reinforces retraction while the robot is moving out of the contact.

4. **Overshoot decay – force falls, but command stays saturated**  
   - As the robot retracts, `F_n` decays 260 → 73 → 20 → 5 N.
   - Because P alone is still enough to push the command to saturation for `|error| > 5 N`, and D is anti-damping, `v_n` remains locked at `-0.0100`.
   - When `F_n ≈ 20 N`, `error ≈ -15`, the controller still wants ~-0.03 m/s; clamp keeps it at -0.01.

5. **Near target – integral and anti-damping prevent settling**  
   - Eventually `F_n` passes through the target band (e.g., ~5 N).
   - `error` is small (≈ -0.1), but the integrator has accumulated a negative bias during the in-band period with negative error.
   - D term, still with the wrong sign, continues to reinforce retraction as long as the end-effector is moving out of contact.
   - Combined, `p + i + d` remains below `-max_out`, so `v_n` stays at `-0.0100` even when the force is within tolerance.

6. **Loss of contact and cycle repeat**  
   - Continued retraction eventually breaks contact; `F_n` falls to 0, `contact=N`.
   - Now `error=+5`, the loop saturates to `v_n=+0.0100` and pushes back into the surface from free space.
   - The whole cycle repeats: 0 → large impact (260 N) → overshoot decay → near-target with continued retraction → contact lost → approach from free space.

This behaviour is exactly what Eppinger & Seering (referenced in the PDF) warn about: a non-linear, heavily saturated force loop acting against a stiff environment behaves like a bang–bang controller with impacts, not like the smooth impedance-controlled contact described by the linear model.

---

## 6. High-Level Solution Direction (Conceptual Plan)

This section only outlines the conceptual fix; it is **not** the code change itself. The goal is to realign the implementation with the theory from `16_HybridControl.pdf`.

The main elements:

1. **Correct the damping term sign and direction**
   - Define penetration velocity consistently with the contact normal:

     ```cpp
     double xdot = v_base.dot(contact_normal_);  // positive into surface
     ```

   - Implement damping in the same sign as slide 19:

     ```cpp
     // a_d ~ k_p * (F_d - F_s) - k_d * xdot
     d_term = -kd * xdot;
     ```

   - This ensures damping always *opposes* motion, removing energy from the contact.

2. **Reduce saturation and keep the loop mostly linear**
   - Choose `kp` and `v_normal_max` so that:

     ```text
     v_n = kp * (F_d - F_s)
     ```

     stays within `±v_normal_max` over the expected force range.
   - Example: if desired force error range is ±10 N and you want max speed 0.01 m/s:

     ```text
     kp ≈ v_normal_max / max_error ≈ 0.01 / 10 = 0.001
     ```

   - This keeps the controller in the linear region around contact, which matches the impedance model assumption.

3. **Re-evaluate the need for integral**
   - The De Luca impedance force control law is **P + damping** only; steady-state performance is handled by the stiffness and the model.
   - For this application, integral:
     - Can be reduced or eliminated, or
     - Used only as a very slow bias with strong anti-windup, to compensate for systematic offsets (e.g., biases in the F/T sensor).

4. **Approach dynamics consistent with theory**
   - Slide 19 explicitly states that the same law is used before contact, guaranteeing:

     ```text
     ẋ_ss = f_d / d_m
     ```

   - In our case, that suggests:
     - Using the same P + damping law before and after contact.
     - Setting `v_normal_max` such that the approach speed is determined primarily by the damping term `d_m`, not clamped at an arbitrary limit.

5. **Test strategy**
   - Start in simulation with:
     - Reduced environment stiffness (so impacts are less violent).
     - Lower gains (`kp`, potentially no `ki` at first, modest `kd` with correct sign).
   - Check that:
     - Force ramps up smoothly when approaching the dome.
     - The loop settles near `F_d` with modest overshoot and no repeated loss/gain of contact.
   - Only then reintroduce integral (if needed) and tune for hardware.

---

## 7. Summary

From logs, code, and theory:
- The **sign error in the damping term**, combined with **aggressive saturation** and a **non-zero integral**, is the root cause of the oscillatory “bang–bang” normal-force behaviour.
- The implementation diverges from De Luca’s slide 19 impedance-based force control, which assumes:
  - Linear, mostly unsaturated dynamics,
  - Pure P on force error,
  - Properly signed velocity damping.
- The observed pattern (0 → 260 N → decay → 5 N → 0, with `v_n` mostly ±max) is exactly what one expects from a saturated, anti-damped force loop against a very stiff environment.

The fix, at a high level, is to:
- Align the velocity damping with the theoretical model (sign and direction).
- Retune gains and saturation so that the loop behaves linearly near the setpoint.
- Minimize or carefully gate integral action, especially across contact transitions.

This document is intended as the conceptual basis for a clean, theory-backed refactor of the normal-force loop, not the patch itself. Once the principles above are acceptable, we can encode them into a concrete change set and a test procedure (Phase 2 in `AGENTS.md` / `plan.md`).

