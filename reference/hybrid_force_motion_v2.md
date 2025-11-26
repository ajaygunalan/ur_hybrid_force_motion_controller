# Bridging Force and Motion: A Complete Guide to Hybrid Control

When a robot moves through free space, we focus on position and velocity. But when it needs to polish a surface, turn a crank, or insert a screw, the environment fights back. The robot encounters a stiff environment that constrains its motion. We cannot just control motion (or we might smash the robot) and we cannot just control force (or the robot flies off into free space). **Hybrid Force/Motion Control** splits the problem into directions where we control motion and directions where we control force.

## The Tale of Two Constraints

Understanding hybrid control begins with the geometry of interaction.

**Natural Constraints** are imposed by the physical environment—the robot has no choice. Along certain directions, the environment prevents motion (you cannot move *into* a rigid wall), so velocity is zero ($v = 0$). Along free directions, the environment cannot push back (a frictionless surface cannot resist a tangential slide), so reaction force is zero ($f = 0$).

**Artificial Constraints** are reference values we *want* to impose through control. They must be complementary to natural constraints. In directions where motion is naturally allowed, we impose desired velocity ("slide at 5 cm/s"). In directions where motion is constrained, we impose desired contact force ("push with 10 N").

The key insight: these two sets of directions are mutually orthogonal. We control force where we can't move, and motion where we can't push.

## Mathematical Parameterization: $T$ and $Y$

We define a **Task Frame** ($RF_t$) at the contact point and use two matrices to separate twists (velocity) and wrenches (forces).

**Feasible Motion ($T$):** Any feasible velocity twist is parameterized by $k$ velocity parameters $\dot{s}$:

$$\binom{v}{\omega} = T(s)\dot{s}$$

**Reaction Forces ($Y$):** Any admissible reaction wrench is parameterized by $m-k$ force parameters $\lambda$:

$$\binom{f}{\mu} = Y(s)\lambda$$

**The Orthogonality Rule:** Because reaction forces do no work on feasible motions (in an ideal frictionless setting):

$$T^T(s)Y(s) = 0$$

## Selection Matrices in Practice

In simpler cases where the task frame aligns with the environment (like a flat wall), we use **selection matrices** to decide which axes are force-controlled and which are motion-controlled.

A diagonal matrix $\Sigma$ (with 1s and 0s) selects admissible reaction forces. The complementary matrix $(I - \Sigma)$ selects feasible velocities. The robot measures forces and velocities, transforms them into the task frame using rotation matrices, and compares against desired values. The combined control:

$$a = \Sigma \cdot (\text{Force Control}) + (I - \Sigma) \cdot (\text{Motion Control})$$

## Three Illustrative Examples

**Sliding a Cube:** Imagine sliding a cube along a rail. You cannot move sideways ($v_y = 0$) or rotate ($\omega_x = 0$). You want to control sliding speed ($v_x$) and downward force ($f_z$). Here $T$ and $Y$ are constant selection matrices of 0s and 1s.

**Turning a Crank:** The task frame moves and rotates by angle $\alpha$ as the crank turns. The handle can only move tangentially ($v_y$) and rotate freely ($\omega_z$). The matrices $T(\alpha)$ and $Y(\alpha)$ become functions of angle, requiring rotation matrix $R(\alpha)$ to map back to the base frame.

**Inserting a Screw:** The most complex case. Linear velocity $v_z$ couples physically to angular velocity $\omega_z$ by screw pitch $p$: $\omega_z = (2\pi/p)v_z$. The matrix $T$ contains the coupling term rather than just 0s and 1s, representing 1-DOF spiraling motion.

## Force Control via Impedance

"Rigid" environments often have finite stiffness (a sensor or rubber tool tip). Instead of pure force control, an **impedance model** regulates contact by making the robot behave like a mass-spring-damper:

$$m_m \ddot{x} + d_m \dot{x} + k_s(x - x_d) = f_d$$

Here $k_s$ is sensor/contact stiffness, $f_d$ is desired force, and $m_m, d_m$ are tunable parameters.

The elegance lies in the approach phase. When $f_s = 0$ (no contact), the controller guarantees steady-state approach speed $\dot{x}_{ss} = f_d / d_m$. The robot naturally transitions from moving through air to pushing against a surface without switching control laws.

## The Control Strategy: Feedback Linearization

**Exact feedback linearization** cancels the robot's nonlinear dynamics (gravity, Coriolis) and decouples the system into two independent linear channels.

**Motion Channel ($s$):** A second-order system controlled with PD:

$$a_s = \ddot{s}_d + K_D(\dot{s}_d - \dot{s}) + K_P(s_d - s)$$

**Force Channel ($\lambda$):** An algebraic (zero-order) system, typically with PI control for robustness against friction:

$$a_\lambda = \lambda_d + K_I \int (\lambda_d - \lambda)\, dt$$

## The Geometric Filter

Real sensors are noisy and surfaces aren't perfectly rigid. Hybrid control acts as a **geometric filter**: we measure joint positions $q$, velocities $\dot{q}$, and forces $F$, then project onto the ideal model using pseudo-inverses:

$$\dot{s} = T^\#(s) J(q) \dot{q}$$
$$\lambda = Y^\#(s) \binom{f}{\mu}$$

This ensures only data consistent with the geometric model reaches the controller—ignoring, for example, friction forces in a "frictionless" model.

## The Inconsistency Problem

Theory assumes perfection. Reality delivers friction, compliance, and uncertain geometry.

**Friction:** Moving along a surface generates tangential force $f_t = \tan(\gamma) f_n$. This "extra" force appears in a direction that should be free motion, confusing the controller.

**Compliance:** Structural flexibility means the robot might measure velocity ($v_n \neq 0$) in supposedly rigid directions.

**Uncertain Geometry:** We rarely know the exact surface angle.

**Sensor Fusion** solves this via Recursive Least Squares (RLS). Position data estimates the tangent (but is noisy). Force data estimates the normal (but is affected by friction). Fusing both streams lets the robot reconstruct unknown contours—like a cinema film reel with 17cm radius—while maintaining constant 20N contact force.

## Experiments: The MIMO Robot

Historical experiments with the MIMO-CRF robot (6-axis force/torque sensor) demonstrated key behaviors. In force control mode with zero reference force, pushing the sensor causes the robot to yield—like a joystick, moving to nullify applied force. In hybrid execution, the robot moved in a straight line (velocity control) while maintaining constant force (20N or 40N) against a surface. Even when force requirements doubled, position error remained negligible ($10^{-4}$ meters).

## Alternative Architecture: Inner-Outer Loop

An alternative implementation uses nested loops. The **inner loop** runs standard position/orientation control. The **outer loop** runs force control that generates position commands. If the robot pushes against a wall without reaching desired force, the outer loop commands motion *into* the wall. The inner loop obeys, increasing contact pressure until target force is achieved.

## Industrial Application: Deburring Windshields

Robotized deburring of car windshields removes excess plastic glue (PVB) from uneven glass edges. The robot follows the nominal path (motion control) while a pneumatic actuator applies controlled normal force to the blade (force control). This "poor man's" force control via pneumatics effectively decouples the task: the robot handles trajectory, the tool handles contact pressure.

## Summary

Hybrid control manages the complex interplay of friction, compliance, and geometry. By defining what is possible (natural constraints) and what is desired (artificial constraints), parameterizing directions with orthogonal matrices $T$ and $Y$, and applying feedback linearization, robots interact with the physical world as skillfully as they move through empty space. Whether using sophisticated sensor fusion or clever mechanical design, the goal is making the robot aware of the physical constraints it encounters.
