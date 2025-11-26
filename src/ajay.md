To visualize your `hybrid_force_motion_controller` using Rerun, you'll need to integrate the Rerun C++ SDK into your ROS 2 package. This will allow you to log the wrench (force), twist (velocity), internal state (like normal force and tangential distance), and the calculated contact frame in real-time.

Here are the detailed steps to add Rerun visualization to your existing package.

### Step 1: Add Rerun Dependency

First, ensure the Rerun C++ SDK is installed on your system (e.g., via `pixi`, `conda`, or by downloading the release artifacts). Then, update your package configuration to find and link against it.

**1. Update `package.xml`**
Add the dependency so `colcon` is aware of it (if you are using a system-installed Rerun).

```xml
<depend>rerun</depend>
```

**2. Update `CMakeLists.txt`**
Find the Rerun package and link it to your node executable.

```cmake
# In CMakeLists.txt

# ... existing find_package calls ...
find_package(rerun REQUIRED)

# ... existing executable definition ...
add_executable(hybrid_force_motion_node src/hybrid_force_motion_node.cpp)

# Add rerun_sdk to target_link_libraries
target_link_libraries(hybrid_force_motion_node
  Eigen3::Eigen
  ${orocos_kdl_LIBRARIES}
  rerun::rerun_sdk  # <--- Add this
)
```

### Step 2: Instrument the Code

Now modify `src/hybrid_force_motion_node.cpp` to initialize Rerun and log data during the control loop.

**1. Include the Header**

```cpp
#include <rerun.hpp>
```

**2. Initialize Rerun in the Class**

Add a `rerun::RecordingStream` member to your `HybridForceMotionNode` class and initialize it in the constructor.

```cpp
// In src/hybrid_force_motion_node.cpp

class HybridForceMotionNode : public rclcpp::Node {
public:
  HybridForceMotionNode()
  : Node("hybrid_force_motion_controller"),
    state_(RunState::WAITING_FOR_START),
    phase_(Phase::SEEK),
    rec("hybrid_force_motion") // Initialize Rerun stream with an application ID
  {
    // ... existing constructor code ...
    
    // Spawn the Rerun Viewer (or connect to a listening one)
    rec.spawn(); 
    
    // Log the static base frame so we have a reference
    rec.log("world", rerun::ViewCoordinates::RIGHT_HAND_Z_UP); 
    
    // ...
  }
  
  // ...
  
private:
  rerun::RecordingStream rec; // Member variable
  // ...
```

**3. Log Data in `ControlLoop`**

Update the `ControlLoop` function to log the relevant vectors and scalars. This allows you to see the force vector, velocity command, and contact frame align in 3D space, alongside time-series plots of the magnitudes.

```cpp
// In src/hybrid_force_motion_node.cpp inside ControlLoop()

void ControlLoop() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    // ... existing checks ...

    // Update time based on ROS clock
    rec.set_time_nanos("ros_time", this->now().nanoseconds());

    // --- 1. Log the End-Effector Pose ---
    // Convert Eigen Isometry to Rerun Transform
    auto t = current_pose_.translation();
    Eigen::Quaterniond q(current_pose_.linear());
    
    rec.log(
        "world/tool",
        rerun::Transform3D(
            rerun::Vec3D(t.x(), t.y(), t.z()), 
            rerun::Quaternion(q.x(), q.y(), q.z(), q.w())
        )
    );

    // --- 2. Log Wrench (Force) ---
    // Visualized as a red arrow originating from the tool
    // We scale the vector length slightly for visibility if needed, or rely on the viewer's scaler
    rec.log(
        "world/tool/force",
        rerun::Arrows3D::from_vectors({
            rerun::Vec3D(force_base_.x(), force_base_.y(), force_base_.z())
        }).with_colors({rerun::Color(255, 0, 0)}) // Red for Force
    );

    // --- 3. Log Twist Command (Velocity) ---
    // Visualized as a green arrow originating from the tool
    Eigen::Vector3d lin_vel = last_twist_cmd_.head<3>();
    rec.log(
        "world/tool/velocity",
        rerun::Arrows3D::from_vectors({
            rerun::Vec3D(lin_vel.x(), lin_vel.y(), lin_vel.z())
        }).with_colors({rerun::Color(0, 255, 0)}) // Green for Velocity
    );

    // --- 4. Log Calculated Contact Frame ---
    // This helps debug if the normal estimation is correct
    // The contact frame is constructed in BroadcastContactFrame, we replicate the rotation logic here
    // or move the calculation earlier in the loop to reuse it.
    {
        Eigen::Vector3d z = contact_normal_;
        Eigen::Vector3d x = tangential_dir_;
        Eigen::Vector3d y = z.cross(x).normalized();
        x = y.cross(z).normalized();
        Eigen::Matrix3d R;
        R.col(0) = x; R.col(1) = y; R.col(2) = z;
        Eigen::Quaterniond q_contact(R);

        rec.log(
            "world/tool/contact_frame",
            rerun::Transform3D(
                rerun::Vec3D(0.0, 0.0, 0.0), // Relative to tool
                rerun::Quaternion(q_contact.x(), q_contact.y(), q_contact.z(), q_contact.w())
            )
        );
    }

    // --- 5. Log Scalar Metrics ---
    // These will show up as time-series graphs
    rec.log("metrics/normal_force", rerun::Scalar(normal_force));
    rec.log("metrics/tangential_distance", rerun::Scalar(tangential_distance_));
    rec.log("metrics/state", rerun::Scalar(static_cast<double>(state_))); // 2=RUNNING, 6=FAULT

    // ... existing rest of function ...
}
```

### Step 3: Build and Run

1.  **Rebuild your workspace**:

    ```bash
    colcon build --packages-select hybrid_force_motion_controller
    source install/setup.bash
    ```

2.  **Run the simulation/hardware launch**:

    ```bash
    ros2 launch hybrid_force_motion_controller hybrid_force_motion_sim.launch.py
    ```

3.  **Rerun Viewer**:
    The Rerun Viewer should spawn automatically. You will see:

      * **3D View**: The tool frame moving, with a red arrow for the measured force and a green arrow for the commanded velocity. A coordinate axis will represent the estimated contact frame.
      * **Time Series**: Graphs for `normal_force` (verifying it holds 5N) and `tangential_distance`.
      * **State**: You can see the state machine transitions in the plot.

### Why this helps

  * **Force Vector vs. Normal**: You can visually confirm if your `force_base_` (red arrow) aligns with the Z-axis of your `contact_frame`. If the red arrow tilts while the contact frame stays flat, your friction estimation might be off.
  * **Velocity Alignment**: You can verify if the green velocity vector is strictly perpendicular to the contact normal (tangential motion).
  * **Transient Spikes**: The high-frequency Rerun stream will catch force spikes during the "Contact Seek" phase that might be missed in standard ROS topic echoing.

[Fubu Toy review](https://www.google.com/search?q=https://www.youtube.com/watch%3Fv%3DF7eK67iP_vI)
This video demonstrates setting up a UR5e with ROS 2 and MoveIt 2, providing relevant context for configuring and running the robot hardware or simulation environment used in your project.