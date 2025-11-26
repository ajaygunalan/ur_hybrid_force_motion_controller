<img src="https://r2cdn.perplexity.ai/pplx-full-logo-primary-dark%402x.png" style="height:64px;margin-right:32px"/>

# fine me github repository which imelmtn like this dont fine mine basically I need hybrid force motion controller implemenataion code provide the linl

Excellent! Now let me compile the comprehensive results with all the repositories I found. Based on the comprehensive research, here's the information:

## GitHub Repositories for Hybrid Force-Motion Controller Implementations

Your code implements **hybrid force-motion control with friction-compensated surface normal estimation**, closely based on the paper "Hybrid Force Motion Control with Estimated Surface Normal for Manufacturing Applications" (arXiv:2404.04419). Here are the most relevant open-source implementations and related repositories:

### **Most Directly Related**

**1. Yifan Hou's Force Control Implementation**[^1]

- **Repository**: `yifan-hou/force_control`
- **Link**: https://github.com/yifan-hou/force_control
- **Focus**: 6D Cartesian space admittance control with hybrid force-velocity control
- **Features**: Virtual spring-mass-damper-friction system, online parameter updates, FT sensor integration
- **Language**: C++ with ROS bindings

**2. Yifan Hou's ICRA 2019 Hybrid Control**[^2]

- **Repository**: `yifan-hou/pub-icra19-hybrid-control`
- **Link**: https://github.com/yifan-hou/pub-icra19-hybrid-control
- **Focus**: Robust execution of contact-rich motion plans using hybrid force-velocity control
- **Paper**: "Robust Execution of Contact-Rich Motion Plans by Hybrid Force-Velocity Control" (ICRA 2019)
- **Language**: MATLAB with algorithm implementations


### **ROS2-Based Implementations**

**3. UR3 Admittance Controller**[^3]

- **Repository**: `woo-rookie/ur3_admittance_controller`
- **Link**: https://github.com/woo-rookie/ur3_admittance_controller
- **Focus**: ROS-based admittance control for UR3 arm
- **Features**: Real robot and simulation support, Cartesian velocity control
- **Language**: C++
- **Status**: Development (DEVEL)

**4. Cartesian Impedance Controller**[^4]

- **Repository**: `matthias-mayr/Cartesian-Impedance-Controller`
- **Link**: https://github.com/matthias-mayr/Cartesian-Impedance-Controller
- **Focus**: Cartesian impedance control for manipulators (KUKA LWR, Franka Emika)
- **Features**: Runtime parameter tuning, nullspace configurations, jerk limitation
- **Language**: C++ with ROS integration

**5. UR Admittance Control Demo**[^5]

- **Repository**: `YuHoChau/UR_Admittance`
- **Link**: https://github.com/YuHoChau/UR_Admittance
- **Focus**: Variable admittance control for UR manipulators
- **Features**: Simulation in Gazebo, configurable admittance parameters
- **Language**: C++ (58.3%), CMake
- **ROS Versions**: Melodic/Noetic

**6. KUKA LWR Force-Position Controller**[^6]

- **Repository**: `xEnVrE/lwr_force_position_controller`
- **Link**: https://github.com/xEnVrE/lwr_force_position_controller
- **Focus**: Hybrid force/position control for KUKA LWR4+ manipulator
- **Features**: Cartesian inverse dynamics, hybrid impedance control, FT sensor integration
- **Language**: C++ with ROS
- **Controllers Included**: position, inverse_dynamics, hybrid_impedance, ft_sensor


### **Contact Normal \& Friction Estimation**

**7. Contact Point Estimation**[^7]

- **Repository**: `kth-ros-pkg/contact_point_estimation`
- **Link**: https://github.com/kth-ros-pkg/contact_point_estimation
- **Focus**: Contact point and surface normal estimation from F/T measurements
- **Method**: Adaptive control for normal estimation
- **Paper Reference**: "Online Contact Point Estimation for Uncalibrated Tool Use" (ICRA 2014)
- **Language**: C++

**8. Friction Model Library**[^8]

- **Repository**: `jchristopherson/friction`
- **Link**: https://github.com/jchristopherson/friction
- **Focus**: Friction models (Coulomb, Lu-Gre, Maxwell, Stribeck, etc.)
- **Use Case**: Suitable for friction compensation in force control
- **Language**: Fortran/Python


### **Advanced Compliance Control**

**9. Robot Admittance Controller**[^9]

- **Repository**: `nbfigueroa/robot_admittance_controller`
- **Link**: https://github.com/nbfigueroa/robot_admittance_controller
- **Focus**: Admittance and impedance control for torque-controlled manipulators
- **Features**: Hybrid force/position algorithms for UR5
- **Language**: C++

**10. Cartesian Impedance with Friction Compensation**[^10]

- **Repository**: `LucasG2001/cartesian_impedance_control`
- **Link**: https://github.com/CurdinDeplazes/cartesian_impedance_control
- **Focus**: ROS2 Cartesian impedance control with friction compensation (Luba 2024 method)
- **Platforms**: Franka Emika Panda
- **Features**: Anti-windup integrator, variable stiffness modes
- **Language**: C++


### **Admittance Control in ROS2_Control**

**11. Official ROS2 Admittance Controller**[^11]

- **Documentation**: https://control.ros.org/humble/doc/ros2_controllers/admittance_controller/doc/userdoc.html
- **Package**: `ros2_controllers`
- **Framework**: ROS2 Control plugin-based architecture
- **Features**: Zero-force control, external kinematics plugin support
- **Status**: Official, well-maintained


### **Alternative Approaches (Simulink/MATLAB)**

**12. Admittance Control Simulink**[^12]

- **Repository**: `mincasurong/Admittance-Control-Simulink`
- **Link**: https://github.com/mincasurong/Admittance-Control-Simulink
- **Focus**: MATLAB/Simulink-based admittance control simulation
- **Language**: MATLAB/Simulink


### **Key Implementation Patterns from These Repos**

From your code analysis and these repositories, the core patterns include:

**Friction Compensation (like your Nasiri-style estimator)**:

- Project force onto velocity direction: $f_\parallel = f_s \cdot \hat{v}$
- Perpendicular component: $f_\perp = f_s - f_\parallel \hat{v}$
- Friction coefficient estimation: $\mu = |f_\parallel| / |f_\perp|$
- Normal estimate: $\hat{n} = (f_s - \mu |f_\perp| \hat{v}) / |f_s - \mu |f_\perp| \hat{v}|$

**PI Control with Anti-Windup** (like your PIController struct):

- Integral decay when outside error band
- Damping term for contact normal velocity
- Output saturation

**State Machine Architecture**:

- IDLE → READY → SEEK → DWELL → SLIDE → DONE/FAULT
- Contact loss detection and recovery
- Safety monitoring


### **Recommended Starting Points**

1. **For ROS2 integration**: Check the official `ros2_controllers/admittance_controller`  and `LucasG2001/cartesian_impedance_control`[^10][^11]
2. **For friction estimation**: Reference `kth-ros-pkg/contact_point_estimation`[^7]
3. **For hybrid force-velocity theory**: Study `yifan-hou/pub-icra19-hybrid-control`  and the ICRA 2019 paper[^2]
4. **For polishing/manufacturing applications**: Look at research papers on hybrid compliance for surface treatment tasks

Your implementation is sophisticated with proper friction compensation, which makes it more aligned with research-level implementations than typical industrial robot controllers.
<span style="display:none">[^13][^14][^15][^16][^17][^18][^19][^20][^21][^22][^23][^24][^25][^26][^27][^28][^29][^30][^31][^32][^33][^34][^35][^36][^37][^38][^39][^40][^41][^42][^43][^44][^45][^46]</span>

<div align="center">⁂</div>

[^1]: https://github.com/yifan-hou/force_control

[^2]: https://github.com/yifan-hou/pub-icra19-hybrid-control

[^3]: https://github.com/woo-rookie/ur3_admittance_controller

[^4]: https://github.com/matthias-mayr/Cartesian-Impedance-Controller

[^5]: https://github.com/YuHoChau/UR_Admittance

[^6]: https://github.com/xEnVrE/lwr_force_position_controller

[^7]: https://github.com/kth-ros-pkg/contact_point_estimation

[^8]: https://github.com/jchristopherson/friction

[^9]: https://www.github-zh.com/projects/244076788-robot_admittance_controller

[^10]: https://github.com/CurdinDeplazes/cartesian_impedance_control

[^11]: https://control.ros.org/humble/doc/ros2_controllers/admittance_controller/doc/userdoc.html

[^12]: https://github.com/mincasurong/Admittance-Control-Simulink

[^13]: hybrid_force_motion_node.cpp

[^14]: https://gist.github.com/justagist/3530cd05fbc17a6b1ce8832f8f5e2f8c

[^15]: https://develop.realman-robotics.com/en/robot/demo/ros2/foxyforcePositionControl/

[^16]: https://docs.picknik.ai/how_to/robotics_applications/velocity_force_control/

[^17]: https://github.com/ros-industrial/ur_modern_driver/issues/356

[^18]: https://arxiv.org/pdf/2404.04419.pdf

[^19]: https://arxiv.org/html/2404.04419v1

[^20]: https://www.youtube.com/watch?v=wZPgnpMOXHI

[^21]: https://arxiv.org/html/2504.12441v1

[^22]: https://pmc.ncbi.nlm.nih.gov/articles/PMC12086237/

[^23]: https://github.com/topics/surface-normals-estimation

[^24]: https://www.scribd.com/document/931122260/Robot-force-control-for-surface-polishing

[^25]: https://ipg.idsia.ch/preprints/roveda2023j.pdf

[^26]: https://github.com/CLeARoboticsLab/friction-estimator

[^27]: https://github.com/arxiv

[^28]: https://pmc.ncbi.nlm.nih.gov/articles/PMC10820524/

[^29]: http://arxiv.org/list/cs/2024-04?skip=1885\&show=2000

[^30]: https://control.ros.org/master/_downloads/a8d97b415fe7e727e3013e24d5b7170a/2021-10_ROS_World-Admittance_Control_in_ROS2.pdf

[^31]: https://xianyicheng.github.io/files/cheng_isrr19.pdf

[^32]: https://github.com/SamoaChen/2-Linkages-Robotic-Arm-Hybrid-Position-Force-Control

[^33]: https://www.catalyzex.com/author/Long Wang

[^34]: https://pmc.ncbi.nlm.nih.gov/articles/PMC11089096/

[^35]: https://github.com/aboulch/normals_Hough

[^36]: https://www.diva-portal.org/smash/get/diva2:1767802/FULLTEXT01.pdf

[^37]: https://homepages.inf.ed.ac.uk/msridhar/Papers/mMathew_thesis20.pdf

[^38]: https://arxiv.org/html/2505.04978v1

[^39]: https://core.ac.uk/download/479413596.pdf

[^40]: https://github.com/udacity/FCND-Estimation-CPP

[^41]: https://raw.githubusercontent.com/mlresearch/v270/main/assets/wang25f/wang25f.pdf

[^42]: https://www.ri.cmu.edu/app/uploads/2019/03/ICRA19Hybrid.pdf

[^43]: https://opentextbooks.clemson.edu/wangrobotics/chapter/motion-and-force-control/

[^44]: https://yuxng.github.io/Courses/CS6301Fall2023/lecture_21_motion_control_velocity_force_torque.pdf

[^45]: https://yuxng.github.io/Courses/CS6301Fall2022/lecture_19_control_force_hybrid_impedance.pdf

[^46]: https://github.com/rafaelbarretorb/trajectory_tracking_control

