# Custom DWA Local Planner in ROS2 Humble

This repository contains a fully custom implementation of the **Dynamic Window Approach (DWA)** local planner, built from scratch without using `nav2_dwb_controller`. The planner drives a differential-drive robot inside Gazebo, performs real-time trajectory sampling, evaluates costs, selects the optimal command velocity, and visualizes trajectories in RViz using `MarkerArray` and `nav_msgs/Path`.

---

## ✨ Features

- Custom DWA implementation in ROS2 (Python)
- Subscribes to `/odom` and `/depth_camera/points`
- Publishes velocity commands to `/cmd_vel`
- Evaluates sampled trajectories based on:
  - Heading cost
  - Clearance cost
  - Velocity cost
- Visualizes planned trajectories in RViz using MarkerArray
- Publishes robot path as `nav_msgs/Path` with QoS durability
- Gazebo simulation with static transform publishers

---

## ⚙️ System Requirements

| Component | Version |
|----------|----------|
| ROS2 | Humble |
| Gazebo | Classic |
| Rviz2 | Latest |
| Ubuntu | 22.04 |

---

## 🧠 Dynamic Window Approach (DWA)

### 1. **Velocity Search Space**
The dynamic window restricts velocity space based on dynamic limits:

\[
V_d = \{ (v, \omega) \mid 
v_{\min} \le v \le v_{\max},\;
\omega_{\min} \le \omega \le \omega_{\max} \}
\]

where:

\[
v_{\min} = v_t - a_{\max} \cdot \Delta t,\quad
v_{\max} = v_t + a_{\max} \cdot \Delta t
\]

\[
\omega_{\min} = \omega_t - \dot{\omega}_{\max} \cdot \Delta t,\quad
\omega_{\max} = \omega_t + \dot{\omega}_{\max} \cdot \Delta t
\]

---

### 2. **Trajectory Prediction**
For each sampled pair \( (v,\omega) \):

\[
x_{t+1} = x_t + v \cdot \cos(\theta) \cdot \Delta t
\]
\[
y_{t+1} = y_t + v \cdot \sin(\theta) \cdot \Delta t
\]
\[
\theta_{t+1} = \theta_t + \omega \cdot \Delta t
\]

---

### 3. **Cost Function**
The best trajectory is chosen by minimizing the weighted sum:

\[
\text{Score} = \alpha \cdot f_{\text{heading}}
 + \beta \cdot f_{\text{clearance}}
 + \gamma \cdot f_{\text{velocity}}
\]

#### **Heading Cost**
\[
f_{\text{heading}} = \pi - \left| \text{atan2}(y_g - y_t,\; x_g - x_t) - \theta_t \right|
\]

#### **Clearance Cost**
\[
f_{\text{clearance}} = \min_{i} \sqrt{(x_t - x_i)^2 + (y_t - y_i)^2}
\]

#### **Velocity Cost**
\[
f_{\text{velocity}} = v
\]

---

## 🚀 Run Instructions

### Build the workspace
```bash
colcon build --symlink-install
source install/setup.bash
