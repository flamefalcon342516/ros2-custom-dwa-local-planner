# Custom DWA Local Planner in ROS2 Humble

This repository contains a custom implementation of the **Dynamic Window Approach (DWA)** local planner for ROS2 Humble and Gazebo. The planner performs real-time trajectory sampling, cost evaluation, obstacle avoidance, command velocity publishing, and trajectory visualization in RViz.

---

## ✨ Features

- Custom DWA algorithm (no Nav2 or DWB)
- Works with `/odom`, `/depth_camera/points`, and `/cmd_vel`
- RViz trajectory visualization using `MarkerArray`
- Publishes path using `nav_msgs/Path` with durable QoS for continuous visibility
- Supports multiple waypoint navigation

---

## ⚙️ System Requirements

| Component | Version |
|----------|----------|
| ROS2 | Humble |
| Gazebo | Classic |
| Ubuntu | 22.04 |
| RViz2 | Latest |

---

## 🧠 Dynamic Window Approach (Theory)

### **Velocity Window**
Search space for valid \( v \) and \( \omega \):

$$
V_d = \{(v, \omega)\ |\ v_{\min} \le v \le v_{\max},\ \omega_{\min} \le \omega \le \omega_{\max}\}
$$

Dynamic limits based on robot acceleration:

$$
v_{\min} = v_t - a_{\max} \cdot \Delta t,\qquad
v_{\max} = v_t + a_{\max} \cdot \Delta t
$$

$$
\omega_{\min} = \omega_t - \dot{\omega}_{\max} \cdot \Delta t,\qquad
\omega_{\max} = \omega_t + \dot{\omega}_{\max} \cdot \Delta t
$$

---

### **Trajectory Prediction**

$$
x_{t+1} = x_t + v \cdot \cos(\theta) \cdot \Delta t
$$

$$
y_{t+1} = y_t + v \cdot \sin(\theta) \cdot \Delta t
$$

$$
\theta_{t+1} = \theta_t + \omega \cdot \Delta t
$$

---

### **Cost Function**

The best velocity command is selected using:

$$
Score = \alpha f_{heading} + \beta f_{clearance} + \gamma f_{velocity}
$$

#### **Heading Cost**
$$
f_{heading} = \pi - \left| \mathrm{atan2}(y_g - y_t,\ x_g - x_t) - \theta_t \right|
$$

#### **Clearance Cost**
$$
f_{clearance} = \min_i \sqrt{(x_t - x_i)^2 + (y_t - y_i)^2}
$$

#### **Velocity Cost**
$$
f_{velocity} = v
$$

---

## 🚀 Run Instructions

Build the workspace:
```bash
colcon build --symlink-install
source install/setup.bash
