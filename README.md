# ros2_slam_nav2

# 🚀 Autonomous Navigation using ROS 2 (SLAM + Localization + Nav2)

This project implements a complete autonomous navigation pipeline using **ROS 2 Humble**, **Gazebo**, **SLAM Toolbox**, and **Nav2**.

The robot operates inside a maze simulation environment and performs:

- ✅ SLAM-based map generation  
- ✅ Map saving  
- ✅ AMCL-based localization  
- ✅ Fully autonomous navigation using Nav2  

---

# 📁 Workspace Structure

```
ros2_ws/
├── src/
│   └── my_publisher/
│       ├── config/
│       │   └── nav2_params.yaml
│       ├── launch/
│       │   └── gazebo.launch.py
│       ├── maps/
│       │   ├── maze_map.yaml
│       │   └── maze_map.pgm
│       ├── urdf/
│       │   └── robot.urdf
│       ├── worlds/
│       │   └── maze_world.world
│       ├── models/
│       ├── setup.py
│       └── package.xml
```

---

# 🧠 System Architecture

## 🔗 Miro Architecture Diagram

Full architecture diagram available here:

👉 **https://miro.com/app/board/uXjVG9EpXQI=/?share_link_id=894400759792**

---

## TF Tree (Final Working Configuration)

```
map
 └── odom
      └── base_link
           ├── wheels
           ├── lidar
           └── camera
```

---

## Core Components

- Gazebo Simulation
- SLAM Toolbox
- Map Server
- AMCL (Localization)
- Nav2 Stack:
  - Planner Server
  - Controller Server
  - BT Navigator
  - Global & Local Costmaps
  - Lifecycle Managers

---

# 🗺️ PHASE 1 — SLAM (Mapping)

## 1️⃣ Launch Gazebo

```bash
ros2 launch my_publisher gazebo.launch.py
```

## 2️⃣ Launch SLAM

```bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true
```

## 3️⃣ Teleoperate Robot

Drive the robot manually until the maze is fully mapped.

Verify map publishing:

```bash
ros2 topic echo /map
```

---

# 💾 PHASE 2 — Save Map

Create maps directory if not existing:

```bash
mkdir -p ~/ros2_ws/src/my_publisher/maps
```

Save the generated map:

```bash
ros2 run nav2_map_server map_saver_cli \
-f ~/ros2_ws/src/my_publisher/maps/maze_map
```

Generated files:

```
maze_map.pgm
maze_map.yaml
```

Stop SLAM after saving.

---

# 📍 PHASE 3 — Localization (AMCL)

## 1️⃣ Launch Gazebo

```bash
ros2 launch my_publisher gazebo.launch.py
```

## 2️⃣ Launch Localization

```bash
ros2 launch nav2_bringup localization_launch.py \
map:=/home/<your_username>/ros2_ws/src/my_publisher/maps/maze_map.yaml \
use_sim_time:=true \
params_file:=/home/gourav047/ros2_ws/src/my_publisher/config/nav2_params.yaml
```

Wait until:

```
Managed nodes are active
```

---

## 3️⃣ Launch RViz

```bash
ros2 launch nav2_bringup rviz_launch.py
```

Set:

```
Fixed Frame = map
```

---

## 4️⃣ Set Initial Pose (Critical Step)

Click **2D Pose Estimate** in RViz and place the robot correctly inside the map.

Verify TF:

```bash
ros2 run tf2_tools view_frames
```

Expected TF chain:

```
map → odom → base_link
```

Localization is now complete.

---

# 🧭 PHASE 4 — Navigation (Nav2)

Launch navigation using the same parameter file:

```bash
ros2 launch nav2_bringup navigation_launch.py \
use_sim_time:=true \
params_file:=/home/gourav047/ros2_ws/src/my_publisher/config/nav2_params.yaml
```

Wait for:

```
lifecycle_manager_navigation: Managed nodes are active
```

---

# 🎯 Send Navigation Goal

In RViz:

- Click **Nav2 Goal**
- Select target location inside maze

Robot will:

- Plan global path
- Generate local trajectory
- Avoid obstacles
- Reach goal autonomously

---

# ⚙️ Important Configuration Notes

## Laser Topic

Robot publishes:

```
/gazebo_ros_laser/out
```

Ensure `nav2_params.yaml` contains:

```yaml
scan_topic: "/gazebo_ros_laser/out"
```

And inside local costmap:

```yaml
observation_sources: laser
laser:
  topic: /gazebo_ros_laser/out
```

---

# 🛠 Common Issues & Fixes

## ❌ Frame [map] does not exist

Cause:
- Initial pose not set
- AMCL not receiving scan

Fix:
- Set 2D Pose Estimate
- Verify correct scan topic

---

## ❌ Robot is out of bounds of costmap

Cause:
- Incorrect initial pose
- Incorrect map origin

Fix:
- Check `maze_map.yaml` origin values
- Place robot within valid map area

---

## ❌ Laser not detected

Check:

```bash
ros2 topic info /gazebo_ros_laser/out
```

Ensure subscription count > 0.

---

# ✅ Complete Execution Order

```
1. Launch Gazebo
2. Run SLAM (only during mapping phase)
3. Save Map
4. Launch Localization (AMCL)
5. Set Initial Pose
6. Launch Navigation
7. Send Goal
```

---

# 📌 Technologies Used

- ROS 2 Humble
- Gazebo
- SLAM Toolbox
- Nav2
- AMCL
- RViz2

---

# 👨‍💻 Author

**gourav-047**

---
