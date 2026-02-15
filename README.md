
# 🦾 Franka Panda Color Sorting Robot 

**ROS 2 Humble/Jazzy • MoveIt 2 • Gazebo • OpenCV • Docker**  
Franka Panda detects Red/Green/Blue cubes → computes 3D poses → picks → sorts into color bins. **Full tutorial + Docker demo ready**.

[![▶️ Demo Video](media/demo.mp4)](https://github.com/rayeneayarii/panda_color_sorter/blob/main/media/demo.mp4)

*Gazebo + RViz2 + real-time sorting*

---

## 📋 Table of Contents
- [Quick Demo (Docker)](#-quick-demo-docker)
- [Full Native Setup](#-full-native-setup)
- [Launch Files](#-launch-files)
- [Usage Commands](#-usage)
- [Customization](#-customization)
- [Troubleshooting](#-troubleshooting)
- [Project Structure](#-project-structure)

---

## 🚀 Quick Demo (Docker - 2 Minutes)

**Prerequisites**: Docker + NVIDIA GPU (optional).

```bash
# 1. GUI forwarding
xhost +local:docker

# 2. One-command demo (spawns Gazebo + Panda + MoveIt)
docker run -it --rm --name panda_demo \
  --network host \
  -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --gpus all \
  curiousutkarsh/franka_panda_color_sorter:humble bash -c "
    cd /root/panda_ws && 
    source install/setup.bash && 
    ros2 launch panda_bringup pick_and_place.launch.py
  "
```

**Terminal 2** (sort cubes):
```bash
# Attach to container
docker exec -it panda_demo bash
source /root/panda_ws/install/setup.bash

# Sort by color
ros2 run pymoveit2 pick_and_place.py target_color:=R  # 🔴 Red
ros2 run pymoveit2 pick_and_place.py target_color:=G  # 🟢 Green
ros2 run pymoveit2 pick_and_place.py target_color:=B  # 🔵 Blue
```

**Stop**: `Ctrl+C` → `docker stop panda_demo`.

---

## 🏗 Full Native Setup (ROS 2 Jazzy)

### 1. Dependencies
```bash
sudo apt update
sudo apt install ros-jazzy-desktop ros-jazzy-gazebo-ros-pkgs \
  ros-jazzy-moveit ros-jazzy-franka-ros2 python3-colcon-common-extensions
```

### 2. Clone & Build
```bash
mkdir -p ~/panda_ws/src
cd ~/panda_ws/src
git clone https://github.com/rayeneayarii/panda_color_sorter.git .
cd ~/panda_ws
rosdep install -y --from-paths src --ignore-src
colcon build --symlink-install
source install/setup.bash
echo "source ~/panda_ws/install/setup.bash" >> ~/.bashrc
```

### 3. Test
```bash
# Full simulation
ros2 launch panda_bringup pick_and_place.launch.py

# Or step-by-step:
ros2 launch panda_description gazebo.launch.py     # Gazebo world
ros2 launch panda_moveit moveit.launch.py          # MoveIt + RViz
ros2 launch panda_controller controller.launch.py  # Controllers
ros2 run panda_vision color_detector               # Vision
```

---

## 🎯 Launch Files Reference

| Launch File | Description | Arguments |
|-------------|-------------|-----------|
| `panda_bringup/pick_and_place.launch.py` | **Full pipeline** (Gazebo + MoveIt + controllers + vision) | `headless:=true` |
| `panda_description/gazebo.launch.py` | Gazebo world + Panda spawn | `world:=empty.world` |
| `panda_moveit/moveit.launch.py` | MoveIt 2 + RViz2 | `robot_description:=panda` |
| `panda_controller/controller.launch.py` | Joint trajectory controller + gripper | `-` |
| `panda_controller/slider_controller.launch.py` | Gripper slider (RViz) | `-` |

**Full pipeline** (recommended):
```bash
ros2 launch panda_bringup pick_and_place.launch.py headless:=false
```

---

## ⌨️ Usage Commands

### 1. Vision Only
```bash
ros2 run panda_vision color_detector
# Output: /color_coordinates → "R,0.600,0.002,1.100"
```

### 2. Motion Only
```bash
ros2 run pymoveit2 pick_and_place.py target_color:=R
# Params: target_color (R/G/B), gripper_open (0.08m)
```

### 3. Debug Topics
```bash
ros2 topic echo /color_coordinates
ros2 topic hz /joint_states
ros2 service list | grep gripper
```

### 4. RViz2 Visualization
```bash
ros2 run rviz2 rviz2 -d src/panda_moveit/rviz/moveit.rviz
```

---

## 🛠 Customization Guide

### Change Colors
`panda_vision/color_detector.py` → edit `color_ranges`:
```python
"R1": [(0, 120, 70), (10, 255, 255)],    # Red low
"R2": [(170,120,70), (180,255,255)],     # Red high
"G":  [(35, 60, 60), (85, 255, 255)],    # Green
"B":  [(90, 60, 60), (140,255,255)]      # Blue
```

### New Objects/World
1. Add SDF/DAE models to `panda_description/models/`
2. Update `panda_description/worlds/empty.world`
3. Adjust bin positions in `pick_and_place.py`

### Parameters
```bash
ros2 run pymoveit2 pick_and_place.py \
  target_color:=R gripper_open:=0.08 plan_time:=5.0
```

---

## 🔍 Project Structure

```
panda_color_sorter/
├── README.md                    # This file
├── docker/run_humble.sh         # Docker launcher
├── media/                       # demo.mp4, demo.png
└── src/Franka_Panda_Color_Sorting_Robot/
    ├── panda_bringup/           # 🧩 Launches
    │   └── launch/pick_and_place.launch.py  ← MAIN
    ├── panda_description/       # 🔧 URDF + Gazebo
    │   ├── urdf/panda.urdf.xacro
    │   └── worlds/empty.world
    ├── panda_vision/            # 👁️  OpenCV detection
    │   └── color_detector.py
    ├── panda_moveit/            # 🤖 MoveIt 2 config
    │   ├── config/panda.srdf
    │   └── launch/moveit.launch.py
    ├── panda_controller/        # ⚙️  Controllers
    └── pymoveit2/               # 🚀 Pick/place logic
        └── pick_and_place.py    ← MAIN LOGIC
```

---

## ❌ Troubleshooting

| Issue | Fix |
|-------|-----|
| **No GUI in Docker** | `xhost +local:docker` + restart Docker |
| **MoveIt planning fails** | Check `/diagnostics`, increase `plan_time:=10.0` |
| **"No transform camera→panda_link0"** | `ros2 run tf2_ros static_transform_publisher ...` |
| **Gripper doesn't close** | `ros2 topic pub /panda/gripper_cmd ...` |
| **Colcon build fails** | `sudo rosdep init && rosdep update` |

**Logs**: `ros2 launch ... --ros-args --log-level DEBUG`

---

## 🎓 What I Learned / Challenges

1. **TF2 transforms**: Camera pixels → robot coordinates (matrix math + projection).
2. **MoveIt 2 debugging**: Collision checking, planning scenes, servo mode.
3. **Docker ROS 2**: GUI forwarding, workspace mounting, GPU passthrough.
4. **Real-time vision**: HSV tuning, contour filtering, 3D reprojection.
5. **Controllers**: Joint trajectory vs position, gripper limits.

**Hardest**: HSV ranges vary by lighting → adaptive thresholding + dual red range.

---

## 👨‍💻 Author

**Rayene Jazzy**  
*MSc Embedded Systems – ISSAT Sousse*  
**Portfolio**: Robotics • ROS 2 • STM32 • Computer Vision  

[![LinkedIn](https://img.shields.io/badge/LinkedIn-Connect-blue?style=flat&logo=linkedin)]([https://linkedin.com/in/rayenejazzy](https://www.linkedin.com/in/rayene-ayarii/))
📧 rayene.ayari03@gmail.com


---

## 📄 License
MIT © 2026. See [LICENSE](LICENSE).
