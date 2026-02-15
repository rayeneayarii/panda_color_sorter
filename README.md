
# 🦾 Franka Panda Color Sorter Robot

**ROS 2 Humble/Jazzy • MoveIt 2 • Gazebo • OpenCV • Docker**  
Complete pick-and-place pipeline: Franka Emika Panda arm detects colored cubes (Red/Green/Blue) via camera → computes 3D coordinates → plans motion → sorts into color-coded bins.

[![Demo](media/demo.png)](https://github.com/rayeneayarii/panda_color_sorter/assets/123456/demo.mp4)
*Drag your Gazebo screenshot/video here → auto-generates embed link*

---
```markdown
## 🎯 What I Built

Full-stack robotics project implementing **color-based object sorting**:

- **Gazebo simulation**: Panda arm + table + 3 bins + colored cubes.
- **Perception**: OpenCV HSV thresholding → 3D pose estimation via TF2.
- **Planning**: MoveIt 2 Python API for grasp/place trajectories.
- **Control**: ROS 2 controllers + Docker for reproducibility.
- **Integration**: Single launch file spawns everything.

**Live demo commands**:
```bash
ros2 run pymoveit2 pick_and_place.py target_color:=R  # Red cubes
ros2 run pymoveit2 pick_and_place.py target_color:=G  # Green  
ros2 run pymoveit2 pick_and_place.py target_color:=B  # Blue
```

---

## 🛠 Challenges I Solved

| Problem | Solution |
|---------|----------|
| **HSV color detection inconsistent** | Dual red range (0-10 + 170-180) + morphological ops + contour filtering |
| **Camera → robot TF unstable** | Static transforms + `camera_link` → `panda_link0` chain validation |
| **MoveIt 2 grasp planning fails** | Pre-grasp retreat pose + collision objects + allow_collisions=True |
| **Docker GUI black screen** | `xhost +local:docker` + `--network host` + `QT_X11_NO_MITSHM=1` |
| **Jazzy controllers crash** | Fallback to stable Humble Docker + native Jazzy dev workspace |

**Toughest**: Coordinate transforms. Camera sees cubes in `camera_link` but MoveIt needs `panda_link0`. Solution: `tf2_ros` static publisher + runtime validation.

---

## 🚀 Quick Start (Docker - Recommended)

**Full demo in 2 commands**:

```bash
# 1. GUI access
xhost +local:docker

# 2. Launch everything
docker run -it --rm --name panda_demo \
  --network host \
  -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  curiousutkarsh/franka_panda_color_sorter:humble bash -c "
  cd /root/panda_ws && source install/setup.bash &&
  ros2 launch panda_bringup pick_and_place.launch.py
"
```

**Sort cubes** (new HOST terminal):
```bash
docker exec -it panda_demo bash -c "
source /root/panda_ws/install/setup.bash &&
ros2 run pymoveit2 pick_and_place.py target_color:=R
"
```

---

## 🏗 Full Structure

```
panda_color_sorter/
├── README.md                 # You're reading it
├── docker/run_humble.sh      # Docker launcher
├── media/                    # demo.png, demo.mp4
└── src/Franka_Panda_Color_Sorting_Robot/
    ├── panda_bringup/        # ros2 launch pick_and_place.launch.py
    ├── panda_description/    # URDF + Gazebo world + meshes
    ├── panda_vision/         # color_detector.py (OpenCV node)
    ├── panda_moveit/         # MoveIt 2 config (SRDF, kinematics.yaml)
    └── pymoveit2/            # pick_and_place.py + robot configs
```

---

## 🔍 Core Code Samples

### 1. Color Detection (`panda_vision/color_detector.py`)

```python
color_ranges = {
    "R1": [(0, 120, 70), (10, 255, 255)],      # Red range 1
    "R2": [(170,120, 70), (180,255,255)],      # Red range 2 (wraparound)
    "G":  [(35, 60, 60), (85, 255, 255)],      # Green
    "B":  [(90, 60, 60), (140,255,255)]        # Blue
}

# HSV → pixel → 3D point → /color_coordinates
pub.publish(f"{color_id},{x},{y},{z}")
```

### 2. Pick & Place (`pymoveit2/pick_and_place.py`)

```python
if msg.data.startswith(target_color):
    x, y, z = map(float, msg.data.split(',')[1:])
    
    # MoveIt 2 grasp pose
    grasp_pose = Panda().get_grasp_pose(x, y, z)
    Panda().move_to_pose(grasp_pose)  # Plan + execute
    
    # Place in bin
    place_pose = Panda().get_place_pose(target_color)
    Panda().move_to_pose(place_pose)
```

---

## 📊 Native ROS 2 Jazzy Setup (Development)

```bash
# Clone + build
mkdir ws_panda && cd ws_panda/src
git clone https://github.com/rayeneayarii/panda_color_sorter.git .
cd .. && colcon build --symlink-install
source install/setup.bash

# Run
ros2 launch panda_bringup pick_and_place.launch.py
```

*Note: Jazzy controllers WIP → Docker Humble is production-ready demo.*

---

## 🎮 Controls & Parameters

| Topic/Service | Description | Example |
|---------------|-------------|---------|
| `/color_coordinates` | Detected cube poses | `R,0.600,0.002,1.100` |
| `target_color` | R/G/B filter | `ros2 run ... target_color:=G` |
| `/panda/gripper_cmd` | Open/close gripper | `ros2 service call ... Float64MultiArray` |

---

## 🔧 Extension Ideas

- **Multi-object stacking** in bins
- **Real Panda arm** (franka_ros2 integration)
- **RL training** environment
- **Web UI** for color selection
- **STM32 vision coprocessor**

---

## 👨‍💻 About Me

**Rayene Jazzy**  
*MSc Electronics & Embedded Systems – ISSAT Sousse*  

**Skills**: ROS 2 • MoveIt 2 • STM32 • OpenCV • Gazebo • Docker • Real-time systems  
**Experience**: RALL-P V2 autonomous robot (PFE @ OORB startup)  

[![LinkedIn](https://img.shields.io/badge/LinkedIn-Profile-blue)](https://linkedin.com/in/rayenejazzy)
📧 rayene@example.com

---

## 📄 License
MIT © 2026 Rayene Jazzy. See [LICENSE](LICENSE).
```

**Copy ALL above** → GitHub → Edit README.md → **Ctrl+A, Delete, Paste** → Commit.

**Add your screenshot**: Repo → `Create new file` → `media/demo.png` → drag image.

**Your portfolio project is now 🔥.** Share the link!
