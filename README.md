# 🦾 Franka Panda Color Sorter

**ROS 2 Humble • MoveIt 2 • Gazebo • OpenCV**  
Franka Emika Panda detects RGB cubes via camera → picks → places into color-coded bins.

[![Demo Video](media/demo.gif)](https://github.com/rayeneayarii/panda_color_sorter/blob/main/media/demo.gif)  
*(Add your video/screenshot to `media/` folder)*

---

## 🚀 Quick Start (Docker)

```bash
# Allow GUI
xhost +local:docker

# Run Humble container
docker run -it --rm --name panda_demo \
  --network host \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  curiousutkarsh/franka_panda_color_sorter:humble bash

# Inside → launch
cd /root/panda_ws && source install/setup.bash
ros2 launch panda_bringup pick_and_place.launch.py

New terminal (host):

bash
docker exec -it panda_demo bash
source /root/panda_ws/install/setup.bash

# Sort RED cubes
ros2 run pymoveit2 pick_and_place.py target_color:=R

# GREEN
ros2 run pymoveit2 pick_and_place.py target_color:=G

# BLUE  
ros2 run pymoveit2 pick_and_place.py target_color:=B

🏗️ Project Structure

text
src/
├── panda_bringup/          # Launches (Gazebo + MoveIt + controllers)
├── panda_description/      # URDF, meshes, Gazebo models (table, bins)
├── panda_vision/           # color_detector.py (OpenCV HSV → 3D points)
├── panda_moveit/           # MoveIt 2 config (SRDF, kinematics)
└── pymoveit2/              # pick_and_place.py motion pipeline
docker/
└── run_humble.sh           # Docker helper script

🎯 How it works

    Perception (panda_vision/color_detector.py):

    text
    RGB → HSV → Threshold → Contours → Camera TF → panda_link0 coordinates
    /color_coordinates: "R,0.600,0.002,1.100"

    Motion (pymoveit2/pick_and_place.py):

        Filter by target_color param (R/G/B).

        MoveIt 2: plan grasp → lift → place → retreat.

        Execute via ROS 2 controllers.

🛠 Native Build (ROS 2 Jazzy)

bash
mkdir -p ws_panda/src
cd ws_panda/src
git clone https://github.com/rayeneayarii/panda_color_sorter.git .
cd ..
colcon build --symlink-install
source install/setup.bash
ros2 launch panda_bringup pick_and_place.launch.py

🔧 Customize

    Colors: Edit HSV ranges in panda_vision/color_detector.py.

    Environment: Replace Gazebo world/models in panda_description/worlds/.

    Logic: Extend pick_and_place.py (stacking, multiple bins, etc.).

👨‍💻 Author

Rayene Jazzy
MSc Embedded Systems – ISSAT Sousse
ROS 2 • STM32 • Computer Vision • RALL-P V2 Autonomous Robot (PFE @ OORB)

LinkedIn • [rayenejazzy@example.com]
📄 License

MIT © 2026 Rayene Jazzy

text

**Copy →** GitHub repo → Edit `README.md` → **Replace all** → Commit.

**Drag your screenshot** to `media/` → update the GIF link to `.png`.

**Done!** Repo now looks professional. Link?
