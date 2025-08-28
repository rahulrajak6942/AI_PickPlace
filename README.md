Perfect 👌 since you already pushed the videos, let’s prepare a **well-structured `README.md`** that explains your project, shows setup instructions, usage, and embeds your demo videos so your professor (or anyone) can follow and reproduce your assignment.

Here’s a draft for your repo:

---

# 🤖 AI Pick & Place with YOLO + OpenManipulator-X

This project integrates **YOLO-based color detection** with the **OpenManipulator-X robotic arm** in Gazebo and MoveIt2 to perform an autonomous **pick and place** task.

The robot detects colored cylinders (red & blue) using a camera feed, computes their position, and performs pick-and-place operations with the manipulator arm.

---

## 📂 Repository Structure

```
AI_PickPlace/
│── open_manipulator_ws/      # OpenManipulator bringup + MoveIt configs
│── yolo_detection_ws/        # YOLO detection + action client nodes
│   ├── src/detection/src/
│   │   ├── detection.py      # Color detection & pose publishing
│   │   ├── action_call.py    # Pick & place action client
│   │   ├── yolov8n.pt        # YOLO pretrained weights
│   │   └── yolo11n.pt
│── moveit_demo.webm          # Demo video of MoveIt planning
│── pick_place.webm           # Demo video of autonomous pick & place
│── README.md
```

---

## ⚙️ Requirements

* **Ubuntu 22.04 / ROS 2 Humble**
* `colcon`, `rviz2`, `gazebo`
* Packages:

  ```bash
  sudo apt install ros-humble-moveit ros-humble-control-msgs ros-humble-cv-bridge ros-humble-vision-opencv
  ```
* Python deps:

  ```bash
  pip install ultralytics opencv-python
  ```

---

## 🚀 Setup

1. Clone this repo:

   ```bash
   git clone https://github.com/rahulrajak6942/AI_PickPlace.git
   cd AI_PickPlace
   ```

2. Build both workspaces:

   ```bash
   cd open_manipulator_ws
   colcon build
   source install/setup.bash

   cd ../yolo_detection_ws
   colcon build
   source install/setup.bash
   ```

---

## 🕹️ Running the System

### 1. Launch Gazebo simulation

```bash
ros2 launch open_manipulator_x_bringup gazebo.launch.py
```

### 2. Launch MoveIt for planning

```bash
ros2 launch open_manipulator_x_moveit_config moveit_core.launch.py
```

### 3. Run teleop (optional, for manual testing)

```bash
ros2 run open_manipulator_x_teleop simple_teleop_node
```

### 4. Run YOLO detection node

(activate venv if needed)

```bash
cd yolo_detection_ws
source ../venv/bin/activate
python3 src/detection/src/detection.py
```

This will detect **red and blue cylinders** and publish their poses to `/detected_target_pose`.

### 5. Run pick & place action node

```bash
python3 src/detection/src/action_call.py
```

This node executes:

1. Open gripper
2. Move to detected pose
3. Lower the arm by 7.5 cm
4. Close gripper (pick)

---

## 🎥 Demo Videos

### MoveIt Planning Demo

![MoveIt Demo](moveit_demo.webm)

### Autonomous Pick & Place

![Pick Place Demo](pick_place.webm)

---

## 📌 Notes

* Adjust `gripper position` values in `action_call.py` if the object slips.
* Detection is currently based on **color masks (red & blue)**, but YOLO weights are also included for object detection.
* Cooldown of **10s** is applied to avoid continuous triggering.

---

## ✨ Credits

* **Rahul Rajak** (IIITDM Kanchipuram)
* Uses **OpenManipulator-X**, **MoveIt2**, **ROS 2 Humble**, and **YOLOv8**

---

Rahul, this README is **assignment-ready** ✅.
Would you like me to also add **screenshots (RViz + Gazebo + detection output)** in the README, or keep only videos?
