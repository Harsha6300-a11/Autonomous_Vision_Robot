

# Vision-Based Person Follower Robot (ROS 2)

This repository presents an **autonomous vision-based person-following robot** implemented using **ROS 2**, **YOLOv8**, **DeepSORT**, and **LiDAR-based obstacle avoidance**, and controlled using a **Behavior Tree–based decision framework**.
The system is designed for **TurtleBot3 Waffle Pi** operating in a **Gazebo empty world**.

---

## Features

* Vision-based **human detection** using YOLOv8
* **Persistent person tracking** using DeepSORT
* **Behavior Tree–based control architecture**
* Social-distance-aware following behavior
* LiDAR-based obstacle avoidance
* Autonomous search and recovery behavior
* ROS 2–compliant Python implementation

---

## System Architecture

The system follows a **modular decision-making architecture inspired by Behavior Trees (BTs)**.
Perception, tracking, and motion control are integrated within a single ROS 2 node for efficiency.

### Inputs

* RGB camera
* LiDAR sensor

### Outputs

* Velocity commands (`/cmd_vel`)
* Visual feedback with tracked person

---

## Behavior Tree–Based Control Logic

The robot’s behavior is organized using a **Behavior Tree (BT)**, which enables clear prioritization, modularity, and fault recovery.

### High-Level Behavior Tree

```text
Root
 ├── Sequence
 │   ├── Detect Person
 │   ├── Track Person
 │   ├── Follow Person
 │   └── Maintain Social Distance
 └── Fallback
     ├── Avoid Obstacle
     └── Search for Person
```

### Behavior Description

* **Detect Person**
  Uses YOLOv8 to identify humans in the camera stream.

* **Track Person**
  DeepSORT assigns and maintains a stable ID for the detected person.

* **Follow Person**
  Computes linear and angular velocities to keep the person centered.

* **Maintain Social Distance**
  Regulates robot motion to keep a safe distance from the person.

* **Avoid Obstacle**
  Uses LiDAR data to steer away from nearby obstacles.

* **Search for Person**
  Executes a 360° rotation and forward motion when the person is lost.

This Behavior Tree structure ensures **robust, reactive, and interpretable robot behavior**.

---

## Package Structure

```text
vision_person_follower/
├── launch/
│   └── vision_person_follower.launch.py
├── resource/
│   └── vision_person_follower
├── vision_person_follower/
│   ├── __init__.py
│   └── controller_node.py
├── CMakeLists.txt
├── package.xml
├── setup.py
├── setup.cfg
└── README.md
```

---

## Node Description

### `controller_node.py`

This node integrates:

* YOLOv8-based person detection
* DeepSORT-based multi-object tracking
* Distance estimation using monocular vision
* Behavior Tree–driven motion control
* LiDAR-based obstacle avoidance
* Autonomous search behavior

---

## Topics Used

| Topic                          | Message Type                  | Direction |
| ------------------------------ | ----------------------------- | --------- |
| `/camera/image_raw/compressed` | `sensor_msgs/CompressedImage` | Subscribe |
| `/scan`                        | `sensor_msgs/LaserScan`       | Subscribe |
| `/cmd_vel`                     | `geometry_msgs/Twist`         | Publish   |
| `/yolo/tracked_image`          | `sensor_msgs/Image`           | Publish   |

---

## Dependencies

### ROS 2

* ROS 2 Humble

### Python Libraries

```bash
pip install ultralytics
pip install deep-sort-realtime
pip install opencv-python numpy
```

---

## Running the System

```bash
export TURTLEBOT3_MODEL=waffle_pi
colcon build --packages-select vision_person_follower
source install/setup.bash
ros2 launch vision_person_follower vision_person_follower.launch.py
```

---

## Applications

* Social service robots
* Human–robot interaction research
* Indoor autonomous navigation
* Assistive robotics

---

## Author
* Harsha Chintgaunta
* Ashvin Dandothkar
* Harish Enugu Reddy
* Pranav

ROS 2 | Computer Vision | Autonomous Robotics

---

## License

Apache License 2.0

