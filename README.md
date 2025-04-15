Perfect — thank you for clarifying. I’ll now distill your project documentation into a **concise, professional, no-fluff README.md** for your GitHub repo. I’ll reference your original doc content to ensure nothing important is missed, while keeping it clean, relevant, and properly structured for a typical ROS 2 project README.

---

## 📑 Final Clean README.md  

```markdown
# ☕ ROS 2 Café Butler Robot (Humble)

A ROS 2 Humble-based simulated service robot that autonomously delivers orders inside a virtual café environment using TurtleBot3, Nav2, and Gazebo.

---

## 📌 Project Overview

The Café Butler Robot simulates a robot waiter tasked with navigating a café, picking up food from a kitchen, and delivering it to tables. It uses a **state machine** for task management and allows user interaction through a **command-line client**. The system handles delivery confirmations, cancellations, and timeout situations.

---

## 📦 Project Structure

```
anu_ws/
├── src/
│   ├── cafe_butler/
│   │   ├── butler_server.py       # Robot state machine controller
│   │   ├── butler_client.py       # Terminal-based client interface
│   │   ├── worlds/
│   │   │   └── cafe_world.world   # Custom Gazebo world
│   ├── turtlebot3_simulations/
│   ├── turtlebot3_navigation2/
```

---

## ⚙️ System Components

- **Nav2 Action Server (`/navigate_to_pose`)**: Handles robot path planning and navigation.
- **ROS 2 Topics**
  - `/butler/command` — orders from client  
  - `/butler/status` — status updates from server  
  - `/butler/confirm` — delivery confirmations  
  - `/butler/cancel` — cancel delivery orders  

---

## 🛠️ Installation & Build

```bash
sudo apt update
sudo apt install ros-humble-navigation2 ros-humble-turtlebot3* ros-humble-gazebo-ros-pkgs
cd ~/anu_ws
colcon build --packages-select cafe_butler
source install/setup.bash
```

---

## 🚀 Running the Simulation

**Open 4 terminals**

**Terminal 1**  
```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

**Terminal 2**  
```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_navigation2 navigation2.launch.py
```

**Terminal 3**  
```bash
ros2 run cafe_butler butler_server
```

**Terminal 4**  
```bash
ros2 run cafe_butler butler_client
```

**In RViz:** Use **2D Pose Estimate** to set the robot's initial pose to `(0.0, -2.0)`

---

## 🎮 Client Commands

| Command | Description |
|:-----------|:---------------------------|
| `1-7`      | Run predefined test scenarios |
| `c`        | Send confirmation |
| `x [table]`| Cancel order for a specific table (e.g. `x 2`) |
| `q`        | Quit the client |

---

## 📊 Test Scenarios

| Test Case | Description |
|:-----------|:--------------------------------|
| **1** | Single table delivery |
| **2** | Delivery with confirmation |
| **3** | Timeout handling for confirmations |
| **4** | Cancel during delivery |
| **5** | Multiple table deliveries |
| **6** | Multiple deliveries with timeouts |
| **7** | Multiple deliveries with mid-run cancellations |

---

## 📌 Conclusion

The Café Butler Robot effectively demonstrates autonomous service robot behavior in a simulated café setting using ROS 2, TurtleBot3, and Nav2. It efficiently handles real-time task management, confirmations, cancellations, and timeouts within a dynamic ROS 2 multi-node framework.

---
