## Project Overview 

The Café Butler Robot is a simulated service robot built to autonomously deliver food from a kitchen to multiple tables inside a café environment. It’s powered by TurtleBot3 Burger, simulated in Gazebo, and navigated via Nav2 (Navigation2) in ROS 2 Humble.
The robot follows user commands through a client interface, making it ideal for demonstrating state machine control, navigation, confirmations, cancellations, and timeouts within a dynamic simulation world.

## Project Structure 

![image](https://github.com/user-attachments/assets/e9a9be3b-7ef9-4fd2-aebb-512f490fa7b5)


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

## Workflow
![image](https://github.com/user-attachments/assets/a702b345-d164-4556-9644-d6a562f85aa5)

## Output 
![image](https://github.com/user-attachments/assets/f25e2fcd-8f12-4af5-b6e5-a1b880a0179c)
![image](https://github.com/user-attachments/assets/1b66a7d8-28e8-4d5e-b68f-2ec8d3b6b8bf)
![image](https://github.com/user-attachments/assets/e0ad8eaa-936e-4c08-b154-76e99b660955)
![image](https://github.com/user-attachments/assets/4914b578-0c68-448d-8c16-a30d1d20e964)


## 📌 Conclusion

The Café Butler Robot effectively demonstrates autonomous service robot behavior in a simulated café setting using ROS 2, TurtleBot3, and Nav2. It efficiently handles real-time task management, confirmations, cancellations, and timeouts within a dynamic ROS 2 multi-node framework.

---
