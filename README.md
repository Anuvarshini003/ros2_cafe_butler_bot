Project Overview: 
The Café Butler Robot is a simulated service robot built to autonomously deliver food from a kitchen to multiple tables inside a café environment. It’s powered by TurtleBot3 Burger, simulated in Gazebo, and navigated via Nav2 (Navigation2) in ROS 2 Humble.
The robot follows user commands through a client interface, making it ideal for demonstrating state machine control, navigation, confirmations, cancellations, and timeouts within a dynamic simulation world.


Project Structure 
![image](https://github.com/user-attachments/assets/d045cf9d-37aa-4884-a27a-008110c367bf)

Installation and Setup
1.	Create Workspace & Clone Packages
mkdir -p ~/anu_ws/src
cd ~/anu_ws/src
Place your cafe_butler, turtlebot3, turtlebot3_msgs, and turtlebot3_simulations folders here.
2.	Build the Workspace
cd ~/anu_ws
colcon build --packages-select cafe_butler
source install/setup.bash
3.	 Running the System
•	Terminal 1: Launch Café Gazebo Simulation
source ~/anu_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch cafe_butler empty_world.launch.py
In RViz, use 2D Pose Estimate to set robot’s initial pose at:
x: 0.0
y: -2.0
•	Terminal 2: Launch Navigation
source ~/anu_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_navigation2 navigation2.launch.py
•	Terminal 3: Start Butler Server
source ~/anu_ws/install/setup.bash
ros2 run cafe_butler butler_server
•	Terminal 4: Start Butler Client
source ~/anu_ws/install/setup.bash
ros2 run cafe_butler butler_client

Client Commands
•	1-7 → Run test scenarios
•	c → Confirm action (at kitchen/table)
•	x [table] → Cancel table order (e.g. x 2)
•	q → Quit client
Test Case Descriptions
Test Case	Description
1	Basic single table delivery
2	Delivery with confirmation at kitchen & table
3	Test timeouts for kitchen/table confirmations
4	Cancel an order while traveling
5	Deliver to multiple tables in sequence
6	Multiple deliveries with timeouts at tables
7	Multiple deliveries with mid-run cancellations

