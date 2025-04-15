☕ ROS 2 Cafe Butler Bot (Humble)
An autonomous café delivery robot simulation built with ROS 2 Humble, TurtleBot3, Gazebo, and Nav2. This robot navigates a simulated café environment, picks up orders from the kitchen, and delivers them to tables — intelligently handling confirmations, cancellations, and timeouts through a simple command-line interface.

📖 What’s This Project About?
This project simulates a smart service robot designed for café environments. The robot moves between the kitchen and tables, taking food orders, waiting for confirmations, responding to cancellations, and efficiently managing delivery operations.

The system integrates:
~Nav2 for autonomous navigation
~Gazebo for realistic simulation
~ROS 2 nodes for robot control and user commands
~A clean, terminal-based client interface to manage deliveries

🚀 Key Highlights
✨ Autonomous navigation with obstacle-aware path planning
✨ Real-time delivery control via terminal commands
✨ Timeout handling for delayed responses
✨ Cancel orders on the fly
✨ Seven interactive test scenarios
✨ Modular, clean ROS 2 Python package structure
✨ Custom-built Gazebo café world

Project Layout:

anu_ws/
├── src/
│   ├── cafe_butler/          # Custom robot control package
│   │   ├── butler_server.py  # State machine controller
│   │   ├── butler_client.py  # Command-line client
│   │   └── worlds/
│   │       └── cafe_world.world  # Custom café simulation
│   ├── turtlebot3_simulations/
│   ├── turtlebot3_navigation2/


 Setup & Installation:
Build the Workspace
cd ~/anu_ws
colcon build --packages-select cafe_butler
source install/setup.bash


 
