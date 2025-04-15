#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Bool
import threading
import time
from enum import Enum
import json
import math
from tf_transformations import quaternion_from_euler

class RobotState(Enum):
    IDLE = 0
    GOING_TO_KITCHEN = 1
    AT_KITCHEN = 2
    GOING_TO_TABLE = 3
    AT_TABLE = 4
    RETURNING_TO_HOME = 5
    RETURNING_TO_KITCHEN = 6

class ButlerServer(Node):
    def __init__(self):
        super().__init__('butler_server')
        
        # Initialize navigation client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        while not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Waiting for navigation action server...')
        
        # Parameters (these can be loaded from a config file)
        self.timeout = 30.0  # Timeout in seconds for confirmations
        
        # Positions in the café world based on the SDF file
        self.positions = {
            'home': {'x': 0.0, 'y': -2.0, 'z': 0.0, 'yaw': 0.0},
            'kitchen': {'x': -1.0, 'y': 2.3, 'z': 0.0, 'yaw': 0.0},
            'table1': {'x': 1.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0},
            'table2': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0},
            'table3': {'x': -1.0, 'y': -1.0, 'z': 0.0, 'yaw': 0.0}
        }
        # Publishers
        self.status_pub = self.create_publisher(String, '/butler/status', 10)
        
        # Subscribers
        self.command_sub = self.create_subscription(
            String, '/butler/command', self.command_callback, 10)
        self.confirm_sub = self.create_subscription(
            Bool, '/butler/confirm', self.confirm_callback, 10)
        self.cancel_sub = self.create_subscription(
            String, '/butler/cancel', self.cancel_callback, 10)
        
        # State variables
        self.state = RobotState.IDLE
        self.current_orders = []
        self.current_table = None
        self.active_task = False
        self.waiting_for_confirmation = False
        self.task_canceled = False
        self.confirmation_timer = None
        self.current_position = 'home'
        self.navigation_in_progress = False
        
        # For tracking navigation goals
        self.current_goal_handle = None
        
        self.get_logger().info('Butler server initialized and ready')
        self.publish_status("Butler ready at home position")
    
    def publish_status(self, message):
        """Publish status message"""
        msg = String()
        msg.data = message
        self.status_pub.publish(msg)
        self.get_logger().info(message)
    
    def command_callback(self, msg):
        """Process commands from client"""
        try:
            cmd_data = json.loads(msg.data)
            cmd_type = cmd_data.get('type', '')
            
            if cmd_type == 'order':
                test_case = cmd_data.get('test_case', 1)
                tables = cmd_data.get('tables', [])
                
                if not tables:
                    self.get_logger().error("No tables specified in order")
                    return
                
                self.get_logger().info(f"Received order for tables: {tables}, Test case: {test_case}")
                self.handle_order(tables, test_case)
                
        except json.JSONDecodeError:
            self.get_logger().error("Invalid command format")
    
    def confirm_callback(self, msg):
        """Handle confirmation from kitchen or table"""
        if self.waiting_for_confirmation:
            self.waiting_for_confirmation = False
            
            if self.confirmation_timer:
                self.confirmation_timer.cancel()
                self.confirmation_timer = None
            
            if self.state == RobotState.AT_KITCHEN:
                self.publish_status("Food received from kitchen")
                # Move to next table after confirmation
                self.go_to_next_table()
            
            elif self.state == RobotState.AT_TABLE:
                self.publish_status(f"Food delivered to table {self.current_table}")
                # Process next table or return to kitchen/home
                self.complete_current_delivery()
            
    def cancel_callback(self, msg):
        """Handle order cancellation"""
        try:
            cancel_data = json.loads(msg.data)
            table = cancel_data.get('table', '')
            
            if not table:
                self.get_logger().error("No table specified for cancellation")
                return
            
            # If it's the current table being processed
            if self.current_table == table:
                self.task_canceled = True
                self.publish_status(f"Order for table {table} canceled")
                
                # Cancel any ongoing confirmation timer
                if self.confirmation_timer:
                    self.confirmation_timer.cancel()
                    self.confirmation_timer = None
                
                # Cancel any ongoing navigation
                if self.current_goal_handle and self.navigation_in_progress:
                    self.current_goal_handle.cancel_goal_async()
                    self.publish_status("Navigation canceled")
                
                # Handle different cancellation states
                if self.state == RobotState.GOING_TO_TABLE:
                    self.publish_status("Cancellation while going to table, returning to kitchen")
                    self.state = RobotState.RETURNING_TO_KITCHEN
                    self.navigate_to('kitchen')
                elif self.state == RobotState.GOING_TO_KITCHEN:
                    self.publish_status("Cancellation while going to kitchen, returning home")
                    self.state = RobotState.RETURNING_TO_HOME
                    self.navigate_to('home')
                
            # If it's in the queue, remove it
            elif table in self.current_orders:
                self.current_orders.remove(table)
                self.publish_status(f"Order for table {table} removed from queue")
                
        except json.JSONDecodeError:
            self.get_logger().error("Invalid cancel command format")
    
    def handle_order(self, tables, test_case):
        """Process orders based on test case"""
        self.current_orders = tables.copy()
        self.test_case = test_case
        
        # Start order processing if robot is idle
        if self.state == RobotState.IDLE:
            self.active_task = True
            self.start_order_process()
    
    def start_order_process(self):
        """Start order processing"""
        self.publish_status("Going to kitchen to pickup food")
        self.state = RobotState.GOING_TO_KITCHEN
        self.navigate_to('kitchen')
    
    def navigate_to(self, location):
        """Navigate to a given location using Nav2 action server"""
        self.get_logger().info(f"Navigating to {location}")
        
        # Create goal pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        
        pos = self.positions[location]
        goal_pose.pose.position.x = pos['x']
        goal_pose.pose.position.y = pos['y']
        goal_pose.pose.position.z = pos['z']
        
        # Convert yaw to quaternion
        q = quaternion_from_euler(0, 0, pos['yaw'])
        goal_pose.pose.orientation.x = q[0]
        goal_pose.pose.orientation.y = q[1]
        goal_pose.pose.orientation.z = q[2]
        goal_pose.pose.orientation.w = q[3]
        
        # Send goal to navigation
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        
        self.get_logger().info(f"Starting navigation to {location}")
        
        # Set flag that navigation is in progress
        self.navigation_in_progress = True
        
        # Send goal and register callbacks
        send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(
            lambda future: self.goal_response_callback(future, location)
        )
    
    def goal_response_callback(self, future, location):
        """Handle the goal response from the action server"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f'Navigation to {location} was rejected')
            self.navigation_in_progress = False
            return
        
        self.current_goal_handle = goal_handle
        self.get_logger().info(f'Navigation to {location} was accepted')
        
        # Get the result of navigation
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(
            lambda future: self.get_result_callback(future, location)
        )
    
    def get_result_callback(self, future, location):
        """Process the result of navigation"""
        self.navigation_in_progress = False
        result = future.result().result
        
        # Check if canceled
        if self.task_canceled and (self.state == RobotState.RETURNING_TO_HOME or 
                                   self.state == RobotState.RETURNING_TO_KITCHEN):
            # Handle based on current return state
            if location == 'home':
                self.handle_arrived_home()
            elif location == 'kitchen':
                self.handle_returned_kitchen()
            return
        
        # Update position and state based on where we navigated to
        self.current_position = location
        
        if location == 'kitchen':
            if self.state == RobotState.GOING_TO_KITCHEN:
                self.handle_arrived_kitchen()
            elif self.state == RobotState.RETURNING_TO_KITCHEN:
                self.handle_returned_kitchen()
                
        elif location.startswith('table'):
            self.handle_arrived_table()
            
        elif location == 'home':
            self.handle_arrived_home()
    
    def feedback_callback(self, feedback_msg):
        """Process navigation feedback (optional)"""
        # We could use this to track progress, but we'll keep it simple
        pass
    
    def handle_arrived_kitchen(self):
        """Handle arrival at kitchen"""
        self.state = RobotState.AT_KITCHEN
        self.publish_status("Arrived at kitchen")
        
        # Test cases 2 and 3a require confirmation or timeout at kitchen
        if self.test_case in [2, 3]:
            self.waiting_for_confirmation = True
            self.publish_status("Waiting for kitchen confirmation")
            self.confirmation_timer = threading.Timer(
                self.timeout, self.handle_confirmation_timeout)
            self.confirmation_timer.start()
        else:
            # No confirmation needed, go to table directly
            self.go_to_next_table()
    
    def handle_arrived_table(self):
        """Handle arrival at table"""
        self.state = RobotState.AT_TABLE
        self.publish_status(f"Arrived at table {self.current_table}")
        
        # Test cases 2, 3b, and 6 require confirmation or timeout at table
        if self.test_case in [2, 3, 6]:
            self.waiting_for_confirmation = True
            self.publish_status(f"Waiting for confirmation from table {self.current_table}")
            self.confirmation_timer = threading.Timer(
                self.timeout, self.handle_confirmation_timeout)
            self.confirmation_timer.start()
        else:
            # No confirmation needed, mark as delivered and proceed
            self.complete_current_delivery()
    
    def handle_returned_kitchen(self):
        """Handle return to kitchen (after deliveries or cancellations)"""
        self.publish_status("Returned to kitchen after deliveries")
        self.navigate_to('home')
        self.state = RobotState.RETURNING_TO_HOME
    
    def handle_arrived_home(self):
        """Handle arrival back at home"""
        self.state = RobotState.IDLE
        self.active_task = False
        self.task_canceled = False
        self.current_orders = []
        self.current_table = None
        self.publish_status("Returned to home position, ready for new orders")
    
    def handle_confirmation_timeout(self):
        """Handle timeout waiting for confirmation"""
        self.waiting_for_confirmation = False
        
        if self.state == RobotState.AT_KITCHEN:
            self.publish_status("Kitchen confirmation timeout, returning to home")
            self.navigate_to('home')
            self.state = RobotState.RETURNING_TO_HOME
            
        elif self.state == RobotState.AT_TABLE:
            self.publish_status(f"Table {self.current_table} confirmation timeout")
            
            if self.test_case == 3:
                # For test case 3b, return to kitchen first
                self.publish_status("Returning to kitchen before home")
                self.navigate_to('kitchen')
                self.state = RobotState.RETURNING_TO_KITCHEN
            elif self.test_case == 6:
                # For test case 6, skip to next table
                if self.current_orders:
                    self.publish_status(f"Skipping to next table due to timeout")
                    self.go_to_next_table()
                else:
                    # All tables processed, return to kitchen
                    self.publish_status("All tables processed, returning to kitchen")
                    self.navigate_to('kitchen')
                    self.state = RobotState.RETURNING_TO_KITCHEN
            else:
                # Default behavior, return home
                self.navigate_to('home')
                self.state = RobotState.RETURNING_TO_HOME
    
    def go_to_next_table(self):
        """Go to the next table in the order queue"""
        if not self.current_orders:
            # No more tables to serve, return to kitchen
            self.publish_status("No more tables to serve, returning to kitchen")
            self.navigate_to('kitchen')
            self.state = RobotState.RETURNING_TO_KITCHEN
            return
        
        # Get next table
        self.current_table = self.current_orders.pop(0)
        self.state = RobotState.GOING_TO_TABLE
        self.publish_status(f"Going to table {self.current_table}")
        self.navigate_to(f"table{self.current_table}")
    
    def complete_current_delivery(self):
        """Complete current delivery and proceed to next table or return"""
        self.publish_status(f"Completed delivery to table {self.current_table}")
        
        # Check if there are more tables to serve
        if self.current_orders:
            self.go_to_next_table()
        else:
            # All tables served, return to kitchen first for test cases 5-7
            if self.test_case >= 5:
                self.publish_status("All tables served, returning to kitchen")
                self.navigate_to('kitchen')
                self.state = RobotState.RETURNING_TO_KITCHEN
            else:
                # For test cases 1-4, return directly home
                self.publish_status("Order completed, returning home")
                self.navigate_to('home')
                self.state = RobotState.RETURNING_TO_HOME

def main(args=None):
    rclpy.init(args=args)
    node = ButlerServer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Butler server shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
