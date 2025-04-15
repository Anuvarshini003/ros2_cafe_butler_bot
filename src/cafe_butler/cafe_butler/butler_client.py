#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import json
import sys
import threading
import time

class ButlerClient(Node):
    def __init__(self):
        super().__init__('butler_client')
        
        # Publishers
        self.command_pub = self.create_publisher(String, '/butler/command', 10)
        self.confirm_pub = self.create_publisher(Bool, '/butler/confirm', 10)
        self.cancel_pub = self.create_publisher(String, '/butler/cancel', 10)
        
        # Subscriber for status updates
        self.status_sub = self.create_subscription(
            String, '/butler/status', self.status_callback, 10)
        
        self.get_logger().info("Butler client initialized. Ready to send commands.")
        self.print_usage()
    
    def status_callback(self, msg):
        """Process status updates from the butler server"""
        status = msg.data
        self.get_logger().info(f"Butler status: {status}")
    
    def print_usage(self):
        """Print usage instructions"""
        print("\n===== FRENCH DOOR CAFÉ BUTLER CONTROL =====")
        print("Available commands:")
        print("1. Single table delivery (basic)")
        print("2. Single table delivery with confirmation")
        print("3. Kitchen/table timeout handling")
        print("4. Cancellation handling")
        print("5. Multiple table deliveries")
        print("6. Multiple table deliveries with table timeout")
        print("7. Multiple table deliveries with cancellation")
        print("c - Send confirmation")
        print("x <table> - Cancel order for table")
        print("q - Quit")
        print("=========================================")
    
    def send_order(self, test_case, tables):
        """Send order command to butler server"""
        cmd = {
            'type': 'order',
            'test_case': test_case,
            'tables': tables
        }
        
        msg = String()
        msg.data = json.dumps(cmd)
        self.command_pub.publish(msg)
        self.get_logger().info(f"Order sent for tables {tables}, test case {test_case}")
    
    def send_confirmation(self):
        """Send confirmation to butler server"""
        msg = Bool()
        msg.data = True
        self.confirm_pub.publish(msg)
        self.get_logger().info("Confirmation sent")
    
    def cancel_order(self, table):
        """Cancel order for specific table"""
        cmd = {
            'table': table
        }
        
        msg = String()
        msg.data = json.dumps(cmd)
        self.cancel_pub.publish(msg)
        self.get_logger().info(f"Cancel sent for table {table}")
    
    def test_case_1(self):
        """Basic single table delivery"""
        self.get_logger().info("TEST CASE 1: Basic single table delivery")
        self.send_order(1, ['1'])
    
    def test_case_2(self):
        """Single table delivery with confirmation"""
        self.get_logger().info("TEST CASE 2: Single table delivery with confirmation")
        self.send_order(2, ['2'])
        
        print("The robot will wait for confirmation at kitchen and table.")
        print("Press 'c' to confirm when needed, or wait for timeout.")
    
    def test_case_3(self):
        """Kitchen/table timeout handling"""
        self.get_logger().info("TEST CASE 3: Kitchen/table timeout handling")
        self.send_order(3, ['3'])
        
        print("The robot will wait for confirmation at kitchen and table.")
        print("Press 'c' to confirm when needed, or wait for timeout.")
    
    def test_case_4(self):
        """Cancellation handling"""
        self.get_logger().info("TEST CASE 4: Cancellation handling")
        self.send_order(4, ['1'])
        
        print("Press 'x 1' to cancel the order when robot is moving.")
    
    def test_case_5(self):
        """Multiple table deliveries"""
        self.get_logger().info("TEST CASE 5: Multiple table deliveries")
        self.send_order(5, ['1', '2', '3'])
    
    def test_case_6(self):
        """Multiple table deliveries with table timeout"""
        self.get_logger().info("TEST CASE 6: Multiple table deliveries with table timeout")
        self.send_order(6, ['1', '2', '3'])
        
        print("The robot will wait for confirmation at each table.")
        print("Press 'c' to confirm when needed, or wait for timeout.")
    
    def test_case_7(self):
        """Multiple table deliveries with cancellation"""
        self.get_logger().info("TEST CASE 7: Multiple table deliveries with cancellation")
        self.send_order(7, ['1', '2', '3'])
        
        print("Press 'x 2' to cancel table 2's order while robot is traveling.")

    def process_command(self, cmd):
        """Process command input from user"""
        if cmd == '1':
            self.test_case_1()
        elif cmd == '2':
            self.test_case_2()
        elif cmd == '3':
            self.test_case_3()
        elif cmd == '4':
            self.test_case_4()
        elif cmd == '5':
            self.test_case_5()
        elif cmd == '6':
            self.test_case_6()
        elif cmd == '7':
            self.test_case_7()
        elif cmd == 'c':
            self.send_confirmation()
        elif cmd.startswith('x '):
            try:
                table = cmd.split()[1]
                self.cancel_order(table)
            except IndexError:
                print("Please specify table number (e.g., 'x 2')")
        elif cmd == 'q':
            return False
        else:
            print("Unknown command")
            self.print_usage()
        
        return True

def main(args=None):
    rclpy.init(args=args)
    client = ButlerClient()
    
    # Create a separate thread for spinning the node
    spin_thread = threading.Thread(target=rclpy.spin, args=(client,))
    spin_thread.daemon = True
    spin_thread.start()
    
    # Main command loop
    try:
        running = True
        while running:
            cmd = input("\nEnter command: ").strip().lower()
            running = client.process_command(cmd)
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
