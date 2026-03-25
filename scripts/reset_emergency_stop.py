#!/usr/bin/env python3
"""
ROS 2 Emergency Stop Reset Service Client
=========================================

Simple script to call the reset emergency stop service.

Usage:
    python3 reset_emergency_stop.py
    
This script calls the /reset_emergency_stop service to reset the emergency stop
state in the autonomous navigation node.
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import sys

class EmergencyStopResetClient(Node):
    def __init__(self):
        super().__init__('emergency_stop_reset_client')
        self.client = self.create_client(Trigger, '/reset_emergency_stop')
        
        if not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Emergency stop reset service not available')
            sys.exit(1)
    
    def reset_emergency_stop(self):
        """Call the reset emergency stop service"""
        request = Trigger.Request()
        
        try:
            future = self.client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if future.result() is not None:
                result = future.result()
                if result.success:
                    self.get_logger().info(f"✅ {result.message}")
                    print(f"✅ {result.message}")
                else:
                    self.get_logger().error(f"❌ {result.message}")
                    print(f"❌ {result.message}")
                    sys.exit(1)
            else:
                self.get_logger().error('Service call failed')
                print("❌ Service call failed")
                sys.exit(1)
                
        except Exception as e:
            self.get_logger().error(f'Service call error: {e}')
            print(f"❌ Service call error: {e}")
            sys.exit(1)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        client = EmergencyStopResetClient()
        client.reset_emergency_stop()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            client.destroy_node()
        except:
            pass
        rclpy.shutdown()

if __name__ == '__main__':
    main()
