#!/usr/bin/env python3
# filepath: /home/sughetto/my_robot_ws/src/wx250s_bringup/scripts/fake_object_motion.py

import rclpy
from rclpy.node import Node
from wx250s_bringup.msg import ObjectState
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import Header
import numpy as np


class FakeObjectMotionPublisher(Node):
    def __init__(self):
        super().__init__('fake_object_publisher')
        
        # Create publisher for ObjectState messages
        self.publisher = self.create_publisher(
            ObjectState,
            '/object_state',
            10)
        
        # Set up timer for 50 Hz publishing rate (20ms period)
        self.timer_period = 0.01  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        # Motion parameters for linear motion
        self.start_x = 0.3      # starting x position
        self.start_y = -0.3     # starting y position
        self.start_z = 0.05     # z position (constant)
        
        self.vel_x = 0.0     # x velocity in m/s
        self.vel_y = 0.05     # y velocity in m/s
        self.vel_z = 0.0        # no movement in z
        
        # Boundaries to keep object in workspace
        self.min_x, self.max_x = 0.25, 0.55  # x boundaries
        self.min_y, self.max_y = -0.4, 0.4  # y boundaries
        
        # Initialize position and start time
        self.pos_x = self.start_x
        self.pos_y = self.start_y
        self.pos_z = self.start_z
        self.start_time = self.get_clock().now()
        self.last_time = self.start_time
        
        self.get_logger().info('Fake object motion publisher started (linear motion)')

    def timer_callback(self):
        # Get current time
        now = self.get_clock().now()
        
        # Calculate time delta since last update
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now
        
        # Update position based on velocity
        self.pos_x += self.vel_x * dt
        self.pos_y += self.vel_y * dt
        
        # Bounce when hitting boundaries
        if self.pos_x <= self.min_x or self.pos_x >= self.max_x:
            self.vel_x = -self.vel_x
            # Ensure position is within bounds
            self.pos_x = max(self.min_x, min(self.pos_x, self.max_x))
            
        if self.pos_y <= self.min_y or self.pos_y >= self.max_y:
            self.vel_y = -self.vel_y
            # Ensure position is within bounds
            self.pos_y = max(self.min_y, min(self.pos_y, self.max_y))
        
        # Create and fill message
        msg = ObjectState()
        msg.header = Header()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = "world"
        
        msg.position = Point(x=self.pos_x, y=self.pos_y, z=self.pos_z)
        msg.velocity = Vector3(x=self.vel_x, y=self.vel_y, z=self.vel_z)
        
        # Publish message
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeObjectMotionPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()