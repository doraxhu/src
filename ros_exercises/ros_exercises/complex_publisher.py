# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
import math
import random

class ComplexPublisher(Node):

    def __init__(self):
        super().__init__('complex_publisher')
        self.publisher_ = self.create_publisher(LaserScan, 'fake_scan', 10)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.prev_scan = None

    def timer_callback(self):
        msg = LaserScan()
	    # LaserScan parameters
        #msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.stamp = rclpy.time.Time().to_msg()
        msg.header.frame_id = "base_link" 

        msg.angle_min = -2/3*math.pi
        msg.angle_max = 2/3*math.pi
        msg.angle_increment = 1/300*math.pi
	    #msg.time_increment
        if self.prev_scan == None:
            msg.scan_time = float(self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec/10**9)
        else:
            msg.scan_time = float(self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec/10**9 - self.prev_scan)
         #   msg.scan_time = 0.05
        self.prev_scan = self.get_clock().now().to_msg().sec
        
        msg.range_min = 1.0
        msg.range_max = 10.0
        msg.ranges = [random.uniform(msg.range_min, msg.range_max) for i in range(int((msg.angle_max-msg.angle_min)/msg.angle_increment) + 1)]
	    #msg.intensities

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.scan_time)


def main(args=None):
    rclpy.init(args=args)

    complex_publisher = ComplexPublisher()

    rclpy.spin(complex_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    complex_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
