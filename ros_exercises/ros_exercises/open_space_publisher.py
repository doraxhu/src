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
from std_msgs.msg import Float32
import math



class ComplexSubscriber(Node):

    def __init__(self):
        super().__init__('open_space_publisher')
        self.subscription = self.create_subscription(
            LaserScan,
            'fake_scan',
            self.listener_callback,
            10)
        timer_period = 0.05
        self.subscription  # prevent unused variable warning
        self.publisher_1 = self.create_publisher(Float32, 'open_space/distance', 10)
        self.publisher_2 = self.create_publisher(Float32, 'open_space/angle', 10)


    def listener_callback(self, msg):
        # find the range element with the greatest value
        # by getting the range from msg
        greatest = Float32()
        greatest.data = msg.ranges[0]
        index = 0
        count = 0
        for ret in msg.ranges:
            if ret > greatest.data: 
                greatest.data = ret
                index = count
            count += 1

        # publish corresponding angle and return value
        self.publisher_1.publish(greatest)
        angle = Float32()
        angle.data = msg.angle_min + index * msg.angle_increment
        self.publisher_2.publish(angle)
        self.get_logger().info('ANGLE: "%s"' % angle)


def main(args=None):
    rclpy.init(args=args)

    complex_subscriber = ComplexSubscriber()

    rclpy.spin(complex_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    complex_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
