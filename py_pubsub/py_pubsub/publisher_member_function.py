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

import numpy as np

#from std_msgs.msg import Int8MultiArray
from eval_msg.msg import StampedInt8Array

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(StampedInt8Array, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.dim = 100
        self.num_bytes = 4*self.dim*self.dim

    def timer_callback(self):
        msg = StampedInt8Array()
        msg.data = np.random.randint(-127, 128, self.num_bytes, dtype=np.int8).tolist()
        msg.stamp = self.get_clock().now().to_msg()

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s" KiB (dim = "%s")' % ((len(msg.data) / 2**10), self.dim))
        self.dim += 50
        self.num_bytes = 4*self.dim*self.dim

        if self.dim == 800:
            self.destroy_node()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
