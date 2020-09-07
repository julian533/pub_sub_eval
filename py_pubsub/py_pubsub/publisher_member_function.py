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

from eval_msg.msg import StampedInt8Array


class MinimalPublisher(Node):

    def __init__(self, dim_start:int, dim_step:int, topic:str = "topic", timer_delay:float=0.5):

        super().__init__("minimal_publisher")
        
        self.dim = dim_start
        self.dim_step = dim_step

        self.publisher = self.create_publisher(StampedInt8Array, topic, 10)
        self.timer = self.create_timer(timer_delay, self.callback)

    def callback(self):
        
        if self.dim >= 800:
            self.timer.stop()

        msg = StampedInt8Array()
        msg.data = np.random.randint(-127, 128, self.dim ** 2 * 4, dtype=np.int8).tolist()
        msg.stamp = self.get_clock().now().nanoseconds

        self.publisher.publish(msg)

        self.get_logger().info(f"Published: {len(msg.data) / 2 ** 10} KiB (dim = {self.dim})")

        self.dim += self.dim_step


#if __name__ == '__main__':
def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher(100, 50)

    rclpy.spin(minimal_publisher)

    # minimal_publisher.destroy_node()
    rclpy.shutdown()
