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

from eval_msg.msg import StampedInt8Array


class MinimalSubscriber(Node):

    def __init__(self, topic:str = "topic"):

        super().__init__("minimal_subscriber")

        self.subscription = self.create_subscription(
            StampedInt8Array, topic, self.callback, 10)

    def callback(self, msg):
        
        diff = (self.get_clock().now().nanoseconds - msg.stamp) / 1e6
        
        self.get_logger().info(f"[{msg.stamp}] Received: {len(msg.data) / 2 ** 10} KiB in {diff:.3f} ms" )


#if __name__ == '__main__':
def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # minimal_subscriber.destroy_node()
    rclpy.shutdown()