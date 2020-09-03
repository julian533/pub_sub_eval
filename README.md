# Resources
https://index.ros.org/doc/ros2/Tutorials/Writing-A-Simple-Py-Publisher-And-Subscriber/

https://index.ros.org/doc/ros2/Tutorials/Custom-ROS2-Interfaces/

https://medium.com/@danieljeswin/introduction-to-programming-with-ros2-topics-e0b72dfd766b

# Quick Start
$ git clone https://github.com/chhas/pub_sub_eval

$ cd pub_sub_eval

$ docker build -t "pub_sub_eval:Dockerfile" .

$ docker run -it \<id\>

Within container:

$ cd ~

$ colcon build --symlink-install

$ source ./install/setup.bash

$ ros2 run py_pubsub talker & ros2 run py_pubsub listener 