FROM ros:foxy-ros-base

RUN apt update
RUN apt upgrade -y
RUN apt install nano
RUN apt install wget
RUN apt install git

WORKDIR /root/
RUN git clone https://github.com/chhas/pub_sub_eval.git
#RUN colcon build --symlink-install
#RUN source ./install/setup.bash
#RUN ros2 run py_pubsub talker & ros2 run py_pubsub listener 
