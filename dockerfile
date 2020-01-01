FROM ros:kinetic
# place here your application's setup specifics
# CMD [ "roslaunch", "my-ros-app my-ros-app.launch" ]

# RUN apt-get update && apt-get install -y apt-utils build-essential psmisc vim-gtk
RUN rm /bin/sh && ln -s /bin/bash /bin/sh
# RUN apt-get update && apt-get install -q -y python-catkin-tools

# get curl and gazebo
RUN apt-get update; apt-get install curl
RUN sudo apt-get -q -y install gazebo7
RUN sudo apt-get -q -y install libgazebo7-dev

# automatically takes the latest version of gazebo: current 9.
# RUN curl -sSL http://get.gazebosim.org | sh

# get pip
RUN curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py
RUN python get-pip.py
RUN python -m pip install tqdm

# RUN apt-get install

# get ros melodic gazebo_ros_packages
RUN apt-get -q -y install ros-kinetic-gazebo-dev ros-kinetic-gazebo-ros ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-ros-control
RUN apt-get update && apt-get install -q -y && apt-get install ros-kinetic-xacro
RUN apt-get -q -y install ros-kinetic-turtlebot
RUN apt-get -q -y install ros-kinetic-desktop-full
RUN apt-get -q -y install ros-kinetic-map-server
# create ws, copy data and make
RUN source /opt/ros/kinetic/setup.bash && \
    mkdir -p /catkin_ws/src && \
    cd /catkin_ws/src && \
    catkin_init_workspace
COPY . /catkin_ws/src/comp_cov_sim
RUN source /opt/ros/kinetic/setup.bash && \
    cd /catkin_ws && \
    catkin_make && \
    catkin_make install

RUN apt-get -q -y install vim

CMD [ "/bin/bash" ]