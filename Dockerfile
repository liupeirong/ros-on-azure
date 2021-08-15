# To enable ssh & remote debugging on app service change the base image to the one below
# FROM mcr.microsoft.com/azure-functions/python:3.0-python3.7-appservice
FROM mcr.microsoft.com/azure-functions/python:3.0-python3.7

ENV AzureWebJobsScriptRoot=/home/site/wwwroot \
    AzureFunctionsJobHost__Logging__Console__IsEnabled=true

# for bash source command to work
SHELL ["/bin/bash", "-c"]

# install ros
RUN echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list
RUN wget http://packages.ros.org/ros.key -O - | apt-key add -
RUN apt-get update && apt-get install -y ros-noetic-ros-base

# install tools, libs, and source required to build cartographer
RUN apt-get update && apt-get install -y python-wstool python3-rosdep ninja-build stow
RUN source /opt/ros/noetic/setup.bash \
    && mkdir catkin_ws \
    && cd catkin_ws \
    && wstool init src \
    && wstool merge -t src https://raw.githubusercontent.com/cartographer-project/cartographer_ros/master/cartographer_ros.rosinstall \
    && wstool update -t src \
    && rosdep init \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y \
    && src/cartographer/scripts/install_abseil.sh

COPY requirements.txt /
RUN pip install -r /requirements.txt

# build cartographer
RUN source /opt/ros/noetic/setup.bash \
    && cd catkin_ws \
    && catkin_make_isolated --install --use-ninja

# Azure Function code
COPY . /home/site/wwwroot
