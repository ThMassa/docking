# Docker image avec Ubuntu 20.04, VSCode, ROS noetic et le rep git
FROM ubuntu:20.04

ARG DEBIAN_FRONTEND=noninteractive

RUN apt update && apt install locales
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
RUN export LANG=en_US.UTF-8

RUN apt install software-properties-common -y
RUN add-apt-repository universe
RUN apt update && apt-get install curl -y && apt-get install wget -y
RUN apt install nano -y

RUN apt install software-properties-common apt-transport-https wget -y
RUN wget -q https://packages.microsoft.com/keys/microsoft.asc -O- | apt-key add -
RUN add-apt-repository "deb [arch=amd64] https://packages.microsoft.com/repos/vscode stable main"
RUN apt install code 

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

RUN apt update
RUN apt install ros-noetic-desktop-full

RUN source /opt/ros/noetic/setup.bash
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN source ~/.bashrc

RUN apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
RUN apt install python3-rosdep
RUN rosdep init
RUN rosdep update

RUN apt-get install git -y
WORKDIR /home/Guerledan
RUN git clone https://github.com/ThMassa/docking.git
WORKDIR /docking



