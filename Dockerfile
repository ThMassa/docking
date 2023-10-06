FROM ubuntu:18.04

ARG DEBIAN_FRONTEND=noninteractive

RUN apt update && apt install locales
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
RUN export LANG=en_US.UTF-8

RUN apt install software-properties-common -y
RUN add-apt-repository universe
RUN apt update && apt-get install curl -y && apt-get install wget -y
RUN apt install nano -y

RUN apt update
RUN apt-get install python2 -y
RUN apt-get install libpython2.7 -y
RUN curl https://bootstrap.pypa.io/pip/2.7/get-pip.py --output get-pip.py -y
RUN python2 get-pip.py

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

RUN apt update
RUN apt-get install ros-melodic-desktop-full -y
RUN apt-get source

RUN source /opt/ros/melodic/setup.bash
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
RUN source ~/.bashrc

RUN apt-get install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential -y
RUN apt install python-rosdep -y
RUN rosdep init
RUN rosdep update

RUN apt install software-properties-common apt-transport-https wget -y
RUN wget -q https://packages.microsoft.com/keys/microsoft.asc -O- | apt-key add -
RUN add-apt-repository "deb [arch=amd64] https://packages.microsoft.com/repos/vscode stable main"
RUN apt-get install code -y

RUN apt-get install git -y

WORKDIR /home/Guerledan
RUN git clone https://github.com/ThMassa/docking.git
WORKDIR /docking



