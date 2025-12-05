FROM ubuntu:focal

ARG UNAME=<user_name_to_replace>
ARG REPO_PATH=<absolute_path_to_replace>
ARG UID=1000
ARG GID=1000


RUN apt-get update && apt-get install -y locales lsb-release curl cmake g++ wget unzip
RUN apt-get install -y git
RUN apt-get install -y gnupg2
ARG DEBIAN_FRONTEND=noninteractive
RUN dpkg-reconfigure locales

# installing ROS
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN apt-get update 
RUN apt-get install -y ros-noetic-desktop-full 
RUN apt-get install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential python3-catkin-tools -y
RUN apt-get install -y python3-pip
RUN apt install python-is-python3
RUN pip install graphviz
RUN pip install dill

# For sound (not working)
RUN apt install libasound2-dev
RUN pip install simpleaudio
RUN apt-get install -y alsa-utils

# dependencies for simulator
RUN apt-get install -y ros-noetic-navigation
RUN apt-get install -y ros-noetic-four-wheel-steering-msgs
RUN apt-get install -y ros-noetic-moveit
RUN apt-get install -y ros-noetic-rviz-visual-tools
RUN apt-get install -y ros-noetic-moveit-visual-tools
RUN apt-get install -y liborocos-bfl-dev
RUN apt-get install -y tmux
RUN apt-get install -y vim
RUN apt-get install -y terminator

RUN rosdep init 
RUN rosdep update

# RUN mkdir -p /home/${UNAME}/dependencies
# RUN cd /home/${UNAME}/dependencies/ && \
#   git clone https://github.com/ros-controls/urdf_geometry_parser.git && \
#   git clone https://github.com/pal-robotics/ddynamic_reconfigure.git && \
#   git clone https://github.com/DLu/wu_ros_tools.git && \
#   git clone https://github.com/wg-perception/people.git
# # to be added to src folder (checkout "noetic" branch for people pkg)
# RUN cd /home/${UNAME}/dependencies/people && git checkout noetic


RUN groupadd -g ${GID} ${UNAME}
RUN useradd -m -u ${UID} -g ${GID} -s /bin/bash ${UNAME} 
RUN echo "${UNAME}:${UNAME}" | chpasswd
RUN usermod -aG sudo ${UNAME}

WORKDIR /home/${UNAME}/

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

COPY entrypoint.sh ${REPO_PATH}/docker/entrypoint.sh
RUN chmod +x ${REPO_PATH}/docker/entrypoint.sh

ENV ENTRY_FILE=${REPO_PATH}/docker/entrypoint.sh

USER ${UNAME}

ENTRYPOINT /bin/bash $ENTRY_FILE