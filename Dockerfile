FROM ros:melodic-perception-bionic

ENV DEBIAN_FRONTEND noninteractive
ENV TERM xterm

RUN apt-get update && apt-get dist-upgrade -y && apt-get install -y --no-install-recommends \
    git \
    build-essential \
    dialog \
    make \
    gcc \
    g++ \
    locales \
    wget \
    software-properties-common \
    sudo \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    libxau6 \
    libxdmcp6 \
    libxcb1 \
    libxext6 \
    libx11-6 \
    tmux \
 && rm -rf /var/lib/apt/lists/*

# Now create the user
ARG UID=1000
ARG GID=1000
RUN addgroup --gid ${GID} openmv
RUN adduser --gecos "ROS User" --disabled-password --uid ${UID} --gid ${GID} openmv
RUN usermod -a -G dialout openmv
RUN mkdir config && echo "ros ALL=(ALL) NOPASSWD: ALL" > config/99_aptget
RUN cp config/99_aptget /etc/sudoers.d/99_aptget
RUN chmod 0440 /etc/sudoers.d/99_aptget && chown root:root /etc/sudoers.d/99_aptget

## isntall dependences
# pip and pyserial
RUN apt-get update && apt-get install -y curl  && curl https://bootstrap.pypa.io/pip/2.7/get-pip.py --output get-pip.py
RUN python2 get-pip.py && python2 -m pip install pyserial

## usb tools and vim 
RUN apt-get install -y usbutils vim

ENV HOME /home/openmv
    
## repository 
RUN mkdir -p ${HOME}/catkin_ws/src 
RUN apt-get update
WORKDIR ${HOME}/catkin_ws/src
RUN git clone https://github.com/EPVelasco/openmv_cam.git && \
    git clone https://github.com/ros-perception/camera_info_manager_py 
    
# Compile the catkin workspace
RUN . /opt/ros/melodic/setup.sh && \
    cd ${HOME}/catkin_ws && \
    catkin_make --only-pkg-with-deps camera_info_manager_py &&\
    catkin_make --only-pkg-with-deps openmv_cam
    
WORKDIR ${HOME}/catkin_ws
CMD ["bash", "-c", "cd ${HOME}/catkin_ws/src/openmv_cam && pull"]
#CMD ["bash", "-c", "source ${HOME}/catkin_ws/devel/setup.bash && rosrun openmv_cam openmv_cam_node.py"]
CMD ["bash", "-c", "source ${HOME}/catkin_ws/devel/setup.bash && roslaunch openmv_cam openmv_cam.launch serial_port:=/dev/ttyACM10"] ## you can change the port number

