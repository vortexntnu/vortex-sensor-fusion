FROM ros:melodic

# FROM arm64v8/ros:melodic

ARG distro=melodic
ENV DEBIAN_FRONTEND=noninteractive

# Create vortex user
RUN useradd -ms /bin/bash \
    --home /home/vortex  vortex
RUN echo "vortex:vortex" | chpasswd
RUN usermod -aG sudo vortex

# ROS package dependencies
RUN apt update && \
    apt install -y \
    apt-utils \
    libeigen3-dev \
    libeigen3-dev \
    libglfw3-dev \
    libglew-dev \
    libjsoncpp-dev \
    ros-$distro-tf \
    ros-$distro-tf2 \
    libtclap-dev \
    wget \
    nano \ 
    protobuf-compiler \
    python-osrf-pycommon \
    python-openpyxl \
    python-yaml \
    python-enum34 \
    python-pip \
    python-catkin-tools \
    libgeographic-dev \
    ros-$distro-vision-opencv \
    ros-$distro-image-transport  \
    software-properties-common	

RUN apt -y full-upgrade 

RUN wget https://bootstrap.pypa.io/pip/2.7/get-pip.py
RUN python get-pip.py
RUN pip install --upgrade setuptools

RUN CMAKE_ARGS="-DOPENCV_ENABLE_NONFREE=ON"

RUN python -m pip install --upgrade pip
RUN pip install  \
    rospkg \
    opencv-python==3.4.2.17 \
    opencv-contrib-python==3.4.2.17 \
    numpy

RUN apt update


RUN echo "source /opt/ros/melodic/setup.bash" >> /home/vortex/.bashrc
RUN echo "source /home/vortex/auv_ws/devel/setup.bash" >> /home/vortex/.bashrc
RUN echo "alias srcdev='source devel/setup.bash'" >> /home/vortex/.bashrc

RUN mkdir -p /home/vortex/auv_ws/src
RUN chown -R vortex /home/vortex/auv_ws

#COPY ./entrypoint.sh /

#CMD ["/bin/bash"]