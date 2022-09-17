FROM ros:noetic-ros-core

RUN apt update

# 1. Install standard packages from apt.
RUN apt install -y make gcc g++

# 2. Install ROS packages from apt.
RUN apt install -y ros-noetic-move-base ros-noetic-image-transport ros-noetic-cv-bridge

# 3. Install Teknic's sFoundation library.
# These instructions are taken straight from `readme.txt' in teknic
COPY teknic /teknic
WORKDIR /teknic/sFoundation
RUN make -j$(nproc)
RUN cp MNuserDriver20.xml /usr/local/lib
RUN cp libsFoundation20.so /usr/local/lib
RUN ldconfig

# 4. Build the Lunabotics code.
COPY ros /lunabotics/ros
COPY utils /lunabotics/utils
WORKDIR /lunabotics/ros/catkin_ws
RUN /ros_entrypoint.sh catkin_make

# 5. Add scripts.
COPY scripts /lunabotics/scripts
