FROM ros:noetic-ros-core

RUN apt update

# 1. Install standard packages from apt.
RUN apt install -y make gcc g++ inetutils-ping net-tools

# 2. Install ROS packages from apt.
RUN apt install -y ros-noetic-move-base ros-noetic-image-transport ros-noetic-cv-bridge

# 3. Install Teknic's sFoundation library.
# These instructions are taken straight from `readme.txt' in teknic
COPY teknic /lunabotics/teknic
WORKDIR /lunabotics/teknic/sFoundation
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

# 6. Install SSH keys.
COPY docker-scripts/david_key /root/.ssh/id_rsa
COPY docker-scripts/david_key.pub /root/.ssh/id_rsa.pub
COPY docker-scripts/david_known_hosts /root/.ssh/known_hosts
