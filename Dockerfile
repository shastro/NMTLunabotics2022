FROM ros:noetic-ros-core

RUN apt-get update

# 1. Install standard packages from apt.
RUN apt-get install -y make gcc g++ inetutils-ping net-tools

# 2. Install ROS packages from apt.
RUN apt-get install -y ros-noetic-move-base ros-noetic-image-transport ros-noetic-cv-bridge

# 3. Install Teknic's sFoundation library.
# These instructions are taken straight from `readme.txt' in teknic
COPY teknic /lunabotics/teknic
WORKDIR /lunabotics/teknic/sFoundation
RUN make -j$(nproc)
RUN cp MNuserDriver20.xml /usr/local/lib
RUN cp libsFoundation20.so /usr/local/lib
RUN ldconfig

# 4. Add scripts.
COPY scripts /lunabotics/scripts

# 5. Install SSH keys and hosts file.
COPY docker-scripts/david_key /root/.ssh/id_rsa
COPY docker-scripts/david_key.pub /root/.ssh/id_rsa.pub
COPY docker-scripts/david_known_hosts /root/.ssh/known_hosts
COPY docker-scripts/david_hosts /etc/hosts

# 6. Leave working directory in place to compile Lunabotics code.
WORKDIR /lunabotics/ros/catkin_ws
