# PX4 wants humble + ubuntu 22.04 (jammy)
# https://docs.px4.io/main/en/ros2/user_guide.html#installation-setup
FROM osrf/ros:humble-desktop-full-jammy

ARG USERNAME=USERNAME
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ARG PX4_VERSION=v1.15.4
ARG DEBIAN_FRONTEND=noninteractive

SHELL [ "/bin/bash", "-c" ]

# Delete user if it exists in container (e.g Ubuntu Noble: ubuntu)
RUN if id -u $USER_UID ; then userdel `id -un $USER_UID` ; fi

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME

# Temp. fix for https://github.com/osrf/docker_images/issues/807
RUN rm /etc/apt/sources.list.d/ros2-latest.list \
    && rm /usr/share/keyrings/ros2-latest-archive-keyring.gpg
RUN apt-get update \
    && apt-get install -y ca-certificates curl
RUN export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') ;\
    curl -L -s -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" \
    && apt-get update \
    && apt-get install /tmp/ros2-apt-source.deb \
    && rm -f /tmp/ros2-apt-source.deb

RUN apt update \
    && apt install -y --no-install-recommends \
    sudo \
    iproute2 \
    udev \
    wget \
    && rm -rf /var/lib/apt/lists/*

RUN apt update && apt upgrade -y

# [Optional] Add sudo support. Omit if you don't need to install software after connecting.
RUN echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Install PX4
WORKDIR /opt

# PX4 source + tool-chain & simulator deps (via ubuntu.sh)
RUN git clone https://github.com/PX4/PX4-Autopilot.git --recursive --branch ${PX4_VERSION} \
    && bash PX4-Autopilot/Tools/setup/ubuntu.sh \
    && rm -rf /var/lib/apt/lists/* 

# Micro XRCE-DDS Agent so ROS 2 can talk to PX4
RUN git clone -b v3.0.1 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git \
    && cd Micro-XRCE-DDS-Agent && cmake -B build -DCMAKE_BUILD_TYPE=Release \
    && cmake --build build -j$(nproc) && cmake --install build \
    && ldconfig && cd /opt && rm -rf Micro-XRCE-DDS-Agent

# Python helpers requested by the guide (empy, pyros-genmsg, …)
RUN python3 -m pip install --no-cache-dir --upgrade pip \
    && python3 -m pip install --no-cache-dir \
    empy==3.3.4 pyros-genmsg setuptools \
    toml numpy pandas jinja2 pyserial pyulog matplotlib packaging pyquaternion kconfiglib jsonschema jinja2 pyros-genmsg future

RUN cd PX4-Autopilot \
    && make px4_sitl

ENV SHELL="/bin/bash"
USER $USERNAME

# ********************************************************
# * Anything else you want to do like clean up goes here *
# ********************************************************

RUN sudo chown -R $USERNAME:$USERNAME /opt/PX4-Autopilot

# Setup bashrc
## Make ros2 public to the system via `source` command
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/$USERNAME/.bashrc
RUN echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /home/$USERNAME/.bashrc

RUN echo "if [ -f \"/workspaces/ros2_ws/install/setup.bash\" ]; then" >> /home/$USERNAME/.bashrc \
    && echo "    source /workspaces/ros2_ws/install/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "else" >> /home/$USERNAME/.bashrc \
    && echo "    echo \"/workspaces/ros2_ws/install/setup.bash has not yet been created.\"" >> /home/$USERNAME/.bashrc \
    && echo "    echo \"Make sure to build your package(s).\"" >> /home/$USERNAME/.bashrc \
    && echo "fi" >> /home/$USERNAME/.bashrc

CMD ["/bin/bash"]
