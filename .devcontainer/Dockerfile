# FROM ros:jazzy-ros-base
FROM osrf/ros:jazzy-desktop-full

ARG USERNAME=USERNAME
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Delete user if it exists in container (e.g Ubuntu Noble: ubuntu)
RUN if id -u $USER_UID ; then userdel `id -un $USER_UID` ; fi

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME
#
# [Optional] Add sudo support. Omit if you don't need to install software after connecting.
RUN apt update \
    && apt install -y --no-install-recommends \
    sudo \
    iproute2 \
    udev \
    wget \
    && rm -rf /var/lib/apt/lists/*
RUN apt update && apt upgrade -y 
RUN echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
ENV SHELL="/bin/bash"

USER $USERNAME

# ********************************************************
# * Anything else you want to do like clean up goes here *
# ********************************************************

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
