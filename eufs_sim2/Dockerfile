# ----------------------------------NOTES------------------------------------
# Build an image using this Dockerfile with the following command:
# docker build --ssh default -t eufs_sim2 . --progress=plain
#
# New users might need to run the following: 
# a) eval "$(ssh-agent -s)"
# b) ssh-add ~/.ssh/id_rsa
#
#
# You can now launch the simulator in two ways:
# 1) Launch directly using the play button next to the image on Docker Desktop.
#    Note: foxglove_bridge uses port 8765 by default. This can be modified if you choose to run the simulator using method 2.
#
# 2) Launch from the terminal:
#    docker run --rm -it -p 8765:8765 eufs_sim2
#
#   Notes:
#   You can customize the Foxglove port inside the container, as well as the port that interfaces with your system.
#   a) Consider 1234 to be the internal port for the Foxglove bridge.
#   b) Consider 5678 to be the external port interfacing with the system (the one you will actually listen to, from Foxglove).
#   The command will be:
#   docker run --rm -it -p 5678:1234 -e CONTAINER_PORT=1234 eufs_sim2
#
#   You can launch a terminal for debugging purposes by adding "bash" to the command, and then launch the simulation. The commands will be:
#   a) docker run --rm -it -p 8765:8765 eufs_sim2 bash
#   b) source $EUFS_MASTER/install/setup.bash; ros2 launch eufs_sim2 eufs_sim2.launch.py & ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=${CONTAINER_PORT} & wait
# ---------------------------------------------------------------------------



# syntax=docker/dockerfile:1.2
FROM ros:humble
SHELL ["/bin/bash", "-c"]

ENV CONTAINER_PORT=8765
EXPOSE ${CONTAINER_PORT}

# Install necessary packages
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && \
    apt-get install --yes --no-install-recommends \
    sudo \
    openssh-client \
    git \
    unzip \
    python3-rosdep\
    python3-pip \
    python3-colcon-common-extensions \
    libignition-common3-dev \
    libfmt-dev \
    libgoogle-glog-dev \
    libceres-dev \
    libjsoncpp-dev \
    ros-humble-ackermann-msgs  \
    ros-humble-geodesy \
    ros-humble-foxglove-bridge \
    ros-humble-xacro \
    ros-humble-joy \
    ros-humble-velodyne-driver \
    ca-certificates \
    && update-ca-certificates \
    && apt-get clean \
    && apt-get install -y python3-skimage
    

# Install python packages
RUN pip3 install xacro tqdm pytest-cov setuptools==68


# Set up SSH known hosts
RUN mkdir -p -m 0600 ~/.ssh && \
    ssh-keyscan -H gitlab.com >> ~/.ssh/known_hosts


# SSH keys validation for gitlab 
RUN --mount=type=ssh if [ -z "$(ls -A ~/.ssh/)" ]; then \
        echo "No SSH keys found. Please generate SSH keys and add them to GitLab." && exit 1; \
    fi


# Temporarily disable stderr redirection to get error output from git clone
ENV REPOSITORY_URL="git@gitlab.com:eufs/infrastructure_group/scripts.git"
RUN --mount=type=ssh git ls-remote ${REPOSITORY_URL} >/dev/null 2>&1 || \ 
    (echo "SSH keys not registered on GitLab. Please add them before continuing." && exit 1)


# Set EUFS environment variables globally in the Dockerfile
ENV EUFS_MASTER=/root/eufs/eufs-master \
    COLCON_DEFAULTS_FILE=/root/eufs/eufs-master/.colcon/defaults.yaml \
    ROS_PYTHON_VERSION=3 \
    EUFS_CLI_HOME=/root/eufs/eufs_cli


# Cloning only necessary packages from EUFS MASTER
RUN --mount=type=ssh git clone git@gitlab.com:eufs/eufs-master.git ${EUFS_MASTER} && \
                    cd $EUFS_MASTER && \
                    git submodule update --init -- src/simulation/eufs_sim2 \
                    src/simulation/vehicle_models \
                    src/simulation/state_lib \
                    src/localisation/map_lib \
                    src/infrastructure/eufs_msgs \
                    src/infrastructure/eufs_gmock_matchers \
                    src/localisation/slam/eufs_logger && \
                    find . -type d -empty ! -path "./.git*" -delete


# Cloning EUFS CLI
RUN --mount=type=ssh git clone git@gitlab.com:eufs/eufs_cli.git ${EUFS_CLI_HOME} && \
    bash ${EUFS_CLI_HOME}/install.sh && \
    eufs update
    

# Pull latest changes
RUN --mount=type=ssh eufs checkout simulation master && eufs checkout infrastructure master && eufs checkout localisation master
RUN --mount=type=ssh eufs pull simulation && eufs pull infrastructure && eufs pull localisation


# Install dependencies
RUN echo 'yaml https://gitlab.com/eufs/eufs_rosdep/-/raw/master/python.yaml' | sudo tee /etc/ros/rosdep/sources.list.d/19-eufs.list
RUN rosdep update && rosdep install --from-paths $EUFS_MASTER/src --ignore-src -r -y
RUN source /opt/ros/humble/setup.bash && \
    cd $EUFS_MASTER && \ 
    colcon build --symlink-install --continue-on-error


# Build ROS packages
RUN bash -c "source /opt/ros/humble/setup.bash && \
    cd $EUFS_MASTER && \
    colcon build --packages-select eufs_msgs eufs_gmock_matchers eufs_logger state_lib"


RUN bash -c "source /opt/ros/humble/setup.bash && \
    cd $EUFS_MASTER && \
    colcon build --packages-select map_lib vehicle_models eufs_sim2"


CMD ["/bin/bash", "-c", "source $EUFS_MASTER/install/setup.bash; ros2 launch eufs_sim2 eufs_sim2.launch.py & ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=${CONTAINER_PORT} & wait"]
