FROM ubuntu:20.04

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=noetic
ENV ROS_VERSION=1.15.15

# Install system dependencies
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common \
    wget \
    ca-certificates \
    openssh-server \
    tmux \
    sudo \
    && rm -rf /var/lib/apt/lists/*

# Add ROS repository
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# Install ROS Noetic
RUN apt-get update && apt-get install -y \
    ros-noetic-desktop-full \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init && rosdep update

# Install additional tools and dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-catkin-tools \
    git \
    vim \
    nano \
    terminator \
    && rm -rf /var/lib/apt/lists/*

# Install OpenVNC and X11 dependencies
RUN apt-get update && apt-get install -y \
    xvfb \
    x11vnc \
    fluxbox \
    xterm \
    x11-apps \
    dbus-x11 \
    tightvncserver \
    && rm -rf /var/lib/apt/lists/*

# Install noVNC for web browser access (simplified version)
RUN apt-get update && apt-get install -y \
    python3-websockify \
    && rm -rf /var/lib/apt/lists/*

# Clone and setup noVNC (minimal version)
RUN git clone --depth 1 https://github.com/novnc/noVNC.git /opt/noVNC \
    && git clone --depth 1 https://github.com/novnc/websockify /opt/noVNC/utils/websockify \
    && ln -s /opt/noVNC/vnc.html /opt/noVNC/index.html

# Install Python dependencies
RUN pip3 install --no-cache-dir \
    numpy \
    matplotlib \
    opencv-python \
    rospkg

# Create ROS workspace
RUN mkdir -p /opt/ros_ws/src
WORKDIR /opt/ros_ws

# Setup SSH
RUN mkdir /var/run/sshd
RUN echo 'root:ros123' | chpasswd
RUN sed -i 's/#PermitRootLogin prohibit-password/PermitRootLogin yes/' /etc/ssh/sshd_config
RUN sed -i 's/#PasswordAuthentication yes/PasswordAuthentication yes/' /etc/ssh/sshd_config

# Source ROS in bashrc
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc
RUN echo "source /opt/ros_ws/devel/setup.bash" >> /root/.bashrc

# Create tmux configuration
RUN echo 'set -g default-terminal "screen-256color"' > /root/.tmux.conf
RUN echo 'set -g history-limit 10000' >> /root/.tmux.conf
RUN echo 'set -g mouse on' >> /root/.tmux.conf
RUN echo 'bind-key -T copy-mode-vi MouseDragEnd1Pane send-keys -X copy-pipe-and-cancel "xclip -selection clipboard -i"' >> /root/.tmux.conf



# Expose SSH port
EXPOSE 22

# Set the default command (will be overridden by docker-compose)
CMD ["bash"] 