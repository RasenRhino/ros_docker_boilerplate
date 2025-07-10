#!/bin/bash

# ROS Noetic VNC Container Startup Script
# This script is executed by docker-compose

set -e

echo "Starting ROS Noetic VNC Container..."
echo "Environment Variables:"
echo "  VNC Resolution: $VNC_RESOLUTION"
echo "  VNC Geometry: $VNC_GEOMETRY"
echo "  VNC Scale: $VNC_SCALE"
echo "  Display: $DISPLAY"
echo "  ROS Workspace: $ROS_WORKSPACE"
echo ""

# Start SSH server
echo "Starting SSH server..."
/usr/sbin/sshd
echo "SSH server started on port 22"
echo ""

# Clean up any existing X lock files
echo "Cleaning up X lock files..."
rm -f /tmp/.X*-lock /tmp/.X11-unix/X*

# Start Xvfb
echo "Starting Xvfb..."
Xvfb :1 -screen $XVFB_SCREEN ${VNC_RESOLUTION}x${XVFB_DEPTH} -ac +extension GLX +render -noreset &
export DISPLAY=:1
echo "Xvfb started on display :1"

# Setup X authentication
echo "Setting up X authentication..."
touch /root/.Xauthority
xauth add :1 . $(mcookie)
echo "X authentication configured"
echo ""

# Wait for Xvfb to initialize
echo "Waiting for Xvfb to initialize..."
sleep 3

# Start Fluxbox window manager
echo "Starting Fluxbox window manager..."
fluxbox -log $FLUXBOX_LOG &
echo "Fluxbox started"
echo ""

# Wait for Fluxbox to initialize
echo "Waiting for Fluxbox to initialize..."
sleep 3

# Start VNC server
echo "Starting VNC server..."
x11vnc -display :1 -nopw -listen $VNC_LISTEN -xkb -ncache $VNC_NOCACHE -ncache_cr -forever -geometry $VNC_GEOMETRY -shared -repeat -scale_cursor $VNC_SCALE_CURSOR -scale $VNC_SCALE -auth guess &
echo "VNC server started on port 5900"
echo ""

# Wait for VNC to initialize
echo "Waiting for VNC to initialize..."
sleep 3

# Start noVNC web interface
echo "Starting noVNC web interface..."
/opt/noVNC/utils/novnc_proxy --vnc $NOVNC_VNC --listen $NOVNC_LISTEN --web $NOVNC_WEB --heartbeat $NOVNC_HEARTBEAT &
echo "noVNC started on port 8080"
echo ""

# Source ROS environment
echo "Sourcing ROS environment..."
source $ROS_SOURCE
if [ -f "$ROS_DEVEL_SOURCE" ]; then
    source $ROS_DEVEL_SOURCE
    echo "ROS workspace sourced"
else
    echo "ROS workspace not built yet - run 'catkin_make' inside container"
fi
echo "ROS environment sourced"
echo ""

# Start tmux session
echo "Starting tmux session..."
tmux new-session -d -s $TMUX_SESSION
tmux send-keys -t $TMUX_SESSION "source $ROS_SOURCE" C-m
tmux send-keys -t $TMUX_SESSION "source $ROS_DEVEL_SOURCE" C-m
tmux send-keys -t $TMUX_SESSION "cd $ROS_WORKSPACE" C-m
echo "Tmux session started"
echo ""

# Print access information
echo "Container startup complete!"
echo "Access methods:"
echo "  - VNC Client: localhost:5900"
echo "  - Browser VNC: http://localhost:8080/vnc.html"
echo "  - SSH: ssh root@localhost -p 2222 (password: ros123)"
echo "  - Tmux: tmux attach-session -t $TMUX_SESSION"
echo ""
echo "Ready for ROS development!"

# Keep container running
exec "$@" 