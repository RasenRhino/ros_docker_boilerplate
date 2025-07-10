# ROS Noetic Docker Setup

A complete ROS Noetic development environment running in Docker. Get ROS running in minutes with GUI tools, SSH access.

**Compatible with:** macOS Silicon, macOS Intel, Linux, Windows (with Docker Desktop)

##  TLDR - Quick Start

```bash
# 1. Start the container
docker-compose up -d --build

# 2. Wait 20 seconds, then connect via SSH
ssh root@localhost -p 2222
# Password: ros123

# 3. Build and run the example
cd /opt/ros_ws
catkin_make
source devel/setup.bash
roslaunch turtle_circle_package turtle_circle.launch
```

**NOTE:** GUI might not work, for that go to VNC

**For GUI** Open `http://localhost:8080/vnc.html` in your browser.


---

## Features 

-  **ROS Noetic** - Full ROS environment with all tools
-  **GUI Access** - RViz, Gazebo, rqt via VNC or web browser
-  **SSH Access** - Terminal-based development with tmux
-  **Example Package** - Turtle circle demo included

## Prerequisites

- Docker Desktop (for your platform)
- A web browser (for GUI access)
- SSH client (built into most systems)

## Quick Start Guide

### 1. Start the Container
```bash
docker-compose up -d --build
```


### 2. Wait for Startup
The container takes about 20 seconds to fully start. Check progress with:
```bash
docker logs ros_noetic_vnc --tail 10
```

### 3. How to Connect 

**Option A: SSH**
```bash
ssh root@localhost -p 2222
# Password: ros123
```

**Option B: GUI**
- Open `http://localhost:8080/vnc.html` in your browser
- Click "Connect"

### 4. Run the Example
```bash
# SSH into the container
ssh root@localhost -p 2222

# Build the workspace
cd /opt/ros_ws
catkin_make
source devel/setup.bash

# Run the turtle circle demo
roslaunch turtle_circle_package turtle_circle.launch
```

## Access Methods

| Method | URL/Command |
|--------|-------------|
| **SSH** | `ssh root@localhost -p 2222` (password : ros123)| 
| **Web VNC** | `http://localhost:8080/vnc.html` | 
| **VNC Client** | `localhost:5900` |

## Ports

- `2222` - SSH access
- `5900` - VNC server
- `8080` - Web VNC interface
- `11311` - ROS master

## Working with ROS

### Your Workspace
Your ROS workspace is mounted at `/opt/ros_ws` 

### Creating Packages
```bash
cd /opt/ros_ws/src
catkin_create_pkg my_package rospy std_msgs
cd /opt/ros_ws
catkin_make
source devel/setup.bash
```

### Running ROS Tools
```bash
# Terminal-based tools (work in SSH)
rostopic list
rostopic echo /topic_name
rosnode list

# GUI tools (use VNC/web browser)
rviz
gazebo
rqt_graph
```

## Example: Turtle Circle Package

The included example shows a complete ROS package:

Will only run in VNC as requires a display server

```bash
# Build and run
cd /opt/ros_ws
catkin_make
source devel/setup.bash
roslaunch turtle_circle_package turtle_circle.launch
```

This will make a turtle move in circles. Open `http://localhost:8080/vnc.html` to see 



## Common Commands

```bash
# Container management
docker-compose up -d          # Start
docker-compose down           # Stop
docker-compose restart        # Restart
docker logs ros_noetic_vnc    # View logs

# SSH access
ssh root@localhost -p 2222    # Connect
# Password: ros123

# ROS workspace
cd /opt/ros_ws               # Go to workspace
catkin_make                  # Build
source devel/setup.bash      # Source workspace
```


### VNC debug 
- Wait 20 seconds for full startup
- Try the web interface: `http://localhost:8080/vnc.html`
- Check logs: `docker logs ros_noetic_vnc`

### GUI Tools Don't Work in SSH
For GUI tools you need a display server. Use VNC :
1. Open `http://localhost:8080/vnc.html`
2. SSH from within the VNC session
3. Now GUI tools will work

### Package Build Issues
```bash
cd /opt/ros_ws
catkin clean
catkin_make
source devel/setup.bash
```


## What's Inside

The container includes:
- Ubuntu 20.04 with ROS Noetic
- SSH server with tmux
- VNC server with Fluxbox window manager
- noVNC web interface
- Example turtle circle package
- All ROS tools (RViz, Gazebo, rqt, etc.)

## How It Works

- Container runs **Ubuntu 20.04** 
- **X11 display server** runs inside the container
- **VNC** provides GUI access from any platform
- **SSH** works the same on all systems


---
