# ROS Noetic Docker Setup for macOS Silicon

A complete ROS Noetic development environment running in Docker on macOS Silicon. Get ROS running in minutes with GUI tools, SSH access, and persistent development sessions.

## 🚀 TLDR - Quick Start

**Want to get ROS running right now? Here's the 30-second version:**

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

**Need GUI tools?** Open `http://localhost:8080/vnc.html` in your browser.

That's it! You now have a full ROS development environment. 🎉

---

## What You Get

- ✅ **ROS Noetic** - Full ROS environment with all tools
- ✅ **GUI Access** - RViz, Gazebo, rqt via VNC or web browser
- ✅ **SSH Access** - Terminal-based development with tmux
- ✅ **Example Package** - Turtle circle demo included
- ✅ **Persistent Storage** - Your files stay between restarts

## Prerequisites

- Docker Desktop for Mac (Apple Silicon version)
- A web browser (for GUI access)
- SSH client (built into macOS)

## Quick Start Guide

### 1. Start the Container
```bash
docker-compose up -d --build
```

### 2. Wait for Startup
The container takes about 20 seconds to fully start. You can check progress with:
```bash
docker logs ros_noetic_vnc --tail 10
```

### 3. Connect and Start Developing

**Option A: SSH (Recommended for coding)**
```bash
ssh root@localhost -p 2222
# Password: ros123
```

**Option B: GUI (For RViz, Gazebo, etc.)**
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

| Method | URL/Command | Best For |
|--------|-------------|----------|
| **SSH** | `ssh root@localhost -p 2222` | Coding, terminal work |
| **Web VNC** | `http://localhost:8080/vnc.html` | GUI tools (RViz, Gazebo) |
| **VNC Client** | `localhost:5900` | Alternative GUI access |

## Ports

- `2222` - SSH access
- `5900` - VNC server
- `8080` - Web VNC interface
- `11311` - ROS master

## Working with ROS

### Your Workspace
Your ROS workspace is mounted at `/opt/ros_ws` and persists between container restarts.

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

```bash
# SSH into container
ssh root@localhost -p 2222

# Build and run
cd /opt/ros_ws
catkin_make
source devel/setup.bash
roslaunch turtle_circle_package turtle_circle.launch
```

This will make a turtle move in circles. Open `http://localhost:8080/vnc.html` to see it in action!

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

## Troubleshooting

### Container Won't Start
```bash
# Check Docker Desktop is running
docker ps

# Check logs
docker logs ros_noetic_vnc
```

### Can't Connect to VNC
- Wait 20 seconds for full startup
- Try the web interface: `http://localhost:8080/vnc.html`
- Check logs: `docker logs ros_noetic_vnc`

### GUI Tools Don't Work in SSH
This is normal! GUI tools need a display server. Use VNC instead:
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

## Tips

- **Use SSH for coding** - Better performance, tmux sessions
- **Use VNC for GUI** - RViz, Gazebo, rqt tools
- **Your files persist** - Everything in `/opt/ros_ws` is saved
- **tmux sessions** - Your work survives disconnections
- **Web VNC is easiest** - No client installation needed

## What's Inside

The container includes:
- Ubuntu 20.04 with ROS Noetic
- SSH server with tmux
- VNC server with Fluxbox window manager
- noVNC web interface
- Example turtle circle package
- All ROS tools (RViz, Gazebo, rqt, etc.)

## Need Help?

1. Check the logs: `docker logs ros_noetic_vnc`
2. Restart the container: `docker-compose restart`
3. Rebuild if needed: `docker-compose up -d --build`

---

**That's it! You now have a complete ROS development environment running on your Mac.** 🚀 