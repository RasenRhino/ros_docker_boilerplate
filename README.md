# ROS Noetic Docker Setup for macOS Silicon

This setup provides ROS Noetic running in Docker with SSH access, tmux sessions, and multiple GUI access methods (VNC/Web) on macOS Silicon (Apple Silicon). The container includes a complete ROS development environment with GUI tools accessible via VNC or web browser.

## Features

- ✅ **ROS Noetic** with full development environment
- ✅ **SSH Access** for terminal-based development
- ✅ **VNC Server** for GUI access via VNC clients
- ✅ **noVNC Web Interface** for browser-based GUI access
- ✅ **tmux Sessions** for persistent development sessions
- ✅ **Mounted Workspace** for persistent file storage
- ✅ **Multiple GUI Tools** (RViz, Gazebo, rqt, etc.)
- ✅ **Turtle Circle Package** included as example

## Prerequisites

- Docker Desktop for Mac (with Apple Silicon support)
- SSH client (built into macOS)
- A VNC viewer (optional, for GUI access)
- **OR** A modern web browser (Chrome, Firefox, Safari, Edge)

## Quick Start

1. **Build and run the container:**
   ```bash
   docker-compose up -d --build
   ```

2. **Wait for container to fully start (15-20 seconds):**
   ```bash
   docker logs ros_noetic_vnc --tail 10
   ```

3. **Connect via SSH (Recommended for development):**
   ```bash
   ssh root@localhost -p 2222
   # Password: ros123
   ```

4. **Connect via GUI (Choose one method):**
   
   **Method 1: Web Browser (Easiest)**
   - Open your web browser
   - Go to `http://localhost:8080/vnc.html`
   - Click "Connect" to access the VNC session
   
   **Method 2: VNC Client**
   - Open your VNC viewer (RealVNC, TigerVNC, etc.)
   - Connect to `localhost:5900`
   - No password is required

## Access Methods

### **SSH Access (Recommended for Development)**
```bash
ssh root@localhost -p 2222
# Password: ros123
```

**Benefits of SSH:**
- ✅ **Best performance** for development
- ✅ **tmux sessions** for persistent work
- ✅ **File editing** with vim/nano
- ✅ **Terminal-based** ROS tools
- ✅ **Port forwarding** for GUI tools
- ✅ **Direct access** to ROS workspace

### **GUI Access (VNC/Web)**
- **Web Browser:** `http://localhost:8080/vnc.html`
- **VNC Client:** `localhost:5900`
- **macOS Screen Sharing:** `vnc://localhost:5900`

## Ports

- `2222`: SSH access
- `5900`: VNC server (GUI access via VNC client)
- `8080`: noVNC web interface (GUI access via web browser)
- `11311`: ROS master
- `11312`: ROS parameter server

## File Structure

```
ros_test/
├── Dockerfile              # ROS Noetic container definition
├── docker-compose.yml      # Container orchestration
├── scripts/
│   └── startup.sh         # Container startup script
├── workspace/              # Your ROS workspace (mounted)
│   └── src/
│       └── turtle_circle_package/  # Example ROS package
├── config/                 # Configuration files (mounted)
└── README.md              # This file
```

## Working with ROS Projects

### **1. SSH into the Container**
```bash
ssh root@localhost -p 2222
# Password: ros123
```

### **2. Navigate to Workspace**
```bash
cd /opt/ros_ws
ls src/  # See your mounted packages
```

### **3. Build the ROS Workspace**
```bash
# First time setup
cd /opt/ros_ws
catkin_make

# Or use catkin build (if you prefer)
catkin build
```

### **4. Source the Workspace**
```bash
source devel/setup.bash
```

### **5. Run the Example Turtle Circle Package**
```bash
# Terminal 1: Start ROS master
roscore

# Terminal 2: Run the turtle circle node
rosrun turtle_circle_package turtle_circle_node.py

# Terminal 3: Run turtlesim to see the turtle
rosrun turtlesim turtlesim_node
```

### **6. Create a New ROS Package**
```bash
# Navigate to src directory
cd src/

# Create a new package
catkin_create_pkg my_robot_package rospy roscpp std_msgs

# Or create a Python-only package
catkin_create_pkg my_python_package rospy std_msgs
```

## Tmux Sessions

The container automatically starts a tmux session called `ros_session`.

### **Tmux Commands:**
```bash
# Attach to existing session
tmux attach -t ros_session

# Create new session
tmux new-session -s my_session

# List sessions
tmux list-sessions

# Split window horizontally
Ctrl+b "

# Split window vertically
Ctrl+b %

# Switch between panes
Ctrl+b arrow_keys

# Detach from session
Ctrl+b d
```

## Example: Turtle Circle Package

The included `turtle_circle_package` demonstrates a complete ROS package:

### **Package Structure:**
```
turtle_circle_package/
├── package.xml
├── CMakeLists.txt
├── scripts/
│   ├── turtle_circle_node.py
│   └── turtle_teleop_enhanced.py
├── launch/
│   ├── turtle_circle.launch
│   └── turtle_teleop.launch
└── README.md
```

### **Running the Turtle Circle:**
```bash
# SSH into container
ssh root@localhost -p 2222

# Build workspace
cd /opt/ros_ws
catkin_make
source devel/setup.bash

# Run the turtle circle
roslaunch turtle_circle_package turtle_circle.launch
```

### **Manual Control:**
```bash
# In another terminal
roslaunch turtle_circle_package turtle_teleop.launch
```

## GUI Tools Access

### **Method 1: VNC/Web Browser**
1. Connect via VNC (`localhost:5900`) or web (`http://localhost:8080/vnc.html`)
2. Open terminal in VNC session
3. Run GUI tools: `rviz`, `gazebo`, `rqt_graph`, etc.

### **Method 2: X11 Forwarding (SSH)**
```bash
# From your Mac terminal
ssh -X root@localhost -p 2222

# Then run GUI tools
rviz
gazebo
rqt_graph
```

## Development Tips

### **1. Persistent Development**
- Use tmux sessions for persistent work
- Your `workspace/` directory is mounted and persists
- All changes in `/opt/ros_ws/src/` are saved to your Mac

### **2. Package Management**
```bash
# Install additional ROS packages
apt-get update
apt-get install ros-noetic-package-name

# Install Python packages
pip3 install package-name
```

### **3. Debugging**
```bash
# Check ROS topics
rostopic list
rostopic echo /topic_name

# Check ROS services
rosservice list

# Check ROS parameters
rosparam list

# Visualize ROS graph
rosrun rqt_graph rqt_graph
```

### **4. Logging**
```bash
# View ROS logs
rqt_console

# Check system logs
docker-compose logs ros_noetic_vnc
```

## Commands Reference

```bash
# Start services
docker-compose up -d

# Stop services
docker-compose down

# View logs
docker-compose logs -f ros_noetic_vnc

# SSH into container
ssh root@localhost -p 2222

# Rebuild container
docker-compose up -d --build

# Remove everything
docker-compose down -v
docker system prune -a
```

## Troubleshooting

### **Container Won't Start**
```bash
# Check if Docker Desktop is running
docker ps

# Check container logs
docker logs ros_noetic_vnc

# Restart Docker Desktop if needed
```

### **VNC Connection Issues**

**Problem: "Cannot connect to VNC"**
- **Solution:** Wait 15-20 seconds for container to fully start
- **Check:** `docker logs ros_noetic_vnc --tail 20`

**Problem: Container keeps restarting**
- **Cause:** X authentication or lock file issues
- **Solution:** The startup script now handles these automatically

**Problem: VNC shows distorted aspect ratio**
- **Solution:** Use browser VNC (`http://localhost:8080/vnc.html`) for best results
- **Alternative:** Adjust VNC client settings to disable scaling

### **SSH Connection Issues**
```bash
# Check if container is running
docker ps

# Check SSH port
docker port ros_noetic_vnc

# Try verbose SSH
ssh -v root@localhost -p 2222
```

### **GUI Not Working**
```bash
# Check if VNC is running
docker exec ros_noetic_vnc ps aux | grep x11vnc

# Check Xvfb
docker exec ros_noetic_vnc ps aux | grep Xvfb

# Restart container
docker-compose restart
```

### **Package Build Issues**
```bash
# Clean build
cd /opt/ros_ws
catkin clean
catkin_make

# Check dependencies
rosdep install --from-paths src --ignore-src -r -y
```

### **Performance Issues**
- Use SSH for terminal work (best performance)
- Use VNC for GUI tools
- Web browser access may be slower but more reliable

## Technical Details

### **Container Services**
- **SSH Server:** Port 22 (mapped to 2222)
- **VNC Server:** x11vnc on port 5900
- **noVNC:** Web interface on port 8080
- **Xvfb:** Virtual display :1
- **Fluxbox:** Window manager
- **tmux:** Session management

### **Environment Variables**
```bash
VNC_RESOLUTION=1364x768
VNC_GEOMETRY=1364x768
VNC_SCALE=1
DISPLAY=:1
ROS_WORKSPACE=/opt/ros_ws
```

### **X Authentication**
The container automatically handles X authentication using:
- Xvfb for virtual display
- xauth for authentication
- x11vnc with `-auth guess` for VNC access

## Notes

- The container runs Ubuntu 20.04 with ROS Noetic
- SSH provides the best development experience
- tmux sessions persist across SSH connections
- The workspace is mounted for persistent development
- All ROS tools (RViz, Gazebo, etc.) are available
- Default SSH password: `ros123`
- VNC resolution: 1364x768 (optimized for web access)
- Container automatically handles X authentication and lock file cleanup 