# Turtle Circle Package

A ROS package that demonstrates turtle movement in circles using turtlesim.

## Features

- **Automatic Circle Movement**: Makes the turtle move in perfect circles
- **Enhanced Teleop**: Manual control with WASD keys
- **Configurable Parameters**: Adjustable speed and radius
- **Real-time Visualization**: RQT plot for movement tracking

## Package Structure

```
turtle_circle_package/
├── package.xml
├── CMakeLists.txt
├── scripts/
│   ├── turtle_circle_node.py
│   └── turtle_teleop_enhanced.py
├── launch/
│   ├── turtle_circle_demo.launch
│   └── turtle_teleop_demo.launch
└── README.md
```

## How to Run

### Prerequisites

1. Make sure ROS Noetic is running in the container
2. Build the workspace: `catkin_make`
3. Source the workspace: `source devel/setup.bash`

### Method 1: Automatic Circle Movement

1. **Start the demo**:
   ```bash
   roslaunch turtle_circle_package turtle_circle_demo.launch
   ```

2. **What you'll see**:
   - Turtlesim window with a turtle
   - Turtle automatically moving in circles
   - RQT plot showing x,y coordinates
   - Console output with position updates

3. **To stop**: Press `Ctrl+C` in the terminal

### Method 2: Manual Control (Teleop)

1. **Start the teleop demo**:
   ```bash
   roslaunch turtle_circle_package turtle_teleop_demo.launch
   ```

2. **Control the turtle**:
   - `W` - Move forward
   - `S` - Move backward
   - `A` - Turn left
   - `D` - Turn right
   - `Space` - Stop
   - `Q` - Quit
   - `R` - Reset position

### Method 3: Individual Nodes

1. **Start turtlesim**:
   ```bash
   rosrun turtlesim turtlesim_node
   ```

2. **Run the circle controller**:
   ```bash
   rosrun turtle_circle_package turtle_circle_node.py
   ```

3. **Or run teleop**:
   ```bash
   rosrun turtle_circle_package turtle_teleop_enhanced.py
   ```

## Parameters

You can modify the circle parameters in the launch file:

- `radius`: Circle radius in meters (default: 2.0)
- `linear_speed`: Forward speed in m/s (default: 1.0)
- `angular_speed`: Calculated automatically as linear_speed/radius

## Troubleshooting

1. **"Package not found"**: Make sure you've built and sourced the workspace
2. **"Permission denied"**: Make the Python scripts executable:
   ```bash
   chmod +x scripts/turtle_circle_node.py
   chmod +x scripts/turtle_teleop_enhanced.py
   ```
3. **Turtle not moving**: Check if turtlesim is running and topics are published

## VNC Access Instructions

1. **Connect via VNC**:
   - Browser: `http://localhost:8080/vnc.html`
   - VNC Client: `localhost:5900`

2. **Open terminal in VNC**:
   ```bash
   # Navigate to workspace
   cd /workspace
   
   # Build the package
   catkin_make
   
   # Source the workspace
   source devel/setup.bash
   
   # Run the demo
   roslaunch turtle_circle_package turtle_circle_demo.launch
   ```

3. **Multiple terminals**: Use `tmux` for multiple terminal sessions
   ```bash
   # Start new tmux session
   tmux new-session -d -s ros_demo
   
   # Split window
   tmux split-window -h
   
   # Run different commands in each pane
   # Pane 0: roslaunch turtle_circle_package turtle_circle_demo.launch
   # Pane 1: rostopic echo /turtle1/pose
   ```

## Code Explanation

### turtle_circle_node.py
- Publishes `Twist` messages to `/turtle1/cmd_vel`
- Subscribes to `/turtle1/pose` for position feedback
- Calculates angular velocity based on desired radius
- Runs at 10Hz control loop

### turtle_teleop_enhanced.py
- Provides manual control via keyboard
- Uses non-blocking key input
- Supports multiple movement commands
- Graceful shutdown handling

## Next Steps

1. **Modify the circle parameters** in the launch file
2. **Add more complex movements** (figure-8, square, etc.)
3. **Implement obstacle avoidance**
4. **Add multiple turtles**
5. **Create custom turtle shapes**

Enjoy watching your turtle move in circles! 🐢⭕ 