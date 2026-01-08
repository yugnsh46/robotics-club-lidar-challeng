# Teleoperated Differential Drive Robot with LIDAR

A CoppeliaSim-based differential drive robot with LIDAR scanning capabilities, ROS2 integration, and autonomous obstacle avoidance - built for the Robotics Club recruitment challenge.

## 🎯 Project Overview

This project implements a teleoperated differential drive robot in CoppeliaSim that can:
- Be controlled remotely via keyboard using ROS2 teleop commands
- Scan its environment using an integrated LIDAR sensor
- Automatically avoid obstacles when detected
- Publish real-time LIDAR data for visualization in RViz2

## 🛠️ Tech Stack

- **Simulation**: CoppeliaSim (Edu version 4.10)
- **Framework**: ROS2 (Jazzy)
- **Programming**: Lua (robot control scripts)
- **Visualization**: RViz2
- **Sensors**: LIDAR (270° FOV), Proximity sensor

## ✨ Features

### Core Functionality
- **Teleoperation**: Full keyboard control via `teleop_twist_keyboard` ----created node for this named = 'teleop_twist_keyboard'
- **LIDAR Integration**: Real-time 270° laser scanning with configurable range (0.1m - 10m)
- **Obstacle Avoidance**: Automatic collision avoidance using proximity sensor also tells how far obstacle is placed in coppeliasim terminal.
- **ROS2 Communication**: 
  - Publishes `/scan` (LaserScan messages)
  - Subscribes to `/cmd_vel` (Twist messages)

### Robot Specifications
- **Drive Type**: 4-wheel differential drive
- **Wheel Radius**: 0.05m
- **Wheel Base**: 0.3m
- **Max Motor Force**: 30N per wheel
- **Obstacle Detection Threshold**: 0.5m

## 📋 Prerequisites

### Software Requirements
```bash
# Operating System
Ubuntu 24.04 (or compatible)

# ROS2 Installation
ROS2 Jazzy

# CoppeliaSim
CoppeliaSim Edu (latest version 4.10)

# Required ROS2 Packages
sudo apt install ros-<jazzy>-teleop-twist-keyboard
sudo apt install ros-<jazzy>-rviz2
```

### CoppeliaSim Setup
1. Download CoppeliaSim from [official website](https://www.coppeliarobotics.com/downloads)
2. Install the ROS2 Interface plugin
3. Source your ROS2 workspace

## 🚀 Installation & Setup

### 1. Clone the Repository
```bash
git clone <[link](https://github.com/yugnsh46/robotics-club-lidar-challenge)>
cd robotics-club-challenge
```

### 2. Prepare CoppeliaSim Scene
- Open CoppeliaSim
- Load the provided `lidar_teleop_robot.ttt` scene file
- Ensure all objects are properly named:
  - `/lidar` - LIDAR sensor
  - `/joint_left`, `/joint_right` - Rear wheels
  - `/front_left`, `/front_right` - Front wheels
  - `/proximitySensor` - Obstacle detection sensor

### 3. Add Lua Script
- In CoppeliaSim, right-click on your robot model
- Select "Add > Associated child script > Non-threaded"
- Copy the contents of `robot_control.lua` into the script editor
- Save the script

### 4. Source ROS2
```bash
source /opt/ros/<jazzy>/setup.bash

```

## 🎮 Running the Project

### Terminal 1: Start CoppeliaSim Simulation
```bash
# Launch CoppeliaSim (from installation directory)
./coppeliaSim.sh

# In CoppeliaSim: File > Open Scene > [your_scene.ttt]
# Click Play button to start simulation
```

### Terminal 2: Launch RViz2
```bash
source /opt/ros/<jazzy>/setup.bash
rviz2
```

**RViz2 Configuration:**
1. Click "Add" button (bottom left)
2. Select "LaserScan"
3. Set topic to `/scan`
4. Set Fixed Frame to `lidar` from 'base_link'
5. Adjust visualization settings as needed

### Terminal 3: Teleoperation Control
```bash
source /opt/ros/<jazzy>/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Controls:**
- `i` - Move forward
- `k` - Stop
- `m` - Move backward
- `j` - Turn left
- `l` - Turn right
- `u` / `o` - Move in curves
- `q` / `z` - Increase/decrease max speeds

## 🔧 Configuration

### Adjusting Robot Parameters
Edit the following variables in `robot_control.lua`:

```lua
-- Wheel parameters (line 42-43)
wheelRadius = 0.05      -- Wheel radius in meters
wheelBase = 0.3         -- Distance between wheels

-- Obstacle avoidance (line 48-49)
obstacleThreshold = 0.5 -- Detection distance in meters
```

### LIDAR Configuration
```lua
-- Detection range (lines 18-19)
sim.setObjectFloatParam(lidar, sim.visionfloatparam_near_clipping, 0.1)
sim.setObjectFloatParam(lidar, sim.visionfloatparam_far_clipping, 10.0)

-- Field of view (line 22)
local fov = math.rad(270)  -- 270 degrees
```

## 📊 Testing & Verification

### Verify ROS2 Topics
```bash
# List active topics
ros2 topic list

# Expected output:
# /scan
# /cmd_vel

# Check LaserScan data
ros2 topic echo /scan

# Monitor message frequency
ros2 topic hz /scan
```

### Test Obstacle Avoidance
1. Drive robot towards an obstacle
2. Observe automatic avoidance behavior when within 0.5m
3. Check CoppeliaSim console for debug messages

## 🐛 Problems Encountered & Solutions

### Problem 1: Robot Moving in Wrong Direction
**Issue**: Robot moved backward when commanded forward

**Solution**: Swapped the signs in the differential drive kinematics calculation:
```lua
-- Changed from:
local leftVel = (linear - angular * wheelBase / 2) / wheelRadius
local rightVel = (linear + angular * wheelBase / 2) / wheelRadius

-- To:
local leftVel = (linear + angular * wheelBase / 2) / wheelRadius
local rightVel = (linear - angular * wheelBase / 2) / wheelRadius
```

### Problem 2: LIDAR Data Not Publishing
**Issue**: `/scan` topic not appearing in ROS2

**Cause**: ROS2 interface not properly initialized

**Solution**: 
- Verified simROS2 module was loaded
- Added error checking for publisher creation
- Ensured CoppeliaSim ROS2 plugin was installed correctly

### Problem 3: Obstacle Avoidance Conflicting with Teleop
**Issue**: Robot behavior unpredictable when both systems active

**Solution**: Implemented priority system where obstacle avoidance overrides teleop commands only when obstacle is within threshold, then returns control smoothly

### Problem 4: LIDAR Visualization Not Showing in RViz2
**Issue**: LaserScan data publishing but not visible

**Solution**:
- Set correct `frame_id` to match Fixed Frame in RViz2
- Adjusted angle_min/angle_max to match sensor FOV
- Verified transform frames were properly set up

## 📁 Project Structure

```
robotics-club-challenge/
├── README.md                 # This file
├── robot_control.lua         # Main robot control script
├── lidar_teleop_robot.ttt    # CoppeliaSim scene file
├── screenshots/              # Demo screenshots
│   ├── Coppeliastim_interface.png
│   ├── Rviz2.png
│   └── terminal_coppeliasim.png
│   └── teleopration_terminal.png
└── demo_video.mp4           # Screen recording
```

## 🎥 Demo Video Contents

## 🎥 Demo Video

📹 **[Watch Full Demo Video](https://drive.google.com/file/d/18V4yRuPQZUUVOw_VSZYWxX5uHFZsqluz/view?usp=drive_link)**

*Note: Video hosted on Google Drive due to file size*

The submitted screen recording shows:
1. **CoppeliaSim window** with robot navigating around obstacles
2. **RViz2 window** displaying real-time LIDAR point cloud --- not working ( i failed to download humble in ubuntu because of my version 24.04 --- but i connected it with rviz2 successfull)
3. **Terminal windows** showing:
   - ROS2 topics running
   - Teleop keyboard control
   - Debug output from robot script
4. **Live demonstration** of:
   - Manual teleoperation
   - Automatic obstacle avoidance
   - LIDAR scanning visualization

## 🔍 Code Highlights

### Differential Drive Kinematics
The robot uses standard differential drive equations to convert linear and angular velocities into individual wheel speeds:

```lua
function twistToDiffDrive(linear, angular)
    local leftVel = (linear + angular * wheelBase / 2) / wheelRadius
    local rightVel = (linear - angular * wheelBase / 2) / wheelRadius
    return leftVel, rightVel
end
```

### Real-time LIDAR Processing
LIDAR data is read from the vision sensor and converted to ROS2 LaserScan format:

```lua
-- Extract distances from vision sensor
local numRays = #aux
scanMsg.ranges = {}
for i = 1, numRays do
    local distance = aux[i]
    if distance < scanMsg.range_min or distance > scanMsg.range_max then
        distance = scanMsg.range_max
    end
    table.insert(scanMsg.ranges, distance)
end
```

### Obstacle Avoidance Logic
Simple reactive behavior that overrides teleop when obstacles detected:

```lua
if detected > 0 and distance < obstacleThreshold then
    local avoidLinear = -0.5   -- Back up
    local avoidAngular = 2.0   -- Turn away
    local leftVel, rightVel = twistToDiffDrive(avoidLinear, avoidAngular)
    setAllWheels(leftVel, rightVel)
end
```

## 📚 Learning Outcomes

Through this project, I gained hands-on experience with:
- **Robot Kinematics**: Understanding differential drive mechanics
- **Sensor Integration**: Working with LIDAR and proximity sensors
- **ROS2 Ecosystem**: Publishers, subscribers, and message types
- **Lua Scripting**: CoppeliaSim API and control loops
- **Real-time Systems**: Synchronizing simulation, control, and visualization
- **Debugging**: Troubleshooting sensor data and motion control issues

## 🙏 Acknowledgments

- Built with assistance from **Claude AI** (as per challenge requirements)
- got to learn new things 
- CoppeliaSim documentation and community resources
- ROS2 tutorials and documentation

## 📧 Contact

**Yugnsh Sethi**  
**yugnshsethi46@gamil.com**  --- github mail id
**ch25bt016@iitdh.ac.in** --- college mail id
**GitHub**:[yugnsh46] --- git username
**robotics-club-lidar-challenge.git** --- [github link](https://github.com/yugnsh46/robotics-club-lidar-challenge.git)

---

**My pledge** -- as i am begineer in this dont have much knowledge used claude AI honestly but I pledge to be a responsible and dedicated member of the Robotics Club.
I will strive to learn, innovate, and apply my knowledge of robotics and technology with honesty and enthusiasm.
I will respect my teammates, mentors, and equipment, and work collaboratively to achieve our goals.
I promise to uphold the values of discipline, creativity, and perseverance, and to represent the Robotics Club with integrity and pride.
I will use my skills for the betterment of society and continue to grow as a learner and engineer.

---

*Submitted for Robotics Club Recruitment Challenge*  
*Contact: Shreyas Reddy — ec24bt033@iitdh.ac.in*