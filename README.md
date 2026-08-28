 ROS 2 Gazebo Line Follower

A camera-based autonomous line-following vehicle simulated using **ROS 2 Jazzy**, **Gazebo Sim**, and **OpenCV**.

The project implements a PID-based steering controller with adaptive speed control, dynamic look-ahead, sharp-turn anticipation, steering smoothing, and automatic line-loss recovery.

---

## 📌 Overview

This project simulates an autonomous vehicle that follows a black track using only a front-mounted camera.

The camera captures the track in Gazebo and publishes image data through ROS 2. The controller processes the image using OpenCV, determines the position of the line relative to the vehicle, and uses a PID controller to generate steering commands.

The vehicle's speed is dynamically adjusted according to the detected line position and upcoming turns.

### Control Pipeline

```text
                  Gazebo Sim
                      │
                      ▼
              Front Camera Sensor
                      │
                      ▼
                /front_camera
                      │
                      ▼
             OpenCV Image Processing
                      │
                      ▼
                Line Detection
                      │
                      ▼
              Position / Error
                      │
                      ▼
               PID Controller
                      │
              ┌───────┴───────┐
              ▼               ▼
         Steering          Speed Control
              │               │
              └───────┬───────┘
                      ▼
                   /cmd_vel
                      │
                      ▼
                 Gazebo Vehicle
🚀 Features
Camera-based line detection
PID steering control
Proportional and derivative control
Dynamic look-ahead horizon
Adaptive vehicle speed
Sharp-turn anticipation
Steering command smoothing
Automatic line-loss detection
Recovery scanning when the line is lost
60 Hz camera update rate
Gazebo-based custom track environment
ROS 2 topic-based communication
🧠 How It Works
1. Camera Input

The vehicle uses a front-mounted camera in Gazebo.

The camera publishes image data to:

/front_camera

The camera is configured with:

Resolution: 640 × 480
Horizontal FOV: 120°
Update rate: 60 Hz
2. Image Processing

The incoming image is converted from BGR to grayscale.

A binary inverse threshold is then applied to isolate the black track from the lighter ground surface.

gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
_, binary = cv2.threshold(gray, 60, 255, cv2.THRESH_BINARY_INV)

The controller examines specific horizontal rows of the image to determine where the track is located.

3. Line Position and Tracking Error

The center of the detected line is calculated from the detected pixels.

The tracking error is:

Error = Camera Center - Line Center

A positive or negative error indicates that the vehicle needs to steer in the corresponding direction.

🎛️ PID Steering Control

The controller uses proportional and derivative terms to calculate the steering command.

The implemented controller follows:

u(t) = Kp × error + Kd × derivative

where:

derivative = current_error - previous_error

Current controller parameters:

Parameter	Value
Kp	0.0048
Ki	0.0000
Kd	0.0055

The integral term is currently disabled to avoid accumulated error causing unwanted steering lag.

The resulting angular velocity is limited to:

-3.2 ≤ angular velocity ≤ 3.2
👀 Dynamic Look-Ahead

Instead of relying only on the part of the track directly in front of the vehicle, the controller also examines a higher point in the camera image.

The look-ahead position changes according to the current vehicle speed.

Higher speed
     ↓
Look further ahead
     ↓
Detect upcoming turns earlier
     ↓
Reduce speed / increase steering response

This helps the vehicle anticipate sharp turns before they reach the immediate tracking region.

⚡ Adaptive Speed Control

Vehicle speed is dynamically adjusted based on the tracking error.

When the line is close to the center of the camera:

Small error → Higher speed

When the vehicle moves away from the line:

Large error → Lower speed

The configured speed limits are:

Parameter	Value
Maximum speed	1.5 m/s
Minimum speed	0.20 m/s
Speed increment	0.030 m/s
Brake deceleration	0.065 m/s per control step

When an upcoming sharp turn is detected, the controller further reduces the target speed.

🔄 Line-Loss Recovery

If the controller cannot detect the line in the immediate tracking region, it switches to a recovery state.

Line detected
     │
     ▼
Normal tracking
     │
     │ Line lost
     ▼
Recovery mode
     │
     ▼
Reduce speed
     │
     ▼
Turn toward last known direction
     │
     ▼
Re-acquire line
     │
     ▼
Resume tracking

The previous tracking error is used to determine the direction of the recovery turn.

🌎 Gazebo Simulation Environment

The project includes a custom Gazebo world containing a closed-loop track with multiple straight sections, corners, and chicanes.

The track includes:

Long straight sections
45° turns
90° turns
S-curve sections
Chicanes
Multiple changes in direction

This provides a more challenging environment for testing the controller than a simple straight or oval track.

🤖 Simulated Vehicle

The simulated vehicle consists of:

Box-based chassis
Front-mounted tracking camera
Velocity control system
ROS 2 /cmd_vel interface

The Gazebo velocity controller receives commands from the ROS 2 node through:

/cmd_vel
🛠️ Technologies Used
Technology	Purpose
ROS 2 Jazzy	Robot middleware and communication
Gazebo Sim	Vehicle and environment simulation
Python	Controller implementation
OpenCV	Image processing and line detection
NumPy	Numerical processing
cv_bridge	ROS image ↔ OpenCV conversion
PID Control	Steering control
📋 Requirements

Before running the project, make sure you have:

Ubuntu / WSL2
ROS 2 Jazzy
Gazebo Sim compatible with ROS 2 Jazzy
Python 3
OpenCV
NumPy
cv_bridge

The ROS 2 Python packages can be installed using:

sudo apt install ros-jazzy-cv-bridge

Python dependencies:

pip install opencv-python numpy
📥 Installation

Clone the repository:

git clone https://github.com/29-Harsh06/line-follower-ros2-gazebo.git

Enter the project directory:

cd line-follower-ros2-gazebo

Source ROS 2 Jazzy:

source /opt/ros/jazzy/setup.bash
▶️ Running the Simulation
1. Start Gazebo

From the project directory:

gz sim line_follower.world

Wait for Gazebo to finish loading the simulation.

2. Start the Line Follower Controller

Open a second terminal.

Source ROS 2 Jazzy:

source /opt/ros/jazzy/setup.bash

Navigate to the project directory:

cd line-follower-ros2-gazebo

Run the controller:

python3 Code2.py

The controller will subscribe to:

/front_camera

and publish velocity commands to:

/cmd_vel
📡 ROS 2 Topics
Camera
/front_camera

Message type:

sensor_msgs/msg/Image

The camera provides the image used for line detection.

Velocity Command
/cmd_vel

Message type:

geometry_msgs/msg/Twist

The controller publishes:

linear.x → vehicle speed
angular.z → steering / rotational velocity
📁 Project Structure
line-follower-ros2-gazebo/
│
├── Code2.py
├── line_follower.world
├── README.md
├── LICENSE
└── .gitignore
Files

Code2.py

Main ROS 2 line-following controller containing:

Camera subscription
OpenCV processing
Line detection
PID controller
Adaptive speed control
Turn anticipation
Recovery logic

line_follower.world

Custom Gazebo simulation environment containing:

Ground plane
Closed-loop track
Simulated vehicle
Front camera
Velocity control plugin

.gitignore

Prevents development files such as the unused local controller file from being committed to the repository.

📊 Controller Logic

The overall control loop can be summarized as:

Camera Frame
     │
     ▼
Grayscale Conversion
     │
     ▼
Binary Thresholding
     │
     ▼
Track Pixel Detection
     │
     ├───────────────┐
     ▼               ▼
Immediate Row    Look-Ahead Row
     │               │
     ▼               ▼
Tracking Error   Turn Detection
     │               │
     └───────┬───────┘
             ▼
        PID Steering
             │
             ▼
       Speed Scaling
             │
             ▼
        /cmd_vel
             │
             ▼
       Vehicle Motion
🔧 Future Improvements

Potential improvements for future versions include:

Converting the project into a standard ROS 2 Python package
Adding a dedicated ROS 2 launch file
Moving PID parameters into a YAML configuration
Adding dynamic parameter tuning
Improving line segmentation using HSV/color-space processing
Adding quantitative performance metrics such as lap time and tracking error
Recording and plotting controller performance
Testing different PID parameter sets automatically
Adding more complex track geometries
Integrating the controller with a physical line-following robot
🎯 Project Goal

The primary goal of this project is to explore closed-loop autonomous vehicle control using camera-based perception and PID feedback in a simulated environment.

The project demonstrates how sensor data can be processed in real time and converted into control commands for autonomous navigation.

📜 License

This project is licensed under the MIT License. See the LICENSE file for details.
