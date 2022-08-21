# Ground Robot "Cali" (version 2021-22), Robotics Laboratory, California State University, Los Angeles
How to edit markdown files - https://www.markdownguide.org/cheat-sheet

# Table of Contents
 - [About](#about)
 - [Packages](#packages)
 - [Installation instructions](#installation-instructions)
 - [System at a glance](#system-at-a-glance)
 - [Simulation](#simulation)
   - [Making a Gazebo World](#making-a-gazebo-world)
   - [Autonomous Navigation](#autonomous-navigation)
   - [Manipulation](#manipulation)
   - [Perception](#perception)
   - [Mission Planner](#mission-planner)
 - [Real Robot](#real-robot)


# About
Robot "Cali" is an open-source robotic platform for research and education. The robot is made of off-the-shelf components and integrates many open-source software packages. The robot has a embodided construct and a digital twin, allowing researchers and educators to develop and test their algorithms in both a simulated enviornment and the real world.  
<img src="https://github.com/CSULA-URC/2021-22/blob/main/doc/Rover%202.JPG" width="600" />
<img src="https://github.com/CSULA-URC/2021-22/blob/main/doc/rovergif1.gif" width="600" />
<img src="https://github.com/CSULA-URC/2021-22/blob/main/doc/roboticarmgif3.gif" width="600" />

# Packages
## rover_autonav
Launch Autonomous Navigation

## manipulation_pkg
Contains manipulator URDF and configuration files for the arm

## cali_moveit_config
Contains Moveit configuration files

## perception_pkg
Contains Perception Pipeline

# Installation instructions
The software is based on ROS-Melodic on Ubuntu 18.04. 

Open up a terminal on your linux machine and execute following:
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/CSULA-URC/2021-22.git 
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
```


# System at a glance
- hardware - four wheel drive platform, one robot arm (scorbot-er3u)  
- software - Ubuntu 18.04, ROS, and packages (Rviz, Gazebo, opencv, ...)  
- computer - Jetson TX2
- sensors - IMU, GPS, Lidar, Antenna, Transceiver, Camera, Encoders

The mechanical systems encompasses the chassis, suspension system, and drive system. The chassis consists of 1"x1" galvanized steel bolt together tubing for modularity. The suspension system consists of an independent radius arm setup with air shock absorbers for off road capabilities. The drive systems consists of 12V motors with 9" beach wheels for sandy and rocky terrain.

The electrical systems encompasses power, sensing, controls, and communications. A 12V 50ah li-on battery is used to power all electronics on the rover with buck and boost converters for voltage differences. Sensing comprises of a 3-D camera, 2-D lidar, absolute encoders, an IMU, and a gps all used to sense and navigate its surroundings. The control system uses microcontrollers, microprocessors, and motor drivers to operate and drive the robot. The communication system uses an omni directional antenna connected through a network system via router for wireless teleoperation.


# Simulation
  ## Making a Gazebo World
  To launch an empty world in gazebo:
   - open a terminal 
   - type gazebo
   - press the key enter, this will open the Gazebo application with an empty world environment 

![Gazebo Instructions 1](https://github.com/CSULA-URC/2021-22/blob/main/doc/gazebo_instruct1.jpg)
  
Next on the top left, access the edit category and select Building Editor
  
![Gazebo Instructions 2](https://github.com/CSULA-URC/2021-22/blob/main/doc/gazebo_instruct2.jpg)
  
This should show below: 
 
![Gazebo Instructions 3](https://github.com/CSULA-URC/2021-22/blob/main/doc/gazebo_instruct3.jpg)

NOTE: This tutorial assumes the user has a “blueprint” of the model. If not, then the user can customize to their needs as shown on the left.  

Using a blueprint image of a map, the user can import it to show on the white square grid above. 
Note that importing an image is only supported by the following formats: .jpg and .png

![Gazebo Instructions 4](https://github.com/CSULA-URC/2021-22/blob/main/doc/gazebo_instruct4.jpg)

Once imported, a scale of the map is needed for proportional consistency of the real-world to the simulations. 

![Gazebo Instructions 5](https://github.com/CSULA-URC/2021-22/blob/main/doc/gazebo_instruct5.jpg)
![Gazebo Instructions 6](https://github.com/CSULA-URC/2021-22/blob/main/doc/gazebo_instruct6.jpg)

Now, simply trace the obstacles on the white grid with the wall option. This represents the areas where laser scans would bounce to emit back data. 

Once the map environment is traced with walls, the user has an option to decorate the barriers with color or visual textures such as a brick wall. 

![Gazebo Instructions 7](https://github.com/CSULA-URC/2021-22/blob/main/doc/gazebo_instruct7.jpg)
![Gazebo Instructions 8](https://github.com/CSULA-URC/2021-22/blob/main/doc/gazebo_instruct8.jpg)
![Gazebo Instructions 9](https://github.com/CSULA-URC/2021-22/blob/main/doc/gazebo_instruct9.jpg)
![Gazebo Instructions 10](https://github.com/CSULA-URC/2021-22/blob/main/doc/gazebo_instruct10.jpg)

To save the model, access the file category on the top left and provide a name of choice including the directory

Note: this only saves the environment as a model and not a world. So be careful to leave the pop up window as the model cannot be edited once exiting. 

This saves the model in the building editor folder directory but can be customized to save in whichever folder desired

![Gazebo Instructions 11](https://github.com/CSULA-URC/2021-22/blob/main/doc/gazebo_instruct11.jpg)

To include items such as a coke can, access the insert category and from the huge number of options, place items by dragging the option to the world environment. 

![Gazebo Instructions 12](https://github.com/CSULA-URC/2021-22/blob/main/doc/gazebo_instruct12.jpg)

To save a world, access the file category and select save world as 
save the end of your file as .world 

![Gazebo Instructions 13](https://github.com/CSULA-URC/2021-22/blob/main/doc/gazebo_instruct13.jpg)
![Gazebo Instructions 14](https://github.com/CSULA-URC/2021-22/blob/main/doc/gazebo_instruct14.jpg)

## Autonomous Navigation
  [Add description]
  ### 1) Mapping
  [Add description]
  ### 2) Localization
  [Add description]
  ### 3) Path Planning
  [Add description]
  ### 4) Launch Autonomous Navigation node

  Open a new terminal:
  ```bash
roslaunch rover_autonav navigation_v2.launch
```

Then in RViz we just need to select a goal pose using the <em><strong>2D Nav Goal</strong></em> tool:

  <img src="https://github.com/CSULA-URC/2021-22/blob/main/doc/autonomous_navigation.gif" width="600" />


  ## Manipulation
  First, we do some testings using <em><strong>rqt_joint_trajectory_controller</strong></em> to control the arm joints and also the <em><strong>/gripper_controller/gripper_cmd/goal</strong></em> topic to control the gripper.
  
  Then, we perform manipulation using <strong>MoveIt</strong>.

  ### 1) rqt_joint_trajectory_controller & <em>/gripper_controller/gripper_cmd/goal</em> topic
  - <em><strong>rqt_joint_trajectory_controller</strong></em>

  Open a new terminal:

  ```bash
  roslaunch rover_autonav arm_fixed_to_ground.launch 
  ```

  Now play with the <em><strong>rqt_joint_trajectory_controller</strong></em> GUI.
  
  <img src="https://github.com/CSULA-URC/2021-22/blob/main/doc/rqt_joint_trajectory_controller.gif" width="600" />

  -  <em><strong>/gripper_controller/gripper_cmd/goal</strong></em> topic

  Now open a second terminal:
  ```bash
  rostopic pub /gripper_controller/gripper_cmd/goal control_msgs/GripperCommandActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  command:
    position: 2.0
    max_effort: 0.0"
  ```

  Set a position of <em>2.0</em> to open the gripper and <em>-2.0</em> to close it.
  



  <img src="https://github.com/CSULA-URC/2021-22/blob/main/doc/open_close_gripper.gif" width="600" />



  ### 2) MoveIt

  Close all previous terminals, then open a new terminal:
  ```bash
roslaunch rover_autonav gazebo_combined_v5.launch
```
  In a second terminal:
  ```bash
roslaunch cali_moveit_config cali_planning_execution.launch
```

  - arm

  <img src="https://github.com/CSULA-URC/2021-22/blob/main/doc/moveit_arm.gif" width="600" />

  - gripper

  <img src="https://github.com/CSULA-URC/2021-22/blob/main/doc/moveit_gripper.gif" width="600" />




## Perception

In order too perceive Cali's surroundings, an Intel Realsense d435 3D camera is used and placed on top of the last link of the arm. The data will then be used in ROS via a topic.

Before launching the perception pipeline the camera needs to be <strong>correctly oriented</strong>.
Thus, we have created a perception pose in Moveit for that matter.

In a third terminal:
  ```bash
roslaunch perception surface_detection_simple.launch
```

<img src="https://github.com/CSULA-URC/2021-22/blob/main/doc/perception.gif" width="600" />

As we can see from the GIF we first obtain the <strong>PointCloud</strong>:

<img src="https://github.com/CSULA-URC/2021-22/blob/main/doc/perception_point_cloud.png" width="600" />


Then with our <strong>surface_detection</strong> algorithm we can detect a surface (purple marker) and an object (green marker) on top of that surface:

<img src="https://github.com/CSULA-URC/2021-22/blob/main/doc/surface_detection.png" width="600" />

From the <strong>surface_detection</strong> we get the <em><strong>/surface_objects</strong></em> topic:

```bash
rostopic echo /surface_objects
```
<img src="https://github.com/CSULA-URC/2021-22/blob/main/doc/echo_surface_objects_v2.png" width="300" />

This gives information about the surfaces and objects detected such as the geometry, postion, orientation or even the color of the object.

## Mission Planner
[Add description]
[Add diagram]

# Real Robot
## Operating the Robotic Arm

Directories: 
ScorboTesting/src/scorbot_movelt/launch
-	ScorboTesting is the name of the workspace 
-	Scorbot_moveit is the name of the package 

Steps: 
1.	Arduino to Jetson 
Open the Arduino Application Software
Verify and upload the code file: R.A.6MotorPID

![Robotic Arm Instructions 1](https://github.com/CSULA-URC/2021-22/blob/main/doc/roboticarm1.jpg)

2.	Rosserial 

Connects ROS to Arduino using rosserial
```bash
rosrun rosserial_python serial_node.py_port:=/dev/ttyACM0_baud:=115200
```

3.	Open a new terminal 

Go to ScorboTesting 

```bash
source devel/setup.bash
roslaunch scorbot_moveit demo.launch 
```

Before moving the robotic arm through simulation, make sure to manually position the arm close to #^o and the end effector slightly pointing up. An example of this is shown below: 

![Robotic Arm Instructions 2](https://github.com/CSULA-URC/2021-22/blob/main/doc/roboticarm2.jpg) ![Robotic Arm Instructions 3](https://github.com/CSULA-URC/2021-22/blob/main/doc/roboticarm3.jpg)

![Robotic Arm Instructions 4](https://github.com/CSULA-URC/2021-22/blob/main/doc/roboticarm4.jpg)

Motion the ball located in between the gears of the end-effector

![Robotic Arm Instructions 5](https://github.com/CSULA-URC/2021-22/blob/main/doc/roboticarm5.jpg) ![Robotic Arm Instructions 5](https://github.com/CSULA-URC/2021-22/blob/main/doc/roboticarm5.jpg)

Planning and executing poses are now permissible 