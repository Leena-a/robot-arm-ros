# robot-arm-ros
Installation steps of ROS package that can be used to plan and execute motion trajectories for a robot arm in simulation and real life.

The steps below uses ROS melodic, 1.14.11 version which runs on ubuntu 18.04.5.


## Installing ROS melodic
> The following steps assumes that the user have Ubuntu 18.04 installed.


Enable restricted, universe, and multiverse in Ubuntu:
``` 
$ sudo add-apt-repository restricted
$ sudo add-apt-repository universe
$ sudo add-apt-repository multiverse
```

Setup the system to accept software from packages.ros.org.
```
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

Setup the keys:
```
$ curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```

Before the installation, check whether the Debian package index is up-to-date:
```
$ sudo apt update
```

Desktop-Full Installation command:
```
$ sudo apt install ros-melodic-desktop-full
```

To automatically add ROS environment variables to your bash session every time a new shell is launched:
```
$ echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
```

Install and initialize system dependencies in ROS:
```
$ sudo apt install python-rosdep
$ sudo rosdep init
$ rosdep update
```

To test your installation, run:
```
$ roscore
```
> `roscore` is a collection of nodes and programs that are pre-requisites of a ROS-based system. You must have a roscore running in order for ROS nodes to communicate. 

## Preparing ROS
Setup the workspace which is where ROS projects are built and stored:
```
$ mkdir -p ~/robot-arm/src
$ cd ~/robot-arm/
$ catkin_make
```
Where `robot-arm` is the name of the workspace, and `catkin_make` is used to the build the project and packages inside the source folder.

## Robot arm package  
Adding `arduino_robot_arm` package to `src` folder:
```
$ cd ~/robot-arm/src
$ sudo apt install git
$ git clone https://github.com/smart-methods/arduino_robot_arm 
```

Dependencies 
```
$ cd ~/robot-arm
$ rosdep install --from-paths src --ignore-src -r -y
$ sudo apt-get install ros-melodic-moveit
$ sudo apt-get install ros-melodic-joint-state-publisher ros-melodic-joint-state-publisher-gui
$ sudo apt-get install ros-melodic-gazebo-ros-control joint-state-publisher
$ sudo apt-get install ros-melodic-ros-controllers ros-melodic-ros-control
```
Compilation 
```
$ catkin_make
```

Controlling the robot arm using joint_state_publisher
```
$ roslaunch robot_arm_pkg check_motors.launch
```
![image](https://user-images.githubusercontent.com/52850659/123440485-82909680-d5db-11eb-9d8b-6be8da7ef07d.png)

The package displays the joint positions which are base_joint, shoulder, elbow, and wrist in a window as sliders. Each slider is set to the joints' min and max limits, except for continuous joints. 


