
# [Doosan Robotics](http://www.doosanrobotics.com/kr/)<img src="https://user-images.githubusercontent.com/47092672/97660147-142f1f00-1ab4-11eb-9d14-48f30a666cdc.PNG" width="10%" align="right">
[![license - apache 2.0](https://img.shields.io/:license-Apache%202.0-yellowgreen.svg)](https://opensource.org/licenses/Apache-2.0)
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
[![support level: community](https://img.shields.io/badge/support%20level-community-lightgray.png)](http://rosindustrial.org/news/2016/10/7/better-supporting-a-growing-ros-industrial-software-platform)
# *overview*
    
    This package provides the function to control all models of Doosan robots in the ROS2(Foxy) environment.
    
    ※ Currently, ROS2 related packages are being updated rapidly. 
       Doosan packages will also be updated from time to time and features will be upgraded.

# *build* 
##### *Doosan Robot ROS2 Package is implemented at ROS2-Foxy.*
    ### We assume that you have installed the ros-foxy-desktop package using the apt-get command.
    ### We recommand the /home/<user_home>/ros2_ws/src
    $ mkdir -p ~/ros2_ws/src
    $ cd ~/ros2_ws/src
    $ git clone https://github.com/doosan-robotics/doosan-robot2.git
    $ git clone https://github.com/ros-controls/ros2_control.git
    $ git clone https://github.com/ros-controls/ros2_controllers.git
    $ cd ~/ros2_ws
    $ rosdep update
    $ rosdep install --from-paths src --ignore-src --rosdistro foxy -r -y
    $ colcon build
    $ . install/setup.bash

#### dependency package list
    $ sudo apt-get install ros-foxy-control-msgs ros-foxy-realtime-tools ros-foxy-xacro ros-foxy-joint-state-publisher-gui

# *usage* <a id="chapter-3"></a>
#### Joint State Publish
It can be run independently without a controller.
Using the robot model and the `joint_state_publisher_gui` package, you can see the robot moving on Rviz.
```bash
$ ros2 launch dsr_launcher2 dsr_joint_state_pub.launch.py model:=a0912 color:=blue
``` 
<img src="https://user-images.githubusercontent.com/47092672/97652654-40da3b00-1aa2-11eb-8621-2a36e3159de0.png" width="70%">

### Virtual Mode
If you are driveing the package without a real robot, use __virtual mode__   
> (DRCF) location: doosan-robot2/common/bin/DRCF  

Run the *DRCF emulator* by entering the command below.
The *DRCF emulator* runs only with __root__ permission.
```bash
$ cd ~/ros2_ws/src/doosan-robot2/common2/bin/DRCF
$ sudo ./DRCF64
``` 
<img src="https://user-images.githubusercontent.com/47092672/97665930-f23b9980-1abf-11eb-95bd-867ce007970d.PNG" width="50%">

##### Run dsr_control2 node 
You can execute the Control Node by using the command below.
```bash
$ ros2 launch dsr_launcher2 single_robot_rviz2.launch.py model:=a0912 color:=blue
```
##### Run the example scripts
The robot can be driven by using the example scripts included in the dsr_example2 package.
Check that the controller and robot are connected normally, and enter the command below.
```bash
$ ros2 run dsr_example2_py dsr_service_motion_simple
```

### Real Mode
Use __real mode__ to drive a real robot   
The default IP of the robot controller is _192.168.127.100_ and the port is _12345_.

##### Run dsr_control2 node 
```bash
$ ros2 launch dsr_launcher2 single_robot_rviz2.launch.py mode:=real host:=192.168.127.100 port:=12345
```

##### Run the example scripts
The robot can be driven by using the example scripts included in the dsr_example2 package.
Check that the controller and robot are connected normally, and enter the command below.
```bash
$ ros2 run dsr_example2_py dsr_service_motion_simple
```
<img src="https://user-images.githubusercontent.com/47092672/97654894-3f5f4180-1aa7-11eb-83f0-90eb071d1f60.gif" width="70%">
