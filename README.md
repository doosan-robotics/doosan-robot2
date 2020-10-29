
# [Doosan Robotics](http://www.doosanrobotics.com/kr/)
[![license - apache 2.0](https://img.shields.io/:license-Apache%202.0-yellowgreen.svg)](https://opensource.org/licenses/Apache-2.0)
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
[![support level: community](https://img.shields.io/badge/support%20level-community-lightgray.png)](http://rosindustrial.org/news/2016/10/7/better-supporting-a-growing-ros-industrial-software-platform)
# *overview*
    It currently contains packages that provide nodes for communication with Doosan's collaborative robot controllers, URDF models for various robot arms
__Not Yet__

# *build* 
##### *Doosan Robot ROS2 Package is implemented at ROS2-Foxy.*
    ### We assume that you have installed the ros-foxy-desktop package using the apt-get command.
    ### We recommand the /home/<user_home>/ros2_ws/src
    cd ~/ros2_ws/src
    git clone https://github.com/doosan-robotics/doosan-robot2.git
    git clone https://github.com/ros-controls/ros2_control.git
    cd ros2_control/
    git reset --hard b9ea092b47a874e51cf73dfdee9b1aa0c2b8afa8
    cd ~/ros2_ws/src
    git clone https://github.com/ros-controls/ros2_controllers.git
    cd ros2_controllers/
    git reset --hard d52902538a970e792ecc604416e22ef90a57f29e
    cd ~/ros2_ws
    rosdep update
    rosdep install --from-paths src --ignore-src --rosdistro foxy -r -y
    colcon build
    . install/setup.bash

#### package list
    sudo apt-get install ros-foxy-control-msgs ros-foxy-realtime-tools ros-foxy-xacro ros-foxy-joint-state-publisher-gui

# *usage* <a id="chapter-3"></a>
#### Operation Mode
##### Virtual Mode
If you are driveing the package without a real robot, use __virtual mode__   
> (DRCF) location: doosan-robot2/common/bin/ DRCF

Run the DRCF emulator by entering the command below.
The DRCF emulator runs only with __root__ permission.
```bash
cd ~/ros2_ws/doosan-robot2/common/bin
./DRCF64
``` 
You can execute the Control Node by using the command below.
```bash
ros2 launch dsr_launcher2 single_robot_rviz.launch.py model:=a0912 color:=blue
```
<img src="https://user-images.githubusercontent.com/47092672/97539997-4d10ba80-1a06-11eb-9aba-db0e6e9f65d2.png" width="70%">

##### Real Mode
Use __real mode__ to drive a real robot   
The default IP of the robot controller is _192.168.127.100_ and the port is _12345_.
```bash
ros2 launch dsr_launcher2 single_robot_rviz.launch.py mode:=real host:=192.168.127.100 port:=12345
