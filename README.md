# [Doosan Robotics](http://www.doosanrobotics.com/kr/)
[![license - apache 2.0](https://img.shields.io/:license-Apache%202.0-yellowgreen.svg)](https://opensource.org/licenses/Apache-2.0)
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
[![support level: community](https://img.shields.io/badge/support%20level-community-lightgray.png)](http://rosindustrial.org/news/2016/10/7/better-supporting-a-growing-ros-industrial-software-platform)
# *overview*

__Not Yet__

# *build* 
##### *Doosan Robot ROS2 Package is implemented at ROS2-Foxy.*
    ### We recommand the /home/<user_home>/ros2_ws/src
    cd ~/ros2_ws/src
    git clone https://github.com/doosan-robotics/doosan-robot2
    rosdep install --from-paths doosan-robot --ignore-src --rosdistro foxy -r -y
    cd ~/ros2_ws
    colcon build
    . install/setup.bash

#### package list
    sudo apt-get install ros-kinetic-rqt* ros-kinetic-moveit* ros-kinetic-industrial-core ros-kinetic-gazebo-ros-control ros-kinetic-joint-state-controller ros-kinetic-effort-controllers ros-kinetic-position-controllers ros-kinetic-ros-controllers ros-kinetic-ros-control ros-kinetic-serial
    
__packages for mobile robot__

    sudo apt-get install ros-kinetic-lms1xx ros-kinetic-interactive-marker-twist-server ros-kinetic-twist-mux ros-kinetic-imu-tools ros-kinetic-controller-manager ros-kinetic-robot-localization


# *usage* <a id="chapter-3"></a>
#### Operation Mode
##### Virtual Mode
If you are driveing without a real robot, use __virtual mode__   
When ROS launches in virtual mode, the emulator(DRCF) runs automatically.
> (DRCF) location: doosan-robot/common/bin/ DRCF

```bash
roslaunch dsr_launcher single_robot_gazebo.launch mode:=virtual
```
_One emulator is required for each robot_

##### Real Mode
Use __real mode__ to drive a real robot   
The default IP of the robot controller is _192.168.127.100_ and the port is _12345_.
```bash
roslaunch dsr_launcher single_robot_gazebo.launch mode:=real host:=192.168.127.100 port:=12345
