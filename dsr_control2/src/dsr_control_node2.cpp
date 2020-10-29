/*
 * dsr_control_node2 
 * Author: Kab Kyoum Kim (kabkyoum.kim@doosan.com)
 *
 * Copyright (c) 2020 Doosan Robotics
 * Use of this source code is governed by the BSD, see LICENSE
*/

#include <signal.h>
#include "rclcpp/rclcpp.hpp"  //ROS2

#include <memory>

#include "dsr_control2/dsr_hw_interface2.h"
#include <controller_manager/controller_manager.hpp>

#include <boost/thread/thread.hpp>

using namespace dsr_control2;

rclcpp::Node::SharedPtr g_node = nullptr;

int g_nKill_dsr_control2 = false; 
void SigHandler(int sig)
{
    // Do some custom action.
    // For example, publish a stop message to some other nodes.
  
    // All the default sigint handler does is call shutdown()
    RCLCPP_INFO(rclcpp::get_logger("dsr_control_node2"), "shutdown time! (sig = %d)",sig);
    
    g_nKill_dsr_control2 = true;

    rclcpp::shutdown();
}

void spin(std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> exe)
{
  exe->spin();
}

int thread_robot_control(rclcpp::Node::SharedPtr g_node, int nPubRate)
{
    auto pArm = std::make_shared<DRHWInterface>(g_node);

    if(pArm->init() != hardware_interface::return_type::OK){    
        RCLCPP_ERROR(rclcpp::get_logger("dsr_control_node2"),"Error initializing robot");
        return -1;
    }

    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    // start the controller manager with the robot hardware
    controller_manager::ControllerManager cm(pArm, executor);

    // load the joint state controller.
    // "ros_controllers" is the resource index from where to look for controllers
    // "ros_controllers::JointStateController" is the class we want to load
    // "my_robot_joint_state_controller" is the name for the node to spawn
    cm.load_controller(
        "dsr_joint_publisher", //"my_robot_joint_state_controller",
        "joint_state_controller/JointStateController");

    // load the trajectory controller
    ///cm.load_controller( 
    ///    "dsr_joint_trajectory_controller", //"my_robot_joint_trajectory_controller",
    ///    "joint_trajectory_controller/JointTrajectoryController");

    // there is no async spinner in ROS 2, so we have to put the spin() in its own thread
    auto future_handle = std::async(std::launch::async, spin, executor);
  
    // we can either configure each controller individually through its services
    // or we use the controller manager to configure every loaded controller
    if (cm.configure() != controller_interface::return_type::SUCCESS) {
      RCLCPP_ERROR(rclcpp::get_logger("dsr_control_node2"), "at least one controller failed to configure");
      return -1;
    }
    // and activate all controller
    if (cm.activate() != controller_interface::return_type::SUCCESS) {
      RCLCPP_ERROR(rclcpp::get_logger("dsr_control_node2"), "at least one controller failed to activate");
      return -1;
    }

    RCLCPP_INFO(rclcpp::get_logger("dsr_control_node2"),"controller_manager is updating!");

    rclcpp::Rate r(nPubRate);

    while(rclcpp::ok())
    {
        if(pArm) pArm->read();
        cm.update();
        if(pArm) pArm->write();

        ///RCLCPP_INFO(rclcpp::get_logger("dsr_control_node2"), "thread_robot_control running...");
        r.sleep();
    }

    RCLCPP_INFO(rclcpp::get_logger("dsr_control_node2"),"thread_robot_control Good-bye!");
    return 0;
}

int main(int argc, char **argv)
{
    //----- init ROS2 ---------------------- 

    rclcpp::init(argc, argv);
    g_node = rclcpp::Node::make_shared("dsr_control_node2");

    // Logger
    ///const rclcpp::Logger logger = rclcpp::get_logger("dsr_control_node2");

/////////////////////////////////////////////////////////////////////////////////////////////////////
    ///for (int i = 0; i < argc; ++i) 
    ///    RCLCPP_INFO(rclcpp::get_logger("dsr_control_node2"),">>>>> argv[%d] = %s",i,argv[i]);

    //----- SET PARA form launch file, if not set form config/default.yaml file -------------------------- 
    g_node->declare_parameter("name", "dsr01");
    g_node->declare_parameter("rate", 100);
    g_node->declare_parameter("standby", 5000);
    g_node->declare_parameter("command", true);
    g_node->declare_parameter("host", "127.0.0.1");
    g_node->declare_parameter("port", 12345);
    g_node->declare_parameter("mode", "virtual");
    g_node->declare_parameter("model", "m1013");
    g_node->declare_parameter("gripper", "none");
    g_node->declare_parameter("mobile", "none");
/////////////////////////////////////////////////////////////////////////////////////////////////////

    RCLCPP_INFO(rclcpp::get_logger("dsr_control_node2"), "g_node = 0x%p",g_node);

    signal(SIGINT, SigHandler);

    //----- get param ---------------------
    int n_rate = 100; //hz
    g_node->get_parameter("rate", n_rate);

    RCLCPP_INFO(rclcpp::get_logger("dsr_control_node2"),"rate is %d", n_rate);

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    boost::thread th_robot_ctl = boost::thread( boost::bind(&thread_robot_control, g_node, n_rate/*hz*/) );  
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    RCLCPP_INFO(rclcpp::get_logger("dsr_control_node2"),"controller_manager is updating!");

    while(rclcpp::ok() && (false==g_nKill_dsr_control2))
    {
        ///rclcpp::spin_some(g_node);
        rclcpp::spin(g_node);
    }

    th_robot_ctl.join(); 

    RCLCPP_INFO(rclcpp::get_logger("dsr_control_node2"),"Good-bye!");

    return 0;
}