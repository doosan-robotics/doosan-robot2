/*********************************************************************
 *
 *  Inferfaces for doosan robot controllor 
 * Author: Kab Kyoum Kim (kabkyoum.kim@doosan.com)
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, Doosan Robotics
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Georgia Institute of Technology nor the names of
 *     its contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
#define _DEBUG_DSR_CTL      1

#ifndef DR_HW_INTERFACE_H
#define DR_HW_INTERFACE_H

//ROS2 #include <ros/ros.h>
#include "rclcpp/rclcpp.hpp"

#include <boost/thread/thread.hpp>
#include <array>
#include <algorithm>  // std::copy

#include <hardware_interface/joint_command_handle.hpp>
#include <hardware_interface/joint_state_handle.hpp>

#include <hardware_interface/robot_hardware.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>  //add

#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/empty.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

// msg
#include "dsr_msgs2/msg/robot_error.hpp"
#include "dsr_msgs2/msg/robot_state.hpp"
#include "dsr_msgs2/msg/robot_stop.hpp"
#include "dsr_msgs2/msg/jog_multi_axis.hpp"



// service
//system
//#include <dsr_msgs2/SetRobotMode.h>
#include "dsr_msgs2/srv/set_robot_mode.hpp"
#include "dsr_msgs2/srv/get_robot_mode.hpp"
#include "dsr_msgs2/srv/set_robot_system.hpp"
#include "dsr_msgs2/srv/get_robot_system.hpp"
#include "dsr_msgs2/srv/get_robot_state.hpp"
#include "dsr_msgs2/srv/set_robot_speed_mode.hpp"
#include "dsr_msgs2/srv/get_robot_speed_mode.hpp"
#include "dsr_msgs2/srv/get_current_pose.hpp"
#include "dsr_msgs2/srv/set_safe_stop_reset_type.hpp"
#include "dsr_msgs2/srv/get_last_alarm.hpp"

// motion
#include "dsr_msgs2/srv/move_joint.hpp"
#include "dsr_msgs2/srv/move_line.hpp"
#include "dsr_msgs2/srv/move_jointx.hpp"
#include "dsr_msgs2/srv/move_circle.hpp"
#include "dsr_msgs2/srv/move_spline_joint.hpp"
#include "dsr_msgs2/srv/move_spline_task.hpp"
#include "dsr_msgs2/srv/move_blending.hpp"
#include "dsr_msgs2/srv/move_spiral.hpp"
#include "dsr_msgs2/srv/move_periodic.hpp"
#include "dsr_msgs2/srv/move_wait.hpp"
#include "dsr_msgs2/srv/jog.hpp"
#include "dsr_msgs2/srv/jog_multi.hpp"
#include "dsr_msgs2/srv/move_pause.hpp"
#include "dsr_msgs2/srv/move_stop.hpp"
#include "dsr_msgs2/srv/move_resume.hpp"
#include "dsr_msgs2/srv/trans.hpp"
#include "dsr_msgs2/srv/fkin.hpp"
#include "dsr_msgs2/srv/ikin.hpp"
#include "dsr_msgs2/srv/set_ref_coord.hpp"
#include "dsr_msgs2/srv/move_home.hpp"
#include "dsr_msgs2/srv/check_motion.hpp"
#include "dsr_msgs2/srv/change_operation_speed.hpp"
#include "dsr_msgs2/srv/enable_alter_motion.hpp"
#include "dsr_msgs2/srv/alter_motion.hpp"
#include "dsr_msgs2/srv/disable_alter_motion.hpp"
#include "dsr_msgs2/srv/set_singularity_handling.hpp"

//----- auxiliary_control
#include "dsr_msgs2/srv/get_control_mode.hpp"          
#include "dsr_msgs2/srv/get_control_space.hpp"         
#include "dsr_msgs2/srv/get_current_posj.hpp"          
#include "dsr_msgs2/srv/get_current_velj.hpp"          
#include "dsr_msgs2/srv/get_desired_posj.hpp"
#include "dsr_msgs2/srv/get_desired_velj.hpp"          
#include "dsr_msgs2/srv/get_current_posx.hpp"          
#include "dsr_msgs2/srv/get_current_tool_flange_posx.hpp"
#include "dsr_msgs2/srv/get_current_velx.hpp"          
#include "dsr_msgs2/srv/get_desired_posx.hpp"
#include "dsr_msgs2/srv/get_desired_velx.hpp"          
#include "dsr_msgs2/srv/get_current_solution_space.hpp" 
#include "dsr_msgs2/srv/get_current_rotm.hpp"          
#include "dsr_msgs2/srv/get_joint_torque.hpp"          
#include "dsr_msgs2/srv/get_external_torque.hpp"      
#include "dsr_msgs2/srv/get_tool_force.hpp"            
#include "dsr_msgs2/srv/get_solution_space.hpp"
#include "dsr_msgs2/srv/get_orientation_error.hpp"

//----- force/stiffness
#include "dsr_msgs2/srv/parallel_axis1.hpp"
#include "dsr_msgs2/srv/parallel_axis2.hpp"
#include "dsr_msgs2/srv/align_axis1.hpp"
#include "dsr_msgs2/srv/align_axis2.hpp"
#include "dsr_msgs2/srv/is_done_bolt_tightening.hpp"
#include "dsr_msgs2/srv/release_compliance_ctrl.hpp"
#include "dsr_msgs2/srv/task_compliance_ctrl.hpp"
#include "dsr_msgs2/srv/set_stiffnessx.hpp"
#include "dsr_msgs2/srv/calc_coord.hpp"
#include "dsr_msgs2/srv/set_user_cart_coord1.hpp"
#include "dsr_msgs2/srv/set_user_cart_coord2.hpp"
#include "dsr_msgs2/srv/set_user_cart_coord3.hpp"
#include "dsr_msgs2/srv/overwrite_user_cart_coord.hpp"
#include "dsr_msgs2/srv/get_user_cart_coord.hpp"
#include "dsr_msgs2/srv/set_desired_force.hpp"
#include "dsr_msgs2/srv/release_force.hpp"
#include "dsr_msgs2/srv/check_position_condition.hpp"
#include "dsr_msgs2/srv/check_force_condition.hpp"
#include "dsr_msgs2/srv/check_orientation_condition1.hpp"
#include "dsr_msgs2/srv/check_orientation_condition2.hpp"
#include "dsr_msgs2/srv/coord_transform.hpp"
#include "dsr_msgs2/srv/get_workpiece_weight.hpp"
#include "dsr_msgs2/srv/reset_workpiece_weight.hpp"

//io
#include "dsr_msgs2/srv/set_ctrl_box_digital_output.hpp"
#include "dsr_msgs2/srv/get_ctrl_box_digital_input.hpp"
#include "dsr_msgs2/srv/set_tool_digital_output.hpp"
#include "dsr_msgs2/srv/get_tool_digital_input.hpp"
#include "dsr_msgs2/srv/set_ctrl_box_analog_output.hpp"
#include "dsr_msgs2/srv/get_ctrl_box_analog_input.hpp"
#include "dsr_msgs2/srv/set_ctrl_box_analog_output_type.hpp"
#include "dsr_msgs2/srv/set_ctrl_box_analog_input_type.hpp"
#include "dsr_msgs2/srv/get_ctrl_box_digital_output.hpp"
#include "dsr_msgs2/srv/get_tool_digital_output.hpp"

//modbus
#include "dsr_msgs2/srv/set_modbus_output.hpp"
#include "dsr_msgs2/srv/get_modbus_input.hpp"
#include "dsr_msgs2/srv/config_create_modbus.hpp"
#include "dsr_msgs2/srv/config_delete_modbus.hpp"

//drl
#include "dsr_msgs2/srv/drl_pause.hpp"
#include "dsr_msgs2/srv/drl_start.hpp"
#include "dsr_msgs2/srv/drl_stop.hpp"
#include "dsr_msgs2/srv/drl_resume.hpp"
#include "dsr_msgs2/srv/get_drl_state.hpp"


//tcp
#include "dsr_msgs2/srv/config_create_tcp.hpp"
#include "dsr_msgs2/srv/config_delete_tcp.hpp"
#include "dsr_msgs2/srv/get_current_tcp.hpp"
#include "dsr_msgs2/srv/set_current_tcp.hpp"

//tool
#include "dsr_msgs2/srv/config_create_tool.hpp"
#include "dsr_msgs2/srv/config_delete_tool.hpp"
#include "dsr_msgs2/srv/get_current_tool.hpp"
#include "dsr_msgs2/srv/set_current_tool.hpp"
#include "dsr_msgs2/srv/set_tool_shape.hpp"

//gripper
#include "dsr_msgs2/srv/robotiq2_f_open.hpp"
#include "dsr_msgs2/srv/robotiq2_f_close.hpp"
#include "dsr_msgs2/srv/robotiq2_f_move.hpp"

//serial
///TODO #include <dsr_msgs2/srv/serial_send_data.h>

// moveit
///TODO #include <moveit_msgs/msg/display_trajectory.hpp>
///TODO #include <moveit_msgs/msg/robot_trajectory.hpp>

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <control_msgs/action/follow_joint_trajectory.hpp>  //ROS2 ???

#include "../../../common2/include/DRFL.h"
//TODO #include "../../../common2/include/dsr_serial.h"


#ifndef PI
#define PI 3.14159265359
#endif
#define deg2rad(deg)  ((deg) * PI / 180.0)
#define rad2deg(rad)  ((rad) * 180.0 / PI)

//_____ defines for Dooan Robot Controller _______________
#define POINT_COUNT         6

// solution space
#define DR_SOL_MIN          0
#define DR_SOL_MAX          7

// posb seg_type
#define DR_LINE             0
#define DR_CIRCLE           1

// move reference
#define DR_BASE             0
#define DR_TOOL             1
#define DR_WORLD            2
#define DR_TC_USER_MIN      101
#define DR_TC_USER_MAX      200

// move mod
#define DR_MV_MOD_ABS       0
#define DR_MV_MOD_REL       1

// move reaction
#define DR_MV_RA_NONE       0
#define DR_MV_RA_DUPLICATE  0
#define DR_MV_RA_OVERRIDE   1

// move command type
#define DR_MV_COMMAND_NORM  0

// movesx velocity
#define DR_MVS_VEL_NONE     0
#define DR_MVS_VEL_CONST    1

// motion state
#define DR_STATE_IDLE       0
#define DR_STATE_INIT       1
#define DR_STATE_BUSY       2
#define DR_STATE_BLEND      3
#define DR_STATE_ACC        4
#define DR_STATE_CRZ        5
#define DR_STATE_DEC        6

// axis
#define DR_AXIS_X           0
#define DR_AXIS_Y           1
#define DR_AXIS_Z           2
#define DR_AXIS_A          10
#define DR_AXIS_B          11
#define DR_AXIS_C          12

// collision sensitivity
#define DR_COLSENS_DEFAULT 20
#define DR_COLSENS_MIN      1   
#define DR_COLSENS_MAX    300

// speed
#define DR_OP_SPEED_MIN     1
#define DR_OP_SPEED_MAX   100

// stop
#define DR_QSTOP_STO        0
#define DR_QSTOP            1
#define DR_SSTOP            2
#define DR_HOLD             3

#define DR_STOP_FIRST       DR_QSTOP_STO
#define DR_STOP_LAST        DR_HOLD

// condition
#define DR_COND_NONE        -10000

// digital I/O
#define DR_DIO_MIN_INDEX    1
#define DR_DIO_MAX_INDEX    16  

// tool digital I/O
#define DR_TDIO_MIN_INDEX   1
#define DR_TDIO_MAX_INDEX   6

// I/O value
#define ON                  1
#define OFF                 0

// Analog I/O mode
#define DR_ANALOG_CURRENT   0
#define DR_ANALOG_VOLTAGE   1

// modbus type
#define DR_MODBUS_DIG_INPUT     0
#define DR_MODBUS_DIG_OUTPUT    1
#define DR_MODBUS_REG_INPUT     2
#define DR_MODBUS_REG_OUTPUT    3
#define DR_DISCRETE_INPUT       0
#define DR_COIL                 1
#define DR_INPUT_REGISTER       2
#define DR_HOLDING_REGISTER     3

#define DR_MODBUS_ACCESS_MAX    32
#define DR_MAX_MODBUS_NAME_SIZE 32

// tp_popup pm_type
#define DR_PM_MESSAGE           0
#define DR_PM_WARNING           1
#define DR_PM_ALARM             2

// tp_get_user_input type
#define DR_VAR_INT              0
#define DR_VAR_FLOAT            1
#define DR_VAR_STR              2
#define DR_VAR_BOOL             3   

// len
#define DR_VELJ_DT_LEN          6
#define DR_ACCJ_DT_LEN          6

#define DR_VELX_DT_LEN          2
#define DR_ACCX_DT_LEN          2

#define DR_ANGLE_DT_LEN         2
#define DR_COG_DT_LEN           3
#define DR_WEIGHT_DT_LEN        3
#define DR_VECTOR_DT_LEN        3
#define DR_ST_DT_LEN            6
#define DR_FD_DT_LEN            6
#define DR_DIR_DT_LEN           6
#define DR_INERTIA_DT_LEN       6
#define DR_VECTOR_U1_LEN        3
#define DR_VECTOR_V1_LEN        3

#define DR_AVOID                0
#define DR_TASK_STOP            1

#define DR_FIFO                 0
#define DR_LIFO                 1

#define DR_FC_MOD_ABS           0
#define DR_FC_MOD_REL           1

#define DR_GLOBAL_VAR_TYPE_BOOL         0
#define DR_GLOBAL_VAR_TYPE_INT          1
#define DR_GLOBAL_VAR_TYPE_FLOAT        2
#define DR_GLOBAL_VAR_TYPE_STR          3
#define DR_GLOBAL_VAR_TYPE_POSJ         4
#define DR_GLOBAL_VAR_TYPE_POSX         5
#define DR_GLOBAL_VAR_TYPE_UNKNOWN      6

#define DR_IE_SLAVE_GPR_ADDR_START      0
#define DR_IE_SLAVE_GPR_ADDR_END       23
#define DR_IE_SLAVE_GPR_ADDR_END_BIT   63

#define DR_DPOS                         0
#define DR_DVEL                         1

#define DR_HOME_TARGET_MECHANIC         0
#define DR_HOME_TARGET_USER             1

#define DR_MV_ORI_TEACH                 0    
#define DR_MV_ORI_FIXED                 1    
#define DR_MV_ORI_RADIAL                2    

#define DR_MV_APP_NONE                  0
#define DR_MV_APP_WELD                  1
//________________________________________________________

typedef struct {
    int	    nLevel;         // INFO =1, WARN =2, ERROR =3 
    int	    nGroup;         // SYSTEM =1, MOTION =2, TP =3, INVERTER =4, SAFETY_CONTROLLER =5   
    int	    nCode;          // error code 
    char    strMsg1[MAX_STRING_SIZE];   // error msg 1
    char    strMsg2[MAX_STRING_SIZE];   // error msg 2
    char    strMsg3[MAX_STRING_SIZE];   // error msg 3
} DR_ERROR, *LPDR_ERROR;

typedef struct {
    int     nRobotState;
    char    strRobotState[MAX_SYMBOL_SIZE];
    float   fCurrentPosj[NUM_JOINT];
    float   fCurrentPosx[NUM_TASK];
    float   fCurrentToolPosx[NUM_TASK];

    int     nActualMode;
    int     nActualSpace;
    
    float   fJointAbs[NUM_JOINT];
    float   fJointErr[NUM_JOINT];
    float   fTargetPosj[NUM_JOINT];
    float   fTargetVelj[NUM_JOINT];
    float   fCurrentVelj[NUM_JOINT];

    float   fTaskErr[NUM_TASK];
    float   fTargetPosx[NUM_TASK];
    float   fTargetVelx[NUM_TASK];
    float   fCurrentVelx[NUM_TASK];
    int     nSolutionSpace;
    float   fRotationMatrix[3][3];

    float   fDynamicTor[NUM_JOINT];
    float   fActualJTS[NUM_JOINT];
    float   fActualEJT[NUM_JOINT];
    float   fActualETT[NUM_JOINT];

    double  dSyncTime;
    int     nActualBK[NUM_JOINT];
    int     nActualBT[NUM_BUTTON];
    float   fActualMC[NUM_JOINT];
    float   fActualMT[NUM_JOINT];
    bool    bCtrlBoxDigitalOutput[16];
    bool    bCtrlBoxDigitalInput[16];
    bool    bFlangeDigitalOutput[6];
    bool    bFlangeDigitalInput[6];

    int     nRegCount;
    string  strModbusSymbol[100];
    int     nModbusValue[100];
  
    int     nAccessControl;
    bool    bHommingCompleted;
    bool    bTpInitialized;
    bool    bMasteringNeed;
    bool    bDrlStopped;
    bool    bDisconnected;

    //--- The following variables have been updated since version M2.50 or higher. ---
	//ROBOT_MONITORING_WORLD
	float   fActualW2B[6];
	float   fCurrentPosW[2][6];
	float   fCurrentVelW[6];
	float   fWorldETT[6];
	float   fTargetPosW[6];
	float   fTargetVelW[6];
	float   fRotationMatrixWorld[3][3];

	//ROBOT_MONITORING_USER
	int     iActualUCN;
	int     iParent;
	float   fCurrentPosU[2][6];
	float   fCurrentVelU[6];
	float   fUserETT[6];
	float   fTargetPosU[6];
	float   fTargetVelU[6];
	float   fRotationMatrixUser[3][3];

    //READ_CTRLIO_INPUT_EX
	float   fActualAI[6];
	bool    bActualSW[3];
	bool    bActualSI[2];
	int     iActualAT[2];

	//READ_CTRLIO_OUTPUT_EX
	float   fTargetAO[2];
	int     iTargetAT[2];

	//READ_ENCODER_INPUT
	bool    bActualES[2];
	int     iActualED[2];
	bool    bActualER[2];
    //---------------------------------------------------------------------------------

} DR_STATE, *LPDR_STATE;

using namespace DRAFramework;

namespace dsr_control2{

    class DRHWInterface : public hardware_interface::RobotHardware
    {
    public:
	    DRHWInterface(rclcpp::Node::SharedPtr& nh);
        virtual ~DRHWInterface();

        hardware_interface::return_type init();
        hardware_interface::return_type read();
        hardware_interface::return_type write();

        int MsgPublisher_RobotState();
        int MsgPublisher_JointState();

        ///int MsgPublisher_RobotError();  현재 미사용 : DRHWInterface::OnLogAlarm 에서 바로 퍼블리싱 함.
        static void OnHommingCompletedCB();
        static void OnProgramStoppedCB(const PROGRAM_STOP_CAUSE iStopCause);
        static void OnMonitoringDataCB(const LPMONITORING_DATA pData);
        static void OnMonitoringDataExCB(const LPMONITORING_DATA_EX pData);
        static void OnTpInitializingCompletedCB();

        static void OnMonitoringCtrlIOCB (const LPMONITORING_CTRLIO pCtrlIO);
        static void OnMonitoringCtrlIOExCB (const LPMONITORING_CTRLIO_EX pCtrlIO);
        static void OnMonitoringModbusCB (const LPMONITORING_MODBUS pModbus);
        static void OnMonitoringStateCB(const ROBOT_STATE eState);
        static void OnMonitoringAccessControlCB(const MONITORING_ACCESS_CONTROL eAccCtrl);
        static void OnLogAlarm(LPLOG_ALARM pLogAlarm);

        std::string GetRobotName();
        std::string GetRobotModel();

    private:
        int  m_nVersionDRCF;
        bool m_bIsEmulatorMode; 

        rclcpp::Node::SharedPtr private_nh_;	

        std::string m_strRobotName;
        std::string m_strRobotModel;
        std::string m_strRobotGripper;

        //----- Service ---------------------------------------------------------------
        /*ROS2	        	   	
        ros::ServiceServer m_nh_system[14];
        ros::ServiceServer m_nh_motion_service[32];
        ros::ServiceServer m_nh_aux_control_service[32];
        ros::ServiceServer m_nh_force_service[32];

        ros::ServiceServer m_nh_io_service[10];
        ros::ServiceServer m_nh_modbus_service[4];
        ros::ServiceServer m_nh_drl_service[10];
        ros::ServiceServer m_nh_tcp_service[4];
        ros::ServiceServer m_nh_tool_service[5];
        ros::ServiceServer m_nh_gripper_service[10];
        ros::ServiceServer m_nh_serial_service[4];
        */

        rclcpp::Service<dsr_msgs2::srv::SetRobotMode>::SharedPtr   m_nh_system_0;

        rclcpp::Service<dsr_msgs2::srv::MoveJoint>::SharedPtr      m_nh_motion_service_0;
        rclcpp::Service<dsr_msgs2::srv::MoveLine>::SharedPtr       m_nh_motion_service_1;



//ROS2 ???        rclcpp::Node::SharedPtr m_nh_system[14];
//ROS2 ???        rclcpp::Node::SharedPtr m_nh_motion_service[32];
//ROS2 ???        rclcpp::Node::SharedPtr m_nh_aux_control_service[32];
//ROS2 ???        rclcpp::Node::SharedPtr m_nh_force_service[32];
//ROS2 ???
//ROS2 ???        rclcpp::Node::SharedPtr m_nh_io_service[10];
//ROS2 ???        rclcpp::Node::SharedPtr m_nh_modbus_service[4];
//ROS2 ???        rclcpp::Node::SharedPtr m_nh_drl_service[10];
//ROS2 ???        rclcpp::Node::SharedPtr m_nh_tcp_service[4];
//ROS2 ???        rclcpp::Node::SharedPtr m_nh_tool_service[5];
//ROS2 ???        rclcpp::Node::SharedPtr m_nh_gripper_service[10];
//ROS2 ???        rclcpp::Node::SharedPtr m_nh_serial_service[4];


        //----- Publisher -------------------------------------------------------------
        rclcpp::Publisher<dsr_msgs2::msg::RobotState>::SharedPtr            m_PubRobotState;
        rclcpp::Publisher<dsr_msgs2::msg::RobotError>::SharedPtr            m_PubRobotError;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr      m_PubtoGazebo;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr                 m_PubSerialWrite;
        ///TODO rclcpp::Publisher<dsr_msgs::msg::JogMultiAxis>::SharedPtr  m_PubJogMultiAxis;

        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr/*rclcpp::Node::SharedPtr*/ m_PubJointState;  //add for TEST

        //----- Subscriber ------------------------------------------------------------
        rclcpp::Subscription<control_msgs::action::FollowJointTrajectory::Goal>::SharedPtr m_sub_joint_trajectory;
        ///TODO rclcpp::Subscription<???>::SharedPtr                        m_sub_joint_position;
        ///TODO rclcpp::Subscription<???>::SharedPtr                        m_SubSerialRead;
        ///TODO rclcpp::Subscription<???>::SharedPtr                        m_sub_jog_multi_axis;

        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr/*rclcpp::Node::SharedPtr*/ m_sub_test;

        // ROS Interface
//ROS2        hardware_interface::JointStateInterface jnt_state_interface;
//ROS2        hardware_interface::PositionJointInterface jnt_pos_interface;
//ROS2        hardware_interface::VelocityJointInterface velocity_joint_interface_;
        std::vector<hardware_interface::JointStateHandle> joint_state_handles_;
        std::vector<hardware_interface::JointCommandHandle> joint_command_handles_;
        std::vector<hardware_interface::OperationModeHandle> joint_mode_handles_;

        std::array<float, NUM_JOINT> m_fCmd_;
        bool m_bCommand_;
        struct Joint{
            double cmd;
            double pos;
            double vel;
            double eff;
            Joint(): cmd(0), pos(0), vel(0), eff(0) {}
        } m_joints[NUM_JOINT];

        //----- SIG Handler --------------------------------------------------------------
//ROS2 ???        void sigint_handler( int signo);

//ROS2  void trajectoryCallback(const control_msgs::FollowJointTrajectoryActionGoal::ConstPtr& msg);
        void trajectoryCallback(const control_msgs::action::FollowJointTrajectory_Goal::SharedPtr msg) const;
        void testSubCallback(const std_msgs::msg::String::SharedPtr msg) const;
//ROS2        void positionCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
//ROS2 ???              void positionCallback(const std_msgs::msg::Float64MultiArray::ConstPtr& msg);

//ROS2        void jogCallback(const dsr_msgs2::JogMultiAxis::ConstPtr& msg);
//ROS2 ???	      void jogCallback(const dsr_msgs2::msg::JogMultiAxis::ConstPtr& msg);

        //----- Threads ------------------------------------------------------------------
        boost::thread m_th_subscribe;   //subscribe thread
        boost::thread m_th_publisher;   //publisher thread
        boost::thread m_th_publisher_joint_states;   //publisher thread

        static void thread_subscribe(rclcpp::Node::SharedPtr nh);
        static void thread_publisher(DRHWInterface* pDRHWInterface, rclcpp::Node::SharedPtr nh, int nPubRate);
        static void thread_publisher_direct_access_joint_states(DRHWInterface* pDRHWInterface, rclcpp::Node::SharedPtr nh, int nPubRate);// '/joint_state'에 직접 퍼블리싱하는 함수(현재 미사용) 
                                                                                                                                         // ros2_control를 통해서 퍼블리싱 함. 

        DR_STATE m_stDrState;
        DR_ERROR m_stDrError;

        //----- Service Call-back functions ----------------------------------------------.

        //----- System
        static void set_robot_mode_cb(const std::shared_ptr<dsr_msgs2::srv::SetRobotMode::Request> req, std::shared_ptr<dsr_msgs2::srv::SetRobotMode::Response> res);

        //----- MOTION
        static void movej_cb(const std::shared_ptr<dsr_msgs2::srv::MoveJoint::Request> req, std::shared_ptr<dsr_msgs2::srv::MoveJoint::Response> res);
        static void movel_cb(const std::shared_ptr<dsr_msgs2::srv::MoveLine::Request> req, std::shared_ptr<dsr_msgs2::srv::MoveLine::Response> res);

        //----- auxiliary_control

        //----- force/stiffness

        //----- TCP

        //----- TOOL

        //----- IO

        //----- MODBUS

        //----- DRL        

        //----- Gripper

        //----- Serial

    };
}

#endif // end
