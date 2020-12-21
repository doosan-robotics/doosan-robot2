#-*- coding: utf-8 -*-

# ##
# @mainpage
# @file     DSR_ROBOT2.py
# @brief    Doosan Robotics ROS2 service I/F module
# @author   kabdol2<kabkyoum.kim@doosan.com>   
# @version  0.10
# @Last update date     2020-12-15
# @details
#
# history
#  -   
#
__ROS2__ = True

import rclpy
import os
import threading, time
from std_msgs.msg import String,Int32,Int32MultiArray,Float32,Float64,Float32MultiArray,Float64MultiArray,MultiArrayLayout,MultiArrayDimension
import sys
sys.dont_write_bytecode = True

#import numpy as np

from DRFC import *
from DR_common2 import *

from dsr_msgs2.msg import *
from dsr_msgs2.srv import *

#-----------------------------------------------------------------------------------
import DR_init
g_node = None
g_node = DR_init.__dsr__node

_robot_id    = DR_init.__dsr__id
_robot_model = DR_init.__dsr__model
_srv_name_prefix   = '' #'/' + _robot_id + _robot_model #ROS2  
_topic_name_prefix = _srv_name_prefix

print("_robot_id ={0}".format(_robot_id))
print("_robot_model ={0}".format(_robot_model))
print("_srv_name_prefix ={0}".format(_srv_name_prefix))
print("_topic_name_prefix ={0}".format(_topic_name_prefix))
#-----------------------------------------------------------------------------------


############### connect to dsr_control2 (ros2 service) ####################################################################### 

#  system Operations
_ros2_set_robot_mode             = g_node.create_client(SetRobotMode,           _srv_name_prefix +"system/set_robot_mode")
_ros2_get_robot_mode             = g_node.create_client(GetRobotMode,           _srv_name_prefix +"system/get_robot_mode")
_ros2_set_robot_system           = g_node.create_client(SetRobotSystem,         _srv_name_prefix +"system/set_robot_system")
_ros2_get_robot_system           = g_node.create_client(GetRobotSystem,         _srv_name_prefix +"system/get_robot_system")
_ros2_get_robot_state            = g_node.create_client(GetRobotState,          _srv_name_prefix +"system/get_robot_state")
_ros2_set_robot_speed_mode       = g_node.create_client(SetRobotSpeedMode,      _srv_name_prefix +"system/set_robot_speed_mode")
_ros2_get_robot_speed_mode       = g_node.create_client(GetRobotSpeedMode,      _srv_name_prefix +"system/get_robot_speed_mode")
_ros2_set_safe_stop_reset_type   = g_node.create_client(SetSafeStopResetType,   _srv_name_prefix +"system/set_safe_stop_reset_type")
_ros2_get_last_alarm             = g_node.create_client(GetLastAlarm,           _srv_name_prefix +"system/get_last_alarm")
_ros2_get_current_pose           = g_node.create_client(GetCurrentPose,         _srv_name_prefix +"system/get_current_pose")

#  motion Operations
_ros2_movej                      = g_node.create_client(MoveJoint,              _srv_name_prefix +"motion/move_joint")
_ros2_movel                      = g_node.create_client(MoveLine,               _srv_name_prefix +"motion/move_line")
_ros2_movejx                     = g_node.create_client(MoveJointx,             _srv_name_prefix +"motion/move_jointx")
_ros2_movec                      = g_node.create_client(MoveCircle,             _srv_name_prefix +"motion/move_circle")
_ros2_movesj                     = g_node.create_client(MoveSplineJoint,        _srv_name_prefix +"motion/move_spline_joint")
_ros2_movesx                     = g_node.create_client(MoveSplineTask,         _srv_name_prefix +"motion/move_spline_task")
_ros2_moveb                      = g_node.create_client(MoveBlending,           _srv_name_prefix +"motion/move_blending")
_ros2_move_spiral                = g_node.create_client(MoveSpiral,             _srv_name_prefix +"motion/move_spiral")
_ros2_move_periodic              = g_node.create_client(MovePeriodic,           _srv_name_prefix +"motion/move_periodic")
_ros2_move_wait                  = g_node.create_client(MoveWait,               _srv_name_prefix +"motion/move_wait")
_ros2_jog                        = g_node.create_client(Jog,                    _srv_name_prefix +"motion/jog")
_ros2_jog_multi                  = g_node.create_client(JogMulti,               _srv_name_prefix +"motion/jog_multi")
_ros2_trans                      = g_node.create_client(Trans,                  _srv_name_prefix +"motion/trans")
_ros2_fkin                       = g_node.create_client(Fkin,                   _srv_name_prefix +"motion/fkin")
_ros2_ikin                       = g_node.create_client(Ikin,                   _srv_name_prefix +"motion/ikin")
_ros2_set_ref_coord              = g_node.create_client(SetRefCoord,            _srv_name_prefix +"motion/set_ref_coord")
_ros2_move_home                  = g_node.create_client(MoveHome,               _srv_name_prefix +"motion/move_home")
_ros2_check_motion               = g_node.create_client(CheckMotion,            _srv_name_prefix +"motion/check_motion")
_ros2_change_operation_speed     = g_node.create_client(ChangeOperationSpeed,   _srv_name_prefix +"motion/change_operation_speed")
_ros2_enable_alter_motion        = g_node.create_client(EnableAlterMotion,      _srv_name_prefix +"motion/enable_alter_motion")
_ros2_alter_motion               = g_node.create_client(AlterMotion,            _srv_name_prefix +"motion/alter_motion")
_ros2_disable_alter_motion       = g_node.create_client(DisableAlterMotion,     _srv_name_prefix +"motion/disable_alter_motion")


# Auxiliary Control Operations
_ros2_get_control_mode               = g_node.create_client(GetControlMode,     _srv_name_prefix +"aux_control/get_control_mode")
_ros2_get_control_space              = g_node.create_client(GetControlSpace,    _srv_name_prefix +"aux_control/get_control_space")
 
_ros2_get_current_posj               = g_node.create_client(GetCurrentPosj,     _srv_name_prefix +"aux_control/get_current_posj")
_ros2_get_current_velj               = g_node.create_client(GetCurrentVelj,     _srv_name_prefix +"aux_control/get_current_velj")
_ros2_get_desired_posj               = g_node.create_client(GetDesiredPosj,     _srv_name_prefix +"aux_control/get_desired_posj")
_ros2_get_desired_velj               = g_node.create_client(GetDesiredVelj,     _srv_name_prefix +"aux_control/get_desired_velj")
 
_ros2_get_current_posx               = g_node.create_client(GetCurrentPosx,     _srv_name_prefix +"aux_control/get_current_posx")
_ros2_get_current_velx               = g_node.create_client(GetCurrentVelx,     _srv_name_prefix +"aux_control/get_current_velx")    
_ros2_get_desired_posx               = g_node.create_client(GetDesiredPosx,     _srv_name_prefix +"aux_control/get_desired_posx")
_ros2_get_desired_velx               = g_node.create_client(GetDesiredVelx,     _srv_name_prefix +"aux_control/get_desired_velx")
 
_ros2_get_current_tool_flange_posx   = g_node.create_client(GetCurrentToolFlangePosx,   _srv_name_prefix +"aux_control/get_current_tool_flange_posx")
 
_ros2_get_current_solution_space     = g_node.create_client(GetCurrentSolutionSpace,    _srv_name_prefix +"aux_control/get_current_solution_space")
_ros2_get_current_rotm               = g_node.create_client(GetCurrentRotm,             _srv_name_prefix +"aux_control/get_current_rotm")    
_ros2_get_joint_torque               = g_node.create_client(GetJointTorque,             _srv_name_prefix +"aux_control/get_joint_torque")
_ros2_get_external_torque            = g_node.create_client(GetExternalTorque,          _srv_name_prefix +"aux_control/get_external_torque")
_ros2_get_tool_force                 = g_node.create_client(GetToolForce,               _srv_name_prefix +"aux_control/get_tool_force")
_ros2_get_solution_space             = g_node.create_client(GetSolutionSpace,           _srv_name_prefix +"aux_control/get_solution_space")
_ros2_get_orientation_error          = g_node.create_client(GetOrientationError,        _srv_name_prefix +"aux_control/get_orientation_error")

# Force/Stiffness Control & others Operations
_ros2_get_workpiece_weight        = g_node.create_client(GetWorkpieceWeight,            _srv_name_prefix +"force/get_workpiece_weight")
_ros2_reset_workpiece_weight      = g_node.create_client(ResetWorkpieceWeight,          _srv_name_prefix +"force/reset_workpiece_weight")
_ros2_set_singularity_handling    = g_node.create_client(SetSingularityHandling,        _srv_name_prefix +"motion/set_singularity_handling")
 
_ros2_parallel_axis1              = g_node.create_client(ParallelAxis1,                 _srv_name_prefix +"force/parallel_axis1")
_ros2_parallel_axis2              = g_node.create_client(ParallelAxis2,                 _srv_name_prefix +"force/parallel_axis2")
_ros2_align_axis1                 = g_node.create_client(AlignAxis1,                    _srv_name_prefix +"force/align_axis1")
_ros2_align_axis2                 = g_node.create_client(AlignAxis2,                    _srv_name_prefix +"force/align_axis2")
_ros2_is_done_bolt_tightening     = g_node.create_client(IsDoneBoltTightening,          _srv_name_prefix +"force/is_done_bolt_tightening")
_ros2_release_compliance_ctrl     = g_node.create_client(ReleaseComplianceCtrl,         _srv_name_prefix +"force/release_compliance_ctrl")
_ros2_task_compliance_ctrl        = g_node.create_client(TaskComplianceCtrl,            _srv_name_prefix +"force/task_compliance_ctrl")
_ros2_set_stiffnessx              = g_node.create_client(SetStiffnessx,                 _srv_name_prefix +"force/set_stiffnessx")
_ros2_calc_coord                  = g_node.create_client(CalcCoord,                     _srv_name_prefix +"force/calc_coord")
_ros2_set_user_cart_coord1        = g_node.create_client(SetUserCartCoord1,             _srv_name_prefix +"force/set_user_cart_coord1")
_ros2_set_user_cart_coord2        = g_node.create_client(SetUserCartCoord2,             _srv_name_prefix +"force/set_user_cart_coord2")
_ros2_set_user_cart_coord3        = g_node.create_client(SetUserCartCoord3,             _srv_name_prefix +"force/set_user_cart_coord3")
_ros2_overwrite_user_cart_coord   = g_node.create_client(OverwriteUserCartCoord,        _srv_name_prefix +"force/overwrite_user_cart_coord")
_ros2_get_user_cart_coord         = g_node.create_client(GetUserCartCoord,              _srv_name_prefix +"force/get_user_cart_coord")
_ros2_set_desired_force           = g_node.create_client(SetDesiredForce,               _srv_name_prefix +"force/set_desired_force")
_ros2_release_force               = g_node.create_client(ReleaseForce,                  _srv_name_prefix +"force/release_force")
_ros2_check_position_condition    = g_node.create_client(CheckPositionCondition,        _srv_name_prefix +"force/check_position_condition")
_ros2_check_force_condition       = g_node.create_client(CheckForceCondition,           _srv_name_prefix +"force/check_force_condition")
_ros2_check_orientation_condition1= g_node.create_client(CheckOrientationCondition1,    _srv_name_prefix +"force/check_orientation_condition1")
_ros2_check_orientation_condition2= g_node.create_client(CheckOrientationCondition2,    _srv_name_prefix +"force/check_orientation_condition2")
_ros2_coord_transform             = g_node.create_client(CoordTransform,                _srv_name_prefix +"force/coord_transform")

#  GPIO Operations
_ros2_set_digital_output         = g_node.create_client(SetCtrlBoxDigitalOutput,        _srv_name_prefix +"io/set_digital_output")
_ros2_get_digital_input          = g_node.create_client(GetCtrlBoxDigitalInput,         _srv_name_prefix +"io/get_digital_input")
_ros2_set_tool_digital_output    = g_node.create_client(SetToolDigitalOutput,           _srv_name_prefix +"io/set_tool_digital_output")
_ros2_get_tool_digital_input     = g_node.create_client(GetToolDigitalInput,            _srv_name_prefix +"io/get_tool_digital_input")
_ros2_set_analog_output          = g_node.create_client(SetCtrlBoxAnalogOutput,         _srv_name_prefix +"io/set_analog_output")
_ros2_get_analog_input           = g_node.create_client(GetCtrlBoxAnalogInput,          _srv_name_prefix +"io/get_analog_input")
_ros2_set_mode_analog_output     = g_node.create_client(SetCtrlBoxAnalogOutputType,     _srv_name_prefix +"io/set_analog_output_type")
_ros2_set_mode_analog_input      = g_node.create_client(SetCtrlBoxAnalogInputType,      _srv_name_prefix +"io/set_analog_input_type")
_ros2_get_digital_output         = g_node.create_client(GetCtrlBoxDigitalOutput,        _srv_name_prefix +"io/get_digital_output")
_ros2_get_tool_digital_output    = g_node.create_client(GetToolDigitalOutput,           _srv_name_prefix +"io/get_tool_digital_output")
 
#  Modbus Operations, 
_ros2_set_modbus_output          = g_node.create_client(SetModbusOutput,                _srv_name_prefix +"modbus/set_modbus_output")
_ros2_get_modbus_input           = g_node.create_client(GetModbusInput,                 _srv_name_prefix +"modbus/get_modbus_input")
_ros2_add_modbus_signal          = g_node.create_client(ConfigCreateModbus,             _srv_name_prefix +"modbus/config_create_modbus")
_ros2_del_modbus_signal          = g_node.create_client(ConfigDeleteModbus,             _srv_name_prefix +"modbus/config_delete_modbus")

# TCP Operations, 
_ros2_set_current_tcp            = g_node.create_client(SetCurrentTcp,                  _srv_name_prefix +"tcp/set_current_tcp")
_ros2_get_current_tcp            = g_node.create_client(GetCurrentTcp,                  _srv_name_prefix +"tcp/get_current_tcp")        
_ros2_config_create_tcp          = g_node.create_client(ConfigCreateTcp,                _srv_name_prefix +"tcp/config_create_tcp")
_ros2_config_delete_tcp          = g_node.create_client(ConfigDeleteTcp,                _srv_name_prefix +"tcp/config_delete_tcp")
 
# Tool Operations, 
_ros2_set_current_tool           = g_node.create_client(SetCurrentTool,                 _srv_name_prefix +"tool/set_current_tool")
_ros2_get_current_tool           = g_node.create_client(GetCurrentTool,                 _srv_name_prefix +"tool/get_current_tool")      
_ros2_config_create_tool         = g_node.create_client(ConfigCreateTool,               _srv_name_prefix +"tool/config_create_tool")
_ros2_config_delete_tool         = g_node.create_client(ConfigDeleteTool,               _srv_name_prefix +"tool/config_delete_tool")
_ros2_set_tool_shape             = g_node.create_client(SetToolShape,                   _srv_name_prefix +"tool/set_tool_shape")

# DRL Operations, 
_ros2_drl_pause                  = g_node.create_client(DrlPause,                       _srv_name_prefix +"drl/drl_pause")
_ros2_drl_resume                 = g_node.create_client(DrlResume,                      _srv_name_prefix +"drl/drl_resume")
_ros2_drl_start                  = g_node.create_client(DrlStart,                       _srv_name_prefix +"drl/drl_start")
_ros2_drl_stop                   = g_node.create_client(DrlStop,                        _srv_name_prefix +"drl/drl_stop")
_ros2_get_drl_state              = g_node.create_client(GetDrlState,                    _srv_name_prefix +"drl/get_drl_state")

########################################################################################################################################

# point count
POINT_COUNT = 6

# solution space
DR_SOL_MIN = 0
DR_SOL_MAX = 7

# posb seg_type
DR_LINE   = 0
DR_CIRCLE = 1

# move reference
DR_BASE        = 0
DR_TOOL        = 1
DR_WORLD       = 2
DR_TC_USER_MIN = 101
DR_TC_USER_MAX = 200

# move mod
DR_MV_MOD_ABS = 0
DR_MV_MOD_REL = 1

# move reaction
DR_MV_RA_NONE      = 0
DR_MV_RA_DUPLICATE = 0
DR_MV_RA_OVERRIDE  = 1

# move command type
DR_MV_COMMAND_NORM = 0

# movesx velocity
DR_MVS_VEL_NONE    = 0
DR_MVS_VEL_CONST   = 1

# motion state
DR_STATE_IDLE   = 0
DR_STATE_INIT   = 1
DR_STATE_BUSY   = 2
DR_STATE_BLEND  = 3
DR_STATE_ACC    = 4
DR_STATE_CRZ    = 5
DR_STATE_DEC    = 6

# axis
DR_AXIS_X = 0
DR_AXIS_Y = 1
DR_AXIS_Z = 2
DR_AXIS_A = 10
DR_AXIS_B = 11
DR_AXIS_C = 12

# collision sensitivity
DR_COLSENS_DEFAULT = 20
DR_COLSENS_MIN = 1   #10   2017/11/08 변경 
DR_COLSENS_MAX = 300 #100  2017/11/08 변경

# speed
DR_OP_SPEED_MIN = 1
DR_OP_SPEED_MAX = 100

# stop
DR_QSTOP_STO = 0
DR_QSTOP     = 1
DR_SSTOP     = 2
DR_HOLD      = 3

DR_STOP_FIRST = DR_QSTOP_STO
DR_STOP_LAST = DR_HOLD

# condition
DR_COND_NONE = -10000

# digital I/O
DR_DIO_MIN_INDEX = 1
DR_DIO_MAX_INDEX = 16   #8 16개로 확장됨 2017/08/18  

# tool digital I/O
DR_TDIO_MIN_INDEX = 1
DR_TDIO_MAX_INDEX = 6

# I/O value
ON = 1
OFF = 0

# Analog I/O mode
DR_ANALOG_CURRENT = 0
DR_ANALOG_VOLTAGE = 1

# modbus type
DR_MODBUS_DIG_INPUT  = 0
DR_MODBUS_DIG_OUTPUT = 1
DR_MODBUS_REG_INPUT  = 2
DR_MODBUS_REG_OUTPUT = 3
DR_DISCRETE_INPUT    = DR_MODBUS_DIG_INPUT
DR_COIL              = DR_MODBUS_DIG_OUTPUT
DR_INPUT_REGISTER    = DR_MODBUS_REG_INPUT
DR_HOLDING_REGISTER  = DR_MODBUS_REG_OUTPUT

DR_MODBUS_ACCESS_MAX    = 32
DR_MAX_MODBUS_NAME_SIZE = 32

# tp_popup pm_type
DR_PM_MESSAGE = 0
DR_PM_WARNING = 1
DR_PM_ALARM   = 2
DR_TP_POPUP_BUTTON_TYPE_STOP_RESUME = 0 # add 2019/04/01
DR_TP_POPUP_BUTTON_TYPE_STOP        = 1 # add 2019/04/01

# tp_get_user_input type
DR_VAR_INT   = 0
DR_VAR_FLOAT = 1
DR_VAR_STR   = 2
DR_VAR_BOOL  = 3 # add 2020/01/29

# len
DR_VELJ_DT_LEN = 6
DR_ACCJ_DT_LEN = 6

DR_VELX_DT_LEN = 2
DR_ACCX_DT_LEN = 2

DR_ANGLE_DT_LEN   = 2
DR_COG_DT_LEN     = 3
DR_WEIGHT_DT_LEN  = 3
DR_VECTOR_DT_LEN  = 3
DR_ST_DT_LEN      = 6
DR_FD_DT_LEN      = 6
DR_DIR_DT_LEN     = 6
DR_INERTIA_DT_LEN = 6
DR_VECTOR_U1_LEN  = 3
DR_VECTOR_V1_LEN  = 3

# set_singular_handling mode 
DR_AVOID     = 0
DR_TASK_STOP = 1
DR_VAR_VEL   = 2        #add 2019/04/01 

# object container type
DR_FIFO      = 0
DR_LIFO      = 1

# set_desired_force mod
DR_FC_MOD_ABS = 0
DR_FC_MOD_REL = 1

# global variable type
DR_GLOBAL_VAR_TYPE_BOOL   = 0
DR_GLOBAL_VAR_TYPE_INT    = 1
DR_GLOBAL_VAR_TYPE_FLOAT  = 2
DR_GLOBAL_VAR_TYPE_STR    = 3
DR_GLOBAL_VAR_TYPE_POSJ   = 4
DR_GLOBAL_VAR_TYPE_POSX   = 5
DR_GLOBAL_VAR_TYPE_UNKNOWN= 6

# Industrial Ethernet(EtherNet/IP, PROFINET) Slave address
DR_IE_SLAVE_GPR_ADDR_START     = 0
DR_IE_SLAVE_GPR_ADDR_END       =23
DR_IE_SLAVE_GPR_ADDR_END_BIT   =63

# 경로 수정 기능
DR_DPOS = 0
DR_DVEL = 1

# Homing 기능
DR_HOME_TARGET_MECHANIC = 0
DR_HOME_TARGET_USER     = 1

# movec ori 옵션 
DR_MV_ORI_TEACH  = 0    #교시자세
DR_MV_ORI_FIXED  = 1    #고정자세
DR_MV_ORI_RADIAL = 2    #원주구속자세

# app_type
DR_MV_APP_NONE   = 0
DR_MV_APP_WELD   = 1

# =============================================================================================
# global variable

DR_CONFIG_PRT_EXT_RESULT = False
DR_CONFIG_PRT_RESULT     = False

_g_blend_state  = False
_g_blend_radius = 0.0

_g_velj = [0.0] * DR_VELJ_DT_LEN
_g_accj = [0.0] * DR_ACCJ_DT_LEN

_g_velx = [0.0] * DR_VELX_DT_LEN
_g_velx[0]= 0.0
_g_velx[1]= DR_COND_NONE

_g_accx = [0.0] * DR_ACCX_DT_LEN
_g_accx[0]= 0.0
_g_accx[1]= DR_COND_NONE

_g_coord = DR_BASE
_g_drl_result_th = None

#_g_tp_lock = threading.Lock()       # only 1 execution allowed with TP

_g_test_cnt =0
_g_test_max =0

_g_analog_output_mode_ch1 = -1
_g_analog_output_mode_ch2 = -1

#DR_SUB_PROGRAM_LIST = dict()    #서브 프로그램 저장 리스트

########################################################################################################################################

def wait(second):
    time.sleep(second)

def print_ext_result(str):
    if DR_CONFIG_PRT_EXT_RESULT:
        # print("[{0}] / {1}".format(strftime("%Y-%m-%d %H:%M:%S", gmtime()), str))
        print("__{0}".format(str))

def print_result(str):
    if DR_CONFIG_PRT_RESULT:
        # print("[{0}] / {1}".format(strftime("%Y-%m-%d %H:%M:%S", gmtime()), str))
        print("__{0}".format(str))

def _check_valid_vel_acc_joint(vel, acc, time):
    if float(time) == 0.0:
        for item in vel:
            if float(item) != 0.0:
                break
        else:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v (0.0, when time = 0.0)", True)

        for item in acc:
            if float(item) != 0.0:
                break
        else:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a (0.0, when time = 0.0)", True)

    return

def _check_valid_vel_acc_task(vel, acc, time):

    if float(time) == 0.0:
        if vel[0] <= 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel1, v1 (0.0, when time = 0.0)", True)
        else:
            if (vel[1]!=DR_COND_NONE) and (vel[1]<=0):
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel2, v2 (0.0, when time = 0.0)", True)

        if acc[0] <= 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc1, a1 (0.0, when time = 0.0)", True)
        else:
            if (acc[1]!=DR_COND_NONE) and (acc[1]<=0):
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc2, a2 (0.0, when time = 0.0)", True)

    return


def set_velj(vel):
    vel_list = None

    # vel
    if type(vel) == int or type(vel) == float:
        if vel <= 0:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : vel")

        vel_list = [vel] * DR_VELJ_DT_LEN
    elif type(vel) == list and len(vel) == POINT_COUNT:
        vel_list = vel

        if is_number(vel_list) != True:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : vel")

        for item in vel:
            if item <= 0:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel")
    else:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : vel")

    # set global velj
    global _g_velj

    _g_velj = vel_list

    print_result("0 = set_velj(vel:{0})".format(dr_form(vel)))
    return 0

def set_accj(acc):
    acc_list = None

    # acc
    if type(acc) == int or type(acc) == float:
        acc_list = [acc] * DR_ACCJ_DT_LEN

        if acc <= 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc")
    elif type(acc) == list and len(acc) == POINT_COUNT:
        acc_list = acc

        if is_number(acc_list) != True:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : acc")

        for item in acc:
            if item <= 0:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc")
    else:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : acc")

    # set global velj
    global _g_accj

    _g_accj = acc_list

    print_result("0 = set_accj(acc:{0})".format(dr_form(acc)))
    return 0

def set_velx(vel1, vel2=DR_COND_NONE):
    #print("vel1={0}, vel2={1}".format(vel1, vel2))

    # vel1
    if type(vel1) != int and type(vel1) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : vel1")

    if vel1 <= 0:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel1")

    # vel2
    if type(vel2) != int and type(vel2) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : vel2")

    if vel2 != DR_COND_NONE:
        if vel2 <= 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel2")

    # set global velx
    global _g_velx

    _g_velx = [vel1, vel2]

    print_result("0 = set_velx(vel1:{0}, vel2:{1})".format(dr_form(vel1), dr_form(vel2)))
    return 0

def set_accx(acc1, acc2=DR_COND_NONE):
    #print("acc1={0}, acc2={1}".format(acc1, acc2))

    # acc1
    if type(acc1) != int and type(acc1) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : acc1")

    if acc1 <= 0:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc1")

    # acc2
    if type(acc2) != int and type(acc2) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : acc2")

    if acc2 != DR_COND_NONE:
        if acc2 <= 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc2")

    # set global accx
    global _g_accx

    _g_accx = [acc1, acc2]

    print_result("0 = set_accx(acc1:{0}, acc2:{1})".format(dr_form(acc1), dr_form(acc2)))
    return 0

# convert : list -> Float64MultiArray
def _ros_listToFloat64MultiArray(list_src):
    _res = []
    for i in list_src:
        item = Float64MultiArray()
        if __ROS2__:
            item.data = [float(x) for x in i]
        else:
            item.data = i
        _res.append(item)
    #print(_res)
    #print(len(_res))
    return _res

# convert : Float64MultiArray -> list
def _ros_Float64MultiArrayTolist(multi_arr_f64):
    _res = []
    for i in range( len(multi_arr_f64) ):
        _res.append( list(multi_arr_f64[i].data) )   
    #print(_res)
    #print(len(_res))
    return _res


##### SYSTEM ##############################################################################################################################


###############################
def set_robot_mode(robot_mode):
    if type(robot_mode) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : robot_mode")

    # ROS2 service call
    if __ROS2__:
        #srv = _ros_set_robot_mode(robot_mode)
        req = SetRobotMode.Request()
        req.robot_mode = robot_mode

        #ret = 0 if (srv.success == True) else -1
        future = _ros2_set_robot_mode.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('set_robot_mode Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                ret = 0 if (result.success == True) else -1                
    return ret




def movej(pos, vel=None, acc=None, time=None, radius=None, mod= DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE, v=None, a=None, t=None, r=None):
    ret = _movej(pos, vel, acc, time, radius, mod, ra, v, a, t, r, _async=0)
    return ret
def amovej(pos, vel=None, acc=None, time=None, radius=None, mod= DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE, v=None, a=None, t=None, r=None):
    ret = _movej(pos, vel, acc, time, radius, mod, ra, v, a, t, r, _async=1)
    return ret
def _movej(pos, vel=None, acc=None, time=None, radius=None, mod= DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE, v=None, a=None, t=None, r=None, _async=0):
    # _pos
    _pos = get_posj(pos)

    # _vel
    temp = get_param(vel, v)
    if temp == None:
        _vel = _g_velj
    else:
        if type(temp) == int or type(temp) == float:
            _vel = [temp] * DR_VELJ_DT_LEN
        elif type(temp) == list and len(temp) == DR_VELJ_DT_LEN:
            _vel = temp
        else:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : vel, v")

    if is_number(_vel) != True:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")

    for item in _vel:
        if item < 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")

    # _acc
    temp = get_param(acc, a)
    if temp == None:
        _acc = _g_accj
    else:
        if type(temp) == int or type(temp) == float:
            _acc = [temp] * DR_ACCJ_DT_LEN
        elif type(temp) == list and len(temp) == DR_ACCJ_DT_LEN:
            _acc = temp
        else:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : acc, a")

    if is_number(_acc) != True:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")

    for item in _acc:
        if item < 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")

    # _time
    _time = get_param(time, t)
    if _time == None:
        _time = 0.0

    if type(_time) != int and type(_time) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : time, t")

    if _time < 0:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : time, t")

    # check vel, acc, time
    #_check_valid_vel_acc(_vel, _acc, _time)
    _check_valid_vel_acc_joint(_vel, _acc, _time)

    # _radius
    _radius = get_param(radius, r)
    if _radius == None:
        global _g_blend_state

        if _g_blend_state == True:
            _radius = _g_blend_radius
        else:
            _radius = 0.0

    if type(_radius) != int and type(_radius) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : radius, r")

    if _radius< 0:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : radius, r")

    # mod
    if type(mod) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : mod")

    if mod != DR_MV_MOD_ABS and mod != DR_MV_MOD_REL:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : mod")

    # ra
    if type(ra) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : ra")

    if ra != DR_MV_RA_OVERRIDE and ra != DR_MV_RA_DUPLICATE:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : ra")

    # _async
    if 1 == _async:
        _radius = 0   

    # ROS service call
    if __ROS2__: 
        #SSS srv = _ros_movej(_pos, _vel[0], _acc[0], _time, mod, _radius, ra, _async)
        req = MoveJoint.Request()
        #for i in range(6):
        #    req.pos[i]      = pos[i] 
        req.pos         = [float(x) for x in _pos]
        req.vel         = float(_vel[0])
        req.acc         = float(_acc[0])
        req.time        = float(_time)
        req.mode        = int(mod)
        req.radius      = float(_radius)
        req.blend_type  = int(ra)
        req.sync_type   = int(_async)

        #RRR ret = 0 if (srv.success == True) else -1
        future = _ros2_movej.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('movej Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                ret = 0 if (result.success == True) else -1                
    else:   
        ret = PythonMgr.py_movej(_pos, _vel, _acc, _time, _radius, mod, ra, _async)
    return ret

def movejx(pos, vel=None, acc=None, time=None, radius=None, ref=None, mod= DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE, sol=0, v=None, a=None, t=None, r=None):
    ret = _movejx(pos, vel, acc, time, radius, ref, mod, ra, sol, v, a, t, r, _async=0)
    return ret 
def amovejx(pos, vel=None, acc=None, time=None, radius=None, ref=None, mod= DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE, sol=0, v=None, a=None, t=None, r=None):
    ret = _movejx(pos, vel, acc, time, radius, ref, mod, ra, sol, v, a, t, r, _async=1)
    return ret 
def _movejx(pos, vel=None, acc=None, time=None, radius=None, ref=None, mod= DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE, sol=0, v=None, a=None, t=None, r=None, _async=0):
    # _pos
    _pos = get_posx(pos)

    # _vel
    temp = get_param(vel, v)
    if temp == None:
        _vel = _g_velj
    else:
        if type(temp) == int or type(temp) == float:
            _vel = [temp] * DR_VELJ_DT_LEN
        elif type(temp) == list and len(temp) == DR_VELJ_DT_LEN:
            _vel = temp
        else:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : vel, v")

    if is_number(_vel) != True:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")

    for item in _vel:
        if item < 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")

    # _acc
    temp = get_param(acc, a)
    if temp == None:
        _acc = _g_accj
    else:
        if type(temp) == int or type(temp) == float:
            _acc = [temp] * DR_ACCJ_DT_LEN
        elif type(temp) == list and len(temp) == DR_ACCJ_DT_LEN:
            _acc = temp
        else:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : acc, a")

    if is_number(_acc) != True:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")

    for item in _acc:
        if item < 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")

    # _time
    _time = get_param(time, t)
    if _time == None:
        _time = 0.0

    if type(_time) != int and type(_time) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : time, t")

    if _time < 0:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : time, t")

    # check vel, acc, time
    #_check_valid_vel_acc(_vel, _acc, _time)
    _check_valid_vel_acc_joint(_vel, _acc, _time)

    # _radius
    _radius = get_param(radius, r)
    if _radius == None:
        global _g_blend_state

        if _g_blend_state == True:
            _radius = _g_blend_radius
        else:
            _radius = 0.0

    if type(_radius) != int and type(_radius) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : radius, r")

    if _radius < 0:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : radius, r")

    # _ref
    if ref == None:
        _ref = _g_coord
    else:
        _ref = ref

    # check ref
    if type(_ref) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : ref")

    if _ref < DR_BASE or _ref > DR_TC_USER_MAX:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref")

    # mod
    if type(mod) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : mod")

    if mod != DR_MV_MOD_ABS and mod != DR_MV_MOD_REL:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : mod")

    # ra
    if type(ra) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type, ra")

    if ra != DR_MV_RA_OVERRIDE and ra != DR_MV_RA_DUPLICATE:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : ra")

    # sol
    if type(sol) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : sol")

    if sol < DR_SOL_MIN or sol > DR_SOL_MAX:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : sol")

    # _async
    if 1 == _async:
        _radius = 0   

    # ROS service call
    if __ROS2__: 
        #srv = _ros_movejx(_pos, _vel[0], _acc[0], _time, mod, _ref, _radius, ra, sol, _async)   
        req = MoveJointx.Request()
        req.pos         = [float(x) for x in _pos]
        req.vel         = float(_vel[0])
        req.acc         = float(_acc[0])
        req.time        = float(_time)
        req.mode        = int(mod)
        req.ref         = int(_ref)
        req.radius      = float(_radius)
        req.blend_type  = int(ra)
        req.sol         = int(sol)
        req.sync_type   = int(_async)

        #ret = srv.success
        #ret = 0 if (srv.success == True) else -1
        future = _ros2_movejx.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('movejx Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                ret = 0 if (result.success == True) else -1                
    else:    
        ret = PythonMgr.py_movejx(_pos, _vel, _acc, _time, _radius, _ref, mod, ra, sol, _async)
        print_ext_result("{0} = PythonMgr.py_movejx(pos:{1}, vel:{2}, acc:{3}, time:{4}, radius:{5}, ref{6}, mod{7}, ra:{8}, sol:{9}, async:{10})" \
                         .format(ret, _pos, dr_form(_vel), dr_form(_acc), _time, _radius, _ref, mod, ra, sol, _async))   
    return ret



def movel(pos, vel=None, acc=None, time=None, radius=None, ref=None, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE, v=None, a=None, t=None, r=None):
    ret = _movel(pos, vel, acc, time, radius, ref, mod, ra, v, a, t, r, _async=0)
    return ret
def amovel(pos, vel=None, acc=None, time=None, radius=None, ref=None, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE, v=None, a=None, t=None, r=None):
    ret = _movel(pos, vel, acc, time, radius, ref, mod, ra, v, a, t, r, _async=1)
    return ret
def _movel(pos, vel=None, acc=None, time=None, radius=None, ref=None, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE, v=None, a=None, t=None, r=None, _async=0):
    # _pos
    _pos = get_normal_pos(pos, def_type=posx)

    # _vel
    temp = get_param(vel, v)
    if temp == None:
        _vel = _g_velx
    else:
        if type(temp) == int or type(temp) == float:
            #_vel = [temp] * DR_VELX_DT_LEN 
            _vel = [0] * DR_VELX_DT_LEN     
            _vel[0] = temp
            _vel[1] = DR_COND_NONE
        elif type(temp) == list and len(temp) == DR_VELX_DT_LEN:
            _vel = temp
        else:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : vel, v")

    if is_number(_vel) != True:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")

    for item in _vel:
        #if item < 0:
        if (item < 0) and (item != DR_COND_NONE):
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")

    #print("_vel[0]={0}, _vel[1]={1}".format(_vel[0],_vel[1]))

    # _acc
    temp = get_param(acc, a)
    if temp == None:
        _acc = _g_accx
    else:
        if type(temp) == int or type(temp) == float:
            #_acc = [temp] * DR_ACCX_DT_LEN 
            _acc = [0] * DR_ACCX_DT_LEN     
            _acc[0] = temp
            _acc[1] = DR_COND_NONE
        elif type(temp) == list and len(temp) == DR_ACCX_DT_LEN:
            _acc = temp
        else:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : acc, a")

    if is_number(_acc) != True:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")

    for item in _acc:
        #if item < 0:
        if (item < 0) and (item != DR_COND_NONE):
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")

    #print("_acc[0]={0}, _acc[1]={1}".format(_acc[0],_acc[1]))

    # _time
    _time = get_param(time, t)
    if _time == None:
        _time = 0.0

    if type(_time) != int and type(_time) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : time, t")

    if _time < 0:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : time, t")

    # check vel, acc, time
    #_check_valid_vel_acc(_vel, _acc, _time)
    _check_valid_vel_acc_task(_vel, _acc, _time)

    # _radius
    _radius = get_param(radius, r)
    if _radius == None:
        global _g_blend_state

        if _g_blend_state == True:
            _radius = _g_blend_radius
        else:
            _radius = 0.0

    if type(_radius) != int and type(_radius) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : radius, r")

    if _radius < 0:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : radius, r")

    # ref
    if ref == None:
        _ref = _g_coord
    else:
        _ref = ref

    # check ref
    if type(_ref) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : ref")

    if _ref < DR_BASE or _ref > DR_TC_USER_MAX:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref")

    # mod
    if type(mod) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : mod")

    if mod != DR_MV_MOD_ABS and mod != DR_MV_MOD_REL:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : mod")

    # ra
    if type(ra) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : ra")

    if ra != DR_MV_RA_OVERRIDE and ra != DR_MV_RA_DUPLICATE:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : ra")

    # qcommand
    if type(_pos) == posx:
        qcommand = 0
    else:
        qcommand = 1

    # _async
    if 1 == _async:
        _radius = 0   

    # ROS service call
    if __ROS2__: 
        req = MoveLine.Request()

        req.pos         = [float(x) for x in _pos]
        req.vel         = [float(x) for x in _vel]
        req.acc         = [float(x) for x in _acc]
        req.time        = float(_time)
        req.radius      = float(_radius)
        req.ref         = int(_ref)
        req.mode        = int(mod)
        req.blend_type  = int(ra)
        req.sync_type   = int(_async)

        #RRR ret = 0 if (srv.success == True) else -1
        future = _ros2_movel.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('movel Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                ret = 0 if (result.success == True) else -1                
    else:    
        ret = PythonMgr.py_movel(_pos, _vel, _acc, _time, _radius, _ref, mod, ra, qcommand, _async)
        print_ext_result("{0} = PythonMgr.py_movel(pos:{1}, vel:{2}, acc:{3}, time:{4}, radius:{5}, ref:{6}, mod:{7}, ra:{8}, qcommand:{9}, async:{10})" \
                         .format(ret, _pos, dr_form(_vel), dr_form(_acc), _time, _radius, _ref, mod, ra, qcommand, _async))
    return ret



def movec(pos1, pos2, vel=None, acc=None, time=None, radius=None, ref=None, mod= DR_MV_MOD_ABS, angle=None, ra=DR_MV_RA_DUPLICATE, v=None, a=None, t=None, r=None, an=None):
    ret = _movec(pos1, pos2, vel, acc, time, radius, ref, mod, angle, ra, v, a, t, r, an, _async=0)
    return ret
def amovec(pos1, pos2, vel=None, acc=None, time=None, radius=None, ref=None, mod= DR_MV_MOD_ABS, angle=None, ra=DR_MV_RA_DUPLICATE, v=None, a=None, t=None, r=None, an=None):
    ret = _movec(pos1, pos2, vel, acc, time, radius, ref, mod, angle, ra, v, a, t, r, an, _async=1)
    return ret
def _movec(pos1, pos2, vel=None, acc=None, time=None, radius=None, ref=None, mod= DR_MV_MOD_ABS, angle=None, ra=DR_MV_RA_DUPLICATE, v=None, a=None, t=None, r=None, an=None, _async=0):

    # _pos1, _pos2
    
    _pos1 = get_normal_pos(pos1, def_type=posx)
    _pos2 = get_normal_pos(pos2, def_type=posx)

    if type(_pos1) != type(_pos2):
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : pos1, ps2")

    # _vel
    temp = get_param(vel, v)
    if temp == None:
        _vel = _g_velx
    else:
        if type(temp) == int or type(temp) == float:
            #_vel = [temp] * DR_VELX_DT_LEN #<����> ���� �� �ΰ� ����
            _vel = [0] * DR_VELX_DT_LEN     #<����> _vel[0]=temp, _vel[1]= DR_COND_NONE
            _vel[0] = temp
            _vel[1] = DR_COND_NONE
        elif type(temp) == list and len(temp) == DR_VELX_DT_LEN:
            _vel = temp
        else:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : vel, v")

    if is_number(_vel) != True:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")

    for item in _vel:
        #if item < 0:
        if (item < 0) and (item != DR_COND_NONE):
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")

    # _acc
    temp = get_param(acc, a)
    if temp == None:
        _acc = _g_accx
    else:
        if type(temp) == int or type(temp) == float:
            #_acc = [temp] * DR_ACCX_DT_LEN #<����> ���� �� �ΰ� ����
            _acc = [0] * DR_ACCX_DT_LEN     #<����> _acc[0]=temp, _acc[1]= DR_COND_NONE
            _acc[0] = temp
            _acc[1] = DR_COND_NONE
        elif type(temp) == list and len(temp) == DR_ACCX_DT_LEN:
            _acc = temp
        else:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : acc, a")

    if is_number(_acc) != True:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")

    for item in _acc:
        #if item < 0:
        if (item < 0) and (item != DR_COND_NONE):
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")

    # _time
    _time = get_param(time, t)
    if _time == None:
        _time = 0.0

    if type(_time) != int and type(_time) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : time, t")

    if _time < 0:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : time, t")

    # check vel, acc, time
    #_check_valid_vel_acc(_vel, _acc, _time)
    _check_valid_vel_acc_task(_vel, _acc, _time)

    # _radius
    _radius = get_param(radius, r)
    if _radius == None:
        global _g_blend_state

        if _g_blend_state == True:
            _radius = _g_blend_radius
        else:
            _radius = 0.0

    if type(_radius) != int and type(_radius) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : radius, r")

    if _radius < 0:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : radius, r")

    # _ref
    if ref == None:
        _ref = _g_coord
    else:
        _ref = ref

    # check ref
    if type(_ref) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : ref")

    if _ref < DR_BASE or _ref > DR_TC_USER_MAX:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref")

    # mod
    if type(mod) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : mod")

    if mod != DR_MV_MOD_ABS and mod != DR_MV_MOD_REL:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : mod")

    # _angle
    temp = get_param(angle, an)
    if temp == None:
        _angle = [0, 0]
    else:
        if type(temp) == int or type(temp) == float:
            _angle = [temp, 0]
        elif type(temp) == list and len(temp) == DR_ANGLE_DT_LEN:
            _angle = temp
        else:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : angle, an")

    if is_number(_angle) != True:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : angle, an")

    for item in _angle:
        if item < 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : angle, an")

    # ra
    if type(ra) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : ra")

    if ra != DR_MV_RA_OVERRIDE and ra != DR_MV_RA_DUPLICATE:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : ra")

    # qcommand
    if type(_pos1) == posx:
        qcommand = 0
    else:
        qcommand = 1

    # _async
    if 1 == _async:
        _radius = 0   

    # ROS service call
    if __ROS2__:
        # make multi pos
        _circle_pos = _ros_listToFloat64MultiArray([_pos1, _pos2])
        #_circle_pos = _ros_listToFloat64MultiArray([ [float(x) for x in _pos1], [float(x) for x in _pos2] ])
        #print(_circle_pos)

        #srv = _ros_movec(_circle_pos, _vel, _acc, _time, mod, _ref, _angle[0], _angle[1], _radius, ra, _async) 
        req = MoveCircle.Request()
        req.pos         = _circle_pos               #??? 검증필요!
        req.vel         = [float(x) for x in _vel]
        req.acc         = [float(x) for x in _acc]
        req.time        = float(_time)
        req.radius      = float(_radius)
        req.ref         = int(_ref)
        req.mode        = int(mod)
        req.angle1      = float(_angle[0])
        req.angle2      = float(_angle[1])
        req.blend_type  = int(ra)
        req.sync_type   = int(_async)

        #RRR ret = 0 if (srv.success == True) else -1
        future = _ros2_movec.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('movec Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                ret = 0 if (result.success == True) else -1                
    else:   
        ret = PythonMgr.py_movec(_pos1, _pos2, _vel, _acc, _time, _radius, _ref, mod, _angle, ra, qcommand, _async)
        print_ext_result("{0} = PythonMgr.py_movec(pos1:{1}, pos2:{2}, vel:{3}, acc:{4}, time:{5}, radius:{6}, ref:{7}, mod:{8}, angle:{9}, ra:{10}, qcommand:{11}, async:{12})" \
                         .format(ret, _pos1, _pos2, dr_form(_vel), dr_form(_acc), _time, _radius, _ref, mod, dr_form(_angle), ra, qcommand, _async))
    return ret


def movesj(pos_list, vel=None, acc=None, time=None, mod= DR_MV_MOD_ABS, v=None, a=None, t=None):
    ret = _movesj(pos_list, vel, acc, time, mod, v, a, t, _async=0)
    return ret
def amovesj(pos_list, vel=None, acc=None, time=None, mod= DR_MV_MOD_ABS, v=None, a=None, t=None):
    ret = _movesj(pos_list, vel, acc, time, mod, v, a, t, _async=1)
    return ret
def _movesj(pos_list, vel=None, acc=None, time=None, mod= DR_MV_MOD_ABS, v=None, a=None, t=None, _async=0):
    # pos_list
    if type(pos_list) != list:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : pos_list")

    for item in pos_list:
        if type(item) != posj:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : pos_list [item]")

    # _vel
    temp = get_param(vel, v)
    if temp == None:
        _vel = _g_velj
    else:
        if type(temp) == int or type(temp) == float:
            _vel = [temp] * DR_VELJ_DT_LEN
        elif type(temp) == list and len(temp) == DR_VELJ_DT_LEN:
            _vel = temp
        else:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : vel, v")

    if is_number(_vel) != True:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")

    for item in _vel:
        if item < 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")

    # _acc
    temp = get_param(acc, a)
    if temp == None:
        _acc = _g_accj
    else:
        if type(temp) == int or type(temp) == float:
            _acc = [temp] * DR_ACCJ_DT_LEN
        elif type(temp) == list and len(temp) == DR_ACCJ_DT_LEN:
            _acc = temp
        else:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : acc, a")

    if is_number(_acc) != True:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")

    for item in _acc:
        if item < 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")

    # _time
    _time = get_param(time, t)
    if _time == None:
        _time = 0.0

    if type(_time) != int and type(_time) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : time, t")

    if _time < 0:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : time, t")

    # check vel, acc, time
    #_check_valid_vel_acc(_vel, _acc, _time)
    _check_valid_vel_acc_joint(_vel, _acc, _time)

    # mod
    if type(mod) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : mod")

    if mod != DR_MV_MOD_ABS and mod != DR_MV_MOD_REL:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : mod")

    # _async
    if 1 == _async:
        _radius = 0   

    # ROS service call
    if __ROS2__:
        # make multi pos
        _spline_posj = _ros_listToFloat64MultiArray(pos_list)
        #srv = _ros_movesj(_spline_posj, len(_spline_posj), _vel, _acc, _time, mod, _async)

        req = MoveSplineJoint.Request()

        req.pos         = _spline_posj               #??? 검증필요!
        req.pos_cnt     = len(_spline_posj)

        if type(_vel) == list:
            _vel = float(_vel[0]) 
        if type(_acc) == list:
            _acc = float(_acc[0]) 
        req.vel         = _vel #[float(x) for x in _vel]
        req.acc         = _acc #[float(x) for x in _acc]

        req.time        = float(_time)
        req.mode        = int(mod)
        req.sync_type   = int(_async)

        #RRR ret = 0 if (srv.success == True) else -1
        future = _ros2_movesj.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('movesj Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                ret = 0 if (result.success == True) else -1                
    else:    
        ret = PythonMgr.py_movesj(pos_list, _vel, _acc, _time, mod, _async)
        print_ext_result("{0} = PythonMgr.py_movesj(pos_list:{1}, vel:{2}, acc:{3}, time:{4}, mod:{5} async:{6})" \
                         .format(ret, dr_form(pos_list), dr_form(_vel), dr_form(_acc), _time, mod, _async))
    return ret



def movesx(pos_list, vel=None, acc=None, time=None, ref=None, mod= DR_MV_MOD_ABS, vel_opt=DR_MVS_VEL_NONE, v=None, a=None, t=None):
    ret = _movesx(pos_list, vel, acc, time, ref, mod, vel_opt, v, a, t, _async=0)
    return ret
def amovesx(pos_list, vel=None, acc=None, time=None, ref=None, mod= DR_MV_MOD_ABS, vel_opt=DR_MVS_VEL_NONE, v=None, a=None, t=None):
    ret = _movesx(pos_list, vel, acc, time, ref, mod, vel_opt, v, a, t, _async=1)
    return ret
def _movesx(pos_list, vel=None, acc=None, time=None, ref=None, mod= DR_MV_MOD_ABS, vel_opt=DR_MVS_VEL_NONE, v=None, a=None, t=None, _async=0):
    # pos_list
    if type(pos_list) != list:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : pos_list")

    for item in pos_list:
        if type(item) != posx:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : pos_list [item]")

    # _vel
    temp = get_param(vel, v)
    if temp == None:
        _vel = _g_velx
    else:
        if type(temp) == int or type(temp) == float:
            #_vel = [temp] * DR_VELX_DT_LEN #<����> ���� �� �ΰ� ����
            _vel = [0] * DR_VELX_DT_LEN     #<����> _vel[0]=temp, _vel[1]= DR_COND_NONE
            _vel[0] = temp
            _vel[1] = DR_COND_NONE
        elif type(temp) == list and len(temp) == DR_VELX_DT_LEN:
            _vel = temp
        else:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : vel, v")

    if is_number(_vel) != True:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")

    for item in _vel:
        #if item < 0:
        if (item < 0) and (item != DR_COND_NONE):
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")

    # _acc
    temp = get_param(acc, a)
    if temp == None:
        _acc = _g_accx
    else:
        if type(temp) == int or type(temp) == float:
            #_acc = [temp] * DR_ACCX_DT_LEN #<����> ���� �� �ΰ� ����
            _acc = [0] * DR_ACCX_DT_LEN     #<����> _acc[0]=temp, _acc[1]= DR_COND_NONE
            _acc[0] = temp
            _acc[1] = DR_COND_NONE
        elif type(temp) == list and len(temp) == DR_ACCX_DT_LEN:
            _acc = temp
        else:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : acc, a")

    if is_number(_acc) != True:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")

    for item in _acc:
        #if item < 0:
        if (item < 0) and (item != DR_COND_NONE):
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")

    # _time
    _time = get_param(time, t)
    if _time == None:
        _time = 0.0

    if type(_time) != int and type(_time) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : time, t")

    if _time < 0:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : time, t")

    # check vel, acc, time
    #_check_valid_vel_acc(_vel, _acc, _time)
    _check_valid_vel_acc_task(_vel, _acc, _time)

    # _ref
    if ref == None:
        _ref = _g_coord
    else:
        _ref = ref

    # check ref
    if type(_ref) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : ref")

    if _ref < DR_BASE or _ref > DR_TC_USER_MAX:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref")

    # mod
    if type(mod) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : mod")

    if mod != DR_MV_MOD_ABS and mod != DR_MV_MOD_REL:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : mod")

    # vel_opt
    if type(vel_opt) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : vel_opt")

    if vel_opt != DR_MVS_VEL_NONE and vel_opt != DR_MVS_VEL_CONST:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel_opt")

    # _async
    if 1 == _async:
        _radius = 0   

    # ROS service call
    if __ROS2__:
        # make multi pos
        _spline_posx = _ros_listToFloat64MultiArray(pos_list)
        #srv = _ros_movesx(_spline_posx, len(_spline_posx), _vel, _acc, _time, mod, _ref, vel_opt, _async)
        req = MoveSplineTask.Request()
        req.pos         = _spline_posx               #??? 검증필요!
        req.pos_cnt     = len(_spline_posx)
        req.vel         = [float(x) for x in _vel]
        req.acc         = [float(x) for x in _acc]
        req.time        = float(_time)
        req.ref         = int(_ref)
        req.mode        = int(mod)
        req.opt         = int(vel_opt)
        req.sync_type   = int(_async)

        #RRR ret = 0 if (srv.success == True) else -1
        future = _ros2_movesx.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('movesx Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                ret = 0 if (result.success == True) else -1                
    else:
        ret = PythonMgr.py_movesx(pos_list, _vel, _acc, _time, _ref, mod, vel_opt, _async)
        print_ext_result("{0} = PythonMgr.py_movesx(pos_list:{1}, vel:{2}, acc:{3}, time:{4}, ref:{5}, mod:{6}, vel_opt:{7}, async:{8})" \
                        .format(ret, dr_form(pos_list), dr_form(_vel), dr_form(_acc), _time, _ref, mod, vel_opt, _async))
    return ret

def moveb(seg_list, vel=None, acc=None, ref=None, time=None, mod=DR_MV_MOD_ABS, v=None, a=None, t=None):
    ret = _moveb(seg_list, vel, acc, ref, time, mod, v, a, t, _async=0)
    return ret
def amoveb(seg_list, vel=None, acc=None, ref=None, time=None, mod=DR_MV_MOD_ABS, v=None, a=None, t=None):
    ret = _moveb(seg_list, vel, acc, ref, time, mod, v, a, t, _async=1)
    return ret
def _moveb(seg_list, vel=None, acc=None, ref=None, time=None, mod=DR_MV_MOD_ABS, v=None, a=None, t=None, _async=0):

    # seg_list
    if type(seg_list) != list:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : seg_list")

    if len(seg_list) == 0:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : seg_list")

    _seg_list = []
    if __ROS2__:
        for seg in seg_list:
            _seg_list.append(seg.to_list())
    else:
        for seg in seg_list:
            if type(seg) != posb:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : seg_list(item)")
            else:
                _seg_list.append(seg.to_list())

    if __ROS2__:
        print(len(_seg_list))
        _ros_seg_list = []
        _tmp_list = []

        for s in range(0, len(_seg_list)):
            #print(s)
            for elemnt in range(0, len(_seg_list[s])):
                if _seg_list[s][elemnt] == None:
                    _seg_list[s][elemnt] = [0.0]*POINT_COUNT
         
            # make [pos1] + [pos2] + [type] + [radius]
            _tmp_list = _seg_list[s][1] + _seg_list[s][2] + [_seg_list[s][0]] + [_seg_list[s][3]]
            _ros_seg_list.append(_tmp_list)
            
    # _vel
    temp = get_param(vel, v)
    if temp == None:
        _vel = _g_velx
    else:
        if type(temp) == int or type(temp) == float:
            #_vel = [temp] * DR_VELX_DT_LEN #<����> ���� �� �ΰ� ����
            _vel = [0] * DR_VELX_DT_LEN     #<����> _vel[0]=temp, _vel[1]= DR_COND_NONE
            _vel[0] = temp
            _vel[1] = DR_COND_NONE
        elif type(temp) == list and len(temp) == DR_VELX_DT_LEN:
            _vel = temp
        else:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : vel, v")

    if is_number(_vel) != True:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")

    for item in _vel:
        #if item < 0:
        if (item < 0) and (item != DR_COND_NONE):
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")

    # _acc
    temp = get_param(acc, a)
    if temp == None:
        _acc = _g_accx
    else:
        if type(temp) == int or type(temp) == float:
            #_acc = [temp] * DR_ACCX_DT_LEN #<����> ���� �� �ΰ� ����
            _acc = [0] * DR_ACCX_DT_LEN     #<����> _acc[0]=temp, _acc[1]= DR_COND_NONE
            _acc[0] = temp
            _acc[1] = DR_COND_NONE
        elif type(temp) == list and len(temp) == DR_ACCX_DT_LEN:
            _acc = temp
        else:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : acc, a")

    if is_number(_acc) != True:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")

    for item in _acc:
        #if item < 0:
        if (item < 0) and (item != DR_COND_NONE):
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")

    # _time
    _time = get_param(time, t)
    if _time == None:
        _time = 0.0

    # check vel, acc, time
    #_check_valid_vel_acc(_vel, _acc, _time)
    _check_valid_vel_acc_task(_vel, _acc, _time)

    # ref
    if ref == None:
        _ref = _g_coord
    else:
        _ref = ref

    # check ref
    if type(_ref) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : ref")

    if _ref < DR_BASE or _ref > DR_TC_USER_MAX:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref")

    # mod
    if type(mod) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : mod")

    if mod != DR_MV_MOD_ABS and mod != DR_MV_MOD_REL:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : mod")

    # _async
    if 1 == _async:
        _radius = 0   

    # ROS Service call
    if __ROS2__:
        seg = _ros_listToFloat64MultiArray(_ros_seg_list)
        #srv = _ros_moveb(seg, len(_ros_seg_list), _vel, _acc, _time, mod, _ref, _async)    
        req = MoveBlending.Request()
        req.segment     = seg               #??? 검증필요!
        req.pos_cnt     = len(_ros_seg_list)
        req.vel         = [float(x) for x in _vel]
        req.acc         = [float(x) for x in _acc]
        req.time        = float(_time)
        req.ref         = int(_ref)
        req.mode        = int(mod)
        req.sync_type   = int(_async)

        #RRR ret = 0 if (srv.success == True) else -1
        future = _ros2_moveb.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('moveb Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                ret = 0 if (result.success == True) else -1                
    else:
        ret = PythonMgr.py_moveb(_seg_list, _vel, _acc, _time, _ref, mod, _async)
        print_ext_result("{0} = PythonMgr.py_moveb(seg_list:{1}, vel:{2}, acc:{3}, time:{4}, ref:{5}, mod:{6}, async:{7})" \
                         .format(ret, dr_form(_seg_list), dr_form(_vel), dr_form(_acc), _time, _ref, mod, _async))
    return ret


def move_spiral(rev=10, rmax=10, lmax=0, vel=None, acc=None, time=None, axis=DR_AXIS_Z, ref=DR_TOOL, v=None, a=None, t=None):
    ret = _move_spiral(rev, rmax, lmax, vel, acc, time, axis, ref, v, a, t, _async=0)
    return ret
def amove_spiral(rev=10, rmax=10, lmax=0, vel=None, acc=None, time=None, axis=DR_AXIS_Z, ref=DR_TOOL, v=None, a=None, t=None):
    ret = _move_spiral(rev, rmax, lmax, vel, acc, time, axis, ref, v, a, t, _async=1)
    return ret
def _move_spiral(rev=10, rmax=10, lmax=0, vel=None, acc=None, time=None, axis=DR_AXIS_Z, ref=DR_TOOL, v=None, a=None, t=None, _async=0):
    # rev
    if type(rev) != int and type(rev) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : rev")

    if rev <= 0:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : rev (Ranges: rev > 0)")

    # rmax
    if type(rmax) != int and type(rmax) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : rmax")

    if rmax <= 0:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value: rmax (Ranges: rmax > 0)")

    # lmax
    if type(lmax) != int and type(lmax) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : lmax")

    # _vel
    temp = get_param(vel, v)
    if temp == None:
        _vel = _g_velx
    else:
        if type(temp) == int or type(temp) == float:
            _vel = [temp] * DR_VELX_DT_LEN
        elif type(temp) == list and len(temp) == DR_VELX_DT_LEN:
            _vel = temp
        else:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : vel, v")

    if is_number(_vel) != True:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")

    for item in _vel:
        if item < 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")

    # _acc
    temp = get_param(acc, a)
    if temp == None:
        _acc = _g_accx
    else:
        if type(temp) == int or type(temp) == float:
            _acc = [temp] * DR_ACCX_DT_LEN
        elif type(temp) == list and len(temp) == DR_ACCX_DT_LEN:
            _acc = temp
        else:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : acc, a")

    if is_number(_acc) != True:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")

    for item in _acc:
        if item < 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")

    # _time
    _time = get_param(time, t)
    if _time == None:
        _time = 0.0

    if type(_time) != int and type(_time) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : time, t")

    if _time < 0:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : time, t")

    # check vel, acc, time
    #_check_valid_vel_acc(_vel, _acc, _time)
    _check_valid_vel_acc_joint(_vel, _acc, _time)

    # axis
    if type(axis) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : axis")

    if axis != DR_AXIS_X and axis != DR_AXIS_Y and axis != DR_AXIS_Z:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : axis")

    # ref
    if type(ref) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : ref")

    if ref != DR_BASE and ref != DR_TOOL and (ref < DR_TC_USER_MIN or ref > DR_TC_USER_MAX):
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref")

    # _async
    if 1 == _async:
        _radius = 0   

    # ROS service call
    if __ROS2__:
        #srv = _ros_move_spiral(rev, rmax, lmax, _vel, _acc, _time, axis, ref, _async)
        req = MoveSpiral.Request()
        req.revolution  = rev               #??? 검증필요!
        req.max_radius  = float(rmax)
        req.max_length  = float(lmax)
        req.vel         = [float(x) for x in _vel]
        req.acc         = [float(x) for x in _acc]
        req.time        = float(_time)
        req.task_axis   = int(axis)
        req.ref         = int(ref)
        req.sync_type   = int(_async)

        #RRR ret = 0 if (srv.success == True) else -1
        future = _ros2_move_spiral.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('move_spiral Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                ret = 0 if (result.success == True) else -1     
    else:    
        ret = PythonMgr.py_move_spiral(rev, rmax, lmax, _vel, _acc, _time, axis, ref, _async)
        print_ext_result("{0} = PythonMgr.py_move_spiral(rev:{1}, rmax:{2}, lmax:{3}, vel:{4}, acc:{5}, time:{6}, axis:{7}, ref:{8}, async:{9})" \
                         .format(ret, dr_form(rev), dr_form(rmax), dr_form(lmax), dr_form(_vel), dr_form(_acc), _time, axis, ref, _async))    
    return ret



def move_periodic(amp, period, atime=None, repeat=None, ref=DR_TOOL):
    ret = _move_periodic(amp, period, atime, repeat, ref, _async=0)
    return ret
def amove_periodic(amp, period, atime=None, repeat=None, ref=DR_TOOL):
    ret = _move_periodic(amp, period, atime, repeat, ref, _async=1)
    return ret
def _move_periodic(amp, period, atime=None, repeat=None, ref=DR_TOOL, _async=0):
    _amp = [POINT_COUNT]
    _period = [POINT_COUNT]
    _atime =0.0
    _repeat =0.0
    _ref=0 

    # amp : float[6] 
    if type(amp) != list:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : amp")
    if len(amp) != POINT_COUNT:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : amp")
    _amp =amp

    # period : float or float[6] 
    if (type(period) != int) and (type(period) != float) and (type(period) != list):
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : period")

    if (type(period) == int) or (type(period) == float):
        _period = [period] * POINT_COUNT
    else: #list �� ��� 
        if len(period) != POINT_COUNT:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : period")
        _period = period

    # atime 
    if atime == None:
        _atime = 0.0
    else:
        if (type(atime) != int) and (type(atime) != float):
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : atime")
        if atime < 0.0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : atime")
        _atime = atime

    # repeat 
    if repeat == None:
        _repeat = 1
    else:
        if (type(repeat) != int) and (type(repeat) != float): 
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : repeat")
        if repeat < 0.0: 
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : repeat")
        _repeat = repeat

    # ref
    if ref == None:
        #_ref = _g_coord
        _ref = DR_TOOL
    else:
        if type(ref) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : ref")

        if ref < DR_BASE or ref > DR_TC_USER_MAX:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref")
        _ref = ref

    # _async
    if 1 == _async:
        _radius = 0   

    # ROS service call
    if __ROS2__:
        #srv = _ros_move_periodic(_amp, _period, _atime, _repeat, _ref, _async)    
        req = MovePeriodic.Request()
        req.amp         = [float(x) for x in _amp]
        req.periodic    = [float(x) for x in _period]
        req.acc         = float(_atime)
        req.repeat      = int(_repeat)
        req.ref         = int(_ref)
        req.sync_type   = int(_async)

        #RRR ret = 0 if (srv.success == True) else -1
        future = _ros2_move_periodic.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

        try:
            result = future.result()
        except Exception as e:
            g_node.get_logger().info('move_periodic Service call failed %r' % (e,))
        else:
            if result == None:
                ret = -1    
            else:        
                ret = 0 if (result.success == True) else -1     

    else:
        ret = PythonMgr.py_move_periodic(_amp, _period, _atime, _repeat, _ref, _async)
        print_ext_result("{0} = PythonMgr.move_periodic(amp:{1}, period:{2}, atime:{3}, repeat{4}, ref:{5}, async:{6})" \
                         .format(ret, dr_form(_amp), dr_form(_period), _atime, _repeat, _ref, _async))
    return ret






########################################################################################################################################
########################################################################################################################################
########################################################################################################################################
class CDsrRobot:
    def __init__(self, robot_id='dsr01', robot_model='m1013'):
        global g_node    
        self._robot_id = robot_id
        self._robot_model = robot_model
        print("CDsrRobot(): self._robot_id = %s, self._robot_model = %s" %(self._robot_id, self._robot_model))

        self._srv_name_prefix   = '' #'/' + self._robot_id + self._robot_model #ROS2


        ############### connect to dsr_control2 (ros service) ####################################################################### 
        #rospy.wait_for_service(self._srv_name_prefix +"/motion/move_joint")
        
        # system Operations
        self._ros2_set_robot_mode             = g_node.create_client(SetRobotMode, self._srv_name_prefix +"/system/set_robot_mode") ; self.req_SetRobotMode = SetRobotMode.Request()
        self._ros2_get_robot_mode             = g_node.create_client(GetRobotMode, self._srv_name_prefix +"/system/get_robot_mode") ; self.req_GetRobotMode = GetRobotMode.Request()
        self._ros2_set_robot_system           = g_node.create_client(SetRobotSystem, self._srv_name_prefix +"/system/set_robot_system") ; self.req_SetRobotSystem = SetRobotSystem.Request()
        self._ros2_get_robot_system           = g_node.create_client(GetRobotSystem, self._srv_name_prefix +"/system/get_robot_system") ; self.req_GetRobotSystem = GetRobotSystem.Request()
        self._ros2_get_robot_state            = g_node.create_client(GetRobotState, self._srv_name_prefix +"/system/get_robot_state") ; self.req_GetRobotState = GetRobotState.Request()
        self._ros2_set_robot_speed_mode       = g_node.create_client(SetRobotSpeedMode, self._srv_name_prefix +"/system/set_robot_speed_mode") ; self.req_SetRobotSpeedMode = SetRobotSpeedMode.Request()
        self._ros2_get_robot_speed_mode       = g_node.create_client(GetRobotSpeedMode, self._srv_name_prefix +"/system/get_robot_speed_mode") ; self.req_GetRobotSpeedMode = GetRobotSpeedMode.Request()
        self._ros2_set_safe_stop_reset_type   = g_node.create_client(SetSafeStopResetType, self._srv_name_prefix +"/system/set_safe_stop_reset_type") ; self.req_SetSafeStopResetType = SetSafeStopResetType.Request()
        self._ros2_get_last_alarm             = g_node.create_client(GetLastAlarm, self._srv_name_prefix +"/system/get_last_alarm") ; self.req_GetLastAlarm = GetLastAlarm.Request()
        self._ros2_get_current_pose           = g_node.create_client(GetCurrentPose, self._srv_name_prefix +"/system/get_current_pose") ; self.req_GetCurrentPose = GetCurrentPose.Request()

    
        #  motion Operations
        self._ros2_movej                      = g_node.create_client(MoveJoint,              self._srv_name_prefix +"/motion/move_joint") ; self.req_MoveJoint = MoveJoint.Request()
        self._ros2_movel                      = g_node.create_client(MoveLine,               self._srv_name_prefix +"/motion/move_line") ; self.req_MoveLine = MoveLine.Request()
        self._ros2_movejx                     = g_node.create_client(MoveJointx,             self._srv_name_prefix +"/motion/move_jointx") ; self.req_MoveJointx = MoveJointx.Request()
        self._ros2_movec                      = g_node.create_client(MoveCircle,             self._srv_name_prefix +"/motion/move_circle") ; self.req_MoveCircle = MoveCircle.Request()
        self._ros2_movesj                     = g_node.create_client(MoveSplineJoint,        self._srv_name_prefix +"/motion/move_spline_joint") ; self.req_MoveSplineJoint = MoveSplineJoint.Request() 
        self._ros2_movesx                     = g_node.create_client(MoveSplineTask,         self._srv_name_prefix +"/motion/move_spline_task") ; self.req_MoveSplineTask = MoveSplineTask.Request()
        self._ros2_moveb                      = g_node.create_client(MoveBlending,           self._srv_name_prefix +"/motion/move_blending") ; self.req_MoveBlending = MoveBlending.Request()
        self._ros2_move_spiral                = g_node.create_client(MoveSpiral,             self._srv_name_prefix +"/motion/move_spiral") ; self.req_MoveSpiral = MoveSpiral.Request()
        self._ros2_move_periodic              = g_node.create_client(MovePeriodic,           self._srv_name_prefix +"/motion/move_periodic") ; self.req_MovePeriodic = MovePeriodic.Request()
        self._ros2_move_wait                  = g_node.create_client(MoveWait,               self._srv_name_prefix +"/motion/move_wait") ; self.req_MoveWait = MoveWait.Request()
        self._ros2_jog                        = g_node.create_client(Jog,                    self._srv_name_prefix +"/motion/jog") ; self.req_Jog = Jog.Request()
        self._ros2_jog_multi                  = g_node.create_client(JogMulti,               self._srv_name_prefix +"/motion/jog_multi") ; self.req_JogMulti = JogMulti.Request()
        self._ros2_trans                      = g_node.create_client(Trans,                  self._srv_name_prefix +"/motion/trans") ; self.req_Trans = Trans.Request()
        self._ros2_fkin                       = g_node.create_client(Fkin,                   self._srv_name_prefix +"/motion/fkin") ; self.req_Fkin = Fkin.Request()
        self._ros2_ikin                       = g_node.create_client(Ikin,                   self._srv_name_prefix +"/motion/ikin") ; self.req_SetRefCoord = Ikin.Request()
        self._ros2_set_ref_coord              = g_node.create_client(SetRefCoord,            self._srv_name_prefix +"/motion/set_ref_coord") ; self.req_SetRefCoord = SetRefCoord.Request()
        self._ros2_move_home                  = g_node.create_client(MoveHome,               self._srv_name_prefix +"/motion/move_home") ; self.req_MoveHome = MoveHome.Request()
        self._ros2_check_motion               = g_node.create_client(CheckMotion,            self._srv_name_prefix +"/motion/check_motion") ; self.req_CheckMotion = CheckMotion.Request()
        self._ros2_change_operation_speed     = g_node.create_client(ChangeOperationSpeed,   self._srv_name_prefix +"/motion/change_operation_speed") ; self.req_ChangeOperationSpeed = ChangeOperationSpeed.Request()
        self._ros2_enable_alter_motion        = g_node.create_client(EnableAlterMotion,      self._srv_name_prefix +"/motion/enable_alter_motion") ; self.req_EnableAlterMotion = EnableAlterMotion.Request()
        self._ros2_alter_motion               = g_node.create_client(AlterMotion,            self._srv_name_prefix +"/motion/alter_motion") ; self.req_AlterMotion = AlterMotion.Request()
        self._ros2_disable_alter_motion       = g_node.create_client(DisableAlterMotion,     self._srv_name_prefix +"/motion/disable_alter_motion") ; self.req_DisableAlterMotion = DisableAlterMotion.Request()


        # Auxiliary Control Operations
        self._ros2_get_control_mode               = g_node.create_client(GetControlMode,     self._srv_name_prefix +"/aux_control/get_control_mode") ; self.req_GetControlMode = GetControlMode.Request()
        self._ros2_get_control_space              = g_node.create_client(GetControlSpace,    self._srv_name_prefix +"/aux_control/get_control_space") ; self.req_GetControlSpace = GetControlSpace.Request()
    
        self._ros2_get_current_posj               = g_node.create_client(GetCurrentPosj,     self._srv_name_prefix +"/aux_control/get_current_posj") ; self.req_GetCurrentPosj = GetCurrentPosj.Request()
        self._ros2_get_current_velj               = g_node.create_client(GetCurrentVelj,     self._srv_name_prefix +"/aux_control/get_current_velj") ; self.req_GetCurrentVelj = GetCurrentVelj.Request()
        self._ros2_get_desired_posj               = g_node.create_client(GetDesiredPosj,     self._srv_name_prefix +"/aux_control/get_desired_posj") ; self.req_GetDesiredPosj = GetDesiredPosj.Request()
        self._ros2_get_desired_velj               = g_node.create_client(GetDesiredVelj,     self._srv_name_prefix +"/aux_control/get_desired_velj") ; self.req_GetDesiredVelj = GetDesiredVelj.Request()
    
        self._ros2_get_current_posx               = g_node.create_client(GetCurrentPosx,     self._srv_name_prefix +"/aux_control/get_current_posx") ; self.req_GetCurrentPosx = GetCurrentPosx.Request()
        self._ros2_get_current_velx               = g_node.create_client(GetCurrentVelx,     self._srv_name_prefix +"/aux_control/get_current_velx") ; self.req_GetCurrentVelx = GetCurrentVelx.Request()
        self._ros2_get_desired_posx               = g_node.create_client(GetDesiredPosx,     self._srv_name_prefix +"/aux_control/get_desired_posx") ; self.req_GetDesiredPosx = GetDesiredPosx.Request()
        self._ros2_get_desired_velx               = g_node.create_client(GetDesiredVelx,     self._srv_name_prefix +"/aux_control/get_desired_velx") ; self.req_GetDesiredVelx = GetDesiredVelx.Request()
     
        self._ros2_get_current_tool_flange_posx   = g_node.create_client(GetCurrentToolFlangePosx,   self._srv_name_prefix +"/aux_control/get_current_tool_flange_posx") ; self.req_GetCurrentToolFlangePosx = GetCurrentToolFlangePosx.Request()
     
        self._ros2_get_current_solution_space     = g_node.create_client(GetCurrentSolutionSpace,    self._srv_name_prefix +"/aux_control/get_current_solution_space") ; self.req_GetCurrentSolutionSpace = GetCurrentSolutionSpace.Request()
        self._ros2_get_current_rotm               = g_node.create_client(GetCurrentRotm,             self._srv_name_prefix +"/aux_control/get_current_rotm") ; self.req_GetCurrentRotm = GetCurrentRotm.Request()
        self._ros2_get_joint_torque               = g_node.create_client(GetJointTorque,             self._srv_name_prefix +"/aux_control/get_joint_torque") ; self.req_GetJointTorque = GetJointTorque.Request()
        self._ros2_get_external_torque            = g_node.create_client(GetExternalTorque,          self._srv_name_prefix +"/aux_control/get_external_torque") ; self.req_GetExternalTorque = GetExternalTorque.Request()
        self._ros2_get_tool_force                 = g_node.create_client(GetToolForce,               self._srv_name_prefix +"/aux_control/get_tool_force") ; self.req_GetToolForce = GetToolForce.Request()
        self._ros2_get_solution_space             = g_node.create_client(GetSolutionSpace,           self._srv_name_prefix +"/aux_control/get_solution_space") ; self.req_GetSolutionSpace = GetSolutionSpace.Request()
        self._ros2_get_orientation_error          = g_node.create_client(GetOrientationError,        self._srv_name_prefix +"/aux_control/get_orientation_error") ; self.req_GetOrientationError = GetOrientationError.Request()

        # Force/Stiffness Control & others Operations
        self._ros2_get_workpiece_weight        = g_node.create_client(GetWorkpieceWeight,            self._srv_name_prefix +"/force/get_workpiece_weight") ; self.req_GetWorkpieceWeight = GetWorkpieceWeight.Request()
        self._ros2_reset_workpiece_weight      = g_node.create_client(ResetWorkpieceWeight,          self._srv_name_prefix +"/force/reset_workpiece_weight") ; self.req_ResetWorkpieceWeight = ResetWorkpieceWeight.Request()
        self._ros2_set_singularity_handling    = g_node.create_client(SetSingularityHandling,        self._srv_name_prefix +"/motion/set_singularity_handling") ; self.req_SetSingularityHandling = SetSingularityHandling.Request()

        self._ros2_parallel_axis1              = g_node.create_client(ParallelAxis1,                 self._srv_name_prefix +"/force/parallel_axis1") ; self.req_ParallelAxis1 = ParallelAxis1.Request()
        self._ros2_parallel_axis2              = g_node.create_client(ParallelAxis2,                 self._srv_name_prefix +"/force/parallel_axis2") ; self.req_ParallelAxis2 = ParallelAxis2.Request()
        self._ros2_align_axis1                 = g_node.create_client(AlignAxis1,                    self._srv_name_prefix +"/force/align_axis1") ; self.req_AlignAxis1 = AlignAxis1.Request()
        self._ros2_align_axis2                 = g_node.create_client(AlignAxis2,                    self._srv_name_prefix +"/force/align_axis2") ; self.req_AlignAxis2 = AlignAxis2.Request()
        self._ros2_is_done_bolt_tightening     = g_node.create_client(IsDoneBoltTightening,          self._srv_name_prefix +"/force/is_done_bolt_tightening") ; self.req_IsDoneBoltTightening = IsDoneBoltTightening.Request()
        self._ros2_release_compliance_ctrl     = g_node.create_client(ReleaseComplianceCtrl,         self._srv_name_prefix +"/force/release_compliance_ctrl") ; self.req_ReleaseComplianceCtrl = ReleaseComplianceCtrl.Request()
        self._ros2_task_compliance_ctrl        = g_node.create_client(TaskComplianceCtrl,            self._srv_name_prefix +"/force/task_compliance_ctrl") ; self.req_TaskComplianceCtrl = TaskComplianceCtrl.Request()
        self._ros2_set_stiffnessx              = g_node.create_client(SetStiffnessx,                 self._srv_name_prefix +"/force/set_stiffnessx") ; self.req_SetStiffnessx = SetStiffnessx.Request()
        self._ros2_calc_coord                  = g_node.create_client(CalcCoord,                     self._srv_name_prefix +"/force/calc_coord") ; self.req_CalcCoord = CalcCoord.Request()
        self._ros2_set_user_cart_coord1        = g_node.create_client(SetUserCartCoord1,             self._srv_name_prefix +"/force/set_user_cart_coord1") ; self.req_SetUserCartCoord1 = SetUserCartCoord1.Request()
        self._ros2_set_user_cart_coord2        = g_node.create_client(SetUserCartCoord2,             self._srv_name_prefix +"/force/set_user_cart_coord2") ; self.req_SetUserCartCoord2 = SetUserCartCoord2.Request()
        self._ros2_set_user_cart_coord3        = g_node.create_client(SetUserCartCoord3,             self._srv_name_prefix +"/force/set_user_cart_coord3") ; self.req_SetUserCartCoord3 = SetUserCartCoord3.Request()
        self._ros2_overwrite_user_cart_coord   = g_node.create_client(OverwriteUserCartCoord,        self._srv_name_prefix +"/force/overwrite_user_cart_coord") ; self.req_OverwriteUserCartCoord = OverwriteUserCartCoord.Request()
        self._ros2_get_user_cart_coord         = g_node.create_client(GetUserCartCoord,              self._srv_name_prefix +"/force/get_user_cart_coord") ; self.req_GetUserCartCoord = GetUserCartCoord.Request()
        self._ros2_set_desired_force           = g_node.create_client(SetDesiredForce,               self._srv_name_prefix +"/force/set_desired_force") ; self.req_SetDesiredForce = SetDesiredForce.Request()
        self._ros2_release_force               = g_node.create_client(ReleaseForce,                  self._srv_name_prefix +"/force/release_force") ; self.req_ReleaseForce = ReleaseForce.Request()
        self._ros2_check_position_condition    = g_node.create_client(CheckPositionCondition,        self._srv_name_prefix +"/force/check_position_condition") ; self.req_CheckPositionCondition = CheckPositionCondition.Request()
        self._ros2_check_force_condition       = g_node.create_client(CheckForceCondition,           self._srv_name_prefix +"/force/check_force_condition") ; self.req_CheckForceCondition = CheckForceCondition.Request()
        self._ros2_check_orientation_condition1= g_node.create_client(CheckOrientationCondition1,    self._srv_name_prefix +"/force/check_orientation_condition1") ; self.req_CheckOrientationCondition1 = CheckOrientationCondition1.Request()
        self._ros2_check_orientation_condition2= g_node.create_client(CheckOrientationCondition2,    self._srv_name_prefix +"/force/check_orientation_condition2") ; self.req_CheckOrientationCondition2 = CheckOrientationCondition2.Request()
        self._ros2_coord_transform             = g_node.create_client(CoordTransform,                self._srv_name_prefix +"/force/coord_transform") ; self.req_CoordTransform = CoordTransform.Request()

        #  GPIO Operations
        self._ros2_set_digital_output         = g_node.create_client(SetCtrlBoxDigitalOutput,        self._srv_name_prefix +"/io/set_digital_output") ; self.req_SetCtrlBoxDigitalOutput = SetCtrlBoxDigitalOutput.Request()
        self._ros2_get_digital_input          = g_node.create_client(GetCtrlBoxDigitalInput,         self._srv_name_prefix +"/io/get_digital_input") ; self.req_GetCtrlBoxDigitalInput = GetCtrlBoxDigitalInput.Request()
        self._ros2_set_tool_digital_output    = g_node.create_client(SetToolDigitalOutput,           self._srv_name_prefix +"/io/set_tool_digital_output") ; self.req_SetToolDigitalOutput= SetToolDigitalOutput.Request()
        self._ros2_get_tool_digital_input     = g_node.create_client(GetToolDigitalInput,            self._srv_name_prefix +"/io/get_tool_digital_input") ; self.req_GetToolDigitalInput = GetToolDigitalInput.Request()
        self._ros2_set_analog_output          = g_node.create_client(SetCtrlBoxAnalogOutput,         self._srv_name_prefix +"/io/set_analog_output") ; self.req_SetCtrlBoxAnalogOutput = SetCtrlBoxAnalogOutput.Request()
        self._ros2_get_analog_input           = g_node.create_client(GetCtrlBoxAnalogInput,          self._srv_name_prefix +"/io/get_analog_input") ; self.req_GetCtrlBoxAnalogInput = GetCtrlBoxAnalogInput.Request()
        self._ros2_set_mode_analog_output     = g_node.create_client(SetCtrlBoxAnalogOutputType,     self._srv_name_prefix +"/io/set_analog_output_type") ; self.req_SetCtrlBoxAnalogOutputType = SetCtrlBoxAnalogOutputType.Request()
        self._ros2_set_mode_analog_input      = g_node.create_client(SetCtrlBoxAnalogInputType,      self._srv_name_prefix +"/io/set_analog_input_type") ; self.req_SetCtrlBoxAnalogInputType = SetCtrlBoxAnalogInputType.Request()
        self._ros2_get_digital_output         = g_node.create_client(GetCtrlBoxDigitalOutput,        self._srv_name_prefix +"/io/get_digital_output") ; self.req_GetCtrlBoxDigitalOutput = GetCtrlBoxDigitalOutput.Request()
        self._ros2_get_tool_digital_output    = g_node.create_client(GetToolDigitalOutput,           self._srv_name_prefix +"/io/get_tool_digital_output") ; self.req_GetToolDigitalOutput = GetToolDigitalOutput.Request()
 
        #  Modbus Operations, 
        self._ros2_set_modbus_output          = g_node.create_client(SetModbusOutput,                self._srv_name_prefix +"/modbus/set_modbus_output") ; self.req_SetModbusOutput = SetModbusOutput.Request()
        self._ros2_get_modbus_input           = g_node.create_client(GetModbusInput,                 self._srv_name_prefix +"/modbus/get_modbus_input") ; self.req_GetModbusInput = GetModbusInput.Request()
        self._ros2_add_modbus_signal          = g_node.create_client(ConfigCreateModbus,             self._srv_name_prefix +"/modbus/config_create_modbus") ; self.req_ConfigCreateModbus = ConfigCreateModbus.Request()
        self._ros2_del_modbus_signal          = g_node.create_client(ConfigDeleteModbus,             self._srv_name_prefix +"/modbus/config_delete_modbus") ; self.req_ConfigDeleteModbus = ConfigDeleteModbus.Request()

        # TCP Operations, 
        self._ros2_set_current_tcp            = g_node.create_client(SetCurrentTcp,                  self._srv_name_prefix +"/tcp/set_current_tcp") ; self.req_SetCurrentTcp = SetCurrentTcp.Request()
        self._ros2_get_current_tcp            = g_node.create_client(GetCurrentTcp,                  self._srv_name_prefix +"/tcp/get_current_tcp") ; self.req_GetCurrentTcp = GetCurrentTcp.Request()
        self._ros2_config_create_tcp          = g_node.create_client(ConfigCreateTcp,                self._srv_name_prefix +"/tcp/config_create_tcp") ; self.req_ConfigCreateTcp = ConfigCreateTcp.Request()
        self._ros2_config_delete_tcp          = g_node.create_client(ConfigDeleteTcp,                self._srv_name_prefix +"/tcp/config_delete_tcp") ; self.req_ConfigDeleteTcp = ConfigDeleteTcp.Request()
 
        # Tool Operations, 
        self._ros2_set_current_tool           = g_node.create_client(SetCurrentTool,                 self._srv_name_prefix +"/tool/set_current_tool") ; self.req_SetCurrentTool = SetCurrentTool.Request()
        self._ros2_get_current_tool           = g_node.create_client(GetCurrentTool,                 self._srv_name_prefix +"/tool/get_current_tool") ; self.req_GetCurrentTool = GetCurrentTool.Request()
        self._ros2_config_create_tool         = g_node.create_client(ConfigCreateTool,               self._srv_name_prefix +"/tool/config_create_tool") ; self.req_ConfigCreateTool = ConfigCreateTool.Request()
        self._ros2_config_delete_tool         = g_node.create_client(ConfigDeleteTool,               self._srv_name_prefix +"/tool/config_delete_tool") ; self.req_ConfigDeleteTool = ConfigDeleteTool.Request()
        self._ros2_set_tool_shape             = g_node.create_client(SetToolShape,                   self._srv_name_prefix +"/tool/set_tool_shape") ; self.req_SetToolShape = SetToolShape.Request()

        # DRL Operations, 
        self._ros2_drl_pause                  = g_node.create_client(DrlPause,                       self._srv_name_prefix +"/drl/drl_pause") ; self.req_DrlPause = DrlPause.Request()
        self._ros2_drl_resume                 = g_node.create_client(DrlResume,                      self._srv_name_prefix +"/drl/drl_resume") ; self.req_DrlResume = DrlResume.Request()
        self._ros2_drl_start                  = g_node.create_client(DrlStart,                       self._srv_name_prefix +"/drl/drl_start") ; self.req_DrlStart = DrlStart.Request()
        self._ros2_drl_stop                   = g_node.create_client(DrlStop,                        self._srv_name_prefix +"/drl/drl_stop") ; self.req_DrlStop = DrlStop.Request()
        self._ros2_get_drl_state              = g_node.create_client(GetDrlState,                    self._srv_name_prefix +"/drl/get_drl_state") ; self.req_GetDrlState = GetDrlState.Request()

        ########################################################################################################################################

        self._g_blend_state = False
        self._g_blend_radius = 0.0

        self._g_velj = [0.0] * DR_VELJ_DT_LEN
        self._g_accj = [0.0] * DR_ACCJ_DT_LEN

        self._g_velx = [0.0] * DR_VELX_DT_LEN
        self._g_velx[0]= 0.0
        self._g_velx[1]= DR_COND_NONE

        self._g_accx = [0.0] * DR_ACCX_DT_LEN
        self._g_accx[0]= 0.0
        self._g_accx[1]= DR_COND_NONE

        self._g_coord = DR_BASE

        self._g_analog_output_mode_ch1 = -1
        self._g_analog_output_mode_ch2 = -1

    ##### SYSTEM ##############################################################################################################################

    def set_robot_mode(self, robot_mode):
        if type(robot_mode) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : robot_mode")

        # ROS service call
        if __ROS2__:
            #srv = self._ros_set_robot_mode(robot_mode)
            req = self.req_SetRobotMode
            req.robot_mode = robot_mode

            #ret = 0 if (srv.success == True) else -1
            future = self._ros2_set_robot_mode.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('set_robot_mode Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = 0 if (result.success == True) else -1                
        return ret





    def movej(self, pos, vel=None, acc=None, time=None, radius=None, mod= DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE, v=None, a=None, t=None, r=None):
        ret = self._movej(pos, vel, acc, time, radius, mod, ra, v, a, t, r, _async=0)
        return ret
    def amovej(self, pos, vel=None, acc=None, time=None, radius=None, mod= DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE, v=None, a=None, t=None, r=None):
        ret = self._movej(pos, vel, acc, time, radius, mod, ra, v, a, t, r, _async=1)
        return ret
    def _movej(self, pos, vel=None, acc=None, time=None, radius=None, mod= DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE, v=None, a=None, t=None, r=None, _async=0):
        # _pos
        _pos = get_posj(pos)
    
        # _vel
        temp = get_param(vel, v)
        if temp == None:
            _vel = self._g_velj
        else:
            if type(temp) == int or type(temp) == float:
                _vel = [temp] * DR_VELJ_DT_LEN
            elif type(temp) == list and len(temp) == DR_VELJ_DT_LEN:
                _vel = temp
            else:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : vel, v")
    
        if is_number(_vel) != True:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")
    
        for item in _vel:
            if item < 0:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")
    
        # _acc
        temp = get_param(acc, a)
        if temp == None:
            _acc = self._g_accj
        else:
            if type(temp) == int or type(temp) == float:
                _acc = [temp] * DR_ACCJ_DT_LEN
            elif type(temp) == list and len(temp) == DR_ACCJ_DT_LEN:
                _acc = temp
            else:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : acc, a")
    
        if is_number(_acc) != True:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")
    
        for item in _acc:
            if item < 0:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")
    
        # _time
        _time = get_param(time, t)
        if _time == None:
            _time = 0.0
    
        if type(_time) != int and type(_time) != float:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : time, t")
    
        if _time < 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : time, t")
    
        # check vel, acc, time
        #_check_valid_vel_acc(_vel, _acc, _time)
        _check_valid_vel_acc_joint(_vel, _acc, _time)
    
        # _radius
        _radius = get_param(radius, r)
        if _radius == None:
            #for ros global _g_blend_state
    
            if self._g_blend_state == True:
                _radius = self._g_blend_radius
            else:
                _radius = 0.0
    
        if type(_radius) != int and type(_radius) != float:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : radius, r")
    
        if _radius< 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : radius, r")
    
        # mod
        if type(mod) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : mod")
    
        if mod != DR_MV_MOD_ABS and mod != DR_MV_MOD_REL:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : mod")
    
        # ra
        if type(ra) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : ra")
    
        if ra != DR_MV_RA_OVERRIDE and ra != DR_MV_RA_DUPLICATE:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : ra")
    
        # _async
        if 1 == _async:
            _radius = 0   
    
        # ROS service call
        if __ROS2__: 
            #SSS srv = self._ros_movej(_pos, _vel[0], _acc[0], _time, mod, _radius, ra, _async)
            req = self.req_MoveJoint
            #for i in range(6):
            #    req.pos[i]      = pos[i] 
            req.pos         = [float(x) for x in _pos]
            req.vel         = float(_vel[0])
            req.acc         = float(_acc[0])
            req.time        = float(_time)
            req.mode        = int(mod)
            req.radius      = float(_radius)
            req.blend_type  = int(ra)
            req.sync_type   = int(_async)
            #RRR ret = 0 if (srv.success == True) else -1
            future = self._ros2_movej.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('movej Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = 0 if (result.success == True) else -1                
        else:   
            ret = PythonMgr.py_movej(_pos, _vel, _acc, _time, _radius, mod, ra, _async)
        return ret

    def movejx(self, pos, vel=None, acc=None, time=None, radius=None, ref=None, mod= DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE, sol=0, v=None, a=None, t=None, r=None):
        ret = _movejx(pos, vel, acc, time, radius, ref, mod, ra, sol, v, a, t, r, _async=0)
        return ret 
    def amovejx(self, pos, vel=None, acc=None, time=None, radius=None, ref=None, mod= DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE, sol=0, v=None, a=None, t=None, r=None):
        ret = _movejx(pos, vel, acc, time, radius, ref, mod, ra, sol, v, a, t, r, _async=1)
        return ret 
    def _movejx(self, pos, vel=None, acc=None, time=None, radius=None, ref=None, mod= DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE, sol=0, v=None, a=None, t=None, r=None, _async=0):
        # _pos
        _pos = get_posx(pos)

        # _vel
        temp = get_param(vel, v)
        if temp == None:
            _vel = self._g_velj
        else:
            if type(temp) == int or type(temp) == float:
                _vel = [temp] * DR_VELJ_DT_LEN
            elif type(temp) == list and len(temp) == DR_VELJ_DT_LEN:
                _vel = temp
            else:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : vel, v")

        if is_number(_vel) != True:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")

        for item in _vel:
            if item < 0:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")

        # _acc
        temp = get_param(acc, a)
        if temp == None:
            _acc = self._g_accj
        else:
            if type(temp) == int or type(temp) == float:
                _acc = [temp] * DR_ACCJ_DT_LEN
            elif type(temp) == list and len(temp) == DR_ACCJ_DT_LEN:
                _acc = temp
            else:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : acc, a")

        if is_number(_acc) != True:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")

        for item in _acc:
            if item < 0:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")

        # _time
        _time = get_param(time, t)
        if _time == None:
            _time = 0.0

        if type(_time) != int and type(_time) != float:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : time, t")

        if _time < 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : time, t")

        # check vel, acc, time
        #_check_valid_vel_acc(_vel, _acc, _time)
        _check_valid_vel_acc_joint(_vel, _acc, _time)

        # _radius
        _radius = get_param(radius, r)
        if _radius == None:
            if self._g_blend_state == True:
                _radius = self._g_blend_radius
            else:
                _radius = 0.0

        if type(_radius) != int and type(_radius) != float:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : radius, r")

        if _radius < 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : radius, r")

        # _ref
        if ref == None:
            _ref = self._g_coord
        else:
            _ref = ref

        # check ref
        if type(_ref) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : ref")

        if _ref < DR_BASE or _ref > DR_TC_USER_MAX:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref")

        # mod
        if type(mod) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : mod")

        if mod != DR_MV_MOD_ABS and mod != DR_MV_MOD_REL:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : mod")

        # ra
        if type(ra) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type, ra")

        if ra != DR_MV_RA_OVERRIDE and ra != DR_MV_RA_DUPLICATE:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : ra")

        # sol
        if type(sol) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : sol")

        if sol < DR_SOL_MIN or sol > DR_SOL_MAX:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : sol")

        # _async
        if 1 == _async:
            _radius = 0   

        # ROS service call
        if __ROS2__: 
            #srv = _ros_movejx(_pos, _vel[0], _acc[0], _time, mod, _ref, _radius, ra, sol, _async)   
            req = self.MoveJointx.Request()
            req.pos         = [float(x) for x in _pos]
            req.vel         = float(_vel[0])
            req.acc         = float(_acc[0])
            req.time        = float(_time)
            req.mode        = int(mod)
            req.ref         = int(_ref)
            req.radius      = float(_radius)
            req.blend_type  = int(ra)
            req.sol         = int(sol)
            req.sync_type   = int(_async)

            #ret = srv.success
            #ret = 0 if (srv.success == True) else -1
            future = self._ros2_movejx.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('movejx Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = 0 if (result.success == True) else -1                
        else:    
            ret = PythonMgr.py_movejx(_pos, _vel, _acc, _time, _radius, _ref, mod, ra, sol, _async)
            print_ext_result("{0} = PythonMgr.py_movejx(pos:{1}, vel:{2}, acc:{3}, time:{4}, radius:{5}, ref{6}, mod{7}, ra:{8}, sol:{9}, async:{10})" \
                             .format(ret, _pos, dr_form(_vel), dr_form(_acc), _time, _radius, _ref, mod, ra, sol, _async))   
        return ret


    def movel(self, pos, vel=None, acc=None, time=None, radius=None, ref=None, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE, v=None, a=None, t=None, r=None):
        ret = _movel(pos, vel, acc, time, radius, ref, mod, ra, v, a, t, r, _async=0)
        return ret
    def amovel(self, pos, vel=None, acc=None, time=None, radius=None, ref=None, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE, v=None, a=None, t=None, r=None):
        ret = _movel(pos, vel, acc, time, radius, ref, mod, ra, v, a, t, r, _async=1)
        return ret
    def _movel(self, pos, vel=None, acc=None, time=None, radius=None, ref=None, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE, v=None, a=None, t=None, r=None, _async=0):
        # _pos
        _pos = get_normal_pos(pos, def_type=posx)

        # _vel
        temp = get_param(vel, v)
        if temp == None:
            _vel = self._g_velx
        else:
            if type(temp) == int or type(temp) == float:
                #_vel = [temp] * DR_VELX_DT_LEN 
                _vel = [0] * DR_VELX_DT_LEN     
                _vel[0] = temp
                _vel[1] = DR_COND_NONE
            elif type(temp) == list and len(temp) == DR_VELX_DT_LEN:
                _vel = temp
            else:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : vel, v")

        if is_number(_vel) != True:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")

        for item in _vel:
            #if item < 0:
            if (item < 0) and (item != DR_COND_NONE):
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")

        #print("_vel[0]={0}, _vel[1]={1}".format(_vel[0],_vel[1]))

        # _acc
        temp = get_param(acc, a)
        if temp == None:
            _acc = self._g_accx
        else:
            if type(temp) == int or type(temp) == float:
                #_acc = [temp] * DR_ACCX_DT_LEN 
                _acc = [0] * DR_ACCX_DT_LEN     
                _acc[0] = temp
                _acc[1] = DR_COND_NONE
            elif type(temp) == list and len(temp) == DR_ACCX_DT_LEN:
                _acc = temp
            else:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : acc, a")

        if is_number(_acc) != True:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")

        for item in _acc:
            #if item < 0:
            if (item < 0) and (item != DR_COND_NONE):
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")

        #print("_acc[0]={0}, _acc[1]={1}".format(_acc[0],_acc[1]))

        # _time
        _time = get_param(time, t)
        if _time == None:
            _time = 0.0

        if type(_time) != int and type(_time) != float:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : time, t")

        if _time < 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : time, t")

        # check vel, acc, time
        #_check_valid_vel_acc(_vel, _acc, _time)
        _check_valid_vel_acc_task(_vel, _acc, _time)

        # _radius
        _radius = get_param(radius, r)
        if _radius == None:
            if self._g_blend_state == True:
                _radius = self._g_blend_radius
            else:
                _radius = 0.0

        if type(_radius) != int and type(_radius) != float:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : radius, r")

        if _radius < 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : radius, r")

        # ref
        if ref == None:
            _ref = self._g_coord
        else:
            _ref = ref

        # check ref
        if type(_ref) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : ref")

        if _ref < DR_BASE or _ref > DR_TC_USER_MAX:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref")

        # mod
        if type(mod) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : mod")

        if mod != DR_MV_MOD_ABS and mod != DR_MV_MOD_REL:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : mod")

        # ra
        if type(ra) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : ra")

        if ra != DR_MV_RA_OVERRIDE and ra != DR_MV_RA_DUPLICATE:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : ra")

        # qcommand
        if type(_pos) == posx:
            qcommand = 0
        else:
            qcommand = 1

        # _async
        if 1 == _async:
            _radius = 0   

        # ROS service call
        if __ROS2__: 
            req = self.MoveLine.Request()

            req.pos         = [float(x) for x in _pos]
            req.vel         = [float(x) for x in _vel]
            req.acc         = [float(x) for x in _acc]
            req.time        = float(_time)
            req.radius      = float(_radius)
            req.ref         = int(_ref)
            req.mode        = int(mod)
            req.blend_type  = int(ra)
            req.sync_type   = int(_async)

            #RRR ret = 0 if (srv.success == True) else -1
            future = self._ros2_movel.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('movel Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = 0 if (result.success == True) else -1                
        else:    
            ret = PythonMgr.py_movel(_pos, _vel, _acc, _time, _radius, _ref, mod, ra, qcommand, _async)
            print_ext_result("{0} = PythonMgr.py_movel(pos:{1}, vel:{2}, acc:{3}, time:{4}, radius:{5}, ref:{6}, mod:{7}, ra:{8}, qcommand:{9}, async:{10})" \
                             .format(ret, _pos, dr_form(_vel), dr_form(_acc), _time, _radius, _ref, mod, ra, qcommand, _async))
        return ret

    def movec(self, pos1, pos2, vel=None, acc=None, time=None, radius=None, ref=None, mod= DR_MV_MOD_ABS, angle=None, ra=DR_MV_RA_DUPLICATE, v=None, a=None, t=None, r=None, an=None):
        ret = _movec(pos1, pos2, vel, acc, time, radius, ref, mod, angle, ra, v, a, t, r, an, _async=0)
        return ret
    def amovec(self, pos1, pos2, vel=None, acc=None, time=None, radius=None, ref=None, mod= DR_MV_MOD_ABS, angle=None, ra=DR_MV_RA_DUPLICATE, v=None, a=None, t=None, r=None, an=None):
        ret = _movec(pos1, pos2, vel, acc, time, radius, ref, mod, angle, ra, v, a, t, r, an, _async=1)
        return ret
    def _movec(self, pos1, pos2, vel=None, acc=None, time=None, radius=None, ref=None, mod= DR_MV_MOD_ABS, angle=None, ra=DR_MV_RA_DUPLICATE, v=None, a=None, t=None, r=None, an=None, _async=0):

        # _pos1, _pos2

        _pos1 = get_normal_pos(pos1, def_type=posx)
        _pos2 = get_normal_pos(pos2, def_type=posx)

        if type(_pos1) != type(_pos2):
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : pos1, ps2")

        # _vel
        temp = get_param(vel, v)
        if temp == None:
            _vel = self._g_velx
        else:
            if type(temp) == int or type(temp) == float:
                #_vel = [temp] * DR_VELX_DT_LEN #<����> ���� �� �ΰ� ����
                _vel = [0] * DR_VELX_DT_LEN     #<����> _vel[0]=temp, _vel[1]= DR_COND_NONE
                _vel[0] = temp
                _vel[1] = DR_COND_NONE
            elif type(temp) == list and len(temp) == DR_VELX_DT_LEN:
                _vel = temp
            else:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : vel, v")

        if is_number(_vel) != True:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")

        for item in _vel:
            #if item < 0:
            if (item < 0) and (item != DR_COND_NONE):
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")

        # _acc
        temp = get_param(acc, a)
        if temp == None:
            _acc = self._g_accx
        else:
            if type(temp) == int or type(temp) == float:
                #_acc = [temp] * DR_ACCX_DT_LEN #<����> ���� �� �ΰ� ����
                _acc = [0] * DR_ACCX_DT_LEN     #<����> _acc[0]=temp, _acc[1]= DR_COND_NONE
                _acc[0] = temp
                _acc[1] = DR_COND_NONE
            elif type(temp) == list and len(temp) == DR_ACCX_DT_LEN:
                _acc = temp
            else:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : acc, a")

        if is_number(_acc) != True:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")

        for item in _acc:
            #if item < 0:
            if (item < 0) and (item != DR_COND_NONE):
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")

        # _time
        _time = get_param(time, t)
        if _time == None:
            _time = 0.0

        if type(_time) != int and type(_time) != float:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : time, t")

        if _time < 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : time, t")

        # check vel, acc, time
        #_check_valid_vel_acc(_vel, _acc, _time)
        _check_valid_vel_acc_task(_vel, _acc, _time)

        # _radius
        _radius = get_param(radius, r)
        if _radius == None:
            if self._g_blend_state == True:
                _radius = self._g_blend_radius
            else:
                _radius = 0.0

        if type(_radius) != int and type(_radius) != float:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : radius, r")

        if _radius < 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : radius, r")

        # _ref
        if ref == None:
            _ref = _g_coord
        else:
            _ref = ref

        # check ref
        if type(_ref) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : ref")

        if _ref < DR_BASE or _ref > DR_TC_USER_MAX:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref")

        # mod
        if type(mod) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : mod")

        if mod != DR_MV_MOD_ABS and mod != DR_MV_MOD_REL:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : mod")

        # _angle
        temp = get_param(angle, an)
        if temp == None:
            _angle = [0, 0]
        else:
            if type(temp) == int or type(temp) == float:
                _angle = [temp, 0]
            elif type(temp) == list and len(temp) == DR_ANGLE_DT_LEN:
                _angle = temp
            else:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : angle, an")

        if is_number(_angle) != True:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : angle, an")

        for item in _angle:
            if item < 0:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : angle, an")

        # ra
        if type(ra) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : ra")

        if ra != DR_MV_RA_OVERRIDE and ra != DR_MV_RA_DUPLICATE:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : ra")

        # qcommand
        if type(_pos1) == posx:
            qcommand = 0
        else:
            qcommand = 1

        # _async
        if 1 == _async:
            _radius = 0   

        # ROS service call
        if __ROS2__:
            # make multi pos
            _circle_pos = _ros_listToFloat64MultiArray([_pos1, _pos2])
            #_circle_pos = _ros_listToFloat64MultiArray([ [float(x) for x in _pos1], [float(x) for x in _pos2] ])
            #print(_circle_pos)

            #srv = _ros_movec(_circle_pos, _vel, _acc, _time, mod, _ref, _angle[0], _angle[1], _radius, ra, _async) 
            req = self.MoveCircle.Request()
            req.pos         = _circle_pos               #??? 검증필요!
            req.vel         = [float(x) for x in _vel]
            req.acc         = [float(x) for x in _acc]
            req.time        = float(_time)
            req.radius      = float(_radius)
            req.ref         = int(_ref)
            req.mode        = int(mod)
            req.angle1      = float(_angle[0])
            req.angle2      = float(_angle[1])
            req.blend_type  = int(ra)
            req.sync_type   = int(_async)

            #RRR ret = 0 if (srv.success == True) else -1
            future = self._ros2_movec.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('movec Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = 0 if (result.success == True) else -1                
        else:   
            ret = PythonMgr.py_movec(_pos1, _pos2, _vel, _acc, _time, _radius, _ref, mod, _angle, ra, qcommand, _async)
            print_ext_result("{0} = PythonMgr.py_movec(pos1:{1}, pos2:{2}, vel:{3}, acc:{4}, time:{5}, radius:{6}, ref:{7}, mod:{8}, angle:{9}, ra:{10}, qcommand:{11}, async:{12})" \
                             .format(ret, _pos1, _pos2, dr_form(_vel), dr_form(_acc), _time, _radius, _ref, mod, dr_form(_angle), ra, qcommand, _async))
        return ret


    def movesj(self, pos_list, vel=None, acc=None, time=None, mod= DR_MV_MOD_ABS, v=None, a=None, t=None):
        ret = _movesj(pos_list, vel, acc, time, mod, v, a, t, _async=0)
        return ret
    def amovesj(self, pos_list, vel=None, acc=None, time=None, mod= DR_MV_MOD_ABS, v=None, a=None, t=None):
        ret = _movesj(pos_list, vel, acc, time, mod, v, a, t, _async=1)
        return ret
    def _movesj(self, pos_list, vel=None, acc=None, time=None, mod= DR_MV_MOD_ABS, v=None, a=None, t=None, _async=0):
        # pos_list
        if type(pos_list) != list:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : pos_list")

        for item in pos_list:
            if type(item) != posj:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : pos_list [item]")

        # _vel
        temp = get_param(vel, v)
        if temp == None:
            _vel = self._g_velj
        else:
            if type(temp) == int or type(temp) == float:
                _vel = [temp] * DR_VELJ_DT_LEN
            elif type(temp) == list and len(temp) == DR_VELJ_DT_LEN:
                _vel = temp
            else:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : vel, v")

        if is_number(_vel) != True:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")

        for item in _vel:
            if item < 0:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")

        # _acc
        temp = get_param(acc, a)
        if temp == None:
            _acc = self._g_accj
        else:
            if type(temp) == int or type(temp) == float:
                _acc = [temp] * DR_ACCJ_DT_LEN
            elif type(temp) == list and len(temp) == DR_ACCJ_DT_LEN:
                _acc = temp
            else:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : acc, a")

        if is_number(_acc) != True:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")

        for item in _acc:
            if item < 0:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")

        # _time
        _time = get_param(time, t)
        if _time == None:
            _time = 0.0

        if type(_time) != int and type(_time) != float:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : time, t")

        if _time < 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : time, t")

        # check vel, acc, time
        #_check_valid_vel_acc(_vel, _acc, _time)
        _check_valid_vel_acc_joint(_vel, _acc, _time)

        # mod
        if type(mod) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : mod")

        if mod != DR_MV_MOD_ABS and mod != DR_MV_MOD_REL:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : mod")

        # _async
        if 1 == _async:
            _radius = 0   

        # ROS service call
        if __ROS2__:
            # make multi pos
            _spline_posj = _ros_listToFloat64MultiArray(pos_list)
            #srv = _ros_movesj(_spline_posj, len(_spline_posj), _vel, _acc, _time, mod, _async)

            req = self.MoveSplineJoint.Request()

            req.pos         = _spline_posj               #??? 검증필요!
            req.pos_cnt     = len(_spline_posj)

            if type(_vel) == list:
                _vel = float(_vel[0]) 
            if type(_acc) == list:
                _acc = float(_acc[0]) 
            req.vel         = _vel #[float(x) for x in _vel]
            req.acc         = _acc #[float(x) for x in _acc]

            req.time        = float(_time)
            req.mode        = int(mod)
            req.sync_type   = int(_async)

            #RRR ret = 0 if (srv.success == True) else -1
            future = self._ros2_movesj.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('movesj Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = 0 if (result.success == True) else -1                
        else:    
            ret = PythonMgr.py_movesj(pos_list, _vel, _acc, _time, mod, _async)
            print_ext_result("{0} = PythonMgr.py_movesj(pos_list:{1}, vel:{2}, acc:{3}, time:{4}, mod:{5} async:{6})" \
                             .format(ret, dr_form(pos_list), dr_form(_vel), dr_form(_acc), _time, mod, _async))
        return ret



    def movesx(self, pos_list, vel=None, acc=None, time=None, ref=None, mod= DR_MV_MOD_ABS, vel_opt=DR_MVS_VEL_NONE, v=None, a=None, t=None):
        ret = _movesx(pos_list, vel, acc, time, ref, mod, vel_opt, v, a, t, _async=0)
        return ret
    def amovesx(self, pos_list, vel=None, acc=None, time=None, ref=None, mod= DR_MV_MOD_ABS, vel_opt=DR_MVS_VEL_NONE, v=None, a=None, t=None):
        ret = _movesx(pos_list, vel, acc, time, ref, mod, vel_opt, v, a, t, _async=1)
        return ret
    def _movesx(self, pos_list, vel=None, acc=None, time=None, ref=None, mod= DR_MV_MOD_ABS, vel_opt=DR_MVS_VEL_NONE, v=None, a=None, t=None, _async=0):
        # pos_list
        if type(pos_list) != list:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : pos_list")

        for item in pos_list:
            if type(item) != posx:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : pos_list [item]")

        # _vel
        temp = get_param(vel, v)
        if temp == None:
            _vel = self._g_velx
        else:
            if type(temp) == int or type(temp) == float:
                #_vel = [temp] * DR_VELX_DT_LEN #<����> ���� �� �ΰ� ����
                _vel = [0] * DR_VELX_DT_LEN     #<����> _vel[0]=temp, _vel[1]= DR_COND_NONE
                _vel[0] = temp
                _vel[1] = DR_COND_NONE
            elif type(temp) == list and len(temp) == DR_VELX_DT_LEN:
                _vel = temp
            else:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : vel, v")

        if is_number(_vel) != True:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")

        for item in _vel:
            #if item < 0:
            if (item < 0) and (item != DR_COND_NONE):
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")

        # _acc
        temp = get_param(acc, a)
        if temp == None:
            _acc = self._g_accx
        else:
            if type(temp) == int or type(temp) == float:
                #_acc = [temp] * DR_ACCX_DT_LEN #<����> ���� �� �ΰ� ����
                _acc = [0] * DR_ACCX_DT_LEN     #<����> _acc[0]=temp, _acc[1]= DR_COND_NONE
                _acc[0] = temp
                _acc[1] = DR_COND_NONE
            elif type(temp) == list and len(temp) == DR_ACCX_DT_LEN:
                _acc = temp
            else:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : acc, a")

        if is_number(_acc) != True:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")

        for item in _acc:
            #if item < 0:
            if (item < 0) and (item != DR_COND_NONE):
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")

        # _time
        _time = get_param(time, t)
        if _time == None:
            _time = 0.0

        if type(_time) != int and type(_time) != float:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : time, t")

        if _time < 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : time, t")

        # check vel, acc, time
        #_check_valid_vel_acc(_vel, _acc, _time)
        _check_valid_vel_acc_task(_vel, _acc, _time)

        # _ref
        if ref == None:
            _ref = _g_coord
        else:
            _ref = ref

        # check ref
        if type(_ref) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : ref")

        if _ref < DR_BASE or _ref > DR_TC_USER_MAX:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref")

        # mod
        if type(mod) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : mod")

        if mod != DR_MV_MOD_ABS and mod != DR_MV_MOD_REL:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : mod")

        # vel_opt
        if type(vel_opt) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : vel_opt")

        if vel_opt != DR_MVS_VEL_NONE and vel_opt != DR_MVS_VEL_CONST:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel_opt")

        # _async
        if 1 == _async:
            _radius = 0   

        # ROS service call
        if __ROS2__:
            # make multi pos
            _spline_posx = _ros_listToFloat64MultiArray(pos_list)
            #srv = _ros_movesx(_spline_posx, len(_spline_posx), _vel, _acc, _time, mod, _ref, vel_opt, _async)
            req = self.MoveSplineTask.Request()
            req.pos         = _spline_posx               #??? 검증필요!
            req.pos_cnt     = len(_spline_posx)
            req.vel         = [float(x) for x in _vel]
            req.acc         = [float(x) for x in _acc]
            req.time        = float(_time)
            req.ref         = int(_ref)
            req.mode        = int(mod)
            req.opt         = int(vel_opt)
            req.sync_type   = int(_async)

            #RRR ret = 0 if (srv.success == True) else -1
            future = self._ros2_movesx.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('movesx Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = 0 if (result.success == True) else -1                
        else:
            ret = PythonMgr.py_movesx(pos_list, _vel, _acc, _time, _ref, mod, vel_opt, _async)
            print_ext_result("{0} = PythonMgr.py_movesx(pos_list:{1}, vel:{2}, acc:{3}, time:{4}, ref:{5}, mod:{6}, vel_opt:{7}, async:{8})" \
                            .format(ret, dr_form(pos_list), dr_form(_vel), dr_form(_acc), _time, _ref, mod, vel_opt, _async))
        return ret

    def moveb(self, seg_list, vel=None, acc=None, ref=None, time=None, mod=DR_MV_MOD_ABS, v=None, a=None, t=None):
        ret = _moveb(seg_list, vel, acc, ref, time, mod, v, a, t, _async=0)
        return ret
    def amoveb(self, seg_list, vel=None, acc=None, ref=None, time=None, mod=DR_MV_MOD_ABS, v=None, a=None, t=None):
        ret = _moveb(seg_list, vel, acc, ref, time, mod, v, a, t, _async=1)
        return ret
    def _moveb(self, seg_list, vel=None, acc=None, ref=None, time=None, mod=DR_MV_MOD_ABS, v=None, a=None, t=None, _async=0):

        # seg_list
        if type(seg_list) != list:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : seg_list")

        if len(seg_list) == 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : seg_list")

        _seg_list = []
        if __ROS2__:
            for seg in seg_list:
                _seg_list.append(seg.to_list())
        else:
            for seg in seg_list:
                if type(seg) != posb:
                    raise DR_Error(DR_ERROR_TYPE, "Invalid type : seg_list(item)")
                else:
                    _seg_list.append(seg.to_list())

        if __ROS2__:
            print(len(_seg_list))
            _ros_seg_list = []
            _tmp_list = []

            for s in range(0, len(_seg_list)):
                #print(s)
                for elemnt in range(0, len(_seg_list[s])):
                    if _seg_list[s][elemnt] == None:
                        _seg_list[s][elemnt] = [0.0]*POINT_COUNT

                # make [pos1] + [pos2] + [type] + [radius]
                _tmp_list = _seg_list[s][1] + _seg_list[s][2] + [_seg_list[s][0]] + [_seg_list[s][3]]
                _ros_seg_list.append(_tmp_list)

        # _vel
        temp = get_param(vel, v)
        if temp == None:
            _vel = self._g_velx
        else:
            if type(temp) == int or type(temp) == float:
                #_vel = [temp] * DR_VELX_DT_LEN #<����> ���� �� �ΰ� ����
                _vel = [0] * DR_VELX_DT_LEN     #<����> _vel[0]=temp, _vel[1]= DR_COND_NONE
                _vel[0] = temp
                _vel[1] = DR_COND_NONE
            elif type(temp) == list and len(temp) == DR_VELX_DT_LEN:
                _vel = temp
            else:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : vel, v")

        if is_number(_vel) != True:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")

        for item in _vel:
            #if item < 0:
            if (item < 0) and (item != DR_COND_NONE):
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")

        # _acc
        temp = get_param(acc, a)
        if temp == None:
            _acc = self._g_accx
        else:
            if type(temp) == int or type(temp) == float:
                #_acc = [temp] * DR_ACCX_DT_LEN #<����> ���� �� �ΰ� ����
                _acc = [0] * DR_ACCX_DT_LEN     #<����> _acc[0]=temp, _acc[1]= DR_COND_NONE
                _acc[0] = temp
                _acc[1] = DR_COND_NONE
            elif type(temp) == list and len(temp) == DR_ACCX_DT_LEN:
                _acc = temp
            else:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : acc, a")

        if is_number(_acc) != True:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")

        for item in _acc:
            #if item < 0:
            if (item < 0) and (item != DR_COND_NONE):
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")

        # _time
        _time = get_param(time, t)
        if _time == None:
            _time = 0.0

        # check vel, acc, time
        #_check_valid_vel_acc(_vel, _acc, _time)
        _check_valid_vel_acc_task(_vel, _acc, _time)

        # ref
        if ref == None:
            _ref = self._g_coord
        else:
            _ref = ref

        # check ref
        if type(_ref) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : ref")

        if _ref < DR_BASE or _ref > DR_TC_USER_MAX:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref")

        # mod
        if type(mod) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : mod")

        if mod != DR_MV_MOD_ABS and mod != DR_MV_MOD_REL:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : mod")

        # _async
        if 1 == _async:
            _radius = 0   

        # ROS Service call
        if __ROS2__:
            seg = _ros_listToFloat64MultiArray(_ros_seg_list)
            #srv = _ros_moveb(seg, len(_ros_seg_list), _vel, _acc, _time, mod, _ref, _async)    
            req = self.MoveBlending.Request()
            req.segment     = seg               #??? 검증필요!
            req.pos_cnt     = len(_ros_seg_list)
            req.vel         = [float(x) for x in _vel]
            req.acc         = [float(x) for x in _acc]
            req.time        = float(_time)
            req.ref         = int(_ref)
            req.mode        = int(mod)
            req.sync_type   = int(_async)

            #RRR ret = 0 if (srv.success == True) else -1
            future = self._ros2_moveb.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('moveb Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = 0 if (result.success == True) else -1                
        else:
            ret = PythonMgr.py_moveb(_seg_list, _vel, _acc, _time, _ref, mod, _async)
            print_ext_result("{0} = PythonMgr.py_moveb(seg_list:{1}, vel:{2}, acc:{3}, time:{4}, ref:{5}, mod:{6}, async:{7})" \
                             .format(ret, dr_form(_seg_list), dr_form(_vel), dr_form(_acc), _time, _ref, mod, _async))
        return ret


    def move_spiral(self, rev=10, rmax=10, lmax=0, vel=None, acc=None, time=None, axis=DR_AXIS_Z, ref=DR_TOOL, v=None, a=None, t=None):
        ret = _move_spiral(rev, rmax, lmax, vel, acc, time, axis, ref, v, a, t, _async=0)
        return ret
    def amove_spiral(self, rev=10, rmax=10, lmax=0, vel=None, acc=None, time=None, axis=DR_AXIS_Z, ref=DR_TOOL, v=None, a=None, t=None):
        ret = _move_spiral(rev, rmax, lmax, vel, acc, time, axis, ref, v, a, t, _async=1)
        return ret
    def _move_spiral(self, rev=10, rmax=10, lmax=0, vel=None, acc=None, time=None, axis=DR_AXIS_Z, ref=DR_TOOL, v=None, a=None, t=None, _async=0):
        # rev
        if type(rev) != int and type(rev) != float:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : rev")

        if rev <= 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : rev (Ranges: rev > 0)")

        # rmax
        if type(rmax) != int and type(rmax) != float:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : rmax")

        if rmax <= 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value: rmax (Ranges: rmax > 0)")

        # lmax
        if type(lmax) != int and type(lmax) != float:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : lmax")

        # _vel
        temp = get_param(vel, v)
        if temp == None:
            _vel = self._g_velx
        else:
            if type(temp) == int or type(temp) == float:
                _vel = [temp] * DR_VELX_DT_LEN
            elif type(temp) == list and len(temp) == DR_VELX_DT_LEN:
                _vel = temp
            else:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : vel, v")

        if is_number(_vel) != True:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")

        for item in _vel:
            if item < 0:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : vel, v")

        # _acc
        temp = get_param(acc, a)
        if temp == None:
            _acc = self._g_accx
        else:
            if type(temp) == int or type(temp) == float:
                _acc = [temp] * DR_ACCX_DT_LEN
            elif type(temp) == list and len(temp) == DR_ACCX_DT_LEN:
                _acc = temp
            else:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : acc, a")

        if is_number(_acc) != True:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")

        for item in _acc:
            if item < 0:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : acc, a")

        # _time
        _time = get_param(time, t)
        if _time == None:
            _time = 0.0

        if type(_time) != int and type(_time) != float:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : time, t")

        if _time < 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : time, t")

        # check vel, acc, time
        #_check_valid_vel_acc(_vel, _acc, _time)
        _check_valid_vel_acc_joint(_vel, _acc, _time)

        # axis
        if type(axis) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : axis")

        if axis != DR_AXIS_X and axis != DR_AXIS_Y and axis != DR_AXIS_Z:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : axis")

        # ref
        if type(ref) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : ref")

        if ref != DR_BASE and ref != DR_TOOL and (ref < DR_TC_USER_MIN or ref > DR_TC_USER_MAX):
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref")

        # _async
        if 1 == _async:
            _radius = 0   

        # ROS service call
        if __ROS2__:
            #srv = _ros_move_spiral(rev, rmax, lmax, _vel, _acc, _time, axis, ref, _async)
            req = self.MoveSpiral.Request()
            req.revolution  = rev               #??? 검증필요!
            req.max_radius  = float(rmax)
            req.max_length  = float(lmax)
            req.vel         = [float(x) for x in _vel]
            req.acc         = [float(x) for x in _acc]
            req.time        = float(_time)
            req.task_axis   = int(axis)
            req.ref         = int(ref)
            req.sync_type   = int(_async)

            #RRR ret = 0 if (srv.success == True) else -1
            future = self._ros2_move_spiral.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('move_spiral Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = 0 if (result.success == True) else -1     
        else:    
            ret = PythonMgr.py_move_spiral(rev, rmax, lmax, _vel, _acc, _time, axis, ref, _async)
            print_ext_result("{0} = PythonMgr.py_move_spiral(rev:{1}, rmax:{2}, lmax:{3}, vel:{4}, acc:{5}, time:{6}, axis:{7}, ref:{8}, async:{9})" \
                             .format(ret, dr_form(rev), dr_form(rmax), dr_form(lmax), dr_form(_vel), dr_form(_acc), _time, axis, ref, _async))    
        return ret



    def move_periodic(self, amp, period, atime=None, repeat=None, ref=DR_TOOL):
        ret = _move_periodic(amp, period, atime, repeat, ref, _async=0)
        return ret
    def amove_periodic(self, amp, period, atime=None, repeat=None, ref=DR_TOOL):
        ret = _move_periodic(amp, period, atime, repeat, ref, _async=1)
        return ret
    def _move_periodic(self, amp, period, atime=None, repeat=None, ref=DR_TOOL, _async=0):
        _amp = [POINT_COUNT]
        _period = [POINT_COUNT]
        _atime =0.0
        _repeat =0.0
        _ref=0 

        # amp : float[6] 
        if type(amp) != list:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : amp")
        if len(amp) != POINT_COUNT:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : amp")
        _amp =amp

        # period : float or float[6] 
        if (type(period) != int) and (type(period) != float) and (type(period) != list):
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : period")

        if (type(period) == int) or (type(period) == float):
            _period = [period] * POINT_COUNT
        else: #list �� ��� 
            if len(period) != POINT_COUNT:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : period")
            _period = period

        # atime 
        if atime == None:
            _atime = 0.0
        else:
            if (type(atime) != int) and (type(atime) != float):
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : atime")
            if atime < 0.0:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : atime")
            _atime = atime

        # repeat 
        if repeat == None:
            _repeat = 1
        else:
            if (type(repeat) != int) and (type(repeat) != float): 
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : repeat")
            if repeat < 0.0: 
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : repeat")
            _repeat = repeat

        # ref
        if ref == None:
            #_ref = _g_coord
            _ref = DR_TOOL
        else:
            if type(ref) != int:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : ref")

            if ref < DR_BASE or ref > DR_TC_USER_MAX:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : ref")
            _ref = ref

        # _async
        if 1 == _async:
            _radius = 0   

        # ROS service call
        if __ROS2__:
            #srv = _ros_move_periodic(_amp, _period, _atime, _repeat, _ref, _async)    
            req = self.MovePeriodic.Request()
            req.amp         = [float(x) for x in _amp]
            req.periodic    = [float(x) for x in _period]
            req.acc         = float(_atime)
            req.repeat      = int(_repeat)
            req.ref         = int(_ref)
            req.sync_type   = int(_async)

            #RRR ret = 0 if (srv.success == True) else -1
            future = self._ros2_move_periodic.call_async(req)
            rclpy.spin_until_future_complete(g_node, future)

            try:
                result = future.result()
            except Exception as e:
                g_node.get_logger().info('move_periodic Service call failed %r' % (e,))
            else:
                if result == None:
                    ret = -1    
                else:        
                    ret = 0 if (result.success == True) else -1     

        else:
            ret = PythonMgr.py_move_periodic(_amp, _period, _atime, _repeat, _ref, _async)
            print_ext_result("{0} = PythonMgr.move_periodic(amp:{1}, period:{2}, atime:{3}, repeat{4}, ref:{5}, async:{6})" \
                             .format(ret, dr_form(_amp), dr_form(_period), _atime, _repeat, _ref, _async))
        return ret
