/*
 *  Inferfaces for doosan robot controllor 
  * Author: Kab Kyoum Kim (kabkyoum.kim@doosan.com)
 *
 * Copyright (c) 2020 Doosan Robotics
 * Use of this source code is governed by the BSD, see LICENSE
*/

#include "dsr_control2/dsr_hw_interface2.h"
#include <boost/thread/thread.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/bind.hpp>
#include <sstream>

#include <unistd.h>     
#include <math.h>

extern rclcpp::Node::SharedPtr g_node; //ROS2

CDRFL Drfl;
//TODO Serial_comm ser_comm;

bool g_bHasControlAuthority = FALSE;
bool g_bTpInitailizingComplted = FALSE;
bool g_bHommingCompleted = FALSE;

DR_STATE    g_stDrState;
DR_ERROR    g_stDrError;

int g_nAnalogOutputModeCh1;
int g_nAnalogOutputModeCh2;


#define STABLE_BAND_JNT     0.05
#define DSR_CTL_PUB_RATE    100  //[hz] 10ms <----- 퍼블리싱 주기, but OnMonitoringDataCB() 은 100ms 마다 불려짐을 유의!   

namespace dsr_control2{

    const char* GetRobotStateString(int nState)
    {
        switch(nState)
        {
        case STATE_INITIALIZING:    return "(0) INITIALIZING";
        case STATE_STANDBY:         return "(1) STANDBY";
        case STATE_MOVING:          return "(2) MOVING";
        case STATE_SAFE_OFF:        return "(3) SAFE_OFF";
        case STATE_TEACHING:        return "(4) TEACHING";
        case STATE_SAFE_STOP:       return "(5) SAFE_STOP";
        case STATE_EMERGENCY_STOP:  return "(6) EMERGENCY_STOP";
        case STATE_HOMMING:         return "(7) HOMMING";
        case STATE_RECOVERY:        return "(8) RECOVERY";
        case STATE_SAFE_STOP2:      return "(9) SAFE_STOP2";
        case STATE_SAFE_OFF2:       return "(10) SAFE_OFF2";
        case STATE_RESERVED1:       return "(11) RESERVED1";
        case STATE_RESERVED2:       return "(12) RESERVED2";
        case STATE_RESERVED3:       return "(13) RESERVED3";
        case STATE_RESERVED4:       return "(14) RESERVED4";
        case STATE_NOT_READY:       return "(15) NOT_READY";

        default:                  return "UNKNOWN";
        }
        return "UNKNOWN";
    }

    int IsInposition(double dCurPosDeg[], double dCmdPosDeg[])
    {
        int cnt=0;
        double dError[NUM_JOINT] ={0.0, };

        for(int i=0;i<NUM_JOINT;i++)
        {
            dError[i] = dCurPosDeg[i] - dCmdPosDeg[i];
            RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"<inpos> %f = %f -%f",dError[i], dCurPosDeg[i], dCmdPosDeg[i]);
            if(fabs(dError[i]) < STABLE_BAND_JNT)
                cnt++;
        }
        if(NUM_JOINT == cnt)
            return true;
        else 
            return false;
    }

    //----- register the call-back functions ----------------------------------------
    void DRHWInterface::OnTpInitializingCompletedCB()
    {
        // request control authority after TP initialized
        cout << "[callback OnTpInitializingCompletedCB] tp initializing completed" << endl;
        g_bTpInitailizingComplted = TRUE;
        //Drfl.ManageAccessControl(MANAGE_ACCESS_CONTROL_REQUEST);
        Drfl.ManageAccessControl(MANAGE_ACCESS_CONTROL_FORCE_REQUEST);

        g_stDrState.bTpInitialized = TRUE;
    }

    void DRHWInterface::OnHommingCompletedCB()
    {
        g_bHommingCompleted = TRUE;
        // Only work within 50msec
        cout << "[callback OnHommingCompletedCB] homming completed" << endl;

        g_stDrState.bHommingCompleted = TRUE;
    }

    void DRHWInterface::OnProgramStoppedCB(const PROGRAM_STOP_CAUSE iStopCause)
    {
        cout << "[callback OnProgramStoppedCB] Program Stop: " << (int)iStopCause << endl;
        g_stDrState.bDrlStopped = TRUE;
    }
    // M2.4 or lower
    void DRHWInterface::OnMonitoringCtrlIOCB (const LPMONITORING_CTRLIO pCtrlIO)
    {
        for (int i = 0; i < NUM_DIGITAL; i++){
            if(pCtrlIO){  
                g_stDrState.bCtrlBoxDigitalOutput[i] = pCtrlIO->_tOutput._iTargetDO[i];  
                g_stDrState.bCtrlBoxDigitalInput[i]  = pCtrlIO->_tInput._iActualDI[i];  
            }
        }
    }
    // M2.5 or higher
    void DRHWInterface::OnMonitoringCtrlIOExCB (const LPMONITORING_CTRLIO_EX pCtrlIO) 
    {
        //RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"DRHWInterface::OnMonitoringCtrlIOExCB");

        for (int i = 0; i < NUM_DIGITAL; i++){
            if(pCtrlIO){  
                g_stDrState.bCtrlBoxDigitalOutput[i] = pCtrlIO->_tOutput._iTargetDO[i];  
                g_stDrState.bCtrlBoxDigitalInput[i]  = pCtrlIO->_tInput._iActualDI[i];  
            }
        }

        //----- In M2.5 version or higher The following variables were added -----
        for (int i = 0; i < 3; i++)
            g_stDrState.bActualSW[i] = pCtrlIO->_tInput._iActualSW[i];

        for (int i = 0; i < 2; i++){
            g_stDrState.bActualSI[i] = pCtrlIO->_tInput._iActualSI[i];
            g_stDrState.fActualAI[i] = pCtrlIO->_tInput._fActualAI[i];
            g_stDrState.iActualAT[i] = pCtrlIO->_tInput._iActualAT[i];
            g_stDrState.fTargetAO[i] = pCtrlIO->_tOutput._fTargetAO[i];
            g_stDrState.iTargetAT[i] = pCtrlIO->_tOutput._iTargetAT[i];
            g_stDrState.bActualES[i] = pCtrlIO->_tEncoder._iActualES[i];
            g_stDrState.iActualED[i] = pCtrlIO->_tEncoder._iActualED[i];
            g_stDrState.bActualER[i] = pCtrlIO->_tEncoder._iActualER[i];
        }  
        //-------------------------------------------------------------------------
    }

    // M2.4 or lower
    void DRHWInterface::OnMonitoringDataCB(const LPMONITORING_DATA pData)
    {
        // This function is called every 100 msec
        // Only work within 50msec
        //RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"DRHWInterface::OnMonitoringDataCB");

        g_stDrState.nActualMode  = pData->_tCtrl._tState._iActualMode;                  // position control: 0, torque control: 1 ?????
        g_stDrState.nActualSpace = pData->_tCtrl._tState._iActualSpace;                 // joint space: 0, task space: 1    

        for (int i = 0; i < NUM_JOINT; i++){
            if(pData){  
                // joint         
                g_stDrState.fCurrentPosj[i] = pData->_tCtrl._tJoint._fActualPos[i];     // Position Actual Value in INC     
                g_stDrState.fCurrentVelj[i] = pData->_tCtrl._tJoint._fActualVel[i];     // Velocity Actual Value
                g_stDrState.fJointAbs[i]    = pData->_tCtrl._tJoint._fActualAbs[i];     // Position Actual Value in ABS
                g_stDrState.fJointErr[i]    = pData->_tCtrl._tJoint._fActualErr[i];     // Joint Error
                g_stDrState.fTargetPosj[i]  = pData->_tCtrl._tJoint._fTargetPos[i];     // Target Position
                g_stDrState.fTargetVelj[i]  = pData->_tCtrl._tJoint._fTargetVel[i];     // Target Velocity
                // task
                g_stDrState.fCurrentPosx[i]     = pData->_tCtrl._tTask._fActualPos[0][i];   //????? <---------이것 2개다 확인할 것  
                g_stDrState.fCurrentToolPosx[i] = pData->_tCtrl._tTask._fActualPos[1][i];   //????? <---------이것 2개다 확인할 것  
                g_stDrState.fCurrentVelx[i] = pData->_tCtrl._tTask._fActualVel[i];      // Velocity Actual Value
                g_stDrState.fTaskErr[i]     = pData->_tCtrl._tTask._fActualErr[i];      // Task Error
                g_stDrState.fTargetPosx[i]  = pData->_tCtrl._tTask._fTargetPos[i];      // Target Position
                g_stDrState.fTargetVelx[i]  = pData->_tCtrl._tTask._fTargetVel[i];      // Target Velocity
                // Torque
                g_stDrState.fDynamicTor[i]  = pData->_tCtrl._tTorque._fDynamicTor[i];   // Dynamics Torque
                g_stDrState.fActualJTS[i]   = pData->_tCtrl._tTorque._fActualJTS[i];    // Joint Torque Sensor Value
                g_stDrState.fActualEJT[i]   = pData->_tCtrl._tTorque._fActualEJT[i];    // External Joint Torque
                g_stDrState.fActualETT[i]   = pData->_tCtrl._tTorque._fActualETT[i];    // External Task Force/Torque

                g_stDrState.nActualBK[i]    = pData->_tMisc._iActualBK[i];              // brake state     
                g_stDrState.fActualMC[i]    = pData->_tMisc._fActualMC[i];              // motor input current
                g_stDrState.fActualMT[i]    = pData->_tMisc._fActualMT[i];              // motor current temperature
            }
        }
        g_stDrState.nSolutionSpace  = pData->_tCtrl._tTask._iSolutionSpace;             // Solution Space
        g_stDrState.dSyncTime       = pData->_tMisc._dSyncTime;                         // inner clock counter  

        for (int i = 5; i < NUM_BUTTON; i++){
            if(pData){
                g_stDrState.nActualBT[i]    = pData->_tMisc._iActualBT[i];              // robot button state
            }
        }

        for(int i = 0; i < 3; i++){
            for(int j = 0; j < 3; j++){
                if(pData){
                    g_stDrState.fRotationMatrix[j][i] = pData->_tCtrl._tTask._fRotationMatrix[j][i];    // Rotation Matrix
                }
            }
        }

        for (int i = 0; i < NUM_FLANGE_IO; i++){
            if(pData){
                g_stDrState.bFlangeDigitalInput[i]  = pData->_tMisc._iActualDI[i];      // Digital Input data             
                g_stDrState.bFlangeDigitalOutput[i] = pData->_tMisc._iActualDO[i];      // Digital output data
            }
        }
    }

    // M2.5 or higher    
    void DRHWInterface::OnMonitoringDataExCB(const LPMONITORING_DATA_EX pData)
    {
        // This function is called every 100 msec
        // Only work within 50msec
        //RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"DRHWInterface::OnMonitoringDataExCB");

        g_stDrState.nActualMode  = pData->_tCtrl._tState._iActualMode;                  // position control: 0, torque control: 1 ?????
        g_stDrState.nActualSpace = pData->_tCtrl._tState._iActualSpace;                 // joint space: 0, task space: 1    

        for (int i = 0; i < NUM_JOINT; i++){
            if(pData){  
                // joint         
                g_stDrState.fCurrentPosj[i] = pData->_tCtrl._tJoint._fActualPos[i];     // Position Actual Value in INC     
                g_stDrState.fCurrentVelj[i] = pData->_tCtrl._tJoint._fActualVel[i];     // Velocity Actual Value
                g_stDrState.fJointAbs[i]    = pData->_tCtrl._tJoint._fActualAbs[i];     // Position Actual Value in ABS
                g_stDrState.fJointErr[i]    = pData->_tCtrl._tJoint._fActualErr[i];     // Joint Error
                g_stDrState.fTargetPosj[i]  = pData->_tCtrl._tJoint._fTargetPos[i];     // Target Position
                g_stDrState.fTargetVelj[i]  = pData->_tCtrl._tJoint._fTargetVel[i];     // Target Velocity
                // task
                g_stDrState.fCurrentPosx[i]     = pData->_tCtrl._tTask._fActualPos[0][i];   //????? <---------이것 2개다 확인할 것  
                g_stDrState.fCurrentToolPosx[i] = pData->_tCtrl._tTask._fActualPos[1][i];   //????? <---------이것 2개다 확인할 것  
                g_stDrState.fCurrentVelx[i] = pData->_tCtrl._tTask._fActualVel[i];      // Velocity Actual Value
                g_stDrState.fTaskErr[i]     = pData->_tCtrl._tTask._fActualErr[i];      // Task Error
                g_stDrState.fTargetPosx[i]  = pData->_tCtrl._tTask._fTargetPos[i];      // Target Position
                g_stDrState.fTargetVelx[i]  = pData->_tCtrl._tTask._fTargetVel[i];      // Target Velocity
                // Torque
                g_stDrState.fDynamicTor[i]  = pData->_tCtrl._tTorque._fDynamicTor[i];   // Dynamics Torque
                g_stDrState.fActualJTS[i]   = pData->_tCtrl._tTorque._fActualJTS[i];    // Joint Torque Sensor Value
                g_stDrState.fActualEJT[i]   = pData->_tCtrl._tTorque._fActualEJT[i];    // External Joint Torque
                g_stDrState.fActualETT[i]   = pData->_tCtrl._tTorque._fActualETT[i];    // External Task Force/Torque

                g_stDrState.nActualBK[i]    = pData->_tMisc._iActualBK[i];              // brake state     
                g_stDrState.fActualMC[i]    = pData->_tMisc._fActualMC[i];              // motor input current
                g_stDrState.fActualMT[i]    = pData->_tMisc._fActualMT[i];              // motor current temperature
            }
        }
        g_stDrState.nSolutionSpace  = pData->_tCtrl._tTask._iSolutionSpace;             // Solution Space
        g_stDrState.dSyncTime       = pData->_tMisc._dSyncTime;                         // inner clock counter  

        for (int i = 5; i < NUM_BUTTON; i++){
            if(pData){
                g_stDrState.nActualBT[i]    = pData->_tMisc._iActualBT[i];              // robot button state
            }
        }

        for(int i = 0; i < 3; i++){
            for(int j = 0; j < 3; j++){
                if(pData){
                    g_stDrState.fRotationMatrix[j][i] = pData->_tCtrl._tTask._fRotationMatrix[j][i];    // Rotation Matrix
                }
            }
        }

        for (int i = 0; i < NUM_FLANGE_IO; i++){
            if(pData){
                g_stDrState.bFlangeDigitalInput[i]  = pData->_tMisc._iActualDI[i];      // Digital Input data             
                g_stDrState.bFlangeDigitalOutput[i] = pData->_tMisc._iActualDO[i];      // Digital output data
            }
        }

        //----- In M2.5 version or higher The following variables were added -----
        for (int i = 0; i < NUM_JOINT; i++){
            g_stDrState.fActualW2B[i] = pData->_tCtrl._tWorld._fActualW2B[i];
            g_stDrState.fCurrentVelW[i] = pData->_tCtrl._tWorld._fActualVel[i];
            g_stDrState.fWorldETT[i] = pData->_tCtrl._tWorld._fActualETT[i];
            g_stDrState.fTargetPosW[i] = pData->_tCtrl._tWorld._fTargetPos[i];
            g_stDrState.fTargetVelW[i] = pData->_tCtrl._tWorld._fTargetVel[i];
            g_stDrState.fCurrentVelU[i] = pData->_tCtrl._tWorld._fActualVel[i];
            g_stDrState.fUserETT[i] = pData->_tCtrl._tUser._fActualETT[i];
            g_stDrState.fTargetPosU[i] = pData->_tCtrl._tUser._fTargetPos[i];
            g_stDrState.fTargetVelU[i] = pData->_tCtrl._tUser._fTargetVel[i];
        }    

        for(int i = 0; i < 2; i++){
            for(int j = 0; j < 6; j++){
                g_stDrState.fCurrentPosW[i][j] = pData->_tCtrl._tWorld._fActualPos[i][j];
                g_stDrState.fCurrentPosU[i][j] = pData->_tCtrl._tUser._fActualPos[i][j];
            }
        }

        for(int i = 0; i < 3; i++){
            for(int j = 0; j < 3; j++){
                g_stDrState.fRotationMatrixWorld[j][i] = pData->_tCtrl._tWorld._fRotationMatrix[j][i];
                g_stDrState.fRotationMatrixUser[j][i] = pData->_tCtrl._tUser._fRotationMatrix[j][i];
            }
        }

        g_stDrState.iActualUCN = pData->_tCtrl._tUser._iActualUCN;
        g_stDrState.iParent    = pData->_tCtrl._tUser._iParent;
        //-------------------------------------------------------------------------
    }

    void DRHWInterface::OnMonitoringModbusCB (const LPMONITORING_MODBUS pModbus)
    {
        g_stDrState.nRegCount = pModbus->_iRegCount;
        for (int i = 0; i < pModbus->_iRegCount; i++){
            cout << "[callback OnMonitoringModbusCB] " << pModbus->_tRegister[i]._szSymbol <<": " << pModbus->_tRegister[i]._iValue<< endl;
            g_stDrState.strModbusSymbol[i] = pModbus->_tRegister[i]._szSymbol;
            g_stDrState.nModbusValue[i]    = pModbus->_tRegister[i]._iValue;
        }
    }

    void DRHWInterface::OnMonitoringStateCB(const ROBOT_STATE eState)
    {
        //This function is called when the state changes.
        //RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"DRHWInterface::OnMonitoringStateCB");    
        // Only work within 50msec
        RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"On Monitor State");
        switch((unsigned char)eState)
        {
#if 0 // TP initializing logic, Don't use in API level. (If you want to operate without TP, use this logic)       
        case eSTATE_NOT_READY:
        if (g_bHasControlAuthority) Drfl.SetRobotControl(CONTROL_INIT_CONFIG);
            break;
        case eSTATE_INITIALIZING:
            // add initalizing logic
            if (g_bHasControlAuthority) Drfl.SetRobotControl(CONTROL_ENABLE_OPERATION);
            break;
#endif      
        case STATE_EMERGENCY_STOP:
            // popup
            break;
        case STATE_STANDBY:
        case STATE_MOVING:
        case STATE_TEACHING:
            break;
        case STATE_SAFE_STOP:
            if (g_bHasControlAuthority) {
                Drfl.SetSafeStopResetType(SAFE_STOP_RESET_TYPE_DEFAULT);
                Drfl.SetRobotControl(CONTROL_RESET_SAFET_STOP);
            }
            break;
        case STATE_SAFE_OFF:
            if (g_bHasControlAuthority){
                Drfl.SetRobotControl(CONTROL_SERVO_ON);
				Drfl.SetRobotMode(ROBOT_MODE_MANUAL);   //Idle Servo Off 후 servo on 하는 상황 발생 시 set_robot_mode 명령을 전송해 manual 로 전환. add 2020/04/28
            } 
            break;
        case STATE_SAFE_STOP2:
            if (g_bHasControlAuthority) Drfl.SetRobotControl(CONTROL_RECOVERY_SAFE_STOP);
            break;
        case STATE_SAFE_OFF2:
            if (g_bHasControlAuthority) {
                Drfl.SetRobotControl(CONTROL_RECOVERY_SAFE_OFF);
            }
            break;
        case STATE_RECOVERY:
            Drfl.SetRobotControl(CONTROL_RESET_RECOVERY);
            break;
        default:
            break;
        }

        cout << "[callback OnMonitoringStateCB] current state: " << GetRobotStateString((int)eState) << endl;
        g_stDrState.nRobotState = (int)eState;
        strncpy(g_stDrState.strRobotState, GetRobotStateString((int)eState), MAX_SYMBOL_SIZE); 
    }

    void DRHWInterface::OnMonitoringAccessControlCB(const MONITORING_ACCESS_CONTROL eAccCtrl)
    {
        // Only work within 50msec

        cout << "[callback OnMonitoringAccessControlCB] eAccCtrl: " << eAccCtrl << endl;
        switch(eAccCtrl)
        {
        case MONITORING_ACCESS_CONTROL_REQUEST:
            Drfl.ManageAccessControl(MANAGE_ACCESS_CONTROL_RESPONSE_NO);
            //Drfl.TransitControlAuth(MANaGE_ACCESS_CONTROL_RESPONSE_YES);
            break;
        case MONITORING_ACCESS_CONTROL_GRANT:
            cout  << "access control granted" << endl;
            g_bHasControlAuthority = TRUE;
            OnMonitoringStateCB(Drfl.GetRobotState());
            break;
        case MONITORING_ACCESS_CONTROL_DENY:
            RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"Access control deny !!!!!!!!!!!!!!!");
            break;
        case MONITORING_ACCESS_CONTROL_LOSS:
            g_bHasControlAuthority = FALSE;
            if (g_bTpInitailizingComplted) {
                Drfl.ManageAccessControl(MANAGE_ACCESS_CONTROL_REQUEST);
                //Drfl.TransitControlAuth(MANAGE_ACCESS_CONTROL_FORCE_REQUEST);
            }
            break;
        default:
            break;
        }
        g_stDrState.nAccessControl = (int)eAccCtrl;
    }

    void DRHWInterface::OnLogAlarm(LPLOG_ALARM pLogAlarm)
    {
        //This function is called when an error occurs.
        auto PubRobotError = g_node->create_publisher<dsr_msgs2::msg::RobotError>("error", 100);
        dsr_msgs2::msg::RobotError msg;

        switch(pLogAlarm->_iLevel)
        {
        case LOG_LEVEL_SYSINFO:
            RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"[callback OnLogAlarm]");
            RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2")," level : %d",(unsigned int)pLogAlarm->_iLevel);
            RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2")," group : %d",(unsigned int)pLogAlarm->_iGroup);
            RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2")," index : %d", pLogAlarm->_iIndex);
            RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2")," param : %s", pLogAlarm->_szParam[0] );
            RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2")," param : %s", pLogAlarm->_szParam[1] );
            RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2")," param : %s", pLogAlarm->_szParam[2] );
            break;
        case LOG_LEVEL_SYSWARN:
            RCLCPP_WARN(rclcpp::get_logger("dsr_hw_interface2"),"[callback OnLogAlarm]");
            RCLCPP_WARN(rclcpp::get_logger("dsr_hw_interface2")," level : %d",(unsigned int)pLogAlarm->_iLevel);
            RCLCPP_WARN(rclcpp::get_logger("dsr_hw_interface2")," group : %d",(unsigned int)pLogAlarm->_iGroup);
            RCLCPP_WARN(rclcpp::get_logger("dsr_hw_interface2")," index : %d", pLogAlarm->_iIndex);
            RCLCPP_WARN(rclcpp::get_logger("dsr_hw_interface2")," param : %s", pLogAlarm->_szParam[0] );
            RCLCPP_WARN(rclcpp::get_logger("dsr_hw_interface2")," param : %s", pLogAlarm->_szParam[1] );
            RCLCPP_WARN(rclcpp::get_logger("dsr_hw_interface2")," param : %s", pLogAlarm->_szParam[2] );
            break;
        case LOG_LEVEL_SYSERROR:
        default:
            RCLCPP_ERROR(rclcpp::get_logger("dsr_hw_interface2"),"[callback OnLogAlarm]");
            RCLCPP_ERROR(rclcpp::get_logger("dsr_hw_interface2")," level : %d",(unsigned int)pLogAlarm->_iLevel);
            RCLCPP_ERROR(rclcpp::get_logger("dsr_hw_interface2")," group : %d",(unsigned int)pLogAlarm->_iGroup);
            RCLCPP_ERROR(rclcpp::get_logger("dsr_hw_interface2")," index : %d", pLogAlarm->_iIndex);
            RCLCPP_ERROR(rclcpp::get_logger("dsr_hw_interface2")," param : %s", pLogAlarm->_szParam[0] );
            RCLCPP_ERROR(rclcpp::get_logger("dsr_hw_interface2")," param : %s", pLogAlarm->_szParam[1] );
            RCLCPP_ERROR(rclcpp::get_logger("dsr_hw_interface2")," param : %s", pLogAlarm->_szParam[2] );
            break;
        }

        g_stDrError.nLevel=(unsigned int)pLogAlarm->_iLevel;
        g_stDrError.nGroup=(unsigned int)pLogAlarm->_iGroup;
        g_stDrError.nCode=pLogAlarm->_iIndex;
        strncpy(g_stDrError.strMsg1, pLogAlarm->_szParam[0], MAX_STRING_SIZE);
        strncpy(g_stDrError.strMsg2, pLogAlarm->_szParam[1], MAX_STRING_SIZE);
        strncpy(g_stDrError.strMsg3, pLogAlarm->_szParam[2], MAX_STRING_SIZE);

        msg.level=g_stDrError.nLevel;
        msg.group=g_stDrError.nGroup;
        msg.code=g_stDrError.nCode;
        msg.msg1=g_stDrError.strMsg1;
        msg.msg2=g_stDrError.strMsg2;
        msg.msg3=g_stDrError.strMsg3;

        PubRobotError->publish(msg);
    }

    //----- register the call-back functions end -------------------------------------
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 
    void MsgScriber(const dsr_msgs2::msg::RobotStop::SharedPtr msg)
    {
        RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"receive msg.stop_mode = %d", msg->stop_mode);
        RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"receive msg.stop_mode = %d", msg->stop_mode);
        RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"receive msg.stop_mode = %d", msg->stop_mode);

        Drfl.MoveStop((STOP_TYPE)msg->stop_mode);
    } 

    int DRHWInterface::MsgPublisher_RobotState()
    {
        dsr_msgs2::msg::RobotState msg;
        dsr_msgs2::msg::ModbusState modbus_state;
        memcpy(&m_stDrState, &g_stDrState, sizeof(DR_STATE));
         
        msg.robot_state         = m_stDrState.nRobotState;
        msg.robot_state_str     = m_stDrState.strRobotState;
        msg.actual_mode         = m_stDrState.nActualMode;
        msg.actual_space        = m_stDrState.nActualSpace;

        for (int i = 0; i < NUM_JOINT; i++)
        {
            msg.current_posj[i]    = m_stDrState.fCurrentPosj[i];
            msg.current_velj[i]    = m_stDrState.fCurrentVelj[i];
            msg.joint_abs[i]       = m_stDrState.fJointAbs[i];
            msg.joint_err[i]       = m_stDrState.fJointErr[i];
            msg.target_posj[i]     = m_stDrState.fTargetPosj[i];
            msg.target_velj[i]     = m_stDrState.fTargetVelj[i];

            msg.current_posx[i]      = m_stDrState.fCurrentPosx[i];
            msg.current_tool_posx[i] = m_stDrState.fCurrentToolPosx[i];
            msg.current_velx[i]    = m_stDrState.fCurrentVelx[i];
            msg.task_err[i]        = m_stDrState.fTaskErr[i];
            msg.target_velx[i]     = m_stDrState.fTargetVelx[i];
            msg.target_posx[i]     = m_stDrState.fTargetPosx[i];

            msg.dynamic_tor[i]     = m_stDrState.fDynamicTor[i];
            msg.actual_jts[i]      = m_stDrState.fActualJTS[i];
            msg.actual_ejt[i]      = m_stDrState.fActualEJT[i];
            msg.actual_ett[i]      = m_stDrState.fActualETT[i];


            msg.actual_bk[i]       = m_stDrState.nActualBK[i];
            msg.actual_mc[i]       = m_stDrState.fActualMC[i];
            msg.actual_mt[i]       = m_stDrState.fActualMT[i];
        }
        msg.solution_space      = m_stDrState.nSolutionSpace;
        msg.sync_time           = m_stDrState.dSyncTime;

        std_msgs::msg::Float64MultiArray arr;

        for (int i = 0; i < 3; i++){
            arr.data.clear();
            for (int j = 0; j < 3; j++){
                arr.data.push_back(m_stDrState.fRotationMatrix[i][j]);
            }
            msg.rotation_matrix.push_back(arr);
        }
        
        for (int i = 0; i < NUM_BUTTON; i++){
            msg.actual_bt[i] = m_stDrState.nActualBT[i];
        }
        for (int i = 0; i < NUM_DIGITAL; i++){
            msg.ctrlbox_digital_input[i]    = m_stDrState.bCtrlBoxDigitalInput[i];
            msg.ctrlbox_digital_output[i]   = m_stDrState.bCtrlBoxDigitalOutput[i];    
        }
        for (int i = 0; i < NUM_FLANGE_IO; i++){
            msg.flange_digital_input[i]     = m_stDrState.bFlangeDigitalInput[i];
            msg.flange_digital_output[i]    = m_stDrState.bFlangeDigitalOutput[i];
        }
        //msg.io_modbus;    GJH
        for (int i = 0; i < m_stDrState.nRegCount; i++){
            modbus_state.modbus_symbol   = m_stDrState.strModbusSymbol[i];
            modbus_state.modbus_value    = m_stDrState.nModbusValue[i];
            msg.modbus_state.push_back(modbus_state);
        }
        //msg.error;        GJH
        msg.access_control      = m_stDrState.nAccessControl;
        msg.homming_completed   = m_stDrState.bHommingCompleted;
        msg.tp_initialized      = m_stDrState.bTpInitialized; 
        msg.mastering_need      = m_stDrState.bMasteringNeed;
        msg.drl_stopped         = m_stDrState.bDrlStopped;
        msg.disconnected        = m_stDrState.bDisconnected;

        //--- The following messages have been updated since version M2.50 or higher ---
        if(m_nVersionDRCF >= 120500)    //M2.5 or later        
        {
            for (int i = 0; i < NUM_JOINT; i++){
                msg.f_actual_w2b[i]   = m_stDrState.fActualW2B[i];
                msg.f_current_vel_world[i] = m_stDrState.fCurrentVelW[i];
                msg.f_world_ext_target_torque[i]    = m_stDrState.fWorldETT[i];
                msg.f_world_ext_target_torque[i]  = m_stDrState.fTargetPosW[i];
                msg.f_target_vel_world[i]  = m_stDrState.fTargetVelW[i];
                msg.f_current_vel_user[i] = m_stDrState.fCurrentVelU[i];
                msg.f_user_ext_task_torque[i]     = m_stDrState.fUserETT[i];
                msg.f_target_pos_user[i]  = m_stDrState.fTargetPosU[i];
                msg.f_target_vel_user[i]  = m_stDrState.fTargetVelU[i];
            }      

            for(int i = 0; i < 2; i++){
                arr.data.clear();
                for(int j = 0; j < 6; j++){
                    arr.data.push_back(m_stDrState.fCurrentPosW[i][j]);
                }
                msg.f_current_pos_world.push_back(arr);
            }
            for(int i = 0; i < 2; i++){
                arr.data.clear();
                for(int j = 0; j < 6; j++){
                    arr.data.push_back(m_stDrState.fCurrentPosU[i][j]);
                }
                msg.f_current_pos_user.push_back(arr);
            }
            for(int i = 0; i < 3; i++){
                arr.data.clear();
                for(int j = 0; j < 3; j++){
                    arr.data.push_back(m_stDrState.fRotationMatrixWorld[i][j]);
                }
                msg.f_rotation_matrix_world.push_back(arr);
            }
            for(int i = 0; i < 3; i++){
                arr.data.clear();
                for(int j = 0; j < 3; j++){
                    arr.data.push_back(m_stDrState.fRotationMatrixUser[i][j]);
                }
                msg.f_rotation_matrix_user.push_back(arr);
            }

            msg.i_actual_user_coord_num = m_stDrState.iActualUCN;
            msg.i_coord_ref    = m_stDrState.iParent;

            for (int i = 0; i < 3; i++)
                msg.b_actual_switch_input[i] = m_stDrState.bActualSW[i];

            for (int i = 0; i < 2; i++){
                msg.b_actual_safety_input[i] = m_stDrState.bActualSI[i];
                msg.f_actual_analog_input[i] = m_stDrState.fActualAI[i];
                msg.i_actual_analog_input_type[i] = m_stDrState.iActualAT[i];
                msg.f_target_analog_output[i] = m_stDrState.fTargetAO[i];
                msg.i_target_analog_output_type[i] = m_stDrState.iTargetAT[i];
                msg.b_actual_encorder_strove_signal[i] = m_stDrState.bActualES[i];
                msg.i_actual_encorder_raw_data[i] = m_stDrState.iActualED[i];
                msg.b_actual_encorder_reset_signal[i] = m_stDrState.bActualER[i];
            }
        }        
        //------------------------------------------------------------------------------

        m_PubRobotState->publish(msg);

        return 0; 
    }

    int DRHWInterface::MsgPublisher_JointState()
    { 
        sensor_msgs::msg::JointState msg;

        memcpy(&m_stDrState, &g_stDrState, sizeof(DR_STATE));
        msg.header.stamp.sec;
        msg.name.push_back("joint1");
        msg.name.push_back("joint2");
        msg.name.push_back("joint3");
        msg.name.push_back("joint4");
        msg.name.push_back("joint5");
        msg.name.push_back("joint6");

        double now_sec = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        //printf("Current time: %lf seconds since the Epoch\n", now_sec);

        long int now_ns;
        struct timespec spec;
        clock_gettime(CLOCK_REALTIME, &spec);
        now_ns = spec.tv_nsec;;
        //printf("Current time: %ld nonoseconds since the Epoch\n", now_ns);

        for (int i = 0; i < NUM_JOINT; i++)
        {
            //msg.current_posj[i]    = m_stDrState.fCurrentPosj[i];
            ///msg.position[i] = m_stDrState.fCurrentPosj[i];
            msg.header.stamp.sec = (long)now_sec;
            msg.header.stamp.nanosec = now_ns;
            msg.position.push_back( deg2rad(m_stDrState.fCurrentPosj[i]) );
        
        }

        m_PubJointState->publish(msg);

        return 0; 
    }

    void DRHWInterface::thread_subscribe(rclcpp::Node::SharedPtr nh)
    {
        rclcpp::executors::MultiThreadedExecutor executor;
        auto node = rclcpp::Node::make_shared("stop_subscriber_node");
        executor.add_node(node);

        auto sub_robot_stop = node->create_subscription<dsr_msgs2::msg::RobotStop>("stop", 100, MsgScriber);
        executor.spin();

        RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"thread_subscribe is stopped ...");
    }

    void DRHWInterface::thread_publisher(DRHWInterface* pDRHWInterface, rclcpp::Node::SharedPtr nh, int nPubRate)
    {
        auto PubRobotState = nh->create_publisher<dsr_msgs2::msg::RobotState>("state",100); //불필요 삭제 예정 

        dsr_msgs2::msg::RobotState msg; //불필요 삭제 예정 

        rclcpp::Rate r(nPubRate);

        while(rclcpp::ok())
        {
            if(pDRHWInterface) pDRHWInterface->MsgPublisher_RobotState();
            r.sleep();
        }

        RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"thread_publisher is stopped ...");
    }          

    void DRHWInterface::thread_publisher_direct_access_joint_states(DRHWInterface* pDRHWInterface, rclcpp::Node::SharedPtr nh, int nPubRate)
    {
        auto PubJointState = nh->create_publisher<sensor_msgs::msg::JointState>("joint_states",100); //불필요 삭제 예정 

        sensor_msgs::msg::JointState msg; 

        rclcpp::Rate r(nPubRate);

        while(rclcpp::ok())
        {
            RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"thread_publisher_direct_access_joint_states Running ...");

            if(pDRHWInterface) pDRHWInterface->MsgPublisher_JointState();
            r.sleep();
        }
    }          

    DRHWInterface::DRHWInterface(rclcpp::Node::SharedPtr& nh):private_nh_(nh)
    {
        //----- get parameters --------------------------------------------------------
        g_node->get_parameter("name", m_strRobotName);
        g_node->get_parameter("model", m_strRobotModel);
        g_node->get_parameter("gripper", m_strRobotGripper);

        RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"name = %s",m_strRobotName.c_str());
        RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"model = %s",m_strRobotModel.c_str());
        RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"gripper = %s",m_strRobotGripper.c_str());
        //------------------------------------------------------------------------------


        RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"name_space is %s, %s",m_strRobotName.c_str(), m_strRobotModel.c_str());

        RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"constructed");

        /*TODO ROS2 ???
        ros::V_string arm_joint_names;
        if(m_strRobotGripper == "robotiq_2f"){
            arm_joint_names =
            boost::assign::list_of("joint1")("joint2")("joint3")("joint4")("joint5")("joint6")("robotiq_85_left_knuckle_joint").convert_to_container<ros::V_string>();
        }
        else if(m_strRobotGripper == "none")
        {
            arm_joint_names =
            boost::assign::list_of("joint1")("joint2")("joint3")("joint4")("joint5")("joint6").convert_to_container<ros::V_string>();
        }
        for(unsigned int i = 0; i < arm_joint_names.size(); i++){
            hardware_interface::JointStateHandle jnt_state_handle(
                arm_joint_names[i],
                &joints[i].pos,
                &joints[i].vel,
                &joints[i].eff);
            jnt_state_interface.registerHandle(jnt_state_handle);

            hardware_interface::JointHandle jnt_pos_handle(
                jnt_state_handle,
                &joints[i].cmd);
            jnt_pos_interface.registerHandle(jnt_pos_handle);
        }
        registerInterface(&jnt_state_interface);
        registerInterface(&jnt_pos_interface);
        */

        // Publisher msg        
        m_PubJointState  = private_nh_->create_publisher<sensor_msgs::msg::JointState>("joint_states",100);
        m_PubRobotState = private_nh_->create_publisher<dsr_msgs2::msg::RobotState>("state",100);
        ///m_PubRobotError = private_nh_->create_publisher<dsr_msgs2::msg::RobotError>("error",100);

        
        //TODO gazebo에 joint position 전달
        //ROS2 m_PubtoGazebo = private_nh_.advertise<std_msgs::Float64MultiArray>("/dsr_joint_position_controller/command",10);
        m_PubtoGazebo = private_nh_->create_publisher<std_msgs::msg::Float64MultiArray>("commands",100);
        
        //TODO moveit의 trajectory/goal를 받아 제어기로 전달
        //ROS2 m_sub_joint_trajectory = private_nh_.subscribe("dsr_joint_trajectory_controller/follow_joint_trajectory/goal", 10, &DRHWInterface::trajectoryCallback, this);
        m_sub_joint_trajectory = private_nh_->create_subscription<moveit_msgs::msg::DisplayTrajectory>("display_planned_path", 10, std::bind(&DRHWInterface::trajectoryCallback, this, std::placeholders::_1));

        ///m_sub_test = private_nh_->create_subscription<std_msgs::msg::String>("topic", 10, std::bind(&DRHWInterface::testSubCallback, this, std::placeholders::_1));

        /*TODO
        // topic echo 명령으로 제어기에 전달
        m_sub_joint_position = private_nh_.subscribe("dsr_joint_position_controller/command", 10, &DRHWInterface::positionCallback, this);
        
        ros::NodeHandle nh_temp;
        m_SubSerialRead = nh_temp.subscribe("serial_read", 100, &Serial_comm::read_callback, &ser_comm);
        m_PubSerialWrite = nh_temp.advertise<std_msgs::String>("serial_write", 100);

        // subscribe : Multi-JOG topic msg
        m_sub_jog_multi_axis = private_nh_.subscribe("jog_multi", 10, &DRHWInterface::jogCallback, this);  
        */

        // system Operations
        //ROS2 m_nh_system[0] = private_nh_.advertiseService("system/set_robot_mode", &DRHWInterface::set_robot_mode_cb, this);
        //auto server_system_0 = private_nh_->create_service<dsr_msgs2::srv::SetRobotMode>("system/set_robot_mode", DRHWInterface::set_robot_mode_cb);
        m_nh_srv_set_robot_mode             = private_nh_->create_service<dsr_msgs2::srv::SetRobotMode>("system/set_robot_mode", DRHWInterface::set_robot_mode_cb);          

        //  motion Operations
        //auto server_motion_service_0  = private_nh_->create_service<dsr_msgs2::srv::MoveJoint>("motion/move_joint", DRHWInterface::movej_cb);
        m_nh_srv_move_joint                 = private_nh_->create_service<dsr_msgs2::srv::MoveJoint>("motion/move_joint", DRHWInterface::move_joint_cb);                                
        m_nh_srv_move_line                  = private_nh_->create_service<dsr_msgs2::srv::MoveLine>("motion/move_line", DRHWInterface::move_line_cb);                        
        m_nh_srv_move_jointx                = private_nh_->create_service<dsr_msgs2::srv::MoveJointx>("motion/move_jointx", DRHWInterface::move_jointx_cb);               
        m_nh_srv_move_circle                = private_nh_->create_service<dsr_msgs2::srv::MoveCircle>("motion/move_circle", DRHWInterface::move_circle_cb);       
        m_nh_srv_move_spline_joint          = private_nh_->create_service<dsr_msgs2::srv::MoveSplineJoint>("motion/move_spline_joint", DRHWInterface::move_spline_joint_cb);      
        m_nh_srv_move_spline_task           = private_nh_->create_service<dsr_msgs2::srv::MoveSplineTask>("motion/move_spline_task", DRHWInterface::move_spline_task_cb);          
        m_nh_srv_move_blending              = private_nh_->create_service<dsr_msgs2::srv::MoveBlending>("motion/move_blending", DRHWInterface::move_blending_cb);          
        m_nh_srv_move_spiral                = private_nh_->create_service<dsr_msgs2::srv::MoveSpiral>("motion/move_spiral", DRHWInterface::move_spiral_cb);              
        m_nh_srv_move_periodic              = private_nh_->create_service<dsr_msgs2::srv::MovePeriodic>("motion/move_periodic", DRHWInterface::move_periodic_cb);              

        // Gripper Operations
        // Serial Operations  


        //  system
//OK    m_nh_srv_set_robot_mode             = private_nh_->create_service<dsr_msgs2::srv::SetRobotMode>("system/set_robot_mode", DRHWInterface::set_robot_mode_cb);          
        m_nh_srv_get_robot_mode             = private_nh_->create_service<dsr_msgs2::srv::GetRobotMode>("system/get_robot_mode", DRHWInterface::get_robot_mode_cb);     
        m_nh_srv_set_robot_system           = private_nh_->create_service<dsr_msgs2::srv::SetRobotSystem>("system/set_robot_system", DRHWInterface::set_robot_system_cb);         
        m_nh_srv_get_robot_system           = private_nh_->create_service<dsr_msgs2::srv::GetRobotSystem>("system/get_robot_system", DRHWInterface::get_robot_system_cb);         
        m_nh_srv_get_robot_state            = private_nh_->create_service<dsr_msgs2::srv::GetRobotState>("system/get_robot_state", DRHWInterface::get_robot_state_cb);        
        m_nh_srv_set_robot_speed_mode       = private_nh_->create_service<dsr_msgs2::srv::SetRobotSpeedMode>("system/set_robot_speed_mode", DRHWInterface::set_robot_speed_mode_cb);    
        m_nh_srv_get_robot_speed_mode       = private_nh_->create_service<dsr_msgs2::srv::GetRobotSpeedMode>("system/get_robot_speed_mode", DRHWInterface::get_robot_speed_mode_cb);       
        m_nh_srv_get_current_pose           = private_nh_->create_service<dsr_msgs2::srv::GetCurrentPose>("system/get_current_pose", DRHWInterface::get_current_pose_cb);   
        m_nh_srv_set_safe_stop_reset_type   = private_nh_->create_service<dsr_msgs2::srv::SetSafeStopResetType>("system/set_safe_stop_reset_type", DRHWInterface::set_safe_stop_reset_type_cb);           
        m_nh_srv_get_last_alarm             = private_nh_->create_service<dsr_msgs2::srv::GetLastAlarm>("system/get_last_alarm", DRHWInterface::get_last_alarm_cb);   

        //  motion
//OK    m_nh_srv_move_joint                 = private_nh_->create_service<dsr_msgs2::srv::MoveJoint>("motion/move_joint", DRHWInterface::move_joint_cb);                                
//OK    m_nh_srv_move_line                  = private_nh_->create_service<dsr_msgs2::srv::MoveLine>("motion/move_line", DRHWInterface::move_line_cb);                        
//OK    m_nh_srv_move_jointx                = private_nh_->create_service<dsr_msgs2::srv::MoveJointx>("motion/move_jointx", DRHWInterface::move_jointx_cb);               
//OK    m_nh_srv_move_circle                = private_nh_->create_service<dsr_msgs2::srv::MoveCircle>("motion/move_circle", DRHWInterface::move_circle_cb);       
//OK    m_nh_srv_move_spline_joint          = private_nh_->create_service<dsr_msgs2::srv::MoveSplineJoint>("motion/move_spline_joint", DRHWInterface::move_spline_joint_cb);      
//OK    m_nh_srv_move_spline_task           = private_nh_->create_service<dsr_msgs2::srv::MoveSplineTask>("motion/move_spline_task", DRHWInterface::move_spline_task_cb);          
//OK    m_nh_srv_move_blending              = private_nh_->create_service<dsr_msgs2::srv::MoveBlending>("motion/move_blending", DRHWInterface::move_blending_cb);          
//OK    m_nh_srv_move_spiral                = private_nh_->create_service<dsr_msgs2::srv::MoveSpiral>("motion/move_spiral", DRHWInterface::move_spiral_cb);              
//OK    m_nh_srv_move_periodic              = private_nh_->create_service<dsr_msgs2::srv::MovePeriodic>("motion/move_periodic", DRHWInterface::move_periodic_cb);              
        m_nh_srv_move_wait                  = private_nh_->create_service<dsr_msgs2::srv::MoveWait>("motion/move_wait", DRHWInterface::move_wait_cb);                  
        m_nh_srv_jog                        = private_nh_->create_service<dsr_msgs2::srv::Jog>("motion/jog", DRHWInterface::jog_cb);              
        m_nh_srv_jog_multi                  = private_nh_->create_service<dsr_msgs2::srv::JogMulti>("motion/jog_multi", DRHWInterface::jog_multi_cb);                      
        m_nh_srv_move_pause                 = private_nh_->create_service<dsr_msgs2::srv::MovePause>("motion/move_pause", DRHWInterface::move_pause_cb);                      
        m_nh_srv_move_stop                  = private_nh_->create_service<dsr_msgs2::srv::MoveStop>("motion/move_stop", DRHWInterface::move_stop_cb);                  
        m_nh_srv_move_resume                = private_nh_->create_service<dsr_msgs2::srv::MoveResume>("motion/move_resume", DRHWInterface::move_resume_cb);                  
        m_nh_srv_trans                      = private_nh_->create_service<dsr_msgs2::srv::Trans>("motion/trans", DRHWInterface::trans_cb);                  
        m_nh_srv_fkin                       = private_nh_->create_service<dsr_msgs2::srv::Fkin>("motion/fkin", DRHWInterface::fkin_cb);              
        m_nh_srv_ikin                       = private_nh_->create_service<dsr_msgs2::srv::Ikin>("motion/ikin", DRHWInterface::ikin_cb);              
        m_nh_srv_set_ref_coord              = private_nh_->create_service<dsr_msgs2::srv::SetRefCoord>("motion/set_ref_coord", DRHWInterface::set_ref_coord_cb);                  
        m_nh_srv_move_home                  = private_nh_->create_service<dsr_msgs2::srv::MoveHome>("motion/move_home", DRHWInterface::move_home_cb);                  
        m_nh_srv_check_motion               = private_nh_->create_service<dsr_msgs2::srv::CheckMotion>("motion/check_motion", DRHWInterface::check_motion_cb);                  
        m_nh_srv_change_operation_speed     = private_nh_->create_service<dsr_msgs2::srv::ChangeOperationSpeed>("motion/change_operation_speed", DRHWInterface::change_operation_speed_cb);                  
        m_nh_srv_enable_alter_motion        = private_nh_->create_service<dsr_msgs2::srv::EnableAlterMotion>("motion/enable_alter_motion", DRHWInterface::enable_alter_motion_cb);                      
        m_nh_srv_alter_motion               = private_nh_->create_service<dsr_msgs2::srv::AlterMotion>("motion/alter_motion", DRHWInterface::alter_motion_cb);              
        m_nh_srv_disable_alter_motion       = private_nh_->create_service<dsr_msgs2::srv::DisableAlterMotion>("motion/disable_alter_motion", DRHWInterface::disable_alter_motion_cb);                  
        m_nh_srv_set_singularity_handling   = private_nh_->create_service<dsr_msgs2::srv::SetSingularityHandling>("motion/set_singularity_handling", DRHWInterface::set_singularity_handling_cb);                      

        //  auxiliary_control
        m_nh_srv_get_control_mode               = private_nh_->create_service<dsr_msgs2::srv::GetControlMode>("aux_control/get_control_mode", DRHWInterface::get_control_mode_cb);                           
        m_nh_srv_get_control_space              = private_nh_->create_service<dsr_msgs2::srv::GetControlSpace>("aux_control/get_control_space", DRHWInterface::get_control_space_cb);                          
        m_nh_srv_get_current_posj               = private_nh_->create_service<dsr_msgs2::srv::GetCurrentPosj>("aux_control/get_current_posj", DRHWInterface::get_current_posj_cb);                                            
        m_nh_srv_get_current_velj               = private_nh_->create_service<dsr_msgs2::srv::GetCurrentVelj>("aux_control/get_current_velj", DRHWInterface::get_current_velj_cb);                                
        m_nh_srv_get_desired_posj               = private_nh_->create_service<dsr_msgs2::srv::GetDesiredPosj>("aux_control/get_desired_posj", DRHWInterface::get_desired_posj_cb);         
        m_nh_srv_get_desired_velj               = private_nh_->create_service<dsr_msgs2::srv::GetDesiredVelj>("aux_control/get_desired_velj", DRHWInterface::get_desired_velj_cb);                                   
        m_nh_srv_get_current_posx               = private_nh_->create_service<dsr_msgs2::srv::GetCurrentPosx>("aux_control/get_current_posx", DRHWInterface::get_current_posx_cb);                               
        m_nh_srv_get_current_tool_flange_posx   = private_nh_->create_service<dsr_msgs2::srv::GetCurrentToolFlangePosx>("aux_control/get_current_tool_flange_posx", DRHWInterface::get_current_tool_flange_posx_cb);                 
        m_nh_srv_get_current_velx               = private_nh_->create_service<dsr_msgs2::srv::GetCurrentVelx>("aux_control/get_current_velx", DRHWInterface::get_current_velx_cb);                         
        m_nh_srv_get_desired_posx               = private_nh_->create_service<dsr_msgs2::srv::GetDesiredPosx>("aux_control/get_desired_posx", DRHWInterface::get_desired_posx_cb);     
        m_nh_srv_get_desired_velx               = private_nh_->create_service<dsr_msgs2::srv::GetDesiredVelx>("aux_control/get_desired_velx", DRHWInterface::get_desired_velx_cb);                             
        m_nh_srv_get_current_solution_space     = private_nh_->create_service<dsr_msgs2::srv::GetCurrentSolutionSpace>("aux_control/get_current_solution_space", DRHWInterface::get_current_solution_space_cb);                     
        m_nh_srv_get_current_rotm               = private_nh_->create_service<dsr_msgs2::srv::GetCurrentRotm>("aux_control/get_current_rotm", DRHWInterface::get_current_rotm_cb);                     
        m_nh_srv_get_joint_torque               = private_nh_->create_service<dsr_msgs2::srv::GetJointTorque>("aux_control/get_joint_torque", DRHWInterface::get_joint_torque_cb);                     
        m_nh_srv_get_external_torque            = private_nh_->create_service<dsr_msgs2::srv::GetExternalTorque>("aux_control/get_external_torque", DRHWInterface::get_external_torque_cb);                
        m_nh_srv_get_tool_force                 = private_nh_->create_service<dsr_msgs2::srv::GetToolForce>("aux_control/get_tool_force", DRHWInterface::get_tool_force_cb);                               
        m_nh_srv_get_solution_space             = private_nh_->create_service<dsr_msgs2::srv::GetSolutionSpace>("aux_control/get_solution_space", DRHWInterface::get_solution_space_cb);       
        m_nh_srv_get_orientation_error          = private_nh_->create_service<dsr_msgs2::srv::GetOrientationError>("aux_control/get_orientation_error", DRHWInterface::get_orientation_error_cb);              
        
        //  force/stiffness
        m_nh_srv_parallel_axis1                 = private_nh_->create_service<dsr_msgs2::srv::ParallelAxis1>("force/parallel_axis1", DRHWInterface::parallel_axis1_cb);  
        m_nh_srv_parallel_axis2                 = private_nh_->create_service<dsr_msgs2::srv::ParallelAxis2>("force/parallel_axis2", DRHWInterface::parallel_axis2_cb);  
        m_nh_srv_align_axis1                    = private_nh_->create_service<dsr_msgs2::srv::AlignAxis1>("force/align_axis1", DRHWInterface::align_axis1_cb);          
        m_nh_srv_align_axis2                    = private_nh_->create_service<dsr_msgs2::srv::AlignAxis2>("force/align_axis2", DRHWInterface::align_axis2_cb);      
        m_nh_srv_is_done_bolt_tightening        = private_nh_->create_service<dsr_msgs2::srv::IsDoneBoltTightening>("force/is_done_bolt_tightening", DRHWInterface::is_done_bolt_tightening_cb);      
        m_nh_srv_release_compliance_ctrl        = private_nh_->create_service<dsr_msgs2::srv::ReleaseComplianceCtrl>("force/release_compliance_ctrl", DRHWInterface::release_compliance_ctrl_cb);          
        m_nh_srv_task_compliance_ctrl           = private_nh_->create_service<dsr_msgs2::srv::TaskComplianceCtrl>("force/task_compliance_ctrl", DRHWInterface::task_compliance_ctrl_cb);          
        m_nh_srv_set_stiffnessx                 = private_nh_->create_service<dsr_msgs2::srv::SetStiffnessx>("force/set_stiffnessx", DRHWInterface::set_stiffnessx_cb);          
        m_nh_srv_calc_coord                     = private_nh_->create_service<dsr_msgs2::srv::CalcCoord>("force/calc_coord", DRHWInterface::calc_coord_cb);      
        m_nh_srv_set_user_cart_coord1           = private_nh_->create_service<dsr_msgs2::srv::SetUserCartCoord1>("force/set_user_cart_coord1", DRHWInterface::set_user_cart_coord1_cb);          
        m_nh_srv_set_user_cart_coord2           = private_nh_->create_service<dsr_msgs2::srv::SetUserCartCoord2>("force/set_user_cart_coord2", DRHWInterface::set_user_cart_coord2_cb);              
        m_nh_srv_set_user_cart_coord3           = private_nh_->create_service<dsr_msgs2::srv::SetUserCartCoord3>("force/set_user_cart_coord3", DRHWInterface::set_user_cart_coord3_cb);      
        m_nh_srv_overwrite_user_cart_coord      = private_nh_->create_service<dsr_msgs2::srv::OverwriteUserCartCoord>("force/overwrite_user_cart_coord", DRHWInterface::overwrite_user_cart_coord_cb);      
        m_nh_srv_get_user_cart_coord            = private_nh_->create_service<dsr_msgs2::srv::GetUserCartCoord>("force/get_user_cart_coord", DRHWInterface::get_user_cart_coord_cb);          
        m_nh_srv_set_desired_force              = private_nh_->create_service<dsr_msgs2::srv::SetDesiredForce>("force/set_desired_force", DRHWInterface::set_desired_force_cb);          
        m_nh_srv_release_force                  = private_nh_->create_service<dsr_msgs2::srv::ReleaseForce>("force/release_force", DRHWInterface::release_force_cb);      
        m_nh_srv_check_position_condition       = private_nh_->create_service<dsr_msgs2::srv::CheckPositionCondition>("force/check_position_condition", DRHWInterface::check_position_condition_cb);          
        m_nh_srv_check_force_condition          = private_nh_->create_service<dsr_msgs2::srv::CheckForceCondition>("force/check_force_condition", DRHWInterface::check_force_condition_cb);      
        m_nh_srv_check_orientation_condition1   = private_nh_->create_service<dsr_msgs2::srv::CheckOrientationCondition1>("force/check_orientation_condition1", DRHWInterface::check_orientation_condition1_cb);             
        m_nh_srv_check_orientation_condition2   = private_nh_->create_service<dsr_msgs2::srv::CheckOrientationCondition2>("force/check_orientation_condition2", DRHWInterface::check_orientation_condition2_cb);            
        m_nh_srv_coord_transform                = private_nh_->create_service<dsr_msgs2::srv::CoordTransform>("force/coord_transform", DRHWInterface::coord_transform_cb);          
        m_nh_srv_get_workpiece_weight           = private_nh_->create_service<dsr_msgs2::srv::GetWorkpieceWeight>("force/get_workpiece_weight", DRHWInterface::get_workpiece_weight_cb);          
        m_nh_srv_reset_workpiece_weight         = private_nh_->create_service<dsr_msgs2::srv::ResetWorkpieceWeight>("force/reset_workpiece_weight", DRHWInterface::reset_workpiece_weight_cb);          

        //  io
        m_nh_srv_set_ctrl_box_digital_output        = private_nh_->create_service<dsr_msgs2::srv::SetCtrlBoxDigitalOutput>("io/set_ctrl_box_digital_output", DRHWInterface::set_ctrl_box_digital_output_cb);    
        m_nh_srv_get_ctrl_box_digital_input         = private_nh_->create_service<dsr_msgs2::srv::GetCtrlBoxDigitalInput>("io/get_ctrl_box_digital_input", DRHWInterface::get_ctrl_box_digital_input_cb);   
        m_nh_srv_set_tool_digital_output            = private_nh_->create_service<dsr_msgs2::srv::SetToolDigitalOutput>("io/set_tool_digital_output", DRHWInterface::set_tool_digital_output_cb);   
        m_nh_srv_get_tool_digital_input             = private_nh_->create_service<dsr_msgs2::srv::GetToolDigitalInput>("io/get_tool_digital_input", DRHWInterface::get_tool_digital_input_cb);   
        m_nh_srv_set_ctrl_box_analog_output         = private_nh_->create_service<dsr_msgs2::srv::SetCtrlBoxAnalogOutput>("io/set_ctrl_box_analog_output", DRHWInterface::set_ctrl_box_analog_output_cb);   
        m_nh_srv_get_ctrl_box_analog_input          = private_nh_->create_service<dsr_msgs2::srv::GetCtrlBoxAnalogInput>("io/get_ctrl_box_analog_input", DRHWInterface::get_ctrl_box_analog_input_cb);   
        m_nh_srv_set_ctrl_box_analog_output_type    = private_nh_->create_service<dsr_msgs2::srv::SetCtrlBoxAnalogOutputType>("io/set_ctrl_box_analog_output_type", DRHWInterface::set_ctrl_box_analog_output_type_cb);   
        m_nh_srv_set_ctrl_box_analog_input_type     = private_nh_->create_service<dsr_msgs2::srv::SetCtrlBoxAnalogInputType>("io/set_ctrl_box_analog_input_type", DRHWInterface::set_ctrl_box_analog_input_type_cb);       
        m_nh_srv_get_ctrl_box_digital_output        = private_nh_->create_service<dsr_msgs2::srv::GetCtrlBoxDigitalOutput>("io/get_ctrl_box_digital_output", DRHWInterface::get_ctrl_box_digital_output_cb);   
        m_nh_srv_get_tool_digital_output            = private_nh_->create_service<dsr_msgs2::srv::GetToolDigitalOutput>("io/get_tool_digital_output", DRHWInterface::get_tool_digital_output_cb);   

        //  modbus
        m_nh_srv_set_modbus_output      = private_nh_->create_service<dsr_msgs2::srv::SetModbusOutput>("modbus/set_modbus_output", DRHWInterface::set_modbus_output_cb);    
        m_nh_srv_get_modbus_input       = private_nh_->create_service<dsr_msgs2::srv::GetModbusInput>("modbus/get_modbus_input", DRHWInterface::get_modbus_input_cb);    
        m_nh_srv_config_create_modbus   = private_nh_->create_service<dsr_msgs2::srv::ConfigCreateModbus>("modbus/config_create_modbus", DRHWInterface::config_create_modbus_cb);
        m_nh_srv_config_delete_modbus   = private_nh_->create_service<dsr_msgs2::srv::ConfigDeleteModbus>("modbus/config_delete_modbus", DRHWInterface::config_delete_modbus_cb);

        //  drl
        m_nh_srv_drl_pause              = private_nh_->create_service<dsr_msgs2::srv::DrlPause>("drl/drl_pause", DRHWInterface::drl_pause_cb);                         
        m_nh_srv_drl_start              = private_nh_->create_service<dsr_msgs2::srv::DrlStart>("drl/drl_start", DRHWInterface::drl_start_cb);    
        m_nh_srv_drl_stop               = private_nh_->create_service<dsr_msgs2::srv::DrlStop>("drl/drl_stop", DRHWInterface::drl_stop_cb);    
        m_nh_srv_drl_resume             = private_nh_->create_service<dsr_msgs2::srv::DrlResume>("drl/drl_resume", DRHWInterface::drl_resume_cb);        
        m_nh_srv_get_drl_state          = private_nh_->create_service<dsr_msgs2::srv::GetDrlState>("drl/get_drl_state", DRHWInterface::get_drl_state_cb);       

        //  tcp
        m_nh_srv_config_create_tcp      = private_nh_->create_service<dsr_msgs2::srv::ConfigCreateTcp>("tcp/config_create_tcp", DRHWInterface::config_create_tcp_cb);    
        m_nh_srv_config_delete_tcp      = private_nh_->create_service<dsr_msgs2::srv::ConfigDeleteTcp>("tcp/config_delete_tcp", DRHWInterface::config_delete_tcp_cb);  
        m_nh_srv_get_current_tcp        = private_nh_->create_service<dsr_msgs2::srv::GetCurrentTcp>("tcp/get_current_tcp", DRHWInterface::get_current_tcp_cb);       
        m_nh_srv_set_current_tcp        = private_nh_->create_service<dsr_msgs2::srv::SetCurrentTcp>("tcp/set_current_tcp", DRHWInterface::set_current_tcp_cb);       

        //  tool
        m_nh_srv_config_create_tool     = private_nh_->create_service<dsr_msgs2::srv::ConfigCreateTool>("tool/config_create_tool", DRHWInterface::config_create_tool_cb); 
        m_nh_srv_config_delete_tool     = private_nh_->create_service<dsr_msgs2::srv::ConfigDeleteTool>("tool/config_delete_tool", DRHWInterface::config_delete_tool_cb);    
        m_nh_srv_get_current_tool       = private_nh_->create_service<dsr_msgs2::srv::GetCurrentTool>("tool/get_current_tool", DRHWInterface::get_current_tool_cb);     
        m_nh_srv_set_current_tool       = private_nh_->create_service<dsr_msgs2::srv::SetCurrentTool>("tool/set_current_tool", DRHWInterface::set_current_tool_cb);     
        m_nh_srv_set_tool_shape         = private_nh_->create_service<dsr_msgs2::srv::SetToolShape>("tool/set_tool_shape", DRHWInterface::set_tool_shape_cb); 

        //  gripper
        m_nh_srv_robotiq2_f_open        = private_nh_->create_service<dsr_msgs2::srv::Robotiq2FOpen>("gripper/robotiq2_f_open", DRHWInterface::robotiq2_f_open_cb); 
        m_nh_srv_robotiq2_f_close       = private_nh_->create_service<dsr_msgs2::srv::Robotiq2FClose>("gripper/robotiq2_f_close", DRHWInterface::robotiq2_f_close_cb);
        m_nh_srv_robotiq2_f_move        = private_nh_->create_service<dsr_msgs2::srv::Robotiq2FMove>("gripper/robotiq2_f_move", DRHWInterface::robotiq2_f_move_cb);

        //  serial
        m_nh_srv_serial_send_data       = private_nh_->create_service<dsr_msgs2::srv::SerialSendData>("gripper/serial_send_data", DRHWInterface::serial_send_data_cb);



        memset(&g_stDrState, 0x00, sizeof(DR_STATE)); 
        memset(&g_stDrError, 0x00, sizeof(DR_ERROR)); 
        memset(&m_stDrState, 0x00, sizeof(DR_STATE));
        memset(&m_stDrError, 0x00, sizeof(DR_ERROR));

        // create threads     
        m_th_subscribe = boost::thread( boost::bind(&thread_subscribe, private_nh_) );
        m_th_publisher    = boost::thread( boost::bind(&thread_publisher, this, private_nh_, DSR_CTL_PUB_RATE/*hz*/) );    //100hz(10ms)
        ///미사용 m_th_publisher_joint_states = boost::thread( boost::bind(&thread_publisher_direct_access_joint_states, this, private_nh_, 10/*DSR_CTL_PUB_RATE*//*hz*/) );    //100hz(10ms)

        g_nAnalogOutputModeCh1 = -1;
        g_nAnalogOutputModeCh2 = -1;

    }
    DRHWInterface::~DRHWInterface()
    {
        //RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"DRHWInterface::~DRHWInterface() 0");
        Drfl.CloseConnection();

        //RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"DRHWInterface::~DRHWInterface() 1");
//ROS2 ???        m_th_publisher.join();   //kill publisher thread
        //RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"DRHWInterface::~DRHWInterface() 2");

//ROS2 ???        m_th_subscribe.join();   //kill subscribe thread 
        RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"DRHWInterface::~DRHWInterface()");
    }

//ROS2    bool DRHWInterface::init()
    hardware_interface::return_type DRHWInterface::init()
    {
        RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"[dsr_hw_interface2] init() ==> setup callback fucntion");

        auto joint_names = {
          "joint1",
          "joint2",
          "joint3",
          "joint4",
          "joint5",
          "joint6",
        };

#if _OLD_ROS2_CONTROL

        joint_state_handles_.resize(joint_names.size());
        joint_command_handles_.resize(joint_names.size());
        joint_mode_handles_.resize(joint_names.size());

        size_t i = 0;
        for (auto & joint_name : joint_names){
            printf("joint_name = %s",joint_name);
            RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"joint_name = %s",joint_name);

            hardware_interface::JointStateHandle state_handle(joint_name, &m_joints[i].pos, &m_joints[i].vel, &m_joints[i].eff);       
            joint_state_handles_[i] = state_handle;
            if (register_joint_state_handle(&joint_state_handles_[i]) != hardware_interface::return_type::OK) {
                throw std::runtime_error("unable to register " + joint_state_handles_[i].get_name());
            }
    
            hardware_interface::JointCommandHandle command_handle(joint_name, &m_joints[i].cmd);
            joint_command_handles_[i] = command_handle;
            if (register_joint_command_handle(&joint_command_handles_[i]) != hardware_interface::return_type::OK)
            {
                throw std::runtime_error("unable to register " + joint_command_handles_[i].get_name());
            }
            ++i;
        }
#else

        size_t i = 0;
        for (auto & joint_name : joint_names)
        {
            printf("joint_name = %s",joint_name);
            RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"joint_name = %s",joint_name);
            register_joint(joint_name, "position", m_joints[i].pos);
            ++i;
        }

#endif 

//-----------------------------------------------------------------------------------------------------
        int nServerPort = 12345;

        RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"INIT@@@@@@@@@@@@@@@@@@@@@@@@@");
        //--- doosan API's call-back fuctions : Only work within 50msec in call-back functions
        Drfl.SetOnTpInitializingCompleted(OnTpInitializingCompletedCB);
        Drfl.SetOnHommingCompleted(OnHommingCompletedCB);
        Drfl.SetOnProgramStopped(OnProgramStoppedCB);
        Drfl.SetOnMonitoringModbus(OnMonitoringModbusCB);
        Drfl.SetOnMonitoringData(OnMonitoringDataCB);           // Callback function in M2.4 and earlier
        Drfl.SetOnMonitoringCtrlIO(OnMonitoringCtrlIOCB);       // Callback function in M2.4 and earlier
        Drfl.SetOnMonitoringState(OnMonitoringStateCB);
        Drfl.SetOnMonitoringAccessControl(OnMonitoringAccessControlCB);
        Drfl.SetOnLogAlarm(OnLogAlarm);
        RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"init() ==> arm is standby");
        std::string host ="";
        std::string mode;

        //----- get parameters --------------------------------------------------------
        g_node->get_parameter("host", host);        
        g_node->get_parameter("port", nServerPort);
        g_node->get_parameter("command", m_bCommand_);    
        g_node->get_parameter("mode", mode);

        RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"host = %s",host.c_str());
        RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"port = %d",nServerPort);
        RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"command = %d",m_bCommand_);
        RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"mode = %s",mode.c_str());
        //------------------------------------------------------------------------------

        RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"host %s, port=%d bCommand: %d, mode: %s", host.c_str(), nServerPort, m_bCommand_, mode.c_str());


        if(Drfl.OpenConnection(host, nServerPort))
        {
            //--- connect Emulator ? ------------------------------    
            if(host == "127.0.0.1") m_bIsEmulatorMode = true; 
            else                    m_bIsEmulatorMode = false;

            //--- Get version -------------------------------------            
            SYSTEM_VERSION tSysVerion = {'\0', };
            assert(Drfl.GetSystemVersion(&tSysVerion));

            //--- Get DRCF version & convert to integer  ----------            
            m_nVersionDRCF = 0; 
            int k=0;
            for(int i=strlen(tSysVerion._szController); i>0; i--)
                if(tSysVerion._szController[i]>='0' && tSysVerion._szController[i]<='9')
                    m_nVersionDRCF += (tSysVerion._szController[i]-'0')*pow(10.0,k++);
            if(m_nVersionDRCF < 100000) m_nVersionDRCF += 100000; 

            RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"_______________________________________________");   
            if(m_bIsEmulatorMode) RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"    Emulator Mode");
            else                  RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"    Real Robot Mode");
            RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"    DRCF version = %s",tSysVerion._szController);
            RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"    DRFL version = %s",Drfl.GetLibraryVersion());
            RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"    m_nVersionDRCF = %d", m_nVersionDRCF);  //ex> M2.40 = 120400, M2.50 = 120500  
            RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"_______________________________________________");   

            if(m_nVersionDRCF >= 120500)    //M2.5 or later        
            {
                Drfl.SetOnMonitoringDataEx(OnMonitoringDataExCB);      //Callback function in version 2.5 and higher
                Drfl.SetOnMonitoringCtrlIOEx(OnMonitoringCtrlIOExCB);  //Callback function in version 2.5 and higher
                Drfl.SetupMonitoringVersion(1);                        //Enabling extended monitoring functions 
            }

            //--- Check Robot State : STATE_STANDBY ---               
            int delay;
            //ROS2 ros::param::param<int>("~standby", delay, 5000);
            delay = 5000;
            while ((Drfl.GetRobotState() != STATE_STANDBY)){
                usleep(delay);
            }

            //--- Set Robot mode : MANUAL or AUTO
            //assert(Drfl.SetRobotMode(ROBOT_MODE_MANUAL));
            assert(Drfl.SetRobotMode(ROBOT_MODE_AUTONOMOUS));

            //--- Set Robot mode : virual or real 
            ROBOT_SYSTEM eTargetSystem = ROBOT_SYSTEM_VIRTUAL;
            if(mode == "real") eTargetSystem = ROBOT_SYSTEM_REAL;
            assert(Drfl.SetRobotSystem(eTargetSystem));

            // to compare with m_joints[].cmd
            for(int i = 0; i < NUM_JOINT; i++){
                RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"[init]::read %d-pos: %7.3f", i, m_joints[i].cmd);
                m_fCmd_[i] = m_joints[i].cmd;
            }

            return hardware_interface::return_type::OK;
        }
        RCLCPP_ERROR(rclcpp::get_logger("dsr_hw_interface2"),"DRHWInterface::init() DRCF connecting ERROR!!!");

        return hardware_interface::return_type::ERROR;
    }

    hardware_interface::return_type DRHWInterface::read()
    {   
        ///RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"DRHWInterface::read()");
#if _OLD_ROS2_CONTROL
        //ROS2 std_msgs::Float64MultiArray msg;
        std_msgs::msg::Float64MultiArray msg;

        // joints.pos, vel, eff should be update
        //ROS_DEBUG("DRHWInterface::read()");
        LPROBOT_POSE pose = Drfl.GetCurrentPose();
        for(int i = 0; i < NUM_JOINT; i++){
            //ROS2 ROS_DEBUG("[DRHWInterface::read] %d-pos: %7.3f", i, pose->_fPosition[i]);
            ///RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"[DRHWInterface::read] %d-pos: %7.3f", i, pose->_fPosition[i]);
            m_joints[i].pos = deg2rad(pose->_fPosition[i]);	//update pos to '/joint_states'
            msg.data.push_back(m_joints[i].pos);
        }
#else      
        std_msgs::msg::Float64MultiArray msg;
        LPROBOT_POSE pose = Drfl.GetCurrentPose();
        size_t i = 0;
        for (auto & joint_name : joint_names)
        {
            m_joints[i].pos = deg2rad(pose->_fPosition[i]);
            
            auto joint_handle = std::make_shared<hardware_interface::JointHandle>(joint_name, "position");
            get_joint_handle(*joint_handle);
            joint_handle->set_value(m_joints[i].pos); 
            msg.data.push_back(m_joints[i].pos);
            //RCLCPP_INFO(rclcpp::get_logger("+++++gazebo msg++++++"),"[init]::read %d-pos: %7.3f", i, msg.data[i]);
            ++i;
        }
        m_PubtoGazebo->publish(msg);
#endif   
        
//TODO        if(m_strRobotGripper != "none"){
//TODO            msg.data.push_back(m_joints[6].pos);
//TODO        }
//TODO        m_PubtoGazebo.publish(msg);

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type DRHWInterface::write()
    {
        ///RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"DRHWInterface::write()");

/*    
        LPROBOT_POSE pose = Drfl.GetCurrentPose();
        for(int i = 0; i < NUM_JOINT; i++){
            //ROS2 ROS_DEBUG("[DRHWInterface::read] %d-pos: %7.3f", i, pose->_fPosition[i]);
            RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"[DRHWInterface::write] %d-pos: %7.3f", i, pose->_fPosition[i]);
            m_joints[i].cmd = deg2rad(pose->_fPosition[i]);	//update pos to Rviz
        }
*/
        return hardware_interface::return_type::OK;
    }

    //----- SIG Handler --------------------------------------------------------------
//ROS2 ???    
/*
    void DRHWInterface::sigint_handler(int signo)
    {
        RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"SIG HANDLER !!!!!!!!!");

        //ROS2 ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
        //ROS2 ros::Publisher pubRobotStop = node->advertise<dsr_msgs2::msg::RobotStop>("/"+m_strRobotName +m_strRobotModel+"/stop",100);
        auto pubRobotStop = g_node->create_publisher<dsr_msgs2::msg::RobotStop>("/"+m_strRobotName +m_strRobotModel+"/stop",100); 

        dsr_msgs2::msg::RobotStop msg;
        
        msg.stop_mode  = STOP_TYPE_QUICK;
        //ROS2 pubRobotStop.publish(msg);
        pubRobotStop->publish(msg);

        RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"[sigint_hangler] CloseConnection");
    }
*/
//ROS2 ???
/*
    void DRHWInterface::positionCallback(const std_msgs::msg::Float64MultiArray::ConstPtr& msg){
        RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"callback: Position received");
        std::array<float, NUM_JOINT> target_pos;
        std::copy(msg->data.cbegin(), msg->data.cend(), target_pos.begin());
        Drfl.MoveJAsync(target_pos.data(), 50, 50);
    }
*/
//ROS2 ???
/*
    void DRHWInterface::jogCallback(const dsr_msgs2::msg::JogMultiAxis::ConstPtr& msg){
        //RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"callback: jogCallback received");

        std::array<float, NUM_JOINT> target_pos;
        std::copy(msg->jog_axis.cbegin(), msg->jog_axis.cend(), target_pos.begin());       
        msg->move_reference;
        msg->speed;

        //RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"jog_axis = %f,%f,%f,%f,%f,%f", target_pos[0],target_pos[1],target_pos[2],target_pos[3],target_pos[4],target_pos[5]);
        //RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"move_reference = %d", msg->move_reference);
        //RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"speed = %f", msg->speed);

        Drfl.MultiJog(target_pos.data(), (MOVE_REFERENCE)msg->move_reference, msg->speed);

    }
*/
    void DRHWInterface::trajectoryCallback(const moveit_msgs::msg::DisplayTrajectory::SharedPtr msg) const
    {
        static int sn_cnt = 0;
        RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"), "trajectoryCallback called XXXXXXXXXXXXXXXXXXXX"); 
        RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"), "trajectoryCallback called XXXXXXXXXXXXXXXXXXXX"); 
        RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"), "trajectoryCallback called XXXXXXXXXXXXXXXXXXXX"); 

        /*
        msg->model_id;           // m1013
        msg->trajectory.size();  // =1
        msg->trajectory[0].joint_trajectory.joint_names[5]; //joint1 ~ joint6

        msg->trajectory[0].joint_trajectory.points.size();
        msg->trajectory[0].joint_trajectory.points[0].positions[5];
        msg->trajectory[0].joint_trajectory.points[0].velocities[5];
        msg->trajectory[0].joint_trajectory.points[0].accelerations[5];
        msg->trajectory[0].joint_trajectory.points[0].effort[5];
        msg->trajectory[0].joint_trajectory.points[0].time_from_start;

        RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"[trajectory] msg->model_id =%s ", msg->model_id.c_str());
        RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"[trajectory] msg->trajectory.size() =%d ", msg->trajectory.size());
        for(int i=0; i<msg->trajectory[0].joint_trajectory.joint_names.size(); i++)
            std::cout << msg->trajectory[0].joint_trajectory.joint_names[i] << std::endl;

        RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"[trajectory] msg->trajectory[0].joint_trajectory.points.size() =%d ", msg->trajectory[0].joint_trajectory.points.size());
        std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
        for(int i=0; i<(int)msg->trajectory[0].joint_trajectory.points.size(); i++){
            for(int j=0; j<(int)msg->trajectory[0].joint_trajectory.joint_names.size(); j++){
              std::cout << msg->trajectory[0].joint_trajectory.points[i].positions[j] << " ";
            }
            std::cout << std::endl;
            std::cout << "###################" << std::endl;
        }
        */

        RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"callback: Trajectory received");
        RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"  msg->trajectory[0].joint_trajectory.points.size() =%d",(int)msg->trajectory[0].joint_trajectory.points.size());   //=10 가변젹 
        RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"  msg->trajectory[0].joint_trajectory.joint_names.size() =%d",(int)msg->trajectory[0].joint_trajectory.joint_names.size()); //=6

        float preTargetTime = 0.0;
        float targetTime = 0.0;
        float targetTime_sec = 0.0;

        float fTargetPos[MAX_SPLINE_POINT][NUM_JOINT] = {0.0, };
        int nCntTargetPos =0; 

        nCntTargetPos = msg->trajectory[0].joint_trajectory.points.size();
        /*
        if(nCntTargetPos > MAX_SPLINE_POINT)
        {
            RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"DRHWInterface::trajectoryCallback over max Trajectory (%d > %d)",nCntTargetPos ,MAX_SPLINE_POINT);
            return; 
        }
        */
       
        rclcpp::Duration d(msg->trajectory[0].joint_trajectory.points[nCntTargetPos-1].time_from_start);
        targetTime = d.nanoseconds();
        RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"[trajectory] targetTime = %f", targetTime);

        for(int i = 0; i < (int)msg->trajectory[0].joint_trajectory.points.size(); i++) //=10
        {
            std::array<float, NUM_JOINT> degrees;
            ///ROS ros::Duration d(msg->trajectory.points[i].time_from_start);    
            rclcpp::Duration d(msg->trajectory[0].joint_trajectory.points[i].time_from_start);
            //targetTime = msg->trajectory.points[i].time_from_start;

            //RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"  msg->goal.trajectory.points[%d].time_from_start = %7.3%f",i,(float)msg->goal.trajectory.points[i].time_from_start );  

            ///ROS targetTime = d.toSec();
            targetTime = d.seconds();

            ///RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"[trajectory] preTargetTime: %7.3f", preTargetTime);
            ///targetTime = targetTime - preTargetTime;
            ///preTargetTime = targetTime;
            ///RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"[trajectory] time_from_start: %7.3f", targetTime);

            RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"[trajectory] [%02d : %.3f] %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f",i ,targetTime,
                rad2deg(msg->trajectory[0].joint_trajectory.points[i].positions[0]),
                rad2deg(msg->trajectory[0].joint_trajectory.points[i].positions[1]),
                rad2deg(msg->trajectory[0].joint_trajectory.points[i].positions[2]),
                rad2deg(msg->trajectory[0].joint_trajectory.points[i].positions[3]),
                rad2deg(msg->trajectory[0].joint_trajectory.points[i].positions[4]),
                rad2deg(msg->trajectory[0].joint_trajectory.points[i].positions[5]) );

            for(int j = 0; j < (int)msg->trajectory[0].joint_trajectory.joint_names.size(); j++)    //=6    
            {
                //RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"[trajectory] %d-pos: %7.3f", j, msg->goal.trajectory.points[i].positions[j]);
                // todo
                //get a position & time_from_start
                //convert radian to degree the position
                //run MoveJ(position, time_From_start)
                
                degrees[j] = rad2deg( msg->trajectory[0].joint_trajectory.points[i].positions[j] );

                fTargetPos[i][j] = degrees[j];

            }
        }

        RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"[trajectory] last targetTime = %f", targetTime);

        Drfl.MoveSJ(fTargetPos, nCntTargetPos, 0.0, 0.0, targetTime, (MOVE_MODE)MOVE_MODE_ABSOLUTE);

        //Drfl.MoveJAsync(degrees.data(), 30, 30, 0, MOVE_MODE_ABSOLUTE, BLENDING_SPEED_TYPE_OVERRIDE);
        //for(int i = 0; i < NUM_JOINT; i++){
        //    RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"[]::cmd %d-pos: %7.3f", i, joints[i].cmd);
        //    cmd_[i] = m_joints[i].cmd;
        //}


    }

    void DRHWInterface::testSubCallback(const std_msgs::msg::String::SharedPtr msg) const
    {
        RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"), "I heard: '%s'", msg->data.c_str()); 
    }

    //----- SYSTEM Service Call-back functions ------------------------------------------------------------
    void DRHWInterface::set_robot_mode_cb(const std::shared_ptr<dsr_msgs2::srv::SetRobotMode::Request> req, std::shared_ptr<dsr_msgs2::srv::SetRobotMode::Response> res){
        RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"DRHWInterface::set_robot_mode_cb() called and calling Drfl.SetRobotMode(%d)",req->robot_mode);

        /*ROS2     
        res.success = false;
        res.success = Drfl.SetRobotMode((ROBOT_MODE)req.robot_mode);   
        return true;
        */
        res->success = Drfl.SetRobotMode((ROBOT_MODE)req->robot_mode); 
    }
    void DRHWInterface::get_robot_mode_cb(const std::shared_ptr<dsr_msgs2::srv::GetRobotMode::Request> req, std::shared_ptr<dsr_msgs2::srv::GetRobotMode::Response> res)
    {       
        if(req) res->success = 0;
    }      
    void DRHWInterface::set_robot_system_cb(const std::shared_ptr<dsr_msgs2::srv::SetRobotSystem::Request> req, std::shared_ptr<dsr_msgs2::srv::SetRobotSystem::Response> res)
    { 
        if(req) res->success = 0;
    }         
    void DRHWInterface::get_robot_system_cb(const std::shared_ptr<dsr_msgs2::srv::GetRobotSystem::Request> req, std::shared_ptr<dsr_msgs2::srv::GetRobotSystem::Response> res)
    {
        if(req) res->success = 0;
    }
    void DRHWInterface::get_robot_state_cb(const std::shared_ptr<dsr_msgs2::srv::GetRobotState::Request> req, std::shared_ptr<dsr_msgs2::srv::GetRobotState::Response> res)        
    {
        if(req) res->success = 0;
    }
    void DRHWInterface::set_robot_speed_mode_cb(const std::shared_ptr<dsr_msgs2::srv::SetRobotSpeedMode::Request> req, std::shared_ptr<dsr_msgs2::srv::SetRobotSpeedMode::Response> res)    
    {
        if(req) res->success = 0;
    }
    void DRHWInterface::get_robot_speed_mode_cb(const std::shared_ptr<dsr_msgs2::srv::GetRobotSpeedMode::Request> req, std::shared_ptr<dsr_msgs2::srv::GetRobotSpeedMode::Response> res)       
    {
        if(req) res->success = 0;
    }
    void DRHWInterface::get_current_pose_cb(const std::shared_ptr<dsr_msgs2::srv::GetCurrentPose::Request> req, std::shared_ptr<dsr_msgs2::srv::GetCurrentPose::Response> res)   
    {
        if(req) res->success = 0;
    }
    void DRHWInterface::set_safe_stop_reset_type_cb(const std::shared_ptr<dsr_msgs2::srv::SetSafeStopResetType::Request> req, std::shared_ptr<dsr_msgs2::srv::SetSafeStopResetType::Response> res)     
    {
        if(req) res->success = 0;
    }
    void DRHWInterface::get_last_alarm_cb(const std::shared_ptr<dsr_msgs2::srv::GetLastAlarm::Request> req, std::shared_ptr<dsr_msgs2::srv::GetLastAlarm::Response> res)   
    {
        if(req) res->success = 0;
    }


    //----- MOTION Service Call-back functions ------------------------------------------------------------
    void DRHWInterface::move_joint_cb(const std::shared_ptr<dsr_msgs2::srv::MoveJoint::Request> req, std::shared_ptr<dsr_msgs2::srv::MoveJoint::Response> res)
    {
        res->success = false;
        std::array<float, NUM_JOINT> target_pos;
        std::copy(req->pos.cbegin(), req->pos.cend(), target_pos.begin());
        if(req->sync_type == 0){
            RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"DRHWInterface::move_joint_cb() called and calling Drfl.MoveJ");
            res->success = Drfl.MoveJ(target_pos.data(), req->vel, req->acc, req->time, (MOVE_MODE)req->mode, req->radius, (BLENDING_SPEED_TYPE)req->blend_type);   
        }
        else{
            RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"DRHWInterface::move_joint_cb() called and calling Drfl.MoveJAsync");
            res->success = Drfl.MoveJAsync(target_pos.data(), req->vel, req->acc, req->time, (MOVE_MODE)req->mode, (BLENDING_SPEED_TYPE)req->blend_type);
        }
    }
    void DRHWInterface::move_line_cb(const std::shared_ptr<dsr_msgs2::srv::MoveLine::Request> req, std::shared_ptr<dsr_msgs2::srv::MoveLine::Response> res)
    {
        res->success = false;
        std::array<float, NUM_JOINT> target_pos;
        std::array<float, 2> target_vel;
        std::array<float, 2> target_acc;
        std::copy(req->pos.cbegin(), req->pos.cend(), target_pos.begin());
        std::copy(req->vel.cbegin(), req->vel.cend(), target_vel.begin());
        std::copy(req->acc.cbegin(), req->acc.cend(), target_acc.begin());

        if(req->sync_type == 0){
            RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"DRHWInterface::move_line_cb() called and calling Drfl.MoveL");
            res->success = Drfl.MoveL(target_pos.data(), target_vel.data(), target_acc.data(), req->time, (MOVE_MODE)req->mode, (MOVE_REFERENCE)req->ref, req->radius, (BLENDING_SPEED_TYPE)req->blend_type);   
        }
        else{
            RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"DRHWInterface::move_line_cb() called and calling Drfl.MoveLAsync");
            res->success = Drfl.MoveLAsync(target_pos.data(), target_vel.data(), target_acc.data(), req->time, (MOVE_MODE)req->mode, (MOVE_REFERENCE)req->ref, (BLENDING_SPEED_TYPE)req->blend_type);
        }
    }
    void DRHWInterface::move_jointx_cb(const std::shared_ptr<dsr_msgs2::srv::MoveJointx::Request> req, std::shared_ptr<dsr_msgs2::srv::MoveJointx::Response> res)               
    {
        res->success = false;
        std::array<float, NUM_TASK> target_pos;
        std::copy(req->pos.cbegin(), req->pos.cend(), target_pos.begin());
        if(req->sync_type == 0){
            //RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"DRHWInterface::movejx_cb() called and calling Drfl.MoveJX");
            res->success = Drfl.MoveJX(target_pos.data(), req->sol, req->vel, req->acc, req->time, (MOVE_MODE)req->mode, (MOVE_REFERENCE)req->ref, req->radius, (BLENDING_SPEED_TYPE)req->blend_type);    
        }
        else{
            //RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"DRHWInterface::movejx_cb() called and calling Drfl.MoveJXAsync");
            res->success = Drfl.MoveJXAsync(target_pos.data(), req->sol, req->vel, req->acc, req->time, (MOVE_MODE)req->mode, (MOVE_REFERENCE)req->ref, (BLENDING_SPEED_TYPE)req->blend_type);    
        }
    }
    void DRHWInterface::move_circle_cb(const std::shared_ptr<dsr_msgs2::srv::MoveCircle::Request> req, std::shared_ptr<dsr_msgs2::srv::MoveCircle::Response> res)       
    {
        res->success = false;
        float fTargetPos[2][NUM_TASK];
        float fTargetVel[2];
        float fTargetAcc[2];
        for(int i = 0; i < 2; i++){
            for(int j = 0; j < NUM_TASK; j++){
                std_msgs::msg::Float64MultiArray pos = req->pos.at(i);
                fTargetPos[i][j] = pos.data[j];
            }
            fTargetVel[i] = req->vel[i];
            fTargetAcc[i] = req->acc[i];
        }
        ///RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"  <xxx pos1> %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f",fTargetPos[0][0],fTargetPos[0][1],fTargetPos[0][2],fTargetPos[0][3],fTargetPos[0][4],fTargetPos[0][5]);
        ///RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"  <xxx pos2> %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f",fTargetPos[1][0],fTargetPos[1][1],fTargetPos[1][2],fTargetPos[1][3],fTargetPos[1][4],fTargetPos[1][5]);
        if(req->sync_type == 0){   
            //RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"DRHWInterface::movec_cb() called and calling Drfl.MoveC");
            res->success = Drfl.MoveC(fTargetPos, fTargetVel, fTargetAcc, req->time, (MOVE_MODE)req->mode, (MOVE_REFERENCE)req->ref, req->radius, (BLENDING_SPEED_TYPE)req->blend_type);      
        }
        else{
            //RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"DRHWInterface::movec_cb() called and calling Drfl.MoveCAsync");
            res->success = Drfl.MoveCAsync(fTargetPos, fTargetVel, fTargetAcc, req->time, (MOVE_MODE)req->mode, (MOVE_REFERENCE)req->ref, (BLENDING_SPEED_TYPE)req->blend_type);  
        }
    }
    void DRHWInterface::move_spline_joint_cb(const std::shared_ptr<dsr_msgs2::srv::MoveSplineJoint::Request> req, std::shared_ptr<dsr_msgs2::srv::MoveSplineJoint::Response> res)      
    {
        res->success = false;
        float fTargetPos[MAX_SPLINE_POINT][NUM_JOINT];

        for(int i=0; i<req->pos_cnt; i++){
            for(int j=0; j<NUM_JOINT; j++){
                std_msgs::msg::Float64MultiArray pos = req->pos.at(i);
                fTargetPos[i][j] = pos.data[j];
            }
        }
        if(req->sync_type == 0){
            //RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"DRHWInterface::movesj_cb() called and calling Drfl.MoveSJ");
            res->success = Drfl.MoveSJ(fTargetPos, req->pos_cnt, req->vel, req->acc, req->time, (MOVE_MODE)req->mode);
        }
        else{
            //RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"DRHWInterface::movesj_cb() called and calling Drfl.MoveSJAsync");
            res->success = Drfl.MoveSJAsync(fTargetPos, req->pos_cnt, req->vel, req->acc, req->time, (MOVE_MODE)req->mode);
        }
    }
    void DRHWInterface::move_spline_task_cb(const std::shared_ptr<dsr_msgs2::srv::MoveSplineTask::Request> req, std::shared_ptr<dsr_msgs2::srv::MoveSplineTask::Response> res)          
    {
        res->success = false;
        float fTargetPos[MAX_SPLINE_POINT][NUM_TASK];
        float fTargetVel[2];
        float fTargetAcc[2];

        for(int i=0; i<req->pos_cnt; i++){
            for(int j=0; j<NUM_TASK; j++){
                std_msgs::msg::Float64MultiArray pos = req->pos.at(i);
                fTargetPos[i][j] = pos.data[j];
            }
          //  fTargetVel[i] = req->vel[i];
          //  fTargetAcc[i] = req->acc[i];
        }
        for(int i=0; i<2; i++){
            fTargetVel[i] = req->vel[i];
            fTargetAcc[i] = req->acc[i];
        }
        if(req->sync_type == 0){
            //RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"DRHWInterface::movesx_cb() called and calling Drfl.MoveSX");
            res->success = Drfl.MoveSX(fTargetPos, req->pos_cnt, fTargetVel, fTargetAcc, req->time, (MOVE_MODE)req->mode, (MOVE_REFERENCE)req->ref, (SPLINE_VELOCITY_OPTION)req->opt);
        }
        else{
            //RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"DRHWInterface::movesx_cb() called and calling Drfl.MoveSXAsync");
            res->success = Drfl.MoveSXAsync(fTargetPos, req->pos_cnt, fTargetVel, fTargetAcc, req->time, (MOVE_MODE)req->mode, (MOVE_REFERENCE)req->ref, (SPLINE_VELOCITY_OPTION)req->opt);
        }
    }
    void DRHWInterface::move_blending_cb(const std::shared_ptr<dsr_msgs2::srv::MoveBlending::Request> req, std::shared_ptr<dsr_msgs2::srv::MoveBlending::Response> res)          
    {
        res->success = false;
        MOVE_POSB posb[req->pos_cnt];
        for(int i=0; i<req->pos_cnt; i++){
            std_msgs::msg::Float64MultiArray segment = req->segment.at(i);
            for(int j=0; j<NUM_TASK; j++){
                posb[i]._fTargetPos[0][j] = segment.data[j];            //0~5
                posb[i]._fTargetPos[1][j] = segment.data[j + NUM_TASK]; //6~11
            }
            posb[i]._iBlendType = segment.data[NUM_TASK + NUM_TASK];    //12
            posb[i]._fBlendRad = segment.data[NUM_TASK + NUM_TASK +1];  //13
        }

        /*
        for(int i=0; i<req->pos_cnt; i++){
            printf("----- segment %d -----\n",i);
            printf("    pos1: %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f\n"
                    ,posb[i]._fTargetPos[0][0], posb[i]._fTargetPos[0][1], posb[i]._fTargetPos[0][2], posb[i]._fTargetPos[0][3], posb[i]._fTargetPos[0][4], posb[i]._fTargetPos[0][5]);

            printf("    pos2: %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f\n"
                    ,posb[i]._fTargetPos[1][0], posb[i]._fTargetPos[1][1], posb[i]._fTargetPos[1][2], posb[i]._fTargetPos[1][3], posb[i]._fTargetPos[1][4], posb[i]._fTargetPos[1][5]);

            printf("    posb[%d]._iblend_type = %d\n",i,posb[i]._iblend_type); 
            printf("    posb[%d]._fBlendRad  = %f\n",i,posb[i]._fBlendRad); 
        }
        */

        std::array<float, 2> target_vel;
        std::array<float, 2> target_acc;
        std::copy(req->vel.cbegin(), req->vel.cend(), target_vel.begin());
        std::copy(req->acc.cbegin(), req->acc.cend(), target_acc.begin());

        if(req->sync_type == 0){
            //RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"DRHWInterface::moveb_cb() called and calling Drfl.MoveB");
            res->success = Drfl.MoveB(posb, req->pos_cnt, target_vel.data(), target_acc.data(), req->time, (MOVE_MODE)req->mode, (MOVE_REFERENCE)req->ref);
        }
        else{
            //RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"DRHWInterface::moveb_cb() called and calling Drfl.MoveBAsync");
            res->success = Drfl.MoveBAsync(posb, req->pos_cnt, target_vel.data(), target_acc.data(), req->time, (MOVE_MODE)req->mode, (MOVE_REFERENCE)req->ref);
        }
    }
    void DRHWInterface::move_spiral_cb(const std::shared_ptr<dsr_msgs2::srv::MoveSpiral::Request> req, std::shared_ptr<dsr_msgs2::srv::MoveSpiral::Response> res)              
    {
        res->success = false;
        std::array<float, 2> target_vel;
        
        std::array<float, 2> target_acc;
        std::copy(req->vel.cbegin(), req->vel.cend(), target_vel.begin());
        std::copy(req->acc.cbegin(), req->acc.cend(), target_acc.begin());

        if(req->sync_type == 0){
            //RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"DRHWInterface::movespiral_cb() called and calling Drfl.MoveSpiral");
            res->success = Drfl.MoveSpiral((TASK_AXIS)req->task_axis, req->revolution, req->max_radius, req->max_length, target_vel.data(), target_acc.data(), req->time, (MOVE_REFERENCE)req->ref);
        }
        else{
            //RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"DRHWInterface::movespiral_cb() called and calling Drfl.MoveSpiralAsync");
            res->success = Drfl.MoveSpiralAsync((TASK_AXIS)req->task_axis, req->revolution, req->max_radius, req->max_length, target_vel.data(), target_acc.data(), req->time, (MOVE_REFERENCE)req->ref);
        }

    }
    void DRHWInterface::move_periodic_cb(const std::shared_ptr<dsr_msgs2::srv::MovePeriodic::Request> req, std::shared_ptr<dsr_msgs2::srv::MovePeriodic::Response> res)              
    {
        res->success = false;
        std::array<float, NUM_TASK> target_amp;
        std::array<float, NUM_TASK> target_periodic;
        std::copy(req->amp.cbegin(), req->amp.cend(), target_amp.begin());
        std::copy(req->periodic.cbegin(), req->periodic.cend(), target_periodic.begin());
        if(req->sync_type == 0){
            //RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"DRHWInterface::moveperiodic_cb() called and calling Drfl.MovePeriodic");
            res->success = Drfl.MovePeriodic(target_amp.data(), target_periodic.data(), req->acc, req->repeat, (MOVE_REFERENCE)req->ref);
        }
        else{
            RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"DRHWInterface::moveperiodic_cb() called and calling Drfl.MovePeriodicAsync");
            res->success = Drfl.MovePeriodicAsync(target_amp.data(), target_periodic.data(), req->acc, req->repeat, (MOVE_REFERENCE)req->ref);
        }

    }
    void DRHWInterface::move_wait_cb(const std::shared_ptr<dsr_msgs2::srv::MoveWait::Request> req, std::shared_ptr<dsr_msgs2::srv::MoveWait::Response> res)                  
    {
        res->success = false;
        //RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"DRHWInterface::movewait_cb() called and calling Drfl.MoveWait");
        res->success = Drfl.MoveWait();
        return true;
    }




    void DRHWInterface::jog_cb(const std::shared_ptr<dsr_msgs2::srv::Jog::Request> req, std::shared_ptr<dsr_msgs2::srv::Jog::Response> res)              
    {
        if(req) res->success = 0;
    }
    void DRHWInterface::jog_multi_cb(const std::shared_ptr<dsr_msgs2::srv::JogMulti::Request> req, std::shared_ptr<dsr_msgs2::srv::JogMulti::Response> res)                      
    {
        if(req) res->success = 0;
    }
    void DRHWInterface::move_pause_cb(const std::shared_ptr<dsr_msgs2::srv::MovePause::Request> req, std::shared_ptr<dsr_msgs2::srv::MovePause::Response> res)                      
    {
        if(req) res->success = 0;
    }
    void DRHWInterface::move_stop_cb(const std::shared_ptr<dsr_msgs2::srv::MoveStop::Request> req, std::shared_ptr<dsr_msgs2::srv::MoveStop::Response> res)                  
    {
        if(req) res->success = 0;
    }
    void DRHWInterface::move_resume_cb(const std::shared_ptr<dsr_msgs2::srv::MoveResume::Request> req, std::shared_ptr<dsr_msgs2::srv::MoveResume::Response> res)                  
    {
        if(req) res->success = 0;
    }
    void DRHWInterface::trans_cb(const std::shared_ptr<dsr_msgs2::srv::Trans::Request> req, std::shared_ptr<dsr_msgs2::srv::Trans::Response> res)                  
    {
        if(req) res->success = 0;
    }
    void DRHWInterface::fkin_cb(const std::shared_ptr<dsr_msgs2::srv::Fkin::Request> req, std::shared_ptr<dsr_msgs2::srv::Fkin::Response> res)              
    {
        if(req) res->success = 0;
    }
    void DRHWInterface::ikin_cb(const std::shared_ptr<dsr_msgs2::srv::Ikin::Request> req, std::shared_ptr<dsr_msgs2::srv::Ikin::Response> res)              
    {
        if(req) res->success = 0;
    }
    void DRHWInterface::set_ref_coord_cb(const std::shared_ptr<dsr_msgs2::srv::SetRefCoord::Request> req, std::shared_ptr<dsr_msgs2::srv::SetRefCoord::Response> res)                  
    {
        if(req) res->success = 0;
    }
    void DRHWInterface::move_home_cb(const std::shared_ptr<dsr_msgs2::srv::MoveHome::Request> req, std::shared_ptr<dsr_msgs2::srv::MoveHome::Response> res)                  
    {
        if(req) res->success = 0;
    }
    void DRHWInterface::check_motion_cb(const std::shared_ptr<dsr_msgs2::srv::CheckMotion::Request> req, std::shared_ptr<dsr_msgs2::srv::CheckMotion::Response> res)                  
    {
        if(req) res->success = 0;
    }
    void DRHWInterface::change_operation_speed_cb(const std::shared_ptr<dsr_msgs2::srv::ChangeOperationSpeed::Request> req, std::shared_ptr<dsr_msgs2::srv::ChangeOperationSpeed::Response> res)        
    {
        if(req) res->success = 0;
    }
    void DRHWInterface::enable_alter_motion_cb(const std::shared_ptr<dsr_msgs2::srv::EnableAlterMotion::Request> req, std::shared_ptr<dsr_msgs2::srv::EnableAlterMotion::Response> res)                 
    {
        if(req) res->success = 0;
    }
    void DRHWInterface::alter_motion_cb(const std::shared_ptr<dsr_msgs2::srv::AlterMotion::Request> req, std::shared_ptr<dsr_msgs2::srv::AlterMotion::Response> res)              
    {
        if(req) res->success = 0;
    }
    void DRHWInterface::disable_alter_motion_cb(const std::shared_ptr<dsr_msgs2::srv::DisableAlterMotion::Request> req, std::shared_ptr<dsr_msgs2::srv::DisableAlterMotion::Response> res)              
    {
        if(req) res->success = 0;
    }
    void DRHWInterface::set_singularity_handling_cb(const std::shared_ptr<dsr_msgs2::srv::SetSingularityHandling::Request> req, std::shared_ptr<dsr_msgs2::srv::SetSingularityHandling::Response> res)  
    {
        if(req) res->success = 0;
    }




    
    //----- AUXILIARY_CONTROL Service Call-back functions ------------------------------------------------------------
    void DRHWInterface::get_control_mode_cb(const std::shared_ptr<dsr_msgs2::srv::GetControlMode::Request> req, std::shared_ptr<dsr_msgs2::srv::GetControlMode::Response> res)                           
    {
        if(req) res->success = 0;
    }
    void DRHWInterface::get_control_space_cb(const std::shared_ptr<dsr_msgs2::srv::GetControlSpace::Request> req, std::shared_ptr<dsr_msgs2::srv::GetControlSpace::Response> res)                          
    {
        if(req) res->success = 0;
    }
    void DRHWInterface::get_current_posj_cb(const std::shared_ptr<dsr_msgs2::srv::GetCurrentPosj::Request> req, std::shared_ptr<dsr_msgs2::srv::GetCurrentPosj::Response> res)                                            
    {
        if(req) res->success = 0;
    }
    void DRHWInterface::get_current_velj_cb(const std::shared_ptr<dsr_msgs2::srv::GetCurrentVelj::Request> req, std::shared_ptr<dsr_msgs2::srv::GetCurrentVelj::Response> res)                                
    {
        if(req) res->success = 0;
    }
    void DRHWInterface::get_desired_posj_cb(const std::shared_ptr<dsr_msgs2::srv::GetDesiredPosj::Request> req, std::shared_ptr<dsr_msgs2::srv::GetDesiredPosj::Response> res)         
    {
        if(req) res->success = 0;
    }
    void DRHWInterface::get_desired_velj_cb(const std::shared_ptr<dsr_msgs2::srv::GetDesiredVelj::Request> req, std::shared_ptr<dsr_msgs2::srv::GetDesiredVelj::Response> res)                                   
    {
        if(req) res->success = 0;
    }
    void DRHWInterface::get_current_posx_cb(const std::shared_ptr<dsr_msgs2::srv::GetCurrentPosx::Request> req, std::shared_ptr<dsr_msgs2::srv::GetCurrentPosx::Response> res)                               
    {
        if(req) res->success = 0;
    }
    void DRHWInterface::get_current_tool_flange_posx_cb(const std::shared_ptr<dsr_msgs2::srv::GetCurrentToolFlangePosx::Request> req, std::shared_ptr<dsr_msgs2::srv::GetCurrentToolFlangePosx::Response> res)                 
    {
        if(req) res->success = 0;
    }
    void DRHWInterface::get_current_velx_cb(const std::shared_ptr<dsr_msgs2::srv::GetCurrentVelx::Request> req, std::shared_ptr<dsr_msgs2::srv::GetCurrentVelx::Response> res)                         
    {
        if(req) res->success = 0;
    }
    void DRHWInterface::get_desired_posx_cb(const std::shared_ptr<dsr_msgs2::srv::GetDesiredPosx::Request> req, std::shared_ptr<dsr_msgs2::srv::GetDesiredPosx::Response> res)     
    {
        if(req) res->success = 0;
    }
    void DRHWInterface::get_desired_velx_cb(const std::shared_ptr<dsr_msgs2::srv::GetDesiredVelx::Request> req, std::shared_ptr<dsr_msgs2::srv::GetDesiredVelx::Response> res)                             
    {
        if(req) res->success = 0;
    }
    void DRHWInterface::get_current_solution_space_cb(const std::shared_ptr<dsr_msgs2::srv::GetCurrentSolutionSpace::Request> req, std::shared_ptr<dsr_msgs2::srv::GetCurrentSolutionSpace::Response> res)                    
    {
        if(req) res->success = 0;
    }
    void DRHWInterface::get_current_rotm_cb(const std::shared_ptr<dsr_msgs2::srv::GetCurrentRotm::Request> req, std::shared_ptr<dsr_msgs2::srv::GetCurrentRotm::Response> res)                     
    {
        if(req) res->success = 0;
    }
    void DRHWInterface::get_joint_torque_cb(const std::shared_ptr<dsr_msgs2::srv::GetJointTorque::Request> req, std::shared_ptr<dsr_msgs2::srv::GetJointTorque::Response> res)                     
    {
        if(req) res->success = 0;
    }
    void DRHWInterface::get_external_torque_cb(const std::shared_ptr<dsr_msgs2::srv::GetExternalTorque::Request> req, std::shared_ptr<dsr_msgs2::srv::GetExternalTorque::Response> res)                
    {
        if(req) res->success = 0;
    }
    void DRHWInterface::get_tool_force_cb(const std::shared_ptr<dsr_msgs2::srv::GetToolForce::Request> req, std::shared_ptr<dsr_msgs2::srv::GetToolForce::Response> res)                               
    {
        if(req) res->success = 0;
    }
    void DRHWInterface::get_solution_space_cb(const std::shared_ptr<dsr_msgs2::srv::GetSolutionSpace::Request> req, std::shared_ptr<dsr_msgs2::srv::GetSolutionSpace::Response> res)       
    {
        if(req) res->success = 0;
    }
    void DRHWInterface::get_orientation_error_cb(const std::shared_ptr<dsr_msgs2::srv::GetOrientationError::Request> req, std::shared_ptr<dsr_msgs2::srv::GetOrientationError::Response> res)              
    {
        if(req) res->success = 0;
    }

    //----- FORCE/STIFFNESS Service Call-back functions ------------------------------------------------------------
    void DRHWInterface::parallel_axis1_cb(const std::shared_ptr<dsr_msgs2::srv::ParallelAxis1::Request> req, std::shared_ptr<dsr_msgs2::srv::ParallelAxis1::Response> res)  
    {
        if(req) res->success = 0;
    }
    void DRHWInterface::parallel_axis2_cb(const std::shared_ptr<dsr_msgs2::srv::ParallelAxis2::Request> req, std::shared_ptr<dsr_msgs2::srv::ParallelAxis2::Response> res)  
    {
        if(req) res->success = 0;
    }
    void DRHWInterface::align_axis1_cb(const std::shared_ptr<dsr_msgs2::srv::AlignAxis1::Request> req, std::shared_ptr<dsr_msgs2::srv::AlignAxis1::Response> res)          
    {
        if(req) res->success = 0;
    }
    void DRHWInterface::align_axis2_cb(const std::shared_ptr<dsr_msgs2::srv::AlignAxis2::Request> req, std::shared_ptr<dsr_msgs2::srv::AlignAxis2::Response> res)      
    {
        if(req) res->success = 0;
    }
    void DRHWInterface::is_done_bolt_tightening_cb(const std::shared_ptr<dsr_msgs2::srv::IsDoneBoltTightening::Request> req, std::shared_ptr<dsr_msgs2::srv::IsDoneBoltTightening::Response> res)      
    {
        if(req) res->success = 0;
    }
    void DRHWInterface::release_compliance_ctrl_cb(const std::shared_ptr<dsr_msgs2::srv::ReleaseComplianceCtrl::Request> req, std::shared_ptr<dsr_msgs2::srv::ReleaseComplianceCtrl::Response> res)         
    {
        if(req) res->success = 0;
    }
    void DRHWInterface::task_compliance_ctrl_cb(const std::shared_ptr<dsr_msgs2::srv::TaskComplianceCtrl::Request> req, std::shared_ptr<dsr_msgs2::srv::TaskComplianceCtrl::Response> res)          
    {
        if(req) res->success = 0;
    }
    void DRHWInterface::set_stiffnessx_cb(const std::shared_ptr<dsr_msgs2::srv::SetStiffnessx::Request> req, std::shared_ptr<dsr_msgs2::srv::SetStiffnessx::Response> res)          
    {
        if(req) res->success = 0;
    }
    void DRHWInterface::calc_coord_cb(const std::shared_ptr<dsr_msgs2::srv::CalcCoord::Request> req, std::shared_ptr<dsr_msgs2::srv::CalcCoord::Response> res)      
    {
        if(req) res->success = 0;
    }
    void DRHWInterface::set_user_cart_coord1_cb(const std::shared_ptr<dsr_msgs2::srv::SetUserCartCoord1::Request> req, std::shared_ptr<dsr_msgs2::srv::SetUserCartCoord1::Response> res)          
    {
        if(req) res->success = 0;
    }
    void DRHWInterface::set_user_cart_coord2_cb(const std::shared_ptr<dsr_msgs2::srv::SetUserCartCoord2::Request> req, std::shared_ptr<dsr_msgs2::srv::SetUserCartCoord2::Response> res)              
    {
        if(req) res->success = 0;
    }
    void DRHWInterface::set_user_cart_coord3_cb(const std::shared_ptr<dsr_msgs2::srv::SetUserCartCoord3::Request> req, std::shared_ptr<dsr_msgs2::srv::SetUserCartCoord3::Response> res)      
    {
        if(req) res->success = 0;
    }
    void DRHWInterface::overwrite_user_cart_coord_cb(const std::shared_ptr<dsr_msgs2::srv::OverwriteUserCartCoord::Request> req, std::shared_ptr<dsr_msgs2::srv::OverwriteUserCartCoord::Response> res)      
    {
        if(req) res->success = 0;
    }
    void DRHWInterface::get_user_cart_coord_cb(const std::shared_ptr<dsr_msgs2::srv::GetUserCartCoord::Request> req, std::shared_ptr<dsr_msgs2::srv::GetUserCartCoord::Response> res)          
    {
        if(req) res->success = 0;
    }
    void DRHWInterface::set_desired_force_cb(const std::shared_ptr<dsr_msgs2::srv::SetDesiredForce::Request> req, std::shared_ptr<dsr_msgs2::srv::SetDesiredForce::Response> res)          
    {
        if(req) res->success = 0;
    }
    void DRHWInterface::release_force_cb(const std::shared_ptr<dsr_msgs2::srv::ReleaseForce::Request> req, std::shared_ptr<dsr_msgs2::srv::ReleaseForce::Response> res)      
    {
        if(req) res->success = 0;
    }
    void DRHWInterface::check_position_condition_cb(const std::shared_ptr<dsr_msgs2::srv::CheckPositionCondition::Request> req, std::shared_ptr<dsr_msgs2::srv::CheckPositionCondition::Response> res)          
    {
        if(req) res->success = 0;
    }
    void DRHWInterface::check_force_condition_cb(const std::shared_ptr<dsr_msgs2::srv::CheckForceCondition::Request> req, std::shared_ptr<dsr_msgs2::srv::CheckForceCondition::Response> res)      
    {
        if(req) res->success = 0;
    }
    void DRHWInterface::check_orientation_condition1_cb(const std::shared_ptr<dsr_msgs2::srv::CheckOrientationCondition1::Request> req, std::shared_ptr<dsr_msgs2::srv::CheckOrientationCondition1::Response> res)             
    {
        if(req) res->success = 0;
    }
    void DRHWInterface::check_orientation_condition2_cb(const std::shared_ptr<dsr_msgs2::srv::CheckOrientationCondition2::Request> req, std::shared_ptr<dsr_msgs2::srv::CheckOrientationCondition2::Response> res)            
    {
        if(req) res->success = 0;
    }
    void DRHWInterface::coord_transform_cb(const std::shared_ptr<dsr_msgs2::srv::CoordTransform::Request> req, std::shared_ptr<dsr_msgs2::srv::CoordTransform::Response> res)          
    {
        if(req) res->success = 0;
    }
    void DRHWInterface::get_workpiece_weight_cb(const std::shared_ptr<dsr_msgs2::srv::GetWorkpieceWeight::Request> req, std::shared_ptr<dsr_msgs2::srv::GetWorkpieceWeight::Response> res)          
    {
        if(req) res->success = 0;
    }
    void DRHWInterface::reset_workpiece_weight_cb(const std::shared_ptr<dsr_msgs2::srv::ResetWorkpieceWeight::Request> req, std::shared_ptr<dsr_msgs2::srv::ResetWorkpieceWeight::Response> res)          
    {
        if(req) res->success = 0;
    }

    //----- IO Service Call-back functions ------------------------------------------------------------
    void DRHWInterface::set_ctrl_box_digital_output_cb(const std::shared_ptr<dsr_msgs2::srv::SetCtrlBoxDigitalOutput::Request> req, std::shared_ptr<dsr_msgs2::srv::SetCtrlBoxDigitalOutput::Response> res)    
    {
        if(req) res->success = 0;
    }
    void DRHWInterface::get_ctrl_box_digital_input_cb(const std::shared_ptr<dsr_msgs2::srv::GetCtrlBoxDigitalInput::Request> req, std::shared_ptr<dsr_msgs2::srv::GetCtrlBoxDigitalInput::Response> res)   
    {
        if(req) res->success = 0;
    }
    void DRHWInterface::set_tool_digital_output_cb(const std::shared_ptr<dsr_msgs2::srv::SetToolDigitalOutput::Request> req, std::shared_ptr<dsr_msgs2::srv::SetToolDigitalOutput::Response> res)   
    {
        if(req) res->success = 0;
    }
    void DRHWInterface::get_tool_digital_input_cb(const std::shared_ptr<dsr_msgs2::srv::GetToolDigitalInput::Request> req, std::shared_ptr<dsr_msgs2::srv::GetToolDigitalInput::Response> res)   
    {
        if(req) res->success = 0;
    }
    void DRHWInterface::set_ctrl_box_analog_output_cb(const std::shared_ptr<dsr_msgs2::srv::SetCtrlBoxAnalogOutput::Request> req, std::shared_ptr<dsr_msgs2::srv::SetCtrlBoxAnalogOutput::Response> res)   
    {
        if(req) res->success = 0;
    }
    void DRHWInterface::get_ctrl_box_analog_input_cb(const std::shared_ptr<dsr_msgs2::srv::GetCtrlBoxAnalogInput::Request> req, std::shared_ptr<dsr_msgs2::srv::GetCtrlBoxAnalogInput::Response> res)   
    {
        if(req) res->success = 0;
    }
    void DRHWInterface::set_ctrl_box_analog_output_type_cb(const std::shared_ptr<dsr_msgs2::srv::SetCtrlBoxAnalogOutputType::Request> req, std::shared_ptr<dsr_msgs2::srv::SetCtrlBoxAnalogOutputType::Response> res)   
    {
        if(req) res->success = 0;
    }
    void DRHWInterface::set_ctrl_box_analog_input_type_cb(const std::shared_ptr<dsr_msgs2::srv::SetCtrlBoxAnalogInputType::Request> req, std::shared_ptr<dsr_msgs2::srv::SetCtrlBoxAnalogInputType::Response> res)       
    {
        if(req) res->success = 0;
    }
    void DRHWInterface::get_ctrl_box_digital_output_cb(const std::shared_ptr<dsr_msgs2::srv::GetCtrlBoxDigitalOutput::Request> req, std::shared_ptr<dsr_msgs2::srv::GetCtrlBoxDigitalOutput::Response> res)   
    {
        if(req) res->success = 0;
    }
    void DRHWInterface::get_tool_digital_output_cb(const std::shared_ptr<dsr_msgs2::srv::GetToolDigitalOutput::Request> req, std::shared_ptr<dsr_msgs2::srv::GetToolDigitalOutput::Response> res)   
    {
        if(req) res->success = 0;
    }

    //----- MODBUS Service Call-back functions ------------------------------------------------------------
    void DRHWInterface::set_modbus_output_cb(const std::shared_ptr<dsr_msgs2::srv::SetModbusOutput::Request> req, std::shared_ptr<dsr_msgs2::srv::SetModbusOutput::Response> res)    
    {
        if(req) res->success = 0;
    }
    void DRHWInterface::get_modbus_input_cb(const std::shared_ptr<dsr_msgs2::srv::GetModbusInput::Request> req, std::shared_ptr<dsr_msgs2::srv::GetModbusInput::Response> res)    
    {
        if(req) res->success = 0;
    }
    void DRHWInterface::config_create_modbus_cb(const std::shared_ptr<dsr_msgs2::srv::ConfigCreateModbus::Request> req, std::shared_ptr<dsr_msgs2::srv::ConfigCreateModbus::Response> res)
    {
        if(req) res->success = 0;
    }
    void DRHWInterface::config_delete_modbus_cb(const std::shared_ptr<dsr_msgs2::srv::ConfigDeleteModbus::Request> req, std::shared_ptr<dsr_msgs2::srv::ConfigDeleteModbus::Response> res)
    {
        if(req) res->success = 0;
    }


    //----- DRL Service Call-back functions ------------------------------------------------------------
    void DRHWInterface::drl_pause_cb(const std::shared_ptr<dsr_msgs2::srv::DrlPause::Request> req, std::shared_ptr<dsr_msgs2::srv::DrlPause::Response> res)                         
    {
        if(req) res->success = 0;
    }
    void DRHWInterface::drl_start_cb(const std::shared_ptr<dsr_msgs2::srv::DrlStart::Request> req, std::shared_ptr<dsr_msgs2::srv::DrlStart::Response> res)    
    {
        if(req) res->success = 0;
    }
    void DRHWInterface::drl_stop_cb(const std::shared_ptr<dsr_msgs2::srv::DrlStop::Request> req, std::shared_ptr<dsr_msgs2::srv::DrlStop::Response> res)    
    {
        if(req) res->success = 0;
    }
    void DRHWInterface::drl_resume_cb(const std::shared_ptr<dsr_msgs2::srv::DrlResume::Request> req, std::shared_ptr<dsr_msgs2::srv::DrlResume::Response> res)        
    {
        if(req) res->success = 0;
    }
    void DRHWInterface::get_drl_state_cb(const std::shared_ptr<dsr_msgs2::srv::GetDrlState::Request> req, std::shared_ptr<dsr_msgs2::srv::GetDrlState::Response> res)       
    {
        if(req) res->success = 0;
    }

    //----- TCP Service Call-back functions ------------------------------------------------------------
    void DRHWInterface::config_create_tcp_cb(const std::shared_ptr<dsr_msgs2::srv::ConfigCreateTcp::Request> req, std::shared_ptr<dsr_msgs2::srv::ConfigCreateTcp::Response> res)    
    {
        if(req) res->success = 0;
    }
    void DRHWInterface::config_delete_tcp_cb(const std::shared_ptr<dsr_msgs2::srv::ConfigDeleteTcp::Request> req, std::shared_ptr<dsr_msgs2::srv::ConfigDeleteTcp::Response> res)  
    {
        if(req) res->success = 0;
    }
    void DRHWInterface::get_current_tcp_cb(const std::shared_ptr<dsr_msgs2::srv::GetCurrentTcp::Request> req, std::shared_ptr<dsr_msgs2::srv::GetCurrentTcp::Response> res)       
    {
        if(req) res->success = 0;
    }
    void DRHWInterface::set_current_tcp_cb(const std::shared_ptr<dsr_msgs2::srv::SetCurrentTcp::Request> req, std::shared_ptr<dsr_msgs2::srv::SetCurrentTcp::Response> res)       
    {
        if(req) res->success = 0;
    }

     //----- TOOL Service Call-back functions ------------------------------------------------------------
    void DRHWInterface::config_create_tool_cb(const std::shared_ptr<dsr_msgs2::srv::ConfigCreateTool::Request> req, std::shared_ptr<dsr_msgs2::srv::ConfigCreateTool::Response> res) 
    {
        if(req) res->success = 0;
    }
    void DRHWInterface::config_delete_tool_cb(const std::shared_ptr<dsr_msgs2::srv::ConfigDeleteTool::Request> req, std::shared_ptr<dsr_msgs2::srv::ConfigDeleteTool::Response> res)    
    {
        if(req) res->success = 0;
    }
    void DRHWInterface::get_current_tool_cb(const std::shared_ptr<dsr_msgs2::srv::GetCurrentTool::Request> req, std::shared_ptr<dsr_msgs2::srv::GetCurrentTool::Response> res)     
    {
        if(req) res->success = 0;
    }
    void DRHWInterface::set_current_tool_cb(const std::shared_ptr<dsr_msgs2::srv::SetCurrentTool::Request> req, std::shared_ptr<dsr_msgs2::srv::SetCurrentTool::Response> res)     
    {
        if(req) res->success = 0;
    }
    void DRHWInterface::set_tool_shape_cb(const std::shared_ptr<dsr_msgs2::srv::SetToolShape::Request> req, std::shared_ptr<dsr_msgs2::srv::SetToolShape::Response> res) 
    {
        if(req) res->success = 0;
    }

    //----- GRIPPER Service Call-back functions ------------------------------------------------------------
    void DRHWInterface::robotiq2_f_open_cb(const std::shared_ptr<dsr_msgs2::srv::Robotiq2FOpen::Request> req, std::shared_ptr<dsr_msgs2::srv::Robotiq2FOpen::Response> res) 
    {
        if(req) res->success = 0;
    }
    void DRHWInterface::robotiq2_f_close_cb(const std::shared_ptr<dsr_msgs2::srv::Robotiq2FClose::Request> req, std::shared_ptr<dsr_msgs2::srv::Robotiq2FClose::Response> res)
    {
        if(req) res->success = 0;
    }
    void DRHWInterface::robotiq2_f_move_cb(const std::shared_ptr<dsr_msgs2::srv::Robotiq2FMove::Request> req, std::shared_ptr<dsr_msgs2::srv::Robotiq2FMove::Response> res)
    {
        if(req) res->success = 0;
    }

    //----- SERIAL Service Call-back functions ------------------------------------------------------------
    void DRHWInterface::serial_send_data_cb(const std::shared_ptr<dsr_msgs2::srv::SerialSendData::Request> req, std::shared_ptr<dsr_msgs2::srv::SerialSendData::Response> res)
    {
        if(req) res->success = 0;
    }

}
