/*    ========================================================================
    =                   Doosan Robot Framework Structure                      =
    =                   Copyright (c) Doosan Robotics.                        =   
    =_______________________________________________________________________  =
    = Title             : Doosan Robot Framwork Structure                     =
    = Author            : Lee Jeong-Woo<jeongwoo1.lee@doosan.com>             =
    = Description       : -                                                   =
    = Version           : 1.0 (GL010105) first release                        =
    =                     1.1 (GF020300) add force control                    =
    =                                    add coordinate sytem control function      =
    =                                    fix GetCurrentTool, GetCurrentTCP function = 
    ======================================================================== */

/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Doosan Robotics
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

#pragma once
#include "DRFC.h"

#ifdef __cplusplus
#include <string>
#include <list>
using namespace std;
#endif

#pragma pack(1)
typedef struct _SYSTEM_VERSION
{
    /* smarttp version */
    char                        _szSmartTp[MAX_SYMBOL_SIZE];
    /* controller version */
    char                        _szController[MAX_SYMBOL_SIZE];
    /* interpreter version */
    char                        _szInterpreter[MAX_SYMBOL_SIZE];
    /* inverter version */
    char                        _szInverter[MAX_SYMBOL_SIZE];
    /* SafetyBoard version */
    char                        _szSafetyBoard[MAX_SYMBOL_SIZE];
    /* robot serial number */
    char                        _szRobotSerial[MAX_SYMBOL_SIZE];
    /* robot model number*/
    char                        _szRobotModel[MAX_SYMBOL_SIZE];
    /* jts board version */
    char                        _szJTSBoard[MAX_SYMBOL_SIZE];
    /* flange board version */
    char                        _szFlangeBoard[MAX_SYMBOL_SIZE];

} SYSTEM_VERSION, *LPSYSTEM_VERSION;

typedef struct _ROBOT_MONITORING_JOINT 
{
    /* Position Actual Value in INC */
    float                       _fActualPos[NUM_JOINT];
    /* Position Actual Value in ABS */
    float                       _fActualAbs[NUM_JOINT];
    /* Velocity Actual Value */
    float                       _fActualVel[NUM_JOINT];
    /* Joint Error */
    float                       _fActualErr[NUM_JOINT];
    /* Target Position */
    float                       _fTargetPos[NUM_JOINT];
    /* Target Velocity */
    float                       _fTargetVel[NUM_JOINT];

} ROBOT_MONITORING_JOINT, *LPROBOT_MONITORING_JOINT;

typedef struct _ROBOT_MONITORING_TASK 
{
    /* Position Actual Value(0: tool, 1: flange) */
    float                       _fActualPos[2][NUMBER_OF_JOINT]; 
    /* Velocity Actual Value */
    float                       _fActualVel[NUMBER_OF_JOINT];
    /* Task Error */
    float                       _fActualErr[NUMBER_OF_JOINT];
    /* Target Position */
    float                       _fTargetPos[NUMBER_OF_JOINT];
    /* Target Velocity */
    float                       _fTargetVel[NUMBER_OF_JOINT];
    /* Solution Space */
    unsigned char               _iSolutionSpace;
    /* Rotation Matrix */
    float                       _fRotationMatrix[3][3];

} ROBOT_MONITORING_TASK, *LPROBOT_MONITORING_TASK;

typedef struct _ROBOT_MONITORING_WORLD
{
    /* world to base relation */
    float                       _fActualW2B[NUMBER_OF_JOINT];
    /* Position Actual Value  (0:tool, 1:flange) */
    float                       _fActualPos[2][NUMBER_OF_JOINT]; 
    /* Velocity Actual Value */
    float                       _fActualVel[NUMBER_OF_JOINT];
    /* External Task Force/Torque */
    float                       _fActualETT[NUMBER_OF_JOINT];    
    /* Target Position */
    float                       _fTargetPos[NUMBER_OF_JOINT];
    /* Target Velocity */
    float                       _fTargetVel[NUMBER_OF_JOINT];   
    /* Rotation Matrix */
    float                       _fRotationMatrix[3][3];

} ROBOT_MONITORING_WORLD, *LPROBOT_MONITORING_WORLD;

typedef struct _ROBOT_MONITORING_USER
{
    /* actual user coord no */
    unsigned char               _iActualUCN;
    /* base: 0 world: 2 */
    unsigned char               _iParent;
    /* Position Actual Value  (0:tool, 1:flange) */
    float                       _fActualPos[2][NUMBER_OF_JOINT]; 
    /* Velocity Actual Value */
    float                       _fActualVel[NUMBER_OF_JOINT];
    /* External Task Force/Torque */
    float                       _fActualETT[NUMBER_OF_JOINT];       
    /* Target Position */
    float                       _fTargetPos[NUMBER_OF_JOINT];
    /* Target Velocity */
    float                       _fTargetVel[NUMBER_OF_JOINT];
    /* Rotation Matrix */
    float                       _fRotationMatrix[3][3];

} ROBOT_MONITORING_USER, *LPROBOT_MONITORING_USER;


typedef struct _ROBOT_MONITORING_TORQUE 
{
    /* Dynamics Torque */
    float                       _fDynamicTor[NUM_JOINT];
    /* Joint Torque Sensor Value */
    float                       _fActualJTS[NUM_JOINT];
    /* External Joint Torque */
    float                       _fActualEJT[NUM_JOINT];
    /* External Task Force/Torque */
    float                       _fActualETT[NUM_JOINT];

} ROBOT_MONITORING_TORQUE, *LPROBOT_MONITORING_TORQUE;

typedef struct _ROBOT_MONITORING_STATE 
{
    /* position control: 0, torque control: 1*/
    unsigned char               _iActualMode;
    /*joint space: 0, task space: 1*/
    unsigned char               _iActualSpace;

} ROBOT_MONITORING_STATE, *LPROBOT_MONITORING_STATE;

typedef struct _ROBOT_MONITORING_DATA
{
    ROBOT_MONITORING_STATE      _tState;
    /* joint */
    ROBOT_MONITORING_JOINT      _tJoint;
    /* task */
    ROBOT_MONITORING_TASK       _tTask;
    /* torque */
    ROBOT_MONITORING_TORQUE     _tTorque;  

} MONITORING_CONTROL, *LPMONITORING_CONTROL;

typedef struct _ROBOT_MONITORING_DATA_EX
{
    ROBOT_MONITORING_STATE      _tState;
    /* joint */
    ROBOT_MONITORING_JOINT      _tJoint;
    /* task */
    ROBOT_MONITORING_TASK       _tTask;
    /* torque */
    ROBOT_MONITORING_TORQUE     _tTorque;
    /* world */
    ROBOT_MONITORING_WORLD      _tWorld;
    /*user */
    ROBOT_MONITORING_USER       _tUser;
} ROBOT_MONITORING_DATA_EX, *LPROBOT_MONITORING_DATA_EX;

typedef ROBOT_MONITORING_DATA_EX
    MONITORING_CONTROL_EX, *LPMONITORING_CONTROL_EX;

typedef struct _MONITORING_MISC
{
    /* inner clock counter */
    double                       _dSyncTime;
    /* Flange Digtal Input data */
    unsigned char                _iActualDI[NUM_FLANGE_IO];
    /* Flange Digtal output data */
    unsigned char                _iActualDO[NUM_FLANGE_IO];
    /* brake state */
    unsigned char                _iActualBK[NUM_JOINT];
    /* robot button state */
    unsigned int                 _iActualBT[NUM_BUTTON];
    /* motor input current */
    float                        _fActualMC[NUM_JOINT];
    /* motro current temperature */
    float                        _fActualMT[NUM_JOINT];
} MONITORING_MISC, *LPMONITORING_MISC;

typedef struct _MONITORING_DATA
{
    MONITORING_CONTROL          _tCtrl;
    /* misc. */
    MONITORING_MISC             _tMisc;

} MONITORING_DATA, *LPMONITORING_DATA;

typedef struct _MONITORING_DATA_EX
{
    MONITORING_CONTROL_EX       _tCtrl;
    /* misc. */
    MONITORING_MISC             _tMisc;

} MONITORING_DATA_EX, *LPMONITORING_DATA_EX;

typedef struct _READ_CTRLIO_INPUT
{
    /* Digtal Input data */
    unsigned char               _iActualDI[NUM_DIGITAL];
#if 0
    /*  Analog Input type */
    unsigned char               _iActualAT[NUM_ANALOG];
#endif
    /* Analog Input data */
    float                       _fActualAI[NUM_ANALOG];
    /* switch input data */
    unsigned char               _iActualSW[NUM_SWITCH];
    /* Safety Input data */
    unsigned char               _iActualSI[NUM_SAFETY_IN];
    /* Encorder Input data */
    unsigned char               _iActualEI[NUM_ENCORDER];
    /* Encorder raw data */
    unsigned int                _iAcutualED[NUM_ENCORDER];

} READ_CTRLIO_INPUT, *LPREAD_CTRLIO_INPUT;

typedef struct _READ_CTRLIO_OUTPUT
{
    /* Digital Output data */
    unsigned char               _iTargetDO[NUM_DIGITAL];
#if 0
    /*  Analog Output type */
    unsigned char               _fTargetAT[NUM_ANALOG];
#endif    
    /* Analog Output data */
    float                       _fTargetAO[NUM_ANALOG];

} READ_CTRLIO_OUTPUT, *LPREAD_CTRLIO_OUTPUT;

typedef struct _READ_CTRLIO_INPUT_EX
{
    /* Digtal Input data */
    unsigned char               _iActualDI[NUM_DIGITAL];
    /* Analog Input data */
    float                       _fActualAI[NUM_ANALOG];
    /* switch input data */
    unsigned char               _iActualSW[NUM_SWITCH];
    /* Safety Input data */
    unsigned char               _iActualSI[NUM_SAFETY_IN];
    /*  Analog Input type */
    unsigned char               _iActualAT[NUM_ANALOG];
    
} READ_CTRLIO_INPUT_EX, *LPREAD_CTRLIO_INPUT_EX;

typedef struct _READ_ENCODER_INPUT
{
    /* Encorder strove signal */
    unsigned char               _iActualES[NUM_ENCORDER];
    /* Encorder raw data */
    unsigned int                _iActualED[NUM_ENCORDER];
    /* Encorder Reset signal */
    unsigned char               _iActualER[NUM_ENCORDER];

} READ_ENCODER_INPUT, *LPREAD_ENCODER_INPUT;

typedef struct _READ_CTRLIO_OUTPUT_EX
{
    /* Digital Output data */
    unsigned char               _iTargetDO[NUM_DIGITAL];
    /* Analog Output data */
    float                       _fTargetAO[NUM_ANALOG];
    /*  Analog Output type */
    unsigned char               _iTargetAT[NUM_ANALOG];

} READ_CTRLIO_OUTPUT_EX, *LPREAD_CTRLIO_OUTPUT_EX;

typedef struct _READ_PROCESS_INPUT
{
    /* Digtal Input data */
    unsigned char               _iActualDI[4];

} READ_PROCESS_INPUT, *LPREAD_PROCESS_INPUT;

typedef struct _MONITORING_CTRLIO
{
    /* input data */
    READ_CTRLIO_INPUT           _tInput;
    /* output data */
    READ_CTRLIO_OUTPUT          _tOutput;
} MONITORING_CTRLIO, *LPMONITORING_CTRLIO;


typedef struct _MONITORING_CTRLIO_EX
{
    /* input data */
    READ_CTRLIO_INPUT_EX        _tInput;
    /* output data */
    READ_CTRLIO_OUTPUT_EX       _tOutput;
    /* input encoder data*/
    READ_ENCODER_INPUT          _tEncoder;
    /* reserved data */
    unsigned char               _szReserved[24];
} MONITORING_CTRLIO_EX, *LPMONITORING_CTRLIO_EX;

typedef struct _MODBUS_REGISTER
{
    /* modbus i/o name */
    char                        _szSymbol[MAX_SYMBOL_SIZE];
    /* modbus i/o  value */
    unsigned short              _iValue;

} MODBUS_REGISTER, *LPMODBUS_REGISTER;

typedef struct _MONITORING_MODBUS
{
    /* modbus i/o count */
    unsigned short              _iRegCount;
    /* modbus i/o values */
    MODBUS_REGISTER             _tRegister[MAX_MODBUS_TOTAL_REGISTERS];

} MONITORING_MODBUS, *LPMONITORING_MODBUS;

typedef struct _LOG_ALARM
{
    /* message level */
    unsigned char               _iLevel;
    /* group no */
    unsigned char               _iGroup;
    /* message no */
    unsigned int                _iIndex;
    /* message param */
    char                        _szParam[3][MAX_STRING_SIZE];
} LOG_ALARM, *LPLOG_ALARM;

typedef LOG_ALARM MONITORING_ALARM;

typedef struct _MESSAGE_PROGRESS
{
    /* current step */
    unsigned char               _iCurrentCount;
    /*  total step  */
    unsigned char               _iTotalCount;
} MESSAGE_PROGRESS, *LPMESSAGE_PROGRESS;

typedef struct _MESSAGE_POPUP
{
    /* message string */
    char                        _szText[MAX_STRING_SIZE];
    /*  Message: 0, Warning: 1, Alarm: 2 */
    unsigned char               _iLevel;
    /*  resuem and stop : 0, ok : 1 */
    unsigned char               _iBtnType;

} MESSAGE_POPUP, *LPMESSAGE_POPUP;

typedef MESSAGE_POPUP MONITORING_POPUP;

typedef struct _MESSAGE_INPUT
{
    /* message string */
    char                        _szText[MAX_STRING_SIZE];
    /*  int: 0, float: 1, string: 2 , bool: 3*/
    unsigned char               _iType;
} MESSAGE_INPUT, *LPMESSAGE_INPUT;

typedef MESSAGE_INPUT MONITORING_INPUT;

typedef struct _CONTROL_BRAKE
{
    /* joint: 0~5 , all joint: 6 */
    unsigned char               _iTargetAxs;
    /*  on: 1 off: 0 */
    unsigned char               _bValue;
} CONTROL_BRAKE, *LPCONTROL_BRAKE;

typedef struct _MOVE_POSB 
{
    /*  q               */
    float                       _fTargetPos[2][NUM_TASK];      
    /* blending motion type (line: 0, circle: 1) */
    unsigned char               _iBlendType;    
    /* blending radius  */
    float                       _fBlendRad;

} MOVE_POSB, *LPMOVE_POSB;

typedef struct _ROBOT_POSE
{
    /* current pose */
    float                       _fPosition[NUM_JOINT];

} ROBOT_POSE, *LPROBOT_POSE;

typedef struct _ROBOT_VEL
{
    /* current velocity */
    float                       _fVelocity[NUM_JOINT];
} ROBOT_VEL, *LPROBOT_VEL;

typedef struct _ROBOT_FORCE
{
    /* current force */
    float                       _fForce[NUM_JOINT];
} ROBOT_FORCE, *LPROBOT_FORCE;

typedef struct _ROBOT_TASK_POSE
{
    /* target pose */
    float                       _fTargetPos[NUM_TASK];
    /*  solution space: 0 ~ 7 */
    unsigned char               _iTargetSol;

} ROBOT_TASK_POSE, *LPROBOT_TASK_POSE;

typedef struct _USER_COORDINATE
{
    unsigned char _iReqId;
    unsigned char _iTargetRef;
    float _fTargetPos[NUM_TASK];
} USER_COORDINATE, *LPUSER_COORDINATE;

typedef struct _MEASURE_TOOL_RESPONSE
{
    /* mass(kg) */
    float                       _fWeight;
    /* center of mass */
    float                       _fXYZ[3];
    
} MEASURE_TOOL_RESPONSE, *LPMEASURE_TOOL_RESPONSE;

typedef struct _CONFIG_TCP
{
    /* target pose */
    float                       _fTargetPos[NUM_JOINT];

} CONFIG_TCP, *LPCONFIG_TCP;

typedef struct _CONFIG_TOOL
{
    /* mass(kg) */
    float                       _fWeight;
    /* center of mass */
    float                       _fXYZ[3];
    /* inertia */
    float                       _fInertia[NUMBER_OF_JOINT];    

} CONFIG_TOOL, *LPCONFIG_TOOL;

typedef struct _MEASURE_TCP_RESPONSE
{
    /* mesure pose  */
    CONFIG_TCP                  _tTCP;
    /* mesure error */
    float                       _fError;

} MEASURE_TCP_RESPONSE, *LPMEASURE_TCP_RESPONSE;

typedef struct _FLANGE_SERIAL_DATA
{
    unsigned char              _iCommand;       // 0 : OPEN, 1: CLOSE
                                                // 2 : SEND, 3: RECV
    union {
        struct {
            /* Baudrate */
            unsigned char               _szBaudrate[7]; // "0115200"(ASCII)
            /* Data Length */
            unsigned char               _szDataLength;  // 0: 1bit, 7: 8bit
            /* Parity */
            unsigned char               _szParity;      // 0x00 : Non-parity, 0x01 : Odd, 0x02 : Even
            /* Stop Bit */
            unsigned char               _szStopBit;     // 0x01 : One stop bit, 0x02 : Two Stop bit

        } _tConfig;

        //unsigned char                   _szConfigData[10];

        struct {
            unsigned short             _iLength;

            unsigned char              _szValue[MAX_SYMBOL_SIZE];

        } _tValue;
    } _tData;
} FLANGE_SERIAL_DATA, *LPFLANGE_SERIAL_DATA;

typedef struct _FLANGE_SER_RXD_INFO
{
    /* size of serial data */
    short                               _iSize;      //max 256bytes
    /* raw serial data */
    unsigned char                       _cRxd[256];

} FLANGE_SER_RXD_INFO, *LPFLANGE_SER_RXD_INFO;

typedef struct _READ_FLANGE_SERIAL
{
    // check ready to read
    unsigned char               _bRecvFlag;    // 0: non-receive, 1: received
} READ_FLANGE_SERIAL, *LPREAD_FLANGE_SERIAL;

#pragma pack()
