/*-----------------------------------------------------------------
	MIT License

	Copyright (c) 2017 Freefly Systems

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in all
	copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
	SOFTWARE.
		
    Filename: "QX_Protocol_App.h"
-----------------------------------------------------------------*/

#ifndef QX_PROTOCOL_APP_H
#define QX_PROTOCOL_APP_H

//****************************************************************************
// Defines
//****************************************************************************

// Gimbal Control Flag Definitions
#define INPUT_CONTROL_RZ_DEFER 0x00
#define INPUT_CONTROL_RZ_RATE 0x01
#define INPUT_CONTROL_RZ_ABS 0x02
#define INPUT_CONTROL_RZ_ABS_MAJ 0x03
#define INPUT_CONTROL_RY_DEFER 0x00
#define INPUT_CONTROL_RY_RATE 0x04
#define INPUT_CONTROL_RY_ABS 0x08
#define INPUT_CONTROL_RY_ABS_MAJ 0x0C
#define INPUT_CONTROL_RX_DEFER 0x00
#define INPUT_CONTROL_RX_RATE 0x10
#define INPUT_CONTROL_RX_ABS 0x20
#define INPUT_CONTROL_RX_ABS_MAJ 0x30
#define INPUT_CONTROL_QUATERNION 0x80
#define INPUT_CONTROL_KILL 0x40

// Lens Control Flag Definitions
#define INPUT_CONTROL_LZ_DEFER 0x00
#define INPUT_CONTROL_LZ_RATE 0x01
#define INPUT_CONTROL_LZ_ABS 0x02
#define INPUT_CONTROL_LI_DEFER 0x00
#define INPUT_CONTROL_LI_RATE 0x04
#define INPUT_CONTROL_LI_ABS 0x08
#define INPUT_CONTROL_LF_DEFER 0x00
#define INPUT_CONTROL_LF_RATE 0x10
#define INPUT_CONTROL_LF_ABS 0x20
#define INPUT_CONTROL_REC 0x40

//****************************************************************************
// Headers
//****************************************************************************

// Standard headers
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

// QX Library headers
#include "QX_Protocol.h"
#include "QX_App_Config.h"
#include "simple_buffer.h"

//****************************************************************************
// Definitions
//****************************************************************************

//****************************************************************************
// Data Types
//****************************************************************************

typedef enum type{
    DEFER = 0,
    RATE,
    ABSOLUTE
}ff_api_control_type_e;

typedef struct {
    float value;
    ff_api_control_type_e type;
} ff_api_realtime_control_t;

// Lens Axis_State Enumeration Sent as Status in Communications
typedef enum {
  Lens_AxisState_Disabled = 0,
  Lens_AxisState_Reset,
  Lens_AxisState_Faulted,
  Lens_AxisState_Move_to_Command,
  Lens_AxisState_Calibrated,
  Lens_AxisState_Uncalibrated,
  Lens_AxisState_Man_Cal_Set_Max,
  Lens_AxisState_Man_Cal_Set_Min,
  Lens_AxisState_Auto_Cal_SensingTorque,
  Lens_AxisState_Auto_Cal_Set_Max,
  Lens_AxisState_Auto_Cal_Set_Min,
} Lens_Axis_State_General_e;

// gimbal and fiz control data
typedef struct {
	
	// gimbal flags
	uint8_t gimbal_kill;
	uint8_t gimbal_position_type_quaternions;
	
	// gimbal controls
	ff_api_realtime_control_t pan;
    ff_api_realtime_control_t tilt;
    ff_api_realtime_control_t roll;
	
	// lens controls
	ff_api_realtime_control_t focus;
    ff_api_realtime_control_t iris;
    ff_api_realtime_control_t zoom;
	
	// fiz control flags
	uint8_t fiz_clearFaults_all_flag;
	uint8_t fiz_autoCalStart_all_flag;
	uint8_t fiz_record_button_flag;
	uint8_t fiz_setSubRangeLim_F_flag;
	uint8_t fiz_setSubRangeLim_I_flag;
	uint8_t fiz_setSubRangeLim_Z_flag;
	
} ff_api_gimbal_and_fiz_control_t;


// gimbal and fiz status data
typedef struct {
	
	// gimbal battery voltage
	float battery_v_left;
	float battery_v_right;
	
	// gimbal status
	uint8_t gimbal_Status1;
	uint8_t gimbal_Status2;
	
	// gimbal quaternions
	float gimbal_i;
	float gimbal_j;
	float gimbal_k;
	float gimbal_r;
	
	// lens positions
	uint16_t focus_position;
	uint16_t iris_position;
	uint16_t zoom_position;
	
	// lens enumerated states
	Lens_Axis_State_General_e focus_state;
	Lens_Axis_State_General_e iris_state;
	Lens_Axis_State_General_e zoom_state;
	
	// fiz status - range limits
	uint8_t fiz_status;
	uint8_t focus_range_limits_active;
	uint8_t iris_range_limits_active;
	uint8_t zoom_range_limits_active;
	
	// camera status
	uint8_t camera_status;
	uint8_t camera_recording;
	
} ff_api_status_t;


// overall api structure
typedef struct {
    //Initialize API and QX Comms
    void (* begin)();
    
    //Send commands to MōVI Pro
    void (* send)();
	
	// Recieve messages from MoVI Pro (and respond if needed)
	void (* recv)();
	
	// Control and Status structs
	ff_api_gimbal_and_fiz_control_t control;
	ff_api_status_t status;
	
} ff_api_t;


//****************************************************************************
// Public Global Vars
//****************************************************************************
extern ff_api_t FreeflyAPI;


//****************************************************************************
// Public Function Prototypes
//****************************************************************************
extern uint8_t *QX_ParsePacket_Cli_MoVI_Ctrl_CB(QX_Msg_t *Msg_p);


#endif

