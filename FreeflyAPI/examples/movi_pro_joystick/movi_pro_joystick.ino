/*-----------------------------------------------------------------
	Copyright 2017 Freefly Systems

	Licensed under the Apache License, Version 2.0 (the "License");
	you may not use this file except in compliance with the License.
	You may obtain a copy of the License at

		http://www.apache.org/licenses/LICENSE-2.0

	Unless required by applicable law or agreed to in writing, software
	distributed under the License is distributed on an "AS IS" BASIS,
	WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
	See the License for the specific language governing permissions and
	limitations under the License.

	Filename: "main.c"
	This code implements a simple movi controller using an STM32 Nucleo dev kit, and an Arduino joystick shield.
	It uses the Freefly QX library to build control and parse stauts packets.
	It sends one control packet "QX277" at approximately 50Hz and recieves a status packet "QX287" packet in response.
	The joystick sheild inputs are as follows:
		- Joystick Button (D2) - cycles between the joystick controlling the gimbal in rate mode, 
			absolute position mode (Euler angles) and FIZ lens motor absolute position.
		- Button D3 - Switches gimbal kill state on/off
		- Button D4 - Resets faults on all FIZ axes and autocalibrates all
		- Button D5 - Limits range on the focus lens motor axis to a range specified by pressing, moving and releasing
		- Button D6 - Camera record start stop
		
-----------------------------------------------------------------*/

#include "Arduino.h"

// Include Freefly API
#include <FreeflyAPI.h>

// Select which Arduino board to use. Mega as default, Uno by uncommenting UNO_BOARD define.
// The Uno is the most basic example, only transmitting control commands, but not able to recieve status
// The Mega allows control and also status via it's aux 1 serial port.
//#define UNO_BOARD

// Select which reference voltage your Ardunio board/sheild uses.
// If the Arduino is a 5V referenced model, but the shield uses 3.3V, leave this uncommented
#define ANALOG_3V3_REF

// Define variables and constants
uint8_t UART_Tx_Buf[64];
float joystick_x, joystick_y, joystick_x_raw, joystick_y_raw;
uint8_t UART_Rx_char;
uint8_t control_state = 0;
int32_t D2_debounce_cntr = 0;
uint32_t print_cntr = 0;

// Prototypes
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max);
void button_debounced_D2_press_event(void);
void button_debounced_D3_press_event(void);
uint8_t get_button_debounced_D2(uint8_t cycles);
uint8_t get_button_debounced_D3(uint8_t cycles);
void print_lens_state(Lens_Axis_State_General_e state);

// Setup code
void setup()
{
    // initialize the serial communication:
    #ifdef UNO_BOARD
    Serial.begin(111111);   // Main serial port for control
    #else
    Serial.begin(115200);   // Main serial port for status
    Serial1.begin(111111);  // Aux serial port for control
    #endif
    
    //Initialize Freefly API variables
    FreeflyAPI.begin();
    
    //Setup Joystick Shield Parameters
    pinMode(2, INPUT);      //Set the Joystick button as an input
    digitalWrite(2, HIGH);  //Enable the pull-up resistor on input
    
    pinMode(3, INPUT);      //Set the Joystick button as an input
    digitalWrite(3, HIGH);  //Enable the pull-up resistor on input
    
    pinMode(4, INPUT);      //Set the Joystick button as an input
    digitalWrite(4, HIGH);  //Enable the pull-up resistor on input
    
    pinMode(5, INPUT);      //Set the Joystick button as an input
    digitalWrite(5, HIGH);  //EEnable the pull-up resistor on input
    
    pinMode(6, INPUT);      //Set the Joystick button as an input
    digitalWrite(6, HIGH);  //Enable the pull-up resistor on input
    
}

// Loop code
void loop()
{
    // Set loop to be roughly 50 HZ
    delay(20);
    
    #ifdef ANALOG_3V3_REF
    // Get Analog Inputs (3.3V Ref)
    float joystick_x_raw = mapfloat((float)analogRead(A0), 0.0f, 675.0f, -1.0f, 1.0f);
    float joystick_y_raw = mapfloat((float)analogRead(A1), 0.0f, 675.0f, -1.0f, 1.0f);
    #else
    // Get Analog Inputs (5V Ref)
    float joystick_x_raw = mapfloat((float)analogRead(A0), 0.0f, 1023.0f, -1.0f, 1.0f);
    float joystick_y_raw = mapfloat((float)analogRead(A1), 0.0f, 1023.0f, -1.0f, 1.0f);
    #endif

    // Range Limit to +/-1.0f
    if (joystick_x_raw > 1.0f){
      joystick_x_raw = 1.0f;
    } else if (joystick_x_raw < -1.0f){
      joystick_x_raw = -1.0f;
    } else {
      joystick_x_raw = joystick_x_raw;
    }
    if (joystick_y_raw > 1.0f){
      joystick_y_raw = 1.0f;
    } else if (joystick_y_raw < -1.0f){
      joystick_y_raw = -1.0f;
    } else {
      joystick_y_raw = joystick_y_raw;
    }
    
    // Handle deadband as Joystick Shield never goes to center
    // Joystick deadband
    if ((joystick_x_raw > 0.07f) || (joystick_x_raw < -0.07f)){
        joystick_x = joystick_x_raw;
    } else {
        joystick_x = 0;
    }
    if ((joystick_y_raw > 0.07f) || (joystick_y_raw < -0.07f)){
        joystick_y = joystick_y_raw;
    } else {
        joystick_y = 0;
    }
    
    // Get gimbal kill command value and apply to the ctrl structure
    get_button_debounced_D3(5);
    
    // Handle control mode
    get_button_debounced_D2(5);
    switch (control_state) 
    {
        // Pan/Tilt rate control
        case 0:
            FreeflyAPI.control.pan.type = RATE;
            FreeflyAPI.control.tilt.type = RATE;
            FreeflyAPI.control.roll.type = DEFER;
            FreeflyAPI.control.focus.type = DEFER;
            FreeflyAPI.control.iris.type = DEFER;
            FreeflyAPI.control.zoom.type = DEFER;
				
            FreeflyAPI.control.pan.value = joystick_x;
            FreeflyAPI.control.tilt.value = joystick_y;
            FreeflyAPI.control.roll.value = 0;
            FreeflyAPI.control.focus.value = 0;
            FreeflyAPI.control.iris.value = 0;
            FreeflyAPI.control.zoom.value = 0;
            break;
            
        // Pan/Tilt absolute position control
        case 1:
            FreeflyAPI.control.pan.type = ABSOLUTE;
            FreeflyAPI.control.tilt.type = ABSOLUTE;
            FreeflyAPI.control.roll.type = DEFER;
            FreeflyAPI.control.focus.type = DEFER;
            FreeflyAPI.control.iris.type = DEFER;
            FreeflyAPI.control.zoom.type = DEFER;
            
            // use value without deadband for position control
            FreeflyAPI.control.pan.value = joystick_x_raw;	
            FreeflyAPI.control.tilt.value = joystick_y_raw;
            FreeflyAPI.control.roll.value = 0;
            FreeflyAPI.control.focus.value = 0;
            FreeflyAPI.control.iris.value = 0;
            FreeflyAPI.control.zoom.value = 0;
            break;
			
        // Focus/Iris absolute position control
        case 2:
            FreeflyAPI.control.pan.type = DEFER;
            FreeflyAPI.control.tilt.type = DEFER;
            FreeflyAPI.control.roll.type = DEFER;
            FreeflyAPI.control.focus.type = ABSOLUTE;
            FreeflyAPI.control.iris.type = ABSOLUTE;
            FreeflyAPI.control.zoom.type = ABSOLUTE;
            
            // use value without deadband for position control
            FreeflyAPI.control.pan.value = 0;
            FreeflyAPI.control.tilt.value = 0;
            FreeflyAPI.control.roll.value = 0;
            FreeflyAPI.control.focus.value = joystick_x_raw;
            FreeflyAPI.control.iris.value = joystick_y_raw;
            FreeflyAPI.control.zoom.value = 0;
            break;
			
        default:
            FreeflyAPI.control.pan.type = DEFER;
            FreeflyAPI.control.tilt.type = DEFER;
            FreeflyAPI.control.roll.type = DEFER;
            FreeflyAPI.control.focus.type = DEFER;
            FreeflyAPI.control.iris.type = DEFER;
            FreeflyAPI.control.zoom.type = DEFER;
            
            FreeflyAPI.control.pan.value = 0;
            FreeflyAPI.control.tilt.value = 0;
            FreeflyAPI.control.roll.value = 0;
            FreeflyAPI.control.focus.value = 0;
            FreeflyAPI.control.iris.value = 0;
            FreeflyAPI.control.zoom.value = 0;
            break;
    }
        
    // Clear lens controller axist faults. If a motor is not detected, the axis will fault and reuire that this is pressed to continue
    FreeflyAPI.control.fiz_clearFaults_all_flag = (LOW == digitalRead(4)) ? 1 : 0;
    FreeflyAPI.control.fiz_autoCalStart_all_flag = FreeflyAPI.control.fiz_clearFaults_all_flag;	// do an auto cal directly out of reset. These can be independent also.
    
    // Press and hold this button to set a sub range limit. Focus is used for example. Press sets range start, release sets range end
    FreeflyAPI.control.fiz_setSubRangeLim_F_flag = (LOW == digitalRead(5)) ? 1 : 0;
    
    // Starts / Stops configured camera
    FreeflyAPI.control.fiz_record_button_flag = (LOW == digitalRead(6)) ? 1 : 0;

    #ifndef UNO_BOARD
    // Get Recieved Messages
    int i = 64;
    uint8_t c;
    while (Serial1.available() > 0)
    {
        c = Serial1.read();
        QX_StreamRxCharSM(QX_COMMS_PORT_UART, c);
    }
    #endif
    
    // Send control packet
    FreeflyAPI.send();
        
    // Empty the send buffer
    for(int i = 0; i < sizeof(UART_Tx_Buf); i++){
        if(BufRemove(1, &UART_Tx_Buf[i]) == 0) break;
        #ifdef UNO_BOARD
        Serial.write(UART_Tx_Buf[i]);
        #else
        Serial1.write(UART_Tx_Buf[i]);
        #endif
    }

    #ifndef UNO_BOARD
    // Print out status packets
    if (print_cntr++ > 20)
    {
        char print_buffer[200];
        char printf_float_buf[4][10];
        print_cntr = 0;
        Serial.print("---------------------------------------------\n\r");
        Serial.print("MoVI Gimbal and Status Data: \n\r");
        if (control_state == 0) Serial.print("Test control mode: Gimbal rate \n\r");
        if (control_state == 1) Serial.print("Test control mode: Gimbal absolute position \n\r");
        if (control_state == 2) Serial.print("Test control mode: Focus and iris absolute position \n\r");
        dtostrf(FreeflyAPI.status.battery_v_left, 4, 2, printf_float_buf[0]);
        sprintf(print_buffer, "Left Battery: %s V\n\r", printf_float_buf[0]);
        Serial.print(print_buffer);
        dtostrf(FreeflyAPI.status.battery_v_right, 4, 2, printf_float_buf[0]);
        sprintf(print_buffer, "Right Battery: %s V\n\r", printf_float_buf[0]);
        Serial.print(print_buffer);
        dtostrf(FreeflyAPI.status.gimbal_i, 4, 4, printf_float_buf[0]);
        dtostrf(FreeflyAPI.status.gimbal_j, 4, 4, printf_float_buf[1]);
        dtostrf(FreeflyAPI.status.gimbal_k, 4, 4, printf_float_buf[2]);
        dtostrf(FreeflyAPI.status.gimbal_r, 4, 4, printf_float_buf[3]);
        sprintf(print_buffer, "Gimbal Position (quaternion): %s, %s, %s, %s \n\r", printf_float_buf[0], printf_float_buf[1], printf_float_buf[2], printf_float_buf[3]);
        Serial.print(print_buffer);
        Serial.print("Camera: ");
        (FreeflyAPI.status.camera_recording == 0) ? Serial.print("Stopped\n\r") : Serial.print("Recording\n\r");
        Serial.print("Lens Range Limits: ");
        (FreeflyAPI.status.focus_range_limits_active == 0) ? Serial.print("Focus: OFF, ") : Serial.print("Focus: ON, ");
        (FreeflyAPI.status.iris_range_limits_active == 0) ? Serial.print("Iris: OFF, ") : Serial.print("Iris: ON, ");
	      (FreeflyAPI.status.zoom_range_limits_active == 0) ? Serial.print("Zoom: OFF \n\r") : Serial.print("Zoom: ON \n\r");
	      sprintf(print_buffer, "Focus Position: %d counts\n\r", FreeflyAPI.status.focus_position);
        Serial.print(print_buffer);
	      sprintf(print_buffer, "Iris Position: %d counts\n\r", FreeflyAPI.status.iris_position);
        Serial.print(print_buffer);
	      sprintf(print_buffer, "Zoom Position: %d counts\n\r", FreeflyAPI.status.zoom_position);
        Serial.print(print_buffer);
	      Serial.print("Focus State: ");
	      print_lens_state(FreeflyAPI.status.focus_state);
	      Serial.print("Iris State: ");
	      print_lens_state(FreeflyAPI.status.iris_state);
	      Serial.print("Zoom State: ");
	      print_lens_state(FreeflyAPI.status.zoom_state);
	      Serial.print("\r\n");
    }
    #endif
    
}

//----------------------------------------------------------------------------
// Print lens state to print buffer
void print_lens_state(Lens_Axis_State_General_e state)
{
	switch (state)
	{
    case Lens_AxisState_Disabled: Serial.print("Disabled"); break;
    case Lens_AxisState_Reset: Serial.print("Reset"); break;
		case Lens_AxisState_Faulted: Serial.print("Faulted"); break;
		case Lens_AxisState_Move_to_Command: Serial.print("Moving to commanded position"); break;
		case Lens_AxisState_Calibrated: Serial.print("Calibrated"); break;
		case Lens_AxisState_Uncalibrated: Serial.print("Uncalibrated"); break;
		case Lens_AxisState_Man_Cal_Set_Max: Serial.print("Manual Cal - Set max"); break;
		case Lens_AxisState_Man_Cal_Set_Min: Serial.print("Manual Cal - Set min"); break;
		case Lens_AxisState_Auto_Cal_SensingTorque: Serial.print("Auto Cal - Sensing torque"); break;
		case Lens_AxisState_Auto_Cal_Set_Max: Serial.print("Auto Cal - Set max"); break;
		case Lens_AxisState_Auto_Cal_Set_Min: Serial.print("Auto Cal - Set min"); break;
		default: break;
	}
	Serial.print("\r\n");
}


//----------------------------------------------------------------------------
uint8_t get_button_debounced_D2(uint8_t cycles)
{
	static uint8_t value = 0;
	static uint8_t value_last = 0;
	static uint8_t debounce_cntr = 0;
	
	if (LOW == digitalRead(2)) // Pressed
	{
		debounce_cntr++;
		if (debounce_cntr >= cycles){
			value = 1;
			debounce_cntr = cycles;
		}
	}
	else	// Raised
	{
		debounce_cntr--;
		if (debounce_cntr <= 0){
			debounce_cntr = 0;
			value = 0; 
		}
	}
	
	if ((value_last == 0) && (value == 1))
	{
		button_debounced_D2_press_event();
	}
	value_last = value;
	
	return value;
}

//----------------------------------------------------------------------------
uint8_t get_button_debounced_D3(uint8_t cycles)
{
	static uint8_t value = 0;
	static uint8_t value_last = 0;
	static uint8_t debounce_cntr = 0;
	
	if (LOW == digitalRead(3)) // Pressed
	{
		debounce_cntr++;
		if (debounce_cntr >= cycles){
			value = 1;
			debounce_cntr = cycles;
		}
	}
	else	// Raised
	{
		debounce_cntr--;
		if (debounce_cntr <= 0){
			debounce_cntr = 0;
			value = 0; 
		}
	}
	
	if ((value_last == 0) && (value == 1))
	{
		button_debounced_D3_press_event();
	}
	value_last = value;
	
	return value;
}


//----------------------------------------------------------------------------
// cycle through control types
void button_debounced_D2_press_event(void)
{
	control_state++;
	if (control_state > 2) control_state = 0;
}


//----------------------------------------------------------------------------
// toggle gimbal kill state
void button_debounced_D3_press_event(void)
{
	FreeflyAPI.control.gimbal_kill = (FreeflyAPI.control.gimbal_kill == 0) ? 1 : 0;
}


//----------------------------------------------------------------------------
//Mapping function
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}



