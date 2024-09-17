//#######################################################################################################
// Name:             jupiter_config.h
// Purpose:          Main configuration header file for Jupiter Robot
// Description:      ESP32 pins and Robot properties can be user defined here 
// Related Files:         
// Author:           logan naidoo, south africa, 2024
//########################################################################################################

// Motor Pins
#define MOTOR1_PWM  32
#define MOTOR1_DIR  33
#define MOTOR1_ENC_B  25
#define MOTOR1_ENC_A  26

#define MOTOR2_PWM  23 //19
#define MOTOR2_DIR  19 //23
#define MOTOR2_ENC_A  18
#define MOTOR2_ENC_B  5

#define MOTOR3_PWM  27
#define MOTOR3_DIR  14
#define MOTOR3_ENC_B  12
#define MOTOR3_ENC_A  13

#define MOTOR4_PWM  17 //16
#define MOTOR4_DIR  16 //17
#define MOTOR4_ENC_A  4
#define MOTOR4_ENC_B  15 

// Define Onboard LED
#define ESP32_LED 2


// FOR PID
#define K_P 2.0 //2.0                      // P constant (See Note (2) below)
#define K_I 5.0 //5.0                      // I constant
#define K_D 0.0 //0.0                      // D constant


// MOTOR AND ROBOT SPECS
#define MOTOR_MAX_RPM 330                   // motor's max RPM          
#define MAX_RPM_RATIO 0.65                  // max RPM allowed for each MAX_RPM_ALLOWED = MOTOR_MAX_RPM * MAX_RPM_RATIO          
#define MOTOR_OPERATING_VOLTAGE 12          // motor's operating voltage (used to calculate max RPM)
#define MOTOR_POWER_MAX_VOLTAGE 12          // max voltage of the motor's power source (used to calculate max RPM)
#define MOTOR_POWER_MEASURED_VOLTAGE 12     // current voltage reading of the power connected to the motor (used for calibration)
#define COUNTS_PER_REV1 1320                // wheel1 encoder's no of ticks per rev
#define COUNTS_PER_REV2 1320                // wheel2 encoder's no of ticks per rev
#define COUNTS_PER_REV3 1320                // wheel3 encoder's no of ticks per rev
#define COUNTS_PER_REV4 1320                // wheel4 encoder's no of ticks per rev
#define WHEEL_DIAMETER 0.090                // wheel's diameter in meters
#define LR_WHEELS_DISTANCE 0.400            // distance between left and right wheels
#define PWM_BITS 10                         // PWM Resolution of the microcontroller
#define PWM_FREQUENCY 8000                  // PWM Frequency
#define PWM_MAX pow(2, PWM_BITS) - 1        // e.g. for 8-bit PWM_MAX = 2^8 - 1 = 256 - 1 = 255
#define PWM_MIN -PWM_MAX
#define USE_PID true                        // true = PID controller ON. Switch OFF (false). See Note (1) below
#define WHEEL_RADIUS 0.045                  // in meters
#define WHEEL_SEPARATION 0.400              // distance between left and rigth wheels (centre to centre) in meters
#define PWM_FWD_MIN 1                       // these two PWM_FWD & PWM_REV are needed to balance the startup power needed 
#define PWM_REV_MIN 1                       // for each direction (helps to drive straight)

// NOTES
// (1)  Without PID control, the robot motors may only spin at higher speed commands from cmd_vel as there is no feedback from the wheel encoders
//      about the current speed. Essentially, there is no 'error' that can be reduced auto-matically by the open loop system.
//      So the operator (human) will need to increase speeds manually until the robot moves.  The operator will be the 'closed-loop" control system.
//      The PID controller continuously adjusts the PWM signals to the motors to get the current speeds to the target speeds using the wheel encoder
//      data as feedback measure of the current speed.
//
// (2)  The Kp, Ki and Kd settings for PID control is different for each motor and robot kinematics.  This will need to be tuned manually.
//      Start with Kp first, with Ki and Kd = 0.  Then adjust each until there is a smooth rotation of the wheels.
//      It may be necessary to watch some YouTube examples about PID tuning.  Also make sure that the PID control loop (dt) has sufficient time to 
//      execute the PID calculations needed to control the system.