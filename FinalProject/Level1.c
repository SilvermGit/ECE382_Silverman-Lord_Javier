#include <stdbool.h>
#include "msp.h"                    // MSP432 microcontroller library
#include "../inc/Clock.h"           // System clock management
#include "../inc/CortexM.h"         // Cortex M specific functions
#include "../inc/PWM.h"             // PWM signal control
#include "../inc/LaunchPad.h"       // MSP432 LaunchPad functionality
#include "../inc/UART0.h"           // UART for data transmission
#include "../inc/Motor.h"           // Motor control
#include "../inc/Bump.h"            // Bump sensors
#include "../inc/TimerA2.h"         // Timer A2
#include "../inc/Nokia5110.h"       // Nokia LCD
#include "../inc/LPF.h"             // Low-pass filter
#include "../inc/Tachometer.h"      // Tachometer for motor feedback
#include <math.h>

// This macro limits a value within a specified minimum and maximum range.
// MINMAX ensures that the value of X stays within the bounds [Min, Max].
// If X is less than Min, it’s set to Min. If X is greater than Max, it’s set to Max.
// This helps prevent overflows or invalid values that could interfere with system stability.
// Usage example:
//      x = MINMAX(0, 100, x);  // This limits x to be between 0 and 100.
// Parameters:
// - Min: The minimum allowable value for X.
// - Max: The maximum allowable value for X.
// - X: The variable being constrained.
#define MINMAX(Min, Max, X) ((X) < (Min) ? (Min) : ((X) > (Max) ? (Max) : (X)))

// Constants defining PWM limits
#define PWMMIN 0        // Minimum PWM value
#define PWMMAX 999      // Maximum PWM value

// This divider scales down the controller output by dividing it inside the Controller.
// Specifically, (Kp * error) / GAIN_DIVIDER is used to compute the controller output.
// This allows us to select real (non-integer) values for Kp, such as 1.01, 2.43, and 3.75,
// which increases flexibility in tuning Kp.
// Without this scaling, we would be restricted to integer values like 1, 2, and 3.
// Note: Using a floating-point type for Kp would slow down computations significantly—over 1000 times.
// By using integers with a divider, we maintain computational efficiency while achieving precise tuning.
#define GAIN_DIVIDER    1000

// Proportional-Integral (PI) Controller Gains
// The values of Kp and Ki should be determined experimentally during tuning.
// Important: This Ki value differs from the Ki seen in lecture notes because it’s normalized:
// Ki(normalized) = Ki * DeltaT / 2, where DeltaT is the time step.
// A tuning tip: Start with Kp set to 0 and adjust Ki to help reduce any steady-state error.

// Select gain values based on experimental tuning results.
// static uint32_t Kp = 1;  // This corresponds to an actual Kp of 0.001 due to division by GainDivider.
// static uint32_t Kp = 91; // Here, Kp is effectively 0.091 (91/1000).

static int32_t Kp = 370;    // Proportional gain, scaled by GAIN_DIVIDER
static int32_t Ki = 135;  // Integral gain, scaled by GAIN_DIVIDER

// Target speed for the motors, in revolutions per minute (rpm)
static uint16_t DESIRED_FRWD_RPM = 70;    // 80 rpm
static uint16_t DESIRED_BKWD_RPM = 80;    // 80 rpm

// =============== IMPORTANT NOTE =====================================
// Use the 'static' storage class specifier to restrict the scope to this file.
// Without a storage class specifier, objects have external linkage ('extern').
// ====================================================================

// Flags and counters for system state tracking
static bool IsControllerEnabled = false; // Do not run the controller if this semaphore is false.

// The number of times the controller has executed.
// Display some information on the LCD when this value reaches a certain threshold.
static uint8_t NumControllerExecuted = 0;   // Counts controller executions for display updates

// Buffers for storing recent speed and PWM values for performance analysis
#define TACHBUFF_SIZE 10  // Tachometer data buffer size for speed averaging
#define BUFFER_SIZE 1000  // Storage size for analysis data over a 20-second period
static uint16_t LeftSpeedBuffer[BUFFER_SIZE];
static uint16_t RightSpeedBuffer[BUFFER_SIZE];
static uint16_t LeftDutyBuffer[BUFFER_SIZE];
static uint16_t RightDutyBuffer[BUFFER_SIZE];
static uint16_t BufferIndex = 0;  // Current position in buffer arrays

// Speed-related and error-tracking variables for the PI controller
static uint16_t LeftSpeed_rpm = 0;
static uint16_t RightSpeed_rpm = 0;
static int32_t ErrorL = 0;  // Current left motor error (target - measured speed)
static int32_t ErrorR = 0;  // Current right motor error (target - measured speed)
static int32_t AccumSpeedErrorL = 0;  // Accumulated error for integral control, left motor
static int32_t AccumSpeedErrorR = 0;  // Accumulated error for integral control, right motor

uint16_t LeftPeriod;        // Timer period for the left motor
uint16_t RightPeriod;       // Timer period for the right motor
int16_t  LeftDistance_mm;   // Distance traveled by the left motor (in mm)
int16_t  RightDistance_mm;  // Distance traveled by the right motor (in mm)
uint16_t SpeedBufferL[BUFFER_SIZE];     // Buffer to store left motor speed data
uint16_t SpeedBufferR[BUFFER_SIZE];     // Buffer to store right motor speed data
uint16_t DistanceBufferL[BUFFER_SIZE];  // Buffer to store left motor distance data
uint16_t DistanceBufferR[BUFFER_SIZE];  // Buffer to store right motor distance data
uint16_t DutyBuffer[BUFFER_SIZE];       // Buffer to store duty cycle values

int16_t DistanceSnapshots_x[BUFFER_SIZE];       // Buffer to store duty cycle values
int16_t DistanceSnapshots_y[BUFFER_SIZE];       // Buffer to store duty cycle values

uint16_t snapshotIndex = 0;

int16_t y_North = 0;
int16_t y_South = 0;
int16_t x_East = 0;
int16_t x_West = 0;

int16_t North_Holder = 0;
int16_t South_Holder = 0;
int16_t East_Holder = 0;
int16_t West_Holder = 0;

int16_t x_distance = 0;
int16_t y_distance = 0;
int16_t total_distance = 0;


// ------------ average ------------
// Calculates and returns the average value of an array of unsigned integers
// Used to smooth tachometer data for speed calculation
static uint16_t average(uint16_t *data, int data_length) {
    uint32_t sum = 0;
    for (int i = 0; i < data_length; i++) {
        sum += data[i];
    }
    return sum/data_length;
}


// Control parameters for various states (distances and duty cycles)
#define FRWD_DIST       700   // Replace this line for forward distance
#define BKWD_DIST       90    // Replace this line for backward distance
#define TR90_DIST       112   // Replace this line for 90 degree turn
#define TR60_DIST       74    // Replace this line for 60 degree turn
#define TR30_DIST       37    // Replace this line for 30 degree turn
#define TR15_DIST       19    // Replace this line for 15 degree turn

#define NUM_STATES      5     // Number of robot states
#define LEN_STR_STATE   5     // String length for state names
static char strState[NUM_STATES][LEN_STR_STATE] = {"STOP", "FRWD", "BKWD", "LFTR", "RGTR"};  // State names

typedef enum {
    Stop = 0,      // Stop state
    Forward = 1,       // Move forward
    Backward = 2,      // Move backward
    LeftTurn = 3,      // Turn left
    RightTurn = 4      // Turn right
} state_t;

static state_t CurrentState = Forward;  // Initial state: Forward
static state_t NextState = Forward;     // Next state variable

typedef enum {
    North = 0,
    East = 1,
    South = 2,
    West = 3
} compass_t;

static compass_t Compass = North;

typedef struct command {
    uint16_t left_permil;     // PWM duty cycle for the left wheel
    uint16_t right_permil;    // PWM duty cycle for the right wheel
    void (*MotorFunction)(uint16_t, uint16_t);  // Motor function to call (e.g., Motor_Forward)
    int32_t dist_mm;          // Wheel displacement in mm
} command_t;

command_t ControlCommands[NUM_STATES] = {
    {0,   0,   &Motor_Stop,        0},          // Stop indefinitely
    {100, 100, &Motor_Forward,     0},  // Move forward until bump sensor triggered
    {250, 250, &Motor_Backward,    60},  // Move backward for 90mm
    {100, 100, &Motor_TurnLeft,    0},          // Turn left
    {100, 100, &Motor_TurnRight,   0}           // Turn right
};


// Function to initialize and clear the LCD display with setup data
// Displays target speeds and initializes the user interface
static void LCDClear(void){

    // Contrast value 0xB1 looks good on red SparkFun.
    // Adjust this from 0xA0 (lighter) to 0xCF (darker) if necessary.
    uint8_t const contrast = 0xB1;
    Nokia5110_SetContrast(contrast);

    Nokia5110_Clear();  // erase entire display
    Nokia5110_SetCursor2(1, 1);
    Nokia5110_OutString("RPM  L   R  ");
    Nokia5110_SetCursor2(2, 1); Nokia5110_OutString("Ref");
    Nokia5110_SetCursor2(2, 5); Nokia5110_OutUDec(DESIRED_FRWD_RPM, 4);
    Nokia5110_SetCursor2(2, 9); Nokia5110_OutUDec(DESIRED_FRWD_RPM, 4);
    Nokia5110_SetCursor2(4, 1); Nokia5110_OutString("CRD  X   Y  ");
}

// Updates LCD display with current motor speeds, control errors, and PWM duties
static void LCDOut(void){
    if (CurrentState == Forward) {
        Nokia5110_SetCursor2(3, 5); Nokia5110_OutUDec(LeftSpeed_rpm, 4);
        Nokia5110_SetCursor2(3, 9); Nokia5110_OutUDec(RightSpeed_rpm, 4);
    }
    else {
        Nokia5110_SetCursor2(3, 5); Nokia5110_OutUDec(ControlCommands[CurrentState].left_permil, 4);
        Nokia5110_SetCursor2(3, 9); Nokia5110_OutUDec(ControlCommands[CurrentState].right_permil, 4);
    }
    Nokia5110_SetCursor2(5, 1); Nokia5110_OutSDec(x_distance, 6);
    Nokia5110_SetCursor2(5, 7); Nokia5110_OutSDec(y_distance, 6);
    if (snapshotIndex == 0) {
        Nokia5110_SetCursor2(6, 1); Nokia5110_OutSDec(DistanceSnapshots_x[snapshotIndex], 6);
        Nokia5110_SetCursor2(6, 7); Nokia5110_OutSDec(DistanceSnapshots_x[snapshotIndex], 6);
    }
    else {
        Nokia5110_SetCursor2(6, 1); Nokia5110_OutSDec(DistanceSnapshots_x[snapshotIndex-1], 6);
        Nokia5110_SetCursor2(6, 7); Nokia5110_OutSDec(DistanceSnapshots_y[snapshotIndex-1], 6);
    }
}


//**********************************************************
// Proportional-Integral (PI) controller for motor speed control
// Utilizes tachometer data to adjust motor speed.
static void Controller(void){
    static uint8_t idxTachoData = 0;             // Index for TachoPeriod arrays.
    static uint16_t LeftTachoPeriod[TACHBUFF_SIZE]; // Array to store left tachometer periods.
    static uint16_t RightTachoPeriod[TACHBUFF_SIZE]; // Array to store right tachometer periods.

    static uint16_t timer_20ms = 0;   // Timer to track elapsed time
    static uint8_t bumpRead = 0x00;   // Stores the bump sensor reading


    static uint16_t count = 0;

    float averageDistance_y = 0;
    float averageDistance_x = 360.0;


    NumControllerExecuted++;  // Increment counter every time the controller runs

    // State transition logic based on bump sensors and distance
    bumpRead = Bump_Read();  // Read bump sensor status
    if (CurrentState != LeftTurn || RightTurn) {
        Tachometer_GetDistances(&LeftDistance_mm, &RightDistance_mm);  // Get current wheel distances
        averageDistance_y = (LeftDistance_mm + RightDistance_mm)/2;
    }
    if (LeftDistance_mm < 0) {
        LeftDistance_mm = -LeftDistance_mm; // Make sure distance reading is positive when left motor is going backwards
    }
    if (RightDistance_mm < 0) {
        RightDistance_mm = -RightDistance_mm; // Make sure distance reading is positive when right motor is going backwards
    }





    total_distance = sqrt(pow(x_distance - 0.0,2) + pow(y_distance - 0.0,2));

    // If controller is disabled, exit the function.
    if(!IsControllerEnabled) {
        return;
    }

    if (CurrentState == Forward) {


        // If program reaches here, controller is enabled.
        // Proceed with speed control.

        // ====================================================================
        // Complete the rest for lab17
        // ====================================================================

        // Obtain tachometer data and store in arrays.
        Tachometer_GetSpeeds(&LeftTachoPeriod[idxTachoData], &RightTachoPeriod[idxTachoData]);
        // Update index in circular buffer.
        idxTachoData = (idxTachoData + 1) % TACHBUFF_SIZE;
        // Step 1: Calculate average tachometer period over last ten values.
        uint16_t leftPeriod = average(LeftTachoPeriod, TACHBUFF_SIZE);
        uint16_t rightPeriod = average(RightTachoPeriod, TACHBUFF_SIZE);

        // Step 2: Calculate actual motor speed (RPM) using tachometer data.
        LeftSpeed_rpm = PULSE2RPM / leftPeriod;
        RightSpeed_rpm = PULSE2RPM / rightPeriod;

        // Step 3: Calculate speed errors by comparing to desired speed.
        ErrorL = DESIRED_FRWD_RPM - LeftSpeed_rpm;
        ErrorR = DESIRED_FRWD_RPM - RightSpeed_rpm;

        // Step 4: Accumulate errors for integral control.
        AccumSpeedErrorL += ErrorL;
        AccumSpeedErrorR += ErrorR;

        // Step 5: Calculate control outputs (duty cycles) using PI control formula.
        uint16_t left_permil = (Kp*ErrorL+Ki*AccumSpeedErrorL) / GAIN_DIVIDER;
        uint16_t right_permil = (Kp*ErrorR+Ki*AccumSpeedErrorR) / GAIN_DIVIDER;

        // Step 6: Ensure duty cycles are within predefined bounds, MINMAX.
        left_permil = MINMAX(PWMMIN, PWMMAX, left_permil);
        right_permil =  MINMAX(PWMMIN, PWMMAX, right_permil);


        // ====================================================================
        // Do not modify anything below this line
        // ====================================================================

        // Step 7: Update motor speed using calculated duty cycles.
        Motor_Forward(left_permil, right_permil);

}

    else {

        AccumSpeedErrorL = 0;   // Reset accumulated errors
        AccumSpeedErrorR = 0;

        // Get the PWM duty cycles for the current state
        uint16_t left_permil = ControlCommands[CurrentState].left_permil;
        uint16_t right_permil = ControlCommands[CurrentState].right_permil;

        // FSM Output: Execute the motor command for the current state
        // Write your code here
        ControlCommands[CurrentState].MotorFunction(left_permil, right_permil);
    }
    switch (CurrentState) {
        case Stop:  // Remain in Stop state indefinitely}
                break;

        case Forward:  // Moving forward

            // Write your code here
            if (bumpRead == 0x04 || bumpRead == 0x08) { // If middle buttons pressed, go to backwards state
                count++;
                count++;
                NextState = Backward;
            }
            else if (bumpRead == 0x01 || bumpRead == 0x02) { // If button 1 or 2 pressed, turn left and set distance arc length to be for a 30 degree turn
                NextState = LeftTurn;
                ControlCommands[NextState].dist_mm = TR15_DIST;
            }
            else if (bumpRead == 0x02) {
                NextState = LeftTurn;
                ControlCommands[NextState].dist_mm = TR30_DIST;
            }
            else if (bumpRead == 0x10) { // If button 5 pressed, turn right and set distance arc length to be for a 60 degree turn
                NextState = RightTurn;
                ControlCommands[NextState].dist_mm = TR15_DIST;
            }
            else if (bumpRead == 0x20) {
                NextState = RightTurn;
                ControlCommands[NextState].dist_mm = TR30_DIST;
            }
            else if (total_distance <= 150 && total_distance >= 1) { // If the distance of left motor is greater than or equal to max forward distance, stop
                NextState = Stop;
            }
            else {
                NextState = Forward;
            }
            break;

        case Backward:  // Moving backward

            // Write your code here
            if (LeftDistance_mm >= ControlCommands[CurrentState].dist_mm) { // If backwards distance is greater than or equal to max backwards distance, turn left
                if (count == 2 || count == 6) {
                    if(x_distance > 0) {
                        NextState = LeftTurn;
                        ControlCommands[NextState].dist_mm = TR90_DIST; // Set the next state to have a left turn arc length for a 90 degree turn
                    }
                    else {
                        NextState = RightTurn;
                        ControlCommands[NextState].dist_mm = TR90_DIST; // Set the next state to have a left turn arc length for a 90 degree turn
                    }
                }
                else if(count == 0 || count == 4) {
                    if(x_distance < 0) {
                        NextState = LeftTurn;
                        ControlCommands[NextState].dist_mm = TR90_DIST; // Set the next state to have a left turn arc length for a 90 degree turn
                    }
                    else {
                        NextState = RightTurn;
                        ControlCommands[NextState].dist_mm = TR90_DIST; // Set the next state to have a left turn arc length for a 90 degree turn
                    }
                }
                else if(count == 1 || count == 5) {
                    if(y_distance < 0) {
                        NextState = LeftTurn;
                        ControlCommands[NextState].dist_mm = TR90_DIST; // Set the next state to have a left turn arc length for a 90 degree turn
                    }
                    else {
                        NextState = RightTurn;
                        ControlCommands[NextState].dist_mm = TR90_DIST; // Set the next state to have a left turn arc length for a 90 degree turn
                    }
                }
                else {
                    if(y_distance > 0) {
                        NextState = LeftTurn;
                        ControlCommands[NextState].dist_mm = TR90_DIST; // Set the next state to have a left turn arc length for a 90 degree turn
                    }
                    else {
                        NextState = RightTurn;
                        ControlCommands[NextState].dist_mm = TR90_DIST; // Set the next state to have a left turn arc length for a 90 degree turn
                    }
                }
                NextState = LeftTurn;
                ControlCommands[NextState].dist_mm = TR90_DIST; // Set the next state to have a left turn arc length for a 90 degree turn
                snapshotIndex++;
                count--;
            }
            break;

        case LeftTurn:  // Turning left

            // Write your code here
            if (LeftDistance_mm >= ControlCommands[CurrentState].dist_mm) {  // Finished left turn
                NextState = Forward;  // Move forward again
            }
            break;

        case RightTurn:  // Turning right
            if (LeftDistance_mm >= ControlCommands[CurrentState].dist_mm) {  // Finished right turn
                NextState = Forward;  // Move forward again
            }
            break;

        default:
            break;
        }


    // Update the timer or reset if transitioning to a new state
    if (CurrentState == NextState) {
        timer_20ms++;  // Stay in current state, increment timer
    } else {
        LeftDistance_mm = 0;
        RightDistance_mm = 0;
        averageDistance_y = 0;
        North_Holder = y_North;
        South_Holder = y_South;
        East_Holder = x_East;
        West_Holder = x_West;
        DistanceSnapshots_x[snapshotIndex] = x_distance;
        DistanceSnapshots_y[snapshotIndex] = y_distance;
        snapshotIndex++;
        timer_20ms = 0;  // New state, reset timer and distance measurements
        Tachometer_ResetSteps();
    }

    if(count == 4) {
       count = 0;
    }
    else if(count==5) {
        count = 1;
    }
    if (count == 0) {
        Compass = North;
    }
    else if (count == 1) {
        Compass = West;
    }
    else if(count == 2) {
        Compass = South;
    }
    else {
        Compass = East;
    }

    if (Compass == North) {
        y_North = averageDistance_y + North_Holder;
    }
    else if (Compass == South) {
        y_South = averageDistance_y + South_Holder;
    }
    else if (Compass == West) {
        x_West = averageDistance_y + West_Holder;
    }
    else {
        x_East = averageDistance_y + East_Holder;
    }
   y_distance = y_North - y_South;
   x_distance = (x_East+averageDistance_x)-x_West;

    // Set the current state to the next state for the next iteration
    CurrentState = NextState;
}



// Main program to initialize peripherals, control loop, and data transmission.
// Runs a controller at 50 Hz to adjust motor speeds and periodically updates an LCD display.
void Level1(void) {

    DisableInterrupts();             // Disable interrupts during initialization
    Clock_Init48MHz();               // Set the system clock to 48 MHz
    LaunchPad_Init();                // Initialize LaunchPad buttons and LEDs
    Bump_Init();                     // Initialize bump sensors
    Motor_Init();                    // Initialize motor controls
    Nokia5110_Init();                // Initialize the Nokia LCD

    LCDClear();                      // Clear the LCD screen

    Tachometer_Init(); // Initialize tachometers to measure wheel distances with set priority

    snapshotIndex = 0;
    uint32_t const baudrate = 115200; // Set UART baud rate for PC communication
    UART0_Init(baudrate);             // Initialize UART0 communication with set baud rate

    // Use TimerA2 to run the controller at 50 Hz (every 20ms)
    uint16_t const period_4us = 5000;       // Timer period to achieve 20ms (5000 x 4us)
    TimerA2_Init(&Controller, period_4us);  // Initialize TimerA2 for controller

    // Variables to control display update rate
    uint16_t const LcdUpdateRate = 5; // LCD updates every 5 controller cycles (every 100ms)

    BufferIndex = 0;                  // Initialize data buffer index
    IsControllerEnabled = false;      // Initially disable the controller
    Tachometer_ResetSteps();          // Reset tachometer step counters

    EnableInterrupts();               // Enable interrupts to start program operation

    // Main program loop
    while(1) {

        // Enter low-power mode, waiting for interrupts
        WaitForInterrupt();

        // Update the LCD display every 10 Hz (5 controller runs)
        // Note: Avoid adding LCDOut inside the ISR since Nokia5110 is a slow device.
        if (LcdUpdateRate == 0) {
            LCDOut();
        }
        if (NumControllerExecuted == LcdUpdateRate) {
            LCDOut();                  // Call function to output data on the LCD
            NumControllerExecuted = 0; // Reset count after LCD update
        }
        // Skip the following code if the controller is enabled, returning to low-power mode
        if (IsControllerEnabled) {
            continue;
        }

        // *************************************************************
        // If the program reaches this point, the controller is disabled.
        // *************************************************************

        LaunchPad_RGB(RGB_OFF); // Turn off RGB LED on LaunchPad
        Motor_Coast();          // Set motors to coast mode (stop gradually)
        Clock_Delay1ms(300);    // Delay to stabilize


        BufferIndex = 0;          // Reset buffer index for new data collection
        AccumSpeedErrorL = 0;     // Reset accumulated speed error (left wheel)
        AccumSpeedErrorR = 0;     // Reset accumulated speed error (right wheel)

        // Enable the controller for active speed control
        IsControllerEnabled = true;

    }
}


