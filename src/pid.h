//
// Created by jan on 4/18/22.
//

#pragma once
#ifndef THERMOINO_PID_H
#define THERMOINO_PID_H

class ThermoinoPID {

public:
    // commonly used functions ************************************************************************************

    // Constructor. Links the PID to Input, Output, Setpoint, initial tuning parameters and control modes.
    explicit ThermoinoPID(uint32_t period);

    // Performs the PID calculation. It should be called every time loop() cycles ON/OFF and calculation frequency
    // can be set using SetMode and SetSampleTime respectively.
    float Compute(float input, float setPoint);

    // Sets and clamps the output to a specific range (0-255 by default).
    void SetOutputLimits(float Min, float Max);

    // available but not commonly used functions ******************************************************************

    // While most users will set the tunings once in the constructor, this function gives the user the option of
    // changing tunings during runtime for Adaptive control.
    void SetTunings(float Kp, float Ki, float Kd);

private:

    float error, lastError, outMin, outMax, lastOutput;

    float A0;           // (P)roportional gain - K_p
    float A1;           // (I)ntegral time - T_i
    float A2;           // (D)erivative time T_d

    uint32_t period;
}; // class ThermoinoPID
#endif //THERMOINO_PID_H
