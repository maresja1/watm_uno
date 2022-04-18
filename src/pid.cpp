//
// Created by jan on 4/18/22.
//

#include <Arduino.h>
#include "pid.h"


/* Constructor ********************************************************************
   The parameters specified here are those for for which we can't set up
   reliable defaults, so we need to have the user set them.
 **********************************************************************************/
ThermoinoPID::ThermoinoPID(uint32_t period)
{
    this->error = 0;
    this->lastError = 0;
    this->lastOutput = 0;
    this->period = period;

    this->A0 = this->A1 = this->A2 = 0;

    ThermoinoPID::SetOutputLimits(0, 100);
}

/* Compute() ***********************************************************************
   This function should be called every time "void loop()" executes. The function
   will decide whether a new PID Output needs to be computed. Returns true
   when the output is computed, false when nothing has been done.
 **********************************************************************************/
float ThermoinoPID::Compute(const float input, const float setPoint) {
    // taken from https://en.wikipedia.org/wiki/PID_controller#Discrete_implementation
//    A0 := Kp + Ki*dt + Kd/dt
//    A1 := -Kp - 2*Kd/dt
//    A2 := Kd/dt
//    error[2] := 0 // e(t-2)
//    error[1] := 0 // e(t-1)
//    error[0] := 0 // e(t)
//    output := u0  // Usually the current value of the actuator
//
//    loop:
//    error[2] := error[1]
//    error[1] := error[0]
//    error[0] := setpoint − measured_value
//    output := output + A0 * error[0] + A1 * error[1] + A2 * error[2]
//    wait(dt)
//    goto loop
    const float errorBeforeLast = lastError;
    lastError = error;
    error = setPoint - input;
    lastOutput += (A0 * error) + (A1 * lastError) + (A2 * errorBeforeLast);
    // consider limiting lastOutput to avoid windup
    return constrain(lastOutput, outMin, outMax);
}

/* SetTunings(....)************************************************************
  This function allows the controller's dynamic performance to be adjusted.
  it's called automatically from the constructor, but tunings can also
  be adjusted on the fly during normal operation.
******************************************************************************/
void ThermoinoPID::SetTunings(float Kp, float Ki, float Kd) {
    // taken from https://en.wikipedia.org/wiki/PID_controller#Discrete_implementation
//    A0 := Kp + Ki*dt + Kd/dt
//    A1 := -Kp - 2*Kd/dt
//    A2 := Kd/dt
    this->A0 = Kp + (Ki * period) + (Kd / period);
    this->A1 = -Kp - (2 * ( Kd / period));
    this->A2 = Kd / period;
}

void ThermoinoPID::SetOutputLimits(float Min, float Max) {
    if (Min >= Max) return;
    outMin = Min;
    outMax = Max;
}