//
// Created by jan on 4/18/22.
//

#include <Arduino.h>
#include "pid.h"

ThermoinoPID::ThermoinoPID(uint32_t period)
{
    this->error = 0;
    this->lastError = 0;
    this->lastOutput = 0;
    this->period = period;
    this->noIntOffset = __FLT_MAX__;

    this->A0 = this->A0_noint = this->A1 = this->A2 = 0;

    ThermoinoPID::setOutputLimits(0, 100);
}

void ThermoinoPID::compute(const float input, const float setPoint) {
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
    if (abs(error) < this->noIntOffset) {
        lastOutput += (A0 * error) + (A1 * lastError) + (A2 * errorBeforeLast);
    } else {
        lastOutput += (A0_noint * error) + (A1 * lastError) + (A2 * errorBeforeLast);
    }
}

float ThermoinoPID::getConstraintedValue() {
    // consider limiting lastOutput to avoid windup
    return constrain(lastOutput, outMin, outMax);
}

float *ThermoinoPID::valPtr() {
    // consider limiting lastOutput to avoid windup
    return &lastOutput;
}

void ThermoinoPID::setParams(float Kp, float Ki, float Kd) {
    // taken from https://en.wikipedia.org/wiki/PID_controller#Discrete_implementation
//    A0 := Kp + Ki*dt + Kd/dt
//    A1 := -Kp - 2*Kd/dt
//    A2 := Kd/dt
    this->A0_noint = Kp + (Kd / period);
    this->A0 = this->A0_noint + (Ki * period);
    this->A1 = -Kp - (2 * ( Kd / period));
    this->A2 = Kd / period;
}

void ThermoinoPID::setOutputLimits(float min, float max) {
    if (min >= max) return;
    outMin = min;
    outMax = max;
}

void ThermoinoPID::setIntegralMaxError(float offset) {
    this->noIntOffset = offset;
}