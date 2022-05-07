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

    this->A0_integral = this->A0_nonintegral = this->A1 = this->A2 = 0;

    ThermoinoPID::setOutputLimits(0.0f, 100.0f);
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
        lastOutput += (A0_nonintegral * error) + (A1 * lastError) + (A2 * errorBeforeLast);
        lastOutputIntegral += A0_integral * error;
    } else {
        lastOutput += (A0_nonintegral * error) + (A1 * lastError) + (A2 * errorBeforeLast);
        lastOutputIntegral = 0;
    }

    // avoid wind-up
    if (lastOutputIntegral > outMax) {
        lastOutputIntegral = outMax;
    }

    if (lastOutputIntegral < outMin) {
        lastOutputIntegral = outMin;
    }
}

float ThermoinoPID::getConstrainedValue() {
    // consider limiting lastOutput to avoid windup
    return constrain(lastOutput + lastOutputIntegral, outMin, outMax);
}

void ThermoinoPID::setValue(float val) {
    lastOutput = val;
    lastOutputIntegral = 0.0f;
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
    this->lastOutput = constrain(this->lastOutput, this->outMin, this->outMax);
//    const float Ki = Ti == 0.0f ? 0.0f : Kp / Ti;
//    const float Kd = Kp * Td;
    this->A0_nonintegral = Kp + (Kd / period);
    this->A0_integral = (Ki * period);
    this->A1 = -Kp - (2 * (Kd / period));
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