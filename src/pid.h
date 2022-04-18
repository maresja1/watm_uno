//
// Created by jan on 4/18/22.
//

#pragma once
#ifndef THERMOINO_PID_H
#define THERMOINO_PID_H

class ThermoinoPID {

public:
    explicit ThermoinoPID(uint32_t period);

    void compute(float input, float setPoint);

    float getConstraintedValue();

    float *valPtr();

    void setOutputLimits(float min, float max);

    // if abs(error) higher than this, integral part is omitted to avoid windup
    void setIntegralMaxError(float offset);

    void setParams(float Kp, float Ki, float Kd);

private:

    float error, lastError, outMin, outMax, noIntOffset, lastOutput;

    float A0;
    // A0 without integral part - anti-windup
    float A0_noint;
    float A1;
    float A2;

    uint32_t period;
}; // class ThermoinoPID
#endif //THERMOINO_PID_H
