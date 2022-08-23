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

    float getConstrainedValue();

    void setValue(float val);

    void setOutputLimits(float min, float max);

    // if abs(error) higher than this, integral part is omitted to avoid windup
    void setIntegralLimit(float offset);

    void setParams(float Kp, float Ki, float Kd);

private:

    float error, outMin, outMax, integralLimit, lastOutput, lastInput, integral;

    float Kp;
    float Ti;
    float Td;

    uint32_t period;
}; // class ThermoinoPID
#endif //THERMOINO_PID_H
