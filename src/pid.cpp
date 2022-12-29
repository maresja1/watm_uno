//
// Created by jan on 4/18/22.
//

#include <Arduino.h>
#include "pid.h"

#if 1
#define DEBUG_SER_PRINT(x) do { Serial.print(F(#x":")); Serial.print(x); Serial.print(F(",")); } while(0)
#define DEBUG_SER_PRINT_LN(x) do { Serial.print(F(#x":")); Serial.println(x); } while(0)
#else
#define DEBUG_SER_PRINT(x) do { Serial.print(x); Serial.print(F(",")); } while(0)
#define DEBUG_SER_PRINT_LN(x) do { Serial.println(x); } while(0)
#endif

ThermoinoPID::ThermoinoPID(uint32_t period)
{
    this->error = 0.0f;
    this->integral = 0.0f;
    this->period = period;
    this->integralLimit = __FLT_MAX__;
    this->lastInput = NAN;

    ThermoinoPID::setOutputLimits(0.0f, 100.0f);
}

void ThermoinoPID::compute(const float input, const float setPoint, Stream * const print) {
    if (isnan(this->lastInput)) {
        this->lastInput = input;
    }
    const float lastError = error;
    error = setPoint - input;


    float proportional = getProportional(error);
    // Trapezoidal Integration
    this->integral = this->integral + ((this->Kp * float(period) * (error + lastError)) / (2 * this->Ti));
    const float derivative = getDerivative(input);

    // anti-windup
    if ((proportional + this->integral) > (outMax + this->integralLimit)) {
        this->integral = (outMax + this->integralLimit) - proportional;
    } else if ((proportional + this->integral) < (outMin - this->integralLimit)) {
        this->integral = (outMin - this->integralLimit) - proportional;
    }

    if (print != nullptr) {
        print->print(proportional, 2);
        print->print(",");
        print->print(derivative, 2);
        print->print(",");
        print->print(this->integral, 2);
    }
    this->lastOutput = proportional + derivative;
    this->lastInput = input;
}

float ThermoinoPID::getDerivative(const float input) const
{
    return (-1.0f * Kp * Td * (input - lastInput)) / float(period);
}

float ThermoinoPID::getProportional(float error) const
{
    return Kp * error;
}

float ThermoinoPID::getConstrainedValue() {
    return constrain(lastOutput + this->integral, outMin, outMax);
}

void ThermoinoPID::setIntegralToValue(float val, const float input, const float setPoint)
{
    const float error = setPoint - input;
    if (!isnan(this->lastInput)) {
        this->integral = val - getProportional(error) - getDerivative(input);
    } else {
        this->integral = val - getProportional(error);
    }
    this->lastOutput = val;
}

void ThermoinoPID::setParams(float Kp, float Ti, float Td) {
    // rescale integral
    if (this->Kp == Kp && this->Ti == Ti && this->Td == Td) {
        return;
    }

    this->integral = (this->integral * this->Ti) / Ti;

    this->Kp = Kp;
    this->Ti = Ti;
    this->Td = Td;
}

void ThermoinoPID::setOutputLimits(float min, float max) {
    if (min >= max) return;
    outMin = min;
    outMax = max;
}

void ThermoinoPID::setIntegralLimit(float offset) {
    this->integralLimit = offset;
}