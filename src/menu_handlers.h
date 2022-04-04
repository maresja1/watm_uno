#include "Thermoino.h"

void *menuHandlerVent(__attribute__((unused)) void *param, int8_t diff)
{
    angle += diff;
    if (angle < 0 || angle > 200 /* overflow */) {
        angle = 0;
    }
    if (angle >= 100) {
        angle = 99;
    }
//    if (diff != 0 && angle != 99 && angle % 5 != 0) {
//        angle = (angle / 5) * 5;
//    }
    currAngle = angle;
    return &angle;
}

void *menuHandlerHeating(__attribute__((unused)) void *param, int8_t diff)
{
    pidRelayOut += float(diff);
    if (pidRelayOut < 0) {
        pidRelayOut = 0.0f;
    }
    if (pidRelayOut  > 10) {
        pidRelayOut = 10.0f;
    }
    return &pidRelayOut;
}

void *menuHandlerBoiler(__attribute__((unused)) void *param, int8_t diff)
{
    config.refTempBoiler += diff;
    return &config.refTempBoiler;
}

void *menuHandlerBoilerIdle(__attribute__((unused)) void *param, int8_t diff)
{
    config.refTempBoilerIdle += diff;
    return &config.refTempBoilerIdle;
}

void *menuHandlerRoom(__attribute__((unused)) void *param, int8_t diff)
{
    config.refTempRoom += float(diff) * 0.2f;
    return &config.refTempRoom;
}

void *menuHandlerRoomTempAdjust(__attribute__((unused)) void *param, int8_t diff)
{
    config.roomTempAdjust += float(diff) * 0.1f;
    return &config.roomTempAdjust;
}

void *menuHandlerDebounceLimitC(__attribute__((unused)) void *param, int8_t diff)
{
    config.debounceLimitC += float(diff) * 0.1f;
    if (config.debounceLimitC <= -10) {
        config.debounceLimitC = 9.9;
    }
    if (config.debounceLimitC >= 10) {
        config.debounceLimitC = -9.9;
    }
    return &config.debounceLimitC;
}

//void *menuHandlerCurveItems(__attribute__((unused)) void *param, int8_t diff)
//{
//    config.curveItems += diff;
//    config.curveItems %= MAX_DELTA_SETTINGS;
//    if (config.curveItems < 0) {
//        config.curveItems = 0;
//    }
//    return &config.curveItems;
//}

//void *menuHandlerCurveItemX(void *param, int8_t diff)
//{
//    uintptr_t index = (uintptr_t) param;
//#if DEBUG_LEVEL > 2
//    Serial.print("menuHandlerCurveItemX - index: ");
//    Serial.print(index);
//    Serial.print(", value: ");
//    Serial.print(maxDeltaSettings[index]);
//    Serial.print(", diff: ");
//    Serial.print(diff);
//    Serial.println("");
//#endif
//    maxDeltaSettings[index] += diff;
//    return &maxDeltaSettings[index];
//}
//
//void *menuHandlerCurveItemY(void *param, int8_t diff)
//{
//    uintptr_t index = (uintptr_t) param;
//#if DEBUG_LEVEL > 2
//    Serial.print("menuHandlerCurveItemY - index: ");
//    Serial.print(index);
//    Serial.print(", value: ");
//    Serial.print(maxDeltaHigh[index]);
//    Serial.print(", diff: ");
//    Serial.print(diff);
//    Serial.println("");
//#endif
//    maxDeltaHigh[index] += diff;
//    return &maxDeltaHigh[index];
//}

void *menuHandlerCircuitRelayForced(__attribute__((unused)) void *param, int8_t diff)
{
    if (diff != 0) {
        config.circuitRelayForced = (config.circuitRelayForced + 1) % 3;
    }
    return &config.circuitRelayForced;
}

void *menuHandlerServoMin(__attribute__((unused)) void *param, int8_t diff)
{
    config.servoMin += int16_t(diff);
    return &config.servoMin;
}

void *menuHandlerServoMax(__attribute__((unused)) void *param, int8_t diff)
{
    config.servoMax += int16_t(diff);
    return &config.servoMax;
}

void *menuHandlerOverheatingLimit(__attribute__((unused)) void *param, int8_t diff)
{
    config.overheatingLimit += diff;
    return &config.overheatingLimit;
}

//void *menuHandlerUnderheatingLimit(__attribute__((unused)) void *param, int8_t diff)
//{
//    config.underheatingLimit += diff;
//    return &config.underheatingLimit;
//}

void *menuHandlerDeltaTempPoly0(__attribute__((unused)) void *param, int8_t diff)
{
    config.deltaTempPoly0 += float(diff) * 0.005f;
    return &config.deltaTempPoly0;
}

void *menuHandlerDeltaTempPoly1(__attribute__((unused)) void *param, int8_t diff)
{
    config.deltaTempPoly1 += float(diff) * 0.005f;
    return &config.deltaTempPoly1;
}

void *handlePIDValueConfig(float *floatVal, int8_t diff)
{
    *floatVal += float(diff) * (
            *floatVal > 5 ? 0.5f :
                (
                    *floatVal > 0.5f ? 0.05f : (*floatVal > 0.05f ? 0.005f : 0.0001f )
                )
            );
    if (*floatVal < 0.0f) {
        *floatVal = 0.0f;
    }
    return floatVal;
}

void *menuHandlerPID_p(__attribute__((unused)) void *param, int8_t diff)
{
    return handlePIDValueConfig(&config.pidKp, diff);
}

void *menuHandlerPID_i(__attribute__((unused)) void *param, int8_t diff)
{
    return handlePIDValueConfig(&config.pidKi, diff);
}

void *menuHandlerPID_d(__attribute__((unused)) void *param, int8_t diff)
{
    return handlePIDValueConfig(&config.pidKd, diff);
}

void *menuHandlerRelayPID_p(__attribute__((unused)) void *param, int8_t diff)
{
    return handlePIDValueConfig(&config.pidRelayKp, diff);
}

void *menuHandlerRelayPID_i(__attribute__((unused)) void *param, int8_t diff)
{
    return handlePIDValueConfig(&config.pidRelayKi, diff);
}

void *menuHandlerRelayPID_d(__attribute__((unused)) void *param, int8_t diff)
{
    return handlePIDValueConfig(&config.pidRelayKd, diff);
}