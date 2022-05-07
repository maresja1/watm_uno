#include "Thermoino.h"

struct ConfigMenuItem bufferMenuItem;

const void *menuHandlerVent(__attribute__((unused)) const void *param, int8_t diff);
const void *menuHandlerHeating(__attribute__((unused)) const void *param, int8_t diff);
const void *menuHandlerBoiler(__attribute__((unused)) const void *param, int8_t diff);
const void *menuHandlerRoom(__attribute__((unused)) const void *param, int8_t diff);
const void *menuHandlerRoomTempAdjust(__attribute__((unused)) const void *param, int8_t diff);
const void *menuHandlerCircuitRelayForced(__attribute__((unused)) const void *param, int8_t diff);
const void *menuHandlerServoMin(__attribute__((unused)) const void *param, int8_t diff);
const void *menuHandlerServoMax(__attribute__((unused)) const void *param, int8_t diff);
const void *menuHandlerOverheatingLimit(__attribute__((unused)) const void *param, int8_t diff);
const void *menuHandlerUnderheatingLimit(__attribute__((unused)) const void *param, int8_t diff);
const void *menuHandlerDeltaTempPoly0(__attribute__((unused)) const void *param, int8_t diff);
const void *menuHandlerDeltaTempPoly1(__attribute__((unused)) const void *param, int8_t diff);
void *handlePIDValueConfig(float *floatVal, int8_t diff);
const void *menuHandlerPID_p(__attribute__((unused)) const void *param, int8_t diff);
const void *menuHandlerPID_i(__attribute__((unused)) const void *param, int8_t diff);
const void *menuHandlerPID_d(__attribute__((unused)) const void *param, int8_t diff);
const void *menuHandlerRelayPID_p(__attribute__((unused)) const void *param, int8_t diff);
const void *menuHandlerRelayPID_i(__attribute__((unused)) const void *param, int8_t diff);
const void *menuHandlerRelayPID_d(__attribute__((unused)) const void *param, int8_t diff);

void menuFormatterUInt8Value(__attribute__((unused)) const void *param, Print &print, const void *value);
void menuFormatterInt16Value(__attribute__((unused)) const void *param, Print &print, const void *value);
void menuFormatterFloatValue(__attribute__((unused)) const void *param, Print &print, const void *value);
void menuFormatterCircuitOverride(__attribute__((unused)) const void *param, Print &print, const void *value);

const char menuManual[] PROGMEM = "Manual %";
const char menuManualHeating[] PROGMEM = "Manual H %";
const char menuBoilerTemp[] PROGMEM = "Boiler \xDF";
const char menuRoomTemp[] PROGMEM = "Room \xDF";
const char menuCircuitRelay[] PROGMEM = "Circuit Relay";
const char menuRoomTempAdj[] PROGMEM = "[E] Room Temp adj.";
const char menuServoMin[] PROGMEM = "[E] Servo Min";
const char menuServoMax[] PROGMEM = "[E] Servo Max";
const char menuOverheating[] PROGMEM = "[E] Overheating\xDF";
const char menuUnderheating[] PROGMEM = "[E] Underheating\xDF";
const char menuDeltaTp1[] PROGMEM = "[E] deltaT p1";
const char menuDeltaTp0[] PROGMEM = "[E] deltaT p0";
const char menuPIDp[] PROGMEM = "[E] BoilPID K_p";
const char menuPIDi[] PROGMEM = "[E] BoilPID K_i";
const char menuPIDd[] PROGMEM = "[E] BoilPID K_d";
const char menuRelayPIDp[] PROGMEM = "[E] RelPID K_p";
const char menuRelayPIDi[] PROGMEM = "[E] RelPID K_i";
const char menuRelayPIDd[] PROGMEM = "[E] RelPID K_d";

const ConfigMenuItem_t menu[] = {
        {
                .name = menuManual,
                .param = nullptr,
                .handler = &menuHandlerVent,
                .formatter = &menuFormatterUInt8Value
        },
        {
                .name = menuManualHeating,
                .param = nullptr,
                .handler = &menuHandlerHeating,
                .formatter = &menuFormatterFloatValue
        },
        {
                .name = menuBoilerTemp,
                .param = nullptr,
                .handler = &menuHandlerBoiler,
                .formatter = &menuFormatterUInt8Value
        },
        {
                .name = menuRoomTemp,
                .param = nullptr,
                .handler = &menuHandlerRoom,
                .formatter = &menuFormatterFloatValue
        },
        {
                .name = menuCircuitRelay,
                .param = nullptr,
                .handler = &menuHandlerCircuitRelayForced,
                .formatter = &menuFormatterCircuitOverride
        },
        {
                .name = menuRoomTempAdj,
                .param = nullptr,
                .handler = &menuHandlerRoomTempAdjust,
                .formatter = &menuFormatterFloatValue
        },
        {
                .name = menuServoMin,
                .param = nullptr,
                .handler = &menuHandlerServoMin,
                .formatter = &menuFormatterInt16Value
        },
        {
                .name = menuServoMax,
                .param = nullptr,
                .handler = &menuHandlerServoMax,
                .formatter = &menuFormatterInt16Value
        },
        {
                .name = menuOverheating,
                .param = nullptr,
                .handler = &menuHandlerOverheatingLimit,
                .formatter = &menuFormatterUInt8Value
        },
        {
                .name = menuUnderheating,
                .param = nullptr,
                .handler = &menuHandlerUnderheatingLimit,
                .formatter = &menuFormatterUInt8Value
        },
        {
                .name = menuDeltaTp1,
                .param = nullptr,
                .handler = &menuHandlerDeltaTempPoly1,
                .formatter = &menuFormatterFloatValue
        },
        {
                .name = menuDeltaTp0,
                .param = nullptr,
                .handler = &menuHandlerDeltaTempPoly0,
                .formatter = &menuFormatterFloatValue
        },
        {
                .name = menuPIDp,
                .param = nullptr,
                .handler = &menuHandlerPID_p,
                .formatter = &menuFormatterFloatValue
        },
        {
                .name = menuPIDi,
                .param = nullptr,
                .handler = &menuHandlerPID_i,
                .formatter = &menuFormatterFloatValue
        },
        {
                .name = menuPIDd,
                .param = nullptr,
                .handler = &menuHandlerPID_d,
                .formatter = &menuFormatterFloatValue
        },
        {
                .name = menuRelayPIDp,
                .param = nullptr,
                .handler = &menuHandlerRelayPID_p,
                .formatter = &menuFormatterFloatValue
        },
        {
                .name = menuRelayPIDi,
                .param = nullptr,
                .handler = &menuHandlerRelayPID_i,
                .formatter = &menuFormatterFloatValue
        },
        {
                .name = menuRelayPIDd,
                .param = nullptr,
                .handler = &menuHandlerRelayPID_d,
                .formatter = &menuFormatterFloatValue
        }
};

const struct ConfigMenuItem *getMenu(int16_t itemIndex)
{
    const ConfigMenuItem_t *pItem = &menu[itemIndex];
    bufferMenuItem.param = pItem->param;
    bufferMenuItem.formatter = pItem->formatter;
    bufferMenuItem.handler = pItem->handler;
    strncpy_P(buffer, pItem->name, MAX_BUFFER_LEN);
    bufferMenuItem.name = buffer;
    return &bufferMenuItem;
}

const void *menuHandlerVent(__attribute__((unused)) const void *param, int8_t diff)
{
    angle += diff * 5;
    if (angle < 0 || angle > 200 /* overflow */) {
        angle = 0;
    }
    if (angle >= 100) {
        angle = 99;
    }
    if (diff != 0 && angle != 99 && angle % 5 != 0) {
        angle = (angle / 5) * 5;
    }
    currAngle = angle;
    return &angle;
}

const void *menuHandlerHeating(__attribute__((unused)) const void *param, int8_t diff)
{
    float pidRelayOut = pidHeatPWM.getConstrainedValue();
    pidRelayOut += diff;
    if (pidRelayOut < 0.0f) {
        pidRelayOut = 0.0f;
    }
    if (pidRelayOut  > 10.0f) {
        pidRelayOut = 10.0f;
    }
    pidHeatPWM.setValue(pidRelayOut);
    return pidHeatPWM.valPtr();
}

const void *menuHandlerBoiler(__attribute__((unused)) const void *param, int8_t diff)
{
    config.refTempBoiler += diff;
    return &config.refTempBoiler;
}

const void *menuHandlerRoom(__attribute__((unused)) const void *param, int8_t diff)
{
    config.refTempRoom += float(diff) * 0.2f;
    return &config.refTempRoom;
}

const void *menuHandlerRoomTempAdjust(__attribute__((unused)) const void *param, int8_t diff)
{
    config.roomTempAdjust += float(diff) * 0.1f;
    return &config.roomTempAdjust;
}

const void *menuHandlerCircuitRelayForced(__attribute__((unused)) const void *param, int8_t diff)
{
    if (diff != 0) {
        config.circuitRelayForced = (config.circuitRelayForced + 1) % 3;
    }
    return &config.circuitRelayForced;
}

const void *menuHandlerServoMin(__attribute__((unused)) const void *param, int8_t diff)
{
    config.servoMin += int16_t(diff);
    return &config.servoMin;
}

const void *menuHandlerServoMax(__attribute__((unused)) const void *param, int8_t diff)
{
    config.servoMax += int16_t(diff);
    return &config.servoMax;
}

const void *menuHandlerOverheatingLimit(__attribute__((unused)) const void *param, int8_t diff)
{
    config.overheatingLimit += diff;
    return &config.overheatingLimit;
}

const void *menuHandlerUnderheatingLimit(__attribute__((unused)) const void *param, int8_t diff)
{
    config.underheatingLimit += diff;
    return &config.underheatingLimit;
}

const void *menuHandlerDeltaTempPoly0(__attribute__((unused)) const void *param, int8_t diff)
{
    config.deltaTempPoly0 += float(diff) * 0.005f;
    return &config.deltaTempPoly0;
}

const void *menuHandlerDeltaTempPoly1(__attribute__((unused)) const void *param, int8_t diff)
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

const void *menuHandlerPID_p(__attribute__((unused)) const void *param, int8_t diff)
{
    return handlePIDValueConfig(&config.pidKp, diff);
}

const void *menuHandlerPID_i(__attribute__((unused)) const void *param, int8_t diff)
{
    return handlePIDValueConfig(&config.pidKi, diff);
}

const void *menuHandlerPID_d(__attribute__((unused)) const void *param, int8_t diff)
{
    return handlePIDValueConfig(&config.pidKd, diff);
}

const void *menuHandlerRelayPID_p(__attribute__((unused)) const void *param, int8_t diff)
{
    return handlePIDValueConfig(&config.pidRelayKp, diff);
}

const void *menuHandlerRelayPID_i(__attribute__((unused)) const void *param, int8_t diff)
{
    return handlePIDValueConfig(&config.pidRelayKi, diff);
}

const void *menuHandlerRelayPID_d(__attribute__((unused)) const void *param, int8_t diff)
{
    return handlePIDValueConfig(&config.pidRelayKd, diff);
}

void menuFormatterUInt8Value(__attribute__((unused)) const void *param, Print &print, const void *value)
{
    lcd.cursor();
    print.print("value: ");
    snprintf(buffer, MAX_BUFFER_LEN, "%8d", *(uint8_t *) value);
    print.print(buffer);
    buffer[0] = '\0';
}

void menuFormatterInt16Value(__attribute__((unused)) const void *param, Print &print, const void *value)
{
    lcd.cursor();
    print.print("value: ");
    snprintf(buffer, MAX_BUFFER_LEN, "%8d", *(int16_t *) value);
    print.print(buffer);
    buffer[0] = '\0';
}

//void menuFormatterInt8Value(__attribute__((unused)) const void *param, Print &print, const void *value)
//{
//    lcd.cursor();
//    print.print("value: ");
//    snprintf(buffer, MAX_BUFFER_LEN, "%8d", *(int8_t *) value);
//    print.print(buffer);
//    buffer[0] = '\0';
//}

void menuFormatterFloatValue(__attribute__((unused)) const void *param, Print &print, const void *value)
{
    lcd.cursor();
    print.print("value: ");
    double doubleValue = (double) *(float *) value;
    if (doubleValue >= 1.0f || doubleValue <= -1.0f) {
        snprintf(buffer, MAX_BUFFER_LEN, "%8.3f", doubleValue);
    } else {
        snprintf(buffer, MAX_BUFFER_LEN, "%8.4f", doubleValue);
    }
    print.print(buffer);
    buffer[0] = '\0';
}

void menuFormatterCircuitOverride(__attribute__((unused)) const void *param, Print &print, const void *value)
{
    switch (*(int8_t *) value) {
        case 0:
            print.print(F("no override"));
            break;
        case 1:
            print.print(F("always enabled"));
            break;
        case 2:
            print.print(F("always disabled"));
            break;
    }
}
