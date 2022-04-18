#pragma once
//
// Created by jan on 4/1/22.
//

#ifndef THERMOINO_H
#define THERMOINO_H

#define _TASK_INLINE

#include <TaskScheduler.h>
#include <LiquidCrystal_I2C.h>
#include "pid.h"

#define DEBUG_LEVEL 0
#define DT_BOILER_PIN 11
#define SERVO_PIN 10
#define CIRCUIT_RELAY_PIN 9
#define DHT21_PIN 8
#define BTN_1_PIN 6
#define BTN_2_PIN 7
#define BTN_3_PIN 4
#define BTN_4_PIN 5

#define USE_DHT_ROOM_TEMP 1
#define USE_DT_ROOM_BOILER 1

#if 0
#define DEBUG_TASK_ENTRY(x) do { Serial.print(F(#x" - e")); Serial.println(""); } while(0)
#define DEBUG_TASK_RET(x) do { Serial.print(F(#x" - r")); Serial.println(""); } while(0)
#else
#define DEBUG_TASK_ENTRY(x)
#define DEBUG_TASK_RET(x)
#endif

#define PRINT_SERIAL_UPDATES 1

#if 0
#define DEBUG_SER_PRINT(x) do { Serial.print(F(#x":")); Serial.print(x); Serial.print(F(",")); } while(0)
#define DEBUG_SER_PRINT_LN(x) do { Serial.print(F(#x":")); Serial.println(x); } while(0)
#else
#define DEBUG_SER_PRINT(x) do { Serial.print(x); Serial.print(F(",")); } while(0)
#define DEBUG_SER_PRINT_LN(x) do { Serial.println(x); } while(0)
#endif

#define MENU_POS_VENT_MANUAL 0
#define MENU_POS_HEAT_MANUAL 1
#define MENU_POS_SERVO_MIN 6
#define MENU_POS_SERVO_MAX 7

#define relay_or_override() config.circuitRelayForced == 0 ? circuitRelay : config.circuitRelayForced == 1

#define MAX_BUFFER_LEN 20
extern char buffer[];

typedef struct ConfigMenuItem {
    const char *name;
    void* param;
    void* (*handler)(void* param, int8_t diff);
    void (*formatter)(void* param, Print &print, void *value);
} ConfigMenuItem_t;

struct Configuration {
    uint8_t refTempBoiler;
    uint8_t refTempBoilerIdle;
    float refTempRoom;
    uint8_t circuitRelayForced;
    int16_t servoMin;
    int16_t servoMax;
    float debounceLimitC;
    uint8_t underheatingLimit;
    uint8_t overheatingLimit;
    // linear interpolation (least squares) of the following points:
    // [boilerTemp - roomTemp, real boilerTemp - boilerTemp]
    float deltaTempPoly1;
    float deltaTempPoly0;
    float roomTempAdjust;
    float pidKp;
    float pidKi;
    float pidKd;
    float pidRelayKp;
    float pidRelayKi;
    float pidRelayKd;
    int16_t settingsSelected;
};

extern Configuration config;
extern uint8_t angle;
extern uint8_t currAngle;
extern int16_t settingsSelectedPrint;
extern float boilerTemp;
extern float roomTemp;
extern float roomHumidity;
extern bool heatNeeded;
extern uint8_t heatNeededOverride; // 0 no override, 1 - override false, 2 or else - override true
extern bool overheating;
extern bool underheating;
extern bool circuitRelay;

extern Task t_stateUpdate_readButtons;
extern Task t_stateUpdate_angleAndRelay;
extern Task t_stateUpdate_readSensors;
//extern Task t_stateUpdate_hotWaterProbe;
extern Task t_stateUpdate_heatNeeded;
extern Task t_stateUpdate_serialReader;
extern Task t_effect_refreshServoAndRelay;
extern Task t_effect_printStatus;
extern Task t_effect_processSettings;

extern LiquidCrystal_I2C lcd;

extern ThermoinoPID pidBoiler;
extern ThermoinoPID pidRelay;

typedef struct Button {
    uint8_t pin;
    uint8_t state;
    uint8_t pressedFor;
} Button_t;

void eepromInit();
void eepromUpdate();
bool processSettings();
void printStatus();
void servoSetPos(int positionPercent);
void sendCurrentStateToRelay(bool state);
void screenSaverWakeup();
void notifySettingsChanged();

void stateUpdate_heatNeeded_cb();
void stateUpdate_simulator_cb();
void stateUpdate_serialReader_cb();
//void stateUpdate_hotWaterProbe_cb();
void stateUpdate_readSensors_cb();
void effect_refreshServoAndRelay_cb();
void effect_printStatus_cb();
void stateUpdate_angleAndRelay_cb();
void stateUpdate_readButtons_cb();
void effect_processSettings_cb();

void serialLineSetup();
void serialLineBufferLoop();
void serialPrintConfig();

void notifyTask(Task *task, bool immediate);

#include "menu.h"

#endif //THERMOINO_H
