#pragma once
//
// Created by jan on 4/1/22.
//

#ifndef THERMOINO_H
#define THERMOINO_H

#define _TASK_INLINE

#include <TaskScheduler.h>

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

#if DEBUG_LEVEL > 0
#define DEBUG_TASK_ENTRY(x) do { Serial.print(millis()); Serial.print(F(" - "#x" - e")); Serial.println(""); } while(0)
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

#define MAX_BUFFER_LEN 20
extern char buffer[];
extern void ser_print_ll(uint64_t ll);

struct State {
    double volume;
    uint32_t volumePulses;
    float volumeFlow;
    uint32_t volumeFlowPulses;
};

struct Config {
    uint8_t Q_div;
    uint8_t Q_offset;
};

extern State state;
extern Config config;

extern Task t_stateUpdate_readSensors;
extern Task t_stateUpdate_serialReader;

typedef struct Button {
    uint8_t pin;
    uint8_t state;
    uint8_t pressedFor;
} Button_t;

void eepromInit();
void eepromUpdate();

void stateUpdate_serialReader_cb();
//void stateUpdate_hotWaterProbe_cb();
void stateUpdate_readSensors_cb();

void serialLineSetup();
void serialLineBufferLoop();
void serialPrintState();
void serialPrintConfig();

void notifyTask(Task *task, bool immediate);

#endif //THERMOINO_H
