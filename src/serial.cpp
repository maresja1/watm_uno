//
// Created by jan on 4/5/22.
//
#include <Arduino.h>

#include "Thermoino.h"


const uint8_t serialLineBufferCapacity = 40;
uint8_t serialLineBufferIdx = 0;
char serialLineBuffer[serialLineBufferCapacity];   // an array to store the received data
boolean serialLineBufferDataReady = false;

void serialLineBufferLoop() {
    if (Serial.available() > 0 && !serialLineBufferDataReady) {
        const char rc = (char) Serial.read();
        if (rc != '\r' && rc != '\n') {
            serialLineBuffer[serialLineBufferIdx++] = rc;
            if (serialLineBufferIdx >= serialLineBufferCapacity) {
                serialLineBufferIdx = 0; // overflow
            }
        } else {
            serialLineBuffer[serialLineBufferIdx] = '\0'; // terminate the string
            serialLineBufferIdx = 0;
            serialLineBufferDataReady = true;
        }
    }
}

void stateUpdate_serialReader_cb()
{
#if 0
    DEBUG_SER_PRINT(circuitTemp);
    DEBUG_SER_PRINT(roomTemp);
    DEBUG_SER_PRINT(boilerTemp);
    DEBUG_SER_PRINT(currAngle);
    DEBUG_SER_PRINT(heatNeeded);
    DEBUG_SER_PRINT_LN(pidRelayOut);
#endif

#define CMD_HNO "HNO"
#define CMD_VENT_SET "O"
#define CMD_BOILER_REF_TEMP_SET "BRT"
#define CMD_PID_BL_Kp "PID_BL_Kp"
#define CMD_PID_BL_Ki "PID_BL_Ki"
#define CMD_PID_BL_Kd "PID_BL_Kd"
#define CMD_PID_CR_Kp "PID_CR_Kp"
#define CMD_PID_CR_Ki "PID_CR_Ki"
#define CMD_PID_CR_Kd "PID_CR_Kd"
#define CMD_MODE "M"
#define literal_len(x) (sizeof(x) - 1)
#define PARSE(x) if (commandBuffer.startsWith(F(x ":"))) { const String &valueBuffer = commandBuffer.substring(literal_len(x ":"));
#define OR_PARSE(x) } else if (commandBuffer.startsWith(F(x ":"))) { const String &valueBuffer = commandBuffer.substring(literal_len(x ":"));

    if (serialLineBufferDataReady) {
        serialLineBufferDataReady = false;
        const String &sBuffer = String(serialLineBuffer);
        if (sBuffer.startsWith(F("DRQ:"))) {
            const String &commandBuffer = sBuffer.substring(4);
            PARSE(CMD_HNO)
                heatNeededOverride = strtol(valueBuffer.c_str(), nullptr, 10);
                notifySettingsChanged();
                screenSaverWakeup();
            OR_PARSE(CMD_VENT_SET)
                settingsSelected = MENU_POS_VENT_MANUAL;
                angle = strtol(valueBuffer.c_str(), nullptr, 10);
                notifySettingsChanged();
                screenSaverWakeup();
            OR_PARSE(CMD_BOILER_REF_TEMP_SET)
                config.refTempBoiler = strtol(valueBuffer.c_str(), nullptr, 10);
                notifySettingsChanged();
            OR_PARSE(CMD_PID_BL_Kp)
                config.pidKp = strtod(valueBuffer.c_str(), nullptr);
                notifySettingsChanged();
            OR_PARSE(CMD_PID_BL_Ki)
                config.pidKi = strtod(valueBuffer.c_str(), nullptr);
                notifySettingsChanged();
            OR_PARSE(CMD_PID_BL_Kd)
                config.pidKd = strtod(valueBuffer.c_str(), nullptr);
                notifySettingsChanged();
            OR_PARSE(CMD_PID_CR_Kp)
                config.pidRelayKp = strtod(valueBuffer.c_str(), nullptr);
                notifySettingsChanged();
            OR_PARSE(CMD_PID_CR_Ki)
                config.pidRelayKi = strtod(valueBuffer.c_str(), nullptr);
                notifySettingsChanged();
            OR_PARSE(CMD_PID_CR_Kd)
                config.pidRelayKd = strtod(valueBuffer.c_str(), nullptr);
                notifySettingsChanged();
            OR_PARSE(CMD_MODE)
                Serial.print("XX");
                Serial.println(sBuffer);
                if (valueBuffer.equals("A")) {
                    settingsSelected = -1;
                    heatNeededOverride = 0;
                    notifySettingsChanged();
                    screenSaverWakeup();
                }
            } else {
                Serial.print(F("Unknown command: "));
                Serial.println(sBuffer);
            }
        }
    }
}

void notifySettingsChanged() {
    notifyTask(&t_effect_printStatus, true);
    notifyTask(&t_stateUpdate_angleAndRelay, true);
    notifyTask(&t_effect_refreshServoAndRelay, true);
    notifyTask(&t_effect_processSettings, false);
}
