//
// Created by jan on 4/5/22.
//
#include <Arduino.h>

#include "Thermoino.h"

#define USE_SOFT_SERIAL 0

const uint8_t serialLineBufferCapacity = 63;
uint8_t serialLineBufferIdx = 0;
char serialLineBuffer[serialLineBufferCapacity + 1] = "";   // an array to store the received data
uint8_t serialLineBufferDataReady = 0x0;

#if USE_SOFT_SERIAL
#include <SoftwareSerial.h>
SoftwareSerial mySerial(13, 12); // RX, TX
#endif

void serialLineSetup() {
    Serial.begin(115200);

    for (int i = 0; i < 10 && !Serial; ++i) {
        // wait for serial port to connect. Needed for native USB port only
        delay(10);
    }
#if USE_SOFT_SERIAL
    mySerial.begin(9600);
#endif
}

void serialLineBufferLoop() {
    while(Serial.available() > 0 && !(serialLineBufferDataReady & 0x1)) {
        int rc = Serial.read();
        if (rc < 0) {
            break;
        } else if (rc != '\r' && rc != '\n') {
            serialLineBuffer[serialLineBufferIdx++] = rc;
            if (serialLineBufferIdx >= serialLineBufferCapacity) {
                serialLineBufferIdx = 0; // overflow
                serialLineBuffer[serialLineBufferCapacity] = 0x0;
                serialLineBufferDataReady |= 0x2;
            }
        } else {
            if (serialLineBufferIdx == 0) {
                // ignore empty lines
                continue;
            }
            serialLineBuffer[serialLineBufferIdx] = '\0'; // terminate the string
            serialLineBufferIdx = 0;
            serialLineBufferDataReady |= 0x1;
        }
    }
}

void stateUpdate_serialReader_cb()
{

#define CMD_HNO "HNO"
#define CMD_VENT_SET "O"
#define CMD_BOILER_REF_TEMP_SET "BRT"
#define CMD_ROOM_REF_TEMP_SET "RRT"
#define CMD_PID_BL_Kp "PID_BL_Kp"
#define CMD_PID_BL_Ki "PID_BL_Ki"
#define CMD_PID_BL_Kd "PID_BL_Kd"
#define CMD_PID_CR_Kp "PID_CR_Kp"
#define CMD_PID_CR_Ki "PID_CR_Ki"
#define CMD_PID_CR_Kd "PID_CR_Kd"
#define CMD_MODE "M"
#define CMD_INTERNAL "INT"
#define literal_len(x) (sizeof(x) - 1)
#define PARSE(x) if (commandBuffer.startsWith(F(x ":"))) { const String &valueBuffer = commandBuffer.substring(literal_len(x ":"));
#define OR_PARSE(x) } else if (commandBuffer.startsWith(F(x ":"))) { const String &valueBuffer = commandBuffer.substring(literal_len(x ":"));

    if (serialLineBufferDataReady & 0x2) {
        serialLineBufferDataReady = 0x0;
#if USE_SOFT_SERIAL
        mySerial.println("serial overflow");
#endif
    } else if (serialLineBufferDataReady & 0x1) {
        serialLineBufferDataReady = 0x0;
        const String &sBuffer = String(serialLineBuffer);
#if USE_SOFT_SERIAL
        mySerial.print("got -> ");
        mySerial.println(sBuffer);
#endif
        if (sBuffer.startsWith(F("DRQ:"))) {
            const String &commandBuffer = sBuffer.substring(4);
            PARSE(CMD_HNO)
                heatNeededOverride = strtol(valueBuffer.c_str(), nullptr, 10);
                notifySettingsChanged();
                screenSaverWakeup();
            OR_PARSE(CMD_VENT_SET)
                config.settingsSelected = MENU_POS_VENT_MANUAL;
                angle = strtol(valueBuffer.c_str(), nullptr, 10);
                notifySettingsChanged();
                screenSaverWakeup();
            OR_PARSE(CMD_BOILER_REF_TEMP_SET)
                config.refTempBoiler = strtol(valueBuffer.c_str(), nullptr, 10);
                notifySettingsChanged();
            OR_PARSE(CMD_ROOM_REF_TEMP_SET)
                config.refTempRoom = strtod(valueBuffer.c_str(), nullptr);
                notifySettingsChanged();
            OR_PARSE(CMD_PID_BL_Kp)
                config.pidKp = strtod(valueBuffer.c_str(), nullptr);
                notifySettingsChanged();
            OR_PARSE(CMD_PID_BL_Ki)
                config.pidTi = strtod(valueBuffer.c_str(), nullptr);
                notifySettingsChanged();
            OR_PARSE(CMD_PID_BL_Kd)
                config.pidTd = strtod(valueBuffer.c_str(), nullptr);
                notifySettingsChanged();
            OR_PARSE(CMD_PID_CR_Kp)
                config.pidRelayKp = strtod(valueBuffer.c_str(), nullptr);
                notifySettingsChanged();
            OR_PARSE(CMD_PID_CR_Ki)
                config.pidRelayTi = strtod(valueBuffer.c_str(), nullptr);
                notifySettingsChanged();
            OR_PARSE(CMD_PID_CR_Kd)
                config.pidRelayTd = strtod(valueBuffer.c_str(), nullptr);
                notifySettingsChanged();
            OR_PARSE(CMD_MODE)
                if (valueBuffer.equals("A")) {
                    config.settingsSelected = -1;
                    heatNeededOverride = 0;
                    notifySettingsChanged();
                    screenSaverWakeup();
                }
            OR_PARSE(CMD_INTERNAL)
                if (valueBuffer.equals("SYNC")) {
                    serialPrintConfig();
                }
            } else {
                // Serial.print(F("Unknown command: "));
                // Serial.println(sBuffer);
            }
        }
    }
}

void serialPrintConfig() {
    Serial.print(F("DRQ:R:"));
    Serial.println(String(relay_or_override()));
    Serial.print(F("DRQ:O:"));
    Serial.println(angle);
    Serial.print(F("DRQ:BRT:"));
    Serial.println(config.refTempBoiler);
    Serial.print(F("DRQ:RRT:"));
    Serial.println(config.refTempRoom);
    Serial.print(F("DRQ:SET:"));
    Serial.println(config.settingsSelected);
    Serial.print(F("DRQ:PID_BL_Kp:"));
    Serial.println(config.pidKp, 4);
    Serial.print(F("DRQ:PID_BL_Ki:"));
    Serial.println(config.pidTi, 4);
    Serial.print(F("DRQ:PID_BL_Kd:"));
    Serial.println(config.pidTd, 4);
    Serial.print(F("DRQ:PID_CR_Kp:"));
    Serial.println(config.pidRelayKp, 4);
    Serial.print(F("DRQ:PID_CR_Ki:"));
    Serial.println(config.pidRelayTi, 4);
    Serial.print(F("DRQ:PID_CR_Kd:"));
    Serial.println(config.pidRelayTd, 4);
    Serial.print(F("DRQ:HPWM:"));
    Serial.println(lround(double(pidHeatPWM.getConstrainedValue())));
}

void notifySettingsChanged() {
    notifyTask(&t_effect_printStatus, true);
    notifyTask(&t_stateUpdate_angleAndRelay, true);
    notifyTask(&t_effect_refreshServoAndRelay, true);
    notifyTask(&t_effect_processSettings, false);
}
