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

#define CMD_SET_Q_DIV "QD"
#define CMD_SET_Q_OFF "QO"
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
            PARSE(CMD_INTERNAL)
                if (valueBuffer.equals("SYNC")) {
                    serialPrintState();
                    serialPrintConfig();
                } else if (valueBuffer.equals("RES")) {
                    state.volume = 0.0f;
                    state.volumePulses = 0;
                    state.volumeFlowPulses = 0;
                    state.volumeFlow = 0.0f;
                    config.Q_div = 6;
                    config.Q_offset = 8;
                    serialPrintState();
                    serialPrintConfig();
                }
            OR_PARSE(CMD_SET_Q_DIV)
                config.Q_div = strtol(valueBuffer.c_str(), nullptr, 10);
                serialPrintConfig();
            OR_PARSE(CMD_SET_Q_OFF)
                config.Q_offset = strtol(valueBuffer.c_str(), nullptr, 10);
                serialPrintConfig();
            } else {
                // Serial.print(F("Unknown command: "));
                // Serial.println(sBuffer);
            }
        }
    }
}

void serialPrintState() {
    Serial.print(F("DRQ:V:"));
    Serial.println(state.volume, 4);
    Serial.print(F("DRQ:VP:"));
    Serial.println(state.volumePulses);
    Serial.print(F("DRQ:VF:"));
    Serial.println(state.volumeFlow, 4);
    Serial.print(F("DRQ:VFP:"));
    Serial.println(state.volumeFlowPulses);
}


void serialPrintConfig() {
    Serial.print(F("DRQ:QD:"));
    Serial.println(config.Q_div);
    Serial.print(F("DRQ:QO:"));
    Serial.println(config.Q_offset);
}