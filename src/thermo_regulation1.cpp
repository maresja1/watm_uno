#include <Arduino.h>
#include <EEPROM.h>
#include <TaskScheduler.h>
#include <avr/wdt.h>

#include "Thermoino.h"

#define USE_SIMULATION 0

const float simulationSpeed = 1.0f;

void eeprom_persist_cb();

Scheduler runner;
//Tasks
// periodic task to read sensors - temperatures, might enable other tasks
Task t_stateUpdate_readSensors(uint64_t(1e3 / simulationSpeed), -1, &stateUpdate_readSensors_cb, &runner);
// periodic task to update state based on instructions from the Serial port
Task t_stateUpdate_serialReader(uint64_t(2e3 / simulationSpeed), -1, &stateUpdate_serialReader_cb, &runner);
// periodic task to update eeprom data
Task t_stateUpdate_eeprom(uint64_t(10e3 / simulationSpeed), -1, &eeprom_persist_cb, &runner);

State state = {
    .volume = 0,
    .volumePulses = 0,
    .volumeFlow = 0,
    .volumeFlowPulses = 0,
};

State prevState = {
    .volume = 0,
    .volumePulses = 0,
    .volumeFlow = 0,
    .volumeFlowPulses = 0,
};

Config config = {
    .Q_div = 6,
    .Q_offset = 8,
};

char buffer[MAX_BUFFER_LEN];

volatile uint8_t pulsesSinceClear = 0;

byte sensorInterrupt = 0;  // 0 = digital pin 2
byte sensorPin = 2;

void pulseCounter();

void setup()
{
    serialLineSetup();

    eepromInit();

    pinMode(sensorPin, INPUT);
    digitalWrite(sensorPin, HIGH);

    attachInterrupt(sensorInterrupt, pulseCounter, FALLING);

    wdt_disable();  /* Disable the watchdog and wait for more than 2 seconds */
    delay(3000);  /* Done so that the Arduino doesn't keep resetting infinitely in case of wrong configuration */
    wdt_enable(WDTO_4S);  /* Enable the watchdog with a timeout of 4 seconds */

    t_stateUpdate_serialReader.enable();
    t_stateUpdate_readSensors.enable();
    t_stateUpdate_eeprom.enable();
#if DEBUG_LEVEL > 0
    Serial.println(F("Initialized..."));
#endif
    serialPrintState();
}

void loop()
{
    runner.execute();
    serialLineBufferLoop();
}

// Number of pulses per L (datasheet for YF-B10)
// uint32_t pulsesPerL = 476;
// float calibrationFactor = pulsesPerL - ;

uint32_t lastUpdate = 0;

// #define CAS __sync_bool_compare_and_swap

uint8_t getPulsesCasAndClear() {
    uint8_t pulses;
    // do {
    //     pulses = pulsesSinceClear;
    // } while (!CAS(&pulsesSinceClear, pulses, 0));
    // return pulses;
    noInterrupts();
    pulses = pulsesSinceClear;
    pulsesSinceClear = 0;
    interrupts();
    return pulses;
}

void stateUpdate_readSensors_cb()
{
    wdt_reset();
    const uint32_t now = millis();
    const uint32_t pulses = getPulsesCasAndClear();

#if DEBUG_LEVEL > 0
    DEBUG_TASK_ENTRY("stateUpdate_readSensors");
#endif
    if (lastUpdate == 0) {
        lastUpdate = millis();
        pulsesSinceClear = 0;
        wdt_reset();
        return;
    }

    const uint32_t sinceLast = now - lastUpdate;
    lastUpdate = now;

    float timeCorrection = static_cast<float>(sinceLast) / 1000.0f;

    float flow; // L/min
    if (pulses > 1) {
        float freq = static_cast<float>(pulses) / timeCorrection;
        flow = (freq + static_cast<float>(config.Q_offset)) / static_cast<float>(config.Q_div);
    } else {
        flow = 0.0f;
    }

    state.volume += flow / 60.0f * timeCorrection;
    state.volumePulses += pulses;
    state.volumeFlow = flow;
    state.volumeFlowPulses = pulses;
    pulsesSinceClear = 0;

#if PRINT_SERIAL_UPDATES
    if(pulses > 0 || prevState.volumeFlowPulses > 0) {
#if DEBUG_LEVEL > 0
        Serial.print(F("Pulses: "));
        ser_print_ll(config.volumePulses);
#endif
        serialPrintState();
#endif
        prevState = state;
    }
#if DEBUG_LEVEL > 0
    DEBUG_TASK_RET("stateUpdate_readSensors");
#endif

    wdt_reset();
}

/**
 * Insterrupt Service Routine
 */
void pulseCounter()
{
#if DEBUG_LEVEL > 0
  Serial.print(F("Pulse"));
#endif
  pulsesSinceClear++;
}

void eeprom_persist_cb()
{
    eepromUpdate();
}

void notifyTask(Task *task, bool immediate)
{
    task->setIterations(1);
    if (immediate) {
        task->enable();
    } else if (!task->isEnabled()) {
        task->enableDelayed(task->getInterval());
    }
}

char yesOrNo(int input);


#define EEPROM_MAGIC 0xDEADBE01

void eepromInit()
{
    uint32_t checkCode;
    int offset = 0;
    EEPROM.get(offset, checkCode);
    offset += sizeof(checkCode);
    if (checkCode == EEPROM_MAGIC) {
        EEPROM.get(offset, config);
        offset += sizeof(config);
        EEPROM.get(offset, state);

	// migration
//    } else if (checkCode == 0xDEADBE00) {
//        EEPROM.get(sizeof(checkCode), config);
//        config.settingsSelected = -1;
	} else {
        eepromUpdate();
    }
}

void eepromUpdate()
{
    uint32_t checkCode = EEPROM_MAGIC;
    int offset = 0;
    EEPROM.put(offset, checkCode);
    offset += sizeof(checkCode);
    EEPROM.put(offset, config);
    offset += sizeof(config);
    EEPROM.put(offset, state);
}

