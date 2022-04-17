#include <Arduino.h>

#include "Thermoino.h"

#include <EEPROM.h>
#include <sTune.h>
#include <QuickPID.h>

#if USE_DHT_ROOM_TEMP
#include <DHT.h>
#endif
#include <Servo.h>
#include <TaskScheduler.h>

#if USE_DT_ROOM_BOILER
#include <OneWire.h>
#include <DallasTemperature.h>
#endif

#define USE_SIMULATION 1
const float simulationSpeed = 1.0f;

Scheduler runner;
//Tasks
// periodic task to read state of buttons - might enable other tasks, if needed
Task t_stateUpdate_readButtons(1e2, -1, &stateUpdate_readButtons_cb, &runner);
// one-shot task to compute desired vent angle and relay status - enabled if status changes require recalculation
Task t_stateUpdate_angleAndRelay(uint64_t(1e3 / simulationSpeed), 1, &stateUpdate_angleAndRelay_cb, &runner);
// periodic task to read sensors - temperatures, might enable other tasks
Task t_stateUpdate_readSensors(uint64_t(2e3 / simulationSpeed), -1, &stateUpdate_readSensors_cb, &runner);
// periodic task for hot water probe - allows relay to let some water through once in a while to allow for better measurement
Task t_stateUpdate_hotWaterProbe(uint64_t(1e4 / simulationSpeed), -1, &stateUpdate_hotWaterProbe_cb, &runner);
// periodic task for PID heat control
Task t_stateUpdate_heatNeeded(uint64_t(1e4 / simulationSpeed), -1, &stateUpdate_heatNeeded_cb, &runner);
// periodic task to update state based on instructions from the Serial port
Task t_stateUpdate_serialReader(uint64_t(1e3 / simulationSpeed), -1, &stateUpdate_serialReader_cb, &runner);
// one-shot effect task to update desired settings of the vent angle and circuit relay - triggered by other tasks
Task t_effect_refreshServoAndRelay(uint64_t(6e3 / simulationSpeed), 1, &effect_refreshServoAndRelay_cb, &runner);
// one-shot effect task to print current status to LCD - triggered by other tasks when smth. changes
Task t_effect_printStatus(1e3, 1, &effect_printStatus_cb, &runner);
// one-shot effect task to save current settings to EEPROM - should always be planned with a delay to avoid too many overwrites
Task t_effect_processSettings(2e3, 1, &effect_processSettings_cb, &runner);

// initialize the library with the numbers of the interface pins
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);

#if USE_DHT_ROOM_TEMP && !USE_SIMULATION
// instance of digital thermometer
DHT digitalThermometer(DHT21_PIN, DHT21);
#endif

#if USE_DT_ROOM_BOILER && !USE_SIMULATION
OneWire oneWire_TempBoiler(DT_BOILER_PIN);
DallasTemperature dtTempBoiler(&oneWire_TempBoiler);
#endif

// servo allowing air to come in the boiler (attached to the vent)
Servo servo;

const uint8_t relayWindowFragments = 30;

Configuration config = {
    .refTempBoiler = 55,
    .refTempBoilerIdle = 50,
    .refTempRoom = 21.0f,
    .circuitRelayForced = 0,  // 0 no override, 1 - override false, 2 or else - override true
    .servoMin = 0,
    .servoMax = 180,
    .debounceLimitC = 2.0f,
    .underheatingLimit = 45,
    .overheatingLimit = 80,
    // least squares of (1,2), (20,6), (44,12) - y = 0.2333x + 1.612
    .deltaTempPoly1 = 0.240f, //0.2333f,
    .deltaTempPoly0 = 0.0f, //1.612f,
    .roomTempAdjust = 0.0f,
    // K_u = 100 (maybe less), T_u = 30s (pretty stable)
    // following params were for sim x10 without rescaling
    .pidKp = 4.0f,
    .pidKi = 0.01f,
    .pidKd = 0.04f,
    // K_u = 0.5 (maybe less), T_u = 800s
    .pidRelayKp = 12.0f,
    .pidRelayKi = 0.0001f,
    .pidRelayKd = 1.60f,
    .settingsSelected = -1
};

char buffer[MAX_BUFFER_LEN];

uint8_t angle = 50;
uint8_t currAngle = 50;
int16_t settingsSelectedPrint = -2;
float boilerTemp = 50.0f;
float roomTemp = 20.0f;
float roomHumidity = 0.0f;
bool heatNeeded = false;
uint8_t heatNeededOverride = 0; // 0 no override, 1 - override false, 2 or else - override true
bool overheating = false;
bool underheating = false;
bool circuitRelay = false;

// backlight screensaver
uint8_t noPressCycles = 0;

// hot water probe - periodically allow water to flow for 10s
bool hotWaterProbeHadRelayOn = false;
bool hotWaterProbeEnforced = false;
uint8_t hotWaterProbeCycles = 0;

float pidBoilerSet = 0, pidBoilerIn = 0, pidBoilerOut = 0;
// K_u = 100 (maybe less), T_u = 30s (pretty stable)
//PID pidBoiler(&pidBoilerIn, &pidBoilerOut, &pidBoilerSet, 33.33f, 0.1f, 0.0f, DIRECT);
QuickPID pidBoiler(&pidBoilerIn, &pidBoilerOut, &pidBoilerSet);
//sTune tuner(&pidBoilerIn, &pidBoilerOut, sTune::NoOvershoot_PID, sTune::directIP, sTune::printOFF);


float pidRelaySet = 0, pidRelayIn = 0, pidRelayOut = 0;
// K_u = 0.5f, T_u = 280s
//PID pidRelay(&pidRelayIn, &pidRelayOut, &pidRelaySet, 0.5f, 0.0f, 0.0f, DIRECT);
QuickPID pidRelay(&pidRelayIn, &pidRelayOut, &pidRelaySet);

uint8_t deviceAddress;

const float boilerPidOutOffset = -50.0f;

void setup()
{
    Serial.begin(115200);

    for (int i = 0; i < 10 && !Serial; ++i) {
        // wait for serial port to connect. Needed for native USB port only
        delay(10);
    }

    eepromInit();

    servoSetPos(angle);
    servo.attach(SERVO_PIN);
    analogReference(EXTERNAL);
    pinMode(SERVO_PIN, OUTPUT);
    digitalWrite(CIRCUIT_RELAY_PIN, HIGH);
    pinMode(CIRCUIT_RELAY_PIN, OUTPUT);
    pinMode(BTN_1_PIN, INPUT_PULLUP);
    pinMode(BTN_2_PIN, INPUT_PULLUP);
    pinMode(BTN_3_PIN, INPUT_PULLUP);
    pinMode(BTN_4_PIN, INPUT_PULLUP);

    // set up the LCD's number of columns and rows:
    lcd.begin(16, 2);
    lcd.print(F("Booting"));

    // set up DHT
#if USE_DHT_ROOM_TEMP && !USE_SIMULATION
    digitalThermometer.begin();
#endif

    delay(1000);
    lcd.write(".");
#if USE_DT_ROOM_BOILER && !USE_SIMULATION
    dtTempBoiler.begin();

    dtTempBoiler.getAddress(&deviceAddress, 0);
    dtTempBoiler.setWaitForConversion(false);
    dtTempBoiler.requestTemperaturesByAddress(&deviceAddress);
#endif

    pidBoiler.SetOutputLimits(0.0f + boilerPidOutOffset, 99.0f + boilerPidOutOffset);
    pidBoiler.SetTunings(config.pidKp, config.pidKi, config.pidKd);
    pidBoiler.SetMode(QuickPID::Control::timer);

    pidRelay.SetTunings(config.pidRelayKp, config.pidRelayKi, config.pidRelayKd);
    pidRelay.SetOutputLimits(0.0f, float(relayWindowFragments));
//    pidRelay.SetMode(QuickPID::Control::timer);

    sendCurrentStateToRelay(circuitRelay);
    lcd.write(".");

//    Serial.println(F("Thermoino 1, built:" __DATE__ " " __TIME__ " (" __FILE__ ") - Setup finished."));

    t_stateUpdate_readButtons.enable();
    t_stateUpdate_readSensors.enable();

    lcd.write(".");

//    t_stateUpdate_hotWaterProbe.enable();
    t_stateUpdate_heatNeeded.enable();
    t_stateUpdate_serialReader.enable();

    notifyTask(&t_effect_printStatus, false);
    notifyTask(&t_stateUpdate_angleAndRelay, false);

    lcd.print(F("ok"));
//    Serial.println(F("Setup finished..."));

    effect_processSettings_cb();
//    tuner.Configure(100, 99, 11, 12, 500, 50, 200);
}

void loop()
{
    runner.execute();
    serialLineBufferLoop();
}

float simulate_radiatorPower(float deltaTemp)
{
//    int16_t radiatorPowerDT30 = 849;
//    int16_t radiatorPowerDT50 = 1679;
//    int16_t radiatorSlope = (radiatorPowerDT50 - radiatorPowerDT30) / 20;
//    int16_t radiatorSlopeBase = radiatorPowerDT30 - (30 * radiatorSlope);

    // polynomial interpolation
    return max(0.0f, (0.264f * deltaTemp * deltaTemp) + 20.38f * deltaTemp);
}

#if USE_SIMULATION
const int8_t radiatorCount = 10;
const int8_t boilerVolume = 36;
const uint32_t boilerPower = 18000;
#define waterHeatCapacity 4180.0f
const uint32_t heatedAirOtherCapacity = 2000000;
const int8_t outsideTemp = 10;
const int16_t circuitVolume = int16_t(float(radiatorCount) * 5.8f);

float circuitTemp = 40;

uint64_t lastSimulate = UINT64_MAX;
float lastPower = 0.0f;
void stateUpdate_simulator_cb()
{
    if (lastSimulate > millis()) {
        lastSimulate = millis();
    }
    const float dTime = float(millis() - lastSimulate) * simulationSpeed / 1000.0f; // should be 1000 or 100 for 10x speed up

    const float circuitPower = float(radiatorCount) * simulate_radiatorPower(circuitTemp - roomTemp);

    // max 8 kW (temp difference 20 C) lost due to insulation
    const float insulationLost = max(0, 8000.0f * ((roomTemp - outsideTemp) / 20)) ;
    const float boilerAsRadiator = simulate_radiatorPower(boilerTemp - roomTemp) * 2;
    const float roomHeat = circuitPower + boilerAsRadiator - insulationLost;

    const float boilerPowerAngle = min(
        max(lastPower - 1000.0f, float(boilerPower * currAngle) / 100),
        lastPower + 1000.0f
    );
    lastPower = boilerPowerAngle;

    const float boilerDelta =  ((boilerPowerAngle - boilerAsRadiator) * dTime) /
            (float(boilerVolume) * waterHeatCapacity);

    boilerTemp += boilerDelta;

    if (circuitRelay) {
        const float conduct = (boilerTemp - circuitTemp) * waterHeatCapacity * 2.639f * 0.2f; // 2.63 is dm^3*s^-1 of the circuit relay
        boilerTemp -= float(double(conduct * dTime) / (double(boilerVolume) * waterHeatCapacity));
        circuitTemp += float(double(conduct * dTime) / (double(circuitVolume) * waterHeatCapacity));
    }

    roomTemp += float(double(roomHeat * dTime) / heatedAirOtherCapacity);
    circuitTemp -= float(double(circuitPower * dTime) / (double(circuitVolume) * waterHeatCapacity));

    lastSimulate = millis();

#if 0
    DEBUG_SER_PRINT(circuitTemp);
    DEBUG_SER_PRINT(dTime);
    DEBUG_SER_PRINT(boilerDelta);
    DEBUG_SER_PRINT(roomTemp);
    DEBUG_SER_PRINT(boilerTemp);
    DEBUG_SER_PRINT(currAngle);
    DEBUG_SER_PRINT(heatNeeded);
    DEBUG_SER_PRINT_LN(pidRelayOut);
#endif
}
#endif

uint8_t heatNeededCurrentFragment = relayWindowFragments * 0.8f;
uint8_t pidRelayAtStartOfWindow = 0;
void stateUpdate_heatNeeded_cb()
{
    const bool lastHeatNeeded = heatNeeded;
    heatNeededCurrentFragment++;
    if (heatNeededCurrentFragment >= relayWindowFragments) {
        heatNeededCurrentFragment = 0;
        pidRelayAtStartOfWindow = lround(pidRelayOut);
    }
    if (heatNeededOverride == 0) {
        heatNeeded = pidRelayAtStartOfWindow > heatNeededCurrentFragment;
//        heatNeeded = (heatNeeded && (roomTemp - config.refTempRoom <= (config.debounceLimitC / 2))) ||
//                     (roomTemp - config.refTempRoom <= -(config.debounceLimitC / 2));
    } else {
        heatNeeded = heatNeededOverride != 1;
    }
    if (lastHeatNeeded != heatNeeded) {
        notifyTask(&t_stateUpdate_angleAndRelay, false);
#if PRINT_SERIAL_UPDATES
        Serial.print(F("DRQ:HN:"));
        Serial.println(heatNeeded);
        Serial.print(F("DRQ:HPWM:"));
        Serial.println(pidRelayAtStartOfWindow);
#endif
    }
}

void stateUpdate_hotWaterProbe_cb()
{
    if (config.settingsSelected != -1) {
        return; // hot water probe disabled in settings
    }
    hotWaterProbeCycles++;
    if (hotWaterProbeCycles > 5) {
        hotWaterProbeCycles = 0;
        if (!hotWaterProbeHadRelayOn) {
            hotWaterProbeEnforced = true;
            notifyTask(&t_stateUpdate_angleAndRelay, false);
            notifyTask(&t_effect_printStatus, false);
        }
    } else if(hotWaterProbeEnforced) {
        // turn off after one cycle of being on
        hotWaterProbeEnforced = false;
        notifyTask(&t_stateUpdate_angleAndRelay, false);
        notifyTask(&t_effect_printStatus, false);
    }
}

void stateUpdate_readSensors_cb()
{
#if DEBUG_LEVEL > 0
    DEBUG_TASK_ENTRY("stateUpdate_readSensors");
#endif
    const float lastBoilerTemp = boilerTemp;
    const float lastRoomTemp = roomTemp;

#if !USE_SIMULATION
#if USE_DHT_ROOM_TEMP
        float lastReading = digitalThermometer.readTemperature(false, false) + config.roomTempAdjust;
        if (roomTemp > 0.0f) {
            // see https://stackoverflow.com/questions/10990618/calculate-rolling-moving-average-in-c/10990656#10990656
            roomTemp = (lastReading + (4 * roomTemp)) / 5; // running average
        } else {
            roomTemp = lastReading;
        }
        roomHumidity = digitalThermometer.readHumidity(false);
#else
        roomTemp = readTemp(ROOM_THERM_PIN);
#endif

#if USE_DT_ROOM_BOILER
        boilerTemp = dtTempBoiler.getTempC(&deviceAddress);
        dtTempBoiler.requestTemperaturesByAddress(&deviceAddress); // make ready for next call
#else
        boilerTemp = readTemp(BOILER_THERM_PIN);
#endif

    // roomTemp is not NaN
    if (roomTemp == roomTemp) {
        boilerTemp += (config.deltaTempPoly1 * (boilerTemp - roomTemp)) + config.deltaTempPoly0;
    }

#else
    stateUpdate_simulator_cb();
#endif

    pidBoilerIn = boilerTemp;
    pidRelayIn = roomTemp;

//    switch (tuner.Run()) {
//        case tuner.sample:
//            break;
//        case tuner.tunings:
//            tuner.GetAutoTunings(&config.pidKp, &config.pidKi, &config.pidKd);
//            pidBoiler.SetMode(QuickPID::Control::timer);
//            pidBoiler.SetTunings(config.pidKp, config.pidKi, config.pidKd);
//            break;
//        case tuner.runPid:
//            break;
//    }
    pidBoiler.Compute();
    if (!underheating) {
        pidRelay.Compute();
    }

    if((boilerTemp != lastBoilerTemp) || (roomTemp != lastRoomTemp)) {
        notifyTask(&t_stateUpdate_angleAndRelay, false);
        notifyTask(&t_effect_printStatus, false);

#if PRINT_SERIAL_UPDATES
        Serial.print(F("DRQ:RT:"));
        Serial.println(String(roomTemp));
        Serial.print(F("DRQ:BT:"));
        Serial.println(String(boilerTemp));
#endif
    }
#if DEBUG_LEVEL > 0
    DEBUG_TASK_RET("stateUpdate_readSensors");
#endif
}

#define relay_or_override() config.circuitRelayForced == 0 ? circuitRelay : config.circuitRelayForced == 1

void stateUpdate_angleAndRelay_cb()
{
#if DEBUG_LEVEL > 0
    DEBUG_TASK_ENTRY("stateUpdate_angleAndRelay");
#endif

//    const float boilerDelta = boilerTemp - float(heatNeeded ? config.refTempBoiler : config.refTempBoilerIdle);
    overheating = (overheating && (boilerTemp - config.overheatingLimit >= (config.debounceLimitC / 2))) ||
                  (boilerTemp - config.overheatingLimit >= -(config.debounceLimitC / 2));

    underheating = (underheating && (boilerTemp - config.underheatingLimit <= (config.debounceLimitC / 2))) ||
                   (boilerTemp - config.underheatingLimit <= -(config.debounceLimitC / 2));

    bool lastCircuitRelay = circuitRelay;
    circuitRelay = (!underheating && (heatNeeded || overheating)) || hotWaterProbeEnforced;

    uint8_t lastAngle = angle;
    if (config.settingsSelected == MENU_POS_VENT_MANUAL) {
        // setting is manual, make no changes
    } else if (config.settingsSelected == MENU_POS_SERVO_MIN) {
        angle = 0;
    } else if (config.settingsSelected == MENU_POS_SERVO_MAX) {
        angle = 99;
    } else {
        angle = max(min((pidBoilerOut - boilerPidOutOffset), 99), 0);

//        for (int i = config.curveItems - 1; i >= 0; i--) {
//            if (boilerDelta >= maxDeltaSettings[i]) {
//                int nextAngle = maxDeltaHigh[i];
//                int nextI = i + 1;
//                if (nextI < config.curveItems) {
//                    // linear interpolation
//                    nextAngle = float(maxDeltaHigh[i]) +
//                                (
//                                        (boilerDelta - float(maxDeltaSettings[i])) *
//                                        (
//                                                float(maxDeltaHigh[nextI] - maxDeltaHigh[i]) /
//                                                float(maxDeltaSettings[nextI] - maxDeltaSettings[i])
//                                        )
//                                );
//                }
//                angle = nextAngle;
//                break;
//            }
//        }
    }

    if (
            config.settingsSelected != MENU_POS_SERVO_MIN &&
            config.settingsSelected != MENU_POS_SERVO_MAX &&
            (lastAngle != angle || lastCircuitRelay != circuitRelay)
    ) {
        notifyTask(&t_effect_refreshServoAndRelay, false);
        notifyTask(&t_effect_printStatus, true);
#if PRINT_SERIAL_UPDATES
        Serial.print(F("DRQ:O:"));
        Serial.println(angle);
        Serial.print(F("DRQ:R:"));
        Serial.println(String(relay_or_override()));
#endif
    }
#if DEBUG_LEVEL > 0
    DEBUG_TASK_RET("stateUpdate_angleAndRelay");
#endif
}

#define MAX_SERVO_STEP 10

void effect_refreshServoAndRelay_cb()
{
#if DEBUG_LEVEL > 0
    DEBUG_TASK_ENTRY("effect_refreshServoAndRelay");
#endif
    const bool circuitRelayOrOverride = relay_or_override();
    if (boilerTemp > 90) {
        // safety mechanism
        servoSetPos(0);
        sendCurrentStateToRelay(true);
    } else {
//        if (angle > currAngle + MAX_SERVO_STEP) {
//            currAngle += MAX_SERVO_STEP;
//            notifyTask(&t_effect_refreshServoAndRelay, false);
//        } else if (angle < currAngle - MAX_SERVO_STEP) {
//            currAngle -= MAX_SERVO_STEP;
//            notifyTask(&t_effect_refreshServoAndRelay, false);
//        } else {
//        }
        currAngle = angle;
        servoSetPos(currAngle);
        sendCurrentStateToRelay(circuitRelayOrOverride);
    }
#if DEBUG_LEVEL > 0
    DEBUG_TASK_RET("effect_refreshServoAndRelay");
#endif
}

void effect_printStatus_cb()
{
#if DEBUG_LEVEL > 0
    DEBUG_TASK_ENTRY("effect_printStatus");
#endif
    printStatus();
#if DEBUG_LEVEL > 0
    DEBUG_TASK_RET("effect_printStatus");
#endif
}

void stateUpdate_readButtons_cb()
{
#if DEBUG_LEVEL > 1
    DEBUG_TASK_ENTRY("stateUpdate_readButtons");
#endif
    if (processSettings()) {
        notifyTask(&t_effect_printStatus, true);
        notifyTask(&t_stateUpdate_angleAndRelay, true);
        notifyTask(&t_effect_refreshServoAndRelay, true);
        notifyTask(&t_effect_processSettings, false);
    }
#if DEBUG_LEVEL > 1
    DEBUG_TASK_RET("stateUpdate_readButtons");
#endif
}

void effect_processSettings_cb()
{
#if DEBUG_LEVEL > 1
    DEBUG_TASK_ENTRY("stateUpdate_readButtons");
#endif
    eepromUpdate();
    pidRelay.SetMode(config.settingsSelected == MENU_POS_HEAT_MANUAL ? QuickPID::Control::manual : QuickPID::Control::timer);
    pidBoiler.SetMode(config.settingsSelected == MENU_POS_VENT_MANUAL ? QuickPID::Control::manual : QuickPID::Control::timer);
    pidBoilerSet = config.refTempBoiler;
    pidRelaySet = config.refTempRoom;
    pidBoiler.SetTunings(config.pidKp, config.pidKi * simulationSpeed, config.pidKd * simulationSpeed);
    pidRelay.SetTunings(config.pidRelayKp, config.pidRelayKi * simulationSpeed, config.pidRelayKd * simulationSpeed);
#if PRINT_SERIAL_UPDATES
    Serial.print(F("DRQ:R:"));
    Serial.println(String(relay_or_override()));
    Serial.print(F("DRQ:O:"));
    Serial.println(angle);
    Serial.print(F("DRQ:BRT:"));
    Serial.println(config.refTempBoiler);
    Serial.print(F("DRQ:SET:"));
    Serial.println(config.settingsSelected);
    Serial.print(F("DRQ:PID_BL_Kp:"));
    Serial.println(config.pidKp);
    Serial.print(F("DRQ:PID_BL_Ki:"));
    Serial.println(config.pidKi);
    Serial.print(F("DRQ:PID_BL_Kd:"));
    Serial.println(config.pidKd);
    Serial.print(F("DRQ:PID_CR_Kp:"));
    Serial.println(config.pidRelayKp);
    Serial.print(F("DRQ:PID_CR_Ki:"));
    Serial.println(config.pidRelayKi);
    Serial.print(F("DRQ:PID_CR_Kd:"));
    Serial.println(config.pidRelayKd);
#endif
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

void sendCurrentStateToRelay(const bool state) {
    hotWaterProbeHadRelayOn |= state;
    digitalWrite(CIRCUIT_RELAY_PIN, !state);
}

int8_t readButton(Button_t *button)
{
    int btn = digitalRead(button->pin);
    if (btn != button->state) {
        button->pressedFor = 0;
        button->state = btn;
        if (btn == LOW) {
            return 1; // released
        } else if (btn == HIGH) {
            return -1; // pressed
        }
    } else if(btn == LOW) {
        button->pressedFor++;
    }
    return 0;
}

Button_t btn1 = {
    .pin = BTN_1_PIN,
    .state = LOW
};
Button_t btn2 = {
    .pin = BTN_2_PIN,
    .state = LOW
};
Button_t btn3 = {
    .pin = BTN_3_PIN,
    .state = LOW
};
Button_t btn4 = {
    .pin = BTN_4_PIN,
    .state = LOW
};

void servoSetPos(int positionPercent)
{
    // int time = 1500 + ((positionPercent - 50)*step);
    float multi = float(config.servoMax - config.servoMin) / 100.0f;
    int16_t value = config.servoMin + int16_t(float(positionPercent) * float(multi));
//    DEBUG_SER_PRINT_LN(value);
    // the minus is to switch direction, 180 is constant used in method write as a maximum,
    servo.write(180 - value);
}

void printSettings()
{
    lcd.noCursor();
    const ConfigMenuItem_t *currentItem = getMenu(config.settingsSelected);

    lcd.setCursor(0, 0);
    lcd.print(currentItem->name);
    lcd.print(' ');

    lcd.setCursor(0, 1);
    currentItem->formatter(currentItem->param, lcd, currentItem->handler(currentItem->param, 0));
    lcd.setCursor(15, 1);
}

char yesOrNo(int input);

void printStatusOverviewTop()
{
    lcd.noCursor();
    lcd.setCursor(0, 0);
    snprintf(buffer, MAX_BUFFER_LEN, "O %2d%%  ", currAngle);
    lcd.print(buffer);
    lcd.setCursor(6, 0);
    snprintf(
        buffer,
        MAX_BUFFER_LEN,
        "B %4.1f\xDF%c%c%c",
        (double) boilerTemp,
        config.circuitRelayForced == 0 ? ' ' : '>',
        config.circuitRelayForced != 0 ? yesOrNo(config.circuitRelayForced == 1) : yesOrNo(circuitRelay),
        yesOrNo(heatNeeded)
    );
    lcd.print(buffer);
}

void printStatusOverviewBottom()
{
    lcd.noCursor();
    lcd.setCursor(0, 1);
    // 2 + 4 + 2 = 8 chars
    snprintf(buffer, MAX_BUFFER_LEN, "H %2d%% ", lround((float(pidRelayAtStartOfWindow) / relayWindowFragments) * 99));
    lcd.print(buffer);
    // 2 + 4 + 1 = 7 chars
    snprintf(buffer, MAX_BUFFER_LEN, "R %4.1f\xDF", (double)roomTemp);
    lcd.print(buffer);
    if (heatNeededOverride != 0) {
        lcd.print(("  \x5E"));
    } else {
        lcd.print(("   "));
    }
}

void printStatusOverview()
{
    printStatusOverviewTop();
    printStatusOverviewBottom();
}

void printStatus()
{
    if (config.settingsSelected != settingsSelectedPrint) {
        lcd.clear();
    }
    if (config.settingsSelected >= 0) {
        printSettings();
    } else {
        printStatusOverview();
    }
    settingsSelectedPrint = config.settingsSelected;
}

#define MAX_MENU_ITEMS MENU_STATIC_ITEMS // (MENU_STATIC_ITEMS + (config.curveItems * 2) + 1)

bool processSettings()
{
    bool stateChanged = false;
    if (readButton(&btn1) == 1) {
        stateChanged = true;
        config.settingsSelected = config.settingsSelected - 1;
        if (config.settingsSelected == -2) {
            config.settingsSelected = MAX_MENU_ITEMS - 2;
        }
    }
    if (readButton(&btn2) == 1) {
        stateChanged = true;
        config.settingsSelected = config.settingsSelected + 1;
        if (config.settingsSelected == MAX_MENU_ITEMS - 1) {
            config.settingsSelected = -1;
        }
    }
    bool anyPressed;
    if (config.settingsSelected >= 0) {
        const ConfigMenuItem_t *currentItem = getMenu(config.settingsSelected);
        if (readButton(&btn3) == 1) {
            stateChanged = true;
            currentItem->handler(currentItem->param, -1);
        } else if (btn3.pressedFor > 10) {
            stateChanged = true;
            currentItem->handler(currentItem->param, -1);
        }

        if (readButton(&btn4) == 1) {
            stateChanged = true;
            currentItem->handler(currentItem->param, 1);
        } else if (btn4.pressedFor > 10) {
            stateChanged = true;
            currentItem->handler(currentItem->param, 1);
        }
        anyPressed = stateChanged;
    } else {
        anyPressed = stateChanged || readButton(&btn3) || readButton(&btn4);
    }
    if (anyPressed) {
        screenSaverWakeup();
    } else {
        if (noPressCycles >= 100) {
#if DEBUG_LEVEL > 1
            Serial.println("Backlight - on");
#endif
//            lcd.noBacklight();
            noPressCycles = 101;
        } else {
            noPressCycles++;
        }
    }
    return stateChanged;
}

#define EEPROM_MAGIC 0xDEADBE00

void eepromInit()
{
    uint32_t checkCode;
    EEPROM.get(0, checkCode);
    if (checkCode == EEPROM_MAGIC) {
        EEPROM.get(sizeof(checkCode), config);
		if (config.settingsSelected < -1) {
			config.settingsSelected = -1;
		} else if (config.settingsSelected > MAX_MENU_ITEMS) {
			config.settingsSelected = -1;
		}
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
//    EEPROM.put(offset, maxDeltaSettings);
//    offset += sizeof(maxDeltaSettings);
//    EEPROM.put(offset, maxDeltaHigh);
}

char yesOrNo(int input)
{
    return input != 0 ? '\xff' : '\xdb';
}

void screenSaverWakeup()
{
    if (noPressCycles == 101) {
#if DEBUG_LEVEL > 1
        Serial.println("Backlight - on");
#endif
        lcd.backlight();
    }
    noPressCycles = 0;
}
