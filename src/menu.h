void *menuHandlerVent(__attribute__((unused)) void *param, int8_t diff);
void *menuHandlerHeating(__attribute__((unused)) void *param, int8_t diff);
void *menuHandlerBoiler(__attribute__((unused)) void *param, int8_t diff);
void *menuHandlerBoilerIdle(__attribute__((unused)) void *param, int8_t diff);
void *menuHandlerRoom(__attribute__((unused)) void *param, int8_t diff);
void *menuHandlerRoomTempAdjust(__attribute__((unused)) void *param, int8_t diff);
void *menuHandlerDebounceLimitC(__attribute__((unused)) void *param, int8_t diff);
void *menuHandlerCircuitRelayForced(__attribute__((unused)) void *param, int8_t diff);
void *menuHandlerServoMin(__attribute__((unused)) void *param, int8_t diff);
void *menuHandlerServoMax(__attribute__((unused)) void *param, int8_t diff);
void *menuHandlerOverheatingLimit(__attribute__((unused)) void *param, int8_t diff);
void *menuHandlerDeltaTempPoly0(__attribute__((unused)) void *param, int8_t diff);
void *menuHandlerDeltaTempPoly1(__attribute__((unused)) void *param, int8_t diff);
void *handlePIDValueConfig(float *floatVal, int8_t diff);
void *menuHandlerPID_p(__attribute__((unused)) void *param, int8_t diff);
void *menuHandlerPID_i(__attribute__((unused)) void *param, int8_t diff);
void *menuHandlerPID_d(__attribute__((unused)) void *param, int8_t diff);
void *menuHandlerRelayPID_p(__attribute__((unused)) void *param, int8_t diff);
void *menuHandlerRelayPID_i(__attribute__((unused)) void *param, int8_t diff);
void *menuHandlerRelayPID_d(__attribute__((unused)) void *param, int8_t diff);

void menuFormatterUInt8Value(__attribute__((unused)) void *param, Print &print, void *value);
void menuFormatterInt16Value(__attribute__((unused)) void *param, Print &print, void *value);
void menuFormatterFloatValue(__attribute__((unused)) void *param, Print &print, void *value);
void menuFormatterCircuitOverride(__attribute__((unused)) void *param, Print &print, void *value);

const char menuManual[] PROGMEM = "Manual %";
const char menuManualHeating[] PROGMEM = "Manual H %";
const char menuBoilerTemp[] PROGMEM = "Boiler \xDF";
const char menuRoomTemp[] PROGMEM = "Room \xDF";
const char menuCircuitRelay[] PROGMEM = "Circuit Relay";
const char menuRoomTempAdj[] PROGMEM = "[E] Room Temp adj.";
const char menuServoMin[] PROGMEM = "[E] Servo Min";
const char menuServoMax[] PROGMEM = "[E] Servo Max";
const char menuBoilerIdle[] PROGMEM = "[E] Boiler Idle";
const char menuDebounce[] PROGMEM = "[E] T. Debounce";
const char menuOverheating[] PROGMEM = "[E] Overheating\xDF";
const char menuDeltaTp1[] PROGMEM = "[E] deltaT p1";
const char menuDeltaTp0[] PROGMEM = "[E] deltaT p0";
const char menuPIDp[] PROGMEM = "[E] BoilPID K_p";
const char menuPIDi[] PROGMEM = "[E] BoilPID K_i";
const char menuPIDd[] PROGMEM = "[E] BoilPID K_d";
const char menuRelayPIDp[] PROGMEM = "[E] RelPID K_p";
const char menuRelayPIDi[] PROGMEM = "[E] RelPID K_i";
const char menuRelayPIDd[] PROGMEM = "[E] RelPID K_d";

#define MENU_STATIC_ITEMS 20
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
                .name = menuBoilerIdle,
                .param = nullptr,
                .handler = &menuHandlerBoilerIdle,
                .formatter = &menuFormatterUInt8Value
        },
        {
                .name = menuDebounce,
                .param = nullptr,
                .handler = &menuHandlerDebounceLimitC,
                .formatter = &menuFormatterFloatValue
        },
        {
                .name = menuOverheating,
                .param = nullptr,
                .handler = &menuHandlerOverheatingLimit,
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

const struct ConfigMenuItem *getMenu(int16_t itemIndex);