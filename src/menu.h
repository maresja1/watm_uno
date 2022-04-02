#include "menu_handlers.h"
#include "menu_formatters.h"

const char menuManual[] PROGMEM = "Manual %";
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

#define MENU_STATIC_ITEMS 19
const ConfigMenuItem_t menu[] = {
        {
                .name = menuManual,
                .param = nullptr,
                .handler = &menuHandlerGate,
                .formatter = &menuFormatterUInt8Value
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
