#define MENU_POS_VENT_MANUAL 0
#define MENU_POS_HEAT_MANUAL 1
#define MENU_POS_SERVO_MIN 6
#define MENU_POS_SERVO_MAX 7
#define MENU_STATIC_ITEMS 18

typedef struct ConfigMenuItem {
    const char *name;
    void* param;
    const void* (*handler)(const void* param, int8_t diff);
    void (*formatter)(const void* param, Print &print, const void *value);
} ConfigMenuItem_t;

const struct ConfigMenuItem *getMenu(int16_t itemIndex);