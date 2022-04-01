
void menuFormatterUInt8Value(__attribute__((unused)) void *param, Print &print, void *value)
{
    lcd.cursor();
    print.print(F("value: "));
    snprintf(buffer, MAX_BUFFER_LEN, "%8d", *(uint8_t *) value);
    print.print(buffer);
    buffer[0] = '\0';
}

void menuFormatterInt16Value(__attribute__((unused)) void *param, Print &print, void *value)
{
    lcd.cursor();
    print.print(F("value: "));
    snprintf(buffer, MAX_BUFFER_LEN, "%8d", *(int16_t *) value);
    print.print(buffer);
    buffer[0] = '\0';
}

__attribute__((unused)) void menuFormatterInt8Value(__attribute__((unused)) void *param, Print &print, void *value)
{
    lcd.cursor();
    print.print(F("value: "));
    snprintf(buffer, MAX_BUFFER_LEN, "%8d", *(int8_t *) value);
    print.print(buffer);
    buffer[0] = '\0';
}

void menuFormatterFloatValue(__attribute__((unused)) void *param, Print &print, void *value)
{
    lcd.cursor();
    print.print(F("value: "));
    snprintf(buffer, MAX_BUFFER_LEN, "%8.3f", (double)*(float*)value);
    print.print(buffer);
    buffer[0] = '\0';
}

void menuFormatterCircuitOverride(__attribute__((unused)) void *param, Print &print, void *value)
{
    switch (*(int8_t *) value) {
        case 0:
            print.print(F("no override"));
            break;
        case 1:
            print.print(F("always enabled"));
            break;
        case 2:
            print.print(F("always disabled"));
            break;
    }
}