#include "info.h"

void    Info::print()
{
    for (int8_t i = 0; i < n_cmd; i++)
    {
        Serial.print(commands[i]);
        Serial.println(numbers[i]);

    }
    clear();
}

void    Info::add(byte cmd, byte num)
{
    commands[n_cmd] = cmd;
    numbers[n_cmd] = num;
    n_cmd++;
}

void    Info::clear()
{
    n_cmd = 0;
}