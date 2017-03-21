#ifndef INFO_H__
#define INFO_H__
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <Arduino.h>

class Info{
private:
    const static int8_t MAX_NUM_DATA = 20;
    byte    commands[20];
    byte    numbers[20];
    int8_t  n_cmd;

public:
    Info():n_cmd(0){}

    void add(byte cmd, byte num);

    void print();
    void clear();
};

#endif // INFO_H__

