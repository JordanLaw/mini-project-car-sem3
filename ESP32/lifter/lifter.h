#include <Arduino.h>
#include "motor.h"

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#ifndef LIFTER_H
#define LIFTER_H

class lifter{
    private:
        motor* lifter_motor; 
        uint8_t upper_limited_switch;
        uint8_t bottom_limited_switch;

        int num_channel;

    public:
        lifter(motor* lifter);

        void set_num_channel(int num);

        void liftUp();
        void liftDown();
        void liftStop();
};

#endif