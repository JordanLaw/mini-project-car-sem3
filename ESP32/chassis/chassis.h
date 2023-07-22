#include <Arduino.h>
#include "motor.h"

#ifndef CHASSIS_H
#define CHASSIS_H

class chassis{
    private:
        motor* fl;
        motor* fr;
        motor* bl;
        motor* br;

        bool isTypeX = true;

    public:
        chassis(motor* fl, motor* fr, motor* bl, motor* br, bool isTypeX);

        void setType(bool isTypeX);

        void move(int x_speed, int y_speed, int w_speed);
};

#endif