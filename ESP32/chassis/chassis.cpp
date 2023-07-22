#include "chassis.h"

chassis::chassis(motor* fl, motor* fr, motor* bl, motor* br, bool isTypeX){
    this->fl = fl;
    this->fr = fr;
    this->bl = bl;
    this->br = br;
    this->isTypeX = isTypeX;
}

void chassis::setType(bool isTypeX){
    this->isTypeX = isTypeX;
}

void chassis::move(int x, int y, int w){
    int frontLeftMotor;
    int backLeftMotor;
    int frontRightMotor;
    int backRightMotor;

    if(isTypeX){
        //type X algorithm
        frontLeftMotor = y + x + w;
        backLeftMotor = y - x + w;
        frontRightMotor = y - x - w;
        backRightMotor = y + x - w;
    }
    else{
        //type O algorithm
        //write it by yourself

        //model ans
        frontLeftMotor = y - x + w;
        backLeftMotor = y + x + w;
        frontRightMotor = y + x - w;
        backRightMotor = y - x - w;
    }

    //set direction
    if(frontLeftMotor > 0){
        fl->setDirection(true);
    }
    else{
        fl->setDirection(false);
    }
    //set the speed
    fl->setSpeed(abs(frontLeftMotor));

    //set direction
    if(frontRightMotor > 0){
        fr->setDirection(true);
    }
    else{
        fr->setDirection(false);
    }
    //set the speed
    fr->setSpeed(abs(frontRightMotor));

    //set direction
    if(backLeftMotor > 0){
        bl->setDirection(true);
    }
    else{
        bl->setDirection(false);
    }
    //set the speed
    bl->setSpeed(abs(backLeftMotor));

    //set direction
    if(backRightMotor > 0){
        br->setDirection(true);
    }
    else{
        br->setDirection(false);
    }
    //set the speed
    br->setSpeed(abs(backRightMotor));
}