#include "lifter.h"

lifter::lifter(motor* lifter){
    this->lifter_motor = lifter;
}

void lifter::set_num_channel(int num){
    num_channel = num;
}

void lifter::liftUp(){
    lifter_motor->setDirection(true);  //based on your setting, you may need to change it to true or false
    lifter_motor->setSpeed(30);   //based on your setting, you may need to adjust the value. It should be between in 0 to 100.
}

void lifter::liftDown(){
    lifter_motor->setDirection(false);  //based on your setting, you may need to change it to true or false
    lifter_motor->setSpeed(30);   //based on your setting, you may need to adjust the value. It should be between in 0 to 100.
}

void lifter::liftStop(){
    //stop the motor
    lifter_motor->setSpeed(0);
}