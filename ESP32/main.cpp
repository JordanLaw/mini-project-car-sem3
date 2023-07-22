#include <Arduino.h>
#include "pinmap.h"
#include "motor.h"
#include "chassis.h"
#include "msg.h"
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "lifter.h"

motor motor1(motor1_pwm, motor1_CWCCW);
motor motor2(motor2_pwm, motor2_CWCCW);
motor motor3(motor3_pwm, motor3_CWCCW);
motor motor4(motor4_pwm, motor4_CWCCW);

motor motor5(motor5_pwm, motor5_CWCCW);
lifter superLifter(&motor5);

int num_channel = 0;  //according to your setting to adjust it
int close_pulse_value = 300;  //according to your setting to adjust it
int open_pulse_value = 600;   //according to your setting to adjust it
Adafruit_PWMServoDriver servo_driver(0x40);

//based on your setting, un-comment one

//motor 1: front left
//motor 2: front right
//motor 3: back left
//motor 4: back right

//type X
chassis superCar(&motor1, &motor2, &motor3, &motor4, true);
//type O
//chassis superCar(&motor1, &motor2, &motor3, &motor4, false);  

msg nanoMsg;

void liftStop(){
  superLifter.liftStop();
}

void openClip(){
    //the pattern of on and off is represent to the angle position of servo
    //based on your setting, you may need to adjust the value
    servo_driver.setPWM(num_channel, 0, open_pulse_value);   
}

void closeClip(){
    //the pattern of on and off is represent to the angle position of servo
    //based on your setting, you may need to adjust the value
    servo_driver.setPWM(num_channel, 0, close_pulse_value);    
}

void setup() {
  pinMode(upper_switch, INPUT_PULLDOWN);
  pinMode(bottom_switch, INPUT_PULLDOWN);

  attachInterrupt(upper_switch, liftStop, RISING);
  attachInterrupt(bottom_switch, liftStop, RISING);

  servo_driver.begin();
  servo_driver.setPWMFreq(50);

  //based on your setting, un-comment one 

  //setting 1
  //motor1.setReversed(true);
  //motor3.setReversed(true);

  //setting 2
  //motor2.setReversed(true);
  //motor4.setReversed(true);

  //based on your setting, un-comment one

  //for debug / simulate
  //nanoMsg.init(&Serial);

  //for jetson nano
  //nanoMsg.init(&Serial2);
}

void loop() {
  if(!nanoMsg.read()){
    return;
  }

  //chassis move
  superCar.move(nanoMsg.getX_speed(), nanoMsg.getY_speed(), nanoMsg.getW_speed());

  //clip
  if(nanoMsg.get_isCloseClip()){
    closeClip();
  }
  else{
    openClip();
  }

  //lifter
  switch(nanoMsg.getLifting_status()){
    case 0x00:
      //up
      if(!digitalRead(upper_switch)){
        superLifter.liftUp();

        //to avoid bouncing and trigger the safety stop, you may need to adjust the delay time
        delay(500);
      }
      else{
        superLifter.liftStop();
      }
      break;

    case 0x01:
      //down
      if(!digitalRead(bottom_switch)){
        superLifter.liftDown();

        //to avoid bouncing and trigger the safety stop, you may need to adjust the delay time
        delay(500);
      }
      else{
        superLifter.liftStop();
      }
      break;

    case 0x02:
      //stop
      superLifter.liftStop();
      break;

    default:
      //for safety, it will stop if the limit switches are triggered.
      if(digitalRead(upper_switch) | digitalRead(bottom_switch)){
        superLifter.liftStop();
      }
      break;
  }
}
