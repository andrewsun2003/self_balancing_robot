#include "MeMegaPi.h"
#include <Wire.h>

Me7SegmentDisplay disp(PORT_6);

MeEncoderOnBoard Encoder_1(SLOT2);
MeEncoderOnBoard Encoder_2(SLOT3);

MeUltrasonicSensor ultraSensor(PORT_7);


void isr_process_encoder1(void)
{
  if(digitalRead(Encoder_1.getPortB()) == 0)
  {
    Encoder_1.pulsePosMinus();
  }
  else
  {
    Encoder_1.pulsePosPlus();;
  }
}

void isr_process_encoder2(void)
{
  if(digitalRead(Encoder_2.getPortB()) == 0)
  {
    Encoder_2.pulsePosMinus();
  }
  else
  {
    Encoder_2.pulsePosPlus();
  }
}


MeGyro gyro;

float kp = 38;
float ki = 1.5;
float kd = 0.05;
float forward = 9.7;
float stop = 8;
float setpoint = forward;
float error_sum;
float error_difference;
float error_prev;
float error;
float motorSpeedInput;
float integralLimit = 1000;
float time_elapsed;
float time_prev = 0;
float time = 0;
float distance = 400;
char direction[] = "RLLLLRRLLR";
int set_angle[] = {-90, 0, 98, 177, -107, -177, 103, 175, -95, -177};
int prev_angle = 0;
int i = 0;


void setup()
{
  attachInterrupt(Encoder_1.getIntNum(), isr_process_encoder1, RISING);
  attachInterrupt(Encoder_2.getIntNum(), isr_process_encoder2, RISING);
  
  //Set PWM 8KHz
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);

  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);

  gyro.begin();
}


void turn(char *array, int *pointer, int angle[]) {
  if (array[*pointer] == 'R') {
    setpoint = stop;
    Encoder_1.setMotorPwm(-motorSpeedInput - 35);
    Encoder_2.setMotorPwm(motorSpeedInput - 35);
    Encoder_1.updateSpeed();
    Encoder_2.updateSpeed();
    if (abs(prev_angle) > abs(angle[*pointer])) {
      if (abs(gyro.getAngleZ()) < abs(angle[*pointer] + 1)) {
      prev_angle = angle[*pointer];
      (*pointer)++;
      setpoint = forward;
    }
    }
    else {
      if (abs(gyro.getAngleZ()) > abs(angle[*pointer]) - 5) {
      prev_angle = angle[*pointer];
      (*pointer)++;
      setpoint = forward;
    }
    }
  }
  else if (array[*pointer] == 'L') {
    setpoint = stop;
    Encoder_1.setMotorPwm(-motorSpeedInput + 35);
    Encoder_2.setMotorPwm(motorSpeedInput + 35);
    Encoder_1.updateSpeed();
    Encoder_2.updateSpeed();
    if (abs(prev_angle) > abs(angle[*pointer])) {
      if (abs(gyro.getAngleZ()) < abs(angle[*pointer] + 1)) {
      prev_angle = angle[*pointer];      
      (*pointer)++;
      setpoint = forward;
    }
    }
    else {
      if (abs(gyro.getAngleZ()) > abs(angle[*pointer]) - 5) {
      prev_angle = angle[*pointer];
      (*pointer)++;
      setpoint = forward;
    }
    }
  }
}


void loop()
{
  gyro.update();
  disp.display(gyro.getAngleZ());


  error = gyro.getAngleY() - setpoint;
  error_sum += error;
  error_difference = (error - error_prev);
  error_prev = error;
  
  if (error_sum > integralLimit){
    error_sum = integralLimit;
  }
   if (error_sum < -integralLimit){
    error_sum = -integralLimit;
  }

  motorSpeedInput = kp*error + ki*error_sum + kd*(error_difference);


  time = millis();
  time_elapsed = (time - time_prev)/1000;

  if (time_elapsed > 0.3) {
    if (setpoint == stop) {
      distance = distance;
    }
    else {
      distance = ultraSensor.distanceCm();
      time_prev = time;
    }
  }


  if (distance < 25 && direction[i] != '\0') {
    if (motorSpeedInput < -200) {
      motorSpeedInput = -200;
    }
    else if (motorSpeedInput > 200) {
      motorSpeedInput = 200;
    }
    turn(direction, &i, set_angle);
  }
  else {
    if (motorSpeedInput < -200) {
      motorSpeedInput = -200;
    }
    else if (motorSpeedInput > 200) {
      motorSpeedInput = 200;
    }

    if (gyro.getAngleZ() > prev_angle) {
      Encoder_1.setMotorPwm(-motorSpeedInput - 10);
      Encoder_2.setMotorPwm(motorSpeedInput - 10);
      Encoder_1.updateSpeed();
      Encoder_2.updateSpeed();
    }
    else if (gyro.getAngleZ() < prev_angle) {
      Encoder_1.setMotorPwm(-motorSpeedInput + 10);
      Encoder_2.setMotorPwm(motorSpeedInput + 10);
      Encoder_1.updateSpeed();
      Encoder_2.updateSpeed();
    }
    else {
      Encoder_1.setMotorPwm(-motorSpeedInput);
      Encoder_2.setMotorPwm(motorSpeedInput);
      Encoder_1.updateSpeed();
      Encoder_2.updateSpeed();
    }
  }
}
