#include "MeMegaPi.h"
#include <Wire.h>

Me7SegmentDisplay disp(PORT_6);

MeEncoderOnBoard Encoder_1(SLOT2);
MeEncoderOnBoard Encoder_2(SLOT3);

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
float setpoint = 8;
float error_sum;
float error_difference;
float error_prev;
float error;
float motorSpeedInput;
float integralLimit = 1000;

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

void loop()
{
  gyro.update();

  disp.display(gyro.getAngleY());

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

  if (motorSpeedInput < -200) {
    motorSpeedInput = -200;
  }
  else if (motorSpeedInput > 200) {
    motorSpeedInput = 200;
  }
  
  Encoder_1.setMotorPwm(-motorSpeedInput);
  Encoder_2.setMotorPwm(motorSpeedInput);
  Encoder_1.updateSpeed();
  Encoder_2.updateSpeed();
}
