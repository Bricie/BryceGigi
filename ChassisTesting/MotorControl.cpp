#include "esp32-hal-ledc.h"

#include "MotorControl.hpp"

//PWM Setting 
struct PWM_t{ 
uint32_t freq ;
uint8_t resolution; 
uint32_t dutycycle;
} PWM;



PWM_t PWM_Channel_0 = { 250, 10, 0 };
PWM_t PWM_Channel_1 = { 250, 10, 0 };
PWM_t PWM_Channel_2 = { 250, 10, 0 };
PWM_t PWM_Channel_3 = { 250, 10, 0 };
PWM_t PWM_Channel_4 = { 50, 10, 0 };


void Motor::Init(){
  /*Initialize the PWM Channels*/ 
  ledcSetup(PWM_CH0, PWM_Channel_0.freq, PWM_Channel_0.resolution);
  ledcSetup(PWM_CH1, PWM_Channel_1.freq, PWM_Channel_1.resolution);
  ledcSetup(PWM_CH2, PWM_Channel_2.freq, PWM_Channel_2.resolution);
  ledcSetup(PWM_CH3, PWM_Channel_3.freq, PWM_Channel_3.resolution);

  /*Attach the Pins to the PWM Channels*/
  ledcAttachPin(Motor_R_IN1, PWM_CH0);
  ledcAttachPin(Motor_R_IN2, PWM_CH1);
  ledcAttachPin(Motor_L_IN1, PWM_CH2);
  ledcAttachPin(Motor_L_IN2, PWM_CH3);

  /*Initialize the PWM DutyCycle to 0% */
  ledcWrite(PWM_CH0, 0);
  ledcWrite(PWM_CH1, 0);
  ledcWrite(PWM_CH2, 0);
  ledcWrite(PWM_CH3, 0);
  
  /*Debug Message*/
  //Serial.println("PWM Channel and DutyCycle for Motors Initialized.");

};

void Servo::Init(){
  /*Initialize the PWM Channel*/ 
  ledcSetup(PWM_CH4, PWM_Channel_4.freq, PWM_Channel_4.resolution);

  /*Attach the Pins to the PWM Channel*/
  ledcAttachPin(Servo_Pin, PWM_CH4);

  /*Initialize the PWM DutyCycle to 0% */
  ledcWrite(PWM_CH4, 0);

  /*Debug Message*/
  //Serial.println("Servo Motor Initialized.");
}
/*For SG90 Servo Motor
  PWM         --> 50Hz  (20ms)
  Dutycycle   --> 1-2ms (5-10%)*/
void Servo::TrunDeg(uint16_t Degree){
  uint16_t Dutycycle = (float(Degree) / 90.0f) * 51.2f + 25.0f ;
  ledcWrite(PWM_CH4, Dutycycle);
  /*For Debug*/
  //Serial.print("Servo Degree: ");
  //Serial.println(Degree);
  //Serial.print("Dutycycle: ");
  //Serial.println(Dutycycle);
  delay(10);
};

void Motor::Moving_Clockwise(uint16_t Speed, uint8_t Wheel){
  if(Wheel == 1 ){
    ledcWrite(PWM_CH0, Speed);
    ledcWrite(PWM_CH1, 0);
    delay(1);
  }
  if(Wheel == 2){
    ledcWrite(PWM_CH2, Speed);
    ledcWrite(PWM_CH3, 0);
    delay(1);
  }
};

void Motor::Moving_AntiClockwise(uint16_t Speed, uint8_t Wheel){
  if(Wheel == 1 ){
    ledcWrite(PWM_CH0, 0);
    ledcWrite(PWM_CH1, Speed);
    delay(10);
  }

  if(Wheel == 2){
    ledcWrite(PWM_CH2, 0);
    ledcWrite(PWM_CH3, Speed);
    delay(10);
  }
};

void Motor::Stop(){
  //Enable Both IN1 and IN2 to HIGH to Stop the Wheel 
    ledcWrite(PWM_CH0, 1024);
    ledcWrite(PWM_CH1, 1024);
    ledcWrite(PWM_CH2, 1024);
    ledcWrite(PWM_CH3, 1024);
    delay(10);
  
};

/*To Find the Relationship between RPM and PWM to adjust the PWM using Target RPM*/
float Motor::RPMtoPWM(float TargetRPM, uint8_t Wheel){
    float TargetPWM = 0.0f;
    float RPM = 25.0f; 
    /*Be Awared of 2 Motor may have a different PWM and RPM ratio*/
    switch (Wheel)
    {
    case LeftWheel:
    /*Find the math relationship
      it's not a linear relationship 
      But can make the estimate value by 2 - 3 range and apply linear estimation*/
    TargetPWM = ((RPM)*1024)/100;

    if(TargetPWM > 1024.0f)
    TargetPWM = 1024.0f;

    return TargetPWM;


    case RightWheel:
    TargetPWM = (RPM*1024)/100;
    
    if(TargetPWM > 1024.0f)
    TargetPWM = 1024.0f;

    return TargetPWM;
    }

}

void Motion::Forwards(uint16_t LeftSpeed, uint16_t RightSpeed){

  Motor::Moving_AntiClockwise(LeftSpeed , LeftWheel);
  Motor::Moving_AntiClockwise(RightSpeed, RightWheel);
  Servo::TrunDeg(90);
};

void Motion::Backwards(uint16_t LeftSpeed, uint16_t RightSpeed){
  Motor::Moving_Clockwise(LeftSpeed, LeftWheel);
  Motor::Moving_Clockwise(RightSpeed, RightWheel);

  Servo::TrunDeg(90);
};  

void Motion::Rightwards(uint16_t LeftSpeed, uint16_t RightSpeed){
  Motor::Moving_Clockwise(RightSpeed, RightWheel );
  Motor::Moving_AntiClockwise(LeftSpeed, LeftWheel);
  Servo::TrunDeg(135);
  
};

void Motion::Leftwards(uint16_t LeftSpeed, uint16_t RightSpeed){
  Motor::Moving_AntiClockwise(RightSpeed, RightWheel);
  Motor::Moving_Clockwise(LeftSpeed, LeftWheel);
  Servo::TrunDeg(45);
};




