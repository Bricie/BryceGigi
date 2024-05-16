#include "esp32-hal-gpio.h"
#include "IRSensing.hpp"

/*Initialize all the IR sensor Data Pins*/
void IR::Init(){
  pinMode(IR_L,INPUT);
  pinMode(IR_M,INPUT);
  pinMode(IR_R,INPUT);
};

/*For the Logic
  0: White 
  1: Black*/
uint8_t IR::Tracking(){
  //M_IR on Track
  if(digitalRead(IR_M) && !(digitalRead(IR_L)) && !(digitalRead(IR_R)))
    return OnTrack;
  //R_IR on Track 
  if((!digitalRead(IR_M) && (!digitalRead(IR_L)) && (digitalRead(IR_R))) || (digitalRead(IR_M) && (!digitalRead(IR_L)) && (digitalRead(IR_R))))
    return IR_ROnTrack;  
  //L_IR on Track 
  if(!digitalRead(IR_M) && (digitalRead(IR_L)) && !(digitalRead(IR_R)) || (digitalRead(IR_M) && (digitalRead(IR_L)) && !(digitalRead(IR_R))))
    return IR_LOnTrack;
  //All on Track
  if(digitalRead(IR_M) && (digitalRead(IR_L)) && (digitalRead(IR_R)))
    return AllOnTrack;

  return OutOfTrack; 
};

