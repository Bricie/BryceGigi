#include "LineTracking.hpp"
#include "IRSensing.hpp"


void LineTracking::FollowingLine( uint8_t Case, uint16_t LeftSpeed, uint16_t RightSpeed ){
  switch (Case)
  {
  case OnTrack:
    Motion::Forwards(LeftSpeed, RightSpeed);
    delay(10);
  break;

  case IR_LOnTrack:

    while(IR::Tracking() != OnTrack && IR::Tracking() != IR_ROnTrack){
      Motion::Leftwards(LeftSpeed*1.5, RightSpeed*1.5);
    }


  break;

  case IR_ROnTrack:
    while(IR::Tracking() != OnTrack && IR::Tracking() != IR_LOnTrack){
      Motion::Rightwards(LeftSpeed*1.5, RightSpeed*1.5);
    }


  break;

  case AllOnTrack:
    Motion::Forwards(LeftSpeed, RightSpeed);
    delay(20);
  break;
  
  case OutOfTrack:
    Motion::Forwards(LeftSpeed, RightSpeed);
    delay(10);

  break;


  }



};