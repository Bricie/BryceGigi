class PID
{
  float Kp, Ki, Kd;
  float error, last_error;
  float integral;
  float out;
  float int_upper;
 
  public:
  PID(float Kp, float Ki, float Kd, float int_upper) : Kp(Kp), Ki(Ki), Kd(Kd), int_upper(int_upper){}
  
  void update(float state, float desired_state)
  {
    last_error = error;
    error = desired_state - state;
    integral += error;
    
    float i_term = Ki * integral;
    if (fabs(i_term) > int_upper) i_term = i_term > 0 ? int_upper : -int_upper;
    
    out = Kp * error + i_term + Kd * (error - last_error);
  }
  float get_control()
  {
    return out;
  }
};
 
const float control_freq = 20;
float last_time;
PID PID_controller(1.0,0.0,0.0,20);
void setup() {
    Serial.begin(115200);
}
 
/*
 * Task 1: Copy the code from Lab1 and return your current position from 0 - 1000(If you cannot understand this comment, please refer back to Lab1 manual)
 */
float obtain_position_fromIR()
{
  //TODO
  
  return 0;
}
 
 
/*
 * Task 2: Copy the code from Lab4
 */
void motorDrive(int power, pin_size_t in1, pin_size_t in2) {
  
  //TODO
  
}
 
/*
 * Task 3: Finish the main control flow
 */
void loop() {
  float dt = (micros() - last_time)/ 1.0e6;
  if (dt > 1.0/control_freq)
  {
    float whereAmI = obtain_position_fromIR();
    float whereDoIWantToGo = 0;                      //TODO
 
    PID_controller.update(0, 0);                     //TODO 
 
    float delta_control = PID_controller.get_control();
    
    float constant_speed = 0;                        //TODO: Default Cruising speed
 
    //Differential Drive
    motorDrive(constant_speed + delta_control, 0, 0) //TODO
    motorDrive(constant_speed - delta_control, 0, 0) //TODO
    
  }
}