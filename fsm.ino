typedef enum {
    Follow_state = 0,       // The initial state, robot should be following line
    Grab_state = 1,         // The state when some obstacle is detected, grab obstacle in this state
    Finished_state = 2      // The finish line is detected, stop the robot in this state
} State_type;
 
void setup() {
    // Place your init code here
 
    Serial.begin(115200);
 
}
 
 
// Implement the line following routine here, exit this function when either:
// The ultrasonic sensor detects an obstacle, returning Grab_state   OR
// The finish line is detected, returning Finished_state
State_type followFunc() {
 
    Serial.println("Entering Line Following State!");
 
    while (true) {
 
        // Detect if an obstacle is in front of the robot, return Grab_state if so
 
        // Detect if the robot has reached the finish line, return Finished_state if so
 
        // Perform PID line following 
 
    }
    
}
 
// Implement the grabbing routine here, exit this function when the robot
// is finished grabbing the obstacle in front of it, and return Follow_state
State_type grabFunc() {
 
    Serial.println("Entering Grabbing State!");
 
    // Perform grabbing action here
 
    return Follow_state;
    
}
 
// Implement a routine that halts the robot, this state does not lead to other states
State_type stopFunc() {
 
    Serial.println("Entering Stop State!");
 
    // Stop the motors here
 
    while(true) {}  // Code execution will be stuck here after stopping the robot
    
}
 
void loop() {
 
    State_type robotState = Follow_state;
 
    while (true) {
        switch (robotState) {
            case Follow_state:
                robotState = followFunc();
                break;
 
            case Grab_state:
                robotState = grabFunc();
                break;
 
            case Finished_state:
                robotState = stopFunc();
                break;
            
            default:
                Serial.println("ERROR! UNKNOWN STATE!");
                break;
            }
    }
 
}