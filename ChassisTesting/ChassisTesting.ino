#include "Pinout.hpp"
#include "esp32-hal-ledc.h"


// Include Arduino FreeRTOS library
// #include "Arduino_FreeRTOS.h"

/*Include the FreeRTOS library*/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

/*Pre-Set Data*/
volatile float LeftSpeed = 220.0f;   // 0.0f - 1024.0f
volatile float RightSpeed = 220.0f;  // 0.0f - 1024.0f

/*-------------------------------------------------------------------------------------------------------------------------*/
/*-------------RFID Reader Task-------------*/
/*--------Stack and Handle Settings---------*/

StackType_t uxRFIDTagReader[configMINIMAL_STACK_SIZE];
StaticTask_t xRFIDTagReaderTCB;
TaskHandle_t RFIDTagReaderTCB;

/*Creating the Class for RFID Reader*/
MFRC522 mfrc522(0x28, RFID_RST);

/*Function for getting the RFID Tag ID Number*/
String getTagUID() {
  String tagUID = "";
  for (byte i = 0; i < mfrc522.uid.size; i++) {
    tagUID += mfrc522.uid.uidByte[i] < 0x10 ? "0" : "";
    tagUID += String(mfrc522.uid.uidByte[i], HEX);
  }
  return tagUID;
};


    // Read data from one of the IMU sensors (e.g., S1)
    /*
    digitalWrite(S1, LOW); // Select IMU

    byte data = SPI.transfer(0); // Read data

    digitalWrite(S1, HIGH); // Deselect 

    Serial.print("Data from IMU: ");

    Serial.println(data, HEX); 

    delay(1000); 
  */


/*Define a Storage to store the previous tagUID*/
String prevRFIDTag;

/*User Task for RFID Tag Reader*/
void RFIDTagReader(void *pvPara) {
  /*Setup for the task*/

  /*Do*/
  while (true) {

    /*DONT CHANGE THE FOLLOWING 2 LINE OF CODE*/
    if (!mfrc522.PICC_IsNewCardPresent() || !mfrc522.PICC_ReadCardSerial())
      vTaskDelay(50);

    /*You may want to setup a check new UID to save procress power
      Comparing the PrevUID and CurrentUID...*/

    String currenttagUID = getTagUID();

    if(currenttagUID != prevRFIDTag){
      Serial.print("RFID Tag: ");
      Serial.println(currenttagUID);
      prevRFIDTag = currenttagUID; 
      //vTaskDelay(500); 
    }


    /*----------------------------------------------------*/
    // FOR DEBUG USAGE
    // Serial.print("RFID Tag: ");
    // Serial.println(currenttagUID);
    /*----------------------------------------------------*/

    /*A delay must be added inside each User Task*/

    vTaskDelay(10);
  }
}



/*-------------------------------------------------------------------------------------------------------------------------*/
/*------------FireBase Task------------*/
/*--------Stack and Handle Settings---------*/

StackType_t uxFireBaseTaskStack[configMINIMAL_STACK_SIZE];
StaticTask_t xFireBaseTaskTCB;
TaskHandle_t FireBaseTaskTCB;

#include <Arduino.h>
#include <WiFi.h>

#include <Firebase_ESP_Client.h>
// Wi-Fi network credentials
#define WIFI_SSID "connect if you can"  // Network SSID (name)
#define WIFI_PASSWORD "blahblah"        // Network password
                                                                                                                                                                                                                                           
// TO DO: Firebase configuration parameters
#define API_KEY "AIzaSyCWzx6g2Qmqdr-6lDt4pgjJJDBZBfAsFP0"                                        // Firebase project API key
#define DATABASE_URL "https://isdn-2602-iot-default-rtdb.asia-southeast1.firebasedatabase.app/"  // Firebase Realtime Database URL
#define USER_EMAIL "gg@g.com"                                                                    // Email for Firebase authentication
#define USER_PASSWORD "blahblahblah"                                                             // Password for Firebase authentication

// Initialize Firebase Data object
FirebaseData fbdo;

FirebaseAuth auth;
FirebaseConfig config;

void FireBaseTask(void *pvPara) {
  /*Setup for the task*/

  // TO DO: Initialize serial communication with 115200 baud rate
  Serial.begin(115200);

  // Connect to Wi-Fi network
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(300);  // Wait 300ms before retrying
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();

  Serial.printf("Firebase Client v%s\n\n", FIREBASE_CLIENT_VERSION);

  // Set up Firebase configuration
  config.api_key = API_KEY;
  auth.user.email = USER_EMAIL;
  auth.user.password = USER_PASSWORD;
  config.database_url = DATABASE_URL;
  //config.token_status_callback = tokenStatusCallback; // Callback function for token status

  // Configure network reconnection behavior
  Firebase.reconnectNetwork(true);

  // Configure SSL buffer sizes for BearSSL engine
  fbdo.setBSSLBufferSize(4096 /* Rx buffer size */, 1024 /* Tx buffer size */);

  // Set the maximum size of the Firebase response payload
  fbdo.setResponseSize(2048);

  // Initialize Firebase connection
  Firebase.begin(&config, &auth);

  // Set the number of decimal places for float values in Firebase
  Firebase.setDoubleDigits(5);

  // Set server response timeout
  config.timeout.serverResponse = 10 * 1000;

  int start_point = 11;
  int end_point = 5;
if (Firebase.ready()) {
    //String incomingText = Serial.readStringUntil('\n'); // Read until a newline character

    if (Firebase.RTDB.getInt(&fbdo, "/demo/start_point")) {
      start_point = fbdo.intData();
      Serial.printf("start: %d\n", start_point);
    } else {
      Serial.printf("Failed to get start_point, reason: %s\n", fbdo.errorReason().c_str());
    }

    if (Firebase.RTDB.getInt(&fbdo, "/demo/end_point")) {
      end_point = fbdo.intData();
      Serial.printf("end: %d\n", end_point);
    } else {
      Serial.printf("Failed to get end_point, reason: %s\n", fbdo.errorReason().c_str());
    }


    Serial.println();

    // Global scope
    int parameters[2] = { start_point, end_point };  // An array that contains two int values
    void *pvParameters = parameters;

    while (true) {
      // loop
    }
  }
}





/*-------------------------------------------------------------------------------------------------------------------------*/
/*------------Line Tracking Task------------*/
/*--------Stack and Handle Settings---------*/

StackType_t uxLineTrackingTaskStack[configMINIMAL_STACK_SIZE];
StaticTask_t xLineTrackingTaskTCB;
TaskHandle_t LineTrackingTaskTCB;


/*User Task for Line Tracking*/
void LineTrackingTask(void *pvPara) {
  /*Setup for the task*/

  /*DO*/
  while (true) {

    LineTracking::FollowingLine(IR::Tracking(), LeftSpeed, RightSpeed);

    /*A delay must be added inside each User Task*/
    vTaskDelay(10);
  }
}


  /*DO NOT Change the code below*/
  /*-------------------------------------------------------------------------------------------------------------------------*/
  /*PID Control for Motor
  PID Function and Settings
  Creating PID_t for Mult. PID Setting 
  PID 1 for LeftWheel
  PID 2 for RightWheel */
  struct PID_t {
    /*Creating the parameters for PID*/
    volatile float Kp;
    volatile float Ki;
    volatile float Kd;

    volatile float target_val;  // The target RPM
    float actual_val;           // Actual RPM Reading
    float err;                  // Error
    float err_last;
    float integral;

    /*General PID Function*/
    float PID_realize(float temp_val) {
      this->err = this->target_val - temp_val;

      this->integral += this->err;

      this->actual_val = this->Kp * this->err + this->Ki * this->integral + this->Kd * (this->err - this->err_last);

      this->err_last = this->err;

      return this->actual_val;
    }

  } PID;

  /*-------------------------------------------------------------------------------------------------------------------------*/
  /*-------------RPM Measure Task-------------*/
  /*--------Stack and Handle Settings---------*/
  StackType_t uxcalculateRPMTaskStack[configMINIMAL_STACK_SIZE];
  StaticTask_t xcalculateRPMTaskTCB;
  TaskHandle_t calculateRPMTaskTCB;

  /*Constants for Encoder
  Find out the encoder resolution by yourself */
  const int encoderResolution = 36;   // Number of pulses per revolution
  const unsigned long interval = 100;  // Time interval in milliseconds 100ms

  /*Encoder to RPM Function and Settings
  Creating RPMCounter_t for 2 Wheel Setting 
  */
  typedef struct RPMCounter_t {

    volatile int encoderPulses;
    unsigned long previousMillis;
    volatile float rpm;


    float RPMCounter() {
      unsigned long currentMillis = millis();

      // Check if the time interval has elapsed
      if (currentMillis - previousMillis >= interval) {
        // Calculate RPM
        float rotations = float(encoderPulses) / ((float)encoderResolution);
        float time = (currentMillis - previousMillis) / 100.0f;  // Convert to seconds
        float rpm = (rotations / time) * 60.0f;

        // Reset encoder pulse count and update previousMillis
        encoderPulses = 0;
        previousMillis = currentMillis;

        // Print RPM
        // Serial.println(rpm);
        vTaskDelay(100 / portTICK_PERIOD_MS);  // Delay for 0.1 second
        return rpm;
      }
    }
  } RPM;

  /*Define 2 Sets of Variables using RPMCounter_t for 2 Wheel 
  Init the RPM related Variables before the task starts   */
  RPMCounter_t LeftMotor = { 0, 0, 0 };
  RPMCounter_t RightMotor = { 0, 0, 0 };


  // Interrupt Service Routine
  // Create a struct to handle 2 motors encoder
  struct Encoder_t {
    int pinAState;
    int pinBState;
    int Encoder_A;
    int Encoder_B;

  } Encoder;

  /*Init the Enocoder related Variables before the task starts*/
  Encoder_t EncoderLeft = { 0, 0, Motor_L_Encoder_A, Motor_L_Encoder_B };
  Encoder_t EncoderRight = { 0, 0, Motor_R_Encoder_A, Motor_R_Encoder_B };

  /*-------------------------------------------------------------------------------------------------------------------------*/
  /*Interrupt Service Routine Function
  Since attachInterrupt() cannot using non Static function 
  Below are 2 IRAM_ATTR function for handle the interrupts for the encoder*/
  void IRAM_ATTR handleLeftEncoderInterrupt() {
    //init the local variable
    int change = 0;

    // Read the current state of the encoder pins
    EncoderLeft.pinAState = digitalRead(EncoderLeft.Encoder_A);
    EncoderLeft.pinBState = digitalRead(EncoderLeft.Encoder_B);

    // Determine the direction of rotation based on the phase change
    if (EncoderLeft.pinAState != EncoderLeft.pinBState) {
      change = (EncoderLeft.pinAState == HIGH) ? 1 : 0;
    } else {
      change = (EncoderLeft.pinAState == HIGH) ? 0 : 1;
    }

    // Update the encoder count
    LeftMotor.encoderPulses += change;
  }

  void IRAM_ATTR handleRightEncoderInterrupt() {
    //init the local variable
    int change = 0;

    // Read the current state of the encoder pins
    EncoderRight.pinAState = digitalRead(EncoderRight.Encoder_A);
    EncoderRight.pinBState = digitalRead(EncoderRight.Encoder_B);

    // Determine the direction of rotation based on the phase change
    if (EncoderRight.pinAState != EncoderRight.pinBState) {
      change = (EncoderRight.pinAState == HIGH) ? 1 : 0;
    } else {
      change = (EncoderRight.pinAState == HIGH) ? 0 : 1;
    }

    // Update the encoder count
    RightMotor.encoderPulses += change;
  }


  /*-------------------------------------------------------------------------------------------------------------------------*/
  /*-------------RPM Measure User Task-------------*/

  void calculateRPMTask(void *pvPara) {
    /*Setup for the Task*/
    /*----------------------------------------------------*/
    /*Define 2 sets PID for 2 Motors*/
    RPMCounter_t TargetRPM;
    /*Change the PID Para. here
    LeftMotor PID*/
    PID_t pid1 = { 0.7f  //Kp
                   ,
                   0.0f  //Ki
                   ,
                   0.0f };  //Kd

    /*RightMotor PID*/
    PID_t pid2 = { 0.3f  //Kp
                   ,
                   0.0f  //Ki
                   ,
                   0.0f };  //Kd

    /*Set the initial Target RPM Here*/
    pid1.target_val = 360.0f;
    pid2.target_val = 360.0f;
    /*----------------------------------------------------*/
    while (true) {
      /*----------------------------------------------------*/
      /*FOR DEBUG USAGE*/
      //Serial.println("RPM Left: ");
      //Serial.print(LeftMotor.RPMCounter());

      //Serial.println("RPM Right: ");
      //Serial.print(RightMotor.RPMCounter());
      /*----------------------------------------------------*/
      /*Setting the actual value to PID function*/
      pid1.actual_val = LeftMotor.RPMCounter();
      pid2.actual_val = RightMotor.RPMCounter();

      /*Compute the PID and Write the Result to Speed of the Wheel*/
      LeftSpeed = Motor::RPMtoPWM(pid1.PID_realize(LeftMotor.RPMCounter()), LeftWheel);
      RightSpeed = Motor::RPMtoPWM(pid1.PID_realize(RightMotor.RPMCounter()), RightWheel);

      /*----------------------------------------------------*/
      /*FOR DEBUG USAGE*/
  
      //Serial.print("Speed Left: ");
      //Serial.println(LeftSpeed);

      //vTaskDelay(500);

      //Serial.print("Speed Right: ");
      //Serial.println(RightSpeed);

    
      /*----------------------------------------------------*/
      /*A delay must be added inside each User Task*/
      vTaskDelay(10);
    }
  }




  /*DO NOT CHANGE THE CODE BELOW*/
  /*----------------------------------------------------*/
  /*------------LED Blinking Task-------------
  --------Stack and Handle Settings---------
To ensure there is visualization that the program is running*/
  StackType_t uxBlinkTaskStack[configMINIMAL_STACK_SIZE];
  StaticTask_t xBlinkTaskTCB;
  TaskHandle_t BlinkTaskTCB;



  void Blink(void *pvPara) {
    /*Setup for the task*/
    pinMode(LED1, OUTPUT);
    /*DO*/
    while (true) {
      digitalWrite(LED1, HIGH);
      vTaskDelay(100);
      digitalWrite(LED1, LOW);
      vTaskDelay(200);
    }
  }
  /*----------------------------------------------------*/
#include <limits.h>
//#include <iostream>

#define V 14

using namespace std;

int minDistance(int dist[], bool sptSet[]) {
  int min = INT_MAX, min_index;

  for (int v = 0; v < V; v++)
    if (sptSet[v] == false && dist[v] <= min)
      min = dist[v], min_index = v;

  return min_index;
}

int direction_now = 90;
bool isStringInRow(String array_2d[][3], int ROWS, int row_index, String target_string) {
  for (int i = 0; i < 3; i++) {
    if (array_2d[row_index][i] == target_string) {
      return true;
    }
  }
  return false;
}
#define MAX_PATH_SIZE 100

int path[MAX_PATH_SIZE];
int pathSize = 0;

void printPath(int parent[], int j) {
  if (parent[j] == -1) {
    path[pathSize++] = j;
    return;
  }

  printPath(parent, parent[j]);
  path[pathSize++] = j;
}

void dijkstra(int graph[][V], int start, int end) {
  int dist[V];
  bool sptSet[V];
  int parent[V];

  for (int i = 0; i < V; i++) {
    parent[start] = -1;
    dist[i] = INT_MAX;
    sptSet[i] = false;
  }

  dist[start] = 0;

  for (int count = 0; count < V - 1; count++) {
    int u = minDistance(dist, sptSet);
    sptSet[u] = true;

    for (int v = 0; v < V; v++)
      if (!sptSet[v] && graph[u][v] && dist[u] + graph[u][v] < dist[v]) {
        parent[v] = u;
        dist[v] = dist[u] + graph[u][v];
      }
  }

  pathSize = 0;
  printPath(parent, end);
  Serial.println("Shortest path from ");
  Serial.print(start);
  Serial.print("to");
  Serial.print(end);
  Serial.println(":   ");

  for (int i = 0; i < pathSize; i++)
    Serial.print(path[i]);
  Serial.print(",  ");
}



void dijkstraTask(void *pvParameters) {
  int start = *((int *)pvParameters);
  int end = *(((int *)pvParameters) + 1);
  int graph[V][V] = {
    { 0, 65.6, 39, 0, 86, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    { 65.6, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    { 39, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 86, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    { 86, 0, 0, 86, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 0, 0, 70, 0, 0, 0, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 0, 70, 0, 0, 0, 0, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 0, 0, 0, 0, 73.32, 0, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 0, 0, 0, 73.32, 0, 0, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 52, 0, 0, 0 },
    { 0, 0, 0, 0, 0, 0, 0, 0, 0, 52, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 35, 0, 100.38 },
    { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 102, 0 },
    { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 102, 0 }
  };

  dijkstra(graph, start, end);


  //const int V = 14;
  int direction[V][V] = {
    { -1, 270, 180, -1, 90, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
    { 45, -1, 90, -1, -1, 225, -1, -1, -1, -1, -1, -1, -1, -1 },
    { 0, 270, -1, 90, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
    { -1, -1, 270, -1, 90, -1, 225, -1, -1, -1, -1, -1, -1, -1 },
    { 0, -1, -1, 270, -1, -1, -1, -1, -1, -1, -1, -1, 180, -1 },
    { -1, 45, -1, -1, -1, -1, -1, -1, -1, -1, 180, 90, -1, -1 },
    { -1, -1, -1, 0, -1, -1, -1, 90, 180, -1, -1, -1, -1, -1 },
    { -1, -1, -1, -1, -1, -1, 270, -1, -1, -1, -1, -1, -1, -1 },
    { -1, -1, -1, -1, -1, -1, 0, -1, -1, 90, -1, -1, -1, 180 },
    { -1, -1, -1, -1, -1, -1, -1, -1, 270, -1, -1, -1, -1, -1 },
    { -1, -1, -1, -1, -1, 0, -1, -1, -1, -1, -1, 90, -1, 180 },
    { -1, -1, -1, -1, -1, 0, -1, -1, -1, -1, 270, -1, 90, -1 },
    { -1, -1, -1, -1, 0, -1, -1, -1, -1, -1, -1, 270, -1, 180 },
    { -1, -1, -1, -1, -1, -1, -1, -1, 0, -1, 270, -1, 90, -1 }
  };

  String location[V][3] = {
    { "c0cd6791", "e0cd6691", "40ce6791" },
    { "10D26691", "A0406791", "404b6791" },
    { "20476891", "60D66691", "60D26691" },
    { "204E6791", "904E6791", "404D6791" },
    { "50C96791", "00506791", "D0C86791" },
    { "D0386891", "B0326891", "50306891" },
    { "50346891", "901E6991", "E03D6891" },
    { "200D6891", "", "" },
    { "D03C6891", "10166991", "F0316891" },
    { "101E6991", "", "" },
    { "B02B6891", "30236891", "B0236891" },
    { "00D76791", "20DA6791", "A0D96791" },
    { "E0246891", "C0066991", "60246891" },
    { "50466791", "403C6791", "C03C6791" }
  };
  
  int i = 0;
  String UID = "none";
  while (!isStringInRow(location, V, end, UID)) {  // not reach the end
    UID = getTagUID();
    Serial.print("UID: ");
    Serial.print(UID);
    while (!isStringInRow(location, V, path[i], UID)) {  // not reach the next node
      Serial.print("to node ");
      Serial.print(path[i + 1]);

      // turn to direction[from][to]
      float degree = direction_now - direction[path[i]-1][path[i+1]-1];
      direction_now = direction[path[i]-1][path[i+1]-1];
      Serial.print("degree: ");
      Serial.print(degree);
      Servo::TrunDeg(degree);
      //forward
      Motion::Forwards(LeftSpeed, RightSpeed);
      LineTracking::FollowingLine(IR::Tracking(), LeftSpeed, RightSpeed);

      //when detected RFID
    }
    i++;
  }
}

  void setup() {
    // put your setup code here, to run once:

    Serial.begin(115200);

    Serial.println("---------Initializing...---------");
    //Set up PWM Channel for Motor
    Motor::Init();
    Serial.println("Wheel Motors Initialized");

    //Set up PWM Channel for Servo Motor
    Servo::Init();
    Serial.println("Servo Motor Initialized");

    //Initialize IR Sensor
    IR::Init();
    Serial.println("IR Sensor Initialized");

    //Initialize RFID Reader
    Wire.begin(RFID_SDA, RFID_SCL);
    mfrc522.PCD_Init();

    Serial.println("RFID Initialized: ");

    /*Initialize IMU*/
    //#define IMU_SCL 8
    //#define IMU_MOSI 18
    //#define IMU_MISO 17
  //#define IMU_DRDY 7

    /*-Wire.begin(IMU_SCL,IMU_MOSI,IMU_MISO);
    
    SPI.setBitOrder(MSBFIRST);

    SPI.setDataMode(SPI_MODE3);

    pinMode(IMU_MISO, OUTPUT);

    digitalWrite(IMU_MISO, HIGH); // Deselect all IMU sensors
    */

    //Initialize the FireBase Connection



    // Serial.println("FireBase Initialized");


    // Init the PinMode for the Encoder Pins
    pinMode(Motor_L_Encoder_A, INPUT_PULLUP);
    pinMode(Motor_L_Encoder_B, INPUT_PULLUP);

    pinMode(Motor_R_Encoder_A, INPUT_PULLUP);
    pinMode(Motor_R_Encoder_B, INPUT_PULLUP);



    // Attach the interrupt service routine to the encoder pins
    attachInterrupt(digitalPinToInterrupt(Motor_L_Encoder_A), handleLeftEncoderInterrupt, CHANGE);
    attachInterrupt(digitalPinToInterrupt(Motor_R_Encoder_A), handleRightEncoderInterrupt, CHANGE);
    Serial.println("Interrupt Pins Initialized");

    Serial.println("---------Initialized---------");

    /*FreeRTOS Task Pinned to core*/
    /*Do not change the config of the core
  Events Run on Core:   Core 0 (For FreeRTOS)
  Arduino Runs on Core: Core 1 (As Default)

  Run Core Tasks Config:
  Core 0: local  task  (Control)
  Core 1: online task (Firebase)*/
    /*xTaskCreatePinnedtoCore: pin the specific task to desired core (esp32 is a dual cores MCU)
  xTaskCreatePinnedToCore(  void((void *pvPara)), Text for the task, Stack (Min. is 1024), const para. , &TaskTCB, uxPriority, Core )*/
  int params[2] = { 0, 8 };  // Start and end vertices
  TaskHandle_t dijkstraTaskTCB;
  //xTaskCreatePinnedToCore(dijkstraTask, "dijkstra", 10000, (void *)params, 3, &dijkstraTaskTCB, 1);
  xTaskCreatePinnedToCore(FireBaseTask, "FireBase", 10000, NULL, 3, &FireBaseTaskTCB, 0);
    xTaskCreatePinnedToCore(Blink, "Blink", 2048, NULL, 1, &BlinkTaskTCB, 1);
    xTaskCreatePinnedToCore(RFIDTagReader, "RFIDReader", 2048, NULL, 2, &RFIDTagReaderTCB, 1);
    xTaskCreatePinnedToCore(LineTrackingTask, "LineTracking", 10000, NULL, 2, &LineTrackingTaskTCB, 1);
    xTaskCreatePinnedToCore(calculateRPMTask, "calcula  teRPM", 10000, NULL, 3, &calculateRPMTaskTCB, 1);

    /*Adding a small delay for the setup()*/

    vTaskDelay(10);
  }


  /*Nothing will run in loop()
  Please do not write any code inside the loop()*/
  void loop() {

  }
