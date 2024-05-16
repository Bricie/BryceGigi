#include "ICM42688.h"
#include "SPI.h"

#define VSPI_CLK 8
#define VSPI_MISO 7
#define VSPI_MOSI 18
#define VSPI_SS 37
// an ICM42688 object with the ICM42688 sensor on SPI bus 0 and chip select pin 10
SPIClass mySPI(VSPI);

ICM42688 IMU(mySPI, VSPI_SS);


void setup() {
  // serial to display data
  Serial.begin(115200);
  Serial.println("initizialing..");



  // while(!Serial) {}
  SPI.begin(VSPI_CLK, VSPI_MISO, VSPI_MOSI, VSPI_SS);
  // pinMode(VSPI_SS, OUTPUT);
  // start communication with IMU
  int status = IMU.begin();
  digitalWrite(VSPI_SS, LOW);
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }
  Serial.println("ax,ay,az,gx,gy,gz,temp_C");
}

void loop() {
  // read the sensor
  IMU.getAGT();
  // display the data
  Serial.print(IMU.accX(),6);
  Serial.print("\t");
  Serial.print(IMU.accY(),6);
  Serial.print("\t");
  Serial.print(IMU.accZ(),6);
  Serial.print("\t");
  Serial.print(IMU.gyrX(),6);
  Serial.print("\t");
  Serial.print(IMU.gyrY(),6);
  Serial.print("\t");
  Serial.print(IMU.gyrZ(),6);
  Serial.print("\t");
  Serial.println(IMU.temp(),6);
  delay(100);
}
