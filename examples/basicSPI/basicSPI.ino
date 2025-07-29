/*!
 * @file basicSPI.ino
 * @brief This routine continuously reads sensor data via the SPI interface and can obtain one piece of data per second. 
 * The obtained data include the current levels of PM1, PM2.5 and PM10 in the air.
 * @n The demo supports FireBeetle-ESP32-E, FireBeetle-ESP32-S3, and FireBeetle-ESP8266.
 * @details Experimental phenomenon: The read data will be output in the serial port monitor.
 * 
 * @copyright Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license The MIT License (MIT)
 * @author [Ouki](ouki.wang@dfrobot.com)
 * @version V1.0
 * @date 2025-07-28
 * @url https://github.com/DFRobot/DFRobot_BMV080
 */

#include "DFRobot_BMV080.h"
#include <string>

SET_LOOP_TASK_STACK_SIZE(60 * 1024); // Set the stack size of the loop task to 60KB

#define SPI_CS_PIN 17 // Define the SPI CS pin.

DFRobot_BMV080_SPI sensor(&SPI,SPI_CS_PIN); // Create an instance of the DFRobot_BMV080_SPI class with the SPI CS pin.

void setup() {
  char id[13]; // Variable to store the chip ID of the BMV080 sensor.
  Serial.begin(115200);
  while(!Serial) delay(100); // Wait for the serial port to be ready.
  // Initialize the sensor.
  Serial.println("SPI continuous reading routine.");
  while(sensor.begin() != 0){
    Serial.println("Initialization of the sensor failed! Please confirm if the sensor chip connection is correct.");
    delay(1000);
  }
  Serial.println("Initialization of the sensor was successful.");
  // Open the BMV080 sensor.
  while(sensor.openBmv080()){
    Serial.println("open failed");
    delay(1000);
  }
  Serial.println("open successful");
  // Get the chip ID of the BMV080 sensor.
  sensor.getBmv080ID(id);
  Serial.println("Chip ID is:" + String(id));
  // Set the measurement mode to continuous mode.
  if(sensor.setBmv080Mode(DFRobot_BMV080_MODE_CONTINUOUS))
    Serial.println("Mode setting successful");
    
}

// define the variables to store the PM data.
float pm1,pm2_5,pm10;
void loop() {
  // Read the PM data from the sensor.
  if(sensor.getBmv080Data(&pm1,&pm2_5,&pm10)){
    Serial.print("pm1:" + String(pm1) + "  " + "pm2.5:" + String(pm2_5) + "  " + "pm10:" + String(pm10));

    if(sensor.ifObstructed()){
      Serial.print("  Obstructed The data may be invalid.");
    }

    Serial.println();
  }
  delay(100);
}
