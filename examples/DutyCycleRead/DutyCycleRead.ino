/*!
 * @file dutyCycRead.ino
 * @brief This routine uses the IIC interface to periodically read data, with a cycle of 20 seconds.
 * @n The obtained data include the current levels of PM1, PM2.5 and PM10 in the air.
 * @n The demo supports FireBeetle-ESP32-E, FireBeetle-ESP32-S3, and FireBeetle-ESP8266.
 * @details Experimental phenomenon: The read data will be output in the serial port monitor.
 * 
 * @copyright Copyright (c) 2025 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license The MIT License (MIT)
 * @author [lbx](liubx8023@gmail.com)
 * @version V1.0
 * @date 2025-09-15
 * @url https://github.com/DFRobot/DFRobot_BMV080
 */

#include "DFRobot_BMV080.h"
#include <string>

SET_LOOP_TASK_STACK_SIZE(60 * 1024); // Set the stack size of the loop task to 60KB

//You can choose to use either the IIC interface or the SPI interface. The default is IIC. 
DFRobot_BMV080_I2C sensor(&Wire, 0x57); // Create an instance of the DFRobot_BMV080_I2C class with the I2C address 0x57.

/* If you want to use SPI, simply remove the following comments, and change the SPI_CS_PIN to the corresponding SPI CS pin.*/
//#define SPI_CS_PIN 17
//DFRobot_BMV080_SPI sensor(&SPI,SPI_CS_PIN); // Create an instance of the DFRobot_BMV080_SPI class with the SPI CS pin.

#define DUTY_CYCLE_PERIOD 20 // Duty cycle period in seconds.
#define INTEGRATION_TIME 10.0f // Integration Time in seconds.

void setup() {
  char id[13]; // Variable to store the chip ID of the BMV080 sensor.
  Serial.begin(115200);
  while(!Serial) delay(100); // Wait for the serial port to be ready.
  // Initialize the sensor.
  Serial.println("Periodic reading routine.");
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
  // Set the duty cycling period of the BMV080 sensor.
  if(!sensor.setDutyCyclingPeriod(DUTY_CYCLE_PERIOD)){
    Serial.println("The measurement period set error!");
  }
  Serial.print("The measurement period is set to:");
  Serial.println(sensor.getDutyCyclingPeriod());
  // Set the integration time of the BMV080 sensor.
  if(!sensor.setIntegrationTime(INTEGRATION_TIME)){
    Serial.println("The integration time set error!");
  }
  Serial.print("The integration time is set to:");
  Serial.println(sensor.getIntegrationTime());
  // enable obstacle detection.
  if(sensor.setObstructionDetection(true)){
    Serial.println("Obstacle detection enable sucessful.");
  }else{
    Serial.println("Obstacle detection failed to open.");
  }
  // Set the measurement mode to duty cycle mode.
  if(sensor.setBmv080Mode(DUTY_CYCLE_MODE)){
    Serial.println("Mode setting successful");
  }else{
    Serial.println("Mode setting failed");
  }
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
