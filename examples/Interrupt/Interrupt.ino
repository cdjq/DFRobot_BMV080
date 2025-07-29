/*!
 * @file interrupt.ino
 * @brief This routine uses the IIC interface and adopts the interrupt mode to obtain data. 
 * @n When the sensor data is ready, an interrupt signal will be generated.
 * @n The obtained data include the current levels of PM1, PM2.5 and PM10 in the air.
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

//You can choose to use either the IIC interface or the SPI interface. The default is IIC. 
//If you want to use SPI, simply remove the following comments.
DFRobot_BMV080_I2C sensor(&Wire, 0x57); // Create an instance of the DFRobot_BMV080_I2C class with the I2C address 0x57.

//#define SPI_CS_PIN 17
//DFRobot_BMV080_SPI sensor(&SPI,SPI_CS_PIN); // Create an instance of the DFRobot_BMV080_SPI class with the SPI CS pin.

#define IRQ_Pin 14 // Define the interrupt pin.

bool dataFlag = false; // Create a flag to indicate whether new data is available.

void setup() {
  char id[13]; // Variable to store the chip ID of the BMV080 sensor.
  Serial.begin(115200);
  while(!Serial) delay(100); // Wait for the serial port to be ready.
  // Initialize the sensor.
  Serial.println("The interrupt routine.");
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
  // Set the interrupt pin.
  setInterruptPin();
}

// define the variables to store the PM data.
float pm1,pm2_5,pm10;
void loop() {
  // Check if new data is available.
  // If the dataFlag is true, it means new data is available.
  if(dataFlag){
    dataFlag = false;
    if(sensor.getBmv080Data(&pm1,&pm2_5,&pm10)){
      Serial.print("pm1:" + String(pm1) + "  " + "pm2.5:" + String(pm2_5) + "  " + "pm10:" + String(pm10));

      if(sensor.ifObstructed()){
        Serial.print("  Obstructed The data may be invalid.");
      }

      Serial.println();
    }
  }
  delay(100);
}

// Function to set the interrupt pin.
// This function checks if the interrupt pin is supported on the board and sets it up.
void setInterruptPin(void){
  if(digitalPinToInterrupt(IRQ_Pin) == -1){
    Serial.println("Interrupt pin not supported on this board.");
  }else{
    Serial.println("Interrupt pin supported.");
  }
  pinMode(IRQ_Pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(IRQ_Pin), IRQ_handler, FALLING);
}

// Interrupt handler function
void IRQ_handler(void)
{
  dataFlag = true;
}