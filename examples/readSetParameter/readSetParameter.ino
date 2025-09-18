/*!
 * @file dutyAndInterrupt.ino
 * @brief This routine will first read the current parameter of the sensor, and then modify the parameter. 
 * @n The obtained data include the current levels of PM1, PM2.5 and PM10 in the air.
 * @n The demo supports FireBeetle-ESP32-E, FireBeetle-ESP32-S3.
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
/*
 * address selection of I2C:
 * --------------------------------------
 * |    CSB    |    MISO    |  Address  |
 * --------------------------------------
 * |     0     |     0      |   0x54    |
 * --------------------------------------
 * |     0     |     1      |   0x55    |
 * --------------------------------------
 * |     1     |     0      |   0x56    |
 * --------------------------------------
 * |     1     |     1      |   0x57    |
 * --------------------------------------
 */
DFRobot_BMV080_I2C sensor(&Wire, 0x57); // Create an instance of the DFRobot_BMV080_I2C class with the I2C address 0x57.

/* If you want to use SPI, simply remove the following comments, and change the SPI_CS_PIN to the corresponding SPI CS pin.*/
//#define SPI_CS_PIN 17
//DFRobot_BMV080_SPI sensor(&SPI,SPI_CS_PIN); // Create an instance of the DFRobot_BMV080_SPI class with the SPI CS pin.

void setup() {
  char id[13]; // Variable to store the chip ID of the BMV080 sensor.
  uint16_t major, minor, patch;
  Serial.begin(115200);
  while(!Serial) delay(100); // Wait for the serial port to be ready.
  // Initialize the sensor.
  Serial.println("The Continuous reading routine.");
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
  sensor.getBmv080DV(major,minor,patch);
  Serial.println("BMV080 version: " + String(major) + "." + String(minor) + "." + String(patch));
  Serial.println();

  Serial.println("Read the current parameter of the sensor:");
  readParameter(); // Read the current parameters of the sensor.
  Serial.println();
  Serial.println("Set the modified parameter of the sensor:");
  setParameter();  // Modify the parameters of the sensor.

  // Set the measurement mode to continuous mode.
  /*!
   * @brief Set the measurement mode of the BMV080 sensor.
   * @param mode The mode to set, either CONTINUOUS_MODE or DUTY_CYCLE_MODE
   *              CONTINUOUS_MODE: Sensor takes measurements continuously
   *              DUTY_CYCLE_MODE: Sensor takes measurements at specified intervals
   * @return 0 successful, other error.
   */
  if(sensor.setBmv080Mode(CONTINUOUS_MODE)){
    Serial.println("Mode setting failed");
  }else{
    Serial.println("Mode setting successful");
  }
}

// define the variables to store the PM data.
float pm1,pm2_5,pm10;
void loop() {
  // Check if new data is available.
  // If the dataFlag is true, it means new data is available.
  if(sensor.getBmv080Data(&pm1,&pm2_5,&pm10)){
    Serial.print("pm1:" + String(pm1) + "  " + "pm2.5:" + String(pm2_5) + "  " + "pm10:" + String(pm10));
    if(sensor.ifObstructed()){
    Serial.print("  Obstructed The data may be invalid.");
    }
    Serial.println();
  }
  delay(100);
}

void readParameter(){ 
    float integrationTime;
    integrationTime = sensor.getIntegrationTime(); // Read the integration time parameter of the sensor.
    if(integrationTime != NAN){
        Serial.println("Integration time: " + String(integrationTime) + "s");
    }

    uint16_t dutyCyclingPeriod;
    dutyCyclingPeriod = sensor.getDutyCyclingPeriod(); // Read the duty cycling period parameter of the sensor.
    if(dutyCyclingPeriod != 0){
        Serial.println("Duty cycling period: " + String(dutyCyclingPeriod) + "s");
    }

    bool obstructionDetection;
    obstructionDetection = sensor.getObstructionDetection(); // Read the obstruction detection parameter of the sensor.
    if(obstructionDetection == true){
        Serial.println("Obstruction detection: Enabled");
    } else if(obstructionDetection == false){
        Serial.println("Obstruction detection: Disabled");
    } else {
        Serial.println("Obstruction detection: read error");
    }

    bool vibrationFiltering;
    vibrationFiltering = sensor.getDoVibrationFiltering(); // Read the vibration filtering parameter of the sensor.
    if(vibrationFiltering == true){
        Serial.println("Vibration filtering: Enabled");
    } else if(vibrationFiltering == false){
        Serial.println("Vibration filtering: Disabled");
    } else {
        Serial.println("Vibration filtering: read error");
    }

    uint8_t measurementAlgorithm;
    measurementAlgorithm = sensor.getMeasurementAlgorithm(); // Read the measurement algorithm parameter of the sensor.
    if(measurementAlgorithm == FAST_RESPONSE){
        Serial.println("Measurement algorithm: Fast response");
    }else if(measurementAlgorithm == BALANCED){
        Serial.println("Measurement algorithm: Balanced");
    }else if(measurementAlgorithm == HIGH_PRECISION){
        Serial.println("Measurement algorithm: High precision");
    }else{
        Serial.println("Measurement algorithm: read error");
    }
}

void setParameter(){
    // Set the integration time parameter of the sensor to 10s.
    // The integration time must be greater than 1s.
    if(sensor.setIntegrationTime(8)){
        Serial.println("Failed to set integration time.");
    } else {
        Serial.println("Set integration time to 8s successfully.");
    }

    // Set the duty cycling period parameter of the sensor to 300s.
    // The duty cycling period must be greater than 12s, and must be greater than the integration time by at least 2s.
    if(sensor.setDutyCyclingPeriod(15)){
        Serial.println("Failed to set duty cycling period.");
    } else {
        Serial.println("Set duty cycling period to 15s successfully.");
    }

    // Enable the obstruction detection feature of the sensor.
    if(sensor.setObstructionDetection(true)){
        Serial.println("Enabled obstruction detection successfully.");
    } else {
        Serial.println("Failed to enable obstruction detection.");
    }

    // Enable the vibration filtering feature of the sensor.
    if(sensor.setDoVibrationFiltering(true)){
        Serial.println("Enabled vibration filtering successfully.");
    } else {
        Serial.println("Failed to enable vibration filtering.");
    }

    // Set the measurement algorithm parameter of the sensor to HIGH_PRECISION.
    /*!
     * @brief Set the measurement algorithm of the BMV080 sensor.
     * @param measurement_algorithm The measurement algorithm to use.
     *                              FAST_RESPONSE //Fast response,suitable for scenarios requiring quick response
     *                              BALANCED //Balanced, suitable for scenarios where a balance needs to be struck between precision and rapid response
     *                              HIGH_PRECISION //High precision, suitable for scenarios requiring high accuracy
     * @return 0 successful, other error.
     */
    if(sensor.setMeasurementAlgorithm(HIGH_PRECISION)){
        Serial.println("Failed to set measurement algorithm.");
    } else {
        Serial.println("Set measurement algorithm to HIGH_PRECISION successfully.");
    }
}