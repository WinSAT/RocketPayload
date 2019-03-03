 /**************************************************************************/
/*
    Sensors_Combined_MEGA.ino V0.1.0
    Sensors Program for Autonomous Multi-Sensor Information Fusion
    Serial print out and save to SD card
    
    Christian Patalas
    Daksh Patel
    Talha Abdulaziz
    Walter Tchoukou 

    June 05 2018

    https://github.com/uwinrockets

*/
/**************************************************************************/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SparkFunBME280.h>
#include <SparkFunCCS811.h>
#include <SPI.h>
#include <NeoTee.h>
#include "SdFat.h"
#include <SimpleTimer.h>

#define BNO055_SAMPLERATE_DELAY_MS (10)
#define CCS811_ADDR 0x5B //Default I2C Address
//#define CCS811_ADDR 0x5A //Alternate I2C Address

//Global sensor objects
CCS811 myCCS811(CCS811_ADDR);
BME280 myBME280;
Adafruit_BNO055 bno = Adafruit_BNO055();

/*Create Variable for Data File*/
SdFat SD;
#define SD_CS_PIN 53
File myFile;
/* Define Pins and Variables for SD Card Module*/
//int pinCS = 53; // Pin 10 on Arduino Uno
//#define SD_CS_PIN 

//Print out Data 
Print *outputs[] = { &Serial, &myFile };  // <--  list all the output destinations here
NeoTee tee( outputs, sizeof(outputs)/sizeof(outputs[0]) );


File Log;
//Print out Settings and sensors Log 
Print *outLog[] = { &Serial, &Log };  // <--  list all the output destinations here
NeoTee lee( outLog, sizeof(outLog)/sizeof(outLog[0]) );

//int Time = 0;

/* Set Local Altitude */
double localAltitude = 1013.25; //hardcode local latitude, but is overwritten later on in code using file on sd card

SimpleTimer ccs_timer;


/**************************************************************************/
/*
    SparkFun Acceleration Sensor
*/
/**************************************************************************/
void sf_Acceleration()
{
  float sensorValue = analogRead(A0);
  float Acceleration;
  Acceleration = (sensorValue - 508) / 2;
  //myFile.print(F("Acceleration: "));
  tee.print(Acceleration);
  //myFile.println(F(" g"));
  tee.print(',');
}

/**************************************************************************/
/*
    DSB1820 Temperature Sensor
*/
/**************************************************************************/

/**************************************************************************/
/*
    CCS811 Environmental Sensor
*/
/**************************************************************************/
void CCS()
{
  //Check to see if data is ready with .dataAvailable()
  myCCS811.readAlgorithmResults();
  //https://learn.sparkfun.com/tutorials/ccs811-air-quality-breakout-hookup-guide#arduino-library-and-usage 
  //Returns calculated CO2 reading in parts per million (ppm)
  outputToSD(myCCS811.getCO2());
  //Returns calculated Total Voltatile Organic Compounds (TVOC) reading in parts per billion (ppb)
  outputToSD(myCCS811.getTVOC());
//  //CCS Temperature (*C)
  outputToSD(myCCS811.getTemperature());
//  //CCS Resistance (Ohm)
//  outputToSD(myCCS811.getResistance());
  float BMEhumid = myBME280.readFloatHumidity();
  float BMEtempC = myBME280.readTempC();

  //This sends the temperature data to the CCS811
  myCCS811.setEnvironmentalData(BMEhumid, BMEtempC);
}

/**************************************************************************/
/*
    Output Sensor Values to File on SD Card
*/
/**************************************************************************/
// Generic pattern for most
template<typename T> void outputToSD( T arg )
{
  tee.print( arg );
  tee.print( ',' );
}


/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/

void ls(char *path) {
  SdBaseFile dir;
  if (!dir.open(path, O_READ) || !dir.isDir()) {
    Serial.println("bad dir");
    return;
  }
  dir.ls(&Serial, LS_R);
}

void setup()
{
  Serial.begin(9600);
  /*if (!SD.begin(SD_CHIP_PIN)) sd.errorHalt();
  SD.ls(&Serial, LS_R);
  Serial.println("Done with root");
  ls("DIR0");
  Serial.println("Done with DIR0");
  */
  Serial.println("test1245");
  //This begins the CCS811 sensor and prints error status of .begin()
  CCS811Core::status returnCode = myCCS811.begin();
  
  lee.print("Initializing SD card...");

  if (!SD.begin(SD_CS_PIN)) {
    lee.println("initialization failed!");
    SD.errorHalt();
    return;
  }
  SD.ls(&Serial, LS_R);
  lee.println("initialization done.");

  File Log = SD.open("log.txt", FILE_WRITE);
  if (Log) {
    lee.println("New Record");
    lee.println("Orientation Sensor Test"); lee.println(" ");
    /* Initialise the sensor */
    if (!bno.begin())
    {
      /* There was a problem detecting the BNO055 ... check your connections */
      lee.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    }
    /* Display some basic information on this sensor */
    displaySensorDetails(Log);
    // lee.println(displaySensorDetails(Log)); --trying to get logging to work
    /* Optional: Display current status */
    displaySensorStatus(Log);
    //printDriverError( CCS811Core::status errorCode, Log )
    //printDriverError(errorCode, Log)
    bno.setExtCrystalUse(false);
    Log.close();
  }

  char *csvHeaders[] = {"Time Since On (ms)",
    "Euler (deg) - X","Euler (deg) - Y","Euler (deg) - Z", 
    "Quaterion - qW","Quaterion - qX","Quaterion - qY","Quaterion - qZ",
    "Accel (m/s^2) - X","Accel (m/s^2) - Y","Accel (m/s^2) - Z",
    "Mag (uT) - X","Mag (uT) - Y", "Mag (uT) - Z",
    "Gravity (m/s^2) - X","Gravity (m/s^2) - Y","Gravity (m/s^2) - Z",
    "Lin. Accel (m/s^2) - X","Lin. Accel (m/s^2) - Y","Lin. Accel (m/s^2) - Z",
    "Gyro (rad/s) - X","Gyro (rad/s) - Y","Gyro (rad/s) - Z",
    "BNO Temp (*C)", 
    "SF Accel1D (g)", 
    "CCS CO2 (ppm)", "CCS TVOC (ppb)", "CCS Temp (*C)",
    "Press. (Pa)", "Altitude (m)", "Humidity (%)", "BME Temp (*C)",
    "Time Since startTime loop (ms)"
  };
  
  File DataLabel = SD.open("MegaSensors.csv", FILE_WRITE);
  if (DataLabel) {
    DataLabel.println("");
    DataLabel.println("New Record");
    int arraySize = sizeof(csvHeaders) / sizeof(csvHeaders[0]);
    for (int i=0; i< arraySize; i++){
      DataLabel.print(csvHeaders[i]);
      DataLabel.print(",");
    }
    DataLabel.close();
  }

  myFile = SD.open("MegaSensors.csv", FILE_WRITE);
  
  File Settings;
  Settings = SD.open("setting.txt", FILE_WRITE);    // settings - this is where the altitude. are checked and extra code is to make sure the number redundany 
  if (Settings) { //figure out how this redundany works
    char buf[8];
    char c = 'a';
    int index = 0;
    index = 0;
    while (c != '\r' && c != '\n' && Settings.available() && index < 7)
    {
      buf[index] = Settings.read();
      index++;
    }
    buf[index] = '\0';
    Settings.close();
    localAltitude = atol(buf)/100.0;
  }
  else {
    tee.println("Error opening settings file");
  }

  /* Initialise the BNO sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print(F("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!"));
    while(1);
  }

  Serial.println(F("Calibration status values: 0=uncalibrated, 3=fully calibrated"));
  displayCalStatus();

  //Initialize BME280
  //For I2C, enable the following and disable the SPI section
  myBME280.settings.commInterface = I2C_MODE;
  myBME280.settings.I2CAddress = 0x77;
  myBME280.settings.runMode = 3; //Normal mode
  myBME280.settings.tStandby = 0;
  myBME280.settings.filter = 4;
  myBME280.settings.tempOverSample = 5;
  myBME280.settings.pressOverSample = 5;
  myBME280.settings.humidOverSample = 5;

  //Calling .begin() causes the settings to be loaded
  delay(10);  //Make sure sensor had enough time to turn on. BME280 requires 2ms to start up.
  byte id = myBME280.begin(); //Returns ID of 0x60 if successful


  delay(2000);
}

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
void loop()
{
  tee.println();
  int count = 0;
    
  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2


  /* Display calibration status for each sensor. */
  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  Serial.print("CALIBRATION: Sys=");
  Serial.print(system, DEC);
  Serial.print( ',' );
  Serial.print(" Gyro=");
  Serial.print(gyro, DEC);
  Serial.print( ',' );
  Serial.print(" Accel=");
  Serial.print(accel, DEC);
  Serial.print( ',' );
  Serial.print(" Mag=");
  Serial.print(mag, DEC);
  Serial.println( ',' );


  //myFile = SD.open("Mega.csv", FILE_WRITE);
  if (myFile)
  {

    //Time Since Turned On (ms)   
    unsigned long startTime = millis();
    outputToSD(startTime);

    /* Get a new sensor event */
    sensors_event_t event;
    bno.getEvent(&event);


    /* Display the floating point data */
    //Euler (deg)
    outputToSD(event.orientation.x);
    outputToSD(event.orientation.y);
    outputToSD(event.orientation.z);

    //Quaternion
    imu::Quaternion quat = bno.getQuat();
    outputToSD(quat.w());
    outputToSD(quat.x());
    outputToSD(quat.y());
    outputToSD(quat.z());

    //Accelerometer (m/s^2)
    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    outputToSD(accel.x());
    outputToSD(accel.y());
    outputToSD(accel.z());

    //Magnetometer (uT)
    imu::Vector<3> magnet = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
    outputToSD(magnet.x());
    outputToSD(magnet.y());
    outputToSD(magnet.z());

    //Gravity (m/s^2)
    imu::Vector<3> gravity = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
    outputToSD(gravity.x());
    outputToSD(gravity.y());
    outputToSD(gravity.z());

    //Linear Acceleration (m/s^2)
    imu::Vector<3> linear = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    outputToSD(linear.x());
    outputToSD(linear.y());
    outputToSD(linear.z());

    //Gyroscope (rad/s)
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    outputToSD(gyro.x());
    outputToSD(gyro.y());
    outputToSD(gyro.z());

    //Temperature (*C)
    int8_t bno_temp = bno.getTemp();
    outputToSD(bno_temp);

    //Sf Acceleration - Stores Acceleration in 1-axis (g)
    sf_Acceleration();

    //CSS811 Sensor - Stores 
    CCS();
    
    /* Sparkfun BME280 and CCS811 */
    //Pressure (Pa)
    outputToSD(myBME280.readFloatPressure());
    //Approx Altitude (m)
    outputToSD(myBME280.readFloatAltitudeMeters());
    //Humidity (%)
    outputToSD(myBME280.readFloatHumidity());
    //BME Temperature (*C)
    outputToSD(myBME280.readTempC());

    //CCS811 Run
    ccs_timer.run();


    //DSB1820 Temperature
//    dsb_Temperature();


    //Time Since startTime Command  (ms)
    outputToSD(millis()-startTime);

//  tee.println();
    myFile.flush();
  }

  delay(20); //Test this Delay 
} // End of Loop

/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/
void displaySensorDetails(File &Log)  //log file on sd card. this is for initial sensor value stuff
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  lee.println(F("------------------------------------"));
  lee.print  (F("Sensor:       ")); lee.println(sensor.name);
  lee.print  (F("Driver Ver:   ")); lee.println(sensor.version);
  lee.print  (F("Unique ID:    ")); lee.println(sensor.sensor_id);
  lee.print  (F("Max Value:    ")); lee.print(sensor.max_value); lee.println(" xxx");
  lee.print  (F("Min Value:    ")); lee.print(sensor.min_value); lee.println(" xxx");
  lee.print  (F("Resolution:   ")); lee.print(sensor.resolution); lee.println(" xxx");
  lee.println(F("------------------------------------"));
  lee.println(F(" "));
}


/**************************************************************************/
/*
    Display some basic info about the sensor status
*/
/**************************************************************************/
void displaySensorStatus(File &Log) //  log file again with a validation for calibration. gives 1 for calibrated
{
  /* Get the system status values (mostly for debugging purposes) */
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);

  lee.println(F(" "));
  lee.print(F("System Status: 0x"));
  lee.println(system_status, HEX);
  lee.print(F("Self Test:     0x"));
  lee.println(self_test_results, HEX);
  lee.print(F("System Error:  0x"));
  lee.println(system_error, HEX);
  lee.println(F(" "));
}

/**************************************************************************/
/*
    Display sensor calibration status
*/
/**************************************************************************/
void displayCalStatus()
{
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t sys, gyro, accel, mag;  //variables declared
  sys = gyro = accel = mag = 0;
  bno.getCalibration(&sys, &gyro, &accel, &mag);

  /* Display the individual values */
  lee.print(F("CALIBRATION: Sys="));
  lee.print(sys, DEC);
  lee.print(F(" Gyro="));
  lee.print(gyro, DEC);
  lee.print(F(" Accel="));
  lee.print(accel, DEC);
  lee.print(F(" Mag="));
  lee.print(mag, DEC);
  lee.print((','));
}

void printDriverError( CCS811Core::status errorCode,File &Log)
{
  switch ( errorCode )
  {
    case CCS811Core::SENSOR_SUCCESS:
      lee.print("SUCCESS");
      break;
    case CCS811Core::SENSOR_ID_ERROR:
      lee.print("ID_ERROR");
      break;
    case CCS811Core::SENSOR_I2C_ERROR:
      lee.print("I2C_ERROR");
      break;
    case CCS811Core::SENSOR_INTERNAL_ERROR:
      lee.print("INTERNAL_ERROR");
      break;
    case CCS811Core::SENSOR_GENERIC_ERROR:
      lee.print("GENERIC_ERROR");
      break;
    default:
      lee.print("Unspecified error.");
  }
}
