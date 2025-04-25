#include <SD.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Servo.h>
#include <vector>


Adafruit_BMP3XX bmp;
Adafruit_ICM20948 imu;
File flightData;
File mel;
Servo Airbrakes_Servo;

uint16_t measurement_delay_us = 65535; 

const int chipSelect = BUILTIN_SDCARD;

float QNH = 1023;  //current sea level barrometric pressure (1015 for Santa Fe Dam, 1023 for home)
const int BMP_address = 0x77;

float pressure;
float altimeter;
float integrated_accel = 0;
float accel;
float velocity;
char charRead;
float groundLevel;
bool HaveGroundLevel = false;

float range_degrees = 60;

char dataStr[200] = "";
char buffer[7];

std::vector<float> pressureData = {};
std::vector<short> altitudeData = {};
std::vector<short> accelData = {};
std::vector<short> estimationData = {};
std::vector<uint32_t> runTime = {};

int State = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);
  delay(2000);

  Airbrakes_Servo.attach(3);

  if (bmp.begin_I2C(BMP_address)) {
    Serial.println("Barometer is present");
  } else {
    Serial.println("Barometer faliure");
    while (1);  //halt program
  }

  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  if (SD.begin(chipSelect)) {
    Serial.println("SD card is present");
  } else {
    Serial.println("SD card failure");
    while (1);  //halt program
  }

    if (imu.begin_I2C()) {
    Serial.println("IMU card is present");
  } else {
    Serial.println("IMU failure");
    while (1);  //halt program
  }
  imu.setAccelRange(ICM20948_ACCEL_RANGE_16_G);
  imu.setGyroRange(ICM20948_GYRO_RANGE_1000_DPS);

  imu.setAccelRateDivisor(4095);

  imu.setGyroRateDivisor(255);

  SDFileInit();
}


// put your main code here, to run repeatedly:
void loop() {

  dataStr[0] = 0;
  //Take sensor input / SITL input
  //  /* Get a new normalized sensor event */
  sensors_event_t a;
  sensors_event_t g;
  sensors_event_t m;
  sensors_event_t t;

  //sensors_event_t g; //do we want gyros??
  altimeter = bmp.readAltitude(QNH);
  imu.getEvent(&a, &g, &t, &m); //add &g to here if we want gyros
  accel = a.acceleration.z;

  velocity += accel;

  //also figure out apogee prediction 
  int estApogee = predictApogee(altimeter, accel);

  DeployBrakes(estApogee);

  SetState(altimeter, accel, velocity);

  //Swich case for the following: Idle, Launch Detection, boost, Burnout, Descent, Chutes, Safe
  switch(State){
    case 0: 
      //await command for launch detect mode
      //periodic beep/flash/servo actuation
      //maybe a diagnostic mode on serial?
      break;

    case 1: //launch detection, triggered manually
      //start logging data
      //watch out for accel spikes!
      break;

    case 2: //boost, triggered by accel spike at liftoff
      //watch for burnout
      break;

    case 3: // coast, triggered by accel < 0
      //deploy airbrakes and calculate apogee
      break;

    case 4: //descent, triggered by velocity < 0 (apogee)
      //retract airbrakes
      break;

    case 5: // safe, triggered by landing (no accel spikes, average a is 9.81, also <5 agl for a few seconds)
      //log data to SD card, beep and light up when finished
      break;

    case 6: // abort if excessive pitch or other anomaly- force write to sd card and retract airbrakes if out
      break;

    default:
      
      break;

  }

}

//intializes SD card files when we write to them
void SDFileInit() {
  flightData = SD.open("flightData.txt", FILE_WRITE);
  if (flightData)  // it opened OK
  {
    Serial.println("Writing headers to flightData.txt");
    flightData.println("Time,Pressure,Altitude,Acceleration,Estimation");
    flightData.close();
    Serial.println("Headers written");
  } else{
    Serial.println("Error opening log.txt");
  }


  mel = SD.open("mel.txt", FILE_WRITE);
  if (mel)  // it opened OK
  {
    Serial.println("Writing headers to mel.txt");
    mel.println("FILE INITIALIZED");
    mel.close();
    Serial.println("Major Event Log initialized");
  } else{
    Serial.println("Error opening mel.txt");
  }
}

//saves data during flight
void SaveData(float pressure, float altitude, float accel, float estimation){
  short altimeter = altitude*10; // converts m to dekameter for precsicrion (1 decimal places), remember to truncate altitude to 1 decomal place
  short accelerometer = accel * 100;//converts m/s^2 to hectometer /s^2 (2 decimal places), remember to truncate to 2 decimal places

  short est = estimation * 10;
  runTime.emplace_back(millis());
  pressureData.emplace_back(pressure);
  altitudeData.emplace_back(altimeter);
  accelData.emplace_back(accelerometer);
  estimationData.emplace_back(est);
}

void SetState(float altitude, float accel, float v){
  //code to set state goes here
  if (State == 1 && accel > 30){
    State = 2;
  }else if (State == 2 && accel >= 0){
    State = 3;
  }else if (State == 3 && v<= 0){
    State = 4;
  }else if (State == 4 && altitude > 5){
    State = 5;
  }else{
    State = 0;
  }
}

float predictApogee(float altitude, float accel){
  int apogee = 0;
  return apogee;
}

void DeployBrakes(float estApogee){
  //code for timing goes here
  Airbrakes_Servo.write(range_degrees); // full extension
}

void WriteToFile(){
  //order is Time,Pressure,Altitude,Acceleration,Estimation
  flightData = SD.open("flightData.txt", FILE_WRITE);

  if (flightData) {
    for (uint i = 0; i<= runTime.size(); i++){
      dataStr[0] = 0;

      itoa(runTime[i] , buffer, 10);
      strcat(dataStr, buffer);     //add it onto the end
      strcat(dataStr, ", ");       //append the delimeter

      dtostrf(pressureData[i], 5, 1, buffer); // leave pressure alone
      strcat(dataStr, buffer);
      strcat(dataStr, ", ");       //append the delimeter

      float a = altitudeData[i]/10;  //remember to divide altitude by 10 before saving 
      dtostrf(a, 5, 1, buffer);  //5 is mininum width, 1 is precision; float value is copied onto buff
      strcat(dataStr, buffer);           //append the coverted float
      strcat(dataStr, ", ");       //append the delimeter

      float accel = accelData[i]/100;   //remember to divide accel by 100 before saving
      dtostrf(accel, 5, 1, buffer);  //5 is mininum width, 1 is precision; float value is copied onto buff
      strcat(dataStr, buffer);           //append the coverted float
      strcat(dataStr, ", ");       //append the delimeter

      float est = estimationData[i]/10; //remember to divide estimation by 10 before saving 
      dtostrf(est, 5, 1, buffer);  //5 is mininum width, 1 is precision; float value is copied onto buff
      strcat(dataStr, buffer);           //append the coverted float
      strcat(dataStr, NULL);       //terminate correctly

      flightData.println(dataStr);

    }
  
  }else {
    Serial.println("error opening csv.txt");
  }

}

