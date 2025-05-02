#include <SD.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Servo.h>
#include <vector>

//Flight Parameters
struct FlightProfile{
  float AltitudeSetpoint_m = 240; //241 for qualifying, 248 and 236 for finals
  float QNH_hPa = 1023;  //current sea level barometric pressure (1015 for Santa Fe Dam, 1023 for home)
  float range_deg = 60; // movement range of servo
};

struct NavData {
  float altitude_m;
  float velocity_m_s = 0;
  float accel_ms2;
};

struct FlightRecord {
  uint32_t timestamp;
  float pressure;
  float altitude;      // meters
  float acceleration;  // m/s^2
  float estimation;    // meters
};

const FlightProfile flightProfile; //const because we won't change parameters during flight



//Other Variables
Adafruit_BMP3XX bmp;
Adafruit_ICM20948 imu;
File flightData;
File mel;
Servo Servo1;
Servo Servo2;
Servo Servo3;

uint16_t measurement_delay_us = 65535; 

const int BMP_address = 0x77;

float pressure_hPa;


char charRead;
float groundLevel_m;
bool HaveGroundLevel = false;

char dataStr[200] = "";
char buffer[7];

std::vector<FlightRecord> flightLog;

uint8_t State = 0; //saves a little space, swich back to int if it doesnt work

//Pin definitions
const uint8_t chipSelect = BUILTIN_SDCARD;
const uint8_t Pyro1 = 6;
const uint8_t Pyro2 = 7;
const uint8_t VoltagePin = 40;
const uint8_t RED = 21;
const uint8_t BLU = 17;
const uint8_t GRN = 14;
const uint8_t BUZZER = 36;


void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);
  delay(2000);

  Servo1.attach(3);
  Servo2.attach(4);
  Servo3.attach(5);

  if (bmp.begin_I2C(BMP_address)) {
    Serial.println("Barometer is present");
  } else {
    Serial.println("Barometer faliure");
    while (1);  //halt program
  }

  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_2X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_16X); //should get us less noise, revert to 8x if it doesnt work
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_1);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  if (SD.begin(chipSelect)) {
    Serial.println("SD card is present");
  } else {
    Serial.println("SD card failure");
    while (1);  //halt program
  }

    if (imu.begin_I2C()) {
    Serial.println("IMU chip is present");
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
  NavData nav = GetNav(); //use SITL_GetNav() for testing

  //also figure out apogee prediction 
  int estApogee = predictApogee(nav.altitude_m, nav.accel_ms2);

  DeployBrakes(estApogee);

  SetState(nav.altitude_m, nav.accel_ms2, nav.velocity_m_s);

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
  FlightRecord record;
  record.timestamp = millis();
  record.pressure = pressure;
  record.altitude = altitude;
  record.acceleration = accel;
  record.estimation = estimation;

  flightLog.push_back(record);
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
  Servo1.write(flightProfile.range_deg); // full extension
}

void WriteToFile(){
  //order is Time,Pressure,Altitude,Acceleration,Estimation
  flightData = SD.open("flightData.txt", FILE_WRITE);

  if (flightData) {
    for (const FlightRecord& record : flightLog) {
      dataStr[0] = 0;

      itoa(record.timestamp, buffer, 10); 
      strcat(dataStr, buffer); //add time to buffer 
      strcat(dataStr, ", "); //append delimeter

      dtostrf(record.pressure, 5, 1, buffer); //5 is minumum width (we need at least this many characters!), 1 is precision (amount of decimal points)
      strcat(dataStr, buffer);
      strcat(dataStr, ", ");

      dtostrf(record.altitude, 5, 1, buffer); 
      strcat(dataStr, buffer);
      strcat(dataStr, ", ");

      dtostrf(record.acceleration, 5, 2, buffer);
      strcat(dataStr, buffer);
      strcat(dataStr, ", ");

      dtostrf(record.estimation, 5, 1, buffer);
      strcat(dataStr, buffer);

      flightData.println(dataStr);
    }

  }else {
    Serial.println("error opening csv.txt");
  }

}

NavData GetNav(){
  //return all navigation data: Altitude, Acceleration, Velocity. TO BE ADDED: Attitude
  NavData data;

  //  /* Get a new normalized sensor event */
  sensors_event_t a;
  sensors_event_t g;
  sensors_event_t m;
  sensors_event_t t;

  data.altitude_m = bmp.readAltitude(flightProfile.QNH_hPa);
  imu.getEvent(&a, &g, &t, &m); 
  data.accel_ms2 = a.acceleration.y;

  data.velocity_m_s += data.accel_ms2;  // integrate acceleration

  return data;
}

NavData SITL_GetNav(){
  NavData data;
  data.altitude_m = 0;
  data.accel_ms2 = 0;
  data.velocity_m_s = 0;
  return data;
}

void DetectLaunch(){
  //detect launch and track t+ time
}

