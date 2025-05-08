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

const FlightProfile flightProfile; //const because we won't change parameters during flight

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


/////////////////SIMULATION
struct FlightSimConfig {
  float mass_kg = 0.5;
  float drag_coefficient = 0.75;
  float cross_section_area_m2 = 0.01;
  float thrust_N = 0;
  float burn_time_s = 2.0;
  float max_thrust_N = 20;
  float air_density = 1.225; // kg/m^3
};

struct ThrustSample {
  float time_s;     // time from ignition in seconds
  float thrust_N;   // thrust in Newtons
};

const ThrustSample thrust_curve[] = {
  {0.00, 0.0}, {0.05, 5.0}, {0.10, 15.0}, {0.25, 20.0}, {0.40, 18.0},
  {0.60, 10.0}, {0.85, 5.0}, {1.20, 2.0}, {1.50, 0.5}, {2.00, 0.0}
};

const int NUM_THRUST_POINTS = sizeof(thrust_curve) / sizeof(thrust_curve[0]);



FlightSimConfig simConfig;
bool isSimulating = false;
unsigned long simStartTime = 0;

float sim_altitude = 0;
float sim_velocity = 0;
float sim_accel = 0;
/////////////////SIMULATION


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

char dataStr[64];
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
  pinMode(Pyro1, OUTPUT);
  pinMode(Pyro2, OUTPUT);
  pinMode(RED, OUTPUT);
  pinMode(BLU, OUTPUT);
  pinMode(GRN, OUTPUT);
  pinMode(BUZZER, OUTPUT);

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
   if (Serial.available()) {
    char command = Serial.read();
    if (command == 'S') {  // Start simulation
      isSimulating = true;
      simStartTime = millis();
      sim_altitude = 0;
      sim_velocity = 0;
      sim_accel = 0;
      Serial.println("Simulation started");
    }
  }

  NavData nav = isSimulating ? SITL_GetNav() : GetNav();
  int estApogee = predictApogee(nav.altitude_m, nav.accel_ms2);
  DeployBrakes(estApogee);
  SetState(nav.altitude_m, nav.accel_ms2, nav.velocity_m_s);
  HandleState();

 
}

void HandleState() {
  switch(State) {
    case 0: HandleIdle(); break;
    case 1: LaunchDetect(); break;
    case 2: Boost(); break;
    case 3: Coast(); break;
    case 4: Descent(); break;
    case 5: Safe(); break;
    default:   break;
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
  switch(State) {
    case 1:
      if (accel > 30) State = 2;
      break;
    case 2:
      if (accel >= 0) State = 3;
      break;
    case 3:
      if (v <= 0) State = 4;
      break;
    case 4:
      if (altitude < groundLevel_m + 5) State = 5;
      break;
    default:
      break;
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

void WriteToFile() {
  flightData = SD.open("flightData.txt", FILE_WRITE);

  if (flightData) {
    for (const FlightRecord& record : flightLog) {
      // Clear and format dataStr safely
      snprintf(dataStr, sizeof(dataStr), "%lu,%.1f,%.1f,%.2f,%.1f",
               record.timestamp,
               record.pressure,
               record.altitude,
               record.acceleration,
               record.estimation);

      flightData.println(dataStr);
      if (!flightData.println(dataStr)) {
      Serial.println("Write failed");
      break;
    }
    flightData.close();  // Important!
    }
  } else {
    Serial.println("error opening flightData.txt");
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

  static unsigned long lastTime = 0;
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  data.velocity_m_s += data.accel_ms2 * dt;
  lastTime = now;


  return data;
}

float getInterpolatedThrust(float time_s) {
  if (time_s <= thrust_curve[0].time_s) return thrust_curve[0].thrust_N;
  if (time_s >= thrust_curve[NUM_THRUST_POINTS - 1].time_s) return 0.0;

  for (int i = 0; i < NUM_THRUST_POINTS - 1; ++i) {
    float t1 = thrust_curve[i].time_s;
    float t2 = thrust_curve[i + 1].time_s;

    if (time_s >= t1 && time_s <= t2) {
      float thrust1 = thrust_curve[i].thrust_N;
      float thrust2 = thrust_curve[i + 1].thrust_N;

      float ratio = (time_s - t1) / (t2 - t1);
      return thrust1 + ratio * (thrust2 - thrust1);
    }
  }

  return 0.0; // shouldn't reach here
}

NavData SITL_GetNav() {
  NavData data;

  unsigned long now = millis();
  float t = (now - simStartTime) / 1000.0f; // seconds since sim start
  float dt = 0.02f;  // fixed timestep

  float thrust = getInterpolatedThrust(t);

  float drag = 0.5f * simConfig.air_density *
               sim_velocity * sim_velocity *
               simConfig.drag_coefficient *
               simConfig.cross_section_area_m2;

  if (sim_velocity < 0) drag *= -1;

  float weight = simConfig.mass_kg * 9.81f;
  float net_force = thrust - drag - weight;

  sim_accel = net_force / simConfig.mass_kg;
  sim_velocity += sim_accel * dt;
  sim_altitude += sim_velocity * dt;

  data.altitude_m = sim_altitude;
  data.accel_ms2 = sim_accel;
  data.velocity_m_s = sim_velocity;

  return data;
}


void HandleIdle() {}
void LaunchDetect() {}
void Boost() {}
void Coast() {}
void Descent() {}
void Safe() {}