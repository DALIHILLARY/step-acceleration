#include <Arduino.h>
#include <Wire.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <TinyGPS++.h>
#include "I2Cdev.h"
#include <math.h>
#include <HardwareSerial.h>
#include <SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library.h>
#define Rak3172_TX 17
#define Rak3172_RX 16 
#define GPS_TX 26
#define GPS_RX 27
#define GPS_SWITCH 14
#define INTERRUPT_PIN 2
#define FIFO_BUFFER_SIZE 42 // maximum packet size plus 2 for the header bytes
MPU6050 mpu;
HardwareSerial gpsSerial(1);
float HOME_LATITUDE = 0.0;
float HOME_LONGITUDE = 0.0;
SFE_MAX1704X lipo;
/* Headings to determine movement direction
   Used to see if the direction user taking leads home or away
  */
 //function prototypes
void calibrateMPU6050();
void increaseGPSSamplingRate();
void decreaseGPSSamplingRate();
float getDistanceFromOrigin();
float getGPSDistanceFromOrigin();
void applyLowPassFilter();
float makovStepLength();
void computeHeading();
void readGPS();
void scanI2c();
void toggleGps();//function to toggle gps power On or Off
bool CheckGps();
void sampleGPS();
void init_rak3172();
bool stepDetected();
float HIGHER_HEADING = 0.0;
float LOWER_HEADING = 0.0;
#define HEADING_DRIFT 30 // this will be added and subtracted from the current heading

float LAST_GPS_DISTANACE = 0.0; // for comparison with current /positive or negative direction

#define HIGHEST_GPS_SAMPLING_INTERVAL 300 // The lowest GPS is allowed to sample 5 min
#define LOWEST_GPS_SAMPLING_INTERVAL 30   // The highest GPS is allowed to sample 30 seconds
#define SAMPLING_FACTOR 2                 // The multipling or decreasing factor for change in sampling

int currentGPSSamplingInterval = LOWEST_GPS_SAMPLING_INTERVAL; // begin off with the lowest sampling rate
unsigned long lastGPSSamplingTime = millis();                  // the last time the GPS was sampled

bool isOutsidePerimeter = false;
bool isMovingAway = false;

unsigned long stepCounter = 0;

//REMOVE THE DEFAULT COORDINATES
float currentLatitude = 0.33237001299858093261;
float currentLongitude = 32.57057189941406250000;

float HOME_RADIUS = 20.0; // in meters

TinyGPSPlus gps; // The TinyGPS++ object

// END GPS SECTION

// TRANSIMISSIONS SECTION
unsigned long previousIdleTransmission = millis();
unsigned long previousActiveTransmission = millis();
uint8_t fifoBuffer[FIFO_BUFFER_SIZE];

float gyroHeading;

float previous_step_length = 0.0; // Initialize previous step length to 0

struct Postion
{
  float x;
  float y;
};
Postion getCurrentPosition();
Postion previous_position = {0.0, 0.0}; // Initialize position to (0,0)
// Postion getCurrentPosition();
// MPU6050 calibration values
int16_t accelXOffset = 0;
int16_t accelYOffset = 0;
int16_t accelZOffset = 0;

int16_t accelX, accelY, accelZ;

// Variables for step detection
const float threshold = 1.0; // Adjust this value according to your sensor and environment
float previousAccel = 0.0;
unsigned long previousTime = 0;

const float accThreshold = 20000;
const float K = 0.73603;
bool isAbove = false;
bool isBelow = true;
float time1 = 0;
float time2 = 0;

float peak = 0;
float valley = 0;

void setup()
{
  Wire.begin();
  Serial.begin(115200);
  Serial2.begin(115200,SERIAL_8N1,Rak3172_RX,Rak3172_TX); // GPS baud rate
  gpsSerial.begin(9600,SERIAL_8N1,GPS_RX,GPS_TX);
  mpu.initialize();
  mpu.setFullScaleAccelRange(0x00);
  pinMode(INTERRUPT_PIN, INPUT);
  pinMode(GPS_SWITCH, OUTPUT); // use digital pin 18 to toggle the GPS
  scanI2c();
  if (lipo.begin() == false) // Connect to the MAX17048 using the default wire port
  {
    Serial.println(F("MAX17048 not detected. Please check wiring. Freezing."));
  }
  // Calibrate MPU6050
  calibrateMPU6050();
  init_rak3172(); //initialize lorawan communcation
 //gpsSerial.println("$PMTK161,0*28");//this turn the gps off
  //Serial.println(CheckGps());
  //toggleGps();
  //CheckGps();
  // readGPS();
}
void loop()
{
  //readGPS();
  while(Serial.available())
  {
    String command=Serial.readString();
    if(command.startsWith("AT"))
    {
      Serial2.println(command);
    }
    else if(command.startsWith("$PMTK"))
    {
      gpsSerial.println(command);
    }
    else if(command=="rr")
    {
      esp_restart();
      //esp_cpu_reset();
    }
    else;
  }
  while(Serial2.available())
  {
    Serial.println(Serial2.readString());
  }
  // if(gpsSerial.available()>0)
  // {
  //   Serial.print(char(gpsSerial.read()));
  // }
  if (stepDetected())
  {
    previous_position = getCurrentPosition();
    Serial.print("heading: ");
    Serial.println(gyroHeading);
    Serial.print("Step Length: ");
    Serial.println(previous_step_length);
    Serial.print("Position: ");
    Serial.print("\t");
    Serial.print(previous_position.x);
    Serial.print("\t");
    Serial.println(previous_position.y);
    Serial.print("\t");
    Serial.print("Steps: ");
    Serial.print(stepCounter);
    Serial.print(F("Distance travelled: "));
    Serial.println(getDistanceFromOrigin());
    Serial.println(F("================================================================="));

    // optimalGPSAlgorithm()  //Call optimal code usage //uncommend to enable

    // Check is user is already outside the perimeter
    if (isOutsidePerimeter)
    {
      if (getDistanceFromOrigin() <= HOME_RADIUS)
      {
        Serial.println("Is back in perimeter detected by inertial sensors.");
      }
      // TODO : Send distress signals to concerned people every 1 minute
      if (millis() - previousActiveTransmission >= 60000)
      {
        readGPS();                                          // Check the GPS on updated position
        float currentPosition = getGPSDistanceFromOrigin(); // Get the current position
        // Check if the new position is within the HOME_RADIUS
        if (currentPosition <= HOME_RADIUS)
        {
          Serial.println("INFO 4 gps: Person is Back in Perimeter");
          isOutsidePerimeter = false; // set flag is in perimeter
          isMovingAway = false;
          // TODO: reset the postioning coordinates for inertia sensor
        } 
        previousActiveTransmission = millis();
        // TODO : Send distress signals to concerned people
      }
    }
    else
    {
      // Check distance from origin not to exceed HOME_RADIUS
      if (getDistanceFromOrigin() >= HOME_RADIUS)
      {
        Serial.println("out of perimeter detected by inertial sensors.");
        readGPS();                                          // Check the GPS on updated position
        float currentPosition = getGPSDistanceFromOrigin(); // Get the current position

        // Check if the new position is within the HOME_RADIUS
        if (currentPosition <= HOME_RADIUS)
        {
          Serial.println("WARNING: Was a false alarm");
          isOutsidePerimeter = false; // set flag is in perimeter
          isMovingAway = false;
          // TODO: reset the postioning coordinates for inertia sensor
        }
        else
        {
          Serial.println("WARNING: User out of perimeter");
          isMovingAway = true;
          isOutsidePerimeter = true; // set flag is outside perimeter

          // TODO : Send distress signals to concerned people every 1 minute
          if (millis() - previousActiveTransmission > 60000)
          {
            previousActiveTransmission = millis();
            // TODO : Send distress signals to concerned people
          }
        }
        LAST_GPS_DISTANACE = currentPosition;
      }
      else
      {
        isOutsidePerimeter = false; // set the flag they are in perimeter
        // Serial.println("Within HOME_RADIUS");

        // TODO : Check if last idle transmission is more than 30 minutes
        if (millis() - previousIdleTransmission > 1800000)
        {
          // TODO : Send idle transmission to concerned people
          previousIdleTransmission = millis();
        }
      }
    }
    Serial.print("Voltage: ");
    Serial.print(lipo.getVoltage());  // Print the battery voltage
    Serial.print("V");

    Serial.print(" Percentage: ");
    Serial.print(lipo.getSOC(), 2); // Print the battery state of charge with 2 decimal places
    Serial.println("%");
  }

  delay(10); // delay to sample meaningful values
}
// Function to optimize GPS sampling
void optimalGPSAlgorithm()
{
  // Check is user is already outside the perimeter
  if (isOutsidePerimeter)
  {
    // TODO: Check the current heading if it the same drift or off
    if (LOWER_HEADING <= gyroHeading && gyroHeading >= HIGHER_HEADING)
    {
      if (((millis() - lastGPSSamplingTime) / 1000) >= currentGPSSamplingInterval)
      {
        Serial.println("INFO: Next GPS read reached");
        sampleGPS();
      }
    }
    else
    {
      Serial.println("INFO: User made a turn");
      HIGHER_HEADING = int(gyroHeading + HEADING_DRIFT) % 360;
      LOWER_HEADING = int(gyroHeading - HEADING_DRIFT) % 360;

      sampleGPS();
    }
  }
  else
  {
    // Check distance from origin not to exceed HOME_RADIUS
    if (getDistanceFromOrigin() >= HOME_RADIUS)
    {
      Serial.println("Inertia out of perimeter");
      readGPS();                                          // Check the GPS on updated position
      float currentPosition = getGPSDistanceFromOrigin(); // Get the current position

      // Check if the new position is within the HOME_RADIUS
      if (currentPosition <= HOME_RADIUS)
      {
        Serial.println("WARNING: Was a false alarm");
        isOutsidePerimeter = false; // set flag is in perimeter
        isMovingAway = false;
        // TODO: reset the postioning coordinates for inertia sensor
      }
      else
      {
        Serial.println("WARNING: User out of perimeter");
        isMovingAway = true;
        isOutsidePerimeter = true; // set flag is outside perimeter
        HIGHER_HEADING = int(gyroHeading + HEADING_DRIFT) % 360;
        LOWER_HEADING = int(gyroHeading - HEADING_DRIFT) % 360;

        // TODO : Send distress signals to concerned people every 1 minute
        if (millis() - previousActiveTransmission > 60000)
        {
          previousActiveTransmission = millis();
          // TODO : Send distress signals to concerned people
        }
      }
      LAST_GPS_DISTANACE = currentPosition;
    }
    else
    {
      isOutsidePerimeter = false; // set the flag they are in perimeter
      // Serial.println("Within HOME_RADIUS");

      // TODO : Check if last idle transmission is more than 30 minutes
      if (millis() - previousIdleTransmission > 1800000)
      {
        // TODO : Send idle transmission to concerned people
        previousIdleTransmission = millis();
      }
    }
  }
}

// Function to detect a step
bool stepDetected()
{
  bool detectedStep = false;
  computeHeading();     // compute heading from gyroscope data
  applyLowPassFilter(); // smooth out the acceleration data

  // Calculate the magnitude of acceleration
  float accelMagnitude = sqrt(pow(accelX, 2) + pow(accelY, 2) + pow(accelZ, 2));

  if (accelMagnitude >= accThreshold && isBelow)
  {
    peak = accelMagnitude;
    isAbove = true;
    isBelow = false;
    time1 = millis();
  }
  else if (accelMagnitude <= accThreshold && isAbove)
  {
    time2 = millis();
    isAbove = false;
    isBelow = true;

    // compute step length
    previous_step_length = makovStepLength();

    if (previous_step_length <= 0.4 ) {
      return false;
    }
    stepCounter = stepCounter + 1;
    valley = accelMagnitude;
    detectedStep = true;
  }

  if (isBelow && accelMagnitude < valley)
  {
    valley = accelMagnitude;
  }
  else if (isAbove && accelMagnitude > peak)
  {
    peak = accelMagnitude;
  }
    //return true;

  return detectedStep;
}

// Function to compute the heading
void computeHeading()
{
  // float gyroX = mpu.getRotationX() / 131.0;
  // float gyroY = mpu.getRotationY() / 131.0;
  // float gyroZ = mpu.getRotationZ() / 131.0;

  // gyroHeading = atan2(gyroY, gyroX) * 180 / M_PI;

  // if (gyroHeading < 0)
  // {
  //   gyroHeading += 360;
  // }
    // get the latest sensor readings from the MPU6050
  mpu.getFIFOBytes(fifoBuffer, sizeof(fifoBuffer)); // read the fifo data into the fifoBuffer

  // check if there is new data in the FIFO buffer
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
  {
    // Get the quaternion values
    Quaternion q;
    mpu.dmpGetQuaternion(&q, fifoBuffer);

    // Calculate the gyroHeading angle
    gyroHeading = atan2(2 * q.x * q.y - 2 * q.w * q.z, 2 * q.w * q.w + 2 * q.x * q.x - 1);
    gyroHeading = gyroHeading * 180 / M_PI; // Convert from radians to degrees
    if (gyroHeading < 0)
    {
      gyroHeading += 360; // Ensure that gyroHeading is between 0 and 360 degrees
    }
  }
  
}

// Function to apply a low-pass filter to acceleration data
void applyLowPassFilter()
{
  const float alpha = 0.2; // Smoothing factor
  float ax_filtered, ay_filtered, az_filtered;

  mpu.getAcceleration(&accelX, &accelY, &accelZ);

  ax_filtered = alpha * accelX + (1 - alpha) * (accelX - accelXOffset);
  ay_filtered = alpha * accelY + (1 - alpha) * (accelY - accelYOffset);
  az_filtered = alpha * accelZ + (1 - alpha) * (accelZ - accelZOffset);

  accelX = (int16_t)ax_filtered;
  accelY = (int16_t)ay_filtered;
  accelZ = (int16_t)az_filtered;
}

// Function to calibrate MPU6050
void calibrateMPU6050()
{
  // Load the DMP firmware onto the MPU6050
  uint8_t devStatus = mpu.dmpInitialize();
  if (devStatus == 0)
  {
    mpu.setDMPEnabled(true);
    Serial.println(F("DMP initialized."));
  }
  else
  {
    Serial.print(F("DMP initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  const int numSamples = 100;

  for (int i = 0; i < numSamples; i++)
  {
    int16_t accelX = mpu.getAccelerationX();
    int16_t accelY = mpu.getAccelerationY();
    int16_t accelZ = mpu.getAccelerationZ();

    accelXOffset += accelX;
    accelYOffset += accelY;
    accelZOffset += accelZ;

    delay(10);
  }

  accelXOffset /= numSamples;
  accelYOffset /= numSamples;
  accelZOffset /= numSamples;
}

float makovStepLength()
{
  // Calculate step length
  return K * (time2 - time1) * pow(peak - valley, 0.25) / 1000;
}

// Function to compute current position  {X, Y}
Postion getCurrentPosition()
{
  float heading = radians(gyroHeading); // convert to radians for actual values. Try cos(270)
  float x_new = previous_position.x + previous_step_length * cos(heading);
  float y_new = previous_position.y + previous_step_length * sin(heading);
  return {x_new, y_new};
}

// Function to compute distance between {0.0} and current position
float getDistanceFromOrigin()
{
  return sqrt(pow(previous_position.x, 2) + pow(previous_position.y, 2));
}

// get current GPS coordinates latitude and longitude
void readGPS()
{
  // if (digitalRead(GPS_SWITCH) == 0)
  // {
  //   digitalWrite(GPS_SWITCH, HIGH); // switch on gps, to get intial readings
  //   delay(100);                     // latch for operation
  //   Serial.println("GPS started");
  // }
  //gpsSerial.println("$PMTK161,0*28"); //this turn the gps on//
  toggleGps();
  while (gpsSerial.available() > 0)
  {
    //Serial.print(char(gpsSerial.read()));
    gps.encode(gpsSerial.read());
    if (gps.location.isValid() && gps.location.isUpdated())
    {
      // Serial.println("GPS Connected");
      currentLatitude = gps.location.lat();
      currentLongitude = gps.location.lng();
      Serial.print("Latitude: ");
      Serial.print(currentLatitude, 20);
      Serial.print(" Longitude: ");
      Serial.println(currentLongitude, 20);

      lastGPSSamplingTime = millis();

      digitalWrite(GPS_SWITCH, LOW); // Turn off GPS
      toggleGps();
      Serial.println("GPS OFF");
      gpsSerial.println("$PMTK161,0*28");//this turn the gps off
      break;
    }
  }
}
// Function to compute GPS distance from home/ origin
float getGPSDistanceFromOrigin()
{
  float R = 6371;                                              // Radius of the earth in km
  float dLat = (currentLatitude - HOME_LATITUDE) * M_PI / 180; // deg2rad below
  float dLon = (currentLongitude - HOME_LONGITUDE) * M_PI / 180;
  float a =
      sin(dLat / 2) * sin(dLat / 2) +
      cos(HOME_LATITUDE * M_PI / 180) * cos(currentLatitude * M_PI / 180) *
          sin(dLon / 2) * sin(dLon / 2);
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));
  float d = R * c; // Distance in km
  d = d * 1000;
  return d;
}

// Function to compute distance between two points
float getDistanceBtnCoordinates(float lat1, float lon1, float lat2, float lon2)
{
  float R = 6371;                          // Radius of the earth in km
  float dLat = (lat2 - lat1) * M_PI / 180; // deg2rad below
  float dLon = (lon2 - lon1) * M_PI / 180;
  float a =
      sin(dLat / 2) * sin(dLat / 2) +
      cos(lat1 * M_PI / 180) * cos(lat2 * M_PI / 180) *
          sin(dLon / 2) * sin(dLon / 2);
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));
  float d = R * c; // Distance in km
  d = d * 1000;
  return d;
}

// Function to get GPS and determine next sampling rate
void sampleGPS()
{

  readGPS(); // Check the GPS on updated position
  float currentDistance = getGPSDistanceFromOrigin();
  if (currentDistance > LAST_GPS_DISTANACE)
  {
    isMovingAway = true;
    // decrease the sampling rate since user is moving further away but go below the LOWEST_GPS_SAMPLING_INTERVAL
    decreaseGPSSamplingRate();
  }
  else
  {
    if (currentDistance <= HOME_RADIUS)
    {
      isOutsidePerimeter = false;
    }
    isMovingAway = false;
    // increase the sampling rate since user is coming back, no worries here
    increaseGPSSamplingRate();
  }
  LAST_GPS_DISTANACE = currentDistance;
}

// Function to decrease GPS sampling rate
void increaseGPSSamplingRate()
{
  // Serial.println("INFO: Increasing sampling rate");
  currentGPSSamplingInterval = currentGPSSamplingInterval / SAMPLING_FACTOR;
  currentGPSSamplingInterval = max(currentGPSSamplingInterval, LOWEST_GPS_SAMPLING_INTERVAL);
}

// Function to increase GPS sampling rate
void decreaseGPSSamplingRate()
{
  // Serial.println("INFO: Decreasing sampling rate");
  currentGPSSamplingInterval = currentGPSSamplingInterval * SAMPLING_FACTOR;
  currentGPSSamplingInterval = min(currentGPSSamplingInterval, HIGHEST_GPS_SAMPLING_INTERVAL);
}
void init_rak3172()
{
  Serial.println(F("initializing rak3172 module for LORAWAN"));
  Serial2.println("AT");
  while(!Serial2.available());
  Serial.println(Serial2.readString());
  Serial2.println("AT+NWM=1");
  while(!Serial2.available());
  Serial.println(Serial2.readString());
  Serial2.println("AT+NJM=1");
  while(!Serial2.available());
  Serial.println(Serial2.readString());
  Serial2.println("AT+CLASS=A");
  while(!Serial2.available());
  Serial.println(Serial2.readString());
  Serial2.println("AT+BAND=4");
  while(!Serial2.available());
  Serial.println(Serial2.readString());
}
bool CheckGps()
{
  bool gpsStatus=false;
  if(gpsSerial.available()>0)
  {
    Serial.println(F("GPS is currently on"));
    gpsStatus=true; // gps is actually on;
  }
  else if(!gpsSerial.available())
  {
    Serial.println(F("GPS is turned off, or in standby mode"));
    gpsStatus=false;
  }
  return gpsStatus;
}
void toggleGps()
{
  if (CheckGps()==true)
  {
    Serial.println(F("Turing gps off with PMTK command"));
    gpsSerial.println("$PMTK161,0*28");
    CheckGps();
  }
  else if (CheckGps()==false)
  {
    Serial.println(F("Turing gps on now"));
    gpsSerial.println("$PMTK161,0*28"); //sending any data on the gps serial port will turn it back on from standby mode according to the datasheet;
    CheckGps(); // confirm turing on;
  }
}
void scanI2c() {
  byte error, address;
  int nDevices = 0;

  delay(5000);
  Wire.begin();
  Serial.println("Scanning for I2C devices ...");
  for(address = 0x01; address < 0x7f; address++){
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0){
      Serial.printf("I2C device found at address 0x%02X\n", address);
      nDevices++;
    } else if(error != 2){
      Serial.printf("Error %d at address 0x%02X\n", error, address);
    }
  }
  if (nDevices == 0){
    Serial.println("No I2C devices found");
  }
}