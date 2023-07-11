#include <Wire.h>
#include <MPU6050.h>
#include <TinyGPS++.h>
#include "I2Cdev.h"
#include <math.h>

MPU6050 mpu;

float HOME_LATITUDE = 0.0;
float HOME_LONGITUDE = 0.0;

/* Headings to determine movement direction
   Used to see if the direction user taking leads home or away
  */
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

float currentLatitude = 0.0;
float currentLongitude = 0.0;

float HOME_RADIUS = 20.0; // in meters

TinyGPSPlus gps; // The TinyGPS++ object

// END GPS SECTION

// TRANSIMISSIONS SECTION
unsigned long previousIdleTransmission = millis();
unsigned long previousActiveTransmission = millis();

#define GPS_SWITCH 18
#define INTERRUPT_PIN 2
#define FIFO_BUFFER_SIZE 42 // maximum packet size plus 2 for the header bytes
uint8_t fifoBuffer[FIFO_BUFFER_SIZE];

float gyroHeading;

float previous_step_length = 0.0; // Initialize previous step length to 0

struct Postion
{
  float x;
  float y;
};

Postion previous_position = {0.0, 0.0}; // Initialize position to (0,0)

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
  Serial.begin(9600);
  Serial2.begin(9600); // GPS baud rate

  mpu.initialize();
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);

  pinMode(INTERRUPT_PIN, INPUT);
  pinMode(GPS_SWITCH, OUTPUT); // use digital pin 18 to toggle the GPS

  // Calibrate MPU6050
  calibrateMPU6050();
}

void loop()
{
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

    // optimalGPSAlgorithm()  //Call optimal code usage //uncommend to enable

    // Check is user is already outside the perimeter
    if (isOutsidePerimeter)
    {
      // TODO : Send distress signals to concerned people every 1 minute
      if (millis() - previousActiveTransmission >= 1800000)
      {
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
        previousActiveTransmission = millis();
        // TODO : Send distress signals to concerned people
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

  delay(50); // delay to sample meaningful values
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

  return detectedStep;
}

// Function to compute the heading
void computeHeading()
{
  float gyroX = mpu.getRotationX() / 131.0;
  float gyroY = mpu.getRotationY() / 131.0;
  float gyroZ = mpu.getRotationZ() / 131.0;

  gyroHeading = atan2(gyroY, gyroX) * 180 / M_PI;

  if (gyroHeading < 0)
  {
    gyroHeading += 360;
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
  // // Load the DMP firmware onto the MPU6050
  // uint8_t devStatus = mpu.dmpInitialize();
  // if (devStatus == 0)
  // {
  //   mpu.setDMPEnabled(true);
  //   Serial.println(F("DMP initialized."));
  // }
  // else
  // {
  //   Serial.print(F("DMP initialization failed (code "));
  //   Serial.print(devStatus);
  //   Serial.println(F(")"));
  // }

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
  if (digitalRead(GPS_SWITCH) == 0)
  {
    digitalWrite(GPS_SWITCH, HIGH); // switch on gps, to get intial readings
    delay(100);                     // latch for operation
    Serial.println("GPS started");
  }
  while (Serial2.available() > 0)
  {
    // Serial.println(Serial2.read());
    gps.encode(Serial2.read());
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
      Serial.println("GPS OFF");
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