#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

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

  mpu.initialize();
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);

  // Calibrate MPU6050
  calibrateMPU6050();
}

void loop()
{

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
    float stepLength = makovStepLength();
    Serial.print("Step Length: "); Serial.print("\t"); Serial.print(stepLength);
    Serial.print("\t");
    Serial.print("Step coefficient: "); Serial.print("\t"); Serial.println(stepLength / K, 6);
    valley = accelMagnitude;
  }

  if (isBelow && accelMagnitude < valley)
  {
    valley = accelMagnitude;
  }
  else if (isAbove && accelMagnitude > peak)
  {
    peak = accelMagnitude;
  }

delay(50); // delay to sample meaningful values
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
    return K * (time2 - time1) * pow(peak - valley, 0.25) / 1000 ;
}

