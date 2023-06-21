#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

// MPU6050 calibration values
int16_t accelXOffset = 0;
int16_t accelYOffset = 0;
int16_t accelZOffset = 0;

// Variables for step detection
const float threshold = 1.0; // Adjust this value according to your sensor and environment
const lowPass = 1000;
float previousAccel = 0.0;
unsigned long previousTime = 0;
float stepLength = 0.0;
const float lowPass = 1200;

// Define window size
const int windowSize = 10;

// Define threshold multiplier (adjust this value as needed)
const float thresholdMultiplier = 1.5;

// Initialize acceleration values and peak detection variables
float accelerationValues[windowSize];
float baseline = 0;
float upperThreshold = 0;

void setup() {
  Wire.begin();
  Serial.begin(9600);

  mpu.initialize();
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  
  // Calibrate MPU6050
  calibrateMPU6050();
}

void loop() {
  // Read acceleration data from MPU6050
  int16_t accelX = mpu.getAccelerationX() - accelXOffset;
  int16_t accelY = mpu.getAccelerationY() - accelYOffset;
  int16_t accelZ = mpu.getAccelerationZ() - accelZOffset;

  Serial.println(accelX);
  Serial.println(accelY);
  Serial.println(accelZ);  
  
  // Calculate the magnitude of acceleration
  float accelMagnitude = sqrt(pow(accelX, 2) + pow(accelY, 2) + pow(accelZ, 2));  
  // Serial.print("Acceleration: ");
  // Serial.print(accelMagnitude);
  // Serial.print("      Previous Magnitude: ");
  // Serial.println(previousAccel);
  // Serial.print("Acceleration: ");
  // Serial.println(accelMagnitude);
  
  // Check for step detection
  if (isPeak(accelMagnitude)) {
    unsigned long currentTime = millis();
    unsigned long deltaTime = currentTime - previousTime;

    // Calculate step length using integration
    float velocity = (previousAccel + accelMagnitude) * (deltaTime / 2.0) / 1000.0; // Convert deltaTime to seconds
    stepLength = velocity * (deltaTime / 1000.0); // Convert deltaTime to seconds

    previousAccel = accelMagnitude;
    previousTime = currentTime;

    // Serial.print("Step Length: ");
    // Serial.print(stepLength);
    // Serial.println(" meters");
  }
  delay(100); //delay to sample meaningful values
}

// Function to calibrate MPU6050
void calibrateMPU6050() {
  const int numSamples = 100;
  
  for (int i = 0; i < numSamples; i++) {
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

// Function to detect peak in acceleration magnitude
bool isPeak(float accelMagnitude) {
  if(accelMagnitude < lowPass) {
    return false;
  }
  static float previousMagnitude = 0.0;
  
  if (accelMagnitude > previousMagnitude && accelMagnitude > threshold) {
    previousMagnitude = accelMagnitude;
    return true;
  }
  
  previousMagnitude = accelMagnitude;
  return false;
}

bool detectStepWindowed()
{
  float acceleration = kalAngleY;
  // Add acceleration value to window
  for (int i = 0; i < windowSize - 1; i++)
  {
    accelerationValues[i] = accelerationValues[i + 1];
  } 
  accelerationValues[windowSize - 1] = acceleration;

  // Compute baseline and threshold
  baseline = computeBaseline(accelerationValues, windowSize);
  upperThreshold = baseline + thresholdMultiplier * computeStdDev(accelerationValues, windowSize);
  lowerThreshold = baseline - thresholdMultiplier * computeStdDev(accelerationValues, windowSize);

  bool executeAcceleration = false; //If we qualify it to be a step
  
  for (int i = 1; i < windowSize - 1; i++)
  {
    // Check for peaks within window
    if (accelerationValues[i] > upperThreshold && accelerationValues[i] > accelerationValues[i - 1] && accelerationValues[i] > accelerationValues[i + 1])
    {
      if(!isSlope || isStart) {
        executeAcceleration = true;
        isSlope = true;
        isStart = false;
        lastPeak = accelerationValues[i];     
      }
    }else if (accelerationValues[i] < lowerThreshold && accelerationValues[i] < accelerationValues[i - 1] && accelerationValues[i] < accelerationValues[i + 1])
    {
          // Check for valleys within window

      if(isSlope || isStart){
        executeAcceleration = true;
        isSlope = false;
        isStart = false;
        lastValley =  accelerationValues[i];       
      }
    }

    if (lastPeak != -999 && lastValley != -999 && executeAcceleration) {
      return true;   
    }
  }


  return false;
}

