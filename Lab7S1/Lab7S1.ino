/* Lab7_I2C_gyro_serial_plotter.ino
 Written by: Clark Hochgraf, revised: Feb 18, 2019 
 Description: ++ PLEASE READ ALL COMMENTS ++
       Demonstrates calculation of roll, pitch and yaw angles using a gyroscope sensor. 
       The gyro outputs the rate of rotation in degrees per second.
       The gyro signal is the integrated (multipled by sample interval) to get degrees.
       Hardware: uses the MPU-6050 6-axis accelerometer and gyro
       Software: uses library for MPU6050, on personal machines, you will have to install 
*/
#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu; // declare a variable called mpu of datatype MPU6050
unsigned long timeStampStartOfLoopMs = 0;
float timeStepS = 0.01;
float pitch,roll,yaw = 0.0f; // pitch, roll and yaw values
Vector normalizedGyroDPS; //stores the three gyroscope readings XYZ in degrees per second (DPS)
//==============================================================================

void setup() {
  Serial.begin(115200);
  // Initialize MPU6050 to have full scale range of 2000 degrees per second
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G)) {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring.");
    delay(1000);
  }
  mpu.calibrateGyro(); // Calibrate gyroscope- must be done with sensor not moving.
  mpu.setThreshold(1);// sets level below which changes in gyro readings are ignored.
                 // helps to reduce noise. 1 = one standard deviation. Range is 0 to 3.
} // setup

//==============================================================================
void loop() {
  timeStampStartOfLoopMs = millis(); // mark the time 
  normalizedGyroDPS = mpu.readNormalizeGyro(); // Read normalized values
  // Calculate Pitch, Roll and Yaw
  pitch = pitch + normalizedGyroDPS.YAxis * timeStepS;
  roll = roll + normalizedGyroDPS.XAxis * timeStepS;
  yaw = yaw + normalizedGyroDPS.ZAxis * timeStepS;
  Serial.print(pitch);
  Serial.print(" ");
  Serial.print(roll);
  Serial.print(" ");
  Serial.println(yaw);  
  // Wait until a full timeStepS has passed before next reading
  delay((timeStepS*1000) - (millis() - timeStampStartOfLoopMs));
} //loop
