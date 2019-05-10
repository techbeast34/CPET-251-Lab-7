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

volatile boolean newDataFlag = false; //boolean flag to indicate that TOV1 OVF has occured
unsigned long startMicroseconds, elapsedMicroseconds; //use for profiling time for tasks

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

  TCCR1A |= 0x02;
  TCCR1A &= 0x0E;
  TCCR1B |= 0x1C;
  TCCR1B &= 0xFC;

  cli();
  TIMSK1 |= 0x01;
  ICR1 = 625; //10ms to Top
  sei();

  Serial.println(TCCR1A, HEX);
  Serial.println(TCCR1B, HEX);
  Serial.println(TIMSK1, HEX);
  Serial.println(ICR1);
} // setup

//==============================================================================
void loop() {
  while (!newDataFlag) {}; // stay stuck here until new data arrives, then run loop
                            // this will occur every 10 millisecond
 // elapsedMicroseconds=micros()-startMicroseconds;  // check time for Timer 1 OVF 
  startMicroseconds=micros(); // mark time at start of main loop code
  normalizedGyroDPS = mpu.readNormalizeGyro();
//  elapsedMicroseconds=micros()-startMicroseconds;  // check time for I2C comms
  // Calculate Pitch, Roll and Yaw
  pitch = pitch + normalizedGyroDPS.YAxis * timeStepS;
  roll = roll + normalizedGyroDPS.XAxis * timeStepS;
  yaw = yaw + normalizedGyroDPS.ZAxis * timeStepS;
//  elapsedMicroseconds=micros()-startMicroseconds;  // check time without prints
  Serial.print(pitch);
  Serial.print(F(" "));
  Serial.print(roll);
  Serial.print(F(" "));
  Serial.println(yaw);
  elapsedMicroseconds=micros()-startMicroseconds;  // check total time main loop
  Serial.print(F("elapsed time in microseconds = "));
  Serial.println(elapsedMicroseconds);
  newDataFlag=false;
} //loop


ISR(TIMER1_OVF_vect){
  newDataFlag = true;
}
