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

#define DIM_GREEN 0x00001F
#define DIM_BLUE 0X001F00
#define DIM_RED 0X1F0000
#define LED_CLOCK_PIN 2
#define LED_DATA_PIN 3


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


  //
  TCCR0A |= 0xA3;
  TCCR0A &= 0xAF;
  TCCR0B &= 0xF7;
  //Set timer0 to mode 3 (fast PWM) and set COM bits to clear on compare

  OCR0A = 120;
  OCR0B = 120;

  DDRD |= 0xE0;
  DDRB |= 0x07;

  PORTB |= 0x02;
  PORTB &= 0xFB;
  PORTD |= 0x80;
  //

  pinMode(LED_CLOCK_PIN, OUTPUT);
  pinMode(LED_DATA_PIN, OUTPUT);
} // setup

//==============================================================================
void loop() {
  while (!newDataFlag) {}; // stay stuck here until new data arrives, then run loop
                            // this will occur every 10 millisecond
 // elapsedMicroseconds=micros()-startMicroseconds;  // check time for Timer 1 OVF 
 // startMicroseconds=micros(); // mark time at start of main loop code
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
  //elapsedMicroseconds=micros()-startMicroseconds;  // check total time main loop
  //Serial.print(F("elapsed time in microseconds = "));
  //Serial.println(elapsedMicroseconds);

  display_color_on_RGB_led(DIM_BLUE); // default is dim blue color
  OCR0A=0; // set motor speed off
  OCR0B=0;
  if (yaw>5) {
    display_color_on_RGB_led(DIM_RED);
    OCR0A=80; // set motor speed slow
    OCR0B=80;
    digitalWrite(10, LOW);  //motor direction pins
    digitalWrite(9, HIGH);   //motor direction pins
    digitalWrite(7, HIGH);
    digitalWrite(8, LOW);
  }
  if (yaw < -5) {
    display_color_on_RGB_led(DIM_GREEN);
    OCR0A=80; // set motor speed slow
    OCR0B=80;
    digitalWrite(10, HIGH);  //motor direction pins
    digitalWrite(9, LOW);   //motor direction 
    digitalWrite(7, LOW);
    digitalWrite(8, HIGH);
  }
  
  
  newDataFlag=false;
} //loop

//=========================================
ISR(TIMER1_OVF_vect){
  newDataFlag = true;
}
//=========================================
void display_color_on_RGB_led(unsigned long color) {
  unsigned long bitmask=0UL; // UL unsigned long literal (forces compiler to use long data type)
  unsigned long masked_color_result=0UL;
  
  //PORTC &= 0xDF; //start with clock low.
  digitalWrite(LED_CLOCK_PIN,LOW);
  
  for(int i=23; i>=0; i--) { // clock out one data bit at a time, starting with the MSB first
    bitmask= (1UL<<i);    // build bit mask. Note must use "1UL" unsigned long literal, not "1"
    masked_color_result = color & bitmask; // reveals just one bit of color at time
    boolean data_bit=!(masked_color_result==0); // this is the bit of data to be clocked out.
    
    digitalWrite(LED_DATA_PIN,data_bit);
    
    //if(data_bit){ PORTC |= 0x10; }
    //else{ PORTC &= 0xEF; }
    
    digitalWrite(LED_CLOCK_PIN,HIGH);
    //PORTC |= 0x20;  
    digitalWrite(LED_CLOCK_PIN,LOW);
    //PORTC &= 0xDF;
  }
  digitalWrite(LED_CLOCK_PIN,HIGH);  
  delay(1); // after writing data to LED driver, must hold clock line  
            // high for 1 ms to latch color data in led shift register.
}//display_color_on_RGB_led()
