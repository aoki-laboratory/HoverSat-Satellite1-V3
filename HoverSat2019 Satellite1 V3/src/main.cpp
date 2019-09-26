//------------------------------------------------------------------//
//Supported MCU:   ESP32 (M5Stack Grey)
//File Contents:   HoverSat Satellite1
//Version number:  Ver.3.0
//Date:            2019.09.25
//------------------------------------------------------------------//
#include <M5Stack.h>
#include <Servo.h>
#include <Wire.h>
#include <WiFi.h>
#include <time.h>
#include "utility/MPU9250.h"
#include "BasicStepperDriver.h"

//Define
//------------------------------------------------------------------//
#define   TIMER_INTERRUPT 1           // ms

#define   ONE_ROTATION_LENGTH 230
#define   REDUCTION_RATIO 6.25
// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
#define MOTOR_STEPS 400
// Target RPM for cruise speed
#define RPM 120
// Microstepping mode. If you hardwired it to save pins, set to the same value here.
#define MICROSTEPS 1
#define DIR 26
#define STEP 15
// 2-wire basic config, microstepping is hardwired on the driver
BasicStepperDriver stepper(MOTOR_STEPS, DIR, STEP);


//Global
//------------------------------------------------------------------//
int flag = 0;
//Time Counter
unsigned long   cnt1;                 // Timer Interrupt 1ms++

unsigned char   current_time = 0; 

//Pattern

//Flag

//Stepper
const int Stepper_Enable_Pin = 25;
int       Stepper_Enable = 1;
int       motor_accel = 666;
int       motor_decel = 666;

// MPU9250
MPU9250 IMU; 
float accelBiasX = 0;
float accelBiasY = 0;
float accelBiasZ = 0;
float gyroBiasZ = 0;

//Ducted Fan
Servo DuctedFan;
static const int DuctedFanPin = 16;

//Wi-Fi
char ssid[] = "Buffalo-G-0CBA";
char pass[] = "hh4aexcxesasx";

// Time
char ntpServer[] = "ntp.nict.jp";
const long gmtOffset_sec = 9 * 3600;
const int  daylightOffset_sec = 0;
struct tm timeinfo;
String dateStr;
String timeStr;

//SD
File file;
String fname_buff;
const char* fname;

//Timer Interrupt
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
volatile int interruptCounter;
boolean led_stat = 0;

//Parameters
unsigned int ex_length = 200;
unsigned int ex_velocity = 200;
unsigned int ex_accel = 200;
unsigned char hover_val = 0;
unsigned char wait = 5;
unsigned char dir_flag = 1;


//Prototype
//------------------------------------------------------------------//
void IRAM_ATTR onTimer(void);
void getTimeFromNTP(void);
void getTime(void);
void Timer_Interrupt( void );

//Setup
//------------------------------------------------------------------//
void setup() {

  // Initialize M5Stack
  M5.begin();

  // Initialize IIC
  Wire.begin();
  Wire.setClock(400000);

  // Initialize Stepper
  pinMode(Stepper_Enable_Pin, OUTPUT);
  stepper.begin(RPM, MICROSTEPS); 
  digitalWrite( Stepper_Enable_Pin, 1);

  // Initialize MPU9250
  IMU.calibrateMPU9250(IMU.gyroBias, IMU.accelBias);
  IMU.initMPU9250();

  M5.Lcd.setTextSize(3);
  M5.Lcd.setTextColor(GREEN ,BLACK);
  M5.Lcd.fillScreen(BLACK);

  // Connect Wi-Fi
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    M5.Lcd.print(".");
  }

  // Get NTP Time
  getTimeFromNTP();
  getTime();

  // Create Log File
  fname_buff  = "/log/Satellite1_log_"
              +(String)(timeinfo.tm_year + 1900)
              +"_"+(String)(timeinfo.tm_mon + 1)
              +"_"+(String)timeinfo.tm_mday
              +"_"+(String)timeinfo.tm_hour
              +"_"+(String)timeinfo.tm_min
              +".csv";
  fname = fname_buff.c_str();

  // Create Log File
  file = SD.open(fname, FILE_APPEND);
  if( !file ) {
    M5.Lcd.setCursor(5, 160);
    M5.Lcd.println("Failed to open sd");
  }

  // Set Timer Interrupt
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer,  TIMER_INTERRUPT * 1000, true);
  timerAlarmEnable(timer);

  pinMode(17,OUTPUT);
  DuctedFan.attach(DuctedFanPin);
  DuctedFan.write(0);
}

//Main
//------------------------------------------------------------------//
void loop() {
  Timer_Interrupt();

  if( cnt1 >= 10000 ) {
    file.close();
    M5.Lcd.setCursor(5, 160);
    M5.Lcd.println("Complete");
  }

  if( cnt1 >= 5000 && flag==0 ) {  
    digitalWrite( Stepper_Enable_Pin, 0);
    stepper.setSpeedProfile(stepper.LINEAR_SPEED, ex_accel*MOTOR_STEPS*REDUCTION_RATIO/ONE_ROTATION_LENGTH, ex_accel*MOTOR_STEPS*REDUCTION_RATIO/ONE_ROTATION_LENGTH);
    stepper.setRPM(ex_velocity*60*REDUCTION_RATIO/ONE_ROTATION_LENGTH);
    stepper.move(ex_length*MOTOR_STEPS*REDUCTION_RATIO/ONE_ROTATION_LENGTH); 
    flag  = 1;
  }

}

// Timer Interrupt
//------------------------------------------------------------------//
void Timer_Interrupt( void ){
  if (interruptCounter > 0) {
    digitalWrite(17,led_stat);
    led_stat = !led_stat;
    //getTime();
    interruptCounter=0;

    // If intPin goes high, all data registers have new data
    // On interrupt, check if data ready interrupt
    if (IMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
    {  
      IMU.readAccelData(IMU.accelCount);  // Read the x/y/z adc values
      IMU.getAres();

      // Now we'll calculate the accleration value into actual g's
      // This depends on scale being set
      IMU.ax = (float)IMU.accelCount[0]; // - accelBias[0];
      IMU.ay = (float)IMU.accelCount[1]; // - accelBias[1];
      IMU.az = (float)IMU.accelCount[2]; // - accelBias[2];

      IMU.readGyroData(IMU.gyroCount);  // Read the x/y/z adc values
      IMU.getGres();

      // Calculate the gyro value into actual degrees per second
      // This depends on scale being set
      IMU.gx = (float)IMU.gyroCount[0];
      IMU.gy = (float)IMU.gyroCount[1];
      IMU.gz = (float)IMU.gyroCount[2];
    }

    //file = SD.open(fname, FILE_APPEND);
    file.print(cnt1);
    file.print(",");
    file.print(IMU.ax);
    file.print(",");
    file.print(IMU.ay);
    file.print(",");
    file.print(IMU.az);
    file.print(",");
    file.print(IMU.gx);
    file.print(",");
    file.print(IMU.gy);
    file.print(",");
    file.print(IMU.gz);
    file.println(",");
    //file.close();
  }
}


// IRAM
//------------------------------------------------------------------//
void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  interruptCounter=1;
  cnt1+=TIMER_INTERRUPT;
  portEXIT_CRITICAL_ISR(&timerMux);
}

//Get Time From NTP
//------------------------------------------------------------------//
void getTimeFromNTP(void){
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  while (!getLocalTime(&timeinfo)) {
    delay(1000);
  }
}

//Get Convert Time
//------------------------------------------------------------------//
void getTime(void){
  getLocalTime(&timeinfo);
  dateStr = (String)(timeinfo.tm_year + 1900)
          + "/" + (String)(timeinfo.tm_mon + 1)
          + "/" + (String)timeinfo.tm_mday;
  timeStr = (String)timeinfo.tm_hour
          + ":" + (String)timeinfo.tm_min
          + ":" + (String)timeinfo.tm_sec;
  current_time = timeinfo.tm_sec;
}
