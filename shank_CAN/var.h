#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "driver/twai.h"

#define RX_PIN 19
#define TX_PIN 18
static bool driver_installed = false;

const int fsrPins[] = {34, 36, 39, 35};//make 34 heel, 35 side lower, 36 side upper, 39 ball
const int numFsrs = 4;
const float ALPHA = 0.2; //lowpass filter constant
const float alpha_imu = 0.98; //complementary filter const
float shank=0;
int lowPassValues[numFsrs] = {0, 0, 0, 0};
int fsrOffsets[numFsrs] = {494,496,495,494};//{530,577,582,574};
int mpuOffsets[6] = {664,	-156,	736, -276, -183, -144};
int16_t gyroXOffset ;
int16_t gyroYOffset ;
int16_t gyroZOffset ;


float shankAngle = 0;
float angleXOffset=0;

int gaitPhase=2;

int fsrValues[numFsrs];
float CoPx;
float CoPy;
unsigned long currentTime;

float coords[][2] = {
  {-2, 12.6}, 
  {-4.6, 3.4}, 
  {-1.5, 7.2}, 
  {-7.5, 18.0} 
};

typedef struct struct_message {
  int time;
  int gaitphase;
  float shankAngle;
  int fsr1;
  int fsr2;
  int fsr3;
  int fsr4;
} struct_message;

struct_message send;


MPU6050 mpu;

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 av;         // [x, y, z]            accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

//MPU6050 mpu6050(Wire);
float angleX = 0;
float angleY = 0;