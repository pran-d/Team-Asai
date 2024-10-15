#include "driver/twai.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <esp_now.h>
#include <WiFi.h>

MPU6050 mpu;
  
#define ZERO_VELOCITY_BAND 0.5

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
float angleZ = 0;
float alpha = 0.02;  // Complementary filter constant
unsigned long previousTime, currentTime, startTime, lastMessageTime;

#define RX_PIN 18
#define TX_PIN 19
#define TRANSMIT_RATE_MS 100

#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -45.0f
#define V_MAX 45.0f
#define I_MAX 18.0f
#define I_MIN -18.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f

bool logging = true;

//variables  for input to motor, in pack_cmd
float p_in = 0.0f;
float v_in = 0.0f;
float kp_in = 0.0f;  // kp = 5 (Pure velocity control)
float kd_in = 0.0f;  // kd = 2 (Pure Position control)
float t_in = 0.0f;

float kpPid = 0.005;
float kdPid = 0.15;
float kiPid = 0.00017;

//measured values - responses from Motor 1
float p_out = 0.0f;
float v_out = 0.0f;
float t_out = 0.0f;

const int MPU9250_ADDR = 0x68;                  // MPU9250 default I2C address
const float RAD_TO_DEG_CUSTOM = 57.2957795131;  // Custom conversion factor from radians to degrees

const float t_accel = 2000; // Acceleration time in milliseconds
const float t_const = 2000; // Constant velocity time in milliseconds
const float t_decel = 2000; // Deceleration time in milliseconds

static bool driver_installed = false;
unsigned long start_time; // To track the start time of the motion
bool motion_active = false; // To track if motion is active

int flag = 0;
int counter = 0;
u_int32_t GRF;
char mode;

typedef struct struct_message {
  int time;
  int gaitphase;
  u_int32_t fsr1;
  u_int32_t fsr2;
  u_int32_t fsr3;
  u_int32_t fsr4;
  float kneeAngle;
  float thighAngle;
  float shankAngle;
} ;

struct_message sensors;