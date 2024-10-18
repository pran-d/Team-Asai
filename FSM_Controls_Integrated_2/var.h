#include "driver/twai.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <esp_now.h>
#include <WiFi.h>
#include <WebServer.h>

MPU6050 mpu;

const char* ssid = "POCO PHONE";
const char* password = "pranav123";
// const char* ssid = "Redmi Note 9";
// const char* password = "arvind05";
// const char* ssid = "NCAHT IITM";
// const char* password = "Ncaht123$";
WebServer server(80);
bool stopFlag = true;
bool formSubmitted = false;  // Flag to indicate form submission

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards

char motor_active = 'a';
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
float angleX = 0, angleY = 0, angleZ=0;
float angleXdot = 0;
float alpha = 0.02;  // Complementary filter constant
unsigned long previousTime, currentTime, startTime, lastMessageTime;
int mpuOffsets[6] = {-935,	-1062, 626, 82,	-6, -10};


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

#define ZERO_VELOCITY_BAND 0.5

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
bool ERROR_STATE = false;
int ERROR_CODE = 0;

int flag = 0;
int counter = 0;


enum States {Ascent, Descent, Walking};
States currentState = Walking;
enum Phases {Standing, HS, MS, TO, Sw};
Phases currentPhase = Standing;
enum Modes {Passive, Stair, HighStep};
Modes currentMode = Passive;

float theta_t; // Thigh angle
float theta_k; // Knee angle
float omega_t;
float GRF;     // Ground Reaction Force
float GRF_Scale = 1; //Scaling between GRF Sum and User Bodyweight
int FSRs;      // FSR readings: 0=Sw, 1=HS, 2=MS, 3=TO
float w1, w2, w3, w4, w5, bw; // Thresholds for GRF, bw can be approx 0.95 0f the user's bodyweight
float theta1, theta2 ,theta3, theta4, theta5, theta6, theta7; // Angle thresholds

const int keyPins[4]={2,4,13,12};
const int keyPinsLED[4] = {5, 16, 17, 26};

typedef struct struct_message {
  int time;
  int gaitphase;
  int fsr1;
  int fsr2;
  int fsr3;
  int fsr4;
  float kneeAngle;
  float thighAngle;
  float shankAngle;
} ;

struct_message sensors;

uint8_t peerAddress[] =  {0xD8, 0xBC, 0x38, 0xE5, 0xBD, 0x00};//{0x2C , 0xBC, 0xBB, 0x0D, 0x75, 0xF0};

// Thresholds to be set
//Stance 
int StanceThresh = 50;

// Ascent
int FSRascentThresh1 = 200, FSRascentThresh2 = 300; 
float Stair_Ascent_a = 0.00001; float Stair_Ascent_b = 0.027381; float Stair_Ascent_c = 0; //Quadratic for Buildup Torque 
float Stair_Ascent_A = 1; float Stair_Ascent_k = 0.068; // Decay for wind down Torque 
float I_max_Ascent = 4.5; //Saturate Build up at this value

//Descent
int FSRdescentThreshLower = 70, FSRdescentThreshUpper = 300; 
float theta_t_ascent = 35, theta_t_walking = 10;
float imuHighVelThresh = 2, imuFlexionThresh = -2, imuStableAngle = 25;
float descentDeadband = 1.5; float descentKp = 10;

float region1 = 0.4, region2 = 1, region3 = 1.5;
float k1 = 0.25, k2 = 0.5, k3 = 1;
float d1 = 0.025, d2 = 0.05, d3 = 0.1;

float iRef = -4.5, iRef1= -2.0;
// int rampTime1 = 100, rampTime2 = 175;
float knee_AscentRampDown= 50, knee_AscentRampDownStop= 25;