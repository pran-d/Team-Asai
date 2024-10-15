//code reads shank angle, fsr values, filters them, estimates gait phase based on proportions, and sends time, fsr values, gait phase and shank angle to the thigh
//changes to make: need not send fsr values, estimate grf and send
#include "var.h"

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < numFsrs; i++) {
    pinMode(fsrPins[i], INPUT);
  }
  Wire.begin();
  IMU_initialize();
  CAN_initialize();
  driver_installed= true;
  
}

void loop() {
  bool FSR_swing = checkBelowThreshold(25, 10);
  
  applyLowPassFilter(fsrValues, lowPassValues, ALPHA);

  calculateCoP(lowPassValues);
  detectFSRPhase(lowPassValues);
  send.gaitphase=gaitPhase;
  IMU_update();

  send.time = millis();
  send.shankAngle = angleY-90;
  
  send.fsr1=lowPassValues[0];
  send.fsr2=lowPassValues[1];
  send.fsr3=lowPassValues[2];
  send.fsr4=lowPassValues[3];

  // Serial.print("Time: ");
  // Serial.print(send.time);
  // Serial.print(" , Shank angle: ");
  // Serial.print(send.shankAngle);
  int sum_fsrs = send.fsr1+send.fsr2+send.fsr3+send.fsr4;
  Serial.print(sum_fsrs);Serial.print(":  ");
  Serial.print(send.fsr1);Serial.print("  ");
  Serial.print(send.fsr2);Serial.print("  ");
  Serial.print(send.fsr3);Serial.print("  ");
  Serial.print(send.fsr4);Serial.print("  ");
  Serial.print("  , Gait Phase: ");
  Serial.println(gaitPhase);
// delay(50);
  pack_cmd();

}

