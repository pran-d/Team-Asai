#include "var.h"

void setup() {
  Serial.begin(115200);
  Wire.begin();

  for(int i=0;i<4;i++)
  {
    pinMode(keyPins[i], INPUT_PULLUP);
  }

  startTime = millis();
  Wire.setClock(200000); // 400kHz I2C clock. Comment this line if having compilation difficulties

  IMU_initialize();
  CAN_initialize();
  driver_installed = true;

  ZeroMode(0x01);  //This is needed for when the code starts running when the motor is on and has already been used ig??s
  EnterMode(0x01);

  initESPNow();
}


void loop(){

  //Extension_Control(-10, 130, 0, 5);

  // IMU_update();
  // float pRef = get_motor_position_from_leg_position(angleX)/(33);
  // Serial.print(angleX); Serial.print(",");
  // Serial.println(pRef);
  // Position_Control(pRef, 6, 2, 0.4);


  while (currentMode==Stair)
    {
      IMU_update();
      CAN_receive();
      // Serial.print(sensors.thighAngle); Serial.print(",");
      // Serial.println(sensors.fsr1);
      switch (currentState) {

        case Walking:
        WalkingStateMachine();
        break;
    
        case Ascent:
        AscentStateMachine();
        break;

        case Descent:
        DescentStateMachine();
        break;
      }
    }


  // Supervisory FSM - decides which mode to be in
  
  /*
  int mode;
  switch(currentMode)
  {
    case Passive:
    while(currentMode==Passive)
    {
      //updateSensors();
      IMU_update();
      WalkingStateMachine();
      mode= readMode();
      if(mode==2)
      {
      currentMode = Stair;
      break;
      }
      else if(mode==3)
      {
      currentMode = Flexion;
      break;
      }
    }
    break;

    case Stair:
    while (currentMode==Stair)
    {
      //updateSensors();
      IMU_update();
      switch (currentState) {
        case Walking:
        WalkingStateMachine();
        break;
        
        case Descent:
        DescentStateMachine();
        break;
    
        case Ascent:
        AscentStateMachine();
        break;
      }
    mode= readMode();
    if(mode==1)
    {
    currentMode = Passive;
    break;
    }
    else if(mode==3)
    {
    currentMode = Flexion;
    break;
    }
    }
    break;

    case Flexion:
    while(currentMode==Flexion)
    {
      //updateSensors();
      IMU_update();
      mode= readMode();
      if(mode==1)
      {
        currentMode = Passive;
        break;
        }
      else if(mode==3)
      {
        currentMode = Flexion;
        break;
        }
        }
    break;
  }
  */

}
