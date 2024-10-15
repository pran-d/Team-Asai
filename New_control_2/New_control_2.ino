#include "variables.h"

void setup(){
  Serial.begin(115200);
  Wire.begin();
  startTime = millis();
  Wire.setClock(200000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  IMU_initialize();
  CAN_initialize();
  ZeroMode(0x01);  //This is needed for when the code starts running when the motor is on and has already been used ig??s
  EnterMode(0x01);
  pinMode(23,OUTPUT);
  digitalWrite(23, HIGH);
}


void loop() {
  if (Serial.available() > 0) {

    mode = Serial.read();

    switch (mode) {

      // Enter 'm' to restart the ESP
      case 'm': 
        ESP.restart();

      // Enter '1' to enter motor mode. It will start running the last commands it had received. Green ON
      case '1':
        EnterMode(0x01);
        break;

      // Enter '0' to exit motor mode. Green OFF
      case '0':
        ExitMode(0x01);
        break;

      // Enter 'z' to set zero position on motor
      case 'z':
      {
        float kp_extra = kp_in;
        float kd_extra = kd_in;
        ::p_in = 0; ::kp_in = 0; ::kd_in = 0;
        do_each_loop(mode);
        ZeroMode(0x01);
        ::kd_in = kd_extra;
        ::kp_in = kp_extra;
      }
      break;

      // Enter 'c <float> <float>' for setting MIT mode K_p and K_d for motor internal controller
      case 'c':
      {  
        //input gains kp, kd for mit mode pid
        bool validInput = false;
        if (Serial.available() > 0) {
          String input = Serial.readStringUntil('\n');
          input.trim();
          int spaceIndex = input.indexOf(' ');
          Serial.println(spaceIndex);

          if (spaceIndex != -1) {
            String kp_str = input.substring(0, spaceIndex);
            String kd_str = input.substring(spaceIndex + 1);

            if (kp_str.length() > 0 && kd_str.length() > 0) {
              ::kp_in = kp_str.toFloat();
              ::kd_in = kd_str.toFloat();
              Serial.printf("kp_in: %.2f, kd_in: %.2f\n", ::kp_in, ::kd_in);
              validInput = true;
            } else {
              Serial.println("Invalid input. Please provide two float values separated by a space.");
            }
          } else {
            Serial.println("Invalid input. Please provide two float values separated by a space.");
          }
        }
        if(!validInput){
          Serial.println("Invalid input - Please reset parameters");
        }
        break;
      }

      // Enter 'r' to reset all input parameters to 0
      case 'r': 
      {
        reset_inputs();
        break;
      }

      // Enter 'i <iRef>' to ramp torque up to iRef
      case 'i':
      {
        if (Serial.available() > 0) 
        {
          reset_inputs();
          String input = Serial.readStringUntil('\n');
          float iRef = input.toFloat();
          iRef = constrain(iRef, -10, 10);  // reset the torque reference to 0
          int counter = 0; int rampTime = 500;
          while(!Serial.available())
          {
            counter = counter+1;
            if (counter < rampTime) 
            {
              ::t_in = (iRef / rampTime) * counter;
            } 
            else
            {
              ::t_in = iRef;
            }
            // if(abs(::v_out)>8)
            // {
            //   highTorque = true;
            //   ::t_in -= 0.1;
            //   setVelocity(-1);
            // }
            do_each_loop(mode);  
          } 
        }
        ::t_in = 0;
        do_each_loop(mode);
        break;
      }

      // Enter 'j <iRef>, <rampTime>' to ramp torque up to iRef
      case 'j':
      {
        if (Serial.available() > 0) 
        {
          reset_inputs();
          String input = Serial.readStringUntil('\n');
          int delimiterIndex = input.indexOf(',');     
          String iRefStr = input.substring(0, delimiterIndex);
          String rampTimeStr = input.substring(delimiterIndex + 1);

          float iRef = iRefStr.toFloat();
          int rampTime = rampTimeStr.toInt();

          iRef = constrain(iRef, -10, 10); 
          
          int counter = 0;
          int rampDownCounter = 0; bool highTorque = false;

          while(!Serial.available())
          {
            counter = counter+1;
            if (counter < rampTime) 
            {
              ::t_in = (iRef / rampTime) * counter;
            } 
            else
            {
              ::t_in = iRef;
            }
            if(abs(::v_out)>8)
            {
              highTorque = true;
              ::t_in -= 0.1;
              setVelocity(-1);
            }
            do_each_loop(mode);  
          }            
        }
        reset_inputs();
        break;
      }

      // Enter 'p <pRef>' for Position control with feedforward torque
      case 'p':  
        {
          if (Serial.available() > 0) {
            String input = Serial.readStringUntil('\n');
            float pRef = input.toFloat();
            setPosition(pRef, (pRef-::p_out)/abs(pRef-::p_out)*0.3);
          }
          break;
      }

      // Enter '7' Flexion damping to prevent buckling
      case '7':  
      {
        reset_inputs();
        float region1 = 0.4, region2 = 1, region3 = 1.5;
        float k1 = 0.25, k2 = 0.5, k3 = 1;
        while(::p_out < 1.4)
        {
          setPosition(1.8, 0.1);
          do_each_loop(mode);
        }
        while(!Serial.available()) {

          if (-::p_in + ::p_out > 0.05 && -::p_in + ::p_out < region1){
            ::kp_in = 0;
            ::kd_in = 0;
            ::t_in = - k1 * (-::p_in + ::p_out) ;
            Serial.print("1 ");
          }

          else if (-::p_in + ::p_out > region1 && -::p_in + ::p_out < region2){
            ::kp_in = 0;
            ::kd_in = 0;
            ::t_in = -0.3 - k2 * (-::p_in + ::p_out);
            Serial.print("2 ");
          }

          else if (-::p_in + ::p_out > region2 && -::p_in + ::p_out < region3){
            ::kp_in = 0;
            ::kd_in = 0;
            ::t_in = -0.3 - k3 * (-::p_in + ::p_out);
            Serial.print("3 ");
          }

          else if (-::p_in + ::p_out > region3){
            ::kp_in = 0;
            ::kd_in = 0;
            ::t_in = -2;
            if(::v_out > 5){
              ::t_in = -2 - ::v_out * 0.08;
            }
            Serial.print("4 ");
          }

          do_each_loop(mode); 
        }
        ::kd_in = 0; ::kp_in = 0; ::t_in = 0;
        break;
      }

      // Enter 't' for test case: extension control - using trapezoidal velocity setpoint based on time counter
      case 't':  
      {
        if (Serial.available() > 0) {
          String input = Serial.readStringUntil('\n');
          input.trim();
          float vRef = input.toFloat();
          float v_max = vRef;
          // constant velocity at the time being, but with gradual increase and gradual decrease
          ::t_in = 0;            
          ::v_in = 0;
          
          float p_term = 0;
          float i_term = 0;
          
          int counter = 0;

          while(!Serial.available()){
            counter = counter+1;
            i_term += constrain(0.01*(::v_in-::v_out), -5, 5);
            // p_term = 1*(v_in-v_out);
            ::t_in = i_term + p_term;
            if (counter < 1000) 
            {
              ::v_in = -5 + ((v_max + 5) / 1000) * counter;
            } 
            else
            {
              ::v_in = v_max;
            }
            do_each_loop(mode);  
          } 

          // End of motion
          ::v_in = 0;
          ::t_in = 0;
          break;
        }
        do_each_loop(mode); 
        break;
      }

      // Enter 'f <vel>' to move the cable to taut position at velocity 'vel'
      case 'f':
      {
        if (Serial.available() > 0) {
          String input = Serial.readStringUntil('\n');
          input.trim();
          float vRef = input.toFloat();
          cable_taut(vRef);
        }
        break;
      }

      default:
        break;
    }
  }
  do_each_loop('M');
}