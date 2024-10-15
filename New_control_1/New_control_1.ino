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
}

void loop() {
  if (Serial.available() > 0) {
    mode = Serial.read();
    switch (mode) {
      case 'm':
        ESP.restart();
      case '1':
        EnterMode(0x01);
        break;
      case '0':
        ExitMode(0x01);
        break;

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

      case 'c':
      {  //input gains kp, kd for mit mode pid
        bool validInput = false;
        while (!validInput) {
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
        }
        break;
      }

      case 'd':
      {  //Input gains kp,kd,ki for external pid on i
        bool validInput = false;
        while (!validInput) {
          if (Serial.available() > 0) {
            String input = Serial.readStringUntil('\n');
            input.trim();
            String input_cpy = input;
            int spaceIndex1 = input.indexOf(' ');
            input = input.substring(spaceIndex1 + 1);
            int spaceIndex2 = input.indexOf(' ');
            if (spaceIndex1 != -1 && spaceIndex2 != -1) {
              String kp_str = input_cpy.substring(0, spaceIndex1);
              String kd_str = input.substring(0, spaceIndex2);
              String ki_str = input.substring(spaceIndex2 + 1);
              if (kp_str.length() > 0 && kd_str.length() > 0 && ki_str.length() > 0) {
                kpPid = kp_str.toFloat();
                kdPid = kd_str.toFloat();
                kiPid = ki_str.toFloat();
                Serial.printf("kpPid: %.3f, kdPid: %.3f,kiPid: %.3f\n", kpPid, kdPid, kiPid);
                validInput = true;
              } else {
                Serial.println("Invalid input. Please provide three float values separated by spaces 1.");
              }
            } else {
              Serial.println("Invalid input. Please provide three float values separated by spaces 2.");
            }
          }
        }
        break;
      }

      case 'r':  // resets torque to 0
      {
        ::t_in = 0;
        break;
      }

      case 'i':  
        {
          if (Serial.available() > 0) {
            String input = Serial.readStringUntil('\n');
            float iRef = input.toFloat();
            float max_GRF = 200;
            float max_Torque = iRef;

            CAN_receive();
            GRF_FSRs();
            if (GRF > 60){
              float max_GRF = 200;
              float max_Torque = 2.5;
              ::t_in = constrain(GRF/max_GRF*max_Torque, 0, max_Torque);
              Serial.print("Input Torque:"); Serial.println(t_in);
            }
            else{
              ::t_in = 0.6;
            }
          
            if(::v_out < -1){
              ::t_in = 0;
            }
          }
          break;
      }

      case 'e':  // extension control - using trapezoidal velocity setpoint
        {
          if (Serial.available() > 0) {
            String input = Serial.readStringUntil('\n');
            input.trim();
            float vRef = input.toFloat();
            float v_max = vRef;
            // constant velocity at the time being, but with gradual increase and gradual decrease
            t_in = 0;            
            float start_angle = -angleY;
            v_in = 0;
            float stop = 10; float margin = 15;
            float total_rotation = -angleY-stop;
            float rise_angle = total_rotation/3;
            
            while(!Serial.available()){
            float angle_rotated = start_angle + angleY;
      
            ::t_in= constrain(0.15*(v_in-v_out), -3, 3);

            if (angle_rotated < rise_angle) {
                // Acceleration phase
              ::v_in = -5 + ((v_max + 5) / rise_angle) * angle_rotated;
            } else if (angle_rotated < (2*rise_angle)) {
                // Constant velocity phase
                ::v_in = v_max;
            } else if (angle_rotated < 3*rise_angle) {
                // Deceleration phase
                float angle_since_decel_start = angle_rotated - 2*rise_angle;
                ::v_in = v_max * (1 - angle_since_decel_start / rise_angle);

            } else {
                // End of motion
                ::v_in = 0;
                ::t_in = 0;
                break;
            }

            if (-angleY < stop) {
              ::v_in = 0;
              ::t_in = 0;
              break;
            }
            do_each_loop(mode); 
            }
      }
      break;
      }

      case 'p':  // Unwrap to unwrap position mode. Position control. Also the walking state !!!
        {
          if (Serial.available() > 0) {
            String input = Serial.readStringUntil('\n');
            float pRef = input.toFloat();
            ::kp_in = 5;
            ::kd_in = 2;
            ::p_in = pRef;
            ::t_in = -0.3;  // reset the torque reference to 0
          }
          break;
      }

      case '7':  // Flexion damping to prevent buckling
      {
        while(!Serial.available()) {
          ::t_in = 0;  // reset the torque reference to 0
          float pRef = 0.7;
          if (-pRef + p_out > 0.05){
            ::kp_in = 5;
            ::kd_in = 3;
            ::p_in = pRef;
            ::t_in = -1.3;
            // Serial.println("Added Feedforward");
          }
          else{
            ::t_in = -0.3;
          }
          do_each_loop(mode); 
        }
        ::kd_in = 0; ::kp_in = 0; ::t_in = 0;
        break;
      }

      case 't':  // extension control - using trapezoidal velocity setpoint
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

              if (counter < 1000) {
                  // Acceleration phase
                ::v_in = -5 + ((v_max + 5) / 1000) * counter;
              } else{
                  // Constant velocity phase
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