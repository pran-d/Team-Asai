void do_each_loop(char fromWhere) {
  counter += 1;
  currentTime = millis();
  previousTime = currentTime;

  IMU_update();  
  CAN_receive();

  if (t_out > I_MAX && v_out > V_MAX){
    ERROR_STATE = 1;
    currentMode = Passive;
  }
  else{
    ERROR_STATE = 0;
  }

  pack_cmd();

  if (counter == 10) {
    Serial.printf("%c, t_out: %0.4f, t_in: %0.4f, p_in: %.4f, p_out: %.4f, v_in: %.4f, v_out: %.4f, kp_in: %.4f, kd_in: %.4f \n kpPid: %.4f,  kdPid: %.4f, kiPid: %.4f\n\n", fromWhere, t_out, t_in, p_in, p_out, v_in, v_out, kp_in, kd_in, kpPid, kdPid, kiPid);
    Serial.printf("angleX: %.2f, angleY: %.2f\n\n", angleX, angleY);
    counter = 0;
  }

  delay(10);
}
void CAN_receive(){
  twai_message_t message_recv;
  while (twai_receive(&message_recv, 0) == ESP_OK) {  // what is this tick thing - SOME UNIT OF TIME IN RTOS
    lastMessageTime = millis();
    
    handle_rx_message(message_recv);
  }
  if(millis()-lastMessageTime > 500){
    Serial.println("Failed to receive data");
  }

  
}

void CAN_initialize(){
  //CAN setup
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)TX_PIN, (gpio_num_t)RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    Serial.println("Driver installed");
  } else {
    Serial.println("Failed to install driver");
    return;
  }

  if (twai_start() == ESP_OK) {
    Serial.println("Driver started");
  } else {
    Serial.println("Failed to start driver");
    return;
  }

  uint32_t alerts_to_enable = TWAI_ALERT_TX_IDLE | TWAI_ALERT_TX_SUCCESS | TWAI_ALERT_TX_FAILED | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_ERROR;
  if (twai_reconfigure_alerts(alerts_to_enable, NULL) == ESP_OK) {
    Serial.println("CAN Alerts reconfigured");
  } else {
    Serial.println("Failed to reconfigure alerts");
    return;
  }
}

void EnterMode(unsigned int motorID) {
  uint8_t data[8];
  data[0] = 0xFF;
  data[1] = 0xFF;
  data[2] = 0xFF;
  data[3] = 0xFF;
  data[4] = 0xFF;
  data[5] = 0xFF;
  data[6] = 0xFF;
  data[7] = 0xFC;
  send_message(motorID, data, 8);
}

// Exit motor start mode
void ExitMode(unsigned int motorID) {
  uint8_t data[8];
  data[0] = 0xFF;
  data[1] = 0xFF;
  data[2] = 0xFF;
  data[4] = 0xFF;
  data[5] = 0xFF;
  data[6] = 0xFF;
  data[7] = 0xFD;
  send_message(motorID, data, 8);
  delay(100);
}

// Enter motor zero position
void ZeroMode(unsigned int motorID) {
  uint8_t data[8];
  data[0] = 0xFF;
  data[1] = 0xFF;
  data[2] = 0xFF;
  data[3] = 0xFF;
  data[4] = 0xFF;
  data[5] = 0xFF;
  data[6] = 0xFF;
  data[7] = 0xFE;
  send_message(motorID, data, 8);
  delay(100);
}

void send_message(uint32_t identifier, uint8_t *data, uint8_t data_length) {
  twai_message_t message;
  message.identifier = identifier;
  message.data_length_code = data_length;

  for (int i = 0; i < data_length; i++) {
    message.data[i] = data[i];
  }

  if (twai_transmit(&message, pdMS_TO_TICKS(1000)) == ESP_OK) {
    //Serial.println("Message queued for transmission");
  } else {
    Serial.println("Failed to queue message for transmission");
  }
}

void pack_cmd() {
  byte data[8];

  // Limit data to be within bounds
  float p_des = constrain(p_in, P_MIN, P_MAX);
  float v_des = constrain(v_in, V_MIN, V_MAX);
  float kp = constrain(kp_in, KP_MIN, KP_MAX);
  float kd = constrain(kd_in, KD_MIN, KD_MAX);
  float t_ff = constrain(t_in, I_MIN, I_MAX);

  // Convert floats to unsigned ints
  unsigned int p_int = float_to_uint(p_des, P_MIN, P_MAX, 16);
  unsigned int v_int = float_to_uint(v_des, V_MIN, V_MAX, 12);
  unsigned int kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
  unsigned int kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12);
  unsigned int t_int = float_to_uint(t_ff, I_MIN, I_MAX, 12);

  // Pack ints into the CAN buffer
  data[0] = p_int >> 8;
  data[1] = p_int & 0xFF;
  data[2] = v_int >> 4;
  data[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
  data[4] = kp_int & 0xFF;
  data[5] = kd_int >> 4;
  data[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);
  data[7] = t_int & 0xFF;

  send_message(0x01, data, 8);
}


static void handle_rx_message(twai_message_t &message) {
  uint8_t data[8];
  // Serial.printf("ID: %x\nByte:", message.identifier);
  if (!(message.rtr)) {
    for (int i = 0; i < message.data_length_code; i++) {
      // Serial.printf(" %d = %02x,", i, message.data[i]);
      data[i] = message.data[i];
    }
    // Serial.println("");
    if(message.identifier==0x01)
    {
      unpack_reply(data);
    }
    else if(message.identifier==0x02)
    unpack_espCan(data);
    else
    unpack_fsrVal(data);


  }
}

unsigned int float_to_uint(float x, float x_min, float x_max, int bits) {
  float span = x_max - x_min;
  float offset = x_min;
  unsigned int result = 0;
  if (bits == 12) {
    result = (unsigned int)((x - offset) * 4095.0 / span);
  } else if (bits == 16) {
    result = (unsigned int)((x - offset) * 65535.0 / span);
  }
  return result;
}

float uint_to_float(unsigned int x_int, float x_min, float x_max, int bits) {
  float span = x_max - x_min;
  float offset = x_min;
  float pgg = 0;
  if (bits == 12) {
    pgg = ((float)x_int) * span / 4095 + offset;
  }
  if (bits == 16) {
    pgg = ((float)x_int) * span / 65535.0 + offset;
  }
  return pgg;
}

void unpack_reply(uint8_t *data) {

  int p_int = (data[1] << 8) | data[2];
  int v_int = (data[3] << 4) | (data[4] >> 4);
  int i_int = ((data[4] & 0xF) << 8) | (data[5]);
  float p = uint_to_float(p_int, P_MIN, P_MAX, 16);
  float v = uint_to_float(v_int, V_MIN, V_MAX, 12);
  float i = uint_to_float(i_int, -I_MAX, I_MAX, 12);
  ::p_out = p;
  ::v_out = v;
  ::t_out = i;
}

void unpack_espCan(uint8_t* data){
  int time = (data[2] << 16) | (data[1] << 8) | data[0];
  // Serial.println(time);
  
  int gaitphase = data[3];
  int shankAngle = ((data[5] << 8 ) | data[4] );
  if(shankAngle>30000)
  {
    shankAngle -= 65535;
  }

  sensors.shankAngle= shankAngle/100.0;
  sensors.time= time;
  sensors.gaitphase= gaitphase;
  
  // Serial.println(gaitphase);
  // Serial.println(data.shankAngle);
  // Serial.println();
}

void unpack_fsrVal(uint8_t* data)
{
  sensors.fsr1 = (data[1] << 8) | data[0];
  sensors.fsr2 = (data[3] << 8) | data[2];
  sensors.fsr3 = (data[5] << 8) | data[4];
  sensors.fsr4 = (data[7] << 8) | data[6];
  // Serial.println(sensors.fsr1);
}


//---------------------------------Controllers-------------------------------
void Extension_Control(float vRef, float stop, float Kp, float Kd){
  if(!ERROR_STATE){
    float v_max = vRef;
    // constant velocity at the time being, but with gradual increase and gradual decrease
    t_in = 0;            
    float start_angle = angleX;
    v_in = 0;
    kp_in = Kp;
    kd_in = Kd;
    float margin = 15;
    float total_rotation = angleX-stop;
    float rise_angle = total_rotation/3;

    float angle_rotated = start_angle - angleX;
        
    t_in= constrain(0.15*(v_in-v_out), -3, 3);

    if (angle_rotated < rise_angle) {
        // Acceleration phase
      v_in = -5 + ((v_max + 5) / rise_angle) * angle_rotated;
    } else if (angle_rotated < (2*rise_angle)) {
        // Constant velocity phase
        v_in = v_max;
    } else if (angle_rotated < 3*rise_angle) {
        // Deceleration phase
        float angle_since_decel_start = angle_rotated - 2*rise_angle;
        v_in = v_max * (1 - angle_since_decel_start / rise_angle);

    } else {
      Serial.println("Reached End of Motion");
        // End of motion
        v_in = 0;
        t_in = 0;
      
    }

    if (sensors.thighAngle < stop) {
      v_in = 0;
      t_in = 0;
      
    }
    do_each_loop('Extension Control'); 

  }
  
  
}

void Flexion_Damping(float max_flexion, float Kp, float Kd){

  if(!ERROR_STATE){
    if(-angleX < 20){
        t_in = 0;
        kp_in = 0;
        kd_in = 0;
      }

    if(-angleX>max_flexion){
        kp_in = Kp; kd_in = Kd; 
        t_in = 
        p_in = p_out-0.25;
      }
    do_each_loop('Flexion Damping');
  }
  
   
}

void Position_Control(float pRef, float Kp, float Kd, float feedforward){
  kp_in = Kp;
  kd_in = Kd;
  p_in = pRef;
  t_in = feedforward;  // reset the torque reference to 0
  do_each_loop('Position Control');
}

void Reset_Torque(){
  t_in = 0;
  // Do we need a do_each_loop here??
}

void Stair_Ascent_Loading(){
  
  CAN_receive();
  GRF_FSRs();
  if (GRF > 60){
    float max_GRF = 200;
    float max_Torque = 2.5;
    float feedforwardTorque = constrain(GRF/max_GRF*max_Torque, 0, max_Torque);
    Serial.print("FeedforwardTorq: ");
    Serial.println(feedforwardTorque);
    Feedforward_torque(feedforwardTorque);
  }
  
}

void Feedforward_torque(float torque){
  t_in = torque;
  do_each_loop('Feedforward Torque'); 
}






//------------- Calibratables-----------

//Inputs is GRF
float FSR_Torque_Lookup_Input[62]{
  1.58,1.9,2.39,2.86,3.1,4.09,4.75,7.36,7.73,8.01,9.32,9.27,9.61,9.88,9.94,9.96,9.87,9.78,9.63,9.43,9.24,9.06,8.9
  ,8.51,8.31,8.06,7.83,7.62,7.5,7.36,7.28,7.23,7.25,7.32,7.46,7.54,7.73,7.94,8.23,8.55,8.85,9.17,9.49,9.66,9.94,10.14,
  10.31,10.45,10.49,10.31,10.14,9.86,9.37,9.45,8.56,7.1,4.63,4.57,3.52,4.01,2.38,1.36
};

//Outputs is Motor Torque
float FSR_Torque_Lookup_Output[62]{
  0.43,0.42,0.42,0.54,0.56,0.6,0.63,0.65,0.68,0.69,0.7,0.71,0.7,0.69,0.68,0.67,0.65,0.63,0.61,0.59,0.57,0.56,0.53,0.5,
  0.48,0.47,0.45,0.43,0.42,0.41,0.4,0.39,0.39,0.38,0.37,0.36,0.36,0.35,0.34,0.33,0.32,0.3,0.29,0.27,0.26,0.24,0.22,0.21
  ,0.19,0.18,0.18,0.18,0.18,0.2,0.22,0.24,0.26,0.28,0.31,0.35,0.38,0.4
};


// Inputs is Thigh Angle, assuming Link AB is grounded
float ThighAngle_MotorPosition_Lookup_Input[299]{
  104.89, 103.56, 102.17, 100.7, 99.13, 97.44, 95.58, 93.49, 91.01, 87.69, 87.02, 86.34, 85.66, 84.96, 84.24, 83.52, 
  82.78, 82.03, 81.26, 80.47, 79.66, 78.83, 77.97, 77.08, 76.16, 75.19, 74.18, 73.1, 71.95, 70.7, 69.31, 67.7, 65.68, 
  62.0, 61.25, 60.54, 59.86, 59.21, 58.58, 57.99, 57.41, 56.85, 56.32, 55.8, 55.29, 54.8, 54.33, 53.87, 53.42, 52.98, 
  52.56, 52.14, 51.73, 51.34, 50.95, 50.57, 50.2, 49.84, 49.48, 49.13, 48.79, 48.45, 48.12, 47.8, 47.48, 47.17, 46.86, 
  46.56, 46.26, 45.97, 45.68, 45.4, 45.12, 44.85, 44.57, 44.31, 44.04, 43.78, 43.53, 43.28, 43.03, 42.78, 42.54, 42.3, 
  42.06, 41.83, 41.6, 41.37, 41.15, 40.92, 40.7, 40.49, 40.27, 40.06, 39.85, 39.64, 39.44, 39.23, 39.03, 38.83, 38.63, 
  38.44, 38.24, 38.05, 37.86, 37.68, 37.49, 37.31, 37.12, 36.94, 36.76, 36.59, 36.41, 36.24, 36.06, 35.89, 35.72, 35.55,
  35.39, 35.22, 35.06, 34.89, 34.73, 34.57, 34.41, 34.25, 34.1, 33.94, 33.79, 33.63, 33.48, 33.33, 33.18, 33.03, 32.89, 
  32.74, 32.59, 32.45, 32.31, 32.16, 32.02, 31.88, 31.74, 31.6, 31.47, 31.33, 31.19, 31.06, 30.92, 30.79, 30.66, 30.52, 
  30.39, 30.26, 30.13, 30.01, 29.88, 29.75, 29.62, 29.5, 29.37, 29.25, 29.13, 29.0, 28.88, 28.76, 28.64, 28.52, 28.4, 
  28.28, 28.16, 28.04, 27.92, 27.81, 27.69, 27.58, 27.46, 27.35, 27.23, 27.12, 27.01, 26.9, 26.78, 26.67, 26.56, 26.45, 
  26.34, 26.23, 26.12, 26.02, 25.91, 25.8, 25.69, 25.59, 25.48, 25.38, 25.27, 25.17, 25.06, 24.96, 24.86, 24.76, 24.65, 
  24.55, 24.45, 24.34, 24.24, 24.14, 24.04, 23.94, 23.84, 23.74, 23.65, 23.55, 23.45, 23.35, 23.25, 23.16, 23.06, 22.96, 
  22.87, 22.77, 22.67, 22.58, 22.48, 22.39, 22.3, 22.2, 22.11, 22.01, 21.92, 21.83, 21.73, 21.64, 21.55, 21.46, 21.36, 
  21.27, 21.18, 21.09, 21.0, 20.91, 20.82, 20.73, 20.64, 20.55, 20.46, 20.37, 20.28, 20.19, 20.1, 20.01, 19.93, 19.84, 
  19.75, 19.67, 19.58, 19.49, 19.4, 19.31, 19.22, 19.14, 19.05, 18.96, 18.88, 18.79, 18.71, 18.62, 18.53, 18.45, 18.36, 
  18.28, 18.19, 18.11, 18.02, 17.94, 17.85, 17.77, 17.68, 17.6, 17.51, 17.43, 17.34, 17.26, 17.18, 17.09, 17.01, 16.92, 
  16.84, 16.76, 16.67, 16.59, 16.51, 16.42, 16.34, 16.26, 16.17, 16.09, 16.01
};

// Output is the change in ext cable length, given that it was initially taut
float ThighAngle_MotorPosition_Lookup_Output[299]{
  77.53, 77.74, 77.92, 78.06, 78.15, 78.18, 78.13, 77.97, 77.64, 76.97, 76.80, 76.62, 76.42, 76.21, 75.98, 75.74, 75.48, 
  75.20, 74.90, 74.57, 74.23, 73.85, 73.45, 73.01, 72.54, 72.02, 71.45, 70.82, 70.12, 69.32, 68.39, 67.25, 65.73, 62.72, 
  62.07, 61.44, 60.83, 60.23, 59.65, 59.08, 58.53, 57.99, 57.46, 56.94, 56.43, 55.93, 55.44, 54.96, 54.49, 54.02, 53.57, 
  53.12, 52.67, 52.24, 51.81, 51.38, 50.97, 50.55, 50.15, 49.75, 49.35, 48.96, 48.57, 48.19, 47.81, 47.44, 47.07, 46.70, 
  46.34, 45.99, 45.63, 45.28, 44.94, 44.60, 44.26, 43.92, 43.59, 43.26, 42.93, 42.61, 42.29, 41.97, 41.66, 41.35, 41.04, 
  40.73, 40.43, 40.13, 39.83, 39.53, 39.24, 38.95, 38.66, 38.37, 38.09, 37.81, 37.53, 37.25, 36.97, 36.70, 36.42, 36.15, 
  35.89, 35.62, 35.35, 35.09, 34.83, 34.57, 34.31, 34.06, 33.80, 33.55, 33.30, 33.05, 32.80, 32.56, 32.31, 32.07, 31.83, 
  31.59, 31.35, 31.11, 30.87, 30.64, 30.40, 30.17, 29.94, 29.71, 29.48, 29.26, 29.03, 28.80, 28.58, 28.36, 28.14, 27.92, 
  27.70, 27.48, 27.26, 27.05, 26.83, 26.62, 26.41, 26.19, 25.98, 25.77, 25.57, 25.36, 25.15, 24.94, 24.74, 24.54, 24.33, 
  24.13, 23.93, 23.73, 23.53, 23.33, 23.13, 22.93, 22.74, 22.54, 22.35, 22.15, 21.96, 21.77, 21.58, 21.39, 21.19, 21.01, 
  20.82, 20.63, 20.44, 20.25, 20.07, 19.88, 19.70, 19.51, 19.33, 19.15, 18.96, 18.78, 18.60, 18.42, 18.24, 18.06, 17.88, 
  17.70, 17.53, 17.35, 17.17, 17.00, 16.82, 16.65, 16.47, 16.30, 16.12, 15.95, 15.78, 15.61, 15.43, 15.26, 15.09, 14.92, 
  14.75, 14.58, 14.41, 14.25, 14.08, 13.91, 13.74, 13.58, 13.41, 13.24, 13.08, 12.91, 12.75, 12.58, 12.42, 12.26, 12.09, 
  11.93, 11.77, 11.60, 11.44, 11.28, 11.12, 10.96, 10.80, 10.64, 10.48, 10.32, 10.16, 10.00, 9.84, 9.68, 9.52, 9.36, 9.20, 
  9.05, 8.89, 8.73, 8.57, 8.42, 8.26, 8.10, 7.95, 7.79, 7.64, 7.48, 7.33, 7.17, 7.01, 6.86, 6.71, 6.55, 6.40, 6.24, 6.09, 
  5.93, 5.78, 5.63, 5.47, 5.32, 5.17, 5.01, 4.86, 4.71, 4.56, 4.40, 4.25, 4.10, 3.95, 3.79, 3.64, 3.49, 3.34, 3.19, 3.03, 
  2.88, 2.73, 2.58, 2.43, 2.27, 2.12, 1.97, 1.82, 1.67, 1.52, 1.36, 1.21, 1.06, 0.91, 0.76, 0.61, 0.46, 0.30, 0.15, 0
};



float get_torque_from_fsr(float input){
  for (int i = 0; i < 62; i++)
  {
    if( input < FSR_Torque_Lookup_Input[i])
    {  
      return FSR_Torque_Lookup_Output[i];      
    }
  }
  return FSR_Torque_Lookup_Output[62];
}

float get_motor_position_from_leg_position(float input){
  for (int i = 0; i < 299 ; i++)
  {
    if (input > ThighAngle_MotorPosition_Lookup_Input[i])
    {  
      return ThighAngle_MotorPosition_Lookup_Output[i];      
    }
  }
  return ThighAngle_MotorPosition_Lookup_Output[299];
}


