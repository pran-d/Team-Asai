void do_each_loop(char fromWhere) {
  counter += 1;
  currentTime = millis();
  previousTime = currentTime;

  IMU_update();  
  CAN_receive();
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
  Serial.println(time);
  
  int gaitphase = data[3];
  int shankAngle = ((data[5] << 8 ) | data[4] );
  if(shankAngle>30000)
  {
    shankAngle -= 65535;
  }

  dataToSend.shankAngle= shankAngle/100.0;
  dataToSend.time= time;
  dataToSend.gaitphase= gaitphase;
  
  Serial.println(gaitphase);
  Serial.println(dataToSend.shankAngle);
  Serial.println();
}

void unpack_fsrVal(uint8_t* data)
{
  dataToSend.fsr1 = (data[1] << 8) | data[0];
  dataToSend.fsr2 = (data[3] << 8) | data[2];
  dataToSend.fsr3 = (data[5] << 8) | data[4];
  dataToSend.fsr4 = (data[7] << 8) | data[6];
  Serial.println(dataToSend.fsr1);
}


void Extension_Control(float vRef, float stop, float Kp, float Kd){
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
      // End of motion
      v_in = 0;
      t_in = 0;
    
  }

  if (angleX < stop) {
    v_in = 0;
    t_in = 0;
  }
  do_each_loop('Extension Control'); 
  
}

void Flexion_Damping(float max_flexion, float Kp, float Kd){
  
  //Need to add some sort of while loop here, earlier we were using while(serial.available)
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

void Position_Control(float pRef, float Kp, float Kd){
  kp_in = Kp;
  kd_in = Kd;
  p_in = pRef;
  t_in = 0;  // reset the torque reference to 0
  do_each_loop('Position Control');
}

void Reset_Torque(){
  t_in = 0;
  // Do we need a do_each_loop here??
}

void CalibrateDeadband(){
  kp_in = 0;
  kd_in = 5;
  v_in = -4;
  t_in = 0;
  deadband_length = 0.4 //NEEDS TO BE CALIBRATED

  deadband_torque_limit = 0.5; //when tout goes above this threshold we are no longer in deadband 

  while(tout<deadband_flexion_limit){
    do_each_loop('Deadband Flexion Limit')
  }
  deadband_extension = p_out
  deadband_flexion = p_out - deadband_length 


}