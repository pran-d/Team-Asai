void constrain_inputs(){
  ::t_in = constrain(::t_in, -6, 6);
}

void do_each_loop(char fromWhere) {
  ::counter += 1;
  ::currentTime = millis();
  ::previousTime = currentTime;

  IMU_update();  
  CAN_receive();
  constrain_inputs();
  
  if(::motor_active=='a')
    pack_cmd();

  // if (::t_out > I_MAX || ::v_out > V_MAX){
  //   ERROR_STATE = 1;
  //   ERROR_CODE = 1;
  //   currentMode = Passive;
  // }
  // else{
  //   ERROR_STATE = 0;
  // }

  if (counter == 10) {
    Serial.printf("\n %c, t_out: %0.4f, t_in: %0.4f, p_in: %.4f, p_out: %.4f, v_in: %.4f, v_out: %.4f, kp_in: %.4f, kd_in: %.4f \n", fromWhere, t_out, t_in, p_in, p_out, v_in, v_out, kp_in, kd_in);
    Serial.printf("thigh_angle: %.2f, shank_angle: %.2f, knee_angle: %.2f \n", sensors.thighAngle, sensors.shankAngle, sensors.kneeAngle);
    Serial.printf("GRF: %d,  %d,  %d, %d \n\n", sensors.fsr1, sensors.fsr2, sensors.fsr3, sensors.fsr4);
    counter = 0;
  }
  delay(10);
}

void reset_inputs(){
  ::kp_in = 0;
  ::kd_in = 0;
  ::v_in = 0;
  ::t_in = 0;
  do_each_loop('r');
}

void CAN_receive(){
  twai_message_t message_recv;
  while (twai_receive(&message_recv, 0) == ESP_OK) {  // what is this tick thing - SOME UNIT OF TIME IN RTOS
    lastMessageTime = millis();
    handle_rx_message(message_recv);
  }
  if(millis()-lastMessageTime > 200){
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
  
  driver_installed = true;
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
  delay(100);
}

// Exit motor start mode
void ExitMode(unsigned int motorID) {
  uint8_t data[8];
  data[0] = 0xFF;
  data[1] = 0xFF;
  data[2] = 0xFF;
  data[3] = 0xFF;
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
    // unpack_reply(data);
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

void unpack_reply(uint8_t *data) 
{
  int p_int = (data[1] << 8) | data[2];
  int v_int = (data[3] << 4) | (data[4] >> 4);
  int i_int = ((data[4] & 0xF) << 8) | (data[5]);
  int error_int = data[7];
  float p = uint_to_float(p_int, P_MIN, P_MAX, 16);
  float v = uint_to_float(v_int, V_MIN, V_MAX, 12);
  float i = uint_to_float(i_int, -I_MAX, I_MAX, 12);
  float error = uint_to_float(error_int, 0, 7, 8);
  ::p_out = p;
  ::v_out = v;
  ::t_out = i;
  delay(1);
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
}

void unpack_fsrVal(uint8_t* data)
{
  sensors.fsr1 = (data[1] << 8) | data[0];
  sensors.fsr2 = (data[3] << 8) | data[2];
  sensors.fsr3 = (data[5] << 8) | data[4];
  sensors.fsr4 = (data[7] << 8) | data[6];
  GRF_FSRs();
  // Serial.print(sensors.fsr1); Serial.print(", "); Serial.print(sensors.fsr2); Serial.print(", "); Serial.print(sensors.fsr3); Serial.print(", "); Serial.print(sensors.fsr4); Serial.println("");
}


//---------------------------------Controllers------------------------------
void Position_Control(float pRef, float Kp, float Kd, float feedforward){
  kp_in = Kp;
  kd_in = Kd;
  p_in = pRef;
  t_in = feedforward;  // reset the torque reference to 0
  do_each_loop('p');
}

void enter_deadband(){
  while (abs(::p_out - 3) > 0.1)
  {
    Position_Control(3, 20, 3, 0);
  }
  reset_inputs();
}

void Stair_Ascent_Loading_GUI(){
  CAN_receive();
  GRF_FSRs();
  int counter_loc = 0;
  
  while(sensors.thighAngle > 10 && abs(sensors.shankAngle) < 20 && sensors.kneeAngle > 10)
  {
    counter_loc +=1;  

    //Torque Build up Controller - Quadratic wrt time 
    ::t_in = -constrain(Stair_Ascent_a*counter_loc*counter_loc + Stair_Ascent_b*counter_loc + Stair_Ascent_c, -I_max_Ascent, I_max_Ascent);

    //Torque wind down Controller - Exponential Decay wrt Knee Angle
    if (sensors.kneeAngle > 25) { 
        ::t_in = -constrain(Stair_Ascent_A*(exp(Stair_Ascent_k*sensors.kneeAngle)-1), -I_max_Ascent, I_max_Ascent);
      }
    else {
      ::t_in = 0;
    } 

    do_each_loop('s');     
    
  }

  currentPhase = Standing;
  currentState = Walking;
  reset_inputs();
  do_each_loop('s');     

}

void Stair_Ascent_Loading()
{
  CAN_receive();
  GRF_FSRs();
  float iRef1 = -2.0;
  float iRef = -4.5;
  float iRef2 = iRef;
  int rampTime1 = 75;
  int rampTime2 = 175;
  iRef = constrain(iRef, -10, 10); 
  int counter_loc = 0;
  bool enter_highAngle = 0;
  int rampDownCounter = 0;
  long startTime = millis();
  bool high_vel = false;
  float t_damp = 0;
  
  while(sensors.thighAngle > 10 && abs(sensors.shankAngle) < 20 && sensors.kneeAngle > 10) //maximum thigh angle from which we allow lifting? 
  {
    counter_loc = counter_loc+1;
    if (counter_loc < rampTime1) 
    {
      ::t_in = (iRef1 / rampTime1) * counter_loc;
    } 
    else if (counter_loc < rampTime2) {
      ::t_in = iRef1 + (iRef - iRef1)/(rampTime2 - rampTime1) * (counter_loc-rampTime1);
    }
    else
    {
      ::t_in = iRef;
    }
    if (sensors.kneeAngle<50)
    {
      if (enter_highAngle == 0) {
          iRef2 = ::t_in;
          enter_highAngle = 1;
      }
      // ::t_in = iRef*(1 - rampDownCounter / rampTime);
      // rampDownCounter+=1;
      if (sensors.kneeAngle > 25) {
        ::t_in = iRef2 * sensors.kneeAngle/25;  
      }
      else {
        ::t_in = 0;
      }
    }

    // if(sensors.thighAngularVelocity>imuHighVelThresh && ::v_out<-7) // if extending at high velocity, slow it down
    // {
    //   t_damp = 0.1 * (-::v_out-7);
    //   ::t_in += t_damp;
    //   setVelocity(-7);
    //   do_each_loop('v');
    // }

    // if(sensors.thighAngle<imuStableAngle && sensors.thighAngularVelocity<imuFlexionThresh){
    //   t_damp = 0.01*abs(sensors.thighAngularVelocity);
    //   ::t_in -= t_damp;
    //   do_each_loop('w');
    // }

    // if(::v_out>7 && ::p_out>3) // if sufficiently flexed and motor still backdriving, give torque
    // {
    //   ::t_in = -2 - ::v_out*0.08;
    // }

    // if(millis()-startTime > 8000) // time limit to climb crossed
    // {
    //   ERROR_STATE = 1;
    //   ERROR_CODE = 3;
    //   Serial.println("Time crossed 6s");
    //   break;
    // }

    // if(currentMode == Passive)
    // {
    //   break;
    // }

    do_each_loop('s');  
  }

  currentPhase = Standing;
  currentState = Walking;
  reset_inputs();
  do_each_loop('s');     
}


void setVelocity(float vRef){
  ::kp_in = 0;
  ::kd_in = 5;
  ::v_in = vRef;  
}

void cable_taut(float vRef){
  bool taut = false; bool moving = false;
  int counter = 0; float filtered_t_out[3];
  long timer = millis();
  while(!taut && !Serial.available() && millis()-timer<8000)
  {
    if(abs(v_out)>3){
      moving = true;
    }
    ::kp_in = 0; ::kd_in = 5;
    ::v_in = vRef; 
    if(abs(t_out)<15) filtered_t_out[counter] = t_out;
    counter+=1;
    if(counter==3){
      counter=0;
    }
    float average_t_out = (filtered_t_out[0]+filtered_t_out[1]+filtered_t_out[2])/3;
    do_each_loop('f');
    if(abs(v_out) < ZERO_VELOCITY_BAND && abs(average_t_out)>0.35 && moving)
    {
      taut = true;
      break;
    }
  }
  ::v_in = 0; ::t_in = 0;
  ::kp_in = 0; ::kd_in = 0;  
  do_each_loop('f'); 
}

void descentController(){
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

  do_each_loop('d');
}
