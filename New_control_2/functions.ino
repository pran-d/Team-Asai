void constrain_inputs(){
  ::t_in = constrain(::t_in, -6, 6);
}

void do_each_loop(char fromWhere) {
  counter += 1;
  currentTime = millis();
  previousTime = currentTime;

  IMU_update();  
  CAN_receive();
  constrain_inputs();
  pack_cmd();

  if (counter == 10) {
    Serial.printf("\n %c, t_out: %0.4f, t_in: %0.4f, p_in: %.4f, p_out: %.4f, v_in: %.4f, v_out: %.4f, kp_in: %.4f, kd_in: %.4f \n", fromWhere, t_out, t_in, p_in, p_out, v_in, v_out, kp_in, kd_in);
    Serial.printf("thigh_angle: %.2f, shank_angle: %.2f, knee_angle: %.2f \n", sensors.thighAngle, sensors.shankAngle, sensors.kneeAngle);
    Serial.printf("GRF: %d,  %d,  %d \n\n", sensors.fsr1, sensors.fsr2, sensors.fsr3);
    counter = 0;
  }
  delay(10);
}

void dmpDataReady() {
  mpuInterrupt = true;
}

void IMU_initialize(){
  //initialize device
  Serial.println(F("Initializing I2C devices..."));
  // mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  //verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  if(devStatus==0){
    mpu.CalibrateAccel(15);
    mpu.CalibrateGyro(15);
    mpu.PrintActiveOffsets();
    mpu.setDMPEnabled(true);
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    int mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
  } else{
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(")");
  }
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
  // Serial.print(sensors.fsr1); Serial.print(", "); Serial.print(sensors.fsr2); Serial.print(", "); Serial.print(sensors.fsr3); Serial.print(", "); Serial.print(sensors.fsr4); Serial.println("");
}


unsigned int float_to_uint(float x, float x_min, float x_max, int bits) 
{
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

float uint_to_float(unsigned int x_int, float x_min, float x_max, int bits) 
{
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

void IMU_update()
{
  if (!dmpReady) {
    Serial.print("h");
    devStatus = mpu.dmpInitialize();
    if(devStatus==0)
      dmpReady=true;
  }
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet  
      // display Euler angles in degrees
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      mpu.dmpGetGyro(&av, fifoBuffer); 
  }
  angleX=ypr[0] * 180/M_PI;
  angleY=ypr[1] * 180/M_PI;
  angleZ=ypr[2] * 180/M_PI;
  sensors.thighAngle= -(angleZ-90);
  sensors.kneeAngle= sensors.thighAngle-sensors.shankAngle;
}

void GRF_FSRs()
{
  GRF=sensors.fsr1+sensors.fsr2+sensors.fsr3+sensors.fsr4;
}

void cable_taut(float vRef){
  bool taut = false; bool moving = false;
  int counter = 0; float filtered_t_out[3];
  while(!taut && !Serial.available())
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
    do_each_loop(::mode);
    if(abs(v_out) < ZERO_VELOCITY_BAND && abs(average_t_out)>0.5 && moving)
    {
      taut = true;
      ::v_in = 0; 
      ::kp_in = 0; ::kd_in = 0;  
      do_each_loop(::mode);
      break;
    }
  } 
}

void reset_inputs(){
  ::kp_in = 0;
  ::kd_in = 0;
  ::v_in = 0;
  ::t_in = 0;
  do_each_loop(::mode);
}

void setVelocity(float vRef){
  ::kp_in = 0;
  ::kd_in = 5;
  ::v_in = vRef;  
}

void setPosition(float pRef, float feedforwardTorque){
  ::kp_in = 4.3;
  ::kd_in = 3.5;
  ::p_in = pRef;
  ::t_in = feedforwardTorque;  // reset the torque reference to 0
}

bool isExtended(){
  float ext_angle = 10;
  if(::angleY - sensors.shankAngle < ext_angle)
    return true;
  else
    return false;
}
