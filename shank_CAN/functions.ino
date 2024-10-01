void IMU_initialize(){
  //initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  //verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  if(devStatus==0){
    // mpu.CalibrateAccel(15);
    // mpu.CalibrateGyro(15);
    mpu.setXAccelOffset(mpuOffsets[0]);
    mpu.setYAccelOffset(mpuOffsets[1]);
    mpu.setZAccelOffset(mpuOffsets[2]);
    mpu.setXGyroOffset(mpuOffsets[3]);
    mpu.setYGyroOffset(mpuOffsets[4]);
    mpu.setZGyroOffset(mpuOffsets[5]);
    
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

void dmpDataReady() {
  mpuInterrupt = true;
}

void IMU_update(){
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
  angleX=ypr[2] * 180/M_PI;
  angleY=ypr[1] * 180/M_PI;
  currentTime = millis();
}

bool checkBelowThreshold(float threshold, float min){
  bool allBelowThreshold = true;
  for(int i = 0; i < numFsrs; i++) {
    fsrValues[i] = readFSR(fsrPins[i]) - fsrOffsets[i]; // Subtract offset
    if (fsrValues[i] < min) { 
      fsrValues[i] = min; // Ensure values are not less than threshold
    }
    if (fsrValues[i] >= threshold) {
      allBelowThreshold = false;
    }
  }
  return allBelowThreshold;
}


int readFSR(int pin) {
  int value = analogRead(pin);
  return (value < 10) ? 10 : value; // Set values less than 10 to 10
}

void applyLowPassFilter(int *values, int *lowPassValues, float alpha) {
  for (int i = 0; i < numFsrs; i++) {
    lowPassValues[i] = round(alpha * (values[i]) + (1 - alpha) * lowPassValues[i]);
  }
}

void calculateCoP(int *values) {
  float sumF = 0; CoPx = 0; CoPy = 0;
  for (int i = 0; i < numFsrs; i++) {
    sumF += values[i];
    CoPx += values[i] * coords[i][0];
    CoPy += values[i] * coords[i][1];
  }
  if (sumF != 0) {
    CoPx /= sumF;
    CoPy /= sumF;
  } 
  else {
    CoPx = 0;
    CoPy = 0;
  }
  CoPx = round(CoPx * 10) / 10.0;
  CoPy = round(CoPy * 10) / 10.0;
}

void detectFSRPhase(int *values) {
  float totalForce = 0;
  float proportions[numFsrs];
  
  for (int i = 0; i < numFsrs; i++) {
    totalForce += values[i];
  }

  for (int i = 0; i < numFsrs; i++) {
    proportions[i] = values[i] / totalForce;
  }

  if (totalForce < 80 ) {
    gaitPhase = 4;  // Swing Phase
    CoPx=0;CoPy=0;
  } 
  else 
  {
    if(gaitPhase == 3 && proportions[3]+proportions[2]<0.55){
      gaitPhase=2;
    }
    else if(gaitPhase == 2 && proportions[0]>0.6){//0.85
      gaitPhase=1;
    }
    else {
      // Conditions for Heel Strike, Mid Stance, Toe Off
        if (proportions[0] > 0.6) {  // FSR2 (Heel)
          gaitPhase = 1;  // Heel Strike
        } 
        else if (proportions[3]+proportions[2]>0.6){//(proportions[3] > 0.3 && proportions[2]>0.3) {  // FSR4 (Toe)
          gaitPhase = 3;  // Toe Off
        } 
        else {
          gaitPhase = 2;  // Mid Stance / Flat Foot
        }
      }
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


void send_message(uint32_t identifier, uint8_t *data, uint8_t data_length) {
  twai_message_t message;
  message.identifier = identifier;
  message.data_length_code = data_length;

  for (int i = 0; i < data_length; i++) {
    message.data[i] = data[i];
  }

  if (twai_transmit(&message, pdMS_TO_TICKS(1000)) == ESP_OK) {
    //Serial.println("Message queued for transmission");
  }
  // else if (twai_transmit(&message, pdMS_TO_TICKS(1000)) == ESP_ERR_TIMEOUT) {
  //   CAN_initialize();
  // }
   else {
    Serial.println("Failed to queue message for transmission");
  }
}

void pack_cmd() {
  byte data[8];
  // Pack ints into the CAN buffer
  data[0] = send.time & 0xFF;
  data[1] = (send.time >> 8) & 0xFF;
  data[2] = (send.time >> 16) & 0xFF;
  data[3] = send.gaitphase;
  // data[0] = 0;
  // data[1] = 0;
  // data[2] = 0;
  int shankSend = round((send.shankAngle*100));
  if(shankSend<0)
  {
    shankSend=65535+shankSend;
  }
  data[4] = shankSend & 0xFF;
  data[5] = (shankSend >>8) & 0xFF;
  // data[4] = 0;
  // data[5] = 0;
  data[6]= 0;
  data[7]= 0;

  send_message(0x02, data, 8);

  byte data_fsr[8];
  data_fsr[0]= send.fsr1 & 0xFF;
  data_fsr[1]= (send.fsr1 >> 8) & 0xFF;
  
  data_fsr[2]= send.fsr2 & 0xFF;
  data_fsr[3]= (send.fsr2 >> 8) & 0xFF;
  
  data_fsr[4]= send.fsr3 & 0xFF;
  data_fsr[5]= (send.fsr3 >> 8) & 0xFF;

  data_fsr[6]= send.fsr3 & 0xFF;
  data_fsr[7]= (send.fsr3 >> 8) & 0xFF;

  send_message(0x03,data_fsr,8);
}
