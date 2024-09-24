void initESPNow() {
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_send_cb(onDataSent);

  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, peerAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
}

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Serial.print("\r\nLast Packet Send Status: ");
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void GRF_FSRs(){
  GRF=0;//data.fsr1+data.fsr2+data.fsr3+data.fsr4;
}


void dmpDataReady() {
  mpuInterrupt = true;
}

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
    // mpu.PrintActiveOffsets();
    mpu.setXAccelOffset(youroffset[0]);
    mpu.setYAccelOffset(youroffset[1]);
    mpu.setZAccelOffset(youroffset[2]);
    mpu.setXGyroOffset(youroffset[3]);
    mpu.setYGyroOffset(youroffset[4]);
    mpu.setZGyroOffset(youroffset[5]);
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
  dataToSend.thighAngle= angleX;
  dataToSend.kneeAngle= angleX-dataToSend.shankAngle;
}


bool thighAngleDecrease()
{

}

bool shankAngleDecrease()
{

}