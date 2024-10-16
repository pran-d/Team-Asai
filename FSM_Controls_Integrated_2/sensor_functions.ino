int checkMode(){
  int pressed = 0;
  if(digitalRead(2)==1) pressed = 1;
  if(digitalRead(12)==0) pressed = 2;
  if(digitalRead(4)==0) pressed = 3;
  if(digitalRead(13)==0) pressed = 4;
  return pressed;
}

void buttonSwitchState(int pressed){
  if(pressed==1)
  { 
    ::currentMode = Passive;
    ::motor_active = 'a';
    reset_inputs();
    EnterMode(0x01);
    Serial.println("Motor mode entered");
    delay(300);
  }
  else if(pressed==2)
  {
    ::currentMode = Stair;
    ::currentState = Walking;
    ::currentPhase = Standing;
    ::motor_active = 'a';
    delay(300);
  }
  else if(pressed==3)
  {
    ::currentMode = Stair;
    ::currentPhase = Standing;
    ::motor_active = 'p';
    delay(300);
  }
  else if(pressed==4)
  {
    ::motor_active = 'p';
    reset_inputs();
    ExitMode(0x01);
    Serial.println("Motor mode exited");
    delay(300);
  }
}

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
  GRF=GRF_Scale*(sensors.fsr1+sensors.fsr2+sensors.fsr3+sensors.fsr4);
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
    mpu.setXAccelOffset(mpuOffsets[0]);
    mpu.setYAccelOffset(mpuOffsets[1]);
    mpu.setZAccelOffset(mpuOffsets[2]);
    mpu.setXGyroOffset(mpuOffsets[3]);
    mpu.setYGyroOffset(mpuOffsets[4]);
    mpu.setZGyroOffset(mpuOffsets[5]);
    
    mpu.setDMPEnabled(true);
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    int mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
  } else{
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println();
  }
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
      mpu.dmpGetGyro(av, fifoBuffer); 
  }
  angleX= ypr[2] * 180/M_PI - 96;
  angleXdot = av[2];
  sensors.thighAngle= angleX;
  sensors.kneeAngle= angleX-sensors.shankAngle;
  sensors.thighAngularVelocity = angleXdot;
}