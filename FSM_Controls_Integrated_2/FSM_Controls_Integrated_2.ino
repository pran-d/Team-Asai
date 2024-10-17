#include "var.h"

void setup() {
  Serial.begin(115200);
  Wire.begin();

  for(int i=0;i<4;i++)
  {
    pinMode(keyPins[i], INPUT_PULLUP);
    pinMode(keyPinsLED[i], OUTPUT);
  }

  startTime = millis();
  Wire.setClock(200000); // 400kHz I2C clock. Comment this line if having compilation difficulties

  IMU_initialize();
  CAN_initialize();
  driver_installed = true;

  // initESPNow();
  pinMode(23, OUTPUT);
  digitalWrite(23, HIGH);

  delay(5000);
  EnterMode(0x01);
  delay(1000);

  int wifi_mode=1;

  if (wifi_mode==1){

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
      delay(1000);
      Serial.println("Connecting to WiFi...");
    }
    Serial.println("Connected to WiFi");

    // Set up the web server routes
    server.on("/", handleRoot);  // Home page route
    server.on("/modify",HTTP_POST, handleModify);  // Handle form submission
    server.on("/Ascent", HTTP_POST, handleAscent);
    server.on("/Descent", HTTP_POST, handleDescent);
    server.on("/Passive", HTTP_POST, handlePassive);

    // Start the web server
    server.begin();
    Serial.println("Web server started");

    Serial.print("ESP32 IP Address: ");
    Serial.println(WiFi.localIP());

  }
  
  // initESPNow();

  int pressed = 0;
  while(pressed==0){
    pressed = checkMode();
    if(wifi_mode==1)
      server.handleClient();
  }

  Serial.println("Calibrating extended position...");
  cable_taut(-7);
  delay(500);
  ZeroMode(0x01); 
  delay(500);
  enter_deadband();
  Serial.println(".......SETUP COMPLETE.......");
}


void loop(){
  // allowing to test with motor not rotating
  if (Serial.available() > 0){
    reset_inputs();
    ::motor_active = Serial.read();
  }
  
  /* 
  switching mode based on buttons pressed:
  1 - Enter Passive FSM mode, enter motor mode with motor active
  2 - Enter Stair FSM mode with motor active
  3 - Enter Stair FSM mode with motor inactive
  4 - Exit motor mode with motor inactive
  */
  
  int pressed = checkMode();
  buttonSwitchState(pressed);

  if (::currentMode==Passive)
  {
    pressed = checkMode();
    buttonSwitchState(pressed);
  }

  if (::currentMode==Stair)
  {
    pressed = checkMode();
    buttonSwitchState(pressed);
    IMU_update();
    CAN_receive();
    
    // esp_err_t result = esp_now_send(peerAddress, (uint8_t *)&sensors, sizeof(sensors));
    switch (::currentState) {

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
  
}