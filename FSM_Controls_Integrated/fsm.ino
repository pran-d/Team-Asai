int readMode(){
  int pressed = 0;
  if(digitalRead(2)==1) pressed = 1;
  if(digitalRead(12)==0) pressed = 2;
  if(digitalRead(4)==0) pressed = 3;
  if(digitalRead(13)==0) pressed = 4;
  return pressed;
}

// void updateSensors(){ 
//   GRF_FSRs();
//   updateThighAngle();
//   //FSR phase and shank angle is automatically updated from the espnow protocol
// }

void WalkingStateMachine() {
  switch (currentPhase) {
    case HS:
      if (FSRs == MS && theta_t <= 5 && theta_k <= 5 && w1 < GRF && GRF < w2) 
      currentPhase = Standing;  //T20      
      else if (FSRs == Sw) 
      currentPhase = Sw; //T21
      break;

    case Standing:
      if (FSRs == Sw) currentPhase = Sw; //T01
      else if (GRF > bw && theta_k <=5 && FSRs == MS) currentPhase = TO; // T03
      break;
    
    case MS:
      if (w1 < GRF && GRF < w2 && theta_k <= 5 && theta_t <= 5 && FSRs== MS) currentPhase = Standing; // T30
      else if (FSRs == TO && theta_t < 0) currentPhase = TO; // T34
      break;

    case TO:
      if (FSRs == Sw) currentPhase = Sw; // T41
      break;

    case Sw:
      if (FSRs == HS && theta_k < 5 && theta_t < theta2 && theta1 < theta_t ) currentPhase = HS; // T12
      else if (theta_k <= 5 && FSRs == HS && w1 < GRF && GRF < w2)
      {
        currentPhase = HS; currentState = Descent; // Walking to Descent
      }
      else if (FSRs == HS && theta_k >theta3)
      {
        currentPhase = HS; currentState = Ascent; // Walking to Ascent
      }
      break;
  }
}

void AscentStateMachine() {
  switch (currentPhase) {
    case Sw:
      if (FSRs == MS && GRF < w2 && GRF > w1 && theta_k <= 5) currentPhase = Standing; // T10
      else if (FSRs == HS && theta_k >theta3) currentPhase = HS; // T12
      else if (FSRs == HS && theta_k < 5 && theta_t < theta2 && theta1 < theta_t )
      {
        currentPhase = HS; currentState = Walking; // Ascent to Walking
      }
      break;

    case HS:
      if (FSRs == MS && GRF > w3 && theta_k > theta3){
        Serial.println("Extend knee");
        currentPhase = MS;
      }  // T23
      else if (FSRs == Sw) 
      {
        currentPhase = Sw;
      }
      break;

    case MS:
    if(thighAngleDecrease()==true||shankAngleDecrease()==true||theta_k <10)
    {
      Serial.println("Flexion Damping");
    } 
    //if(FSRs == MS && theta_k <= 5) currentPhase = TS; // T34
    if (FSRs == TO && theta_t < 10) 
    {
      Serial.println("Flexion Damping");
      currentPhase = TO;
    }

    else if (w1 < GRF && GRF < w2 && theta_k <= 5) currentPhase = Standing; // T30

    else if (FSRs == Sw) currentPhase = Sw; // T31

    // else if (FSRs ==MS &&  theta3 < theta_k && theta_k <theta4 ) 
    // {
    //   currentState = Descent; // Ascent to Descent
    //   currentPhase = MS;
    // }
    break;

    // case TS:
    //   if (FSRs == TO && theta_t < 0) currentPhase = TO; // T45
    //   break;

    case TO:
      if (FSRs == Sw) currentPhase = Sw;
      break;

  }
}

void DescentStateMachine() {
  switch (currentPhase) {
    case Sw:
      if (theta_k <= 5 && FSRs == HS) currentPhase = HS; // T12
      else if (FSRs == HS && theta_k < 5 && theta_t < theta2 && theta1 < theta_t )
      {
        currentPhase = HS; currentState = Walking; // Descent to Walking
      }
      break;

    case HS:
      if (theta_k <= 5 && FSRs == HS && w1 < GRF && GRF < w2) 
      currentPhase = Standing; // T20
      break;

    case Standing:
       if (FSRs == Sw) currentPhase = Sw; //T01
       else if (FSRs == MS && GRF > bw && theta_k < theta4 && theta_k > theta3) currentPhase = MS; //T03
       break;

    case MS:
      if (theta_k > theta5 && FSRs == TO) currentPhase = TO; // T34
      else if (FSRs == Sw) currentPhase = Sw; //T31
      break;

    case TO:
      if (FSRs == Sw) 
      currentPhase = Sw; // T41
      break;

  }
}

void FlexionStateMachine(){
  switch(FSRs)
  {
    case Sw:
    int flexStat;
    flexStat=0;
    //updateSensors();
    IMU_update();
    while(FSRs==Sw)
    {
      if(omega_t>0)
      {
        if(theta_t>theta6)// && flexStat==0)
        {
          flexStat=1;
          Serial.println("Knee angle mapped with thigh angle");
        }
        }
// check accuracy of the omega readings from mpu
      else{//omega_t <= 0
        if(theta_t<theta7 && flexStat==1){
          Serial.println("Knee left passive");//
          }
      }
      //updateSensors();
      IMU_update();
      }
    break;

    // default: IMU_update();//updateSensors();
    // break;

    }
  }

