void WalkingStateMachine() 
{
  server.handleClient();
  Serial.println("Entered Walking State");
  FSRs=sensors.gaitphase;

  switch (::currentPhase) {
  
    case Standing: //take fsr readings for standing and midstance, compare sum of fsr values
    { 
      Serial.println("In Standing");

      // standing to swing transition
      if (GRF < StanceThresh) 
      {
        currentPhase = Sw; 
        Serial.println("Standing -> SW");
      }

      // Walking to Ascent
      if (GRF > FSRascentThresh1 && sensors.thighAngle > theta_t_ascent)
      {
        Serial.println("SW -> MS and Walking -> Stair Ascent");        
        currentPhase = MS; 
        currentState = Ascent;
      }
      break;
    }

    case Sw:
    {
      Serial.println("In Swing");

      // Walking to Ascent
      if (GRF > FSRascentThresh2 && sensors.thighAngle > theta_t_ascent)
      {
        Serial.println("SW -> MS and Walking -> Stair Ascent");        
        currentPhase = MS; 
        currentState = Ascent; 
      }
      break;
    }
  }
}

void AscentStateMachine() {
  server.handleClient();
  switch (currentPhase) {
    case Sw:
    {
      if (GRF > FSRascentThresh1 && sensors.thighAngle > theta_t_ascent)
      {        
        currentPhase = MS; currentState = Ascent; // Walking to Ascent
      }
      if (GRF > FSRascentThresh1 && sensors.thighAngle < theta_t_walking ) 
      {
        Serial.println("Ascent to Walking");        
        currentPhase = Standing; 
        currentState = Walking; // Ascent to Walking 
      }
      break;
    }

    case MS:
    {
      if(sensors.thighAngle > theta_t_ascent && GRF > FSRascentThresh2){
        Serial.println("Start Climbing");
        // Stair_Ascent_Loading();
        Stair_Ascent_Loading_GUI();
        Serial.println("Stair Ascent Finished");
        enter_deadband();
      }
      break;
    }
  }
}

void DescentStateMachine() {
  server.handleClient();
  // Serial.printf("%f, %f, %f, %f, %f \n", region1, region2, region3, k1, k2, k3);
  switch (::currentPhase) {
    case Sw:
    {
      if (GRF > FSRdescentThreshUpper)
      {        
        currentPhase = MS; 
        Serial.println("********* Descent Sw -> MS *********");
      }
      while(::p_out < descentDeadband - 0.2)
      {
        Position_Control(descentDeadband, descentKp, 2.2, 0);
      }
      reset_inputs();
      break;
    }

    case MS:
    {
      if(GRF < FSRdescentThreshLower){
        currentPhase = Sw;
        Serial.println("********* Descent MS -> Sw *********");
      }
      descentController();
      break;
    }
  }
}
