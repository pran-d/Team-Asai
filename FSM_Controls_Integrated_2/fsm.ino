void WalkingStateMachine() 
{
  enter_deadband();
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
      if (GRF > FSRascentThresh && sensors.thighAngle > theta_t_ascent)
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
      if (GRF > FSRascentThresh && sensors.thighAngle > theta_t_ascent)
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
  switch (currentPhase) {
    case Sw:
    {
      if (GRF > FSRascentThresh && sensors.thighAngle > theta_t_ascent)
      {        
        currentPhase = MS; currentState = Ascent; // Walking to Ascent
      }
      if (GRF > FSRascentThresh && sensors.thighAngle < theta_t_walking ) 
      {
        Serial.println("Ascent to Walking");        
        currentPhase = Standing; 
        currentState = Walking; // Ascent to Walking 
      }
      break;
    }

    case MS:
    {
      if(sensors.thighAngle > theta_t_ascent && GRF > 350){
        Serial.println("Start Climbing");
        Stair_Ascent_Loading();
        Serial.println("Stair Ascent Finished");
      }
      break;
    }
  }
}

void DescentStateMachine() {
  switch (::currentPhase) {
    case Sw:
    {
      if (GRF > FSRdescentThreshUpper)
      {        
        currentPhase = MS; 
        Serial.println("********* Descent Sw -> MS *********");
      }
      while(::p_out < 1.4)
      {
        Position_Control(1.8, 5, 2, 0.1);
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
