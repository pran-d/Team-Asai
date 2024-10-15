int readMode(){
  int pressed = 0;
  if(digitalRead(2)==1) pressed = 1;
  if(digitalRead(12)==0) pressed = 2;
  if(digitalRead(4)==0) pressed = 3;
  if(digitalRead(13)==0) pressed = 4;
  return pressed;
}

void WalkingStateMachine() 
{
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
      if(sensors.thighAngle > theta_t_ascent && sensors.fsr3 > 350){
        while (::p_out > 0.3)
        {
          // Position_Control(0, 5, 2, -0.1);
          cable_taut(-10);
        }
        Serial.println("Start Climbing");
        Stair_Ascent_Loading();
        Serial.println("Stair Ascent Finished");
        while (::p_out < 1.5)
        {
          Position_Control(1.8, 5, 2, 0.1);
        }
      }
      break;
    }
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
