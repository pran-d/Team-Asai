// Serve the HTML form when visiting the root page

void handleRoot() {
  String htmlForm = R"(
    <!DOCTYPE html>
    <html>
      <head>
        <title>Variable Input</title>
        <style>
          table {
            width: 50%;
            margin: auto;
            border-collapse: collapse;
          }
          td {
            padding: 10px;
            text-align: center;
          }
          input {
            width: 100%;
            padding: 5px;
          }
          label {
            font-weight: bold;
          }
        </style>
      </head>
      
      <body>
        <form action="/modify" method="POST">
          <table>
            <!-- Row for StanceThresh -->
            <tr>
              <td>
                <label for="">StanceThresh:</label><br>
                <input type="text" id="StanceThresh" name="StanceThresh" value=")";
  htmlForm += String(StanceThresh); // Insert region1 variable with 2 decimal places
            

  htmlForm += R"(">
              </td>

              <td>
                <label for="FSRdescentThreshLower">FSRdescentThreshLower:</label><br>
                <input type="text" id="FSRdescentThreshLower" name="FSRdescentThreshLower" value=")";
  htmlForm += String(FSRdescentThreshLower); // Insert region2 variable with 2 decimal places
  htmlForm += R"(">
              </td>

              <td>
                <label for="FSRdescentThreshUpper">FSRdescentThreshUpper:</label><br>
                <input type="text" id="FSRdescentThreshUpper" name="FSRdescentThreshUpper" value=")";
  htmlForm += String(FSRdescentThreshUpper); // Insert region3 variable with 2 decimal places
  

  htmlForm += R"(">
              </td>
            </tr>

            <!-- Row for FSRascentThresh1, FSRascentThresh2 -->
            <tr>
              <td>
                <label for="FSRascentThresh1">FSRascentThresh1:</label><br>
                <input type="text" id="FSRascentThresh1" name="FSRascentThresh1" value=")";
  htmlForm += String(FSRascentThresh1); // Insert k2 variable with 2 decimal places
  htmlForm += R"(">
              </td>

              <td>
                <label for="FSRascentThresh2">FSRascentThresh2:</label><br>
                <input type="text" id="FSRascentThresh2" name="FSRascentThresh2" value=")";
  htmlForm += String(FSRascentThresh2);


  htmlForm += R"(">
              </td>
            </tr>

            <!-- Row for theta_t_walking, theta_t_ascent -->
            <tr>
              <td>
                <label for="theta_t_walking">theta_t_walking:</label><br>
                <input type="text" id="theta_t_walking" name="theta_t_walking" value=")";
  htmlForm += String(theta_t_walking, 2); // Insert k2 variable with 2 decimal places
  htmlForm += R"(">
              </td>

              <td>
                <label for="theta_t_ascent">theta_t_ascent:</label><br>
                <input type="text" id="theta_t_ascent" name="theta_t_ascent" value=")";
  htmlForm += String(theta_t_ascent, 2); // Insert k3 variable with 2 decimal places


  htmlForm += R"(">
              </td>
            </tr>

            <!-- Row for imuHighVelThresh, imuFlexionThresh, imuStableAngle -->
            <tr>
              <td>
                <label for="imuHighVelThresh">imuHighVelThresh:</label><br>
                <input type="text" id="imuHighVelThresh" name="imuHighVelThresh" value=")";
  htmlForm += String(imuHighVelThresh, 2); // Insert region1 variable with 2 decimal places
  htmlForm += R"(">
              </td>
              <td>
                <label for="imuFlexionThresh">imuFlexionThresh:</label><br>
                <input type="text" id="imuFlexionThresh" name="imuFlexionThresh" value=")";
  htmlForm += String(imuFlexionThresh, 2); // Insert region2 variable with 2 decimal places
  htmlForm += R"(">
              </td>
              <td>
                <label for="imuStableAngle">imuStableAngle:</label><br>
                <input type="text" id="imuStableAngle" name="imuStableAngle" value=")";
  htmlForm += String(imuStableAngle, 2); // Insert region3 variable with 2 decimal places
  

  htmlForm += R"(">
              </td>
            </tr>

            <!-- Row for Stair_Ascent_a, Stair_Ascent_b, float Stair_Ascent_c -->
            <tr>
              <td>
                <label for="Stair_Ascent_a">Stair_Ascent_a:</label><br>
                <input type="text" id="Stair_Ascent_a" name="Stair_Ascent_a" value=")";
  htmlForm += String(Stair_Ascent_a, 6); // Insert region1 variable with 2 decimal places
  htmlForm += R"(">
              </td>
              <td>
                <label for="Stair_Ascent_b">Stair_Ascent_b:</label><br>
                <input type="text" id="Stair_Ascent_b" name="Stair_Ascent_b" value=")";
  htmlForm += String(Stair_Ascent_b, 6); // Insert region2 variable with 2 decimal places
  htmlForm += R"(">
              </td>
              <td>
                <label for="Stair_Ascent_c">Stair_Ascent_c:</label><br>
                <input type="text" id="Stair_Ascent_c" name="Stair_Ascent_c" value=")";
  htmlForm += String(Stair_Ascent_c, 6);


  htmlForm += R"(">
              </td>
            </tr>

            <!-- Row for Stair_Ascent_A, Stair_Ascent_k -->
            <tr>
              <td>
                <label for="Stair_Ascent_A">Stair_Ascent_A:</label><br>
                <input type="text" id="Stair_Ascent_A" name="Stair_Ascent_A" value=")";
  htmlForm += String(Stair_Ascent_A, 6); // Insert k2 variable with 2 decimal places
  htmlForm += R"(">
              </td>

              <td>
                <label for="Stair_Ascent_k">Stair_Ascent_k:</label><br>
                <input type="text" id="Stair_Ascent_k" name="Stair_Ascent_k" value=")";
  htmlForm += String(Stair_Ascent_k, 6);

  htmlForm += R"(">
              </td>
              <td>
                <label for="I_max_Ascent">I_max_Ascent:</label><br>
                <input type="text" id="I_max_Ascent" name="I_max_Ascent" value=")";
  htmlForm += String(I_max_Ascent, 2);

  htmlForm += R"(">
              </td>
            </tr>

            <!-- Row for k1, k2, k3 -->
            <tr>
              <td>
                <label for="k1">k1:</label><br>
                <input type="text" id="k1" name="k1" value=")";
  htmlForm += String(k1, 2); // Insert region1 variable with 2 decimal places
  htmlForm += R"(">
              </td>
              <td>
                <label for="k2">k2:</label><br>
                <input type="text" id="k2" name="k2" value=")";
  htmlForm += String(k2, 2); // Insert region2 variable with 2 decimal places
  htmlForm += R"(">
              </td>
              <td>
                <label for="k3">k3:</label><br>
                <input type="text" id="k3" name="k3" value=")";
  htmlForm += String(k3, 2); // Insert region3 variable with 2 decimal places

  htmlForm += R"(">
              </td>
            </tr>

            <!-- Row for d1, d2, d3 -->
            <tr>
              <td>
                <label for="d1">d1:</label><br>
                <input type="text" id="d1" name="d1" value=")";
  htmlForm += String(d1, 3); // Insert region1 variable with 2 decimal places
  htmlForm += R"(">
              </td>
              <td>
                <label for="d2">d2:</label><br>
                <input type="text" id="d2" name="d2" value=")";
  htmlForm += String(d2, 3); // Insert region2 variable with 2 decimal places
  htmlForm += R"(">
              </td>
              <td>
                <label for="d3">d3:</label><br>
                <input type="text" id="d3" name="d3" value=")";
  htmlForm += String(d3, 3); // Insert region3 variable with 2 decimal places

  htmlForm += R"(">
                </td>
              </tr>

              <!-- Row for descentDeadband, descentKp -->
              <tr>
                <td>
                  <label for="descentDeadband">descentDeadband:</label><br>
                  <input type="text" id="descentDeadband" name="descentDeadband" value=")";
  htmlForm += String(descentDeadband, 2); // Insert region1 variable with 2 decimal places
  
  htmlForm += R"(">
              </td>
              
              <td>
                  <label for="descentKp">descentKp:</label><br>
                  <input type="text" id="descentKp" name="descentKp" value=")";
  htmlForm += String(descentKp, 2);
  htmlForm += R"(">
              </td>
            </tr>

            <!-- Row for region1, region2, region3 -->
            <tr>
              <td>
                <label for="region1">region1:</label><br>
                <input type="text" id="region1" name="region1" value=")";
  htmlForm += String(region1, 2); // Insert region1 variable with 2 decimal places
  htmlForm += R"(">
              </td>
              <td>
                <label for="region2">region2:</label><br>
                <input type="text" id="region2" name="region2" value=")";
  htmlForm += String(region2, 2); // Insert region2 variable with 2 decimal places
  htmlForm += R"(">
              </td>
              <td>
                <label for="region3">region3:</label><br>
                <input type="text" id="region3" name="region3" value=")";
  htmlForm += String(region3, 2); // Insert region3 variable with 2 decimal places

  htmlForm += R"(">
              </td>
            </tr>

            <!-- Row for knee_AscentRampDown, knee_AscentRampDownStop -->
            <tr>
              <td>
                <label for="knee_AscentRampDown">knee_AscentRampDown:</label><br>
                <input type="text" id="knee_AscentRampDown" name="knee_AscentRampDown" value=")";
  htmlForm += String(knee_AscentRampDown, 2); // Insert k2 variable with 2 decimal places
  htmlForm += R"(">
              </td>

              <td>
                <label for="knee_AscentRampDownStop">knee_AscentRampDownStop:</label><br>
                <input type="text" id="knee_AscentRampDownStop" name="knee_AscentRampDownStop" value=")";
  htmlForm += String(knee_AscentRampDownStop, 2); // Insert k3 variable with 2 decimal places

  htmlForm += R"(">
              </td>
            </tr>

            <!-- Row for iRef, iRef1 -->
            <tr>
              <td>
                <label for="iRef">iRef:</label><br>
                <input type="text" id="iRef" name="iRef" value=")";
  htmlForm += String(iRef, 2); // Insert k2 variable with 2 decimal places
  htmlForm += R"(">
              </td>

              <td>
                <label for="iRef1">iRef1:</label><br>
                <input type="text" id="iRef1" name="iRef1" value=")";
  htmlForm += String(iRef1, 2); // Insert k2 variable with 2 decimal places
  
  htmlForm += R"(">
              </td>
            </tr>
          </table>

          <div style="text-align: center; margin-top: 20px;">
            <input type="submit" value="Submit">
          </div>
        </form>

        <form action="/Ascent" method="POST">
      <div style="text-align: center; margin-top: 20px;">
        <input type="submit" value="Ascent">
      </div>
    </form>

        <form action="/Descent" method="POST">
      <div style="text-align: center; margin-top: 20px;">
        <input type="submit" value="Descent">
      </div>
    </form>

      <form action="/Passive" method="POST">
      <div style="text-align: center; margin-top: 20px;">
        <input type="submit" value="Passive">
      </div>
    </form>

      </body>
    </html>
  )";
  
  server.send(200, "text/html", htmlForm);
}

void handleModify() {
  if (server.hasArg("StanceThresh")) {
    StanceThresh = server.arg("StanceThresh").toInt();  // get first character
  }

  if (server.hasArg("theta_t_walking")) {
    theta_t_walking = server.arg("theta_t_walking").toFloat();
  }
  if (server.hasArg("theta_t_ascent")) {
    theta_t_ascent = server.arg("theta_t_ascent").toFloat();
  }


  if (server.hasArg("FSRdescentThreshLower")) {
    FSRdescentThreshLower = server.arg("FSRdescentThreshLower").toInt();
  }
  if (server.hasArg("FSRdescentThreshUpper")) {
    FSRdescentThreshUpper = server.arg("FSRdescentThreshUpper").toInt();
  }

  if (server.hasArg("FSRascentThresh1")) {
    FSRascentThresh1 = server.arg("FSRascentThresh1").toInt();
  }
  if (server.hasArg("FSRascentThresh2")) {
    FSRascentThresh2 = server.arg("FSRascentThresh2").toInt();
  }


  if (server.hasArg("imuHighVelThresh")) {
    imuHighVelThresh = server.arg("imuHighVelThresh").toFloat();
  }
  if (server.hasArg("imuFlexionThresh")) {
    imuFlexionThresh = server.arg("imuFlexionThresh").toFloat();
  }
  if (server.hasArg("imuStableAngle")) {
    imuStableAngle = server.arg("imuStableAngle").toFloat();
  }


  if (server.hasArg("k1")) {
    k1 = server.arg("k1").toFloat();
  }
  if (server.hasArg("k2")) {
    k2 = server.arg("k2").toFloat();
  }
  if (server.hasArg("k3")) {
    k3 = server.arg("k3").toFloat();
  }


  if (server.hasArg("Stair_Ascent_a")) {
    Stair_Ascent_a = server.arg("Stair_Ascent_a").toFloat();
  }
  if (server.hasArg("Stair_Ascent_b")) {
    Stair_Ascent_b = server.arg("Stair_Ascent_b").toFloat();
  }
  if (server.hasArg("Stair_Ascent_c")) {
    Stair_Ascent_c = server.arg("Stair_Ascent_c").toFloat();
  }


  if (server.hasArg("Stair_Ascent_A")) {
    Stair_Ascent_A = server.arg("Stair_Ascent_A").toFloat();
  }
  if (server.hasArg("Stair_Ascent_k")) {
    Stair_Ascent_k = server.arg("Stair_Ascent_k").toFloat();
  }

  if (server.hasArg("I_max_Ascent")) {
    I_max_Ascent = server.arg("I_max_Ascent").toFloat();
  }

  if (server.hasArg("region1")) {
    region1 = server.arg("region1").toFloat();
  }
  if (server.hasArg("region2")) {
    region2 = server.arg("region2").toFloat();
  }
  if (server.hasArg("region3")) {
    region3 = server.arg("region3").toFloat();
  }


  if (server.hasArg("iRef")) {
    iRef = server.arg("iRef").toFloat();
  }
  if (server.hasArg("iRef1")) {
    iRef1 = server.arg("iRef1").toFloat();
  }
  Serial.print("I_max_ascent= ");Serial.println(I_max_Ascent);

  // Send a simple response back to the client (the browser won't reload)
  server.send(200, "text/plain", "Values updated");
}

void handleAscent()
{
  currentMode= Stair;
  currentPhase= Standing;
  currentState= Walking;
  server.send(200, "text/plain", "Ascent");
}

void handleDescent()
{
  currentMode= Stair;
  currentPhase= Standing;
  currentState= Walking;
  server.send(200, "text/plain", "Descent");
}

void handlePassive()
{
  currentMode= Passive;
  currentPhase= Standing;
  currentState= Walking;
  server.send(200, "text/plain", "Passive");
}
// Handle form submission and update variables
// void handleModify() {
//   // Handle Mode (single character)
//   if (server.hasArg("mode")) {
//     String modeInput = server.arg("mode");
//     mode = modeInput.charAt(0);
//     Serial.print("Received character: "); Serial.println(mode);
//   }

//   // Handle k1, k2, k3 (floats)
//   if (server.hasArg("k1")) {
//     k1 = server.arg("k1").toFloat();  // Convert string to float and update k1
//   }

//   if (server.hasArg("k2")) {
//     k2 = server.arg("k2").toFloat();  // Convert string to float and update k2
//   }

//   if (server.hasArg("k3")) {
//     k3 = server.arg("k3").toFloat();  // Convert string to float and update k3
//   }

//   // Handle region1, region2, region3 (floats)
//   if (server.hasArg("region1")) {
//     region1 = server.arg("region1").toFloat();  // Convert string to float and update region1
//   }

//   if (server.hasArg("region2")) {
//     region2 = server.arg("region2").toFloat();  // Convert string to float and update region2
//   }

//   if (server.hasArg("region3")) {
//     region3 = server.arg("region3").toFloat();  // Convert string to float and update region3
//   }

  // Send a response back to the client
  // String response = "<h1>Values Updated!</h1>";
  // response += "<p>Updated values are as follows:</p>";
  // response += "<ul>";
  // response += "<li>Mode: " + String(mode) + "</li>";
  // response += "<li>k1: " + String(k1, 2) + "</li>";  // Two decimal places for floats
  // response += "<li>k2: " + String(k2, 2) + "</li>";
  // response += "<li>k3: " + String(k3, 2) + "</li>";
  // response += "<li>region1: " + String(region1, 2) + "</li>";
  // response += "<li>region2: " + String(region2, 2) + "</li>";
  // response += "<li>region3: " + String(region3, 2) + "</li>";
  // response += "</ul>";
  // response += "<a href=\"/\">Go Back</a>";  // Link to go back to the form page

  // Send the response back to the client
  //server.send(200, "text/html", response);
  // }