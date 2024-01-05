// Pot Mappings. Format VMax | VMin | AMax | AMin | PIN | PrevPosition | Prev Time
float R_Thaigh[7] = {0.71, 0.15, 90, -90, A1, 0, 0};
float R_Knee[7]   = {0.68, 0.18, 90, -65, A0, 0, 0};
float R_Ankle[7]  = {0.52, 0.33, 27, -45, A2, 0, 0};
float L_Thaigh[7]  = {0.58, 0.04, 90, -90, A3, 0, 0};
float L_Knee[7]  = {0.11, 0.66, 90, -90, A4, 0, 0};
float L_Ankle[7]  = {0.47, 0.28, 27, -40, A5, 0, 0};

// Number of readings to use for the moving average
const int numReadings = 10;
bool LContact = false;
bool RContact = false;

// Arrays to store readings and indices for each joint
int readings[6][numReadings];
int indices[6] = {0, 0, 0, 0, 0, 0};

void setup() {
  Serial.begin(9600); // Initialize serial communication at 9600 baud

  // Initialize arrays to 0
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < numReadings; ++j) {
      readings[i][j] = 0;
    }
  }
}

void loop() {
  int L_Foot_Reading = analogRead(A6);
  int R_Foot_Reading = analogRead(A7);

  float L_Foot = (L_Foot_Reading * 3.3) / 4095.0;
  float R_Foot = (R_Foot_Reading * 3.3) / 4095.0;

  if (L_Foot > 0.75) {
    LContact = true;
  } else {
    LContact = false;
  }

  if (R_Foot > 0.75) {
    RContact = true;
  } else {
    RContact = false;
  }
  

  // Read angles for each joint
  int joint_angles[6]  = {
    potToVoltage(R_Thaigh),
    potToVoltage(R_Knee),
    potToVoltage(R_Ankle),
    potToVoltage(L_Thaigh),
    potToVoltage(L_Knee),
    potToVoltage(L_Ankle)
  };

  float averaged_angles[6]; // Array to store averaged angles for each joint

  // Update the moving average for each joint
  for (int i = 0; i < 6; ++i) {
    // Subtract the oldest reading
    readings[i][indices[i]] = joint_angles[i];
    // Calculate the average
    int total = 0;
    for (int j = 0; j < numReadings; ++j) {
      total += readings[i][j];
    }
    int average = total / numReadings;
    averaged_angles[i] = average; // Store the averaged angle
    // Send the averaged value for each joint
    Serial.print(average);
    if (i < 5) {
      Serial.print(",");
    }
  }

  Serial.print(",");
  velocity(R_Thaigh, averaged_angles[0]);
  velocity(R_Knee, averaged_angles[1]);
  velocity(R_Ankle, averaged_angles[2]);

  velocity(L_Thaigh, averaged_angles[3]);
  velocity(L_Knee, averaged_angles[4]);
  velocity(L_Ankle, averaged_angles[5]);

  Serial.print(LContact);
  Serial.print(",");
  Serial.print(RContact);
  Serial.println();

  // Move to the next position in the array
  for (int i = 0; i < 6; ++i) {
    indices[i] = (indices[i] + 1) % numReadings;
  }

  delay(1000); // Delay for 1 second (adjust as needed)
}

// Function to convert raw ADC value to voltage
float potToVoltage(float joint[7]) {
  int rawValue = analogRead(joint[4]);
  // Arduino Due has a 12-bit ADC, so the maximum value is 2^12 - 1 = 4095
  const float referenceVoltage = 3.3; // Arduino Due operates at 3.3V
  // Convert raw ADC value to voltage
  float voltage = (rawValue * referenceVoltage) / 4095.0;
  int angle = int(fmap(voltage, joint[1], joint[0], joint[3], joint[2]));

  return angle;
}




float velocity(float joint[7], float averagedAngle) {
  // Calculate velocity using averaged angle
  float deltaTime = (millis() - joint[6]) / 1000.0; // Time in seconds
  float deltaAngle = averagedAngle - joint[5];
  float velocity = deltaAngle / deltaTime; // Angular velocity in degrees per second
  // Update the stored previous position and time
  joint[5] = averagedAngle;  // Update prevPosition with averaged angle
  joint[6] = millis();       // Update prevTime

  Serial.print(velocity);
  Serial.print(',');
  return velocity;
}



float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
