// Pot Mappings. Format VMax | VMin | AMax | AMin | PIN | PrevPosition | Prev Time
float R_Thaigh[7] = {0.71, 0.15, 90, -90, A1, 0, 0}; // Added 2 elements for prevPosition and prevTime
float R_Knee[7]   = {0.68, 0.18, 90, -65, A0, 0, 0}; // Added 2 elements for prevPosition and prevTime
float R_Ankle[7]  = {0.52, 0.33, 27, -45, A2, 0, 0}; // Added 2 elements for prevPosition and prevTime
float L_Thaigh[7]  = {0.58, 0.04, 90, -90, A3, 0, 0}; // Added 2 elements for prevPosition and prevTime
float L_Knee[7]  = {0.11, 0.66, 90, -90, A4, 0, 0}; // Added 2 elements for prevPosition and prevTime
float L_Ankle[7]  = {0.47, 0.28, 27, -40, A5, 0, 0}; // Added 2 elements for prevPosition and prevTime


void setup() {
  Serial.begin(9600); // Initialize serial communication at 9600 baud
}

void loop() {
  int ThaighAngle = potToVoltage(L_Ankle);

  // Print the voltages to the serial monitor
  Serial.print("Potentiometer 1 Voltage: ");
  Serial.print(ThaighAngle);
  Serial.println(" Degrees");


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

float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
