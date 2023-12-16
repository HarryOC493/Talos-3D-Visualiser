const int trigPin = 2;
const int echoPin = 3;

float duration, distance;

void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  Serial.begin(9600);
}

float prevDistance = 0;

void loop() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance = (duration * 0.0343) / 2; // Speed of sound in air is approximately 343 meters per second
  
  Serial.print("Distance: ");
  
  if (distance >= (prevDistance + 100)) {
    Serial.print(prevDistance);
    Serial.println(" cm");
  } else {
    Serial.print(distance);
    Serial.println(" cm");
    prevDistance = distance;
  }
  
  delay(500);
}
