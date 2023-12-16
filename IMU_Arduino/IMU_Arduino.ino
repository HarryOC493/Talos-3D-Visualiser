#include <Arduino_LSM9DS1.h>
#include <Wire.h> // Include the Wire library for I2C communication

void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Started");

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Acceleration in g's");
  Serial.println("X\tY\tZ");

  Serial.print("Gyroscope sample rate = ");
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Gyroscope in degrees/second");
  Serial.println("X\tY\tZ");

  Serial.println("Azimuth (degrees)");

  Wire.begin(); // Initialize the I2C communication for the magnetometer
}

void loop() {
  float x_acc, y_acc, z_acc;
  float x_gyro, y_gyro, z_gyro;

  // Read accelerometer data
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x_acc, y_acc, z_acc);
  }

  // Read gyroscope data
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(x_gyro, y_gyro, z_gyro);
  }

  // Read magnetometer data and calculate azimuth
  float x_mag, y_mag, z_mag;
  float azimuth;

  if (IMU.magneticFieldAvailable()) {
    IMU.readMagneticField(x_mag, y_mag, z_mag);

    // Calculate azimuth (angle between magnetic north and XY components)
    azimuth = atan2(y_mag, x_mag) * (180.0 / PI);
    if (azimuth < 0) {
      azimuth += 360.0; // Ensure azimuth is in the range [0, 360)
    }
  }

  // Print accelerometer, gyroscope, and azimuth data
  Serial.print(x_acc);
  Serial.print(',');
  Serial.print(y_acc);
  Serial.print(',');
  Serial.print(z_acc);
  Serial.print(',');

  Serial.print(x_gyro);
  Serial.print(',');
  Serial.print(y_gyro);
  Serial.print(',');
  Serial.print(z_gyro);
  Serial.print(',');

  Serial.println(azimuth);

  delay(1000); // Delay for 1 second (adjust as needed)
}
