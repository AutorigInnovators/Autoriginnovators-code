#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Servo.h>


int data = 0;
// Initialize MPU6050 and Servo
Adafruit_MPU6050 mpu;
Servo leftServo, rightServo;

const int leftServoPin = 9;
const int rightServoPin = 10;

// Tilt Thresholds (in degrees)
const float TILT_DEPLOY_THRESHOLD = 40.0;
const float TILT_RETRACT_THRESHOLD = 5.0;

// Servo Angles
const int SERVO_DEPLOY_ANGLE = 110;
const int SERVO_RETRACT_ANGLE = 0;

// State to track if the outriggers are deployed
bool outriggersDeployed = false;
char bluetoothCommand;  // Stores received Bluetooth command


void setup() {
   //Serial.begin(115200);   // For debugging on Serial Monitor
    Serial.begin(9600);    // For Bluetooth communication (default baud rate)

    leftServo.attach(leftServoPin);
    rightServo.attach(rightServoPin);
    retractOutriggers(); // Start with outriggers retracted

      // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip!");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 connected!");

    // Configure MPU6050
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

  delay(100);
}

void loop() {
  // Handle Bluetooth Input
    if (Serial.available() > 0) {
        bluetoothCommand = Serial.read(); // Read Bluetooth command
        handleBluetoothCommand(bluetoothCommand); // Process the command
    }

     // Get sensor data
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Calculate tilt angle (approximation using accelerometer data)
  float pitch = atan2(a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180 / PI;
  float roll = atan2(a.acceleration.y, a.acceleration.z) * 180 / PI;

 // Print tilt values for debugging
  Serial.print("Pitch: ");
  Serial.println(pitch);
  Serial.print(" Roll: ");
  Serial.println(roll);

  // Control Outriggers Based on Tilt


  if ((abs(pitch) > TILT_DEPLOY_THRESHOLD || abs(roll) > TILT_DEPLOY_THRESHOLD) && !outriggersDeployed) {
    //if ((abs(pitch) > tiltThreshold) && !outriggersDeployed) {
    Serial.println("Tilt detected! Deploying outriggers.");
    deployOutriggers();
    delay(2000); // Short delay for sensor stability
}


else if ((abs(pitch) <= TILT_RETRACT_THRESHOLD && abs(roll) <= TILT_RETRACT_THRESHOLD) && outriggersDeployed) {
      //else if ((abs(pitch) <= tiltThreshold1)  && outriggersDeployed) {
    Serial.println("Tilt back to normal. Retracting outriggers.");
     retractOutriggers();
         delay(2000); // Short delay for sensor stability


  }
   delay(100); // Short delay for sensor stability
  }
  


// Deploy Outriggers
void deployOutriggers() {
    leftServo.write(SERVO_DEPLOY_ANGLE);
    rightServo.write(SERVO_DEPLOY_ANGLE);
    outriggersDeployed = true;
}

// Retract Outriggers
void retractOutriggers() {
    leftServo.write(SERVO_RETRACT_ANGLE);
    rightServo.write(SERVO_RETRACT_ANGLE);
    outriggersDeployed = false;
}

void handleBluetoothCommand(char command) {
    switch (command) {
        case '1': // Command to deploy outriggers
            Serial.println("Bluetooth: Deploying outriggers.");
            deployOutriggers();
                delay(2000); // Short delay for sensor stability

            break;

        case '0': // Command to retract outriggers
            Serial.println("Bluetooth: Retracting outriggers.");
            retractOutriggers();
                delay(2000); // Short delay for sensor stability

            break;

        case '2': // Example: Additional command
            Serial.println("Bluetooth: Additional command executed.");
            // Add custom behavior here if needed
            break;

        default: // Unknown command
            Serial.println("Bluetooth: Unknown command.");
            break;
    }
}
  
