#include <SCServo.h>

HardwareSerial ServoSerial(PB7, PB6);
SMS_STS servo;

#define SERVO_ID 1

// PI gains — tune these
float Kp = 0.5;
float Ki = 0.1;

float targetSpeed = 200;  // steps/sec (change this to set desired speed)
float integral = 0;
unsigned long lastTime = 0;

void setup() {
  Serial.begin(115200);
  while(!Serial);
  
  ServoSerial.begin(1000000);
  servo.pSerial = &ServoSerial;
  delay(1000);
  
  // Set wheel mode for velocity control
  servo.WheelMode(SERVO_ID);
  delay(100);
  
  Serial.println("PI Velocity Control Running!");
  Serial.println("Target speed: " + String(targetSpeed) + " steps/sec");
  Serial.println("Target(steps/s) | Actual(steps/s) | Error | Output | Pos");
  
  lastTime = millis();
}

void loop() {
  // Calculate time step
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0; // convert to seconds
  lastTime = now;

  // Get feedback from servo
  int actualSpeed = servo.ReadSpeed(SERVO_ID);
  int actualPos   = servo.ReadPos(SERVO_ID);

  // Skip if feedback invalid
  if (actualSpeed == -1) {
    Serial.println("Servo not responding!");
    delay(100);
    return;
  }

  // PI calculation
  float error = targetSpeed - actualSpeed;
  integral += error * dt;

  // Anti-windup — clamp integral
  integral = constrain(integral, -1000, 1000);

  float output = (Kp * error) + (Ki * integral);

  // Clamp output to valid servo range
  output = constrain(output, -32767, 32767);

  // Send velocity command to servo
  servo.WriteSpe(SERVO_ID, (int)output, 50);

  // Print data
  Serial.print(targetSpeed);
  Serial.print(" | ");
  Serial.print(actualSpeed);
  Serial.print(" | ");
  Serial.print(error);
  Serial.print(" | ");
  Serial.print(output);
  Serial.print(" | ");
  Serial.println(actualPos);

  delay(50); // 20Hz control loop
}
