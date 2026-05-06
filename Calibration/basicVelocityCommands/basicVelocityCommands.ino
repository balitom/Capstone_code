#include <SCServo.h>

HardwareSerial ServoSerial(PB7, PB6); // RX=PB7, TX=PB6

SMS_STS servo;

#define SERVO_ID 1

void setup() {
  Serial.begin(115200);
  while(!Serial);
  Serial.println("Velocity Control Ready!");
  Serial.println("Commands: f=forward, b=backward, s=stop");
  Serial.println("Speed: 1-9 (1=slow, 9=fast)");
  
  ServoSerial.begin(1000000);
  servo.pSerial = &ServoSerial;
  delay(1000);

  // Set to wheel/velocity mode
  servo.WheelMode(SERVO_ID);
  delay(100);
}

void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();
    
    if (cmd == 'f') {
      // Forward at speed 500
      servo.WriteSpe(SERVO_ID, 500, 50);
      Serial.println("Forward!");
    }
    else if (cmd == 'b') {
      // Backward at speed 500
      servo.WriteSpe(SERVO_ID, -500, 50);
      Serial.println("Backward!");
    }
    else if (cmd == 's') {
      // Stop
      servo.WriteSpe(SERVO_ID, 0, 50);
      Serial.println("Stopped!");
    }
    // Speed control 1-9
    else if (cmd >= '1' && cmd <= '9') {
      int spd = (cmd - '0') * 111;
      servo.WriteSpe(SERVO_ID, spd, 50);
      Serial.print("Speed set to: ");
      Serial.println(spd);
    }
  }
}
