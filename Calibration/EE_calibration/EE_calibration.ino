#include <Wire.h>
#include <MLX90393.h> //From https://github.com/tedyapo/arduino-MLX90393 by Theodore Yapo
#include <SCServo.h>

#define SERVO_ID 4
#define GRIPPER_OPEN 1200 // encoder reading when gripper is open
#define TOLERANCE 10 // reading tolerance
#define OPEN_SPEED 500
#define STOP 0

#define GRIPPED 300 // magnetic flux when gripped
#define FLUX_TOLERANCE 10

HardwareSerial ServoSerial(D0, D1); // RX=D0, TX=D1 
SMS_STS servo;

MLX90393 mlx0;
MLX90393::txyz data; //Create a structure, called data, of four floats (t, x, y, and z)

float DataR[35];
float Data[35];

// Define the size of the moving average filter
const int filterSize = 20;
// Initialize an array to store past readings
float filterArray[filterSize];

const int detail = 1; 

// flags
bool open = false;
bool gripped = false;
bool motion_end = false;

void setup() {
  Serial.begin(115200);

  sensorSetup(); 

  ServoSerial.begin(1000000);
  servo.pSerial = &ServoSerial;
  delay(1000);

  // Set to wheel/velocity mode
  servo.WheelMode(SERVO_ID);
  delay(100);

  servo.WriteSpe(SERVO_ID, 0);

  // Get feedback from servo
  int actualSpeed = servo.ReadSpeed(SERVO_ID); // counts
  int actualPos   = servo.ReadPos(SERVO_ID); // counts
  int actualLoad = servo.ReadLoad(SERVO_ID);

  Serial.println("Speed:" + String(actualSpeed));
  Serial.println("Position:" + String(actualPos));
  Serial.println("Load:" + String(actualLoad));

  Serial.println((servo.ReadPos(SERVO_ID)-GRIPPER_OPEN));
  delay(1000); 

  while(abs(servo.ReadPos(SERVO_ID)-GRIPPER_OPEN) > TOLERANCE) {
    Serial.println((servo.ReadPos(SERVO_ID)-GRIPPER_OPEN));
    servo.WriteSpe(SERVO_ID, OPEN_SPEED);
  }
  servo.WriteSpe(SERVO_ID, STOP);
}

void loop() {
  mlx0.readData(data); //Read the values from the sensor
  DataR[0]=data.x;DataR[1]=data.y;DataR[2]=data.z;

  float norm = sqrt(data.x * data.x + data.y * data.y + data.z * data.z);

  if (detail > 0) { 
    Serial.print(movingAverage(DataR[0]-Data[0]),0);
    Serial.print(",");
    Serial.print(movingAverage(DataR[1]-Data[1]),0);
    Serial.print(",");
    Serial.println(movingAverage(DataR[2]-Data[2]),0);
  }

  Serial.println(norm); 

  // Get feedback from servo
  int actualSpeed = servo.ReadSpeed(SERVO_ID); // counts
  int actualPos   = servo.ReadPos(SERVO_ID); // counts
  int actualLoad = servo.ReadLoad(SERVO_ID);

  Serial.println("Speed:" + String(actualSpeed));
  Serial.println("Position:" + String(actualPos));
  Serial.println("Load:" + String(actualLoad));
  delay(500);
}

// Function to add a new reading to the filter array and return the average
float movingAverage(float newValue) {
    // Shift all previous readings to make room for the new reading
    for (int i = filterSize - 1; i > 0; i--) {
        filterArray[i] = filterArray[i - 1];
    }
    // Add the new reading
    filterArray[0] = newValue;

    // Calculate the average
    float sum = 0;
    for (int i = 0; i < filterSize; i++) {
        sum += filterArray[i];
    }
    return sum / filterSize;
}

void sensorSetup() {
  Wire.begin();
  Wire.setClock(100000);   // stay at 100 kHz for now
  delay(50);

  // address search for I2C (debugging) 
  Serial.println("Scanning...");
  for (byte addr = 8; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    byte error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("Device found at 0x");
      Serial.println(addr, HEX);
    }
  }
  Serial.println("Done");

  mlx0.begin(0,0);
  mlx0.setOverSampling(0);
  mlx0.setDigitalFiltering(0);
  delay(100);

  mlx0.readData(data); //Read the values from the sensor
  Data[0]=data.x;Data[1]=data.y;Data[2]=data.z;
  delay(300);
  //read again in case first read was corrupted
  mlx0.readData(data); //Read the values from the sensor
  Data[0]=data.x;Data[1]=data.y;Data[2]=data.z;
}


