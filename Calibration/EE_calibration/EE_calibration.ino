#include <Wire.h>
#include <MLX90393.h> //From https://github.com/tedyapo/arduino-MLX90393 by Theodore Yapo
#include <SCServo.h>

#define SERVO_ID 1

HardwareSerial ServoSerial(PB7, PB6); // RX=PB7, TX=PB6 
SMS_STS servo;

MLX90393 mlx0;
MLX90393::txyz data; //Create a structure, called data, of four floats (t, x, y, and z)

float DataR[35];
float Data[35];

// Define the size of the moving average filter
const int filterSize = 20;

// Initialize an array to store past readings
float filterArray[filterSize];

void setup() {
  Serial.begin(115200);

  Wire.begin();
  Wire.setClock(100000);   // stay at 100 kHz for now
  delay(50);

  sensorSetup(); 

  ServoSerial.begin(1000000);
  servo.pSerial = &ServoSerial;
  delay(1000);

  // Set to wheel/velocity mode
  servo.WheelMode(SERVO_ID);
  delay(100);
}

void loop() {
  mlx0.readData(data); //Read the values from the sensor
  DataR[0]=data.x;DataR[1]=data.y;DataR[2]=data.z;

  Serial.print(movingAverage(DataR[0]-Data[0]),0);
  Serial.print(",");
  Serial.print(movingAverage(DataR[1]-Data[1]),0);
  Serial.print(",");
  Serial.println(movingAverage(DataR[2]-Data[2]),0);
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


