#include <Wire.h>
#include <MLX90393.h> //From https://github.com/tedyapo/arduino-MLX90393 by Theodore Yapo
#include <SCServo.h>

#define SERVO_ID 1 //  need to calibrate
#define GRIPPER_OPEN 3340 // encoder reading when gripper is open (open is positive speed)
#define OPEN_TOLERANCE 10 // reading tolerance
#define OPEN_SPEED 150 // counts per second
#define STOP 0
#define MAX_LOAD 200 // maximum load before stopping gripper (safety) 
#define CALIBRATION_LOAD 80 // load when fingers touches sides
#define LOAD_TOLERANCE 5
#define CALIBRATION_ZONE 2500

#define GRIP_SPEED -100
#define GRIPPED 550.0 // magnetic flux when gripped
#define FLUX_TOLERANCE 10
#define GRIP_DONE "GD"
#define GRIP_COMMAND "G"
#define SENSOR_INTERVAL_MS 20

#define DROP_SPEED 50
#define RELEASE_DONE "RD"
#define RELEASE_COMMAND "R"
#define RELEASE_DELAY_MS 500

#define ACTION_TIMEOUT 10

#define SKIP_CALIBRATION false

#define sgn(x) ((x) < 0 ? -1 : ((x) > 0 ? 1 : 0))

HardwareSerial ServoSerial(D0, D1); // RX=D0, TX=D1 
SMS_STS servo;

MLX90393 mlx0;
MLX90393::txyz data; //Create a structure, called data, of four floats (t, x, y, and z)

float DataR[35];
float Data[35];

// Define the size of the moving average filter
const int filterSize = 10;
// Initialize an array to store past readings
float filterArray[filterSize];

// 0: no detail, 1: magnet only, 2: all
const int detail = 0; 

// flags
bool open = false;
bool gripped = false;
bool motion_end = false;

bool gripping = false;
bool releasing = false; 

void setup() {
  Serial.begin(115200);
  Serial.begin(9600); 

  motorSetup(); 
  
  ServoSerial.begin(1000000);
  servo.pSerial = &ServoSerial;
  delay(1000);

  // Set to wheel/velocity mode
  servo.WheelMode(SERVO_ID);
  delay(100);

  servo.WriteSpe(SERVO_ID, STOP);
  servo.EnableTorque(SERVO_ID, STOP);

  // Get feedback from servo
  servoDetails(); 

  delay(1000); 

  Serial.println("Press SPACE to continue");
  while(true) {
    if (Serial.read() == ' ') {
      break; 
    }
  }

  // Calibration
  int diff = 0; 
  if (not SKIP_CALIBRATION){
    while(abs(servo.ReadLoad(SERVO_ID)) < CALIBRATION_LOAD) {
    
    // doesn't really work
    if ((abs(servo.ReadLoad(SERVO_ID)) > MAX_LOAD) and (servo.ReadPos(SERVO_ID) < CALIBRATION_ZONE)) {
      eStop(); 
    }

    if (Serial.read() == ' ') {
      eStop(); 
    }

    Serial.println("Calibrating motor... (SPACE to stop)");
    if (detail > 1) {
      Serial.println("Load: " + String(servo.ReadLoad(SERVO_ID)));
    }
    if (abs(servo.ReadLoad(SERVO_ID)) > CALIBRATION_LOAD) {
      open = true; 
      servo.WriteSpe(SERVO_ID, STOP);
      servo.EnableTorque(SERVO_ID, STOP); 
      gripped = false; 
      Serial.println("Calibration complete"); 
      break; 
    }

    diff = GRIPPER_OPEN - servo.ReadPos(SERVO_ID); 
    servo.WriteSpe(SERVO_ID, OPEN_SPEED);
    }
  }
  

  open = true; 
  servo.WriteSpe(SERVO_ID, STOP);
  servo.EnableTorque(SERVO_ID, STOP);
  delay(200); 

  Serial.println("Calibration complete"); 
  if (detail > 0) {
    Serial.println("Position after calibration:" + String(servo.ReadPos(SERVO_ID)));
    Serial.println("Load after calibration:" + String(servo.ReadLoad(SERVO_ID)));
  }

  sensorSetup(); 

  delay(1000);
}

void loop() {
  int load = abs(servo.ReadLoad(SERVO_ID)); 
  if (load > MAX_LOAD) {
      eStop(); 
  }

  servoDetails(); 

  float norm = magnetDetails(); 

  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    Serial.println(command); 

    manualServoControl(command);

    if (command == GRIP_COMMAND) {
      if (not gripped){
        grip(); 
      } else {Serial.println("Already gripped");}  
    } 

    if (command == RELEASE_COMMAND) {
      if (gripped) {
        release(); 
      } else {Serial.println("Already released");}
      
    }
  }

  delay(100);
}


void grip() {
  unsigned long lastRead = 0;
  // maybe can take away while when including ROS
  while(abs(servo.ReadLoad(SERVO_ID)) < MAX_LOAD) {
    mlx0.readData(data);
    // Serial.println("RAW x:" + String(data.x) + " baseline:" + String(Data[0]));
    // Serial.println("RAW y:" + String(data.y) + " baseline:" + String(Data[1]));
    // Serial.println("RAW z:" + String(data.z) + " baseline:" + String(Data[2]));
    delay(SENSOR_INTERVAL_MS);
    float mag_flux = magnetDetails(); 
    delay(SENSOR_INTERVAL_MS);
    
    
    if (detail > 0) {
      Serial.println("Flux:" + String(mag_flux));
    }

    if (mag_flux > GRIPPED) {
    servo.WriteSpe(SERVO_ID, STOP, 50);
    gripped = true; 
    Serial.println(GRIP_DONE); 
    return;
    //while(true); 
    } 

  if (not gripped) {
    if (detail > 0) {
      Serial.println("GRIPPING...");
    }
    servo.WriteSpe(SERVO_ID, GRIP_SPEED); 
    }
  }
  // if exceed maximum load, estop
  eStop();
}


void release() {
  servo.WriteSpe(SERVO_ID, DROP_SPEED);
  unsigned long startTime = millis();

  // drop object first (so load resets)
  while (millis() - startTime < RELEASE_DELAY_MS) {
    if (detail > 0){
      Serial.println("DROPPING...");
    }
  }

  servo.WriteSpe(SERVO_ID, OPEN_SPEED);
  while (true) {
    int load = abs(servo.ReadLoad(SERVO_ID));
    if (load > MAX_LOAD) {
      eStop();
    }
    if (load > CALIBRATION_LOAD) break;
    if (detail > 0) {
      Serial.println("RELEASING...");
    }
  }

  servo.WriteSpe(SERVO_ID, STOP, 50);
  gripped = false;
  Serial.println(RELEASE_DONE);
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


void motorSetup() {
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
  Serial.println("Motor connected");
}


void sensorSetup() {
  mlx0.begin(0,0);
  mlx0.setOverSampling(0);
  mlx0.setDigitalFiltering(0);
  delay(100);

  mlx0.readData(data); //Read the values from the sensor
  Data[0]=data.x;Data[1]=data.y;Data[2]=data.z;
  delay(500);
  //read again in case first read was corrupted
  mlx0.readData(data); //Read the values from the sensor
  Data[0]=data.x;Data[1]=data.y;Data[2]=data.z;
}


void eStop() {
  servo.WriteSpe(SERVO_ID, STOP);
  servo.EnableTorque(SERVO_ID, STOP);
  Serial.println("EMERGENCY STOP ACTIVATED");

  while(true) {
    delay(1000); 
  }
}


void servoDetails() {
  int actualSpeed = servo.ReadSpeed(SERVO_ID); // counts
  int actualPos   = servo.ReadPos(SERVO_ID); // counts
  float actualLoad = servo.ReadLoad(SERVO_ID);
  float actualCur = servo.ReadCurrent(SERVO_ID); 

  if (detail > 1){
    Serial.println("Speed:" + String(actualSpeed));
    Serial.println("Position:" + String(actualPos));
    Serial.println("Load:" + String(actualLoad));
    Serial.println("Current:" + String(actualCur));
  }
  
}


float magnetDetails() {
  mlx0.readData(data); //Read the values from the sensor
  DataR[0]=data.x;DataR[1]=data.y;DataR[2]=data.z;

  float diffx = DataR[0]-Data[0]; 
  float diffy = DataR[1]-Data[1]; 
  float diffz =  DataR[2]-Data[2]; 

  float avg_x = movingAverage(diffx); 
  float avg_y = movingAverage(diffy); 
  float avg_z =  movingAverage(diffz); 

  // float norm = sqrt(avg_x * avg_x + avg_y * avg_y + avg_z * avg_z);
  float norm = sqrt(diffx * diffx + diffy * diffy + diffz * diffz);
  // float avg_norm = movingAverage(norm); 

  if (detail > 0) { 
    Serial.print(diffx,0);
    Serial.print(",");
    Serial.print(diffy,0);
    Serial.print(",");
    Serial.println(diffz,0);

    Serial.println("Flux norm: " + String(norm)); 
  }

  return norm; 
}


void manualServoControl(String cmd) {
    if (cmd == "f") {
      servo.WriteSpe(SERVO_ID, OPEN_SPEED, 50);
      Serial.println("Forward!");
      delay(1000);
      servo.WriteSpe(SERVO_ID, STOP, 50);
    }
    else if (cmd == "b") {
      servo.WriteSpe(SERVO_ID, -OPEN_SPEED, 50); 
      Serial.println("Backward!");
      delay(1000);
      servo.WriteSpe(SERVO_ID, STOP, 50);

    }
    // else if (cmd == "s") {
    //   // Stop
    //   servo.WriteSpe(SERVO_ID, STOP, 50);
    //   servo.EnableTorque(SERVO_ID, STOP);
    //   Serial.println("Stopped!");
    // }

    else if (cmd == "s") {
      eStop(); 
    }
}

