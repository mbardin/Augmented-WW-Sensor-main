
//libraries to include
#include <SparkFun_SDP3x_Arduino_Library.h>
#include<Wire.h>
#include <BluetoothSerial.h>
#include <MPU6050.h> // use a different library to account for gravity?

BluetoothSerial SerialBT;
MPU6050 mpu;
SDP3X mySensor; //rename once working
// continuius sensing is on 0x21

const int pwrLED = 2; //green 220 Ohm (make larger)
const int msgLED = 4; //red 220 Ohm
const int button1 = 12;
const int button2 = 13;

const int MPU_ADDR = 0x68; // I2C address of the MPU-6050
int16_t AcX, AcY, AcZ, GyX, GyY, GyZ; // values for 6050
// use SDP31 for temp if desired

int pB1 = 0;
int pB2 = 0;
int button1State = 0;
int button2State = 0;


void setup() {
  pinMode(pwrLED, OUTPUT);
  pinMode(msgLED, OUTPUT);
  pinMode(button1, INPUT_PULLUP);
  pinMode(button2, INPUT_PULLUP);

  SerialBT.begin("Cyberinet V.1.1"); // Device name that will appear on computer.
  delay(1000);
  Serial.begin(115200);
  Wire.begin(21, 22, 100000); // sda, scl, clock speed
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU−6050)
  Wire.endTransmission(true);
  SerialBT.println("6050 Setup Complete");
  Serial.println("6050 Setup Complete");

  mySensor.stopContinuousMeasurement();

  // Initialize sensor
  // Note: this would fail if continuous measurements were already running
  if (mySensor.begin() == false)
  {
    Serial.println(F("SDP3X not detected. Check connections. Freezing..."));
    while (1)
      ; // Do nothing more
  }





  mySensor.startContinuousMeasurement(true, true);
  SerialBT.println("SDP31 Setup Complete");
  Serial.println("SDP31 Setup Complete");
  startLights();

}

void loop() {
  delay(100);
  get6050();
  delay(100);
  getButtons();
  //delay(100);
  getAir();

}

void startLights() {
  digitalWrite(pwrLED, HIGH);
  digitalWrite(msgLED, HIGH);
  delay(500);
  digitalWrite(msgLED, LOW);
  delay(500);
  digitalWrite(msgLED, HIGH);
  delay(500);
  digitalWrite(msgLED, LOW);
  delay(500);
  digitalWrite(msgLED, HIGH);
  delay(1000);
  digitalWrite(msgLED, LOW);
}

void get6050() {
  //map values here?

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(true);
  Wire.beginTransmission(MPU_ADDR);
  Wire.requestFrom(MPU_ADDR, 14, true); // request a total of 14 registers
  AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  GyX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  Serial.print("AcX "); Serial.println(AcX);
  SerialBT.print("AcX "); SerialBT.println(AcX);

  Serial.print("AcY "); Serial.println(AcY);
  SerialBT.print("AcY "); SerialBT.println(AcY);

  Serial.print("AcZ "); Serial.println(AcZ);
  SerialBT.print("AcX "); SerialBT.println(AcZ);

  Serial.print("GyX "); Serial.println(GyX);
  SerialBT.print("GyX "); SerialBT.println(GyX);

  Serial.print("GyY "); Serial.println(GyY);
  SerialBT.print("GyY "); SerialBT.println(GyY);

  Serial.print("GyZ "); Serial.println(GyZ);
  SerialBT.print("GyZ "); SerialBT.println(GyZ);

}

void getButtons() { // currently wenever pressed. look into holding and release options
  pB1 = button1State;
  pB2 = button2State;
  button1State = digitalRead(button1);
  button2State = digitalRead(button2);

  if (button1State > pB1) {
    Serial.print("button1 "); Serial.println(button1State);
    SerialBT.print("button1 "); SerialBT.println(button1State);
  }

  if (button2State > pB2) {
    Serial.print("button2 "); Serial.println(button2State);
    SerialBT.print("button2 "); SerialBT.println(button2State);
  }

}

void getAir() {
  
  // look at combining this function with the 6050 function.
  // maybe telling it to talk to a specific address first will make life easier.


  Wire.beginTransmission(0x21);

  Wire.endTransmission(true);
  Wire.beginTransmission(0x21);

  float diffPressure; // Storage for the differential pressure
  float temperature; // Storage for the temperature

  // Read the averaged differential pressure and temperature from the sensor
  mySensor.readMeasurement(&diffPressure, &temperature); // Read the measurement

  Serial.print(F("Differential pressure is: "));
  Serial.print(diffPressure, 2);
  Serial.print(F(" (Pa)  Temperature is: "));
  Serial.print(temperature, 2);
  Serial.println(F(" (C)"));
}
