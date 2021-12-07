
// Augmented Woodwind Sensor v 0.3
// code by Matthew A. Bardin
// this version Dec 10 2021


//libraries used:
#include <Q2HX711.h>
#include <Wire.h>
#include <MPU6050.h>
#include <BluetoothSerial.h>
#include <arduino-timer.h>
#include <LibPrintf.h>

// include libraries in software download for user access.

const int MPU_addr = 0x68; // I2C address of the MPU-6050
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
//int avg_size = 10; // #pts to average over

//pins used:
const int button1 = 13;
const int button2 = 12;
const int ledPin = 2; // power on indicator
const int pwrLed = 3; //low battery indicator light
const int indexPress = 33; // FSR's
const int middlePress = 32;
const int ringPress  = 35;
const byte MPS_OUT_pin = 25; // OUT data pin
const byte MPS_SCK_pin = 18; // clock data pin

const int modePin = 10; //pin for DIP switch. will change between wireless or wired. adjust value once switch is connected.

Q2HX711 MPS20N0040D(MPS_OUT_pin, MPS_SCK_pin); // start comm with the HX710B


// variables to store pin values & initial values:
int button1State = 0;
int button2State = 0;
int indexPressVal = 0;
int middlePressVal = 0;
int ringPressVal = 0;
float airP = 0;
float avgVal = 0;
int values[5] = {0}; //change here and below to change number of samples for AP sensor.
int ind = 0;
int mode = 0;

float gy = 0;
float ap = 0;
float fingP1 = 0;
float fingP2 = 0;
float fingP3 = 0;
float butt1 = 0;
float butt2 = 0;
float pgy = 0;
float pap = 0;
float pfingP1 = 0;
float pfingP2 = 0;
float pfingP3 = 0;
float pbutt1 = 0;
float pbutt2 = 0;

int batteryLevel = 0; // for checking battery level
float voltage = 0;
float battPerc = 0;
auto timer = timer_create_default();
// timer used to check battery

BluetoothSerial SerialBT;
MPU6050 mpu;

void setup() {
  // set pins to input or output
  pinMode(button1, INPUT);
  pinMode(button2, INPUT);
  pinMode(indexPress, INPUT);
  pinMode(middlePress, INPUT);
  pinMode(ringPress, INPUT);
  pinMode(ledPin, OUTPUT);
  pinMode(pwrLed, OUTPUT);
  pinMode(modePin, INPUT);

  mode = digitalRead(modePin);
  if(mode == 1){
    // set to wireless mode
    // use SerialBT.print() for transmitting.
  } else {
    // set to wired mode.
    // use Serial.print() for transmitting
  }


  // begins bluetooth communication
  // add dip switch to change between wired and wireless modes.
  SerialBT.begin("AugmentedWWSensor"); // Devce name that will appear on computer
  delay(1000); // delays one second before beginning serial communication

  Serial.begin(115200);

  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  // begins timer to check battery level every 5 minutes
  timer.every(300000, checkBattery); //time in ms and callback

  // power on light sequence. last thing in setup.
  startLights();
}

void loop() {
 
  gy = mpu_read(); // gyro+accel
  delay(2); // small delay between each sensor reading.
  ap = airPressureRead();
  delay(2);
  fingP1 = fingerPressureRead1(); // 3 FSRs.
  fingP2 = fingerPressureRead2();
  fingP3 = fingerPressureRead3();
  delay(2);
  butt1 = buttonRead1();
  butt2 = buttonRead2();
  delay(2);

  
 pgy = gy; // stores values from previous loop. used to compare with current values. make into seperate function or leave in loop?
 pap = ap;
 pfingP1 = fingP1;
 pfingP2 = fingP2;
 pfingP3 = fingP3;
 pbutt1 = butt1;
 pbutt2 = butt2;


  
  // check value is appropriate before send.

  // bluetooth communication from the computer
  String inputFromOtherSide;
  if (SerialBT.available()) {
    inputFromOtherSide = SerialBT.readString();
    SerialBT.println("You had entered: ");
    SerialBT.println(inputFromOtherSide);
  }

  timer.tick(); // tick the timer every loop. calls battery check every 5 minutes.
}

void startLights() {
  // turns light off, blinks 2x, then remains on
  digitalWrite(ledPin, LOW);
  digitalWrite(pwrLed, LOW);
  delay(500);
  digitalWrite(ledPin, HIGH);
  digitalWrite(pwrLed, HIGH);
  delay(500);
  digitalWrite(ledPin, LOW);
  delay(500);
  digitalWrite(ledPin, HIGH);
  delay(500);
  digitalWrite(ledPin, LOW);
  digitalWrite(pwrLed, LOW);
  delay(1000);
  digitalWrite(ledPin, HIGH);

  Serial.println("Ready to begin");
  SerialBT.println("Ready to begin"); // sends message when all setup has been completed.
}

bool checkBattery(void *) {
  batteryLevel = analogRead(A0); // double check which pin the electricity is coming in on
  voltage = batteryLevel * 5 / 4095 // double check. this is a 12 bit board so I am using 12 bits here.
  battPerc = map(voltage, 3.6, 4.2, 0, 100);
  
  Serial.print("Battery at approx. ");
  Serial.print(battPer);
  Serial.print("%.");

  SerialBT.print("Battery at approx. ");
  SerialBT.print(battPer);
  SerialBT.print("%.");

  if(battPerc < 30.0){
    Serial.print("Battery is low, please charge before next performance.");
    SerialBT.print("Battery is low, please charge before next performance.");
    digitalWrite(pwrLed, HIGH);
  } else {
    digitalWrite(pwrLed, LOW);
  }

  return true; // keep timer active? true
}


void mpu_read() { // gets accel and gyro data. transmits it
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true); // request a total of 14 registers
  AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  GyX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)


  // scaling and truncating
  // don't use map();
  for (int i = 0; i < 6; i ++) {
    if (i == 0) {
      valueMapping(AcX);
    } else if (i == 1) {
      valueMapping(AcY);
    } else if (i == 2) {
      valueMapping(AcZ);
    } else if (i == 3) {
      valueMapping(GyX);
    } else if (i == 4) {
      valueMapping(GyY);
    } else if (i == 5) {
      valueMapping(GyZ);
    }
  }

    if((pgy * pgy) - (gy * gy) > 1 ){ // double check all ranges of sensors. replace 1 with appropriate value
  trans("gyro", gy);
  } // set up accel. currently is part of same function as gyro. monitor and send values seperately.


  // transmit accel values
  //  SerialBT.print("ax ");
  //  SerialBT.println(AcX);
  //  SerialBT.print("az ");
  //  SerialBT.println(AcY);
  //  SerialBT.print("az ");
  //  SerialBT.println(AcZ);
  //
  //  // transmits gyro values
  //  SerialBT.print("gx ");
  //  SerialBT.println(GyX);
  //  SerialBT.print("gy ");
  //  SerialBT.println(GyY);
  //  SerialBT.print("gz ");
  //  SerialBT.println(GyZ);

  // transmits wire serial for testing/ out of redundency
  // datta always transmits
  // transmit accel values
  //  Serial.print("ax ");
  //  Serial.println(AcX);
  //  Serial.print("az ");
  //  Serial.println(AcY);
  //  Serial.print("az ");
  //  Serial.println(AcZ);
  //
  //
  //  // transmits gyro values
  //  Serial.print("gx ");
  //  Serial.println(GyX);
  //  Serial.print("gy ");
  //  Serial.println(GyY);
  //  Serial.print("gz ");
  //  Serial.println(GyZ);

}

void airPressureRead() { //adjust this function to match current settings
  values[ind] = MPS20N0040D.read();
  ind++;
  if (ind >= 5) { // change "5 in this function if changed above
    ind = 0;
  }
   airP = 0;
  for (int i = 0; i < 5; i++) {
    airP += values[i];
    airP / 5;
  }

  // SerialBT.print("p ");
  // SerialBT.println(airP);
  // Serial.print("p ");
  // Serial.println(airP);

  void mapped = valueMapping(airP);

   if((pap * pap) - (ap * ap) > 1 ){ 
  trans("airPressure", ap);
  }
  //return mapped;

  //  float avg_val = 0.0; // variable for averaging
  //  for (int ii = 0; ii < avg_size; ii++) {
  //    avg_val += MPS20N0040D.read(); // add multiple ADC readings
  //    delay(50); // delay between readings
  //  }
  //  avg_val /= avg_size;
  //  //avgVal = map(avg_val, 800000.0, 17000000.0, 0.0, 1.0); // scale here or in max?
  //  // data always transmits
  //  SerialBT.print("p ");
  //  SerialBT.println(avg_val, 0); // print out the average // currently sending unscaled value
  //
  //  // transmits wire serial for testing/ out of redundency
  //  Serial.print("p ");
  //  Serial.println(avg_val, 0); // print out the average
}

void buttonRead1() {
  button1State = digitalRead(button1);
  
  // only transmits values when valuese are present. helps to eliminate data clutter/floating values
  // Values default to 0 when not conneceted through pull-down resistor.

  float mapped = valueMapping(button1State);

  if((pbutt1 * pbutt1) - (butt1 * butt1) > 0.5 ){ 
  trans("b1", butt1); 
  }
  //return mapped;
}

void buttonRead2() {
 
  button2State = digitalRead(button2);
  float mapped = valueMapping(button2State);

  if((pbutt2 * pbutt2) - (butt2 * butt2) > 0.5 ){ 
  trans("b2", butt2); 
  }
  //return mapped;
}


void fingerPressureRead1() { 
  indexPressVal = analogRead(indexPress);
  float mapped = valueMapping(indexPress);

   if((pfingP1 * pfingP1) - (fingP1 * fingP1) > 1 ){ 
  trans("index", fingP1);
  }
  //return mapped;
}

void fingerPressureRead2() { 
  middlePressVal = analogRead(middlePress);
  float mapped = valueMapping(middlePress);

  if((pfingP2 * pfingP2) - (fingP2 * fingP2) > 1 ){ 
  trans("middle", fingP2);
  }
  //return mapped;
}

void fingerPressureRead3() { 
  ringPressVal = analogRead(ringPress);
  float mapped = valueMapping(ringPress);

  if((pfingP3 * pfingP3) - (fingP3 * fingP3) > 1 ){ 
  trans("ring", fingP3);
  }
  //return mapped;
}

char* valueMapping(int value) { // change char* to void if not returning values
  float mapped = value / 4095; // 12 bit board.
  char sendValue[7];
  sprintf(sendValue, "%.5f", mapped);  // might need "%.5d" as second argument. current should work.
  puts(sendValue);
  Serial.print(sendValue);
  return sendValue;
}

void trans(string label, float transmit){ // transmits a label and the data. make sure labels match in max patches.
SerialBT.print(label);
SerialBT.print(" ");
SerialBT.println(transmit);

Serial.print(label);
Serial.print(" ");
Serial.println(transmit);
}

