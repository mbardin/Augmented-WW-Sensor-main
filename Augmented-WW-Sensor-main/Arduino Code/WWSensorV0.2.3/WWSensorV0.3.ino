
// Augmented Woodwind Sensor v 0.6.9
// code by Matthew A. Bardin
// this version Dec 10 2021

//libraries used:
//#include <Q2HX711.h> //for old air sensor. replace it with the new one
#include <Wire.h>
#include <MPU6050.h> //gyro library
#include <BluetoothSerial.h>
#include <arduino-timer.h>
#include <LibPrintf.h>
#include <I2S.h> //microphone library
#include <sdpsensor.h> //library for airflow sensor. set for ESP32 board.
// #include <SparkFun_SDP3x_Arduino_Library.h> //backup airflow sensor library

// include libraries in software download for user access.

const int MPU_addr = 0x68; // I2C address of the MPU-6050
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
//int avg_size = 10; // #pts to average over

//pins used:
const int button1 = 13;
const int button2 = 12;
const int ledPin = 2;      // power on indicator
const int pwrLed = 4;      //low battery indicator light
const int indexPress = 33; // FSR's
const int middlePress = 32;
const int ringPress = 35;
const int micPin = 27; //microphone pin. placeholder value. update with actual pin number.
const byte MPS_OUT_pin = 25; // OUT data pin
const byte MPS_SCK_pin = 18; // clock data pin
const int senseCheckPin = 25;
const int modePin = 14; //pin for DIP switch. will change between wireless or wired. adjust value once switch is connected.

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
int mode = 1; //defaults to wired version for testing. make into wireless for final. make result of DIP switch

float gy = 0; //figure out which of these I can remove once the code is working.
float acc = 0;
float accel = 0;
float ap = 0;
float fingP1 = 0;
float fingP2 = 0;
float fingP3 = 0;
float butt1 = 0;
float butt2 = 0;
float pgy = 0;
float pacc = 0;
float pap = 0;
float pfingP1 = 0;
float pfingP2 = 0;
float pfingP3 = 0;
float pbutt1 = 0;
float pbutt2 = 0;
float air = 0;
float gyScope = 0;
float acScope = 0;
float mic = 0;
float pmic = 0;
float micSample = 0;


int batteryLevel = 0; // for checking battery level
float voltage = 0;
float battPerc = 0;
auto timer = timer_create_default();
// timer used to check battery

BluetoothSerial SerialBT;
MPU6050 mpu;

void setup()
{
  // set pins to input or output
  pinMode(button1, INPUT);
  pinMode(button2, INPUT);
  pinMode(indexPress, INPUT);
  pinMode(middlePress, INPUT);
  pinMode(ringPress, INPUT);
  pinMode(micPin, INPUT);
  pinMode(ledPin, OUTPUT);
  pinMode(pwrLed, OUTPUT);
  pinMode(modePin, INPUT);
  //mode = digitalRead(modePin); // uncomment to enable wireless mode

  // begins bluetooth communication
  // add dip switch to change between wired and wireless modes.
  SerialBT.begin("AugmentedWWSensor"); // Device name that will appear on computer. Give an updated name when done.
  delay(1000);                         // delays one second before beginning serial communication

  Serial.begin(115200);

  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0);    // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  // begins timer to check battery level every 5 minutes
  timer.every(300000, checkBattery); //time in ms and callback

  // power on light sequence. last thing in setup.
  startLights();
}

void loop()
{
  // read from each sensor. setup gyro/accel and ap.
  gy = sense("gyro "); // gyro+accel
  accel = sense("accel ");
  ap = sense("air ");
  fingP1 = sense("index ");
  fingP2 = sense("middle ");
  fingP3 = sense("ring ");
  butt1 = sense("button1 ");
  butt2 = sense("button2 ");
  mic = sense("mic");

  // check if each reading is above threshold to send. helps to minimize noise and latency
  check("gyro ", pgy, gy, 10);
  check("accel ", pacc, acc, 10);
  check("air ", pap, ap, 100);
  check("index ", pfingP1, fingP1, 10);
  check("middle ", pfingP2, fingP2, 10);
  check("ring ", pfingP3, fingP3, 10);
  check("button1 ", pbutt1, butt1, 0.5);
  check("button2 ", pbutt2, butt2, 0.5);
  check("mic", 0, 1, 0.5); // adjust for values when testing microphone sensitivity. make into variables.

  pgy = gy; // stores values from previous loop. used to compare with current values in check()
  pap = ap;
  pmic = mic;
  pfingP1 = fingP1;
  pfingP2 = fingP2;
  pfingP3 = fingP3;
  pbutt1 = butt1;
  pbutt2 = butt2;

  // bluetooth communication from the computer
  String inputFromOtherSide;
  if (SerialBT.available())
  {
    inputFromOtherSide = SerialBT.readString();
    SerialBT.println("You had entered: ");
    SerialBT.println(inputFromOtherSide);
  }

  timer.tick(); // tick the timer every loop. calls battery check every 5 minutes.
}

void startLights()
{
  // turns light off, blinks 2x, then remains on. power led only on then off until low power below 30%
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

bool checkBattery(void *)
{
  batteryLevel = analogRead(A0);    // double check which pin the electricity is coming in on
  voltage = batteryLevel * 5 / 4095 // double check. this is a 12 bit board so I am using 12 bits here.
            battPerc = map(voltage, 3.6, 4.2, 0, 100);

  if (mode == 1)
  {
    // set to wireless mode
    SerialBT.print("Battery at approx. ");
    SerialBT.print(battPer);
    SerialBT.print("%.");

    if (battPerc < 30.0)
    {
      SerialBT.print("Battery is low, please charge before next performance.");
      digitalWrite(pwrLed, HIGH);
    }
    else
    {
      digitalWrite(pwrLed, LOW);
    }
  }
  else
  {
    // set to wired mode.
    Serial.print("Battery at approx. ");
    Serial.print(battPer);
    Serial.print("%.");

    if (battPerc < 30.0)
    {
      Serial.print("Battery is low, please charge before next performance.");
    }
    else
    {
      digitalWrite(pwrLed, LOW);
    }
  }

  return true; // keep timer active? true
}

float mpu_read(char label)
{ // gets accel and gyro data. transmits it
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true); // request a total of 14 registers
  AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  GyX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  if (label == "accel ")
  {
    float a = [ AcX, AcY, AcZ ]; //currently sends array. Will need to monitor and send all values seperately.
  }
  else if (label == "gyro ")
  {
    float a = [ GyX, GyY, GyZ ];
  }
  return a;
}

float airPressureRead()
{ //adjust this function to match current settings
  values[ind] = MPS20N0040D.read();
  ind++;
  if (ind >= 5)
  { // change "5 in this function if changed above
    ind = 0;
  }
  airP = 0;
  for (int i = 0; i < 5; i++)
  {
    airP += values[i];
    airP / 5;
  }

  return airP;

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
  //  // transmits wire serial for testing/ out of redundancy
  //  Serial.print("p ");
  //  Serial.println(avg_val, 0); // print out the average
}

float sense(char sensor)
{

  if (sensor == "index ")
  {
    indexPressVal = analogRead(indexPress);
    float mapped = valueMapping(indexPress);
  }
  else if (sensor == "middle ")
  {
    middlePressVal = analogRead(middlePress);
    float mapped = valueMapping(middlePress);
  }
  else if (sensor == "ring ")
  {
    ringPressVal = analogRead(ringPress);
    float mapped = valueMapping(ringPress);
  }
  else if (sensor == "button1 ")
  {
    button1State = digitalRead(button1);
    float mapped = valueMapping(button1State);
  }
  else if (sensor == "button2 ")
  {
    button2State = digitalRead(button2);
    float mapped = valueMapping(button2State);
  }
  else if (sensor == "gyro ")
  {
    gyScope = mpu_read("gyro "); //this one returns an array.
    float mapped = valueMapping(gyScope);
  }
  else if (sensor == "accel")
  {
    acScope = mpu_read("accel "); //this one returns an array.
    float mapped = valueMapping(acScope);
  }
  else if (sensor == "air ")
  {
    air = airPressureRead();
    float mapped = valueMapping(airP);
  }
  else if (sensor == "mic"){
    micSample = I2S.read();
    float mapped = valueMapping(micSample);
  }
  else
  {
    if (mode == 1)
    {
      // set to wireless mode
      SerialBT.println("Err: Not a valid sensor designation.");
    }
    else
    {
      // set to wired mode.
      Serial.println("Err: Not a valid sensor designation.");
    }
  }

  return mapped;
}

char *valueMapping(int value)
{ // change char* to void if not returning values
  value.foreach() = char
  { //check formatting. I want this to happen for each value sent in because of arrays.
    float mapped = value / 4095;
    char sendValue[7];
    sprintf(sendValue, "%.5f", mapped); // might need "%.5d" as second argument. current should work.
    puts(sendValue);
    //Serial.print(sendValue);
    return sendValue;
  }
}

void check(char label, float value1, float value2, float trigger) //label, previous, current, range
{
  if ((value2 * value2) - (value1 * value1) >= trigger) //only sends if value is large enough
  {
    trans(label, value2);
  }
}

void trans(char label, float transmit)
{ // transmits a label and the data. make sure labels match in max patches.
  if (mode == 1)
  {
    // wireless mode
    SerialBT.print(label);
    SerialBT.println(transmit);
  }
  else
  {
    // wired mode.
    Serial.print(label);
    Serial.println(transmit);
  }
}
