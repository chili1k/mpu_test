#include <avr/wdt.h>
#include <Bounce2.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#define INTERRUPT_PIN 2
#define PIN_BUTTON_1 3
#define PIN_BUTTON_2 4
#define PRINT_DELAY 1000
#define INTERRUPT_TIMEOUT 2000

#define PRINT_CSV_ITEM(x) Serial.print(x); Serial.print(",");
#define PRINT_LAST_CSV_ITEM(x) Serial.print(x); Serial.println();


MPU6050 mpu;
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
VectorFloat gravity;    // [x, y, z]            gravity vector
Quaternion q;           // [w, x, y, z]         quaternion container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

Bounce button1;
Bounce button2;

unsigned long lastUpdateTime = 0;
unsigned long time = 0;

bool wasButton1Pushed;
bool wasButton2Pushed;

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
volatile unsigned long lastInterruptTime;
void dmpDataReady() {
    mpuInterrupt = true;
    lastInterruptTime = millis();
}

void setup() {
  wdt_enable(WDTO_4S);

  pinMode(PIN_BUTTON_1, INPUT);
  button1.attach(PIN_BUTTON_1);
  button1.interval(20);

  pinMode(PIN_BUTTON_2, INPUT);
  button2.attach(PIN_BUTTON_2);
  button2.interval(20);


  Wire.begin();
  Wire.setClock(100000); 
  mpu.initialize();

  Serial.begin(9600);
  Serial.println(F("Initializing I2C devices..."));
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  //mpu.setXAccelOffset(-4816);
  //mpu.setYAccelOffset(-8075);
  //mpu.setZAccelOffset(-20331);
  //mpu.setXGyroOffset(-580);
  //mpu.setYGyroOffset(-37);
  //mpu.setZGyroOffset(-81);

//  mpu.setXGyroOffset(220);
//  mpu.setYGyroOffset(76);
//  mpu.setZGyroOffset(-85);
//  mpu.setZAccelOffset(1688);
  
  if (devStatus == 0) {
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    Serial.println(F("DMP ready!"));
    dmpReady = true;

    packetSize = mpu.dmpGetFIFOPacketSize();
    Serial.print(F("FIFO packet size: "));
    Serial.println(packetSize);
  } else {
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  lastInterruptTime = millis();

  // csv header
  Serial.println("Tm,Btn1,Btn2,yaw,pitch,roll,qw,qx,qy,qz");
}

void printGyro() {
  if ((millis() - lastInterruptTime) >= INTERRUPT_TIMEOUT) {
    Serial.println(F("Error. No data from i2c seen for 2 seconds."));
    return;
  }

  int btn1 = wasButton1Pushed ? 1 : 0;
  int btn2 = wasButton2Pushed ? 1 : 0;
  PRINT_CSV_ITEM(time);
  PRINT_CSV_ITEM(btn1);
  PRINT_CSV_ITEM(btn2);
  PRINT_CSV_ITEM(ypr[0]);
  PRINT_CSV_ITEM(ypr[1]);
  PRINT_CSV_ITEM(ypr[2]);
  PRINT_CSV_ITEM(q.w);
  PRINT_CSV_ITEM(q.x);
  PRINT_CSV_ITEM(q.y);
  PRINT_LAST_CSV_ITEM(q.z);
}

void readGyro() {
  wdt_reset();
  
  if (!dmpReady) return;

  // no data yet
  if (!mpuInterrupt) {
    return;
  }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  fifoCount = mpu.getFIFOCount();

  if (fifoCount == 0) {
    Serial.println(F("FIFO count 0"));
  } else if (fifoCount == 1024) {
    Serial.println(F("FIFO overflow"));
    mpu.resetFIFO();
    Serial.println(F("FIFO reset"));
  } else { 
    if (fifoCount % packetSize != 0) {
      Serial.print(F("Packet is corrupted. Fifo count: "));
      Serial.println(fifoCount);
      mpu.resetFIFO();
    } else {
      //Serial.print(F("Reading packets. Fifo count: "));
      //Serial.println(fifoCount);
      while (fifoCount >= packetSize) {
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;
      }


      //Serial.println(F("Get quaternion"));
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      //Serial.println(F("Get gravity"));
      mpu.dmpGetGravity(&gravity, &q);
      //Serial.println(F("Get ypr"));
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    }
  }
}

void loop() {
  readGyro();

  button1.update();
  button2.update();
  // if button was pressed at any point during the last interval
  wasButton1Pushed = wasButton1Pushed || button1.read();
  wasButton2Pushed = wasButton2Pushed || button2.read();

  unsigned long now = millis();

  if ((now - lastUpdateTime) >= PRINT_DELAY) {
    printGyro();

    lastUpdateTime = now;
    wasButton1Pushed = false;
    wasButton2Pushed = false;
    time++;
  }
}
