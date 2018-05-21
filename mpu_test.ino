#include<Wire.h>
#include <Bounce2.h>

#define MPU_ADDR 0x68  // I2C address of the MPU-6050
#define MAX_ITERS 100
#define PIN_BUTTON_1 3
#define PIN_BUTTON_2 4
#define PRINT_DELAY 1000

#define PRINT_CSV_ITEM(x) Serial.print(x); Serial.print(",");
#define PRINT_LAST_CSV_ITEM(x) Serial.print(x); Serial.println();

int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
Bounce button1;
Bounce button2;

unsigned long lastUpdateTime = 0;
unsigned long time = 0;

bool wasButton1Pushed;
bool wasButton2Pushed;

void setup(){
  Wire.begin();
  Wire.setClock(100000);
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.begin(115200);

  pinMode(PIN_BUTTON_1, INPUT);
  button1.attach(PIN_BUTTON_1);
  button1.interval(20);

  pinMode(PIN_BUTTON_2, INPUT);
  button2.attach(PIN_BUTTON_2);
  button2.interval(20);

  // csv header
  Serial.println("Tm,Btn1,Btn2,AcX,AcY,AcZ,GyX,GyY,GyZ");
}

void printCsvItem(bool isLastItem) {
  Serial.print(AcX);
  Serial.print(",");
}

void printLastCsvItem() {
  Serial.print(AcX);
  Serial.println();
}

void printGyro() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR,14,true);  // request a total of 14 registers

  AcX=AcY=AcZ=Tmp=GyX=GyY=GyZ=0;
  for (int i = 0; i < MAX_ITERS; i++) {
    AcX+=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
    AcY+=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    AcZ+=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    Tmp+=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    GyX+=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    GyY+=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    GyZ+=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  }

  AcX/=MAX_ITERS;
  AcY/=MAX_ITERS;
  AcZ/=MAX_ITERS;
  Tmp/=MAX_ITERS;
  GyX/=MAX_ITERS;
  GyY/=MAX_ITERS;
  GyZ/=MAX_ITERS;

  int btn1 = wasButton1Pushed ? 1 : 0;
  int btn2 = wasButton2Pushed ? 1 : 0;

  PRINT_CSV_ITEM(time);
  PRINT_CSV_ITEM(btn1);
  PRINT_CSV_ITEM(btn2);
  PRINT_CSV_ITEM(AcX);
  PRINT_CSV_ITEM(AcY);
  PRINT_CSV_ITEM(AcZ);
  //PRINT_CSV_ITEM(Tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet
  PRINT_CSV_ITEM(GyX);
  PRINT_CSV_ITEM(GyY);
  PRINT_LAST_CSV_ITEM(GyZ);
}

void loop(){
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
