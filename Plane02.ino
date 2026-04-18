#include <SPI.h>
#include <RF24.h>
#include <nRF24L01.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <I2Cdev.h>
#include <Wire.h>

#define CERX 12
#define CSNRX 11
#define CETX 8
#define CSNTX 7
#define INTERRUPTMPU6050_PIN 2

// โครงสร้างข้อมูล (24 Bytes)
struct __attribute__((packed)) mpuData {
  float yaw   = 0.0f;
  float pitch = 0.0f;
  float roll  = 0.0f;
  float gx    = 0.0f;
  float gy    = 0.0f;
  float gz    = 0.0f;
};
mpuData mpuDataToSent;

struct __attribute__((packed)) joystickData {
  uint8_t thrust = 0;
  uint8_t yaw    = 0;
  uint8_t pitch  = 0;
  uint8_t roll   = 0;
};
joystickData joystickReceive;

// DMP mpu6050
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];
MPU6050 mpu;

// nRF24
RF24 radioRX(CERX, CSNRX);
RF24 radioTX(CETX, CSNTX);
const uint8_t pipeAddress[6] = "PLANE"; 

uint32_t debugTimer = 0;
void debug(bool txOk, bool rxOk) {
  Serial.print(">>> TX: ");
  if (txOk) {
    Serial.print("SUCCESS [Gx:"); Serial.print(mpuDataToSent.gx, 3);
    Serial.print(" GY:"); Serial.print(mpuDataToSent.gy, 3);
    Serial.print(" GZ:"); Serial.print(mpuDataToSent.gz, 3);
    Serial.print(" Y:"); Serial.print(mpuDataToSent.yaw, 3);
    Serial.print(" P:"); Serial.print(mpuDataToSent.pitch, 3);
    Serial.print(" R:"); Serial.print(mpuDataToSent.roll, 3);
    Serial.print("]");
  } else {
    Serial.print("FAILED ");
  }

  Serial.print(" | <<< RX: ");
  if (rxOk) {
    Serial.print("RECEIVED [T:"); Serial.print(joystickReceive.thrust);
    Serial.print(" Y:"); Serial.print(joystickReceive.yaw);
    Serial.print(" P:"); Serial.print(joystickReceive.pitch);
    Serial.print(" R:"); Serial.print(joystickReceive.roll);
    Serial.println("]");
  } else {
    Serial.println("EMPTY");
  }
}

volatile bool mpuInterruptFlag = false;
void dmpDataReady() {
  mpuInterruptFlag = true;
}

void setup() {
  pinMode(INTERRUPTMPU6050_PIN, INPUT);
  Serial.begin(115200); 
  delay(500);
  
  Wire.begin();
  Wire.setClock(400000); // 400kHz 

  Serial.println(F("Initializing MPU6050..."));
  mpu.initialize();

  if (mpu.dmpInitialize() == 0) {
    // ใส่ค่า Offset 
    mpu.setXAccelOffset(0); mpu.setYAccelOffset(0); mpu.setZAccelOffset(0);
    mpu.setXGyroOffset(0);  mpu.setYGyroOffset(0);  mpu.setZGyroOffset(0);  
    
    mpu.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(INTERRUPTMPU6050_PIN), dmpDataReady, RISING);
  } else {
    Serial.println(F("DMP Initialization failed."));
  }
  
  // ตั้งค่า nRF24 
  digitalWrite(CSNRX, HIGH); digitalWrite(CERX, LOW);
  digitalWrite(CSNTX, HIGH); digitalWrite(CETX, LOW);

  if (radioRX.begin()){
    radioRX.openReadingPipe(1, pipeAddress);
    radioRX.setPALevel(RF24_PA_MAX);
    radioRX.setDataRate(RF24_250KBPS);
    radioRX.setChannel(120);
    radioRX.setAutoAck(false); 
    radioRX.startListening();
  }else{ Serial.println("No RX"); }

  if (radioTX.begin()){
    radioTX.openWritingPipe(pipeAddress);
    radioTX.setPALevel(RF24_PA_MAX);
    radioTX.setDataRate(RF24_250KBPS);
    radioTX.setChannel(120);
    radioTX.setAutoAck(false); 
    radioTX.stopListening();
  }else{ Serial.println("No TX"); }
}

void loop() {
  // ตรวจสอบ Interrupt จาก MPU6050
  bool lastTXStatus = false;
  bool lastRXStatus = false;

  if (mpuInterruptFlag) {
    mpuInterruptFlag = false;
    
    // ดึงข้อมูลชุดล่าสุด
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { 
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

      // เก็บใส่ Struct
      mpuDataToSent.yaw = ypr[0] * 180 / M_PI;
      mpuDataToSent.pitch = ypr[1] * 180 / M_PI;
      mpuDataToSent.roll = ypr[2] * 180 / M_PI;
      mpuDataToSent.gx = gravity.x;
      mpuDataToSent.gy = gravity.y;
      mpuDataToSent.gz = gravity.z;

      // ส่งข้อมูลทันที (ใช้เวลาประมาณ 1-2ms ที่ 250KBPS)
      lastTXStatus = radioTX.write(&mpuDataToSent, sizeof(mpuDataToSent));
    }
  }

  // ฝั่งรับ: ถ้ามีคำสั่งจากรีโมทเข้ามา
  if (radioRX.available()) {
    radioRX.read(&joystickReceive, sizeof(joystickReceive));
    lastRXStatus = true;
  }

  // เรียกใช้ Debug
  // if (millis() - debugTimer > 100) { // Print ทุกๆ 100ms
  //   debug(lastTXStatus, lastRXStatus);
  //   debugTimer = millis();
  // }
}