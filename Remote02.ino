#include <SPI.h>
#include <RF24.h>
#include <nRF24L01.h>

const uint8_t CERX  = 12;
const uint8_t CSNRX = 11;
const uint8_t CETX  = 8;
const uint8_t CSNTX = 7;

const uint8_t THRUST_PIN = 34;
const uint8_t YAW_PIN    = 35;
const uint8_t PITCH_PIN  = 32;
const uint8_t ROLL_PIN   = 33;

struct __attribute__((packed)) mpuData {
  float yaw   = 0.0f;
  float pitch = 0.0f;
  float roll  = 0.0f;
  float gx    = 0.0f;
  float gy    = 0.0f;
  float gz    = 0.0f;
};
mpuData mpuDataReceive;

struct __attribute__((packed)) joystickData {
  uint8_t thrust = 0;
  uint8_t yaw    = 0;
  uint8_t pitch  = 0;
  uint8_t roll   = 0;
};
joystickData joystick;

const uint16_t joystickSampleTime = 10; //ms
uint32_t joystickSampleTimer = 0;
uint32_t joystickSampleCount = 0;

uint32_t thrustSum = 0;
uint32_t yawSum    = 0;
uint32_t pitchSum  = 0;
uint32_t rollSum   = 0;

RF24 radioRX(CERX, CSNRX);
RF24 radioTX(CETX, CSNTX);
const uint8_t pipeAddress[6] = "PLANE";

uint32_t debugTimer = 0;
void debug(bool txOk, bool rxOk) {
  Serial.print(">>> TX: ");
  if (txOk) {
    Serial.print("SUCCESS [T:"); Serial.print(joystick.thrust);
    Serial.print(" Y:"); Serial.print(joystick.yaw);
    Serial.print(" P:"); Serial.print(joystick.pitch);
    Serial.print(" R:"); Serial.print(joystick.roll);
    Serial.print("]");
  } else {
    Serial.print("FAILED ");
  }

  Serial.print(" | <<< RX: ");
  if (rxOk) {
    Serial.print("RECEIVED [Y:"); Serial.print(mpuDataReceive.yaw, 1);
    Serial.print(" P:"); Serial.print(mpuDataReceive.pitch, 1);
    Serial.print(" R:"); Serial.print(mpuDataReceive.roll, 1);
    Serial.println("]");
  } else {
    Serial.println("EMPTY");
  }
}

void setup() {
  pinMode(THRUST_PIN, INPUT); pinMode(YAW_PIN, INPUT);
  pinMode(PITCH_PIN, INPUT); pinMode(ROLL_PIN, INPUT);
  analogReadResolution(8);

  Serial.begin(115200);
  delay(500);

  if(radioRX.begin()) {
    radioRX.openReadingPipe(1, pipeAddress);
    radioRX.setPALevel(RF24_PA_MAX);
    radioRX.setDataRate(RF24_250KBPS);
    radioRX.setChannel(120);
    radioRX.setAutoAck(false);
    radioRX.startListening();
    Serial.println("RX Ready");
  } else { Serial.println("No RX"); }

  if(radioTX.begin()) {
    radioTX.openWritingPipe(pipeAddress);
    radioTX.setPALevel(RF24_PA_MAX);
    radioTX.setDataRate(RF24_250KBPS);
    radioTX.setChannel(120);
    radioTX.setAutoAck(false);
    radioTX.stopListening();
    Serial.println("TX Ready");
  } else { Serial.println("No TX"); }
}

void loop() {
  uint32_t currentMillis = millis();
  bool lastTXStatus = false;
  bool lastRXStatus = false;

  if (currentMillis - joystickSampleTimer < joystickSampleTime) {
    // สะสมค่า
    thrustSum += analogRead(THRUST_PIN);
    yawSum    += analogRead(YAW_PIN);
    pitchSum  += analogRead(PITCH_PIN);
    rollSum   += analogRead(ROLL_PIN);
    joystickSampleCount++;
  } else {
    if (joystickSampleCount > 0) {
      joystick.thrust = (thrustSum / joystickSampleCount);
      joystick.yaw    = (yawSum / joystickSampleCount);
      joystick.pitch  = (pitchSum / joystickSampleCount);
      joystick.roll   = (rollSum / joystickSampleCount);

      // ส่งข้อมูล
      lastTXStatus = radioTX.write(&joystick, sizeof(joystick));
    }

    //Reset
    thrustSum = 0;
    yawSum    = 0;
    pitchSum  = 0;
    rollSum   = 0;
    joystickSampleCount = 0;
    joystickSampleTimer = currentMillis; 
  }

  //รับข้อมูล
  if(radioRX.available())
  {
    radioRX.read(&mpuDataReceive, sizeof(mpuDataReceive));
    lastRXStatus = true;
  }

  // เรียกใช้ Debug
  // if (millis() - debugTimer > 100) { // Print ทุกๆ 100ms
  //   debug(lastTXStatus, lastRXStatus);
  //   debugTimer = millis();
  // }

}