#include <SPI.h>
#include <RF24.h>
#include <nRF24L01.h>

const uint8_t CERX  = 12;
const uint8_t CSNRX = 11;
const uint8_t CETX  = 8;
const uint8_t CSNTX = 7;

const uint8_t THRUST_PIN = A0;
const uint8_t YAW_PIN    = A1;
const uint8_t PITCH_PIN  = A2;
const uint8_t ROLL_PIN   = A3;

struct __attribute__((packed)) joystickData {
  uint16_t thrust = 0;
  uint16_t yaw    = 0;
  uint16_t pitch  = 0;
  uint16_t roll   = 0;
};

joystickData joystick;

const uint16_t joystickSampleTime = 10; 
uint32_t joystickSampleTimer = 0;
uint32_t joystickSampleCount = 0;

uint32_t thrustSum = 0;
uint32_t yawSum    = 0;
uint32_t pitchSum  = 0;
uint32_t rollSum   = 0;

RF24 radioRX(CERX, CSNRX);
RF24 radioTX(CETX, CSNTX);
const uint8_t pipeAddress[6] = "PLANE";

void setup() {
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
      radioTX.write(&joystick, sizeof(joystick));
    }

    //Reset
    thrustSum = 0;
    yawSum    = 0;
    pitchSum  = 0;
    rollSum   = 0;
    joystickSampleCount = 0;
    joystickSampleTimer = currentMillis; 
  }
}