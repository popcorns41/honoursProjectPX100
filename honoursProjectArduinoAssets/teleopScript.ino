#include <Dynamixel2Arduino.h>
#define DXL_SERIAL  Serial1
#define DXL_DIR_PIN 2
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

const float TICKS_PER_REV = 4096.0f;

void setup() {
  Serial.begin(115200);        // USB → host
  dxl.begin(57600);
  dxl.setPortProtocolVersion(2.0);
  for (int id = 1; id <= 4; id++) dxl.torqueOff(id);
}

void loop() {
  for (int id = 1; id <= 4; id++) {
    int32_t raw = dxl.getPresentPosition(id);
    int32_t wrapped = raw % (int32_t)TICKS_PER_REV;
    if (wrapped < 0) wrapped += (int32_t)TICKS_PER_REV;
    float rad = (wrapped / TICKS_PER_REV) * TWO_PI;
    Serial.print(rad, 6);
    if (id < 4) Serial.print(',');
  }
  Serial.println();
  delay(10);   // ~100 Hz
}