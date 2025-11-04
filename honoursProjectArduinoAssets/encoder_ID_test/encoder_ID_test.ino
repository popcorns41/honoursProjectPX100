#include <Dynamixel2Arduino.h>
#define DXL_SERIAL Serial1
#define DXL_DIR_PIN 2
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

void setup() {
  Serial.begin(115200);
  dxl.begin(57600);
  dxl.setPortProtocolVersion(2.0);
  for(int id=1; id<=4; id++) dxl.torqueOff(id);
}

void loop() {
  for(int id=1; id<=4; id++) {
    int32_t pos = dxl.getPresentPosition(id);
    float radians = fmod((pos / 4096.0f) * 2.0f * PI, 2.0f*PI);
    Serial.print(radians, 6);
    if(id<4) Serial.print(',');
  }
  Serial.println();
  delay(10);
}