#include <Dynamixel2Arduino.h>
#define DXL_SERIAL Serial1
#define DXL_DIR_PIN 2
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

void setup() {
  Serial.begin(115200);
  dxl.begin(57600);
  dxl.setPortProtocolVersion(2.0);
  dxl.torqueOff(1);
  // change this motor's ID to X
  dxl.setID(1, 4);
}
void loop() {}