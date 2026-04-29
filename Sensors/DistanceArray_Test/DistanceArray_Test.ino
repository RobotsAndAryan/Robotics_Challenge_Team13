#include <Wire.h>
#include <SparkFun_VL53L5CX_Library.h>

SparkFun_VL53L5CX imager;
VL53L5CX_ResultsData data;

int res = 0;
int w = 0;

void setup() {
  delay(5000);
  Serial.begin(115200);
  Serial.println("start");

  Wire.begin();
  Wire.setClock(400000);

  if (!imager.begin()) {
    Serial.println("failed to find sensor");
    while (1);
  }

  imager.setResolution(8 * 8);

  res = imager.getResolution();
  w = sqrt(res);

  imager.setRangingFrequency(15);
  imager.startRanging();
}

void loop() {
  if (imager.isDataReady()) {
    if (imager.getRangingData(&data)) {
      for (int y = 0; y <= w * (w - 1); y += w) {
        for (int x = w - 1; x >= 0; x--) {
          Serial.print(data.distance_mm[x + y]);
          Serial.print(",");
        }
      }
      Serial.println();
    }
  }
  delay(5);
}