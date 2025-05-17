#include <SPI.h>
#include <vector>

static const uint8_t TSOP_PL_PIN = 8;

void initTSOPs() {
  pinMode(TSOP_PL_PIN, OUTPUT);
  digitalWrite(TSOP_PL_PIN, HIGH);

  SPI.begin();
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
}

std::vector<bool> readTSOPs() {
  std::vector<bool> sensors(24);

  digitalWrite(TSOP_PL_PIN, LOW);
  delayMicroseconds(1);
  digitalWrite(TSOP_PL_PIN, HIGH);

  uint8_t data[3];
  for (int i = 0; i < 3; ++i) {
    data[i] = SPI.transfer(0x00);
  }

  for (int byteIdx = 0; byteIdx < 3; ++byteIdx) {
    for (int bit = 0; bit < 8; ++bit) {
      bool isActive = !(data[byteIdx] & (1 << (7 - bit)));
      sensors[byteIdx * 8 + bit] = isActive;
    }
  }

  return sensors;
}
