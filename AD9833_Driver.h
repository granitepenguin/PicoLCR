// AD9833_Driver.h
//Runs on PICOLCR via SPI 
#pragma once
#include <Arduino.h>
#include <SPI.h>

class AD9833_Driver {
public:
  explicit AD9833_Driver(uint8_t fsyncPin, uint32_t mclkHz = 25000000UL)
    : _fsync(fsyncPin), _mclk(mclkHz) {}

  // Initialize SPI, reset the chip, and start a continuous sine at startHz.
  bool begin(uint32_t startHz = 1000) {
    pinMode(_fsync, OUTPUT);
    digitalWrite(_fsync, HIGH);
    SPI.begin();

    // Assert RESET while loading registers; B28 enables 28-bit FREQ writes
    // as two consecutive 14-bit halves.
    writeReg(CTRL_REG | CTRL_B28 | CTRL_RESET);
    setFrequencyHz(startHz);
    writeReg(PHASE0_REG | 0x0000);
    writeReg(CTRL_REG | CTRL_B28);  // release RESET, sine resumes
    return true;
  }

  // Change the output frequency at runtime without resetting the DDS.
  // Both 14-bit halves of FREQ0 are written while B28 remains enabled,
  // allowing continuous operation during normal frequency changes.
  void setFrequencyHz(uint32_t hz) {
    uint64_t freq_word = ((uint64_t)hz << 28) / _mclk;
    uint16_t lsb14 = (uint16_t)(freq_word & 0x3FFF);
    uint16_t msb14 = (uint16_t)((freq_word >> 14) & 0x3FFF);

    writeReg(FREQ0_W | lsb14);
    writeReg(FREQ0_W | msb14);
  }

  void stop() {
    writeReg(CTRL_REG | CTRL_B28 | CTRL_SLEEP1 | CTRL_SLEEP12);
  }

private:
  static constexpr uint16_t CTRL_REG    = 0x0000;
  static constexpr uint16_t FREQ0_W     = 0x4000;
  static constexpr uint16_t PHASE0_REG  = 0xC000;
  static constexpr uint16_t CTRL_B28     = 1 << 13;
  static constexpr uint16_t CTRL_RESET   = 1 << 8;
  static constexpr uint16_t CTRL_SLEEP1  = 1 << 7;
  static constexpr uint16_t CTRL_SLEEP12 = 1 << 6;

  void writeReg(uint16_t word) {
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE2));
    digitalWrite(_fsync, LOW);
    SPI.transfer((uint8_t)(word >> 8));
    SPI.transfer((uint8_t)(word & 0xFF));
    digitalWrite(_fsync, HIGH);
    SPI.endTransaction();
  }

  uint8_t  _fsync;
  uint32_t _mclk;
};
