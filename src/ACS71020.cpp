#include "ACS71020.h"

ACS71020::ACS71020() {}

void ACS71020::begin(uint8_t address, TwoWire &wirePort) {
  _i2cAddress = address;
  _i2cPort = &wirePort;
}

ACS71020ERR ACS71020::unlock() {
  return writeRegister(0x2F, CUSTOMER_ACCESS_CODE);
}

ACS71020ERR ACS71020::readRegister(uint8_t address, uint32_t &data) {
  _i2cPort->beginTransmission(_i2cAddress);
  _i2cPort->write(address);
  uint8_t i2cResult = _i2cPort->endTransmission();

  if (i2cResult != 0)
    return (ERR_I2C_ERROR);

  uint8_t toRead = _i2cPort->requestFrom(_i2cAddress, (uint8_t)4);
  if (toRead != 4)
    return (ERR_I2C_ERROR);  

  data = Wire.read();
  data |= Wire.read() << 8;
  data |= Wire.read() << 16;
  data |= Wire.read() << 24;

  return SUCCESS;
}

ACS71020ERR ACS71020::writeRegister(uint8_t address, uint32_t data) {
  _i2cPort->beginTransmission(_i2cAddress);
  _i2cPort->write(address);

  Wire.write(data);
  Wire.write(data >> 8);
  Wire.write(data >> 16);
  Wire.write(data >> 24);
  uint8_t i2cResult = _i2cPort->endTransmission();

  if (i2cResult != 0)
    return (ERR_I2C_ERROR);

  return SUCCESS;
}

ACS71020ERR ACS71020::readRMS(float &voltage, float &current) {
  REGISTER_20_t store;
  ACS71020ERR error = readRegister(REGISTER_VOLATILE_20, store.data.all);

  if (error != SUCCESS)
    return error;

  voltage = convertVoltage((float)store.data.bits.vrms);

  current = convertCurrent((float)store.data.bits.irms);

  return SUCCESS;
}

ACS71020ERR ACS71020::readRMSAvgSec(float &voltage_avg_sec, float &current_avg_sec) {
  REGISTER_26_t store;
  ACS71020ERR error = readRegister(REGISTER_VOLATILE_26, store.data.all);

  if (error != SUCCESS)
    return error;

  voltage_avg_sec = convertVoltage((float)store.data.bits.vrmsavgonesec);

  current_avg_sec = convertCurrent((float)store.data.bits.irmsavgonesec);

  return SUCCESS;
}

ACS71020ERR ACS71020::readPowerActive(float &pActive) {
  REGISTER_21_t store;
  ACS71020ERR error = readRegister(REGISTER_VOLATILE_21, store.data.all);

  if (error != SUCCESS)
    return error;

  int value = store.data.bits.pactive;
  if (value & 0x10000) {
      value = -(~(value - 1) & 0x1FFFF);
  }

  pActive = convertPower((float)value);

  return SUCCESS;
}

ACS71020ERR ACS71020::readPowerActiveAvgSec(float &pActive_avg_sec) {
  REGISTER_28_t store;
  ACS71020ERR error = readRegister(REGISTER_VOLATILE_28, store.data.all);

  if (error != SUCCESS)
    return error;

  int value = store.data.bits.pactavgonesec;
  if (value & 0x10000) {
      value = -(~(value - 1) & 0x1FFFF);
  }

  pActive_avg_sec = convertPower((float)value);

  return SUCCESS;
}

ACS71020ERR ACS71020::readPowerApparent(float &pApparent) {
  REGISTER_22_t store;
  ACS71020ERR error = readRegister(REGISTER_VOLATILE_22, store.data.all);

  if (error != SUCCESS)
    return error;

  pApparent = convertPower((float)store.data.bits.papparent);

  return SUCCESS;
}

ACS71020ERR ACS71020::readPowerReactive(float &pReactive) {
  REGISTER_23_t store;
  ACS71020ERR error = readRegister(REGISTER_VOLATILE_23, store.data.all);

  if (error != SUCCESS)
    return error;

  pReactive = convertPower((float)store.data.bits.pimag);

  return SUCCESS;
}

ACS71020ERR ACS71020::readPowerFactor(float &pFactor) {
  REGISTER_24_t store;
  ACS71020ERR error = readRegister(REGISTER_VOLATILE_24, store.data.all);

  if (error != SUCCESS)
    return error;

  int value = store.data.bits.pfactor;
  if (value & 0x400) {
    value |= 0xF800;
  }

  pFactor = value / 0x200;

  return SUCCESS;
}

float ACS71020::convertVoltage(float voltage) {
  return(voltage / 0x8000) * _voltageSensingRange;
}

float ACS71020::convertCurrent(float current) {
  return(current / 0x4000) * _currentSensingRange;
}

float ACS71020::convertPower(float power) {
  return(power / 0x8000) * _voltageSensingRange * _currentSensingRange;
}