#ifndef ACS71020_h
#define ACS71020_h

#include <Arduino.h>
#include <Wire.h>

const uint8_t DEFAULT_I2C_ADDRESS = 0x60;
//Customer Access Code - stored in volatile register 0x2F
const uint32_t CUSTOMER_ACCESS_CODE = 0x4F70656E;
const float DEFAULT_SENSE_RES = 1800;
const float DEFAULT_DIVIDER_RES = 4000000;
// VOLTAGE_RANGE = ΔVin_max * ((R1 + R2 + R3 + R4 + Rsense) / Rsense)
const float DEFAULT_VOLTAGE_RANGE = 611.386;
const float DEFAULT_CURRENT_RANGE = 90;

//EMPROM Registers
const uint8_t REGISTER_EPROM_0B = 0x0B;
const uint8_t REGISTER_EPROM_0C = 0x0C;
const uint8_t REGISTER_EPROM_0D = 0x0D;

//Shadow Registers
const uint8_t REGISTER_SHADOW_1B = 0x1B;
const uint8_t REGISTER_SHADOW_1C = 0x1C;
const uint8_t REGISTER_SHADOW_1D = 0x1D;

//Volatile Registers
const uint8_t REGISTER_VOLATILE_20 = 0x20;
const uint8_t REGISTER_VOLATILE_21 = 0x21;
const uint8_t REGISTER_VOLATILE_22 = 0x22;
const uint8_t REGISTER_VOLATILE_23 = 0x23;
const uint8_t REGISTER_VOLATILE_24 = 0x24;
const uint8_t REGISTER_VOLATILE_25 = 0x25;
const uint8_t REGISTER_VOLATILE_26 = 0x26;
const uint8_t REGISTER_VOLATILE_27 = 0x27;
const uint8_t REGISTER_VOLATILE_28 = 0x28;
const uint8_t REGISTER_VOLATILE_29 = 0x29;
const uint8_t REGISTER_VOLATILE_2A = 0x2A;
const uint8_t REGISTER_VOLATILE_2C = 0x2C;
const uint8_t REGISTER_VOLATILE_2D = 0x2D;
const uint8_t REGISTER_VOLATILE_2F = 0x2F;
const uint8_t REGISTER_VOLATILE_30 = 0x30;

//Error result
typedef enum {
  SUCCESS = 0,
  ERR_I2C_ERROR,
  ERR_REGISTER_READ_MODIFY_WRITE_FAILURE
} ACS71020ERR;

// Registers 0B and 1B have the same bitmap except for ecc
typedef struct
{
  union
  {
    uint32_t all;
    struct
    {
      uint32_t qvo_fine : 9;
      uint32_t sns_fine : 9;
      uint32_t crs_sns : 3;
      uint32_t iavgselen : 1;
      uint32_t reserved : 4;
      uint32_t ECC : 6;
    } bits;
  } data;
} REGISTER_0B_t;

// Registers 0C and 1C have the same bitmap except for ecc
typedef struct
{
  union
  {
    uint32_t all;
    struct
    {
      uint32_t rms_avg_1 : 7;
      uint32_t rms_avg_2 : 10;
      uint32_t reserved : 9;
      uint32_t ECC : 6;
    } bits;
  } data;
} REGISTER_0C_t;

typedef struct
{
  union
  {
    uint32_t all;
    struct
    {
      uint32_t pacc_trim : 7;
      uint32_t ichan_del_en : 1;
      uint32_t reserved1 : 1;
      uint32_t chan_del_sel: 3;
      uint32_t reserved2: 1;
      uint32_t fault: 8;
      uint32_t fltdly: 3;
      uint32_t halfcycle_en: 1;
      uint32_t squarewave_en: 1;
      uint32_t ECC : 6;
    } bits;
  } data;
} REGISTER_0D_t;

typedef struct
{
  union
  {
    uint32_t all;
    struct
    {
      uint32_t vrms : 15;
      uint32_t reserved1: 1;
      uint32_t irms : 15;
      uint32_t reserved2: 1;
    } bits;
  } data;
} REGISTER_20_t;

typedef struct
{
  union
  {
    uint32_t all;
    struct
    {
      uint32_t pactive : 17;
    } bits;
  } data;
} REGISTER_21_t;

typedef struct
{
  union
  {
    uint32_t all;
    struct
    {
      uint32_t papparent : 16;
    } bits;
  } data;
} REGISTER_22_t;

typedef struct
{
  union
  {
    uint32_t all;
    struct
    {
      uint32_t pimag : 16;
    } bits;
  } data;
} REGISTER_23_t;

typedef struct
{
  union
  {
    uint32_t all;
    struct
    {
      uint32_t pfactor : 11;
    } bits;
  } data;
} REGISTER_24_t;

typedef struct
{
  union
  {
    uint32_t all;
    struct
    {
      uint32_t vrmsavgonesec : 15;
      uint32_t reserved1: 1;
      uint32_t irmsavgonesec : 15;
      uint32_t reserved2: 1;
    } bits;
  } data;
} REGISTER_26_t;

typedef struct
{
  union
  {
    uint32_t all;
    struct
    {
      uint32_t pactavgonesec : 17;
    } bits;
  } data;
} REGISTER_28_t;

class ACS71020 {
  public:

    ACS71020();

    void begin(uint8_t address = DEFAULT_I2C_ADDRESS, TwoWire &wirePort = Wire);
    ACS71020ERR unlock();
    ACS71020ERR readRegister(uint8_t address, uint32_t &data);
    ACS71020ERR writeRegister(uint8_t address, uint32_t data);
    ACS71020ERR readRMS(float &voltage, float &current);
    ACS71020ERR readRMSAvgSec(float &voltage_avg_sec, float &current_avg_sec);
    ACS71020ERR readPowerActive(float &pActive);
    ACS71020ERR readPowerActiveAvgSec(float &pActive_avg_sec);
    ACS71020ERR readPowerApparent(float &pApparent);
    ACS71020ERR readPowerReactive(float &pReactive);
    ACS71020ERR readPowerFactor(float &pFactor);
    //ACS71020ERR readInstantaneous(float *vInst, float *iInst, float *pInst);

  private:

    TwoWire * _i2cPort;
    uint8_t _i2cAddress = DEFAULT_I2C_ADDRESS;
    float _senseResistance = DEFAULT_SENSE_RES;
    float _dividerResistance = DEFAULT_DIVIDER_RES;
    float _voltageSensingRange = DEFAULT_VOLTAGE_RANGE;
    float _currentSensingRange = DEFAULT_CURRENT_RANGE;

    float convertVoltage(float voltage);
    float convertCurrent(float current);
    float convertPower(float power);

};

#endif //ACS71020_h