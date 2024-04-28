
// ESP32 I2C Scanner
// Based on code of Nick Gammon  http://www.gammon.com.au/forum/?id=10896
// ESP32 DevKit - Arduino IDE 1.8.5
// Device tested PCF8574 - Use pullup resistors 3K3 ohms !
// PCF8574 Default Freq 100 KHz

#include <Arduino.h>
#include <Wire.h>
#include <bitset>
#include <ACS71020.h>

#define kNOERROR 0
#define kREADERROR 1
#define kWRITEERROR 2

const uint32_t WRITE = 0x00;
const uint32_t READ = 0x80;
const uint32_t COMMAND_MASK = 0x80;
const uint32_t ADDRESS_MASK = 0x7F;

unsigned long nextTime;

uint32_t default0B = 0b1000010001000000001110;

ACS71020 mySensor;

byte acs_address;

uint16_t Read(uint8_t address, uint32_t &value)
{
  uint16_t results = kNOERROR;
  Wire.beginTransmission(acs_address);
  Wire.write(address);
  results = Wire.endTransmission();
  if (results == kNOERROR)
  {
    Wire.requestFrom(acs_address, 4u);
    value = Wire.read(); // receive a byte as character
    value |= Wire.read() << 8; // receive a byte as character
    value |= Wire.read() << 16; // receive a byte as character
    value |= Wire.read() << 24; // receive a byte as character
  }
  else
  {
    Serial.println("Error");
  }
  return results;
}

uint16_t Write(uint8_t address, uint32_t value)
{
  uint16_t results = kNOERROR;
  Wire.beginTransmission(acs_address);
  // Send the address then the value (least significant byte first)
  Wire.write(address);
  Wire.write(value);
  Wire.write(value >> 8);
  Wire.write(value >> 16);
  Wire.write(value >> 24);
  results = Wire.endTransmission();

  if (address < 0x10)
  {
    delay(30); // If writing to EEPROM delay 30 ms
  }
  return results;
}

void updateRegister0B(int qvo_fine, int sns_fine) {
    // Calculate two's complement for qvo_fine if it's negative
    if (qvo_fine < 0) {
        qvo_fine = ((~(-qvo_fine) + 1) & 0x1FF); // 9-bit two's complement
    } else {
        qvo_fine &= 0x1FF; // Ensure qvo_fine is only 9 bits
    }

    if (sns_fine < 0) {
        sns_fine = ((~(-sns_fine) + 1) & 0x1FF); // 9-bit two's complement
    } else {
        sns_fine &= 0x1FF; // Ensure qvo_fine is only 9 bits
    }

    // Extracting unchanged parts from the original register value
    // Assuming original register value is 0b1000010001000000001110 = 0x084007E
    uint32_t actual = 0;
    mySensor.readRegister(REGISTER_SHADOW_1B, actual);
    uint32_t crs_sns = (actual >> 18) & 0x7;      // Bits 20:18
    uint32_t iavgselen = (actual >> 21) & 0x1;    // Bit 21
    uint32_t unused = (actual >> 22) & 0xF;       // Bits 25:22
    uint32_t ecc = (actual >> 26) & 0x3F;         // Bits 31:26

    // Constructing the new register value
    uint32_t newRegister = 0;
    newRegister |= (ecc << 26);
    newRegister |= (unused << 22);
    newRegister |= (iavgselen << 21);
    newRegister |= (crs_sns << 18);
    newRegister |= (sns_fine << 9);
    newRegister |= qvo_fine;

    Serial.print("0x0B - ");
    Serial.println(newRegister, HEX);

    mySensor.writeRegister(REGISTER_SHADOW_1B, newRegister);
}

void updateRegister0D(int pacc_trim) {
    // Calculate two's complement for pacc_trim if it's negative
    if (pacc_trim < 0) {
        pacc_trim = ((~(-pacc_trim) + 1) & 0x7F); // 7-bit two's complement
    } else {
        pacc_trim &= 0x7F; // Ensure pacc_trim is only 7 bits
    }

    // Extracting unchanged parts from the original register value
    // Assuming original register value is 0x3FFFFFF = 67108863
    uint32_t actual = 0;
    mySensor.readRegister(REGISTER_SHADOW_1D, actual);
    uint32_t ichan_del_en = (actual >> 7) & 0x1;       // Bit 7
    uint32_t unused1 = (actual >> 8) & 0x1;            // Bit 8
    uint32_t chan_del_sel = (actual >> 9) & 0x7;       // Bits 11:9
    uint32_t unused2 = (actual >> 12) & 0x1;           // Bit 12
    uint32_t fault = (actual >> 13) & 0xFF;            // Bits 20:13
    uint32_t fltdly = (actual >> 21) & 0x7;            // Bits 23:21
    uint32_t halfcycle_en = (actual >> 24) & 0x1;      // Bit 24
    uint32_t squarewave_en = (actual >> 25) & 0x1;     // Bit 25
    uint32_t ecc = (actual >> 26) & 0x3F;              // Bits 31:26

    // Constructing the new register value
    uint32_t newRegister = 0;
    newRegister |= (ecc << 26);
    newRegister |= (squarewave_en << 25);
    newRegister |= (halfcycle_en << 24);
    newRegister |= (fltdly << 21);
    newRegister |= (fault << 13);
    newRegister |= (unused2 << 12);
    newRegister |= (chan_del_sel << 9);
    newRegister |= (unused1 << 8);
    newRegister |= (ichan_del_en << 7);
    newRegister |= pacc_trim; // Set pacc_trim at bits 6:0

    Serial.print("0x0D - ");
    Serial.println(newRegister, HEX);

    mySensor.writeRegister(REGISTER_SHADOW_1D, newRegister);
}

void setup()
{
  Serial.begin(115200);
  // Initialize serial
  // Turn on the pullup so the determination of communication protocol can be made.
  // pinMode(ProtocolSelectPin, INPUT_PULLUP);
  // delay(50); // Wait for the pullup to take affect
  // UseI2C = (digitalRead(ProtocolSelectPin) == HIGH);
  //  Initialize I2C
  Wire.begin();
  Wire.setClock(400000);

  while (!Serial)
    ;
  Serial.println("Using I2C version of ACS71020");
  nextTime = millis();

  byte error;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for (acs_address = 1; acs_address < 127; acs_address++)
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(acs_address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (acs_address < 16)
        Serial.print("0");
      Serial.print(acs_address, HEX);
      Serial.println("  !");

      nDevices++;
      break;
    }
    else if (error == 4)
    {
      Serial.print("Unknown error at address 0x");
      if (acs_address < 16)
        Serial.print("0");
      Serial.println(acs_address, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");

  mySensor.begin(acs_address, Wire);
  mySensor.unlock();
  //Write(0x2F, 0x4F70656E); // Unlock device
  //Write(0x30, 0x00000001);
  // If the Arduino has built in USB, keep the next line
  // in to wait for the Serial to initialize

  //Write(0x1B, 0x11E0F); //Default
  //mySensor.writeRegister(REGISTER_SHADOW_1B, 0x211E0F);
  //Write(0x1B, 0x211E0F);
  //            -|--|--------|
  //mySensor.writeRegister(REGISTER_EPROM_0B, 0x22EC0F);
  delay(100);
  //Write(0x1B, 0b1000010001000000001110);
  //Write(0x1B, 0x211E0F);
  //Write(0x1B, 0x13219);
  //delay(100);
  //mySensor.writeRegister(REGISTER_SHADOW_1C, 0x1F40040);
  //Write(0x1C, 0x1F40040);
  //Write(0x1C, 0b00000000000000000000000001000000);
  //mySensor.writeRegister(REGISTER_EPROM_0C, 0x1F40040);
  delay(100);
  //Write(0x1C, 0b10000000001000000);
  //delay(100);

  // Write 0x1D Default 3FFFFFF
  //mySensor.writeRegister(REGISTER_EPROM_0D, 0x3FFFF81);
  delay(100);
  //Write(0x1D, 0x3FFFFFF);
  //delay(100);

  uint32_t tuning;
  uint32_t crs_sns;
  uint32_t iavgselen;
  uint32_t average;
  uint32_t rms_avg_1;
  uint32_t rms_avg_2;
  uint32_t trim;
  uint32_t pacc_trim;
  uint32_t ichan_del_en;
  uint32_t chan_del_sel;
  uint32_t fault;
  uint32_t fitdly;
  uint32_t halfcycle_en;
  uint32_t squarewave_en;
  uint32_t events;
  uint32_t vevent_cycs;
  uint32_t vadc_rate_set;
  uint32_t overvreg;
  uint32_t undervreg;
  uint32_t delaycnt_sel;
  mySensor.readRegister(0x0B, tuning);
  Serial.print("Ox1B - ");
  Serial.println(tuning, HEX);
  int qvo_fine = tuning & 0x1FF;
  if (qvo_fine & 0x100) {
    // Convert from two's complement to integer
    qvo_fine = -(qvo_fine & 0xFF);
  } else {
    qvo_fine = qvo_fine & 0xFF;
  }
  Serial.print("qvo_fine: ");
  Serial.println(qvo_fine);
  int sns_fine = (tuning >> 9) & 0x1FF;
  if (sns_fine & 0x100) {
    // Convert from two's complement to integer
    sns_fine = -(sns_fine & 0xFF);
  } else {
    sns_fine = sns_fine & 0xFF;
  }
  Serial.print("sns_fine: ");
  Serial.println(sns_fine);
  crs_sns = (tuning >> 18) & 0b111;
  Serial.print("crs_sns: ");
  Serial.println(crs_sns);
  iavgselen = (tuning >> 21) & 0x1;
  Serial.print("iavgselen: ");
  Serial.println(iavgselen);
  mySensor.readRegister(0x0C, average);
  Serial.print("Ox1C - ");
  Serial.println(average, HEX);
  rms_avg_1 = average & 0x7F;
  Serial.print("rms_avg_1: ");
  Serial.println(rms_avg_1);
  rms_avg_2 = (average >> 7) & 0x3FF;
  Serial.print("rms_avg_2: ");
  Serial.println(rms_avg_2);
  mySensor.readRegister(0x0D, trim);
  Serial.print("Ox1D - ");
  Serial.println(trim, HEX);
  Serial.println(trim, HEX);
  pacc_trim = trim & 0x7F;
  Serial.print("pacc_trim: ");
  Serial.println(pacc_trim);
  ichan_del_en = (trim >> 7) & 0x1;
  Serial.print("ichan_del_en: ");
  Serial.println(ichan_del_en);
  chan_del_sel = (trim >> 9) & 0x1F;
  Serial.print("chan_del_sel: ");
  Serial.println(chan_del_sel);
  fault = (trim >> 13) & 0xFF;
  Serial.print("fault: ");
  Serial.println(fault);
  fitdly = (trim >> 21) & 0x1F;
  Serial.print("fitdly: ");
  Serial.println(fitdly);
  halfcycle_en = (trim >> 24) & 0x1;
  Serial.print("halfcycle_en: ");
  Serial.println(halfcycle_en);
  squarewave_en = (trim >> 25) & 0x1;
  Serial.print("squarewave_en: ");
  Serial.println(squarewave_en);
  vevent_cycs = events & 0x1F;
  Serial.print("vevent_cycs: ");
  Serial.println(vevent_cycs);
  vadc_rate_set = (events >> 5) & 0x1;
  Serial.print("vadc_rate_set: ");
  Serial.println(vadc_rate_set);
  overvreg = (events >> 8) & 0x3F;
  Serial.print("overvreg: ");
  Serial.println(overvreg);
  undervreg = (events >> 14) & 0x3F;
  Serial.print("undervreg: ");
  Serial.println(undervreg);
  delaycnt_sel = (events >> 20) & 0x1;
  Serial.print("delaycnt_sel: ");
  Serial.println(delaycnt_sel);
  delay(5000);
}

void loop()
{
  /*
  int analog36 = analogRead(36);
  int qvo_fine = map(analog36, 0, 4095, -256, 255);
  int analog39 = analogRead(39);
  int sns_fine = map(analog39, 0, 4095, -256, 255);
  updateRegister0B(qvo_fine, sns_fine);
  Serial.print("qvo_fine");
  Serial.print(" - ");
  Serial.println(qvo_fine);
  Serial.print("sns_fine");
  Serial.print(" - ");
  Serial.println(sns_fine);

  int analog34 = analogRead(34);
  int pacc_trim = map(analog34, 0, 4095, -64, 63);
  updateRegister0D(pacc_trim);

  //Serial.print(">qvo_fine:");
  //Serial.println(qvo_fine);
  //Serial.print(">sns_fine:");
  //Serial.println(sns_fine);
  Serial.print(">pacc_trim:");
  Serial.println(pacc_trim);
  */

  uint32_t vrms_irms;
  uint32_t vrms;
  uint32_t irms;
  uint32_t pactive;
  uint32_t papparent;
  uint32_t pimag;
  uint32_t pfactor;
  uint32_t numptsout;
  uint32_t vrmsavgonesec_irmsavgonesec;
  uint32_t vrmsavgonesec;
  uint32_t irmsavgonesec;
  uint32_t vrmsavgonemin_irmsavgonemin;
  uint32_t vrmsavgonemin;
  uint32_t irmsavgonemin;
  uint32_t pactavgonesec;
  uint32_t pactavgonemin;
  uint32_t vcodes;
  uint32_t icodes;
  uint32_t pinstant;
  uint32_t flags;

  float voltage = 0.0;
  float current = 0.0;
  float voltage_avg_sec = 0.0;
  float current_avg_sec = 0.0;

  mySensor.readRMS(voltage, current);

  Serial.print(">voltage:");
  Serial.println(voltage);
  Serial.print(">current:");
  Serial.println(current);

  mySensor.readRMSAvgSec(voltage_avg_sec, current_avg_sec);

  Serial.print(">voltage_avg_sec:");
  Serial.println(voltage_avg_sec);
  Serial.print(">current_avg_sec:");
  Serial.println(current_avg_sec);

  float active_power = 0.0;
  float active_power_avg_sec = 0.0;
  float apparent_power = 0.0;
  float reactive_power = 0.0;
  float power_factor = 0.0;

  mySensor.readPowerActive(active_power);
  mySensor.readPowerActiveAvgSec(active_power_avg_sec);
  mySensor.readPowerApparent(apparent_power);
  mySensor.readPowerReactive(reactive_power);
  mySensor.readPowerFactor(power_factor);

  Serial.print(">active_power:");
  Serial.println(active_power);
  Serial.print(">active_power_avg_sec:");
  Serial.println(active_power_avg_sec);
  Serial.print(">apparent_power:");
  Serial.println(apparent_power);
  Serial.print(">reactive_power:");
  Serial.println(reactive_power);
  Serial.print(">power_factor:");
  Serial.println(power_factor);
  
  delay(10); // wait 5 seconds for next scan
}