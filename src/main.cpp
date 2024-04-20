
// ESP32 I2C Scanner
// Based on code of Nick Gammon  http://www.gammon.com.au/forum/?id=10896
// ESP32 DevKit - Arduino IDE 1.8.5
// Device tested PCF8574 - Use pullup resistors 3K3 ohms !
// PCF8574 Default Freq 100 KHz

#include <Arduino.h>
#include <Wire.h>
#include <bitset>

#define kNOERROR 0
#define kREADERROR 1
#define kWRITEERROR 2

const uint32_t WRITE = 0x00;
const uint32_t READ = 0x80;
const uint32_t COMMAND_MASK = 0x80;
const uint32_t ADDRESS_MASK = 0x7F;

unsigned long nextTime;

byte acs_address;

float qToFloat(int32_t fixedPointValue, int fractionalBits)
{
  // Cast to float and shift down by the number of fractional bits
  return static_cast<float>(fixedPointValue) / (1 << fractionalBits);
}

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

  Write(0x2F, 0x4F70656E); // Unlock device
  //Write(0x30, 0x00000001);
  // If the Arduino has built in USB, keep the next line
  // in to wait for the Serial to initialize

  //Write(0x1B, 0x11E0F); //Default
  Write(0x1B, 0x211E0F);
  //            -|--|--------|
  //Write(0x1B, 0b1000100000111111111110);
  //Write(0x1B, 0x211E0F);
  //Write(0x1B, 0x13219);
  delay(100);
  Write(0x1C, 0x1F40040);
  //Write(0x1C, 0b00000000000000000000000001000000);
  //Write(0x1C, 0b10000000001000000);
  delay(100);
  uint32_t tuning;
  //uint32_t qvo_fine;
  //uint32_t sns_fine;
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
  Read(0x1B, tuning);
  int qvo_fine = tuning & 0x1FF;
  if (qvo_fine & 0x100) {
    qvo_fine = -(qvo_fine & 0xFF);  // Convert from two's complement to integer
  } else {
    qvo_fine = qvo_fine & 0xFF;
  }
  Serial.print("qvo_fine: ");
  Serial.println(qvo_fine);
  int sns_fine = (tuning >> 9) & 0x1FF;
  if (sns_fine & 0x100) {
    sns_fine = -(sns_fine & 0xFF);  // Convert from two's complement to integer
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
  Read(0x1C, average);
  Serial.println(average, HEX);
  rms_avg_1 = average & 0x7F;
  Serial.print("rms_avg_1: ");
  Serial.println(rms_avg_1);
  rms_avg_2 = (average >> 7) & 0x3FF;
  Serial.print("rms_avg_2: ");
  Serial.println(rms_avg_2);
  Read(0x0D, trim);
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
  //teleplot.update("vrms", 10);
}

void loop()
{
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
  Read(0x20, vrms_irms);
  Read(0x21, pactive);
  Read(0x22, papparent);
  Read(0x23, pimag);
  Read(0x24, pfactor);
  Read(0x25, numptsout);
  Read(0x26, vrmsavgonesec_irmsavgonesec);
  Read(0x27, vrmsavgonemin_irmsavgonemin);
  Read(0x28, pactavgonesec);
  Read(0x29, pactavgonemin);
  Read(0x2A, vcodes);
  Read(0x2B, icodes);
  Read(0x2C, pinstant);
  Read(0x2D, flags);
  //Serial.println(vrms_irms);
  //std::bitset<32> bits(vrms_irms);
  //Serial.println(bits.to_string().c_str());
  vrms = vrms_irms & 0x7FFF;
  auto v_rms=((float)vrms)/32768;
  v_rms=v_rms*(432);
  Serial.print(">v_rms:");
  Serial.println(v_rms);
  //Serial.print(">vrms:");
  //Serial.println(vrms);

  
  float volts = (float)vrms;
  
  volts /= 32768.0; //Convert from codes to the fraction of ADC Full Scale (16-bit)
  volts *= 275; //Convert to mV (Differential Input Range is +/- 250mV)
  volts /= 1000; //Convert to Volts
  //Correct for the voltage divider: (RISO1 + RISO2 + RSENSE) / RSENSE
  //Or:  (RISO1 + RISO2 + RISO3 + RISO4 + RSENSE) / RSENSE
  float resistorMultiplier = (4001800) / 1800;
  volts *= resistorMultiplier;
  //*vRMS = volts;
  //volts /= 32768.0;
  //volts *= 517;
  Serial.print(">volts:");
  Serial.println(volts);
  //Serial.println(volts);

  irms = (vrms_irms >> 16) & 0x7FFF;
  //Serial.print(">irms:");
  //Serial.println(irms);
  auto i_rms=((float)irms)/16384;
  i_rms=i_rms*(90);
  Serial.print(">i_rms:");
  Serial.println(i_rms);

  /*
  papparent = papparent & 0xFFFF;
  auto p_rms = ((float)papparent)/32768;
  p_rms = p_rms * 432 * 90;
  Serial.print(">p_rms:");
  Serial.println(p_rms);
  */

  pactive = pactive & 0x1FFFF;
  auto f = ((pactive & 0x7FFF) / 32768.0);
  f = f*432*90;
  if (pactive & 0x10000)
    f = -f;
  //auto p_rms = ((float)pactive)/32768;
  //p_rms = p_rms * 432 * 90;
  Serial.print(">f:");
  Serial.println(f);

  /*
  vrmsavgonesec = vrmsavgonesec_irmsavgonesec & 0x7FFF;
  auto v_rms_avg = ((float)vrmsavgonesec)/32768;
  v_rms_avg = v_rms_avg*(432);
  Serial.print(">v_rms_avg:");
  Serial.println(v_rms_avg);*/

  irmsavgonesec = (vrmsavgonesec_irmsavgonesec >> 16) & 0x7FFF;
  auto i_rms_avg = ((float)irmsavgonesec)/16384;
  i_rms_avg=i_rms_avg*(90);
  Serial.print(">i_rms_avg:");
  Serial.println(i_rms_avg);

  pactavgonesec = pactavgonesec & 0x1FFFF;
  auto p_act_rms_avg = ((float)pactive)/32768;
  //p_act_rms_avg = p_act_rms_avg * 432 * 90;
  Serial.print(">p_act_rms_avg:");
  Serial.println(pactavgonesec);

  /*
  union
  {
    int16_t Signed;
    uint16_t unSigned;
  } signedUnsigned;
  signedUnsigned.unSigned = irms; //Extract irms as signed int
  float amps = (float)signedUnsigned.Signed;
  amps /= 16384.0; //Convert from codes to the fraction of ADC Full Scale (16-bit)
  amps *= 90; //Convert to Amps
  Serial.println(amps);

  pactive = pactive & 0x1FFFF;
  Serial.print(">pactive:");
  Serial.println(pactive);
  paparent = paparent & 0xFFFF;
  Serial.print(">paparent:");
  Serial.println(paparent);
  */
  /*
  Serial.printf("vrms = %ul\n", vrms);
  irms = (vrms_irms >> 16) & 0x7FFF;
  Serial.printf("irms = %ul\n", irms);
  pactive = pactive & 0x1FFFF;
  Serial.printf("pactive = %dl\n", pactive);
  paparent = paparent & 0xFFFF;
  Serial.printf("paparent = %ul\n", paparent);
  pimag = pimag & 0x1FFFF;
  Serial.printf("pimag = %ul\n", pimag);
  pfactor = pfactor & 0x7FF;
  Serial.printf("pfactor = %dl\n", pfactor);
  numptsout = numptsout & 0x1FF;
  Serial.printf("numptsout = %ul\n", numptsout);
  vrmsavgonesec = vrmsavgonesec_irmsavgonesec & 0x7FFF;
  Serial.printf("vrmsavgonesec = %ul\n", vrmsavgonesec);
  irmsavgonesec = (vrmsavgonesec_irmsavgonesec >> 16) & 0x7FFF;
  Serial.printf("irmsavgonesec = %ul\n", irmsavgonesec);
  */
 /*
  vrmsavgonesec = vrmsavgonesec_irmsavgonesec & 0x7FFF;
  Serial.print(">vrmsavgonesec:");
  Serial.println(vrmsavgonesec);
  irmsavgonesec = (vrmsavgonesec_irmsavgonesec >> 16) & 0x7FFF;
  Serial.print(">irmsavgonesec:");
  Serial.println(irmsavgonesec);
  pactavgonesec = pactavgonesec & 0x1FFFF;
  Serial.print(">pactavgonesec:");
  Serial.println(pactavgonesec);
  */
  /*
  vrmsavgonemin = vrmsavgonemin_irmsavgonemin & 0x7FFF;
  Serial.print(">vrmsavgonemin:");
  Serial.println(vrmsavgonemin);
  irmsavgonemin = (vrmsavgonemin_irmsavgonemin >> 16) & 0x7FFF;
  Serial.print(">irmsavgonemin:");
  Serial.println(irmsavgonemin);
  */
  /*
  Serial.printf("vrmsavgonemin = %ul\n", vrmsavgonemin);
  float testa = qToFloat(vrmsavgonemin, 15);
  Serial.println(testa);
  irmsavgonemin = (vrmsavgonemin_irmsavgonemin >> 16) & 0x7FFF;
  Serial.printf("irmsavgonemin = %ul\n", irmsavgonemin);
  pactavgonesec = pactavgonesec & 0x1FFFF;
  Serial.printf("pactavgonesec = %ul\n", pactavgonesec);
  */
  //pactavgonemin = pactavgonemin & 0x1FFFF;
  //Serial.print(">pactavgonemin:");
  //Serial.println(pactavgonemin);
  /*
  Serial.printf("pactavgonemin = %ul\n", pactavgonemin);
  */
  //vcodes = vcodes & 0x1FFFF;
  //Serial.print(">vcodes:");
  //Serial.println(vcodes);
  /*
  Serial.printf("vcodes = %ul\n", vcodes);
  std::bitset<32> bitsv(vcodes);
  Serial.println(bitsv.to_string().c_str());
  */
  //icodes = icodes & 0x1FFFF;
  //Serial.print(">icodes:");
  //Serial.println(icodes);
  /*
  icodes = icodes & 0x1FFFF;
  Serial.printf("icodes = %ul\n", icodes);
  Serial.printf("pinstant = %ul\n", pinstant);
  Serial.print("pospf = ");
  Serial.println((flags >> 6) & 0x1);
  Serial.print("posangle = ");
  Serial.println((flags >> 5) & 0x1);
  Serial.print("undervoltage = ");
  Serial.println((flags >> 4) & 0x1);
  */
  //Serial.print(">undervoltage:");
  //Serial.println((flags >> 4) & 0x1);
  /*
  Serial.print("overvoltage = ");
  Serial.println((flags >> 3) & 0x1);
  Serial.print("faultlatched = ");
  Serial.println((flags >> 2) & 0x1);
  Serial.print("faultout = ");
  Serial.println((flags >> 1) & 0x1);
  Serial.print("vzerocrossout = ");
  Serial.println((flags >> 0) & 0x1);
  Serial.println();
  */
  //Serial.print(">vzerocrossout:");
  //Serial.println((flags >> 0) & 0x1);
  delay(10); // wait 5 seconds for next scan
}