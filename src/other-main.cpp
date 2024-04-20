
// ESP32 I2C Scanner
// Based on code of Nick Gammon  http://www.gammon.com.au/forum/?id=10896
// ESP32 DevKit - Arduino IDE 1.8.5
// Device tested PCF8574 - Use pullup resistors 3K3 ohms !
// PCF8574 Default Freq 100 KHz
/*
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

float qToFloat(int32_t fixedPointValue, int fractionalBits) {
    // Cast to float and shift down by the number of fractional bits
    return static_cast<float>(fixedPointValue) / (1 << fractionalBits);
}


uint16_t Read(uint8_t address, uint32_t &value)
{
  uint16_t results = kNOERROR;
  Wire.beginTransmission(0x66);
  Wire.write(address);
  results = Wire.endTransmission();
  if (results == kNOERROR)
  {
    Wire.requestFrom(0x66, 4u);
    value = Wire.read(); // receive a byte as character
  } else {
    Serial.println("Error");
  }
  return results;
}

uint16_t Write(uint8_t address, uint32_t value)
{
  uint16_t results = kNOERROR;
  Wire.beginTransmission(0x66);
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
    //pinMode(ProtocolSelectPin, INPUT_PULLUP);
    //delay(50); // Wait for the pullup to take affect
    //UseI2C = (digitalRead(ProtocolSelectPin) == HIGH);
        // Initialize I2C
        Wire.begin();
        Wire.setClock(400000);
    Write(0x2F, 0x4F70656E);   // Unlock device
    // If the Arduino has built in USB, keep the next line
    // in to wait for the Serial to initialize
    while (!Serial);
        Serial.println("Using I2C version of ACS71020");
    nextTime = millis();
}

void loop()
{
    uint16_t Irms;
      uint16_t Vrms;
      byte buff[4];
      Wire.beginTransmission(0x66);
      Wire.write( 0x20);
      Wire.endTransmission();
      Wire.requestFrom(0x66, 4);
      int nrrr = 0;
      while (Wire.available()) {   // slave may send less than requested
        char c = Wire.read();    // receive a byte as character
        buff[nrrr++] = c;
      }
    
      Vrms = (buff[1] << 8) + buff[0];
      bitClear(Vrms, 15);
      Irms = (buff[3] << 8) + buff[2];
      bitClear(Irms, 15);
    
      float testVRM =  ((float)Vrms) * 0.00003051757; //or /32768.0
      float testVRMMM = qToFloat(Vrms, 15);
      //Serial.println(testVRM);
      

      // both give same value
     testVRMMM = testVRMMM *325;
      Serial.println(testVRMMM);

     float testIRM = qToFloat(Irms, 14); // ((float)Irms) * 0.00006103515;

     float testIRMS =  ((float)Irms) * 0.00006103515; //or /32768.0
     testIRMS = testIRMS * 30;         // wait 5 seconds for next scan
      delay(500);
}
*/