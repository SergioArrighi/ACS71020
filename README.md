001000101100100000001110

# ACS71020
This library is heavily inspired from [SparkFun_ACS37800_Power_Monitor_Arduino_Library](https://github.com/sparkfun/SparkFun_ACS37800_Power_Monitor_Arduino_Library/tree/main)

It is a minimal and readapted version.

 - **unlock()** -> Unlocks the device, necessary to write to EPROM.
 - **readRegister(uint8_t  address, uint32_t  &data)** -> Reads 32 bits of data from register through I2C.
 - **writeRegister(uint8_t  address, uint32_t  data)** -> Writes 32 bits to specified registry using I2C bus.
 - **readRMS(float  &voltage, float  &current)** -> Reads the voltage (V) and current (A) RMS values from registry 0x20.
 - **readRMSAvgSec(float  &voltage_avg_sec, float  &current_avg_sec)** -> Reads voltage (V) and current (I) RMS average values over the last second (register 0x26).
 - **readPowerActive(float  &pActive)** -> Reads active power (W) from registry 0x21.
 - **readPowerActiveAvgSec(float  &pActive_avg_sec)** -> Reads active power (W) average value over the last second.
 - **readPowerApparent(float  &pApparent)** -> Reads apparent power (VA) from registry 0x22.
 - **readPowerReactive(float  &pReactive)** -> Reads reactive power (VAR) from registry 0x23.
 - **readPowerFactor(float  &pFactor)** -> Reads power factor from registry 0x24.

### ACS71020 default settings
0x0B (0x11E0F)
qvo_fine: 15
sns_fine: 143
crs_sns: 0
iavgselen: 0

0x0C (0x0)
rms_avg_1: 0
rms_avg_2: 0

0x0D (0x1FE000)
pacc_trim: 0
ichan_del_en: 0
chan_del_sel: 16
fault: 255
fitdly: 0
halfcycle_en: 0
squarewave_en: 0

vevent_cycs: 0
vadc_rate_set: 0
overvreg: 0
undervreg: 0
delaycnt_sel: 0

### Chips settings
|Code|0x0B|0x0C|0x0D|
|--|--|--|--|
| 2239096K|0x22EC0F|0x1F40040|0x3FFFF81|