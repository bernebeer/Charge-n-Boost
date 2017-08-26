/* My comments */

#include "Powerbank.h"
#include <Wire.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

// Powerbank object
Powerbank::Powerbank() {
  
}

// Function to intialise powerbank, set charger ic, fuel gauge ic, pinmodes etc
void Powerbank::init(unsigned int fast_chrg_current, unsigned int input_current) {

  // Clear the reset bit
  MCUSR &= ~_BV(WDRF);
  
  // Disable the WDT
  WDTCSR |= _BV(WDCE) | _BV(WDE);
  WDTCSR = 0;

  // Set pinmodes
  pinMode( BOOST_EN_PIN, OUTPUT );
  pinMode( CHARGE_EN_PIN, OUTPUT );
  pinMode( BTNPIN, INPUT_PULLUP );
  pinMode( SLIDE_POLE_4_PIN, INPUT_PULLUP );
  pinMode( SLIDE_POLE_1_PIN, INPUT_PULLUP );
  pinMode( LED_DATA_PIN, OUTPUT );
  pinMode( LEDFET_PIN, OUTPUT );

  digitalWrite( LEDFET_PIN, LOW );

  // Start I2c
  Wire.begin();
  
  // Set ADC reference to Internal (input current pin regulates to 0.8V; Ouput current pin regulates to 1.244V)
  analogReference(INTERNAL);

  // Initialise BQ25895
  // Reset all registers
  writeReg8(BQ25895_ADDRESS, BQ25895_REG_RESET, B10111001);
  // Init ADC, force D+ D- detection
  writeReg8(BQ25895_ADDRESS, BQ25895_REG_ADC_DATALINE_CONFIG, B11111101);
  // Set minimum system voltage to 3.0V (SYS ADC and TS ADC conk out below this value), disable OTG
  writeReg8(BQ25895_ADDRESS, BQ25895_REG_WD_CE_SYSVOLT_CONFIG, B11010000);
  // Disable enter ship mode delay
  writeReg8(BQ25895_ADDRESS, BQ25895_REG_BATFET_CONFIG, B01000000);
  // Disable stat pin (pad is exposed but unnecessary because of WS2812B's)
  writeReg8(BQ25895_ADDRESS, BQ25895_REG_STAT_TIMER_CONFIG, B11011101);
  // Set fast charge current limit
  uint8_t data = 0;
  if( fast_chrg_current <= 5000 ) {
    data = fast_chrg_current / 64;
  }
  else {
    data = 1000 / 64;
  }
  writeReg8(BQ25895_ADDRESS, BQ25895_REG_CHRG_CURRENT_CONFIG, data);
  // Set input current limit
  data = ( ( input_current / 50 ) - 2 );
  data = data | B01000000;
  writeReg8(BQ25895_ADDRESS, BQ25895_REG_INP_LIM, data);
  // Set termination current limit to 512mA
  writeReg8(BQ25895_ADDRESS, BQ25895_REG_PRE_TERM_LIM, B00011000);

  // Initialise LC709203F as per datasheet startup procedure
  // Set to operational mode
  writeReg16(LC709203F_ADDRESS, LC709203F_REG_POWER, 0x0001);
  // Set parasitic impedance, 200mOhms (rough estimation connector, cable and fuse resistance)
  writeReg16(LC709203F_ADDRESS, LC709203F_REG_APA, 0x00C8);
  // Set battery profile, Battery type 1 (3.7V; 4.2V), 
  writeReg16(LC709203F_ADDRESS, LC709203F_REG_BAT_PROF, 0x0001);
  // Set initial RSOC
  writeReg16(LC709203F_ADDRESS, LC709203F_INIT_RSOC, 0xAA55);
  // Set temp obtaining method
  writeReg16(LC709203F_ADDRESS, LC709203F_REG_TEMP_METH, 0x0000);
  // Set cell temperature
  // Todo: hardcoded to 25 degrees C atm, will have to update when taking TS measurements form BQ25895
  writeReg16(LC709203F_ADDRESS, LC709203F_REG_TEMP, 0x0BA6);

  // Test CRC8 PROGMEM by doing a temperature write and reading it back, if it matches we know the write has been succesful
  /*
  writeReg16(LC709203F_ADDRESS, LC709203F_REG_TEMP, 0x0D03);
  Serial.println( readReg16(LC709203F_ADDRESS, LC709203F_REG_TEMP), HEX);
  */
  
}

// Function to reset charger ic watchdog, will need to be called every 40 seconds; Update temp information fuel gauge
void Powerbank::resetWatchdog() {
  writeReg8(BQ25895_ADDRESS, BQ25895_REG_WD_CE_SYSVOLT_CONFIG, B11010000); 
}

// Function to enable boost converter ic
void Powerbank::enableBoost( boolean setting ) {
  if ( setting == true ) {
    digitalWrite( BOOST_EN_PIN, HIGH );
  }
  else {
    digitalWrite( BOOST_EN_PIN, LOW );
  }
}

// Function to enable boost converter ic
void Powerbank::enableCharge( boolean setting ) {
  if ( setting == true ) {
    digitalWrite( CHARGE_EN_PIN, LOW );
  }
  else {
    digitalWrite( CHARGE_EN_PIN, HIGH );
  }
}

// Function to enable high voltage usb output to 5.5V, handy when output current is high and voltage drop in cables become a problem
void Powerbank::highVoltageMode( boolean setting ) {
  if ( setting == true ) {
    pinMode( HI_VOLT_EN_PIN , OUTPUT );
  }
  else {
    pinMode( HI_VOLT_EN_PIN , INPUT );
  }
}

byte Powerbank::getSlidePosition() {
  byte data = 0;
  if ( digitalRead(SLIDE_POLE_4_PIN) == LOW && digitalRead(SLIDE_POLE_1_PIN) == HIGH )  {
    data = 1;
    return data;
  }
  else if ( digitalRead(SLIDE_POLE_4_PIN) == HIGH && digitalRead(SLIDE_POLE_1_PIN) == HIGH )  {
    data = 2;
    return data;
  }
  else {
    data = 3;
    return data;
  }
}

// Function to retrieve input current, current flowing into device through vBus
float Powerbank::getInputCurrent( byte slideSetting ) {
  // Iin = (KILIM x VILIM) / (RILIM x 0.8)
  float data = ( 350.0 * (1100.0 / 1024.0 * analogRead(ILIMPIN)) )  /  ( 390.0 / slideSetting * 0.8 ) ;
  return data;
}

// Function to retrieve charging current flowing into battery
int Powerbank::getChargeCurrent() {
  byte data = readReg8(BQ25895_ADDRESS, BQ25895_REG_ADC_CHRG_CURRENT);
  return (data * 50);
}

// Function to retrieve SYS voltage (BQ25895 SYS voltage, feeds input of boost converter)
int Powerbank::getSysVoltage() {
  byte data = readReg8(BQ25895_ADDRESS, BQ25895_REG_ADC_SYS_VOLT);
  int sysVoltage = ( (data & B01111111) * 20 ) + 2304;
  return sysVoltage;
}

// Function to retrieve voltage at charger input
// Todo: return vbus voltage only when charger IC has accurate voltage readign
int Powerbank::getVbusVoltage() {
  byte data = readReg8(BQ25895_ADDRESS, BQ25895_REG_ADC_VBUS_VOLT);
  int vbusVoltage = ( data & B01111111 ) * 100 + 2600;
  if ( (data & B10000000) >> 7 ) {
    return vbusVoltage;
  }
  else {
    return 0;
  }
}

// Function to retrieve battery voltage, read from fuel gauge ic
int Powerbank::getBatteryVoltage() {
  int data = 0;
  data = readReg16(LC709203F_ADDRESS, LC709203F_REG_CELL_VOLT);
  return data;
}

// Function to retrieve current flowing out of boost converter, derived from voltage at CC pin TPS61236
float Powerbank::getOutputCurrent() {
  float data = ( 1100.0 / 1024.0 * analogRead(CCPIN) ) / 37000.0 * 100000.0;
  return data;
}

// Function to retrieve current charge status of charger ic
byte Powerbank::getChargeStatus() {
  byte data = readReg8(BQ25895_ADDRESS, BQ25895_REG_VBUS_CHRG_STAT);
  data = ( (data & B00011000) >> 3 );
  return data;  
}

// Function to disable batfet separating battery and SYS
void Powerbank::batfetDisable() {
  writeReg8(BQ25895_ADDRESS, BQ25895_REG_BATFET_CONFIG, B01100000);  
}

// Function to check if batfet is disabled
boolean Powerbank::isBatfetDisabled() {
  byte data = readReg8(BQ25895_ADDRESS, BQ25895_REG_BATFET_CONFIG);
  if ( (data & B00100000) >> 5 ) {
    return true;
  }
  else {
    return false;
  }
}

// Function to enable batfet
void Powerbank::batfetEnable() {
  writeReg8(BQ25895_ADDRESS, BQ25895_REG_BATFET_CONFIG, B01000000);
}

// Function to poll for a button press
boolean Powerbank::btnPressed() {
  boolean data;
  if ( digitalRead(BTNPIN) == LOW ) {
    data = true;
  }
  else {
    data = false;
  }
  return data;
}

// Function to retrieve type of device at charger input, SDP, DCP, High voltage charger etc
byte Powerbank::getVbusInputType() {
  byte data = readReg8(BQ25895_ADDRESS, BQ25895_REG_VBUS_CHRG_STAT);
  data = (data & B11100000) >> 5;
  return data;
}

// Used by sleepTimeWake() to set sleep time
void Powerbank::sleepTimerRoutine( int numberOfCycles, byte interval ) {

  for ( int i = 0; i < numberOfCycles; i++ ) {
    // disable ADC
    ADCSRA = 0;
    // clear various "reset" flags
    MCUSR = 0;     
    // allow changes, disable reset
    WDTCSR = bit (WDCE) | bit (WDE);
    // set interrupt mode and an interval 
    WDTCSR = 0b01000000 | interval;
    wdt_reset();  // pat the dog
    
    set_sleep_mode (SLEEP_MODE_PWR_DOWN);  
    noInterrupts ();           // timed sequence follows
    sleep_enable();
   
    // turn off brown-out enable in software
    MCUCR = bit (BODS) | bit (BODSE);
    MCUCR = bit (BODS); 
    interrupts ();             // guarantees next instruction executed
    sleep_cpu ();
  }
  
}

// Interrupt service routine when sleep timer expires
ISR (WDT_vect) {
   wdt_disable();  // disable watchdog
}  // end of WDT_vect

// Function to enable low power sleep for x amount of seconds
void Powerbank::sleepTimeWake(int seconds) {
  
  // Disable batfet bq25895
  writeReg8(BQ25895_ADDRESS, BQ25895_REG_BATFET_CONFIG, B01100000);

  // Disable battery monitor
  writeReg8(BQ25895_ADDRESS, BQ25895_REG_ADC_DATALINE_CONFIG, B00111101);

  // BQ25895 to hiz
  writeReg8(BQ25895_ADDRESS, BQ25895_REG_INP_LIM, B11001010);

  // Set LC709203F to sleepmode
  writeReg16(LC709203F_ADDRESS, LC709203F_REG_POWER, 0x0002);

  unsigned int numberOf8sCycles = seconds / 8;
  byte numberOf4sCycles = ( seconds - (8 * numberOf8sCycles)) / 4;
  byte numberOf2sCycles = ( seconds - (8 * numberOf8sCycles) - (4 * numberOf4sCycles) ) / 2;
  byte numberOf1sCycles = ( seconds - (8 * numberOf8sCycles) - (4 * numberOf4sCycles) - (2 * numberOf2sCycles) );

  sleepTimerRoutine( numberOf8sCycles, 0b100001 );
  sleepTimerRoutine( numberOf4sCycles, 0b100000 );
  sleepTimerRoutine( numberOf2sCycles, 0b000111 );
  sleepTimerRoutine( numberOf1sCycles, 0b000110 ); 
    
  // cancel sleep as a precaution
  sleep_disable();

  // Do an Atmega328 self reset after 15ms WDT timeout
  wdt_enable(WDTO_15MS);
  while(1) {};

}

// Function to sleep the powerbank and wake from button press, reduce current consumption to 45uA
void Powerbank::sleepBtnWake() {

  // Disable batfet bq25895
  writeReg8(BQ25895_ADDRESS, BQ25895_REG_BATFET_CONFIG, B01100000);

  // Disable battery monitor
  writeReg8(BQ25895_ADDRESS, BQ25895_REG_ADC_DATALINE_CONFIG, B00111101);

  // BQ25895 to hiz
  writeReg8(BQ25895_ADDRESS, BQ25895_REG_INP_LIM, B11001010);

  // Set LC709203F to sleepmode
  writeReg16(LC709203F_ADDRESS , LC709203F_REG_POWER, 0x0002);

  // Disable boost
  digitalWrite(BOOST_EN_PIN, LOW);

  // disable ADC
  ADCSRA = 0;  

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);  
  sleep_enable();  
  noInterrupts ();  
  attachInterrupt (INTERRUPT_ID_BTN, wake, FALLING);
  attachInterrupt (INTERRUPT_ID_BQ25895, wake, FALLING);
  EIFR = bit (INTF1);
  EIFR = bit (INTF0);
  
  // turn off brown-out enable in software
  // BODS must be set to one and BODSE must be set to zero within four clock cycles
  MCUCR = bit (BODS) | bit (BODSE);
  // The BODS bit is automatically cleared after three clock cycles
  MCUCR = bit (BODS); 

  interrupts();
  sleep_cpu();

  ADCSRA = 135;

  // Debounce delay
  delay(500);

  // Do an Atmega328 self reset after 15ms WDT timeout
  wdt_enable(WDTO_15MS);
  while(1) {};
  
}

// Function to sleep the powerbank but keep booster output enabled, higher current draw but able to wake on load detect (inact pin)
void Powerbank::sleepInactWake() {
  
}

// Function to retrieve battery percentage from fuel gauge ic
byte Powerbank::getBatteryLevel() {
  word data = 0;
  data = readReg16(LC709203F_ADDRESS, LC709203F_REG_RSOC);
  byte lowByteData = lowByte(data);
  return lowByteData;
}

// Function to read battery temperature using thermistor in voltage divider
// Todo: REGN voltage varies when Vbus is attached, 4.8V when not attached, ca. 6V when 9V attached
float Powerbank::getBatteryTemp( int betaValue, boolean outputCelsius ) {
  byte data = readReg8(BQ25895_ADDRESS, BQ25895_REG_ADC_TEMP);
  float val = (data * 0.465) + 21.0; // Get BQ25895 ADC reading TS pin (expressed as a % of REGN voltage)
  
  val = 4.8 * val / 100.0; // Calculate voltage on TS pin (from ADC reading and REGN voltage)
  val = ( val * 5230.0 ) / ( 4.8 - val ); // Calculate total resistance of R2 (from voltage TS pin and R1 5230ohm resistor)
  val = ( 1.0 / ( (1.0 / val) - (1.0 / 30100.0) ) ); // Calculate NTC thermistor resistance (from total resistance R2 and 30100ohm parallel resistor)

  // Calculate degrees Celsius from NTC thermistor resistance using Steinhart Hart
  val = val / 10000.0;
  val = log(val);
  val /= betaValue;
  val += 1.0 / (25.0 + 273.15);
  val = 1.0 / val;
  if (outputCelsius) {
    val -= 273.15;
  }
  return val;
}

int Powerbank::getIcoLimit() {
  int data = 0;
  data = readReg8(BQ25895_ADDRESS, BQ25895_REG_ICO_LIM);
  data = (data & B00111111) * 50 + 100;
  return data;
}


// Function to run when sleep mode is interrupted, exit sleep and remove interrupts
void wake() {
  sleep_disable();  
  detachInterrupt(INTERRUPT_ID_BTN);  
  detachInterrupt(INTERRUPT_ID_BQ25895); 
}

// Function to look up CRC-8 CCITT error detecting code needed for LC709203F I2c error checking (see https://en.wikipedia.org/wiki/Cyclic_redundancy_check)
// Todo: calculate CRC-8 CCITT code instead of using ram hungry lookup table
// Done: Function now takes value from table in flash memory
byte Powerbank::crc8ccitt(const void * data, size_t size) {
  byte val = 0;
  byte * pos = (byte *) data;
  byte * end = pos + size;
  while (pos < end) {
    val = pgm_read_byte_near(CRC_TABLE + (val ^ *pos));
    pos++;
  }
  return val;
}

// General function to read 8 bit I2c registers
byte Powerbank::readReg8(int deviceAddress, int regAddress) {
  byte data = 0;
  Wire.beginTransmission(deviceAddress);
  Wire.write(regAddress);
  Wire.endTransmission();
  Wire.requestFrom(deviceAddress, 1);
  data = Wire.read();
  return(data);
}

// General function to read 16 bit I2c registers, repeated start enabled. Ignores CRC-8 code sent by LC709203F, works fine without
word Powerbank::readReg16(int deviceAddress, int regAddress) {
  word data = 0;
  Wire.beginTransmission(deviceAddress);
  Wire.write(regAddress);
  Wire.endTransmission(false);
  Wire.requestFrom(deviceAddress, 2);
  byte lowByteData = Wire.read();
  byte highByteData = Wire.read();
  data = word(highByteData, lowByteData);
  return(data);
}

// General function to write to 8 bit I2c registers
void Powerbank::writeReg8(int deviceAddress, int regAddress, byte data) {
  Wire.beginTransmission(deviceAddress);
  Wire.write(regAddress);
  Wire.write(data);
  Wire.endTransmission();
}

// General function to write to 16 bit I2c registers (specifically for LC709203F as CRC-8 error checking is needed and included in function)
void Powerbank::writeReg16(int deviceAddress, int regAddress, word data) {
  // Setup array to hold bytes to send including CRC-8
  byte crcCalcArray[5];
  crcCalcArray[0] = 0x16;
  crcCalcArray[1] = regAddress;
  crcCalcArray[2] = lowByte(data);
  crcCalcArray[3] = highByte(data);
  // Take bytes from intended I2c sequence and encode to CRC-8, place CRC-8 code in array
  crcCalcArray[4] = crc8ccitt( crcCalcArray, 4 );
  // Point to device
  Wire.beginTransmission(deviceAddress);
  // Point to register
  Wire.write(regAddress);
  // Send low byte
  Wire.write(crcCalcArray[2]);
  // Send high byte
  Wire.write(crcCalcArray[3]);
  // Send CRC-8 byte
  Wire.write(crcCalcArray[4]);
  // Transmit
  Wire.endTransmission();
}