# Charge 'n Boost library for Charge 'n Boost programmable powerbank board.

    mypb.init( int fastChargeCurrent );

**Description**
Enables charge IC. Enables Boost IC. Resets all charger registers to default value. Sets battery fastcharge current limit. Begins Atmega328P I2C. Sets Atmega328P ADC reference to internal 1.1V. Typically placed in setup.

**Parameters**
fastChargeCurrent: Set battery fast charge current limit (500 – 5000mA). fastChargeCurrent is automatically limited by detected USB input current rating and/or slide setting battery charge current limit. The lower of which is set as the limit.

**Returns**
None

----------


    mypb.resetWatchdog();

**Description**
Resets charger IC watchdog timer and re-enables charging function. Function must be called every 40 seconds (or less). Safety feature. In the unlikely event of microcontroller failure with I/O at hi-Z, charging and boosting  functions are disabled. In the event of microcontroller failure with charge-enable output LOW, charge IC will reset to standalone mode (2A charging).

**Parameters**
None.

**Returns**
None.

----------

    mypb.getInputCurrent();

**Description**
Gets current flowing into device.

**Parameters**
None.

**Returns**
Input current in mA (int).


----------


    mypb.getChargeCurrent();

**Description**
Gets current flowing into battery. Value is retrieved using BQ25895’s internal 7 bits ADC. Reading is battery charge current only, discharge current is not monitored.

**Parameters**
None.

**Returns**
Charge current in mA, 50mA resolution (int).


----------


    mypb.getBatteryLevel();

**Description**
Gets current battery level. Value is retrieved over I2C from fuel gauge.

**Parameters**
None.

**Returns**
Battery relative state of charge as a percentage (byte).


----------


    mypb.getOutputCurrent();

**Description**
Gets boost regulator (usb output) current. Current is calculated using Atmega328p internal 1.1V voltage reference and boost regulator constant current pin regulating to 1.244V at full current output.

**Parameters**
None.

**Returns**
Output current in mA (0.0 – 3500.0, float).


----------


    mypb.getBatteryVoltage();

**Description**
Gets battery voltage from fuel gauge over I2C.

**Parameters**
None.

**Returns**
Battery voltage in mV (unsigned long).


----------


    mypb.getVbusVoltage();

**Description**
Gets voltage on charger input. 7 Bits Value retrieved from charger IC over I2C.

**Parameters**
None.

**Returns**
Voltage on charger input in mV, range 2600 – 15300mV (100mV resolution, unsigned int).


----------


    mypb.getBatTemp( int betaValue, boolean setting );

**Description**
Gets the temperature from thermistor. Make sure the thermistor bead is placed as close to the battery as possible. Note: The charge IC will constantly monitor battery temperature, when out of temperature bounds (0° – 45° C), charge functions will cease.

**Parameters**
betavalue: the betavalue of the 10K NTC thermistor at room temperature (25° C). This value can be found in your thermistor’s datasheet and is used by a simplified Steinhart Hart equation to calculate temperature based on BQ25895’s voltage reading at the TS pin. setting: False for Kelvin or True for Celsius.

**Returns**
Temperature in degrees Kelvin or Celsius (byte).


----------


    mypb.getChargeStatus();

Description
Checks current charge status.

**Parameters**
None.

**Returns**
Returns corresponding charge status value (byte):
0 = Not charging
1 = Pre-charge
2 = Fast charging
3 = Charge termination done


----------


    mypb.getVbusInputType();

**Description**
Checks detected charger input type. Detection is done automatically by charger IC and stored in one of its registers.

**Parameters**
None.

**Returns**
Returns corresponding input type value (byte):
0 = No input
2 = USB CDP (1.5A)
3 = USB DCP (3.25A)
4 = Adjustable High voltage DCP (1.5A, Quickcharge)
5 = Unknown Adapter
6 = Non Standard adapter


----------


    mypb.btnPressed();

**Description**
Polls for button press (button is connected to hardware interrupt pin (PD3), buttonpress detect is possible without polling but will need to be coded manually).

**Parameters**
None.

**Returns**
True if button is pressed (boolean).


----------


    mypb.highVoltageMode( boolean setting );

**Description**
Sets boost regulator output to 5.5V instead of default 5.1V. Useful when expecting significant voltage drop at higher output current (usb contact resistance, cable resistance etc). Calculate your estimated voltage drop using this nifty calculator.

**Parameters**
setting (boolean): True will enable high voltage output, false will return to default 5.1V.

**Returns**
None.


----------


    mypb.sleepBtnWake();

**Description**
Forces charger, microcontroller and fuel gauge into sleep mode. Mosfet separating charger and attached battery is turned off. Charger IC is set to Hi-Z mode. Microcontroller is set to sleep mode. Fuel gauge IC set to sleep mode. Wake interrupts are attached to button press and charger interrupt. When device exits sleep mode after interrupt (button press or BQ25895 interrupt i.e. Vbus attach), a full Atmega328P reset is executed (reset after 15 ms watchdog expires), essentially restarting from start of sketch with all microcontroller registers reset. Current consumption is reduced to 45uA.

**Parameters**
None.

**Returns**
None.


----------



