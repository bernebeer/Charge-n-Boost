# Charge-n-Boost
<div class="code-desc">
<pre>mypb.init( int fastChargeCurrent, int inputCurrent );</pre>
<h4>Description</h4>
Resets all charger registers to default value. Sets fastcharge current limit. Sets input current limit. Begins Atmega328P I2C. Sets Atmega328P ADC reference to internal 1.1V. Typically placed in setup.
<h4>Parameters</h4>
fastChargeCurrent: Set fast charge current limit (500 - 5000mA). inputCurrent: Set charger input current limit (500 - 3250mA). Input current value is ignored if charger detects attached USB charger with lower current rating. Input current value is also ignored if physical slide switch is set to lower current value.
<h4>Returns</h4>
None

</div>
<div class="code-desc">
<pre>mypb.resetWatchdog();</pre>
<h4>Description</h4>
Resets charger IC watchdog timer. Function must be called every 100 seconds (or less) to avoid charger IC reset to default register settings (standalone mode).
<h4>Parameters</h4>
None.
<h4>Returns</h4>
None.

</div>
<div class="code-desc">
<pre>mypb.getChargeCurrent();</pre>
<h4>Description</h4>
Gets current going into battery. Value is retrieved using BQ25895's internal 7 bits ADC.
<h4>Parameters</h4>
None.
<h4>Returns</h4>
Charge current in mA, 50mA resolution (int).

</div>
<div class="code-desc">
<pre>mypb.getBatteryLevel();</pre>
<h4>Description</h4>
Gets current battery level. Value is retrieved over I2C from fuel gauge.
<h4>Parameters</h4>
None.
<h4>Returns</h4>
Battery relative state of charge as a percentage (byte).

</div>
<div class="code-desc">
<pre>mypb.getOutputCurrent();</pre>
<h4>Description</h4>
Gets boost regulator (usb output) current. Current is calculated using Atmega328p internal 1.1V voltage reference and boost regulator constant current pin regulating to 1.244V at full current output.
<h4>Parameters</h4>
None.
<h4>Returns</h4>
Output current in mA (0.0 - 3500.0, float).

</div>
<div class="code-desc">
<pre>mypb.getBatteryVoltage();</pre>
<h4>Description</h4>
Gets battery voltage from fuel gauge over I2C.
<h4>Parameters</h4>
None.
<h4>Returns</h4>
Battery voltage in mV (unsigned long).

</div>
<div class="code-desc">
<pre>mypb.getVbusVoltage();</pre>
<h4>Description</h4>
Gets voltage on charger input. 7 Bits Value retrieved from charger IC over I2C.
<h4>Parameters</h4>
None.
<h4>Returns</h4>
Voltage on charger input in mV, range 2600 - 15300mV (100mV resolution, unsigned int).

</div>
<div class="code-desc">
<pre>mypb.isCharging();</pre>
<h4>Description</h4>
Checks if battery is being charged.
<h4>Parameters</h4>
None.
<h4>Returns</h4>
True if battery is precharging or fastcharging (boolean).

</div>
