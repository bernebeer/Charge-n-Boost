# Charge 'n Boost library for Charge 'n Boost programmable powerbank board.

<div class="code-desc">

<pre>mypb.getChargeStatus();</pre>

#### Description

Checks current charge status.

#### Parameters

None.

#### Returns

Returns corresponding charge status value (byte):  
0 = Not charging  
1 = Pre-charge  
2 = Fast charging  
3 = Charge termination done

</div>

<div class="code-desc">

<pre>mypb.getVbusInputType();</pre>

#### Description

Checks detected charger input type. Detection is done automatically by charger IC and stored in one of its registers.

#### Parameters

None.

#### Returns

Returns corresponding input type value (byte):  
0 = No input  
2 = USB CDP (1.5A)  
3 = USB DCP (3.25A)  
4 = Adjustable High voltage DCP (1.5A, Quickcharge)  
5 = Unknown Adapter  
6 = Non Standard adapter

</div>

<div class="code-desc">

<pre>mypb.btnPressed();</pre>

#### Description

Polls for button press (button is connected to hardware interrupt pin (PD3), buttonpress detect is possible without polling but will need to be coded manually).

#### Parameters

None.

#### Returns

True if button is pressed (boolean).

</div>

<div class="code-desc">

<pre>mypb.highVoltageMode( boolean setting );</pre>

#### Description

Sets boost regulator output to 5.5V instead of default 5.1V. Useful when expecting significant voltage drop at higher output current (usb contact resistance, cable resistance etc). Calculate your estimated voltage drop using [this nifty calculator](http://www.calculator.net/voltage-drop-calculator.html?material=copper&wiresize=52.96&voltage=5.1&phase=dc&noofconductor=1&distance=1&distanceunit=meters&amperes=3.5&x=65&y=7).

#### Parameters

_setting (boolean):_ True will enable high voltage output, false will return to default 5.1V.

#### Returns

None.

</div>

<div class="code-desc">

<pre>mypb.sleepBtnWake();</pre>

#### Description

Forces charger, microcontroller and fuel gauge into sleep mode. Mosfet separating charger and attached battery is turned off. Charger IC is set to Hi-Z mode. Microcontroller is set to sleep mode. Fuel gauge IC set to sleep mode. Wake interrupts are attached to button press and charger interrupt. When device exits sleep mode after interrupt (button press or BQ25895 interrupt i.e. Vbus attach), a full Atmega328P reset is executed (reset after 15 ms watchdog expires), essentially restarting from start of sketch with all microcontroller registers reset. Current consumption is reduced to 45uA.

#### Parameters

None.

#### Returns

None.

</div>
