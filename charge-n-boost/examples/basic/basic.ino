#include "Powerbank.h"
#include <Wire.h>

#define MAX_INPUT_CURRENT       3000
#define MAX_FASTCHARGE_CURRENT  2000

// Instantiate Powerbank object, name it anything, in this case 'mypb'
Powerbank mypb;

unsigned long previousMillis = 0;

void setup() {
  
  Serial.begin(115200);

  // Initialise powerbank: full reset, set fastcharge current limit and input current limit, 
  mypb.init(MAX_FASTCHARGE_CURRENT, MAX_INPUT_CURRENT);
  
  pinMode(LEDFETPIN, OUTPUT);
  
}

void loop() {

  // Check powerbank every second, remember to reset powerbank watchdog timer
  unsigned long currentMillis = millis();
  if ( currentMillis - previousMillis >= 1000 ) {

    // Reset BQ25895 watchdog timer, prevents BQ25895 register reset to default, needed every 40 seconds if BQ25895 did not receive an I2c write
    mypb.resetWatchdog();

    // Serial print powerbank data
    
    // Get detected input USB type
    Serial.print("Vbus input type: \t");
    Serial.print( mypb.getVbusInputType() );
    Serial.println("\t 0 = No input, 2 = USB CDP (1.5A), 3 = USB DCP (3.25A), 4 = Adj. Hi-V. DCP (1.5A), 5 = Unk. Adap., 6 = Non Std. Adap.");

    // Get input voltage
    Serial.print("Vbus voltage: \t\t");
    Serial.print( mypb.getVbusVoltage() );
    Serial.println("mV");

    // Get charge current
    Serial.print("Charge current: \t");
    Serial.print( mypb.getChargeCurrent() );
    Serial.println("mA");

    // Get battery level
    Serial.print("Battery level: \t\t");
    Serial.print( mypb.getBatteryLevel() );
    Serial.println("%");

    // Get battery voltage
    Serial.print("Battery voltage: \t");
    Serial.print( mypb.getBatteryVoltage() );
    Serial.println("mV");

    // Get output current
    Serial.print("Output current: \t");
    Serial.print( mypb.getOutputCurrent(), 0 );
    Serial.println("mA");

    // Get system voltage
    Serial.print("System voltage: \t");
    Serial.print( mypb.getSysVoltage() );
    Serial.println("mV");

    // Check if battery mosfet is disabled
    Serial.print("Batfet disabled: \t");
    if ( mypb.isBatfetDisabled() ) {
      Serial.println("Yes");
    }
    else {
      Serial.println("No");
    }

    // Get charge status ( 0 = Not charging, 1 = Pre-charge, 2 = Fast charging, 3 = Charge termination done )
    Serial.print("Charge status: \t\t");
    Serial.print( mypb.getChargeStatus() );
    Serial.println("\t 0 = Not charging, 1 = Pre-charge, 2 = Fast charging, 3 = Charge termination done");
        
    // Print empty line for readability
    Serial.println();
    previousMillis = currentMillis;
    
  }

  if ( mypb.btnPressed() ) {
    mypb.sleepBtnWake();    
  }
  
}