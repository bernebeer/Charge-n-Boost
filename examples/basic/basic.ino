#include "Powerbank.h"
#include <Wire.h>
#include <FastLED.h>

#define MAX_INPUT_CURRENT       3000
#define MAX_FASTCHARGE_CURRENT  2000
#define NUM_LEDS                3

// Instantiate Powerbank object, name it anything, in this case 'mypb'
Powerbank mypb;

CRGB leds[NUM_LEDS];

unsigned long previousMillis = 0;
unsigned long previousBrightnessMillis = 0;
unsigned long previousRunningMillis = 0;
boolean ledDirection = 1;
int brightness = 0;
unsigned long timestampIsActive = 0;

void setup() {
  
  Serial.begin(57600);
  
  mypb.init(MAX_FASTCHARGE_CURRENT, MAX_INPUT_CURRENT);

  FastLED.addLeds<NEOPIXEL, LED_DATA_PIN>(leds, NUM_LEDS);

  mypb.enableBoost( true );

  Serial.println("Reset!");
  
} // End setup

void loop() {

  digitalWrite( CHARGE_EN_PIN, LOW );

  switch( map( mypb.getBatteryLevel(), 0, 100, 1, 4 )  ) {

    case 1:
      pulseLeds( 100, 0, 0, 3 );
      break;
    case 2:
      pulseLeds( 0, 0, 100, 3 );
      break;
    case 3:
      pulseLeds( 0, 100, 0, 3 );
      break;
    default:
      pulseLeds( 100, 100, 100, 3 );
      break;
      
  }
  
  // Check powerbank every second
  unsigned long currentMillis = millis();
  if ( currentMillis - previousMillis >= 1000) {

    // Reset BQ25895 watchdog timer, prevents BQ25895 register reset to default, needed at least every 40 seconds if BQ25895 did not receive an I2c write
    mypb.resetWatchdog();

    // Serial print powerbank data
        // Get charge status ( 0 = Not charging, 1 = Pre-charge, 2 = Fast charging, 3 = Charge termination done )
    Serial.print("Charge status: \t\t");
    Serial.print( mypb.getChargeStatus() );
    Serial.println("\t 0 = Not charging, 1 = Pre-charge, 2 = Fast charging, 3 = Charge termination done");
    
    // Get detected input USB type
    Serial.print("Vbus input type: \t");
    Serial.print( mypb.getVbusInputType() );
    Serial.println("\t 0 = No input, 2 = USB CDP (1.5A), 3 = USB DCP (3.25A), 4 = Adj. Hi-V. DCP (1.5A), 5 = Unk. Adap., 6 = Non Std. Adap.");

    // Get battery level
    Serial.print("Battery level: \t\t");
    Serial.print( mypb.getBatteryLevel() );
    Serial.println("%");

    // Get battery voltage
    Serial.print("Battery voltage: \t");
    Serial.print( mypb.getBatteryVoltage() );
    Serial.println("mV");

    // Get input voltage
    Serial.print("Vbus voltage: \t\t");
    Serial.print( mypb.getVbusVoltage() );
    Serial.println("mV");

    // Get system voltage
    Serial.print("System voltage: \t");
    Serial.print( mypb.getSysVoltage() );
    Serial.println("mV");

    if ( mypb.getBatteryVoltage() < 3000 ) {
      mypb.sleepBtnWake();
    }

    // Get input current
    Serial.print("Input current: \t\t");
    Serial.print( mypb.getInputCurrent( 3 ), 0 );
    Serial.println("mA");

    // Get charge current
    Serial.print("Charge current: \t");
    Serial.print( mypb.getChargeCurrent() );
    Serial.println("mA");

    // Get output current
    Serial.print("Output current: \t");
    Serial.print( mypb.getOutputCurrent(), 0 );
    Serial.println("mA");

    // Get battery temperature
    Serial.print("Battery temp: \t\t");
    Serial.print( mypb.getBatteryTemp( 3435, false ), 0);
    Serial.println(" Kelvin");

    // Get Input Current Optimizer Limit (ICO)
    Serial.print("ICO: \t\t\t");
    Serial.print( mypb.getIcoLimit() );
    Serial.println(" mA");

    // Check if battery mosfet is disabled
    Serial.print("Batfet disabled: \t");
    if ( mypb.isBatfetDisabled() ) {
      Serial.println("Yes");
    }
    else {
      Serial.println("No");
    }

    // Check slide position
    Serial.print("Slide position: \t");
    Serial.println( mypb.getSlidePosition() );

    Serial.println();

    if( mypb.getOutputCurrent() > 1000 ) {
      mypb.highVoltageMode( true );
    }
    else {
      mypb.highVoltageMode( false );
    }

    if ( mypb.getOutputCurrent() > 100 || mypb.getChargeStatus() > 0 ) {
      timestampIsActive = millis();
    }

    Serial.print("Free RAM: ");
    Serial.println(freeRam());

    previousMillis = millis();
    
  }  
  /*
  if ( millis() - timestampIsActive > 5000 ) {
    mypb.sleepBtnWake();
  }
  */
  
} // End loop

boolean pulseLeds( byte red, byte green, byte blue, int numberOfLeds ) {

  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB::Black;
  }
  
  for (int i = 0; i < numberOfLeds; i++) {
    leds[i].setRGB( red, green, blue );
  }

  unsigned long currentBrightnessMillis = millis();
  if ( currentBrightnessMillis - previousBrightnessMillis > 4 ) {
    if ( ledDirection == 1 ) {
      brightness++;
      if ( brightness > 200 ) {
        ledDirection = 0;
      }
    } else {
      brightness--;
      if ( brightness == 0 ) {
        ledDirection = 1;
      }
    }
    FastLED.setBrightness(brightness);
    previousBrightnessMillis = millis();
  }
  
  FastLED.show(); 
                    
}

int freeRam () 
{
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}