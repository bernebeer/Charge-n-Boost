/* My comments */

#ifndef powerbank

#define powerbank

#if (ARDUINO >=100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define CCPIN                             A0
#define BTNPIN                            3
#define INT_INACT_INTBQ_PIN               2
#define BOOST_EN_PIN                      5
#define BOOST_HIV_PIN                     6
#define ILIM_PIN                          A3
#define BOOST_CURRENT_PIN                 A6

#define BQ25895_ADDRESS                   0x6a // 7 bits address
#define MAX17043_ADDRESS                  0x36
#define LC709203F_ADDRESS                 0x0b 

#define BQ25895_REG_INP_LIM               0x00
#define BQ25895_REG_ADC_DATALINE_CONFIG   0x02
#define BQ25895_REG_WD_CE_SYSVOLT_CONFIG  0x03
#define BQ25895_REG_CHRG_CURRENT_CONFIG   0x04
#define BQ25895_REG_BATFET_CONFIG         0x09
#define BQ25895_REG_VBUS_CHRG_STAT        0x0b
#define BQ25895_REG_ADC_SYS_VOLT          0x0f
#define BQ25895_REG_ADC_VBUS_VOLT         0x11
#define BQ25895_REG_ADC_CHRG_CURRENT      0x12
#define BQ25895_REG_RESET                 0x14

#define MAX17043_REG_VCELL                0x02
#define MAX17043_REG_SOC                  0x04
#define MAX17043_REG_MODE                 0x06
#define MAX17043_REG_CONFIG               0x0d

#define LC709203F_REG_1PC_SOC             0x0d
#define LC709203F_REG_TEMP_METHOD         0x16
#define LC709203F_REG_VCELL               0x09
#define LC709203F_REG_APA                 0x0b

class Powerbank  {
  
  public:
    Powerbank();
    void init(unsigned int fast_chrg_current, unsigned int input_current);
    void resetWatchdog();
    int getChargeCurrent();
    int getBatteryLevel();
    float getOutputCurrent();
    unsigned long getBatteryVoltage();
    unsigned int getSysVoltage();
    unsigned int getVbusVoltage();
    boolean isCharging();
    boolean isBatfetDisabled();
    void batfetDisable();
    void batfetEnable();
    boolean btnPressed();
    void restartFuelGauge();
    byte vbusInputType();
    void sleepBtnWake();    

  private:
    byte readReg8(int deviceAddress, int regAddress);
    void writeReg16(int deviceAddress, int regAddress, word data);
    void writeReg8(int deviceAddress, int regAddress, byte data);
    
}; // End class in semi-colon...

void wake();

#endif
