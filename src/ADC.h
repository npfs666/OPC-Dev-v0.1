#ifndef ADC_h
#define ADC_h

#include "ADS1120.h"

#define SPI_CLK  18
#define SPI_MISO 16
#define SPI_MOSI 19
#define SPI_CS   17
#define SPI_DRDY 8

class RTDSensor {

    #define TYPE_2WIRE 2
    #define TYPE_3WIRE 3
    #define TYPE_4WIRE 4

    public:
    bool newMeasure;
    uint8_t type;
    double_t value;
    int64_t sum;
    uint16_t samples, sampleCount;
    RTDSensor();
    void reset();
    void add(int32_t value);
    void compute();
    double_t readValue();
};

class ADC {
  public:
    ADS1120 ads1120;
    RTDSensor rtd;

    ADC();
    void init(uint8_t type, uint16_t samples);
    void set4WirePT100();
    void set3WirePT100();
    void set3WireIDAC();
    void invert3WireIDAC();
    void startContinuous(voidFuncPtr itFunction);
    void setTempCalibration();
    void calRefResistor(unsigned int resistanceValue);
    void calTempProbe(float offset);
    //void interruptADC();
  };
#endif