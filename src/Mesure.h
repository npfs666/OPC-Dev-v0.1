#ifndef MESURE_h
#define MESURE_h

#include "ADS1120.h"

#define SPI_CLK  18
#define SPI_MISO 16
#define SPI_MOSI 19
#define SPI_CS   17
#define SPI_DRDY 20

#define SW_3_WIRE 28
#define SW_4_WIRE 27

const double rtdInterpol[] = {-500.00, 
							-219.415, -196.509, -173.118, -149.304, -125.122, -100.617, -75.827, -50.781, -25.501, 0.000,
							  25.686, 51.571, 77.660, 103.958, 130.469, 157.198, 184.152, 211.336, 238.756, 266.419};

class RTDSensor {

    #define TYPE_2WIRE 2
    #define TYPE_3WIRE 3
    #define TYPE_4WIRE 4

    public:
    uint8_t measurementType;
    uint8_t analogSwitchPin;
    double_t avgValue;
    int64_t sum;
    uint16_t samples, sampleCount;
    int16_t offset;
    uint8_t gain;

    RTDSensor();
    RTDSensor(uint8_t measurementType, uint8_t analogSwitchPin, uint16_t samples, int16_t offset);
    void reset();
    void add(int32_t value);
    void compute();
    double_t readValue();
    
};



class ADC {

  public:
    uint8_t measurementSamples;
    uint8_t numRTDSensors;
    uint8_t curRTDSensor;
    double_t refResistanceValue = 1649.735;
    ADS1120 ads1120;
    RTDSensor rtd[3];
    bool newMeasurement;
    

    ADC();
    void init();
    void addRTD(uint8_t number, uint8_t type, uint8_t switchPin, uint16_t samples, int16_t offset);
    void startContinuous();
    void stop();
    void restart();
    void calRefResistor(double_t resistanceValue);
    double_t getResistanceValue(uint8_t value);
    double_t getRTDTempQuadratic(double_t rtd);
    double_t getRTDTempInterpolation(double_t rtd);

    void set4WirePT100();
    void set3WirePT100();
    void invert3WireIDAC();
  };
#endif