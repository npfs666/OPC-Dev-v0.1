#ifndef MESURE_h
#define MESURE_h

#include "ADS1120.h"

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
    float_t offset;

    RTDSensor();
    RTDSensor(uint8_t measurementType, uint8_t analogSwitchPin, uint16_t samples, float_t offset);
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
    void addRTD(uint8_t number, uint8_t type, uint8_t switchPin, uint16_t samples, float_t offset);
    void startContinuous();
    void stop();
    void restart();
    void resetCounts();
    void calRefResistor(double_t resistanceValue);
    double_t getRH(double_t tempSeche, double_t tempHumide);
    double_t getResistanceValue(uint8_t id);
    double_t getRTDTempQuadratic(double_t rtd);
    double_t getRTDTempInterpolation(uint8_t id);
    

    void set4WirePT100();
    void set3WirePT100();
    void invert3WireIDAC();
  };



class Regulateur {

  #define Chauffer 1
  #define Refroidir 2

  public:

    float_t SP;
    uint8_t mode;

};

class Regulation {
  
  public:

    Regulateur regulateur[2];
};
#endif