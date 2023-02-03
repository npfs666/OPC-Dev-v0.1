#ifndef MESURE_h
#define MESURE_h

#include "ADS1120.h"

/* Array of fixed values RTD interpolation
  each line is +10 Ohms, starting from 0 (0 Ohms <-> -500°C, 10 <-> -219.415, ...)
*/
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
    int32_t sum;
    uint16_t samples, sampleCount;
    float_t offset;
    double_t resistance;
    double_t temperature;

    RTDSensor();
    RTDSensor(uint8_t measurementType, uint8_t analogSwitchPin, uint16_t samples, float_t offset);
    void reset();
    void add(int32_t value);
    void compute();
    double_t readValue();
    
};


/**
 * @brief 
 * Il faudrait pouvoir rajouter des régulateurs comme pour les sondes.
 * addReg(TEMP OU RH, régulation positive ou négative (chauffer/humidifier) & (refroidir/déshum), )
 * 
 * il faudrait passer par une classe intermédiaire pour gérer les entrée (T°c ou RH), pour en avoir qu'une à gérer
 *  -> étape 1 faire une classe d'entrées/lectures
 *  -> ex : addMeasurement(RH ou T°C, RTD1, RTD2, ...)
 *  -> on va donc retrouver un tableau de grandeurs mesurées
 * 
 * ensuite on peut rajouter un régulateur sur une grandeur mesurée et c'est configurable par le menu utilisateur
 */
class Regulateur {


  #define MODE_HEATING 1
  #define MODE_COOLING 2

  public:

    float_t SP;
    uint8_t mode;
    float_t hysteresis;

};

class Measurement {

  #define MEASUERMENT_TEMPERATURE 1
  #define MEASUERMENT_RELATIVE_HUMIDITY 2
  public:
  uint8_t rtd1;
  uint8_t rtd2;
  uint8_t type;
  float_t value;

    Measurement();
    Measurement(uint8_t rtdIndex);
    Measurement(uint8_t rtdIndexDry, uint8_t rtdIndexWet);
};



class SensorBoard {

  public:
    double_t refResistanceValue = 1649.735; // Value of Rref after calibration
    double_t calResistanceValue = 100.057;
    double_t calTemperature = 25.4;

    uint8_t numRTDSensors;  // number of RTD sensors
    uint8_t curRTDSensor;   // cur sensor index
    bool newMeasurement;    // ADC has finished accumulating values, data is readable
    
    ADS1120 ads1120;  // ADC
    RTDSensor rtd[3]; // Sensor array
    Regulateur reg[2];// Regulator array

    
    SensorBoard();
    void init();
    void addRTD(uint8_t type, uint8_t switchPin, uint16_t samples, float_t offset);
    //void addMeasurement(uint8_t type, uint8_t rtd1, uint8_t rtd2);
    void startContinuous();
    void stop();
    void restart();
    void resetCounts();
    void calRefResistor();
    void convertToTemperature(float_t systemTemperature);
    double_t getRH(uint8_t dryID, uint8_t wetID, double_t atmPressure3);
    double_t getResistanceValue(uint8_t id, float_t systemTemperature);
    double_t getRTDTempQuadratic(double_t rtd);
    double_t getRTDTempInterpolation(uint8_t id);
    void set4WirePT100();
    void set3WirePT100();
    void invert3WireIDAC();
  };

#endif