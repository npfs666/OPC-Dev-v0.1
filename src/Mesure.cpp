#include <pinout.h>
#include "Mesure.h"


RTDSensor::RTDSensor() {}
/**
 * @brief Construct a new RTDSensor::RTDSensor object
 * 
 * @param type 3 or 4 Wire type
 * @param switchPin Mux associated pin
 * @param samples 4 samples -> 1bit improve, 16 -> 2bits, 64 -> 3bits, 256 -> 4bits (oversampling)
 * @param offset Sensor offset in °C
 */
RTDSensor::RTDSensor(uint8_t type, uint8_t switchPin, uint16_t samples, float_t offset) {

    this->measurementType = type;
    this->analogSwitchPin = switchPin;
    this->samples = samples;
    this->offset = offset;
    this->reset();
}
void RTDSensor::add(int32_t value)
{
    sum += value;
    sampleCount++;
}
void RTDSensor::reset()
{
    sum = 0;
    sampleCount = 0;
}
void RTDSensor::compute()
{
    avgValue = sum / (samples / 1.0);
    this->reset();
}
double_t RTDSensor::readValue()
{
    return avgValue;
}







ADC::ADC()
{
}
/**
 * @brief Variables init and ADC init
 */
void ADC::init()
{
    this->numRTDSensors = 0;
    this->newMeasurement = false;
    ads1120.begin(SPI_CLK, SPI_MISO, SPI_MOSI, SPI_CS, SPI_DRDY);
}

/**
 * @brief Adds a sensor to the list
 * 
 * @param number id
 * @param type TYPE_2WIRE TYPE_3WIRE TYPE_4WIRE
 * @param switchPin mux pin
 * @param samples 4 samples -> 1bit improve, 16 -> 2bits, 64 -> 3bits, 256 -> 4bits (oversampling)
 * @param offset Sensor offset
 */
void ADC::addRTD(uint8_t number, uint8_t type, uint8_t switchPin, uint16_t samples, float_t offset) {

    this->rtd[number] = RTDSensor(type, switchPin, samples, offset);
    this->numRTDSensors++;
}

/**
 * @brief Configures the ADC to measure a 4-Wire RTD
 *
 */
void ADC::set4WirePT100()
{
    ads1120.setConversionMode(CONVERSION_SINGLE_SHOT);
    digitalWrite(SW_3_WIRE, LOW);
    digitalWrite(SW_4_WIRE, HIGH);
    ads1120.setMultiplexer(MUX_AINP_AIN0_AINN_AIN1);
    ads1120.setFIR(FIR_50HZ);
    ads1120.setVoltageRef(VREF_EXTERNAL_REFP0_REFN0);
    ads1120.setIDAC1routing(IDAC_AIN3_REFN1);
    ads1120.setIDACcurrent(CURRENT_1000_UA);
    ads1120.setGain(8);
}
/**
 * @brief Configures the ADC to measure a 3-Wire RTD
 *
 */
void ADC::set3WirePT100()
{
    ads1120.setConversionMode(CONVERSION_SINGLE_SHOT);
    digitalWrite(SW_3_WIRE, HIGH);
    digitalWrite(SW_4_WIRE, LOW);
    ads1120.setMultiplexer(MUX_AINP_AIN0_AINN_AIN1);
    ads1120.setFIR(FIR_50HZ);
    ads1120.setVoltageRef(VREF_EXTERNAL_REFP0_REFN0);
    ads1120.setIDAC1routing(IDAC_AIN3_REFN1);
    ads1120.setIDAC2routing(IDAC_AIN2);
    ads1120.setIDACcurrent(CURRENT_500_UA);
    ads1120.setGain(16);
}

// Inverse les sources de courant (current chopping) pour annuler leurs inexactitudes
void ADC::invert3WireIDAC()
{
    //ads1120.setConversionMode(CONVERSION_SINGLE_SHOT); // Stop conversion in case
    ads1120.setIDAC1routing(IDAC_AIN2);
    ads1120.setIDAC2routing(IDAC_AIN3_REFN1);
    //ads1120.setConversionMode(CONVERSION_CONTINUOUS);
    //ads1120.startSync();
}

// Lance une conversion continue, gérée par interruption
void ADC::startContinuous()
{
    // Préparation de la première lecture
    curRTDSensor = 0;
    digitalWrite(rtd[curRTDSensor].analogSwitchPin, HIGH);
    if (rtd[curRTDSensor].measurementType == TYPE_3WIRE) {
        set3WirePT100();
    } else if(rtd[curRTDSensor].measurementType == TYPE_4WIRE) {
        set4WirePT100();
    }

    delay(1);
    ads1120.setDataRate(DATARATE_90_SPS);
    ads1120.setConversionMode(CONVERSION_CONTINUOUS);
    ads1120.startSync();
}

// Met en pause la conversion continue
void ADC::stop() {
    ads1120.setConversionMode(CONVERSION_SINGLE_SHOT);

    // Reset des multiplexeurs d'entrées
    for( uint8_t i = 0; i < numRTDSensors; i++ )  {
        digitalWrite(rtd[curRTDSensor].analogSwitchPin, LOW);
    }
}

// Relance une conversion, continue avec les anciens paramètres
void ADC::restart() {
    ads1120.setConversionMode(CONVERSION_CONTINUOUS);
    ads1120.startSync();
}

// Reset les sommes des sondes et leur boucle de mesure à 0
void ADC::resetCounts() {
    for( uint8_t i = 0; i < numRTDSensors; i++ )  {
        rtd[i].reset();
    }
}


void ADC::calRefResistor(double_t resistanceValue)
{
}

/**
 * @brief Convertit une valeur entière sur 16bits en Resistance en fonction
 * de la valeur de la résistance de calibration refResistanceValue.
 * 
 * @param rtdSensor ID du capteur
 * @param systemTemperature température actuelle du système
 * @return double_t Résistance en Ohms
 */
double_t ADC::getResistanceValue(uint8_t rtdSensor, float_t systemTemperature) {

    #define TEMPERATURE_COEFFICIENT_PPM_C 9 // ancien calcul 7.5
    #define TEMPERATURE_AT_CALIBRATION 24.0

    double_t gain = 1;
    if( rtd[rtdSensor].measurementType == TYPE_3WIRE ) {
        gain = 8.0;
    } else if (rtd[rtdSensor].measurementType == TYPE_4WIRE ) {
        gain = 8.0;
    }

    double_t Rrtd = (rtd[rtdSensor].avgValue * refResistanceValue) / (32767.0 * gain);

    // Compensation de la mesure 
    // 7.5ppm mesuré (système entier) avec la diff entre la plage 24°C et 12°C
    float_t ppm = (systemTemperature - TEMPERATURE_AT_CALIBRATION) * TEMPERATURE_COEFFICIENT_PPM_C;
    Rrtd = Rrtd * (1 + ppm/1000000);

    return Rrtd;
}

/**
 * @brief Conversion d'une resistance en température via le calcul par interpolation (plus précis qu'une fonction pour une approche réelle)
 * 
 * @param id sensor ID
 * @param systemTemperature actual temperature of the board
 * @return double_t temperature in °C
 */
double_t ADC::getRTDTempInterpolation(uint8_t id, float_t systemTemperature) {

    double_t Rrtd = this->getResistanceValue(id, systemTemperature);

    int16_t index=(int16_t) (Rrtd/10);
    double_t frac = (double_t)(Rrtd/10.0) - index;
    double_t a = rtdInterpol[index];

    double_t temperature = 0;

    // Si valeur juste, on lis la case directement
    if (index == Rrtd / 10)
    {
        temperature = rtdInterpol[index];
    }
    // Sinon approximation par interpolation du des valeurs du tableau
    double_t b = rtdInterpol[index + 1] / 2.0;
    double_t c = rtdInterpol[index - 1] / 2.0;
    temperature = (double_t)a + frac * (b - c + frac * (c + b - a));

    return temperature + rtd[id].offset;
}

/**
 * @brief Calcule l'humidité relative à partir d'une température sèche et humide
 * 
 * @param tempSeche température de la sonde sèche en °C
 * @param tempHumide température du bulbe humide en °C
 * @param pressionAtm en KPa
 * @return double_t Humidité relative en %
 */
double_t ADC::getRH(double_t tempSeche, double_t tempHumide, double_t pressionAtm) {

    // 1: Calcul de la "constante" psychrométrique
    // Capacité thermique massique de l'air [kJ/kg.°C]
    // 0.00006 * tempSeche + 1.005; ancien calcul
    double_t Cp = (3 * tempSeche)/50000.0 + 1.005;
    // Energie de vaporiation de l'eau [kJ/kg]
    double_t lambda = -2.3664 * tempSeche + 2501;
    double_t A = Cp / (lambda * 0.622 ); // [1/°C]

    // Pression athmosphérique [kPa]
	double_t P = pressionAtm;

    double_t pVs = 0.6108 * pow(2.71828, ((17.27 * tempHumide)/(tempHumide + 237.3))); // [kPa]
    double_t pV = pVs - A*P*(tempSeche-tempHumide); // [kPa]
    double_t pVs2 = 0.6108 * pow(2.71828, ((17.27 * tempSeche)/(tempSeche + 237.3))); // [kPa]

    // Ancien calcul
    //double_t pVs = pow(10,(2.7877+(7.625*mes1)/(241.6+mes1)));
    //double_t pV = pVs - 0.000667*102.3000*(mes2-mes1);
    //double_t pVs2 = pow(10,(2.7877+(7.625*mes2)/(241.6+mes2)));

    double_t rh = ((double_t)pV/pVs2)*100.0;

    return rh;
}

