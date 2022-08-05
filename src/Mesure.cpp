
#include "SPI.h"
#include "Arduino.h"
#include "Mesure.h"



RTDSensor::RTDSensor() {
}

RTDSensor::RTDSensor(uint8_t type, uint8_t switchPin, uint16_t samples, int16_t offset) {

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
 * @brief 
 * 
 * @param type 
 * @param samples 4 samples -> 1bit improve, 16 -> 2bits, 64 -> 3bits, 256 -> 4bits
 */
void ADC::init()
{
    //this->measurementSamples = samples;
    this->numRTDSensors = 0;
    this->newMeasurement = false;
    ads1120.begin(SPI_CLK, SPI_MISO, SPI_MOSI, SPI_CS, SPI_DRDY);
}

void ADC::addRTD(uint8_t number, uint8_t type, uint8_t switchPin, uint16_t samples, int16_t offset) {

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
    //ads1120.setIDAC2routing(IDAC_AIN3_REFN1);
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
void ADC::invert3WireIDAC()
{
    //ads1120.setConversionMode(CONVERSION_SINGLE_SHOT); // Stop conversion in case
    ads1120.setIDAC1routing(IDAC_AIN2);
    ads1120.setIDAC2routing(IDAC_AIN3_REFN1);
    //delay(5);
}
void ADC::startContinuous()
{
    // Préparation de la première lecture
    this->curRTDSensor = 0;
    digitalWrite(rtd[this->curRTDSensor].analogSwitchPin, HIGH);
    if (rtd[this->curRTDSensor].measurementType == TYPE_3WIRE) {
        set3WirePT100();
    } else if(rtd[this->curRTDSensor].measurementType == TYPE_4WIRE) {
        set4WirePT100();
    }

    ads1120.setConversionMode(CONVERSION_CONTINUOUS);
    ads1120.setDataRate(DATARATE_90_SPS);
    ads1120.startSync();
}

void ADC::stop() {
    ads1120.setConversionMode(CONVERSION_SINGLE_SHOT);
}

void ADC::restart() {
    ads1120.setConversionMode(CONVERSION_CONTINUOUS);
    ads1120.startSync();
}


void ADC::calRefResistor(double_t resistanceValue)
{
}


double_t ADC::getResistanceValue(uint8_t rtdSensor) {

    double_t gain = 1;
    if( rtd[rtdSensor].measurementType == TYPE_3WIRE ) {
        gain = 8.0;
    } else if (rtd[rtdSensor].measurementType == TYPE_4WIRE ) {
        gain = 8.0;
    }

    double_t Rrtd = (rtd[rtdSensor].avgValue * refResistanceValue) / (32767.0 * gain);

    return Rrtd;
}



double_t ADC::getRTDTempInterpolation(double_t Rrtd) {

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

    return temperature;
}

