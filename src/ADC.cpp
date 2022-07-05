#include "ADC.h"
#include "SPI.h"
#include "Arduino.h"



RTDSensor::RTDSensor() {
    reset();
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
    value = sum / (samples / 1.0);
    newMeasure = true;
    this->reset();
}
double_t RTDSensor::readValue()
{
    newMeasure = false;
    return value;
}

ADC::ADC()
{
    rtd.reset();
    gain=0;
}
/**
 * @brief 
 * 
 * @param type 
 * @param samples 4 samples -> 1bit improve, 16 -> 2bits, 64 -> 3bits, 256 -> 4bits
 */
void ADC::init(uint8_t type, uint16_t samples)
{
    rtd.type = type;
    rtd.samples = samples;
    ads1120.begin(SPI_CLK, SPI_MISO, SPI_MOSI, SPI_CS, SPI_DRDY);
}
/**
 * @brief Configures the ADC to measure a 4-Wire RTD
 *
 */
void ADC::set4WirePT100()
{
    ads1120.setConversionMode(CONVERSION_SINGLE_SHOT);
    ads1120.setMultiplexer(MUX_AINP_AIN0_AINN_AIN1);
    ads1120.setFIR(FIR_50HZ);
    ads1120.setVoltageRef(VREF_EXTERNAL_REFP0_REFN0);
    ads1120.setIDAC1routing(IDAC_AIN3_REFN1);
    //ads1120.setIDAC2routing(IDAC_AIN3_REFN1);
    ads1120.setIDACcurrent(CURRENT_1000_UA);
    ads1120.setGain(8);
    gain=8;
}
/**
 * @brief Configures the ADC to measure a 3-Wire RTD
 *
 */
void ADC::set3WirePT100()
{
    ads1120.setConversionMode(CONVERSION_SINGLE_SHOT);
    ads1120.setMultiplexer(MUX_AINP_AIN0_AINN_AIN1);
    ads1120.setFIR(FIR_50HZ);
    ads1120.setVoltageRef(VREF_EXTERNAL_REFP0_REFN0);
    ads1120.setIDACcurrent(CURRENT_500_UA);
    ads1120.setGain(16);
    gain=16;
    //set3WireIDAC();
}
void ADC::set3WireIDAC()
{
    //ads1120.setConversionMode(CONVERSION_SINGLE_SHOT); // Stop conversion in case
    ads1120.setIDAC1routing(IDAC_AIN3_REFN1);
    ads1120.setIDAC2routing(IDAC_AIN2);
    //delay(5);
}
void ADC::invert3WireIDAC()
{
    //ads1120.setConversionMode(CONVERSION_SINGLE_SHOT); // Stop conversion in case
    ads1120.setIDAC1routing(IDAC_AIN2);
    ads1120.setIDAC2routing(IDAC_AIN3_REFN1);
    //delay(5);
}
void ADC::startContinuous(voidFuncPtr itFunction)
{
    ads1120.setConversionMode(CONVERSION_CONTINUOUS);
    ads1120.setDataRate(DATARATE_45_SPS);
    attachInterrupt(digitalPinToInterrupt(SPI_DRDY), itFunction, FALLING);
    ads1120.startSync();
}
void ADC::setTempCalibration()
{
}
void ADC::calRefResistor(unsigned int resistanceValue)
{
}
void ADC::calTempProbe(float offset)
{
}

uint8_t ADC::getGain()
{
    return gain;
}


/*
void interruptADC(ADC *t)
{
    t->rtd.add(t->ads1120.readADC());

    // If 3 wire, chopp the current sources
    if ((t->rtd.type == TYPE_3WIRE) && (t->rtd.sampleCount == (t->rtd.samples / 2)))
    {
        t->invert3WireIDAC();
        t->ads1120.startSync();
    }

    // If all samples are measured, compute the result
    if (t->rtd.sampleCount == t->rtd.samples)
    {
        t->rtd.compute();

        if (t->rtd.type == TYPE_3WIRE)
        {
            t->set3WireIDAC();
            t->ads1120.startSync();
        }
    }
}*/