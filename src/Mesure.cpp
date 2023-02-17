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

    #define ALPHA 0.6
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
/**
 * @brief Low pass EMA (exponential moving average) Filter
 * 
 * @param value 
 */
void RTDSensor::addLP(int32_t value)
{
    if( this->val == 0 ) 
        this->val = value;
    else
        this->val = (double_t) ((ALPHA * value) + (1.0 - ALPHA) * this->val);
    //Serial.print(this->val,2); Serial.print(" | ");Serial.println(value);
    //Serial.println(this->val);
    sum += this->val;
    sampleCount++;
}
void RTDSensor::reset()
{
    sum = 0;
    sampleCount = 0;
    val = 0;
}
void RTDSensor::compute()
{
    avgValue = (double_t) sum / samples;
    //Serial.print(sum);Serial.println("  |  ");
    this->reset();
}
double_t RTDSensor::readValue()
{
    return avgValue;
}



Measurement::Measurement()
{
}
Measurement::Measurement(uint8_t rtdIndex)
{
    type = MEASUERMENT_TEMPERATURE;
    rtd1 = rtdIndex;
}
/**
 * @brief Variables init and ADC init
 */
Measurement::Measurement(uint8_t rtdIndexDry, uint8_t rtdIndexWet)
{
    type = MEASUERMENT_RELATIVE_HUMIDITY;
    rtd1 = rtdIndexDry;
    rtd2 = rtdIndexWet;
}



SensorBoard::SensorBoard()
{
}
/**
 * @brief Variables init and ADC init
 */
void SensorBoard::init()
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
void SensorBoard::addRTD(uint8_t type, uint8_t switchPin, uint16_t samples, float_t offset) {

    rtd[numRTDSensors] = RTDSensor(type, switchPin, samples, offset);
    numRTDSensors++;
}

/**
 * @brief Configures the ADC to measure a 4-Wire PT-100 RTD
 *
 */
void SensorBoard::set4WirePT100()
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
 * @brief Configures the ADC to measure a 3-Wire PT-100 RTD
 *
 */
void SensorBoard::set3WirePT100()
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


/**
 * @brief Invert the IDAC current source in 3-Wire measurement, to cancel their différences (current chopping)
 * 
 */
void SensorBoard::invert3WireIDAC()
{
    ads1120.setConversionMode(CONVERSION_SINGLE_SHOT); // Stop conversion
    ads1120.setIDAC1routing(IDAC_AIN2);
    ads1120.setIDAC2routing(IDAC_AIN3_REFN1);
    delay(10);
    restart();
}

// Starts continuous conversion of the ADC
void SensorBoard::startContinuous()
{
    // Init first read
    curRTDSensor = 0;
    digitalWrite(rtd[curRTDSensor].analogSwitchPin, HIGH);  // Input analog switch ON
    if (rtd[curRTDSensor].measurementType == TYPE_3WIRE) {
        set3WirePT100();
    } else if(rtd[curRTDSensor].measurementType == TYPE_4WIRE) {
        set4WirePT100();
    }

    delay(10);
    ads1120.setDataRate(DATARATE_20_SPS);   // No 50/60Hz filtering above 20 SPS
    ads1120.setConversionMode(CONVERSION_CONTINUOUS);
    ads1120.startSync();
}

// Pauses conversion
void SensorBoard::stop() {
    ads1120.setConversionMode(CONVERSION_SINGLE_SHOT);

    // Reset input analog switches
    for( uint8_t i = 0; i < numRTDSensors; i++ )  {
        digitalWrite(rtd[curRTDSensor].analogSwitchPin, LOW);
    }
}

// Restarts conversion
void SensorBoard::restart() {
    ads1120.setConversionMode(CONVERSION_CONTINUOUS);
    ads1120.startSync();
}

// Resets RTD sums
void SensorBoard::resetCounts() {
    for( uint8_t i = 0; i < numRTDSensors; i++ )  {
        rtd[i].reset();
    }
}

// Ne pas faire la calibration dans une interrupt, ça ne fonctionne pas avec les delays
// à caler à un endroit et terminer
void SensorBoard::calRefResistor()
{
    #define CALIBRATION_SAMPLES 64.0

    digitalWrite(rtd[0].analogSwitchPin, HIGH);
    set4WirePT100();

    int32_t sum = 0;    

    for( uint8_t i = 0; i < CALIBRATION_SAMPLES; i++ ) {
        delay(10);
        sum += ads1120.readADC_Single();
    }
    digitalWrite(rtd[0].analogSwitchPin, LOW);
    double_t readVal = sum / CALIBRATION_SAMPLES ;

    refResistanceValue = (calResistanceValue * 32768.0 * 8.0 ) / readVal;
    calTemperature = ads1120.readInternalTemp();

    Serial.print(readVal,2);Serial.print(" | ");
    Serial.print(refResistanceValue,3);Serial.print(" | ");
    Serial.println(calTemperature,2);
}

void SensorBoard::convertToTemperature(float_t systemTemperature) {

    // Navigation dans les sondes déclarées
    for( uint8_t i = 0; i < this->numRTDSensors; i++ ) {
      
        rtd[i].resistance = getResistanceValue(i, systemTemperature);
        rtd[i].temperature = getRTDTempInterpolation(i);
    } 
}



/**
 * @brief Convertit une valeur entière sur 16bits en Resistance en fonction
 * de la valeur de la résistance de calibration refResistanceValue.
 * 
 * @param rtdSensor sensor index
 * @param systemTemperature température actuelle du système
 * @return double_t Résistance en Ohms
 */
double_t SensorBoard::getResistanceValue(uint8_t rtdSensor, float_t systemTemperature) {

    #define TEMPERATURE_COEFFICIENT_PPM_C 7.5 // ancien calcul 7.5
    //#define TEMPERATURE_AT_CALIBRATION 25.4

    double_t gain = 1;
    if( rtd[rtdSensor].measurementType == TYPE_3WIRE ) {
        gain = 8.0;
    } else if (rtd[rtdSensor].measurementType == TYPE_4WIRE ) {
        gain = 8.0;
    }

    double_t Rrtd = (rtd[rtdSensor].avgValue * refResistanceValue) / (32768.0 * gain);

    // Compensation de la mesure 
    // 7.5ppm mesuré (système entier) avec la diff entre la plage 24°C et 12°C
    //double_t ppm = (systemTemperature - calTemperature) * TEMPERATURE_COEFFICIENT_PPM_C;
    //Rrtd = Rrtd * (1 + ppm/1000000.0);

    return Rrtd;
}

/**
 * @brief Conversion d'une resistance en température via le calcul par interpolation (plus précis qu'une fonction pour une approche réelle)
 * 
 * @param id sensor index
 * @return double_t temperature in °C
 */
double_t SensorBoard::getRTDTempInterpolation(uint8_t id) {

    double_t Rrtd = rtd[id].resistance;

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
 * @param pressionAtm en Pa
 * @return double_t Humidité relative en %
 */
double_t SensorBoard::getRH(uint8_t dryID, uint8_t wetID, double_t atmPressure) {

    double_t dryTemperature = rtd[dryID].temperature;
    double_t wetTemperature = rtd[wetID].temperature;

    // 1: Calcul de la "constante" psychrométrique
    // Capacité thermique massique de l'air [kJ/kg.°C]
    // 0.00006 * tempSeche + 1.005; ancien calcul
    double_t Cp = (3 * dryTemperature)/50000.0 + 1.005;
    // Energie de vaporiation de l'eau [kJ/kg]
    double_t lambda = -2.3664 * wetTemperature + 2501;
    double_t A = Cp / (lambda * 0.622 ); // [1/°C]

    // Atmospheric pressure [kPa]
	double_t P = atmPressure / 1000.0F;

    double_t pVs = 0.6108 * pow(2.71828, ((17.27 * wetTemperature)/(wetTemperature + 237.3))); // [kPa]
    double_t pV = pVs - A*P*(dryTemperature-wetTemperature); // [kPa]
    double_t pVs2 = 0.6108 * pow(2.71828, ((17.27 * dryTemperature)/(dryTemperature + 237.3))); // [kPa]

    // Ancien calcul
    //double_t pVs = pow(10,(2.7877+(7.625*mes1)/(241.6+mes1)));
    //double_t pV = pVs - 0.000667*102.3000*(mes2-mes1);
    //double_t pVs2 = pow(10,(2.7877+(7.625*mes2)/(241.6+mes2)));

    double_t rh = ((double_t)pV/pVs2)*100.0;

    return rh;
}








