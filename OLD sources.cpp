class MesureADC
{
	// Données
public:
	double somme;
	const int SAMPLES = 100.0;
	uint16_t sampleCount;
	bool newMes = false;

	MesureADC()
	{
		this->reset();
	};

	void reset()
	{
		somme = 0;
		sampleCount = 0;
	};

	void add(int value)
	{
		somme += value;
		// Serial.println("  "+(String)value);
		sampleCount++;
	}

	double compute()
	{
		double res = somme / (SAMPLES / 1.0);
		// Serial.println("  "+(String)somme+ " " +SAMPLES+"  "+res);
		this->newMes = true;
		this->reset();
		return res;
	}
};






void initADCContinuous()
{
	adc.setDataRate(DATARATE_45_SPS);
	adc.setConversionMode(CONVERSION_CONTINUOUS);
	adc.setMultiplexer(MUX_AINP_AIN0_AINN_AIN1);
	adc.setFIR(FIR_50HZ);
	adc.setVoltageRef(VREF_EXTERNAL_REFP0_REFN0);
	adc.setIDAC1routing(IDAC_AIN3_REFN1);
	adc.setGain(8);
	adc.setIDACcurrent(CURRENT_1000_UA);
	attachInterrupt(digitalPinToInterrupt(8), adcInterrupt, FALLING);
	adc.startSync();
	// DEBUG CHECK REGISTER
	// Serial.println((String)adc.readRegister(0) + " " + (String)adc.readRegister(1) + " " + (String)adc.readRegister(2));
}

void initADCSingle()
{
	adc.setDataRate(DATARATE_20_SPS);
	adc.setConversionMode(CONVERSION_SINGLE_SHOT);
	adc.setMultiplexer(MUX_AINP_AIN0_AINN_AIN1);
	adc.setFIR(FIR_50HZ);
	adc.setVoltageRef(VREF_EXTERNAL_REFP0_REFN0);
	adc.setIDAC1routing(IDAC_AIN3_REFN1);
	adc.setGain(8);
	//adc.setTemperatureMode(0x01);
	// DEBUG CHECK REGISTER
	// Serial.println((String)adc.readRegister(0) + " " + (String)adc.readRegister(1) + " " + (String)adc.readRegister(2));
}



double code;
void adcInterrupt()
{

	int val = adc.readADC();
	mesure.add(val);
	// delay(1000);
	// Serial.println(val);
	// delay(1000);

	if (mesure.sampleCount == mesure.SAMPLES)
	{
		code = mesure.compute();
	}
}















/**
 * @file main.cpp
 *
 *
 *
 * @author GAOU (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-02-07
 *
 * @copyright Copyright (c) 2022
 *
 *
 *
 * Code a enlever du SPI.cpp pour virer le TinyUSB qui marche pas
// * #ifdef USE_TINYUSB
// For Serial when selecting TinyUSB.  Can't include in the core because Arduino IDE
// will not link in libraries called from the core.  Instead, add the header to all
// the standard libraries in the hope it will still catch some user cases where they
// use these libraries.
// See https://github.com/earlephilhower/arduino-pico/issues/167#issuecomment-848622174
//#include <Adafruit_TinyUSB.h>
//#endif
 */

//#include <Arduino.h>
#include <SPI.h>
#include <ADC.h>




//void adcInterrupt();
void rotencInterrupt(void);
//ADS1120 adc;
//MesureADC mesure;
//double moyenneADC = 0;
ADC adc;



void setup()
{

	delay(3000);
	Serial.begin(115200);
	Serial.println("ADS1120 Protoboard");

	adc.begin(SCK, MISO, MOSI, SS, 8);

	// initADCContinuous();
	//initADCSingle();

	pinMode(14, INPUT);
	pinMode(15, INPUT);
	attachInterrupt(digitalPinToInterrupt(14), rotencInterrupt, FALLING);

	// pinMode(LED_BUILTIN, OUTPUT);
}

void rotencInterrupt(void) {
	;
	Serial.println(digitalRead(15));
}


void loop()
{
	// One shot
	/*adc.setIDACcurrent(6);
	delay(1000);
	int val = adc.readADC_Single();
	Serial.println(val);
	adc.setIDACcurrent(0);*/

	//Serial.print("Internal temp  : ");
	//Serial.println(adc.readInternalTemp(), 1);

	delay(1000);
	Serial.println("okok");

	if (mesure.newMes)
	{

		double res = code;

		// Application de l'erreur PGA
		// res += 1;

		// res -= 80; // Calibration de l'adc via resistance fixe
		// double Rrtd = (res * 1658.4) / (32767 * 8.0);
		double Rrtd = (res * 1650.24) / (32767 * 8.0);
		// Rrtd *= 2;

		double calib0CRTD = 100.04; // Valeur de la resistance @ 0°C

		// double temp = (Rrtd-calib0CRTD)/0.385;	// Approximation linéaire de la T°C
		double temp = -244.83 + Rrtd * (2.3419 + 0.0010664 * Rrtd);

		Serial.print((String)res + " ; ");
		Serial.print(Rrtd, 3);
		Serial.print(" ; ");
		Serial.println(temp, 3);
		// Serial.print(";");
		// Serial.println(millis()/1000);

		mesure.newMes = false;
	}
}


