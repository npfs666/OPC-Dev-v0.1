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

#include <SPI.h>
#include <ADC.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_ST7789.h>
#include "pico/stdlib.h"
#include <math.h>
//#include "hardware/timer.h"


// Ecran SPI TFT
#define LCD_SCK 14
#define LCD_MOSI 15
#define LCD_DC 12
#define LCD_CS 13
#define LCD_RESET 11
Adafruit_ST7789 tft = Adafruit_ST7789(&SPI1, LCD_CS, LCD_DC, LCD_RESET);

// Encodeur rotatif
#define ROTENC_A 26
#define ROTENC_B 22
#define ROTENC_CLIC 21

// Analog switches
#define SW_3_WIRE 28
#define SW_4_WIRE 27
#define SW_MUX_1 2
#define SW_MUX_2 1
#define SW_MUX_3 0

void rotencInterrupt();
void rotencInterruptClic();
void adcInterrupt();

ADC adc;
bool mux_1 = true;
double_t mes1=1, mes2=1;

const double rtdInterpol[] = {-500.00, 
							-219.415, -196.509, -173.118, -149.304, -125.122, -100.617, -75.827, -50.781, -25.501, 0.000,
							  25.686, 51.571, 77.660, 103.958, 130.469, 157.198, 184.152, 211.336, 238.756, 266.419};


void setup()
{
	delay(2000);
	Serial.begin(115200);
	Serial.println("Open Process Controller");

	// Configuration des pins de commandes des analog switches
	pinMode(SW_3_WIRE, OUTPUT);
	pinMode(SW_4_WIRE, OUTPUT);
	pinMode(SW_MUX_1, OUTPUT);
	pinMode(SW_MUX_2, OUTPUT);
	pinMode(SW_MUX_3, OUTPUT);

	digitalWrite(SW_4_WIRE, HIGH);
	digitalWrite(SW_MUX_1, HIGH);
	mux_1 = true;
	delay(20);

	/*adc.init(TYPE_3WIRE, 64);
	adc.set3WirePT100();
	adc.set3WireIDAC();*/
	adc.init(TYPE_4WIRE, 64);
	adc.set4WirePT100();
	adc.startContinuous(adcInterrupt);
}

void setup1()
{
	delay(2000);

	// Configuration de l'encodeur rotatif
	pinMode(ROTENC_A, INPUT);
	pinMode(ROTENC_B, INPUT);
	attachInterrupt(digitalPinToInterrupt(ROTENC_A), rotencInterrupt, FALLING);
	attachInterrupt(digitalPinToInterrupt(ROTENC_CLIC), rotencInterruptClic, FALLING);

	// Configuration du SPI de l'écran
	SPI1.setSCK(LCD_SCK);
	SPI1.setTX(LCD_MOSI);
	tft.init(135, 240); // Init ST7789 240x240
	tft.setRotation(3);
	tft.fillScreen(ST77XX_BLACK);

	// Test initial
	/*tft.setTextColor(ST77XX_WHITE); // Draw white text
	tft.cp437(true);				 // Use full 256 char 'Code Page 437' font
	tft.setTextSize(2);
	tft.setCursor(0, 0); // Start at top-left corner
	tft.printf("Consigne");*/
}

/**
 * Cpu1 : contrôle des IT utilisateur et écran
 * @brief
 *
 */
//uint32_t  start = to_us_since_boot(get_absolute_time());
//uint32_t  temps = to_us_since_boot(get_absolute_time()) - start;
//Serial.println(temps);
void loop1()
{

	if (adc.rtd.newMeasure)
	{
		// lecture de la dernière série de valeurs
		double_t Rrtd = 0;
		double_t tempInter = 0;

		if( !mux_1 ) {
			mes2 = adc.rtd.readValue();
			
			Rrtd = (mes2 * 1649.735) / (32767 * (double_t)adc.getGain());
		
			// Application d'un offset de T°C
			//double_t calib0CRTD = 100.04; // Valeur de la resistance @ 0°C

			// Conversion de la résistance d'une RTD en température via la méthode d'interpolation
			int16_t index=(int16_t) (Rrtd/10);
			double_t frac = (double_t)(Rrtd/10.0) - index;
			double_t a = rtdInterpol[index];

			// Si valeur juste, on lis la case directement
			if (index == Rrtd / 10)
			{
				tempInter = rtdInterpol[index];
			}
			// Sinon approximation par interpolation du des valeurs du tableau
			double_t b = rtdInterpol[index + 1] / 2.0;
			double_t c = rtdInterpol[index - 1] / 2.0;
			mes2 = (double_t)a + frac * (b - c + frac * (c + b - a));
			mes2 += 0.3;
		} else {
			mes1 = adc.rtd.readValue();

			Rrtd = (mes1 * 1649.735) / (32767 * (double_t)adc.getGain());
		
			// Application d'un offset de T°C
			//double_t calib0CRTD = 100.04; // Valeur de la resistance @ 0°C

			// Conversion de la résistance d'une RTD en température via la méthode d'interpolation
			int16_t index=(int16_t) (Rrtd/10);
			double_t frac = (double_t)(Rrtd/10.0) - index;
			double_t a = rtdInterpol[index];
			
			// Si valeur juste, on lis la case directement
			if (index == Rrtd / 10)
			{
				tempInter = rtdInterpol[index];
			}
			// Sinon approximation par interpolation du des valeurs du tableau
			double_t b = rtdInterpol[index + 1] / 2.0;
			double_t c = rtdInterpol[index - 1] / 2.0;
			mes1 = (double_t)a + frac * (b - c + frac * (c + b - a));
		}
		//double_t res = 

		// Application de l'erreur PGA
		// res += 1;

		// Conversion de la valeur numérique en résistance (Ohm)
		// Rref (@21°c) = 1649.797
		// Rref (@18°c) = 1650.56 (ancienne cal)		
	

		// Envoi sur le port série
		
		Serial.print((String)mes1 + " ; ");
		Serial.print(Rrtd, 4);
		Serial.print(" ; ");
		Serial.println(tempInter, 4);

		// Affichage écran
		char TX[50];
		tft.fillScreen(ST77XX_BLACK);
		tft.setTextSize(3);				 // Normal 1:1 pixel scale
		tft.setTextColor(ST77XX_WHITE); // Draw white text
		tft.cp437(true);				 // Use full 256 char 'Code Page 437' font
		tft.setTextSize(2);
		tft.setCursor(0, 0); // Start at top-left corner
		tft.printf("Consigne");
		tft.setTextSize(2);
		tft.setCursor(0, 20); // Start at top-left corner
		tft.setTextColor(ST77XX_GREEN);
		sprintf(TX,"%3.3lf", mes1); //  XXX.XX
		//tft.fillRect(0, 20, 84, 36, ST77XX_BLACK);
		tft.printf(TX);
		tft.setTextSize(2);
		tft.setCursor(0, 40); // Start at top-left corner
		tft.setTextColor(ST77XX_GREEN);
		sprintf(TX,"%3.3lf", mes2); //  XXX.XX
		//tft.fillRect(0, 40, 108, 64, ST77XX_BLACK);
		tft.setCursor(0, 40); // Start at top-left corner
		tft.printf(TX);

		
		double_t pVs = pow(10,(2.7877+(7.625*mes1)/(241.6+mes1)));
		double_t pV = pVs - 0.00066*101300*(mes2-mes1);
		double_t pVs2 = pow(10,(2.7877+(7.625*mes2)/(241.6+mes2)));
		double_t rh = ((double_t)pV/pVs2)*100.0;

		tft.setTextSize(3);
		tft.setCursor(0, 60); // Start at top-left corner
		tft.setTextColor(ST77XX_RED);
		sprintf(TX,"%3.3lf", rh); //  XXX.XX
		//tft.fillRect(0, 40, 108, 64, ST77XX_BLACK);
		tft.setCursor(0, 70); // Start at top-left corner
		tft.printf(TX);

		
		/** 
		 * La lecture de la t°C interne du rasp est lié a Vcc = 3v3
		 * dont il faut une alim précise ou un vref externe pour que ça fonctionne correctement
		 * et la résolution est plutot faible
		*/
		//double internalTemp = analogReadTemp();
		


	}
}

void rotencInterrupt(void)
{
	//delayMicroseconds(10);
	PinStatus b = digitalRead(ROTENC_B);
	Serial.println(b);
}

void rotencInterruptClic(void) {

	Serial.println("Click");
}

void adcInterrupt()
{
	int val = adc.ads1120.readADC();
	adc.rtd.add(val);

	// Cas particulier de la mesure en 3 fils (current chopping) : 
	// inversion des sources d'exitation de courant à la moitié de la série, pour supprimer leur inégalité de courant
	if ((adc.rtd.type == TYPE_3WIRE) && (adc.rtd.sampleCount == (adc.rtd.samples / 2)) && (adc.rtd.samples % 2 == 0))
	{
		adc.invert3WireIDAC();
	}

	// If all samples are measured, compute the result
	if (adc.rtd.sampleCount == adc.rtd.samples)
	{
		adc.rtd.compute();

		adc.stop();
		if( mux_1 ) {
			digitalWrite(SW_MUX_2, HIGH);
			digitalWrite(SW_MUX_1, LOW);
			mux_1 = false;
		} else {
			digitalWrite(SW_MUX_1, HIGH);
			digitalWrite(SW_MUX_2, LOW);
			mux_1 = true;
		}
		delay(10);
		adc.restart();

		if (adc.rtd.type == TYPE_3WIRE)
		{
			adc.set3WireIDAC();
		}
	}
}

/**
 * CPU0 : contrôle de la mesure ADC et de la régulation
 * @brief
 *
 */
void loop()
{

}