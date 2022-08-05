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
#include <Mesure.h>
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
//#define SW_3_WIRE 28
//#define SW_4_WIRE 27
#define SW_MUX_1 2
#define SW_MUX_2 1
#define SW_MUX_3 0

void rotencInterrupt();
void rotencInterruptClic();
void adcInterrupt();

ADC adc;


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

	adc.init();
	adc.addRTD(0, TYPE_4WIRE, SW_MUX_1, 64, 0);
	adc.addRTD(1, TYPE_4WIRE, SW_MUX_2, 64, 0);
	attachInterrupt(digitalPinToInterrupt(SPI_DRDY), adcInterrupt, FALLING);
	adc.startContinuous();
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

	if (adc.newMeasurement)
	{
		// lecture de la dernière série de valeurs
		// Lire toutes les valeurs de resistances directement en liste, conversion en t° ensuite (long en calcul)
		double_t rtd1 = adc.getResistanceValue(0);
		double_t rtd2 = adc.getResistanceValue(1);
		adc.newMeasurement = false;
	
		double_t mes1, mes2;

		mes1 = adc.getRTDTempInterpolation(rtd1);
		mes2 = adc.getRTDTempInterpolation(rtd2);

		// Envoi sur le port série
		//Serial.print((String)mes1 + " ; ");
		Serial.print(rtd1, 4);
		Serial.print(" ; ");
		Serial.println(rtd2, 4);
		//Serial.println(tempInter, 4);

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
		sprintf(TX,"%3.3lf %3.3lf", mes1, rtd1); //  XXX.XX
		//tft.fillRect(0, 20, 84, 36, ST77XX_BLACK);
		tft.printf(TX);
		tft.setTextSize(2);
		tft.setCursor(0, 40); // Start at top-left corner
		tft.setTextColor(ST77XX_GREEN);
		sprintf(TX,"%3.3lf %3.3lf", mes2, rtd2); //  XXX.XX
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



/**
 * CPU0 : contrôle de la mesure ADC et de la régulation
 * @brief
 *
 */
void loop()
{

}








void adcInterrupt() {

    int value = adc.ads1120.readADC();

    adc.rtd[adc.curRTDSensor].add(value);

    // Cas particulier de la mesure en 3 fils (current chopping) : 
	// inversion des sources d'exitation de courant à la moitié de la série, pour supprimer leur inégalité de courant
	if ((adc.rtd[adc.curRTDSensor].measurementType == TYPE_3WIRE) 
        && (adc.rtd[adc.curRTDSensor].sampleCount == (adc.rtd[adc.curRTDSensor].samples / 2)) 
        && (adc.rtd[adc.curRTDSensor].samples % 2 == 0) )
	{
		adc.invert3WireIDAC();
	}

    // If all samples are measured, compute the result
    if (adc.rtd[adc.curRTDSensor].sampleCount == adc.rtd[adc.curRTDSensor].samples)
	{
        adc.stop();

		adc.rtd[adc.curRTDSensor].compute();

        digitalWrite(adc.rtd[adc.curRTDSensor].analogSwitchPin, LOW);
		adc.curRTDSensor++;

        // Fin de la boucle de mesure on recommence
        if( adc.curRTDSensor == adc.numRTDSensors ) {
            adc.curRTDSensor = 0;
			adc.newMeasurement = true;
        }

        // Allumage de l'analog switch concerné par la conversion
        digitalWrite(adc.rtd[adc.curRTDSensor].analogSwitchPin, HIGH);

        // Initialisation du type de mesure
        if (adc.rtd[adc.curRTDSensor].measurementType == TYPE_3WIRE) {
			adc.set3WirePT100();
		} else if(adc.rtd[adc.curRTDSensor].measurementType == TYPE_4WIRE) {
            adc.set4WirePT100();
        }

        // Pause et relance de la conversion continue
		delay(1);
		adc.restart();
    }

}