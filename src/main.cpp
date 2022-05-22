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


// Ecran SPI OLED
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_DC 7
#define OLED_CS 13
#define OLED_RESET 9
// Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT,&SPI1, OLED_DC, OLED_RESET, OLED_CS);
Adafruit_ST7789 tft = Adafruit_ST7789(&SPI1, OLED_CS, OLED_DC, OLED_RESET);

void rotencInterrupt();
void adcInterrupt();
void printScreen(double_t temp);

double_t G_Temperature;
ADC adc;

void setup()
{
	delay(2000);
	Serial.begin(115200);
	Serial.println("Open Process Controller");

	/*adc.init(TYPE_3WIRE, 50);
	adc.set3WirePT100();
	adc.set3WireIDAC();*/
	adc.init(TYPE_4WIRE, 50);
	adc.set4WirePT100();
	adc.startContinuous(adcInterrupt);
}

void setup1()
{
	delay(2000);
	pinMode(14, INPUT);
	pinMode(15, INPUT);
	attachInterrupt(digitalPinToInterrupt(14), rotencInterrupt, FALLING);

	SPI1.setSCK(10);
	SPI1.setTX(11);
	

	// SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
	/*if(!display.begin(SSD1306_SWITCHCAPVCC)) {
		Serial.println(F("SSD1306 allocation failed"));
		for(;;); // Don't proceed, loop forever
	}*/
	tft.init(135, 240); // Init ST7789 240x240
	tft.setRotation(1);
	tft.fillScreen(ST77XX_BLACK);
}

/**
 * Cpu1 : contrôle des IT utilisateur et écran
 * @brief
 *
 */
void loop1()
{

	if (adc.rtd.newMeasure)
	{
		// lecture de la dernière valeur
		double_t res = adc.rtd.readValue();

		// Application de l'erreur PGA
		// res += 1;

		double_t Rrtd = (res * 1650.56) / (32767 * 16.0);

		double_t calib0CRTD = 100.04; // Valeur de la resistance @ 0°C

		// double temp = (Rrtd-calib0CRTD)/0.385;	// Approximation linéaire de la T°C
		double_t temp = -244.83 + Rrtd * (2.3419 + 0.0010664 * Rrtd);

		G_Temperature = temp;

		Serial.print((String)res + " ; ");
		Serial.print(Rrtd, 3);
		Serial.print(" ; ");
		Serial.println(temp, 3);

		/*display.clearDisplay();
		display.setTextSize(3);				 // Normal 1:1 pixel scale
		display.setTextColor(SSD1306_WHITE); // Draw white text
		display.cp437(true);				 // Use full 256 char 'Code Page 437' font
		display.setTextSize(2);
		display.setCursor(0, 0); // Start at top-left corner
		display.printf("Consigne");
		display.setTextSize(2);
		display.setCursor(0, 20); // Start at top-left corner
		display.printf("%3.3lf", Rrtd);
		display.setTextSize(3);
		display.setCursor(0, 40); // Start at top-left corner
		display.printf("%3.3lf", temp);
		display.display();*/
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
		sprintf(TX,"%3.3lf", Rrtd); //  XXX.XX
		//tft.fillRect(0, 20, 84, 36, ST77XX_BLACK);
		tft.printf(TX);
		tft.setTextSize(3);
		tft.setCursor(0, 40); // Start at top-left corner
		tft.setTextColor(ST77XX_RED);
		sprintf(TX,"%3.3lf", temp); //  XXX.XX
		//tft.fillRect(0, 40, 108, 64, ST77XX_BLACK);
		
		tft.setCursor(0, 40); // Start at top-left corner
		tft.printf(TX);

		
		double internalTemp = analogReadTemp();
		tft.setTextSize(3);
		tft.setCursor(0, 40); // Start at top-left corner
		tft.setTextColor(ST77XX_RED);
		sprintf(TX,"%2.1lf", internalTemp); //  XXX.XX
		//tft.fillRect(0, 40, 108, 64, ST77XX_BLACK);
		tft.setCursor(0, 60); // Start at top-left corner
		tft.printf(TX);

		// Serial.print(";");
		// Serial.println(millis()/1000);
	}
}

void rotencInterrupt(void)
{
	Serial.println(digitalRead(15));
}

void adcInterrupt()
{
	int val = adc.ads1120.readADC();
	adc.rtd.add(val);
	// Serial.println(val);

	// If 3 wire, chopp the current sources
	if ((adc.rtd.type == TYPE_3WIRE) && (adc.rtd.sampleCount == (adc.rtd.samples / 2)) && (adc.rtd.samples % 2 == 0))
	{
		// Serial.println(adc.rtd.sum);
		// adc.ads1120.startSync();
		adc.invert3WireIDAC();
		// adc.ads1120.startSync();
		// Serial.println("inversion");
		// adc.ads1120.startSync();
	}

	// If all samples are measured, compute the result
	if (adc.rtd.sampleCount == adc.rtd.samples)
	{
		adc.rtd.compute();

		if (adc.rtd.type == TYPE_3WIRE)
		{
			// adc.ads1120.startSync();
			adc.set3WireIDAC();
			// adc.ads1120.startSync();
			// adc.ads1120.startSync();
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
	// One shot
	/*adc.setIDACcurrent(6);
	delay(1000);
	int val = adc.readADC_Single();
	Serial.println(val);
	adc.setIDACcurrent(0);*/

	// Serial.print("Internal temp  : ");
	// Serial.println(adc.readInternalTemp(), 1);

	// delay(1000);
	// Serial.println("okok");

	/*if (adc.rtd.newMeasure)
	{
		// lecture de la dernière valeur
		double_t res = adc.rtd.readValue();

		// Application de l'erreur PGA
		// res += 1;

		double_t Rrtd = (res * 1650.56) / (32767 * 8.0);

		double_t calib0CRTD = 100.04; // Valeur de la resistance @ 0°C

		// double temp = (Rrtd-calib0CRTD)/0.385;	// Approximation linéaire de la T°C
		double_t temp = -244.83 + Rrtd * (2.3419 + 0.0010664 * Rrtd);

		G_Temperature = temp;
		Serial.print((String)res + " ; ");
		Serial.print(Rrtd, 3);
		Serial.print(" ; ");
		Serial.println(temp, 3);

		// Serial.print(";");
		// Serial.println(millis()/1000);
	}*/
}

/*void printScreen(double_t temp)
{
	display.clearDisplay();

	display.setTextSize(3);				 // Normal 1:1 pixel scale
	display.setTextColor(SSD1306_WHITE); // Draw white text
	display.setCursor(0, 30);			 // Start at top-left corner
	display.cp437(true);				 // Use full 256 char 'Code Page 437' font

	// double_t temp = 100.13;
	display.printf("%3.3lf", temp);
	display.display();
}*/