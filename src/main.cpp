/**
 * @file main.cpp
 *
 *
 *
 * @author GAOU (arstaligtredan.fr)
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
#include <pinout.h>
#include <SPI.h>
#include <Mesure.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "pico/stdlib.h"
#include <math.h>
#include <menu.h>
#include <menuIO/adafruitGfxOut.h>
#include <menuIO/keyIn.h>
#include <menuIO/chainStream.h>
#include <menuIO/serialOut.h>
#include <menuIO/serialIn.h>
#include <EEPROM.h>

using namespace Menu;

ADC adc;

#include "myMenu.h"




Adafruit_ST7789 tft = Adafruit_ST7789(&SPI1, LCD_CS, LCD_DC, LCD_RESET);
Adafruit_BME280 bme;
void adcInterrupt();

// Déclaration des éléments du menu
//serialIn serial(Serial);
MENU_INPUTS(in);
#define MAX_DEPTH 5
MENU_OUTPUTS(out, MAX_DEPTH, 
  ADAGFX_OUT(tft, colors, 12, 18, {0, 0, 20, 7}),
  NONE);
NAVROOT(nav, mainMenu, MAX_DEPTH, in, out);




// Gestion de l'écran d'accueil/idle
result idle(menuOut &o, idleEvent e)
{
  // Si on rentre en écran de base (lancer l'ADC par exemple)
  if (e == idleStart) {
	
	adc.resetCounts();
	adc.startContinuous();
  }

  // Retour dans la partie menu (couper l'ADC par ex)
  if (e == idleEnd ) {
	adc.stop();
  }

  return proceed;
}



// Interruption du clic bouton
void IsrButton(void)
{
  /*if (!digitalRead(ROTARY_PIN_BUT)) {
    nav.doNav(navCmds::escCmd);
  } else {
    nav.doNav(navCmds::enterCmd);
  }*/
  nav.doNav(navCmds::enterCmd);
  //delay(25); // Ceci fait tout déconner je ne sais pas pour quelle raison
}
// Interruption de l'encodeur rotatif
void IsrRotenc(void)
{
  if (digitalRead(ROTENC_B))
    nav.doNav(navCmds::upCmd);
  else
    nav.doNav(navCmds::downCmd);
  
  //delay(10); // Ceci fait tout déconner je ne sais pas pour quelle raison
}





void setup()
{
	Serial.begin(115200);
	delay(3000);
	Serial.println("Open Process Controller v0.1");

	// Configuration des pins de commandes des analog switches
	pinMode(SW_3_WIRE, OUTPUT);
	pinMode(SW_4_WIRE, OUTPUT);
	pinMode(SW_MUX_1, OUTPUT);
	pinMode(SW_MUX_2, OUTPUT);
	pinMode(SW_MUX_3, OUTPUT);

	adc.init();
	adc.addRTD(0, TYPE_4WIRE, SW_MUX_1, 64, 0.02);
	adc.addRTD(1, TYPE_4WIRE, SW_MUX_2, 64, 0);
	adc.resetCounts();
	attachInterrupt(digitalPinToInterrupt(SPI_DRDY), adcInterrupt, FALLING);
}

void setup1()
{
	delay(3000);

	// Configuration de l'encodeur rotatif
	pinMode(ROTENC_A, INPUT);
	pinMode(ROTENC_B, INPUT);
	pinMode(ROTENC_CLIC, INPUT);
	attachInterrupt(digitalPinToInterrupt(ROTENC_A), IsrRotenc, FALLING);
	attachInterrupt(digitalPinToInterrupt(ROTENC_CLIC), IsrButton, FALLING);

	nav.idleTask = idle; // point a function to be used when menu is suspended
  	nav.timeOut=1; //10sec

	// Configuration du SPI de l'écran
	SPI1.setSCK(LCD_SCK);
	SPI1.setTX(LCD_MOSI);
	tft.init(135, 240); // Init ST7789 240x240
	tft.setSPISpeed(48000000);
	tft.setRotation(3);
	tft.setTextSize(2);
	tft.setTextWrap(false);
	tft.cp437(true);				 // Use full 256 char 'Code Page 437' font
	//tft.fillScreen(ST77XX_BLACK);

	// Configuration du BME280
	Wire.setSCL(9);
    Wire.setSDA(8);
    bme.begin(0x76, &Wire);
}



/**
 * Cpu1 : contrôle des IT utilisateur et écran
 * @brief
 *
 */
void loop1()
{
	
	nav.poll(); // this device only draws when needed
	
	// if nouvelle mesure && qu'on est en idle
	if(  nav.idleTask == nav.sleepTask ) {

		if (adc.newMeasurement)
		{
		// lecture de la dernière série de valeurs
		// Lire toutes les valeurs de resistances directement en liste, conversion en t° ensuite (long en calcul)
		double_t rtd1 = adc.getResistanceValue(0);
		double_t rtd2 = adc.getResistanceValue(1);
		double_t mes1 = adc.getRTDTempInterpolation(0);
		double_t mes2 = adc.getRTDTempInterpolation(1);
		adc.newMeasurement = false;		

		// Envoi sur le port série
		Serial.print(mes1, 2);
		Serial.print(" ; ");
		Serial.println(mes2, 2);
		//Serial.print(" ; ");
		//Serial.print(bme.readPressure() / 100.0F);
		//Serial.print(" ; ");
		//Serial.println(bme.readHumidity());
		
		double_t rh = adc.getRH(mes2, mes1, (bme.readPressure() / 1000.0F) );

		// Affichage écran
		char TX[50];

		// Affichage de la mesure
		tft.setTextSize(4);
		tft.setCursor(0, 10);
		tft.setTextColor(ST77XX_RED, ST77XX_BLACK);
		sprintf(TX,"%03.2lf", rh);
		tft.printf(TX);
		// Affichage de la consigne
		tft.setTextSize(4);
		tft.setCursor(0, 60);
		tft.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
		sprintf(TX,"%03.1lf", 98.5);
		tft.printf(TX);
		// Affichage de l'état des sorties
		tft.setTextSize(2);
		tft.setCursor(20, 110);
		tft.setTextColor(ST77XX_ORANGE, ST77XX_BLACK);
		tft.printf("SP1");
		tft.setTextSize(2);
		tft.setCursor(80, 110);
		tft.setTextColor(ST77XX_ORANGE, ST77XX_BLACK);
		tft.printf("SP2");

		/*
		//tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK); // Draw white text
		//tft.setTextSize(2);
		//tft.setCursor(0, 0); // Start at top-left corner
		//tft.printf("Consigne");
		tft.setTextSize(2);
		tft.setCursor(0, 20);
		tft.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
		sprintf(TX,"%3.3lf %3.3lf", mes1, rtd1); //  XXX.XX
		tft.printf(TX);
		//tft.setTextSize(2);
		tft.setCursor(0, 40);
		tft.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
		sprintf(TX,"%3.3lf %3.3lf", mes2, rtd2); //  XXX.XX
		tft.setCursor(0, 40);
		tft.printf(TX);
		tft.setTextSize(3);
		tft.setCursor(0, 60);
		tft.setTextColor(ST77XX_RED, ST77XX_BLACK);
		sprintf(TX,"%3.3lf", rh); //  XXX.XX
		tft.setCursor(0, 70);
		tft.printf(TX);
		tft.setTextSize(2);*/

		/*Serial.println(sizeof(double_t));
		Serial.println(sizeof(int64_t));
		Serial.println(sizeof(uint16_t));
		Serial.println(sizeof(float_t));
		Serial.println(sizeof(adc.rtd[0]));*/
		
		// On peux enregistrer des objets complets et les rapatrier, donc impeccable
		/*EEPROM.begin(512);
		EEPROM.put(0, adc);

		ADC adc2;
		Serial.println(adc2.rtd[0].measurementType);
		EEPROM.get(0, adc2);
		Serial.println(adc2.rtd[0].measurementType);
		EEPROM.end();*/

		/** 
		 * La lecture de la t°C interne du rasp est lié a Vcc = 3v3
		 * dont il faut une alim précise ou un vref externe pour que ça fonctionne correctement
		 * et la résolution est plutot faible
		*/
		//double internalTemp = analogReadTemp()
		}
	}

	delay(10);
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

    int32_t value = adc.ads1120.readADC();
	
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