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
 * MIT license, all text above must be included in any redistribution 
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

SensorBoard board;
bool enterCalMode = false;
void adcInterrupt();
void menuConfCal(void) {

	rp2040.fifo.push(PAUSE_ADC_INTERRUPTS);
	delay(10);
	board.calRefResistor();
	rp2040.fifo.push(RESUME_ADC_INTERRUPTS);
	//nav.exit();
}

#include "myMenu.h"




Adafruit_ST7789 tft = Adafruit_ST7789(&SPI1, LCD_CS, LCD_DC, LCD_RESET);
Adafruit_BME280 bme;

double temperatureADC = 0;

// Menu declaration
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
	
	board.resetCounts();
	board.startContinuous();
  }

  // Retour dans la partie menu (couper l'ADC par ex)
  if (e == idleEnd ) {
	board.stop();
  }

  return proceed;
}



// Interruption du clic bouton
void IsrButton(void)
{
 	nav.doNav(navCmds::enterCmd);
}
// Interruption de l'encodeur rotatif
void IsrRotenc(void)
{
  if (digitalRead(ROTENC_B))
    nav.doNav(navCmds::upCmd);
  else
    nav.doNav(navCmds::downCmd);
}





void setup()
{
	//Serial.begin(115200);
	delay(100);
	//Serial.println("Open Process Controller v0.1");

	// Configuration des pins de commandes des analog switches
	pinMode(SW_3_WIRE, OUTPUT);
	pinMode(SW_4_WIRE, OUTPUT);
	pinMode(SW_MUX_1, OUTPUT);
	pinMode(SW_MUX_2, OUTPUT);
	pinMode(SW_MUX_3, OUTPUT);

	// Set pin 23 HIGH to switch the pico DC-DC converter to PWM (improved ripple)
	// Improves a lot measurement stability
	pinMode(23, OUTPUT);
	digitalWrite(23, HIGH);

	board.init();
	board.addRTD(TYPE_4WIRE, SW_MUX_1, 32, 0);
	board.addRTD(TYPE_4WIRE, SW_MUX_2, 32, 0);

	attachInterrupt(digitalPinToInterrupt(SPI_DRDY), adcInterrupt, FALLING);
}

void setup1()
{
	Serial.begin(115200);
	delay(100);

	// Configuration de l'encodeur rotatif
	pinMode(ROTENC_A, INPUT);
	pinMode(ROTENC_B, INPUT);
	pinMode(ROTENC_CLIC, INPUT);
	attachInterrupt(digitalPinToInterrupt(ROTENC_A), IsrRotenc, FALLING);
	attachInterrupt(digitalPinToInterrupt(ROTENC_CLIC), IsrButton, FALLING);

	// Configuration du SPI de l'écran
	SPI1.setSCK(LCD_SCK);
	SPI1.setTX(LCD_MOSI);
	tft.init(135, 240);
	tft.setSPISpeed(48000000);
	tft.setRotation(3);
	tft.setTextSize(2);
	tft.setTextWrap(false);
	tft.cp437(true);	// Use full 256 char 'Code Page 437' font

	// Configuration du BME280
	Wire.setSCL(9);
    Wire.setSDA(8);
    bme.begin(0x76, &Wire);

	nav.idleTask = idle;	// point a function to be used when menu is suspended
  	nav.timeOut=10;	// in seconds
	nav.exit();
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

		if (board.newMeasurement)
		{
		float_t temperature = bme.readTemperature(); // Lecture de la T°C actuelle du système de mesure
		float_t pressure = bme.readPressure(); 		 // Pa

		// lecture de la dernière série de valeurs
		// Lire toutes les valeurs de resistances directement en liste, conversion en t° ensuite (long en calcul)
		board.convertToTemperature(temperatureADC);
		double_t rh = board.getRH(0, 1, pressure);
		board.newMeasurement = false;	

		// Envoi sur le port série
		Serial.print(board.rtd[0].avgValue, 2); Serial.print(" ;  ");
		Serial.print(temperatureADC, 1); Serial.print(" C ;  ");
		Serial.print(board.rtd[0].resistance, 4); Serial.print(" ; ");
		Serial.print(board.rtd[0].temperature, 3); Serial.print(" ||||| ");
		Serial.print(board.rtd[1].resistance, 4); Serial.print(" ; ");
		Serial.print(board.rtd[1].temperature, 3); Serial.print(" ||||| ");
		Serial.print((pressure/100.0F), 1); Serial.print(" ; ");
		Serial.print(rh, 2);
		Serial.println("");
		

		

		// Affichage écran
		char TX[50];

		// Affichage de la mesure
		tft.setTextSize(4);
		tft.setCursor(0, 5);
		tft.setTextColor(ST77XX_RED, ST77XX_BLACK);
		//sprintf(TX,"%03.2lf", rh);
		sprintf(TX,"%03.4lf", board.rtd[0].resistance);
		tft.printf(TX);
		// test
		tft.setTextSize(1);
		tft.setCursor(0, 40);
		tft.setTextColor(ST77XX_RED, ST77XX_BLACK);
		sprintf(TX,"%03.3lf", board.rtd[0].temperature);
		tft.printf(TX);
		// test2
		tft.setTextSize(1);
		tft.setCursor(60, 40);
		tft.setTextColor(ST77XX_RED, ST77XX_BLACK);
		sprintf(TX,"%03.3lf", board.rtd[1].temperature);
		tft.printf(TX);
		// Affichage de la consigne
		tft.setTextSize(4);
		tft.setCursor(0, 60);
		tft.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
		//sprintf(TX,"%03.1lf", 98.5);
		sprintf(TX,"%04.1lf", (pressure/100.0F));
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
		}
	}

	delay(1);
}





/**
 * CPU0 : contrôle de la mesure ADC et de la régulation
 * @brief
 *
 */
void loop()
{	
	if(  rp2040.fifo.available() > 0) {
		uint32_t val = rp2040.fifo.pop();
		
		// Pauses ADC IRQ while calibrating
		if( val == PAUSE_ADC_INTERRUPTS ) {
			irq_set_enabled(13, false); // Pause IO interrupts
		} else if( val == RESUME_ADC_INTERRUPTS ) {
			irq_set_enabled(13, true); // Resume IO interrupts
			nav.exit();
		}
	}

	delay(10);
}








void adcInterrupt() {

    int32_t value = board.ads1120.readADC();
	
    board.rtd[board.curRTDSensor].add(value);

    // Cas particulier de la mesure en 3 fils (current chopping) : 
	// inversion des sources d'exitation de courant à la moitié de la série, pour supprimer leur inégalité de courant
	if ((board.rtd[board.curRTDSensor].measurementType == TYPE_3WIRE) 
        && (board.rtd[board.curRTDSensor].sampleCount == (board.rtd[board.curRTDSensor].samples / 2)) 
        && (board.rtd[board.curRTDSensor].samples % 2 == 0) )
	{
		board.invert3WireIDAC();
	}

    // If all samples are measured, compute the result
    if (board.rtd[board.curRTDSensor].sampleCount == board.rtd[board.curRTDSensor].samples)
	{
        board.stop();

		temperatureADC = board.ads1120.readInternalTemp();	// T°C interne de l'ADC

		board.rtd[board.curRTDSensor].compute();

        digitalWrite(board.rtd[board.curRTDSensor].analogSwitchPin, LOW);
		board.curRTDSensor++;

        // Fin de la boucle de mesure on recommence
        if( board.curRTDSensor == board.numRTDSensors ) {
            board.curRTDSensor = 0;
			board.newMeasurement = true;
        }

        // Allumage de l'analog switch concerné par la conversion
        digitalWrite(board.rtd[board.curRTDSensor].analogSwitchPin, HIGH);

        // Initialisation du type de mesure
        if (board.rtd[board.curRTDSensor].measurementType == TYPE_3WIRE) {
			board.set3WirePT100();
		} else if(board.rtd[board.curRTDSensor].measurementType == TYPE_4WIRE) {
            board.set4WirePT100();
        }

        // Pause et relance de la conversion continue
		delay(10);
		board.restart();
    }

}