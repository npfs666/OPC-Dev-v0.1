#ifndef _MYMENU_H
#define _MYMENU_H

#define SENSORS 2
#define OUTPUTS 1

// define menu colors --------------------------------------------------------
//  {{disabled normal,disabled selected},{enabled normal,enabled selected, enabled editing}}
// monochromatic color table
#define ST77XX_GRAY RGB565(128,128,128)
const colorDef<uint16_t> colors[6] MEMMODE = {
    {{(uint16_t)ST77XX_BLACK, (uint16_t)ST77XX_BLACK}, {(uint16_t)ST77XX_BLACK, (uint16_t)ST77XX_GRAY, (uint16_t)ST77XX_RED}},     // bgColor
    {{(uint16_t)ST77XX_CYAN, (uint16_t)ST77XX_CYAN}, {(uint16_t)ST77XX_WHITE, (uint16_t)ST77XX_WHITE, (uint16_t)ST77XX_WHITE}},     // fgColor
    {{(uint16_t)ST77XX_WHITE, (uint16_t)ST77XX_BLACK}, {(uint16_t)ST77XX_YELLOW, (uint16_t)ST77XX_YELLOW, (uint16_t)ST77XX_RED}},   // valColor
    {{(uint16_t)ST77XX_WHITE, (uint16_t)ST77XX_BLACK}, {(uint16_t)ST77XX_WHITE, (uint16_t)ST77XX_YELLOW, (uint16_t)ST77XX_YELLOW}}, // unitColor
    {{(uint16_t)ST77XX_WHITE, (uint16_t)ST77XX_CYAN}, {(uint16_t)ST77XX_BLACK, (uint16_t)ST77XX_GRAY, (uint16_t)ST77XX_WHITE}},     // cursorColor
    {{(uint16_t)ST77XX_WHITE, (uint16_t)ST77XX_YELLOW}, {(uint16_t)ST77XX_WHITE, (uint16_t)ST77XX_BLACK, (uint16_t)ST77XX_BLACK}},       // titleColor
};

double_t SP1=0, SP2=0;
//altFIELD(decPlaces<3>::menuField,test2,"Test"," Ohm",98.000,102.000,0.1,0.001,doNothing,noEvent,wrapStyle),
// FIELD(test,"Power","mOhm",98000,102000,100,1,doNothing,noEvent,noStyle),

MENU(subMenuOpr, " Operateur", doNothing, noEvent, wrapStyle, 
  FIELD(SP1,"SP1:"," C",0,100,10,1,doNothing,noEvent,wrapStyle),
  FIELD(SP2,"SP2:"," C",0,100,1,0.1,doNothing,noEvent,wrapStyle),
  EXIT("<Back"));


SELECT(adc.rtd[0].measurementType,selMenuConfInpType1,"Mesure",doNothing,noEvent,wrapStyle
  ,VALUE("2 fils",2,doNothing,noEvent)
  ,VALUE("3 fils",3,doNothing,noEvent)
  ,VALUE("4 fils",4,doNothing,noEvent)
);
MENU(subMenuConfInp1, " Sonde 1", doNothing, noEvent, wrapStyle, 
  SUBMENU(selMenuConfInpType1),
  FIELD(adc.rtd[0].offset,"Offset:"," C",-5,5,1,0.1,doNothing,noEvent,wrapStyle),
  EXIT("<Back"));


#if SENSORS >= 2
SELECT(adc.rtd[1].measurementType,selMenuConfInpType2,"Mesure",doNothing,noEvent,wrapStyle
  ,VALUE("2 fils",2,doNothing,noEvent)
  ,VALUE("3 fils",3,doNothing,noEvent)
  ,VALUE("4 fils",4,doNothing,noEvent)
);
MENU(subMenuConfInp2, " Sonde 2", doNothing, noEvent, wrapStyle, 
  SUBMENU(selMenuConfInpType2),
  FIELD(adc.rtd[1].offset,"Offset:"," C",-5,5,1,0.1,doNothing,noEvent,wrapStyle),
  EXIT("<Back"));
#endif
#if SENSORS >= 3
SELECT(adc.rtd[2].measurementType,selMenuConfInpType3,"Mesure",doNothing,noEvent,wrapStyle
  ,VALUE("2 fils",2,doNothing,noEvent)
  ,VALUE("3 fils",3,doNothing,noEvent)
  ,VALUE("4 fils",4,doNothing,noEvent)
);
MENU(subMenuConfInp3, " Sonde 3", doNothing, noEvent, wrapStyle, 
  SUBMENU(selMenuConfInpType3),
  FIELD(adc.rtd[2].offset,"Offset:"," C",-5,5,1,0.1,doNothing,noEvent,wrapStyle),
  EXIT("<Back"));
#endif

MENU(subMenuConfInp, " Sondes", doNothing, noEvent, wrapStyle, 
  SUBMENU(subMenuConfInp1),
#if SENSORS >= 2
  SUBMENU(subMenuConfInp2),
#endif
#if SENSORS >= 3
  SUBMENU(subMenuConfInp3),
#endif
  EXIT("<Back"));



SELECT(reg.regulateur[0].mode,selMenuConfCntrSP1Mode,"Mode",doNothing,noEvent,wrapStyle
  ,VALUE("Chauffer",1,doNothing,noEvent)
  ,VALUE("Refroidir",2,doNothing,noEvent)
);
MENU(subMenuConfCntrSP1, " Sortie 1", doNothing, noEvent, wrapStyle, 
  SUBMENU(selMenuConfCntrSP1Mode),
  FIELD(reg.regulateur[0].SP,"Hysteresis:"," C",0,10,1,0.1,doNothing,noEvent,wrapStyle),
  EXIT("<Back"));

#if OUTPUTS >= 2
SELECT(reg.regulateur[1].mode,selMenuConfCntrSP2Mode,"Mode",doNothing,noEvent,wrapStyle
  ,VALUE("Deshumidifier",2,doNothing,noEvent)
  ,VALUE("Humidifier",3,doNothing,noEvent)
);
MENU(subMenuConfCntrSP2, " Sortie 2", doNothing, noEvent, wrapStyle, 
  SUBMENU(selMenuConfCntrSP2Mode),
  FIELD(reg.regulateur[1].SP,"Hysteresis:"," C",0,10,1,0.1,doNothing,noEvent,wrapStyle),
  EXIT("<Back"));
#endif

MENU(subMenuConfCntr, " Regulateur", doNothing, noEvent, wrapStyle,
     SUBMENU(subMenuConfCntrSP1),
#if OUTPUTS >= 2
     SUBMENU(subMenuConfCntrSP2),
#endif
     EXIT("<Back"));

MENU(subMenuConf, " Configuration", doNothing, noEvent, wrapStyle, 
  SUBMENU(subMenuConfInp), 
  SUBMENU(subMenuConfCntr), 
  EXIT("<Back"));


MENU(mainMenu, " Menu principal", doNothing, noEvent, wrapStyle, 
  SUBMENU(subMenuOpr),
  SUBMENU(subMenuConf),
  EXIT("<Back"));



#endif // _MYMENU_H