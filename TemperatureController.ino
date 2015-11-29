// 
// Copyright (c) 2014 - Allen Bauer - http://blog.thereadoracleatdelphi.com
// Under MIT License
// 

#include <PID_AutoTune_v0.h>
#include <PID_v1.h>
#include <SPI.h>
#include <MAX31855.h>
#include <stdio.h>
#include <LiquidCrystal.h>
#include <EEPROM.h>
#include <buttons.h>
#include <avr/wdt.h>

#define BOARD_REV3

#define Flash(string) reinterpret_cast<const __FlashStringHelper*>(string)

#define RS 2
#if defined(BOARD_REV3)
#define EN 8
#else
#define EN 3
#endif
#define D0 4
#define D1 5
#define D2 6
#define D3 7

//Buttons
Button UP_button;
Button OK_button;
Button DN_button;

#define UP 1
#define DN 2
#define OK 3
#define ES 0x80
#define ESC 0xFF

#define CELSIUS 0
#define FAHRENHEIT 1

#define TM1 0
#define TM2 1
#define TC1 2
#define TC2 3

#define TempNone 0x00
#define TempControlled 0x01
#define TempHit 0x02
#define TempError 0x04
#define TempModeMask 0x30
#define TempModeAuto 0x00
#define TempModeOn 0x10
#define TempModeOff 0x20

typedef struct _TempData {
	double Current_Temp;
	double Set_Temp;
	double Error_Temp;
	double Max_Temp;
	double Start_Temp;
	double Output_Value;
	int Sensor;
	uint8_t Flags;
} TempData;

//Thermocouples
Adafruit_MAX31855 Thermocouple_One(A4);
Adafruit_MAX31855 Thermocouple_Two(A5);

//Temps

TempData Extruder = {
	0.0, 250.0, 245.0, 0.0, 0.0, 0.0, TC1, TempNone
};
TempData Bed = {
	0.0, 120.0, 115.0, 0.0, 0.0, 0.0, TC2, TempNone
};

int Display_Units = 0;
bool Restarted = false;
bool Reset = false;

unsigned long SerialBaud = 19200;

enum CtrlMode {MainMenu, Menu, Running};

CtrlMode ctrlMode = MainMenu;

#define MOSFET_Extruder 9
#define MOSFET_Bed 10

#if defined(BOARD_REV3)
#define FanControl 3
#define MaxFanTicks 240 // Max value for fan time

int FanTicks = 0; // Fan on for n to MaxFanTime seconds
bool Heating = false;
unsigned long lastFanTick = 0;
#endif

#define Thermistor_One A6
#define Thermistor_Two A7

double OutputValueExtruder;
double OutputValueBed;

typedef struct _PIDData {
	double KP;
	double KI;
	double KD;
} PIDData;

PIDData ExtruderTuning = {300, 0.05, 300};
PIDData BedTuning = {300, 0.5, 50};

PID PIDExtruder(&Extruder.Current_Temp, &Extruder.Output_Value, &Extruder.Set_Temp, ExtruderTuning.KP, ExtruderTuning.KI, ExtruderTuning.KD, DIRECT);
PID PIDBed(&Bed.Current_Temp, &Bed.Output_Value, &Bed.Set_Temp, BedTuning.KP, BedTuning.KI, BedTuning.KD, DIRECT);

LiquidCrystal lcd(RS, EN, D0, D1, D2, D3);

void setCursor(LiquidCrystal &lcd, uint8_t col, uint8_t row)
{
  int row_offsets[] = { 0x00, 0x40, 0x10, 0x50 };
  lcd.command(LCD_SETDDRAMADDR | (col + row_offsets[row]));
}

template <class T> int EEPROM_write(int ee, const T& value)
{
    const byte* p = (const byte*)(const void*)&value;
    int i;
    for (i = 0; i < sizeof(value); i++)
        EEPROM.write(ee++, *p++);
    return i;
}

template <class T> int EEPROM_read(int ee, T& value)
{
    byte* p = (byte*)(void*)&value;
    int i;
    for (i = 0; i < sizeof(value); i++)
        *p++ = EEPROM.read(ee++);
    return i;
}

bool Start(void *context, uint8_t index);
bool TempSetup(void *context, uint8_t index);
bool PIDTuning(void *context, uint8_t index);
bool SensorSetup(void *context, uint8_t index);
bool SerialSetup(void *context, uint8_t index);
bool SetSerialBaud(void *context, uint8_t index);
bool DisplayUnitSetup(void *context, uint8_t index);
bool SetCelsiusUnits(void *context, uint8_t index);
bool SetFahrenheitUnits(void *context, uint8_t index);
bool ShowEEPROM(void *context, uint8_t index);
bool SetExtruderTemp(void *context, uint8_t index);
bool SetBedTemp(void *context, uint8_t index);
bool SetExtruderError(void *context, uint8_t index);
bool SetBedError(void *context, uint8_t index);
bool SetExtruderSensor(void *context, uint8_t index);
bool SetBedSensor(void *context, uint8_t index);
bool SetThermistor1(void *context, uint8_t index);
bool SetThermistor2(void *context, uint8_t index);
bool SetThermocouple1(void *context, uint8_t index);
bool SetThermocouple2(void *context, uint8_t index);
bool ReadSensors(void *context, uint8_t index);
bool ShowPinModes(void *context, uint8_t index);
bool ShowTimerMode(void *context, uint8_t index);
bool WatchdogReset(void *context, uint8_t index);
bool SetExtruderPID(void *context, uint8_t index);
bool SetBedPID(void *context, uint8_t index);
bool TuneExtruderPID(void *context, uint8_t index);
bool TuneBedPID(void *context, uint8_t index);
bool SetKP(void *context, uint8_t index);
bool SetKI(void *context, uint8_t index);
bool SetKD(void *context, uint8_t index);
int read_thermistor(int THERMISTOR);
void ShowResetReason(int RR);
uint8_t getPinMode(uint8_t pin);

typedef bool (*menuFunc_t)(void *context, uint8_t index);

typedef struct _MenuData {
	const char * PROGMEM menuText;
	menuFunc_t menuFunc;
	bool enabled;
} MenuData;

const char mainMenu_title[] PROGMEM       = {"Main Menu"};
const char mainMenu_start[] PROGMEM       = {"Start"};
const char mainMenu_tempSetup[] PROGMEM   = {"Temp Setup"};
const char mainMenu_sensorSetup[] PROGMEM = {"Sensor Setup"};
const char mainMenu_pidTuning[] PROGMEM   = {"PID Tuning"};
const char mainMenu_unitSetup[] PROGMEM   = {"Temp Unit Setup"};
const char mainMenu_readSensors[] PROGMEM = {"Read Sensors"};
const char mainMenu_showEEPROM[] PROGMEM  = {"Show EEPROM"};
const char mainMenu_showPinModes[] PROGMEM= {"Show Pin Modes"};
const char mainMenu_showTimerMode[] PROGMEM={"Show Timer Mode"};
const char mainMenu_watchDogReset[] PROGMEM={"Watchdog Reset"};
const char mainMenu_serialSetup[] PROGMEM = {"Serial Setup"};

const char menu_Exit[] PROGMEM         = {"Exit"};

#define tempMenu_title mainMenu_tempSetup
const char tempMenu_Extruder[] PROGMEM = {"Extruder Temp"};
const char tempMenu_Bed[] PROGMEM      = {"Bed Temp"};
const char tempMenu_Extruder_Error[] PROGMEM = {"Extruder Error"};
const char tempMenu_Bed_Error[] PROGMEM= {"Bed Error"};

const char setTemp_prompt[] PROGMEM    = {"Set Temp: "};

const char setError_prompt[] PROGMEM   = {"Set Err:  "};

#define sensorMenu_title mainMenu_sensorSetup
const char sensorMenu_Extruder[] PROGMEM	= {"Extruder Sensor"};
const char sensorMenu_Bed[] PROGMEM      = {"Bed Sensor"};

#define pidTuningMenu_title mainMenu_pidTuning
const char pidTuningMenu_ExtruderPID[] PROGMEM     = {"Extruder PID"};
const char pidTuningMenu_BedPID[] PROGMEM          = {"Bed PID"};
const char pidTuningMenu_ExtruderTunePID[] PROGMEM = {"Extruder Tune"};
const char pidTuningMenu_BedTunePID[] PROGMEM      = {"Bed Tune"};

#define pidTuningMenuExtruder_title pidTuningMenu_ExtruderPID
#define pidTuningMenuBed_title pidTuningMenu_BedPID
const char pidTuningMenu_KP[] PROGMEM = {"Set KP"};
const char pidTuningMenu_KI[] PROGMEM = {"Set KI"};
const char pidTuningMenu_KD[] PROGMEM = {"Set KD"};

const char setKP_prompt[] PROGMEM = {"KP: "};
const char setKI_prompt[] PROGMEM = {"KI: "};
const char setKD_prompt[] PROGMEM = {"KD: "};

const char tuningExtruder_title[] PROGMEM = {"Tuning Extruder"};
const char tuningBed_title[] PROGMEM      = {"Tuning Bed"};

#define unitMenu_title mainMenu_unitSetup
const char unitMenu_CELSIUS[] PROGMEM   = {"Celsius"};
const char unitMenu_Farenheit[] PROGMEM = {"Fahrenheit"};

#define extruderMenu_title sensorMenu_Extruder
#define bedMenu_title sensorMenu_Bed
const char sensorMenu_Thermistor1[] PROGMEM   = {"Thermistor 1"};
const char sensorMenu_Thermistor2[] PROGMEM   = {"Thermistor 2"};
const char sensorMenu_Thermocouple1[] PROGMEM = {"Thermocouple 1"};
const char sensorMenu_Thermocouple2[] PROGMEM = {"Thermocouple 2"};

const char label_Thermistor1[] PROGMEM   = {"TM1: "};
const char label_Thermistor2[] PROGMEM   = {"TM2: "};
const char label_Thermocouple1[] PROGMEM = {"TC1: "};
const char label_Thermocouple2[] PROGMEM = {"TC2: "};

const char extruderText[] PROGMEM = {"Extruder:"};
const char bedText[] PROGMEM      = {"Bed:     "};
const char sensorText[] PROGMEM   = {"Sensor:  "};

const char extruderModeError[] PROGMEM = {"Extrdr Mode Err!"};
const char bedModeError[] PROGMEM = {"Bed Mode Err!"};
const char rsModeError[] PROGMEM = {"RS Mode Err!"};
const char enModeError[] PROGMEM = {"EN Mode Err!"};
const char timerModeError[] PROGMEM = {"Timer Mode Err!"};
const char resettingModesMsg[] PROGMEM = {"Resetting Modes!"};

const char sINPUT[] PROGMEM = {"INPUT"};
const char sPULLUP[] PROGMEM = {"PULLUP"};
const char sOUTPUT[] PROGMEM = {"OUTPUT"};
const char sERROR[] PROGMEM = {"Error"};

const char sExtruder[] PROGMEM = {"Extruder: "};
const char sBed[] PROGMEM      = {"Bed:      "};
const char sEnable[] PROGMEM   = {"Enable:   "};
const char sRegSel[] PROGMEM   = {"Reg Sel:  "};

const char sTCCR1A[] PROGMEM   = {"TCCR1A: "};
const char sTCCR1B[] PROGMEM   = {"TCCR1B: "};
const char sGTCCR[] PROGMEM    = {"GTCCR:  "};

const char sOFF[] PROGMEM    = {"Off"};
const char sON[] PROGMEM     = {"On"};

#define baudMenu_title mainMenu_serialSetup
const char baudMenu_9600[] PROGMEM   = {"9600"};
const char baudMenu_19200[] PROGMEM  = {"19200"};
const char baudMenu_38400[] PROGMEM  = {"38400"};
const char baudMenu_57600[] PROGMEM  = {"57600"};
const char baudMenu_115200[] PROGMEM = {"115200"};

MenuData mainMenu[] = {
	{mainMenu_start, Start, true},
	{mainMenu_readSensors, ReadSensors, true},
	{mainMenu_tempSetup, TempSetup, true},
	{mainMenu_sensorSetup, SensorSetup, true},
	{mainMenu_serialSetup, SerialSetup, true},
	{mainMenu_unitSetup, DisplayUnitSetup, true},
	{mainMenu_pidTuning, PIDTuning, true},
	{mainMenu_showEEPROM, ShowEEPROM, true},
	{mainMenu_showPinModes, ShowPinModes, true},
	{mainMenu_showTimerMode, ShowTimerMode, true},
	{mainMenu_watchDogReset, WatchdogReset, true},
};

MenuData tempMenu[] = {
	{tempMenu_Extruder, SetExtruderTemp, true},
	{tempMenu_Bed, SetBedTemp, true},
	{tempMenu_Extruder_Error, SetExtruderError, true},
	{tempMenu_Bed_Error, SetBedError, true},
	{menu_Exit, NULL, true},
};

MenuData baudMenu[] = {
	{baudMenu_9600, SetSerialBaud, true},
	{baudMenu_19200, SetSerialBaud, true},
	{baudMenu_38400, SetSerialBaud, true},
	{baudMenu_57600, SetSerialBaud, true},
	{baudMenu_115200, SetSerialBaud, true},
	{menu_Exit, NULL, true},
};

MenuData pidTuningMenu[] = {
	{pidTuningMenu_ExtruderPID, SetExtruderPID, true},
	{pidTuningMenu_BedPID, SetBedPID, true},
	{pidTuningMenu_ExtruderTunePID, TuneExtruderPID, true},
	{pidTuningMenu_BedTunePID, TuneBedPID, true},
	{menu_Exit, NULL, true},
};

MenuData pidTuningMenuValues[] = {
	{pidTuningMenu_KP, SetKP, true},
	{pidTuningMenu_KI, SetKI, true},
	{pidTuningMenu_KD, SetKD, true},
	{menu_Exit, NULL, true},
};

MenuData sensorMenu[] = {
	{sensorMenu_Extruder, SetExtruderSensor, true},
	{sensorMenu_Bed, SetBedSensor, true},
	{menu_Exit, NULL, true},
};

MenuData unitMenu[] = {
	{unitMenu_CELSIUS, SetCelsiusUnits, true},
	{unitMenu_Farenheit, SetFahrenheitUnits, true},
};

MenuData sensorPickMenu[] = {
	{sensorMenu_Thermistor1, SetThermistor1, true},
	{sensorMenu_Thermistor2, SetThermistor2, true},
	{sensorMenu_Thermocouple1, SetThermocouple1, true},
	{sensorMenu_Thermocouple2, SetThermocouple2, true},
};

#define numMainMenuItems (sizeof(mainMenu) / sizeof(MenuData))
#define numTempMenuItems (sizeof(tempMenu) / sizeof(MenuData))
#define numPIDTuningMenuItems (sizeof(pidTuningMenu) / sizeof(MenuData))
#define numPIDTuningMenuValueItems (sizeof(pidTuningMenuValues) / sizeof(MenuData))
#define numSensorMenuItems (sizeof(sensorMenu) / sizeof(MenuData))
#define numSensorPickMenuItems (sizeof(sensorPickMenu) / sizeof(MenuData))
#define numUnitMenuItems (sizeof(unitMenu) / sizeof(MenuData))
#define numBaudMenuItems (sizeof(baudMenu) / sizeof(MenuData))

unsigned char KeyScan(unsigned long scanFor);

#if defined(BOARD_REV3)
void ProcessFan(void)
{
	unsigned long now = millis();
	if (now - lastFanTick >= 1000)
	{
		if (Heating)
		{
			if (FanTicks < MaxFanTicks)
				FanTicks++;
		}
		else
		{
			if (FanTicks > 0)
				FanTicks--;
		}
		lastFanTick = now;
	}
	if (FanTicks > 0)
		analogWrite(FanControl, 255);
	else
		analogWrite(FanControl, 64);
}
#endif

void DisplayMenu(MenuData *menu, uint8_t itemCount, const char PROGMEM *title, int8_t offset, int8_t selected, bool enabled)
{
	int maxCount = (itemCount > 3) ? 3 : itemCount;
	int row = 0;
	lcd.clear();
	lcd.print(Flash(title));
	setCursor(lcd, 0, ++row);
	for	(int i = 0; i < maxCount; i++)
	{
		lcd.print((selected == (i + offset)) ? ((enabled) ? "\x7E" : "\x7F") : " ");
		lcd.print(Flash(menu[i + offset].menuText));
		setCursor(lcd, 0, ++row);
	}
}

void ProcessMenu(MenuData *menu, uint8_t itemCount, const char PROGMEM *title, void *context = NULL)
{
	unsigned char key;
	int8_t offset = 0;
	int8_t selected = 0;
	bool update = true;
	while (1)
	{
		if (menu == &mainMenu[0])
			ctrlMode = MainMenu;
		else
			ctrlMode = Menu;
		if (update)
		{
			delay(100);
			DisplayMenu(menu, itemCount, title, offset, selected, menu[selected].enabled);
			update = false;
		}
		key = KeyScan(0);
		switch (key)
		{
		case UP:
			{
				selected--;
				if (selected < offset)
				{
					if (selected < 0)
					{
						selected = itemCount - 1;
						offset = selected - 2;
						if (offset < 0) 
							offset = 0;
					} else
						offset = selected;
				}
				update = true;
				break;
			}
		case DN:
			{
				selected++;
				if (selected >= min(itemCount, offset + 3))
				{
					if (selected >= itemCount)
					{
						selected = 0;
						offset = 0;
					} else
						offset++;
				}
				update = true;
				break;
			}
		case OK:
			{
				if (menu[selected].enabled)
				{
					if (menu[selected].menuFunc != NULL)
					{
						if (menu[selected].menuFunc(context, selected))
							return;
					} else
						return;
					update = true;
				}
				break;
			}
		case ESC: return;
		default:
			{
				if (key & 0x80)
				{
					unsigned char sel = key & ~0x80;
					if (menu[sel].enabled && menu[sel].menuFunc != NULL)
					{
						if (menu[sel].menuFunc(context, sel))
							return;
					}
					update = true;	
				}
			}
		}
	}
}

void ProcessValue(double &value, double delta, int lowLimit, int highLimit, const char PROGMEM *title, const char PROGMEM *prompt)
{
	bool update = true;
	int row;
	while (1)
	{
		ctrlMode = Menu;
		if (update)
		{
			delay(100);
			row = 0;
			lcd.clear();
			lcd.print(Flash(title));
			setCursor(lcd, 0, ++row);
			lcd.print(Flash(prompt));
			lcd.print(value);
			update = false;
		}
		switch (KeyScan(0))
		{
		case UP:
			{
				value += delta;
				if (value >= highLimit)
					value = lowLimit;
				update = true;
				break;
			}
		case DN:
			{
				value -= delta;
				if (value <= lowLimit)
					value = highLimit;
				update = true;
				break;
			}
		case ESC:
		case OK:
			{
				return;
			}
		}
	}
}

typedef struct _SerialContext {
	String command;
	unsigned char key;
} SerialContext;

double ReadTemperature(int sensor);
void SaveData(void);
void LoadData(void);

void SendOK(void)
{
	Serial.println(F("OK"));
}

String NextToken(String &str)
{
	String result;
	uint8_t i = 0;
	str.trim();
	while ((i < str.length()) && ((str[i] >= 'A' && str[i] <= 'Z') || (str[i] >= '0' && str[i] <= '9'))) i++;
	result = str.substring(0, i);
	str.remove(0, i);
	return result;
}

void SendLabel(const char PROGMEM *label, bool verbose)
{
	if (verbose && label != NULL)
	{
		Serial.print(Flash(label));
		Serial.print(F(": "));
	}
}

void SendTemp(const char PROGMEM *label, double temp, bool verbose, bool newLine)
{
	SendLabel(label, verbose);
	Serial.print(temp);
	if (newLine)
		Serial.println();
	else
		Serial.print(' ');
}

void SendSensor(const char PROGMEM *label, int sensor, bool verbose, bool newLine)
{
	SendLabel(label, verbose);
	switch (sensor)
	{
	case TM1: Serial.print(F("TM1")); break;
	case TM2: Serial.print(F("TM2")); break;
	case TC1: Serial.print(F("TC1")); break;
	case TC2: Serial.print(F("TC2")); break;
	default:
		Serial.print(F("Invalid: "));
		Serial.print(sensor);
	}
	if (newLine)
		Serial.println();
	else
		Serial.print(' ');
}

void SendFlags(const char PROGMEM *label, uint8_t mode, bool verbose, bool newLine)
{
	SendLabel(label, verbose);
	if (mode < 16)
		Serial.print('0');
	Serial.print(mode, HEX);
	if (newLine)
		Serial.println();
	else
		Serial.print(' ');
}

bool GetStatus(void *context, uint8_t index)
{
	String params = *(String *)context;
	String param = NextToken(params);
	bool verbose = param == F("V");
	SendTemp(sensorMenu_Thermistor1, ReadTemperature(TM1), verbose, true);
	SendTemp(sensorMenu_Thermistor2, ReadTemperature(TM2), verbose, true);
	SendTemp(sensorMenu_Thermocouple1, ReadTemperature(TC1), verbose, true);
	SendTemp(sensorMenu_Thermocouple2, ReadTemperature(TC2), verbose, true);
	SendTemp(tempMenu_Extruder, Extruder.Set_Temp, verbose, false);
	SendTemp(NULL, Extruder.Error_Temp, verbose, false);
	SendSensor(NULL, Extruder.Sensor, verbose, false);
	SendFlags(NULL, Extruder.Flags, verbose, true);
	SendTemp(tempMenu_Bed, Bed.Set_Temp, verbose, false);
	SendTemp(NULL, Bed.Error_Temp, verbose, false);
	SendSensor(NULL, Bed.Sensor, verbose, false);
	SendFlags(NULL, Bed.Flags, verbose, true);
	if (verbose)
	{
		Serial.print(F("Control Mode: "));
	}
	if (ctrlMode == MainMenu)
		Serial.println(F("MainMenu"));
	else if (ctrlMode == Menu)
		Serial.println(F("Menu"));
	else if (ctrlMode == Running)
		Serial.println(F("Running"));
	else
		Serial.println(F("<Err>"));
	return true;
}

enum DataType {typFloatRef, typIntRef, typCharRef, typSensorRef, typSensorVal};

typedef struct _Value {
	const char * PROGMEM name;
	DataType type;
	bool readOnly;
	union {
		void *anyRef;
		double *floatRef;
		int *intRef;
		uint8_t *charRef;
		uint8_t charVal;
	};
} Value;

const char value_ECT[] PROGMEM = {"ECT"}; //Extruder Current Temp
const char value_EST[] PROGMEM = {"EST"}; //Extruder Set Temp
const char value_EET[] PROGMEM = {"EET"}; //Extruder Error Temp
const char value_BCT[] PROGMEM = {"BCT"}; //Bed Current Temp
const char value_BST[] PROGMEM = {"BST"}; //Bed Set Temp
const char value_BET[] PROGMEM = {"BET"}; //Bed Error Temp
const char value_EKP[] PROGMEM = {"EKP"}; //Extruder PID KP
const char value_EKI[] PROGMEM = {"EKI"}; //Extruder PID KI
const char value_EKD[] PROGMEM = {"EKD"}; //Extruder PID KD
const char value_BKP[] PROGMEM = {"BKP"}; //Bed PID KP
const char value_BKI[] PROGMEM = {"BKI"}; //Bed PID KI
const char value_BKD[] PROGMEM = {"BKD"}; //Bed PID KD
const char value_TM1[] PROGMEM = {"TM1"}; //Thermistor 1
const char value_TM2[] PROGMEM = {"TM2"}; //Thermistor 2
const char value_TC1[] PROGMEM = {"TC1"}; //Thermocouple 1
const char value_TC2[] PROGMEM = {"TC2"}; //Thermocouple 2
const char value_CTL[] PROGMEM = {"CTL"}; //ctrlMode

const Value dataValues[] = {
	{value_ECT, typSensorRef, false, {&Extruder.Sensor}},
	{value_EST, typFloatRef, false, {&Extruder.Set_Temp}},
	{value_EET, typFloatRef, false, {&Extruder.Error_Temp}},
	{value_BCT, typSensorRef, false, {&Bed.Sensor}},
	{value_BST, typFloatRef, false, {&Bed.Set_Temp}},
	{value_BET, typFloatRef, false, {&Bed.Error_Temp}},
	{value_EKP, typFloatRef, false, {&ExtruderTuning.KP}},
	{value_EKI, typFloatRef, false, {&ExtruderTuning.KI}},
	{value_EKD, typFloatRef, false, {&ExtruderTuning.KD}},
	{value_BKP, typFloatRef, false, {&BedTuning.KP}},
	{value_BKI, typFloatRef, false, {&BedTuning.KI}},
	{value_BKD, typFloatRef, false, {&BedTuning.KD}},
	{value_TM1, typSensorVal, true, {(void *)((int)TM1)}},
	{value_TM2, typSensorVal, true, {(void *)((int)TM2)}},
	{value_TC1, typSensorVal, true, {(void *)((int)TC1)}},
	{value_TC2, typSensorVal, true, {(void *)((int)TC2)}},
	{value_CTL, typCharRef, true, {&ctrlMode}},
};

#define numDataValues (sizeof(dataValues) / sizeof(Value))

const Value *GetDataValueRec(String param)
{
	int i;
	if (param.length() > 0)
		for (i = 0; i < numDataValues; i++)
			if (param.compareTo(Flash(dataValues[i].name)) == 0)
				return &dataValues[i];
	return NULL;
}

void ResetRunningState(void *ref)
{
	if (ctrlMode == Running)
	{
		if (&Extruder.Current_Temp <= ref && ref <= &Extruder.Flags)
		{
			Extruder.Max_Temp = 0;
			Extruder.Flags &= ~(TempHit | TempError);
		} else
		if (&Bed.Current_Temp <= ref && ref <= &Bed.Flags)
		{
			Bed.Max_Temp = 0;
			Bed.Flags &= ~(TempHit | TempError);
		}
	}
}

bool GetValue(void *context, uint8_t index)
{
	int sensor;
	String params = *(String *)context;
	String param = NextToken(params);
	const Value *valRec = GetDataValueRec(param);
	if (valRec != NULL)
	{
		switch (valRec->type)
		{
		case typFloatRef: Serial.println(*valRec->floatRef); break;
		case typIntRef:   Serial.println(*valRec->intRef); break;
		case typCharRef:  Serial.println(*valRec->charRef); break;
		case typSensorRef: SendTemp(NULL, ReadTemperature(*valRec->intRef), false, true); break;
		case typSensorVal: SendTemp(NULL, ReadTemperature(valRec->charVal), false, true); break;
		}
		return true;
	}
	return false;
}

bool SetValue(void *context, uint8_t index)
{
	double *floatRef = NULL;
	int *intRef = NULL;
	uint8_t *charRef = NULL;
	String params = *(String *)context;
	String param = NextToken(params);
	const Value *valRec = GetDataValueRec(param);
	param = NextToken(params);
	if (valRec != NULL && !valRec->readOnly && param.length() > 0)
	{
		switch (valRec->type)
		{
		case typFloatRef: floatRef = valRec->floatRef; break;
		case typSensorRef:
		case typIntRef:   intRef = valRec->intRef; break;
		case typCharRef:  charRef = valRec->charRef; break;
		}
		if (floatRef != NULL)
		{
			double newVal = param.toFloat();
			if (newVal > 0.0 && newVal <= 500.0)
			{
				*floatRef = newVal;
				return true;
			}
		}
		if (intRef != NULL)
		{
			int newVal = param.toInt();
			*intRef = newVal;
			return true;
		}
		if (charRef != NULL)
		{
			uint8_t newVal = param.toInt();
			*charRef = newVal;
			return true;
		}
		ResetRunningState(valRec->anyRef);
	}
	return false;
}

bool EEData(void *context, uint8_t index)
{
	String params = *(String *)context;
	String param = NextToken(params);
	if (param == F("L"))
		LoadData();
	else if (param == F("S"))
		SaveData();
	else
		return false;
	SendOK();
	return true;
}

bool SetTempMode(TempData &tempData, String &params)
{
	if (ctrlMode == Running)
	{
		String param = NextToken(params);
		if (param.length() > 0)
		{
			if (param == "A")
				tempData.Flags = (tempData.Flags & ~TempModeMask) | TempModeAuto;
			else if (param == "O")
				tempData.Flags = (tempData.Flags & ~TempModeMask) | TempModeOn;
			else if (param == "F")
				tempData.Flags = (tempData.Flags & ~TempModeMask) | TempModeOff;
			else
				return false;
		} else
			return false;
	} else
		return false;
	return true;
}

bool SetMode(void *context, uint8_t index)
{
	SerialContext *ctx = (SerialContext *)context;
	String params = ctx->command;
	String param = NextToken(params);
	if (param == "O")
	{
	  if (ctrlMode == MainMenu)
		  ctx->key = 0x80;
	  else
		  return false;
	} else if (param == "F")
	{
		if (ctrlMode == Running)
		{
			ctrlMode = MainMenu;
		}
	} else if (param == "M")
	{
		param = NextToken(params);
		if (param.length() > 0)
			ctx->key = 0x80 | param.toInt();
		else
			return false;
	} else if (param == "K")
	{
		param = NextToken(params);
		if (param.length() > 0)
			ctx->key = param.toInt();
		else
			return false;
	} else if (param == "E")
	{
		return SetTempMode(Extruder, params);
	} else if (param == "B")
	{
		return SetTempMode(Bed, params);
	} else
		return false;
	return true;
}

bool CancelModes(void *context, uint8_t index)
{
	Reset = true;
	return true;
}

const char commandMenu_Status[] PROGMEM = {"ST"};
const char commandMenu_GetValue[] PROGMEM = {"GV"};
const char commandMenu_SetValue[] PROGMEM = {"SV"};
const char commandMenu_EEData[] PROGMEM = {"EE"};
const char commandMenu_SetMode[] PROGMEM = {"SM"};
const char commandMenu_Reset[] PROGMEM = {"RS"};
const char commandMenu_Attn[] PROGMEM   = {""};

MenuData commandMenu[] = {
	{commandMenu_Attn, NULL, true},
	{commandMenu_Status, GetStatus, true},
	{commandMenu_GetValue, GetValue, true},
	{commandMenu_SetValue, SetValue, true},
	{commandMenu_EEData, EEData, true},
	{commandMenu_SetMode, SetMode, true},
	{commandMenu_Reset, CancelModes, true},
};

#define numCommandMenuItems (sizeof(commandMenu) / sizeof(MenuData))

unsigned char ProcessSerial()
{
	SerialContext ctx;
	ctx.key = 0;
	if (Serial.available())
	{
		uint8_t i;
		ctx.command = Serial.readStringUntil('\r');
		String cmd;
		ctx.command.toUpperCase();
//		Serial.println(command);
		cmd = NextToken(ctx.command);
//		Serial.println(cmd);
		if (cmd == F("AT"))
		{
			cmd = NextToken(ctx.command);
			for	(i = 0; i < numCommandMenuItems; i++)
			{
				if (cmd.compareTo(Flash(commandMenu[i].menuText)) == 0)
				{
					if (commandMenu[i].menuFunc != NULL)
					{
						if (!commandMenu[i].menuFunc(&ctx, i))
							break;
					}
					SendOK();
					return ctx.key;
				}
			}
		}
		Serial.println(F("ERROR"));
	}
	return ctx.key;
}

void LoadPIDData(PIDData &data, int offset)
{
	offset += EEPROM_read(offset, data.KP);
	offset += EEPROM_read(offset, data.KI);
	offset += EEPROM_read(offset, data.KD);
}

void SavePIDData(PIDData &data, int offset)
{
	offset += EEPROM_write(offset, data.KP);
	offset += EEPROM_write(offset, data.KI);
	offset += EEPROM_write(offset, data.KD);
}

void LoadData()
{
	uint16_t valid = word(EEPROM.read(1), EEPROM.read(0));
	if (valid == 0x55AA)
	{
		Extruder.Sensor = EEPROM.read(2);
		Bed.Sensor = EEPROM.read(3);
		Extruder.Set_Temp = word(EEPROM.read(5),EEPROM.read(4));
		Bed.Set_Temp = word(EEPROM.read(7),EEPROM.read(6));
		Extruder.Error_Temp = word(EEPROM.read(11), EEPROM.read(10));
		Bed.Error_Temp = word(EEPROM.read(13), EEPROM.read(12));
		Display_Units = EEPROM.read(8);
		Restarted = (bool)EEPROM.read(9);
		LoadPIDData(ExtruderTuning, 14);
		LoadPIDData(BedTuning, 26);
		EEPROM_read(38, SerialBaud);
		if ((long)SerialBaud <= 0)
			SerialBaud = 19200;
	}
}

void SaveData()
{
	uint16_t valid = word(EEPROM.read(1), EEPROM.read(0));
	if (valid != 0x55AA)
	{
		EEPROM.write(0, 0xAA);
		EEPROM.write(1, 0x55);
	}
	EEPROM.write(2, Extruder.Sensor);
	EEPROM.write(3, Bed.Sensor);
	EEPROM.write(4, ((int)Extruder.Set_Temp) & 0xFF);
	EEPROM.write(5, ((int)Extruder.Set_Temp) >> 8);
	EEPROM.write(6, ((int)Bed.Set_Temp) & 0xFF);
	EEPROM.write(7, ((int)Bed.Set_Temp) >> 8);
	EEPROM.write(8, Display_Units);
	EEPROM.write(9, 0);
	EEPROM.write(10, ((int)Extruder.Error_Temp) & 0xFF);
	EEPROM.write(11, ((int)Extruder.Error_Temp) >> 8);
	EEPROM.write(12, ((int)Bed.Error_Temp) & 0xFF);
	EEPROM.write(13, ((int)Bed.Error_Temp) >> 8);
	SavePIDData(ExtruderTuning, 14);
	SavePIDData(BedTuning, 26);
	EEPROM_write(38, SerialBaud);
}

void SetRestarted(bool value)
{
	EEPROM.write(9, (uint8_t)value);
}

unsigned char KeyScan(unsigned long scanFor)
{
	unsigned long end = millis() + scanFor;
	while (1)
	{
#if defined(BOARD_REV3)
		ProcessFan();
#endif
		unsigned char key = ProcessSerial();
		if (key != 0)
			return key;
		if (UP_button.check() == ON) return (UP); 
		if (DN_button.check() == ON) return (DN); 
		if (OK_button.check() == ON) return (OK);
		if (Reset) return (ESC);
		if ((scanFor == 0) || (millis() >= end))
			break;
	}
	return NULL;
}

void setup()
{
	int RR = MCUSR;
	MCUSR = 0;
	wdt_disable();

	LoadData();

	Serial.begin(SerialBaud);
//	Serial.setTimeout(500);

	UP_button.setMode(OneShot);
	UP_button.assign(A1);
	DN_button.setMode(OneShot);
	DN_button.assign(A2);
	OK_button.setMode(OneShot);
	OK_button.assign(A3);

	pinMode(MOSFET_Extruder, OUTPUT);
	pinMode(MOSFET_Bed, OUTPUT);

#if defined(BOARD_REV3)
	pinMode(FanControl, OUTPUT);
	lastFanTick = millis();
#endif
	
	analogWrite(MOSFET_Extruder, 0);
	analogWrite(MOSFET_Bed, 0);
#if defined(BOARD_REV3)
	analogWrite(FanControl, 64);
#endif

	lcd.begin(16, 4); // 16 columns X 4 rows

	//lcd.print("Hello World");

	if (!Restarted)
		ShowResetReason(RR);
}

const char upButton[] PROGMEM = {"Up Button"};
const char dnButton[] PROGMEM = {"Dn Button"};
const char okButton[] PROGMEM = {"Ok Button"};

void loop()
{
	/*
	unsigned char key = KeyScan();
	switch (key)
	{
	case UP:
		{
			lcd.clear();
			lcd.print(Flash(upButton));
			break;
		}
	case DN:
		{
			lcd.clear();
			lcd.print(Flash(dnButton));
			break;
		}
	case OK:
		{
			lcd.clear();
			lcd.print(Flash(okButton));
			break;
		}
	}
	*/
	Reset = false;
	if (!Restarted)
		ProcessMenu(mainMenu, numMainMenuItems, mainMenu_title);
	else
	{
		SetRestarted(false);
		Restarted = false;
		Start(NULL, 0);
	}
}

void Restart()
{
	SetRestarted(true);
	wdt_enable(WDTO_15MS);
	while (1);
}

void InitPID()
{
	PIDExtruder.SetTunings(ExtruderTuning.KP, ExtruderTuning.KI, ExtruderTuning.KP);
	PIDBed.SetTunings(BedTuning.KP, BedTuning.KI, BedTuning.KP);
    PIDExtruder.SetMode(AUTOMATIC);
    PIDBed.SetMode(AUTOMATIC);
	Extruder.Flags = TempModeAuto;
	Bed.Flags = TempModeAuto;
}

double ReadTemperature(int sensor)
{
	switch (sensor & 0x3)
	{
	case TM1: return read_thermistor(Thermistor_One);
	case TM2: return read_thermistor(Thermistor_Two);
	case TC1: return Thermocouple_One.readCelsius();
	case TC2: return Thermocouple_Two.readCelsius();
	}
}

double TempForDisplay(double temp, int units)
{
	if (units != CELSIUS)
		temp = (temp * 9.0) / 5.0 + 32;
	return temp;
}

double TempInternal(double temp, int units)
{
	if (units != CELSIUS)
		temp = ((temp - 32) * 5.0) / 9.0;
	return temp;
}

bool Manage_Temp(PID *pid, PID_ATune *tuner, TempData &tempData, int mosfet)
{
	double newTemp;
	unsigned long started = millis();
	while (1)
	{
		newTemp = ReadTemperature(tempData.Sensor);
		if (!isnan(newTemp) && !isinf(newTemp))
			break;
		if (started - millis() >= 1000)
		{
			tempData.Flags |= TempError;
			return true;
		}
	}
	tempData.Current_Temp = newTemp;
	if (tempData.Start_Temp <= 0.0)
		tempData.Start_Temp = newTemp;
	if ((tempData.Flags & TempModeMask) != TempModeOff)
	{
		double errorOffset = tempData.Set_Temp - tempData.Error_Temp;
		if (tuner != NULL)
		{
			if (tuner->Runtime() != 0) return false;
		}
		else if (pid != NULL)
			pid->Compute();
		else
			return false;
		if (tempData.Set_Temp < tempData.Max_Temp)
			tempData.Max_Temp = tempData.Set_Temp;
		if ((tempData.Flags & TempModeMask) == TempModeOn)
			tempData.Output_Value = 255;
		if (tempData.Current_Temp >= tempData.Set_Temp)
			tempData.Flags = (tempData.Flags & ~TempModeMask) | TempModeAuto | TempHit;
		if (tempData.Current_Temp > tempData.Max_Temp && tempData.Current_Temp <= tempData.Set_Temp)
			tempData.Max_Temp = tempData.Current_Temp;
		if (!(tempData.Flags & TempControlled) && ((tempData.Start_Temp >= (tempData.Max_Temp - errorOffset)) || 
			(tempData.Current_Temp >= (tempData.Start_Temp + errorOffset))))
			tempData.Flags |= TempControlled;
		if (tempData.Flags & TempControlled)
		{
			tempData.Flags &= ~TempError;
			if (((tempData.Flags & TempHit) && tempData.Current_Temp <= tempData.Error_Temp) || (tempData.Current_Temp <= (tempData.Max_Temp - errorOffset)))
				tempData.Flags |= TempError;
		}
	}
	else
		tempData.Output_Value = 0;
	analogWrite(mosfet, tempData.Output_Value);
	return true;
}

void ManageTemperatures()
{
	Manage_Temp(&PIDExtruder, NULL, Extruder, MOSFET_Extruder);
	Manage_Temp(&PIDBed, NULL, Bed, MOSFET_Bed);
}

uint8_t ValidateModes()
{
	if (getPinMode(MOSFET_Extruder) != OUTPUT)
		return MOSFET_Extruder;
	if (getPinMode(MOSFET_Bed) != OUTPUT)
		return MOSFET_Bed;
	if (getPinMode(RS) != OUTPUT)
		return RS;
	if (getPinMode(EN) != OUTPUT)
		return EN;
	return NOT_A_PIN;
}

uint8_t ValidateTimer()
{
	uint8_t tccr1a = TCCR1A;
	uint8_t tccr1b = TCCR1B;
	uint8_t gtccr  = GTCCR;
	if (((tccr1a & (_BV(WGM10) | _BV(WGM11) | _BV(COM1B0) | _BV(COM1A0))) != 0x01) || 
		((tccr1b & (_BV(ICNC1) | _BV(ICES1) | _BV(WGM13) | _BV(WGM12) | _BV(CS12) | _BV(CS11) | _BV(CS10))) != 0x03) ||
		((gtccr  & (_BV(TSM) | _BV(PSRASY) | _BV(PSRSYNC))) != 0x00))
		return TIMER1A;
	else
		return NOT_ON_TIMER;
}

void ResetPinModes()
{
	pinMode(MOSFET_Extruder, OUTPUT);
	pinMode(MOSFET_Bed, OUTPUT);
	pinMode(RS, OUTPUT);
	pinMode(EN, OUTPUT);
}

void ResetTimerModes()
{
	TCCR1A = _BV(WGM10);
	TCCR1B = _BV(CS10) | _BV(CS11);
	GTCCR  = 0x00;
}

void DisplayTempData(const char PROGMEM *title, TempData &tempData, bool errorFlash, int &row)
{
	lcd.print(Flash(title));
	lcd.print(tempData.Flags & TempHit ? '*' : ' ');
	if (!(tempData.Flags & TempError) || errorFlash)
		lcd.print(TempForDisplay(tempData.Current_Temp, Display_Units));
	setCursor(lcd, 0, ++row);
	lcd.print(Flash(setTemp_prompt));
	if ((tempData.Flags & TempModeMask) == TempModeAuto)
		lcd.print(TempForDisplay(tempData.Set_Temp, Display_Units));
	else if	((tempData.Flags & TempModeMask) == TempModeOn)
		lcd.print(Flash(sON));
	else
		lcd.print(Flash(sOFF));
}

bool Start(void *context, uint8_t index)
{
	uint8_t modeError;
	uint8_t timerError;
	uint8_t key;
	bool ErrorFlash = true;
	int ErrorCount = 0;
	int row;
	unsigned long lastTime = millis() - 1000;
	unsigned long now;
	OutputValueExtruder = 0;
	OutputValueBed = 0;
	Extruder.Flags &= ~(TempHit | TempControlled | TempError);
	Extruder.Max_Temp = 0.0;
	Extruder.Start_Temp = 0.0;
	Bed.Flags &= ~(TempHit | TempControlled | TempError);
	Bed.Max_Temp = 0.0;
	Bed.Start_Temp = 0.0;
#if defined(BOARD_REV3)
	Heating = true;
#endif
	ctrlMode = Running;
	delay(100);
	InitPID();
	while (ctrlMode == Running)
	{
		ManageTemperatures();
		key = KeyScan(100);
		if (key == UP)
		{
			if ((Extruder.Flags & TempModeMask) == TempModeOff)
			{
				Extruder.Flags &= ~TempHit;
				Extruder.Max_Temp = 0.0;
				Extruder.Flags = (Extruder.Flags & ~TempModeMask) | TempModeAuto;
			}
			else
				Extruder.Flags += TempModeOn;
		}
		else if (key == DN)
		{
			if ((Bed.Flags & TempModeMask) == TempModeOff)
			{
				Bed.Flags &= ~TempHit;
				Bed.Max_Temp = 0.0;
				Bed.Flags = (Bed.Flags & ~TempModeMask) | TempModeAuto;
			}
			else
				Bed.Flags += TempModeOn;
		}
		else if	(key == OK || key == ESC)
			break;
		now = millis();
		if (now - lastTime >= 1000)
		{
			lastTime = now;
			modeError = ValidateModes();
			timerError = ValidateTimer();
			row = 0;
			lcd.clear();
			if (modeError == NOT_A_PIN && timerError == NOT_ON_TIMER)
			{
				DisplayTempData(extruderText, Extruder, ErrorFlash, row);
				setCursor(lcd, 0, ++row);
				DisplayTempData(bedText, Bed, ErrorFlash, row);
				if (Extruder.Flags & TempError || Bed.Flags & TempError)
				{
					if (++ErrorCount > 10)
						Restart();
				}
				else
					ErrorCount = 0;
			} 
			else
			{
				ResetPinModes();
				ResetTimerModes();
				if (modeError == MOSFET_Extruder)
					lcd.print(Flash(extruderModeError));
				else if (modeError == MOSFET_Bed)
					lcd.print(Flash(bedModeError));
				else if (modeError == RS)
					lcd.print(Flash(rsModeError));
				else if (modeError == EN)
					lcd.print(Flash(enModeError));
				else if (timerError == TIMER1A)
					lcd.print(Flash(timerModeError));
				setCursor(lcd, 0, ++row);
				lcd.print(Flash(resettingModesMsg));
				key = KeyScan(2000);
				if (key == OK || key == ESC)
					break;
			}
			ErrorFlash = !ErrorFlash;
		}
	}
	analogWrite(MOSFET_Extruder, 0);
	analogWrite(MOSFET_Bed, 0);
	PIDExtruder.SetMode(MANUAL);
	PIDBed.SetMode(MANUAL);
#if defined(BOARD_REV3)
	Heating = false;
#endif
	ctrlMode = MainMenu;
	return true;
}

bool TempSetup(void *context, uint8_t index)
{
	ProcessMenu(tempMenu, numTempMenuItems, tempMenu_title);
	return false;
}

bool PIDTuning(void *context, uint8_t index)
{
	ProcessMenu(pidTuningMenu, numPIDTuningMenuItems, pidTuningMenu_title);
	return false;
}

bool SensorSetup(void *context, uint8_t index)
{
	ProcessMenu(sensorMenu, numSensorMenuItems, sensorMenu_title);
	return false;
}

bool DisplayUnitSetup(void *context, uint8_t index)
{
	ProcessMenu(unitMenu, numUnitMenuItems, unitMenu_title, &Display_Units);
	SaveData();
	return true;
}

bool SetCelsiusUnits(void *context, uint8_t index)
{
	*(int *)context = CELSIUS;
	return true;
}

bool SetFahrenheitUnits(void *context, uint8_t index)
{
	*(int *)context = FAHRENHEIT;
	return true;
}

bool SetExtruderTemp(void *context, uint8_t index)
{
	double temp = TempForDisplay(Extruder.Set_Temp, Display_Units);
	ProcessValue(temp, 1.0, (Display_Units == CELSIUS) ? 0 : 32, (Display_Units == CELSIUS) ? 350 : 660, tempMenu_Extruder, setTemp_prompt);
	Extruder.Set_Temp = TempInternal(temp, Display_Units);
	Extruder.Error_Temp = Extruder.Set_Temp - 5;
	SaveData();
	return false;
}

bool SetExtruderError(void *context, uint8_t index)
{
	double temp = TempForDisplay(Extruder.Error_Temp, Display_Units);
	ProcessValue(temp, 1.0, (Display_Units == CELSIUS) ? 0 : 32, (Display_Units == CELSIUS) ? 345 : 651, tempMenu_Extruder_Error, setError_prompt);
	Extruder.Error_Temp = TempInternal(temp, Display_Units);
	SaveData();
	return false;
}

bool SetBedTemp(void *context, uint8_t index)
{
	double temp = TempForDisplay(Bed.Set_Temp, Display_Units);
	ProcessValue(temp, 1.0, (Display_Units == CELSIUS) ? 0 : 32, (Display_Units == CELSIUS) ? 350 : 660, tempMenu_Bed, setTemp_prompt);
	Bed.Set_Temp = TempInternal(temp, Display_Units);
	Bed.Error_Temp = Bed.Set_Temp - 5;
	SaveData();
	return false;
}

bool SetBedError(void *context, uint8_t index)
{
	double temp = TempForDisplay(Bed.Error_Temp, Display_Units);
	ProcessValue(temp, 1.0, (Display_Units == CELSIUS) ? 0 : 32, (Display_Units == CELSIUS) ? 145 : 293, tempMenu_Bed_Error, setError_prompt);
	Bed.Error_Temp = TempInternal(temp, Display_Units);
	return false;
}

bool SetExtruderPID(void *context, uint8_t index)
{
	ProcessMenu(pidTuningMenuValues, numPIDTuningMenuValueItems, pidTuningMenuExtruder_title, &ExtruderTuning);
	return false;
}

bool SetBedPID(void *context, uint8_t index)
{
	ProcessMenu(pidTuningMenuValues, numPIDTuningMenuValueItems, pidTuningMenuBed_title, &BedTuning);
	return false;
}

void TunePID(PID_ATune *tuner, TempData &tempData, int mosfet, const char PROGMEM *title, PIDData PROGMEM *pidData)
{
	int row;
	uint8_t key;
//	TempMode mode = tmAuto;
	unsigned long lastTime = millis() - 1000;
	unsigned long now;
//	output = 0;
	tempData.Output_Value = 0.0;
	tuner->SetNoiseBand(1);
	tuner->SetControlType(1); // set to PID
	tuner->SetOutputStep(255);
	tuner->SetLookbackSec(20);
	tuner->Cancel();
	while (1)
	{
		if (!Manage_Temp(NULL, tuner, tempData, mosfet))
			break;
		key = KeyScan(100);
		if (key == OK || key == ESC)
		{
			tuner->Cancel();
			tempData.Output_Value = 0.0;
			analogWrite(mosfet, 0);
			return;
		}
		now = millis();
		if (now - lastTime > 1000)
		{
			lastTime = now;
			row = 0;
			lcd.clear();
			lcd.print(Flash(title));
			setCursor(lcd, 0, ++row);
			lcd.print(Flash(sensorText));
			lcd.print(TempForDisplay(tempData.Current_Temp, Display_Units));
		}
	}
	tempData.Output_Value = 0.0;
	analogWrite(mosfet, 0);
	pidData->KP = tuner->GetKp();
	pidData->KI = tuner->GetKi();
	pidData->KD = tuner->GetKd();
	row = 0;
	lcd.clear();
	lcd.print(Flash(title));
	setCursor(lcd, 0, ++row);
	lcd.print(Flash(setKP_prompt));
	lcd.print(pidData->KP);
	setCursor(lcd, 0, ++row);
	lcd.print(Flash(setKI_prompt));
	lcd.print(pidData->KI);
	setCursor(lcd, 0, ++row);
	lcd.print(Flash(setKD_prompt));
	lcd.print(pidData->KD);
	do
	{
		key = KeyScan(5000);
	}
	while(key != OK && key != ESC);
	SaveData();
}

bool TuneExtruderPID(void *context, uint8_t index)
{
	PID_ATune tuner(&Extruder.Set_Temp, &Extruder.Output_Value);
	TunePID(&tuner, Extruder, MOSFET_Extruder, tuningExtruder_title, &ExtruderTuning);
	return false;
}

bool TuneBedPID(void *context, uint8_t index)
{
	PID_ATune tuner(&Bed.Set_Temp, &Bed.Output_Value);
	TunePID(&tuner, Bed, MOSFET_Bed, tuningBed_title, &BedTuning);
	return false;
}

bool SetKP(void *context, uint8_t index)
{
	PIDData *pidData = (PIDData *)context;
	ProcessValue(pidData->KP, 1.0, 1.0, 400, (pidData == &ExtruderTuning) ? pidTuningMenuExtruder_title : pidTuningMenuBed_title, setKP_prompt);
	SaveData();
	return false;
}

bool SetKI(void *context, uint8_t index)
{
	PIDData *pidData = (PIDData *)context;
	ProcessValue(pidData->KI, .01, 0.00, 2.00, (pidData == &ExtruderTuning) ? pidTuningMenuExtruder_title : pidTuningMenuBed_title, setKI_prompt);
	SaveData();
	return false;
}

bool SetKD(void *context, uint8_t index)
{
	PIDData *pidData = (PIDData *)context;
	ProcessValue(pidData->KD, 1.0, 1.0, 400, (pidData == &ExtruderTuning) ? pidTuningMenuExtruder_title : pidTuningMenuBed_title, setKD_prompt);
	SaveData();
	return false;
}

bool SetExtruderSensor(void *context, uint8_t index)
{
	for	(int i = 0; i < numSensorPickMenuItems; i++)
		sensorPickMenu[i].enabled = (i != Bed.Sensor);
	ProcessMenu(sensorPickMenu, numSensorPickMenuItems, extruderMenu_title, &Extruder.Sensor);
	SaveData();
	return false;
}

bool SetBedSensor(void *context, uint8_t index)
{
	for	(int i = 0; i < numSensorPickMenuItems; i++)
		sensorPickMenu[i].enabled = (i != Extruder.Sensor);
	ProcessMenu(sensorPickMenu, numSensorPickMenuItems, bedMenu_title, &Bed.Sensor);
	SaveData();
	return false;
}

bool SetThermistor1(void *context, uint8_t index)
{
	*(int *)context = TM1;
	return true;
}

bool SetThermistor2(void *context, uint8_t index)
{
	*(int *)context = TM2;
	return true;
}

bool SetThermocouple1(void *context, uint8_t index)
{
	*(int *)context = TC1;
	return true;
}

bool SetThermocouple2(void *context, uint8_t index)
{
	*(int *)context = TC2;
	return true;
}

bool ReadSensors(void *context, uint8_t index)
{
	int row;
	uint8_t key;
	double temp;
	delay(100);
	while (1)
	{
		row = 0;
		lcd.clear();
		lcd.print(Flash(label_Thermistor1));
		temp = TempForDisplay(ReadTemperature(TM1), Display_Units);
		lcd.print(temp);
		setCursor(lcd, 0, ++row);
		lcd.print(Flash(label_Thermistor2));
		temp = TempForDisplay(ReadTemperature(TM2), Display_Units);
		lcd.print(temp);
		setCursor(lcd, 0, ++row);
		lcd.print(Flash(label_Thermocouple1));
		temp = TempForDisplay(ReadTemperature(TC1), Display_Units);
		lcd.print(temp);
		setCursor(lcd, 0, ++row);
		lcd.print(Flash(label_Thermocouple2));
		temp = TempForDisplay(ReadTemperature(TC2), Display_Units);
		lcd.print(temp);
		setCursor(lcd, 0, ++row);
		key = KeyScan(1000);
		if (key == OK || key == ESC)
			break;
	}
	return true;
}

bool ShowEEPROM(void *context, uint8_t index)
{
	uint8_t key;
	int offset = 0;
	bool update = true;
	while (1)
	{
		if (update)
		{
			lcd.clear();
			for	(int i = 0; i < 16; i++)
			{
				if (i % 4 == 0)
					setCursor(lcd, 0, i / 4);
				uint8_t data = EEPROM.read(i + offset);
				if (data < 16)
					lcd.print('0');
				lcd.print(data, 16);
				lcd.print(' ');
			}
			update = false;
			delay(100);
		}
		key = KeyScan(0);
		if (key == UP)
		{
			if (offset > 0)
			{
				offset -= 4;
				update = true;
			}
		}
		else if (key == DN)
		{
			if (offset < (1024 - 16))
			{
				offset += 4;
				update = true;
			}
		}
		else if (key == OK || key == ESC)
			break;
	}
	return true;
}

bool SerialSetup(void *context, uint8_t index)
{
	int i;
	for (i = 0; i < numBaudMenuItems; i++)
		baudMenu[i].enabled = true;
	switch (SerialBaud)
	{
	case 9600: baudMenu[0].enabled = false; break;
	case 19200: baudMenu[1].enabled = false; break;
	case 38400: baudMenu[2].enabled = false; break;
	case 57600: baudMenu[3].enabled = false; break;
	case 115200: baudMenu[4].enabled = false; break;
	}
	ProcessMenu(baudMenu, numBaudMenuItems, baudMenu_title);
	return true;
}

bool SetSerialBaud(void *context, uint8_t index)
{
	switch (index)
	{
	case 0: SerialBaud = 9600; break;
	case 1: SerialBaud = 19200; break;
	case 2: SerialBaud = 38400; break;
	case 3: SerialBaud = 57600; break;
	case 4: SerialBaud = 115200; break;
	}
	SaveData();
	return true;
}

// Thermistor lookup table for RepRap Temperature Sensor Boards (http://make.rrrf.org/ts)
// Made with createTemperatureLookup.py (http://svn.reprap.org/trunk/reprap/firmware/Arduino/utilities/createTemperatureLookup.py)
// ./createTemperatureLookup.py --r0=100000 --t0=25 --r1=0 --r2=4700 --beta=3950 --max-adc=1023
// r0: 100000
// t0: 25
// r1: 0
// r2: 4700
// beta: 3950
// max adc: 1023
#define NUMTEMPS 40
short temptable[NUMTEMPS][2] = {
   {1, 938},
   {27, 326},
   {53, 269},
   {79, 239},
   {105, 219},
   {131, 204},
   {157, 192},
   {183, 182},
   {209, 174},
   {235, 166},
   {261, 160},
   {287, 153},
   {313, 148},
   {339, 143},
   {365, 138},
   {391, 133},
   {417, 129},
   {443, 125},
   {469, 120},
   {495, 116},
   {521, 113},
   {547, 109},
   {573, 105},
   {599, 101},
   {625, 98},
   {651, 94},
   {677, 90},
   {703, 86},
   {729, 82},
   {755, 78},
   {781, 74},
   {807, 70},
   {833, 65},
   {859, 60},
   {885, 54},
   {911, 48},
   {937, 41},
   {963, 31},
   {989, 18},
   {1015, -8}
};


int read_thermistor(int THERMISTOR)
{
	int rawtemp = analogRead(THERMISTOR);
	int current_celsius = 0;

	byte i;
	for (i=1; i<NUMTEMPS; i++)
	{
		if (temptable[i][0] > rawtemp)
		{
			int realtemp  = temptable[i-1][1] + (rawtemp - temptable[i-1][0]) * (temptable[i][1] - temptable[i-1][1]) / (temptable[i][0] - temptable[i-1][0]);

			if (realtemp > 255)
			realtemp = 255; 

			current_celsius = realtemp;

			break;
		}
	}

	// Overflow: We just clamp to 0 degrees celsius
	if (i == NUMTEMPS)
		current_celsius = 0;

   return current_celsius;
}

const char sPORF[] PROGMEM =  {"PORF - Power On"};
const char sEXTRF[] PROGMEM = {"EXTRF- External"};
const char sBORF[] PROGMEM =  {"BORF - Brown Out"};
const char sWDRF[] PROGMEM =  {"WDRF - Watch Dog"};

const char *const sRR[] PROGMEM = {sPORF, sEXTRF, sBORF, sWDRF};

void ShowResetReason(int RR)
{
	int bit = 0;
	int row = 0;
	unsigned char key;
	unsigned long timeout;
	unsigned long start;
	unsigned long elapsed;
	lcd.clear();

	while (bit < 4)
	{
		if (RR & (1 << bit))
		{
			//lcd.print(Flash(sRR[bit]));
			lcd.print(Flash((const char *)pgm_read_word(&(sRR[bit]))));
			setCursor(lcd, 0, ++row);
		}
		bit++;
	}

	start = millis();
	timeout = 5000;
	while (1) 
	{
		key = KeyScan(timeout);
		if (key == OK || key == ESC)
			break;
		elapsed = millis() - start;
		if (elapsed < timeout)
			timeout -= elapsed;
		else
			break;
	} 
}

uint8_t getPinMode(uint8_t pin)
{
	uint8_t bit = digitalPinToBitMask(pin);
	uint8_t port = digitalPinToPort(pin);
	volatile uint8_t *reg, *out;

	if (port == NOT_A_PIN) return (uint8_t)0xFF;

	reg = portModeRegister(port);
	out = portOutputRegister(port);

	if (!(*reg & bit))
		if (*out & bit)
			return INPUT_PULLUP;
		else
			return INPUT;
	else
		return OUTPUT;
}

void PrintMode(const char PROGMEM *title, uint8_t pin)
{
	uint8_t mode;
	lcd.print(Flash(title));
	mode = getPinMode(pin);
	if (mode == INPUT)
		lcd.print(Flash(sINPUT));
	else if (mode == INPUT_PULLUP)
		lcd.print(Flash(sPULLUP));
	else if (mode == OUTPUT)
		lcd.print(Flash(sOUTPUT));
	else
		lcd.print(Flash(sERROR));
}

bool ShowPinModes(void *context, uint8_t index)
{
	uint8_t key;
	lcd.clear();
	int row = 0;
	PrintMode(sExtruder, MOSFET_Extruder);
	setCursor(lcd, 0, ++row);
	PrintMode(sBed, MOSFET_Bed);
	setCursor(lcd, 0, ++row);
	PrintMode(sRegSel, RS);
	setCursor(lcd, 0, ++row);
	PrintMode(sEnable, EN);
	while (1)
	{
		key = KeyScan(1000);
		if (key == OK || key == ESC)
			break;
	}
	return true;
}

void PrintBin(uint8_t value)
{
	uint8_t mask = 0x80;
	while (mask)
	{
		if (value & mask)
			lcd.print('1');
		else
			lcd.print('0');
		mask >>= 1;
	}
}

bool ShowTimerMode(void *context, uint8_t index)
{
	uint8_t TCR;
	uint8_t key;
	lcd.clear();
	int row = 0;
	lcd.print(Flash(sTCCR1A));
	TCR = TCCR1A;
	PrintBin(TCR);
	setCursor(lcd, 0, ++row);
	lcd.print(Flash(sTCCR1B));
	TCR = TCCR1B;
	PrintBin(TCR);
	setCursor(lcd, 0, ++row);
	lcd.print(Flash(sGTCCR));
	TCR = GTCCR;
	PrintBin(TCR);
	while (1)
	{
		key = KeyScan(1000);
		if (key == OK || key == ESC)
			break;
	}
	return true;
}

bool WatchdogReset(void *context, uint8_t index)
{
	wdt_enable(WDTO_15MS);
	while (1);
}