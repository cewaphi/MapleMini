// vim: noexpandtab tabstop=2 shiftwidth=2 softtabstop=2
/*
    ChibiOS - Copyright (C) 2006..2015 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/*
 * MapleMini has a STM32F103CB
 * Docs: http://docs.leaflabs.com/static.leaflabs.com/pub/leaflabs/maple-docs/0.0.12/hardware/maple-mini.html
 * Schematic: https://github.com/leaflabs/maplemini/raw/master/maplemini.pdf
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ch.h"
#include "hal.h"
#include "test.h"

#include "shell.h"
#include "chprintf.h"

#include "wdtSetup.h"
#include "obmqSetup.h"
#include "usbcfg.h"

#include "base64.h"

#include "nullstreams.h"

#define MIN(x, y) ( ((x)<(y)) ? (x) : (y) )


//#define SHELL_OPTION_PARSING_DEBUG

/* Configuration for command "gpio" */
struct pinPort {
	bool as_gpio;
	bool as_adc;
	char *pinNrString;
	uint8_t pin;
	GPIO_TypeDef *gpio;
	uint8_t adcchan;
};

#define ADC_CHANNEL_NONE 255

struct pinPort pinPorts[] = {
	//	GPIO?, ADC?,  MaplePin, Pin, PinGroup, ADC Channel
	{false, true,  "3",      0,  GPIOB,    ADC_CHANNEL_IN8},  // works
	{false, true,  "4",      7,  GPIOA,    ADC_CHANNEL_IN7},  // TBD
	{false, true,  "5",      6,  GPIOA,    ADC_CHANNEL_IN6},  // TBD
	{false, true,  "6",      5,  GPIOA,    ADC_CHANNEL_IN5},  // TBD
	{false, true,  "7",      4,  GPIOA,    ADC_CHANNEL_IN4},  // TBD
	{false, true,  "8",      3,  GPIOA,    ADC_CHANNEL_IN3},  // works
	{false, true,  "9",      2,  GPIOA,    ADC_CHANNEL_IN2},  // TBD
	{true, false, "12",     15,  GPIOC,    ADC_CHANNEL_NONE}, // works
	{true, false, "13",     14,  GPIOC,    ADC_CHANNEL_NONE}, // works
	{true, false, "17",      5,  GPIOB,    ADC_CHANNEL_NONE}, // GPIO TO BE TESTED
	{true, false, "18",      4,  GPIOB,    ADC_CHANNEL_NONE}, // works
	{true, false, "19",      3,  GPIOB,    ADC_CHANNEL_NONE}, // works
	{true, false, "20",     15,  GPIOA,    ADC_CHANNEL_NONE}, // works
	{true, false, "21",     14,  GPIOA,    ADC_CHANNEL_NONE}, // works
	{true, false, "22",     13,  GPIOA,    ADC_CHANNEL_NONE}, // works
	{true, false, "25",     10,  GPIOA,    ADC_CHANNEL_NONE}, // works
	{true, false, "26",      9,  GPIOA,    ADC_CHANNEL_NONE}, // works
	// 27 is used for pwm capture input (GPIOA8)
	{true, false, "28",     15,  GPIOB,    ADC_CHANNEL_NONE}, // works
	{true, false, "29",     14,  GPIOB,    ADC_CHANNEL_NONE}, // works
	{true, false, "30",     13,  GPIOB,    ADC_CHANNEL_NONE}, // works
	{true, false, "31",     12,  GPIOB,    ADC_CHANNEL_NONE}, // works
};

int pinIndexForMaplePin(const char * maplePin, bool onlyIfGpio, bool onlyIfAdc, bool assertIfNone)
{
	unsigned i;
	for(i = 0; i < sizeof(pinPorts)/sizeof(pinPorts[0]); i++) {
		if(onlyIfGpio && !pinPorts[i].as_gpio)
			continue;
		if(onlyIfAdc && !pinPorts[i].as_adc)
			continue;
		if(0 == strcmp(pinPorts[i].pinNrString, maplePin))
			return i;
	}
	while(assertIfNone) { /* endless loop */ };
	return -1;
}

/* Configuration for command "uart" */
struct uartPort {
	char *uartPortString;
	uint8_t txPin, rxPin;
	GPIO_TypeDef *txGpio, *rxGpio;
	SerialDriver *sdp;
};

struct uartPort uartPorts[] = {
	{"1", 6, 7, GPIOB, GPIOB, &SD1},  //Maple: Tx: D16, Rx: D15; STM32: Tx: PB6,  Rx: PB7 ; 5V tolerant; REMAP!
	{"2", 2, 3, GPIOA, GPIOA, &SD2},  //Maple: Tx: D9,  Rx: D8;  STM32: Tx: PA2,  Rx: PA3 ; NOT 5V tolerant
	{"3", 10, 11, GPIOB, GPIOB, &SD3} //Maple: Tx: D1,  Rx: D0;  STM32: Tx: PB10, Rx: PB11; 5V tolerant
};

/* Configuration for I2C commands and sensors */
I2CConfig i2ccfg = {
	OPMODE_I2C,
	8000,		// 10kHZ I2C Frequency
	STD_DUTY_CYCLE
};

struct i2cPort {
	I2CDriver *i2interf;
	GPIO_TypeDef *gpioSCL;
	uint8_t pinSCL;
	GPIO_TypeDef *gpioSDA;
	uint8_t pinSDA;
};

struct i2cPort i2cPorts[] = {
	{ &I2CD1, GPIOB, 6, GPIOB, 7 },
	{ &I2CD2, GPIOB, 10, GPIOB, 11 }
};

typedef enum {
	TEMP,
	HUMID
} sensorType;

typedef struct {
	I2CDriver *port;		// Which I2C Port it is connected to
	uint8_t addr;			// Address of sensor @ port
	sensorType type;		// "TEMP", "HUMID", etc..
	uint8_t cmd;			// Command to request data
	float (*fDecode)(uint16_t);	// Pointer to a function that decodes received data
	bool immediate;			// Indicates whether sensor provides data immediately or after measurement
} i2cSensor;

#define MAX_NUM_SENSORS 16
i2cSensor sensorList[MAX_NUM_SENSORS];

/*===========================================================================*/
/* Command line related.                                                     */
/*===========================================================================*/

#define SHELL_WA_SIZE   THD_WORKING_AREA_SIZE(8192)

static void cmd_gpio(BaseSequentialStream *chp, int argc, char *argv[]) {
	uint8_t i;
	char *dOpt = NULL, *pOpt = NULL, *vOpt = NULL;

	obmqSendMessage(MSG_CMD_GPIO);

	for(i = 0; i < argc; i++) {
		if(strcmp(argv[i], "-d") == 0) {
			if(++i >= argc) continue;
#ifdef SHELL_OPTION_PARSING_DEBUG
			chprintf(chp, "detected -d with val: %s\r\n", argv[i]);
#endif
			dOpt = argv[i];
		}
		else if(strcmp(argv[i], "-p") == 0) {
			if(++i >= argc) continue;
#ifdef SHELL_OPTION_PARSING_DEBUG
			chprintf(chp, "detected -p with val: %s\r\n", argv[i]);
#endif
			pOpt = argv[i];
		}
		else if(strcmp(argv[i], "-v") == 0) {
			if(++i >= argc) continue;
#ifdef SHELL_OPTION_PARSING_DEBUG
			chprintf(chp, "detected -v with val: %s\r\n", argv[i]);
#endif
			vOpt = argv[i];
		}
	}

	if(!dOpt
			|| !pOpt
			|| ((strcmp(dOpt, "in") == 0) && vOpt)
			|| ((strcmp(dOpt, "out") == 0) && !vOpt)
			|| !(((strcmp(dOpt, "out") == 0) || (strcmp(dOpt, "in") == 0)))) {
		chprintf(chp, "Usage: gpio -d <direction> -p <pin> [-v <value>]\r\n"
				"\twith direction:\r\n"
				"\t\tin | out\r\n"
				"\twith pin:\r\n"
				"\t\t");
		for(i = 0; i < sizeof(pinPorts)/sizeof(pinPorts[0]); i++)
			if(pinPorts[i].as_gpio)
				chprintf(chp, "%s | ", pinPorts[i].pinNrString);
		chprintf(chp,	 "\r\n"
				"\twith val: (only if direction out)\r\n"
				"\t\t0 | 1\r\n");
		return;
	}

	for(i = 0; i < sizeof(pinPorts)/sizeof(pinPorts[0]); i++) {
		if((pinPorts[i].as_gpio) && (strcmp(pOpt, pinPorts[i].pinNrString) == 0)) {
			if(strcmp(dOpt, "out") == 0) {
				palSetPadMode(pinPorts[i].gpio, pinPorts[i].pin, PAL_MODE_OUTPUT_PUSHPULL);
				if(strcmp(vOpt, "1") == 0) {
					palSetPad(pinPorts[i].gpio, pinPorts[i].pin);
#ifdef SHELL_OPTION_PARSING_DEBUG
					chprintf(chp, "Set of Pin %s\r\n", pinPorts[i].pinNrString);
#else
					chprintf(chp, "Ok\r\n");
#endif
				}
				else if(strcmp(vOpt, "0") == 0) {
					palClearPad(pinPorts[i].gpio, pinPorts[i].pin);
#ifdef SHELL_OPTION_PARSING_DEBUG
					chprintf(chp, "Clear of Pin %s\r\n", pinPorts[i].pinNrString);
#else
					chprintf(chp, "Ok\r\n");
#endif
				}
			} else if(strcmp(dOpt, "in") == 0) {
				palSetPadMode(pinPorts[i].gpio, pinPorts[i].pin, PAL_MODE_INPUT);
#ifdef SHELL_OPTION_PARSING_DEBUG
				chprintf(chp, "Read of Pin %s:%d\r\n",
						pinPorts[i].pinNrString,
						palReadPad(pinPorts[i].gpio, pinPorts[i].pin));
#else
				chprintf(chp, "%d\r\n", palReadPad(pinPorts[i].gpio, pinPorts[i].pin));
#endif
			} else {
				chprintf(chp, "Bad direction '%s'\r\n", dOpt);
			}
			return; // there is at most one matching gpio. no need to check the others.
		}
	}
	chprintf(chp, "Bad gpio '%s'\r\n", pOpt);
}





static void cmd_uart(BaseSequentialStream *chp, int argc, char *argv[]) {
	uint8_t i;
	char *dOpt = NULL, *pOpt = NULL, *vOpt = NULL, *lOpt = NULL, *tOpt = NULL, *bOpt = NULL;

	obmqSendMessage(MSG_CMD_UART);

	for(i = 0; i < argc; i++) {
		if(strcmp(argv[i], "-d") == 0) {
			if(++i >= argc) continue;
			dOpt = argv[i];
#ifdef SHELL_OPTION_PARSING_DEBUG
			chprintf(chp, "Direction %s detected\r\n", dOpt);
#endif
		}
		else if(strcmp(argv[i], "-p") == 0) {
			if(++i >= argc) continue;
			pOpt = argv[i];
#ifdef SHELL_OPTION_PARSING_DEBUG
			chprintf(chp, "Port %s detected\r\n", pOpt);
#endif
		}
		else if(strcmp(argv[i], "-v") == 0) {
			if(++i >= argc) continue;
			vOpt = argv[i];
#ifdef SHELL_OPTION_PARSING_DEBUG
			chprintf(chp, "Value %s detected\r\n", vOpt);
#endif
		}
		else if(strcmp(argv[i], "-l") == 0) {
			if(++i >= argc) continue;
			lOpt = argv[i];
#ifdef SHELL_OPTION_PARSING_DEBUG
			chprintf(chp, "Length %s detected\r\n", lOpt);
#endif
		}
		else if(strcmp(argv[i], "-t") == 0) {
			if(++i >= argc) continue;
			tOpt = argv[i];
#ifdef SHELL_OPTION_PARSING_DEBUG
			chprintf(chp, "Timeout %s detected\r\n", tOpt);
#endif
		}
		else if(strcmp(argv[i], "-b") == 0) {
			if(++i >= argc) continue;
			bOpt = argv[i];
#ifdef SHELL_OPTION_PARSING_DEBUG
			chprintf(chp, "Baudrate %s detected\r\n", bOpt);
#endif
		}
	}

	if((!dOpt)
			|| (!pOpt)
			|| !(((strcmp(dOpt, "out") == 0) || (strcmp(dOpt, "in") == 0)))
			|| ((strcmp(dOpt, "in") == 0) && vOpt)
			|| ((strcmp(dOpt, "in") == 0) && !lOpt && !tOpt && !vOpt)
			|| ((strcmp(dOpt, "in") == 0) && !lOpt && tOpt && !vOpt)
			|| ((strcmp(dOpt, "in") == 0) && lOpt && !tOpt && !vOpt)
			|| ((strcmp(dOpt, "out") == 0) && lOpt)
			|| ((strcmp(dOpt, "out") == 0) && tOpt)
			|| ((strcmp(dOpt, "out") == 0) && !vOpt)
			|| (bOpt && !((strcmp(bOpt, "9600") == 0)
					|| (strcmp(bOpt, "19200") == 0)
					|| (strcmp(bOpt, "38400") == 0)
					|| (strcmp(bOpt, "57600") == 0)
					|| (strcmp(bOpt, "115200") == 0)))) {
		chprintf(chp, "Usage: depending on direction\r\n"
				"\treading from uart:\r\n"
				"\t\tuart -d in -p <port> -l <length> -t <timeout> [-b <baudrate>]\r\n"
				"\twriting to uart:\r\n"
				"\t\tuart -d out -p <port> -v <data> [-b <baudrate>]\r\n"
				"\r\n"
				"\twith port:\r\n"
				"\t\t1 | 2 | 3\r\n"
				"\t\t\t(assigned pins:\r\n"
				"\t\t\t\tPort    MapleMini STM32   5V tolerant?\r\n"
				"\t\t\t\t 1 Rx:   15        PB7     yes\r\n"
				"\t\t\t\t 1 Tx:   16        PB6     yes\r\n"
				"\t\t\t\t 2 Rx:    8        PA3     NO\r\n"
				"\t\t\t\t 2 Tx:    9        PA2     NO\r\n"
				"\t\t\t\t 3 Rx:    0        PB11    yes\r\n"
				"\t\t\t\t 3 Tx:    1        PB10    yes\r\n"
				"\t\t\t)\r\n"
				"\r\n"
				"\twith length:\r\n"
				"\t\tinteger -- blocks until bytes are read or timeout\r\n"
				"\r\n"
				"\twith timeout:\r\n"
				"\t\tinteger -- timeout in ticks\r\n"
				"\r\n"
				"\twith baudrate:\r\n"
				"\t\tinteger -- optional baudrate parameter\r\n"
				"\t\t(if no paramter is given default baudrate is 115200)\r\n"
				"\t\tsupported baudrates are 9600, 19200, 38400, 57600, 115200\r\n");
		return;
	}

	SerialConfig baudrateConfig;
	if(bOpt)
		baudrateConfig.speed = atoi(bOpt);
	else
		baudrateConfig.speed = 115200;

	for(i = 0; i < sizeof(uartPorts)/sizeof(uartPorts[0]); i++) {
		if(strcmp(pOpt, uartPorts[i].uartPortString) == 0) {
			palSetPadMode(uartPorts[i].txGpio, uartPorts[i].txPin, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
			palSetPadMode(uartPorts[i].rxGpio, uartPorts[i].rxPin, PAL_MODE_INPUT);
			sdStart(uartPorts[i].sdp, &baudrateConfig);
			if(strcmp(dOpt, "in") == 0) {
				unsigned char rawString[MAX_RAW_STRING_LENGTH], encodedString[MAX_BASE64_STRING_LENGTH];
				int timeout = atoi(tOpt);
				int length = atoi(lOpt);
				if(length > MAX_RAW_STRING_LENGTH)
					chprintf(chp, "Length %d not supported. Maximum Length is %d.", length, MAX_RAW_STRING_LENGTH);
				if(sdReadTimeout(uartPorts[i].sdp, rawString, length, timeout)) {
					size_t encodedLength;
					base64_encode(rawString, length, encodedString, &encodedLength);
					chprintf(chp, "%.*s\r\n", encodedLength, encodedString);
				} else {
					chprintf(chp, "Timeout\r\n");
				}
			} else if(strcmp(dOpt, "out") == 0) {
				size_t decodedLength;
				unsigned char decodedString[MAX_RAW_STRING_LENGTH];
				base64_decode((unsigned char *)vOpt, strlen(vOpt), decodedString, &decodedLength);
				sdWrite(uartPorts[i].sdp, (uint8_t *) decodedString, decodedLength);
				chprintf(chp, "Ok\r\n");
			} else {
				chprintf(chp, "Bad direction '%s'\r\n", dOpt);
			}
			return; // there is at most one matching port. no need to check the others.
		}
	}
	chprintf(chp, "Bad port '%s'\r\n", pOpt);
}





#define xstr(s) str(s)
#define str(s) #s

static void cmd_version(BaseSequentialStream *chp, int argc, char *argv[]) {
	(void)argc;
	(void)argv;

	obmqSendMessage(MSG_CMD_VERSION);

	chprintf(chp, "Version: OpenTestJig MapleMini v%d.%d.%d-" xstr(VERSION_GIT_HASH) "\r\n",
			VERSION_MAJOR,
			VERSION_MINOR,
			VERSION_REVISION);
}





float widthMean, widthModVariance, periodMean, periodModVariance;
uint32_t widthSampleCounter, periodSampleCounter;

static void icuwidthcb(ICUDriver *icup) {
	float prevWidthMean = widthMean, prevWidthModVariance = widthModVariance;
	icucnt_t currentWidth = icuGetWidthX(icup);

	if(widthSampleCounter == 0) {
		widthSampleCounter++;
		widthMean = (float)currentWidth;
	} else {
		widthSampleCounter++;
		widthMean = prevWidthMean + (((float) currentWidth - prevWidthMean) / widthSampleCounter);
		widthModVariance = prevWidthModVariance + (((float) currentWidth - prevWidthMean) * (currentWidth - widthMean));
	}
}

static void icuperiodcb(ICUDriver *icup) {
	float prevPeriodMean = periodMean, prevPeriodModVariance = periodModVariance;
	icucnt_t currentPeriod = icuGetPeriodX(icup);

	if(periodSampleCounter == 0) {
		periodSampleCounter++;
		periodMean = (float)currentPeriod;
	} else {
		periodSampleCounter++;
		periodMean = prevPeriodMean + (((float) currentPeriod - prevPeriodMean) / periodSampleCounter);
		periodModVariance = prevPeriodModVariance + (((float) currentPeriod - prevPeriodMean) * (currentPeriod - periodMean));
	}
}

#define PWM_CAPTURE_CLOCK 1000000

static void cmd_pwmcapture(BaseSequentialStream *chp, int argc, char *argv[]) {
	(void)argv;

	obmqSendMessage(MSG_CMD_PWMCAPTURE);

	// PWM is using P27 GPIOA8
	if(0 != argc) {
		chprintf(chp, "Usage: no parameters required.\r\n"
				"pwmcapture will sample a PWM signal on P27 (GPIO A8)\r\n");
		return;
	}

	wdtEnable(1);
	wdtAdd(60);

	ICUConfig icucfg = {
		ICU_INPUT_ACTIVE_HIGH,
		PWM_CAPTURE_CLOCK,                                    /* 1MHz ICU clock frequency.   */
		icuwidthcb,
		icuperiodcb,
		NULL,
		ICU_CHANNEL_1,
		0
	};

	widthMean = widthModVariance = periodMean = periodModVariance = 0.0f;
	widthSampleCounter = periodSampleCounter = 0;

	icuStart(&ICUD1, &icucfg);
	icuStartCapture(&ICUD1);

	icuEnableNotifications(&ICUD1);

	chThdSleepMilliseconds(2000);

	icuStopCapture(&ICUD1);
	icuStop(&ICUD1);

	wdtEnable(0);

	chprintf(chp, "frequency=%f\r\n"
			"dutycycle=%f\r\n"
			"captureclock=%d\r\n"
			"widthMean=%f\r\n"
			"widthVariance=%f\r\n"
			"periodMean=%f\r\n"
			"periodVariance=%f\r\n",
			(float) PWM_CAPTURE_CLOCK / periodMean,
			widthMean / periodMean,
			PWM_CAPTURE_CLOCK,
			widthMean,
			widthModVariance / widthSampleCounter,
			periodMean,
			periodModVariance / periodSampleCounter);
}





static void cmd_pwm(BaseSequentialStream *chp, int argc, char *argv[]) {
	uint8_t i;
	char *dOpt = NULL, *fOpt = NULL;

	obmqSendMessage(MSG_CMD_PWM);

	// Parsing
	for(i = 0; i < argc; i++) {
		if(strcmp(argv[i], "-d") == 0) {
			if(++i >= argc) continue;
			dOpt = argv[i];
		}
		else if(strcmp(argv[i], "-f") == 0) {
			if(++i >= argc) continue;
			fOpt = argv[i];
		}
	}

	if (!dOpt || !fOpt)
		goto exit_with_usage;


	PWMConfig pwmcfg = {
		atoi(fOpt)*1000,                            /* PWM clock frequency.   */
		1000,                                       /* PWM period */
		NULL,
		{
			{PWM_OUTPUT_DISABLED, NULL},
			{PWM_OUTPUT_ACTIVE_HIGH, NULL},
			{PWM_OUTPUT_DISABLED, NULL},
			{PWM_OUTPUT_DISABLED, NULL}
		},
		0,
		0
	};

	palSetPadMode(GPIOA, 7, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
	pwmStart(&PWMD3, &pwmcfg);
	pwmEnableChannel(&PWMD3, 1, PWM_PERCENTAGE_TO_WIDTH(&PWMD3, atof(dOpt)*10000));

	chprintf(chp, "Enabled PWM on P4 with a frequency of %dHz and a duty cycle of %d%%\r\n", atoi(fOpt), (int)(atof(dOpt)*100));

	return;

exit_with_usage:
	chprintf(chp, "Usage: pwm -f [Frequency] -d [DutyCycle]\r\n"
			"\tFrequency range: 1-10000Hz\r\n"
			"\tDutyCycle: 0-1.0\r\n");
}





static void cmd_pwm_linearunit(BaseSequentialStream *chp, int argc, char *argv[]) {
	uint8_t i;
	char *move_fraction = NULL

	obmqSendMessage(MSG_CMD_PWM);

	// Parsing
	for(i = 0; i < argc; i++) {
		if(strcmp(argv[i], "move") == 0) {
			if(++i >= argc) continue;
			move_fraction = atoi(argv[i]);
			if((move_fraction > 1) || (move_fraction < 0))
				goto exit_with_usage;
		}
	//	else if(strcmp(argv[i], "-f") == 0) {
	//		if(++i >= argc) continue;
	//		fOpt = argv[i];
	//	}
	}

	if (!dOpt || !fOpt)
		goto exit_with_usage;


	PWMConfig pwmcfg = {
		atoi(fOpt)*1000,                            /* PWM clock frequency.   */
		1000,                                       /* PWM period */
		NULL,
		{
			{PWM_OUTPUT_DISABLED, NULL},
			{PWM_OUTPUT_ACTIVE_HIGH, NULL},
			{PWM_OUTPUT_DISABLED, NULL},
			{PWM_OUTPUT_DISABLED, NULL}
		},
		0,
		0
	};

	palSetPadMode(GPIOA, 7, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
	pwmStart(&PWMD3, &pwmcfg);
	pwmEnableChannel(&PWMD3, 1, PWM_PERCENTAGE_TO_WIDTH(&PWMD3, atof(dOpt)*10000));

	chprintf(chp, "Enabled PWM on P4 with a frequency of %dHz and a duty cycle of %d%%\r\n", atoi(fOpt), (int)(atof(dOpt)*100));

	return;

exit_with_usage:
	chprintf(chp, "Usage: pwm -f [Frequency] -d [DutyCycle]\r\n"
			"\tFrequency range: 1-10000Hz\r\n"
			"\tDutyCycle: 0-1.0\r\n");
}





static void pulses_equidistant(GPIO_TypeDef *gpio, uint8_t pin, unsigned count, bool startLow, unsigned pulseLen_ms)
{
	unsigned i;
	for(i = 0; i < count; ++i) {
		if(startLow) {
			palSetPad(gpio, pin);
			chThdSleepMilliseconds(pulseLen_ms);
		}
		palClearPad(gpio, pin);
		chThdSleepMilliseconds(pulseLen_ms);
		if(!startLow) {
			palSetPad(gpio, pin);
			chThdSleepMilliseconds(pulseLen_ms);
		}
	}
}

static void pulses_ramped(GPIO_TypeDef *gpio, uint8_t pin, unsigned count, bool startLow, unsigned minPulseLen_ms, unsigned rampSteps)
{
	// min_delay determines the target speed
	uint16_t rampLength = MIN(count/2, rampSteps);
	uint8_t delay = minPulseLen_ms + rampSteps;
	unsigned i;

	for(i = 0; i < count; i++) {
		if(startLow) {
			palSetPad(gpio, pin);
			chThdSleepMilliseconds(delay);
		}
		palClearPad(gpio, pin);
		chThdSleepMilliseconds(delay);
		if(!startLow) {
			palSetPad(gpio, pin);
			chThdSleepMilliseconds(delay);
		}

		if (i >= count - rampLength)
			delay++;
		else if (i < rampLength)
			delay--;
	}
}






/* Function for the rotary table/turntable */
static void cmd_motor_rotary(BaseSequentialStream *chp, int argc, char *argv[]) {
	// MapleMini pin declaration for use with the SMCI35 (gpio number arbitrary)
	const char maplePinPulses[] = "25";
	const char maplePinDirection[] = "21";
	const char maplePinEnable[] = "29"; // releases break
	// The following two pins (30 / 31) refer to (Input 2 / Input 3) on SMCI35
	// These are used to call a predefined profile on the motor controller
	// When GPIO off/on are represented by 0/1 the profiles are defined as follows:
	// 0 0 ROTATE_CCW
	// 1 0 ROTATE_CW
	// 0 1 Turn 45 degrees CW
	// 1 1 home (drive to reference position)
	const char maplePinMode1[] = "30";
	const char maplePinMode2[] = "31";

	int pinPulses, pinDirection, pinEnable, pinMode1, pinMode2;

	int mode = -1;
	int direction = -1;
	int pulses = -1;
	int geared_pulses = -1;

	const uint8_t gear_reduction = 48; // RL-D-50 has a gear reduction of 1:48

	// define modes for the rotary unit
	enum modeFlags { MODE_ROTATE = 1, MODE_HOME=2};

	// define clockwise and counterclockwise motion
	enum directionFlags {ROTATE_CW = 0, ROTATE_CCW = 1};

	int i;

	// Parsing
	for(i = 0; i < argc; i++) {
		if(strcmp(argv[i], "rotate") == 0) {
			if(++i >= argc)
				continue;
			pulses = atoi(argv[i]);
			direction = (pulses < 0) ? ROTATE_CCW : ROTATE_CW;
			// Direction set by direction parameter
			// pulses need to be a positive int
			pulses = abs(pulses);
			geared_pulses = pulses * gear_reduction;
			mode = MODE_ROTATE;
		}
		else if(strcmp(argv[i], "home") == 0) {
			mode = MODE_HOME;
			if(++i >= argc)
				continue;
		}
	}

	if(!(mode == MODE_ROTATE || mode == MODE_HOME))
		goto exit_with_usage;

	// SET UP OF PINS
	// pin for pulses
	pinPulses = pinIndexForMaplePin(maplePinPulses, true, false, true);
	palSetPadMode(pinPorts[pinPulses].gpio, pinPorts[pinPulses].pin, PAL_MODE_OUTPUT_PUSHPULL);
	palClearPad(pinPorts[pinPulses].gpio, pinPorts[pinPulses].pin);

	// pin for direction
	pinDirection = pinIndexForMaplePin(maplePinDirection, true, false, true);
	palSetPadMode(pinPorts[pinDirection].gpio, pinPorts[pinDirection].pin, PAL_MODE_OUTPUT_PUSHPULL);
	// set direction pin
	if(direction == ROTATE_CCW)
		palSetPad(pinPorts[pinDirection].gpio, pinPorts[pinDirection].pin);
	else if(direction == ROTATE_CW)
		palClearPad(pinPorts[pinDirection].gpio, pinPorts[pinDirection].pin);

	// pin for enable
	pinEnable = pinIndexForMaplePin(maplePinEnable, true, false, true);
	palSetPadMode(pinPorts[pinEnable].gpio, pinPorts[pinEnable].pin, PAL_MODE_OUTPUT_PUSHPULL);
	palClearPad(pinPorts[pinEnable].gpio, pinPorts[pinEnable].pin);

	// first pin for mode
	pinMode1 = pinIndexForMaplePin(maplePinMode1, true, false, true);
	palSetPadMode(pinPorts[pinMode1].gpio, pinPorts[pinMode1].pin, PAL_MODE_OUTPUT_PUSHPULL);

	// second pin for mode
	pinMode2 = pinIndexForMaplePin(maplePinMode2, true, false, true);
	palSetPadMode(pinPorts[pinMode2].gpio, pinPorts[pinMode2].pin, PAL_MODE_OUTPUT_PUSHPULL);

	// set mode pins
	if(mode == MODE_ROTATE) {
		palSetPad(pinPorts[pinMode1].gpio, pinPorts[pinMode1].pin);
		palClearPad(pinPorts[pinMode2].gpio, pinPorts[pinMode2].pin);
	}
	else if(mode == MODE_HOME) {
		palSetPad(pinPorts[pinMode1].gpio, pinPorts[pinMode1].pin);
		palSetPad(pinPorts[pinMode2].gpio, pinPorts[pinMode2].pin);
	}


	char* direction_literal = NULL; //[25];
	float rotational_angle;

	switch(mode) {
		// 1: rotate
		case 1:
			chprintf(chp, "Selected mode: rotate\n\r\n");
			chprintf(chp, "Pin to use: %s, Pulses to produce %d\n\r\n",
					pinPorts[pinPulses].pinNrString,
					geared_pulses);
			chThdSleepMilliseconds(200);
			pulses_equidistant(pinPorts[pinPulses].gpio, pinPorts[pinPulses].pin, geared_pulses, false, 2);
			if(direction == ROTATE_CW)
				direction_literal = "clockwise (CW)";
			else if(direction == ROTATE_CCW)
				direction_literal = "counterclockwise (CCW)";
			// Each sent pulse will result in 1.8 degrees rotation
			rotational_angle = pulses * 1.8;
			chprintf(chp, "Number of pulses sent: %d\r\nDirection: %s\r\n", geared_pulses, direction_literal);
			chprintf(chp, "Angle of rotation: %.1f\r\n", rotational_angle);
			// reset direction pin at the end of the operation to 0
			palClearPad(pinPorts[pinDirection].gpio, pinPorts[pinDirection].pin);
			break;
		// 2: home
		case 2:
			chprintf(chp, "Selected mode: home\r\n");
			chprintf(chp, "\tRotary unit returns to reference position\r\n");
			palSetPad(pinPorts[pinEnable].gpio, pinPorts[pinEnable].pin);
			chThdSleepMilliseconds(200);
			palClearPad(pinPorts[pinEnable].gpio, pinPorts[pinEnable].pin);
			break;
		default:
			chprintf(chp, "Select one of the available modes\r\n");
			break;
	}
	return;


exit_with_usage:
	chprintf(chp, "Usage: motor_rotary <mode> <arguments>\r\n"
			"\tNumber of pulses to turn "
			"(1 pulse = 1,8 degrees the motor and 0,0375 degrees the rotary table)\r\n"
			"\tNote: The given number of pulses is multiplied by 48 (gear reduction).\r\n"
			"\t      -> 1 pulse as argument sends 48 pulses to the motor and "
			"turns the rotary table by 1.8 degrees.\r\n"
			"\tModes:\r\n"
			"\t 'home' - move to reference position to calibrate angle measurement\r\n"
			"\t 'rotate' <pulses>\r\n"
			"\t\t <pulses> Option 1: +(number of pulses as int) - clockwise (CW) rotation\r\n"
			"\t\t <pulses> Option 2: -(number of pulses as int) - counterclockwise (CCW) rotation\r\n"
			);
}


static void cmd_motor_linear(BaseSequentialStream *chp, int argc, char *argv[]) {
	// Pin declaration for use with the C5-E (gpio number arbitrary)
	const char maplePinPulses[] = "26";
	const char maplePinDirection[] = "28";
	const char maplePinEnable[] = "22";

	int pinPulses, pinDirection, pinEnable;

	// available modes
	enum modeFlags {MODE_MOVE = 1, MODE_HOME = 2};

	int mode = -1;
	int direction = -1;
	int pulses = -1;

	// define motion directions: towards motor/homing position
	enum directionFlags {DIRECTION_MOTOR = 0, DIRECTION_HOME = 1};

	int i;

	// Parsing
	for(i = 0; i < argc; i++) {
		if(strcmp(argv[i], "move") == 0) {
			if(++i >= argc)
				continue;
			pulses = atoi(argv[i]);
			mode = MODE_MOVE;
			// direction is set by presign of the given argument
			direction = (pulses < 0) ? DIRECTION_HOME : DIRECTION_MOTOR;
			// Direction set by direction parameter
			// pulses need to be a positive int
			pulses = abs(pulses);
		}
		else if(strcmp(argv[i], "home") == 0) {
			mode = MODE_HOME;
			if(++i >= argc)
				continue;
		}
	}

	if(!(mode == MODE_MOVE || mode == MODE_HOME))
		goto exit_with_usage;

	/* Pin definition for the pulse and direction outputs */

	if(mode == MODE_MOVE) {
		// Configure the pins just when the mode "move" is used and pulses will be send
		pinPulses = pinIndexForMaplePin(maplePinPulses, true, false, true);
		palSetPadMode(pinPorts[pinPulses].gpio, pinPorts[pinPulses].pin, PAL_MODE_OUTPUT_PUSHPULL);
		palClearPad(pinPorts[pinPulses].gpio, pinPorts[pinPulses].pin);

		pinDirection = pinIndexForMaplePin(maplePinDirection, true, false, true);
		palSetPadMode(pinPorts[pinDirection].gpio, pinPorts[pinDirection].pin, PAL_MODE_OUTPUT_PUSHPULL);
		if(direction == DIRECTION_MOTOR)
			palSetPad(pinPorts[pinDirection].gpio, pinPorts[pinDirection].pin);
		else
			palClearPad(pinPorts[pinDirection].gpio, pinPorts[pinDirection].pin);
	}

	/* Pin defition for the start/enable signal*/
	pinEnable = pinIndexForMaplePin(maplePinEnable, true, false, true);
	palSetPadMode(pinPorts[pinEnable].gpio, pinPorts[pinEnable].pin, PAL_MODE_OUTPUT_PUSHPULL);
	palClearPad(pinPorts[pinEnable].gpio, pinPorts[pinEnable].pin); // Reset pins as its default state is high

	char* direction_literal = NULL; //[25];

	switch(mode) {
		// 1: move
		case 1:
			chprintf(chp, "Selected mode: move\r\n");
			chprintf(chp, "\tMovement of the linear unit defined by <pulses> argument\n\r\n");
			palSetPad(pinPorts[pinEnable].gpio, pinPorts[pinEnable].pin);
			chThdSleepMilliseconds(2000);
			chprintf(chp, "Pin to use: %s, Pulses to produce: %d\n\r\n",
					pinPorts[pinPulses].pinNrString,
					pulses);
			pulses_ramped(pinPorts[pinPulses].gpio, pinPorts[pinPulses].pin, pulses, false, 5, 10);
			if(direction == DIRECTION_MOTOR) {
				direction_literal = "towards motor";
				}
			else if(direction == DIRECTION_HOME) {
				direction_literal = "towards homing position";
				}
			chprintf(chp, "Number of pulses sent: %d\r\nDirection: %s\r\n", pulses, direction_literal);
			if(direction == DIRECTION_HOME)
				palClearPad(pinPorts[pinDirection].gpio, pinPorts[pinDirection].pin);
			palClearPad(pinPorts[pinEnable].gpio, pinPorts[pinEnable].pin);
			break;

		// 2: home
		case 2:
			chprintf(chp, "Selected mode: home\r\n");
			chprintf(chp, "\tLinear unit returns to reference position\r\n");
			palSetPad(pinPorts[pinEnable].gpio, pinPorts[pinEnable].pin);
			chThdSleepMilliseconds(200);
			palClearPad(pinPorts[pinEnable].gpio, pinPorts[pinEnable].pin);
			break;

		default:
			chprintf(chp, "Select one of the available modes\r\n");
			break;
	}
	return;

exit_with_usage:
	chprintf(chp, "Usage: motor_linear  <mode> <arguments>\r\n"
			"\tNumber of pulses to turn (1 pulse = 1,8 degrees the motor and 0,9424mm/pulse the linear unit)\r\n"
			"\tModes:\r\n"
			"\t 'home' - move to reference position to calibrate distance measurement \r\n"
			"\t 'move' <pulses> - linear movement according to the given pulses\r\n"
			"\t\t <pulses> Option 1: +(number of pulses as int) - movement towards motor\r\n"
			"\t\t <pulses> Option 2: -(number of pulses as int) - movement towards homing position\r\n"
			);
}




/* Decoder functions for different sensors */
static float decodeTempAD(uint16_t raw_data) {
	int16_t raw_data_s;

	// Decode return value into a float representing degrees celsius
	if (raw_data & 0x8000)			// If MSB is set value is negative
		raw_data_s = raw_data - 65536;
	else
		raw_data_s = raw_data;
	return (float)(raw_data_s / 128.0);
}

static float decodeTempSI(uint16_t raw_data) {
	return (float)((175.72*raw_data)/65536) - 46.85;
}

static float decodeHumidSI(uint16_t raw_data) {
	return (float)(125*raw_data)/65536.0 - 6.0;
}

static int scanSensors(void) {
	int numSensors = 0;	     // Number of sensors found
	int i, j;		    // Iterators

	// Constants for temperature sensor
	const i2caddr_t base_addr_t = 0x48; // Base address of sensors
	const uint8_t regID_t = 0x0B;	   // Addr of ID register
	const uint8_t manfID_t = 0x19;    // Manufacturer ID of the sensor used
	const uint8_t regConf_t = 0x03;  // Configuration register
	const uint8_t confVal_t = 0x80; // Value for configuration register. Enables 16 bit mode

	// Constants for humidity sensor
	const i2caddr_t base_addr_h = 0x40;		  // Address of humidity sensors
	const uint8_t cmdID_h[2] = {0xFC, 0xC9};	 // Cmd to read second half of 64bit ID
	const uint8_t prodID_h[3] = {0x0D, 0x14, 0x15};	// Possible product IDs for sensor used
	const uint8_t regConf_h = 0xE6;		       // Configuration register
	const uint8_t confVal_h = 0x3A;		      // Value for configuration register. 13 bit RH mode

	// Delete old entries
	memset(sensorList, 0, sizeof(i2cSensor)*MAX_NUM_SENSORS);

	wdtEnable(1);

	// 'Ping' each possible device location
	for (i = 0; i < 2; i++) {
		// Activate I2C port
		I2CDriver *i2cdx = i2cPorts[i].i2interf;
		palSetPadMode(i2cPorts[i].gpioSCL, i2cPorts[i].pinSCL, PAL_MODE_STM32_ALTERNATE_OPENDRAIN);
		palSetPadMode(i2cPorts[i].gpioSDA, i2cPorts[i].pinSDA, PAL_MODE_STM32_ALTERNATE_OPENDRAIN);
		i2cStart(i2cdx, &i2ccfg);
		chThdSleepMilliseconds(100);

		// Temperature sensor (ADT7410)
		for (j = 0; j < 4; j++) {
			uint8_t id[2];
			msg_t m = i2cMasterTransmitTimeout(i2cdx, base_addr_t + j, &regID_t, 1, id, 2, 100);	// Have to read 2 bytes, since 1 byte reads don't work because of faulty I2C Cell in the STM32s
			if (m == MSG_OK && i2cGetErrors(i2cdx) == I2C_NO_ERROR && manfID_t == (id[0] >> 3)) {		// Found sensor
				sensorList[numSensors++] = (i2cSensor){i2cdx, base_addr_t + j, TEMP, 0x00, &decodeTempAD, true};	// Add entry to our sensorList

				// Configure sensor
				uint8_t data[2];
				data[0] = regConf_t; data[1] = confVal_t;
				i2cMasterTransmitTimeout(i2cdx, base_addr_t + j, data, 2, NULL, 0, 100);
			}
		}

		wdtAdd(1);

		// Humidity sensor (SI7021)
		uint8_t id[6];	// Product ID stored in first byte | Format: 2 Bytes followed by CRC followed by 2 Bytes followed by CRC
		msg_t m = i2cMasterTransmitTimeout(i2cdx, base_addr_h, cmdID_h, 2, id, 4, 100);
		if (m == MSG_OK && i2cGetErrors(i2cdx) == I2C_NO_ERROR && (id[0] == prodID_h[0] || id[0] == prodID_h[1] || id[0] == prodID_h[2])) {	// Found sensor
			// This sensor can measure humidity as well as temperature, so we add two entries for it
			sensorList[numSensors++] = (i2cSensor){i2cdx, base_addr_h, TEMP, 0xE0, &decodeTempSI, true};
			sensorList[numSensors++] = (i2cSensor){i2cdx, base_addr_h, HUMID, 0xF5, &decodeHumidSI, false};				// Add entry to our sensorList

			// Configure sensor
			uint8_t data[2];
			data[0] = regConf_h; data[1] = confVal_h;
			i2cMasterTransmitTimeout(i2cdx, base_addr_h, data, 2, NULL, 0, 100);

			// Do an initial measurement, else doing a temp measurement before humidity returns a random value
			i2cMasterTransmitTimeout(i2cdx, base_addr_h, &sensorList[numSensors-1].cmd, 1, NULL, 0, 100);
		}

		// Deactivate I2C port
		i2cStop(i2cdx);
		palSetPadMode(i2cPorts[i].gpioSCL, i2cPorts[i].pinSCL, PAL_MODE_INPUT);
		palSetPadMode(i2cPorts[i].gpioSDA, i2cPorts[i].pinSDA, PAL_MODE_INPUT);

		wdtAdd(1);
	}

	wdtEnable(0);

	// Mark end of array
	if (numSensors < MAX_NUM_SENSORS)
		sensorList[numSensors].port = NULL;

	return numSensors;
}





static float readSensorTimeout(BaseSequentialStream *chp, uint8_t sensorID, uint16_t timeout) {
	// Locals
	float retVal = -1000;		// Default return value for errors

	// Check input
	if (sensorID > MAX_NUM_SENSORS) {
		chprintf(chp, "Specified sensor (%d) is outside of valid range (0-%d)\r\n", sensorID, MAX_NUM_SENSORS);
		return retVal;
	}
	if (sensorList[sensorID].port == NULL) {
		chprintf(chp, "Specified sensor not in list. Did you do a 'sensor scan' before?\r\n");
		return retVal;
	}
	if (sensorList[sensorID].type != TEMP && sensorList[sensorID].type != HUMID) {
		chprintf(chp, "Specified sensor is not a temperature/humidity sensor!\r\n");
		return retVal;
	}

	// Request temperature
	uint8_t i2c_rx_buf[2];
	uint16_t data_raw;

	if (sensorList[sensorID].immediate) {
		msg_t m = i2cMasterTransmitTimeout(sensorList[sensorID].port, sensorList[sensorID].addr, &sensorList[sensorID].cmd, 1, i2c_rx_buf, sizeof(i2c_rx_buf), 100);
		data_raw = (((int16_t)i2c_rx_buf[0]) << 8) + i2c_rx_buf[1];				// Network Byte Order -> Host Byte Order
		if (i2cGetErrors(sensorList[sensorID].port) == I2C_ACK_FAILURE && (m == MSG_TIMEOUT || m == MSG_RESET)) {
			chprintf(chp, "Sensor not responding. Is it connected?\r\n");
		} else if (m == MSG_OK) {
			retVal = sensorList[sensorID].fDecode(data_raw);
		}
	} else {
		systime_t startt = chVTGetSystemTime();
		bool resp = false;

		// Initiate measurement
		i2cMasterTransmitTimeout(sensorList[sensorID].port, sensorList[sensorID].addr, &sensorList[sensorID].cmd, 1, NULL, 0, 100);

		// Wait for response
		while(ST2MS(chVTTimeElapsedSinceX(startt)) < timeout) {
			msg_t m = i2cMasterReceiveTimeout(sensorList[sensorID].port, sensorList[sensorID].addr, i2c_rx_buf, sizeof(i2c_rx_buf), 100);
			if (i2cGetErrors(sensorList[sensorID].port) != I2C_ACK_FAILURE && m == MSG_OK) {
				data_raw = (((int16_t)i2c_rx_buf[0]) << 8) + i2c_rx_buf[1];				// Network Byte Order -> Host Byte Order
				retVal = sensorList[sensorID].fDecode(data_raw);
				resp = true;
			}
		}

		if (!resp)
			chprintf(chp, "Sensor did not respond in time\r\n");
	}

	return retVal;
}

static float readSensor(BaseSequentialStream *chp, uint8_t sensorID) {
	return readSensorTimeout(chp, sensorID, 100);
}

static void cmd_sensor(BaseSequentialStream *chp, int argc, char *argv[]) {
	// Locals
	uint8_t i;
	char *tOpt = NULL;
	bool rTemp = false;
	bool rHumid = false;
	bool rScan = false;
	bool rShow = false;

	obmqSendMessage(MSG_CMD_SENSOR);

	// Check input
	if (argc > 4)
		goto exit_with_usage;

	// Parse args
	for(i = 0; i < argc; i++) {
		if(strcmp(argv[i], "temp") == 0) {
			if (rTemp || rHumid)
				goto exit_with_usage;
			rTemp = true;
		} else if(strcmp(argv[i], "humid") == 0) {
			if (rTemp || rHumid)
				goto exit_with_usage;
			rHumid = true;
		} else if(strcmp(argv[i], "-t") == 0) {
			if(++i >= argc) continue;
			tOpt = argv[i];
		} else if(strcmp(argv[i], "scan") == 0) {
			if (argc > 1)
				goto exit_with_usage;
			rScan = true;
		} else if(strcmp(argv[i], "show") == 0) {
			if (argc > 1)
				goto exit_with_usage;
			rShow = true;
		} else if(strcmp(argv[i], "-h") == 0) {
			goto exit_with_usage;
		} else {
			chprintf(chp, "Unrecognized parameter %s\r\n", argv[i]);
			goto exit_with_usage;
		}
	}

	if (!rTemp && !rHumid && !(rScan || rShow))
		goto exit_with_usage;

	if (!rScan && sensorList[0].port == NULL)
		chprintf(chp, "No entries in list. Do a 'sensor scan' first!\r\n");

	if (rScan) {
		int numSens = scanSensors();
		chprintf(chp, "Found %d sensors\r\n", numSens);
	}

	if (rShow) {
		for (i=0; i < MAX_NUM_SENSORS; i++) {
			if (sensorList[i].port != NULL)
				chprintf(chp, "%d | %s-sensor on port %d @ address %d\r\n", i, sensorList[i].type == TEMP ? "Temperature" : "Humidity", sensorList[i].port == &I2CD1 ? 1 : 2, sensorList[i].addr);
			else
				break;
		}
	}

	wdtEnable(1);

	// Activate I2C ports
	if (rTemp || rHumid) {
		for (i = 0; i < 2; i++) {
			I2CDriver *i2cdx = i2cPorts[i].i2interf;
			palSetPadMode(i2cPorts[i].gpioSCL, i2cPorts[i].pinSCL, PAL_MODE_STM32_ALTERNATE_OPENDRAIN);
			palSetPadMode(i2cPorts[i].gpioSDA, i2cPorts[i].pinSDA, PAL_MODE_STM32_ALTERNATE_OPENDRAIN);
			i2cStart(i2cdx, &i2ccfg);
		}
	}

	wdtAdd(1);

	if (rTemp) {
		// Acquire and print temperature(s)
		for(i = (tOpt ? atoi(tOpt) : 0); i <= (tOpt ? atoi(tOpt) : MAX_NUM_SENSORS-1); i++) {
			if (sensorList[i].port == NULL)
				break;
			if (sensorList[i].type != TEMP) {
				if (tOpt)
					chprintf(chp, "Specified sensor is not a temperature sensor\r\n");
				continue;
			}
			float ret = readSensor(chp, i);
			chprintf(chp, "Temperature read from sensor #%d: %f%s\r\n", i, ret, (ret > -55 ? "°C" : " |ERROR!"));
			wdtAdd(1);
		}
	} else if (rHumid) {
		// Acquire and print humidity(s)
		for(i = (tOpt ? atoi(tOpt) : 0); i <= (tOpt ? atoi(tOpt) : MAX_NUM_SENSORS-1); i++) {
			if (sensorList[i].port == NULL)
				break;
			if (sensorList[i].type != HUMID) {
				if (tOpt)
					chprintf(chp, "Specified sensor is not a humidity sensor\r\n");
				continue;
			}
			float ret = readSensor(chp, i);
			chprintf(chp, "Humidity read from sensor #%d: %f%s\r\n", i, ret, (ret > 0 ? "%" : " |ERROR!"));
			wdtAdd(1);
		}
	}

	wdtAdd(1);

	// Deactivate I2C ports
	if (rTemp || rHumid) {
		for (i = 0; i < 2; i++) {
			I2CDriver *i2cdx = i2cPorts[i].i2interf;
			palSetPadMode(i2cPorts[i].gpioSCL, i2cPorts[i].pinSCL, PAL_MODE_INPUT);
			palSetPadMode(i2cPorts[i].gpioSDA, i2cPorts[i].pinSDA, PAL_MODE_INPUT);
			i2cStop(i2cdx);
		}
	}

	wdtEnable(0);

	return;

	// This part is only entered when the user entered something wrong
exit_with_usage:
	chprintf(chp, "Usage: sensor temp|humid [-t <target>] [scan|show]\r\n"
			"\twith temp:\r\n"
			"\t\tRead a temperature sensor\r\n"
			"\twith humid:\r\n"
			"\t\tRead a humidity sensor\r\n"
			"\twith target:\r\n"
			"\t\t0-7 (0-3 on I2CD1 and 4-7 on I2CD2)\r\n"
			"\t\tIf no target is defined every sensor will be read\r\n"
			"\twith scan:\r\n"
			"\t\tScans for sensors on the I2C interfaces\r\n"
			"\t\tCan not be used with any other parameters!\r\n"
			"\twith show:\r\n"
			"\t\tShows a list of sensors found during the last scan operation\r\n"
			"\t\tCan not be used with any other parameters!\r\n");
}





static void cmd_pollsensors(BaseSequentialStream *chp, int argc, char *argv[]) {
	// Locals
	uint8_t i;
	uint16_t pollrate;
	NullStream nullStr;
	bool respSensors[MAX_NUM_SENSORS];	// Indicating whether we got a response from the sensor yet
	systime_t startt;

	obmqSendMessage(MSG_CMD_POLLSENSORS);

	if (argc != 2)
		goto exit_with_usage;

	if (strcmp(argv[0], "-p") != 0)
		goto exit_with_usage;

	pollrate = atoi(argv[1]);
	if (!(pollrate > 0 && pollrate <= 200))
		goto exit_with_usage;

	nullObjectInit(&nullStr);

	wdtEnable(0);

	// Activate I2C ports
	for (i = 0; i < 2; i++) {
		I2CDriver *i2cdx = i2cPorts[i].i2interf;
		palSetPadMode(i2cPorts[i].gpioSCL, i2cPorts[i].pinSCL, PAL_MODE_STM32_ALTERNATE_OPENDRAIN);
		palSetPadMode(i2cPorts[i].gpioSDA, i2cPorts[i].pinSDA, PAL_MODE_STM32_ALTERNATE_OPENDRAIN);
		i2cStart(i2cdx, &i2ccfg);
	}

	while(true) {
		wdtAdd(1);

		startt = chVTGetSystemTime();
		memset(respSensors, false, sizeof(respSensors));

		for (i = 0; i < MAX_NUM_SENSORS; i++) {
			if (sensorList[i].port == NULL)
				break;

			if (sensorList[i].immediate) {
				// Fetch available data
				float ret = (sensorList[i].type == TEMP || sensorList[i].type == HUMID) ? readSensor((BaseSequentialStream *)&nullStr, i) : -1000;
				chprintf(chp, "%s %d %f %s\r\n",
						sensorList[i].type == TEMP ? "T" : sensorList[i].type == HUMID ? "H" : "U",
						i,
						ret != -1000 ? ret : 0,
						ret == -1000 ? "ERROR" : "OK"	);
				respSensors[i] = true;
			} else {
				// Initiate measurement
				i2cMasterTransmitTimeout(sensorList[i].port, sensorList[i].addr, &sensorList[i].cmd, 1, NULL, 0, 100);
			}
		}

		// Request data until we get an ACK
		int32_t elapsedTime, tmp;
		while((tmp = (int32_t)ST2MS(chVTTimeElapsedSinceX(startt))) < 1000/pollrate) {
			elapsedTime = tmp;
			bool allResp = true;
			for (i = 0; i < MAX_NUM_SENSORS; i++) {
				if (sensorList[i].port == NULL)
					break;

				if (!sensorList[i].immediate && !respSensors[i]) {
					uint8_t i2c_rx_buf[2];
					uint16_t data_raw;

					msg_t m = i2cMasterReceiveTimeout(sensorList[i].port, sensorList[i].addr, i2c_rx_buf, sizeof(i2c_rx_buf), 100);
					if (i2cGetErrors(sensorList[i].port) != I2C_ACK_FAILURE && m == MSG_OK) {
						data_raw = (((int16_t)i2c_rx_buf[0]) << 8) + i2c_rx_buf[1];				// Network Byte Order -> Host Byte Order
						float ret = sensorList[i].fDecode(data_raw);
						chprintf(chp, "%s %d %f %s\r\n",
								sensorList[i].type == TEMP ? "T" : sensorList[i].type == HUMID ? "H" : "U",
								i,
								ret,
								"OK"	);
						respSensors[i] = true;
					} else {
						allResp = false;
					}
				}
			}

			if (allResp)
				break;
		}

		wdtAdd(1);

		for (i = 0; i < MAX_NUM_SENSORS; i++) {
			if (sensorList[i].port == NULL)
				break;

			if (!respSensors[i])
				chprintf(chp, "%s %d %f NORSP\r\n",
						sensorList[i].type == TEMP ? "T" : sensorList[i].type == HUMID ? "H" : "U",
						i,
						0	);
		}

		if (chnGetTimeout(&SDU1, TIME_IMMEDIATE) != Q_TIMEOUT)//iqGetTimeout(&(SDU1).ibqueue, TIME_IMMEDIATE) != Q_TIMEOUT)//sdGetTimeout(&SDU1, TIME_IMMEDIATE) != Q_TIMEOUT)
			break;

		int16_t timeDiff = (int16_t)(1000/pollrate - elapsedTime);
		if (timeDiff >= 0) {
			if (timeDiff > 1)
				chThdSleepMilliseconds(1000/pollrate - elapsedTime);
		} else {
			chprintf(chp, "U 999 %f SLOW\r\n", 0);
			break;
		}
	}

	wdtAdd(1);

	// Deactivate I2C ports
	for (i = 0; i < 2; i++) {
		I2CDriver *i2cdx = i2cPorts[i].i2interf;
		palSetPadMode(i2cPorts[i].gpioSCL, i2cPorts[i].pinSCL, PAL_MODE_INPUT);
		palSetPadMode(i2cPorts[i].gpioSDA, i2cPorts[i].pinSDA, PAL_MODE_INPUT);
		i2cStop(i2cdx);
	}

	wdtEnable(0);

	return;

exit_with_usage:
	chprintf(chp, "Usage: pollsensors -p [polling_rate]\r\n"
			"\tPolling rate range: 1-200\r\n"
			"\tOutput Format: Type SensorID Value Status\r\n"
			"\t\tType: 'T'(Temperature), 'H'(Humidity), or 'U'(Unknown)\r\n"
			"\t\tSensorID: Unsigned Integer\r\n"
			"\t\tValue: Float\r\n"
			"\t\tStatus: 'OK', 'ERROR' or 'NORSP' (No response)\r\n"
			"\tHitting any key will exit the application\r\n"
			"\tSpecial Output: U 999 0.000000000 SLOW\r\n"
			"\t\tIndicates polling rate is too high\r\n");
}





void get_unique_device_id96(uint8_t ptr96bit[96/8])
{
	uint8_t * uid_register = (uint8_t*)0x1ffff7e8;
	memcpy(ptr96bit, uid_register, 96/8);
}

static void cmd_uniqueid(BaseSequentialStream *chp, int argc, char *argv[]) {
	// print unique identifier of STM32F1 CPU
	(void)argc;
	(void)argv;

	obmqSendMessage(MSG_CMD_UNIQUEID);

	uint8_t uid96[96/8];
	unsigned int i;

	get_unique_device_id96(uid96);
	chprintf(chp, "UID ");
	for(i = 0; i < sizeof(uid96); ++i) {
		chprintf(chp, "%02x", uid96[i]);
	}
	chprintf(chp, "\r\n");
}





static bool adc_conversion_ok;

static void adc_bad_callback(ADCDriver *adcp, adcerror_t err)
{
	(void)adcp;
	(void)err;
	adc_conversion_ok = false;
}

static ADCConversionGroup adcgrpcfg = {
	FALSE,                              // not circular
	1,                                  // number of conversion channels
	NULL,                               // ADC conversion end callback
	adc_bad_callback,                   // ADC conversion error callback
	0,                                  // CR1
	0,                                  // CR2
	ADC_SMPR1_SMP_AN10(ADC_SAMPLE_1P5)| // set all channels to 1.5 cycle timings
		ADC_SMPR1_SMP_AN11(ADC_SAMPLE_1P5)|
		ADC_SMPR1_SMP_AN12(ADC_SAMPLE_1P5)|
		ADC_SMPR1_SMP_AN13(ADC_SAMPLE_1P5)|
		ADC_SMPR1_SMP_AN14(ADC_SAMPLE_1P5)|
		ADC_SMPR1_SMP_AN15(ADC_SAMPLE_1P5)|
		ADC_SMPR1_SMP_SENSOR(ADC_SAMPLE_1P5)|
		ADC_SMPR1_SMP_VREF(ADC_SAMPLE_1P5), // SMPR1
	ADC_SMPR2_SMP_AN0(ADC_SAMPLE_1P5)|
		ADC_SMPR2_SMP_AN1(ADC_SAMPLE_1P5)|
		ADC_SMPR2_SMP_AN2(ADC_SAMPLE_1P5)|
		ADC_SMPR2_SMP_AN3(ADC_SAMPLE_1P5)|
		ADC_SMPR2_SMP_AN4(ADC_SAMPLE_1P5)|
		ADC_SMPR2_SMP_AN5(ADC_SAMPLE_1P5)|
		ADC_SMPR2_SMP_AN6(ADC_SAMPLE_1P5)|
		ADC_SMPR2_SMP_AN7(ADC_SAMPLE_1P5)|
		ADC_SMPR2_SMP_AN8(ADC_SAMPLE_1P5)|
		ADC_SMPR2_SMP_AN9(ADC_SAMPLE_1P5),  // SMPR2
	ADC_SQR1_NUM_CH(1),                 // SQR1 conversion group 13..16+len
	0,                                  // SQR2 conversion group 7..12
	0                                   // SQR3 conversion group 1..6
};

static void cmd_adc(BaseSequentialStream *chp, int argc, char *argv[]) {
	char *pOpt = NULL;
	int pin;
	static adcsample_t samples[1];
	unsigned i;

	obmqSendMessage(MSG_CMD_ADC);

	if(argc == 1)
		pOpt = argv[0];

	if(NULL == pOpt) {
		chprintf(chp, "Usage: adc <pin>\r\n"
				"\twith pin:\r\n\t\t");
		for(i = 0; i < sizeof(pinPorts)/sizeof(pinPorts[0]); i++)
			if(pinPorts[i].as_adc)
				chprintf(chp, "%s | ", pinPorts[i].pinNrString);
		chprintf(chp, "\r\n");
		return;
	}

	pin = pinIndexForMaplePin(pOpt, false, true, false);
	if(pin < 0) {
		chprintf(chp, "Bad ADC pin '%s'\r\n", pOpt);
		return;
	}

	wdtEnable(1);

	adcStart(&ADCD1, NULL);

	// configure selected pad as analog in
	palSetPadMode(pinPorts[pin].gpio, pinPorts[pin].pin, PAL_MODE_INPUT_ANALOG);

	adc_conversion_ok = true;

	adcgrpcfg.sqr3 = ADC_SQR3_SQ1_N(pinPorts[pin].adcchan);

	adcConvert(&ADCD1, &adcgrpcfg, samples, 1);

	adcStop(&ADCD1);

	palSetPadMode(pinPorts[pin].gpio, pinPorts[pin].pin, PAL_MODE_INPUT);

	wdtEnable(0);

	if(adc_conversion_ok)
		chprintf(chp, "OK %u\r\n", samples[0]);
	else
		chprintf(chp, "FAIL conversion error\r\n");
}





static void cmd_reset(BaseSequentialStream *chp, int argc, char *argv[]) {
	(void)argc;
	(void)argv;

	obmqSendMessage(MSG_CMD_RESET);

	chprintf(chp, "Starting watchdog to trigger a reset!\r\n");

	wdtEnable(1);

	while(1)
		/* wait for WDT timeout */
		;
}





static const ShellCommand commands[] = {
	{"gpio", cmd_gpio},
	{"uart", cmd_uart},
	{"version", cmd_version},
	{"pwmcapture", cmd_pwmcapture},
	{"pwm", cmd_pwm},
	{"sensor", cmd_sensor},
	{"pollsensors", cmd_pollsensors},
	{"uniqueid", cmd_uniqueid},
	{"adc", cmd_adc},
	{"reset", cmd_reset},
	{"motor_rotary", cmd_motor_rotary},
	{"motor_linear", cmd_motor_linear},
	{NULL, NULL}
};

static const ShellConfig shell_cfg1 = {
	(BaseSequentialStream *)&SDU1,
	commands
};

/*===========================================================================*/
/* Generic code.                                                             */
/*===========================================================================*/

/*
 * Application entry point.
 */
int main(void) {
	thread_t *shelltp = NULL;
	unsigned int i;

	/*
	 * System initializations.
	 * - HAL initialization, this also initializes the configured device drivers
	 *   and performs the board-specific initializations.
	 * - Kernel initialization, the main() function becomes a thread and the
	 *   RTOS is active.
	 */
	halInit();
	chSysInit();

	/* for security reasons: all GPIOs as Input */
	for(i = 0; i < sizeof(pinPorts)/sizeof(pinPorts[0]); i++) {
		palSetPadMode(pinPorts[i].gpio, pinPorts[i].pin, PAL_MODE_INPUT);
	}
	for(i = 0; i < sizeof(uartPorts)/sizeof(uartPorts[0]); i++) {
		palSetPadMode(uartPorts[i].rxGpio, uartPorts[i].rxPin, PAL_MODE_INPUT);
		palSetPadMode(uartPorts[i].txGpio, uartPorts[i].txPin, PAL_MODE_INPUT);
	}

	/*
	 * setup watchdog
	 */
	wdtSetup();

	/*
	 * Initializes a serial-over-USB CDC driver.
	 */
	sduObjectInit(&SDU1);
	sduStart(&SDU1, &serusbcfg);

	/*
	 * Activates the USB driver and then the USB bus pull-up on D+.
	 * Note, a delay is inserted in order to not have to disconnect the cable
	 * after a reset.
	 */
	usbDisconnectBus(serusbcfg.usbp);
	chThdSleepMilliseconds(1500);
	usbStart(serusbcfg.usbp, &usbcfg);
	usbConnectBus(serusbcfg.usbp);

	/*
	 * Shell manager initialization.
	 */
	shellInit();

	/*
	 * Creates the OBMQ thread.
	 */
	obmqSetup();
	obmqSendMessage(MSG_INIT);

	/* remap */
	AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_DISABLE | AFIO_MAPR_USART1_REMAP;

	/* Initial scan for I2C sensors */
	scanSensors();

	/*
	 * Normal main() thread activity, in this demo it does nothing except
	 * sleeping in a loop and check the button state.
	 */
	while (true) {
		if (!shelltp && (SDU1.config->usbp->state == USB_ACTIVE))
			shelltp = shellCreate(&shell_cfg1, SHELL_WA_SIZE, NORMALPRIO);
		else if (chThdTerminatedX(shelltp)) {
			chThdRelease(shelltp);    /* Recovers memory of the previous shell.   */
			shelltp = NULL;           /* Triggers spawning of a new shell.        */
		}
		chThdSleepMilliseconds(1000);
	}
}
