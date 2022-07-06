/**
 * Copyright (c) 2017, Arm Limited and affiliates.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdio.h>
#include <mbed.h>
#include "mbed_stats.h"


#include "lorawan/LoRaWANInterface.h"
#include "lorawan/system/lorawan_data_structures.h"
#include "events/EventQueue.h"

// Application helpers
#include "DummySensor.h"
#include "trace_helper.h"
#include "lora_radio_helper.h"


//#include "DFRobot_AS7341.h"

//DFRobot_AS7341 spectralSen(PA_15, PB_15, 400000); 

I2C i2c(PA_15, PB_15); // SDA pin, SCL pin

#define REG_AS7341_ASTATUS  0X60
#define SEN_ADDRESS         0x39
/*
#define REG_AS7341_CH0_DATA_L  0X61
#define REG_AS7341_CH0_DATA_H  0X62
#define REG_AS7341_ITIME_L  0X63
#define REG_AS7341_ITIME_M  0X64
#define REG_AS7341_ITIME_H  0X65
#define REG_AS7341_CH1_DATA_L  0X66
#define REG_AS7341_CH1_DATA_H  0X67
#define REG_AS7341_CH2_DATA_L  0X68
#define REG_AS7341_CH2_DATA_H  0X69
#define REG_AS7341_CH3_DATA_L  0X6A
#define REG_AS7341_CH3_DATA_H  0X6B
#define REG_AS7341_CH4_DATA_L  0X6C
#define REG_AS7341_CH4_DATA_H  0X6D
#define REG_AS7341_CH5_DATA_L  0X6E
#define REG_AS7341_CH5_DATA_H  0X6F
*/
#define REG_AS7341_CONFIG      0X70
#define REG_AS7341_STAT        0X71
#define REG_AS7341_EDGE        0X72
#define REG_AS7341_CPIO        0X73
#define REG_AS7341_LED         0X74

#define REG_AS7341_ENABLE      0X80
#define REG_AS7341_ATIME       0X81
#define REG_AS7341_WTIME       0X83

#define REG_AS7341_SP_TH_L_LSB 0X84
#define REG_AS7341_SP_TH_L_MSB 0X85
#define REG_AS7341_SP_TH_H_LSB 0X86
#define REG_AS7341_SP_TH_H_MSB 0X87
#define REG_AS7341_AUXID       0X90
#define REG_AS7341_REVID       0X91

#define REG_AS7341_ID          0X92
#define REG_AS7341_STATUS_1    0X93
#define REG_AS7341_ASTATUS     0X94


#define REG_AS7341_CH0_DATA_L  0X95
#define REG_AS7341_CH0_DATA_H  0X96
#define REG_AS7341_CH1_DATA_L  0X97
#define REG_AS7341_CH1_DATA_H  0X98
#define REG_AS7341_CH2_DATA_L  0X99
#define REG_AS7341_CH2_DATA_H  0X9A
#define REG_AS7341_CH3_DATA_L  0X9B
#define REG_AS7341_CH3_DATA_H  0X9C
#define REG_AS7341_CH4_DATA_L  0X9D
#define REG_AS7341_CH4_DATA_H  0X9E
#define REG_AS7341_CH5_DATA_L  0X9F
#define REG_AS7341_CH5_DATA_H  0XA0

#define REG_AS7341_STATUS_2    0XA3
#define REG_AS7341_STATUS_3    0XA4
#define REG_AS7341_STATUS_5    0XA6
#define REG_AS7341_STATUS_6    0XA7
#define REG_AS7341_CFG_0       0XA9
#define REG_AS7341_CFG_1       0XAA
#define REG_AS7341_CFG_3       0XAC
#define REG_AS7341_CFG_6       0XAF
#define REG_AS7341_CFG_8       0XB1
#define REG_AS7341_CFG_9       0XB2
#define REG_AS7341_CFG_10      0XB3
#define REG_AS7341_CFG_12      0XB5


#define REG_AS7341_PERS          0XBD
#define REG_AS7341_GPIO_2        0XBE
#define REG_AS7341_ASTEP_L       0XCA
#define REG_AS7341_ASTEP_H       0XCB
#define REG_AS7341_AGC_GAIN_MAX  0XCF
#define REG_AS7341_AZ_CONFIG     0XD6
#define REG_AS7341_FD_TIME_1     0XD8
#define REG_AS7341_TIME_2        0XDA
#define REG_AS7341_CFG0          0XD7
#define REG_AS7341_STATUS        0XDB
#define REG_AS7341_INTENAB       0XF9
#define REG_AS7341_CONTROL       0XFA
#define REG_AS7341_FIFO_MAP      0XFC
#define REG_AS7341_FIFO_LVL      0XFD
#define REG_AS7341_FDATA_L       0XFE
#define REG_AS7341_FDATA_H       0XFF


#define AS7341_GPIO               4

#define ERR_OK             0      //OK
#define ERR_DATA_BUS      -1      //Data Bus error
#define ERR_IC_VERSION    -2      //Chip version mismatch 

const uint8_t ERROR_AS7341X_MEASUREMENT_TIMEOUT = 0x04;
const uint8_t ERROR_NONE = 0x0;

uint8_t lastError;

char i2cBuff[34];
char regBuff[6];

using namespace events;

// Max payload size can be LORAMAC_PHY_MAXPAYLOAD.
// This example only communicates with much shorter messages (<30 bytes).
// If longer messages are used, these buffers must be changed accordingly.
uint8_t tx_buffer[30];
uint8_t rx_buffer[30];

/*
 * Sets up an application dependent transmission timer in ms. Used only when Duty Cycling is off for testing
 */
#define TX_TIMER                        10000

/**
 * Maximum number of events for the event queue.
 * 10 is the safe number for the stack events, however, if application
 * also uses the queue for whatever purposes, this number should be increased.
 */
#define MAX_NUMBER_OF_EVENTS            10

/**
 * Maximum number of retries for CONFIRMED messages before giving up
 */
#define CONFIRMED_MSG_RETRY_COUNTER     3

/**
 * Dummy pin for dummy sensor
 */
#define PB_4                            0

/**
 * Dummy sensor class object
 */
DS1820  ds1820(PB_4);

/**
* This event queue is the global event queue for both the
* application and stack. To conserve memory, the stack is designed to run
* in the same thread as the application and the application is responsible for
* providing an event queue to the stack that will be used for ISR deferment as
* well as application information event queuing.
*/
static EventQueue ev_queue(MAX_NUMBER_OF_EVENTS *EVENTS_EVENT_SIZE);

/**
 * Event handler.
 *
 * This will be passed to the LoRaWAN stack to queue events for the
 * application which in turn drive the application.
 */
static void lora_event_handler(lorawan_event_t event);

/**
 * Constructing Mbed LoRaWANInterface and passing it the radio object from lora_radio_helper.
 */
static LoRaWANInterface lorawan(radio);

/**
 * Application specific callbacks
 */
static lorawan_app_callbacks_t callbacks;

/**
 * Entry point for application
 */


static uint8_t readReg(uint8_t addr, uint8_t reg) {
  char regAddr[1] = {reg};
  char data[1];
  i2c.write(addr<<1, regAddr, 1, true);
  i2c.read(addr<<1, data, 1, false);
  i2c.stop();
  return data[0];
}

static uint16_t readReg16(uint8_t addr, uint8_t reg) {
  char regAddr[1] = {reg};
  char data[2];
  i2c.write(addr<<1, regAddr, 1, true);
  i2c.read(addr<<1, data, 2, false);
  i2c.stop();
  //return (data[0] << 8 | data[1]);
  uint16_t result = data[1] << 8;
  result += data[0];
  return result;
}

static bool writeReg(uint8_t addr, uint8_t reg, uint8_t value) {
  char msg[2] = {reg, value};
  return i2c.write(addr<<1, msg, 2, false);
}

static bool writeReg16(uint8_t addr, uint8_t reg, uint16_t value) {
  uint8_t MSB = value >> 8;
  uint8_t LSB = value & 0xFF;
  char msg[3] = {reg, MSB, LSB};
  return i2c.write(addr<<1, msg, 3, false);
}


bool configureSensor()
{
    uint8_t WhoAmI = readReg(SEN_ADDRESS, REG_AS7341_ID);
    //printf("\n Printing unprocessed Who Am I: %d", WhoAmI);
    WhoAmI = WhoAmI >> 2;
    //printf("\n Printing Who Am I: %d", WhoAmI);
    wait_us(20000);

    uint8_t enableStat = readReg(SEN_ADDRESS, REG_AS7341_ENABLE);
    //printf("\n Printing Enable Status: %d", enableStat);
    wait_us(20000);

    uint8_t enableSensor = (1 << 0);
    enableSensor |= enableSensor;
    int enableSucess = writeReg(SEN_ADDRESS, REG_AS7341_ENABLE, enableSensor);
    if(enableSucess == 0){
    //printf("\n Enable Write sucess");
    }
    wait_us(20000);
  
    enableStat = readReg(SEN_ADDRESS, REG_AS7341_ENABLE);
    //printf("\n Printing Enable Status: %d", enableStat);
    wait_us(20000);

    uint8_t aTime = readReg(SEN_ADDRESS, REG_AS7341_ATIME);
    //printf("\n Printing ATIME: %d", aTime);
    wait_us(20000);

    uint8_t writeAtime = 29;
    int atimeSucess = writeReg(SEN_ADDRESS, REG_AS7341_ATIME, writeAtime);
    if(atimeSucess == 0){
    //printf("\n ATIME Write sucess");
    }
    wait_us(20000);

    aTime = readReg(SEN_ADDRESS, REG_AS7341_ATIME);
    //printf("\n Printing ATIME: %d", aTime);
    wait_us(20000);

    uint8_t aStepL = readReg(SEN_ADDRESS, REG_AS7341_ASTEP_L);
    //printf("\n Unprocessed ATIME LSB: %d", aStepL);
    uint8_t aStepH = readReg(SEN_ADDRESS, REG_AS7341_ASTEP_H);    
    //printf("\n Unprocessed ATIME HSB: %d", aStepH);
    uint16_t aStep = aStepH<<8 | (aStepL & 0xFF);
    //printf("\n Written ATIME: %d", aStep);
   

    uint8_t gainStat = readReg(SEN_ADDRESS, REG_AS7341_CFG_1);
    //printf("\n Printing Gain Status: %d", gainStat);
    wait_us(20000);

    if (WhoAmI == 9)
   		return true;
	else
		return false; 

}

bool isBitSet(uint8_t registerAddress, uint8_t const bitPosition)
{
	uint8_t value = readReg(SEN_ADDRESS, registerAddress);
	uint8_t mask = 1 << bitPosition;

	if ((value & mask) != 0)
		return true;
	else
		return false;
}


uint16_t readSingleChannelValue()
{
	uint8_t value = readReg(SEN_ADDRESS, REG_AS7341_ENABLE);
	value |= (1 << 1);
	writeReg(SEN_ADDRESS, REG_AS7341_ENABLE, value);

	lastError = ERROR_NONE;
	uint16_t start = us_ticker_read();
    uint16_t delta = 0;
	do
	{
		delta = us_ticker_read();
		if (delta - start > 500000)
		{
			lastError = ERROR_AS7341X_MEASUREMENT_TIMEOUT;
			return 0;
		}
	} while (!isBitSet(REG_AS7341_STATUS_2, 6));

	uint16_t result = readReg16(SEN_ADDRESS, REG_AS7341_CH0_DATA_L);
	return result;
}



unsigned int read415nm()
{	
	// F1 -> ADC0
    writeReg(SEN_ADDRESS, REG_AS7341_ENABLE, 0x01);
    writeReg(SEN_ADDRESS, REG_AS7341_CFG_9, 0x10);
    writeReg(SEN_ADDRESS, REG_AS7341_INTENAB, 0x01);
    writeReg(SEN_ADDRESS, REG_AS7341_CFG_6, 0x10);
    writeReg(SEN_ADDRESS, 0x0, 0x00);
    writeReg(SEN_ADDRESS, 0x01, 0x01);
    writeReg(SEN_ADDRESS, 0x02, 0x00);
    writeReg(SEN_ADDRESS, 0x03, 0x00);
    writeReg(SEN_ADDRESS, 0x04, 0x00);
    writeReg(SEN_ADDRESS, 0x05, 0x00);
    writeReg(SEN_ADDRESS, 0x06, 0x00);
    writeReg(SEN_ADDRESS, 0x07, 0x00);
    writeReg(SEN_ADDRESS, 0x08, 0x00);
    writeReg(SEN_ADDRESS, 0x09, 0x00);
    writeReg(SEN_ADDRESS, 0x0a, 0x00);
    writeReg(SEN_ADDRESS, 0x0b, 0x00);
    writeReg(SEN_ADDRESS, 0x0c, 0x00);
    writeReg(SEN_ADDRESS, 0x0d, 0x00);
    writeReg(SEN_ADDRESS, 0x0e, 0x00);
    writeReg(SEN_ADDRESS, 0x0f, 0x00);
    writeReg(SEN_ADDRESS, 0x10, 0x00);
    writeReg(SEN_ADDRESS, 0x11, 0x00);
    writeReg(SEN_ADDRESS, 0x12, 0x00);
    writeReg(SEN_ADDRESS, 0x13, 0x00);
    writeReg(SEN_ADDRESS, REG_AS7341_ENABLE, 0x11);
    return (unsigned int)readSingleChannelValue();
}

unsigned int read445nm()
{	
	// F1 -> ADC0
    writeReg(SEN_ADDRESS, REG_AS7341_ENABLE, 0x01);
    writeReg(SEN_ADDRESS, REG_AS7341_CFG_9, 0x10);
    writeReg(SEN_ADDRESS, REG_AS7341_INTENAB, 0x01);
    writeReg(SEN_ADDRESS, REG_AS7341_CFG_6, 0x10);
    writeReg(SEN_ADDRESS, 0x0, 0x00);
    writeReg(SEN_ADDRESS, 0x01, 0x00);
    writeReg(SEN_ADDRESS, 0x02, 0x00);
    writeReg(SEN_ADDRESS, 0x03, 0x00);
    writeReg(SEN_ADDRESS, 0x04, 0x00);
    writeReg(SEN_ADDRESS, 0x05, 0x01);
    writeReg(SEN_ADDRESS, 0x06, 0x00);
    writeReg(SEN_ADDRESS, 0x07, 0x00);
    writeReg(SEN_ADDRESS, 0x08, 0x00);
    writeReg(SEN_ADDRESS, 0x09, 0x00);
    writeReg(SEN_ADDRESS, 0x0a, 0x00);
    writeReg(SEN_ADDRESS, 0x0b, 0x00);
    writeReg(SEN_ADDRESS, 0x0c, 0x00);
    writeReg(SEN_ADDRESS, 0x0d, 0x00);
    writeReg(SEN_ADDRESS, 0x0e, 0x00);
    writeReg(SEN_ADDRESS, 0x0f, 0x00);
    writeReg(SEN_ADDRESS, 0x10, 0x00);
    writeReg(SEN_ADDRESS, 0x11, 0x00);
    writeReg(SEN_ADDRESS, 0x12, 0x00);
    writeReg(SEN_ADDRESS, 0x13, 0x00);
    writeReg(SEN_ADDRESS, REG_AS7341_ENABLE, 0x11);
    return (unsigned int)readSingleChannelValue();
}

unsigned int read480nm()
{	
	// F1 -> ADC0
    writeReg(SEN_ADDRESS, REG_AS7341_ENABLE, 0x01);
    writeReg(SEN_ADDRESS, REG_AS7341_CFG_9, 0x10);
    writeReg(SEN_ADDRESS, REG_AS7341_INTENAB, 0x01);
    writeReg(SEN_ADDRESS, REG_AS7341_CFG_6, 0x10);
    writeReg(SEN_ADDRESS, 0x0, 0x00);
    writeReg(SEN_ADDRESS, 0x01, 0x00);
    writeReg(SEN_ADDRESS, 0x02, 0x00);
    writeReg(SEN_ADDRESS, 0x03, 0x00);
    writeReg(SEN_ADDRESS, 0x04, 0x00);
    writeReg(SEN_ADDRESS, 0x05, 0x00);
    writeReg(SEN_ADDRESS, 0x06, 0x00);
    writeReg(SEN_ADDRESS, 0x07, 0x00);
    writeReg(SEN_ADDRESS, 0x08, 0x00);
    writeReg(SEN_ADDRESS, 0x09, 0x00);
    writeReg(SEN_ADDRESS, 0x0a, 0x00);
    writeReg(SEN_ADDRESS, 0x0b, 0x00);
    writeReg(SEN_ADDRESS, 0x0c, 0x00);
    writeReg(SEN_ADDRESS, 0x0d, 0x00);
    writeReg(SEN_ADDRESS, 0x0e, 0x00);
    writeReg(SEN_ADDRESS, 0x0f, 0x10);
    writeReg(SEN_ADDRESS, 0x10, 0x00);
    writeReg(SEN_ADDRESS, 0x11, 0x00);
    writeReg(SEN_ADDRESS, 0x12, 0x00);
    writeReg(SEN_ADDRESS, 0x13, 0x00);
    writeReg(SEN_ADDRESS, REG_AS7341_ENABLE, 0x11);
    return (unsigned int)readSingleChannelValue();
}

unsigned int read515nm()
{	
	// F1 -> ADC0
    writeReg(SEN_ADDRESS, REG_AS7341_ENABLE, 0x01);
    writeReg(SEN_ADDRESS, REG_AS7341_CFG_9, 0x10);
    writeReg(SEN_ADDRESS, REG_AS7341_INTENAB, 0x01);
    writeReg(SEN_ADDRESS, REG_AS7341_CFG_6, 0x10);
    writeReg(SEN_ADDRESS, 0x0, 0x00);
    writeReg(SEN_ADDRESS, 0x01, 0x00);
    writeReg(SEN_ADDRESS, 0x02, 0x00);
    writeReg(SEN_ADDRESS, 0x03, 0x00);
    writeReg(SEN_ADDRESS, 0x04, 0x00);
    writeReg(SEN_ADDRESS, 0x05, 0x10);
    writeReg(SEN_ADDRESS, 0x06, 0x00);
    writeReg(SEN_ADDRESS, 0x07, 0x00);
    writeReg(SEN_ADDRESS, 0x08, 0x00);
    writeReg(SEN_ADDRESS, 0x09, 0x00);
    writeReg(SEN_ADDRESS, 0x0a, 0x00);
    writeReg(SEN_ADDRESS, 0x0b, 0x00);
    writeReg(SEN_ADDRESS, 0x0c, 0x00);
    writeReg(SEN_ADDRESS, 0x0d, 0x01);
    writeReg(SEN_ADDRESS, 0x0e, 0x00);
    writeReg(SEN_ADDRESS, 0x0f, 0x00);
    writeReg(SEN_ADDRESS, 0x10, 0x00);
    writeReg(SEN_ADDRESS, 0x11, 0x00);
    writeReg(SEN_ADDRESS, 0x12, 0x00);
    writeReg(SEN_ADDRESS, 0x13, 0x00);
    writeReg(SEN_ADDRESS, REG_AS7341_ENABLE, 0x11);
    return (unsigned int)readSingleChannelValue();
}

unsigned int read555nm()
{	
	// F1 -> ADC0
    writeReg(SEN_ADDRESS, REG_AS7341_ENABLE, 0x01);
    writeReg(SEN_ADDRESS, REG_AS7341_CFG_9, 0x10);
    writeReg(SEN_ADDRESS, REG_AS7341_INTENAB, 0x01);
    writeReg(SEN_ADDRESS, REG_AS7341_CFG_6, 0x10);
    writeReg(SEN_ADDRESS, 0x0, 0x00);
    writeReg(SEN_ADDRESS, 0x01, 0x00);
    writeReg(SEN_ADDRESS, 0x02, 0x00);
    writeReg(SEN_ADDRESS, 0x03, 0x00);
    writeReg(SEN_ADDRESS, 0x04, 0x00);
    writeReg(SEN_ADDRESS, 0x05, 0x00);
    writeReg(SEN_ADDRESS, 0x06, 0x10);
    writeReg(SEN_ADDRESS, 0x07, 0x00);
    writeReg(SEN_ADDRESS, 0x08, 0x00);
    writeReg(SEN_ADDRESS, 0x09, 0x10);
    writeReg(SEN_ADDRESS, 0x0a, 0x00);
    writeReg(SEN_ADDRESS, 0x0b, 0x00);
    writeReg(SEN_ADDRESS, 0x0c, 0x00);
    writeReg(SEN_ADDRESS, 0x0d, 0x01);
    writeReg(SEN_ADDRESS, 0x0e, 0x00);
    writeReg(SEN_ADDRESS, 0x0f, 0x00);
    writeReg(SEN_ADDRESS, 0x10, 0x00);
    writeReg(SEN_ADDRESS, 0x11, 0x00);
    writeReg(SEN_ADDRESS, 0x12, 0x00);
    writeReg(SEN_ADDRESS, 0x13, 0x00);
    writeReg(SEN_ADDRESS, REG_AS7341_ENABLE, 0x11);
    return (unsigned int)readSingleChannelValue();
}


unsigned int read590nm()
{	
	// F1 -> ADC0
    writeReg(SEN_ADDRESS, REG_AS7341_ENABLE, 0x01);
    writeReg(SEN_ADDRESS, REG_AS7341_CFG_9, 0x10);
    writeReg(SEN_ADDRESS, REG_AS7341_INTENAB, 0x01);
    writeReg(SEN_ADDRESS, REG_AS7341_CFG_6, 0x10);
    writeReg(SEN_ADDRESS, 0x0, 0x00);
    writeReg(SEN_ADDRESS, 0x01, 0x00);
    writeReg(SEN_ADDRESS, 0x02, 0x00);
    writeReg(SEN_ADDRESS, 0x03, 0x00);
    writeReg(SEN_ADDRESS, 0x04, 0x01);
    writeReg(SEN_ADDRESS, 0x05, 0x00);
    writeReg(SEN_ADDRESS, 0x06, 0x00);
    writeReg(SEN_ADDRESS, 0x07, 0x00);
    writeReg(SEN_ADDRESS, 0x08, 0x00);
    writeReg(SEN_ADDRESS, 0x09, 0x00);
    writeReg(SEN_ADDRESS, 0x0a, 0x00);
    writeReg(SEN_ADDRESS, 0x0b, 0x00);
    writeReg(SEN_ADDRESS, 0x0c, 0x00);
    writeReg(SEN_ADDRESS, 0x0d, 0x00);
    writeReg(SEN_ADDRESS, 0x0e, 0x10);
    writeReg(SEN_ADDRESS, 0x0f, 0x00);
    writeReg(SEN_ADDRESS, 0x10, 0x00);
    writeReg(SEN_ADDRESS, 0x11, 0x00);
    writeReg(SEN_ADDRESS, 0x12, 0x00);
    writeReg(SEN_ADDRESS, 0x13, 0x00);
    writeReg(SEN_ADDRESS, REG_AS7341_ENABLE, 0x11);
    return (unsigned int)readSingleChannelValue();
}

unsigned int read630nm()
{	
	// F1 -> ADC0
    writeReg(SEN_ADDRESS, REG_AS7341_ENABLE, 0x01);
    writeReg(SEN_ADDRESS, REG_AS7341_CFG_9, 0x10);
    writeReg(SEN_ADDRESS, REG_AS7341_INTENAB, 0x01);
    writeReg(SEN_ADDRESS, REG_AS7341_CFG_6, 0x10);
    writeReg(SEN_ADDRESS, 0x0, 0x00);
    writeReg(SEN_ADDRESS, 0x01, 0x00);
    writeReg(SEN_ADDRESS, 0x02, 0x00);
    writeReg(SEN_ADDRESS, 0x03, 0x00);
    writeReg(SEN_ADDRESS, 0x04, 0x00);
    writeReg(SEN_ADDRESS, 0x05, 0x00);
    writeReg(SEN_ADDRESS, 0x06, 0x00);
    writeReg(SEN_ADDRESS, 0x07, 0x01);
    writeReg(SEN_ADDRESS, 0x08, 0x00);
    writeReg(SEN_ADDRESS, 0x09, 0x00);
    writeReg(SEN_ADDRESS, 0x0a, 0x01);
    writeReg(SEN_ADDRESS, 0x0b, 0x00);
    writeReg(SEN_ADDRESS, 0x0c, 0x00);
    writeReg(SEN_ADDRESS, 0x0d, 0x00);
    writeReg(SEN_ADDRESS, 0x0e, 0x00);
    writeReg(SEN_ADDRESS, 0x0f, 0x00);
    writeReg(SEN_ADDRESS, 0x10, 0x00);
    writeReg(SEN_ADDRESS, 0x11, 0x00);
    writeReg(SEN_ADDRESS, 0x12, 0x00);
    writeReg(SEN_ADDRESS, 0x13, 0x00);
    writeReg(SEN_ADDRESS, REG_AS7341_ENABLE, 0x11);
    return (unsigned int)readSingleChannelValue();
}

unsigned int read680nm()
{	
	// F1 -> ADC0
    writeReg(SEN_ADDRESS, REG_AS7341_ENABLE, 0x01);
    writeReg(SEN_ADDRESS, REG_AS7341_CFG_9, 0x10);
    writeReg(SEN_ADDRESS, REG_AS7341_INTENAB, 0x01);
    writeReg(SEN_ADDRESS, REG_AS7341_CFG_6, 0x10);
    writeReg(SEN_ADDRESS, 0x0, 0x00);
    writeReg(SEN_ADDRESS, 0x01, 0x00);
    writeReg(SEN_ADDRESS, 0x02, 0x00);
    writeReg(SEN_ADDRESS, 0x03, 0x10);
    writeReg(SEN_ADDRESS, 0x04, 0x00);
    writeReg(SEN_ADDRESS, 0x05, 0x00);
    writeReg(SEN_ADDRESS, 0x06, 0x00);
    writeReg(SEN_ADDRESS, 0x07, 0x00);
    writeReg(SEN_ADDRESS, 0x08, 0x00);
    writeReg(SEN_ADDRESS, 0x09, 0x00);
    writeReg(SEN_ADDRESS, 0x0a, 0x00);
    writeReg(SEN_ADDRESS, 0x0b, 0x00);
    writeReg(SEN_ADDRESS, 0x0c, 0x00);
    writeReg(SEN_ADDRESS, 0x0d, 0x00);
    writeReg(SEN_ADDRESS, 0x0e, 0x01);
    writeReg(SEN_ADDRESS, 0x0f, 0x00);
    writeReg(SEN_ADDRESS, 0x10, 0x00);
    writeReg(SEN_ADDRESS, 0x11, 0x00);
    writeReg(SEN_ADDRESS, 0x12, 0x00);
    writeReg(SEN_ADDRESS, 0x13, 0x00);
    writeReg(SEN_ADDRESS, REG_AS7341_ENABLE, 0x11);
    return (unsigned int)readSingleChannelValue();
}

float readSingleBasicCountChannelValue(uint16_t raw)
{
	uint16_t aTime = 29;
	uint16_t aStep = 999;
	uint16_t tint = (aTime + 1) * (aStep * 1) * 2.78 / 1000;
	uint8_t gainValue = 9;
	float gain;
	if (gainValue == 0)
	{
		gain = 0.5f;
	}
	else
	{
		gain = pow(2, (gainValue - 1));
	}
	return (float(raw) / (gain * tint));
}


int main(void)
{
  
    bool sucessConfig = configureSensor();
    if (sucessConfig == true)
    {
    printf("\n Sensor Sucessfully Connected with Default Setup.");
    }

    unsigned int singleChannelRaw = read415nm();
	if (lastError == ERROR_NONE)
	{
		printf("\n 415 nm raw value is: %d", singleChannelRaw);
	}
	else
	{
		printf("\n Error: %d", lastError);
	}

    wait_us(100000);

    singleChannelRaw = read445nm();
	if (lastError == ERROR_NONE)
	{
		printf("\n 445 nm raw value is: %d", singleChannelRaw);
	}
	else
	{
		printf("\n Error: %d", lastError);
	}

    wait_us(100000);

    singleChannelRaw = read480nm();
	if (lastError == ERROR_NONE)
	{
		printf("\n 480 nm raw value is: %d", singleChannelRaw);
	}
	else
	{
		printf("\n Error: %d", lastError);
	}

    wait_us(100000);

    singleChannelRaw = read515nm();
	if (lastError == ERROR_NONE)
	{
		printf("\n 515 nm raw value is: %d", singleChannelRaw);
	}
	else
	{
		printf("\n Error: %d", lastError);
	}

    wait_us(100000);

    singleChannelRaw = read555nm();
	if (lastError == ERROR_NONE)
	{
		printf("\n 555 nm raw value is: %d", singleChannelRaw);
	}
	else
	{
		printf("\n Error: %d", lastError);
	}

    wait_us(100000);

    singleChannelRaw = read590nm();
	if (lastError == ERROR_NONE)
	{
		printf("\n 595 nm raw value is: %d", singleChannelRaw);
	}
	else
	{
		printf("\n Error: %d", lastError);
	}

    wait_us(100000);

    singleChannelRaw = read630nm();
	if (lastError == ERROR_NONE)
	{
		printf("\n 630 nm raw value is: %d", singleChannelRaw);
	}
	else
	{
		printf("\n Error: %d", lastError);
	}

    wait_us(100000);

    singleChannelRaw = read680nm();
	if (lastError == ERROR_NONE)
	{
		printf("\n 680 nm raw value is: %d", singleChannelRaw);
	}
	else
	{
		printf("\n Error: %d", lastError);
	}

    wait_us(100000);








    // setup tracing
    setup_trace();  

    // stores the status of a call to LoRaWAN protocol
    lorawan_status_t retcode;

    // Initialize LoRaWAN stack
    if (lorawan.initialize(&ev_queue) != LORAWAN_STATUS_OK) {
        printf("\r\n LoRa initialization failed! \r\n");
        return -1;
    }

    printf("\r\n Mbed LoRaWANStack initialized \r\n");

    // prepare application callbacks
    callbacks.events = mbed::callback(lora_event_handler);
    lorawan.add_app_callbacks(&callbacks);

    // Set number of retries in case of CONFIRMED messages
    if (lorawan.set_confirmed_msg_retries(CONFIRMED_MSG_RETRY_COUNTER)
            != LORAWAN_STATUS_OK) {
        printf("\r\n set_confirmed_msg_retries failed! \r\n\r\n");
        return -1;
    }

    printf("\r\n CONFIRMED message retries : %d \r\n",
           CONFIRMED_MSG_RETRY_COUNTER);

    if (lorawan.disable_adaptive_datarate() != LORAWAN_STATUS_OK) {
        printf("\r\n ADR disable failed! \r\n");
        return -1;
    }

    if (lorawan.set_datarate(3) != LORAWAN_STATUS_OK) {
        printf("\r\n Set DR failed! \r\n");
        return -1;
    }

    retcode = lorawan.connect();

    if (retcode == LORAWAN_STATUS_OK ||
            retcode == LORAWAN_STATUS_CONNECT_IN_PROGRESS) {
    } else {
        printf("\r\n Connection error, code = %d \r\n", retcode);
        return -1;
    }

    printf("\r\n Connection - In Progress ...\r\n");

    // make your event queue dispatching events forever
    ev_queue.dispatch_forever();

    return 0;
    

}

/**
 * Sends a message to the Network Server
 */
static void send_message()
{
    uint16_t packet_len;
    int16_t retcode;
    int32_t sensor_value;

 

    /* //i2c.read(SEN_ADDRESS << 1, i2cBuff, 2);
    int tmp = int((i2cBuff[0]<<8)|i2cBuff[1]);
    printf("\n the ack is %d: ", tmp);
    

 
    uint8_t astep = 999;
    uint8_t temp = astep >> 8;
    i2c.write(temp);
    i2c.write(SEN_ADDRESS<<1);
    i2c.write(REG_AS7341_ASTEP_L);
    uint8_t drt = astep &= 0xff;
    i2c.write(drt);
*/
    wait_us(2000);



	
    /* astep read */
    /*
    char lsbBuff[2];
    char hsbBuff[2];
  
    i2c.write(SEN_ADDRESS<<1);
    i2c.write(REG_AS7341_ASTEP_H);
    wait_us(2);

    i2c.read(SEN_ADDRESS<<1, lsbBuff, 2);
    uint8_t dat1;
    dat1 = hsbBuff[0];
    dat1 = dat1 << 8;
    
    i2c.write(SEN_ADDRESS<<1);
    i2c.write(REG_AS7341_ASTEP_L);

    i2c.read(SEN_ADDRESS<<1, lsbBuff, 2);
    uint8_t dat2;
    dat2 = hsbBuff[0];
    dat1 |= dat2;
    printf("\n Value Read AT: %d", dat1);
*/


/*
	uint8_t address = 0x39;

		if (!i2c.write(address << 1, NULL, 0)) // 0 returned is ok
		{																								  
			printf("I2C device found at address 0x%02X (0x%02X in 8-bit)\n", address, address << 1);

		}
		wait_us(200000);

		printf("No");
	printf(" device found\n\n");




    ThisThread::sleep_for(1s);

    int nack = 0;
    char cmd[2] = {0};


    nack = i2c.write(addrs << 1, NULL, 0);
    //Returns 0 on success (ack), nonzero on failure (nack)
    printf("nack is %d \r\n", nack); // Fails

    ThisThread::sleep_for(10ms);

    cmd[0] = revID;
    i2c.write(addrs, cmd, 1);
    i2c.read(addrs, cmd, 1);

    printf("cmd[0] is %d\r\n", cmd[0]);

    while (1) ThisThread::sleep_for(1s);

*/

    if (ds1820.begin()) {
        ds1820.startConversion();
        sensor_value = ds1820.read();
        printf("\r\n Dummy Sensor Value = %d \r\n", sensor_value);
        ds1820.startConversion();
    } else {
        printf("\r\n No sensor found \r\n");
        return;
    }

    packet_len = sprintf((char *) tx_buffer, "Dummy Sensor Value is %d",
                         sensor_value);

    retcode = lorawan.send(MBED_CONF_LORA_APP_PORT, tx_buffer, packet_len,
                           MSG_UNCONFIRMED_FLAG);

    if (retcode < 0) {
        retcode == LORAWAN_STATUS_WOULD_BLOCK ? printf("send - WOULD BLOCK\r\n")
        : printf("\r\n send() - Error code %d \r\n", retcode);

        if (retcode == LORAWAN_STATUS_WOULD_BLOCK) {
            //retry in 3 seconds
            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
                ev_queue.call_in(3000, send_message);
            }
        }
        return;
    }

    printf("\r\n %d bytes scheduled for transmission \r\n", retcode);
    memset(tx_buffer, 0, sizeof(tx_buffer));

    
    //Added pk
    printf("Deep sleep allowed: %i\r\n", sleep_manager_can_deep_sleep());
    ThisThread::sleep_for(30000);
    
    system_reset();

}

/**
 * Receive a message from the Network Server
 */
static void receive_message()
{


    uint8_t port;
    int flags;
    int16_t retcode = lorawan.receive(rx_buffer, sizeof(rx_buffer), port, flags);

    if (retcode < 0) {
        printf("\r\n receive() - Error code %d \r\n", retcode);
        return;
    }

    printf(" RX Data on port %u (%d bytes): ", port, retcode);
    for (uint8_t i = 0; i < retcode; i++) {
        printf("%02x ", rx_buffer[i]);
    }
    printf("\r\n");
    
    memset(rx_buffer, 0, sizeof(rx_buffer));

    

}
//Test git
/**
 * Event handler
 */
static void lora_event_handler(lorawan_event_t event)
{
    switch (event) {
        case CONNECTED:
            printf("\r\n Connection - Successful \r\n");
            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
                send_message();
            } else {
                ev_queue.call_every(TX_TIMER, send_message);
            }

            break;
        case DISCONNECTED:
            ev_queue.break_dispatch();
            printf("\r\n Disconnected Successfully \r\n");
            break;
        case TX_DONE:
            printf("\r\n Message Sent to Network Server \r\n");
            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
                send_message();
            }
            break;
        case TX_TIMEOUT:
        case TX_ERROR:
        case TX_CRYPTO_ERROR:
        case TX_SCHEDULING_ERROR:
            printf("\r\n Transmission Error - EventCode = %d \r\n", event);
            // try again
            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
                send_message();
            }
            break;
        case RX_DONE:
            printf("\r\n Received message from Network Server \r\n");
            receive_message();
            break;
        case RX_TIMEOUT:
        case RX_ERROR:
            printf("\r\n Error in reception - Code = %d \r\n", event);
            break;
        case JOIN_FAILURE:
            printf("\r\n OTAA Failed - Check Keys \r\n");
            break;
        case UPLINK_REQUIRED:
            printf("\r\n Uplink required by NS \r\n");
            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
                send_message();
            }
            break;
        default:
            MBED_ASSERT("Unknown Event");
    }
}

// EOF
