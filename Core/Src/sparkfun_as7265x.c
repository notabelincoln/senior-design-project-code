/*
  This is a library written for the AMS AS7265x Spectral Triad (Moonlight)
  SparkFun sells these at its website: www.sparkfun.com
  Do you like this library? Help support SparkFun. Buy a board!
  https://www.sparkfun.com/products/15050

  Written by Nathan Seidle & Kevin Kuwata @ SparkFun Electronics, October 25th, 2018

  Modified by Abraham Jordan to work with STM32F303K8

  The Spectral Triad is a three sensor platform to do 18-channel spectroscopy.

  https://github.com/sparkfun/SparkFun_AS7265X_Arduino_Library

  Development environment specifics:
  Arduino IDE 1.8.5

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "sparkfun_as7265x.h"

#ifdef _SPARKFUN_AS7265X_H
//Initializes the sensor with basic settings
//Returns false if sensor is not detected
/*
uint8_t begin(I2C_HandleTypeDef *hi2c)
{
	uint8_t value;
	uint8_t buffer;
	UART_HandleTypeDef ret;

	ret = HAL_I2C_IsDeviceReady(&hi2c1, AS7265X_ADDR << 1, 2, HAL_MAX_DELAY);
	if (ret != HAL_OK) {
		strcpy((char *)buffer, "Sensor array not found");
		sprintf(buffer_uart, "%d - %X\r\n", buffer, (uint8_t)buffer_i2c);
		HAL_UART_Transmit(&huart2, buffer_uart, strlen((char *)buffer_uart), HAL_MAX_DELAY);
		return ret;
	} //Check for sensor presence

	//Check to see if both slaves are detected
	value = virtualReadRegister(AS7265X_DEV_SELECT_CONTROL);
	if ((value & 0x30) == 0)
		return value; //Test if Slave1 and 2 are detected. If not, bail.

	setBulbCurrent(AS7265X_LED_CURRENT_LIMIT_12_5MA, AS7265x_LED_WHITE);
	setBulbCurrent(AS7265X_LED_CURRENT_LIMIT_12_5MA, AS7265x_LED_IR);
	setBulbCurrent(AS7265X_LED_CURRENT_LIMIT_12_5MA, AS7265x_LED_UV);

	disableBulb(AS7265x_LED_WHITE); //Turn off bulb to avoid heating sensor
	disableBulb(AS7265x_LED_IR);
	disableBulb(AS7265x_LED_UV);

	setIndicatorCurrent(AS7265X_INDICATOR_CURRENT_LIMIT_8MA); //Set to 8mA (maximum)
	enableIndicator();

	setIntegrationCycles(49); //50 * 2.8ms = 140ms. 0 to 255 is valid.
	//If you use Mode 2 or 3 (all the colors) then integration time is double. 140*2 = 280ms between readings.

	setGain(AS7265X_GAIN_64X); //Set gain to 64x

	setMeasurementMode(AS7265X_MEASUREMENT_MODE_6CHAN_ONE_SHOT); //One-shot reading of VBGYOR

	enableInterrupt();

	return (true); //We're all setup!
}
*/
/*

uint8_t getDeviceType(I2C_HandleTypeDef *hi2c)
{
	return (virtualReadRegister(AS7265X_HW_VERSION_HIGH));
}
*/
/*
uint8_t getHardwareVersion(I2C_HandleTypeDef *hi2c)
{
	return (virtualReadRegister(AS7265X_HW_VERSION_LOW));
}
*/

/*
uint8_t getMajorFirmwareVersion(I2C_HandleTypeDef *hi2c)
{
	virtualWriteRegister(AS7265X_FW_VERSION_HIGH, 0x01); //Set to 0x01 for Major
	virtualWriteRegister(AS7265X_FW_VERSION_LOW, 0x01);  //Set to 0x01 for Major

	return (virtualReadRegister(AS7265X_FW_VERSION_LOW));
}
*/

/*
uint8_t getPatchFirmwareVersion(I2C_HandleTypeDef *hi2c)
{
	virtualWriteRegister(AS7265X_FW_VERSION_HIGH, 0x02); //Set to 0x02 for Patch
	virtualWriteRegister(AS7265X_FW_VERSION_LOW, 0x02);  //Set to 0x02 for Patch

	return (virtualReadRegister(AS7265X_FW_VERSION_LOW));
}
*/

/*
uint8_t getBuildFirmwareVersion(I2C_HandleTypeDef *hi2c)
{
	virtualWriteRegister(AS7265X_FW_VERSION_HIGH, 0x03); //Set to 0x03 for Build
	virtualWriteRegister(AS7265X_FW_VERSION_LOW, 0x03);  //Set to 0x03 for Build

	return (virtualReadRegister(AS7265X_FW_VERSION_LOW));
}
*/

//Tells IC to take all channel measurements and polls for data ready flag
/*
void takeMeasurements(I2C_HandleTypeDef *hi2c)
{
	setMeasurementMode(AS7265X_MEASUREMENT_MODE_6CHAN_ONE_SHOT); //Set mode to all 6-channels, one-shot

	//Wait for data to be ready
	while (dataAvailable() == false)
		HAL_Delay(AS7265X_POLLING_DELAY);

	//Readings can now be accessed via getCalibratedA(), getJ(), etc
}
*/

//Turns on all bulbs, takes measurements of all channels, turns off all bulbs
/*
void takeMeasurementsWithBulb(I2C_HandleTypeDef *hi2c)
{
	enableBulb(AS7265x_LED_WHITE);
	enableBulb(AS7265x_LED_IR);
	enableBulb(AS7265x_LED_UV);

	takeMeasurements();

	disableBulb(AS7265x_LED_WHITE); //Turn off bulb to avoid heating sensor
	disableBulb(AS7265x_LED_IR);
	disableBulb(AS7265x_LED_UV);
}
*/

//Get the various color readings
/*
uint16_t getG(I2C_HandleTypeDef *hi2c)
{
	return (getChannel(AS7265X_R_G_A, AS72652_VISIBLE));
}
uint16_t getH(I2C_HandleTypeDef *hi2c)
{
	return (getChannel(AS7265X_S_H_B, AS72652_VISIBLE));
}
uint16_t getI(I2C_HandleTypeDef *hi2c)
{
	return (getChannel(AS7265X_T_I_C, AS72652_VISIBLE));
}
uint16_t getJ(I2C_HandleTypeDef *hi2c)
{
	return (getChannel(AS7265X_U_J_D, AS72652_VISIBLE));
}
uint16_t getK(I2C_HandleTypeDef *hi2c)
{
	return (getChannel(AS7265X_V_K_E, AS72652_VISIBLE));
}
uint16_t getL(I2C_HandleTypeDef *hi2c)
{
	return (getChannel(AS7265X_W_L_F, AS72652_VISIBLE));
}

//Get the various NIR readings
uint16_t getR(I2C_HandleTypeDef *hi2c)
{
	return (getChannel(AS7265X_R_G_A, AS72651_NIR));
}
uint16_t getS(I2C_HandleTypeDef *hi2c)
{
	return (getChannel(AS7265X_S_H_B, AS72651_NIR));
}
uint16_t getT(I2C_HandleTypeDef *hi2c)
{
	return (getChannel(AS7265X_T_I_C, AS72651_NIR));
}
uint16_t getU(I2C_HandleTypeDef *hi2c)
{
	return (getChannel(AS7265X_U_J_D, AS72651_NIR));
}
uint16_t getV(I2C_HandleTypeDef *hi2c)
{
	return (getChannel(AS7265X_V_K_E, AS72651_NIR));
}
uint16_t getW(I2C_HandleTypeDef *hi2c)
{
	return (getChannel(AS7265X_W_L_F, AS72651_NIR));
}

//Get the various UV readings
uint16_t getA(I2C_HandleTypeDef *hi2c)
{
	return (getChannel(AS7265X_R_G_A, AS72653_UV));
}
uint16_t getB(I2C_HandleTypeDef *hi2c)
{
	return (getChannel(AS7265X_S_H_B, AS72653_UV));
}
uint16_t getC(I2C_HandleTypeDef *hi2c)
{
	return (getChannel(AS7265X_T_I_C, AS72653_UV));
}
uint16_t getD(I2C_HandleTypeDef *hi2c)
{
	return (getChannel(AS7265X_U_J_D, AS72653_UV));
}
uint16_t getE(I2C_HandleTypeDef *hi2c)
{
	return (getChannel(AS7265X_V_K_E, AS72653_UV));
}
uint16_t getF(I2C_HandleTypeDef *hi2c)
{
	return (getChannel(AS7265X_W_L_F, AS72653_UV));
}
*/

//A the 16-bit value stored in a given channel registerReturns
/*
uint16_t getChannel(uint8_t channelRegister, uint8_t device, I2C_HandleTypeDef *hi2c)
{
	selectDevice(device);
	uint16_t colorData = virtualReadRegister(channelRegister) << 8; //High uint8_t
	colorData |= virtualReadRegister(channelRegister + 1);          //Low uint8_t
	return (colorData);
}
*/

//Returns the various calibration data
/*
float getCalibratedA(I2C_HandleTypeDef *hi2c)
{
	return (getCalibratedValue(AS7265X_R_G_A_CAL, AS72653_UV));
}
float getCalibratedB(I2C_HandleTypeDef *hi2c)
{
	return (getCalibratedValue(AS7265X_S_H_B_CAL, AS72653_UV));
}
float getCalibratedC(I2C_HandleTypeDef *hi2c)
{
	return (getCalibratedValue(AS7265X_T_I_C_CAL, AS72653_UV));
}
float getCalibratedD(I2C_HandleTypeDef *hi2c)
{
	return (getCalibratedValue(AS7265X_U_J_D_CAL, AS72653_UV));
}
float getCalibratedE(I2C_HandleTypeDef *hi2c)
{
	return (getCalibratedValue(AS7265X_V_K_E_CAL, AS72653_UV));
}
float getCalibratedF(I2C_HandleTypeDef *hi2c)
{
	return (getCalibratedValue(AS7265X_W_L_F_CAL, AS72653_UV));
}

//Returns the various calibration data
float getCalibratedG(I2C_HandleTypeDef *hi2c)
{
	return (getCalibratedValue(AS7265X_R_G_A_CAL, AS72652_VISIBLE));
}
float getCalibratedH(I2C_HandleTypeDef *hi2c)
{
	return (getCalibratedValue(AS7265X_S_H_B_CAL, AS72652_VISIBLE));
}
float getCalibratedI(I2C_HandleTypeDef *hi2c)
{
	return (getCalibratedValue(AS7265X_T_I_C_CAL, AS72652_VISIBLE));
}
float getCalibratedJ(I2C_HandleTypeDef *hi2c)
{
	return (getCalibratedValue(AS7265X_U_J_D_CAL, AS72652_VISIBLE));
}
float getCalibratedK(I2C_HandleTypeDef *hi2c)
{
	return (getCalibratedValue(AS7265X_V_K_E_CAL, AS72652_VISIBLE));
}
float getCalibratedL(I2C_HandleTypeDef *hi2c)
{
	return (getCalibratedValue(AS7265X_W_L_F_CAL, AS72652_VISIBLE));
}

float getCalibratedR(I2C_HandleTypeDef *hi2c)
{
	return (getCalibratedValue(AS7265X_R_G_A_CAL, AS72651_NIR));
}
float getCalibratedS(I2C_HandleTypeDef *hi2c)
{
	return (getCalibratedValue(AS7265X_S_H_B_CAL, AS72651_NIR));
}
float getCalibratedT(I2C_HandleTypeDef *hi2c)
{
	return (getCalibratedValue(AS7265X_T_I_C_CAL, AS72651_NIR));
}
float getCalibratedU(I2C_HandleTypeDef *hi2c)
{
	return (getCalibratedValue(AS7265X_U_J_D_CAL, AS72651_NIR));
}
float getCalibratedV(I2C_HandleTypeDef *hi2c)
{
	return (getCalibratedValue(AS7265X_V_K_E_CAL, AS72651_NIR));
}
float getCalibratedW(I2C_HandleTypeDef *hi2c)
{
	return (getCalibratedValue(AS7265X_W_L_F_CAL, AS72651_NIR));
}
*/

//Given an address, read four bytes and return the floating point calibrated value
/*
float getCalibratedValue(uint8_t calAddress, uint8_t device, I2C_HandleTypeDef *hi2c)
{
	selectDevice(device);

	uint8_t b0, b1, b2, b3;
	b0 = virtualReadRegister(calAddress + 0);
	b1 = virtualReadRegister(calAddress + 1);
	b2 = virtualReadRegister(calAddress + 2);
	b3 = virtualReadRegister(calAddress + 3);

	//Channel calibrated values are stored big-endian
	uint32_t calBytes = 0;
	calBytes |= ((uint32_t)b0 << (8 * 3));
	calBytes |= ((uint32_t)b1 << (8 * 2));
	calBytes |= ((uint32_t)b2 << (8 * 1));
	calBytes |= ((uint32_t)b3 << (8 * 0));

	return (convertBytesToFloat(calBytes));
}
*/

//Given 4 bytes returns the floating point value
/*
float convertBytesToFloat(uint32_t myLong)
{
	float myFloat;
	memcpy(&myFloat, &myLong, 4); //Copy bytes into a float
	return (myFloat);
}
*/

//Mode 0: 4 channels out of 6 (see datasheet)
//Mode 1: Different 4 channels out of 6 (see datasheet)
//Mode 2: All 6 channels continuously
//Mode 3: One-shot reading of all channels
/*
void setMeasurementMode(uint8_t mode, I2C_HandleTypeDef *hi2c)
{
	if (mode > 0x03)
		mode = 0x03; //Error check

	//Read, mask/set, write
	uint8_t value = virtualReadRegister(AS7265X_CONFIG); //Read
	value &= 0xF3;                                 //Clear BANK bits
	value |= (mode << 2);                                //Set BANK bits with user's choice
	virtualWriteRegister(AS7265X_CONFIG, value);         //Write
}
*/

//Sets the gain value
//Gain 0: 1x (power-on default, I2C_HandleTypeDef *hi2c)
//Gain 1: 3.7x
//Gain 2: 16x
//Gain 3: 64x
/*
void setGain(uint8_t gain, I2C_HandleTypeDef *hi2c)
{
	if (gain > 0x3)
		gain = 0x3;

	//Read, mask/set, write
	uint8_t value = virtualReadRegister(AS7265X_CONFIG); //Read
	value &= 0xCF;                                 //Clear GAIN bits
	value |= (gain << 4);                                //Set GAIN bits with user's choice
	virtualWriteRegister(AS7265X_CONFIG, value);         //Write
}
*/

//Sets the integration cycle amount
//Give this function a byte from 0 to 255.
//Time will be 2.8ms * [integration cycles + 1]
/*
void setIntegrationCycles(uint8_t cycleValue, I2C_HandleTypeDef *hi2c)
{
	virtualWriteRegister(AS7265X_INTERGRATION_TIME, cycleValue); //Write
}
*/

/*
void enableInterrupt(I2C_HandleTypeDef *hi2c)
{
	//Read, mask/set, write
	uint8_t value = virtualReadRegister(AS7265X_CONFIG); //Read
	value |= (1 << 6);                                   //Set INT bit
	virtualWriteRegister(AS7265X_CONFIG, value);         //Write
}
*/

//Disables the interrupt pin
/*
void disableInterrupt(I2C_HandleTypeDef *hi2c)
{
	//Read, mask/set, write
	uint8_t value = virtualReadRegister(AS7265X_CONFIG); //Read
	value &= ~(1 << 6);                                  //Clear INT bit
	virtualWriteRegister(AS7265X_CONFIG, value);         //Write
}
*/

//Checks to see if DRDY flag is set in the control setup register
/*
uint8_t dataAvailable(I2C_HandleTypeDef *hi2c)
{
	uint8_t value = virtualReadRegister(AS7265X_CONFIG);
	return (value & (1 << 1)); //Bit 1 is DATA_RDY
}
*/

//Enable the LED or bulb on a given device
/*
void enableBulb(uint8_t device, I2C_HandleTypeDef *hi2c)
{
	selectDevice(device);

	//Read, mask/set, write
	uint8_t value = virtualReadRegister(AS7265X_LED_CONFIG);
	value |= (1 << 3); //Set the bit
	virtualWriteRegister(AS7265X_LED_CONFIG, value);
}
*/

//Disable the LED or bulb on a given device
/*
void disableBulb(uint8_t device, I2C_HandleTypeDef *hi2c)
{
	selectDevice(device);

	//Read, mask/set, write
	uint8_t value = virtualReadRegister(AS7265X_LED_CONFIG);
	value &= ~(1 << 3); //Clear the bit
	virtualWriteRegister(AS7265X_LED_CONFIG, value);
}
*/

//Set the current limit of bulb/LED.
//Current 0: 12.5mA
//Current 1: 25mA
//Current 2: 50mA
//Current 3: 100mA
/*
void setBulbCurrent(uint8_t current, uint8_t device, I2C_HandleTypeDef *hi2c)
{
	selectDevice(device);

	// set the current
	if (current > 0x03)
		current = 0x03;                                        //Limit to two bits
	uint8_t value = virtualReadRegister(AS7265X_LED_CONFIG); //Read
	value &= 0xCF;                                     //Clear ICL_DRV bits
	value |= (current << 4);                                 //Set ICL_DRV bits with user's choice
	virtualWriteRegister(AS7265X_LED_CONFIG, value);         //Write
}
*/

//As we read various registers we have to point at the master or first/second slave
/*
void selectDevice(uint8_t device, I2C_HandleTypeDef *hi2c)
{
	//Set the bits 0:1. Just overwrite whatever is there because masking in the correct value doesn't work.
	virtualWriteRegister(AS7265X_DEV_SELECT_CONTROL, device);

	//This fails
	//uint8_t value = virtualReadRegister(AS7265X_DEV_SELECT_CONTROL);
	//value &= 0xFC; //Clear lower two bits
	//if(device < 3) value |= device; //Set the bits
	//virtualWriteRegister(AS7265X_DEV_SELECT_CONTROL, value);
}
*/

//Enable the onboard indicator LED
/*
void enableIndicator(I2C_HandleTypeDef *hi2c)
{
	selectDevice(AS72651_NIR);

	//Read, mask/set, write
	uint8_t value = virtualReadRegister(AS7265X_LED_CONFIG);
	value |= (1 << 0); //Set the bit

	virtualWriteRegister(AS7265X_LED_CONFIG, value);
}
*/

//Disable the onboard indicator LED
/*
void disableIndicator(I2C_HandleTypeDef *hi2c)
{
	selectDevice(AS72651_NIR);

	//Read, mask/set, write
	uint8_t value = virtualReadRegister(AS7265X_LED_CONFIG);
	value &= ~(1 << 0); //Clear the bit

	virtualWriteRegister(AS7265X_LED_CONFIG, value);
}
*/

//Set the current limit of onboard LED. Default is max 8mA = 0x03.
/*
void setIndicatorCurrent(uint8_t current, I2C_HandleTypeDef *hi2c)
{
	selectDevice(AS72651_NIR);

	if (current > 0x03)
		current = 0x03;
	//Read, mask/set, write
	uint8_t value = virtualReadRegister(AS7265X_LED_CONFIG); //Read
	value &= 0xF9;                                     //Clear ICL_IND bits
	value |= (current << 1);                                 //Set ICL_IND bits with user's choice

	virtualWriteRegister(AS7265X_LED_CONFIG, value); //Write
}
*/

//Returns the temperature of a given device in C
/*
uint8_t getTemperature(uint8_t deviceNumber, I2C_HandleTypeDef *hi2c)
{
	selectDevice(deviceNumber);
	return (virtualReadRegister(AS7265X_DEVICE_TEMP));
}
*/

//Returns an average of all the sensor temps in C
/*
float getTemperatureAverage(I2C_HandleTypeDef *hi2c)
{
	float average = 0;

	for (uint8_t x = 0; x < 3; x++)
		average += getTemperature(x);

	return (average / 3);
}
*/

//Does a soft reset
//Give sensor at least 1000ms to reset
/*
void softReset(I2C_HandleTypeDef *hi2c)
{
	//Read, mask/set, write
	uint8_t value = virtualReadRegister(AS7265X_CONFIG); //Read
	value |= (1 << 7);                                   //Set RST bit, automatically cleared after reset
	virtualWriteRegister(AS7265X_CONFIG, value);         //Write
}
*/

//Read a virtual register from the AS7265x
uint8_t virtualReadRegister(uint8_t virtualAddr, I2C_HandleTypeDef *hi2c)
{
	uint8_t status;

	//Do a prelim check of the read register
	status = readRegister(AS7265X_STATUS_REG, hi2c);
	if (status & AS7265X_RX_VALID) //There is data to be read
	{
		readRegister(AS7265X_READ_REG, hi2c); //Read the byte but do nothing with it
	}

	//Wait for WRITE flag to clear
	while (1)
	{
		status = readRegister(AS7265X_STATUS_REG, hi2c);
		if (!(status & AS7265X_TX_VALID))
			break; // If TX bit is clear, it is ok to write
		HAL_Delay(AS7265X_POLLING_DELAY);
	}

	// Send the virtual register address (bit 0 should be 0 to indicate we are reading a register).
	writeRegister(AS7265X_WRITE_REG, virtualAddr, hi2c);

	//Wait for READ flag to be set
	while (!(readRegister(AS7265X_STATUS_REG, hi2c) & AS7265X_RX_VALID))
	{
		HAL_Delay(AS7265X_POLLING_DELAY);
	}

	uint8_t incoming = readRegister(AS7265X_READ_REG, hi2c);
	return (incoming);
}

//Write to a virtual register in the AS726x
void virtualWriteRegister(uint8_t virtualAddr, uint8_t dataToWrite, I2C_HandleTypeDef *hi2c)
{
	uint8_t status;

	//Wait for WRITE register to be empty
	while (1)
	{
		status = readRegister(AS7265X_STATUS_REG, hi2c);
		if ((status & AS7265X_TX_VALID) == 0)
			break; // No inbound TX pending at slave. Okay to write now.
		HAL_Delay(AS7265X_POLLING_DELAY);
	}

	// Send the virtual register address (setting bit 7 to indicate we are writing to a register).
	writeRegister(AS7265X_WRITE_REG, (virtualAddr | 1 << 7), hi2c);

	//Wait for WRITE register to be empty
	while (1)
	{
		status = readRegister(AS7265X_STATUS_REG, hi2c);
		if ((status & AS7265X_TX_VALID) == 0)
			break; // No inbound TX pending at slave. Okay to write now.
		HAL_Delay(AS7265X_POLLING_DELAY);
	}

	// Send the data to complete the operation.
	writeRegister(AS7265X_WRITE_REG, dataToWrite, hi2c);
}

//Reads from a give location from the AS726x
uint8_t readRegister(uint8_t addr, I2C_HandleTypeDef *hi2c)
{
	uint8_t buffer;
	HAL_StatusTypeDef ret;

	ret = HAL_I2C_Master_Transmit(hi2c, AS7265X_ADDRS, &addr, sizeof(addr), HAL_MAX_DELAY);
	if (ret != HAL_OK)
		return ret; //Device failed to ack

	ret = HAL_I2C_Master_Receive(hi2c, AS7265X_ADDRS, &buffer, sizeof(buffer), HAL_MAX_DELAY);
	if (ret != HAL_OK)
		return ret; //Device failed to ack

	return buffer;
}

//Write a value to a spot in the AS726x
uint8_t writeRegister(uint8_t addr, uint8_t val, I2C_HandleTypeDef *hi2c)
{
	uint8_t buffer[2];
	HAL_StatusTypeDef ret;

	buffer[0] = addr;
	buffer[1] = val;

	ret = HAL_I2C_Master_Transmit(hi2c, AS7265X_ADDRS, buffer, sizeof(buffer), HAL_MAX_DELAY);
	if (ret != HAL_OK)
		return ret; //Device failed to ack

	ret = HAL_I2C_Master_Receive(hi2c, AS7265X_ADDRS, buffer, sizeof(buffer), HAL_MAX_DELAY);
	if (ret != HAL_OK)
		return ret; //Device failed to ack

	return 0;
}
#endif
