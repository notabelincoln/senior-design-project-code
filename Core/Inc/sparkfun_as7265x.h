
/*
  This is a library written for the AMS AS7265x Spectral Triad (Moonlight)
  SparkFun sells these at its website: www.sparkfun.com
  Do you like this library? Help support SparkFun. Buy a board!
  https://www.sparkfun.com/products/15050

  Originally written by Nathan Seidle & Kevin Kuwata @ SparkFun Electronics, October 25th, 2018

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

#include <stdint.h>
#include "stm32f3xx_hal.h"

#ifndef _SPARKFUN_AS7265X_H
#define _SPARKFUN_AS7265X_H

#define AS7265X_ADDR 0x49 //7-bit unshifted default I2C Address
#define AS7265X_ADDRS (AS7265X_ADDR << 1) // 7-bit shifted default I2C Address

#define AS7265X_STATUS_REG 0x00
#define AS7265X_WRITE_REG 0X01
#define AS7265X_READ_REG 0x02

#define AS7265X_TX_VALID 0x02
#define AS7265X_RX_VALID 0x01

//Register addresses
#define AS7265X_HW_VERSION_HIGH 0x00
#define AS7265X_HW_VERSION_LOW 0x01

#define AS7265X_FW_VERSION_HIGH 0x02
#define AS7265X_FW_VERSION_LOW 0x03

#define AS7265X_CONFIG 0x04
#define AS7265X_INTERGRATION_TIME 0x05
#define AS7265X_DEVICE_TEMP 0x06
#define AS7265X_LED_CONFIG 0x07

//Raw channel registers
#define AS7265X_R_G_A 0x08
#define AS7265X_S_H_B 0x0A
#define AS7265X_T_I_C 0x0C
#define AS7265X_U_J_D 0x0E
#define AS7265X_V_K_E 0x10
#define AS7265X_W_L_F 0x12

//Calibrated channel registers
#define AS7265X_R_G_A_CAL 0x14
#define AS7265X_S_H_B_CAL 0x18
#define AS7265X_T_I_C_CAL 0x1C
#define AS7265X_U_J_D_CAL 0x20
#define AS7265X_V_K_E_CAL 0x24
#define AS7265X_W_L_F_CAL 0x28

#define AS7265X_DEV_SELECT_CONTROL 0x4F

#define AS7265X_COEF_DATA_0 0x50
#define AS7265X_COEF_DATA_1 0x51
#define AS7265X_COEF_DATA_2 0x52
#define AS7265X_COEF_DATA_3 0x53
#define AS7265X_COEF_DATA_READ 0x54
#define AS7265X_COEF_DATA_WRITE 0x55

//Settings

#define AS7265X_POLLING_DELAY 5 //Amount of ms to wait between checking for virtual register changes

#define AS72651_NIR 0x00
#define AS72652_VISIBLE 0x01
#define AS72653_UV 0x02

#define AS7265x_LED_WHITE 0x00 //White LED is connected to x51
#define AS7265x_LED_IR 0x01    //IR LED is connected to x52
#define AS7265x_LED_UV 0x02    //UV LED is connected to x53

#define AS7265X_LED_CURRENT_LIMIT_12_5MA 0x00
#define AS7265X_LED_CURRENT_LIMIT_25MA 0x01
#define AS7265X_LED_CURRENT_LIMIT_50MA 0x02
#define AS7265X_LED_CURRENT_LIMIT_100MA 0x03

#define AS7265X_INDICATOR_CURRENT_LIMIT_1MA 0x00
#define AS7265X_INDICATOR_CURRENT_LIMIT_2MA 0x01
#define AS7265X_INDICATOR_CURRENT_LIMIT_4MA 0x02
#define AS7265X_INDICATOR_CURRENT_LIMIT_8MA 0x03

#define AS7265X_GAIN_1X 0x00
#define AS7265X_GAIN_37X 0x01
#define AS7265X_GAIN_16X 0x02
#define AS7265X_GAIN_64X 0x03

#define AS7265X_MEASUREMENT_MODE_4CHAN 0x00
#define AS7265X_MEASUREMENT_MODE_4CHAN_2 0x01
#define AS7265X_MEASUREMENT_MODE_6CHAN_CONTINUOUS 0x02
#define AS7265X_MEASUREMENT_MODE_6CHAN_ONE_SHOT 0x03

uint8_t getDeviceType(I2C_HandleTypeDef *hi2c);
uint8_t getHardwareVersion(I2C_HandleTypeDef *hi2c);
uint8_t getMajorFirmwareVersion(I2C_HandleTypeDef *hi2c);
uint8_t getPatchFirmwareVersion(I2C_HandleTypeDef *hi2c);
uint8_t getBuildFirmwareVersion(I2C_HandleTypeDef *hi2c);

// uint8_t getTemperature(uint8_t deviceNumber = 0); //Get temp in C of the master IC
float getTemperatureAverage(I2C_HandleTypeDef *hi2c);                    //Get average of all three ICs

void takeMeasurements(I2C_HandleTypeDef *hi2c);
void takeMeasurementsWithBulb(I2C_HandleTypeDef *hi2c);

void enableIndicator(I2C_HandleTypeDef *hi2c); //Blue status LED
void disableIndicator(I2C_HandleTypeDef *hi2c);

void enableBulb(uint8_t device, I2C_HandleTypeDef *hi2c);
void disableBulb(uint8_t device, I2C_HandleTypeDef *hi2c);

void setGain(uint8_t gain, I2C_HandleTypeDef *hi2c);            //1 to 64x
void setMeasurementMode(uint8_t mode, I2C_HandleTypeDef *hi2c); //4 channel, other 4 channel, 6 chan, or 6 chan one shot
void setIntegrationCycles(uint8_t cycleValue, I2C_HandleTypeDef *hi2c);

void setBulbCurrent(uint8_t current, uint8_t device, I2C_HandleTypeDef *hi2c); //
void setIndicatorCurrent(uint8_t current, I2C_HandleTypeDef *hi2c);            //0 to 8mA

void enableInterrupt(I2C_HandleTypeDef *hi2c);
void disableInterrupt(I2C_HandleTypeDef *hi2c);

void softReset(I2C_HandleTypeDef *hi2c);

uint8_t dataAvailable(I2C_HandleTypeDef *hi2c); //Returns true when data is available

//Returns the various calibration data
float getCalibratedA();
float getCalibratedB();
float getCalibratedC();
float getCalibratedD();
float getCalibratedE();
float getCalibratedF();

float getCalibratedG();
float getCalibratedH();
float getCalibratedI();
float getCalibratedJ();
float getCalibratedK();
float getCalibratedL();

float getCalibratedR();
float getCalibratedS();
float getCalibratedT();
float getCalibratedU();
float getCalibratedV();
float getCalibratedW();

//Get the various raw readings
uint16_t getA(I2C_HandleTypeDef *hi2c);
uint16_t getB(I2C_HandleTypeDef *hi2c);
uint16_t getC(I2C_HandleTypeDef *hi2c);
uint16_t getD(I2C_HandleTypeDef *hi2c);
uint16_t getE(I2C_HandleTypeDef *hi2c);
uint16_t getF(I2C_HandleTypeDef *hi2c);

uint16_t getG(I2C_HandleTypeDef *hi2c);
uint16_t getH(I2C_HandleTypeDef *hi2c);
uint16_t getI(I2C_HandleTypeDef *hi2c);
uint16_t getJ(I2C_HandleTypeDef *hi2c);
uint16_t getK(I2C_HandleTypeDef *hi2c);
uint16_t getL(I2C_HandleTypeDef *hi2c);

uint16_t getR(I2C_HandleTypeDef *hi2c);
uint16_t getS(I2C_HandleTypeDef *hi2c);
uint16_t getT(I2C_HandleTypeDef *hi2c);
uint16_t getU(I2C_HandleTypeDef *hi2c);
uint16_t getV(I2C_HandleTypeDef *hi2c);
uint16_t getW(I2C_HandleTypeDef *hi2c);

uint16_t getChannel(uint8_t channelRegister, uint8_t device, I2C_HandleTypeDef *hi2c);
float getCalibratedValue(uint8_t calAddress, uint8_t devic, I2C_HandleTypeDef *hi2ce);
float convertBytesToFloat(uint32_t myLong);

void selectDevice(uint8_t device, I2C_HandleTypeDef *hi2c); //Change between the x51, x52, or x53 for data and settings

uint8_t getTemperature(uint8_t deviceNumber, I2C_HandleTypeDef *hi2c);
float getTemperatureAverage(I2C_HandleTypeDef *hi2c);
void softReset(I2C_HandleTypeDef *hi2c);

uint8_t virtualReadRegister(uint8_t virtualAddr, I2C_HandleTypeDef *hi2c);
void virtualWriteRegister(uint8_t virtualAddr, uint8_t dataToWrite, I2C_HandleTypeDef *hi2c);

uint8_t readRegister(uint8_t addr, I2C_HandleTypeDef *hi2c);
uint8_t writeRegister(uint8_t addr, uint8_t val, I2C_HandleTypeDef *hi2c);

#endif
