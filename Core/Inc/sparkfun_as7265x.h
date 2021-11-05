
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

#ifndef _SPARKFUN_AS7265X_H
#define _SPARKFUN_AS7265X_H

#define AS7265X_ADDR 0x49 //7-bit unshifted default I2C Address

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

#endif
