#define REG_DEVID     (0x00)   // R. Device ID. Should hold 0xE5.
#define REG_BW_RATE   (0x2C)   // R/W. D3:D0 - Freq; D4 - Low power
#define SPI_MODE      (0x03)

#define SET_MEASURE_BIT (0b00001000)
#define REG_POWER_CTL (0x2D)
/* Aceleration data registers.
 * X0 - Lower byte
 * X1 - Higher byte
 * Total bit resolution - 10 bits
 */
#define REG_DATA_X0   (0x32)
#define REG_DATA_X1   (0x33)
#define REG_DATA_Y0   (0x34)
#define REG_DATA_Y1   (0x35)
#define REG_DATA_Z0   (0x36)
#define REG_DATA_Z1   (0x37)







#define DEVID               (0xE5)
