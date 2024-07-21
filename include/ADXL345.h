#pragma once

#define SPI_MODE        (0x03u)

#define CMD_BITS        (0x02u)
#define ADR_BITS        (0x06u)
#define TRANS_BITS      (CMD_BITS + ADR_BITS)

#define CMD_WRITE       (0b00u)
#define CMD_WRITE_BURST (0b01u)
#define CMD_READ        (0b10u)
#define CMD_BURST       (0b11u)


#define REG_DEVID       (0x00u)
#define REG_POWER_CTL   (0x2Cu)

/* Aceleration data registers.
 * X0 - Lower byte
 * X1 - Higher byte
 * Total bit resolution - 10 bits
 */
#define REG_DATA_X0     (0x32u)
#define REG_DATA_X1     (0x33u)
#define REG_DATA_Y0     (0x34u)
#define REG_DATA_Y1     (0x35u)
#define REG_DATA_Z0     (0x36u)
#define REG_DATA_Z1     (0x37u)


#define DEVID               (0xE5u)
