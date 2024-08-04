#pragma once

#define SPI_MODE        (0x03u)

#define CMD_BITS        (0x02u)
#define ADR_BITS        (0x06u)
#define TRANS_BITS      (CMD_BITS + ADR_BITS)

#define CMD_WRITE       (0b00u)
#define CMD_WRITE_BURST (0b01u)
#define CMD_READ        (0b10u)
#define CMD_READ_BURST  (0b11u)


#define REG_DEVID       (0x00u)
#define REG_BW_RATE     (0x2Cu)
#define REG_POWER_CTL   (0x2Du)
#define REG_INT_ENABLE  (0x2Eu) /* | (1ULL << 7) to enable DATA_READY int out */
#define REG_INT_MAP     (0x2Fu) /* & ~(1ULL << 7) set data out to be int1 pin */
#define REG_FIFO_CTL    (0x38u)

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

// class Adxl345 {
// private:
//     Adxl345_();
//     accspi::Spi comm_;

// public:
//     static esp_err_t Init(
//         int mosi, int miso, int sclk, int cs,
//         int int_data_rdy);
// }
