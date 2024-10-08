#pragma once

#include "esp_err.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"


class Spi {
private:
    // If left uninitialized set variables to 0
    spi_bus_config_t                spiBusCfg_{};
    spi_device_interface_config_t   spiInterfaceCfg_{};
    spi_device_handle_t             handle_{};
    spi_host_device_t               spiPeripheral_{}; /* SPI 2 or 3 */

    spi_transaction_t               spiTransaction_{};

    esp_err_t TransferByte(
            const uint8_t regAddr,
            const uint8_t data,
            const uint8_t command);

    esp_err_t TransferMultipleBytes(
            const uint8_t regAddr,
            void* txBuf,
            void* rxBuf,
            size_t dataLength,
            const uint8_t command);

public:
    Spi(
            const spi_host_device_t spiPeripheral,
            const int pinMosi,
            const int pinMiso,
            const int pinSclk);

    // Mode is 0 to 3 (00, 01, 10, 11)
    esp_err_t RegisterDevice(
            const uint8_t mode,
            const int csPin,
            const uint8_t commandLenght,
            const uint8_t addrLenght,
            const uint8_t dummyLlenght,
            const int busSpeed = 100,
            const uint8_t csEnaPretrans = 0,
            const uint8_t csEnaPposttrans = 0,
            const uint32_t flags = 0);
// Flags
// SPI_TRANS_MODE_DIO                Transmit/receive data in 2-bit mode
// SPI_TRANS_MODE_QIO                Transmit/receive data in 4-bit mode
// SPI_TRANS_USE_RXDATA              Receive into rx_data member of spi_transaction_t instead into memory at rx_buffer.
// SPI_TRANS_USE_TXDATA              Transmit tx_data member of spi_transaction_t instead of data at tx_buffer. Do not set tx_buffer when using this.
// SPI_TRANS_MODE_DIOQIO_ADDR        Also transmit address in mode selected by SPI_MODE_DIO/SPI_MODE_QIO
// SPI_TRANS_VARIABLE_CMD            Use the ``command_bits`` in ``spi_transaction_ext_t`` rather than default value in ``spi_device_interface_config_t``.
// SPI_TRANS_VARIABLE_ADDR           Use the ``address_bits`` in ``spi_transaction_ext_t`` rather than default value in ``spi_device_interface_config_t``.
// SPI_TRANS_VARIABLE_DUMMY          Use the ``dummy_bits`` in ``spi_transaction_ext_t`` rather than default value in ``spi_device_interface_config_t``.
// SPI_TRANS_CS_KEEP_ACTIVE          Keep CS active after data transfer
// SPI_TRANS_MULTILINE_CMD           The data lines used at command phase is the same as data phase (otherwise, only one data line is used at command phase)
// SPI_TRANS_MODE_OCT                Transmit/receive data in 8-bit mode
// SPI_TRANS_MULTILINE_ADDR          The data lines used at address phase is the same as data phase (otherwise, only one data line is used at address phase)
// SPI_TRANS_DMA_BUFFER_ALIGN_MANUAL By default driver will automatically re-alloc dma buffer if it doesn't meet hardware alignment or dma_capable requirements, this flag is for you to disable this feature, you will need to take care of the alignment otherwise driver will return you error ESP_ERR_INVALID_ARG

    uint8_t ReadRegister(
            const uint8_t regAddr,
            const uint8_t command);

    esp_err_t WriteRegister(
            const uint8_t regAddr,
            const uint8_t regData,
            const uint8_t command);

    esp_err_t SetBit(
            const uint8_t regAddr,
            const uint8_t bitIdx,
            const uint8_t commandWrite,
            const uint8_t commandRead);

    esp_err_t ClrBit(
        const uint8_t regAddr,
        const uint8_t bitIdx,
        const uint8_t commandWrite,
        const uint8_t commandRead);

    esp_err_t WriteRegisterMultipleBytes(
            const uint8_t regAddr,
            void* regDataBuffer,
            const uint8_t byteCount,
            const uint8_t command);

    esp_err_t ReadRegisterMultipleBytes(
            const uint8_t regAddr,
            void* regDataBuffer,
            const uint8_t byteCount,
            const uint8_t command);
}; /* class Spi */
