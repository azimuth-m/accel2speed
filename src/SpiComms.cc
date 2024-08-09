#include "esp_err.h"
#include "driver/spi_common.h"
#include "esp_intr_alloc.h"
#include "SpiComms.h"

esp_err_t accspi::Spi::Init(
        const spi_host_device_t spiPeripheral,
        const int pinMosi,
        const int pinMiso,
        const int pinSclk) {

    spiPeripheral_            = spiPeripheral;
    spiTransaction_.tx_buffer = nullptr;
    spiTransaction_.rx_buffer = nullptr;

    // std::memset(&spi_bus_cfg_, 0, sizeof(spi_bus_config_t));
    spiBusCfg_.mosi_io_num   = pinMosi;
    spiBusCfg_.miso_io_num   = pinMiso;
    spiBusCfg_.sclk_io_num   = pinSclk;
    spiBusCfg_.quadwp_io_num = -1; // write protect
    spiBusCfg_.quadhd_io_num = -1; // Hold
    // spi_bus_cfg_.intr_flags = ESP_INTR_FLAG_INTRDISABLED;

    esp_err_t status = ESP_OK;
    status |= spi_bus_initialize(
            spiPeripheral,
            &spiBusCfg_,
            SPI_DMA_CH_AUTO);
    return status;
};

esp_err_t accspi::Spi::RegisterDevice(
        const uint8_t mode,
        const int csPin,
        const uint8_t commandLenght,
        const uint8_t addrLenght,
        const uint8_t dummyLenght,
        const int busSpeed,
        const uint8_t csEnaPretrans,
        const uint8_t csEnaPosttrans,
        const uint32_t flags) {

    esp_err_t status = ESP_OK;

    // Init to zero
    spiInterfaceCfg_ = {};
    spiInterfaceCfg_.command_bits = commandLenght;
    spiInterfaceCfg_.address_bits = addrLenght;
    spiInterfaceCfg_.dummy_bits = dummyLenght;
    spiInterfaceCfg_.clock_speed_hz = busSpeed;
    spiInterfaceCfg_.mode = mode;


    spiInterfaceCfg_.spics_io_num = csPin;
    spiInterfaceCfg_.cs_ena_pretrans = csEnaPretrans;
    spiInterfaceCfg_.cs_ena_posttrans = csEnaPosttrans;
    spiInterfaceCfg_.flags = flags;

    // If buffer is <= 0, panic
    spiInterfaceCfg_.queue_size = 5;

    status |= spi_bus_add_device(
            spiPeripheral_,
            &spiInterfaceCfg_,
            &handle_);
    return status;
}

esp_err_t accspi::Spi::TransferByte(
        const uint8_t regAddress,
        const uint8_t data,
        const uint8_t command) {

    accspi::Spi::spiTransaction_.flags =
            SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA;
    accspi::Spi::spiTransaction_.cmd = command;
    accspi::Spi::spiTransaction_.length = TRANS_BITS;
    accspi::Spi::spiTransaction_.addr = regAddress;
    accspi::Spi::spiTransaction_.tx_data[0] = data;

    return spi_device_transmit(handle_, &spiTransaction_);
}

esp_err_t accspi::Spi::TransferMultipleBytes(
        const uint8_t regAddr,
        uint8_t* txBuf,
        uint8_t* rxBuf,
        size_t dataLength,
        const uint8_t command) {
    spi_transaction_t spiTransactionMultibyte;

    if (dataLength < 1) {
        dataLength = 1;
    }

    spiTransactionMultibyte.flags = 0;
    spiTransactionMultibyte.length = (8 * dataLength);
    spiTransactionMultibyte.rxlength = 0;
    spiTransactionMultibyte.cmd = command;
    spiTransactionMultibyte.addr = regAddr;
    spiTransactionMultibyte.tx_buffer = txBuf;
    spiTransactionMultibyte.rx_buffer = rxBuf;

    return spi_device_transmit(handle_, &spiTransactionMultibyte);
}

uint8_t accspi::Spi::ReadRegister(
        const uint8_t regAddress,
        const uint8_t command) {

    accspi::Spi::TransferByte(regAddress, 0, command);
    return accspi::Spi::spiTransaction_.rx_data[0];
}

esp_err_t accspi::Spi::WriteRegister(
        const uint8_t regAddr,
        const uint8_t regData,
        const uint8_t command) {

    esp_err_t status{ESP_OK};
    status |= TransferByte(regAddr, regData, command);
    return status;
}

esp_err_t accspi::Spi::WriteRegisterMultipleBytes(
        const uint8_t regAddr,
        uint8_t* regDataBuffer,
        const uint8_t byteCount,
        const uint8_t command) {
    return TransferMultipleBytes(
            regAddr, regDataBuffer, nullptr, byteCount, command);
}

esp_err_t accspi::Spi::ReadRegisterMultipleBytes(
        const uint8_t regAddr,
        uint8_t* regDataBuffer,
        const uint8_t byteCount,
        const uint8_t command) {
    return TransferMultipleBytes(
            regAddr, nullptr, regDataBuffer, byteCount, command);
}
