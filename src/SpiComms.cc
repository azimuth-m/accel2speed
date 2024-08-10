#include "esp_err.h"
#include "driver/spi_common.h"
#include "esp_intr_alloc.h"
#include "SpiComms.h"

Spi::Spi(
        const spi_host_device_t spiPeripheral,
        const int pinMosi,
        const int pinMiso,
        const int pinSclk) {

    spiPeripheral_            = spiPeripheral;
    spiTransaction_.tx_buffer = nullptr;
    spiTransaction_.rx_buffer = nullptr;
    spiBusCfg_.mosi_io_num   = pinMosi;
    spiBusCfg_.miso_io_num   = pinMiso;
    spiBusCfg_.sclk_io_num   = pinSclk;
    spiBusCfg_.quadwp_io_num = -1; // write protect
    spiBusCfg_.quadhd_io_num = -1; // Hold
    // spi_bus_cfg_.intr_flags = ESP_INTR_FLAG_INTRDISABLED;

    spi_bus_initialize(
            spiPeripheral,
            &spiBusCfg_,
            SPI_DMA_CH_AUTO);
};

esp_err_t Spi::RegisterDevice(
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

esp_err_t Spi::TransferByte(
        const uint8_t regAddress,
        const uint8_t data,
        const uint8_t command) {

    Spi::spiTransaction_.flags =
            SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA;
    Spi::spiTransaction_.cmd = command;
    Spi::spiTransaction_.length = 8;
    Spi::spiTransaction_.addr = regAddress;
    Spi::spiTransaction_.tx_data[0] = data;

    return spi_device_transmit(handle_, &spiTransaction_);
}

esp_err_t Spi::TransferMultipleBytes(
        const uint8_t regAddr,
        void* txBuf,
        void* rxBuf,
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

uint8_t Spi::ReadRegister(
        const uint8_t regAddress,
        const uint8_t command) {

    TransferByte(regAddress, 0, command);
    return Spi::spiTransaction_.rx_data[0];
}

esp_err_t Spi::WriteRegister(
        const uint8_t regAddr,
        const uint8_t regData,
        const uint8_t command) {

    esp_err_t status = ESP_OK;
    status |= TransferByte(regAddr, regData, command);
    return status;
}

esp_err_t Spi::SetBit(
        const uint8_t regAddr,
        const uint8_t bitIdx,
        const uint8_t commandWrite,
        const uint8_t commandRead) {

    uint8_t oldValue;
    oldValue = Spi::ReadRegister(regAddr, commandRead);
    oldValue |= (1u << bitIdx);
    return Spi::WriteRegister(regAddr, oldValue, commandWrite);
}

esp_err_t Spi::ClearBit(
        const uint8_t regAddr,
        const uint8_t bitIdx,
        const uint8_t commandWrite,
        const uint8_t commandRead) {

    uint8_t oldValue;
    oldValue = Spi::ReadRegister(regAddr, commandRead);
    oldValue &= ~(1u << bitIdx);
    return Spi::WriteRegister(regAddr, oldValue, commandWrite);
}

esp_err_t Spi::WriteRegisterMultipleBytes(
        const uint8_t regAddr,
        void* regDataBuffer,
        const uint8_t byteCount,
        const uint8_t command) {
    return TransferMultipleBytes(
            regAddr, regDataBuffer, nullptr, byteCount, command);
}

esp_err_t Spi::ReadRegisterMultipleBytes(
        const uint8_t regAddr,
        void* regDataBuffer,
        const uint8_t byteCount,
        const uint8_t command) {
    return TransferMultipleBytes(
            regAddr, nullptr, regDataBuffer, byteCount, command);
}
