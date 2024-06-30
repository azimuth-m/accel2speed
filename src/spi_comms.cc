#include "esp_err.h"
#include "driver/spi_common.h"
#include "esp_intr_alloc.h"
#include "spi_comms.h"

esp_err_t accspi::Spi::init(
        const spi_host_device_t spi_peripheral,
        const int pin_mosi,
        const int pin_miso,
        const int pin_sclk) {

    spi_peripheral_            = spi_peripheral;
    spi_transaction_.tx_buffer = nullptr;
    spi_transaction_.rx_buffer = nullptr;

    // std::memset(&spi_bus_cfg_, 0, sizeof(spi_bus_config_t));
    spi_bus_cfg_.mosi_io_num   = pin_mosi;
    spi_bus_cfg_.miso_io_num   = pin_miso;
    spi_bus_cfg_.sclk_io_num   = pin_sclk;
    spi_bus_cfg_.quadwp_io_num = -1; // write protect
    spi_bus_cfg_.quadhd_io_num = -1; // Hold
    // spi_bus_cfg_.intr_flags = ESP_INTR_FLAG_INTRDISABLED;

    esp_err_t status = ESP_OK;
    status |= spi_bus_initialize(
            spi_peripheral,
            &spi_bus_cfg_,
            SPI_DMA_CH_AUTO);
    return status;
};

esp_err_t accspi::Spi::registerDevice(
        const uint8_t mode,
        const int cs_pin,
        const uint8_t command_lenght,
        const uint8_t addr_lenght,
        const uint8_t dummy_lenght,
        const int bus_speed,
        const uint8_t cs_ena_pretrans,
        const uint8_t cs_ena_posttrans,
        const uint32_t flags) {

    esp_err_t status = ESP_OK;

    // Init to zero
    spi_interface_cfg_ = {};
    spi_interface_cfg_.command_bits = command_lenght;
    spi_interface_cfg_.address_bits = addr_lenght;
    spi_interface_cfg_.dummy_bits = dummy_lenght;
    spi_interface_cfg_.clock_speed_hz = bus_speed;
    spi_interface_cfg_.mode = mode;

    spi_interface_cfg_.spics_io_num = cs_pin;
    spi_interface_cfg_.cs_ena_pretrans=cs_ena_pretrans;
    spi_interface_cfg_.cs_ena_posttrans=cs_ena_posttrans;
    spi_interface_cfg_.flags=flags;

    // If buffer is <= 0, panic
    spi_interface_cfg_.queue_size = 5;

    status |= spi_bus_add_device(
            spi_peripheral_,
            &spi_interface_cfg_,
            &handle_);
    return status;
}

esp_err_t accspi::Spi::transferByte(
        const uint8_t reg_address,
        const uint8_t data,
        const uint8_t command) {

    accspi::Spi::spi_transaction_.flags =
            SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA;
    accspi::Spi::spi_transaction_.cmd = command;
    accspi::Spi::spi_transaction_.length = 8;
    accspi::Spi::spi_transaction_.addr = reg_address;
    accspi::Spi::spi_transaction_.tx_data[0] = data;

    return spi_device_transmit(handle_, &spi_transaction_);
}

uint8_t accspi::Spi::readRegister(
        const uint8_t reg_address,
        const uint8_t command) {

    accspi::Spi::transferByte(reg_address, 0, command);
    return accspi::Spi::spi_transaction_.rx_data[0];
}
