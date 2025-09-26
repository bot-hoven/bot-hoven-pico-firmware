#include "include/SPIHandler.h"
#include "pico/stdlib.h"
#include <stdio.h>

SPIHandler::SPIHandler(spi_inst_t *port, uint miso, uint mosi, uint sck,
                       uint cs, uint32_t baud)
    : spi_(port), pin_miso(miso), pin_mosi(mosi), pin_sck(sck), pin_cs(cs),
      baudrate(baud) {}

void SPIHandler::begin() {
  spi_init(spi_, baudrate);
  spi_set_format(spi_, 8, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST);
  spi_set_slave(spi_, true);
  gpio_set_function(pin_miso, GPIO_FUNC_SPI);
  gpio_set_function(pin_mosi, GPIO_FUNC_SPI);
  gpio_set_function(pin_sck, GPIO_FUNC_SPI);
  gpio_set_function(pin_cs, GPIO_FUNC_SPI);
}

void SPIHandler::read_frame(uint8_t *buf, size_t len) {
  int bytes_read = spi_read_blocking(spi_, 0x00, buf, len);
//   printf("Bytes read on read: %d", bytes_read);
}

void SPIHandler::write_response(const uint8_t *buf, size_t len) {
  int bytes_written = spi_write_blocking(spi_, buf, len);
//   printf("Bytes written on write: %d", bytes_written);

}
