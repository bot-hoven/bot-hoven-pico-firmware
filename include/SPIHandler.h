#include "hardware/spi.h"

class SPIHandler {
public:
  SPIHandler(spi_inst_t *port, uint miso, uint mosi, uint sck, uint cs,
             uint32_t baud);

  // initialize SPI peripheral
  void begin();

  // read exactly len bytes (blocks until complete)
  void read_frame(uint8_t *buf, size_t len);

  // write exactly len bytes (blocks until clocked out)
  void write_response(const uint8_t *buf, size_t len);

private:
  spi_inst_t *spi_;
  uint pin_miso;
  uint pin_mosi;
  uint pin_sck;
  uint pin_cs;
  uint32_t baudrate;
};
