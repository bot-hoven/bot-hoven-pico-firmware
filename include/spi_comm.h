#include "hardware/spi.h"

unsigned char* spi_slave_read_8_n_blocking(spi_inst_t *spi, unsigned char *data,  int len);
unsigned char spi_sub_read_8_blocking(spi_inst_t *spi);
void spi_sub_write_8_blocking(spi_inst_t *spi, unsigned char data);
void spi_sub_write_8_n_blocking(spi_inst_t *spi, unsigned char *data, int len);