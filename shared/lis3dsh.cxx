#include "lis3dsh.h"
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/gpio.h>

#define REG_READ(x)			((x & 0x7F) | (1 << 7))
#define REG_WRITE(x)		((x & 0x7F) | (0 << 7))

lis3dsh::lis3dsh(uint32_t spi) : spi(spi){
}

void lis3dsh::writeRegister(uint8_t reg, uint8_t value) {
	spi_transfer((REG_WRITE(reg) << 8) | value);
}

uint8_t lis3dsh::readRegister(uint8_t reg) {
	return spi_transfer(REG_READ(reg) << 8);
}

uint8_t lis3dsh::spi_transfer(uint16_t payload) {
	gpio_clear(GPIOE, GPIO3);
	uint16_t output = spi_xfer(spi, payload);
	gpio_set(GPIOE, GPIO3);
	return output & 0xFF;
}

void lis3dsh::readXyz(int16_t *xyz) {
	uint8_t xl = readRegister(LIS3DSH_REG_OUT_X_L);
	uint8_t xh = readRegister(LIS3DSH_REG_OUT_X_H);

	uint8_t yl = readRegister(LIS3DSH_REG_OUT_Y_L);
	uint8_t yh = readRegister(LIS3DSH_REG_OUT_Y_H);

	uint8_t zl = readRegister(LIS3DSH_REG_OUT_Z_L);
	uint8_t zh = readRegister(LIS3DSH_REG_OUT_Z_H);

	xyz[0] = (xh << 8) | xl;
	xyz[1] = (yh << 8) | yl;
	xyz[2] = (zh << 8) | zl;
}