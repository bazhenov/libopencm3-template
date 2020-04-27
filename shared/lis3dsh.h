#include <stdint.h>

#define LIS3DSH_REG_OUT_T	0x0C

/**
 * Read only status register containing 0x3F as a value
 */
#define LIS3DSH_REG_WHO_AM_I	0x0F

#define LIS3DSH_REG_CTRL_REG4	0x20
#define LIS3DSH_REG_CTRL_REG5	0x24

#define LIS3DSH_FSCALE_2G		(0b000 << 3)
#define LIS3DSH_FSCALE_4G		(0b001 << 3)
#define LIS3DSH_FSCALE_6G		(0b010 << 3)
#define LIS3DSH_FSCALE_8G		(0b011 << 3)
#define LIS3DSH_FSCALE_16G	(0b100 << 3)

#define LIS3DSH_ODR_POWER_DOWN		(0b0000 << 4)
#define LIS3DSH_ODR_POWER_3HZ			(0b0001 << 4)
#define LIS3DSH_ODR_POWER_12HZ		(0b0011 << 4)
#define LIS3DSH_ODR_POWER_25HZ		(0b0100 << 4)
#define LIS3DSH_ODR_POWER_50HZ		(0b0101 << 4)
#define LIS3DSH_ODR_POWER_100HZ		(0b0110 << 4)
#define LIS3DSH_ODR_POWER_400HZ		(0b0111 << 4)
#define LIS3DSH_ODR_POWER_800HZ		(0b1000 << 4)
#define LIS3DSH_ODR_POWER_1600HZ	(0b1001 << 4)

#define LIS3DSH_BDU_ENABLED				(1 << 3)
#define LIS3DSH_Z_ENABLED					(1 << 2)
#define LIS3DSH_Y_ENABLED					(1 << 1)
#define LIS3DSH_X_ENABLED					(1 << 0)

#define LIS3DSH_XYZ_ENABLED				(LIS3DSH_X_ENABLED | LIS3DSH_Y_ENABLED | LIS3DSH_Z_ENABLED)

#define LIS3DSH_REG_OUT_X_L		0x28
#define LIS3DSH_REG_OUT_X_H		0x29
#define LIS3DSH_REG_OUT_Y_L		0x2A
#define LIS3DSH_REG_OUT_Y_H		0x2B
#define LIS3DSH_REG_OUT_Z_L		0x2C
#define LIS3DSH_REG_OUT_Z_H		0x2D

class lis3dsh {

	private:
		uint32_t spi;
		uint8_t spi_transfer(uint16_t input);

	public:
		lis3dsh(uint32_t spi);

		uint8_t readRegister(uint8_t reg);
		void writeRegister(uint8_t ref, uint8_t value);
		void readXyz(int16_t *xyz);
};
