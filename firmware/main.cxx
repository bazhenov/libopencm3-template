#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/usb/usbd.h>
#include <cstring>
#include <FreeRTOSConfig.h>
#include <FreeRTOS.h>
#include <task.h>
#include <lis3dsh.h>
#include <hd44780.h>
#include <usbcdc.h>

static void systick_setup(void) {
	rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
	/* clock rate / 1000 to get 1mS interrupt rate */
	systick_set_reload(168000);
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
	systick_counter_enable();
	/* this done last */
	systick_interrupt_enable();
}

static void spi_setup() {
	rcc_periph_clock_enable(RCC_GPIOE);
	gpio_mode_setup(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLDOWN, GPIO3);
	gpio_set(GPIOE, GPIO3);

	// Use A5/6/7 as SPI
	rcc_periph_clock_enable(RCC_GPIOA);
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO5 | GPIO6 | GPIO7);
	gpio_set_af(GPIOA, GPIO_AF5, GPIO5 | GPIO6 | GPIO7);

	rcc_periph_clock_enable(RCC_SPI1);
	spi_init_master(SPI1, SPI_CR1_BAUDRATE_FPCLK_DIV_32,
									SPI_CR1_CPOL_CLK_TO_1_WHEN_IDLE,
									SPI_CR1_CPHA_CLK_TRANSITION_2,
									SPI_CR1_DFF_16BIT,
									SPI_CR1_MSBFIRST);
	spi_enable_software_slave_management(SPI1);
	spi_enable_ss_output(SPI1);
	spi_enable(SPI1);
}

static void led_task(void *) {
  for (;;) {
    gpio_toggle(GPIOD, GPIO12);
    vTaskDelay(500 / portTICK_PERIOD_MS);
		gpio_toggle(GPIOD, GPIO12);
    vTaskDelay(500 / portTICK_PERIOD_MS);

		gpio_toggle(GPIOD, GPIO12);
    vTaskDelay(100 / portTICK_PERIOD_MS);
		gpio_toggle(GPIOD, GPIO12);
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

static void task_accel(void *arg) {
	lis3dsh *acc = reinterpret_cast<lis3dsh*>(arg);

	if ( acc->readRegister(LIS3DSH_REG_WHO_AM_I) != 0x3F ) {
		// Incorrect register value reporting error
		for (;;) {
			gpio_toggle(GPIOD, GPIO13);
			vTaskDelay(pdMS_TO_TICKS(100));
		}
	}

	acc->writeRegister(LIS3DSH_REG_CTRL_REG4, LIS3DSH_ODR_POWER_50HZ | LIS3DSH_XYZ_ENABLED);
	acc->writeRegister(LIS3DSH_REG_CTRL_REG5, LIS3DSH_FSCALE_2G);

	int16_t xyz[3];

	while (1) {
		acc->readXyz(xyz);
		int8_t temp = acc->readRegister(0x0C);

		usb_vcp_printf("XYZ: [%10d, %10d, %10d]\n", xyz[0], xyz[1], xyz[2]);
		gpio_toggle(GPIOD, GPIO13);
		vTaskDelay(pdMS_TO_TICKS(500));
	}
}

static void usb_task(void *) {
	static char line[128];
	int pos = 0;

	for (;;) {
		vTaskDelay(pdMS_TO_TICKS(1));

		while ( usb_vcp_avail() > 0 ) {
			char next_char = usb_vcp_recv_byte();
			usb_vcp_printf("%c", next_char);
			if ( next_char == '\r' ) {
				line[pos] = 0;
				usb_vcp_printf("\n");

				if ( strcmp(line, "hello") == 0 ) {
					usb_vcp_printf("Hello to you too\n");
				} else if ( strcmp(line, "top") == 0 ) {
					
				} else {
					usb_vcp_printf("Unknown command: %s\n", line);
				}
				usb_vcp_printf("> ");

				pos = 0;
			} else {
				line[pos++] = next_char;
			}
		}
	}
}

static void lcd_task(void *) {
	Hd44780 led;

	for(;;);
}

int main(void) {
	systick_setup();
	
	rcc_periph_clock_enable(RCC_GPIOD);
	rcc_periph_clock_enable(RCC_GPIOE);
	rcc_periph_clock_enable(RCC_GPIOA);

	spi_setup();
	static lis3dsh acc(SPI1);

	usb_vcp_init();

	gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO12 | GPIO13);
	
	xTaskCreate(led_task, "LED", 100, NULL, configMAX_PRIORITIES - 1, NULL);
	xTaskCreate(task_accel, "task_accel", 100, &acc, configMAX_PRIORITIES - 1, NULL);
	xTaskCreate(usb_task, "usb_task", 100, NULL, configMAX_PRIORITIES - 1, NULL);
	xTaskCreate(lcd_task, "lcd_task", 100, NULL, configMAX_PRIORITIES - 1, NULL);
	
	vTaskStartScheduler();

	for (;;);
	return 0;
}
