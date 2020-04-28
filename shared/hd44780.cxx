#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <FreeRTOSConfig.h>
#include <FreeRTOS.h>
#include <task.h>
#include "hd44780.h"

#define LCD_FUNCTIONSET 0x20
#define LCD_8BITMODE 0x10
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00

#define LCD_5x10DOTS 0x04
#define LCD_5x8DOTS 0x00

#define LCD_DISPLAYCONTROL 0x08
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00

#define DB_PORT GPIOA
#define DB_PINS (GPIO0 | GPIO1 | GPIO2 | GPIO3 | GPIO4 | GPIO5 | GPIO6 | GPIO7)

#define RS_PORT GPIOE
#define RS_PIN GPIO0

#define RW_PORT GPIOE
#define RW_PIN GPIO1

#define E_PORT GPIOE
#define E_PIN GPIO2

Hd44780::Hd44780() {
  // Db pins
  gpio_mode_setup(DB_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, DB_PINS);
  gpio_set_output_options(DB_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ,  DB_PINS);

  // RS pin
  gpio_mode_setup(RS_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, RS_PIN);
  gpio_set_output_options(RS_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ,  RS_PIN);

  // RW pin
  gpio_mode_setup(RW_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, RW_PIN);
  gpio_set_output_options(RW_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ,  RW_PIN);
  
  // E pin
  gpio_mode_setup(E_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, E_PIN);
  gpio_set_output_options(E_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ,  E_PIN);

  vTaskDelay(pdMS_TO_TICKS(40));
  write(false, false, LCD_FUNCTIONSET | LCD_8BITMODE);
  vTaskDelay(pdMS_TO_TICKS(10));
  write(false, false, LCD_FUNCTIONSET | LCD_8BITMODE);
  vTaskDelay(pdMS_TO_TICKS(10));
  write(false, false, LCD_FUNCTIONSET | LCD_8BITMODE);
  vTaskDelay(pdMS_TO_TICKS(10));

  write(false, false, LCD_FUNCTIONSET | LCD_8BITMODE | LCD_2LINE | LCD_5x8DOTS);
  vTaskDelay(pdMS_TO_TICKS(10));

  write(false, false, LCD_DISPLAYCONTROL | LCD_BLINKON | LCD_DISPLAYON | LCD_CURSORON);
}

void Hd44780::write(bool rs, bool rw, uint8_t db) {
  if (rs)
    gpio_set(RS_PORT, RS_PIN);
  else
    gpio_clear(RS_PORT, RS_PIN);


  if (rw)
    gpio_set(RW_PORT, RW_PIN);
  else
    gpio_clear(RW_PORT, RW_PIN);

  for (uint16_t gpio=GPIO0; gpio<GPIO8; gpio <<= 1) {
    if (db & gpio) 
      gpio_set(DB_PORT, gpio);
    else
      gpio_clear(DB_PORT, gpio);
  }

  gpio_set(E_PORT, E_PIN);
  vTaskDelay(pdMS_TO_TICKS(2));
  gpio_clear(E_PORT, E_PIN);
  vTaskDelay(pdMS_TO_TICKS(2));
  gpio_set(E_PORT, E_PIN);
  
}