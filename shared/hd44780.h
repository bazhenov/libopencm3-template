#pragma once

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <FreeRTOSConfig.h>
#include <FreeRTOS.h>
#include <task.h>

#define LCD_CLEARDISPLAY 0x01

#define LCD_RETURNHOME 0x02

#define LCD_ENTRYMODESET 0x04
#define LCD_ENTRYRIGHT 0x00
#define LCD_ENTRYLEFT 0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

#define LCD_DISPLAYCONTROL 0x08
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00

#define LCD_CURSORSHIFT 0x10
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80

#define LCD_FUNCTIONSET 0x20
#define LCD_8BITMODE 0x10
#define LCD_4BITMODE 0x00
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00
#define LCD_5x10DOTS 0x04
#define LCD_5x8DOTS 0x00

typedef struct {
  uint32_t port;
  uint16_t pin;
} pin_t;

template<
  pin_t const& RS,
  pin_t const& RW,
  pin_t const& E,
  // 8 pins assumed to be starting from DB0
  pin_t const& DB0
>
class Hd44780 {

  private:
    uint16_t db_pins;
    uint8_t displayControl;

  public:
    Hd44780() : displayControl(LCD_CURSOROFF | LCD_BLINKOFF | LCD_DISPLAYON) {
      // Db pins, Setting 8 pins started from db0
      uint16_t pins = DB0.pin;
      pins |= pins << 1;
      pins |= pins << 2;
      pins |= pins << 4;
      this->db_pins = pins;

      gpio_mode_setup(DB0.port, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, this->db_pins);
      gpio_set_output_options(DB0.port, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, this->db_pins);

      // RS pin
      gpio_mode_setup(RS.port, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, RS.pin);
      gpio_set_output_options(RS.port, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ,  RS.pin);

      // RW pin
      gpio_mode_setup(RW.port, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, RW.pin);
      gpio_set_output_options(RW.port, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ,  RW.pin);
      
      // E pin
      gpio_mode_setup(E.port, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, E.pin);
      gpio_set_output_options(E.port, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ,  E.pin);

      // HD44780U startup sequnce. See datasheet p. 45
      vTaskDelay(pdMS_TO_TICKS(41));

      writeInstruction<false>(LCD_FUNCTIONSET | LCD_8BITMODE);
      vTaskDelay(pdMS_TO_TICKS(5));
      writeInstruction<false>(LCD_FUNCTIONSET | LCD_8BITMODE);
      vTaskDelay(pdMS_TO_TICKS(1));
      writeInstruction<false>(LCD_FUNCTIONSET | LCD_8BITMODE);
      vTaskDelay(pdMS_TO_TICKS(1));
      writeInstruction<false>(LCD_FUNCTIONSET | LCD_8BITMODE | LCD_2LINE | LCD_5x8DOTS);
      vTaskDelay(pdMS_TO_TICKS(1));

      writeInstruction(LCD_DISPLAYCONTROL | LCD_DISPLAYOFF);

      writeInstruction(LCD_CLEARDISPLAY);

      writeInstruction(LCD_ENTRYMODESET | LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT);

      
      this->updateDisplayControl();
    }

    void clear() {
      writeInstruction(LCD_CLEARDISPLAY);
    }

    void blink() {
      this->displayControl |= LCD_BLINKON;
      this->updateDisplayControl();
    }

    void noBlink() {
      this->displayControl &= ~LCD_BLINKON;
      this->updateDisplayControl();
    }

    void cursor() {
      this->displayControl |= LCD_CURSORON;
      this->updateDisplayControl();
    }

    void noCursor() {
      this->displayControl &= ~LCD_CURSORON;
      this->updateDisplayControl();
    }

    void print(char *str) {
      while (*str != '\0') {
        writeData(*str);
        str++;
      }
    }
  
  private:
    inline void updateDisplayControl() {
      writeInstruction(LCD_DISPLAYCONTROL | this->displayControl);
    }

    template<bool wait_for_busyflag = true>
    void writeInstruction(uint8_t db) { 
      gpio_clear(RS.port, RS.pin);
      gpio_clear(RW.port, RW.pin);

      gpio_mode_setup(DB0.port, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, DB0.pin << 7);
      gpio_set_output_options(DB0.port, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, DB0.pin << 7);
      for (uint16_t gpio=DB0.pin; gpio <= DB0.pin << 7; gpio <<= 1) {
        if (db & 1) 
          gpio_set(DB0.port, gpio);
        else
          gpio_clear(DB0.port, gpio);
         db >>= 1;
      }

      strobe();

      if ( wait_for_busyflag )
        wait_for_busy();
    }

    void writeData(uint8_t db) { 
      gpio_set(RS.port, RS.pin);
      gpio_clear(RW.port, RW.pin);

      gpio_clear(DB0.port, this->db_pins);
      for (uint16_t gpio=DB0.pin; gpio <= DB0.pin << 7; gpio <<= 1) {
        if (db & 1) 
          gpio_set(DB0.port, gpio);
        db >>= 1;
      }

      strobe();
      wait_for_busy();
    }

    void wait_for_busy() {
      gpio_set(RW.port, RW.pin);
      gpio_clear(RS.port, RS.pin);

      // Temporary switching BF pin in input mode
      gpio_mode_setup(DB0.port, GPIO_MODE_INPUT, GPIO_PUPD_NONE, DB0.pin << 7);
      do {
        strobe();
      } while ( gpio_get(DB0.port, DB0.pin << 7) );
      gpio_mode_setup(DB0.port, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, DB0.pin << 7);
    }

    static inline void strobe() {
      gpio_set(E.port, E.pin);
      gpio_clear(E.port, E.pin);
      for (int i=0; i<100; i++) {
        __asm__("nop");
      }
      gpio_set(E.port, E.pin);
    }
};