#pragma once

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <FreeRTOSConfig.h>
#include <FreeRTOS.h>
#include <task.h>

#define INSTRUCTION_CLEARDISPLAY      (1 << 0)

#define INSTRUCTION_RETURNHOME        (1 << 1)

#define INSTRUCTION_ENTRYMODESET(x)   ((1 << 2) | (x & 0b00000011))
#define LCD_ENTRYRIGHT 0x00
#define LCD_ENTRYLEFT 0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

#define INSTRUCTION_DISPLAYCONTROL(x) ((1 << 3) | (x & 0b00000111))
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00

#define INSTRUCTION_SHIFT(x)          ((1 << 4) | (x & 0b00001100))

#define INSTRUCTION_FUNCTIONSET(x)    ((1 << 5) | (x & 0b00011100))
#define LCD_8BITMODE 0x10
#define LCD_4BITMODE 0x00
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00
#define LCD_5x10DOTS 0x04
#define LCD_5x8DOTS 0x00

#define INSTRUCTION_SET_CGRAM_ADDR(x) ((1 << 6) | (x & 0b00111111))
#define INSTRUCTION_SET_DDRAM_ADDR(x) ((1 << 7) | (x & 0b01111111))

#define LCD_COLS 64

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
      db_pins = pins;

      gpio_mode_setup(DB0.port, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, db_pins);
      gpio_set_output_options(DB0.port, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, db_pins);

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

      writeInstruction<false>(INSTRUCTION_FUNCTIONSET(LCD_8BITMODE));
      vTaskDelay(pdMS_TO_TICKS(5));
      writeInstruction<false>(INSTRUCTION_FUNCTIONSET(LCD_8BITMODE));
      vTaskDelay(pdMS_TO_TICKS(1));
      writeInstruction<false>(INSTRUCTION_FUNCTIONSET(LCD_8BITMODE));
      vTaskDelay(pdMS_TO_TICKS(1));
      writeInstruction<false>(INSTRUCTION_FUNCTIONSET(LCD_8BITMODE | LCD_2LINE | LCD_5x8DOTS));
      vTaskDelay(pdMS_TO_TICKS(1));

      writeInstruction(INSTRUCTION_DISPLAYCONTROL(LCD_DISPLAYOFF));

      writeInstruction(INSTRUCTION_CLEARDISPLAY);

      writeInstruction(INSTRUCTION_ENTRYMODESET(LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT));
      // HD44780U startup sequnce complete
      
      updateDisplayControl();
    }

    void clear() {
      writeInstruction(INSTRUCTION_CLEARDISPLAY);
    }

    void blink() {
      displayControl |= LCD_BLINKON;
      updateDisplayControl();
    }

    void noBlink() {
      displayControl &= ~LCD_BLINKON;
      updateDisplayControl();
    }

    void cursor() {
      displayControl |= LCD_CURSORON;
      updateDisplayControl();
    }

    void noCursor() {
      displayControl &= ~LCD_CURSORON;
      updateDisplayControl();
    }

    void position(uint8_t col, uint8_t row) {
      writeInstruction(INSTRUCTION_SET_DDRAM_ADDR(col + LCD_COLS * row));
    }

    void print(char *str) {
      while (*str) {
        writeData(*str);
        str++;
      }
    }
  
  private:
    inline void updateDisplayControl() {
      writeInstruction(INSTRUCTION_DISPLAYCONTROL(this->displayControl));
    }

    void write_db(uint8_t db) {
      gpio_clear(DB0.port, this->db_pins);
      for (uint16_t gpio=DB0.pin; gpio <= DB0.pin << 7; gpio <<= 1) {
        if (db & 1) 
          gpio_set(DB0.port, gpio);
        db >>= 1;
      }
    }

    template<bool wait_for_busyflag = true>
    void writeInstruction(uint8_t db) { 
      gpio_clear(RS.port, RS.pin);
      gpio_clear(RW.port, RW.pin);
      write_db(db);
      strobe();

      if ( wait_for_busyflag )
        wait_for_busy();
    }

    void writeData(uint8_t db) { 
      gpio_set(RS.port, RS.pin);
      gpio_clear(RW.port, RW.pin);
      write_db(db);
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
      for (int i=0; i<10; i++) {
        __asm__("nop");
      }
      gpio_set(E.port, E.pin);
    }
};