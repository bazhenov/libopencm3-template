#include <libopencm3/stm32/gpio.h>

class Hd44780 {

  public:
    Hd44780();

  private:
    void write(bool rs, bool rw, uint8_t db);
};