/*
 * entry of program
 *
 *
 */

#include "fsl_debug_console.h"
#include "configuration.h"
#include "lcd_PCD8544.h"
#include "mcu_MKL02Z32VFM4.h"

#include "fsl_gpio.h"


int main(void)
{

  //initial MCU
  initial_mcu();
    

  //initial devices
  lcd_initial();

  PRINTF("Start write LCD. \r\n");
  lcd_update_display_buffer(0, 1, "hello");
  lcd_update_display_buffer(1, 4, "hellow world");
  lcd_update_display_buffer(2, 5, "I'm ok");
  lcd_update_display_buffer(3, 7, "blablabla");
  lcd_update_display_buffer(4, 4, "hahahah");
  lcd_update_display_buffer(5, 5, "!!!");

  PRINTF("refresh LCD. \r\n");
  lcd_refresh_screen();

  while(1)
  {

  }
  return 0;
}
