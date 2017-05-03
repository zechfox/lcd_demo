/*
 * mcu MKL02Z32VFM4
 *
 *
 */
#include "fsl_common.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "fsl_spi.h"
#include "fsl_smc.h"
#include "fsl_debug_console.h"
#include "fsl_lpsci.h"

#include "configuration.h"
#include "mcu_MKL02Z32VFM4.h"

static void CLOCK_CONFIG_FllStableDelay(void)
{
  uint32_t i = 30000U;
  while (i--)
  {
    __NOP();
  }
}

bool initial_mcu(void)
{
  PRINTF("Start initial MCU. \r\n");
  if(!initial_run_clk())
  {
    return false;
  }

  if(!initial_gpio())
  {
    return false;
  }

  if(!initial_uart())
  {
    return false;
  }

  if(!initial_master_spi())
  {
    return false;
  }
  if(!initial_i2c())
  {
    return false;
  }

  if(!initial_adc())
  {
    return false;
  }

  PRINTF("Initial MCU finished. \r\n");

  return true;
}

bool initial_run_clk(void)
{
  PRINTF("Start initial MCU Clock as RUN Mode. \r\n");

  const mcg_config_t mcgConfig_BOARD_BootClockRUN =
  {
    .mcgMode = kMCG_ModeFEE,                  /* FEE - FLL Engaged External */
    .irclkEnableMode = kMCG_IrclkEnable,      /* MCGIRCLK enabled, MCGIRCLK disabled in STOP mode */
    .ircs = kMCG_IrcSlow,                     /* Slow internal reference clock selected */
    .fcrdiv = 0x0U,                           /* Fast IRC divider: divided by 1 */
    .frdiv = 0x0U,                            /* FLL reference clock divider: divided by 1 */
    .drs = kMCG_DrsMid,                       /* Mid frequency range */
    .dmx32 = kMCG_Dmx32Fine,                  /* DCO is fine-tuned for maximum frequency with 32.768 kHz reference */
  };
  const sim_clock_config_t simConfig_BOARD_BootClockRUN =
  {
    .clkdiv1 = 0x10000U,                      /* SIM_CLKDIV1 - OUTDIV1: /1, OUTDIV4: /2 */
  };
  const osc_config_t oscConfig_BOARD_BootClockRUN =
  {
    .freq = 32768U,                           /* Oscillator frequency: 32768Hz */
    .capLoad = (OSC_CAP0P),                   /* Oscillator capacity load: 0pF */
    .workMode = kOSC_ModeOscLowPower,         /* Oscillator low power */
    .oscerConfig =
      {
        .enableMode = kOSC_ErClkEnable,   /* Enable external reference clock, disable external reference clock in STOP mode */
      }
  };

  /* Set the system clock dividers in SIM to safe value. */
  CLOCK_SetSimSafeDivs();
  /* Initializes OSC0 according to board configuration. */
  CLOCK_InitOsc0(&oscConfig_BOARD_BootClockRUN);
  CLOCK_SetXtal0Freq(oscConfig_BOARD_BootClockRUN.freq);
  /* Set MCG to FEE mode. */
  CLOCK_BootToFeeMode(kMCG_OscselOsc,
                      mcgConfig_BOARD_BootClockRUN.frdiv,
                      mcgConfig_BOARD_BootClockRUN.dmx32,
                      mcgConfig_BOARD_BootClockRUN.drs,
                      CLOCK_CONFIG_FllStableDelay);
  /* Configure the Internal Reference clock (MCGIRCLK). */
  CLOCK_SetInternalRefClkConfig(mcgConfig_BOARD_BootClockRUN.irclkEnableMode,
                                mcgConfig_BOARD_BootClockRUN.ircs,
                                mcgConfig_BOARD_BootClockRUN.fcrdiv);
  /* Set the clock configuration in SIM module. */
  CLOCK_SetSimConfig(&simConfig_BOARD_BootClockRUN);
  /* Set SystemCoreClock variable. */
  SystemCoreClock = RUN_MODE_CORE_CLOCK;

  PRINTF("Initial MCU clock finished. \r\n");

  return true;

}

bool initial_vlpr_clk(void)
{
  PRINTF("Start initial MCU clock as VLPR. \r\n");

  PRINTF("Initial MCU clock finished. \r\n");

  return true;
}


bool initial_gpio(void)
{

  PRINTF("Start initial MCU GPIO. \r\n");
  
  unsigned int pin_masks = 0;

  /* Define the init structure for the input pin*/
  gpio_pin_config_t input_pin_config = {
    kGPIO_DigitalInput, 0,
  };

  /* Define the init structure for the output pin*/
  gpio_pin_config_t output_pin_config = {
    kGPIO_DigitalOutput, 0,
  };

#if defined(GPIO_A_OUTPUT_PINS) || defined(GPIO_A_INPUT_PINS)
  CLOCK_EnableClock(kCLOCK_PortA); 

#ifdef GPIO_A_INPUT_PINS  
  pin_masks = GPIO_A_INPUT_PINS;
  for(int i = 0;i < 32;i++)
  {
    if(pin_masks & (1 << i))
    {
      PORT_SetPinMux(PORTA, i, kPORT_MuxAsGpio);
      GPIO_PinInit(GPIOA, i, &input_pin_config);
      PRINTF("Configure GPIO A Pin %d as Input.\n", i);
    }
  }
#endif

#ifdef GPIO_A_OUTPUT_PINS
  pin_masks = GPIO_A_OUTPUT_PINS;
  for(int i = 0;i < 32;i++)
  {
    if(pin_masks & (1 << i))
    {
      PORT_SetPinMux(PORTA, i, kPORT_MuxAsGpio);
      GPIO_PinInit(GPIOA, i, &output_pin_config);
      PRINTF("Configure GPIO A Pin %d as Output.\n", i);
    }
  }
#endif

#endif

#if defined(GPIO_B_OUTPUT_PINS) || defined(GPIO_B_INPUT_PINS)
  CLOCK_EnableClock(kCLOCK_PortB); 

#ifdef GPIO_B_INPUT_PINS  
  pin_masks = GPIO_B_INPUT_PINS;
  for(int i = 0;i < 32;i++)
  {
    if(pin_masks & (1 << i))
    {
      PORT_SetPinMux(PORTB, i, kPORT_MuxAsGpio);
      GPIO_PinInit(GPIOB, i, &input_pin_config);
      PRINTF("Configure GPIO B Pin %d as Input.\n", i);

    }
  }
#endif

#ifdef GPIO_B_OUTPUT_PINS
  pin_masks = GPIO_B_OUTPUT_PINS;
  for(int i = 0;i < 32;i++)
  {
    if(pin_masks & (1 << i))
    {
      PORT_SetPinMux(PORTB, i, kPORT_MuxAsGpio);
      GPIO_PinInit(GPIOB, i, &output_pin_config);
      PRINTF("Configure GPIO B Pin %d as Output.\n", i);
    }
  }
#endif

#endif
  PRINTF("Initial MCU GPIO finished. \r\n");

  return true;
}

bool initial_uart(void)
{
  PRINTF("Start initial MCU UART. \r\n");
  
#ifdef USE_UART0

#if !defined(GPIO_A_OUTPUT_PINS) || !defined(GPIO_A_INPUT_PINS)
    CLOCK_EnableClock(kCLOCK_PortA);
#endif

#if !defined(GPIO_B_OUTPUT_PINS) || !defined(GPIO_B_INPUT_PINS)
    CLOCK_EnableClock(kCLOCK_PortB);
#endif

#define SOPT5_UART0RXSRC_UART_RX      0x00u   /*!< UART0 receive data source select: UART0_RX pin */
#define SOPT5_UART0TXSRC_UART_TX      0x00u   /*!< UART0 transmit data source select: UART0_TX pin */


  PORT_SetPinMux(PORTB, 1, kPORT_MuxAlt2);            /* PORTB1 (pin 17) is configured as UART0_TX */
  PORT_SetPinMux(PORTB, 2, kPORT_MuxAlt2);            /* PORTB2 (pin 18) is configured as UART0_RX */
  SIM->SOPT5 = ((SIM->SOPT5 &
    (~(SIM_SOPT5_UART0TXSRC_MASK | SIM_SOPT5_UART0RXSRC_MASK))) /* Mask bits to zero which are setting */
      | SIM_SOPT5_UART0TXSRC(SOPT5_UART0TXSRC_UART_TX)       /* UART0 transmit data source select: UART0_TX pin */
      | SIM_SOPT5_UART0RXSRC(SOPT5_UART0RXSRC_UART_RX)       /* UART0 receive data source select: UART0_RX pin */
  );

  CLOCK_SetLpsci0Clock(0x1); /* select FLL clock for LPSCI */
#ifdef UART0_DEBUG
  DbgConsole_Init((uint32_t)UART0, UART_BAUDRATE, DEBUG_CONSOLE_DEVICE_TYPE_LPSCI, CLOCK_GetFllFreq());
#else
  lpsci_config_t config;
  /*
   * config.parityMode = kLPSCI_ParityDisabled;
   * config.stopBitCount = kLPSCI_OneStopBit;
   * config.enableTx = false;
   * config.enableRx = false;
  */
  LPSCI_GetDefaultConfig(&config);
  config.baudRate_Bps = UART_BAUDRATE;
  config.enableTx = true;
  config.enableRx = true;
  LPSCI_Init(UART0, &config, CLOCK_GetFllFreq());
#endif

#endif
  PRINTF("Initial MCU UART finished. \r\n");

  return true;

}

bool initial_master_spi(void)
{
  PRINTF("Start initial MCU SPI as master. \r\n");
  
#ifdef USE_SPI

  spi_master_config_t masterConfig = {0};
  uint32_t sourceClock = 0U;
#if (GPIO_A_OUTPUT_PINS & 0xE0) || (GPIO_A_OUTPUT_PINS & 0xE0)
#error Inconsistance configuration: SPI pins were occupied by GPIO A.
#endif
#if (GPIO_B_OUTPUT_PINS & 0x01) || (GPIO_B_OUTPUT_PINS & 0x01)
#error Inconsistance configuration: SPI pins were occupied by GPIO B.
#endif

#if !defined(GPIO_A_OUTPUT_PINS) || !defined(GPIO_A_INPUT_PINS)
    CLOCK_EnableClock(kCLOCK_PortA);
#endif

#if !defined(GPIO_B_OUTPUT_PINS) || !defined(GPIO_B_INPUT_PINS)
    CLOCK_EnableClock(kCLOCK_PortB);
#endif

  PORT_SetPinMux(PORTA, 5, kPORT_MuxAlt3);            /* PORTA5 (pin 9) is configured as SPI0_SS_b */
  PORT_SetPinMux(PORTA, 6, kPORT_MuxAlt3);            /* PORTA6 (pin 10) is configured as SPI0_MISO */
  PORT_SetPinMux(PORTA, 7, kPORT_MuxAlt3);            /* PORTA7 (pin 15) is configured as SPI0_MOSI */
  PORT_SetPinMux(PORTB, 0, kPORT_MuxAlt3);            /* PORTB0 (pin 16) is configured as SPI0_SCK */

  /* Init SPI master */
  /*
   * masterConfig->enableStopInWaitMode = false;
   * masterConfig->polarity = kSPI_ClockPolarityActiveHigh;
   * masterConfig->phase = kSPI_ClockPhaseFirstEdge;
   * masterConfig->direction = kSPI_MsbFirst;
   * masterConfig->dataMode = kSPI_8BitMode;
   * masterConfig->txWatermark = kSPI_TxFifoOneHalfEmpty;
   * masterConfig->rxWatermark = kSPI_RxFifoOneHalfFull;
   * masterConfig->pinMode = kSPI_PinModeNormal;
   * masterConfig->outputMode = kSPI_SlaveSelectAutomaticOutput;
   * masterConfig->baudRate_Bps = 500000U;
   */
  SPI_MasterGetDefaultConfig(&masterConfig);
  sourceClock = DEFAULT_SPI_MASTER_CLK_FREQ;
  SPI_MasterInit(DEFAULT_SPI_MASTER, &masterConfig, sourceClock);


#endif
  PRINTF("Initial MCU SPI finished. \r\n");

  return true;
}

bool initial_i2c(void)
{
  PRINTF("Start initial MCU I2C. \r\n");

  PRINTF("Initial MCU I2C finished. \r\n");

  return true;
}

bool initial_adc(void)
{
  PRINTF("Start initial MCU ADC. \r\n");

  PRINTF("Initial MCU ADC finished. \r\n");

  return true;
}



