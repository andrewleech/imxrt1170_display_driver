/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!GlobalInfo
product: Pins v13.0
processor: MIMXRT1176xxxxx
package_id: MIMXRT1176DVMAA
mcu_data: ksdk2_0
processor_version: 13.0.1
board: MIMXRT1170-EVK
pin_labels:
- {pin_num: R5, pin_signal: GPIO_LPSR_10, label: 'JTAG_nTRST/J1[3]/LPSPI6_SCK /J26[9]/DMIC_DATA1', identifier: DMIC_DATA1}
- {pin_num: D9, pin_signal: GPIO_DISP_B2_10, label: 'LPUART2_TXD/BT_UART_TXD/U354[4]/U16[2]/J25[3]/J9[4]', identifier: BT_UART_TXD;D1}
- {pin_num: A6, pin_signal: GPIO_DISP_B2_11, label: 'LPUART2_RXD/BT_UART_RXD/U16[3]/U355[20]/J9[2]', identifier: BT_UART_RXD;D0}
- {pin_num: B6, pin_signal: GPIO_DISP_B2_12, label: 'RGMII1_PHY_INTB/U10[31]/BT_UART_CTS/U16[5]/U355[19]/J9[6]', identifier: GPIO_DISP_B2_12;D2}
- {pin_num: M13, pin_signal: GPIO_AD_04, label: 'SIM1_PD/J44[C8]/USER_LED_CTL1/J9[8]/J25[7]', identifier: SIM1_PD;D3}
- {pin_num: P13, pin_signal: GPIO_AD_05, label: 'SIM1_PWR_FAIL/J9[12]/J25[5]/LCD_LPTE', identifier: SIM1_PWR_FAIL;D5}
- {pin_num: N13, pin_signal: GPIO_AD_06, label: 'USB_OTG2_OC/U18[A2]/J9[10]/AUD_INT', identifier: D4}
- {pin_num: L14, pin_signal: GPIO_AD_26, label: 'CSI_PWR_CTL/USER_LED_CTL2/J50[16]', identifier: RED_LED}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */

#include "fsl_common.h"
#include "fsl_iomuxc.h"
#include "fsl_gpio.h"
#include "pin_mux.h"

/* FUNCTION ************************************************************************************************************
 * 
 * Function Name : BOARD_InitBootPins
 * Description   : Calls initialization functions.
 * 
 * END ****************************************************************************************************************/
void BOARD_InitBootPins(void) {
}

/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
BOARD_InitLpuartPins:
- options: {callFromInitBoot: 'false', coreID: cm7, enableClock: 'true'}
- pin_list:
  - {pin_num: M15, peripheral: LPUART1, signal: RXD, pin_signal: GPIO_AD_25, software_input_on: Enable, pull_up_down_config: Pull_Down, pull_keeper_select: Keeper,
    open_drain: Disable, drive_strength: Normal, slew_rate: Slow}
  - {pin_num: L13, peripheral: LPUART1, signal: TXD, pin_signal: GPIO_AD_24, pull_up_down_config: Pull_Down, pull_keeper_select: Keeper, open_drain: Disable, drive_strength: Normal,
    slew_rate: Slow}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitLpuartPins, assigned for the Cortex-M7F core.
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void BOARD_InitLpuartPins(void) {
  CLOCK_EnableClock(kCLOCK_Iomuxc);           /* LPCG on: LPCG is ON. */

  IOMUXC_SetPinMux(
      IOMUXC_GPIO_AD_24_LPUART1_TXD,          /* GPIO_AD_24 is configured as LPUART1_TXD */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_AD_25_LPUART1_RXD,          /* GPIO_AD_25 is configured as LPUART1_RXD */
      1U);                                    /* Software Input On Field: Force input path of pad GPIO_AD_25 */
  IOMUXC_SetPinConfig(
      IOMUXC_GPIO_AD_24_LPUART1_TXD,          /* GPIO_AD_24 PAD functional properties : */
      0x00U);                                 /* Slew Rate Field: Slow Slew Rate
                                                 Drive Strength Field: normal drive strength
                                                 Pull / Keep Select Field: Pull Disable, Highz
                                                 Pull Up / Down Config. Field: Weak pull down
                                                 Open Drain Field: Disabled
                                                 Domain write protection: Both cores are allowed
                                                 Domain write protection lock: Neither of DWP bits is locked */
  IOMUXC_SetPinConfig(
      IOMUXC_GPIO_AD_25_LPUART1_RXD,          /* GPIO_AD_25 PAD functional properties : */
      0x00U);                                 /* Slew Rate Field: Slow Slew Rate
                                                 Drive Strength Field: normal drive strength
                                                 Pull / Keep Select Field: Pull Disable, Highz
                                                 Pull Up / Down Config. Field: Weak pull down
                                                 Open Drain Field: Disabled
                                                 Domain write protection: Both cores are allowed
                                                 Domain write protection lock: Neither of DWP bits is locked */
}


/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
BOARD_InitMipiPanelPins:
- options: {callFromInitBoot: 'false', coreID: cm7, enableClock: 'true'}
- pin_list:
  - {pin_num: A4, peripheral: GPIO11, signal: 'gpio_io, 16', pin_signal: GPIO_DISP_B2_15, software_input_on: Disable}
  - {pin_num: N8, peripheral: LPI2C5, signal: SCL, pin_signal: GPIO_LPSR_05, software_input_on: Enable, pull_up_down_config: Pull_Down, pull_keeper_select: Keeper,
    open_drain: Enable, drive_strength: Normal, slew_rate: Slow}
  - {pin_num: N7, peripheral: LPI2C5, signal: SDA, pin_signal: GPIO_LPSR_04, software_input_on: Enable, pull_up_down_config: Pull_Down, pull_keeper_select: Keeper,
    open_drain: Enable, drive_strength: Normal, slew_rate: Slow}
  - {pin_num: N12, peripheral: GPIO8, signal: 'gpio_io, 31', pin_signal: GPIO_AD_00, software_input_on: Enable, pull_up_down_config: Pull_Down, pull_keeper_select: Keeper,
    open_drain: Disable, drive_strength: Normal, slew_rate: Slow}
  - {pin_num: R14, peripheral: GPIO9, signal: 'gpio_io, 00', pin_signal: GPIO_AD_01, pull_up_down_config: Pull_Down, pull_keeper_select: Keeper, open_drain: Disable,
    drive_strength: Normal, slew_rate: Slow}
  - {pin_num: R13, peripheral: GPIO9, signal: 'gpio_io, 01', pin_signal: GPIO_AD_02}
  - {pin_num: K17, peripheral: GPIO9, signal: 'gpio_io, 29', pin_signal: GPIO_AD_30}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitMipiPanelPins, assigned for the Cortex-M7F core.
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void BOARD_InitMipiPanelPins(void) {
  CLOCK_EnableClock(kCLOCK_Iomuxc);           /* LPCG on: LPCG is ON. */
  CLOCK_EnableClock(kCLOCK_Iomuxc_Lpsr);      /* LPCG on: LPCG is ON. */

  IOMUXC_SetPinMux(
      IOMUXC_GPIO_AD_00_GPIO8_IO31,           /* GPIO_AD_00 is configured as GPIO8_IO31 */
      1U);                                    /* Software Input On Field: Force input path of pad GPIO_AD_00 */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_AD_01_GPIO9_IO00,           /* GPIO_AD_01 is configured as GPIO9_IO00 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_AD_02_GPIO9_IO01,           /* GPIO_AD_02 is configured as GPIO9_IO01 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_AD_30_GPIO9_IO29,           /* GPIO_AD_30 is configured as GPIO9_IO29 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_DISP_B2_15_GPIO11_IO16,     /* GPIO_DISP_B2_15 is configured as GPIO11_IO16 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_LPSR_04_LPI2C5_SDA,         /* GPIO_LPSR_04 is configured as LPI2C5_SDA */
      1U);                                    /* Software Input On Field: Force input path of pad GPIO_LPSR_04 */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_LPSR_05_LPI2C5_SCL,         /* GPIO_LPSR_05 is configured as LPI2C5_SCL */
      1U);                                    /* Software Input On Field: Force input path of pad GPIO_LPSR_05 */
  IOMUXC_SetPinConfig(
      IOMUXC_GPIO_AD_00_GPIO8_IO31,           /* GPIO_AD_00 PAD functional properties : */
      0x00U);                                 /* Slew Rate Field: Slow Slew Rate
                                                 Drive Strength Field: normal drive strength
                                                 Pull / Keep Select Field: Pull Disable, Highz
                                                 Pull Up / Down Config. Field: Weak pull down
                                                 Open Drain Field: Disabled
                                                 Domain write protection: Both cores are allowed
                                                 Domain write protection lock: Neither of DWP bits is locked */
  IOMUXC_SetPinConfig(
      IOMUXC_GPIO_AD_01_GPIO9_IO00,           /* GPIO_AD_01 PAD functional properties : */
      0x00U);                                 /* Slew Rate Field: Slow Slew Rate
                                                 Drive Strength Field: normal drive strength
                                                 Pull / Keep Select Field: Pull Disable, Highz
                                                 Pull Up / Down Config. Field: Weak pull down
                                                 Open Drain Field: Disabled
                                                 Domain write protection: Both cores are allowed
                                                 Domain write protection lock: Neither of DWP bits is locked */
  IOMUXC_SetPinConfig(
      IOMUXC_GPIO_LPSR_04_LPI2C5_SDA,         /* GPIO_LPSR_04 PAD functional properties : */
      0x20U);                                 /* Slew Rate Field: Slow Slew Rate
                                                 Drive Strength Field: normal driver
                                                 Pull / Keep Select Field: Pull Disable
                                                 Pull Up / Down Config. Field: Weak pull down
                                                 Open Drain LPSR Field: Enabled
                                                 Domain write protection: Both cores are allowed
                                                 Domain write protection lock: Neither of DWP bits is locked */
  IOMUXC_SetPinConfig(
      IOMUXC_GPIO_LPSR_05_LPI2C5_SCL,         /* GPIO_LPSR_05 PAD functional properties : */
      0x20U);                                 /* Slew Rate Field: Slow Slew Rate
                                                 Drive Strength Field: normal driver
                                                 Pull / Keep Select Field: Pull Disable
                                                 Pull Up / Down Config. Field: Weak pull down
                                                 Open Drain LPSR Field: Enabled
                                                 Domain write protection: Both cores are allowed
                                                 Domain write protection lock: Neither of DWP bits is locked */
}


/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
BOARD_InitMicPins:
- options: {callFromInitBoot: 'false', coreID: cm7, enableClock: 'true'}
- pin_list:
  - {pin_num: U8, peripheral: MIC, signal: CLK, pin_signal: GPIO_LPSR_08, slew_rate: Fast}
  - {pin_num: P5, peripheral: MIC, signal: 'mic_bitstream, 00', pin_signal: GPIO_LPSR_09, slew_rate: Fast}
  - {pin_num: R5, peripheral: MIC, signal: 'mic_bitstream, 01', pin_signal: GPIO_LPSR_10}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitMicPins, assigned for the Cortex-M7F core.
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void BOARD_InitMicPins(void) {
  CLOCK_EnableClock(kCLOCK_Iomuxc_Lpsr);      /* LPCG on: LPCG is ON. */

  IOMUXC_SetPinMux(
      IOMUXC_GPIO_LPSR_08_MIC_CLK,            /* GPIO_LPSR_08 is configured as MIC_CLK */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_LPSR_09_MIC_BITSTREAM0,     /* GPIO_LPSR_09 is configured as MIC_BITSTREAM0 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_LPSR_10_MIC_BITSTREAM1,     /* GPIO_LPSR_10 is configured as MIC_BITSTREAM1 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinConfig(
      IOMUXC_GPIO_LPSR_08_MIC_CLK,            /* GPIO_LPSR_08 PAD functional properties : */
      0x03U);                                 /* Slew Rate Field: Fast Slew Rate
                                                 Drive Strength Field: high driver
                                                 Pull / Keep Select Field: Pull Disable
                                                 Pull Up / Down Config. Field: Weak pull down
                                                 Open Drain LPSR Field: Disabled
                                                 Domain write protection: Both cores are allowed
                                                 Domain write protection lock: Neither of DWP bits is locked */
  IOMUXC_SetPinConfig(
      IOMUXC_GPIO_LPSR_09_MIC_BITSTREAM0,     /* GPIO_LPSR_09 PAD functional properties : */
      0x03U);                                 /* Slew Rate Field: Fast Slew Rate
                                                 Drive Strength Field: high driver
                                                 Pull / Keep Select Field: Pull Disable
                                                 Pull Up / Down Config. Field: Weak pull down
                                                 Open Drain LPSR Field: Disabled
                                                 Domain write protection: Both cores are allowed
                                                 Domain write protection lock: Neither of DWP bits is locked */
}


/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
BOARD_InitTestPins:
- options: {callFromInitBoot: 'false', prefix: TEST_GPIO_, coreID: cm7, enableClock: 'true'}
- pin_list:
  - {pin_num: A6, peripheral: GPIO5, signal: 'gpio_mux_io, 12', pin_signal: GPIO_DISP_B2_11, identifier: D0, direction: OUTPUT}
  - {pin_num: D9, peripheral: GPIO5, signal: 'gpio_mux_io, 11', pin_signal: GPIO_DISP_B2_10, identifier: D1, direction: OUTPUT}
  - {pin_num: B6, peripheral: GPIO5, signal: 'gpio_mux_io, 13', pin_signal: GPIO_DISP_B2_12, identifier: D2, direction: OUTPUT}
  - {pin_num: M13, peripheral: GPIO3, signal: 'gpio_mux_io, 03', pin_signal: GPIO_AD_04, identifier: D3, direction: OUTPUT}
  - {pin_num: N13, peripheral: GPIO3, signal: 'gpio_mux_io, 05', pin_signal: GPIO_AD_06, direction: OUTPUT}
  - {pin_num: P13, peripheral: GPIO3, signal: 'gpio_mux_io, 04', pin_signal: GPIO_AD_05, identifier: D5, direction: OUTPUT}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitTestPins, assigned for the Cortex-M7F core.
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void BOARD_InitTestPins(void) {
  CLOCK_EnableClock(kCLOCK_Iomuxc);           /* LPCG on: LPCG is ON. */

  /* GPIO configuration of D3 on GPIO_AD_04 (pin M13) */
  gpio_pin_config_t D3_config = {
      .direction = kGPIO_DigitalOutput,
      .outputLogic = 0U,
      .interruptMode = kGPIO_NoIntmode
  };
  /* Initialize GPIO functionality on GPIO_AD_04 (pin M13) */
  GPIO_PinInit(GPIO3, 3U, &D3_config);

  /* GPIO configuration of D5 on GPIO_AD_05 (pin P13) */
  gpio_pin_config_t D5_config = {
      .direction = kGPIO_DigitalOutput,
      .outputLogic = 0U,
      .interruptMode = kGPIO_NoIntmode
  };
  /* Initialize GPIO functionality on GPIO_AD_05 (pin P13) */
  GPIO_PinInit(GPIO3, 4U, &D5_config);

  /* GPIO configuration of D4 on GPIO_AD_06 (pin N13) */
  gpio_pin_config_t D4_config = {
      .direction = kGPIO_DigitalOutput,
      .outputLogic = 0U,
      .interruptMode = kGPIO_NoIntmode
  };
  /* Initialize GPIO functionality on GPIO_AD_06 (pin N13) */
  GPIO_PinInit(GPIO3, 5U, &D4_config);

  /* GPIO configuration of D1 on GPIO_DISP_B2_10 (pin D9) */
  gpio_pin_config_t D1_config = {
      .direction = kGPIO_DigitalOutput,
      .outputLogic = 0U,
      .interruptMode = kGPIO_NoIntmode
  };
  /* Initialize GPIO functionality on GPIO_DISP_B2_10 (pin D9) */
  GPIO_PinInit(GPIO5, 11U, &D1_config);

  /* GPIO configuration of D0 on GPIO_DISP_B2_11 (pin A6) */
  gpio_pin_config_t D0_config = {
      .direction = kGPIO_DigitalOutput,
      .outputLogic = 0U,
      .interruptMode = kGPIO_NoIntmode
  };
  /* Initialize GPIO functionality on GPIO_DISP_B2_11 (pin A6) */
  GPIO_PinInit(GPIO5, 12U, &D0_config);

  /* GPIO configuration of D2 on GPIO_DISP_B2_12 (pin B6) */
  gpio_pin_config_t D2_config = {
      .direction = kGPIO_DigitalOutput,
      .outputLogic = 0U,
      .interruptMode = kGPIO_NoIntmode
  };
  /* Initialize GPIO functionality on GPIO_DISP_B2_12 (pin B6) */
  GPIO_PinInit(GPIO5, 13U, &D2_config);

  IOMUXC_SetPinMux(
      IOMUXC_GPIO_AD_04_GPIO_MUX3_IO03,       /* GPIO_AD_04 is configured as GPIO_MUX3_IO03 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_AD_05_GPIO_MUX3_IO04,       /* GPIO_AD_05 is configured as GPIO_MUX3_IO04 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_AD_06_GPIO_MUX3_IO05,       /* GPIO_AD_06 is configured as GPIO_MUX3_IO05 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_DISP_B2_10_GPIO_MUX5_IO11,  /* GPIO_DISP_B2_10 is configured as GPIO_MUX5_IO11 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_DISP_B2_11_GPIO_MUX5_IO12,  /* GPIO_DISP_B2_11 is configured as GPIO_MUX5_IO12 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_DISP_B2_12_GPIO_MUX5_IO13,  /* GPIO_DISP_B2_12 is configured as GPIO_MUX5_IO13 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_GPR->GPR42 = ((IOMUXC_GPR->GPR42 &
    (~(BOARD_INITTESTPINS_IOMUXC_GPR_GPR42_GPIO_MUX3_GPIO_SEL_LOW_MASK))) /* Mask bits to zero which are setting */
      | IOMUXC_GPR_GPR42_GPIO_MUX3_GPIO_SEL_LOW(0x00U) /* GPIO3 and CM7_GPIO3 share same IO MUX function, GPIO_MUX3 selects one GPIO function: 0x00U */
    );
}


/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
BOARD_InitLEDPins:
- options: {callFromInitBoot: 'false', coreID: cm7, enableClock: 'true'}
- pin_list:
  - {pin_num: L14, peripheral: GPIO9, signal: 'gpio_io, 25', pin_signal: GPIO_AD_26, direction: OUTPUT, pull_up_down_config: no_init}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitLEDPins, assigned for the Cortex-M7F core.
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void BOARD_InitLEDPins(void) {
  CLOCK_EnableClock(kCLOCK_Iomuxc);           /* LPCG on: LPCG is ON. */

  /* GPIO configuration of RED_LED on GPIO_AD_26 (pin L14) */
  gpio_pin_config_t RED_LED_config = {
      .direction = kGPIO_DigitalOutput,
      .outputLogic = 0U,
      .interruptMode = kGPIO_NoIntmode
  };
  /* Initialize GPIO functionality on GPIO_AD_26 (pin L14) */
  GPIO_PinInit(GPIO9, 25U, &RED_LED_config);

  IOMUXC_SetPinMux(
      IOMUXC_GPIO_AD_26_GPIO9_IO25,           /* GPIO_AD_26 is configured as GPIO9_IO25 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
}

/***********************************************************************************************************************
 * EOF
 **********************************************************************************************************************/