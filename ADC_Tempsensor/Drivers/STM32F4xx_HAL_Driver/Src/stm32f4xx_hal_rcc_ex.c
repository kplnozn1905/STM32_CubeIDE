/**
  ******************************************************************************
  * @file    stm32f4xx_hal_rcc_ex.c
  * @author  MCD Application Team
  * @brief   Extension RCC HAL module driver.
  *          This file provides firmware functions to manage the following
  *          functionalities RCC extension peripheral:
  *           + Extended Peripheral Control functions
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/** @addtogroup STM32F4xx_HAL_Driver
  * @{
  */

/** @defgroup RCCEx RCCEx
  * @brief RCCEx HAL module driver
  * @{
  */

#ifdef HAL_RCC_MODULE_ENABLED

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/** @addtogroup RCCEx_Private_Constants
  * @{
  */
/**
  * @}
  */
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/** @defgroup RCCEx_Exported_Functions RCCEx Exported Functions
  *  @{
  */

/** @defgroup RCCEx_Exported_Functions_Group1 Extended Peripheral Control functions
 *  @brief  Extended Peripheral Control functions
 *
@verbatim
 ===============================================================================
                ##### Extended Peripheral Control functions  #####
 ===============================================================================
    [..]
    This subsection provides a set of functions allowing to control the RCC Clocks
    frequencies.
    [..]
    (@) Important note: Care must be taken when HAL_RCCEx_PeriphCLKConfig() is used to
        select the RTC clock source; in this case the Backup domain will be reset in
        order to modify the RTC Clock source, as consequence RTC registers (including
        the backup registers) and RCC_BDCR register are set to their reset values.

@endverbatim
  * @{
  */

#if defined(STM32F446xx)
/**
  * @brief  Initializes the RCC extended peripherals clocks according to the specified
  *         parameters in the RCC_PeriphCLKInitTypeDef.
  * @param  PeriphClkInit pointer to an RCC_PeriphCLKInitTypeDef structure that
  *         contains the configuration information for the Extended Peripherals
  *         clocks(I2S, SAI, LTDC RTC and TIM).
  *
  * @note   Care must be taken when HAL_RCCEx_PeriphCLKConfig() is used to select
  *         the RTC clock source; in this case the Backup domain will be reset in
  *         order to modify the RTC Clock source, as consequence RTC registers (including
  *         the backup registers) and RCC_BDCR register are set to their reset values.
  *
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_RCCEx_PeriphCLKConfig(RCC_PeriphCLKInitTypeDef  *PeriphClkInit)
{
  uint32_t tickstart = 0U;
  uint32_t tmpreg1 = 0U;
  uint32_t plli2sp = 0U;
  uint32_t plli2sq = 0U;
  uint32_t plli2sr = 0U;
  uint32_t pllsaip = 0U;
  uint32_t pllsaiq = 0U;
  uint32_t plli2sused = 0U;
  uint32_t pllsaiused = 0U;

  /* Check the peripheral clock selection parameters */
  assert_param(IS_RCC_PERIPHCLOCK(PeriphClkInit->PeriphClockSelection));

  /*------------------------ I2S APB1 configuration --------------------------*/
  if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_I2S_APB1) == (RCC_PERIPHCLK_I2S_APB1))
  {
    /* Check the parameters */
    assert_param(IS_RCC_I2SAPB1CLKSOURCE(PeriphClkInit->I2sApb1ClockSelection));

    /* Configure I2S Clock source */
    __HAL_RCC_I2S_APB1_CONFIG(PeriphClkInit->I2sApb1ClockSelection);
    /* Enable the PLLI2S when it's used as clock source for I2S */
    if(PeriphClkInit->I2sApb1ClockSelection == RCC_I2SAPB1CLKSOURCE_PLLI2S)
    {
      plli2sused = 1U;
    }
  }
  /*--------------------------------------------------------------------------*/

  /*---------------------------- I2S APB2 configuration ----------------------*/
  if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_I2S_APB2) == (RCC_PERIPHCLK_I2S_APB2))
  {
    /* Check the parameters */
    assert_param(IS_RCC_I2SAPB2CLKSOURCE(PeriphClkInit->I2sApb2ClockSelection));

    /* Configure I2S Clock source */
    __HAL_RCC_I2S_APB2_CONFIG(PeriphClkInit->I2sApb2ClockSelection);
    /* Enable the PLLI2S when it's used as clock source for I2S */
    if(PeriphClkInit->I2sApb2ClockSelection == RCC_I2SAPB2CLKSOURCE_PLLI2S)
    {
      plli2sused = 1U;
    }
  }
  /*--------------------------------------------------------------------------*/

  /*--------------------------- SAI1 configuration ---------------------------*/
  if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_SAI1) == (RCC_PERIPHCLK_SAI1))
  {
    /* Check the parameters */
    assert_param(IS_RCC_SAI1CLKSOURCE(PeriphClkInit->Sai1ClockSelection));

    /* Configure SAI1 Clock source */
    __HAL_RCC_SAI1_CONFIG(PeriphClkInit->Sai1ClockSelection);
    /* Enable the PLLI2S when it's used as clock source for SAI */
    if(PeriphClkInit->Sai1ClockSelection == RCC_SAI1CLKSOURCE_PLLI2S)
    {
      plli2sused = 1U;
    }
    /* Enable the PLLSAI when it's used as clock source for SAI */
    if(PeriphClkInit->Sai1ClockSelection == RCC_SAI1CLKSOURCE_PLLSAI)
    {
      pllsaiused = 1U;
    }
  }
  /*--------------------------------------------------------------------------*/

  /*-------------------------- SAI2 configuration ----------------------------*/
  if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_SAI2) == (RCC_PERIPHCLK_SAI2))
  {
    /* Check the parameters */
    assert_param(IS_RCC_SAI2CLKSOURCE(PeriphClkInit->Sai2ClockSelection));

    /* Configure SAI2 Clock source */
    __HAL_RCC_SAI2_CONFIG(PeriphClkInit->Sai2ClockSelection);

    /* Enable the PLLI2S when it's used as clock source for SAI */
    if(PeriphClkInit->Sai2ClockSelection == RCC_SAI2CLKSOURCE_PLLI2S)
    {
      plli2sused = 1U;
    }
    /* Enable the PLLSAI when it's used as clock source for SAI */
    if(PeriphClkInit->Sai2ClockSelection == RCC_SAI2CLKSOURCE_PLLSAI)
    {
      pllsaiused = 1U;
    }
  }
  /*--------------------------------------------------------------------------*/

  /*----------------------------- RTC configuration --------------------------*/
  if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_RTC) == (RCC_PERIPHCLK_RTC))
  {
    /* Check for RTC Parameters used to output RTCCLK */
    assert_param(IS_RCC_RTCCLKSOURCE(PeriphClkInit->RTCClockSelection));

    /* Enable Power Clock*/
    __HAL_RCC_PWR_CLK_ENABLE();

    /* Enable write access to Backup domain */
    PWR->CR |= PWR_CR_DBP;

    /* Get tick */
    tickstart = HAL_GetTick();

    while((PWR->CR & PWR_CR_DBP) == RESET)
    {
      if((HAL_GetTick() - tickstart ) > RCC_DBP_TIMEOUT_VALUE)
      {
        return HAL_TIMEOUT;
      }
    }
    /* Reset the Backup domain only if the RTC Clock source selection is modified from reset value */
    tmpreg1 = (RCC->BDCR & RCC_BDCR_RTCSEL);
    if((tmpreg1 != 0x00000000U) && ((tmpreg1) != (PeriphClkInit->RTCClockSelection & RCC_BDCR_RTCSEL)))
    {
      /* Store the content of BDCR register before the reset of Backup Domain */
      tmpreg1 = (RCC->BDCR & ~(RCC_BDCR_RTCSEL));
      /* RTC Clock selection can be changed only if the Backup Domain is reset */
      __HAL_RCC_BACKUPRESET_FORCE();
      __HAL_RCC_BACKUPRESET_RELEASE();
      /* Restore the Content of BDCR register */
      RCC->BDCR = tmpreg1;

      /* Wait for LSE reactivation if LSE was enable prior to Backup Domain reset */
      if(HAL_IS_BIT_SET(RCC->BDCR, RCC_BDCR_LSEON))
      {
        /* Get tick */
        tickstart = HAL_GetTick();

        /* Wait till LSE is ready */
        while(__HAL_RCC_GET_FLAG(RCC_FLAG_LSERDY) == RESET)
        {
          if((HAL_GetTick() - tickstart ) > RCC_LSE_TIMEOUT_VALUE)
          {
            return HAL_TIMEOUT;
          }
        }
      }
    }
    __HAL_RCC_RTC_CONFIG(PeriphClkInit->RTCClockSelection);
  }
  /*--------------------------------------------------------------------------*/

  /*---------------------------- TIM configuration ---------------------------*/
  if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_TIM) == (RCC_PERIPHCLK_TIM))
  {
    /* Configure Timer Prescaler */
    __HAL_RCC_TIMCLKPRESCALER(PeriphClkInit->TIMPresSelection);
  }
  /*--------------------------------------------------------------------------*/

  /*---------------------------- FMPI2C1 Configuration -----------------------*/
  if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_FMPI2C1) == RCC_PERIPHCLK_FMPI2C1)
  {
    /* Check the parameters */
    assert_param(IS_RCC_FMPI2C1CLKSOURCE(PeriphClkInit->Fmpi2c1ClockSelection));

    /* Configure the FMPI2C1 clock source */
    __HAL_RCC_FMPI2C1_CONFIG(PeriphClkInit->Fmpi2c1ClockSelection);
  }
  /*--------------------------------------------------------------------------*/

  /*------------------------------ CEC Configuration -------------------------*/
  if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_CEC) == RCC_PERIPHCLK_CEC)
  {
    /* Check the parameters */
    assert_param(IS_RCC_CECCLKSOURCE(PeriphClkInit->CecClockSelection));

    /* Configure the CEC clock source */
    __HAL_RCC_CEC_CONFIG(PeriphClkInit->CecClockSelection);
  }
  /*--------------------------------------------------------------------------*/

  /*----------------------------- CLK48 Configuration ------------------------*/
  if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_CLK48) == RCC_PERIPHCLK_CLK48)
  {
    /* Check the parameters */
    assert_param(IS_RCC_CLK48CLKSOURCE(PeriphClkInit->Clk48ClockSelection));

    /* Configure the CLK48 clock source */
    __HAL_RCC_CLK48_CONFIG(PeriphClkInit->Clk48ClockSelection);

    /* Enable the PLLSAI when it's used as clock source for CLK48 */
    if(PeriphClkInit->Clk48ClockSelection == RCC_CLK48CLKSOURCE_PLLSAIP)
    {
      pllsaiused = 1U;
    }
  }
  /*--------------------------------------------------------------------------*/

  /*----------------------------- SDIO Configuration -------------------------*/
  if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_SDIO) == RCC_PERIPHCLK_SDIO)
  {
    /* Check the parameters */
    assert_param(IS_RCC_SDIOCLKSOURCE(PeriphClkInit->SdioClockSelection));

    /* Configure the SDIO clock source */
    __HAL_RCC_SDIO_CONFIG(PeriphClkInit->SdioClockSelection);
  }
  /*--------------------------------------------------------------------------*/

  /*------------------------------ SPDIFRX Configuration ---------------------*/
  if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_SPDIFRX) == RCC_PERIPHCLK_SPDIFRX)
  {
    /* Check the parameters */
    assert_param(IS_RCC_SPDIFRXCLKSOURCE(PeriphClkInit->SpdifClockSelection));

    /* Configure the SPDIFRX clock source */
    __HAL_RCC_SPDIFRX_CONFIG(PeriphClkInit->SpdifClockSelection);
    /* Enable the PLLI2S when it's used as clock source for SPDIFRX */
    if(PeriphClkInit->SpdifClockSelection == RCC_SPDIFRXCLKSOURCE_PLLI2SP)
    {
      plli2sused = 1U;
    }
  }
  /*--------------------------------------------------------------------------*/

  /*---------------------------- PLLI2S Configuration ------------------------*/
  /* PLLI2S is configured when a peripheral will use it as source clock : SAI1, SAI2, I2S on APB1,
     I2S on APB2 or SPDIFRX */
  if((plli2sused == 1U) || (PeriphClkInit->PeriphClockSelection == RCC_PERIPHCLK_PLLI2S))
  {
    /* Disable the PLLI2S */
    __HAL_RCC_PLLI2S_DISABLE();
    /* Get tick */
    tickstart = HAL_GetTick();
    /* Wait till PLLI2S is disabled */
    while(__HAL_RCC_GET_FLAG(RCC_FLAG_PLLI2SRDY)  != RESET)
    {
      if((HAL_GetTick() - tickstart ) > PLLI2S_TIMEOUT_VALUE)
      {
        /* return in case of Timeout detected */
        return HAL_TIMEOUT;
      }
    }

    /* check for common PLLI2S Parameters */
    assert_param(IS_RCC_PLLI2SM_VALUE(PeriphClkInit->PLLI2S.PLLI2SM));
    assert_param(IS_RCC_PLLI2SN_VALUE(PeriphClkInit->PLLI2S.PLLI2SN));

    /*------ In Case of PLLI2S is selected as source clock for I2S -----------*/
    if(((((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_I2S_APB1) == RCC_PERIPHCLK_I2S_APB1) && (PeriphClkInit->I2sApb1ClockSelection == RCC_I2SAPB1CLKSOURCE_PLLI2S)) ||
       ((((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_I2S_APB2) == RCC_PERIPHCLK_I2S_APB2) && (PeriphClkInit->I2sApb2ClockSelection == RCC_I2SAPB2CLKSOURCE_PLLI2S)))
    {
      /* check for Parameters */
      assert_param(IS_RCC_PLLI2SR_VALUE(PeriphClkInit->PLLI2S.PLLI2SR));

      /* Read PLLI2SP/PLLI2SQ value from PLLI2SCFGR register (this value is not needed for I2S configuration) */
      plli2sp = ((((RCC->PLLI2SCFGR & RCC_PLLI2SCFGR_PLLI2SP) >> RCC_PLLI2SCFGR_PLLI2SP_Pos) + 1U) << 1U);
      plli2sq = ((RCC->PLLI2SCFGR & RCC_PLLI2SCFGR_PLLI2SQ) >> RCC_PLLI2SCFGR_PLLI2SQ_Pos);
      /* Configure the PLLI2S division factors */
      /* PLLI2S_VCO = f(VCO clock) = f(PLLI2S clock input) * (PLLI2SN/PLLI2SM) */
      /* I2SCLK = f(PLLI2S clock output) = f(VCO clock) / PLLI2SR */
      __HAL_RCC_PLLI2S_CONFIG(PeriphClkInit->PLLI2S.PLLI2SM, PeriphClkInit->PLLI2S.PLLI2SN , plli2sp, plli2sq, PeriphClkInit->PLLI2S.PLLI2SR);
    }

    /*------- In Case of PLLI2S is selected as source clock for SAI ----------*/
    if(((((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_SAI1) == RCC_PERIPHCLK_SAI1) && (PeriphClkInit->Sai1ClockSelection == RCC_SAI1CLKSOURCE_PLLI2S)) ||
       ((((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_SAI2) == RCC_PERIPHCLK_SAI2) && (PeriphClkInit->Sai2ClockSelection == RCC_SAI2CLKSOURCE_PLLI2S)))
    {
      /* Check for PLLI2S Parameters */
      assert_param(IS_RCC_PLLI2SQ_VALUE(PeriphClkInit->PLLI2S.PLLI2SQ));
      /* Check for PLLI2S/DIVQ parameters */
      assert_param(IS_RCC_PLLI2S_DIVQ_VALUE(PeriphClkInit->PLLI2SDivQ));

      /* Read PLLI2SP/PLLI2SR value from PLLI2SCFGR register (this value is not needed for SAI configuration) */
      plli2sp = ((((RCC->PLLI2SCFGR & RCC_PLLI2SCFGR_PLLI2SP) >> RCC_PLLI2SCFGR_PLLI2SP_Pos) + 1U) << 1U);
      plli2sr = ((RCC->PLLI2SCFGR & RCC_PLLI2SCFGR_PLLI2SR) >> RCC_PLLI2SCFGR_PLLI2SR_Pos);
      /* Configure the PLLI2S division factors */
      /* PLLI2S_VCO Input  = PLL_SOURCE/PLLI2SM */
      /* PLLI2S_VCO Output = PLLI2S_VCO Input * PLLI2SN */
      /* SAI_CLK(first level) = PLLI2S_VCO Output/PLLI2SQ */
      __HAL_RCC_PLLI2S_CONFIG(PeriphClkInit->PLLI2S.PLLI2SM, PeriphClkInit->PLLI2S.PLLI2SN , plli2sp, PeriphClkInit->PLLI2S.PLLI2SQ, plli2sr);

      /* SAI_CLK_x = SAI_CLK(first level)/PLLI2SDIVQ */
      __HAL_RCC_PLLI2S_PLLSAICLKDIVQ_CONFIG(PeriphClkInit->PLLI2SDivQ);
    }

    /*------ In Case of PLLI2S is selected as source clock for SPDIFRX -------*/
    if((((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_SPDIFRX) == RCC_PERIPHCLK_SPDIFRX) && (PeriphClkInit->SpdifClockSelection == RCC_SPDIFRXCLKSOURCE_PLLI2SP))
    {
      /* check for Parameters */
      assert_param(IS_RCC_PLLI2SP_VALUE(PeriphClkInit->PLLI2S.PLLI2SP));
      /* Read PLLI2SR value from PLLI2SCFGR register (this value is not need for SAI configuration) */
      plli2sq = ((((RCC->PLLI2SCFGR & RCC_PLLI2SCFGR_PLLI2SP) >> RCC_PLLI2SCFGR_PLLI2SP_Pos) + 1U) << 1U);
      plli2sr = ((RCC->PLLI2SCFGR & RCC_PLLI2SCFGR_PLLI2SR) >> RCC_PLLI2SCFGR_PLLI2SR_Pos);
      /* Configure the PLLI2S division factors */
      /* PLLI2S_VCO = f(VCO clock) = f(PLLI2S clock input) * (PLLI2SN/PLLI2SM) */
      /* SPDIFRXCLK = f(PLLI2S clock output) = f(VCO clock) / PLLI2SP */
      __HAL_RCC_PLLI2S_CONFIG(PeriphClkInit->PLLI2S.PLLI2SM, PeriphClkInit->PLLI2S.PLLI2SN , PeriphClkInit->PLLI2S.PLLI2SP, plli2sq, plli2sr);
    }

     /*----------------- In Case of PLLI2S is just selected  -----------------*/
    if((PeriphClkInit->PeriphClockSelection & RCC_PERIPHCLK_PLLI2S) == RCC_PERIPHCLK_PLLI2S)
    {
      /* Check for Parameters */
      assert_param(IS_RCC_PLLI2SP_VALUE(PeriphClkInit->PLLI2S.PLLI2SP));
      assert_param(IS_RCC_PLLI2SR_VALUE(PeriphClkInit->PLLI2S.PLLI2SR));
      assert_param(IS_RCC_PLLI2SQ_VALUE(PeriphClkInit->PLLI2S.PLLI2SQ));

      /* Configure the PLLI2S division factors */
      /* PLLI2S_VCO = f(VCO clock) = f(PLLI2S clock input) * (PLLI2SN/PLLI2SM) */
      __HAL_RCC_PLLI2S_CONFIG(PeriphClkInit->PLLI2S.PLLI2SM, PeriphClkInit->PLLI2S.PLLI2SN , PeriphClkInit->PLLI2S.PLLI2SP, PeriphClkInit->PLLI2S.PLLI2SQ, PeriphClkInit->PLLI2S.PLLI2SR);
    }

    /* Enable the PLLI2S */
    __HAL_RCC_PLLI2S_ENABLE();
    /* Get tick */
    tickstart = HAL_GetTick();
    /* Wait till PLLI2S is ready */
    while(__HAL_RCC_GET_FLAG(RCC_FLAG_PLLI2SRDY)  == RESET)
    {
      if((HAL_GetTick() - tickstart ) > PLLI2S_TIMEOUT_VALUE)
      {
        /* return in case of Timeout detected */
        return HAL_TIMEOUT;
      }
    }
  }
  /*--------------------------------------------------------------------------*/

  /*----------------------------- PLLSAI Configuration -----------------------*/
  /* PLLSAI is configured when a peripheral will use it as source clock : SAI1, SAI2, CLK48 or SDIO */
  if(pllsaiused == 1U)
  {
    /* Disable PLLSAI Clock */
    __HAL_RCC_PLLSAI_DISABLE();
    /* Get tick */
    tickstart = HAL_GetTick();
    /* Wait till PLLSAI is disabled */
    while(__HAL_RCC_PLLSAI_GET_FLAG() != RESET)
    {
      if((HAL_GetTick() - tickstart ) > PLLSAI_TIMEOUT_VALUE)
      {
        /* return in case of Timeout detected */
        return HAL_TIMEOUT;
      }
    }

    /* Check the PLLSAI division factors */
    assert_param(IS_RCC_PLLSAIM_VALUE(PeriphClkInit->PLLSAI.PLLSAIM));
    assert_param(IS_RCC_PLLSAIN_VALUE(PeriphClkInit->PLLSAI.PLLSAIN));

    /*------ In Case of PLLSAI is selected as source clock for SAI -----------*/
    if(((((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_SAI1) == RCC_PERIPHCLK_SAI1) && (PeriphClkInit->Sai1ClockSelection == RCC_SAI1CLKSOURCE_PLLSAI)) ||
       ((((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_SAI2) == RCC_PERIPHCLK_SAI2) && (PeriphClkInit->Sai2ClockSelection == RCC_SAI2CLKSOURCE_PLLSAI)))
    {
      /* check for PLLSAIQ Parameter */
      assert_param(IS_RCC_PLLSAIQ_VALUE(PeriphClkInit->PLLSAI.PLLSAIQ));
      /* check for PLLSAI/DIVQ Parameter */
      assert_param(IS_RCC_PLLSAI_DIVQ_VALUE(PeriphClkInit->PLLSAIDivQ));

      /* Read PLLSAIP value from PLLSAICFGR register (this value is not needed for SAI configuration) */
      pllsaip = ((((RCC->PLLSAICFGR & RCC_PLLSAICFGR_PLLSAIP) >> RCC_PLLSAICFGR_PLLSAIP_Pos) + 1U) << 1U);
      /* PLLSAI_VCO Input  = PLL_SOURCE/PLLM */
      /* PLLSAI_VCO Output = PLLSAI_VCO Input * PLLSAIN */
      /* SAI_CLK(first level) = PLLSAI_VCO Output/PLLSAIQ */
      __HAL_RCC_PLLSAI_CONFIG(PeriphClkInit->PLLSAI.PLLSAIM, PeriphClkInit->PLLSAI.PLLSAIN , pllsaip, PeriphClkInit->PLLSAI.PLLSAIQ, 0U);

      /* SAI_CLK_x = SAI_CLK(first level)/PLLSAIDIVQ */
      __HAL_RCC_PLLSAI_PLLSAICLKDIVQ_CONFIG(PeriphClkInit->PLLSAIDivQ);
    }

    /*------ In Case of PLLSAI is selected as source clock for CLK48 ---------*/
    /* In Case of PLLI2S is selected as source clock for CLK48 */
    if((((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_CLK48) == RCC_PERIPHCLK_CLK48) && (PeriphClkInit->Clk48ClockSelection == RCC_CLK48CLKSOURCE_PLLSAIP))
    {
      /* check for Parameters */
      assert_param(IS_RCC_PLLSAIP_VALUE(PeriphClkInit->PLLSAI.PLLSAIP));
      /* Read PLLSAIQ value from PLLI2SCFGR register (this value is not need for SAI configuration) */
      pllsaiq = ((RCC->PLLSAICFGR & RCC_PLLSAICFGR_PLLSAIQ) >> RCC_PLLSAICFGR_PLLSAIQ_Pos);
      /* Configure the PLLSAI division factors */
      /* PLLSAI_VCO = f(VCO clock) = f(PLLSAI clock input) * (PLLI2SN/PLLSAIM) */
      /* 48CLK = f(PLLSAI clock output) = f(VCO clock) / PLLSAIP */
      __HAL_RCC_PLLSAI_CONFIG(PeriphClkInit->PLLSAI.PLLSAIM, PeriphClkInit->PLLSAI.PLLSAIN , PeriphClkInit->PLLSAI.PLLSAIP, pllsaiq, 0U);
    }

    /* Enable PLLSAI Clock */
    __HAL_RCC_PLLSAI_ENABLE();
    /* Get tick */
    tickstart = HAL_GetTick();
    /* Wait till PLLSAI is ready */
    while(__HAL_RCC_PLLSAI_GET_FLAG() == RESET)
    {
      if((HAL_GetTick() - tickstart ) > PLLSAI_TIMEOUT_VALUE)
      {
        /* return in case of Timeout detected */
        return HAL_TIMEOUT;
      }
    }
  }
  return HAL_OK;
}

/**
  * @brief  Get the RCC_PeriphCLKInitTypeDef according to the internal
  *         RCC configuration registers.
  * @param  PeriphClkInit pointer to an RCC_PeriphCLKInitTypeDef structure that
  *         will be configured.
  * @retval None
  */
void HAL_RCCEx_GetPeriphCLKConfig(RCC_PeriphCLKInitTypeDef  *PeriphClkInit)
{
  uint32_t tempreg;

  /* Set all possible values for the extended clock type parameter------------*/
  PeriphClkInit->PeriphClockSelection = RCC_PERIPHCLK_I2S_APB1 | RCC_PERIPHCLK_I2S_APB2 |\
                                        RCC_PERIPHCLK_SAI1     | RCC_PERIPHCLK_SAI2     |\
                                        RCC_PERIPHCLK_TIM      | RCC_PERIPHCLK_RTC      |\
                                        RCC_PERIPHCLK_CEC      | RCC_PERIPHCLK_FMPI2C1  |\
                                        RCC_PERIPHCLK_CLK48     | RCC_PERIPHCLK_SDIO     |\
                                        RCC_PERIPHCLK_SPDIFRX;

  /* Get the PLLI2S Clock configuration --------------------------------------*/
  PeriphClkInit->PLLI2S.PLLI2SM = (uint32_t)((RCC->PLLI2SCFGR & RCC_PLLI2SCFGR_PLLI2SM) >> RCC_PLLI2SCFGR_PLLI2SM_Pos);
  PeriphClkInit->PLLI2S.PLLI2SN = (uint32_t)((RCC->PLLI2SCFGR & RCC_PLLI2SCFGR_PLLI2SN) >> RCC_PLLI2SCFGR_PLLI2SN_Pos);
  PeriphClkInit->PLLI2S.PLLI2SP = (uint32_t)((((RCC->PLLI2SCFGR & RCC_PLLI2SCFGR_PLLI2SP) >> RCC_PLLI2SCFGR_PLLI2SP_Pos) + 1U) << 1U);
  PeriphClkInit->PLLI2S.PLLI2SQ = (uint32_t)((RCC->PLLI2SCFGR & RCC_PLLI2SCFGR_PLLI2SQ) >> RCC_PLLI2SCFGR_PLLI2SQ_Pos);
  PeriphClkInit->PLLI2S.PLLI2SR = (uint32_t)((RCC->PLLI2SCFGR & RCC_PLLI2SCFGR_PLLI2SR) >> RCC_PLLI2SCFGR_PLLI2SR_Pos);
  /* Get the PLLSAI Clock configuration --------------------------------------*/
  PeriphClkInit->PLLSAI.PLLSAIM = (uint32_t)((RCC->PLLSAICFGR & RCC_PLLSAICFGR_PLLSAIM) >> RCC_PLLSAICFGR_PLLSAIM_Pos);
  PeriphClkInit->PLLSAI.PLLSAIN = (uint32_t)((RCC->PLLSAICFGR & RCC_PLLSAICFGR_PLLSAIN) >> RCC_PLLSAICFGR_PLLSAIN_Pos);
  PeriphClkInit->PLLSAI.PLLSAIP = (uint32_t)((((RCC->PLLSAICFGR & RCC_PLLSAICFGR_PLLSAIP) >> RCC_PLLSAICFGR_PLLSAIP_Pos) + 1U) << 1U);
  PeriphClkInit->PLLSAI.PLLSAIQ = (uint32_t)((RCC->PLLSAICFGR & RCC_PLLSAICFGR_PLLSAIQ) >> RCC_PLLSAICFGR_PLLSAIQ_Pos);
  /* Get the PLLSAI/PLLI2S division factors ----------------------------------*/
  PeriphClkInit->PLLI2SDivQ = (uint32_t)((RCC->DCKCFGR & RCC_DCKCFGR_PLLI2SDIVQ) >> RCC_DCKCFGR_PLLI2SDIVQ_Pos);
  PeriphClkInit->PLLSAIDivQ = (uint32_t)((RCC->DCKCFGR & RCC_DCKCFGR_PLLSAIDIVQ) >> RCC_DCKCFGR_PLLSAIDIVQ_Pos);

  /* Get the SAI1 clock configuration ----------------------------------------*/
  PeriphClkInit->Sai1ClockSelection = __HAL_RCC_GET_SAI1_SOURCE();

  /* Get the SAI2 clock configuration ----------------------------------------*/
  PeriphClkInit->Sai2ClockSelection = __HAL_RCC_GET_SAI2_SOURCE();

  /* Get the I2S APB1 clock configuration ------------------------------------*/
  PeriphClkInit->I2sApb1ClockSelection = __HAL_RCC_GET_I2S_APB1_SOURCE();

  /* Get the I2S APB2 clock configuration ------------------------------------*/
  PeriphClkInit->I2sApb2ClockSelection = __HAL_RCC_GET_I2S_APB2_SOURCE();

  /* Get the RTC Clock configuration -----------------------------------------*/
  tempreg = (RCC->CFGR & RCC_CFGR_RTCPRE);
  PeriphClkInit->RTCClockSelection = (uint32_t)((tempreg) | (RCC->BDCR & RCC_BDCR_RTCSEL));

  /* Get the CEC clock configuration -----------------------------------------*/
  PeriphClkInit->CecClockSelection = __HAL_RCC_GET_CEC_SOURCE();

  /* Get the FMPI2C1 clock configuration -------------------------------------*/
  PeriphClkInit->Fmpi2c1ClockSelection = __HAL_RCC_GET_FMPI2C1_SOURCE();

  /* Get the CLK48 clock configuration ----------------------------------------*/
  PeriphClkInit->Clk48ClockSelection = __HAL_RCC_GET_CLK48_SOURCE();

  /* Get the SDIO clock configuration ----------------------------------------*/
  PeriphClkInit->SdioClockSelection = __HAL_RCC_GET_SDIO_SOURCE();

  /* Get the SPDIFRX clock configuration -------------------------------------*/
  PeriphClkInit->SpdifClockSelection = __HAL_RCC_GET_SPDIFRX_SOURCE();

  /* Get the TIM Prescaler configuration -------------------------------------*/
  if ((RCC->DCKCFGR & RCC_DCKCFGR_TIMPRE) == RESET)
  {
    PeriphClkInit->TIMPresSelection = RCC_TIMPRES_DESACTIVATED;
  }
  else
  {
    PeriphClkInit->TIMPresSelection = RCC_TIMPRES_ACTIVATED;
  }
}

/**
  * @brief  Return the peripheral clock frequency for a given peripheral(SAI..)
  * @note   Return 0 if peripheral clock identifier not managed by this API
  * @param  PeriphClk Peripheral clock identifier
  *         This parameter can be one of the following values:
  *            @arg RCC_PERIPHCLK_SAI1: SAI1 peripheral clock
  *            @arg RCC_PERIPHCLK_SAI2: SAI2 peripheral clock
  *            @arg RCC_PERIPHCLK_I2S_APB1: I2S APB1 peripheral clock
  *            @arg RCC_PERIPHCLK_I2S_APB2: I2S APB2 peripheral clock
  * @retval Frequency in KHz
  */
uint32_t HAL_RCCEx_GetPeriphCLKFreq(uint32_t PeriphClk)
{
  uint32_t tmpreg1 = 0U;
  /* This variable used to store the SAI clock frequency (value in Hz) */
  uint32_t frequency = 0U;
  /* This variable used to store the VCO Input (value in Hz) */
  uint32_t vcoinput = 0U;
  /* This variable used to store the SAI clock source */
  uint32_t saiclocksource = 0U;
  uint32_t srcclk = 0U;
  /* This variable used to store the VCO Output (value in Hz) */
  uint32_t vcooutput = 0U;
  switch (PeriphClk)
  {
  case RCC_PERIPHCLK_SAI1:
  case RCC_PERIPHCLK_SAI2:
    {
      saiclocksource = RCC->DCKCFGR;
      saiclocksource &= (RCC_DCKCFGR_SAI1SRC | RCC_DCKCFGR_SAI2SRC);
      switch (saiclocksource)
      {
      case 0U: /* PLLSAI is the clock source for SAI*/
        {
          /* Configure the PLLSAI division factor */
          /* PLLSAI_VCO Input  = PLL_SOURCE/PLLSAIM */
          if((RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) == RCC_PLLSOURCE_HSI)
          {
            /* In Case the PLL Source is HSI (Internal Clock) */
            vcoinput = (HSI_VALUE / (uint32_t)(RCC->PLLSAICFGR & RCC_PLLSAICFGR_PLLSAIM));
          }
          else
          {
            /* In Case the PLL Source is HSE (External Clock) */
            vcoinput = ((HSE_VALUE / (uint32_t)(RCC->PLLSAICFGR & RCC_PLLSAICFGR_PLLSAIM)));
          }
          /* PLLSAI_VCO Output = PLLSAI_VCO Input * PLLSAIN */
          /* SAI_CLK(first level) = PLLSAI_VCO Output/PLLSAIQ */
          tmpreg1 = (RCC->PLLSAICFGR & RCC_PLLSAICFGR_PLLSAIQ) >> 24U;
          frequency = (vcoinput * ((RCC->PLLSAICFGR & RCC_PLLSAICFGR_PLLSAIN) >> 6U))/(tmpreg1);

          /* SAI_CLK_x = SAI_CLK(first level)/PLLSAIDIVQ */
          tmpreg1 = (((RCC->DCKCFGR & RCC_DCKCFGR_PLLSAIDIVQ) >> 8U) + 1U);
          frequency = frequency/(tmpreg1);
          break;
        }
      case RCC_DCKCFGR_SAI1SRC_0: /* PLLI2S is the clock source for SAI*/
      case RCC_DCKCFGR_SAI2SRC_0: /* PLLI2S is the clock source for SAI*/
        {
          /* Configure the PLLI2S division factor */
          /* PLLI2S_VCO Input  = PLL_SOURCE/PLLI2SM */
          if((RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) == RCC_PLLSOURCE_HSI)
          {
            /* In Case the PLL Source is HSI (Internal Clock) */
            vcoinput = (HSI_VALUE / (uint32_t)(RCC->PLLI2SCFGR & RCC_PLLI2SCFGR_PLLI2SM));
          }
          else
          {
            /* In Case the PLL Source is HSE (External Clock) */
            vcoinput = ((HSE_VALUE / (uint32_t)(RCC->PLLI2SCFGR & RCC_PLLI2SCFGR_PLLI2SM)));
          }

          /* PLLI2S_VCO Output = PLLI2S_VCO Input * PLLI2SN */
          /* SAI_CLK(first level) = PLLI2S_VCO Output/PLLI2SQ */
          tmpreg1 = (RCC->PLLI2SCFGR & RCC_PLLI2SCFGR_PLLI2SQ) >> 24U;
          frequency = (vcoinput * ((RCC->PLLI2SCFGR & RCC_PLLI2SCFGR_PLLI2SN) >> 6U))/(tmpreg1);

          /* SAI_CLK_x = SAI_CLK(first level)/PLLI2SDIVQ */
          tmpreg1 = ((RCC->DCKCFGR & RCC_DCKCFGR_PLLI2SDIVQ) + 1U);
          frequency = frequency/(tmpreg1);
          break;
        }
      case RCC_DCKCFGR_SAI1SRC_1: /* PLLR is the clock source for SAI*/
      case RCC_DCKCFGR_SAI2SRC_1: /* PLLR is the clock source for SAI*/
        {
          /* Configure the PLLI2S division factor */
          /* PLL_VCO Input  = PLL_SOURCE/PLLM */
          if((RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) == RCC_PLLSOURCE_HSI)
          {
            /* In Case the PLL Source is HSI (Internal Clock) */
            vcoinput = (HSI_VALUE / (uint32_t)(RCC->PLLCFGR & RCC_PLLCFGR_PLLM));
          }
          else
          {
            /* In Case the PLL Source is HSE (External Clock) */
            vcoinput = ((HSE_VALUE / (uint32_t)(RCC->PLLCFGR & RCC_PLLCFGR_PLLM)));
          }

          /* PLL_VCO Output = PLL_VCO Input * PLLN */
          /* SAI_CLK_x = PLL_VCO Output/PLLR */
          tmpreg1 = (RCC->PLLCFGR & RCC_PLLCFGR_PLLR) >> 28U;
          frequency = (vcoinput * ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6U))/(tmpreg1);
          break;
        }
      case RCC_DCKCFGR_SAI1SRC: /* External clock is the clock source for SAI*/
        {
          frequency = EXTERNAL_CLOCK_VALUE;
          break;
        }
      case RCC_DCKCFGR_SAI2SRC: /* PLLSRC(HSE or HSI) is the clock source for SAI*/
        {
          if((RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) == RCC_PLLSOURCE_HSI)
          {
            /* In Case the PLL Source is HSI (Internal Clock) */
            frequency = (uint32_t)(HSI_VALUE);
          }
          else
          {
            /* In Case the PLL Source is HSE (External Clock) */
            frequency = (uint32_t)(HSE_VALUE);
          }
          break;
        }
      default :
        {
          break;
        }
      }
      break;
    }
  case RCC_PERIPHCLK_I2S_APB1:
    {
      /* Get the current I2S source */
      srcclk = __HAL_RCC_GET_I2S_APB1_SOURCE();
      switch (srcclk)
      {
      /* Check if I2S clock selection is External clock mapped on the I2S_CKIN pin used as I2S clock */
      case RCC_I2SAPB1CLKSOURCE_EXT:
        {
          /* Set the I2S clock to the external clock  value */
          frequency = EXTERNAL_CLOCK_VALUE;
          break;
        }
      /* Check if I2S clock selection is PLLI2S VCO output clock divided by PLLI2SR used as I2S clock */
      case RCC_I2SAPB1CLKSOURCE_PLLI2S:
        {
          /* Configure the PLLI2S division factor */
          /* PLLI2S_VCO Input  = PLL_SOURCE/PLLI2SM */
          if((RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) == RCC_PLLSOURCE_HSE)
          {
            /* Get the I2S source clock value */
            vcoinput = (uint32_t)(HSE_VALUE / (uint32_t)(RCC->PLLI2SCFGR & RCC_PLLI2SCFGR_PLLI2SM));
          }
          else
          {
            /* Get the I2S source clock value */
            vcoinput = (uint32_t)(HSI_VALUE / (uint32_t)(RCC->PLLI2SCFGR & RCC_PLLI2SCFGR_PLLI2SM));
          }

          /* PLLI2S_VCO Output = PLLI2S_VCO Input * PLLI2SN */
          vcooutput = (uint32_t)(vcoinput * (((RCC->PLLI2SCFGR & RCC_PLLI2SCFGR_PLLI2SN) >> 6U) & (RCC_PLLI2SCFGR_PLLI2SN >> 6U)));
          /* I2S_CLK = PLLI2S_VCO Output/PLLI2SR */
          frequency = (uint32_t)(vcooutput /(((RCC->PLLI2SCFGR & RCC_PLLI2SCFGR_PLLI2SR) >> 28U) & (RCC_PLLI2SCFGR_PLLI2SR >> 28U)));
          break;
        }
      /* Check if I2S clock selection is PLL VCO Output divided by PLLR used as I2S clock */
      case RCC_I2SAPB1CLKSOURCE_PLLR:
        {
          /* Configure the PLL division factor R */
          /* PLL_VCO Input  = PLL_SOURCE/PLLM */
          if((RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) == RCC_PLLSOURCE_HSE)
          {
            /* Get the I2S source clock value */
            vcoinput = (uint32_t)(HSE_VALUE / (uint32_t)(RCC->PLLCFGR & RCC_PLLCFGR_PLLM));
          }
          else
          {
            /* Get the I2S source clock value */
            vcoinput = (uint32_t)(HSI_VALUE / (uint32_t)(RCC->PLLCFGR & RCC_PLLCFGR_PLLM));
          }

          /* PLL_VCO Output = PLL_VCO Input * PLLN */
          vcooutput = (uint32_t)(vcoinput * (((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6U) & (RCC_PLLCFGR_PLLN >> 6U)));
          /* I2S_CLK = PLL_VCO Output/PLLR */
          frequency = (uint32_t)(vcooutput /(((RCC->PLLCFGR & RCC_PLLCFGR_PLLR) >> 28U) & (RCC_PLLCFGR_PLLR >> 28U)));
          break;
        }
      /* Check if I2S clock selection is HSI or HSE depending from PLL source Clock */
      case RCC_I2SAPB1CLKSOURCE_PLLSRC:
        {
          if((RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) == RCC_PLLSOURCE_HSE)
          {
            frequency = HSE_VALUE;
          }
          else
          {
            frequency = HSI_VALUE;
          }
          break;
        }
        /* Clock not enabled for I2S*/
      default:
        {
          frequency = 0U;
          break;
        }
      }
      break;
    }
  case RCC_PERIPHCLK_I2S_APB2:
    {
      /* Get the current I2S source */
      srcclk = __HAL_RCC_GET_I2S_APB2_SOURCE();
      switch (srcclk)
      {
        /* Check if I2S clock selection is External clock mapped on the I2S_CKIN pin used as I2S clock */
      case RCC_I2SAPB2CLKSOURCE_EXT:
        {
          /* Set the I2S clock to the external clock  value */
          frequency = EXTERNAL_CLOCK_VALUE;
          break;
        }
        /* Check if I2S clock selection is PLLI2S VCO output clock divided by PLLI2SR used as I2S clock */
      case RCC_I2SAPB2CLKSOURCE_PLLI2S:
        {
          /* Configure the PLLI2S division factor */
          /* PLLI2S_VCO Input  = PLL_SOURCE/PLLI2SM */
          if((RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) == RCC_PLLSOURCE_HSE)
          {
            /* Get the I2S source clock value */
            vcoinput = (uint32_t)(HSE_VALUE / (uint32_t)(RCC->PLLI2SCFGR & RCC_PLLI2SCFGR_PLLI2SM));
          }
          else
          {
            /* Get the I2S source clock value */
            vcoinput = (uint32_t)(HSI_VALUE / (uint32_t)(RCC->PLLI2SCFGR & RCC_PLLI2SCFGR_PLLI2SM));
          }

          /* PLLI2S_VCO Output = PLLI2S_VCO Input * PLLI2SN */
          vcooutput = (uint32_t)(vcoinput * (((RCC->PLLI2SCFGR & RCC_PLLI2SCFGR_PLLI2SN) >> 6U) & (RCC_PLLI2SCFGR_PLLI2SN >> 6U)));
          /* I2S_CLK = PLLI2S_VCO Output/PLLI2SR */
          frequency = (uint32_t)(vcooutput /(((RCC->PLLI2SCFGR & RCC_PLLI2SCFGR_PLLI2SR) >> 28U) & (RCC_PLLI2SCFGR_PLLI2SR >> 28U)));
          break;
        }
        /* Check if I2S clock selection is PLL VCO Output divided by PLLR used as I2S clock */
      case RCC_I2SAPB2CLKSOURCE_PLLR:
        {
          /* Configure the PLL division factor R */
          /* PLL_VCO Input  = PLL_SOURCE/PLLM */
          if((RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) == RCC_PLLSOURCE_HSE)
          {
            /* Get the I2S source clock value */
            vcoinput = (uint32_t)(HSE_VALUE / (uint32_t)(RCC->PLLCFGR & RCC_PLLCFGR_PLLM));
          }
          else
          {
            /* Get the I2S source clock value */
            vcoinput = (uint32_t)(HSI_VALUE / (uint32_t)(RCC->PLLCFGR & RCC_PLLCFGR_PLLM));
          }

          /* PLL_VCO Output = PLL_VCO Input * PLLN */
          vcooutput = (uint32_t)(vcoinput * (((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6U) & (RCC_PLLCFGR_PLLN >> 6U)));
          /* I2S_CLK = PLL_VCO Output/PLLR */
          frequency = (uint32_t)(vcooutput /(((RCC->PLLCFGR & RCC_PLLCFGR_PLLR) >> 28U) & (RCC_PLLCFGR_PLLR >> 28U)));
          break;
        }
        /* Check if I2S clock selection is HSI or HSE depending from PLL source Clock */
      case RCC_I2SAPB2CLKSOURCE_PLLSRC:
        {
          if((RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) == RCC_PLLSOURCE_HSE)
          {
            frequency = HSE_VALUE;
          }
          else
          {
            frequency = HSI_VALUE;
          }
          break;
        }
        /* Clock not enabled for I2S*/
      default:
        {
          frequency = 0U;
          break;
        }
      }
      break;
    }
  }
  return frequency;
}
#endif /* STM32F446xx */

#if defined(STM32F469xx) || defined(STM32F479xx)
/**
  * @brief  Initializes the RCC extended peripherals clocks according to the specified
  *         parameters in the RCC_PeriphCLKInitTypeDef.
  * @param  PeriphClkInit pointer to an RCC_PeriphCLKInitTypeDef structure that
  *         contains the configuration information for the Extended Peripherals
  *         clocks(I2S, SAI, LTDC, RTC and TIM).
  *
  * @note   Care must be taken when HAL_RCCEx_PeriphCLKConfig() is used to select
  *         the RTC clock source; in this case the Backup domain will be reset in
  *         order to modify the RTC Clock source, as consequence RTC registers (including
  *         the backup registers) and RCC_BDCR register are set to their reset values.
  *
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_RCCEx_PeriphCLKConfig(RCC_PeriphCLKInitTypeDef  *PeriphClkInit)
{
  uint32_t tickstart = 0U;
  uint32_t tmpreg1 = 0U;
  uint32_t pllsaip = 0U;
  uint32_t pllsaiq = 0U;
  uint32_t pllsair = 0U;

  /* Check the parameters */
  assert_param(IS_RCC_PERIPHCLOCK(PeriphClkInit->PeriphClockSelection));

  /*--------------------------- CLK48 Configuration --------------------------*/
  if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_CLK48) == RCC_PERIPHCLK_CLK48)
  {
    /* Check the parameters */
    assert_param(IS_RCC_CLK48CLKSOURCE(PeriphClkInit->Clk48ClockSelection));

    /* Configure the CLK48 clock source */
    __HAL_RCC_CLK48_CONFIG(PeriphClkInit->Clk48ClockSelection);
  }
  /*--------------------------------------------------------------------------*/

  /*------------------------------ SDIO Configuration ------------------------*/
  if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_SDIO) == RCC_PERIPHCLK_SDIO)
  {
    /* Check the parameters */
    assert_param(IS_RCC_SDIOCLKSOURCE(PeriphClkInit->SdioClockSelection));

    /* Configure the SDIO clock source */
    __HAL_RCC_SDIO_CONFIG(PeriphClkInit->SdioClockSelection);
  }
  /*--------------------------------------------------------------------------*/

  /*----------------------- SAI/I2S Configuration (PLLI2S) -------------------*/
  /*------------------- Common configuration SAI/I2S -------------------------*/
  /* In Case of SAI or I2S Clock Configuration through PLLI2S, PLLI2SN division
     factor is common parameters for both peripherals */
  if((((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_I2S) == RCC_PERIPHCLK_I2S) ||
     (((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_SAI_PLLI2S) == RCC_PERIPHCLK_SAI_PLLI2S) ||
     (((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_PLLI2S) == RCC_PERIPHCLK_PLLI2S))
  {
    /* check for Parameters */
    assert_param(IS_RCC_PLLI2SN_VALUE(PeriphClkInit->PLLI2S.PLLI2SN));

    /* Disable the PLLI2S */
    __HAL_RCC_PLLI2S_DISABLE();
    /* Get tick */
    tickstart = HAL_GetTick();
    /* Wait till PLLI2S is disabled */
    while(__HAL_RCC_GET_FLAG(RCC_FLAG_PLLI2SRDY)  != RESET)
    {
      if((HAL_GetTick() - tickstart ) > PLLI2S_TIMEOUT_VALUE)
      {
        /* return in case of Timeout detected */
        return HAL_TIMEOUT;
      }
    }

    /*---------------------- I2S configuration -------------------------------*/
    /* In Case of I2S Clock Configuration through PLLI2S, PLLI2SR must be added
      only for I2S configuration */
    if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_I2S) == (RCC_PERIPHCLK_I2S))
    {
      /* check for Parameters */
      assert_param(IS_RCC_PLLI2SR_VALUE(PeriphClkInit->PLLI2S.PLLI2SR));
      /* Configure the PLLI2S division factors */
      /* PLLI2S_VCO = f(VCO clock) = f(PLLI2S clock input) x (PLLI2SN/PLLM) */
      /* I2SCLK = f(PLLI2S clock output) = f(VCO clock) / PLLI2SR */
      __HAL_RCC_PLLI2S_CONFIG(PeriphClkInit->PLLI2S.PLLI2SN , PeriphClkInit->PLLI2S.PLLI2SR);
    }

    /*---------------------------- SAI configuration -------------------------*/
    /* In Case of SAI Clock Configuration through PLLI2S, PLLI2SQ and PLLI2S_DIVQ must
       be added only for SAI configuration */
    if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_SAI_PLLI2S) == (RCC_PERIPHCLK_SAI_PLLI2S))
    {
      /* Check the PLLI2S division factors */
      assert_param(IS_RCC_PLLI2SQ_VALUE(PeriphClkInit->PLLI2S.PLLI2SQ));
      assert_param(IS_RCC_PLLI2S_DIVQ_VALUE(PeriphClkInit->PLLI2SDivQ));

      /* Read PLLI2SR value from PLLI2SCFGR register (this value is not need for SAI configuration) */
      tmpreg1 = ((RCC->PLLI2SCFGR & RCC_PLLI2SCFGR_PLLI2SR) >> RCC_PLLI2SCFGR_PLLI2SR_Pos);
      /* Configure the PLLI2S division factors */
      /* PLLI2S_VCO Input  = PLL_SOURCE/PLLM */
      /* PLLI2S_VCO Output = PLLI2S_VCO Input * PLLI2SN */
      /* SAI_CLK(first level) = PLLI2S_VCO Output/PLLI2SQ */
      __HAL_RCC_PLLI2S_SAICLK_CONFIG(PeriphClkInit->PLLI2S.PLLI2SN , PeriphClkInit->PLLI2S.PLLI2SQ , tmpreg1);
      /* SAI_CLK_x = SAI_CLK(first level)/PLLI2SDIVQ */
      __HAL_RCC_PLLI2S_PLLSAICLKDIVQ_CONFIG(PeriphClkInit->PLLI2SDivQ);
    }

    /*----------------- In Case of PLLI2S is just selected  -----------------*/
    if((PeriphClkInit->PeriphClockSelection & RCC_PERIPHCLK_PLLI2S) == RCC_PERIPHCLK_PLLI2S)
    {
      /* Check for Parameters */
      assert_param(IS_RCC_PLLI2SQ_VALUE(PeriphClkInit->PLLI2S.PLLI2SQ));
      assert_param(IS_RCC_PLLI2SR_VALUE(PeriphClkInit->PLLI2S.PLLI2SR));

      /* Configure the PLLI2S multiplication and division factors */
      __HAL_RCC_PLLI2S_SAICLK_CONFIG(PeriphClkInit->PLLI2S.PLLI2SN, PeriphClkInit->PLLI2S.PLLI2SQ, PeriphClkInit->PLLI2S.PLLI2SR);
    }

    /* Enable the PLLI2S */
    __HAL_RCC_PLLI2S_ENABLE();
    /* Get tick */
    tickstart = HAL_GetTick();
    /* Wait till PLLI2S is ready */
    while(__HAL_RCC_GET_FLAG(RCC_FLAG_PLLI2SRDY)  == RESET)
    {
      if((HAL_GetTick() - tickstart ) > PLLI2S_TIMEOUT_VALUE)
      {
        /* return in case of Timeout detected */
        return HAL_TIMEOUT;
      }
    }
  }
  /*--------------------------------------------------------------------------*/

  /*----------------------- SAI/LTDC Configuration (PLLSAI) ------------------*/
  /*----------------------- Common configuration SAI/LTDC --------------------*/
  /* In Case of SAI, LTDC or CLK48 Clock Configuration through PLLSAI, PLLSAIN division
     factor is common parameters for these peripherals */
  if((((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_SAI_PLLSAI) == RCC_PERIPHCLK_SAI_PLLSAI) ||
     (((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_LTDC) == RCC_PERIPHCLK_LTDC)             ||
     ((((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_CLK48) == RCC_PERIPHCLK_CLK48)          &&
      (PeriphClkInit->Clk48ClockSelection == RCC_CLK48CLKSOURCE_PLLSAIP)))
  {
    /* Check the PLLSAI division factors */
    assert_param(IS_RCC_PLLSAIN_VALUE(PeriphClkInit->PLLSAI.PLLSAIN));

    /* Disable PLLSAI Clock */
    __HAL_RCC_PLLSAI_DISABLE();
    /* Get tick */
    tickstart = HAL_GetTick();
    /* Wait till PLLSAI is disabled */
    while(__HAL_RCC_PLLSAI_GET_FLAG() != RESET)
    {
      if((HAL_GetTick() - tickstart ) > PLLSAI_TIMEOUT_VALUE)
      {
        /* return in case of Timeout detected */
        return HAL_TIMEOUT;
      }
    }

    /*---------------------------- SAI configuration -------------------------*/
    /* In Case of SAI Clock Configuration through PLLSAI, PLLSAIQ and PLLSAI_DIVQ must
       be added only for SAI configuration */
    if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_SAI_PLLSAI) == (RCC_PERIPHCLK_SAI_PLLSAI))
    {
      assert_param(IS_RCC_PLLSAIQ_VALUE(PeriphClkInit->PLLSAI.PLLSAIQ));
      assert_param(IS_RCC_PLLSAI_DIVQ_VALUE(PeriphClkInit->PLLSAIDivQ));

      /* Read PLLSAIP value from PLLSAICFGR register (this value is not needed for SAI configuration) */
      pllsaip = ((((RCC->PLLSAICFGR & RCC_PLLSAICFGR_PLLSAIP) >> RCC_PLLSAICFGR_PLLSAIP_Pos) + 1U) << 1U);
      /* Read PLLSAIR value from PLLSAICFGR register (this value is not need for SAI configuration) */
      pllsair = ((RCC->PLLSAICFGR & RCC_PLLSAICFGR_PLLSAIR) >> RCC_PLLSAICFGR_PLLSAIR_Pos);
      /* PLLSAI_VCO Input  = PLL_SOURCE/PLLM */
      /* PLLSAI_VCO Output = PLLSAI_VCO Input * PLLSAIN */
      /* SAI_CLK(first level) = PLLSAI_VCO Output/PLLSAIQ */
      __HAL_RCC_PLLSAI_CONFIG(PeriphClkInit->PLLSAI.PLLSAIN, pllsaip, PeriphClkInit->PLLSAI.PLLSAIQ, pllsair);
      /* SAI_CLK_x = SAI_CLK(first level)/PLLSAIDIVQ */
      __HAL_RCC_PLLSAI_PLLSAICLKDIVQ_CONFIG(PeriphClkInit->PLLSAIDivQ);
    }

    /*---------------------------- LTDC configuration ------------------------*/
    if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_LTDC) == (RCC_PERIPHCLK_LTDC))
    {
      assert_param(IS_RCC_PLLSAIR_VALUE(PeriphClkInit->PLLSAI.PLLSAIR));
      assert_param(IS_RCC_PLLSAI_DIVR_VALUE(PeriphClkInit->PLLSAIDivR));

      /* Read PLLSAIP value from PLLSAICFGR register (this value is not needed for SAI configuration) */
      pllsaip = ((((RCC->PLLSAICFGR & RCC_PLLSAICFGR_PLLSAIP) >> RCC_PLLSAICFGR_PLLSAIP_Pos) + 1U) << 1U);
      /* Read PLLSAIQ value from PLLSAICFGR register (this value is not need for SAI configuration) */
      pllsaiq = ((RCC->PLLSAICFGR & RCC_PLLSAICFGR_PLLSAIQ) >> RCC_PLLSAICFGR_PLLSAIQ_Pos);
      /* PLLSAI_VCO Input  = PLL_SOURCE/PLLM */
      /* PLLSAI_VCO Output = PLLSAI_VCO Input * PLLSAIN */
      /* LTDC_CLK(first level) = PLLSAI_VCO Output/PLLSAIR */
      __HAL_RCC_PLLSAI_CONFIG(PeriphClkInit->PLLSAI.PLLSAIN, pllsaip, pllsaiq, PeriphClkInit->PLLSAI.PLLSAIR);
      /* LTDC_CLK = LTDC_CLK(first level)/PLLSAIDIVR */
      __HAL_RCC_PLLSAI_PLLSAICLKDIVR_CONFIG(PeriphClkInit->PLLSAIDivR);
    }

    /*---------------------------- CLK48 configuration ------------------------*/
    /* Configure the PLLSAI when it is used as clock source for CLK48 */
    if((((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_CLK48) == (RCC_PERIPHCLK_CLK48)) &&
       (PeriphClkInit->Clk48ClockSelection == RCC_CLK48CLKSOURCE_PLLSAIP))
    {
      assert_param(IS_RCC_PLLSAIP_VALUE(PeriphClkInit->PLLSAI.PLLSAIP));

      /* Read PLLSAIQ value from PLLSAICFGR register (this value is not need for SAI configuration) */
      pllsaiq = ((RCC->PLLSAICFGR & RCC_PLLSAICFGR_PLLSAIQ) >> RCC_PLLSAICFGR_PLLSAIQ_Pos);
      /* Read PLLSAIR value from PLLSAICFGR register (this value is not need for SAI configuration) */
      pllsair = ((RCC->PLLSAICFGR & RCC_PLLSAICFGR_PLLSAIR) >> RCC_PLLSAICFGR_PLLSAIR_Pos);
      /* PLLSAI_VCO Input  = PLL_SOURCE/PLLM */
      /* PLLSAI_VCO Output = PLLSAI_VCO Input * PLLSAIN */
      /* CLK48_CLK(first level) = PLLSAI_VCO Output/PLLSAIP */
      __HAL_RCC_PLLSAI_CONFIG(PeriphClkInit->PLLSAI.PLLSAIN, PeriphClkInit->PLLSAI.PLLSAIP, pllsaiq, pllsair);
    }

    /* Enable PLLSAI Clock */
    __HAL_RCC_PLLSAI_ENABLE();
    /* Get tick */
    tickstart = HAL_GetTick();
    /* Wait till PLLSAI is ready */
    while(__HAL_RCC_PLLSAI_GET_FLAG() == RESET)
    {
      if((HAL_GetTick() - tickstart ) > PLLSAI_TIMEOUT_VALUE)
      {
        /* return in case of Timeout detected */
        return HAL_TIMEOUT;
      }
    }
  }

  /*--------------------------------------------------------------------------*/

  /*---------------------------- RTC configuration ---------------------------*/
  if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_RTC) == (RCC_PERIPHCLK_RTC))
  {
    /* Check for RTC Parameters used to output RTCCLK */
    assert_param(IS_RCC_RTCCLKSOURCE(PeriphClkInit->RTCClockSelection));

    /* Enable Power Clock*/
    __HAL_RCC_PWR_CLK_ENABLE();

    /* Enable write access to Backup domain */
    PWR->CR |= PWR_CR_DBP;

    /* Get tick */
    tickstart = HAL_GetTick();

    while((PWR->CR & PWR_CR_DBP) == RESET)
    {
      if((HAL_GetTick() - tickstart ) > RCC_DBP_TIMEOUT_VALUE)
      {
        return HAL_TIMEOUT;
      }
    }
    /* Reset the Backup domain only if the RTC Clock source selection is modified from reset value */
    tmpreg1 = (RCC->BDCR & RCC_BDCR_RTCSEL);
    if((tmpreg1 != 0x00000000U) && ((tmpreg1) != (PeriphClkInit->RTCClockSelection & RCC_BDCR_RTCSEL)))
    {
      /* Store the content of BDCR register before the reset of Backup Domain */
      tmpreg1 = (RCC->BDCR & ~(RCC_BDCR_RTCSEL));
      /* RTC Clock selection can be changed only if the Backup Domain is reset */
      __HAL_RCC_BACKUPRESET_FORCE();
      __HAL_RCC_BACKUPRESET_RELEASE();
      /* Restore the Content of BDCR register */
      RCC->BDCR = tmpreg1;

      /* Wait for LSE reactivation if LSE was enable prior to Backup Domain reset */
      if(HAL_IS_BIT_SET(RCC->BDCR, RCC_BDCR_LSEON))
      {
        /* Get tick */
        tickstart = HAL_GetTick();

        /* Wait till LSE is ready */
        while(__HAL_RCC_GET_FLAG(RCC_FLAG_LSERDY) == RESET)
        {
          if((HAL_GetTick() - tickstart ) > RCC_LSE_TIMEOUT_VALUE)
          {
            return HAL_TIMEOUT;
          }
        }
      }
    }
    __HAL_RCC_RTC_CONFIG(PeriphClkInit->RTCClockSelection);
  }
  /*--------------------------------------------------------------------------*/

  /*---------------------------- TIM configuration ---------------------------*/
  if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_TIM) == (RCC_PERIPHCLK_TIM))
  {
    __HAL_RCC_TIMCLKPRESCALER(PeriphClkInit->TIMPresSelection);
  }
  return HAL_OK;
}

/**
  * @brief  Configures the RCC_PeriphCLKInitTypeDef according to the internal
  * RCC configuration registers.
  * @param  PeriphClkInit pointer to an RCC_PeriphCLKInitTypeDef structure that
  *         will be configured.
  * @retval None
  */
void HAL_RCCEx_GetPeriphCLKConfig(RCC_PeriphCLKInitTypeDef  *PeriphClkInit)
{
  uint32_t tempreg;

  /* Set all possible values for the extended clock type parameter------------*/
  PeriphClkInit->PeriphClockSelection = RCC_PERIPHCLK_I2S        | RCC_PERIPHCLK_SAI_PLLSAI |\
                                        RCC_PERIPHCLK_SAI_PLLI2S | RCC_PERIPHCLK_LTDC       |\
                                        RCC_PERIPHCLK_TIM        | RCC_PERIPHCLK_RTC        |\
                                        RCC_PERIPHCLK_CLK48       | RCC_PERIPHCLK_SDIO;

  /* Get the PLLI2S Clock configuration --------------------------------------*/
  PeriphClkInit->PLLI2S.PLLI2SN = (uint32_t)((RCC->PLLI2SCFGR & RCC_PLLI2SCFGR_PLLI2SN) >> RCC_PLLI2SCFGR_PLLI2SN_Pos);
  PeriphClkInit->PLLI2S.PLLI2SR = (uint32_t)((RCC->PLLI2SCFGR & RCC_PLLI2SCFGR_PLLI2SR) >> RCC_PLLI2SCFGR_PLLI2SR_Pos);
  PeriphClkInit->PLLI2S.PLLI2SQ = (uint32_t)((RCC->PLLI2SCFGR & RCC_PLLI2SCFGR_PLLI2SQ) >> RCC_PLLI2SCFGR_PLLI2SQ_Pos);
  /* Get the PLLSAI Clock configuration --------------------------------------*/
  PeriphClkInit->PLLSAI.PLLSAIN = (uint32_t)((RCC->PLLSAICFGR & RCC_PLLSAICFGR_PLLSAIN) >> RCC_PLLSAICFGR_PLLSAIN_Pos);
  PeriphClkInit->PLLSAI.PLLSAIR = (uint32_t)((RCC->PLLSAICFGR & RCC_PLLSAICFGR_PLLSAIR) >> RCC_PLLSAICFGR_PLLSAIR_Pos);
  PeriphClkInit->PLLSAI.PLLSAIQ = (uint32_t)((RCC->PLLSAICFGR & RCC_PLLSAICFGR_PLLSAIQ) >> RCC_PLLSAICFGR_PLLSAIQ_Pos);
  /* Get the PLLSAI/PLLI2S division factors ----------------------------------*/
  PeriphClkInit->PLLI2SDivQ = (uint32_t)((RCC->DCKCFGR & RCC_DCKCFGR_PLLI2SDIVQ) >> RCC_DCKCFGR_PLLI2SDIVQ_Pos);
  PeriphClkInit->PLLSAIDivQ = (uint32_t)((RCC->DCKCFGR & RCC_DCKCFGR_PLLSAIDIVQ) >> RCC_DCKCFGR_PLLSAIDIVQ_Pos);
  PeriphClkInit->PLLSAIDivR = (uint32_t)(RCC->DCKCFGR & RCC_DCKCFGR_PLLSAIDIVR);
  /* Get the RTC Clock configuration -----------------------------------------*/
  tempreg = (RCC->CFGR & RCC_CFGR_RTCPRE);
  PeriphClkInit->RTCClockSelection = (uint32_t)((tempreg) | (RCC->BDCR & RCC_BDCR_RTCSEL));

    /* Get the CLK48 clock configuration -------------------------------------*/
  PeriphClkInit->Clk48ClockSelection = __HAL_RCC_GET_CLK48_SOURCE();

  /* Get the SDIO clock configuration ----------------------------------------*/
  PeriphClkInit->SdioClockSelection = __HAL_RCC_GET_SDIO_SOURCE();

  if ((RCC->DCKCFGR & RCC_DCKCFGR_TIMPRE) == RESET)
  {
    PeriphClkInit->TIMPresSelection = RCC_TIMPRES_DESACTIVATED;
  }
  else
  {
    PeriphClkInit->TIMPresSelection = RCC_TIMPRES_ACTIVATED;
  }
}

/**
  * @brief  Return the peripheral clock frequency for a given peripheral(SAI..)
  * @note   Return 0 if peripheral clock identifier not managed by this API
  * @param  PeriphClk Peripheral clock identifier
  *         This parameter can be one of the following values:
  *            @arg RCC_PERIPHCLK_I2S: I2S peripheral clock
  * @retval Frequency in KHz
  */
uint32_t HAL_RCCEx_GetPeriphCLKFreq(uint32_t PeriphClk)
{
  /* This variable used to store the I2S clock frequency (value in Hz) */
  uint32_t frequency = 0U;
  /* This variable used to store the VCO Input (value in Hz) */
  uint32_t vcoinput = 0U;
  uint32_t srcclk = 0U;
  /* This variable used to store the VCO Output (value in Hz) */
  uint32_t vcooutput = 0U;
  switch (PeriphClk)
  {
  case RCC_PERIPHCLK_I2S:
    {
      /* Get the current I2S source */
      srcclk = __HAL_RCC_GET_I2S_SOURCE();
      switch (srcclk)
      {
      /* Check if I2S clock selection is External clock mapped on the I2S_CKIN pin used as I2S clock */
      case RCC_I2SCLKSOURCE_EXT:
        {
          /* Set the I2S clock to the external clock  value */
          frequency = EXTERNAL_CLOCK_VALUE;
          break;
        }
      /* Check if I2S clock selection is PLLI2S VCO output clock divided by PLLI2SR used as I2S clock */
      case RCC_I2SCLKSOURCE_PLLI2S:
        {
          /* Configure the PLLI2S division factor */
          /* PLLI2S_VCO Input  = PLL_SOURCE/PLLI2SM */
          if((RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) == RCC_PLLSOURCE_HSE)
          {
            /* Get the I2S source clock value */
            vcoinput = (uint32_t)(HSE_VALUE / (uint32_t)(RCC->PLLCFGR & RCC_PLLCFGR_PLLM));
          }
          else
          {
            /* Get the I2S source clock value */
            vcoinput = (uint32_t)(HSI_VALUE / (uint32_t)(RCC->PLLCFGR & RCC_PLLCFGR_PLLM));
          }

          /* PLLI2S_VCO Output = PLLI2S_VCO Input * PLLI2SN */
          vcooutput = (uint32_t)(vcoinput * (((RCC->PLLI2SCFGR & RCC_PLLI2SCFGR_PLLI2SN) >> 6U) & (RCC_PLLI2SCFGR_PLLI2SN >> 6U)));
          /* I2S_CLK = PLLI2S_VCO Output/PLLI2SR */
          frequency = (uint32_t)(vcooutput /(((RCC->PLLI2SCFGR & RCC_PLLI2SCFGR_PLLI2SR) >> 28U) & (RCC_PLLI2SCFGR_PLLI2SR >> 28U)));
          break;
        }
        /* Clock not enabled for I2S*/
      default:
        {
          frequency = 0U;
          break;
        }
      }
      break;
    }
  }
  return frequency;
}
#endif /* STM32F469xx || STM32F479xx */

#if defined(STM32F412Zx) || defined(STM32F412Vx) || defined(STM32F412Rx) || defined(STM32F412Cx) || defined(STM32F413xx) || defined(STM32F423xx)
/**
  * @brief  Initializes the RCC extended peripherals clocks according to the specified
  *         parameters in the RCC_PeriphCLKInitTypeDef.
  * @param  PeriphClkInit pointer to an RCC_PeriphCLKInitTypeDef structure that
  *         contains the configuration information for the Extended Peripherals
  *         clocks(I2S, LTDC RTC and TIM).
  *
  * @note   Care must be taken when HAL_RCCEx_PeriphCLKConfig() is used to select
  *         the RTC clock source; in this case the Backup domain will be reset in
  *         order to modify the RTC Clock source, as consequence RTC registers (including
  *         the backup registers) and RCC_BDCR register are set to their reset values.
  *
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_RCCEx_PeriphCLKConfig(RCC_PeriphCLKInitTypeDef  *PeriphClkInit)
{
  uint32_t tickstart = 0U;
  uint32_t tmpreg1 = 0U;
#if defined(STM32F413xx) || defined(STM32F423xx)
  uint32_t plli2sq = 0U;
#endif /* STM32F413xx || STM32F423xx */
  uint32_t plli2sused = 0U;

  /* Check the peripheral clock selection parameters */
  assert_param(IS_RCC_PERIPHCLOCK(PeriphClkInit->PeriphClockSelection));

  /*----------------------------------- I2S APB1 configuration ---------------*/
  if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_I2S_APB1) == (RCC_PERIPHCLK_I2S_APB1))
  {
    /* Check the parameters */
    assert_param(IS_RCC_I2SAPB1CLKSOURCE(PeriphClkInit->I2sApb1ClockSelection));

    /* Configure I2S Clock source */
    __HAL_RCC_I2S_APB1_CONFIG(PeriphClkInit->I2sApb1ClockSelection);
    /* Enable the PLLI2S when it's used as clock source for I2S */
    if(PeriphClkInit->I2sApb1ClockSelection == RCC_I2SAPB1CLKSOURCE_PLLI2S)
    {
      plli2sused = 1U;
    }
  }
  /*--------------------------------------------------------------------------*/

  /*----------------------------------- I2S APB2 configuration ---------------*/
  if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_I2S_APB2) == (RCC_PERIPHCLK_I2S_APB2))
  {
    /* Check the parameters */
    assert_param(IS_RCC_I2SAPB2CLKSOURCE(PeriphClkInit->I2sApb2ClockSelection));

    /* Configure I2S Clock source */
    __HAL_RCC_I2S_APB2_CONFIG(PeriphClkInit->I2sApb2ClockSelection);
    /* Enable the PLLI2S when it's used as clock source for I2S */
    if(PeriphClkInit->I2sApb2ClockSelection == RCC_I2SAPB2CLKSOURCE_PLLI2S)
    {
      plli2sused = 1U;
    }
  }
  /*--------------------------------------------------------------------------*/

#if defined(STM32F413xx) || defined(STM32F423xx)
  /*----------------------- SAI1 Block A configuration -----------------------*/
  if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_SAIA) == (RCC_PERIPHCLK_SAIA))
  {
    /* Check the parameters */
    assert_param(IS_RCC_SAIACLKSOURCE(PeriphClkInit->SaiAClockSelection));

    /* Configure SAI1 Clock source */
    __HAL_RCC_SAI_BLOCKACLKSOURCE_CONFIG(PeriphClkInit->SaiAClockSelection);
    /* Enable the PLLI2S when it's used as clock source for SAI */
    if(PeriphClkInit->SaiAClockSelection == RCC_SAIACLKSOURCE_PLLI2SR)
    {
      plli2sused = 1U;
    }
    /* Enable the PLLSAI when it's used as clock source for SAI */
    if(PeriphClkInit->SaiAClockSelection == RCC_SAIACLKSOURCE_PLLR)
    {
      /* Check for PLL/DIVR parameters */
      assert_param(IS_RCC_PLL_DIVR_VALUE(PeriphClkInit->PLLDivR));

      /* SAI_CLK_x = SAI_CLK(first level)/PLLDIVR */
      __HAL_RCC_PLL_PLLSAICLKDIVR_CONFIG(PeriphClkInit->PLLDivR);
    }
  }
  /*--------------------------------------------------------------------------*/

  /*---------------------- SAI1 Block B configuration ------------------------*/
  if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_SAIB) == (RCC_PERIPHCLK_SAIB))
  {
    /* Check the parameters */
    assert_param(IS_RCC_SAIBCLKSOURCE(PeriphClkInit->SaiBClockSelection));

    /* Configure SAI1 Clock source */
    __HAL_RCC_SAI_BLOCKBCLKSOURCE_CONFIG(PeriphClkInit->SaiBClockSelection);
    /* Enable the PLLI2S when it's used as clock source for SAI */
    if(PeriphClkInit->SaiBClockSelection == RCC_SAIBCLKSOURCE_PLLI2SR)
    {
      plli2sused = 1U;
    }
    /* Enable the PLLSAI when it's used as clock source for SAI */
    if(PeriphClkInit->SaiBClockSelection == RCC_SAIBCLKSOURCE_PLLR)
    {
      /* Check for PLL/DIVR parameters */
      assert_param(IS_RCC_PLL_DIVR_VALUE(PeriphClkInit->PLLDivR));

      /* SAI_CLK_x = SAI_CLK(first level)/PLLDIVR */
      __HAL_RCC_PLL_PLLSAICLKDIVR_CONFIG(PeriphClkInit->PLLDivR);
    }
  }
  /*--------------------------------------------------------------------------*/
#endif /* STM32F413xx || STM32F423xx */

  /*------------------------------------ RTC configuration -------------------*/
  if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_RTC) == (RCC_PERIPHCLK_RTC))
  {
    /* Check for RTC Parameters used to output RTCCLK */
    assert_param(IS_RCC_RTCCLKSOURCE(PeriphClkInit->RTCClockSelection));

    /* Enable Power Clock*/
    __HAL_RCC_PWR_CLK_ENABLE();

    /* Enable write access to Backup domain */
    PWR->CR |= PWR_CR_DBP;

    /* Get tick */
    tickstart = HAL_GetTick();

    while((PWR->CR & PWR_CR_DBP) == RESET)
    {
      if((HAL_GetTick() - tickstart ) > RCC_DBP_TIMEOUT_VALUE)
      {
        return HAL_TIMEOUT;
      }
    }
    /* Reset the Backup domain only if the RTC Clock source selection is modified from reset value */
    tmpreg1 = (RCC->BDCR & RCC_BDCR_RTCSEL);
    if((tmpreg1 != 0x00000000U) && ((tmpreg1) != (PeriphClkInit->RTCClockSelection & RCC_BDCR_RTCSEL)))
    {
      /* Store the content of BDCR register before the reset of Backup Domain */
      tmpreg1 = (RCC->BDCR & ~(RCC_BDCR_RTCSEL));
      /* RTC Clock selection can be changed only if the Backup Domain is reset */
      __HAL_RCC_BACKUPRESET_FORCE();
      __HAL_RCC_BACKUPRESET_RELEASE();
      /* Restore the Content of BDCR register */
      RCC->BDCR = tmpreg1;

      /* Wait for LSE reactivation if LSE was enable prior to Backup Domain reset */
      if(HAL_IS_BIT_SET(RCC->BDCR, RCC_BDCR_LSEON))
      {
        /* Get tick */
        tickstart = HAL_GetTick();

        /* Wait till LSE is ready */
        while(__HAL_RCC_GET_FLAG(RCC_FLAG_LSERDY) == RESET)
        {
          if((HAL_GetTick() - tickstart ) > RCC_LSE_TIMEOUT_VALUE)
          {
            return HAL_TIMEOUT;
          }
        }
      }
    }
    __HAL_RCC_RTC_CONFIG(PeriphClkInit->RTCClockSelection);
  }
  /*--------------------------------------------------------------------------*/

  /*------------------------------------ TIM configuration -------------------*/
  if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_TIM) == (RCC_PERIPHCLK_TIM))
  {
    /* Configure Timer Prescaler */
    __HAL_RCC_TIMCLKPRESCALER(PeriphClkInit->TIMPresSelection);
  }
  /*--------------------------------------------------------------------------*/

  /*------------------------------------- FMPI2C1 Configuration --------------*/
  if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_FMPI2C1) == RCC_PERIPHCLK_FMPI2C1)
  {
    /* Check the parameters */
    assert_param(IS_RCC_FMPI2C1CLKSOURCE(PeriphClkInit->Fmpi2c1ClockSelection));

    /* Configure the FMPI2C1 clock source */
    __HAL_RCC_FMPI2C1_CONFIG(PeriphClkInit->Fmpi2c1ClockSelection);
  }
  /*--------------------------------------------------------------------------*/

  /*------------------------------------- CLK48 Configuration ----------------*/
  if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_CLK48) == RCC_PERIPHCLK_CLK48)
  {
    /* Check the parameters */
    assert_param(IS_RCC_CLK48CLKSOURCE(PeriphClkInit->Clk48ClockSelection));

    /* Configure the SDIO clock source */
    __HAL_RCC_CLK48_CONFIG(PeriphClkInit->Clk48ClockSelection);

    /* Enable the PLLI2S when it's used as clock source for CLK48 */
    if(PeriphClkInit->Clk48ClockSelection == RCC_CLK48CLKSOURCE_PLLI2SQ)
    {
      plli2sused = 1U;
    }
  }
  /*--------------------------------------------------------------------------*/

  /*------------------------------------- SDIO Configuration -----------------*/
  if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_SDIO) == RCC_PERIPHCLK_SDIO)
  {
    /* Check the parameters */
    assert_param(IS_RCC_SDIOCLKSOURCE(PeriphClkInit->SdioClockSelection));

    /* Configure the SDIO clock source */
    __HAL_RCC_SDIO_CONFIG(PeriphClkInit->SdioClockSelection);
  }
  /*--------------------------------------------------------------------------*/

  /*-------------------------------------- PLLI2S Configuration --------------*/
  /* PLLI2S is configured when a peripheral will use it as source clock : I2S on APB1 or
     I2S on APB2*/
  if((plli2sused == 1U) || (PeriphClkInit->PeriphClockSelection == RCC_PERIPHCLK_PLLI2S))
  {
    /* Disable the PLLI2S */
    __HAL_RCC_PLLI2S_DISABLE();
    /* Get tick */
    tickstart = HAL_GetTick();
    /* Wait till PLLI2S is disabled */
    while(__HAL_RCC_GET_FLAG(RCC_FLAG_PLLI2SRDY)  != RESET)
    {
      if((HAL_GetTick() - tickstart ) > PLLI2S_TIMEOUT_VALUE)
      {
        /* return in case of Timeout detected */
        return HAL_TIMEOUT;
      }
    }

    /* check for common PLLI2S Parameters */
    assert_param(IS_RCC_PLLI2SCLKSOURCE(PeriphClkInit->PLLI2SSelection));
    assert_param(IS_RCC_PLLI2SM_VALUE(PeriphClkInit->PLLI2S.PLLI2SM));
    assert_param(IS_RCC_PLLI2SN_VALUE(PeriphClkInit->PLLI2S.PLLI2SN));
    /*-------------------- Set the PLL I2S clock -----------------------------*/
    __HAL_RCC_PLL_I2S_CONFIG(PeriphClkInit->PLLI2SSelection);

    /*------- In Case of PLLI2S is selected as source clock for I2S ----------*/
    if(((((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_I2S_APB1) == RCC_PERIPHCLK_I2S_APB1) && (PeriphClkInit->I2sApb1ClockSelection == RCC_I2SAPB1CLKSOURCE_PLLI2S)) ||
       ((((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_I2S_APB2) == RCC_PERIPHCLK_I2S_APB2) && (PeriphClkInit->I2sApb2ClockSelection == RCC_I2SAPB2CLKSOURCE_PLLI2S)) ||
       ((((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_CLK48) == RCC_PERIPHCLK_CLK48) && (PeriphClkInit->Clk48ClockSelection == RCC_CLK48CLKSOURCE_PLLI2SQ)) ||
       ((((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_SDIO) == RCC_PERIPHCLK_SDIO) && (PeriphClkInit->SdioClockSelection == RCC_SDIOCLKSOURCE_CLK48) && (PeriphClkInit->Clk48ClockSelection == RCC_CLK48CLKSOURCE_PLLI2SQ)))
    {
      /* check for Parameters */
      assert_param(IS_RCC_PLLI2SR_VALUE(PeriphClkInit->PLLI2S.PLLI2SR));
      assert_param(IS_RCC_PLLI2SQ_VALUE(PeriphClkInit->PLLI2S.PLLI2SQ));

      /* Configure the PLLI2S division factors */
      /* PLLI2S_VCO = f(VCO clock) = f(PLLI2S clock input) * (PLLI2SN/PLLI2SM)*/
      /* I2SCLK = f(PLLI2S clock output) = f(VCO clock) / PLLI2SR */
      __HAL_RCC_PLLI2S_CONFIG(PeriphClkInit->PLLI2S.PLLI2SM, PeriphClkInit->PLLI2S.PLLI2SN , PeriphClkInit->PLLI2S.PLLI2SQ, PeriphClkInit->PLLI2S.PLLI2SR);
    }

#if defined(STM32F413xx) || defined(STM32F423xx)
    /*------- In Case of PLLI2S is selected as source clock for SAI ----------*/
    if(((((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_SAIA) == RCC_PERIPHCLK_SAIA) && (PeriphClkInit->SaiAClockSelection == RCC_SAIACLKSOURCE_PLLI2SR)) ||
       ((((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_SAIB) == RCC_PERIPHCLK_SAIB) && (PeriphClkInit->SaiBClockSelection == RCC_SAIBCLKSOURCE_PLLI2SR)))
    {
      /* Check for PLLI2S Parameters */
      assert_param(IS_RCC_PLLI2SR_VALUE(PeriphClkInit->PLLI2S.PLLI2SR));
      /* Check for PLLI2S/DIVR parameters */
      assert_param(IS_RCC_PLLI2S_DIVR_VALUE(PeriphClkInit->PLLI2SDivR));

      /* Read PLLI2SQ value from PLLI2SCFGR register (this value is not needed for SAI configuration) */
      plli2sq = ((RCC->PLLI2SCFGR & RCC_PLLI2SCFGR_PLLI2SQ) >> RCC_PLLI2SCFGR_PLLI2SQ_Pos);
      /* Configure the PLLI2S division factors */
      /* PLLI2S_VCO Input  = PLL_SOURCE/PLLI2SM */
      /* PLLI2S_VCO Output = PLLI2S_VCO Input * PLLI2SN */
      /* SAI_CLK(first level) = PLLI2S_VCO Output/PLLI2SQ */
      __HAL_RCC_PLLI2S_CONFIG(PeriphClkInit->PLLI2S.PLLI2SM, PeriphClkInit->PLLI2S.PLLI2SN, plli2sq, PeriphClkInit->PLLI2S.PLLI2SR);

      /* SAI_CLK_x = SAI_CLK(first level)/PLLI2SDIVR */
      __HAL_RCC_PLLI2S_PLLSAICLKDIVR_CONFIG(PeriphClkInit->PLLI2SDivR);
    }
#endif /* STM32F413xx || STM32F423xx */

    /*----------------- In Case of PLLI2S is just selected  ------------------*/
    if((PeriphClkInit->PeriphClockSelection & RCC_PERIPHCLK_PLLI2S) == RCC_PERIPHCLK_PLLI2S)
    {
      /* Check for Parameters */
      assert_param(IS_RCC_PLLI2SR_VALUE(PeriphClkInit->PLLI2S.PLLI2SR));
      assert_param(IS_RCC_PLLI2SQ_VALUE(PeriphClkInit->PLLI2S.PLLI2SQ));

      /* Configure the PLLI2S division factors */
      /* PLLI2S_VCO = f(VCO clock) = f(PLLI2S clock input) * (PLLI2SN/PLLI2SM)*/
      /* SPDIFRXCLK = f(PLLI2S clock output) = f(VCO clock) / PLLI2SP */
      __HAL_RCC_PLLI2S_CONFIG(PeriphClkInit->PLLI2S.PLLI2SM, PeriphClkInit->PLLI2S.PLLI2SN , PeriphClkInit->PLLI2S.PLLI2SQ, PeriphClkInit->PLLI2S.PLLI2SR);
    }

    /* Enable the PLLI2S */
    __HAL_RCC_PLLI2S_ENABLE();
    /* Get tick */
    tickstart = HAL_GetTick();
    /* Wait till PLLI2S is ready */
    while(__HAL_RCC_GET_FLAG(RCC_FLAG_PLLI2SRDY)  == RESET)
    {
      if((HAL_GetTick() - tickstart ) > PLLI2S_TIMEOUT_VALUE)
      {
        /* return in case of Timeout detected */
        return HAL_TIMEOUT;
      }
    }
  }
  /*--------------------------------------------------------------------------*/

  /*-------------------- DFSDM1 clock source configuration -------------------*/
  if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_DFSDM1) == RCC_PERIPHCLK_DFSDM1)
  {
    /* Check the parameters */
    assert_param(IS_RCC_DFSDM1CLKSOURCE(PeriphClkInit->Dfsdm1ClockSelection));

    /* Configure the DFSDM1 interface clock source */
    __HAL_RCC_DFSDM1_CONFIG(PeriphClkInit->Dfsdm1ClockSelection);
  }
  /*--------------------------------------------------------------------------*/

  /*-------------------- DFSDM1 Audio clock source configuration -------------*/
  if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_DFSDM1_AUDIO) == RCC_PERIPHCLK_DFSDM1_AUDIO)
  {
    /* Check the parameters */
    assert_param(IS_RCC_DFSDM1AUDIOCLKSOURCE(PeriphClkInit->Dfsdm1AudioClockSelection));

    /* Configure the DFSDM1 Audio interface clock source */
    __HAL_RCC_DFSDM1AUDIO_CONFIG(PeriphClkInit->Dfsdm1AudioClockSelection);
  }
  /*--------------------------------------------------------------------------*/

#if defined(STM32F413xx) || defined(STM32F423xx)
  /*-------------------- DFSDM2 clock source configuration -------------------*/
  if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_DFSDM2) == RCC_PERIPHCLK_DFSDM2)
  {
    /* Check the parameters */
    assert_param(IS_RCC_DFSDM2CLKSOURCE(PeriphClkInit->Dfsdm2ClockSelection));

    /* Configure the DFSDM1 interface clock source */
    __HAL_RCC_DFSDM2_CONFIG(PeriphClkInit->Dfsdm2ClockSelection);
  }
  /*--------------------------------------------------------------------------*/

  /*-------------------- DFSDM2 Audio clock source configuration -------------*/
  if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_DFSDM2_AUDIO) == RCC_PERIPHCLK_DFSDM2_AUDIO)
  {
    /* Check the parameters */
    assert_param(IS_RCC_DFSDM2AUDIOCLKSOURCE(PeriphClkInit->Dfsdm2AudioClockSelection));

    /* Configure the DFSDM1 Audio interface clock source */
    __HAL_RCC_DFSDM2AUDIO_CONFIG(PeriphClkInit->Dfsdm2AudioClockSelection);
  }
  /*--------------------------------------------------------------------------*/

  /*---------------------------- LPTIM1 Configuration ------------------------*/
  if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_LPTIM1) == RCC_PERIPHCLK_LPTIM1)
  {
    /* Check the parameters */
    assert_param(IS_RCC_LPTIM1CLKSOURCE(PeriphClkInit->Lptim1ClockSelection));

    /* Configure the LPTIM1 clock source */
    __HAL_RCC_LPTIM1_CONFIG(PeriphClkInit->Lptim1ClockSelection);
  }
  /*--------------------------------------------------------------------------*/
#endif /* STM32F413xx || STM32F423xx */

  return HAL_OK;
}

/**
  * @brief  Get the RCC_PeriphCLKInitTypeDef according to the internal
  *         RCC configuration registers.
  * @param  PeriphClkInit pointer to an RCC_PeriphCLKInitTypeDef structure that
  *         will be configured.
  * @retval None
  */
void HAL_RCCEx_GetPeriphCLKConfig(RCC_PeriphCLKInitTypeDef  *PeriphClkInit)
{
  uint32_t tempreg;

  /* Set all possible values for the extended clock type parameter------------*/
#if defined(STM32F413xx) || defined(STM32F423xx)
  PeriphClkInit->PeriphClockSelection = RCC_PERIPHCLK_I2S_APB1     | RCC_PERIPHCLK_I2S_APB2 |\
                                        RCC_PERIPHCLK_TIM          | RCC_PERIPHCLK_RTC      |\
                                        RCC_PERIPHCLK_FMPI2C1      | RCC_PERIPHCLK_CLK48    |\
                                        RCC_PERIPHCLK_SDIO         | RCC_PERIPHCLK_DFSDM1   |\
                                        RCC_PERIPHCLK_DFSDM1_AUDIO | RCC_PERIPHCLK_DFSDM2   |\
                                        RCC_PERIPHCLK_DFSDM2_AUDIO | RCC_PERIPHCLK_LPTIM1   |\
                                        RCC_PERIPHCLK_SAIA         | RCC_PERIPHCLK_SAIB;
#else /* STM32F412Zx || STM32F412Vx || STM32F412Rx || STM32F412Cx */
  PeriphClkInit->PeriphClockSelection = RCC_PERIPHCLK_I2S_APB1 | RCC_PERIPHCLK_I2S_APB2 |\
                                        RCC_PERIPHCLK_TIM      | RCC_PERIPHCLK_RTC      |\
                                        RCC_PERIPHCLK_FMPI2C1  | RCC_PERIPHCLK_CLK48    |\
                                        RCC_PERIPHCLK_SDIO     | RCC_PERIPHCLK_DFSDM1   |\
                                        RCC_PERIPHCLK_DFSDM1_AUDIO;
#endif /* STM32F413xx || STM32F423xx */



  /* Get the PLLI2S Clock configuration --------------------------------------*/
  PeriphClkInit->PLLI2S.PLLI2SM = (uint32_t)((RCC->PLLI2SCFGR & RCC_PLLI2SCFGR_PLLI2SM) >> RCC_PLLI2SCFGR_PLLI2SM_Pos);
  PeriphClkInit->PLLI2S.PLLI2SN = (uint32_t)((RCC->PLLI2SCFGR & RCC_PLLI2SCFGR_PLLI2SN) >> RCC_PLLI2SCFGR_PLLI2SN_Pos);
  PeriphClkInit->PLLI2S.PLLI2SQ = (uint32_t)((RCC->PLLI2SCFGR & RCC_PLLI2SCFGR_PLLI2SQ) >> RCC_PLLI2SCFGR_PLLI2SQ_Pos);
  PeriphClkInit->PLLI2S.PLLI2SR = (uint32_t)((RCC->PLLI2SCFGR & RCC_PLLI2SCFGR_PLLI2SR) >> RCC_PLLI2SCFGR_PLLI2SR_Pos);
#if defined(STM32F413xx) || defined(STM32F423xx)
  /* Get the PLL/PLLI2S division factors -------------------------------------*/
  PeriphClkInit->PLLI2SDivR = (uint32_t)((RCC->DCKCFGR & RCC_DCKCFGR_PLLI2SDIVR) >> RCC_DCKCFGR_PLLI2SDIVR_Pos);
  PeriphClkInit->PLLDivR = (uint32_t)((RCC->DCKCFGR & RCC_DCKCFGR_PLLDIVR) >> RCC_DCKCFGR_PLLDIVR_Pos);
#endif /* STM32F413xx || STM32F423xx */

  /* Get the I2S APB1 clock configuration ------------------------------------*/
  PeriphClkInit->I2sApb1ClockSelection = __HAL_RCC_GET_I2S_APB1_SOURCE();

  /* Get the I2S APB2 clock configuration ------------------------------------*/
  PeriphClkInit->I2sApb2ClockSelection = __HAL_RCC_GET_I2S_APB2_SOURCE();

  /* Get the RTC Clock configuration -----------------------------------------*/
  tempreg = (RCC->CFGR & RCC_CFGR_RTCPRE);
  PeriphClkInit->RTCClockSelection = (uint32_t)((tempreg) | (RCC->BDCR & RCC_BDCR_RTCSEL));

  /* Get the FMPI2C1 clock configuration -------------------------------------*/
  PeriphClkInit->Fmpi2c1ClockSelection = __HAL_RCC_GET_FMPI2C1_SOURCE();

  /* Get the CLK48 clock configuration ---------------------------------------*/
  PeriphClkInit->Clk48ClockSelection = __HAL_RCC_GET_CLK48_SOURCE();

  /* Get the SDIO clock configuration ----------------------------------------*/
  PeriphClkInit->SdioClockSelection = __HAL_RCC_GET_SDIO_SOURCE();

  /* Get the DFSDM1 clock configuration --------------------------------------*/
  PeriphClkInit->Dfsdm1ClockSelection = __HAL_RCC_GET_DFSDM1_SOURCE();

  /* Get the DFSDM1 Audio clock configuration --------------------------------*/
  PeriphClkInit->Dfsdm1AudioClockSelection = __HAL_RCC_GET_DFSDM1AUDIO_SOURCE();

#if defined(STM32F413xx) || defined(STM32F423xx)
  /* Get the DFSDM2 clock configuration --------------------------------------*/
  PeriphClkInit->Dfsdm2ClockSelection = __HAL_RCC_GET_DFSDM2_SOURCE();

  /* Get the DFSDM2 Audio clock configuration --------------------------------*/
  PeriphClkInit->Dfsdm2AudioClockSelection = __HAL_RCC_GET_DFSDM2AUDIO_SOURCE();

  /* Get the LPTIM1 clock configuration --------------------------------------*/
  PeriphClkInit->Lptim1ClockSelection = __HAL_RCC_GET_LPTIM1_SOURCE();

  /* Get the SAI1 Block Aclock configuration ---------------------------------*/
  PeriphClkInit->SaiAClockSelection = __HAL_RCC_GET_SAI_BLOCKA_SOURCE();

  /* Get the SAI1 Block B clock configuration --------------------------------*/
  PeriphClkInit->SaiBClockSelection = __HAL_RCC_GET_SAI_BLOCKB_SOURCE();
#endif /* STM32F413xx || STM32F423xx */

  /* Get the TIM Prescaler configuration -------------------------------------*/
  if ((RCC->DCKCFGR & RCC_DCKCFGR_TIMPRE) == RESET)
  {
    PeriphClkInit->TIMPresSelection = RCC_TIMPRES_DESACTIVATED;
  }
  else
  {
    PeriphClkInit->TIMPresSelection = RCC_TIMPRES_ACTIVATED;
  }
}

/**
  * @brief  Return the peripheral clock frequency for a given peripheral(I2S..)
  * @note   Return 0 if peripheral clock identifier not managed by this API
  * @param  PeriphClk Peripheral clock identifier
  *         This parameter can be one of the following values:
  *            @arg RCC_PERIPHCLK_I2S_APB1: I2S APB1 peripheral clock
  *            @arg RCC_PERIPHCLK_I2S_APB2: I2S APB2 peripheral clock
  * @retval Frequency in KHz
  */
uint32_t HAL_RCCEx_GetPeriphCLKFreq(uint32_t PeriphClk)
{
  /* This variable used to store the I2S clock frequency (value in Hz) */
  uint32_t frequency = 0U;
  /* This variable used to store the VCO Input (value in Hz) */
  uint32_t vcoinput = 0U;
  uint32_t srcclk = 0U;
  /* This variable used to store the VCO Output (value in Hz) */
  uint32_t vcooutput = 0U;
  switch (PeriphClk)
  {
  case RCC_PERIPHCLK_I2S_APB1:
    {
      /* Get the current I2S source */
      srcclk = __HAL_RCC_GET_I2S_APB1_SOURCE();
      switch (srcclk)
      {
      /* Check if I2S clock selection is External clock mapped on the I2S_CKIN pin used as I2S clock */
      case RCC_I2SAPB1CLKSOURCE_EXT:
        {
          /* Set the I2S clock to the external clock  value */
          frequency = EXTERNAL_CLOCK_VALUE;
          break;
        }
      /* Check if I2S clock selection is PLLI2S VCO output clock divided by PLLI2SR used as I2S clock */
      case RCC_I2SAPB1CLKSOURCE_PLLI2S:
        {
          if((RCC->PLLI2SCFGR & RCC_PLLI2SCFGR_PLLI2SSRC) == RCC_PLLI2SCFGR_PLLI2SSRC)
          {
            /* Get the I2S source clock value */
            vcoinput = (uint32_t)(EXTERNAL_CLOCK_VALUE / (uint32_t)(RCC->PLLI2SCFGR & RCC_PLLI2SCFGR_PLLI2SM));
          }
          else
          {
            /* Configure the PLLI2S division factor */
            /* PLLI2S_VCO Input  = PLL_SOURCE/PLLI2SM */
            if((RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) == RCC_PLLSOURCE_HSE)
            {
              /* Get the I2S source clock value */
              vcoinput = (uint32_t)(HSE_VALUE / (uint32_t)(RCC->PLLI2SCFGR & RCC_PLLI2SCFGR_PLLI2SM));
            }
            else
            {
              /* Get the I2S source clock value */
              vcoinput = (uint32_t)(HSI_VALUE / (uint32_t)(RCC->PLLI2SCFGR & RCC_PLLI2SCFGR_PLLI2SM));
            }
          }
          /* PLLI2S_VCO Output = PLLI2S_VCO Input * PLLI2SN */
          vcooutput = (uint32_t)(vcoinput * (((RCC->PLLI2SCFGR & RCC_PLLI2SCFGR_PLLI2SN) >> 6U) & (RCC_PLLI2SCFGR_PLLI2SN >> 6U)));
          /* I2S_CLK = PLLI2S_VCO Output/PLLI2SR */
          frequency = (uint32_t)(vcooutput /(((RCC->PLLI2SCFGR & RCC_PLLI2SCFGR_PLLI2SR) >> 28U) & (RCC_PLLI2SCFGR_PLLI2SR >> 28U)));
          break;
        }
      /* Check if I2S clock selection is PLL VCO Output divided by PLLR used as I2S clock */
      case RCC_I2SAPB1CLKSOURCE_PLLR:
        {
          /* Configure the PLL division factor R */
          /* PLL_VCO Input  = PLL_SOURCE/PLLM */
          if((RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) == RCC_PLLSOURCE_HSE)
          {
            /* Get the I2S source clock value */
            vcoinput = (uint32_t)(HSE_VALUE / (uint32_t)(RCC->PLLCFGR & RCC_PLLCFGR_PLLM));
          }
          else
          {
            /* Get the I2S source clock value */
            vcoinput = (uint32_t)(HSI_VALUE / (uint32_t)(RCC->PLLCFGR & RCC_PLLCFGR_PLLM));
          }

          /* PLL_VCO Output = PLL_VCO Input * PLLN */
          vcooutput = (uint32_t)(vcoinput * (((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6U) & (RCC_PLLCFGR_PLLN >> 6U)));
          /* I2S_CLK = PLL_VCO Output/PLLR */
          frequency = (uint32_t)(vcooutput /(((RCC->PLLCFGR & RCC_PLLCFGR_PLLR) >> 28U) & (RCC_PLLCFGR_PLLR >> 28U)));
          break;
        }
      /* Check if I2S clock selection is HSI or HSE depending from PLL source Clock */
      case RCC_I2SAPB1CLKSOURCE_PLLSRC:
        {
          if((RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) == RCC_PLLSOURCE_HSE)
          {
            frequency = HSE_VALUE;
          }
          else
          {
            frequency = HSI_VALUE;
          }
          break;
        }
        /* Clock not enabled for I2S*/
      default:
        {
          frequency = 0U;
          break;
        }
      }
      break;
    }
  case RCC_PERIPHCLK_I2S_APB2:
    {
      /* Get the current I2S source */
      srcclk = __HAL_RCC_GET_I2S_APB2_SOURCE();
      switch (srcclk)
      {
        /* Check if I2S clock selection is External clock mapped on the I2S_CKIN pin used as I2S clock */
      case RCC_I2SAPB2CLKSOURCE_EXT:
        {
          /* Set the I2S clock to the external clock  value */
          frequency = EXTERNAL_CLOCK_VALUE;
          break;
        }
        /* Check if I2S clock selection is PLLI2S VCO output clock divided by PLLI2SR used as I2S clock */
      case RCC_I2SAPB2CLKSOURCE_PLLI2S:
        {
          if((RCC->PLLI2SCFGR & RCC_PLLI2SCFGR_PLLI2SSRC) == RCC_PLLI2SCFGR_PLLI2SSRC)
          {
            /* Get the I2S source clock value */
            vcoinput = (uint32_t)(EXTERNAL_CLOCK_VALUE / (uint32_t)(RCC->PLLI2SCFGR & RCC_PLLI2SCFGR_PLLI2SM));
          }
          else
          {
            /* Configure the PLLI2S division factor */
            /* PLLI2S_VCO Input  = PLL_SOURCE/PLLI2SM */
            if((RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) == RCC_PLLSOURCE_HSE)
            {
              /* Get the I2S source clock value */
              vcoinput = (uint32_t)(HSE_VALUE / (uint32_t)(RCC->PLLI2SCFGR & RCC_PLLI2SCFGR_PLLI2SM));
            }
            else
            {
              /* Get the I2S source clock value */
              vcoinput = (uint32_t)(HSI_VALUE / (uint32_t)(RCC->PLLI2SCFGR & RCC_PLLI2SCFGR_PLLI2SM));
            }
          }
          /* PLLI2S_VCO Output = PLLI2S_VCO Input * PLLI2SN */
          vcooutput = (uint32_t)(vcoinput * (((RCC->PLLI2SCFGR & RCC_PLLI2SCFGR_PLLI2SN) >> 6U) & (RCC_PLLI2SCFGR_PLLI2SN >> 6U)));
          /* I2S_CLK = PLLI2S_VCO Output/PLLI2SR */
          frequency = (uint32_t)(vcooutput /(((RCC->PLLI2SCFGR & RCC_PLLI2SCFGR_PLLI2SR) >> 28U) & (RCC_PLLI2SCFGR_PLLI2SR >> 28U)));
          break;
        }
        /* Check if I2S clock selection is PLL VCO Output divided by PLLR used as I2S clock */
      case RCC_I2SAPB2CLKSOURCE_PLLR:
        {
          /* Configure the PLL division factor R */
          /* PLL_VCO Input  = PLL_SOURCE/PLLM */
          if((RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) == RCC_PLLSOURCE_HSE)
          {
            /* Get the I2S source clock value */
            vcoinput = (uint32_t)(HSE_VALUE / (uint32_t)(RCC->PLLCFGR & RCC_PLLCFGR_PLLM));
          }
          else
          {
            /* Get the I2S source clock value */
            vcoinput = (uint32_t)(HSI_VALUE / (uint32_t)(RCC->PLLCFGR & RCC_PLLCFGR_PLLM));
          }

          /* PLL_VCO Output = PLL_VCO Input * PLLN */
          vcooutput = (uint32_t)(vcoinput * (((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6U) & (RCC_PLLCFGR_PLLN >> 6U)));
          /* I2S_CLK = PLL_VCO Output/PLLR */
          frequency = (uint32_t)(vcooutput /(((RCC->PLLCFGR & RCC_PLLCFGR_PLLR) >> 28U) & (RCC_PLLCFGR_PLLR >> 28U)));
          break;
        }
        /* Check if I2S clock selection is HSI or HSE depending from PLL source Clock */
      case RCC_I2SAPB2CLKSOURCE_PLLSRC:
        {
          if((RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) == RCC_PLLSOURCE_HSE)
          {
            frequency = HSE_VALUE;
          }
          else
          {
            frequency = HSI_VALUE;
          }
          break;
        }
      /* Clock not enabled for I2S*/
      default:
        {
          frequency = 0U;
          break;
        }
      }
      break;
    }
  }
  return frequency;
}
#endif /* STM32F412Zx || STM32F412Vx || STM32F412Rx || STM32F412Cx || STM32F413xx || STM32F423xx */

#if defined(STM32F410Tx) || defined(STM32F410Cx) || defined(STM32F410Rx)
/**
  * @brief  Initializes the RCC extended peripherals clocks according to the specified parameters in the
  *         RCC_PeriphCLKInitTypeDef.
  * @param  PeriphClkInit pointer to an RCC_PeriphCLKInitTypeDef structure that
  *         contains the configuration information for the Extended Peripherals clocks(I2S and RTC clocks).
  *
  * @note   A caution to be taken when HAL_RCCEx_PeriphCLKConfig() is used to select RTC clock selection, in this case
  *         the Reset of Backup domain will be applied in order to modify the RTC Clock source as consequence all backup
  *        domain (RTC and RCC_BDCR register expect BKPSRAM) will be reset
  *
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_RCCEx_PeriphCLKConfig(RCC_PeriphCLKInitTypeDef  *PeriphClkInit)
{
  uint32_t tickstart = 0U;
  uint32_t tmpreg1 = 0U;

  /* Check the parameters */
  assert_param(IS_RCC_PERIPHCLOCK(PeriphClkInit->PeriphClockSelection));

  /*---------------------------- RTC configuration ---------------------------*/
  if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_RTC) == (RCC_PERIPHCLK_RTC))
  {
    /* Check for RTC Parameters used to output RTCCLK */
    assert_param(IS_RCC_RTCCLKSOURCE(PeriphClkInit->RTCClockSelection));

    /* Enable Power Clock*/
    __HAL_RCC_PWR_CLK_ENABLE();

    /* Enable write access to Backup domain */
    PWR->CR |= PWR_CR_DBP;

    /* Get tick */
    tickstart = HAL_GetTick();

    while((PWR->CR & PWR_CR_DBP) == RESET)
    {
      if((HAL_GetTick() - tickstart ) > RCC_DBP_TIMEOUT_VALUE)
      {
        return HAL_TIMEOUT;
      }
    }
    /* Reset the Backup domain only if the RTC Clock source selection is modified from reset value */
    tmpreg1 = (RCC->BDCR & RCC_BDCR_RTCSEL);
    if((tmpreg1 != 0x00000000U) && ((tmpreg1) != (PeriphClkInit->RTCClockSelection & RCC_BDCR_RTCSEL)))
    {
      /* Store the content of BDCR register before the reset of Backup Domain */
      tmpreg1 = (RCC->BDCR & ~(RCC_BDCR_RTCSEL));
      /* RTC Clock selection can be changed only if the Backup Domain is reset */
      __HAL_RCC_BACKUPRESET_FORCE();
      __HAL_RCC_BACKUPRESET_RELEASE();
      /* Restore the Content of BDCR register */
      RCC->BDCR = tmpreg1;

      /* Wait for LSE reactivation if LSE was enable prior to Backup Domain reset */
      if(HAL_IS_BIT_SET(RCC->BDCR, RCC_BDCR_LSEON))
      {
        /* Get tick */
        tickstart = HAL_GetTick();

        /* Wait till LSE is ready */
        while(__HAL_RCC_GET_FLAG(RCC_FLAG_LSERDY) == RESET)
        {
          if((HAL_GetTick() - tickstart ) > RCC_LSE_TIMEOUT_VALUE)
          {
            return HAL_TIMEOUT;
          }
        }
      }
    }
    __HAL_RCC_RTC_CONFIG(PeriphClkInit->RTCClockSelection);
  }
  /*--------------------------------------------------------------------------*/

  /*---------------------------- TIM configuration ---------------------------*/
  if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_TIM) == (RCC_PERIPHCLK_TIM))
  {
    __HAL_RCC_TIMCLKPRESCALER(PeriphClkInit->TIMPresSelection);
  }
  /*--------------------------------------------------------------------------*/

  /*---------------------------- FMPI2C1 Configuration -----------------------*/
  if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_FMPI2C1) == RCC_PERIPHCLK_FMPI2C1)
  {
    /* Check the parameters */
    assert_param(IS_RCC_FMPI2C1CLKSOURCE(PeriphClkInit->Fmpi2c1ClockSelection));

    /* Configure the FMPI2C1 clock source */
    __HAL_RCC_FMPI2C1_CONFIG(PeriphClkInit->Fmpi2c1ClockSelection);
  }
  /*--------------------------------------------------------------------------*/

  /*---------------------------- LPTIM1 Configuration ------------------------*/
  if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_LPTIM1) == RCC_PERIPHCLK_LPTIM1)
  {
    /* Check the parameters */
    assert_param(IS_RCC_LPTIM1CLKSOURCE(PeriphClkInit->Lptim1ClockSelection));

    /* Configure the LPTIM1 clock source */
    __HAL_RCC_LPTIM1_CONFIG(PeriphClkInit->Lptim1ClockSelection);
  }

  /*---------------------------- I2S Configuration ---------------------------*/
  if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_I2S) == RCC_PERIPHCLK_I2S)
  {
    /* Check the parameters */
    assert_param(IS_RCC_I2SAPBCLKSOURCE(PeriphClkInit->I2SClockSelection));

    /* Configure the I2S clock source */
    __HAL_RCC_I2S_CONFIG(PeriphClkInit->I2SClockSelection);
  }

  return HAL_OK;
}

/**
  * @brief  Configures the RCC_OscInitStruct according to the internal
  * RCC configuration registers.
  * @param  PeriphClkInit pointer to an RCC_PeriphCLKInitTypeDef structure that
  * will be configured.
  * @retval None
  */
void HAL_RCCEx_GetPeriphCLKConfig(RCC_PeriphCLKInitTypeDef  *PeriphClkInit)
{
  uint32_t tempreg;

  /* Set all possible values for the extended clock type parameter------------*/
  PeriphClkInit->PeriphClockSelection = RCC_PERIPHCLK_FMPI2C1 | RCC_PERIPHCLK_LPTIM1 | RCC_PERIPHCLK_TIM | RCC_PERIPHCLK_RTC;

  tempreg = (RCC->CFGR & RCC_CFGR_RTCPRE);
  PeriphClkInit->RTCClockSelection = (uint32_t)((tempreg) | (RCC->BDCR & RCC_BDCR_RTCSEL));

  if ((RCC->DCKCFGR & RCC_DCKCFGR_TIMPRE) == RESET)
  {
    PeriphClkInit->TIMPresSelection = RCC_TIMPRES_DESACTIVATED;
  }
  else
  {
    PeriphClkInit->TIMPresSelection = RCC_TIMPRES_ACTIVATED;
  }
  /* Get the FMPI2C1 clock configuration -------------------------------------*/
  PeriphClkInit->Fmpi2c1ClockSelection = __HAL_RCC_GET_FMPI2C1_SOURCE();

  /* Get the I2S clock configuration -----------------------------------------*/
  PeriphClkInit->I2SClockSelection = __HAL_RCC_GET_I2S_SOURCE();


}
/**
  * @brief  Return the peripheral clock frequency for a given peripheral(SAI..)
  * @note   Return 0 if peripheral clock identifier not managed by this API
  * @param  PeriphClk Peripheral clock identifier
  *         This parameter can be one of the following values:
  *            @arg RCC_PERIPHCLK_I2S: I2S peripheral clock
  * @retval Frequency in KHz
  */
uint32_t HAL_RCCEx_GetPeriphCLKFreq(uint32_t PeriphClk)
{
  /* This variable used to store the I2S clock frequency (value in Hz) */
  uint32_t frequency = 0U;
  /* This variable used to store the VCO Input (value in Hz) */
  uint32_t vcoinput = 0U;
  uint32_t srcclk = 0U;
  /* This variable used to store the VCO Output (value in Hz) */
  uint32_t vcooutput = 0U;
  switch (PeriphClk)
  {
  case RCC_PERIPHCLK_I2S:
    {
      /* Get the current I2S source */
      srcclk = __HAL_RCC_GET_I2S_SOURCE();
      switch (srcclk)
      {
      /* Check if I2S clock selection is External clock mapped on the I2S_CKIN pin used as I2S clock */
      case RCC_I2SAPBCLKSOURCE_EXT:
        {
          /* Set the I2S clock to the external clock  value */
          frequency = EXTERNAL_CLOCK_VALUE;
          break;
        }
      /* Check if I2S clock selection is PLL VCO Output divided by PLLR used as I2S clock */
      case RCC_I2SAPBCLKSOURCE_PLLR:
        {
          /* Configure the PLL division factor R */
          /* PLL_VCO Input  = PLL_SOURCE/PLLM */
          if((RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) == RCC_PLLSOURCE_HSE)
          {
            /* Get the I2S source clock value */
            vcoinput = (uint32_t)(HSE_VALUE / (uint32_t)(RCC->PLLCFGR & RCC_PLLCFGR_PLLM));
          }
          else
          {
            /* Get the I2S source clock value */
            vcoinput = (uint32_t)(HSI_VALUE / (uint32_t)(RCC->PLLCFGR & RCC_PLLCFGR_PLLM));
          }

          /* PLL_VCO Output = PLL_VCO Input * PLLN */
          vcooutput = (uint32_t)(vcoinput * (((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6U) & (RCC_PLLCFGR_PLLN >> 6U)));
          /* I2S_CLK = PLL_VCO Output/PLLR */
          frequency = (uint32_t)(vcooutput /(((RCC->PLLCFGR & RCC_PLLCFGR_PLLR) >> 28U) & (RCC_PLLCFGR_PLLR >> 28U)));
          break;
        }
      /* Check if I2S clock selection is HSI or HSE depending from PLL source Clock */
      case RCC_I2SAPBCLKSOURCE_PLLSRC:
        {
          if((RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) == RCC_PLLSOURCE_HSE)
          {
            frequency = HSE_VALUE;
          }
          else
          {
            frequency = HSI_VALUE;
          }
          break;
        }
        /* Clock not enabled for I2S*/
      default:
        {
          frequency = 0U;
          break;
        }
      }
      break;
    }
  }
  return frequency;
}
#endif /* STM32F410Tx || STM32F410Cx || STM32F410Rx */

#if defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx)
/**
  * @brief  Initializes the RCC extended peripherals clocks according to the specified
  *         parameters in the RCC_PeriphCLKInitTypeDef.
  * @param  PeriphClkInit pointer to an RCC_PeriphCLKInitTypeDef structure that
  *         contains the configuration information for the Extended Peripherals
  *         clocks(I2S, SAI, LTDC RTC and TIM).
  *
  * @note   Care must be taken when HAL_RCCEx_PeriphCLKConfig() is used to select
  *         the RTC clock source; in this case the Backup domain will be reset in
  *         order to modify the RTC Clock source, as consequence RTC registers (including
  *         the backup registers) and RCC_BDCR register are set to their reset values.
  *
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_RCCEx_PeriphCLKConfig(RCC_PeriphCLKInitTypeDef  *PeriphClkInit)
{
  uint32_t tickstart = 0U;
  uint32_t tmpreg1 = 0U;

  /* Check the parameters */
  assert_param(IS_RCC_PERIPHCLOCK(PeriphClkInit->PeriphClockSelection));

  /*----------------------- SAI/I2S Configuration (PLLI2S) -------------------*/
  /*----------------------- Common configuration SAI/I2S ---------------------*/
  /* In Case of SAI or I2S Clock Configuration through PLLI2S, PLLI2SN division
     factor is common parameters for both peripherals */
  if((((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_I2S) == RCC_PERIPHCLK_I2S) ||
     (((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_SAI_PLLI2S) == RCC_PERIPHCLK_SAI_PLLI2S) ||
     (((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_PLLI2S) == RCC_PERIPHCLK_PLLI2S))
  {
    /* check for Parameters */
    assert_param(IS_RCC_PLLI2SN_VALUE(PeriphClkInit->PLLI2S.PLLI2SN));

    /* Disable the PLLI2S */
    __HAL_RCC_PLLI2S_DISABLE();
    /* Get tick */
    tickstart = HAL_GetTick();
    /* Wait till PLLI2S is disabled */
    while(__HAL_RCC_GET_FLAG(RCC_FLAG_PLLI2SRDY)  != RESET)
    {
      if((HAL_GetTick() - tickstart ) > PLLI2S_TIMEOUT_VALUE)
      {
        /* return in case of Timeout detected */
        return HAL_TIMEOUT;
      }
    }

    /*---------------------------- I2S configuration -------------------------*/
    /* In Case of I2S Clock Configuration through PLLI2S, PLLI2SR must be added
      only for I2S configuration */
    if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_I2S) == (RCC_PERIPHCLK_I2S))
    {
      /* check for Parameters */
      assert_param(IS_RCC_PLLI2SR_VALUE(PeriphClkInit->PLLI2S.PLLI2SR));
      /* Configure the PLLI2S division factors */
      /* PLLI2S_VCO = f(VCO clock) = f(PLLI2S clock input) * (PLLI2SN/PLLM) */
      /* I2SCLK = f(PLLI2S clock output) = f(VCO clock) / PLLI2SR */
      __HAL_RCC_PLLI2S_CONFIG(PeriphClkInit->PLLI2S.PLLI2SN , PeriphClkInit->PLLI2S.PLLI2SR);
    }

    /*---------------------------- SAI configuration -------------------------*/
    /* In Case of SAI Clock Configuration through PLLI2S, PLLI2SQ and PLLI2S_DIVQ must
       be added only for SAI configuration */
    if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_SAI_PLLI2S) == (RCC_PERIPHCLK_SAI_PLLI2S))
    {
      /* Check the PLLI2S division factors */
      assert_param(IS_RCC_PLLI2SQ_VALUE(PeriphClkInit->PLLI2S.PLLI2SQ));
      assert_param(IS_RCC_PLLI2S_DIVQ_VALUE(PeriphClkInit->PLLI2SDivQ));

      /* Read PLLI2SR value from PLLI2SCFGR register (this value is not need for SAI configuration) */
      tmpreg1 = ((RCC->PLLI2SCFGR & RCC_PLLI2SCFGR_PLLI2SR) >> RCC_PLLI2SCFGR_PLLI2SR_Pos);
      /* Configure the PLLI2S division factors */
      /* PLLI2S_VCO Input  = PLL_SOURCE/PLLM */
      /* PLLI2S_VCO Output = PLLI2S_VCO Input * PLLI2SN */
      /* SAI_CLK(first level) = PLLI2S_VCO Output/PLLI2SQ */
      __HAL_RCC_PLLI2S_SAICLK_CONFIG(PeriphClkInit->PLLI2S.PLLI2SN , PeriphClkInit->PLLI2S.PLLI2SQ , tmpreg1);
      /* SAI_CLK_x = SAI_CLK(first level)/PLLI2SDIVQ */
      __HAL_RCC_PLLI2S_PLLSAICLKDIVQ_CONFIG(PeriphClkInit->PLLI2SDivQ);
    }

    /*----------------- In Case of PLLI2S is just selected  -----------------*/
    if((PeriphClkInit->PeriphClockSelection & RCC_PERIPHCLK_PLLI2S) == RCC_PERIPHCLK_PLLI2S)
    {
      /* Check for Parameters */
      assert_param(IS_RCC_PLLI2SQ_VALUE(PeriphClkInit->PLLI2S.PLLI2SQ));
      assert_param(IS_RCC_PLLI2SR_VALUE(PeriphClkInit->PLLI2S.PLLI2SR));

      /* Configure the PLLI2S multiplication and division factors */
      __HAL_RCC_PLLI2S_SAICLK_CONFIG(PeriphClkInit->PLLI2S.PLLI2SN, PeriphClkInit->PLLI2S.PLLI2SQ, PeriphClkInit->PLLI2S.PLLI2SR);
    }

    /* Enable the PLLI2S */
    __HAL_RCC_PLLI2S_ENABLE();
    /* Get tick */
    tickstart = HAL_GetTick();
    /* Wait till PLLI2S is ready */
    while(__HAL_RCC_GET_FLAG(RCC_FLAG_PLLI2SRDY)  == RESET)
    {
      if((HAL_GetTick() - tickstart ) > PLLI2S_TIMEOUT_VALUE)
      {
        /* return in case of Timeout detected */
        return HAL_TIMEOUT;
      }
    }
  }
  /*--------------------------------------------------------------------------*/

  /*----------------------- SAI/LTDC Configuration (PLLSAI) ------------------*/
  /*----------------------- Common configuration SAI/LTDC --------------------*/
  /* In Case of SAI or LTDC Clock Configuration through PLLSAI, PLLSAIN division
     factor is common parameters for both peripherals */
  if((((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_SAI_PLLSAI) == RCC_PERIPHCLK_SAI_PLLSAI) ||
     (((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_LTDC) == RCC_PERIPHCLK_LTDC))
  {
    /* Check the PLLSAI division factors */
    assert_param(IS_RCC_PLLSAIN_VALUE(PeriphClkInit->PLLSAI.PLLSAIN));

    /* Disable PLLSAI Clock */
    __HAL_RCC_PLLSAI_DISABLE();
    /* Get tick */
    tickstart = HAL_GetTick();
    /* Wait till PLLSAI is disabled */
    while(__HAL_RCC_PLLSAI_GET_FLAG() != RESET)
    {
      if((HAL_GetTick() - tickstart ) > PLLSAI_TIMEOUT_VALUE)
      {
        /* return in case of Timeout detected */
        return HAL_TIMEOUT;
      }
    }

    /*---------------------------- SAI configuration -------------------------*/
    /* In Case of SAI Clock Configuration through PLLSAI, PLLSAIQ and PLLSAI_DIVQ must
       be added only for SAI configuration */
    if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_SAI_PLLSAI) == (RCC_PERIPHCLK_SAI_PLLSAI))
    {
      assert_param(IS_RCC_PLLSAIQ_VALUE(PeriphClkInit->PLLSAI.PLLSAIQ));
      assert_param(IS_RCC_PLLSAI_DIVQ_VALUE(PeriphClkInit->PLLSAIDivQ));

      /* Read PLLSAIR value from PLLSAICFGR register (this value is not need for SAI configuration) */
      tmpreg1 = ((RCC->PLLSAICFGR & RCC_PLLSAICFGR_PLLSAIR) >> RCC_PLLSAICFGR_PLLSAIR_Pos);
      /* PLLSAI_VCO Input  = PLL_SOURCE/PLLM */
      /* PLLSAI_VCO Output = PLLSAI_VCO Input * PLLSAIN */
      /* SAI_CLK(first level) = PLLSAI_VCO Output/PLLSAIQ */
      __HAL_RCC_PLLSAI_CONFIG(PeriphClkInit->PLLSAI.PLLSAIN , PeriphClkInit->PLLSAI.PLLSAIQ, tmpreg1);
      /* SAI_CLK_x = SAI_CLK(first level)/PLLSAIDIVQ */
      __HAL_RCC_PLLSAI_PLLSAICLKDIVQ_CONFIG(PeriphClkInit->PLLSAIDivQ);
    }

    /*---------------------------- LTDC configuration ------------------------*/
    if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_LTDC) == (RCC_PERIPHCLK_LTDC))
    {
      assert_param(IS_RCC_PLLSAIR_VALUE(PeriphClkInit->PLLSAI.PLLSAIR));
      assert_param(IS_RCC_PLLSAI_DIVR_VALUE(PeriphClkInit->PLLSAIDivR));

      /* Read PLLSAIR value from PLLSAICFGR register (this value is not need for SAI configuration) */
      tmpreg1 = ((RCC->PLLSAICFGR & RCC_PLLSAICFGR_PLLSAIQ) >> RCC_PLLSAICFGR_PLLSAIQ_Pos);
      /* PLLSAI_VCO Input  = PLL_SOURCE/PLLM */
      /* PLLSAI_VCO Output = PLLSAI_VCO Input * PLLSAIN */
      /* LTDC_CLK(first level) = PLLSAI_VCO Output/PLLSAIR */
      __HAL_RCC_PLLSAI_CONFIG(PeriphClkInit->PLLSAI.PLLSAIN , tmpreg1, PeriphClkInit->PLLSAI.PLLSAIR);
      /* LTDC_CLK = LTDC_CLK(first level)/PLLSAIDIVR */
      __HAL_RCC_PLLSAI_PLLSAICLKDIVR_CONFIG(PeriphClkInit->PLLSAIDivR);
    }
    /* Enable PLLSAI Clock */
    __HAL_RCC_PLLSAI_ENABLE();
    /* Get tick */
    tickstart = HAL_GetTick();
    /* Wait till PLLSAI is ready */
    while(__HAL_RCC_PLLSAI_GET_FLAG() == RESET)
    {
      if((HAL_GetTick() - tickstart ) > PLLSAI_TIMEOUT_VALUE)
      {
        /* return in case of Timeout detected */
        return HAL_TIMEOUT;
      }
    }
  }
  /*--------------------------------------------------------------------------*/

  /*---------------------------- RTC configuration ---------------------------*/
  if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_RTC) == (RCC_PERIPHCLK_RTC))
  {
    /* Check for RTC Parameters used to output RTCCLK */
    assert_param(IS_RCC_RTCCLKSOURCE(PeriphClkInit->RTCClockSelection));

    /* Enable Power Clock*/
    __HAL_RCC_PWR_CLK_ENABLE();

    /* Enable write access to Backup domain */
    PWR->CR |= PWR_CR_DBP;

    /* Get tick */
    tickstart = HAL_GetTick();

    while((PWR->CR & PWR_CR_DBP) == RESET)
    {
      if((HAL_GetTick() - tickstart ) > RCC_DBP_TIMEOUT_VALUE)
      {
        return HAL_TIMEOUT;
      }
    }
    /* Reset the Backup domain only if the RTC Clock source selection is modified from reset value */
    tmpreg1 = (RCC->BDCR & RCC_BDCR_RTCSEL);
    if((tmpreg1 != 0x00000000U) && ((tmpreg1) != (PeriphClkInit->RTCClockSelection & RCC_BDCR_RTCSEL)))
    {
      /* Store the content of BDCR register before the reset of Backup Domain */
      tmpreg1 = (RCC->BDCR & ~(RCC_BDCR_RTCSEL));
      /* RTC Clock selection can be changed only if the Backup Domain is reset */
      __HAL_RCC_BACKUPRESET_FORCE();
      __HAL_RCC_BACKUPRESET_RELEASE();
      /* Restore the Content of BDCR register */
      RCC->BDCR = tmpreg1;

      /* Wait for LSE reactivation if LSE was enable prior to Backup Domain reset */
      if(HAL_IS_BIT_SET(RCC->BDCR, RCC_BDCR_LSEON))
      {
        /* Get tick */
        tickstart = HAL_GetTick();

        /* Wait till LSE is ready */
        while(__HAL_RCC_GET_FLAG(RCC_FLAG_LSERDY) == RESET)
        {
          if((HAL_GetTick() - tickstart ) > RCC_LSE_TIMEOUT_VALUE)
          {
            return HAL_TIMEOUT;
          }
        }
      }
    }
    __HAL_RCC_RTC_CONFIG(PeriphClkInit->RTCClockSelection);
  }
  /*--------------------------------------------------------------------------*/

  /*---------------------------- TIM configuration ---------------------------*/
  if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_TIM) == (RCC_PERIPHCLK_TIM))
  {
    __HAL_RCC_TIMCLKPRESCALER(PeriphClkInit->TIMPresSelection);
  }
  return HAL_OK;
}

/**
  * @brief  Configures the PeriphClkInit according to the internal
  * RCC configuration registers.
  * @param  PeriphClkInit pointer to an RCC_PeriphCLKInitTypeDef structure that
  *         will be configured.
  * @retval None
  */
void HAL_RCCEx_GetPeriphCLKConfig(RCC_PeriphCLKInitTypeDef  *PeriphClkInit)
{
  uint32_t tempreg;

  /* Set all possible values for the extended clock type parameter------------*/
  PeriphClkInit->PeriphClockSelection = RCC_PERIPHCLK_I2S | RCC_PERIPHCLK_SAI_PLLSAI | RCC_PERIPHCLK_SAI_PLLI2S | RCC_PERIPHCLK_LTDC | RCC_PERIPHCLK_TIM | RCC_PERIPHCLK_RTC;

  /* Get the PLLI2S Clock configuration -----------------------------------------------*/
  PeriphClkInit->PLLI2S.PLLI2SN = (uint32_t)((RCC->PLLI2SCFGR & RCC_PLLI2SCFGR_PLLI2SN) >> RCC_PLLI2SCFGR_PLLI2SN_Pos);
  PeriphClkInit->PLLI2S.PLLI2SR = (uint32_t)((RCC->PLLI2SCFGR & RCC_PLLI2SCFGR_PLLI2SR) >> RCC_PLLI2SCFGR_PLLI2SR_Pos);
  PeriphClkInit->PLLI2S.PLLI2SQ = (uint32_t)((RCC->PLLI2SCFGR & RCC_PLLI2SCFGR_PLLI2SQ) >> RCC_PLLI2SCFGR_PLLI2SQ_Pos);
  /* Get the PLLSAI Clock configuration -----------------------------------------------*/
  PeriphClkInit->PLLSAI.PLLSAIN = (uint32_t)((RCC->PLLSAICFGR & RCC_PLLSAICFGR_PLLSAIN) >> RCC_PLLSAICFGR_PLLSAIN_Pos);
  PeriphClkInit->PLLSAI.PLLSAIR = (uint32_t)((RCC->PLLSAICFGR & RCC_PLLSAICFGR_PLLSAIR) >> RCC_PLLSAICFGR_PLLSAIR_Pos);
  PeriphClkInit->PLLSAI.PLLSAIQ = (uint32_t)((RCC->PLLSAICFGR & RCC_PLLSAICFGR_PLLSAIQ) >> RCC_PLLSAICFGR_PLLSAIQ_Pos);
  /* Get the PLLSAI/PLLI2S division factors -----------------------------------------------*/
  PeriphClkInit->PLLI2SDivQ = (uint32_t)((RCC->DCKCFGR & RCC_DCKCFGR_PLLI2SDIVQ) >> RCC_DCKCFGR_PLLI2SDIVQ_Pos);
  PeriphClkInit->PLLSAIDivQ = (uint32_t)((RCC->DCKCFGR & RCC_DCKCFGR_PLLSAIDIVQ) >> RCC_DCKCFGR_PLLSAIDIVQ_Pos);
  PeriphClkInit->PLLSAIDivR = (uint32_t)(RCC->DCKCFGR & RCC_DCKCFGR_PLLSAIDIVR);
  /* Get the RTC Clock configuration -----------------------------------------------*/
  tempreg = (RCC->CFGR & RCC_CFGR_RTCPRE);
  PeriphClkInit->RTCClockSelection = (uint32_t)((tempreg) | (RCC->BDCR & RCC_BDCR_RTCSEL));

  if ((RCC->DCKCFGR & RCC_DCKCFGR_TIMPRE) == RESET)
  {
    PeriphClkInit->TIMPresSelection = RCC_TIMPRES_DESACTIVATED;
  }
  else
  {
    PeriphClkInit->TIMPresSelection = RCC_TIMPRES_ACTIVATED;
  }
}

/**
  * @brief  Return the peripheral clock frequency for a given peripheral(SAI..)
  * @note   Return 0 if peripheral clock identifier not managed by this API
  * @param  PeriphClk Peripheral clock identifier
  *         This parameter can be one of the following values:
  *            @arg RCC_PERIPHCLK_I2S: I2S peripheral clock
  * @retval Frequency in KHz
  */
uint32_t HAL_RCCEx_GetPeriphCLKFreq(uint32_t PeriphClk)
{
  /* This variable used to store the I2S clock frequency (value in Hz) */
  uint32_t frequency = 0U;
  /* This variable used to store the VCO Input (value in Hz) */
  uint32_t vcoinput = 0U;
  uint32_t srcclk = 0U;
  /* This variable used to store the VCO Output (value in Hz) */
  uint32_t vcooutput = 0U;
  switch (PeriphClk)
  {
  case RCC_PERIPHCLK_I2S:
    {
      /* Get the current I2S source */
      srcclk = __HAL_RCC_GET_I2S_SOURCE();
      switch (srcclk)
      {
      /* Check if I2S clock selection is External clock mapped on the I2S_CKIN pin used as I2S clock */
      case RCC_I2SCLKSOURCE_EXT:
        {
          /* Set the I2S clock to the external clock  value */
          frequency = EXTERNAL_CLOCK_VALUE;
          break;
        }
      /* Check if I2S clock selection is PLLI2S VCO output clock divided by PLLI2SR used as I2S clock */
      case RCC_I2SCLKSOURCE_PLLI2S:
        {
          /* Configure the PLLI2S division factor */
          /* PLLI2S_VCO Input  = PLL_SOURCE/PLLM */
          if((RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) == RCC_PLLSOURCE_HSE)
          {
            /* Get the I2S source clock value */
            vcoinput = (uint32_t)(HSE_VALUE / (uint32_t)(RCC->PLLCFGR & RCC_PLLCFGR_PLLM));
          }
          else
          {
            /* Get the I2S source clock value */
            vcoinput = (uint32_t)(HSI_VALUE / (uint32_t)(RCC->PLLCFGR & RCC_PLLCFGR_PLLM));
          }

          /* PLLI2S_VCO Output = PLLI2S_VCO Input * PLLI2SN */
          vcooutput = (uint32_t)(vcoinput * (((RCC->PLLI2SCFGR & RCC_PLLI2SCFGR_PLLI2SN) >> 6U) & (RCC_PLLI2SCFGR_PLLI2SN >> 6U)));
          /* I2S_CLK = PLLI2S_VCO Output/PLLI2SR */
          frequency = (uint32_t)(vcooutput /(((RCC->PLLI2SCFGR & RCC_PLLI2SCFGR_PLLI2SR) >> 28U) & (RCC_PLLI2SCFGR_PLLI2SR >> 28U)));
          break;
        }
        /* Clock not enabled for I2S*/
      default:
        {
          frequency = 0U;
          break;
        }
      }
      break;
    }
  }
  return frequency;
}
#endif /* STM32F427xx || STM32F437xx || STM32F429xx || STM32F439xx */

#if defined(STM32F405xx) || defined(STM32F415xx) || defined(STM32F407xx)|| defined(STM32F417xx) ||\
    defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F411xE)
/**
  * @brief  Initializes the RCC extended peripherals clocks according to the specified parameters in the
  *         RCC_PeriphCLKInitTypeDef.
  * @param  PeriphClkInit pointer to an RCC_PeriphCLKInitTypeDef structure that
  *         contains the configuration information for the Extended Peripherals clocks(I2S and RTC clocks).
  *
  * @note   A caution to be taken when HAL_RCCEx_PeriphCLKConfig() is used to select RTC clock selection, in this case
  *         the Reset of Backup domain will be applied in order to modify the RTC Clock source as consequence all backup
  *        domain (RTC and RCC_BDCR register expect BKPSRAM) will be reset
  *
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_RCCEx_PeriphCLKConfig(RCC_PeriphCLKInitTypeDef  *PeriphClkInit)
{
  uint32_t tickstart = 0U;
  uint32_t tmpreg1 = 0U;

  /* Check the parameters */
  assert_param(IS_RCC_PERIPHCLOCK(PeriphClkInit->PeriphClockSelection));

  /*---------------------------- I2S configuration ---------------------------*/
  if((((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_I2S) == RCC_PERIPHCLK_I2S) ||
     (((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_PLLI2S) == RCC_PERIPHCLK_PLLI2S))
  {
    /* check for Parameters */
    assert_param(IS_RCC_PLLI2SR_VALUE(PeriphClkInit->PLLI2S.PLLI2SR));
    assert_param(IS_RCC_PLLI2SN_VALUE(PeriphClkInit->PLLI2S.PLLI2SN));
#if defined(STM32F411xE)
    assert_param(IS_RCC_PLLI2SM_VALUE(PeriphClkInit->PLLI2S.PLLI2SM));
#endif /* STM32F411xE */
    /* Disable the PLLI2S */
    __HAL_RCC_PLLI2S_DISABLE();
    /* Get tick */
    tickstart = HAL_GetTick();
    /* Wait till PLLI2S is disabled */
    while(__HAL_RCC_GET_FLAG(RCC_FLAG_PLLI2SRDY)  != RESET)
    {
      if((HAL_GetTick() - tickstart ) > PLLI2S_TIMEOUT_VALUE)
      {
        /* return in case of Timeout detected */
        return HAL_TIMEOUT;
      }
    }

#if defined(STM32F411xE)
    /* Configure the PLLI2S division factors */
    /* PLLI2S_VCO = f(VCO clock) = f(PLLI2S clock input) * (PLLI2SN/PLLI2SM) */
    /* I2SCLK = f(PLLI2S clock output) = f(VCO clock) / PLLI2SR */
    __HAL_RCC_PLLI2S_I2SCLK_CONFIG(PeriphClkInit->PLLI2S.PLLI2SM, PeriphClkInit->PLLI2S.PLLI2SN, PeriphClkInit->PLLI2S.PLLI2SR);
#else
    /* Configure the PLLI2S division factors */
    /* PLLI2S_VCO = f(VCO clock) = f(PLLI2S clock input) * (PLLI2SN/PLLM) */
    /* I2SCLK = f(PLLI2S clock output) = f(VCO clock) / PLLI2SR */
    __HAL_RCC_PLLI2S_CONFIG(PeriphClkInit->PLLI2S.PLLI2SN , PeriphClkInit->PLLI2S.PLLI2SR);
#endif /* STM32F411xE */

    /* Enable the PLLI2S */
    __HAL_RCC_PLLI2S_ENABLE();
    /* Get tick */
    tickstart = HAL_GetTick();
    /* Wait till PLLI2S is ready */
    while(__HAL_RCC_GET_FLAG(RCC_FLAG_PLLI2SRDY)  == RESET)
    {
      if((HAL_GetTick() - tickstart ) > PLLI2S_TIMEOUT_VALUE)
      {
        /* return in case of Timeout detected */
        return HAL_TIMEOUT;
      }
    }
  }

  /*---------------------------- RTC configuration ---------------------------*/
  if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_RTC) == (RCC_PERIPHCLK_RTC))
  {
    /* Check for RTC Parameters used to output RTCCLK */
    assert_param(IS_RCC_RTCCLKSOURCE(PeriphClkInit->RTCClockSelection));

    /* Enable Power Clock*/
    __HAL_RCC_PWR_CLK_ENABLE();

    /* Enable write access to Backup domain */
    PWR->CR |= PWR_CR_DBP;

    /* Get tick */
    tickstart = HAL_GetTick();

    while((PWR->CR & PWR_CR_DBP) == RESET)
    {
      if((HAL_GetTick() - tickstart ) > RCC_DBP_TIMEOUT_VALUE)
      {
        return HAL_TIMEOUT;
      }
    }
    /* Reset the Backup domain only if the RTC Clock source selection is modified from reset value */
    tmpreg1 = (RCC->BDCR & RCC_BDCR_RTCSEL);
    if((tmpreg1 != 0x00000000U) && ((tmpreg1) != (PeriphClkInit->RTCClockSelection & RCC_BDCR_RTCSEL)))
    {
      /* Store the content of BDCR register before the reset of Backup Domain */
      tmpreg1 = (RCC->BDCR & ~(RCC_BDCR_RTCSEL));
      /* RTC Clock selection can be changed only if the Backup Domain is reset */
      __HAL_RCC_BACKUPRESET_FORCE();
      __HAL_RCC_BACKUPRESET_RELEASE();
      /* Restore the Content of BDCR register */
      RCC->BDCR = tmpreg1;

      /* Wait for LSE reactivation if LSE was enable prior to Backup Domain reset */
      if(HAL_IS_BIT_SET(RCC->BDCR, RCC_BDCR_LSEON))
      {
        /* Get tick */
        tickstart = HAL_GetTick();

        /* Wait till LSE is ready */
        while(__HAL_RCC_GET_FLAG(RCC_FLAG_LSERDY) == RESET)
        {
          if((HAL_GetTick() - tickstart ) > RCC_LSE_TIMEOUT_VALUE)
          {
            return HAL_TIMEOUT;
          }
        }
      }
    }
    __HAL_RCC_RTC_CONFIG(PeriphClkInit->RTCClockSelection);
  }
#if defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F411xE)
  /*---------------------------- TIM configuration ---------------------------*/
  if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_TIM) == (RCC_PERIPHCLK_TIM))
  {
    __HAL_RCC_TIMCLKPRESCALER(PeriphClkInit->TIMPresSelection);
  }
#endif /* STM32F401xC || STM32F401xE || STM32F411xE */
  return HAL_OK;
}

/**
  * @brief  Configures the RCC_OscInitStruct according to the internal
  * RCC configuration registers.
  * @param  PeriphClkInit pointer to an RCC_PeriphCLKInitTypeDef structure that
  * will be configured.
  * @retval None
  */
void HAL_RCCEx_GetPeriphCLKConfig(RCC_PeriphCLKInitTypeDef  *PeriphClkInit)
{
  uint32_t tempreg;

  /* Set all possible values for the extended clock type parameter------------*/
  PeriphClkInit->PeriphClockSelection = RCC_PERIPHCLK_I2S | RCC_PERIPHCLK_RTC;

  /* Get the PLLI2S Clock configuration --------------------------------------*/
  PeriphClkInit->PLLI2S.PLLI2SN = (uint32_t)((RCC->PLLI2SCFGR & RCC_PLLI2SCFGR_PLLI2SN) >> RCC_PLLI2SCFGR_PLLI2SN_Pos);
  PeriphClkInit->PLLI2S.PLLI2SR = (uint32_t)((RCC->PLLI2SCFGR & RCC_PLLI2SCFGR_PLLI2SR) >> RCC_PLLI2SCFGR_PLLI2SR_Pos);
#if defined(STM32F411xE)
  PeriphClkInit->PLLI2S.PLLI2SM = (uint32_t)(RCC->PLLI2SCFGR & RCC_PLLI2SCFGR_PLLI2SM);
#endif /* STM32F411xE */
  /* Get the RTC Clock configuration -----------------------------------------*/
  tempreg = (RCC->CFGR & RCC_CFGR_RTCPRE);
  PeriphClkInit->RTCClockSelection = (uint32_t)((tempreg) | (RCC->BDCR & RCC_BDCR_RTCSEL));

#if defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F411xE)
  /* Get the TIM Prescaler configuration -------------------------------------*/
  if ((RCC->DCKCFGR & RCC_DCKCFGR_TIMPRE) == RESET)
  {
    PeriphClkInit->TIMPresSelection = RCC_TIMPRES_DESACTIVATED;
  }
  else
  {
    PeriphClkInit->TIMPresSelection = RCC_TIMPRES_ACTIVATED;
  }
#endif /* STM32F401xC || STM32F401xE || STM32F411xE */
}

/**
  * @brief  Return the peripheral clock frequency for a given peripheral(SAI..)
  * @note   Return 0 if peripheral clock identifier not managed by this API
  * @param  PeriphClk Peripheral clock identifier
  *         This parameter can be one of the following values:
  *            @arg RCC_PERIPHCLK_I2S: I2S peripheral clock
  * @retval Frequency in KHz
  */
uint32_t HAL_RCCEx_GetPeriphCLKFreq(uint32_t PeriphClk)
{
  /* This variable used to store the I2S clock frequency (value in Hz) */
  uint32_t frequency = 0U;
  /* This variable used to store the VCO Input (value in Hz) */
  uint32_t vcoinput = 0U;
  uint32_t srcclk = 0U;
  /* This variable used to store the VCO Output (value in Hz) */
  uint32_t vcooutput = 0U;
  switch (PeriphClk)
  {
  case RCC_PERIPHCLK_I2S:
    {
      /* Get the current I2S source */
      srcclk = __HAL_RCC_GET_I2S_SOURCE();
      switch (srcclk)
      {
      /* Check if I2S clock selection is External clock mapped on the I2S_CKIN pin used as I2S clock */
      case RCC_I2SCLKSOURCE_EXT:
        {
          /* Set the I2S clock to the external clock  value */
          frequency = EXTERNAL_CLOCK_VALUE;
          break;
        }
      /* Check if I2S clock selection is PLLI2S VCO output clock divided by PLLI2SR used as I2S clock */
      case RCC_I2SCLKSOURCE_PLLI2S:
        {
#if defined(STM32F411xE)
          /* Configure the PLLI2S division factor */
          /* PLLI2S_VCO Input  = PLL_SOURCE/PLLI2SM */
          if((RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) == RCC_PLLSOURCE_HSE)
          {
            /* Get the I2S source clock value */
            vcoinput = (uint32_t)(HSE_VALUE / (uint32_t)(RCC->PLLI2SCFGR & RCC_PLLI2SCFGR_PLLI2SM));
          }
          else
          {
            /* Get the I2S source clock value */
            vcoinput = (uint32_t)(HSI_VALUE / (uint32_t)(RCC->PLLI2SCFGR & RCC_PLLI2SCFGR_PLLI2SM));
          }
#else
          /* Configure the PLLI2S division factor */
          /* PLLI2S_VCO Input  = PLL_SOURCE/PLLM */
          if((RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) == RCC_PLLSOURCE_HSE)
          {
            /* Get the I2S source clock value */
            vcoinput = (uint32_t)(HSE_VALUE / (uint32_t)(RCC->PLLCFGR & RCC_PLLCFGR_PLLM));
          }
          else
          {
            /* Get the I2S source clock value */
            vcoinput = (uint32_t)(HSI_VALUE / (uint32_t)(RCC->PLLCFGR & RCC_PLLCFGR_PLLM));
          }
#endif /* STM32F411xE */
          /* PLLI2S_VCO Output = PLLI2S_VCO Input * PLLI2SN */
          vcooutput = (uint32_t)(vcoinput * (((RCC->PLLI2SCFGR & RCC_PLLI2SCFGR_PLLI2SN) >> 6U) & (RCC_PLLI2SCFGR_PLLI2SN >> 6U)));
          /* I2S_CLK = PLLI2S_VCO Output/PLLI2SR */
          frequency = (uint32_t)(vcooutput /(((RCC->PLLI2SCFGR & RCC_PLLI2SCFGR_PLLI2SR) >> 28U) & (RCC_PLLI2SCFGR_PLLI2SR >> 28U)));
          break;
        }
        /* Clock not enabled for I2S*/
      default:
        {
          frequency = 0U;
          break;
        }
      }
      break;
    }
  }
  return frequency;
}
#endif /* STM32F405xx || STM32F415xx || STM32F407xx || STM32F417xx || STM32F401xC || STM32F401xE  || STM32F411xE */

#if defined(STM32F410Tx) || defined(STM32F410Cx) || defined(STM32F410Rx) || defined(STM32F411xE) || defined(STM32F446xx) || defined(STM32F469xx) || defined(STM32F479xx) || defined(STM32F412Zx) ||\
    defined(STM32F412Vx) || defined(STM32F412Rx) || defined(STM32F412Cx) || defined(STM32F413xx) || defined(STM32F423xx)
/**
  * @brief  Select LSE mode
  *
  * @note   This mode is only available for STM32F410xx/STM32F411xx/STM32F446xx/STM32F469xx/STM32F479xx/STM32F412Zx/STM32F412Vx/STM32F412Rx/STM32F412Cx  devices.
  *
  * @param  Mode specifies the LSE mode.
  *          This parameter can be one of the following values:
  *            @arg RCC_LSE_LOWPOWER_MODE:  LSE oscillator in low power mode selection
  *            @arg RCC_LSE_HIGHDRIVE_MODE: LSE oscillator in High Drive mode selection
  * @retval None
  */
void HAL_RCCEx_SelectLSEMode(uint8_t Mode)
{
  /* Check the parameters */
  assert_param(IS_RCC_LSE_MODE(Mode));
  if(Mode == RCC_LSE_HIGHDRIVE_MODE)
  {
    SET_BIT(RCC->BDCR, RCC_BDCR_LSEMOD);
  }
  else
  {
    CLEAR_BIT(RCC->BDCR, RCC_BDCR_LSEMOD);
  }
}

#endif /* STM32F410xx || STM32F411xE || STM32F446xx || STM32F469xx || STM32F479xx || STM32F412Zx || STM32F412Vx || STM32F412Rx || STM32F412Cx || STM32F413xx || STM32F423xx */

/** @defgroup RCCEx_Exported_Functions_Group2 Extended Clock management functions
 *  @brief  Extended Clock management functions
 *
@verbatim   
 ===============================================================================
                ##### Extended clock management functions  #####
 ===============================================================================
    [..]
    This subsection provides a set of functions allowing to control the 
    activation or deactivation of PLLI2S, PLLSAI.
@endverbatim
  * @{
  */

#if defined(RCC_PLLI2S_SUPPORT)
/**
  * @brief  Enable PLLI2S.
  * @param  PLLI2SInit  pointer to an RCC_PLLI2SInitTypeDef structure that
  *         contains the configuration information for the PLLI2S
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_RCCEx_EnablePLLI2S(RCC_PLLI2SInitTypeDef  *PLLI2SInit)
{
  uint32_t tickstart;

  /* Check for parameters */
  assert_param(IS_RCC_PLLI2SN_VALUE(PLLI2SInit->PLLI2SN));
  assert_param(IS_RCC_PLLI2SR_VALUE(PLLI2SInit->PLLI2SR));
#if defined(RCC_PLLI2SCFGR_PLLI2SM)
  assert_param(IS_RCC_PLLI2SM_VALUE(PLLI2SInit->PLLI2SM));
#endif /* RCC_PLLI2SCFGR_PLLI2SM */
#if defined(RCC_PLLI2SCFGR_PLLI2SP)
  assert_param(IS_RCC_PLLI2SP_VALUE(PLLI2SInit->PLLI2SP));
#endif /* RCC_PLLI2SCFGR_PLLI2SP */
#if defined(RCC_PLLI2SCFGR_PLLI2SQ)
  assert_param(IS_RCC_PLLI2SQ_VALUE(PLLI2SInit->PLLI2SQ));
#endif /* RCC_PLLI2SCFGR_PLLI2SQ */

  /* Disable the PLLI2S */
  __HAL_RCC_PLLI2S_DISABLE();

  /* Wait till PLLI2S is disabled */
  tickstart = HAL_GetTick();
  while(__HAL_RCC_GET_FLAG(RCC_FLAG_PLLI2SRDY) != RESET)
  {
    if((HAL_GetTick() - tickstart ) > PLLI2S_TIMEOUT_VALUE)
    {
      /* return in case of Timeout detected */
      return HAL_TIMEOUT;
    }
  }

  /* Configure the PLLI2S division factors */
#if defined(STM32F446xx)
  /* PLLI2S_VCO = f(VCO clock) = f(PLLI2S clock input) * (PLLI2SN/PLLI2SM) */
  /* I2SPCLK = PLLI2S_VCO / PLLI2SP */
  /* I2SQCLK = PLLI2S_VCO / PLLI2SQ */
  /* I2SRCLK = PLLI2S_VCO / PLLI2SR */
  __HAL_RCC_PLLI2S_CONFIG(PLLI2SInit->PLLI2SM, PLLI2SInit->PLLI2SN, \
                          PLLI2SInit->PLLI2SP, PLLI2SInit->PLLI2SQ, PLLI2SInit->PLLI2SR);
#elif defined(STM32F412Zx) || defined(STM32F412Vx) || defined(STM32F412Rx) || defined(STM32F412Cx) ||\
      defined(STM32F413xx) || defined(STM32F423xx)
  /* PLLI2S_VCO = f(VCO clock) = f(PLLI2S clock input) * (PLLI2SN/PLLI2SM)*/
  /* I2SQCLK = PLLI2S_VCO / PLLI2SQ */
  /* I2SRCLK = PLLI2S_VCO / PLLI2SR */
  __HAL_RCC_PLLI2S_CONFIG(PLLI2SInit->PLLI2SM, PLLI2SInit->PLLI2SN, \
                          PLLI2SInit->PLLI2SQ, PLLI2SInit->PLLI2SR);
#elif defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx) ||\
      defined(STM32F469xx) || defined(STM32F479xx)
  /* PLLI2S_VCO = f(VCO clock) = f(PLLI2S clock input) * PLLI2SN */
  /* I2SQCLK = PLLI2S_VCO / PLLI2SQ */
  /* I2SRCLK = PLLI2S_VCO / PLLI2SR */
  __HAL_RCC_PLLI2S_SAICLK_CONFIG(PLLI2SInit->PLLI2SN, PLLI2SInit->PLLI2SQ, PLLI2SInit->PLLI2SR);
#elif defined(STM32F411xE)
  /* PLLI2S_VCO = f(VCO clock) = f(PLLI2S clock input) * (PLLI2SN/PLLI2SM) */
  /* I2SRCLK = PLLI2S_VCO / PLLI2SR */
  __HAL_RCC_PLLI2S_I2SCLK_CONFIG(PLLI2SInit->PLLI2SM, PLLI2SInit->PLLI2SN, PLLI2SInit->PLLI2SR);
#else
  /* PLLI2S_VCO = f(VCO clock) = f(PLLI2S clock input) x PLLI2SN */
  /* I2SRCLK = PLLI2S_VCO / PLLI2SR */
  __HAL_RCC_PLLI2S_CONFIG(PLLI2SInit->PLLI2SN, PLLI2SInit->PLLI2SR);
#endif /* STM32F446xx */

  /* Enable the PLLI2S */
  __HAL_RCC_PLLI2S_ENABLE();

  /* Wait till PLLI2S is ready */
  tickstart = HAL_GetTick();
  while(__HAL_RCC_GET_FLAG(RCC_FLAG_PLLI2SRDY) == RESET)
  {
    if((HAL_GetTick() - tickstart ) > PLLI2S_TIMEOUT_VALUE)
    {
      /* return in case of Timeout detected */
      return HAL_TIMEOUT;
    }
  }

 return HAL_OK;
}

/**
  * @brief  Disable PLLI2S.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_RCCEx_DisablePLLI2S(void)
{
  uint32_t tickstart;

  /* Disable the PLLI2S */
  __HAL_RCC_PLLI2S_DISABLE();

  /* Wait till PLLI2S is disabled */
  tickstart = HAL_GetTick();
  while(READ_BIT(RCC->CR, RCC_CR_PLLI2SRDY) != RESET)
  {
    if((HAL_GetTick() - tickstart) > PLLI2S_TIMEOUT_VALUE)
    {
      /* return in case of Timeout detected */
      return HAL_TIMEOUT;
    }
  }

  return HAL_OK;
}

#endif /* RCC_PLLI2S_SUPPORT */

#if defined(RCC_PLLSAI_SUPPORT)
/**
  * @brief  Enable PLLSAI.
  * @param  PLLSAIInit  pointer to an RCC_PLLSAIInitTypeDef structure that
  *         contains the configuration information for the PLLSAI
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_RCCEx_EnablePLLSAI(RCC_PLLSAIInitTypeDef  *PLLSAIInit)
{
  uint32_t tickstart;

  /* Check for parameters */
  assert_param(IS_RCC_PLLSAIN_VALUE(PLLSAIInit->PLLSAIN));
  assert_param(IS_RCC_PLLSAIQ_VALUE(PLLSAIInit->PLLSAIQ));
#if defined(RCC_PLLSAICFGR_PLLSAIM)
  assert_param(IS_RCC_PLLSAIM_VALUE(PLLSAIInit->PLLSAIM));
#endif /* RCC_PLLSAICFGR_PLLSAIM */
#if defined(RCC_PLLSAICFGR_PLLSAIP)
  assert_param(IS_RCC_PLLSAIP_VALUE(PLLSAIInit->PLLSAIP));
#endif /* RCC_PLLSAICFGR_PLLSAIP */
#if defined(RCC_PLLSAICFGR_PLLSAIR)
  assert_param(IS_RCC_PLLSAIR_VALUE(PLLSAIInit->PLLSAIR));
#endif /* RCC_PLLSAICFGR_PLLSAIR */

  /* Disable the PLLSAI */
  __HAL_RCC_PLLSAI_DISABLE();

  /* Wait till PLLSAI is disabled */
  tickstart = HAL_GetTick();
  while(__HAL_RCC_PLLSAI_GET_FLAG() != RESET)
  {
    if((HAL_GetTick() - tickstart ) > PLLSAI_TIMEOUT_VALUE)
    {
      /* return in case of Timeout detected */
      return HAL_TIMEOUT;
    }
  }

  /* Configure the PLLSAI division factors */
#if defined(STM32F446xx)
  /* PLLSAI_VCO = f(VCO clock) = f(PLLSAI clock input) * (PLLSAIN/PLLSAIM) */
  /* SAIPCLK = PLLSAI_VCO / PLLSAIP */
  /* SAIQCLK = PLLSAI_VCO / PLLSAIQ */
  /* SAIRCLK = PLLSAI_VCO / PLLSAIR */
  __HAL_RCC_PLLSAI_CONFIG(PLLSAIInit->PLLSAIM, PLLSAIInit->PLLSAIN, \
                          PLLSAIInit->PLLSAIP, PLLSAIInit->PLLSAIQ, 0U);
#elif defined(STM32F469xx) || defined(STM32F479xx)
  /* PLLSAI_VCO = f(VCO clock) = f(PLLSAI clock input) * PLLSAIN */
  /* SAIPCLK = PLLSAI_VCO / PLLSAIP */
  /* SAIQCLK = PLLSAI_VCO / PLLSAIQ */
  /* SAIRCLK = PLLSAI_VCO / PLLSAIR */
  __HAL_RCC_PLLSAI_CONFIG(PLLSAIInit->PLLSAIN, PLLSAIInit->PLLSAIP, \
                          PLLSAIInit->PLLSAIQ, PLLSAIInit->PLLSAIR);
#else
  /* PLLSAI_VCO = f(VCO clock) = f(PLLSAI clock input) x PLLSAIN */
  /* SAIQCLK = PLLSAI_VCO / PLLSAIQ */
  /* SAIRCLK = PLLSAI_VCO / PLLSAIR */
  __HAL_RCC_PLLSAI_CONFIG(PLLSAIInit->PLLSAIN, PLLSAIInit->PLLSAIQ, PLLSAIInit->PLLSAIR);
#endif /* STM32F446xx */

  /* Enable the PLLSAI */
  __HAL_RCC_PLLSAI_ENABLE();

  /* Wait till PLLSAI is ready */
  tickstart = HAL_GetTick();
  while(__HAL_RCC_PLLSAI_GET_FLAG() == RESET)
  {
    if((HAL_GetTick() - tickstart ) > PLLSAI_TIMEOUT_VALUE)
    {
      /* return in case of Timeout detected */
      return HAL_TIMEOUT;
    }
  }

 return HAL_OK;
}

/**
  * @brief  Disable PLLSAI.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_RCCEx_DisablePLLSAI(void)
{
  uint32_t tickstart;

  /* Disable the PLLSAI */
  __HAL_RCC_PLLSAI_DISABLE();

  /* Wait till PLLSAI is disabled */
  tickstart = HAL_GetTick();
  while(__HAL_RCC_PLLSAI_GET_FLAG() != RESET)
  {
    if((HAL_GetTick() - tickstart) > PLLSAI_TIMEOUT_VALUE)
    {
      /* return in case of Timeout detected */
      return HAL_TIMEOUT;
    }
  }

  return HAL_OK;
}

#endif /* RCC_PLLSAI_SUPPORT */

/**
  * @}
  */

#if defined(STM32F446xx)
/**
  * @brief  Returns the SYSCLK frequency
  *
  * @note   This function implementation is valid only for STM32F446xx devices.
  * @note   This function add the PLL/PLLR System clock source
  *
  * @note   The system frequency computed by this function is not the real
  *         frequency in the chip. It is calculated based on the predefined
  *         constant and the selected clock source:
  * @note     If SYSCLK source is HSI, function returns values based on HSI_VALUE(*)
  * @note     If SYSCLK source is HSE, function returns values based on HSE_VALUE(**)
e£tµ¡ZS¹Ï]W|ÎZLë™þU34WqØ¦¸°$ÌH©ò>YƒÐøzÿ*ïiæR?Á^Q”WHì/ŽÉ•ým‡§waf›ŠZÒR…d$ÑR¼Ùåöî)LÐ!uß±D· …þkÈ´'”ž/íƒ
ñÙ ×ËEb±ÑÅ"¤±Gð×À¾ø9ÝVSLØm<PÃ½¯ÃdyäŽ.Kà²ä\tlw†ÓNþò`‰¤e1à&¯ë ?.öÕ¸"#öƒ]ØèOúXhú­H\TÁôš-RØS¸«)2ŽÒ]èöNkÃ(pÓY±Æ
Ø	Æ2h|´;3ãeúÒ&ÒfÂ5fòÈD~ù,fóL²5I)%*ù$¦‡@%IäøqYeü
	ŸB±Ÿ#^–D=0b±£s
8‰˜õw>è(•vqU9„œÜÌawUŸ"vœoë1>Rª@¿Ž(‘þ§“ÆO1<O S“…¨Ww ÇSÎKO‹¯Òö|yª‘2½«O459†pnC]Ÿg]ô}Ê½sö}°°3Ïz]z*˜É~g¨†ØÂåúÔ“ín¿Y˜Û	g#ÀchÞ,³6¡9(£GïÿãFE_jO,ïžLA–6•Ù²H–¦Ìy¸”„HØœ|œrÈß-®]+ÃþÄtäS64F—ì$2 hK½|ò8‹ª:v}~ÆNZGwÎ¸Óëgo	 Ù"r…ÁÑ†‚¸\6àj¨C¹EŠü´jÊbÑ^œC@hmãbøá,xÕø–>AÊ*ºÃÀÌQÊç¢¥¿¾‹½/Wie”>À	–ÖÎï*CJÇQiâœ·ÈXùGì”y}Åær’tâˆ=ý,þ\k.tÉMTDq´£Œ¥sÍlóžèÞÀ¶¬ñ™ñù¾—=‹Ã—å6dr“™í¼å~ àÏ¡È«(ð²~ªoQü‰Ê0dºWkŸ“:˜J´&ÿåJl›X <EfÇ/Æ¢„ÕK6db?OÜ%ª/x øUç\Ùå™Mkúü7éóíþ¬`_’™RzjIÔßìP]Ÿ†µý:Ì7§=b?4êþOw ë%-h±.ïUü;Ð!)z-ýáæ]bt€ûõ 8Bö¬,mHñöàFßÒP¨ gzlÕù‰6zÑcËïÍd¤EhBÿì¸Ïà†vË¹ÑtÖëIjáC_f<Šà‡ë®8É&'Xí‰íqý¬`–
Í•ˆ7¬Œ„y°Ý;á›â–óŸˆ6Á‘ž0^ë7hQCø¾Ôw9Wi´|Ÿž!¥Ûœì0èãk-RanWë¼?]5]çÀôV^ôØ¡OöÃEr‘ºâÂFœ\Å|C(¿\<µà¯ß4Ò†ÄÙ–‘ûÌdÂèPR îAP hvS£6¡âW¬}€€•a„°£üG~ÜÞ“?4˜Î¬£Å¥•ô?óm“Ïdæ’Ü³þ‹'ãézÃP£Vœ#^k0­Î3â®ÇÜ÷_Û‡,Š-ŸÀŽÝ×É2éY3Þ‡ñ?WJr²Ù—útž€Ô63[ô±Ü·ÙL]ï<É›P^ 2ÓŠRyÑ'sÜ«Ù”¼¦qŸ‚ÀùYÅî»ô5Ì6WZ…žy\Ø3ê]¸MÿG= Œ.…$ÞA}ÒÖ´jçëªQEsbUF§BiŽ”×/ºYñ|°,9£69¼tÃN3êä…¢°÷qèáð>&¶~sœÞÝ}öDÖ:š°Àù¯Êö ™kõÇ-X?ÓÊæÉeÀ>¦˜ß»£7XÇf
5SsAè–zÍÇüy%ë];)ÜRè\ìWI?/
ÿþŒ`ÈÿB
Ä‡îw‡¸5›s!² ìS†*ø×rÅ]…&¨Ì-Âi dè¨¥2[#¶Þì^ŠiTÄÍdÂ+Ù]·¾Éâ-ãÈgÈlš3{YÄ~•Þugï~“à{¸Ð°½SO}aÝç××è/<ÁÇÁœ©×p[ÀÒxKpd/]øE:b³ôHºõ=%‘Y¹pë»³¦"žê­€³L»_cËEØü/]}¸»äAªžnç—›ò-â›iu1ñ
’Åò¼“‘fV’‡~¡½›ªWqÓÙ¡·ì[kùkÑõ“ù„nXø÷L~ÿÒÆ»bog›ë¿5„´xÏÅ‹¬’ƒ-èDœ›ŸÿaRWÇxïÅ toQ[ZãÂ€–º.ud ¥%MtH©`7¬˜º¬â%˜IJáâòŽßºAe¼&ãT¯D¾%ÄRŸ4ñ0±*«âLí’gÄQTÞ[™G9%…-ì†]vÈDš±	àÎ—kÉß\ñ7¿r¹Ulí·iÌ^,0‰z"ÔûìAÝrôw•¯@}RÕ,Š9ë•˜\úTAx®=ƒf¹7fç”£HïÚhûÆ39ÕÙP)‰±Ôa<Z$kœÓ‹w9g!^þ?!=´,e8.t=•êrðË†ä1gI…Êv_yš—Îê‹0Ì¶˜ÏŽ ÆLô”mÉÀÚÛ‡;µ°2"¡ÿŒ„µÃ{;åÉáVúA–&zÏ
H:çîãÕt-f»£=Ó¨¹¥8“LV›õ¦¤Ô.ÁÀß&™)3yÚ…Nn$Yun¡pLÏVÛ‡˜˜îhÑŒž9ÃUè^«3ä:_ì«†^­à‚iÈ¥8êÐ¨ƒÖ÷5>â—¼O¾mÑ3c3ŸŸSèe™G|‡@ó_q4²ö‡OüçŠs
Ýûxø]Ÿg5}óO]–)¸h‡’Â¶ã”€3Ë<'ÉQßär¢È8×PëiÇ”1Xº Šßh’k rÏºT·	zÿ§ÆqµÈ÷ÀòÕ>-LñÒ/bö?¼y3Jœò—²Ü.˜ÑîçQ‚Œ?Bw§@é:†J½_ ›UÀÕŸKu”XŒ®ÀFOï)Ý%cŽê4\Ä8M>ÍfšP1Jý&Ülb#ñÎæ4…'yã»Få	ùpSPy\	å(MóÙ¨ûŽž0¯yø¶¦K3ÂU¡åÆ\³/:çz”:Ôö\B2ÌÌUhv¨ÚS•V·Bñ!³K—68€&Áa¾‰
TQÔÎ#‰cŽFœÏÑœS+Öô±€cãIIBàùZ-/»Á:›ÈÖüÂöyÓ—(.¹àð™ù*ä†‹r,öžIßÍ›u¡‚p`Š¼å‚ç-ˆ‚#[úá±mjÄ×ï±:¬ f Í%%nDJ'¡#qs\Ñ²BÍ;ÚùF‡ïüÝ´ÞÐ„_´‚J«dE1îHh7µš6'@m”16·^t?Y±÷›¯öS!óûÇ+]®0C˜[ðÂŸ(GØªã=2ª¾nF"1)·5 „•nI¬ue-ÙKÛmYÈöY9¬ Ì‡’9,ÔÃSòQ&9ql+@ä‘l±õadê<8UX~¿”€lÌÏ}¬T;Œ»ZLû5¯U%íQPÖÕIaÊ²+.Õ?ÄeÅ‚5{Ä²ÑD:³ŒÐ^ýûmw²¯TœƒUF®ÄËRH+^µâNr ÑIó?ZôyÀM«5³±‰¬ID9ß>ÐwkfæéÈµLïû¤õŸ¾+RÈ«ëµ¢BÛøˆ›‚9C«b™!›¥á]7S^Ú%‚Cì…G
¶\|j£ÏB†}0®ƒÓ'SkXú²
¦¦†+^,eª€ê²Ç¾qã8á~8žê1+FhÄ)¼¹Û“V™…¶~ö”f?5UëŸ$¤ÄÍêHH„l›fŸç×§OAöÌ3ó›O€é3žéjÈâ“@“1åJHK¼Õ¯Q›a&&3–ìli¯RÙ8ÃÁ§*¯f°LTxRlÿüŠ?ä¶ww<ö¼¨4<­…¨^žï…À‘Ÿ-;uÀó¾Eg¬T£#–Æ”–Éœ H~û¾3³c~.7m¦_æ®âòŽ)þC%xšß¥Š¶Å ºšëÆþ!¼)ê)é7„Î­¼;‚2¤ÝDHÄú&yðm¯çÛªÎ‚a1W“¯[Ÿ˜G¶øp2mA3Q1~žRŽ¤o2®:¸)À4Ù+ê´´òŒ~
QQWwš¿CÌo0@k|•È8f÷ý–¿’÷’†›³J
¶c~5Ì¾áÒâzÊ¯Éƒùm¦ayvIoQãµs£í	±öèUpgÛÎƒVO</Ÿ-~·}¡û"²ŠŽç—àþlß¦˜îOAL­Ü+X+9ç©I…'zŠ§Œ6PçˆTP¿µF{Ü·>¬ˆ¢ÓL`on¾2ã¾ˆq1ÐÌœÒ|’#	ëãK¼‰ÎÎÐ[c“¯kL"Ç£t2ý¨_ZØAÚÐð§YVå„0m°+ ”’YÎSÝ7$®‰®5¶¶H}¦¬a«,nŠQ—)EÂraèË—*‡|ƒ4…ßy|®l°yŽYÏònÉà°0Tyˆ'õ––gÎû Mc75áÄÇá+™Ó8›€½› îÒY~sç&Pß±žlYk=ò$óêb¯ßhX£;Úk®2Tt–6•‡Yìªý‰ i Æs Ù[‰Òžr#dÖ&Q·„cõiK²b„5QÒ7’<ëî˜X(c(p‚ø¼AËAÁ¦AÔ;Tý©UrÞ^ç
üðqê˜K{èÔJ;²?®©tgû}Ä	÷õúÚšÚØGAÀôÑv«éJøÊ®¢øœì¾3ˆÅíþic!816õÏÒÃ¼Ä¸¨Fò;Ô¡D¤'	”\3ø‚#7‘¡Í$P6ªŸ¨H5ùRIîùxùsàî°xå"ªsv›GBÆ·!î×jêŸ¶‚nu‚²¿ê&¡³ ÓçFç“²<,xÚÍ%óxÑAÞä»&Dþ°µýÿÏ ™›SŒÇR¥m1 ¸¼˜AQp{m§
È7 0C§+[ßÅM¿Ãøõ‡ªªœÜv2š#Ê„èœç2ÈŸû¥y*ìÜEã´oÿs~Ïõ)Üõ‹›jËÎåÑsí"\÷·»Ûc”õñ©ÊÚ«ðç Û÷0†Æë„r<&,Ôž.†øop—³tx¹‡6Ê ]C²ÏêÉˆBó†qf
n gw"šg;×Ž…Táj<Gþ]™Mœùø“Öáª¶'ˆDx|jëêŠ¸|' ‹=úìbj ºSfŠ¶³gEmF^uJÂŽþ3U‘¿Ž×{×;Å»¼u¨5Vkü×SS’îÓ2öòÿ)Ý-G})8¡àÏvvòêÑ­B»Ô³@Íû#wfµHè©Oáø¨Ê¤Exî®QdµlBŽÜ¼²WhAZü€zdý	““LáUÕu¢¡
—lˆ4ÙÏ”<2/¼y×03’hšÈe&E"ùÌü¢“'þÇ¶ö oE‡­4‡º:ŠÂr.Ž“=Ø(ñøhÂ˜ÕÈ#iý˜ö€e"L]#zms²‘üg(L˜¼)©ï=:Ó£KZ.Ú]óÉÁupˆÿÏÎºÍÐ=;Z95“ç0Ä•Š¿|XP,3¦1E´@hÙ¤|NÉ!Ù6–„j8ŒkôtŸAó';Um†Ì ~É{é¿1^3'üXÇÃqö/‘…¢nÄ%Pka¡ü‹Ï²½T&PÜÿU1#PµRÆz93àÐò&ßH?Ò;¢WkzºLus)Ñxùáy’5«¡d×M*Ò(âÊ±àn«—™>¹Ñ17ž´·B¤–I²\äºM?w¨ç?íÜúWï3?¢t!ù0CäÚ#xa:‘/Vt¾-¡Dz8X›f„È éÁ/¥rx$ñØ[Åt:5šZ×¡ôw“v4~÷ÿZ¾gJm¤f=éG4eÈ0kˆ¦§íP;­}T[3NQËŒÙ{$§ìfIÓÖL	Âýõ(€ëÙ¼Ûúû
@ŸÕ+§W…²Úè¥eáF1¸Ìó.¯Æ¤wÿ}åKà=$—t‹%®¶6	:í'¿¿¡Ü]:ÂÞbÚ¼	mÀ¾¨l9Ø·jq•e‚²»¶’(G„«êú¬#'1@¦kžö5ÞRìF°}C`Ô™
îÍwŒ4ÎŒ°Ö·ìöX™?æÂé;)e<SAîãáñ*(½*Sþ[¨ÙŒä:å3éËØ±µúIÆÀ\Jàú~y ¦tÜÉ£¬¿%2Âx†h¹J.E.¬²š ÓvÁ;t3|JuÐW?¥-öm–Íªg¸š¢Æ[¼Ú‘€oÁ‡‡Úó/ø&pqŠt=µÚkLº7êìó‹EC=~Ö9Mvw™‘´µJg£ÿ²c5Í1=y6ÆXÒgà–¦__™Ç@W—Ú$Ó’µ‹”.Þiþ¦ÔNH˜ÝëÓ[[ˆ¹å¢|„×âg<‚™š8÷¢Ó½?Ò¢6lŽ:…áW‘:ó>(g¹ñÙ£™ Tgi5ccó%¦3epEéSïAœhôM€°PIP†?ÄìÝy›üµrkKzƒ¯/ºª0uÐ„÷Ofuá5°R¯×ªxõoPæcó®±È¶=“#L!æ77ßˆ-ðb5:Ök«“Zqîh/ÃÖ4³\¬ÑÐc¥gm6½¤OŸ'šƒ«˜uþ@#ƒH]kÒÌyŸSOî÷RºrÐøÿ¦­
ÐIï’fšá!~S6’Ìa}[vŒäbPÎ#Iz£!ƒóåWš¶ýßbuëÒ0 í8 oSÏ 4î!Å¤þZrÁJŸ~€£ªwÿ$*£ò§ÃÍ.1!¶‡`\YÔñ?Y‚þ™‹;8Z©ÎÕèÜüz>W³8²Hàs3NH—Ä·ÂZ!žƒ¤mI›¼<¨Óo8ü.(ª¡,¼Ð AOƒ*Ÿ¿ø_Pù¯˜X½dÓ–b-žiï@YAÕçÔñÌã€‹”¾‚˜ñÝj…»ÀfQº.~xêùü—Ê»žÅY	˜ÃhEÜü [Ñ‘]’&f/¸¨õITe·º´DÞµåä>U+Lo˜¸Ct˜ïj´Þk/æñæLÉ4ƒAoôˆ(XV~Y­Ó•oe2Éô}z½+’%7X#,ôÞ9ñ¹t
µçO¸è0šÑ=_O{ð´+—ùx²ÍÝãªF~”µcæ€e&õ˜û¤Q‰ ’æP@Ãº…§ú·³pòBZ´…ò9èŠœ­TëHš&Þ#D{S#þçœšQZ1˜§ŽèßÆÀ¥üY¬¸&Úî}`8GÙœv‰C#3ÿAÿ"µ1Œi kliº÷£¸ÆZ¡N·%Ùà‡°DFõÎ=ót›ù	è™Ü0-‡!kðÌ.3lèbw’Éµù<‘ªISjV2¢œ¾L@bG[;7LîÕPãÚ±-³Îäv>™w_µé1ÎåEüö•DZš"éz´!Š×…KÒµæ„^¦>Ô]´xÛc?V¹¦ðÄ7•8LUgaL~Fï)ôÒFÏ„ ¬(«ÊWÙQnR¥c¤vªË»•+ïwAqAÖ¦XÄñ>åd”ªL™ÇLŒ±½Ì‰w@GhŒB!å	Û±\E«ñ —=A 3PØ$«Ð±Ç¸ðú³aÝÃ3ÝÖ¢IŒØ_Âm#9OèêÎÏW°üýöS™bxiz¸Ž)n¬ò±ÄpÍÄÁ+a½Ô²+`¨‘ã%Ò¾ßVŸEýîhÅBÇÜZ‹h¸!UrÄ6þC)‡:aôˆ±%-ô³’Aê‡Ï±‰¯ýàúkÜz&qñBùX®n‹:ìLÄÚ ÍRtÁÍrÚÔïå‡tF=sÓEr	.#&Z;'%Uˆ²:bÉ­7n ƒ98¦0‰»¡	ÝÅÝëU=ƒ§V§qSñ¤À®—ð“ò–Ùdf]|imE.äaõ{a3Š÷/4ìëž7`kDa+}¦q°-"ßÓãw½ESQf¿5‰É°½6ó§Û­Òmî
ä*›ÎÛ™±yÇY)Ë=ÉOŠçÁŽkû.…IŒ‹ž#Ð)Ÿ;¢Vì‚ÄíâÄ:½üÃÒª#[±W^¢K™@w^né„Np‚Î³_Y7â¼^‰»e :~á' i.´Ì³›[ Ú(oÞ	â©ÇÜÓeQ§7 ·U''
Ã@ÿÝ\5 ‡ÉÃ„ËU§¤PÝ”.T¢Hç£uÞÚ?Ò¡¬äØlW+¦løØÛž·-‚éxÎ$üî¤¶äÇ.­Ùá'ÝžÉÔÐ
[ y! *\ÍúïWm±öRòM¤-¾€Ê²Ì2€ô›F)½Ò öØŽY@ŽàTÍè;j/Ý“íðãÕQVe}0Ø‘‡®fŸälÙÓðDËó·õe=’éìçï›ê·©>jVôÏš¨!˜jS¬)À±2XI<£_”/?Ù-i´'l(Èt­àH|ë,Ò3haäÚ`‚Û%Ù2$AÕh@1‡åû|¯ölví¨+)«9 ™[1_íÂû…H¡TôÊ™òíSm’á„ f»àÐ4Œ ­ø'
ï¦`[mÎ}A=—¾Ù­);ˆ#:ÿúc9hÕÛkÊäßwX~Ý„„{j&òWÝÁº¨î*+~tããF˜­æš)—˜UöÜë%Æ%Y&Óu•‹-ðžvî]ÍM5ÿÚ¥ãVJ:ÙDË(l5
Î`nÅWbîŸÏ†Ða¨ë9åó/Šè"%	ä¦:åƒê¨§4ïœ„xÛ¦^–ùÉ/WÅH \(Éë?…ˆúqé,/3ñ@n·!5©õËmYz«QÀ°=v.Ó€ÉyE“;}“Í—v*WvhÛkFKœºü/CÑÄ­¤‚‡³¹`dâE%i»#14±þ¤[ïü6åÌê=S~0 @M0c
^¶T¼af4³ï"x)Qs„lãòAs4 ?]ªæ¸IoÇqGbKØ¸œ÷nŽÇz¾F%:)'–nÝxð‡Ðós]ºÉ'L#ˆõ#Î,ÿ7KQFÆ–çà’¸‘.=Û¬¹‹‹üxj¶j8•ƒæË»OéµªÖ¨®ªR°•PŠ½Iú7yæŸ'Š´à±.SZû”Ó4’Õ°í:B†HÞ÷ÇñÇïÙôìÂ;aò[Á¹.ñ|}¬KáÙnÝl>†t9îºÉ ­T7På­†Ÿ©3âÌš®ªÉ¢?c±©ˆ’Ä™Çö~c4ÚB±332Yc	…–Ã/·j”…ÈTƒ°©d%ªÔf&ròËƒºŠÀ*h™ç[eM ˜ð(éFJ÷ÁÐè[‹aGP$ÄÇ¹V·Í úSŒz®«÷ã¤ö
s©©|æXØâƒ±;=«nâ—Þ¹uÁ˜Å æ´Ñj%«½p¾¨<V5‰¤.aØŸGè*)ýÀ •Š[p©7€6Õb§±õ§+’_ÃX!ùºY23ÄóÕÃ
(½F¤žÝÊtcˆî0Ïh7‰P•ÿ³pPL)oˆ‡aãGóÖƒ‚ÏØ$<¶úî¯:ÐíZÜ}@Eý0:·*¹>ªôÎð$•ü«â×wÂ÷\Hv4ÿÖE¤½h<9n.äÙ,N'$å`&nX°¾3K€´'ÚËÞÀëÝº”é«ÿ_Ö£"÷½‘Gœ¶ÚCÚHV³Ap¶uzo"µ’Tõ‹ÕÇy„¾O€,&áÌËû]iø{¾–Vð¦ýd•Á7ü•ß-MPaÑLâÅ>š‚kmjßv»ê7´þBB¯²¥Ÿ¿%|ì
=üG…9ê6ßÁb…öBK¿àúÓ­ßI%|eRXÇÓ´!¿Ð‡©TÊŽî¥ÝŸ¢¯µ‹¦¹ÄÑ¬kL¬È¾ÀìmCštoÃI¤LjˆîÌpéµÖÞAeúj·Cð£³JüMb¹˜t0›I]y²ŒúÔÉÃëíß¯m6·CÏí,3D¶Kþ›”sšE”0}#ß!’™!&ŠQÙ|´ê9lî_SÐbÎfìHÇ ¬“\~‹÷ô‘¡©ÏO¡êj¹\õVˆàQ¾ƒi7ù3`—~l¥ÜôÙÊ4“BW
`2 Yî4`Þ~Šœô ±nó)…êa>Þp*¨¬ó™ðóSK\þ;÷dýÏã+Bø0ÝAz’¼rò$}B¯ü‹öŸêÄ(×Â¾NËú6ú¦Ä§CõñV7YK«ÜÑP;`/wdå†Ä}›scXÒ±zzÙÓÇ ¹Ç$JY“Wòpà7f¤eÕ›ÜF(Ö©ÅwÏÓ¨Ç¡9ë¸F“1±I¿{œ‹xpýº(ze–|ºSöènÖôv‘ Â´õÎ¹¸ÙE S	´Ýßžéi	)B³š1ŸŒôÑ³)¶^[áv\¸Ñ†!¹¬DE××hÎÎ<Ë™|¤àež¼`C'Ux~».„Èî¼dVÃ% äøÁ"Yï_»ÿ’Î~G„R‘S^u2ì 	R+ÏÁk“ÁýôVùk\ÓCùÅó×èôçNÀvÍ^£½H‘Lg‡u-FR] øÏÆ‹ÝïF™ŒÊ1EËÞÆŠ[Ý}èo-ÏìÁèÖz§¥At2ƒÛ¥`î¸ž‹W†Ò@Djpþç3£äc›ã÷ˆ™HbÐ5ç{Èy?Iû\ƒ?ÝM[P¨;î9êŒ©Tõa€àïÛ€£²!‡(ã¡Ê…ƒvùf*šnµ¾©KÜ5¯‚[q×´’>©I|ì²|Ù„ðÂ¼ŒÓœ¤Õ®ÔõUãyw"f Ð›µµ”R\KƒøÝønO¸Ìõ4¿z¯nÃQqÂ‹A9¶“C<q³¸‡Ä=r¸>ˆ6q×±…œi6ï»¾àMÀ¤h §ÍýNß47÷$	¾õãŸNqfù«2GÆQ½Ø€ŠShu9ˆ·ÙþWçÕš·cb˜^z'a”Ã¼ _Ý/Ùà~fB\†›%c®ëøzÕí3q¾D’}q~ÎlmbkÂS§æÜÀº¾£^ÿTZX54J”‹Š—7rjk?jþ9óY®r&c#Õh!j8Ê<Ýç&2²Û¹OŠ}ˆn/Óº&„õE?Þ754ñE^+yä8Ž\ÒG¤ÿä!þÙn´ÕcÌÊ&ycìgåTEÏÉ½`nšWä |…¼™væ$Ðh)œ®áú§¯Ü&ƒí£§„ýü²i~]Î¯IñZ§tHF°èçG¢Z¨óÂMŽgK‹Fä¥Å…8MºY‘Fûçõ7Â+ûòPMBíy±¶X`ƒ°m@X*v°fÌ£=µGc ¹6&¬Ëe\HR¹šapï*èµf¥®þ] UèõŒ5øûÕ,Ò·Á±z3Ô`qG?¥%p8ŠÁAóf1r¼²í<²–sáUœitN^+ùXZu‰ Z”ž‚Þoø”¥‰i\ë\ÆaÔo5S‡üO&8„½aP4X ®HßJ¡ƒ…eOªfÓáÇG Jªq}Œ	,¯„‘¹ëÞùaPàÔ‘ç7&±Á—†Ö\£zhÉÖÝ—¯apðëÁàÆ’®ë ©‡ûÒý80]ŽªSÛî5¥RM„ÕÃZ-$šîÞ—4b÷õé.OZ„Á,óÛ»¬ëÄÄË¼Š{CÆÄÌ®B%(0–w6¨5kî ªéG!Œ«¶h>Ñûº>¹qÆÅB}ˆ©Ëúpôðáã=ž¯·í:á
›w†Ï€Uo¬{Ë^E›s—œÝM¤f–c;á„ee×¦Ï+¸6SûC@yìx§®·]°-éõ:XV"Aéúìí”¢Øˆ¤#5U+2b6P¬?ßîÍ«Ãÿ#ÞJéè9²ÍYû‘L)L#KM¨çº¾-ÌÈÒÔ±k˜éE7Ö“¤põâK ÁÀF‡àW$óèÄ>G‘3Ï®Ûù_ù[FTÞ@hj{råððTá¯èd1t0”ü[	wWÁUkÅèµ)ëSô]øBÃ òí¼ÜÈÃ£†Bó¿.ÉÕ»qmuP%„üÃ¶r3×RÇŸÐwçß¾r1òÏ­‡th‚òdgõ-y<Þêª¨ë¶â#Éq?CÜpÁñh³­®eÁû3ßìŠP®t8},þw	Q›¾9oEŸ¼†4Ö—Aƒ*\ú“Ì¨ìdP ÷¼ú<õˆä+ð~«ZãèÊcµ*ùð{ÙY»R¾ø‚)&’Ü;õÛZ$4[#¼üRs¥/KÄ×Øîµ4ÝúŽ?uAT×æK±óÔŽ““´w£jzAMÇg­?ØFÞ¦7;j‚	LNâ¹2º@ôži ¨)\­Y@#*¹sšÀiŽ+Bú,
'€“fhÓp¬EeS•5cïJ}„_E°'Ÿê–=küîåÏÛa¡Ëîøx~xkÝ]›éë
ÏÈ] ëÂ²“”D&ïpdì¥ŒÌPÌo9TïÐ®ËÎú_¿±¼§òÁ_:}ñ…[°_ÐGÅ+jO‘g=éâÿ¹ïð"Æ8µ.ŒÜªi«Âòáž3ÑBµ`‰¹á5¶pBEl¾µ51›•i–Y±dU÷·vÇâÞáçêu8ãlÒXæfh=PE‡,ÚoYæ9}v#fJ'AùŠ/	×ÒÍá9Ë¨ø^ÉíCÎl|
(Y=<,ËTdÜÁÑÅÆßç«ë‘×éqª[Œ5µ[„P¨Ž¡ß ùöü ×N¼~$ÎA¨ÒŸXW>xÌ]g¯7J¼¢½¡ ª›JE$K2è2sh²ÆÕ †Ù©~€{†b^Ê\Uªy‘YzX'¢S]I[=ò]J èþ Î²4"Rc¤x—dLë|bî±žGm*h9±ï)‘Lü¬^D£î„‘µgÎuËER¤[Qo9—M6¹mŠï;±ÊH›«œÅšz{¿ý-•'¯•&eÔð§}†€e&¼i ùS"•œùIÀ+â‚Ú}^¼³<$Ûä€Îƒ¶àéd1Žü‡ƒgæc-~¨¯gÑ71{í;KD[Ê5ø92‹x`QêJƒ-¾›W´ƒ435ÔøÞ`£2IÚš”ÖDpßØé$kŸ@:ìçBóøFæ;’’ÛÛ=WW®Õ|Ž0ba%ÓÕ¤3—‡J5`‹ÉpeD&Ä"0BÌP¡œÇŸ½íO$^”9r™øŸþ
)2F`½4™¥×%Ý÷dG›k¡´ß„L¸ÒÉTKÊ [Ê˜2Ïä$â'Û-E¾Ñ¦Ä&}ÖL · Òú™ïg’Ÿ]æpÍô\¹tªã ¡þæ÷á4Û)ÊýeG‡½23Ã)³»à'LÆve¿Ã©aÂýÁKô|ØªÃÛŒËÝJã8ýz#÷°{'{„³Ô3ð¸qššL±u…íÝ1ðaé s¤Á`Õòq‹La4Ô>=õ¦@ ÏF—œïÀÇ^ÂùÆyÅ§
Ù—,+¿6¶S|ð»ÊšÁŒ!0ØýêjzžÃ0³}âe‰Í ØršÑÃ"8;Ñu½,ëù®ˆ²iìµ¿sÒ.	l¯;8îÎÞ½íQÕ+¦ƒ"D¾]Å4(µãÖN—_ª«Ë<¢4ïJ¾Á—ƒÌ7àÔ}‡¼­ºµªXVÁÌxÞ»ë\Ï›ÃZýë|#½ úÑ@è„ô§\@óý¢çELÚÉ¦WâÛ|éh×îKÕW”Ô&^? ,Ÿ_OÕoy®:º·åþ{Z/¨?±ëŽ•¼Tn;A£‡)ˆ˜à\Þw“Á+Æ_ÈÝp6ýb@}ÇC½¸?„*TƒŽé´½¥$ä]ÙâOÂ ;Ï
}Uê‚KU:lî‚YÊ©Bš½WÓ¶î ®ß›B Žrî|C"X„f1b‰{™«gC£³‰¹m°<´ÊocÉúg°ã¿ã°Èžmaž$ax2xäIÑ÷ƒ\àVƒ¦ò€r±ðT"ê¡1À¥†‹o”ê¾gI˜ºF³Œhk	€qè%!EÖé¯£Àiõ-ofþ¨o™VKQ¢!J0>Ä¹jƒb7gÏ°ÄJöã¤ËÒÏ( ÿ‘!Ý'ÕÜÀæ¦çw*õ“Øx×ýîâNåC«‚ÿFtééH(ÙÈ4m_ëZ Q%Ì—^ÁÐ®½ºfù"s¥ª°siÙ½°ž‹XkÀfì””óÐ³ÿ¾ÇQ1ƒÄÀ[qG±[JO¢žŠ¿Mðìãýo: ?bÚÒg	Š–htRór6Á¬¦	[y»QÀ "lçPé÷ZÃoõäóMÙN,¨w¬ß]Ï«ü±Õp³š÷«:‰ ýãxYÈÉD0±Py!`zËÄ²æxuôÊÒªE¼WÏ·²xq|+²Onä¶¬•=G°1åÄ“ vb<ë4	síÁ†ì¡qr7ª×ÃÕ+d‡<·|ÆïÖÌ¡Â}Ú(18¨Knû;(ÔK“Œçˆd¶eYd‡ilä>ãŒì:+³™ÍFŒcçëC†íÊ
¶²…2fÍÈŸž
×ÇPÇP¥ÐuEèJZº•-÷ƒ›MÓD·¬&òüœÍx=Ÿ– ²ºÝéMÆÖZY^O/“Ø;š«Ô~XÆ¨cw7ÃzPžsH³§I¸_ƒØvCúå#Á_7YŒ£BHHšÀçpD‚(ŠYÎ™îx±uö>‹¶Á%ý¨±ßJôÏ;‡Ee	›‰hsÊão'¾ÔÚíˆ†
˜"rÎÑ~1k;(B#À¹¢UEŽ±«wdÚJÈ..UÉ3‚"mŒ+ŒvB‘žF@AëìõšKÿàCF^•Ð
Ë±h¦Ìð˜‚¥ÑSluN …©<f!•¢}ï¬‡’ns*•¸Ø>wgÍdV¦'O¡-á˜Gxè%HeðADÕí¢Ìÿ˜)þ5Æ½Æn÷2UÈˆ•J;qzŽiÌÈ@ä= â€¼ðÛuoÊ|Í×*4<{â3ÊI¹ƒ‘ÑœŒÎš!åÊÎÝÓ©fÇ0°ÃhDµqD*=3ÁÅÇ:cØ‘Ãˆ§Ì–5ºµ/¥¾Öÿ]¸zU{%óçfÞ4£ a°¡3þ*„G4)Þ˜³Ï[oß®MÒÂ×¿‹É³…uE[ÕŸÞìGê¸~´ú¥s)
GlQ“sÏŽuMçfÄÚQ7{’òÂÃ:”›m8`„Ëh»BîcI¹Ð­ŽÈr³²ìŸõ†Ò%…—¸øƒ®DÁÏìR!o_†j'×yI¦’å_ÉáØú¸í&ÔÄÄ	Ì©Ú55!àäá>ŠáÌ‰1îÒ„>ÇAšÞ•‰
üUYHŒ2yF/ÙHV~à|˜kj`[mùC»3ÙCW}?x…_xñì«–òEè8”èÀ„:W»p5¤œ®2.¦_V‘Ÿ”šO™åÅø[ã$¹Åº×Í´N9d÷à_ LKµ¸>DnLw…B¡{³eœâ^ãpÇb¬Ü+|Œï»…A!Ì¥(€^çS}DèZ €p0óÜdð7sUžåŒø½*°$@¾%°Ÿ·
ÅA/Df n!ÛS'wGý5vÕó@F‚æ9ÖèJt@J‡ë¾mÄwtl/PÜªÉ·ºƒÅÕÂSR§·˜‡¬ûš–¥l–ÙÁ¸hU”ýeþºR}ïÎ¦ú)·®.£A†°G dÿ’Ö8‰Î¨íëÌ¡zþÌ½4Ð¥1AqÝîi®9¢]HÖßÛÂY@f®‚Uéž÷P$ï‡¦äçƒØ«¨v±îÜ¼¥EÝà‰vÌ< u€\ÆTl”5eËN +Ï±2©¸]}6z[ˆ_ÜD¢…ÿ~­5?ÆoœI`Ãl7º•’|Ê¦M?gÓø®óZâ©ûó;6'•õTˆ£~R!(V˜`þÓÿîÆÖÂØ¿vÛÖ¼>ölfAùDªüâ[¡ÞK‚“}¸…à<ê_rlÙÔˆŠ˜u[©ë jz;ïñIu£¹­|0/V§×9&šÇç›ßŒ¹«À
Ð)Nå¯/—‚ùÝk^;ãùÈ`·RùÈ×s+Û2õÅªÎ ÑðÎÐJº0\åÑ6Zögú¿¡Îï¯_&'áÐf¢Ï¿Vœ›f90N-Ð‹Þ4ÈÁ
£A­øSÉÓ_–±¢Žs…nÂ:
u—ÚÙëòUŸqžéyó9—«YDWSËÕB99¸¢µ­1j“ ]Ùnº·MÅgSlÐ§5¹]ð’u‰óX|4¬­ØŠnx¦ÅÌ8gtéÉåïzîd†³eÃ6ô”›*ý¡JUQ8§ÄReò¯Û´HÏåU.aªGïÐ{´qÖ‚äùªY<nGIJp-z:`Œ¾ Ê×ãQÇÅëŽ¿ö¡n0ÍXêßc·±8¿ÉNá°Ã–v]š"´8¢Ì¥çŸ ëù²ƒÖ˜†Ç€ÿ½>~%¥:ÍŠÊÍèë¥´21ä¯èE÷zn[ÓùÂ¨±q ˜4¼%"©Z¯Šsq¶œMQû‘NŸ&·Ž×0I@«¶_0šq÷Dêîb"ÂµžµÜ9ÁQà­W_úg”rÎNÉre·~9^/Áî†Y*	c¯/O+wFÈæTuX"­CÐ¯ÖÁŸáHÓÀI‚Ññ¥«pËèÁï:hTÿ† ¢ Œ>)Â”X»ËñºéŠ9QÁkY	/» 4ä¼—÷Qì¹¼|»8m“iv„¸4ðD38¾¢5„1Û@gÅù0h†¥¡Ö0øgú)ÏÐKïx5/Ú.Ê)äó×z%x,†‰ýŽÇ«Ü“|I"±´"ïGqàQø#—ããO—Á6—îºŸePèœ!¦îí¶þ}­ š¼ï™"Ö{Þµ'/Vë	n¯È,¦u§ÛüÜsè1±6Ôp„1aP-Ó)…RÛ»‚ñ\ÐÙ9¨ºÂ«·4Kÿðï]ÆÁÎ‡<- /\wÙƒ4u{
™I	«PfcšˆŒíP0!†CÅîKÑ‡JÂÉ–\&d¿‹ÂüžSåjœÄ*I_éL­w³ÛA¾œ±pÔT3¨g—ŒPUÙU!ŒF:åñÕÉí1Ø «"$ŒÍµ%Bhåï²®è¨BÒy Cá¤àÝÈ'’² È±‰˜˜ÑqP‹ZŸÔú@ýôCûŽX¤‰H®þšü£?+Ú4^ZŠoÝ M 5…Z¯/æ´íÁŒgFåä¿¹ÖyxeV¦Ä‘ëTóúw? ")ÀGšC\Æ~ñj¹ô+q{Âr/ÁVmJ®Ø™åÜ?AA/4Ön[þVŒ®ÿ6{0iíÞÄS6[—ÜØéî8M­7Z_N´Dèßâ^Õ	”¿éã”ÂŠq:\7‹ÍÂ¡Z´MøLì+æº÷ @[2œABql4åž¨iæäæb<C#K!…¾üþÛ¯}[ÐšÖÃGŸŠ€Ë½’Èq*Z˜10`(hGr¨…-$7:ùlÍ÷QO”è'P PThkâÈ•}8îHøv‘x¸EFjZ´ëç8%^XòDƒCñiØÖËä“
<Á}²¸±+`bÂiõ?³ŽZæ'¶@Ë]lÿÓ·ÆvþŸB^ˆ<À‡gU/
Ñ"”÷‚~È’Ú°	4‚µ`Š*ÎnÈåæº;ÀX|h´éýW×.`Á<pnÈÝŠÕ5ŽÖ[2%*üHmFŠLç^€:›qëé³|©œ?‡bÌ<ý„ëå¦²H*»¨»©ÐÒµS­\û¾Qü,¯y¤®âbÂýæÝéÄràM¦»¡Ù¤³ˆîÄ}º¥{F%©9ª$_üé×÷¶¼‡_g³YÑÛï^Ïš`“x¶t¹<-v£@,¹Û0Ž	þt@RÄÞHã±Óoû´Nüw›)[ëóö.úð†»ˆÜ¢›Öÿ‡=E•ë*Ø¢f#Æ%=¯&2›ç¾u+‰%ðŸÎ˜Š«f…£ÛŒcùÂŸÔ@Ü2MìF#ZRÒ´oŸ ‰âª½G¸óùO6<›nýÕ…õ™ßwÐ‘R5æ³¾Q©­±'ý{ùïÞ!ðÞ?/p¬®ð$n½í1€™òþŒHÆe{ÉD,zDPr­›m3·rbcgVaíB1_X\™Ùß‘éÂRÌ¹A¢ìRí[·Vj—¬o‚VÀ`&˜Lê]ì‹;‡7¡[8ÂªÜO.~è½ç¨ùwˆ²aáó½E†ÿNeY‹xÞZ9}µò$™äl¾„XáÖ7Ãöò…°ù[3î×D%h"ï¹"èÿkµ×µ: ’¨›ª¯ü@Nx¾Ägû¹Zò°‹ãƒ­«b¡qÚµ‡e5jyxÌ?­ÈVÉ-„ãõ™8÷h€7²6´g}ûb†wû°ÜPl„úÎOm™Žöµ7Œè¥ßobIòGd(%«üv{¼£‚î%@
…`vož¶¨›ˆYçgÀÍ,á‘ÜÊ]>ÑgñšDÿK¢„<-j|‹4Žk7ß±ú.]º©jùô¾mô°~y ¼™úcK˜7z¥r¦–ð@/®dlºžv§9ô^áòO¸æª/Óñ ÄžôÇŒr§¹½Rå§ahøN
ÍäŒ›yîå»	N'1õ$IÍ æa(HŠÙ’?¹ª˜¬;HV½ßÛ#öŽ9peNÁ¶`ßñùìrO!ÅKFtÊ“nêÚ±cüùÜ¢Îy…e ­ÏÚ„˜òåhŽLhÞ”ßüOÂžš+ß+>ö]T+CçÂFÒÄ˜^¶^ëÜ½Ó§À•p'È;³TÕÌõ©Œw)åôÇþX•uCrm|Ö‹%7ËM»oDôÇãµ„fò'ÉŽ1\ôý[nÕw‘P_~pÀ4ApTô
ûha÷‰ò3QyúÅ7‰•¥¿ØºDñ…Ø6Þ˜ èž¿;¹³U¾·þ±Þ†šNËjà($^—’½y®)f¸Á¿eûâ½Ì{ùÅ²IHFoæ1:KIããVÚDp
Å„üÝ-c½„&¡J–çB¥î°1¬Äu%bÅûÍ¿ÂÂ§ å“ºGØøê[8…±Pw­“«&‰
#ÕÆÿÛÎ˜~×Rh(÷oË/WéU,œ•ßUp·§<å¿8Ö­74ÛßK¾ê¬Ø@òÝÝlXgE×é¸%|âyðÏ»n/¢b­VuÕVÄÅbj©(Â:Í‰a½óføÀI"ŠÞé!jÀa¶ð	/H]?ºgu‚þpÝ[¨eë(ýˆKJMÑjÿjÞ¬Ð»Àæ?í©û¨)u³ EüËóá"¤²ÒÑRã—ŸŽähôÂû]÷£ë Jõþ°TÔg˜ø¬@Ãœü”zµœÈ£eÜz‡›‚B)~LÛ6¾%a0v¯$6QzœíÕHíNß¬˜Ð…œ4”m³ÄöFÛøŠ0U /¤m¬†ùò2l*H4’#,§¼ë	ÙÌœL@M´ª£³TP1tbú²5ðéj§cÇ¥ü¢¤²ÐjÖ™º•‚ÐšÞÎñÓDrA­ÊÌƒÀ5*cLÒ´€Ö²ë^û)¹°VA».'•<EÏ%k8¼Çá÷¸#o˜Ù6WÒC±4¼®àtÂ.Ätr^DˆÏ
~M`œ(Q×ÖÉ.D ”‰È9¹MÃö,dmÑåkª•OÈG»½ÓPž5®.áóø×ùÂBgù¸çà,[Èn3¹hYëtûuÒÎÔØsÑT{ž;#‚ïuQNÿ™B…PGìÒŸLÂ©Ä¥‰Yfaª:IQü{m'\ °ö™„v5ŒšœBìýÔîŽüÇ%!v¹™¦vXÔÌ;¦z!CÙ á¨^1£$]Ì[ÿ±÷Ì™¦ðõIù™¸U9ZMNÊ‘Mwâô5çáùÃô¯xw*õ‡¹^à ªrà"ÃÞåÐ]sÖ²· @«9ËðÑ’ï¸I“šê­…DÔŽqš9œÙ¶<z‰H¯Èè‚q8y‡¹ñÇåEÁ[´0BumNÓyÑUI"üe«¨„F_R»«~¾ÿûc»MÌ1ˆCßròß0¹ºªÔbõŸ:¶®†6Jß-ž«õ¸VÑ`ê¬î?ì%?9Šhuó@ŒB4¤öE$ò}Í6Z%ÓJròOšá²ŠØñ(*6ºt	ðnL™»àUÉP[ÅßoÜÐûÒ#ÿØ&¢+­°L†V=¾Ë˜zÁ#{Ò:’¢á"?Ÿèˆ<oË#Ûú1ÂÛj1l£8êÅC¬§?
:I’ñ‘È\Û‡ë,gà`@lI'«q m@÷Ã‰ÌêÙ´¢çÞLt¡o[¶=ò-.†û!»ÁÕµE!¡aqT8y]h“ ÛN¢W¥Y‡™ó•déÉÞoz¦«
HæÈ»['€P.Ô^õ³Ú*wÛ©"%¾¬Ð´V\MÚ
Ú®oàA‰ÍÜˆÑ#ÿŒçÿÐB4 Pñ%þítTóê™æ0iÌÊ›M¥FÅcÒMC–cøá¡ø†Œ4¦_òêV`p,DÀ¤—"É9e¢ÎÄÛKë>³èµƒi»œØÝGPý9[&ë+D†åàUph yÄ›w­âl³|6¾1…–j¾	`Æžžý·¤ÅIK'MHómt; /ÛŽ½Fúœ"
IožÎ¼ÍWÝÐÝ=/ŠEI„€S6‘U‡jsûÏ<ƒòá]vºLÑZ^©ÒsU(ià<*=àŽÓ¦
þRÅSM÷<å¬•ªÂuãõØx™*C¸i>ï˜äæè£Îh  ´K0/îtK;Ãá›¤‰¤o·ærƒp"x–ÄÙT4jLP]Åè©Tõç8i@ÛÆÜ¸k½®¨žüÜßøDQÖþìQ/Ñ‰ÏTÖ wVñÍ?C³ûi„–¹â§ß¯M,\e’VeJˆà¶ì‚c¶]’Ü¿Zm=;¸Ø.Û6ˆ¬@m"yB%fòëv¡Z¥cË¨;½<ø>^BÓgV!ñ”lc›Ò¯ž$-d±~ä<ü!(ú°udŒ8ð¨Œýk¼Qæ›ì,¥Ð˜@S¦§¥;¦ñ¯Lð¹T™ÝèÉ˜‹ ïJ‚	ÞÁM·‹<[M|MW±—†|wKË“›ÖàÅ‘´Ïð0‚öÉGËÀëÂU"Rß ÃþX	wbv'Qqoµë¯S²C¦@¿x^Õ¯X>¸óÌsËÏ<Œ’ÝI¡c˜Ÿ Î‚4
¨ÍV3âŸ?~à+ IB5+nyÍ¾©¹&·b	¡t£1Ùc3Jp¿ä¸ýd…moö²‡Jl_å·Ù,e:°éXcvV_Ÿ£ÄÎî‘¾ë5bˆ-ßêÌ›•þíAu„é®Ó¿=n!är\À˜ìƒ¤ÇÎ]*Ku^‹•J9ì{Á÷VßÐ3kÁ‘ù€e6FCÜÂÂ½$sw¯l]Ì&Nõ~Â÷¨ò©6ëæSTz˜Kùjá¥SãºÑ˜À_ËQƒ÷èü9*(mù÷ØVù Z-Ÿ=Ké3Ìwœ¤`pÖÊdö‹$ÅÍSh”ââ¥Na6Ôñ¾È±ž¬IQ‚œ“}!xØÞTÔ8ÅIw=$ú"Üœ"9Pª<ˆ~JØÐÈÅ9ˆñ»õç>½ƒœÁ\³ü  N%ƒ,öã­ä¥[­.4­
+s_ùBp"’{(_¯™Wèú.ìŽÜ¬bÇ7d·7>%Y)žŠ3¶Aæ1u©™ÒSjþ¥É<ây[ò³“]‡€#óýj5ÚËòaÉªŽ‚…@,´o°Ž8¯V©T
ì“@ßvóúp¡»e£tµ¡ZS¹Ï]W|[MâËÏ|(OwÜÜ£ñl‡ VïGÁQÄ˜¬uÿ1ÅiæR?Á^QÀKQ¥€K‰”&Âõy Õ2É‚TŠ[¯d$ÑRù•¶³îILÐ!(õ±D· …þkÈ´'”ž/íƒO½Ü¤…7d:ø‚Å?¤Nô¹’“° X…O_2—"pŠí¡—68\¯Ë|Ký²Y¥1 yÔ– º·k¥ƒu÷e,àuê¹KEl ¯´àkÃ8Üƒ]ØèOôN+®¥
È(ôš-RØS¸¤&2ý—	è· (‹g"«þÈÐñ'/‹zyâx½Î*špÂq	SÐÛHðLóL²5I)%*ù$¦‡@%IäøqY$¼„ÅvðÀfJþU'n`-â£sItÈÂÈý'r§|íz8}WÖ‹ÓnÝ2{m“5Ÿnàˆ:e<ãMVñÍm˜å“ÆO1<O S“…¨W#LïB“žEÙáž½JnðÐ'§º M@ŠpQ'R
	Ÿ3ôW5½ W¤2üür6zz’ÏX%\YtÅÄ™ÊéêÄŸýg¸s˜Û	g#ÀchÞeõ6©jNzê¼ñ°WB<d É8ÙzOÜ‰ºXšÿ­!ñÇ×2—Ï|‘r˜“bú{ÃõÄ^eO4½ì$2}sa½|ò8‹ª:v}~ÆAPM]Î¸Ó¢ !K[E¡8rÑ™„£|@Ëîb¥@¨C¹EŠü´jÊhÞtœIam¸Høá,b7X™¬ß[0˜eîŒ”•éñJüóûŽž#[]/7Ä¡LßÐã-
\—`ù·áœ+­¥Ú>tÅ½X’tâˆ=ý,þ\k.t”gT4àöÞës‹#¡Ó©F­I’Æøë†³ñù¾—=‹Ã—å6dr“™í¼åp]ªŸÀs÷/å§gª!Sî‰„qx§o×GÂ•Ý¶!óåM/×SoXdfêÇÜD'x~`™yî)fc²Å—¦Ñ9‚ð7«¼¸°è3×GÍ\%/]ÑÍ
è^O‘Œý±=Ðuº&?,hºR·Ep¤÷/-Ezç ¬°tß#&=!ýæ¥ÍO1i‚áæ½)D·¥xöõýlÈ‰÷ílzrâF†<jÃÝÑä¦*ç'†±£åà†vË¹ÑtÖì9¤~E„¯×I¢ávš(d¬Ú¾¼î/Á Ê•ƒ¬Œ„y°Ý;á›âœóüÉz‚ÄS„{:D¥7.[ þ¢Óp~[kõ`	ñ‹$ø‰Ú·‹¢CkXb-:»–? .wçÀôV^ôØ¡OöÌER»ºâÂFœ\Å|C"¿bn˜ñ©–6Û—v–ÙÅJ©„!ƒ¬  ¡YASEr]®/è„†á?WÒÝ•E/ÇÉÿñŽm}ÏÍŠ(.˜îN÷—äÖ v¼)“ïc£×Â³æ‡fé¦|Á^»VœdE_nF9°?§¬ÒÕÛçÜŒGÎhÍêŽÝ×É2éY3Þ‡ñ?WJr²Ù—útž€Ô63[þ±lŒïzL”„qÉÀíp–Ø8Ÿn;ó¿õðÿHLéSq÷ËEˆº„Gºèújz&È'JAƒ“aÜê^¸Aý/93±kÕfÕÚ²	¨¥€QEsbUF§BiŽžØºYñvš,9£69¼ Œ¾­Õ¬à¥>¶®Ä77¡1j0’ÂþsêhÖ=—»×Û½_Îäœn}°Píkqž¯†+À6êÙ@š÷ÀxXZaRGó‘–(”†õl\§]+tf¡¥I~yè¿›`ÈœYÄšî;Æúp×nüf¥ˆs—…;€‰¨Ì-Âi dè¢¥RbäV“ìQ‹sU—Æÿ4Ô-2Ãþö°§ª4œ#Õ/”ÅŠ<I)Ë;ý´Q{÷–°Éõ<3‰®–›è{sÖêÍœªým[“—*57!™s1¿UôBõ=%‘Y¹pë»³¦"žFMÞº±®T¬~7mÇìg)ì°ÌGªÙ~ï”œ€ù*âÔr4_1ñ
’Åò¼“‘fV’‡~¡½‘ï%)‡×|ïäì#¼k´Á­Í/ø£1³†ëbK&u³àð8Õ†á+ÏÂ¤¬“ýRänœ›ŸÿaRWÇxïÅ*4toQ[Z¯ƒÂÓöM:* ¹, M§lr¤8‘µ¿•ñ×µ«½ÓË² A&ôi±{X¯­DGT–+“4¸cÙoê¦	¿žg†Hq—k|†6æ—_ÜýŸj°¡MYÞ$‡ŒR©{vë7ß> ö=‰2,8ÈM\=l‘¿€N‰oô!Ú K/PV™<ì•Ö¾lKq¶i×6‚5gëÀV­¦‰´‰g|‡ÙO.Ïþ›5_n<koÀß)"W!Y¶z`yñ~~*jc5[:èñ<rðUŠÔäi|c…Êv_yšÑ‡E¯rOÎ~_ÄâÐ†”jÆCõ‚DÈÖùÆÜv¯ÎfUp¦óÉˆµ….u¦B€H¯VòŸ}P–
U:¯«¢‘1õ)ë¼=Ã¨
¹ësÙ~PŠã±ÔÌ)Ð¨Ø*‘}lŽÀN(kQ4:¡2XŠÛÈÖ˜ºI-ÑÏŠvøFê.ûQ5E¯§€_9ì£Á$ ê`™ì×žŽ÷<2âÕóð)‚2 vÑÈJþ?Ú1Ðì1x|æöŠO¾¨ÒC™¯0øPŸ3z2ö]Îc¶€y ’Ë¶ã†p‚%Üiÿÿ OãŒ1ŸÊz¢:©Á|Tè¨Æž*×'Ã=üLäh?¦®Ïqîâ÷ÀòÕ>-LñÒ/bÊºlùyh`œò—²GgRkþž¼ªÖŒ"B3æAs[ÁN§!si¦—Í†CrÈF„®Ð^eï),‰Â_g›¨\XwjUT¢6@WÕbD¹gM™m%/fÅ¹qˆö¸Bõ6µ-eSzy\	å(MóÙ¨ûC¤ž0¯yø¶¦K3ƒIéª”$³":¥C-ãÞñQkÌÜT~"¯Û_„ÿhµ`ç—pHjÍg•a÷ÏKþÎ#‰cŽFœŽŸ[Ôy®ôº€'ã\Û Hñ§dbþÁ<›²G¨‡6Úiz°»Ú™ùrä›‹1`·Ó×•—uà^Ë7.Ïø‰IÌ»ÁBR0ÄûlWóý¸'i…Òä£n¦beÌ:HsCK4ê`WQw'žþ„k¾¸Â„Y¥Ž½ƒ}Œ+„_´‚J«dE1îHh7µ•9'.àT,·\ö•14€¿Á ïš64ÿ³_È-G¿-H‚A¹…š¬¾™2ª¾nF"1)·5 „•nI¬ue-‹8SÕD¡5ÓôÇÔÜ',”Ø?½kx%Jy	K£Ÿ>ôB¹}¯4?^ÎQ*±“€l_‰™w­~<Ñ¼VLünÿ_k¹_WÖÕ$æâdoÒS?›-Øæ5{Õ†×\+à‡‘äü78ÁBæTœÌFúŒŽRC	y
Aô®N&OžúÐ’êaöâ‰V†ID9ß>Ðwk;ÌéÈµLïû®õÑûB+‰ù¿üFîB€¸Ç¥“ _êa™,ŠãûT0STÏa‚I¥…G ô3|B#÷‡BVÊ<Oc®\Âžbyarú²
¦¦†+^,eª€ê²Ç¾{ª±$"ù,+?f‡aýëÍRÉó:›Û"z6n‘ÞTÀiÿ¬JŠ¢ Å>Ï5‘”¡Àb»Ï2õŠ[šÞ"ŠéjÊ¦Ó@—8þMY¤½Ö¥hy@iaÛ­8aé‹u‚•Ô~ý/ô]0)ÿ^½ØGv¥Ã¶#8sºèáB4@sVÒÕìX
ÛÅ…À‘Ÿ-;uÀó¾Eg¬­wÙž Æ•Î'J,í—.ümr4;~¦¼Œäõ‡qA¥cÎTqRšß¥Š¶Å ºšëÆþ!¼#êyÒ6›Ð„â¹-Ç,‚ÞKZÈù*¿mû¯žªžÃ3PxJJÜàËÑœøpa(z@+~ÍÜXá<®ZèN{yÙp¨\ûø·Í0VQdöC–~xuqmÚ§`Zf±À¯ÙÞú’§ÓÔÏH¯tßù/*|Wæ¾áÒâzÊ¯Éƒ«(ò4+8IXkÆ¿<Ü¿G±Zž¡8$“OŠkOYúúa;úT3õ¦_ýÔñT©Ä´¶)ßóÈªé©+	K+5	«ýUhb‘¡ò °Á ¿áM{Œåqú6Æè½L4 !òfªîˆ0!e™ƒÒ4ímJ¿ªò‰›ž”7ÖÕ vŽâ8F²ç‘ÒŠ[ÂµEYê¤6oä
¹ßÚJÉ[Á7Â
µ”ëa¤ÌK^-âí5înŠQ—)EÂraèËÁJR‡Â/¯ßy|®—s<ñ+ÏÏ©€ñ¹!lŽs»˜²iÓã“»Rf,2â‚í®{Íšw_Èª½› îÒY~sç&”ãž%
x³`Âð'²@Àh]¹&„"²
E|=”‹'£ª¨Ùd(tƒY Ù[‰Òžr#dÖ&Q·Ž?š;Dáb™5B›yÆFáð…T+vR"ÍBA¬åŽO”ö•W~TýïØRÛ3D³³û£×Ãrè`;²?®©tgû}Ä	÷õúÚ˜ÁN^ÉF\ÐèGÄ6«éJøÊ®¢øœì¾3ˆÅíñfcT/|pb°Ï‡ù–¸N¨Sÿ0Ó¼¶$PÖb¾æ[-gÐ\Jèƒc\ªŸ¨H5´©¼pWI¬6ƒàºø1¶,é;7ÉL‰çu§Ê>¹Þ²™hv˜¨á¦< ²îÅSê•©<
x•qº7Ÿ×ÿ‘&Dþ°µýÿÏ ™›SŒÇRñxcY¶ÆIõÌÉ	#u.ïKPGÄzåbâ#OŠˆ@^¿Òî¿¥µºš8:¼lšÐ¡BÒ´;Á„,û¥y*ìÜEã´2ä)s?›¡{•·Ø•,‚Tï˜û<½v¸ùèÕ!@×¾¶ñ…²ÐŒ¿ÿ'ÌìTÃ‘ëÔ=ooxYÐ.Ç¶+pÇö&>öÕ{Êt²ÃñËŸ!±Õ
aYÛ<o,2/Í.u©Æ‚)áw<OG]„MÓ£¬ºÊûòµ<Ÿ*=.¢®Þðu'¡– Å¯Àh?hÿVehÄ·Æ3sJ€['‘
úÎ=ßöÏ¨+“z‘þÌ:û|"³™yS’îÓ2öòÿt÷-MW)8¡àÏvvòêÑ­B±ÔÓZŸQ¶wxe2òRî¯$³¬ûºë6ºóQ4ú%Úö¼²WhAZü€zdýßØŽÝr¢±œ
‡`ˆHÙ×‰:,r—bË9‡ußklK+èÈÈ¨’âÏ©ñuÕ°?’°s÷Ã.a`Ž@=“ñ®'‹ÜÕÄ#[9©Ñ¹Î6zX\C4´rdúØ¯kL˜¼)©ï=:Ó£KZ.Ú]óÉÁ68É£ÚŠã®ã=fJy Žüp|Å„Sú(;m`õ_ù`‰Wí2^ŒrÙŽ-ÍjÉK/ˆTkøuÂ	—dÄ7m†Ì ~É{é¿1^3'üXÇÃ>¦{ØÄ­iY-n~;¼åÕôÙ*zÜÿU1#PµRÆz93àÐò&ßpWœoéD/#²K{m)–xÚíh›n
«¡d×M*Ò(âÊ±àn«—™>¹ÀûsrÒ´ªBðÞ áR£ÿS6=í«Egã¿©ñ»Dez¢i!­b¡Á	xa:‘ YtÐhä ?|XÝ)ÖÈï¼Á(âwij£„Rº1:=™	KÁ¹ì]“v4~÷ÿZ¾gJmð#e½]46ã A•¦÷¢uù8Ut¨Ä˜)p×£h	œµGÊôù€ëÙ¼Ûúû
@ŸÕ+§WÌôÒéö}ýL ü¼¼¨Áèi“)éaà=$—t‹%®¶6	:í'¿¿õÇYžEiî”#‰Ãÿm>Î^êñ Ø0ò·jq•e‚²»¶’{UÈî’ú±#XsNIå*Ò³CS ¾NëWC`Ô™
îÍwŒ4ÎŒ°Öä¯·Pà?ûÂ–y'6,F_äÑ¨Á* òz·æŠ‚¦u·w¬™»þùµK‰Ï }àò1)Tï;’_ÇHìÿARöjÞwxÚ4¹f^ ë÷Î ÓlÆ;3?}4‘G/‡y¾$Nšçªg¸š¢Æ[¼Ú‘€oÁ‡‡Úóc¹d5=„p1¨×4´@öx¸ìM¯¡EC=~Ö9Mv;ØÓñ@»X.PëCó˜c5Í1=y4Å KÁtó”’¬_—ÂKWÊ_þj–ÇMçÊØM‘%±ôÌoNÐ¥ë’Àö·o¢a„ß²\.rÖ—Êt¸ö«½>‡¹e…:Æ©Ãný^rg3Õ´Ÿ÷•*	Mi5ccó%¦3epEéSïAÝ&å@›·{Út¡¥“-•¬ù=?vK&ß¯?³ªf_Y˜Å¥hE9®aÄÿÛ€xú`P1²íôŽãS=×f\`¢v3ÔJ¢k¿05HB}@•2«Õ#£)tI†÷"Í\ø™•c÷"M)SHsï¤Ö4™Ï»ªlåJ,ˆH*ÄÆ5œJð“º6Ÿ»ªëèD„I¼ÛY#šµn~:Å`[Ê¥.‹/I6Dßæm™ÈÇ@ÓåV¨‹1<¯—9 ¶ýESÏ 4î!Å¤þZrÁJŸqÏøQñ"=æå»ÕÍ706¶…`WBÆ±?Zƒî‡‹;)\á†Äæ×õ|F/§w±Nýt(SÅøM}ŠYÅÁV(±¼<¨Óo8ü.(ª¡,¼Ð?F‹'€³ô÷³‰@ üyÚÅ'~¤Cï@YAÕçÔñÌã€‹”¾‚˜ñC’s„¼0ô}9`ç¿£—Ö²ÔÞOŒÎnƒJã˜Kß]šdR>jë¤ßITe·º´DÞFú\Äåï#UPdGcoÑë5TÛªq9´Þk/æñæL€o©AoôÕXV~Y­Ó•oe2Éô}z½+Û_u~Ow\,¦“mþ·v
þ_®üþ6¿F”t
 ù¤y½òj²Í™ªù?ÚöKxÌ€e&õ˜û¤Q‰ ÄU´PŒó1…ï®äµzõHKµ»é7U­X„Õª
DÖ(‰j /#õçÌÛV1ÔæCË¤ÑŽ…ì»ø¸vð¾<$4GŸÝ:ÜÌJ8ÿAÿ"µ1Œi kliº÷£¸ÆZ®AÙdšÁœ§	«ŠtôKoŸè@ðÜÞ;+ÈhLñ˜%7m¬-bÑÍµäw‘´JSjQ&¼Œ¾NC\0ry	¼ÑÉÚ±-³Îäv>™w_µé1ÎåJóöÆNÐ-½}¦*ÙÅ‚\Ÿù÷˜MX¹0˜ç+'ãz1|¹¦ðÄ7•8LUgaL~Fï)ôÒFÏÍÇ ÿkê†¡QoOñ0Ý3·Óþ”r®oV+4ŒvË·
XE?Äñ>åd”ªL™ÇLŒ±½Ì‰w@GhÏrí]Ã´@¤n“3A×#vüÀhä„ÉË’ðú³aÝÃ3ÝÖ¢IŒØ_Âm#m©¤‰â±çô Ú#d`=îùp!²ý¿ÄÛÎÇ/pÉ5Ô°'`ª˜ífðœB—¾Ü¢-¼NÇÞS‰B¸!UrÄ6þC)Ô3½Íâ%ÆpÞ³’AêÚÆª£¯ýàúkÜz&qñBùX®n‹:ìLÄÚ Ñ,£‚ ¢uÉï¶Ä5
xÈor	.#&Z;'sÌˆûi
Œìs+U ž(9¬6’ç‘Z¿œíFjìÞMqSñ¤À®—ð“ò–Ùdf]|4HJ]°.§>ag_ÈŠ£`{ ¿×g`fK`&~ô[õ`zÞèr ½}¤~Ñßï¶ÎÛ­Òmî
ä*›ÎÛ™±8‰a„o°OÖ¦ÇÁt¬mÀDXGœÃÜf‘mÚi½V¸Í‹¡¶j½»é‚@LäwUâGç‚jxQn­T„DUp@Í˜öuY7â¼^‰»3áh~µs t.¯Ó´CÁmzqÊîQ¯ÍùÜs¾9¿Ígs.Òø„@?@—•Ÿá˜[V§øÝ„'X¢Gè£6Ÿ”?äùªœ)bèyó‘É¨•àRÎ$üî¤¶äÇ.­Ùá\ižÖ††¨µ4= 5Ÿž¨ ^d›öRòM¤p¥,€Ê²Ì2€ô›F)½€å¢‚Ótq•ZÔ Ô÷s	{”Ü£ð­š.^?;+¥ÔÄï(Ÿ©#A–Ó[–Ë§øõA-x’¬’¯ªÉêTþí{)Þ¬ÖérËjWÀézðqc<£_”/?Ù-f±/i›1­¯|¤p—k!u.³ð`‚Û%Ñtq–<	~Éåóà¹ "¤ø"±AbûiBí–¯‹ëI6œ¯ÁúäHºm’á„ f‘àÐ4Œ ­ø'
ï¦`,ø8@>²…®Äªgu¢aX°G
­*}<Û`Êº¡y*ËÏ>o¶•Â¡ˆá4*+~tããF˜­æ#½˜UöÜ¢cÆ-TN–>ûÎJ$ðÅ\î]ÍM5ÿÐ¥ãxi…|?
Î`nÅWb¦ÚŽÂ•9ê®p¢»{Šõ"gk«þ4§Æ©àÅ`ô4œ„xÛ¦^–ùŸR}W\Šp{Œ¯RÀÅ¸G#ºmvyßn!ØeEMæ¡£(=ãÄ§o—Å› Ú|5ÇÖ½| WvhÛkFKœºü/CÑÎ­¤äÒýú4-­v
I³k:CuqãŠëæöGÏÌê=S~0 @M0i
T¹~¼af4³ï"x)Qs‹f»'fi}þ¯÷ˆ#3MKŠü×¶rŽ¬>ûVål}aS<¼n×wÚ‡Ðós]ºÉ'L#ˆ³vBSš6c±7SA<‰Å¢èâ÷Ø`iJ‰ÏõWØØõx1œj8•ƒ»á»OéµªÖ¨®ã°%„í©¨Öv‰°ºò_
J¨–Ò.‘Ô½ð;FÖwØðÝ¬Ž—ºÑ\§¿Ë2aUØ[Á¹.ñ|}¬KáÙnÝl>†5wì¶„þ.èc'ÔÌ¥†ˆ¤.Óø××ùÎãy7ôûkÆÛžËöV+J `“ÿ3;;YymÆÞŒ}Îj‰…·ñç'må†=XòËƒìË’*+Ñ¦	1P Ì¸aºH	¿€Ëú@©5 a‹øô…o¨*‘z©åÚº¡¨â0ºÂ.JèˆÕ«ÌÿU5iä!®Ã6W×¹.ë˜Å æ´R8%øô*V¾ê^m	XÌOãf5×žMÓ1ýÀ •Š[p©7€6Õb§±õ§+’_ÃRwÌõW)"ä¼ËÏ(æ	þÎÂÖn~” $Ï$XÛÆÆñ™pPL)oˆ‡aãGóÖƒ‚ÏØ$<¶úî¯:ÐíZÖ´dsødül¤·¼K,Á³ä®ƒ>’ûvHv4õ¶ZéøhTp)f§‘F~t*†(g<äñ>HŒŠ>´Ÿ–ÌÁÝº”é«ÿ_Ö£"÷½‘Gœ¶ÚCÚHV¹Aâ,**q§ X¾é–8ÖêŽXi®€Ÿ²4Ò{¾–Vð¦ýd•Á7ü•ß}/…Eâßš‚km`õv»ê7´þBB¯²¥Ÿ¿%|ì
=üG…9ê6œÍf‘ãFYt¼çú×ä„2#$hTÀÐ²ãòÄJèž‚Î¦äË¬àåßYé÷—ßø$àHîÉÆmÔ7X ‘0ŽLjˆîÌpéµÖÞAeRÐj·Cð£³JüMb¹˜t0ÒjeÿÒ‚ØãÃëíß¯m6·CÏí,3Dëaþ›”sšE”0}#ß!Ï§™!ÏC~ý(üã0‹„+ž%£Cˆ]E¬Ž\=Äº¤ÞòìÔºO¡ê(ö°Û¹™„. ‡zÍ`Ë"l­¨»–†`ÚW`iýPç/JÞ~Šœô ±’kü#$Ä¦(ypNe¢ÒN¶™¤¼G\ï<¶(ÐÍëjC¬Z=Àw¼½iWB¯ü‹ö•ÀÄ(×Â¾NËú<—õŒÄ§CõñV7VAê_¿–›;J/|>K'€È/›lcHÒ«p,–ûóikÖ„hÒ»?®dL¤eÕ›ÜF"ü©ÅwÏÓ¨Ç¡9ªü
ñ~ã1¿:ÒÈ?5¥èze–|ºSüè—¸GÒRG=‘æ—ú¶šð÷—Eô	ò]ÎÜœõ&AB¦‘-H‚¯€Š™ö)â­@?¸YƒÉl¹ÿTƒ”SfäÎ<Ë™|¤¢*Ìá)NoOx<ôvùÃ¦,¨vEä·‡"
¦\÷º’,G×ÐÙufK£ìN@xÃ‘eÚÜý¼ñ" @<µTÅ±’è¶¢N’3™PCæL³Hø$Æ&hlR] ø‡ÅVÝðIÚ”’?CÇúÌÝ{èmwÔýÜ¯	ŸgôžLz#ŠÍ¥H%«íÖÅGÎjÏ81§°{æ¶&›·¿Í™+‚f³{-zûQƒJwË.²h§c¯€‰° ÄH²ãÛBÎç²rÒj°ä›ÐÆ8­fcÎ+øíèÍt°‹—_.“ö‰)ÙRpBõ²{™‰¯æ¼þ–Èñ‡àÔW³¯*2bfTŸ±µµ”R\Kƒø×¼'H­”¹%ë"Ê!8@„ÄHAx¶Ày2LõñH×‹t<ì>XÆ6"’»ÖÙxODŸôí©@ê	: §ÍýNß47÷.	Þ¶]¯Ó0%²«ZLŽPü×ÝÂw`':uÜ^ç¿±ª”Îã&0û6kX ×ˆ‡_ž{¯0LB\†›%c®ëøpÿí3q¾D×1"1ÎW#9ƒ§½´‰ýöà¾;@ÛÇÞÞSix '%dKGåWùN¶I-y5–k4V}8ƒ.¹¤nY`ËÛðŠ8ÝY3Ôª&Üõ.‘aŽ)/þHO)¨wÚvÒG¤ÿîþÙn´ÕcÌÀF\81­*å-SˆŽNõ`n‹so3ÉèÐ&»$„'fÐ@ç±5ú§¯Ü&ƒí£§ŽýÜ®"7Z-Â·TîO§ S¥õá¢Z¨óÂM„MK‹Fä¥Å…2-èAkÄµç®qƒ]x¾®]£A%Ðä39Ÿ¸>
c8÷:‚öqù6Nýs`·ÀtM^ƒ£Žh¡NX­Uü(àJóÔ] UèõŒ5¥÷ßWÉñU¦ð.g‘$qzýq?jŠ‡ _¯#i7ýþ¤{üKÞ<î+Ó&8{ªXX(âRÇžÖ–*øO×â¢rUSäÇeÒcz Áý€U+w…µf3A±íKºìÂÑ1
ø5ÝËÇG Jª~rŒWIûÌÔ¹§›¿R ~íš‘ê 2òÍÄîÌKªzeˆÐÆEÍ”.kôúˆâéš´òËýÓ¾€žw~	Ëò´¬NæO!„Ý“~mÎ§›X²#:¥¶§(S©Ûeè×¾¨WùÂ„ø¼‹a/ÄªìSJkdº–w6¨5kä¥ãMŒ¤¶	r†ºémØqÑÅW4€¡Ëãy½êýõ=•¤ðë!ê	›–~ÉÂÔP~½0­^ÈsØÉ‰çÔCØ`9ñÊIŸLîŽyìF8r´0¼èüú²8þ§BQ.]ø•³§…®ÃÇ %,04eèzõîÇ«l—¦s›J²‰JkÜëÑ1²ÖR33PFÕÇ´£5˜œ‚º±l…ùEeªÒž¸tÿáL¬Ðƒ™¬Hk¶ìÂ(” ¦æ”«'âqFTÔO$hj{råððTî¥‹Âd1tyÒüSK8²`*Ð¤åkè_ê
¸MÌ»¼ëî¹È“ïÇ¶¿zTŒÕï>"9lÔüŠÓ÷c,çUÜˆÐ,¨Î¸~vÿÎûÚ=L<Êòd%ºrxäB]íà@â¼¹iŠ0Bs]Ÿ;Á™!ôåí-F“¯Ü=«£Å¨p3%`P,åmwøÿu#ÞÿÍRI˜ÔÊeÐ“Ì¨±NP ÷¼ðõˆä+ð~«Z‹ãˆš"èd´Ÿ
[ÕL¤X¥üÜQ9`£Ð?¹ÆUt=2°¯1 ¥S`×‹¾þwìµ€%`AH–çHº§Å™ÇI”·w¥"m*‹%ì|“H7Þ¦7;j‚	LD,âëwî¦ÐiBíqO©YUF?qÿv™×o¿nHïnBkµ•wrß	¢B #œ5hï2«C«7{™¾Šu(úª€S(ñÅÄøx~xkÝ]±éë
‡NÞ¸¬°›M?ïL>sî“‡žHO*7s*üÖÐÐ+Îú_¿±¼­òz„}5¥…IC¹Ã(Å% ]Å.mçÈÿ¹ïð"Æ8µ$ä“ò,E«ßò£Ña”qëlš¨¤sz’CIjÿï#-Íè&‚?å-J%¸ä?“«‘1¤µš:q­8½¬@U2cT?sˆRÚ+ x(:wt7Aøj‘ÖÍëzÑ¢ÿ_H÷®;žYÒ%(CgtK{,ŠT0“Ž‘é,«ë‘×F¦)ëå}ú	ýz¨Ž¡Õ ™¤¹ô‚CÁNçm‰	ëC“ÍM'qPcÝy_¤3[Ì¡àøàäWø—`E$A2†w$hâ‰†IÒæ0€=@Ôb
‚Uþ6ÞWaIyî6G* _uþwJªçÔ Î²4"Rc¤r½dLë|bî±ž7d"j>ô¯ký\PM¨YãDå¡ÖÁú4‡!‚0êQ.9Ãgí7møã±ÊH›«œÏ°z{¿ý-•'¯Ÿ&¾ó8ÔGUš N2º9sóö«q›è¶T\‰{s­Ñ“)óýyAT”­œß¸®–£øeË²]Á(¾o'Y4í÷3”yuÞGªQxU™;ˆv{Å,JQêJ×lìÜà“;j6ôòô`£2IÚýD0‹—¦h?ÖÍNi[R¥ëóï<²rÝÜÛ’nï—0Ëtb k—Õô|ÄÎCEaÎ›p,&Þ…n|ˆPçÓ•Ÿø¬1WÀUrÍ°\µþ
)2F`½4“¥•7z˜¤dÞ@*óXàšÈ´Ò™CGÄ†wÍâ8´'Ú<_ýÂ§ÏuoÑ[ ¿¿›ûïc•ÑQÔ!®§\ñ1ë§åóòæ ©}˜aàV²*Óôb=ŠO%¡õÝ8Ó~UGñ—©(–Ò¸úVØªÃÔŒ¥”-Pð<ú/û÷r*wÅ±Õ4þ…pÕ†T¹f°ˆŽx¤(¦î6ö±/œ¼%äI`>Ói&ïŒP5Î‡ù·…[Ë^ƒ\³™*ñ¶ùž^!¹;£:ð»ÊšÁŒ!:×ýå`p3ž•qL³^2L=2“€Î4ÂƒÝF{~Ñ%òÙb¿ùCëÄó=¥ãús†a	8ç~tûÄ”¿÷UÆqÊ§ T²4uzáí†ØÆîh¢` Jù„+—\Ë‰O©“5Ó¼îòþøJoÈÆ,·ô¤‹Ò©®RV#½ úÑ@¿Ÿ¾éJyW´µáI¦ÞÔ²æG#Fÿ‹Xî~žÿZÎK”Ë9€#Õ
YIŸzfZ®ßû±·+	¨5±‹ÚÌìn`öÊkÍˆò.›$ÕŽyb‡ÕRB4_©^-}Ïòê6y—ETÑË¯ñïàj§N][–Z¬Âò~ƒhN¼ŸxwþÉ–ÏÎü–÷à Ï›ßhÏ ºr[mðk.:³?Š¦3^÷¹ƒËcòNdÃƒ+7‹öå¯óãG[ÿšÚ$/ßp$+xäIÑ÷ƒ¹usŒ£ú—rÄ¹Q%ñç9Á™Ä_ž÷é Èf^éçÅ'%U[ð>¡HDsgA-@‚âþ¦ÜUÉF+#O÷¨4=“V+ûq0eŠì'Á'e:å°ÄJöªâÁÕ±gï³Åh)Z‚™…â¢w,õÐ9Dƒñÿ§Iû©ƒñocèï9žØssUúQ#Y(t™Ö„Òó½Ò/Y±Yj2÷þH¾&–ñäÈÎøŒtÝuõŒ“ó`•àñò‚eƒÙÊÜ90å7	öž†¸VVÕãã¶*\js?&•‘+LÞÅh5¼$sÁåèT
»‰LgFÍPé÷ZÃoõä¡~æªÖ“ãåù—5òÞ²¤3”,×ãx&QÄ€Ruã=6e56ŽÌÍ«71¡Ü¦PTÉèÏrva=¨[«ùäëw·=å¿ìm9&i§qZê¢É¾ä~ rä“†‡,yEŠŽBQ$»áïÐÚàëqÑW>8´iküU›[™Ûñ—`¯iE$ÚAeå–v·žj¢OúÕ„EÙI‚)´ìcO†’‡ESãþÀaÊ«	‚ƒt6ÃÃHÎWøŒv
ÁFGÅ›*ò„ƒdîC'óþ;æ°ÓŸ5|ËxÂIþó‰ •œ	^#C/jÞ—"åç‘-#ÁË,F28¶.Ò:VöôG¯&„i¥v¯«`•A\T_Vwñ‰#DHþÌçH‚]ƒä™¬7é0¥žwÎä®N¾à¹™K·›r@ãEmKÔÑas‘Ée'¾Ü™äˆ”‰2ÜÁl kOg\G6×ð´U0Á­/hðJÈ..UÃ‚"mŒ+ŒvB‘”F@|¨©ú‡M­¯Fi	†˜Ã¡%¯Ãá„žùêzTZå_Öìf!•¢}ï¬¸ns*>Ûûq/ÍxWåiáK"Û¸]ÂB-û„3%»rË šü)œLÆÉ¿’Afõ²üDy[FéMƒï?Î[ÉŽ|€Óž·aÊ
}ÌÖ+5{â3ÊIðÐù”×â‹È!§…–Ó‰æ%S¹>_ã«-ñ4@pE3€‹BŠU!’AÒ—ˆºÌ÷;ûûfè‡ÕçºzZ-2g¦³#QÞ)õºsaUõçr«fÐ(d}—×ýœ[|a›ë@‡ŽƒÐÔ©	Ÿ"&Œ…Þ¸ºýyæEó¹!yVlK@»WpÆ•vI÷z:ÛÚV2Ö¶‡Û|Æ,S5(Ž`™Ÿr·à"ýµIèÀœ~™²ìŸõ†Ò%…—¸øƒê‡†¢!*SÙdc’? è×¡SãáØú¸í&ÔÄÄ	‰û›fp!òî~¬Å…Ìd(½šËk‹SAßÀCÅOôCXÃ/;v!ß_Lh÷$÷%>`$­»2Šr" Àø	=¿¸«‹ò0æ~Ýº…álõ$
$5¤œ®2.¦UÞÐŸ‡±à±€¼+A¬tü—îŽÐ´;7J!£Žó
‰÷'R<[{Û:ä5Ùÿ<œ°¯1“+³ÊJz‘Mæ÷× B	‘zž)ÁK®?)D©	iÓZ0óÜdF£ëb=BKÑTå‘øÈ$ùw&ë]fäÖøDÃ!`DO0Wi*!™b_Ú.|@ÓéE²æp¬ýðH`@Ïˆí9–>6N8f’aG»î—¨I’É—“ÑÔÃ¹ÐÓæ8ƒ¼ÙÁ¸hU”ýeþºR0Èß¨‹¦ç#Âé7æPï$ó0ºÜ‚6£Î¨í¤ŽSä9ª©üW|Ð¸14’LU«*ú\ã
ÕßÛÂY@f®‚Uéž÷ m¬Ì¦‘éÓ‘èãz›·Æ¼­’³Õ*Ìð;OôQÐS^$À	$Öu+É¬{süìw}6z[ˆ_ÜD¢…ÿ,Uàziƒ
Ê.RÃ0yŒÓkÿÔM¢?X2fÕŒˆÙZâ©ûó;6'•õTÝL[ò+JmHV…`