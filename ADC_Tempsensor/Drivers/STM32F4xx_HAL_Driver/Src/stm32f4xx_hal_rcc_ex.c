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
e�t��ZS��]W|�ZL��U�34Wqئ���$�H��>Y����z�*�i�R?��^Q�WH�/����m��wa�f��Z�R�d$�R�����)L�!u߱D� ��kȴ'��/�
�٠��Eb���"��G�����9�VSL�m<Pý��dy�.K��\tlw��N��`���e1�&�� ?.�ո"�#��]��O�Xh��H\T���-R�S��)2��]��Nk�(p�Y��
�	�2h|�;3�e��&�f�5f��D~�,f�L�5I)%*�$��@%I��qYe�
	��B��#^�D=0b��s
8����w>�(�vqU9����awU�"v�o�1>R�@��(�����O1<O��S���Ww��S�KO����|y��2��O459�pnC]�g]�}ʽs�}��3�z]z*��~g������ԓ�n�Y��	g#�ch�,�6�9(�G���FE_jO,�LA�6�ٲH���y���H؜|�r��-�]+���t�S64F��$2 hK�|�8��:v}~�NZGwθ��go	 �"r��ц��\6�j�C�E���j�b�^�C@hm�b��,�x���>A�*����Q�������/Wie�>�	����*CJ�Qi✷�X�G�y}��r�t�=�,�\k.�t�MTDq����s�l������������=�×�6dr����~ �ϡ��(�~�oQ���0d�Wk��:��J�&��Jl�X <Ef�/Ƣ��K6db?O�%�/x �U�\��Mk��7�����`_��RzjI���P]����:�7�=b?4��Ow��%-h�.�U�;��!)z-���]bt����8B��,mH���F��P��gzl���6zяc���d�EhB�����v�˹�t��Ij�C_f<����8�&'X��q��`�
͕�7���y��;���6���0^�7hQC���w9Wi�|��!�ۜ�0��k-RanW�?]5]����V^�ءO��Er�����F�\�|C(�\<���4҆�ٖ���d��PR �AP hvS�6���W�}���a�����G~�ޓ?4�ά�ť��?�m��d�ܳ��'��z�P�V�#^k0��3⮝���_ۇ,�-������2�Y3އ�?WJr�ٗ�t���63[��ܷ�L]��<ɛP^�2ӊRy�'sܫٔ��q����Y���5�6WZ��y\�3�]�M�G= �.�$�A}�ִj��QEsbUF�Bi���/�Y�|�,9�69�t�N3�䅢��q���>&�~s���}�D�:���������k��-X?����e�>��߻�7X�f
5SsA荖z���y%�];)�R�\�WI?/
���`��B
ć�w��5�s!� �S�*��r�]�&��-�i d訥2[#���^�iT���d�+�]����-��g�l�3{Y�~��ug�~��{�а��SO}a�����/<������p[��xKpd/]�E:b��H��=%�Y�p뻝��"�ꭀ�L�_c�E��/]}���A��n珗��-�iu1�
��򼓑fV��~����Wq�����[k�k�����nX��L~��ƻbog��5���x�ŋ���-�D����aRW�x�� toQ[Z���.ud �%MtH�`7������%�IJ���ߺAe�&�T�D�%�R�4�0�*��L�g�QT�[�G9%�-��]v�D���	���k��\�7�r�Ul�i�^,0�z"���Aݐr�w��@}R�,�9땘\�TAx�=�f�7f��H��h��39��P)���a<Z$k�Ӌw9g!^�?!=�,e8.t=��r�ˆ�1gI��v_y�����0̶�ώ �L��m���ۇ;��2"�������{;���V�A��&z�
H:����t-�f���=Ө��8�LV�����.���&��)3yڅNn$Yun�pL�Vۇ���hь�9�U�^�3�:_쫆^���iȥ8�Ш�����5>◼O�m�3c3��S�e�G|�@�_q4���O��s
��x�]�g5}�O]�)��h��¶㔀3�<'�Q��r��8��P�iǔ1X����h�k�rϺT�	z���q������>-L��/b��?�y3J���.����Q��?Bw�@�:�J�_ �U�՟�Ku�X���FO�)��%c��4\�8M>�f�P1J�&�lb#���4�'y�F�	�pSPy\	�(M�٨���0�y���K3�U���\�/:�z�:��\B2��Uhv��S�V�B�!�K�68�&�a��
TQ��#�c�F�����S+����c�I�IB��Z-/��:������yӗ(.����*䆋r,��I�͛u��p`�����-��#[��mj���:� f �%%nDJ'�#qs\ѲB�;��F���ݴ���_��J�dE1�Hh7��6'@m�16��^�t?Y������S!���+]�0C�[�(Gت�=2��nF"1)�5 ��nI�ue-�K�mY���Y9��̇�9,��S�Q&9ql+@�l��ad�<8U�X~���l��}�T;��ZL�5�U%�QP��Iaʲ+.��?�eł5{Ĳ�D:���^��mw��T��UF���RH+^��Nr �I�?Z�y�M�5����ID9�>�wkf��ȵL������+Rȫ��B�����9C�b�!���]7S^�%�C�G
�\|j��B�}0���'SkX��
���+^,e���Ǿq�8�~8��1+Fh�)��ۓV���~��f?5U��$�����HH�l�f��קOA��3�O��3��j��@�1�JHK��կQ�a&&3��li�R�8���*�f�LTxRl���?䏶ww<���4<���^������-;u��Eg�T�#�Ɣ�ɜ H~��3�c~.7m�_���)��C%x�ߥ��� �����!�)�)�7��έ�;�2��DH��&y�m��۪΂a1W��[��G��p2mA3Q1~�R��o2�:�)�4�+����~
QQWw��C�o0@k|��8f������������J
��c~5̾���zʯɃ�m�ayvIoQ�s��	���Upg���VO</��-~�}��"������lߦ��OAL��+X+9�I�'z���6P�TP��F{ܷ>����L`on�2㾈q1�̜�|�#	��K�����[c��kL"ǣt2��_Z�A���YV�0�m�+���Y�S�7�$���5��H}��a�,n�Q�)E�ra���*�|�4��y|��l�y�Y��n��0Ty�'���g����Mc75����+��8�������Y�~s�&P߱�lYk=�$��b��hX�;�k�2Tt�6��Y��� i �s �[�Ҟr#d�&Q��c�iK�b�5Q�7�<��X(c(p���A�A��A�;T���U�r�^�
��q꘍K{��J;�?��tg�}č	���ښ��GA���v��J�ʮ����3����ic!816���üĸ�F�;ԡD�'	�\3��#7���$P6���H5�RI��x�s���x�"�sv�GBƷ!��jꟶ�nu����&�� ��F瓲<,x��%�x�A��&D������ ��S��R�m1 �����AQp{m�
�7�0C�+�[��M���������v2�#ʄ���2ȟ��y*��E�o�s~��)����j����s�"\����c���ʁګ�� ��0���r<&,��.��op��tx��6� ]C���ɈB�qf
�n gw"�g;׎�T�j<G�]�M�����᪶'�Dx|j�ꊸ|'��=���bj �Sf���gEmF�^uJ���3U����{�;Ż�u�5Vk��SS���2���)�-G})8���vv��э�B�Գ@��#wf�H�O����ʤEx�Qd�lB�ܼ�WhAZ��zd�	��L�U�u���
�l�4�ϔ<2/�y�03�h��e&E"�����'�Ƕ� oE��4��:��r.��=�(��h��#i����e"L]#�zms���g(L��)��=:ӣKZ.�]���up���κ��=;Z95��0ĕ��|XP,3�1E�@h��|N�!ٝ6��j�8�k�t�A�'�;Um�� ~�{�1^3'�X��q�/���n�%Pka���ϲ�T&P��U1#P�R�z93���&�H?�;�Wkz�Lus)�x��y�5��d�M*�(�ʱ�n���>��ѐ17���B��I�\�M?w��?����W�3?�t!�0C��#xa:�/Vt�-�Dz8X�f�Ƞ��/�rx$��[�t:5�Zס�w�v4~��Z�gJm�f=�G4e�0k����P;�}T[3NQˌ�{$��fI��L	���(��ټ���
@��+�W����e�F1���.�Ƥw�}�K�=�$�t�%��6�	:�'����]�:��b�ڼ	m���l�9طjq�e�����(G�����#'1@�k��5�R�F�}C`ԙ
��w�4Ό�ַ��X�?���;)e<SA����*(�*S�[�ٌ�:�3��ر��I��\J��~y �t�����%�2�x�h�J.E.��� �v�;t3|Ju�W?�-�m�ͪg����[�ڑ�o�����/�&pq�t=��k�L�7���EC=~�9Mvw����Jg���c5�1=y6�X�g����__��@W��$Ӓ���.�i���NH����[[����|���g<���8��ӽ?Ң6l�:��W�:�>(g��٣� Tgi5cc�%�3epE�S�A�h�M��PIP�?���y���rkKz��/��0uЄ�Ofu�5�R�תx�oP�c�ȶ=�#L!�77��-�b5:�k��Zq�h/��4�\���c�gm6��O�'����u�@#�H]k��y�SO��R�r�����
�I�f��!~S6��a}[v��bP�#Iz��!���W����bu��0 �8�oS� 4�!Ť��Zr�J�~���w�$*����.1!��`\Y��?Y����;8Z������z>W�8�H�s3NH�ķ�Z!���mI��<��o8�.(��,�� AO�*���_P���X�dӖ�b-�i�@YA�������������j���fQ�.~x����ʻ��Y	��hE���[��]�&f/���ITe���Dލ����>U+Lo��Ct��j��k/���L�4�Ao�(XV~Y���oe2��}z�+�%7X#,��9�t
��O��0��=_O{�+���x����F~��c�e&����Q� ��P@ú������p�BZ���9����T�H�&�#D{S#�眚QZ1���������Y��&��}`8Gٜv��C#3�A�"�1�i kli�����Z�N��%����DF��=�t��	��0-�!k��.3l�bw�ɵ�<��ISjV2���L@bG[;7L�ՐP�ڱ-���v>�w�_��1��E���DZ�"�z�!�ׅKҵ�^�>�]�x�c?V����7�8LUgaL~F�)��Fτ���(��W�QnR�c�v�˻�+�wAq�A֝�X��>�d��L��L���̉w@Gh�B!�	۱\E�� �=A�3Pؐ$�бǸ���a��3�֢I��_�m#9O����W����S�bxiz��)n���p���+a�Բ+`���%Ҿ�V�E���h�B��Z�h�!Ur�6�C)�:a�%�-���A�ϱ������k�z&q�B�X�n�:�L�� �Rt��r����tF=s�Er	.#&Z;'%�U��:bɭ7n��98�0���	����U=��V��qS�������df]|imE.�a�{a3���/4��7`kDa+}�q�-"���w�ESQf�5�ɰ�6�ۭ�m�
�*��ۙ�y�Y)�=�O�����k�.�I���#�)�;��V�����:�����#[�W^�K�@w^n��Np�γ_Y7�^��e�:~�' i.�̳�[ �(o�	�����eQ�7��U''
�@��\5 ��Ä�U��Pݔ.T�H�u��?ҡ����lW+�l��۞�-��x�$���.���'ݞ���
[�y! *\���Wm��R�M�-�����2���F)�Ҡ�؎Y@��T��;j/ݓ����QVe}0ؑ���f��l���D���e=��������>jV�Ϛ�!�jS��)��2XI<�_�/?�-i�'l(�t��H|�,�3ha��`��%�2$A�h@1���|��lv�+)�9 �[1_����H�T�ʙ��S�m�ᄠf���4� ��'
�`[m�}A=���٭);�#:��c9h��k���wX~݄�{j&�W�����*+~�t��F�����)��U���%�%Y&�u��-�v�]�M5��ڥ�VJ:�D�(l5
�`n�Wb�φ�a��9��/��"%	�:�ꨧ4���xۦ^���/W�H \(��?���q�,/3�@n�!5���mYz�Q��=v.Ӏ�yE�;}�͗v*Wvh�kFK���/C�ĭ�����`d�E%i��#14���[��6���=S~0 @M�0c
^�T�af4��"x)Qs�l��As4 ?]��Io�qGbKظ��n��z��F%:)'�n�x����s]��'L#��#�,�7KQFƖ�����.=۬����xj�j8���˻O遵�֨��R��P��I�7y�'���.SZ���4�հ�:B�H������ٝ���;a�[��.�|}�K��n��l>�t9� �T7P卭���3�̚����ɢ?c����ę��~c4�B�332Yc	���/�j���T���d%��f&r������*h��[eM ��(�FJ����[�aGP$���V�� �S��z������
s��|�X؁⃱;=�n�޹u��� ��j%��p��<V5��.a؟G�*)�� ���[p�7��6�b����+�_�X!��Y23����
(�F����tc��0�h7��P���pPL)o��a�G�փ����$<���:��Z�}@E�0:�*�>����$�����w��\Hv4��E��h<9n.��,N'$�`&nX��3K��'����������_֣"���G�����C�HV�Ap�uzo"��T����y��O�,&����]i�{��V��d��7���-MPa�L��>��kmj�v��7��BB������%|�
=�G�9�6��b��BK���ӭ�I�%|eRX�Ӵ!�Ї�Tʎ��ݟ�������ѬkL�Ⱦ��mC�to�I�Lj���p���Ae�j�C���J�Mb��t0�I]y��������߯m6�C��,3D�K���s�E�0}#�!���!�&�Q�|��9l�_S�b�f�H� ��\~������ϐO��j�\�V��Q��i7�3�`�~l�����4�BW
`2�Y�4`�~���� ��n�)��a>�p*�����SK\�;�d���+B�0�Az��r�$}B�������(�¾N��6���ħC��V7YK���P�;`/wd��}�scXұzz��� ��$JY�W�p�7f�e՛�F(֩�w�Өǡ9�F�1�I�{��xp��(ze�|�S��n���v��´�ι��E�S	��ߞ�i	)B��1����ѳ)�^[�v\�ц!��DE��h��<˙|��e��`C'Ux~�.���dV�% ���"Y�_����~G�R�S^�u2�	R+��k����V�k\�C�������N�v�^��H�Lg�u-FR] ��Ƌ��F���1E��Ɗ[�}�o-�����z��At2�ۥ`�W��@D�jp��3��c�����Hb�5�{�y?I�\�?�M[P�;�9ꌩT�a�������!�(�ʅ�v�f*�n���K�5���[q״�>�I|�|ل�¼�Ӝ�ծ��U�yw"f Л���R\K����nO���4�z�n�QqA9��C<q����=r�>�6qױ��i6﻾�M��h ���N�47�$	���Nqf��2G�Q�؀�Shu9����W�՚�cb�^z'a�ü _�/��~fB\��%c���z��3q�D�}q~�lmbk�S�������^�TZX54J����7rjk?j�9��Y�r&c#�h!j8�<��&2�۹O�}�n/Ӻ&��E?�7�54�E^+y�8�\�G���!��n��c�ʐ&yc�g�TE���`n�W� |���v�$�h)�������&������i~]ίI�Z�tHF���G��Z����M�gK�F�Ņ8�M�Y�F���7�+��PMB�y��X`��m@X*v�f̣=�Gc �6&��e\HR���ap�*��f���] U���5���,ҷ��z3�`qG?�%�p8��A�f1r���<��s�U�itN^+�XZu� Z����o����i\�\�a�o5S���O&8��aP4X��H�J���eO�f���G J�q}�	,�������aP�ԑ�7&�����\�zh�����ap����ƒ�렩����80]��S��5�RM���Z-$����4b���.OZ��,�ۻ�������{C��̮B%(0��w6�5k� ��G!���h>���>�q��B}����p����=����:�
��w�πUo�{�^E�s���M��f�c;�ee���+�6S�C@y�x���]�-��:XV"A���픢؈�#5U+2b6P�?��ͫ��#�J��9���Y��L)L#KM�纾-���Աk��E7֝��p��K���F��W$���>G�3Ϯ��_�[FT�@hj{r���Tᯁ�d1t0��[	wW�Uk��)�S�]�Bà����ã�B�.�ջqmuP%��Ý�r3�Rǟ�w�߾r1�ϭ�th��dg�-y<������#�q?C�p��h���e���3��P�t8},�w	Q��9oE���4֗A�*\��̨�dP����<���+�~�Z����c�*��{�Y�R���)&��;��Z$4[#��Rs�/K����4���?uAT��K��Ԏ���w�jzAM�g�?�Fަ7;j�	LN�2�@��i �)\�Y@#*�s��i�+B�,
'��fh�p�EeS�5c�J}�_E�'��=k�����a����x~xk�]���
��] ��²��D&�pd�����P�o9T�Ю���_������_:}�[�_�G�+jO�g=������"�8�.�ܪi����3�B�`���5�p�BEl��51��i�Y�dU��v������u8�l�X�fh=PE�,�oY�9}v#fJ'A��/	����9˨�^��C��l|
(Y=<,�Td�����������q�[�5�[�P���� ������N�~$�A�ҟXW>x�]g�7J��������JE$K2�2sh��� �٩~�{�b^�\U�y�YzX'�S]I[=�]J��� β4"Rc�x�dL�|b��Gm*h9��)�L��^D����g�u�ER�[Qo9�M6�m��;��H���Śz{��-�'��&e��}��e&�i���S"���I�+��}^��<$��΃����d1����g�c-~��g�71�{�;KD[�5�92�x`Q�J�-��W��435���`�2Iښ��Dp���$k��@:��B��F�;����=WW��|�0ba%�դ3��J5`��peD&��"0B�P��ǟ��O$^�9r����
)2F`�4���%��dG�k��߄L���TK� [ʘ2��$�'�-E�Ѧ�&}�L �������g��]�p��\�t�㠡����4�)��eG��23�)���'L�ve�éa��K�|ت�ی��J�8�z#��{'{���3�q��L�u���1�a�s��`��q�La4�>=��@ �F�����^���yŧ
ٗ,+�6�S|�ʚ��!0���jz��0�}�e�� �r���"8;�u��,�����i쵿s�.	l�;8�Ώ���Q�+��"D�]�4(���N�_���<�4�J�����7��}������XV��x޻�\ϛ�Z��|�#� ��@���\@����EL��ɦW��|�h��K�W��&^?�,�_O�oy�:����{Z/�?�뎕�Tn;A��)���\�w��+�_��p6�b@}�C��?�*T��鴽�$�]��O ;�
}U�KU:l�YʩB��WӶ� �ߛB �r�|C"X�f1b��{��gC����m�<��oc��g����Ȟma�$ax2x�I���\�V���r��T"�1����o��gI��F��hk	�q�%!E�鯣�i�-of��o�VKQ�!J0>Ĺj�b7g���J�����(���!�'�����w*���x����N�C���Ft��H(��4m_�Z Q%̗^�Ю��f�"s���siٽ����X�k�f씔�г���Q1����[qG�[JO����M����o: ?b��g	��htR�r6���	[y�Q� "l�P��Z�o���M�N,�w��]ϫ���p����:� ��xY���D0�Py!`z�Ĳ�xu���ҪE�W���xq|+�On䶬�=G�1�ē vb<�4	s����qr7����+d�<�|����̡�}�(18�Kn�;(�K����d�eYd�il�>��:+����F�c��C���
���2f����
��P�P��uE�JZ��-���M�D��&����x=�� ����M��ZY^O/��;���~Xƨcw7�zP�sH��I�_��vC��#�_7Y��BHH���pD�(�YΙ�x�u��>���%����J��;�Ee	��hs��o'���툆
�"r��~1k;(B#���UE����wd�J�..U�3�"m�+�vB��F@A����K��CF^��
˱h��𘂥�SluN���<f!��}﬇�ns*���>wg�dV�'O�-�Gx�%H�e��AD�����)�5ƽ�n�2UȈ�J;qz�i��@�=������uo�|��*4<{�3�I���ќ�Κ!����өf�0��hD�qD*=3���:c��È�̖5��/����]�zU{%��f�4��a��3�*�G4)ޘ�Ϗ[o߮M��׿�ɳ�uE[՟��G�~���s)
GlQ�sώuM�f��Q7{����:��m8�`��h�B�cI�����r������%������D���R!o_�j'�yI���_������&���	̩�55!���>��̉1�҄>�A�ޕ�
�UYH�2yF/�HV~�|�kj`[m�C�3�CW}?x��_x�쫖�E�8����:W�p5���2.�_V�����O����[�$�ź�ʹN9d��_�LK���>DnLw�B�{��e��^�p�b��+|�ﻅA!��(�^�S}D�Z �p0��d��7sU����*�$@�%���
�A/Df n!�S'wG�5v��@F��9���Jt@J��m�wtl/P��ɷ�����SR���������l�����hU��e��R}���Φ�)��.�A��G�d���8��������z�̽4Х1Aq��i�9�]H����Y@f��U��P$��ث�v��ܼ�E���v�<�u�\�Tl�5e�N +ϱ2��]}6z[�_�D���~�5?�o�I`�l7���|ʁ�M?g����Z���;6'��T��~R!(V�`�������ؿv�ּ>�lfA�D���[��K��}���<�_rl�����u[�� jz;��Iu���|0/V��9&���ߌ���
�)N��/����k^;���`�R�����s+�2�Ū�� ����J�0\��6Z�g�����_&'��f�ϿV��f90N-Ћ�4��
�A��S��_����s�n�:
u�����U�q��y�9��YDWS��B�99����1j� ]�n��M�gSlЧ5�]�u��X|4��؊nx���8gt����z�d��e�6���*��JUQ8��Re�۴H��U.a�G�Џ{�qւ���Y<nGIJp-z:`�� ���Q��������n0�X���c���8��N���v]�"�8�̥矠����ց��ǀ��>~%�:͊���르21��E�zn[��¨�q �4�%"�Z��sq��MQ��N�&���0I@��_0�q�D��b"µ���9�Q�W_�g�r�N�re�~�9^/��Y*	c��/O+wF��TuX"�CЯ����H��I���p����:hT�� � �>)X����9Q�kY	/��4众��Q칼|�8m�iv��4�D38��5�1�@g��0h����0�g�)��K�x5/�.�)���z%x,����ǫܓ|I"��"�Gq�Q�#���O��6�eP�!����}� ����"�{��'/V�	n��,�u����s�1�6�p�1aP-�)�Rۻ��\��9��«�4K���]��·<-�/\wك4u{
�I	�Pfc����P0!�C��K�чJ�ɖ\&d�����S�j��*I_�L�w��A���p�T3�g��PU�U!�F:�����1� �"$�͵%Bh�ﲮ�B�y C�����'�� ������qP�Z���@��C��X��H�����?+�4^Z�oݠM 5�Z�/����gF�俹�yxeV�đ�T��w?�")�G�C\�~�j��+q{�r/�Vm�J�ؙ��?AA/4�n[�V���6{0i���S6[�����8M�7Z_N�D���^�	����q:\7��¡Z�M�L�+�� @[2�ABql4���i���b<C#K!����ۯ}[К��G���˽��q*Z�10`(hG�r���-$7:�l��QO��'P PThk�ȕ}8�H�v�x��EFjZ���8%^X�D�C�i�����
<�}���+`b�i�?��Z�'�@�]l����v��B^�<��gU/
�"���~��ڰ	4��`�*�n��溍;�X|h���W�.`�<pn�݊�5��[2%*�H�m�F�L�^�:�q��|��?�b�<���妲H*�����ҵS�\��Q�,�y���b������r�M���٤����}��{F%�9�$_�������_g�Y���^Ϛ`�x�t�<-v�@,��0�	�t@R��H��o��N�w�)[���.�����ܢ����=E���*آf#�%=��&2��u+�%�����f��یc��@�2M�F#ZRҴo� �⪽G���O6�<�n�Յ����wБR5���Q���'�{���!��?/p���$n��1�����H�e{�D,zDPr��m3�rbcgVa�B1_X\��ߑ��R̝�A��R�[�Vj��o�V�`&�L�]�;�7�[8ª�O.~���w��a��E��NeY�x�Z9}��$��l��X��7����[3��D%h"�"��k�׵: ������@Nx��g��Z��ロ�b�q���e5jyx�?��V�-����8�h�7�6�g}�b�w���Pl���Om����7���obI��Gd(%��v{����%@
�`vo�����Y�g��,����]>�g�D�K��<-j|��4�k7߱�.]��j���m��~y ���cK�7z�r���@/�dl��v�9�^��O��/�� Ğ�Ǎ�r���R��ah�N
�䌛y��	N'1�$I� �a(H���?����;HV���#��9peN��`����rO!�KFt��n�ڱc��ܢ�y�e���ڄ���h�Lhޔ��O�+�+>�]T+C��F�Ę^�^�ܽӧ��p'�;�T�����w)����X�uCrm|֋%7�M�oD����f�'��1\���[n�w�P_~p�4Ap�T�
�ha���3Qy��7����غD��6ޘ 螿;��U�����ކ�N�j�($^���y�)f���e���{�ŲIHFo�1:KI��V�Dp
ń��-c��&�J���B���1��u%bŐ��Ϳ��� 哺G���[8��Pw���&�
#����Θ~�Rh(�o�/W�U,���Up��<�8��74��K���@���lXgE��%|�y�ϻn/��b�Vu�V��bj�(�:͉a��f��I"���!j�a��	/H]?�gu��p�[��e�(��KJM�j�jެл��?���)u��E�����"����R㗟��h���]��� J���T�g����@����z��ȣe�z���B)~L�6�%a0v�$6Qz���H�N߬�Ѕ�4�m���F���0U /�m����2l*H4�#,���	�̜L@M����TP1tb��5��j�cǥ�����j�����К����DrA��̃��5*cLҴ�ֲ�^�)��VA�.'�<E��%k8�����#o��6W�C���4���t��.�tr^D��
~M`�(Q���.D ���9�M��,dm��k��O�G���P�5�.���׏��Bg����,[�n3�hY�t�u����s�T{�;#��uQN��B�PG�ҟL©ĥ�Yfa�:IQ�{m'\ ����v5���B������%!v���vX��;�z!C����^1�$]�[���̙���I���U9ZM�NʑMw��5������xw*���^���r�"����]sֲ� @�9��ђ�I��ꭅDԎq�9�ٶ<z�H���q8y�����E�[�0BumN�y�UI"�e���F_R���~���c�M�1�C�r��0����b��:���6J�-����V�`��?�%?9�hu�@�B4��E$�}�6Z%�Jr�O�ᲊ��(*6�t	�nL���U�P[��o����#��&�+��L�V=�˘z�#{�:���"?��<o�#��1��j1l�8��C��?
:I���\���,g�`@lI'�q�m�@�É��ٴ���Lt�o[�=�-.��!����E!�aqT8y]h� �N�W�Y���d���o�z��
H���['�P.�^���*w��"%��дV\M�
ڮo�A��܈�#�����B4 P�%��tT���0i���M�F�c�MC�c�����4�_��V`p,D���"�9e����K�>�赃i����GP�9[&�+D���Uph yěw��l�|6�1��j�	`ƞ������IK'MH�mt;�/ێ�F��"
I�o�μ�W���=/�EI��S6�U�js��<���]v��L�Z^��sU(i�<*=��Ӧ
�R�SM�<嬕��u���x�*C�i>�����h ��K0/�tK;�ᛤ��o��r�p"x���T4jLP]��T��8i@��ܸk��������DQ���Q/щ�T� wV��?C��i����߯M,\e�VeJ������c�]�ܿZm=;��.�6��@m"yB%f��v�Z�c˨;�<��>^B�gV!�lc��ү�$-d�~�<�!(��ud�8��k�Q��,�И@S����;��L�T������ �J�	���M��<[M|MW���|wK˓����ő���0����G����U"R� ��X	wbv'Qqo��S�C�@�x^��X>���s��<���I�c�� ��4
��V3�?~�+ IB5+ny;��&�b	�t�1�c3Jp���d�mo���Jl_��,e:��XcvV_�����5b�-��̛���Au��ӿ=n!�r\���샤��]*Ku^��J9�{��V��3k����e6FC��½$sw�l]�&N�~����6��STz�K�j�S�ј�_�Q����9*(m���V� Z-�=K�3�w��`�p�ʏd��$��Sh���Na6��ȱ��IQ���}!x��T�8�Iw=$�"ܜ"9P�<�~J����9����>����\��  N%�,���[�.4�
+s_�Bp"�{(_��W��.�ܬb�7d�7>%Y)��3�A�1u���Sj���<�y[򝳓]��#��j5���a�����@,�o��8�V��T
�@�v��p��e�t��ZS��]W|�[M�ˏ�|(Ow�ܣ��l� V�G�QĘ��u�1�i�R?��^Q�KQ���K��&��y �2ɂT�[�d$�R�����IL�!(��D� ��kȴ'��/�O�ܤ��7d:���?�N������X�O_2�"p��68\��|K��Y�1 yԖ ��k��u�e,�u�KEl ���k�8܃]��O�N+��
�(��-R�S��&2��	� (�g"�����'/�zy�x��*�p�q	S��H�L�L�5I)%*�$��@%I��qY$���v��fJ�U'n`-�sIt����'r�|�z8}W֋�n�2{m�5�n��:e<�MV��m�卓�O1<O��S���W#L�B��E�ួJn��'�� M@�pQ'R
	�3�W5�� W�2��r�6zz��X%\Yt�ę���ğ�g�s��	g#�ch�e�6�jNz���WB<d��8�zO܉�X���!���2��|�r��b�{���^e�O4��$2}sa�|�8��:v}~�APM]θӢ !K[E�8rљ��|@��b�@�C�E���j�h�t�Iam�H��,�b7X���[0�e����J�����#[]/7��L����-
\�`����+���>tŽX�t�=�,�\k.�t�gT4����s�#�өF�I���놳����=�×�6dr����p]����s�/�g�!Sqx�o�G����!��M/�SoXd�f����D'x~`�y�)fc�����9��7�����3�G�\%/]��
�^O����=�u�&?,h�R�Ep��/-Ez� ��tߍ#&=!���O1i���)D��x���lȉ��lzr�F��<j����*�'������v�˹�t��9�~E���I��v�(d�ھ��/� ʕ����y��;�����z��S�{:D�7.[ ���p~[k�`	�$��ڷ��CkXb-:��? .w����V^�ءO��ER�����F�\�|C"�bn��6ۗ�v���J��!��  �YASEr]�/脆�?W�ݕE/����m}�͊(.���N���֠v�)��c��³�f�|�^�V�dE_nF9��?������܌G�h�����2�Y3އ�?WJr�ٗ�t���63[��l��zL��q���p��8�n;����HL�Sq��E���G���jz&�'JA��a��^�A�/9�3�k�f�ڲ	���QEsbUF�Bi����Y�v�,9�69� ���լ�>���77�1j0���s�h�=���۽_��n�}�P�kq����+�6��@���x�XZaRG�(���l\�]+tf���I~y进`ȜYĚ�;��p�n�f��s��;����-�i d袥Rb�V��Q�sU���4�-2�������4�#�/�Ŋ<I)�;��Q{�����<3�����{s��͜��m[��*57!�s1�U�B��=%�Y�p뻝��"�FM޺��T�~�7m��g)��G��~��*��r4_1�
��򼓑fV��~����%)��|���#�k�����/��1����bK&u���8Ն�+�¤���R�n����aRW�x��*4toQ[Z�����M:* �, M�lr�8����������ˍ� A&�i�{X��DGT�+�4�c�o�	��g�Hq�k�|�6���_���j��MY�$��R�{v�7�>��=�2,8�M\=l���N��o�!ڠK/P�V�<���lKq�i�6�5g��V�����g|��O.���5_n<ko���)"W!Y�z`y�~~*jc5[:��<r�U���i|c��v_y�чE�rO�~_��І�j�C��D�����v��fUp��Ɉ��.u�B�H�V���}P�
U:����1�)됼=è
��s�~P����)Ш�*��}l��N(kQ4:�2X���֘�I-�ϊv��F�.�Q5E���_9��$��`���מ���<2����)�2 v��J�?�1��1x|���O���C��0�P�3z2�]�c��y��˶㍆p�%�i�� O�1��z�:��|T�ƞ*�'�=��L�h?���q������>-L��/bʺl�yh`��Gg�Rk����֌"B3�A�s[�N�!s�i��͆Cr�F���^e�),��_g��\�XwjUT�6@W�bD�gM�m%/fŏ�q���B�6�-eSzy\	�(M�٨�C��0�y���K3�I骔$�":�C-���Qk��T~"��_��h�`��pHj�g�a��K��#�c�F����[�y����'�\� H�db��<���G���6��iz��ڙ�r䛋1`��ו�u�^�7.���I̻�BR0��lW���'i���n�be�:HsCK4�`WQw'���k��Y����}�+�_��J�dE1�Hh7��9'.�T,�\��14�����64��_�-G�-H�A������2��nF"1)�5 ��nI�ue-��8S��D�5�����',��?�kx%Jy	K��>�B�}�4?^�Q*���l_��w�~<ѼVL�n�_k�_W��$��do�S�?�-��5{Ն�\+�����78�B�T��F���RC	y
A��N&O�����a��V�ID9�>�wk;��ȵL������B+����F�B��ǥ� _�a�,���T0ST�a�I��G �3|B#��BV�<Oc�\byar��
���+^,e���Ǿ{��$"��,+?f�a�돝�R��:��"z6n��T�i���J�� �>�5����b��2��[��"��jʦ�@�8�MY��֥hy�@iaۭ8a��u���~�/�]0)�^��Gv�ö#8s���B4@sV���X
������-;u��Eg��wٞ�ƕ�'J,�.�mr4;~������qA�c�TqR�ߥ��� �����!�#�y�6�Є�-�,��KZ��*�m������3PxJJ������pa(z@+~��X�<�Z�N{�y�p�\����0VQd��C�~xuqmڧ`Zf�����������H�t��/*|W����zʯɃ�(�4+8IXkƿ<ܿG�Z��8$��O�kOY��a;�T3��_���T����)��Ȫ�+	K+5	��U�hb��� �� ��M{��q�6��L4 !�f��0!e��ҁ4�mJ��򉛞�7�� v��8F���Ҋ[µEY�6�o�
���J�[�7�
���a��K^-��5�n�Q�)E�ra���JR��/��y|��s<�+�ϩ��!l�s���i����Rf,2��{͚w_Ȫ�����Y�~s�&��%
x�`��'�@�h]�&�"�
E|�=��'����d(t�Y �[�Ҟr#d�&Q��?�;D�b�5B�y�F���T+vR"�BA���O���W~T���R�3�D������r�`;�?��tg�}č	���ژ�N^�F\��G�6��J�ʮ����3����fcT/|pb�χ����N�S�0Ӽ�$P�b��[-g�\J�c\���H5���pWI�6���1�,�;7�L��u��>�޲�hv���<����Sꕩ<
x��q�7����&D������ ��S��R�xcY��I���	#u.�KPG�z�b�#�O��@^������8:�l�СBҴ;��,��y*��E�2�)s?��{��ؕ,�T���<�v����!@׾��Ќ��'��TÑ��=oox�Y�.Ƕ+p��&>��{�t���˟!��
aY�<o,2/�.u�Ƃ)�w<O�G]�Mӣ�����<�*=.����u'�� ů�h?h�Vehķ�3sJ�['�
��=����+�z���:�|"��yS���2���t�-MW)8���vv��э�B���Z�Q�wxe2�R�$������6��Q4�%����WhAZ��zd�����r���
�`�H�׉:,r�b�9�u��klK+��Ȩ��ϩ�uհ?��s��.a`��@=��'����#[9�ѹ�6zX\C4�rd�دkL��)��=:ӣKZ.�]���68ɣڊ��=fJy ��p|�ńS�(;m`�_�`�W�2^�rَ-��j�K/�Tk�u�	�d�7m�� ~�{�1^3'�X��>�{�ĭi�Y-n~;�����*z��U1#P�R�z93���&�pW�o�D/#�K{m)�x��h�n
��d�M*�(�ʱ�n���>����srҴ�B�� �R��S6=�Eg㿩��Dez�i!�b��	xa:� Yt�h� ?|X�)����(�wij��R�1:=�	K���]�v4~��Z�gJm�#e�]46� A����u�8Ut�Ę)pףh	��G�����ټ���
@��+�W�����}�L ������i�)�a�=�$�t�%��6�	:�'����Y�Ei�#���m>�^�� �0�jq�e�����{U����#XsNI�*ҳCS��N�WC`ԙ
��w�4Ό��䯷P�?�y'6,F_�Ѩ�* �z�抂�u�w������K�� }��1)T�;�_�H��AR�j�w�x�4�f^ ��� �l�;3?}4�G/�y�$N��g����[�ڑ�o�����c�d5=�p1��4�@�x��M��EC=~�9Mv;���@�X.P�C�c5�1=y4� K�t���_��KW�_�j��M���M�%���oNХ����o�a�߲\.r֗�t����>��e�:Ʃ�n�^rg3մ���*	Mi5cc�%�3epE�S�A�&�@��{�t���-���=?vK&߯?��f_Y�ťhE9�a��ۀx�`P�1����S=�f\`�v3�J�k�05HB}@�2��#�)tI��"�\���c�"M)SHs��4�ϻ�l�J,�H*��5�J��6�����D�I��Y#��n~:��`[ʥ.�/I6D��m���@��V��1<��9 ��ES� 4�!Ť��Zr�J�q���Q�"=����706��`WBƱ?Z�;)\�����|F/�w�N�t(Sō��M}�Y��V(��<��o8�.(��,��?F�'������@ �yڍ�'~�C�@YA������������C�s���0�}9`翣�ֲ��O��n��J��K�]�dR>j��ITe���DލF�\���#UPdGco��5T۪q9��k/���L�o�Ao��XV~Y���oe2��}z�+�_u~Ow\,��m��v
�_���6�F�t
 ��y��j�͙��?��Kx̀e&����Q� �U�P��1��䵍z�HK���7U�X����
D�(�j /#����V1��Cˤю����v�<$4G��:��J8�A�"�1�i kli�����Z�A��d����	��t�Ko��@���;+�hL�%7m�-b�͵�w��JSjQ&���NC\0ry	�����ڱ-���v>�w�_��1��J���N�-�}�*�ł\����MX�0��+'�z1|����7�8LUgaL~F�)��F��Ǡ�k��QoO�0�3����r�oV+4�v˝�
XE?��>�d��L��L���̉w@Gh�r�]ô@��n�3A�#v��h��˒���a��3�֢I��_�m#m������� �#d`=��p!�������/p�5԰'`���f��B��ܢ-�N��S�B�!Ur�6�C)�3���%�p޳�A��ƪ������k�z&q�B�X�n�:�L�� �,�� �u���5
x�or	.#&Z;'s���i
��s+U��(9�6��Z����Fj��M��qS�������df]|4HJ]�.�>ag_Ȋ�`{���g`fK`&~�[�`z���r �}�~с���΍ۭ�m�
�*��ۙ�8�a�o�O�֦��t�m�DXG���f�m�i��V�͋���j���@L�wU�G��jxQn�T�DUp@͘�uY7�^��3�h~�s t.�Ӵ�C�mzq��Q����s�9��gs.���@?@����[V��݄'X�G�6��?�����)b�y�ɨ��R�$���.���\i�ֆ���4= 5��� ^d��R�M�p�,����2���F)��墂�tq�Z� ��s	{�ܣ�.^?;+�ԍ��(��#A��[�˧��A-x�������T��{)ެ��r�jW��z��qc<�_�/?�-f�/i�1��|�p�k!u.��`��%�tq�<	~���� "��"�Ab�iB햯��I6�����H�m�ᄠf���4� ��'
�`,�8@>���Īgu�aX�G
�*}<��`ʺ�y*���>o��¡��4*+~�t��F�����#��U���c�-TN�>��J$��\�]�M5��Х�xi��|?
�`n�Wb�ڎ9�p��{��"gk��4�Ʃ��`�4��xۦ^���R}W\�p{��R�ŸG#�mvy�n!�eEM档(=�ħo�ś �|5�ֽ| Wvh�kFK���/C�έ�����4-�v
I��k:Cuq����G���=S~0 @M�0i
T�~�af4��"x)Qs�f��'fi}����#3MK��׶r��>�V�l}aS<�n�wڇ��s]��'L#��vBS�6c�7SA<�Ţ����`iJ���W���x1�j8����O遵�֨����%�����v����_
J���.�Խ�;F�w��ݬ����\���2aU�[��.�|}�K��n��l>�5w춄�.�c'�̥���.���������y7��k�ې���V+J `��3;;Yym�ތ}�j������'m�=X����˒*+Ѧ	1P ̸a�H	����@�5 a����o�*��z��ں���0��.J��ի��U5i�!��6W׹.�� �R�8%��*V��^m	X�O�f5מM�1�� ���[p�7��6�b����+�_�Rw��W)"企��(�	����n~��$�$X����pPL)o��a�G�փ����$<���:��Z��ds�d�l����K,��䮃>��vHv4��Z��hTp)f��F~t*�(g<��>H��>����������_֣"���G�����C�HV�A�,**q��X�閏8���Xi����4�{��V��d��7���}/�E����km`�v��7��BB������%|�
=�G�9�6��f��FYt������2#$hT�в���J���Φ�ˬ���Y�����$�H����m�7X �0�Lj���p���AeR�j�C���J�Mb��t0�je�҂�����߯m6�C��,3D�a���s�E�0}#�!ϧ�!�C~��(��0��+�%�C�]E��\=ĺ����ԺO��(��۹��. �z�`�"l�����`�W`i�P�/J�~���� ��k�#$Ħ(y�pNe��N����G\�<�(���jC�Z=�w���iWB�������(�¾N��<���ħC��V7VA�_���;J/|>K'�ȁ/�lcHҫp,���ikքh��?�dL�e՛�F"���w�Өǡ9��
�~�1�:��?5��ze�|�S����G�RG=��������E�	�]�ܜ�&AB��-H������)��@?�Y��l��T��Sf��<˙|��*��)NoOx<�v��æ,�vE䷇"
�\����,G���ufK��N@xÑe�����" @�<�Tű�趢N�3�PC�L�H�$�&hlR] ����V��Iڔ�?C��̝�{�mw��ܯ	�g��Lz#�ͥH%����Gΐj�81��{�&���͙+�f�{�-z�Q�Jw�.�h�c���� �H���B��r�j����8�fc�+����t���_.���)�RpB��{����������W��*2bfT�����R\K����'H���%�"�!�8@��HAx��y2L��H׋t<�>X�6"����xOD���@��	: ���N�47�.	޶]��0%��ZL�P����w`':u�^翱����&0�6kX ׈�_�{��0LB\��%c���p��3q�D�1"1�W#9���������;@����Six '%dKG�W��N�I-y5�k4V}8�.��nY`����8�Y3Ԫ&��.�a�)/�HO)�w�v�G�����n��c���F\81�*�-S��N�`n�s�o3���&�$�'f�@�5����&����ܮ"7Z-·T�O� S�����Z����M�MK�F�Ņ2�-�Ak���q�]x��]�A%��39��>
c8�:��q�6N�s`��tM^���h�NX�U�(�J��] U���5���W��U��.g�$qz�q�?j�� _�#i7���{�K�<�+�&8{�XX(�RǞ֖*�O��rUS��e�cz ���U+w��f3A��K����1
�5���G J�~r�WI��Թ���R ~횑� 2�����K�ze���E͔.k����隴���Ӿ��w~	����N�O!���~mΧ�X�#:���(S��e�׾�W����a/����SJkd��w6�5k���M���	r���m�q��W4����y����=����!�	��~���P~�0�^�s�ɉ��C�`9��I�L�y�F8r�0�����8��BQ.]�������Ǡ%,04e�z��ǫl��s�J��Jk���1��R33PF�Ǵ�5�����l��Ee�Ҟ�t��L�Ѓ��Hk���(� �攫'�qFT�O$hj{r���T�d1ty��SK8�`*Ф�k�_�
�M̻���ȓ����zT���>"9l�����c,�U܈�,�θ~v����=L<��d%�rx��B]��@⼹i�0Bs]�;��!���-F���=����p3%`P,�mw��u#���RI���eГ̨�NP�������+�~�Z�㈚"�d��
[�L�X���Q9`��?��Ut=2��1 �S`�׋��w쵀%`AH��H��ř�I��w�"m*�%�|�H7ަ7;j�	LD,��w���iB�qO�YUF?q�v��o�nH�nBk��wr�	�B #�5h�2�C�7{���u(����S�(����x~xk�]���
��N޸����M?�L>s�����H�O*7s*����+��_�����z�}5��IC��(�% ]�.m������"�8�$��,E����a�q�l���s�z�CIj��#-��&�?�-J%��?���1���:q�8��@U2cT?s�R�+�x(:wt7A��j����zѢ�_H��;�Y�%(CgtK{,�T0�������,���F�)��}�	�z���� ����C�N�m�	�C��M'qPc�y_�3[̡����W��`E$A2�w$h≆IҐ�0�=@�b
�U�6�WaIy�6G* _u�wJ��� β4"Rc�r�dL�|b��7d"j>��k�\PM�Y�D����4�!�0�Q.9�g�7m����H���ϰz{��-�'��&���8�GU� N2�9s���q��T\�{s�ѓ)��yAT���߸����e˲]��(�o'Y4��3�yu�G�QxU�;�v{�,JQ�J�l����;j6���`�2Iڐ��D0���h?��Ni[R����<�r��ےn�0�tb k���|��CEaΛp,&ޅn|�P�ӕ���1W�UrͰ\��
)2F`�4���7z��d�@*�X����ҝ�CGĆw��8�'�<_�§�uo�[ �������c��Q�!��\�1����栩}�a�V�*��b=�O%���8�~UG�(�Ҹ��Vت�Ԍ��-P�<�/��r*wű�4��pՆT�f���x�(��6��/��%�I`>�i&�P5�����[�^�\��*���^!�;�:�ʚ��!:���`p3��qL�^2L�=2���4�F{~�%��b��C���=���s�a	8�~t�Ĕ���U�qʧ T�4�uz�����h�`�J��+�\ˉO��5������Jo��,����ҍ��RV�#� ��@����JyW���I��Բ�G#F��X�~��Z�K��9�#�
YI�zfZ�����+	�5�����n`��k͈�.�$Վyb���RB4_�^-}���6y�ET�˯���j�N][�Z���~�hN��xw��������� ϛ�h� �r[m�k.:��?��3^����c�NdÃ+7�����G[���$/�p$+x�I����us����rĹQ%��9����_��� �f^���'%U[�>�HDsgA-@�����U�F+#O��4=�V+�q0e��'�'e:���J����ձg��h�)Z�����w,�А9D����I����oc��9��ssU�Q#Y(t������/Y�Yj2��H�&�������t�u����`����e����90�7	����VV���*\js?&��+L��h5�$s���T
��LgF�P��Z�o���~��������5�޲�3�,��x&Qď�Ru�=6e56��ͫ71�܏��P�T���rva=�[����w�=��m9&i�qZ�ɾ�~ r䓆�,yE��BQ$�������q�W>8�ik�U�[����`�iE$�Ae�v��j�O�ՄE�I�)��cO���ES���aʫ	��t6��H�W��v
�FGś*�d�C'��;�ӟ5|�x�I����	^#C/jޗ"��-#��,F28�.�:V��G�&�i�v��`�A\T_Vw��#DH���H�]���7�0��w��N�๙K��r@�EmK��as��e'�ܙ䈔�2��l kOg\G6��U0����/h�J�..U��"m�+�vB��F@|����M��Fi	��á%��ᄞ��zTZ�_��f!��}﬍�ns*>���q/�xW�i�K"۸]�B-���3%�r�ˠ��)�L�ɿ�Af���Dy[F�M��?�[Ɏ|�Ӟ�a�
}��+5{�3�I�������!���Ӊ�%S�>_�-�4@pE3��B�U!�Aҗ����;��f����zZ-2g��#Q�)��saU��r�f�(d}�����[|a��@���Ѝԩ	�"&��޸��y�E�!yVlK@�WpƕvI�z:��V2ֶ��|Ɓ,S5(�`��r��"��I���~������%���������!*S�dc�? �סS������&���	���fp!��~��Ņ�d(���k�SAߏ�C�O�CX�/;v!�_Lh�$�%>`$��2�r" ��	=�����0�~ݺ��l�$
$5���2.�U�П��ీ�+A�t���д;7J!���
���'R<[{�:�5��<���1�+��Jz�M��� B	�z�)�K�?)D�	i�Z0��dF��b=BK�T���$�w&�]f���D�!`DO0Wi*!�b_�.|@��E��p���H`@ψ�9�>6N8f�aG�I�Ɂ�����ù���8�����hU��e��R0�ߨ���#��7�P��$�0�܂6�����S�9���W|и14�LU�*�\�
����Y@f��U�� m�̦��ӑ��z��Ƽ����*��;O�Q�S^$�	$�u+ɬ{s��w}6z[�_�D���,U�zi�
�.R�0y�ӏk��M�?X2fՌ��Z���;6'��T�L[�+JmHV�`