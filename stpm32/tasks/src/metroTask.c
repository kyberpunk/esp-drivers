/**
*   @file      metroTask.c
*   @author    STMicroelectronics
*   @version   V1.0
*   @date      11-March-2016
*   @brief     This source code includes Metrology legal task related functions
*   @note      (C) COPYRIGHT 2013 STMicroelectronics
*
* @attention
* THE PRESENT SOFTWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
*/

/*******************************************************************************
* INCLUDE FILES:
*******************************************************************************/
#include "metroTask.h"
#include "metrology.h"
#include <stdint.h>
#include <string.h>
#include "st_device.h"

/** @addtogroup LEGAL
  * @{
  */

/*******************************************************************************
* CONSTANTS & MACROS:
*******************************************************************************/

#define FACTOR_POWER_ON_ENERGY      (858)   // (3600 * 16000000 / 0x4000000) = 858.3...


       /*+------------------------------------------------------------------------------------+
         |                                        U32                                         |
         |---------------------|-------------------|-------------------|----------------------|
         |     STPM EXT4       |     STPM EXT3     |     STPM EXT2     |     STPM EXT1        |
         |---------------------|-------------------|-------------------|----------------------|
         |    u4   |     u4    |   u4    |   u4    |     u4  |     u4  |      u4   |  u4      |
         |---------|-----------|--------------------------------------------------------------|
         |CH masks | STPM type |CH masks |STPM type|CH masks |STPM type|  CH masks |STPM type |
         |---------|-----------|--------------------------------------------------------------|

        STPM CFG EXTx (u8):
        -----------------
        MSB u4 : Channel  Mask :  Channels affected to STPM
            0 : No Channel affected
            1 : Channel 1 affected
            2 : Channel 2 affected
            4 : Channel 3 affected
            8 : Channel 4 affected

        LSB u4 :  STPM type : 6 to 8
            0 : No STPM
            6 : STPM32
            7 : STPM33
            8 : STPM34

        EX : STPM EXT 1: One STPM34 with Channels 2 and 3 affected on it
        LSB u4 = 8 (STPM34)
        MSB u4 = 6 ( 4:Channel 3 + 2:Channel 2)

        STPM CONFIG : U32 = 0x00000068

        +------------------------------------------------------------------------------------+*/

const nvmLeg_t metroDefault = {
    0x00000036, // config
  {                 // data1[19] STPM (Config for CT)
    0x040000a0,
    0x240000a0,
    0x000004e0,
    0x00000000,
    0x003ff800,
    0x003ff800,
    0x003ff800,
    0x003ff800,
    0x00000fff,
    0x00000fff,
    0x00000fff,
    0x00000fff,
    0X0f270327,//0x03270327,
    0x03270327,
    0x00000000,
    0x00000000,
    0x00000000,
    0x00000000,
    0x00004007,
  },
  {                // powerFact[2]
	32837980,//30154605,      // ch 1
	32837980//30154605      // ch 2
  },
  {                // voltageFact[2]
	116880,//116274,        // ch 1
	116880//116274        // ch 2
  },
  {                // currentFact[2]
	28095,//25934,         // ch 1
	28095//25934         // ch 2
  }
};

/*******************************************************************************
* TYPES:
*******************************************************************************/

/*******************************************************************************
* GLOBAL VARIABLES:
*******************************************************************************/
metroData_t metroData;
METRO_Device_Config_t Tab_METRO_Global_Devices_Config[NB_MAX_DEVICE];

extern METRO_Device_Config_t Tab_METRO_internal_Devices_Config[NB_MAX_DEVICE];

/*******************************************************************************
* LOCAL FUNCTION PROTOTYPES:
*******************************************************************************/
static void METRO_UpdateData(void);

/*******************************************************************************
* LOCAL VARIABLES:
*******************************************************************************/

/*******************************************************************************
*
*                       IMPLEMENTATION: Public functions
*
*******************************************************************************/


/*******************************************************************************
*
*                       IMPLEMENTATION: Private functions
*
*******************************************************************************/

int8_t METRO_ApplyConfig(uint32_t in_stpm_config, uint32_t in_stpm_data)
{
  if ((in_stpm_config & 0x0000000F) != 0)
  {
    /* Check if Config device inside RAM can permit access to EXT chip */
    if ( Tab_METRO_internal_Devices_Config[EXT1].device > 0)
    {
    /* write configuration into STPM */
    	uint8_t result = Metro_Write_Block_to_Device(EXT1, 0, 19, (uint32_t*)in_stpm_data);
    	if (result == -1)
    	{
    		return -1;
    	}
    	/* Read back configuration to show the read block access */
    	result = Metro_Read_Block_From_Device(EXT1, 0, 19, (uint32_t *)&Tab_METRO_internal_Devices_Config[EXT1].metro_stpm_reg);
    	if (result == -1)
    	{
        	return -1;
        }
    	// Compare written data with default configuration
    	if (memcmp((void*)in_stpm_data, (void*)&Tab_METRO_internal_Devices_Config[EXT1].metro_stpm_reg, 19 * sizeof(uint32_t)) != 0)
    	{
    		return -1;
    	}
    }
  }
  return 0;
}

/**
  * @brief  This function implements the Metrology init
  * @param  None
  * @retval None
  */
METRO_error_t METRO_Init()
{
  /* initialization device type and number of channel */
  metrology_platform_log(MET_LOG_INFO, "Metro_Setup: Devices setup");
  Metro_Setup(0, (uint32_t)metroDefault.config);
  
  metrology_platform_log(MET_LOG_INFO, "Metro_power_up_device");
  /*power STPM properly with EN pin to set it in UART or SPI mode*/
  Metro_power_up_device();
  
  metrology_platform_log(MET_LOG_INFO, "Metro_Init...");
  /* initialization steps for STPM device */
  Metro_Init();

#ifdef UART_XFER_STPM3X /* UART MODE */   
  /* Change UART speed for STPM communication between Host and EXT1*/
  metrology_platform_log(MET_LOG_INFO, "Metro_UartSpeed");
  Metro_UartSpeed(USART_SPEED);
#endif
  metrology_platform_log(MET_LOG_INFO, "METRO_ApplyConfig");
  /* Write configuration to STPM device and read back configuration from STPM device */   
  if (METRO_ApplyConfig((uint32_t)metroDefault.config,(uint32_t)metroDefault.data1) == -1)
  {
	  metrology_platform_log(MET_LOG_ERROR, "Apply config failed!");
	  return METRO_ERROR;
  }
  metrology_platform_log(MET_LOG_ERROR, "Apply config success");

  metrology_platform_log(MET_LOG_INFO, "Metro_Set_Hardware_Factors");
/* Initialize the factors for the computation */
  Metro_Set_Hardware_Factors( CHANNEL_1, (uint32_t)metroDefault.powerFact[0], (uint32_t)metroDefault.powerFact[0]/ FACTOR_POWER_ON_ENERGY,(uint32_t)metroDefault.voltageFact[0],(uint32_t)metroDefault.currentFact[0]);
  Metro_Set_Hardware_Factors( CHANNEL_2, (uint32_t)metroDefault.powerFact[1], (uint32_t)metroDefault.powerFact[1]/ FACTOR_POWER_ON_ENERGY,(uint32_t)metroDefault.voltageFact[1],(uint32_t)metroDefault.currentFact[1]);
   
  if(Tab_METRO_internal_Devices_Config[EXT1].device != 0)
  {
    /* Set default latch device type inside Metro struct for Ext chips */
     Metro_Register_Latch_device_Config_type(EXT1, LATCH_SYN_SCS);
  }
  return METRO_OK;
}

METRO_error_t METRO_Get_Data() {
	if (Tab_METRO_internal_Devices_Config[EXT1].device != 0) {
		return (Metro_Get_Data_device(EXT1) == 0) ? METRO_OK : METRO_ERROR;
	}
	return METRO_ERROR;
}


/**
  * @brief  This function implements the Metrology update data
  *         Pickup the data 
  * @param  None
  * @retval None
  */
METRO_error_t METRO_Update_Measures()
{
  METRO_error_t result = METRO_ERROR;
  if(Tab_METRO_internal_Devices_Config[EXT1].device != 0)
  {
	  result = METRO_Get_Data();
	  METRO_UpdateData();
  }
  return result;
}

/**
  * @brief  This function implements the Metrology latch device
  *         Set the HW latch for next update
  * @param  None
  * @retval None
  */
void METRO_Latch_Measures() {
	if(Tab_METRO_internal_Devices_Config[EXT1].device != 0)	{
		Metro_Set_Latch_device_type(EXT1,LATCH_SYN_SCS);
	}
}

/**
  * @brief  This function implements the Metrology latch device
  *         Set the HW latch for next update
  * @param  None
  * @retval None
  */
void METRO_Latch_And_Reset()
{
	if(Tab_METRO_internal_Devices_Config[EXT1].device != 0)	{
		Metro_Set_Latch_and_reset(EXT1);
	}
}

/**
  * @brief  This function updates the Metro measurements values
  * @param  None
  * @retval None
  */
static void METRO_UpdateData(void) {
	//metrology_platform_log(MET_LOG_INFO, "Update measurement data");
	metroData.energyActive = Metro_Read_energy(CHANNEL_1, E_W_ACTIVE);
	metroData.energyReactive = Metro_Read_energy(CHANNEL_1, E_REACTIVE);
	metroData.energyApparent = Metro_Read_energy(CHANNEL_1, E_APPARENT);

	metroData.powerActive = Metro_Read_Power(CHANNEL_1, W_ACTIVE);
	metroData.powerReactive = Metro_Read_Power(CHANNEL_1, REACTIVE);
	metroData.powerApparent = Metro_Read_Power(CHANNEL_1, APPARENT_RMS);

	metroData.nbPhase = Metro_Read_PHI(CHANNEL_1);
	Metro_Read_RMS(CHANNEL_1, &metroData.rmsvoltage, &metroData.rmscurrent, 1);

}

/**
  * @}
  */

/* End Of File */
