/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief
 *           Implement a multi-boot system to boot from different applications in APROM.
 *           A LDROM code and 4 APROM code are implemented in this sample code.
 * @copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define DEF_APPLICATION_START_ADDR	FMC_APROM_BASE
#define DEF_APPLICATION_SIZE				(0x80000-FMC_FLASH_PAGE_SIZE*2-FMC_FLASH_PAGE_SIZE*2-FMC_FLASH_PAGE_SIZE*4)	// 480k bytes
#define DEF_PARAMETER_1_START_ADDR	(DEF_APPLICATION_START_ADDR+DEF_APPLICATION_SIZE)
#define DEF_PARAMETER_1_SIZE				(FMC_FLASH_PAGE_SIZE*2)		//8k bytes
#define DEF_PARAMETER_2_START_ADDR	(DEF_PARAMETER_1_START_ADDR+DEF_PARAMETER_1_SIZE)
#define DEF_PARAMETER_2_SIZE				(FMC_FLASH_PAGE_SIZE*2)		//8k bytes

#define AOS_PARTITION_PARAMETER			 		DEF_PARAMETER_2_START_ADDR
#define AOS_PARTITION_APPLICATION    		DEF_APPLICATION_START_ADDR
#define AOS_PARTITION_APPLICATION_SIZE	DEF_APPLICATION_SIZE

#define DEF_CLONER_START_ADDR				(DEF_APPLICATION_SIZE+DEF_PARAMETER_1_SIZE+DEF_PARAMETER_2_SIZE)
#define DEF_CLONER_START_SIZE				(0x80000 - DEF_CLONER_START_ADDR)

#define AOS_OTA_FLAG_MAGIC					0xA5A5A5A5

typedef enum {
	UPG_NORMAL = 0,
	UPG_SWITCH_APP,
	UPG_CHECK_APP,
	UPG_BOOT_FAILED,
} Upgrade_flag_enum;

typedef struct
{
	uint32_t ota_len;
	uint32_t ota_crc;
} ota_reboot_info_t;

#pragma pack(1)
typedef struct {
	uint32_t magic;
	uint32_t upgrade_flag;
	ota_reboot_info_t m_ota_info;	
} Ota_flag;
#pragma pack()

Ota_flag sOtaFlag = {0} ;

static void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);

    /* Enable HXT clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Wait for HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Switch HCLK clock source to HXT */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HXT,CLK_CLKDIV0_HCLK(1));

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(FREQ_192MHZ);

    /* Set both PCLK0 and PCLK1 as HCLK/2 */
    CLK->PCLKDIV = CLK_PCLKDIV_PCLK0DIV2 | CLK_PCLKDIV_PCLK1DIV2;

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART module clock source as HXT and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HXT, CLK_CLKDIV0_UART0(1));

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk);
    SYS->GPB_MFPH |= (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    /* Lock protected registers */
    SYS_LockReg();
}

static void UART0_Init(void)
{
    /* Configure UART0 and set UART0 baud rate */
    UART0->LINE = 0x3;
    UART0->BAUD = 0x30000066;
}

/**
 * @brief       Routine to send a char
 * @param[in]   ch Character to send to debug port.
 * @returns     Send value from UART debug port
 * @details     Send a target char to UART debug port .
 */
static void SendChar_ToUART(int ch)
{
    while (UART0->FIFOSTS & UART_FIFOSTS_TXFULL_Msk);

    UART0->DAT = ch;
    if(ch == '\n')
    {
        while (UART0->FIFOSTS & UART_FIFOSTS_TXFULL_Msk);
        UART0->DAT = '\r';
    }
}

static void PutString(char *str)
{
    while (*str != '\0')
    {
        SendChar_ToUART(*str++);
    }
}


int FLASH_read_at(uint32_t address, uint32_t data, uint32_t len_bytes)
{
    int i;
    uint32_t *dst = (uint32_t *)data;

    for (i = 0; i < len_bytes; i += 4)
        *(dst + i / 4) = FMC_Read(address + i);

    return i;
}

static void Put4Bytes(int data)
{
		int i;
		char ch;
		for ( i=7;i>=0;i-- )
		{
			ch = ((data >> (i*4)) & 0x0F);
			if ( (int)ch > 9 )
				ch = ch + 'a' - 10;
			else
				ch = ch + '0';
			SendChar_ToUART(ch);
		}
}

void aos_ota_flag_dump ( Ota_flag* psOtaFlag )
{
		PutString("Read OTA flag at 0x");
			Put4Bytes(AOS_PARTITION_PARAMETER);	
		PutString("\n");
	
		PutString("magic: 0x");
			Put4Bytes(psOtaFlag->magic);	
		PutString("\n");
	
		PutString("upgrade_flag: 0x");
			Put4Bytes(psOtaFlag->upgrade_flag);	
		PutString("\n");
	
		PutString("ota_crc: 0x");
			Put4Bytes(psOtaFlag->m_ota_info.ota_crc);	
		PutString("\n");

		PutString("ota_len: 0x");
			Put4Bytes(psOtaFlag->m_ota_info.ota_len);	
		PutString("\n");
}


#if DEF_CLONER

#define USE_4_BYTES_MODE            0            /* W25Q20 does not support 4-bytes address mode. */
#define ROUND_DOWN(a,b) 					(((a) / (b)) * (b))
#define ROUND_SIZE(a,b) 					(((a+b-1) / (b)) * (b))

#define MIN(a,b)        					(((a) < (b)) ? (a) : (b))
#define FLASH_PAGE_SIZE 					FMC_FLASH_PAGE_SIZE

#define DEF_SPIM_FLASH_START_ADDR				(0x0)

uint8_t page_cache[FLASH_PAGE_SIZE]={0};


void aos_erase_ota_flag(int crc32_length )
{
		uint32_t  u32ChkSum_AP;    /* temporary data */	
		u32ChkSum_AP = FMC_GetChkSum ( AOS_PARTITION_APPLICATION, crc32_length );
		if (u32ChkSum_AP == 0xFFFFFFFF)
			PutString("Failed on checksum!\n");
		else
		{
			printf("CRC32: %x\n", u32ChkSum_AP);       /* print out APROM CRC32 check sum value */

			printf("Erase OTA flag!\n");			 
			FMC->ISPCMD = FMC_ISPCMD_PAGE_ERASE; 			/* ISP page erase command                   */
			FMC->ISPADDR = AOS_PARTITION_PARAMETER;    /* page address                             */
			FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;  			/* trigger ISP page erase and no wait       */
		} //else
}


void aos_clone_fw(void)
{
		Ota_flag* psOtaFlag = &sOtaFlag;
		uint32_t 	ota_length=0;
	
		FLASH_read_at( (uint32_t)AOS_PARTITION_PARAMETER, (uint32_t)psOtaFlag, sizeof(Ota_flag) );

		aos_ota_flag_dump(psOtaFlag);

		ota_length 		= (psOtaFlag->m_ota_info.ota_len > AOS_PARTITION_APPLICATION_SIZE)? 0 : ROUND_SIZE( psOtaFlag->m_ota_info.ota_len, FLASH_PAGE_SIZE ) ;

		if ( (psOtaFlag->magic != AOS_OTA_FLAG_MAGIC) || (ota_length==0)  )
		{
			printf("Wrong OTA flag!!\r\n");		
			return;
		
		}	else {
	
			int i, j;
			uint32_t *pdata;
			
			int page_num=ota_length/FLASH_PAGE_SIZE;
			
			printf("page_num: %d \n", page_num);
			
			FMC_ENABLE_AP_UPDATE();
			
			printf("Erasing APROM !!\r\n");
			for ( i=0; i<page_num; i++ )
			{
				printf("Erasing page_num=%x\r\n", DEF_APPLICATION_START_ADDR + i*FMC_FLASH_PAGE_SIZE );					
				FMC_Erase ( DEF_APPLICATION_START_ADDR + i*FMC_FLASH_PAGE_SIZE );
			}
			
			printf("Clone APROM !!\r\n");
			for ( i=DEF_SPIM_FLASH_START_ADDR; i<(DEF_SPIM_FLASH_START_ADDR+ota_length); i+=FLASH_PAGE_SIZE )
			{
				// Read 4k bytes
				SPIM_IO_Read (i, USE_4_BYTES_MODE, FLASH_PAGE_SIZE, page_cache, OPCODE_FAST_READ, 1, 1, 1, 1);
				
				// Write 4k Bytes
				for ( j=0; j<FLASH_PAGE_SIZE; j+=8 )	
				{
					pdata = (uint32_t*)&page_cache[j];
					FMC_Write8Bytes((uint32_t)(j+i), pdata[0], pdata[1]);
				}
			}
			printf("Clone done !!\r\n");
			
			aos_erase_ota_flag ( ota_length );

			FMC_DISABLE_AP_UPDATE();
		}
}

static void spim_init ( void )
{
	/* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk);
    SYS->GPB_MFPH |= (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    /* Init SPIM multi-function pins, MOSI(PC.0), MISO(PC.1), CLK(PC.2), SS(PC.3), D3(PC.4), and D2(PC.5) */
    SYS->GPC_MFPL &= ~(SYS_GPC_MFPL_PC0MFP_Msk | SYS_GPC_MFPL_PC1MFP_Msk | SYS_GPC_MFPL_PC2MFP_Msk |
                       SYS_GPC_MFPL_PC3MFP_Msk | SYS_GPC_MFPL_PC4MFP_Msk | SYS_GPC_MFPL_PC5MFP_Msk);
    SYS->GPC_MFPL |= SYS_GPC_MFPL_PC0MFP_SPIM_MOSI | SYS_GPC_MFPL_PC1MFP_SPIM_MISO |
                     SYS_GPC_MFPL_PC2MFP_SPIM_CLK | SYS_GPC_MFPL_PC3MFP_SPIM_SS |
                     SYS_GPC_MFPL_PC4MFP_SPIM_D3 | SYS_GPC_MFPL_PC5MFP_SPIM_D2;
    PC->SMTEN |= GPIO_SMTEN_SMTEN2_Msk;

    /* Set SPIM I/O pins as high slew rate up to 80 MHz. */
    PC->SLEWCTL = (PC->SLEWCTL & 0xFFFFF000) |
                  (0x1<<GPIO_SLEWCTL_HSREN0_Pos) | (0x1<<GPIO_SLEWCTL_HSREN1_Pos) |
                  (0x1<<GPIO_SLEWCTL_HSREN2_Pos) | (0x1<<GPIO_SLEWCTL_HSREN3_Pos) |
                  (0x1<<GPIO_SLEWCTL_HSREN4_Pos) | (0x1<<GPIO_SLEWCTL_HSREN5_Pos);
}

void spim_flash_init (void)
{
		uint8_t     idBuf[3];
		spim_init ( );
	
		SYS_UnlockReg();                   /* Unlock register lock protect */

    SPIM_SET_CLOCK_DIVIDER(1);        /* Set SPIM clock as HCLK divided by 2 */

    SPIM_SET_RXCLKDLY_RDDLYSEL(0);    /* Insert 0 delay cycle. Adjust the sampling clock of received data to latch the correct data. */
    SPIM_SET_RXCLKDLY_RDEDGE();       /* Use SPI input clock rising edge to sample received data. */

    SPIM_SET_DCNUM(8);                /* Set 8 dummy cycle. */
    
    if (SPIM_InitFlash(1) != 0)        /* Initialized SPI flash */
    {
        printf("SPIM flash initialize failed!\n");
        goto lexit;
    }
		
    SPIM_ReadJedecId(idBuf, sizeof (idBuf), 1);
    printf("SPIM get JEDEC ID=0x%02X, 0x%02X, 0x%02X\n", idBuf[0], idBuf[1], idBuf[2]);
		
    SPIM_Enable_4Bytes_Mode(USE_4_BYTES_MODE, 1);

lexit:
		return;
}

int32_t main(void)
{	
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART0_Init();             /* Initialize UART0 */

    /* Unlock protected registers */
    SYS_UnlockReg();

	 	printf("Hello, I am AOS Cloner!!\n");

		spim_flash_init();
	
    /* Enable FMC ISP function */
    FMC_Open();

		aos_clone_fw();
	 	printf("Jump to LDROM!!\n");
		
		while((UART0->FIFOSTS & UART_FIFOSTS_TXEMPTY_Msk)==0);
		FMC_SetVectorPageAddr(FMC_LDROM_BASE);
		FMC_SET_LDROM_BOOT();
		
    /* Reset CPU only to reset to new vector page */
    SYS_ResetCPU(); 

    /* Reset System to reset to new vector page. */
    // NVIC_SystemReset();

    while(1);
}

#else
int aos_is_ota_update ( void )
{
		Ota_flag* psOtaFlag = &sOtaFlag;
		int ret = 0;
		int ota_length=0;
	
		FLASH_read_at( (uint32_t)AOS_PARTITION_PARAMETER, (uint32_t)psOtaFlag, sizeof(Ota_flag) );

		aos_ota_flag_dump ( psOtaFlag );
	
		if ( (psOtaFlag->magic != AOS_OTA_FLAG_MAGIC) )
			goto exit_aos_ota_update;
		
		switch ( psOtaFlag->upgrade_flag )
		{
			case UPG_SWITCH_APP:
			
				ota_length 		= (psOtaFlag->m_ota_info.ota_len > AOS_PARTITION_APPLICATION_SIZE)? 0 : (psOtaFlag->m_ota_info.ota_len+4095)/4096*4096 ;
				if ( ota_length != 0 )	
						ret = 1;
				
				break;
				
			default:
				ret = 0;
			break;
			
		}		

exit_aos_ota_update:
		return ret;
}

int32_t main(void)
{
		uint32_t u32WhatAddrToBoot=DEF_APPLICATION_START_ADDR;
	
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART0_Init();             /* Initialize UART0 */

    /* Unlock protected registers */
    SYS_UnlockReg();
	
	 	PutString("Hello, LD!!\n");

    /* Enable FMC ISP function */
    FMC_Open();
	
		if ( aos_is_ota_update() )
		{
			u32WhatAddrToBoot = DEF_CLONER_START_ADDR;
 		  PutString("Jump to Cloner!!\n");		
		} else 
		 	PutString("Jump to AOS!!\n");

		PutString("Will boot from 0x");
			Put4Bytes(u32WhatAddrToBoot);	
		PutString("\n");

		while((UART0->FIFOSTS & UART_FIFOSTS_TXEMPTY_Msk)==0);
		FMC_SetVectorPageAddr(u32WhatAddrToBoot);
		FMC_SET_APROM_BOOT();
		
    /* Reset CPU only to reset to new vector page */
    SYS_ResetCPU();

    /* Reset System to reset to new vector page. */
    // NVIC_SystemReset();

    while(1);
}
#endif

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
