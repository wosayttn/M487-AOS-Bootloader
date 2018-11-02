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
#define AOS_OTA_FLAG_MAGIC							0xA5A5A5A5

#define DEF_TEST 0

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

#if DEF_TEST
static uint16_t ReceiveBytes(int32_t cnt)
{
		#define RECEIVE_TIMEROUT  0x100
    if(cnt!=0) cnt*=0x10000;
    while(1)
    {
        if(cnt--==1)
        {
            return RECEIVE_TIMEROUT;
        }
        if((UART0->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk) == 0)
        {
            return (UART0->DAT);
        }
    }
}
#endif

/*
 *  Dual bank background program state
 */
typedef enum 
{
    DB_STATE_START,                              /* Start background dual bank program       */
    DB_STATE_ERASE,                              /* Executing ISP page erase                 */
    DB_STATE_PROGRAM,                            /* Executing ISP write                      */
    DB_STATE_DONE,                               /* All queued ISP operations finished. Idle */
    DB_STATE_FAIL                                /* ISP command failed or verify error       */
} E_DB_ACTION;

static volatile E_DB_ACTION  db_state = DB_STATE_DONE;   /* dual bank background program state       */
static volatile uint32_t  ota_length, crc32_length;             /* dual bank program remaining length       */
static volatile uint32_t  ota_dst_addr;               /* dual bank program current flash address  */
static volatile uint32_t  ota_src_addr;               /* dual bank program current flash address  */

static volatile int g_eraseonly=0;
static volatile int g_jumptoap=1;

void aos_write_firmware(void)
{
		uint32_t data=0xffffffff;
	
    if (db_state == DB_STATE_DONE)               /* Background program is in idle state      */
    {
        return ;
    }

    if (ota_length == 0)                          /* Background program done?                 */
    {
        db_state = DB_STATE_DONE;                /* enter idle state                         */
        return;
    }

    if (FMC->MPSTS & FMC_MPSTS_MPBUSY_Msk)
        return;                                  /* ISP is busy, postpone to next called     */

    /*
     *  Dual-bank background program...
     */
    switch (db_state)
    {
    case DB_STATE_START:
        if (ota_dst_addr & ~FMC_PAGE_ADDR_MASK)
        {
            PutString("address not aligned!\n");
            db_state = DB_STATE_FAIL;
            break;
        }
        if (ota_length & ~FMC_PAGE_ADDR_MASK)
        {
            PutString("length not aligned!\n");
            db_state = DB_STATE_FAIL;
            break;
        }
        db_state = DB_STATE_ERASE;           /* Next state is to erase flash            */
        break;

    case DB_STATE_ERASE:
        //PutString("Erase 0x");
				//	Put4Bytes(ota_dst_addr);
        //PutString("\n");
        FMC->ISPCMD = FMC_ISPCMD_PAGE_ERASE; /* ISP page erase command                   */
        FMC->ISPADDR = ota_dst_addr;              /* page address                             */
        FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;  /* trigger ISP page erase and no wait       */

        db_state = DB_STATE_PROGRAM;         /* Next state is to program flash           */
        break;

    case DB_STATE_PROGRAM:
        //if ((ota_dst_addr & ~FMC_PAGE_ADDR_MASK) == 0)
        //    PutString("Erase done\n");

				if ( !g_eraseonly ) 
				{
				
					//data = FMC_Read(ota_src_addr);
					
					FMC->ISPCMD = FMC_ISPCMD_READ;
					FMC->ISPADDR = ota_src_addr;
					FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
					while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk) { }
					data =  FMC->ISPDAT;
					
					FMC->ISPCMD = FMC_ISPCMD_PROGRAM;    /* ISP word program command                 */
					FMC->ISPADDR = ota_dst_addr;              /* word program address                     */
					FMC->ISPDAT = data;               /* 32-bits data to be programmed            */
					FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;  /* trigger ISP program and no wait          */

				}

        ota_dst_addr += 4;                   /* advance to next word                     */
				ota_src_addr += 4;
        ota_length -= 4;
        if ((ota_dst_addr & ~FMC_PAGE_ADDR_MASK) == 0)
        {
            /* have reached start of next page          */
            db_state = DB_STATE_ERASE;       /* next state, erase page                   */
        }
        break;

    default:
        PutString("Unknown db_state state!\n");
        while (1);
    }
}

void erase_ota_flag(void)
{	
   uint32_t  u32ChkSum_AP, u32ChkSum_OTA;    /* temporary data */	
   /*
     *  Request FMC hardware to run CRC32 calculation on APROM bank 0.
     *  Note that FMC CRC32 checksum calculation area must not cross bank boundary.
   */
	 g_jumptoap = 0;
	
   u32ChkSum_AP = FMC_GetChkSum ( AOS_PARTITION_APPLICATION, crc32_length );
   if (u32ChkSum_AP == 0xFFFFFFFF)
			PutString("Failed on checksum!\n");
	 else {

			PutString("CRC32:");
			Put4Bytes(u32ChkSum_AP);       /* print out APROM CRC32 check sum value */
			PutString("\n");

			if ( (u32ChkSum_OTA=FMC_GetChkSum ( AOS_PARTITION_OTA_TEMP, crc32_length )) == u32ChkSum_AP  )
			{
				 PutString("Erase OTA flag!\n");			 
				 FMC->ISPCMD = FMC_ISPCMD_PAGE_ERASE; 			/* ISP page erase command                   */
				 FMC->ISPADDR = AOS_PARTITION_PARAMETER;    /* page address                             */
				 FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;  			/* trigger ISP page erase and no wait       */
			 
				 g_jumptoap = 1;
			 
					return ;
			} //chksum
		 
		}	//else

		PutString("chksum error!!\n");
}

void aos_upgrade_firmware ( void )
{
  PutString("Upgrading...\n");
	
	#if DEF_TEST
		ota_length 		= AOS_PARTITION_APPLICATION_SIZE;
	#else
		ota_length 		= (sOtaFlag.m_ota_info.ota_len > AOS_PARTITION_APPLICATION_SIZE)? 0 : (sOtaFlag.m_ota_info.ota_len+4095)/4096*4096 ;
	#endif
		crc32_length  = ota_length;
	
		if (ota_length==0)	return;
		
		ota_dst_addr	=	AOS_PARTITION_APPLICATION;					/* dual bank program current flash address  */
		ota_src_addr  = AOS_PARTITION_OTA_TEMP;							/* OTA TEMP */

		db_state = DB_STATE_START;
	
		while ( db_state != DB_STATE_DONE  )
			aos_write_firmware();
    
		erase_ota_flag();
}

void aos_kv_ota_flag_dump ( Ota_flag* psOtaFlag )
{
//#if !DEF_TEST
#if 0	
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
#endif
}

int FLASH_read_at(uint32_t address, uint32_t data, uint32_t len_bytes)
{
    int i;
    uint32_t *dst = (uint32_t *)data;

    for (i = 0; i < len_bytes; i += 4)
        *(dst + i / 4) = FMC_Read(address + i);

    return i;
}

int aos_ota_update ( void )
{
		Ota_flag* psOtaFlag = &sOtaFlag;
		int ret = -1;
	
		FMC_ENABLE_AP_UPDATE();
	
		FLASH_read_at( (uint32_t)AOS_PARTITION_PARAMETER, (uint32_t)psOtaFlag, sizeof(Ota_flag) );

		aos_kv_ota_flag_dump(psOtaFlag);

		#if !DEF_TEST	
		if ( (psOtaFlag->magic != AOS_OTA_FLAG_MAGIC) )
			goto exit_aos_ota_update;
		#endif
		
		switch ( psOtaFlag->upgrade_flag )
		{
			case UPG_SWITCH_APP:
				//Copy OTA_TEMP to Application partition.
				aos_upgrade_firmware();
				break;
				
			default:
			#if DEF_TEST	
				aos_upgrade_firmware();
			#endif
			break;
		}
		
		ret = 0;		
exit_aos_ota_update:
		FMC_DISABLE_AP_UPDATE();
		return ret;
}

int32_t main(void)
{
    uint8_t ch;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART0_Init();             /* Initialize UART0 */

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable FMC ISP function */
    FMC_Open();

#if DEF_TEST
		ch = ReceiveBytes(0);
	  switch(ch)
    {
    case '1':
				g_eraseonly = 1;
				break;
    default:
        break;
    }
#endif
		
		aos_ota_update();
		while((UART0->FIFOSTS & UART_FIFOSTS_TXEMPTY_Msk)==0);
	
		if ( g_jumptoap )
		{
 		  PutString("Jump to APROM!!\n");
			while((UART0->FIFOSTS & UART_FIFOSTS_TXEMPTY_Msk)==0);
			
			FMC_SetVectorPageAddr(0x0);
			FMC_SET_APROM_BOOT();
		}
		
    /* Reset CPU only to reset to new vector page */
    SYS_ResetCPU();

    /* Reset System to reset to new vector page. */
    // NVIC_SystemReset();

    while(1);
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
