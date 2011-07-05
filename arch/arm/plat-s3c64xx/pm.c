/* linux/arch/arm/plat-s3c64xx/pm.c
 *
 * Copyright (c) 2004,2006 Simtec Electronics
 *	Ben Dooks <ben@simtec.co.uk>
 *
 * S3C24XX Power Manager (Suspend-To-RAM) support
 *
 * See Documentation/arm/Samsung-S3C24XX/Suspend.txt for more information
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Parts based on arch/arm/mach-pxa/pm.c
 *
 * Thanks to Dimitry Andric for debugging
*/

#include <linux/init.h>
#include <linux/suspend.h>
#include <linux/errno.h>
#include <linux/time.h>
#include <linux/interrupt.h>
#include <linux/crc32.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/serial_core.h>
#include <linux/io.h>
#include <linux/platform_device.h>

#include <asm/cacheflush.h>

#include <plat/map-base.h>
#include <plat/regs-serial.h>
#include <plat/regs-clock.h>
#include <plat/regs-gpio.h>
#include <plat/regs-onenand.h>
#include <plat/gpio-cfg.h>

#include <mach/hardware.h>
#include <mach/regs-mem.h>
#include <mach/regs-irq.h>
#include <asm/gpio.h>

#include <asm/mach/time.h>

#include <plat/pm.h>
#include <plat/s3c64xx-dvfs.h>

unsigned long s3c_pm_flags;

int (*bml_suspend_fp)(struct device *dev, u32 state, u32 level);
EXPORT_SYMBOL(bml_suspend_fp);

int (*bml_resume_fp)(struct device *dev, u32 level);
EXPORT_SYMBOL(bml_resume_fp);

/* for external use */
#define PFX "s3c64xx-pm: "

#include <plat/power-clock-domain.h>

#ifdef CONFIG_S3C64XX_DOMAIN_GATING

static int domain_hash_map[15]={0, 1, 2, 2, 3, 3, 3, 4, 4, 4, 5, 5, 5, 6, 7};
#ifdef CONFIG_S3C64XX_DOMAIN_GATING_DEBUG
static char *domain_name[] = {"G","V", "I", "P", "F", "S", "ETM", "IROM"}; 
static char *domain_module_name[] = {"3D","MFC","JPG","CAM","2D","TV","SCA","ROT","PST","LCD","DM0","DM1","SEC","ETM","IRM"};
static char *dompowers[] = {"TOP","V","I","P","F","S","ETM","G"};
#endif /* CONFIG_S3C64XX_DOMAIN_GATING_DEBUG */

static spinlock_t power_lock;

static unsigned int s3c_domain_off_stat = 0x1FFC;

static void s3c_init_domain_power(void)
{
	spin_lock_init(&power_lock);
	s3c_set_normal_cfg(S3C64XX_DOMAIN_V, S3C64XX_LP_MODE, S3C64XX_MFC);
//	s3c_set_normal_cfg(S3C64XX_DOMAIN_G, S3C64XX_LP_MODE, S3C64XX_3D);
	s3c_set_normal_cfg(S3C64XX_DOMAIN_I, S3C64XX_LP_MODE, S3C64XX_JPEG);
	s3c_set_normal_cfg(S3C64XX_DOMAIN_I, S3C64XX_LP_MODE, S3C64XX_CAMERA);
//	s3c_set_normal_cfg(S3C64XX_DOMAIN_P, S3C64XX_LP_MODE, S3C64XX_2D);
	s3c_set_normal_cfg(S3C64XX_DOMAIN_P, S3C64XX_LP_MODE, S3C64XX_TVENC);
	s3c_set_normal_cfg(S3C64XX_DOMAIN_P, S3C64XX_LP_MODE, S3C64XX_SCALER);
	s3c_set_normal_cfg(S3C64XX_DOMAIN_F, S3C64XX_LP_MODE, S3C64XX_ROT);
	s3c_set_normal_cfg(S3C64XX_DOMAIN_F, S3C64XX_LP_MODE, S3C64XX_POST);
	s3c_set_normal_cfg(S3C64XX_DOMAIN_S, S3C64XX_LP_MODE, S3C64XX_SDMA0);
	s3c_set_normal_cfg(S3C64XX_DOMAIN_S, S3C64XX_LP_MODE, S3C64XX_SDMA1);
//	s3c_set_normal_cfg(S3C64XX_DOMAIN_S, S3C64XX_LP_MODE, S3C64XX_SECURITY);
//	s3c_set_normal_cfg(S3C64XX_DOMAIN_IROM, S3C64XX_LP_MODE, S3C64XX_IROM);
	/* LCD on. */
	s3c_set_normal_cfg(S3C64XX_DOMAIN_F, S3C64XX_ACTIVE_MODE, S3C64XX_LCD);

	/* ETM on. */
	s3c_set_normal_cfg(S3C64XX_DOMAIN_ETM, S3C64XX_ACTIVE_MODE, S3C64XX_ETM);

}
#ifdef CONFIG_S3C64XX_DOMAIN_GATING_DEBUG
int domain_off_check_n(unsigned int domain)
{
    unsigned int msk;
    switch( domain ) {
	case S3C64XX_DOMAIN_V:
		msk=S3C64XX_DOMAIN_V_MASK; break;
		
	case S3C64XX_DOMAIN_G:
		msk=S3C64XX_DOMAIN_G_MASK; break;
	
	case S3C64XX_DOMAIN_I:
		msk=S3C64XX_DOMAIN_I_MASK; break;
	
	case S3C64XX_DOMAIN_P:
		msk=S3C64XX_DOMAIN_P_MASK; break;
	
	case S3C64XX_DOMAIN_F:
		msk=S3C64XX_DOMAIN_F_MASK; break;
	
	case S3C64XX_DOMAIN_S:
		msk=S3C64XX_DOMAIN_S_MASK; break;
	
	case S3C64XX_DOMAIN_ETM:
		msk=S3C64XX_DOMAIN_ETM_MASK; break;
	
	case S3C64XX_DOMAIN_IROM:
		msk=S3C64XX_DOMAIN_IROM_MASK; break;
	
	default:
	    return 1;
    }
    return (s3c_domain_off_stat & msk);
}

static void print_domains(void) {
int i;
unsigned int dom=__raw_readl(S3C_BLK_PWR_STAT);
	for (i=0;i<8;i++)
		if (dom & (1<<(i))) printk("%s ",dompowers[i]);
}

static void print_blocking_modules(unsigned int mask) {
int i;
	for (i=0;i<15;i++)
		if (mask & (1<<i)) printk("%s ",domain_module_name[i]);
}
#endif

int domain_off_check(unsigned int domain)
{
    unsigned int msk;
    switch( domain ) {
	case S3C64XX_DOMAIN_V:
		msk=S3C64XX_DOMAIN_V_MASK; break;
		
	case S3C64XX_DOMAIN_G:
		msk=S3C64XX_DOMAIN_G_MASK; break;
	
	case S3C64XX_DOMAIN_I:
		msk=S3C64XX_DOMAIN_I_MASK; break;
	
	case S3C64XX_DOMAIN_P:
		msk=S3C64XX_DOMAIN_P_MASK; break;
	
	case S3C64XX_DOMAIN_F:
		msk=S3C64XX_DOMAIN_F_MASK; break;
	
	case S3C64XX_DOMAIN_S:
		msk=S3C64XX_DOMAIN_S_MASK; break;
	
	case S3C64XX_DOMAIN_ETM:
		msk=S3C64XX_DOMAIN_ETM_MASK; break;
	
	case S3C64XX_DOMAIN_IROM:
		msk=S3C64XX_DOMAIN_IROM_MASK; break;
	
	default:
	    return 1;
    }
    return (s3c_domain_off_stat & msk)?0:1;
}

EXPORT_SYMBOL(domain_off_check);

void s3c_set_normal_cfg(unsigned int config, unsigned int flag, unsigned int deviceID)
{
	unsigned int normal_cfg;
	int power_off_flag = 0;
	spin_lock(&power_lock);
	normal_cfg = __raw_readl(S3C_NORMAL_CFG);
	if(flag == S3C64XX_ACTIVE_MODE) {
		s3c_domain_off_stat |= (1 << deviceID);
		if(!(normal_cfg & config)) {
			normal_cfg |= (config);
			__raw_writel(normal_cfg, S3C_NORMAL_CFG);
#ifdef CONFIG_S3C64XX_DOMAIN_GATING_DEBUG
			printk("Domain-%s:%s ON (",
				domain_name[domain_hash_map[deviceID]],domain_module_name[deviceID]);
			print_domains();
			printk(") , modules:");
			print_blocking_modules(s3c_domain_off_stat);
			printk("\n");
#endif /* CONFIG_S3C64XX_DOMAIN_GATING_DEBUG */
		}
		
	}
	else if(flag == S3C64XX_LP_MODE) {
		s3c_domain_off_stat &= (~( 1 << deviceID));
#ifdef CONFIG_S3C64XX_DOMAIN_GATING_DEBUG
		power_off_flag = domain_off_check_n(config);
		if(power_off_flag == 0) {
#else
		power_off_flag = domain_off_check(config);
		if(power_off_flag == 1) {
#endif
			if(normal_cfg & config) {
				normal_cfg &= (~config);
				__raw_writel(normal_cfg, S3C_NORMAL_CFG);
#ifdef CONFIG_S3C64XX_DOMAIN_GATING_DEBUG
			printk("Domain-%s:%s OFF (",
				domain_name[domain_hash_map[deviceID]],domain_module_name[deviceID]);
			print_domains();
			printk(") , modules:");
			print_blocking_modules(s3c_domain_off_stat);
			printk("\n");
#endif /* CONFIG_S3C64XX_DOMAIN_GATING_DEBUG */
			}
		}
#ifdef CONFIG_S3C64XX_DOMAIN_GATING_DEBUG
		 else { printk("!Domain-%s:%s OFF cfg:%x, off_st=%x Blocking modules: ",
		    domain_name[domain_hash_map[deviceID]],domain_module_name[deviceID], normal_cfg, s3c_domain_off_stat);
			print_blocking_modules(power_off_flag); printk("\n"); }
#endif
	}
	spin_unlock(&power_lock);

}
EXPORT_SYMBOL(s3c_set_normal_cfg);

int s3c_wait_blk_pwr_ready(unsigned int config)
{
	unsigned int blk_pwr_stat;
	int timeout=200;
	
	/* Wait max 20 ms */
	while (!((blk_pwr_stat = __raw_readl(S3C_BLK_PWR_STAT)) & config)) {
		if (timeout == 0) {
			printk(KERN_ERR "config %x: blk power never ready.\n", config);
			return 1;
		}
		timeout--;
		udelay(100);
	}
#ifdef CONFIG_S3C64XX_DOMAIN_GATING_DEBUG
if (timeout<200)
    printk("BLK Power:%x took %d usec\n",config, (200-timeout)*100);
#endif
	return 0;
}
EXPORT_SYMBOL(s3c_wait_blk_pwr_ready);
#endif /* CONFIG_S3C64XX_DOMAIN_GATING */

static struct sleep_save core_save[] = {
	SAVE_ITEM(S3C_SDMA_SEL),
	SAVE_ITEM(S3C_HCLK_GATE),
	SAVE_ITEM(S3C_PCLK_GATE),
	SAVE_ITEM(S3C_SCLK_GATE),
	SAVE_ITEM(S3C_MEM0_CLK_GATE),
	SAVE_ITEM(S3C_CLK_SRC),
	SAVE_ITEM(S3C_CLK_DIV0),
	SAVE_ITEM(S3C_CLK_DIV1),
	SAVE_ITEM(S3C_CLK_DIV2),
	SAVE_ITEM(S3C_APLL_CON),
	SAVE_ITEM(S3C_MPLL_CON),
	SAVE_ITEM(S3C_EPLL_CON0),
	SAVE_ITEM(S3C_EPLL_CON1),
	SAVE_ITEM(S3C_NORMAL_CFG),
	SAVE_ITEM(S3C_AHB_CON0),
};

static struct sleep_save gpio_save[] = {
	SAVE_ITEM(S3C64XX_GPACON),
	SAVE_ITEM(S3C64XX_GPADAT),
	SAVE_ITEM(S3C64XX_GPAPUD),
	SAVE_ITEM(S3C64XX_GPACONSLP),
	SAVE_ITEM(S3C64XX_GPAPUDSLP),
	SAVE_ITEM(S3C64XX_GPBCON),
	SAVE_ITEM(S3C64XX_GPBDAT),
	SAVE_ITEM(S3C64XX_GPBPUD),
	SAVE_ITEM(S3C64XX_GPBCONSLP),
	SAVE_ITEM(S3C64XX_GPBPUDSLP),
	SAVE_ITEM(S3C64XX_GPCCON),
	SAVE_ITEM(S3C64XX_GPCDAT),
	SAVE_ITEM(S3C64XX_GPCPUD),
	SAVE_ITEM(S3C64XX_GPCCONSLP),
	SAVE_ITEM(S3C64XX_GPCPUDSLP),
	SAVE_ITEM(S3C64XX_GPDCON),
	SAVE_ITEM(S3C64XX_GPDDAT),
	SAVE_ITEM(S3C64XX_GPDPUD),
	SAVE_ITEM(S3C64XX_GPDCONSLP),
	SAVE_ITEM(S3C64XX_GPDPUDSLP),
	SAVE_ITEM(S3C64XX_GPECON),
	SAVE_ITEM(S3C64XX_GPEDAT),
	SAVE_ITEM(S3C64XX_GPEPUD),
	SAVE_ITEM(S3C64XX_GPECONSLP),
	SAVE_ITEM(S3C64XX_GPEPUDSLP),
	SAVE_ITEM(S3C64XX_GPFCON),
	SAVE_ITEM(S3C64XX_GPFDAT),
	SAVE_ITEM(S3C64XX_GPFPUD),
	SAVE_ITEM(S3C64XX_GPFCONSLP),
	SAVE_ITEM(S3C64XX_GPFPUDSLP),
	SAVE_ITEM(S3C64XX_GPGCON),
	SAVE_ITEM(S3C64XX_GPGDAT),
	SAVE_ITEM(S3C64XX_GPGPUD),
	SAVE_ITEM(S3C64XX_GPGCONSLP),
	SAVE_ITEM(S3C64XX_GPGPUDSLP),
	SAVE_ITEM(S3C64XX_GPHCON0),
	SAVE_ITEM(S3C64XX_GPHCON1),
	SAVE_ITEM(S3C64XX_GPHDAT),
	SAVE_ITEM(S3C64XX_GPHPUD),
	SAVE_ITEM(S3C64XX_GPHCONSLP),
	SAVE_ITEM(S3C64XX_GPHPUDSLP),
	SAVE_ITEM(S3C64XX_GPICON),
	SAVE_ITEM(S3C64XX_GPIDAT),
	SAVE_ITEM(S3C64XX_GPIPUD),
	SAVE_ITEM(S3C64XX_GPICONSLP),
	SAVE_ITEM(S3C64XX_GPIPUDSLP),
	SAVE_ITEM(S3C64XX_GPJCON),
	SAVE_ITEM(S3C64XX_GPJDAT),
	SAVE_ITEM(S3C64XX_GPJPUD),
	SAVE_ITEM(S3C64XX_GPJCONSLP),
	SAVE_ITEM(S3C64XX_GPJPUDSLP),
	SAVE_ITEM(S3C64XX_GPKCON),
	SAVE_ITEM(S3C64XX_GPKCON1),
	SAVE_ITEM(S3C64XX_GPKDAT),
	SAVE_ITEM(S3C64XX_GPKPUD),
	SAVE_ITEM(S3C64XX_GPLCON),
	SAVE_ITEM(S3C64XX_GPLCON1),
	SAVE_ITEM(S3C64XX_GPLDAT),
	SAVE_ITEM(S3C64XX_GPLPUD),
	SAVE_ITEM(S3C64XX_GPMCON),
	SAVE_ITEM(S3C64XX_GPMDAT),
	SAVE_ITEM(S3C64XX_GPMPUD),
	SAVE_ITEM(S3C64XX_GPNCON),
	SAVE_ITEM(S3C64XX_GPNDAT),
	SAVE_ITEM(S3C64XX_GPNPUD),
	SAVE_ITEM(S3C64XX_GPOCON),
	SAVE_ITEM(S3C64XX_GPODAT),
	SAVE_ITEM(S3C64XX_GPOPUD),
	SAVE_ITEM(S3C64XX_GPOCONSLP),
	SAVE_ITEM(S3C64XX_GPOPUDSLP),
	SAVE_ITEM(S3C64XX_GPPCON),
	SAVE_ITEM(S3C64XX_GPPDAT),
	SAVE_ITEM(S3C64XX_GPPPUD),
	SAVE_ITEM(S3C64XX_GPPCONSLP),
	SAVE_ITEM(S3C64XX_GPPPUDSLP),
	SAVE_ITEM(S3C64XX_GPQCON),
	SAVE_ITEM(S3C64XX_GPQDAT),
	SAVE_ITEM(S3C64XX_GPQPUD),
	SAVE_ITEM(S3C64XX_GPQCONSLP),
	SAVE_ITEM(S3C64XX_GPQPUDSLP),
	SAVE_ITEM(S3C64XX_PRIORITY),
	SAVE_ITEM(S3C64XX_SPCON),

	/* Special register */
	SAVE_ITEM(S3C64XX_SPC_BASE),
	SAVE_ITEM(S3C64XX_SPCONSLP),
	SAVE_ITEM(S3C64XX_SLPEN),
	SAVE_ITEM(S3C64XX_MEM0CONSTOP),
	SAVE_ITEM(S3C64XX_MEM1CONSTOP),
	SAVE_ITEM(S3C64XX_MEM0CONSLP0),
	SAVE_ITEM(S3C64XX_MEM0CONSLP1),
	SAVE_ITEM(S3C64XX_MEM1CONSLP),
	SAVE_ITEM(S3C64XX_MEM0DRVCON),
	SAVE_ITEM(S3C64XX_MEM1DRVCON),
};

/* this lot should be really saved by the IRQ code */
/* VICXADDRESSXX initilaization to be needed */
static struct sleep_save irq_save[] = {
	SAVE_ITEM(S3C64XX_VIC0INTSELECT),
	SAVE_ITEM(S3C64XX_VIC1INTSELECT),
	SAVE_ITEM(S3C64XX_VIC0INTENABLE),
	SAVE_ITEM(S3C64XX_VIC1INTENABLE),
	SAVE_ITEM(S3C64XX_VIC0SOFTINT),
	SAVE_ITEM(S3C64XX_VIC1SOFTINT),
};

static struct sleep_save sromc_save[] = {
	SAVE_ITEM(S3C64XX_SROM_BW),
	SAVE_ITEM(S3C64XX_SROM_BC0),
	SAVE_ITEM(S3C64XX_SROM_BC1),
	SAVE_ITEM(S3C64XX_SROM_BC2),
	SAVE_ITEM(S3C64XX_SROM_BC3),
	SAVE_ITEM(S3C64XX_SROM_BC4),
	SAVE_ITEM(S3C64XX_SROM_BC5),
};

static struct sleep_save onenand_save[] = {
	SAVE_ITEM(S3C_MEM_CFG0),
	SAVE_ITEM(S3C_BURST_LEN0),
	SAVE_ITEM(S3C_MEM_RESET0),
	SAVE_ITEM(S3C_INT_ERR_STAT0),
	SAVE_ITEM(S3C_INT_ERR_MASK0),
	SAVE_ITEM(S3C_INT_ERR_ACK0),
	SAVE_ITEM(S3C_ECC_ERR_STAT0),
	SAVE_ITEM(S3C_MANUFACT_ID0),
	SAVE_ITEM(S3C_DEVICE_ID0),
	SAVE_ITEM(S3C_DATA_BUF_SIZE0),
	SAVE_ITEM(S3C_BOOT_BUF_SIZE0),
	SAVE_ITEM(S3C_BUF_AMOUNT0),
	SAVE_ITEM(S3C_TECH0),
	SAVE_ITEM(S3C_FBA_WIDTH0),
	SAVE_ITEM(S3C_FPA_WIDTH0),
	SAVE_ITEM(S3C_FSA_WIDTH0),
	SAVE_ITEM(S3C_REVISION0),
	SAVE_ITEM(S3C_DATARAM00),
	SAVE_ITEM(S3C_DATARAM10),
	SAVE_ITEM(S3C_SYNC_MODE0),
	SAVE_ITEM(S3C_TRANS_SPARE0),
	SAVE_ITEM(S3C_DBS_DFS_WIDTH0),
	SAVE_ITEM(S3C_PAGE_CNT0),
	SAVE_ITEM(S3C_ERR_PAGE_ADDR0),
	SAVE_ITEM(S3C_BURST_RD_LAT0),
	SAVE_ITEM(S3C_INT_PIN_ENABLE0),
	SAVE_ITEM(S3C_INT_MON_CYC0),
	SAVE_ITEM(S3C_ACC_CLOCK0),
	SAVE_ITEM(S3C_SLOW_RD_PATH0),
	SAVE_ITEM(S3C_ERR_BLK_ADDR0),
	SAVE_ITEM(S3C_FLASH_VER_ID0),
	SAVE_ITEM(S3C_FLASH_AUX_CNTRL0),
	SAVE_ITEM(S3C_FLASH_AFIFO_CNT0),

	SAVE_ITEM(S3C_MEM_CFG1),
	SAVE_ITEM(S3C_BURST_LEN1),
	SAVE_ITEM(S3C_MEM_RESET1),
	SAVE_ITEM(S3C_INT_ERR_STAT1),
	SAVE_ITEM(S3C_INT_ERR_MASK1),
	SAVE_ITEM(S3C_INT_ERR_ACK1),
	SAVE_ITEM(S3C_ECC_ERR_STAT1),
	SAVE_ITEM(S3C_MANUFACT_ID1),
	SAVE_ITEM(S3C_DEVICE_ID1),
	SAVE_ITEM(S3C_DATA_BUF_SIZE1),
	SAVE_ITEM(S3C_BOOT_BUF_SIZE1),
	SAVE_ITEM(S3C_BUF_AMOUNT1),
	SAVE_ITEM(S3C_TECH1),
	SAVE_ITEM(S3C_FBA_WIDTH1),
	SAVE_ITEM(S3C_FPA_WIDTH1),
	SAVE_ITEM(S3C_FSA_WIDTH1),
	SAVE_ITEM(S3C_REVISION1),
	SAVE_ITEM(S3C_DATARAM01),
	SAVE_ITEM(S3C_DATARAM11),
	SAVE_ITEM(S3C_SYNC_MODE1),
	SAVE_ITEM(S3C_TRANS_SPARE1),
	SAVE_ITEM(S3C_DBS_DFS_WIDTH1),
	SAVE_ITEM(S3C_PAGE_CNT1),
	SAVE_ITEM(S3C_ERR_PAGE_ADDR1),
	SAVE_ITEM(S3C_BURST_RD_LAT1),
	SAVE_ITEM(S3C_INT_PIN_ENABLE1),
	SAVE_ITEM(S3C_INT_MON_CYC1),
	SAVE_ITEM(S3C_ACC_CLOCK1),
	SAVE_ITEM(S3C_SLOW_RD_PATH1),
	SAVE_ITEM(S3C_ERR_BLK_ADDR1),
	SAVE_ITEM(S3C_FLASH_VER_ID1),
	SAVE_ITEM(S3C_FLASH_AUX_CNTRL1),
	SAVE_ITEM(S3C_FLASH_AFIFO_CNT1),
};

#ifdef CONFIG_S3C_PM_DEBUG

#define SAVE_UART(va) \
	SAVE_ITEM((va) + S3C_ULCON), \
	SAVE_ITEM((va) + S3C_UCON), \
	SAVE_ITEM((va) + S3C_UFCON), \
	SAVE_ITEM((va) + S3C_UMCON), \
	SAVE_ITEM((va) + S3C_UBRDIV)

static struct sleep_save uart_save[] = {
	SAVE_UART(S3C24XX_VA_UART0),
	SAVE_UART(S3C24XX_VA_UART1),
#ifndef CONFIG_CPU_S3C2400
	SAVE_UART(S3C24XX_VA_UART2),
#endif	/* CONFIG_CPU_S3C2400 */
};

/* debug
 *
 * we send the debug to printascii() to allow it to be seen if the
 * system never wakes up from the sleep
*/

extern void printascii(const char *);

void pm_dbg(const char *fmt, ...)
{
	va_list va;
	char buff[256];

	va_start(va, fmt);
	vsprintf(buff, fmt, va);
	va_end(va);

	printascii(buff);
}

static void s3c_pm_debug_init(void)
{
	unsigned long tmp = __raw_readl(S3C_CLKCON);

	/* re-start uart clocks */
	tmp |= S3C_CLKCON_UART0;
	tmp |= S3C_CLKCON_UART1;
	tmp |= S3C_CLKCON_UART2;

	__raw_writel(tmp, S3C_CLKCON);
	udelay(10);
}

#define DBG(fmt...) pm_dbg(fmt)
#else
#ifdef CONFIG_PM_DEBUG
#define DBG printk
#else
#define DBG(fmt...)
#endif
#define s3c6410_pm_debug_init() do { } while(0)
#endif	/* CONFIG_S3C_PM_DEBUG */

#if defined(CONFIG_S3C_PM_CHECK) && CONFIG_S3C_PM_CHECK_CHUNKSIZE != 0

/* suspend checking code...
 *
 * this next area does a set of crc checks over all the installed
 * memory, so the system can verify if the resume was ok.
 *
 * CONFIG_S3C6410_PM_CHECK_CHUNKSIZE defines the block-size for the CRC,
 * increasing it will mean that the area corrupted will be less easy to spot,
 * and reducing the size will cause the CRC save area to grow
*/

#define CHECK_CHUNKSIZE (CONFIG_S3C_PM_CHECK_CHUNKSIZE * 1024)

static u32 crc_size;	/* size needed for the crc block */
static u32 *crcs;	/* allocated over suspend/resume */

typedef u32 *(run_fn_t)(struct resource *ptr, u32 *arg);

/* s3c6410_pm_run_res
 *
 * go thorugh the given resource list, and look for system ram
*/

static void s3c6410_pm_run_res(struct resource *ptr, run_fn_t fn, u32 *arg)
{
	while (ptr != NULL) {
		if (ptr->child != NULL)
			s3c6410_pm_run_res(ptr->child, fn, arg);

		if ((ptr->flags & IORESOURCE_MEM) &&
		    strcmp(ptr->name, "System RAM") == 0) {
			DBG("Found system RAM at %08lx..%08lx\n",
			    ptr->start, ptr->end);
			arg = (fn)(ptr, arg);
		}

		ptr = ptr->sibling;
	}
}

static void s3c6410_pm_run_sysram(run_fn_t fn, u32 *arg)
{
	s3c6410_pm_run_res(&iomem_resource, fn, arg);
}

static u32 *s3c6410_pm_countram(struct resource *res, u32 *val)
{
	u32 size = (u32)(res->end - res->start)+1;

	size += CHECK_CHUNKSIZE-1;
	size /= CHECK_CHUNKSIZE;

	DBG("Area %08lx..%08lx, %d blocks\n", res->start, res->end, size);

	*val += size * sizeof(u32);
	return val;
}

/* s3c6410_pm_prepare_check
 *
 * prepare the necessary information for creating the CRCs. This
 * must be done before the final save, as it will require memory
 * allocating, and thus touching bits of the kernel we do not
 * know about.
*/

static void s3c6410_pm_check_prepare(void)
{
	crc_size = 0;

	s3c6410_pm_run_sysram(s3c6410_pm_countram, &crc_size);

	DBG("s3c6410_pm_prepare_check: %u checks needed\n", crc_size);

	crcs = kmalloc(crc_size+4, GFP_KERNEL);
	if (crcs == NULL)
		printk(KERN_ERR "Cannot allocated CRC save area\n");
}

static u32 *s3c6410_pm_makecheck(struct resource *res, u32 *val)
{
	unsigned long addr, left;

	for (addr = res->start; addr < res->end;
	     addr += CHECK_CHUNKSIZE) {
		left = res->end - addr;

		if (left > CHECK_CHUNKSIZE)
			left = CHECK_CHUNKSIZE;

		*val = crc32_le(~0, phys_to_virt(addr), left);
		val++;
	}

	return val;
}

/* s3c6410_pm_check_store
 *
 * compute the CRC values for the memory blocks before the final
 * sleep.
*/

static void s3c6410_pm_check_store(void)
{
	if (crcs != NULL)
		s3c6410_pm_run_sysram(s3c6410_pm_makecheck, crcs);
}

/* in_region
 *
 * return TRUE if the area defined by ptr..ptr+size contatins the
 * what..what+whatsz
*/

static inline int in_region(void *ptr, int size, void *what, size_t whatsz)
{
	if ((what+whatsz) < ptr)
		return 0;

	if (what > (ptr+size))
		return 0;

	return 1;
}

static u32 *s3c6410_pm_runcheck(struct resource *res, u32 *val)
{
	void *save_at = phys_to_virt(s3c6410_sleep_save_phys);
	unsigned long addr;
	unsigned long left;
	void *ptr;
	u32 calc;

	for (addr = res->start; addr < res->end;
	     addr += CHECK_CHUNKSIZE) {
		left = res->end - addr;

		if (left > CHECK_CHUNKSIZE)
			left = CHECK_CHUNKSIZE;

		ptr = phys_to_virt(addr);

		if (in_region(ptr, left, crcs, crc_size)) {
			DBG("skipping %08lx, has crc block in\n", addr);
			goto skip_check;
		}

		if (in_region(ptr, left, save_at, 32*4 )) {
			DBG("skipping %08lx, has save block in\n", addr);
			goto skip_check;
		}

		/* calculate and check the checksum */

		calc = crc32_le(~0, ptr, left);
		if (calc != *val) {
			printk(KERN_ERR PFX "Restore CRC error at "
			       "%08lx (%08x vs %08x)\n", addr, calc, *val);

			DBG("Restore CRC error at %08lx (%08x vs %08x)\n",
			    addr, calc, *val);
		}

	skip_check:
		val++;
	}

	return val;
}

/* s3c6410_pm_check_restore
 *
 * check the CRCs after the restore event and free the memory used
 * to hold them
*/

static void s3c6410_pm_check_restore(void)
{
	if (crcs != NULL) {
		s3c6410_pm_run_sysram(s3c6410_pm_runcheck, crcs);
		kfree(crcs);
		crcs = NULL;
	}
}

#else

#define s3c6410_pm_check_prepare() do { } while(0)
#define s3c6410_pm_check_restore() do { } while(0)
#define s3c6410_pm_check_store()   do { } while(0)
#endif	/* defined(CONFIG_S3C_PM_CHECK) && CONFIG_S3C_PM_CHECK_CHUNKSIZE != 0 */

/* helper functions to save and restore register state */

void s3c6410_pm_do_save(struct sleep_save *ptr, int count)
{
	for (; count > 0; count--, ptr++) {
		ptr->val = __raw_readl(ptr->reg);
		//DBG("saved %p value %08lx\n", ptr->reg, ptr->val);
	}
}

/* s3c6410_pm_do_restore
 *
 * restore the system from the given list of saved registers
 *
 * Note, we do not use DBG() in here, as the system may not have
 * restore the UARTs state yet
*/

void s3c6410_pm_do_restore(struct sleep_save *ptr, int count)
{
	for (; count > 0; count--, ptr++) {
		//printk(KERN_DEBUG "restore %p (restore %08lx, was %08x)\n",
		       //ptr->reg, ptr->val, __raw_readl(ptr->reg));

		__raw_writel(ptr->val, ptr->reg);
	}
}

/* s3c6410_pm_do_restore_core
 *
 * similar to s36410_pm_do_restore_core
 *
 * WARNING: Do not put any debug in here that may effect memory or use
 * peripherals, as things may be changing!
*/
void s3c6410_pm_do_restore_core(struct sleep_save *ptr, int count)
{
	for (; count > 0; count--, ptr++) 
		__raw_writel(ptr->val, ptr->reg);
}

/* s3c6410_pm_do_save_phy
 *
 * save register of system
 *
 * Note, I made this function to support driver with ioremap.
 * If you want to use this function, you should to input as first parameter
 * struct sleep_save_phy type
*/

int s3c_pm_do_save_phy(struct sleep_save_phy *ptr, struct platform_device *pdev, int count)
{
	void __iomem *target_reg;
	struct resource *res;
	u32 reg_size;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if(res == NULL){
		printk(KERN_ERR "%s resource get error\n",__FUNCTION__);
		return 0;
	}
	reg_size = res->end - res->start + 1;
	target_reg = ioremap(res->start,reg_size);

	for (; count > 0; count--, ptr++) {
		ptr->val = readl(target_reg + (ptr->reg));
	}

	return 0;
}

/* s3c6410_pm_do_restore_phy
 *
 * restore register of system
 *
 * Note, I made this function to support driver with ioremap.
 * If you want to use this function, you should to input as first parameter
 * struct sleep_save_phy type
*/

int s3c_pm_do_restore_phy(struct sleep_save_phy *ptr, struct platform_device *pdev, int count)
{
	void __iomem *target_reg;
	struct resource *res;
	u32 reg_size;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if(res == NULL){
		printk(KERN_ERR "%s resource get error\n",__FUNCTION__);
		return 0;
	}
	reg_size = res->end - res->start + 1;
	target_reg = ioremap(res->start,reg_size);

	for (; count > 0; count--, ptr++) {
		writel(ptr->val, (target_reg + ptr->reg));
	}

	return 0;
}

extern void s3c_config_wakeup_source(void);
extern void s3c_config_sleep_gpio(void);
extern void s3c_config_wakeup_gpio(void);

static unsigned int s3c_eint_mask_val;

static void s3c6410_pm_configure_extint(void)
{
	/* for each of the external interrupts (EINT0..EINT15) we
	 * need to check wether it is an external interrupt source,
	 * and then configure it as an input if it is not
	*/
	s3c_eint_mask_val = __raw_readl(S3C_EINT_MASK);

	s3c_config_wakeup_source();
}

void (*pm_cpu_prep)(void);
void (*pm_cpu_sleep)(void);

#define any_allowed(mask, allow) (((mask) & (allow)) != (allow))

/* s3c6410_pm_enter
 *
 * central control for sleep/resume process
*/

//extern unsigned int extra_eint0pend;

static int s3c6410_pm_enter(suspend_state_t state)
{
	static unsigned long regs_save[16];
	unsigned int tmp;
	unsigned int wakeup_stat = 0x0;
	unsigned int eint0pend = 0x0;

//	unsigned int jumpaddr = phys_to_virt(0x50000400);
//	unsigned int informvalue = 0x0;
//	unsigned int tmp1;
//	unsigned int tmp2 = 0x0;
//	int i;

	// ensure the debug is initialised (if enabled)

	DBG("s3c6410_pm_enter(%d)\n", state);

//	if (bml_suspend_fp)
//		bml_suspend_fp(NULL, 0, 0);

	if (pm_cpu_prep == NULL || pm_cpu_sleep == NULL) {
		printk(KERN_ERR PFX "error: no cpu sleep functions set\n");
		return -EINVAL;
	}

	// prepare check area if configured
	s3c6410_pm_check_prepare();

	// store the physical address of the register recovery block
	s3c6410_sleep_save_phys = virt_to_phys(regs_save);

	DBG("s3c6410_sleep_save_phys=0x%08lx\n", s3c6410_sleep_save_phys);
	// save all necessary core registers not covered by the drivers

	s3c6410_pm_do_save(gpio_save, ARRAY_SIZE(gpio_save));
	s3c6410_pm_do_save(irq_save, ARRAY_SIZE(irq_save));
	s3c6410_pm_do_save(core_save, ARRAY_SIZE(core_save));
	s3c6410_pm_do_save(sromc_save, ARRAY_SIZE(sromc_save));
	s3c6410_pm_do_save(onenand_save, ARRAY_SIZE(onenand_save));
	//FIXME ADD SDHC SAVE???


// Marc version of the sleep process - winmo disass...

	//F NOT FLIGHTMODUS

	// ensure INF_REG0  has the resume address 
//	__raw_writel(virt_to_phys(s3c6410_cpu_resume), S3C_INFORM0); //ensure 0x503 has the right resume addres --> is this also the JUMP DISSASEMBLE!!!
	//asm("b s3c6410_cpu_resume");
//	int aap = virt_to_phys(s3c6410_cpu_resume);
//	asm("b aap");
//    asm volatile("ldr  r1, =0x5030000c"); // 0xe59f1658
//    asm volatile("ldr  r0, [r1] "); // 0xe5910000
//    asm volatile("mov  pc, r0"); //0xe1a0f000 //TEST IN WINMO! 503146a4 OF 8 nops enhetzelfde adres
//	asm volatile("ldr  r1, =0x7e00fa00"); //0xe59515e0
//	asm volatile("ldr  r0, [r1] "); //0xe5910000 --> OK
//	asm volatile("mov  pc, r0");//0xe1a0f000

/*	asm volatile ("SUB r0, #0xC"); //E240000C ->fa04 , works
	asm volatile("ldr  r1, [r0]"); //E5901000, works
	asm volatile("mov  pc, r1");//E1a0f001
	asm ("nop");//e320f000
	asm ("nop");//e320f000
	asm ("nop");//e320f000.


PFW 0x50008000 1 0xE240000C
PFW 0x50008004 1 0xE5901000
PFW 0x50008008 1 0xE1a0f001
PFW 0x5000800C 1 0xe320f000
PFW 0x50008010 1 0xe320f000
PFW 0x50008014 1 0xe320f000
PFW 0x7e00Fa00 1 0x50009114 !!! WORKS!!!
PFW 0x7e00Fa0C 1 0xF
PFW 0x7e00Fa04 1 0xF


*/


//E59F01A8
	//asm ("ldr  r0, 0x7e00fa0C"); 
	//asm volatile ("ldr  r0, 0x7e00fa0C"); 
//	asm ("ldr  r0, -4"); 
//	asm volatile ("ldr  r0, =0x7e00fa0C"); 
//A7 51 00 EA0051a7 11101010 000000000101000110100111 --> 51A7
//NOP = e1a00000 asm volatile("mov  r0, r0");
	
	DBG(" resume address: %x \n", virt_to_phys(s3c6410_cpu_resume)); //5040f4a0
	printk("SLEEP: resume address WINMO = %x \n", __raw_readl(phys_to_virt(0x50008000))); //resume addres =e59b0004
/*	__raw_writel(0xe59515e0, (phys_to_virt(0x50300000))); //PFW 0x50300000 1 0xe59f1658
	__raw_writel(0xe5910000, (phys_to_virt(0x50300004))); //PFW 0x50300004 1 0xe5910000
	__raw_writel(0xe1a0f000, (phys_to_virt(0x50300008)));// PFW 0x50300008 1 0xe1a0f000
//50000400
	__raw_writel(0xe59515e0, (phys_to_virt(0x50000400))); //PFW 0x50300000 1 0xe59f1658
	__raw_writel(0xe5910000, (phys_to_virt(0x50000404))); //PFW 0x50300004 1 0xe5910000
	__raw_writel(0xe1a0f000, (phys_to_virt(0x50000408)));// PFW 0x50300008 1 0xe1a0f000	//__raw_writel(0xcc129fe5, (phys_to_virt(0x50300000)));//PFW 0x5030000c 1 0x503146a4
*/
//write return code to RAM
	__raw_writel(0xE240000C, (phys_to_virt(0x50008000))); //PFW 0x50300000 1 0xe59f1658
	__raw_writel(0xE5901000, (phys_to_virt(0x50008004))); //PFW 0x50300004 1 0xe5910000
	__raw_writel(0xE1a0f001, (phys_to_virt(0x50008008)));// PFW 0x50300008 1 0xe1a0f000	//__raw_write
	__raw_writel(0xe320f000, (phys_to_virt(0x5000800C)));// PFW 0x50300008 1 0xe1a0f000	//__raw_write
	__raw_writel(0xe320f000, (phys_to_virt(0x50008010)));// PFW 0x50300008 1 0xe1a0f000	//__raw_write
	__raw_writel(0xe320f000, (phys_to_virt(0x50008014)));// PFW 0x50300008 1 0xe1a0f000	//__raw_write
	__raw_writel(virt_to_phys(s3c6410_cpu_resume), S3C_INFORM0);

//	__raw_writel(virt_to_phys(s3c6410_cpu_resume), (phys_to_virt(0x5000040c)));
//	__raw_writel(virt_to_phys(s3c6410_cpu_resume), (phys_to_virt(0x5000800c)));
//	__raw_writel(virt_to_phys(s3c6410_cpu_resume), (phys_to_virt(0x5030000c)));


	//__raw_writel(0x80000000, S3C_INFORM1);
        //OTHERS (f900) routine sub_278
	//CONFIRM right eboot
	//LDR     R1, =(loc_C6E0+2) WHERE IS LOC_C6E0 + 2!!!!!!!!!!!! op adress Eo)
	//LDR     R1, =unk_7FF7
	//THEORY CANT FIND SOMETHING BECAUSE NOT IN RAM OR SOMETHING -->RESET
	//3c6400E0 IS IT INSTEAD OF A KEY MAYBE A CODE?

	// set the irq configuration for wake 
	s3c6410_pm_configure_extint();

	//GPNPUD 838 ROUTINE BIC 0xC00, ORR 0x800
	tmp = __raw_readl(S3C64XX_GPNPUD);
	tmp &= ~(0xC00);
	tmp |= (0x800); 
	__raw_writel(tmp, S3C64XX_GPNPUD);	


	//GPNCON 830 routine BIC 0xC00, ORR 0x800
	tmp = __raw_readl(S3C64XX_GPNCON);
	tmp &= ~(0xC00);
	tmp |= (0x800); 
	__raw_writel(tmp, S3C64XX_GPNCON);
	//GPLPUD 81c ROUTINE BIC 0xC00000
	tmp = __raw_readl(S3C64XX_GPLPUD);
	tmp &= ~(0xC00000);
	__raw_writel(tmp, S3C64XX_GPLPUD);	

	//GPLCON1 814 ROUTINE BIC 0xF000
	tmp = __raw_readl(S3C64XX_GPLCON1);
	tmp &= ~(0xF000);
	__raw_writel(tmp, S3C64XX_GPLCON1);	

	//GPMPUD 828 ROUTINE BIC 0x30
	tmp = __raw_readl(S3C64XX_GPMPUD);
	tmp &= ~(0x30);
	__raw_writel(tmp, S3C64XX_GPMPUD);	

	//GPMCON 820 ROUTINE BIC 0xF00 ORR 0x300
	tmp = __raw_readl(S3C64XX_GPMCON);
	tmp &= ~(0xF00);//WINMO BIC 0x61
	tmp |= (0x300); //WINMO ORR 0x60
	__raw_writel(tmp, S3C64XX_GPMCON);	

	//OTHERS ROUTINE BIC ???? (50874) -->THINK ALREADY IN CLOCK-s3c64xx

	//GPPCON BIC 0xc000000 ORR 0x8000000
	tmp = __raw_readl(S3C64XX_GPPCON);
	tmp &= ~(0xc000000);
	tmp |= (0x8000000);
	__raw_writel(tmp, S3C64XX_GPPCON);	

	//GPKCON0  800 BIC 0xF0 ORR 0x10
	tmp = __raw_readl(S3C64XX_GPKCON);
	tmp &= ~(0xf0);
	tmp |= (0x10);
	__raw_writel(tmp, S3C64XX_GPKCON);
	//GPKDAT  808 BIC 0x2
	tmp = __raw_readl(S3C64XX_GPKDAT);
	tmp &= ~(0x2);
	__raw_writel(tmp, S3C64XX_GPKDAT);
	//GPKCON1  818 BIC 0x100
	tmp = __raw_readl(S3C64XX_GPKCON1);
	tmp &= ~(0x100);
	__raw_writel(tmp, S3C64XX_GPKCON1);


	//WINMO EINT_MASK SHOULD BE 0xDF7FDDF, EINTOCONś NOT KNOWN YET! 1101111101111111110111011111
						//EINT 5, 9, 19, 25 --> NO DPRAM??
	 
	//PWR_CONFIG ROUTINE (50AF0)
	//LOAD GPLDAT BIC 0x40
	tmp = __raw_readl(S3C64XX_GPLDAT);
	tmp &= ~(0x40);
	__raw_writel(tmp, S3C64XX_GPLDAT);

	//STORE 0xF in INFORM3
	__raw_writel(0xF, S3C_INFORM3);

	DBG(" payload = %x \n", __raw_readl(phys_to_virt(0x50300000)));
	DBG(" payload = %x \n", __raw_readl(phys_to_virt(0x50300004)));
	DBG(" payload = %x \n", __raw_readl(phys_to_virt(0x50300008)));
	DBG(" resume addres after = %x \n", __raw_readl(phys_to_virt(0x5030000c))); //resume addres = 5040f4a0
	DBG(" S3C_APLL_LOCK = %x \n", __raw_readl(S3C_APLL_LOCK));
	DBG(" S3C_MPLL_LOCK = %x \n", __raw_readl(S3C_MPLL_LOCK));
	DBG(" S3C_EPLL_LOCK = %x \n", __raw_readl(S3C_EPLL_LOCK));
	DBG(" S3C_APLL_CON = %x \n", __raw_readl(S3C_APLL_CON));
	DBG(" S3C_MPLL_CON = %x \n", __raw_readl(S3C_MPLL_CON));
	DBG(" S3C_EPLL_CON0 = %x \n", __raw_readl(S3C_EPLL_CON0));
	DBG(" S3C_EPLL_CON1 = %x \n", __raw_readl(S3C_EPLL_CON1));
	DBG(" S3C_CLK_SRC = %x \n", __raw_readl(S3C_CLK_SRC));
	DBG(" S3C_CLK_DIV0 = %x \n", __raw_readl(S3C_CLK_DIV0));
	DBG(" S3C_CLK_DIV1 = %x \n", __raw_readl(S3C_CLK_DIV1));
	DBG(" S3C_CLK_DIV2 = %x \n", __raw_readl(S3C_CLK_DIV2));
	DBG(" S3C_CLK_OUT = %x \n", __raw_readl(S3C_CLK_OUT));
	DBG(" S3C_HCLK_GATE = %x \n", __raw_readl(S3C_HCLK_GATE));	
	DBG(" S3C_PCLK_GATE = %x \n", __raw_readl(S3C_PCLK_GATE));
	DBG(" S3C_SCLK_GATE = %x \n", __raw_readl(S3C_SCLK_GATE));
	DBG(" S3C_MEM0_CLK_GATE = %x \n", __raw_readl(S3C_MEM0_CLK_GATE));
	DBG(" S3C_MEM_SYS_CFG = %x \n", __raw_readl(S3C_MEM_SYS_CFG));
	DBG(" S3C_MEM_CFG_STAT = %x \n", __raw_readl(S3C_MEM_CFG_STAT));
	DBG(" S3C_QOS_OVERRIDE1 = %x \n", __raw_readl(S3C_QOS_OVERRIDE1));
	DBG(" S3C_PWR_CFG = %x \n", __raw_readl(S3C_PWR_CFG));
	DBG(" S3C_EINT_MASK = %x \n", __raw_readl(S3C_EINT_MASK));
	DBG(" S3C_STOP_CFG = %x \n", __raw_readl(S3C_STOP_CFG));
	DBG(" S3C_SLEEP_CFG = %x \n", __raw_readl(S3C_SLEEP_CFG));
	DBG(" S3C_NORMAL_CFG = %x \n", __raw_readl(S3C_NORMAL_CFG));
	DBG(" S3C_STOP_MEM_CFG = %x \n", __raw_readl(S3C_STOP_MEM_CFG));
	DBG(" S3C_OSC_FREQ = %x \n", __raw_readl(S3C_OSC_FREQ));
	DBG(" S3C_OSC_STABLE = %x \n", __raw_readl(S3C_OSC_STABLE));
	DBG(" S3C_PWR_STABLE = %x \n", __raw_readl(S3C_PWR_STABLE));
	DBG(" S3C_MISC_CON = %x \n", __raw_readl(S3C_MISC_CON));
	DBG(" S3C_OTHERS = %x \n", __raw_readl(S3C_OTHERS));
	DBG(" S3C_RST_STAT = %x \n", __raw_readl(S3C_RST_STAT));
	DBG(" S3C_WAKEUP_STAT = %x \n", __raw_readl(S3C_WAKEUP_STAT));
	DBG(" S3C_BLK_PWR_STAT = %x \n", __raw_readl(S3C_BLK_PWR_STAT));
	DBG(" S3C_INFORM0 = %x \n", __raw_readl(S3C_INFORM0));
	DBG(" S3C_INFORM1 = %x \n", __raw_readl(S3C_INFORM1));
	DBG(" S3C_INFORM2 = %x \n", __raw_readl(S3C_INFORM2));
	DBG(" S3C64XX_SPCONSLP = %x \n", __raw_readl(S3C64XX_SPCONSLP));
	DBG(" S3C64XX_SLPEN = %x \n", __raw_readl(S3C64XX_SLPEN));


	// call cpu specific preperation

	pm_cpu_prep();

	// flush cache back to ram

	flush_cache_all();
	s3c6410_pm_check_store();

	s3c_config_sleep_gpio();	

	tmp = __raw_readl(S3C64XX_SPCONSLP); //HAVE TO FIND THIS ONE
	tmp &= ~(0x3 << 12);
// Reset Out: set it output, value 1! ????
	__raw_writel(tmp | (0x1 << 12), S3C64XX_SPCONSLP);

	// send the cpu to sleep... 

	__raw_writel(0xffffffff, S3C64XX_VIC0INTENCLEAR);
	__raw_writel(0xffffffff, S3C64XX_VIC1INTENCLEAR);
	__raw_writel(0xffffffff, S3C64XX_VIC0SOFTINTCLEAR);
	__raw_writel(0xffffffff, S3C64XX_VIC1SOFTINTCLEAR);

	__raw_writel(0x03E00001, S3C_HCLK_GATE); //WINMO
	__raw_writel(0xF2040000, S3C_PCLK_GATE); //WINMO ??
	__raw_writel(0x00000000, S3C_SCLK_GATE); //WINMO
	__raw_writel(0x00000000, S3C_MEM0_CLK_GATE);//??

	__raw_writel(__raw_readl(S3C_NORMAL_CFG)|0x1f600,S3C_NORMAL_CFG);
#ifdef CONFIG_S3C64XX_DOMAIN_GATING
	s3c_wait_blk_pwr_ready(0x7f);		// wait until on 
#endif
	__raw_writel(0x2, S3C64XX_SLPEN); //WINMO CHECK, changed position

	//__raw_writel(0xffffffff, S3C64XX_EINT0PEND); //WINMO
	//__raw_writel(0xffffffff, S3C64XX_EINT0MASK); //WINMO

	__raw_writel(0x1, S3C_OSC_STABLE);
	__raw_writel(0x3, S3C_PWR_STABLE);

	//INFORM1 ROUTINE (CHECK RESUME ADDR)
/*	for (i = 0; i < 26; i++){ //essentially adds 0x40000000 everytime R2 is not null otherwise, nul
	 tmp1 =  __raw_readl(jumpaddr + (4*i));
	 tmp1 =  (tmp1 & 0x1);	 
	 tmp1 =  (tmp1 << 31);
	 tmp1 |= (tmp1 >> 1);
	 informvalue += tmp1;
	}
*/	__raw_writel(/*informvalue*/0, S3C_INFORM1);

	 //WINMO: invalidate cache routine


	// Set WFI instruction to SLEEP mode
	tmp = __raw_readl(S3C_PWR_CFG);
	tmp &= ~((0x3<<5) /*| 1*/);			//WINMO BIC 0x61
// 00-normal 01-idle 10-stop 11-sleep
	tmp |= (0x3<<5);				//WINMO ORR 0x60
	__raw_writel(tmp, S3C_PWR_CFG);
	tmp = __raw_readl(S3C_SLEEP_CFG); //CHECK WINMO:  BIC 0x61
	tmp &= ~(0x61<<0);
	__raw_writel(tmp, S3C_SLEEP_CFG);


	//Clear WAKEUP_STAT register for next wakeup -jc.lee 
	// If this register do not be cleared, Wakeup will be failed
	__raw_writel(__raw_readl(S3C_WAKEUP_STAT), S3C_WAKEUP_STAT);

	// s3c6410_cpu_save will also act as our return point from when
	// we resume as it saves its own register state, so use the return
	// code to differentiate return from save and return from sleep



	//MARC: STORE THE RESUME ADRESS IN THE @ CONTROL REGISTER 5788C

	if (s3c6410_cpu_save(regs_save) == 0) {
		flush_cache_all();
		pm_cpu_sleep();
	}

	// restore the cpu state
	cpu_init();

	__raw_writel(s3c_eint_mask_val, S3C_EINT_MASK);

	// restore the system state
	s3c6410_pm_do_restore_core(core_save, ARRAY_SIZE(core_save));
	s3c6410_pm_do_restore(sromc_save, ARRAY_SIZE(sromc_save));
	s3c6410_pm_do_restore(gpio_save, ARRAY_SIZE(gpio_save));
	s3c6410_pm_do_restore(irq_save, ARRAY_SIZE(irq_save));
	s3c6410_pm_do_restore(onenand_save, ARRAY_SIZE(onenand_save));
	//FIXME ADHC restore
	__raw_writel(0x0, S3C64XX_SLPEN);

	wakeup_stat = __raw_readl(S3C_WAKEUP_STAT);
	eint0pend = __raw_readl(S3C64XX_EINT0PEND);

	__raw_writel(eint0pend, S3C64XX_EINT0PEND);

	DBG("post sleep, preparing to return\n");

	s3c6410_pm_check_restore();

//	extra_eint0pend = eint0pend;

	pr_info("%s: WAKEUP_STAT(0x%08x), EINT0PEND(0x%08x)\n",
			__func__, wakeup_stat, eint0pend);

	s3c_config_wakeup_gpio();	

//	if (bml_resume_fp)
//		bml_resume_fp(NULL, 0);

	// ok, let's return from sleep
	DBG("S3C6410 PM Resume (post-restore)\n");
	DBG(" S3C_RST_STAT = %x \n", __raw_readl(S3C_RST_STAT));
	DBG(" S3C_WAKEUP_STAT = %x \n", __raw_readl(S3C_WAKEUP_STAT));
	DBG(" S3C_BLK_PWR_STAT = %x \n", __raw_readl(S3C_BLK_PWR_STAT));
	DBG(" S3C_INFORM0 = %x \n", __raw_readl(S3C_INFORM0));
	DBG(" S3C_INFORM1 = %x \n", __raw_readl(S3C_INFORM1));
	DBG(" S3C_INFORM2 = %x \n", __raw_readl(S3C_INFORM2));
	return 0;
}

static struct platform_suspend_ops s3c6410_pm_ops = {
	.enter		= s3c6410_pm_enter,
	.valid		= suspend_valid_only_mem,
};

/* s3c6410_pm_init
 *
 * Attach the power management functions. This should be called
 * from the board specific initialisation if the board supports
 * it.
*/

int __init s3c6410_pm_init(void)
{
	printk("S3C6410 Power Management, (c) 2008 Samsung Electronics\n");

	suspend_set_ops(&s3c6410_pm_ops);

#ifdef CONFIG_S3C64XX_DOMAIN_GATING
	s3c_init_domain_power();
#endif /* CONFIG_S3C64XX_DOMAIN_GATING */
	/* set to zero value unused H/W IPs clock */
	__raw_writel(S3C_HCLK_MASK, S3C_HCLK_GATE);
	__raw_writel(S3C_SCLK_MASK, S3C_SCLK_GATE);
	__raw_writel(S3C_PCLK_MASK, S3C_PCLK_GATE);
	/* enable onenand0, onenand1 memory clock */
//	__raw_writel(0x18, S3C_MEM0_CLK_GATE);
	__raw_writel(0xffffffdb, S3C_MEM0_CLK_GATE); //ORIGINAL 0x18 WINMO ffffffdb
	return 0;
}
