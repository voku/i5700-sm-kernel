/*
 *  Copyright (C) 2004 Samsung Electronics
 *             SW.LEE <hitchcar@samsung.com>
 *            - based on Russell King : pcf8583.c
 * 	      - added  smdk24a0, smdk2440
 *            - added  poseidon (s3c24a0+wavecom)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *  Driver for FIMC2.x Camera Decoder
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/i2c-id.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/clk.h>

#include <mach/hardware.h>

#include <plat/gpio-cfg.h>
#include <plat/egpio.h>    

#include "../s3c_camif.h"

#include "s5ka3d.h"


// function define
//#define CONFIG_LOAD_FILE
#define I2C_BURST_MODE //dha23 100325 카메라 기동 시간 줄이기 위해 I2C Burst mode 사용.
// Purpose of verifying I2C operaion. must be ignored later.
//#define LOCAL_CONFIG_S5KA3D_I2C_TEST

static struct i2c_driver s5ka3d_driver;

static void s5ka3d_sensor_gpio_init(void);
void s5ka3d_sensor_enable(void);
static void s5ka3d_sensor_disable(void);

static int s5ka3d_sensor_init(void);
static void s5ka3d_sensor_exit(void);

static int s5ka3d_sensor_change_size(struct i2c_client *client, int size);

/*** functions included for Front camera ***************/

extern void s5k5ca_sensor_disable(void);
static int s5ka3d_sensor_write(struct i2c_client *, unsigned short subaddr, unsigned short val);
static long s5ka3d_set_sensor_mode(struct i2c_client *, int mode);
//static int s5ka3d_sensor_write_list(struct samsung_short_t *list,int size, char *name);

/*** functions included for Front camera ***************/

#ifdef CONFIG_FLASH_AAT1271A
	extern int aat1271a_flash_init(void);
	extern void aat1271a_flash_exit(void);
	extern void aat1271a_falsh_camera_control(int ctrl);
	extern void aat1271a_falsh_movie_control(int ctrl);
#endif

#ifdef CONFIG_LOAD_FILE
	static int s5ka3d_regs_table_write(char *name);
#endif

/* 
 * MCLK: 24MHz, PCLK: 54MHz
 * 
 * In case of PCLK 54MHz
 *
 * Preview Mode (1024 * 768)  
 * 
 * Capture Mode (2048 * 1536)
 * 
 * Camcorder Mode
 */
static camif_cis_t s5ka3d_data = {
	itu_fmt:       	CAMIF_ITU601,
	order422:      	CAMIF_CRYCBY,
	camclk:        	24000000,		
	source_x:      	640,		
	source_y:      	480,
	win_hor_ofst:  	0,
	win_ver_ofst:  	0,
	win_hor_ofst2: 	0,
	win_ver_ofst2: 	0,
	polarity_pclk: 	0,
	polarity_vsync:	1,
	polarity_href: 	0,
	reset_type:		CAMIF_RESET,
	reset_udelay: 	5000,
};


#define S5KA3D_ID 0xC4

static unsigned short s5ka3d_normal_i2c[] = { (S5KA3D_ID >> 1), I2C_CLIENT_END };
static unsigned short s5ka3d_ignore[] = { I2C_CLIENT_END };
static unsigned short s5ka3d_probe[] = { I2C_CLIENT_END };

static int previous_scene_mode = -1;
static int previous_WB_mode = 0;
static int af_mode = -1;
static unsigned short lux_value = 0;

static unsigned short AFPosition = 0x00FF; 
static unsigned short DummyAFPosition = 0x00FE;
bool isS5ka3dEnabled = false;


static struct i2c_client_address_data s5ka3d_addr_data = {
	.normal_i2c = s5ka3d_normal_i2c,
	.ignore		= s5ka3d_ignore,
	.probe		= s5ka3d_probe,
};

/**** LNT new added on 11062010**********/

static int s5ka3d_sensor_write2(struct i2c_client *client, unsigned short subaddr, unsigned short val)
{
	unsigned char buf[4];
	struct i2c_msg msg = { 0x3C, 0, 4, buf };
	buf[0] = (subaddr >> 8);
	buf[1] = (subaddr & 0xFF);
	buf[2] = (val >> 8);
	buf[3] = (val & 0xFF);

        if(i2c_transfer(client->adapter, &msg, 1) == 1)
        {
            return 0;
        }
        else
        {
            printk("[S5K4CA] s5k4ca_sensor_write fail \n");        
            return -EIO;
        }
//	return i2c_transfer(s5k4ca_client->adapter, &msg, 1) == 1 ? 0 : -EIO;
}

/***** end of new function *************/


static int s5ka3d_sensor_read(struct i2c_client *client,
		unsigned short subaddr, unsigned short *data)
{
	int ret;
	unsigned char buf[2];
	struct i2c_msg msg = { client->addr, 0, 2, buf };
	
	buf[0] = (subaddr >> 8);
	buf[1] = (subaddr & 0xFF);

	ret = i2c_transfer(client->adapter, &msg, 1) == 1 ? 0 : -EIO;
	if (ret == -EIO) 
		goto error;

	msg.flags = I2C_M_RD;
	
	ret = i2c_transfer(client->adapter, &msg, 1) == 1 ? 0 : -EIO;
	if (ret == -EIO) 
		goto error;

	*data = ((buf[0] << 8) | buf[1]);

error:
	return ret;
}
/*
static int s5ka3d_sensor_write(struct i2c_client *client,unsigned short subaddr, unsigned short val)
{
	if(subaddr == 0xdddd)
	{
			msleep(val);
			printk("delay time(%d msec)\n", val);
	}	
	else
	{					
		unsigned char buf[4];
		struct i2c_msg msg = { client->addr, 0, 4, buf };

		buf[0] = (subaddr >> 8);
		buf[1] = (subaddr & 0xFF);
		buf[2] = (val >> 8);
		buf[3] = (val & 0xFF);

//		return i2c_transfer(client->adapter, &msg, 1) == 1 ? 0 : -EIO;
                if(i2c_transfer(client->adapter, &msg, 1) == 1)
                    return 0;
                else{
                    printk("[**ASWOOGI**] s5ka3d_sensor_write_list error\n"); 
                    return -EIO;
                }
	}
}
*/
static int s5ka3d_sensor_write_list(struct i2c_client *client, struct samsung_short_t *list, char *name)
{
	int i, ret;
	ret = 0;

      // 	printk("s5ka3d_sensor_write_list( %s ) \n", name); 
    
#ifdef CONFIG_LOAD_FILE 
	s5ka3d_regs_table_write(name);	
#else
	for (i = 0; list[i].subaddr != 0xff; i++)
	{
        	//printk("s5ka3d_sensor_write_list  addr 0x%04x, value 0x%04x\n", list[i].subaddr, list[i].value);

		if(s5ka3d_sensor_write(client,list[i].subaddr, list[i].value) < 0){
                	printk("[**ASWOOGI**] s5ka3d_sensor_write_list error\n"); 
            			return -1;
                }
	}
#endif
	return ret;
}

#ifdef I2C_BURST_MODE //dha23 100325

#define BURST_MODE_SET			1
#define BURST_MODE_END			2
#define NORMAL_MODE_SET			3
#define MAX_INDEX				1000
static int s5ka3d_sensor_burst_write_list(struct i2c_client *client, struct samsung_short_t *list,char *name)
{
	__u8 temp_buf[MAX_INDEX];
	int index_overflow = 1;
	int new_addr_start = 0;
	int burst_mode = NORMAL_MODE_SET;
	unsigned short pre_subaddr = 0;
	struct i2c_msg msg = { client->addr, 0, 4, temp_buf };
	int i=0, ret=0;
	unsigned int index = 0;
	
	printk("s5ka3d_sensor_burst_write_list( %s ) \n", name); 
	printk("[PGH] on write func s5ka3d_client->addr : %x\n", client->addr);
    
#ifdef CONFIG_LOAD_FILE 
	s5ka3d_regs_table_write(name);	
#else
	for (i = 0; list[i].subaddr != 0xffff; i++)
	{
		if(list[i].subaddr == 0xdddd)
		{
		    /*
			if (list[i].value == 0x0010)
				msleep(10);
			else if (list[i].value == 0x0020)
				msleep(20);
			else if (list[i].value == 0x0030)
				msleep(30);
			else if (list[i].value == 0x0040)
				msleep(40);
			else if (list[i].value == 0x0050)
				msleep(50);
			else if (list[i].value == 0x0100)
				msleep(100);
		    */
		    msleep(list[i].value);
			printk("delay 0x%04x, value 0x%04x\n", list[i].subaddr, list[i].value);
		}	
		else
		{					
			if( list[i].subaddr == list[i+1].subaddr )
			{
				burst_mode = BURST_MODE_SET;
				if((list[i].subaddr != pre_subaddr) || (index_overflow == 1))
				{
					new_addr_start = 1;
					index_overflow = 0;
				}
			}
			else
			{
				if(burst_mode == BURST_MODE_SET)
				{
					burst_mode = BURST_MODE_END;
					if(index_overflow == 1)
					{
						new_addr_start = 1;
						index_overflow = 0;
					}
				}
				else
				{
					burst_mode = NORMAL_MODE_SET;
				}
			}

			if((burst_mode == BURST_MODE_SET) || (burst_mode == BURST_MODE_END))
			{
				if(new_addr_start == 1)
				{
					index = 0;
					memset(temp_buf, 0x00 ,1000);
					index_overflow = 0;

					temp_buf[index] = (list[i].subaddr >> 8);
					temp_buf[++index] = (list[i].subaddr & 0xFF);

					new_addr_start = 0;
				}
				
				temp_buf[++index] = (list[i].value >> 8);
				temp_buf[++index] = (list[i].value & 0xFF);
				
				if(burst_mode == BURST_MODE_END)
				{
					msg.len = ++index;

					ret = i2c_transfer(client->adapter, &msg, 1) == 1 ? 0 : -EIO;
					if( ret < 0)
					{
						printk("i2c_transfer fail ! \n");
						return -1;
					}
				}
				else if( index >= MAX_INDEX-1 )
				{
					index_overflow = 1;
					msg.len = ++index;
					
					ret = i2c_transfer(client->adapter, &msg, 1) == 1 ? 0 : -EIO;
					if( ret < 0)
					{
						printk("I2C_transfer Fail ! \n");
						return -1;
					}
				}
				
			}
			else
			{
				memset(temp_buf, 0x00 ,4);
			
				temp_buf[0] = (list[i].subaddr >> 8);
				temp_buf[1] = (list[i].subaddr & 0xFF);
				temp_buf[2] = (list[i].value >> 8);
				temp_buf[3] = (list[i].value & 0xFF);

				msg.len = 4;
				ret = i2c_transfer(client->adapter, &msg, 1) == 1 ? 0 : -EIO;
				if( ret < 0)
				{
					printk("I2C_transfer Fail ! \n");
					return -1;
				}
			}
		}
		
		pre_subaddr = list[i].subaddr;
	}
#endif
	return ret;
}

#endif


static int s5ka3d_sensor_ae_awb_lock(struct i2c_client *client)
{
}

static int s5ka3d_sensor_ae_awb_unlock(struct i2c_client *client)
{
}

static void s5ka3d_sensor_get_id(struct i2c_client *client)
{
	unsigned short id = 0;

	printk("[**ASWOOGI**] s5ka3d_sensor_get_id\n"); 
	
//	s5ka3d_sensor_write(client, 0x002C, 0x7000);
//	s5ka3d_sensor_write(client, 0x002E, 0x01FA);
//	s5ka3d_sensor_read(client, 0x0F12, &id);

	printk("Sensor ID(0x%04x) is %s!\n", id, (id == 0x4CA4) ? "Valid" : "Invalid"); 
}

static void s5ka3d_sensor_gpio_init(void)
{
        int test_value =0;
	printk("[**ASWOOGI**] s5ka3d_sensor_gpio_init\n"); 
    
	I2C_CAM_DIS;
	MCAM_RST_DIS;
        test_value = gpio_get_value(GPIO_MCAM_RST_N);
        printk("[PGH] 2 GPIO_MCAM_RST_N  : %d\n",test_value);	
    
	VCAM_RST_DIS;
        test_value = gpio_get_value(GPIO_SCAM_RST_N);
        printk("[PGH] 2 GPIO_SCAM_RST_N  : %d\n",test_value);	
    
	CAM_PWR_DIS;
        test_value = gpio_get_value(GPIO_CAM_EN);
        printk("[PGH] 2 GPIO_CAM_EN  : %d\n",test_value);	
    
	MCAM_STB_DIS;
        test_value = gpio_get_value(GPIO_CAM_STANDBY);
        printk("[PGH] 2 GPIO_CAM_STANDBY  : %d\n",test_value);	
    
	VCAM_STB_DIS;
        test_value = gpio_get_value(GPIO_SCAM_STANDBY);
        printk("[PGH] 2 GPIO_SCAM_STANDBY  : %d\n",test_value);	
    
}

#if defined(CONFIG_LDO_LP8720)
extern void	s5ka3d_sensor_power_init(void);	
#endif

void s5ka3d_sensor_enable(void)
{

        int test_value =0;
	printk("[**ASWOOGI**] s5ka3d_sensor_enable\n"); 

        //s5k5ca_sensor_disable();
	s5ka3d_sensor_gpio_init();

#if defined(CONFIG_LDO_LP8720)
	s5ka3d_sensor_power_init();	
#endif
	CAM_PWR_EN;
        test_value = gpio_get_value(GPIO_CAM_EN);
        printk("[PGH] 2 GPIO_CAM_EN  : %d\n",test_value);	

	msleep(1);

	VCAM_STB_EN;
        test_value = gpio_get_value(GPIO_SCAM_STANDBY);
        printk("[PGH] 2 GPIO_SCAM_STANDBY  : %d\n",test_value);	

    
	/* > 0 ms */
	msleep(1);

	/* MCLK Set */
	clk_set_rate(cam_clock, s5ka3d_data.camclk);

	/* MCLK Enable */
	clk_enable(cam_clock);
	clk_enable(cam_hclk);
	
	msleep(5);

	VCAM_RST_EN;
        test_value = gpio_get_value(GPIO_SCAM_RST_N);
        printk("[PGH] 2 GPIO_SCAM_RST_N  : %d\n",test_value);	


	msleep(1);

	I2C_CAM_EN;

	msleep(40);

	isS5ka3dEnabled = true;

 
}

static void s5ka3d_sensor_disable(void)
{
        printk("[**ASWOOGI**] s5ka3d_sensor_disable\n");

        if(isS5ka3dEnabled != true)
        {
            printk("[**ASWOOGI**] s5ka3d is not enabled.. return!!\n");
            return;
        }
        else
        {
            printk("[**ASWOOGI**] s5ka3d is enabled!!\n");
        }
        
        
	I2C_CAM_DIS;
	//VCAM_STB_DIS;
	//msleep(1);
	//MCAM_STB_DIS;   


	/* > 0 ms */
	msleep(1);
	VCAM_RST_DIS;

	msleep(1);
	MCAM_RST_DIS;	     

	/* > 20 cycles */
	msleep(1);

	/* MCLK Disable */
	clk_disable(cam_clock);
	clk_disable(cam_hclk);

	msleep(1);

	VCAM_STB_DIS;

	/* > 0 ms */
	msleep(1);

	AF_PWR_DIS;

	CAM_PWR_DIS;

	 isS5ka3dEnabled = false;
}

static int sensor_init(struct i2c_client *client)
{
	return 0;
}

/*

static int s5ka3d_sensor_write_list(struct samsung_short_t *list,int size, char *name)
{

        int ret = 0;
        int i;


        for (i = 0; i < size; i++)
        {
                if(s5ka3d_sensor_write(list[i].subaddr, list[i].value) < 0)
                {
                        printk("<=PCAM=> sensor_write_list fail...-_-\n");
                        return -1;
                }
        }
        return ret;
}
*/
static int s5ka3d_sensor_write(struct i2c_client *s5ka3d_client, unsigned short subaddr, unsigned short val)
{
        unsigned char buf[2] = {0};
        struct i2c_msg msg = { s5ka3d_client->addr, 0, 2, buf };

        buf[0] = subaddr;
        buf[1] = val;

//      return i2c_transfer(s5ka3d_client->adapter, &msg, 1) == 1 ? 0 : -EIO;
        if(i2c_transfer(s5ka3d_client->adapter, &msg, 1) == 1)
        {
            return 0;
        }
        else
        {
            printk("[s5ka3d] s5ka3d_sensor_write fail \n");
            return -EIO;
        }
}


static long s5ka3d_set_sensor_mode(struct i2c_client *client,int mode)
{
        //long rc = 0;

        printk("[CAM-SENSOR] =Sensor Mode ");
        switch (mode) {
        case SENSOR_PREVIEW:
                printk("[PGH]-> Preview \n");
                break;

        case SENSOR_CAPTURE:
                printk("[PGH}-> Capture \n");
                break;

	    case SENSOR_CAMCORDER:
			    printk("[PGH}-> Camcorder \n");
			    break;

	  // case SENSOR_RAW_SNAPSHOT_MODE:
         //       printk("[PGH}-> Capture RAW \n");
           //     break;

        default:
                return -EINVAL;
        }

        return 0;
}

 
static int sensor_init_probe(struct i2c_client *client)
{
        int rc = 0;

        printk("%s/%d \n", __func__, __LINE__);

        s5ka3d_sensor_write_list(client, reg_vt_init, "reg_vt_init");
        msleep(10);

        return rc;

}
static int s5ka3d_sensor_mode_set(struct i2c_client *client, int type)
{

                        
	return 0;
}

static int s5ka3d_sensor_change_size(struct i2c_client *client, int size)
{
	switch (size) {
		case SENSOR_XGA:
			s5ka3d_sensor_mode_set(client, SENSOR_PREVIEW);
			break;

		case SENSOR_QXGA:
			s5ka3d_sensor_mode_set(client, SENSOR_CAPTURE);
			break;		
	
		default:
			printk("Unknown Size! (Only XGA & QXGA)\n");
			break;
	}

	return 0;
}

static int s5ka3d_sensor_af_control(struct i2c_client *client, int type)
{
               
    return 0 ;//ret;
}

static int s5ka3d_sensor_change_effect(struct i2c_client *client, int type)
{
	int i, size;	
	
	printk("Effects Mode ");
#if 0
	switch (type)
	{
		case 0:
		default:
			printk("-> Mode None\n");
			s5ka3d_sensor_write_list(client,s5ka3d_effect_off_04,"s5ka3d_effect_off_04");
		break;

		case 1:
			printk("-> Mode Gray\n");
			s5ka3d_sensor_write_list(client,s5ka3d_effect_gray_04,"s5ka3d_effect_gray_04");
		break;

		case 2:
			printk("-> Mode Sepia\n");
			s5ka3d_sensor_write_list(client,s5ka3d_effect_sepia_04,"s5ka3d_effect_sepia_04");
		break;

		case 3:
			printk("-> Mode Negative\n");
			s5ka3d_sensor_write_list(client,s5ka3d_effect_negative_04,"s5ka3d_effect_negative_04");
		break;
		
		case 4:
			printk("-> Mode Aqua\n");
			s5ka3d_sensor_write_list(client,s5ka3d_effect_aqua_04,"s5ka3d_effect_aqua_04");
		break;

		case 5:
			printk("-> Mode Sketch\n");
			s5ka3d_sensor_write_list(client,s5ka3d_effect_sketch_04,"s5ka3d_effect_sketch_04");
		break;
	}
#endif
	return 0;

}

static int s5ka3d_sensor_change_br(struct i2c_client *client, int type)
{
	int i, size;

	printk("Brightness Mode \n");
#if 0
	switch (type)
	{
		case 0: 
		default :
			printk("-> Brightness Minus 4\n");
			s5ka3d_sensor_write_list(client,s5ka3d_br_minus4_04,"s5ka3d_br_minus4_04");
		break;

		case 1:
			printk("-> Brightness Minus 3\n");	
			s5ka3d_sensor_write_list(client,s5ka3d_br_minus3_04,"s5ka3d_br_minus3_04");
		break;
		
		case 2:
			printk("-> Brightness Minus 2\n");
			s5ka3d_sensor_write_list(client,s5ka3d_br_minus2_04,"s5ka3d_br_minus2_04");
		break;
		
		case 3:				
			printk("-> Brightness Minus 1\n");
			s5ka3d_sensor_write_list(client,s5ka3d_br_minus1_04,"s5ka3d_br_minus1_04");
		break;
		
		case 4:
			printk("-> Brightness Zero\n");
			s5ka3d_sensor_write_list(client,s5ka3d_br_zero_04,"s5ka3d_br_zero_04");
		break;

		case 5:
			printk("-> Brightness Plus 1\n");
			s5ka3d_sensor_write_list(client,s5ka3d_br_plus1_04,"s5ka3d_br_plus1_04");
		break;

		case 6:
			printk("-> Brightness Plus 2\n");
			s5ka3d_sensor_write_list(client,s5ka3d_br_plus2_04,"s5ka3d_br_plus2_04");
		break;

		case 7:
			printk("-> Brightness Plus 3\n");
			s5ka3d_sensor_write_list(client,s5ka3d_br_plus3_04,"s5ka3d_br_plus3_04");
		break;

		case 8:
			printk("-> Brightness Plus 4\n");
			s5ka3d_sensor_write_list(client,s5ka3d_br_plus4_04,"s5ka3d_br_plus4_04");
		break;
		
	}
#endif
	return 0;	
}

static int s5ka3d_sensor_change_wb(struct i2c_client *client, int type)
{
	int i, size;
	
	printk("White Balance Mode ");
#if 0
	switch (type)
	{
		case 0:
		default :
			printk("-> WB auto mode\n");
			s5ka3d_sensor_write_list(client,s5ka3d_wb_auto_04,"s5ka3d_wb_auto_04");
			previous_WB_mode = type;
		break;
		
		case 1:
			printk("-> WB Sunny mode\n");
			s5ka3d_sensor_write_list(client,s5ka3d_wb_sunny_04,"s5ka3d_wb_sunny_04");
			previous_WB_mode = type;
		break;

		case 2:
			printk("-> WB Cloudy mode\n");
			s5ka3d_sensor_write_list(client,s5ka3d_wb_cloudy_04,"s5ka3d_wb_cloudy_04");
			previous_WB_mode = type;
		break;

		case 3:
			printk("-> WB Flourescent mode\n");
			s5ka3d_sensor_write_list(client,s5ka3d_wb_fluorescent_04,"s5ka3d_wb_fluorescent_04");
			previous_WB_mode = type;
		break;

		case 4:
			printk("-> WB Tungsten mode\n");
			s5ka3d_sensor_write_list(client,s5ka3d_wb_tungsten_04,"s5ka3d_wb_tungsten_04");
			previous_WB_mode = type;
		break;
	}
#endif    
	return 0;
}

static int s5ka3d_sensor_change_scene_mode(struct i2c_client *client, int type)
{
	int i, size;
#if 0
	printk("[CAM-SENSOR] =Scene Mode %d",type);
	if(previous_scene_mode != 0 && type != 0)
	{
		printk("-> Pre-auto-set");
		s5ka3d_sensor_write_list(client,s5ka3d_scene_mode_off_04,"s5ka3d_scene_mode_off_04");
	}

	switch (type)
	{
		case 0:
			printk("-> auto\n");
			s5ka3d_sensor_write_list(client,s5ka3d_scene_mode_off_04,"s5ka3d_scene_mode_off_04");
			previous_scene_mode = type;
			break;
		case 1:
			printk("-> portrait\n");
			s5ka3d_sensor_write_list(client,s5ka3d_scene_portrait_on_04,"s5ka3d_scene_portrait_on_04");
			previous_scene_mode = type;
			break;
		case 2:
			printk("-> landscape\n");
			s5ka3d_sensor_write_list(client,s5ka3d_scene_landscape_on_04,"s5ka3d_scene_landscape_on_04");
			previous_scene_mode = type;
			break;
		case 3:
			printk("-> night\n");
			s5ka3d_sensor_write_list(client,s5ka3d_nightmode_on_04,"s5ka3d_nightmode_on_04");
			previous_scene_mode = type;
			break;
		case 4:
			printk("-> sunset\n");
			s5ka3d_sensor_write_list(client,s5ka3d_scene_sunset_on_04,"s5ka3d_scene_sunset_on_04");
			previous_scene_mode = type;
			break;
		case 5:
			printk("-> sports\n");
			s5ka3d_sensor_write_list(client,s5ka3d_scene_sports_on_04,"s5ka3d_scene_sports_on_04");
			previous_scene_mode = type;
			break;
		case 6:
			printk("-> fireworks\n");
			s5ka3d_sensor_write_list(client,s5ka3d_scene_fireworks_on_04,"s5ka3d_scene_fireworks_on_04");
			previous_scene_mode = type;
			break;
		case 7:
			printk("-> candlelight\n");
			s5ka3d_sensor_write_list(client,s5ka3d_scene_candlelight_on_04,"s5ka3d_scene_candlelight_on_04");
			previous_scene_mode = type;
			break;
		case 8:
			printk("-> text\n");
			s5ka3d_sensor_write_list(client,s5ka3d_scene_text_on_04,"s5ka3d_scene_text_on_04");
			previous_scene_mode = type;
			break;
		case 9:
			printk("-> beach snow\n");
			s5ka3d_sensor_write_list(client,s5ka3d_scene_beach_snow_on_04,"s5ka3d_scene_beach_snow_on_04");
			previous_scene_mode = type;
			break;
		case 10:
			printk("-> party indoor\n");
			s5ka3d_sensor_write_list(client,s5ka3d_scene_party_indoor_on_04,"s5ka3d_scene_party_indoor_on_04");
			previous_scene_mode = type;
			break;
		default :
			printk("-> UnKnow Scene Mode\n");
			break;
	}

	previous_scene_mode = type;
#endif
	return 0;
}

static int s5ka3d_sensor_photometry(struct i2c_client *client, int type)
{
	int i, size;	
	
	printk("Metering Mode ");

	switch (type)
	{
		case 0:
			printk("-> spot\n");
//			s5ka3d_sensor_write_list(client,s5ka3d_metering_spot,"s5ka3d_metering_spot");
			break;
		case 1:
			printk("-> matrix\n");
//			s5ka3d_sensor_write_list(client,s5ka3d_metering_matrix,"s5ka3d_metering_matrix");
			break;
		case 2:
			printk("-> center\n");
//			s5ka3d_sensor_write_list(client,s5ka3d_metering_center,"s5ka3d_metering_center");
			break;
		default :
			printk("-> UnKnown Metering Mode\n");
			break;
	
	}
	return 0;	

}


static int s5ka3d_sensor_command(struct i2c_client *client, unsigned int cmd, void *arg)
{
	struct v4l2_control *ctrl;
	unsigned short *w_data;		/* To support user level i2c */	
	s5ka3d_short_t *r_data;

	int ret=0;

	switch (cmd)
	{
		case SENSOR_INIT:
			//ret = sensor_init(client);
                        ret = sensor_init_probe(client);
			break;

		case USER_ADD:
			break;

		case USER_EXIT:
			s5ka3d_sensor_exit();
			break;

		case SENSOR_EFFECT:
//			ctrl = (struct v4l2_control *)arg;
//			s5ka3d_sensor_change_effect(client, ctrl->value);
			break;

		case SENSOR_BRIGHTNESS:
//			ctrl = (struct v4l2_control *)arg;
//			s5ka3d_sensor_change_br(client, ctrl->value);
			break;

		case SENSOR_WB:
//			ctrl = (struct v4l2_control *)arg;
//			s5ka3d_sensor_change_wb(client, ctrl->value);
			break;

		case SENSOR_SCENE_MODE:
//			ctrl = (struct v4l2_control *)arg;
//			s5ka3d_sensor_change_scene_mode(client, ctrl->value);
			break;

		case SENSOR_AF:
//			ctrl = (struct v4l2_control *)arg;
//			ret = s5ka3d_sensor_af_control(client, ctrl->value);
			break;

		case SENSOR_MODE_SET:
		        ctrl = (struct v4l2_control *)arg;
                      ret =  s5ka3d_set_sensor_mode(client,ctrl->value);
		//	s5ka3d_sensor_mode_set(client, ctrl->value);
			break;

		case SENSOR_XGA:
//			s5ka3d_sensor_change_size(client, SENSOR_XGA);	
			break;

		case SENSOR_QXGA:
//			s5ka3d_sensor_change_size(client, SENSOR_QXGA);	
			break;

		case SENSOR_QSVGA:
//			s5ka3d_sensor_change_size(client, SENSOR_QSVGA);
			break;

		case SENSOR_VGA:
//			s5ka3d_sensor_change_size(client, SENSOR_VGA);
			break;

		case SENSOR_SVGA:
//			s5ka3d_sensor_change_size(client, SENSOR_SVGA);
			break;

		case SENSOR_SXGA:
//			s5ka3d_sensor_change_size(client, SENSOR_SXGA);
			break;

		case SENSOR_UXGA:
//			s5ka3d_sensor_change_size(client, SENSOR_UXGA);
			break;

		case SENSOR_USER_WRITE:
//			w_data = (unsigned short *)arg;
//			s5ka3d_sensor_user_write(client, w_data);
			break;

		case SENSOR_USER_READ:
//			r_data = (s5ka3d_short_t *)arg;
//			s5ka3d_sensor_user_read(client, r_data);
			break;
	
		case SENSOR_FLASH_CAMERA:
			ctrl = (struct v4l2_control *)arg;
#ifdef CONFIG_FLASH_AAT1271A
			aat1271a_falsh_camera_control(ctrl->value);	
#endif			
			break;

		case SENSOR_FLASH_MOVIE:
			ctrl = (struct v4l2_control *)arg;
#ifdef CONFIG_FLASH_AAT1271A
			aat1271a_falsh_movie_control(ctrl->value);	
#endif
			break;

		case SENSOR_EXIF_DATA:
//			exif_data = (exif_data_t *)arg;
//			s5ka3d_sensor_exif_read(client, exif_data);	
			break;

		case SENSOR_PHOTOMETRY:
//		    ctrl = (struct v4l2_control *)arg;
//			s5ka3d_sensor_photometry(client, ctrl->value);
			break;
		

		default:
		    printk("[CAM-SENSOR] no command type %d \n",cmd);
			break;
	}

	return ret;
}

static int s5ka3d_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret,err=0;

	if ( !i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA) ) {
		printk(KERN_INFO "byte op is not permited.\n");
		return err;
	}

	client->addr = S5KA3D_ID >> 1;
	client->driver = &s5ka3d_driver;

	s5ka3d_data.sensor = client;

	printk("[CAM-SENSOR][s5ka3d] probed!\n");
    
	return ret;

}

static int s5ka3d_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info)
{
	strlcpy(info->type, "s5ka3d", I2C_NAME_SIZE);
	return 0;
}


static int __devexit s5ka3d_i2c_remove(struct i2c_client *client)
{
    printk("[CAM-SENSOR][s5ka3d] removed\n");
	return 0;
}

static struct i2c_device_id s5ka3d_id[] = {
	{ "s5ka3d", 0 },
	{}
};

MODULE_DEVICE_TABLE(i2c, s5ka3d_id);

static struct i2c_driver s5ka3d_driver = {
	.driver = {
		.name = "s5ka3d",
	},
	.class 		= I2C_CLASS_HWMON,
	.probe		= s5ka3d_i2c_probe,
	.remove		= __devexit_p(s5ka3d_i2c_remove),
	.detect		= s5ka3d_i2c_detect,
	.command = s5ka3d_sensor_command,
	.id_table	= s5ka3d_id,
	.address_data	= &s5ka3d_addr_data,
};



#ifdef CONFIG_LOAD_FILE

#include <linux/vmalloc.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/slab.h>

#include <asm/uaccess.h>

static char *s5ka3d_regs_table = NULL;

static int s5ka3d_regs_table_size;

void s5ka3d_regs_table_init(void)
{
	struct file *filp;
	char *dp;
	long l;
	loff_t pos;
	int i;
	int ret;
	mm_segment_t fs = get_fs();

	printk("%s %d\n", __func__, __LINE__);

	set_fs(get_ds());
#if 0
	filp = filp_open("/data/camera/s5ka3d.h", O_RDONLY, 0);
#else
	filp = filp_open("/sdcard/s5ka3d.h", O_RDONLY, 0);
#endif
	if (IS_ERR(filp)) {
		printk("file open error\n");
		return;
	}
	l = filp->f_path.dentry->d_inode->i_size;	
	printk("l = %ld\n", l);
	dp = kmalloc(l, GFP_KERNEL);
	if (dp == NULL) {
		printk("Out of Memory\n");
		filp_close(filp, current->files);
	}
	pos = 0;
	memset(dp, 0, l);
	ret = vfs_read(filp, (char __user *)dp, l, &pos);
	if (ret != l) {
		printk("Failed to read file ret = %d\n", ret);
		kfree(dp);
		filp_close(filp, current->files);
		return;
	}

	filp_close(filp, current->files);
	
	set_fs(fs);

	s5ka3d_regs_table = dp;
	
	s5ka3d_regs_table_size = l;

	*((s5ka3d_regs_table + s5ka3d_regs_table_size) - 1) = '\0';

	printk("s5ka3d_regs_table 0x%08x, %ld\n", dp, l);
}

void s5ka3d_regs_table_exit(void)
{
	printk("%s %d\n", __func__, __LINE__);
	if (s5ka3d_regs_table) {
		kfree(s5ka3d_regs_table);
		s5ka3d_regs_table = NULL;
	}	
}

static int s5ka3d_regs_table_write(char *name)
{
	char *start, *end, *reg, *data;	
	unsigned short addr, value;
	char reg_buf[7], data_buf[7];

	*(reg_buf + 6) = '\0';
	*(data_buf + 6) = '\0';

	start = strstr(s5ka3d_regs_table, name);
	
	end = strstr(start, "};");

	while (1) {	
		/* Find Address */	
		reg = strstr(start,"{ 0x");		
		if (reg)
			start = (reg + 16);
		if ((reg == NULL) || (reg > end))
			break;
		/* Write Value to Address */	
		if (reg != NULL) {
			memcpy(reg_buf, (reg + 2), 6);	
			memcpy(data_buf, (reg + 10), 6);	
			addr = (unsigned short)simple_strtoul(reg_buf, NULL, 16); 
			value = (unsigned short)simple_strtoul(data_buf, NULL, 16); 
			printk("addr 0x%04x, value 0x%04x\n", addr, value);
/*
			if (addr == 0xdddd)
			{
			    if (value == 0x0010)
					mdelay(10);
				else if (value == 0x0020)
					mdelay(20);
				else if (value == 0x0030)
					mdelay(30);
				else if (value == 0x0040)
					mdelay(40);
				else if (value == 0x0050)
					mdelay(50);
				else if (value == 0x0100)    
					mdelay(100);
	
				mdelay(value);

				printk("delay 0x%04x, value 0x%04x\n", addr, value);
			}	
			else
*/			
				s5ka3d_sensor_write(s5ka3d_data.sensor, addr, value);
		}
	}

	return 0;
}

#endif

static int s5ka3d_sensor_init(void)
{
	int ret;

      printk("\n >>>>>>>>>>>>>> %s <<<<<<<<<<<< \n",__func__);

#ifdef CONFIG_LOAD_FILE
	s5ka3d_regs_table_init();
#endif

	s5ka3d_sensor_enable();
	
	s3c_camif_open_sensor(&s5ka3d_data);

	if (s5ka3d_data.sensor == NULL)
		if ((ret = i2c_add_driver(&s5ka3d_driver)))
			return ret;

	if (s5ka3d_data.sensor == NULL) {
		i2c_del_driver(&s5ka3d_driver);	
		return -ENODEV;
	}

	s3c_camif_register_sensor(&s5ka3d_data);
	
	return 0;
}

static void s5ka3d_sensor_exit(void)
{
	s5ka3d_sensor_disable();

#ifdef CONFIG_LOAD_FILE
	s5ka3d_regs_table_exit();
#endif
	
	if (s5ka3d_data.sensor != NULL)
		s3c_camif_unregister_sensor(&s5ka3d_data);
}

static struct v4l2_input s5ka3d_input = {
	.index		= 0,
	.name		= "Camera Input (S5KA3D)",
	.type		= V4L2_INPUT_TYPE_CAMERA,
	.audioset	= 1,
	.tuner		= 0,
	.std		= V4L2_STD_PAL_BG | V4L2_STD_NTSC_M,
	.status		= 0,
};

static struct v4l2_input_handler s5ka3d_input_handler = {
	s5ka3d_sensor_init,
	s5ka3d_sensor_exit	
};

#ifdef CONFIG_VIDEO_SAMSUNG
static int s5ka3d_sensor_add(void)
{
#ifdef CONFIG_FLASH_AAT1271A
	aat1271a_flash_init();
#endif

        printk(" \n >>>>>>>> %s <<<<<<<<< \n",__func__);
	return s3c_camif_add_sensor(&s5ka3d_input, &s5ka3d_input_handler);
}

static void s5ka3d_sensor_remove(void)
{
	if (s5ka3d_data.sensor != NULL)
		i2c_del_driver(&s5ka3d_driver);
#ifdef CONFIG_FLASH_AAT1271A
	aat1271a_flash_exit();
#endif
	s3c_camif_remove_sensor(&s5ka3d_input, &s5ka3d_input_handler);
}

module_init(s5ka3d_sensor_add)
module_exit(s5ka3d_sensor_remove)

MODULE_AUTHOR("Jinsung, Yang <jsgood.yang@samsung.com>");
MODULE_DESCRIPTION("I2C Client Driver For FIMC V4L2 Driver");
MODULE_LICENSE("GPL");
#else
int s5ka3d_sensor_add(void)
{

printk("\n >>>>>>>>>>>>> %s <<<<<<<<<<<<<<<< \n",__func__);

#ifdef CONFIG_FLASH_AAT1271A
	aat1271a_flash_init();
#endif	
#ifdef LOCAL_CONFIG_S5KA3D_I2C_TEST
	return s5ka3d_sensor_init();
#else
	return s3c_camif_add_sensor(&s5ka3d_input, &s5ka3d_input_handler);
#endif
}

void s5ka3d_sensor_remove(void)
{
	if (s5ka3d_data.sensor != NULL)
		i2c_del_driver(&s5ka3d_driver);
#ifdef CONFIG_FLASH_AAT1271A
	aat1271a_flash_exit();
#endif
	s3c_camif_remove_sensor(&s5ka3d_input, &s5ka3d_input_handler);
}
#endif
