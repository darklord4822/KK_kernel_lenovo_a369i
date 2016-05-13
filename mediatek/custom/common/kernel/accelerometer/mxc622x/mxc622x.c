#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>


#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>


#include <cust_acc.h>
#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#include "mxc622x.h"
#include <linux/hwmsen_helper.h>
/*----------------------------------------------------------------------------*/
#define I2C_DRIVERID_MXC6225 342
/*----------------------------------------------------------------------------*/
#define DEBUG 1
/*----------------------------------------------------------------------------*/
//#define CONFIG_MXC6225_LOWPASS   /*apply low pass filter on output*/
#define SW_CALIBRATION

/*----------------------------------------------------------------------------*/
#define MXC6225_AXIS_X          0
#define MXC6225_AXIS_Y          1
#define MXC6225_AXIS_Z          2
#define MXC6225_AXES_NUM        3
#define MXC6225_DATA_LEN        2
#define MXC6225_DEV_NAME        "MXC6225"
#define FTM_CUST_ACC "/data/mxc6225"

#define POWER_NONE_MACRO MT65XX_POWER_NONE

/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
static const struct i2c_device_id mxc6225_i2c_id[] = {{"MXC6225",0},{}};
static struct i2c_board_info __initdata i2c_mxc6225={ I2C_BOARD_INFO("MXC6225", (MXC6225_I2C_SLAVE_WRITE_ADDR>>1))};
/*the adapter id will be available in customization*/
//static unsigned short mxc6225_force[] = {0x00, MXC6225_I2C_SLAVE_WRITE_ADDR, I2C_CLIENT_END, I2C_CLIENT_END};
//static const unsigned short *const mxc6225_forces[] = { mxc6225_force, NULL };
//static struct i2c_client_address_data mxc6225_addr_data = { .forces = mxc6225_forces,};

/*----------------------------------------------------------------------------*/
static int mxc6225_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int mxc6225_i2c_remove(struct i2c_client *client);
static int mxc6225_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info);

#if defined(MTK_AUTO_DETECT_ACCELEROMETER)
static int  mxc6225_local_init(void);
static int  mxc6225_remove(void);

static int mxc6225_init_flag =0; // 0<==>OK -1 <==> fail
#endif

static volatile int suspend_lock=0; //add by libo 20130702

/*----------------------------------------------------------------------------*/
typedef enum {
    ADX_TRC_FILTER  = 0x01,
    ADX_TRC_RAWDATA = 0x02,
    ADX_TRC_IOCTL   = 0x04,
    ADX_TRC_CALI	= 0X08,
    ADX_TRC_INFO	= 0X10,
} ADX_TRC;
/*----------------------------------------------------------------------------*/
struct scale_factor{
    u8  whole;
    u8  fraction;
};
/*----------------------------------------------------------------------------*/
struct data_resolution {
    struct scale_factor scalefactor;
    int                 sensitivity;
};
/*----------------------------------------------------------------------------*/
#define C_MAX_FIR_LENGTH (32)
/*----------------------------------------------------------------------------*/
struct data_filter {
    s16 raw[C_MAX_FIR_LENGTH][MXC6225_AXES_NUM];
    int sum[MXC6225_AXES_NUM];
    int num;
    int idx;
};

#if defined(MTK_AUTO_DETECT_ACCELEROMETER)
static struct sensor_init_info mxc6225_init_info = {
            .name = "mxc6225",
                    .init = mxc6225_local_init,
                            .uninit = mxc6225_remove,

                                  };
#endif

/*----------------------------------------------------------------------------*/
struct mxc6225_i2c_data {
    struct i2c_client *client;
    struct acc_hw *hw;
    struct hwmsen_convert   cvt;

    /*misc*/
    struct data_resolution *reso;
    atomic_t                trace;
    atomic_t                suspend;
    atomic_t                selftest;
    atomic_t				filter;
    s16                     cali_sw[MXC6225_AXES_NUM+1];

    /*data*/
    s8                      offset[MXC6225_AXES_NUM+1];  /*+1: for 4-byte alignment*/
    s16                     data[MXC6225_AXES_NUM+1];

#if defined(CONFIG_MXC6225_LOWPASS)
    atomic_t                firlen;
    atomic_t                fir_en;
    struct data_filter      fir;
#endif
    /*early suspend*/
#if defined(CONFIG_HAS_EARLYSUSPEND)
    struct early_suspend    early_drv;
#endif
};
/*----------------------------------------------------------------------------*/
static struct i2c_driver mxc6225_i2c_driver = {
            .driver = {
                          //        .owner          = THIS_MODULE,
                          .name           = "MXC6225",
                      },
                      .probe      		= mxc6225_i2c_probe,
                                      .remove    			= mxc6225_i2c_remove,
                                                      .detect				= mxc6225_i2c_detect,
#if !defined(CONFIG_HAS_EARLYSUSPEND)
                                                                   .suspend            = mxc6225_suspend,
                                                                                         .resume             = mxc6225_resume,
#endif
                                                                                                               .id_table = mxc6225_i2c_id,
                                                                                                                           //	.address_data = &mxc6225_addr_data,
                                                                                                                       };

/*----------------------------------------------------------------------------*/
static struct i2c_client *mxc6225_i2c_client = NULL;
#if !defined(MTK_AUTO_DETECT_ACCELEROMETER)
static struct platform_driver mxc6225_gsensor_driver;
#endif
static struct mxc6225_i2c_data *obj_i2c_data = NULL;
static bool sensor_power = true;
static GSENSOR_VECTOR3D gsensor_gain;
static char selftestRes[8]= {0};

//add by huangrunming 20120907
static int auto_now_data[MXC6225_AXES_NUM] = {0};

/*----------------------------------------------------------------------------*/
#define GSE_TAG                  "[Gsensor] "
#define GSE_FUN(f)               printk(KERN_INFO GSE_TAG"%s\n", __FUNCTION__)
#define GSE_ERR(fmt, args...)    printk(KERN_ERR GSE_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define GSE_LOG(fmt, args...)    printk(KERN_INFO GSE_TAG fmt, ##args)
/*----------------------------------------------------------------------------*/
static struct data_resolution mxc6225_data_resolution[] = {
            /*8 combination by {FULL_RES,RANGE}*/
            {{ 3, 9}, 256},   /*+/-2g  in 10-bit resolution:  3.9 mg/LSB*/
            {{ 7, 8}, 128},   /*+/-4g  in 10-bit resolution:  7.8 mg/LSB*/
            {{15, 6},  64},   /*+/-8g  in 10-bit resolution: 15.6 mg/LSB*/
            {{31, 2},  32},   /*+/-16g in 10-bit resolution: 31.2 mg/LSB*/
            {{ 3, 9}, 256},   /*+/-2g  in 10-bit resolution:  3.9 mg/LSB (full-resolution)*/
            {{ 3, 9}, 256},   /*+/-4g  in 11-bit resolution:  3.9 mg/LSB (full-resolution)*/
            {{ 3, 9}, 256},   /*+/-8g  in 12-bit resolution:  3.9 mg/LSB (full-resolution)*/
            {{ 3, 9}, 256},   /*+/-16g in 13-bit resolution:  3.9 mg/LSB (full-resolution)*/
        };
/*----------------------------------------------------------------------------*/
static struct data_resolution mxc6225_offset_resolution = {{15, 6}, 64};

static int MXC6225_SetPowerMode(struct i2c_client *client, bool enable);

////////////
/*----------------------------------------------------------------------------*/
/*
static int hwmsen_read_byte_sr(struct i2c_client *client, u8 addr, u8 *data)
{
   u8 buf;
    int ret = 0;

    client->addr = (client->addr & I2C_MASK_FLAG) | I2C_WR_FLAG |I2C_RS_FLAG;
    buf = addr;
	ret = i2c_master_send(client, (const char*)&buf, 1<<8 | 1);
    //ret = i2c_master_send(client, (const char*)&buf, 1);
    if (ret < 0) {
        printk("send command error!!\n");
        return -EFAULT;
    }

    *data = buf;
	client->addr = client->addr& I2C_MASK_FLAG;
    return 0;
}
*/
static int hwmsen_read_byte_sr(struct i2c_client *client, u8 addr, u8 *data)
{
    u8 buf;
    int ret = 0;
    int err = 0;//uint8_t loop_i;


    /* Caller should check parameter validity.*/
    if (data == NULL)
    {
        return -EINVAL;
    }

    if (err = hwmsen_read_block(client, addr, data, 1))
    {
        printk("MYCAT hwmsen_read_block error : %d", err);
    }


    return 0;
}


/*--------------------MXC6225 power control function----------------------------------*/
static void MXC6225_power(struct acc_hw *hw, unsigned int on)
{
    static unsigned int power_on = 0;

    if (hw->power_id != POWER_NONE_MACRO)		// have externel LDO
    {
        GSE_LOG("power %s\n", on ? "on" : "off");
        if (power_on == on)	// power status not change
        {
            GSE_LOG("ignore power control: %d\n", on);
        }
        else if (on)	// power on
        {
            if (!hwPowerOn(hw->power_id, hw->power_vol, "MXC6225"))
            {
                GSE_ERR("power on fails!!\n");
            }
        }
        else	// power off
        {
            if (!hwPowerDown(hw->power_id, "MXC6225"))
            {
                GSE_ERR("power off fail!!\n");
            }
        }
    }
    power_on = on;
}
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
static int MXC6225_SetDataResolution(struct mxc6225_i2c_data *obj)
{

    /*set g sensor dataresolution here*/

    /*MXC6225 only can set to 10-bit dataresolution, so do nothing in mxc6225 driver here*/

    /*end of set dataresolution*/



    /*we set measure range from -2g to +2g in MXC6225_SetDataFormat(client, MXC6225_RANGE_2G),
                                                       and set 10-bit dataresolution MXC6225_SetDataResolution()*/

    /*so mxc6225_data_resolution[0] set value as {{ 3, 9}, 256} when declaration, and assign the value to obj->reso here*/

    obj->reso = &mxc6225_data_resolution[2];// 0
    return 0;

    /*if you changed the measure range, for example call: MXC6225_SetDataFormat(client, MXC6225_RANGE_4G),
    you must set the right value to mxc6225_data_resolution*/

}
/*----------------------------------------------------------------------------*/
static int MXC6225_ReadData(struct i2c_client *client, s16 data[MXC6225_AXES_NUM])
{
    struct mxc6225_i2c_data *priv = i2c_get_clientdata(client);
    char addr = MXC6225_REG_DATAX0;
    char buf[MXC6225_DATA_LEN] = {0};
    int err = 0;

    if (NULL == client)
    {
        err = -EINVAL;
    }
    else if (err = hwmsen_read_block(client, addr, buf, MXC6225_DATA_LEN))
    {
        GSE_ERR("error: %d\n", err);
    }
    else
    {
        data[MXC6225_AXIS_X] = (signed char)(buf[0]);
        data[MXC6225_AXIS_Y] = (signed char)(buf[1]);
        //data[MXC6225_AXIS_Y] = (signed char)(buf[0]);
        //data[MXC6225_AXIS_X] = -((signed char)(buf[1]));
        data[MXC6225_AXIS_Z] = 0;
        //printk("[ ACC MXC6225 ] : data[0] = %d, data[1] = %d\n", buf[0], buf[1]);
    }

    return err;
}
/*----------------------------------------------------------------------------*/
static int MXC6225_ReadOffset(struct i2c_client *client, s8 ofs[MXC6225_AXES_NUM])
{
    int err;
#ifdef SW_CALIBRATION
    ofs[0]=ofs[1]=ofs[2]=0x0;
#else
    if (err = hwmsen_read_block(client, MXC6225_REG_OFSX, ofs, MXC6225_AXES_NUM))
    {
        GSE_ERR("error: %d\n", err);
    }
#endif
    //printk("offesx=%x, y=%x, z=%x",ofs[0],ofs[1],ofs[2]);

    return err;
}
/*----------------------------------------------------------------------------*/
static int MXC6225_ResetCalibration(struct i2c_client *client)
{/*
        	struct mxc6225_i2c_data *obj = i2c_get_clientdata(client);
        	u8 ofs[4]={0,0,0,0};
        	int err;

        	#ifdef SW_CALIBRATION

        	#else
        		if(err = hwmsen_write_block(client, MXC6225_REG_OFSX, ofs, 4))
        		{
        			GSE_ERR("error: %d\n", err);
        		}
        	#endif

        	memset(obj->cali_sw, 0x00, sizeof(obj->cali_sw));
        	memset(obj->offset, 0x00, sizeof(obj->offset));
        	return err;    */

    return 0;
}
/*----------------------------------------------------------------------------*/
static int MXC6225_ReadCalibration(struct i2c_client *client, int dat[MXC6225_AXES_NUM])
{
    struct mxc6225_i2c_data *obj = i2c_get_clientdata(client);
    int err;
    int mul;

#ifdef SW_CALIBRATION
    mul = 0;//only SW Calibration, disable HW Calibration
#else
    if ((err = MXC6225_ReadOffset(client, obj->offset))) {
        GSE_ERR("read offset fail, %d\n", err);
        return err;
    }
    mul = obj->reso->sensitivity/mxc6225_offset_resolution.sensitivity;
#endif

    dat[obj->cvt.map[MXC6225_AXIS_X]] = obj->cvt.sign[MXC6225_AXIS_X]*(obj->offset[MXC6225_AXIS_X]*mul + obj->cali_sw[MXC6225_AXIS_X]);
    dat[obj->cvt.map[MXC6225_AXIS_Y]] = obj->cvt.sign[MXC6225_AXIS_Y]*(obj->offset[MXC6225_AXIS_Y]*mul + obj->cali_sw[MXC6225_AXIS_Y]);
    dat[obj->cvt.map[MXC6225_AXIS_Z]] = obj->cvt.sign[MXC6225_AXIS_Z]*(obj->offset[MXC6225_AXIS_Z]*mul + obj->cali_sw[MXC6225_AXIS_Z]);

    return 0;
}
/*----------------------------------------------------------------------------*/
static int MXC6225_ReadCalibrationEx(struct i2c_client *client, int act[MXC6225_AXES_NUM], int raw[MXC6225_AXES_NUM])
{
    /*raw: the raw calibration data; act: the actual calibration data*/
    struct mxc6225_i2c_data *obj = i2c_get_clientdata(client);
    int err;
    int mul;



#ifdef SW_CALIBRATION
    mul = 0;//only SW Calibration, disable HW Calibration
#else
    if (err = MXC6225_ReadOffset(client, obj->offset))
    {
        GSE_ERR("read offset fail, %d\n", err);
        return err;
    }
    mul = obj->reso->sensitivity/mxc6225_offset_resolution.sensitivity;
#endif

    raw[MXC6225_AXIS_X] = obj->offset[MXC6225_AXIS_X]*mul + obj->cali_sw[MXC6225_AXIS_X];
    raw[MXC6225_AXIS_Y] = obj->offset[MXC6225_AXIS_Y]*mul + obj->cali_sw[MXC6225_AXIS_Y];
    raw[MXC6225_AXIS_Z] = obj->offset[MXC6225_AXIS_Z]*mul + obj->cali_sw[MXC6225_AXIS_Z];

    act[obj->cvt.map[MXC6225_AXIS_X]] = obj->cvt.sign[MXC6225_AXIS_X]*raw[MXC6225_AXIS_X];
    act[obj->cvt.map[MXC6225_AXIS_Y]] = obj->cvt.sign[MXC6225_AXIS_Y]*raw[MXC6225_AXIS_Y];
    act[obj->cvt.map[MXC6225_AXIS_Z]] = obj->cvt.sign[MXC6225_AXIS_Z]*raw[MXC6225_AXIS_Z];

    return 0;
}
/*----------------------------------------------------------------------------*/
static int MXC6225_WriteCalibration(struct i2c_client *client, int dat[MXC6225_AXES_NUM])
{
    struct mxc6225_i2c_data *obj = i2c_get_clientdata(client);
    int err;
    int cali[MXC6225_AXES_NUM], raw[MXC6225_AXES_NUM];
    int lsb = mxc6225_offset_resolution.sensitivity;
    int divisor = obj->reso->sensitivity/lsb;

    if (err = MXC6225_ReadCalibrationEx(client, cali, raw))	/*offset will be updated in obj->offset*/
    {
        GSE_ERR("read offset fail, %d\n", err);
        return err;
    }

    GSE_LOG("OLDOFF: (%+3d %+3d %+3d): (%+3d %+3d %+3d) / (%+3d %+3d %+3d)\n",
            raw[MXC6225_AXIS_X], raw[MXC6225_AXIS_Y], raw[MXC6225_AXIS_Z],
            obj->offset[MXC6225_AXIS_X], obj->offset[MXC6225_AXIS_Y], obj->offset[MXC6225_AXIS_Z],
            obj->cali_sw[MXC6225_AXIS_X], obj->cali_sw[MXC6225_AXIS_Y], obj->cali_sw[MXC6225_AXIS_Z]);

    /*calculate the real offset expected by caller*/
    cali[MXC6225_AXIS_X] += dat[MXC6225_AXIS_X];
    cali[MXC6225_AXIS_Y] += dat[MXC6225_AXIS_Y];
    cali[MXC6225_AXIS_Z] += dat[MXC6225_AXIS_Z];

    GSE_LOG("UPDATE: (%+3d %+3d %+3d)\n",
            dat[MXC6225_AXIS_X], dat[MXC6225_AXIS_Y], dat[MXC6225_AXIS_Z]);

#ifdef SW_CALIBRATION
    obj->cali_sw[MXC6225_AXIS_X] = obj->cvt.sign[MXC6225_AXIS_X]*(cali[obj->cvt.map[MXC6225_AXIS_X]]);
    obj->cali_sw[MXC6225_AXIS_Y] = obj->cvt.sign[MXC6225_AXIS_Y]*(cali[obj->cvt.map[MXC6225_AXIS_Y]]);
    obj->cali_sw[MXC6225_AXIS_Z] = obj->cvt.sign[MXC6225_AXIS_Z]*(cali[obj->cvt.map[MXC6225_AXIS_Z]]);
#else
    obj->offset[MXC6225_AXIS_X] = (s8)(obj->cvt.sign[MXC6225_AXIS_X]*(cali[obj->cvt.map[MXC6225_AXIS_X]])/(divisor));
    obj->offset[MXC6225_AXIS_Y] = (s8)(obj->cvt.sign[MXC6225_AXIS_Y]*(cali[obj->cvt.map[MXC6225_AXIS_Y]])/(divisor));
    obj->offset[MXC6225_AXIS_Z] = (s8)(obj->cvt.sign[MXC6225_AXIS_Z]*(cali[obj->cvt.map[MXC6225_AXIS_Z]])/(divisor));

    /*convert software calibration using standard calibration*/
    obj->cali_sw[MXC6225_AXIS_X] = obj->cvt.sign[MXC6225_AXIS_X]*(cali[obj->cvt.map[MXC6225_AXIS_X]])%(divisor);
    obj->cali_sw[MXC6225_AXIS_Y] = obj->cvt.sign[MXC6225_AXIS_Y]*(cali[obj->cvt.map[MXC6225_AXIS_Y]])%(divisor);
    obj->cali_sw[MXC6225_AXIS_Z] = obj->cvt.sign[MXC6225_AXIS_Z]*(cali[obj->cvt.map[MXC6225_AXIS_Z]])%(divisor);

    GSE_LOG("NEWOFF: (%+3d %+3d %+3d): (%+3d %+3d %+3d) / (%+3d %+3d %+3d)\n",
            obj->offset[MXC6225_AXIS_X]*divisor + obj->cali_sw[MXC6225_AXIS_X],
            obj->offset[MXC6225_AXIS_Y]*divisor + obj->cali_sw[MXC6225_AXIS_Y],
            obj->offset[MXC6225_AXIS_Z]*divisor + obj->cali_sw[MXC6225_AXIS_Z],
            obj->offset[MXC6225_AXIS_X], obj->offset[MXC6225_AXIS_Y], obj->offset[MXC6225_AXIS_Z],
            obj->cali_sw[MXC6225_AXIS_X], obj->cali_sw[MXC6225_AXIS_Y], obj->cali_sw[MXC6225_AXIS_Z]);

    if (err = hwmsen_write_block(obj->client, MXC6225_REG_OFSX, obj->offset, MXC6225_AXES_NUM))
    {
        GSE_ERR("write offset fail: %d\n", err);
        return err;
    }
#endif

    return err;
}
/*----------------------------------------------------------------------------*/
static int MXC6225_CheckDeviceID(struct i2c_client *client)
{
    u8 databuf[2];
    int res = 0;
    /*
    	memset(databuf, 0, sizeof(u8)*2);
    	databuf[0] = MXC6225_REG_DEVID;

    	res = i2c_master_send(client, databuf, 0x1);
    	if(res <= 0)
    	{
    		goto exit_MXC6225_CheckDeviceID;
    	}

    	udelay(500);

    	databuf[0] = 0x0;
    	res = i2c_master_recv(client, databuf, 0x01);
    	if(res <= 0)
    	{
    		goto exit_MXC6225_CheckDeviceID;
    	}*/

    if (hwmsen_read_byte_sr(client, MXC6225_REG_DEVID, databuf))
    {
        goto exit_MXC6225_CheckDeviceID;
    }
        return MXC6225_SUCCESS;

exit_MXC6225_CheckDeviceID:
    if (res <= 0)
    {
        return MXC6225_ERR_I2C;
    }

    return MXC6225_SUCCESS;
}
/*----------------------------------------------------------------------------*/
static int MXC6225_SetPowerMode_ext(struct i2c_client *client, bool enable)
{
    u8 databuf[2];
    int res = 0;
    u8 addr = MXC6225_REG_DETECTION;
    struct mxc6225_i2c_data *obj = i2c_get_clientdata(client);

    printk("MXC6225_SetPowerMode ok! enable=%d \n",enable);

    if (enable == sensor_power )
    {
        GSE_LOG("Sensor power status is newest!\n");
        return MXC6225_SUCCESS;
    }

    //if(hwmsen_read_block(client, addr, databuf, 0x01))
    if (hwmsen_read_byte_sr(client, addr, databuf))
    {
        GSE_ERR("read power ctl register err!\n");
        return MXC6225_ERR_I2C;
    }

    if (enable == TRUE)
    {
        databuf[0] &= ~MXC6225_MEASURE_MODE;
    }
    else
    {
        databuf[0] |= MXC6225_MEASURE_MODE;
    }
    databuf[1] = databuf[0];
    databuf[0] = MXC6225_REG_DETECTION;


    res = i2c_master_send(client, databuf, 0x2);

    if (res <= 0)
    {
        GSE_LOG("set power mode failed!\n");
        return MXC6225_ERR_I2C;
    }
    else if (atomic_read(&obj->trace) & ADX_TRC_INFO)
    {
        GSE_LOG("set power mode ok %d!\n", databuf[1]);
    }

    //GSE_LOG("MXC6225_SetPowerMode ok!\n");

    sensor_power = enable;

    mdelay(20);

    return MXC6225_SUCCESS;
}

//add by libo 20130702
static int MXC6225_SetPowerMode(struct i2c_client *client, bool enable)
{
    if (suspend_lock == 1)
    {
        GSE_LOG("Gsensor device have suspend_lock!\n");
        return MXC6225_SUCCESS;
    }
	return MXC6225_SetPowerMode_ext(client,enable);
}
/*----------------------------------------------------------------------------*/
static int MXC6225_SetDataFormat(struct i2c_client *client, u8 dataformat)
{
    struct mxc6225_i2c_data *obj = i2c_get_clientdata(client);
    u8 databuf[10];
    int res = 0;
    /*
    	memset(databuf, 0, sizeof(u8)*10);

    	if(hwmsen_read_block(client, MXC6225_REG_DATA_FORMAT, databuf, 0x01))
    	{
    		printk("mxc6225 read Dataformat failt \n");
    		return MXC6225_ERR_I2C;
    	}

    	databuf[0] &= ~MXC6225_RANGE_MASK;
    	databuf[0] |= dataformat;
    	databuf[1] = databuf[0];
    	databuf[0] = MXC6225_REG_DATA_FORMAT;


    	res = i2c_master_send(client, databuf, 0x2);

    	if(res <= 0)
    	{
    		return MXC6225_ERR_I2C;
    	}

    	//printk("MXC6225_SetDataFormat OK! \n");

    	*/
    return MXC6225_SetDataResolution(obj);


}
/*----------------------------------------------------------------------------*/
static int MXC6225_SetBWRate(struct i2c_client *client, u8 bwrate)
{/*
        	u8 databuf[10];
        	int res = 0;

        	memset(databuf, 0, sizeof(u8)*10);

        	if(hwmsen_read_block(client, MXC6225_REG_BW_RATE, databuf, 0x01))
        	{
        		printk("mxc6225 read rate failt \n");
        		return MXC6225_ERR_I2C;
        	}

        	databuf[0] &= ~MXC6225_BW_MASK;
        	databuf[0] |= bwrate;
        	databuf[1] = databuf[0];
        	databuf[0] = MXC6225_REG_BW_RATE;


        	res = i2c_master_send(client, databuf, 0x2);

        	if(res <= 0)
        	{
        		return MXC6225_ERR_I2C;
        	}
        	*/
    //printk("MXC6225_SetBWRate OK! \n");

    return MXC6225_SUCCESS;
}
/*----------------------------------------------------------------------------*/
static int MXC6225_SetIntEnable(struct i2c_client *client, u8 intenable)
{/*
        			u8 databuf[10];
        			int res = 0;

        			res = hwmsen_write_byte(client, MXC6225_INT_REG_1, 0x00);
        			if(res != MXC6225_SUCCESS)
        			{
        				return res;
        			}
        			res = hwmsen_write_byte(client, MXC6225_INT_REG_2, 0x00);
        			if(res != MXC6225_SUCCESS)
        			{
        				return res;
        			}*/
    printk("MXC6225 disable interrupt ...\n");

    /*for disable interrupt function*/

    return MXC6225_SUCCESS;
}

/*----------------------------------------------------------------------------*/
static int mxc6225_init_client(struct i2c_client *client, int reset_cali)
{
    struct mxc6225_i2c_data *obj = i2c_get_clientdata(client);
    int res = 0;
    printk("MYCAT mxc6225_init_client \n");

    res = MXC6225_CheckDeviceID(client);
    if (res != MXC6225_SUCCESS)
    {
        return res;
    }
    printk("MXC6225_CheckDeviceID ok \n");


    res = MXC6225_SetBWRate(client, MXC6225_BW_100HZ);
    if (res != MXC6225_SUCCESS )
    {
        return res;
    }
    printk("MXC6225_SetBWRate OK!\n");

    res = MXC6225_SetDataFormat(client, MXC6225_RANGE_2G);
    if (res != MXC6225_SUCCESS)
    {
        return res;
    }
    printk("MXC6225_SetDataFormat OK!\n");

    gsensor_gain.x = gsensor_gain.y = gsensor_gain.z = obj->reso->sensitivity;


    res = MXC6225_SetIntEnable(client, 0x00);
    if (res != MXC6225_SUCCESS)
    {
        return res;
    }
    printk("MXC6225 disable interrupt function!\n");

    res = MXC6225_SetPowerMode(client, false);
    if (res != MXC6225_SUCCESS)
    {
        return res;
    }
    printk("MXC6225_SetPowerMode OK!\n");


    if (0 != reset_cali)
    {
        /*reset calibration only in power on*/
        res = MXC6225_ResetCalibration(client);
        if (res != MXC6225_SUCCESS)
        {
            return res;
        }
    }



    printk("mxc6225_init_client OK!\n");
#ifdef CONFIG_MXC6225_LOWPASS
    memset(&obj->fir, 0x00, sizeof(obj->fir));
#endif

    mdelay(20);

    return MXC6225_SUCCESS;
}
/*----------------------------------------------------------------------------*/
static int MXC6225_ReadChipInfo(struct i2c_client *client, char *buf, int bufsize)
{
    u8 databuf[10];

    memset(databuf, 0, sizeof(u8)*10);

    if ((NULL == buf)||(bufsize<=30))
    {
        return -1;
    }

    if (NULL == client)
    {
        *buf = 0;
        return -2;
    }

    sprintf(buf, "MXC6225 Chip");
    return 0;
}
/*----------------------------------------------------------------------------*/
static int MXC6225_ReadSensorData(struct i2c_client *client, char *buf, int bufsize)
{
    struct mxc6225_i2c_data *obj = (struct mxc6225_i2c_data*)i2c_get_clientdata(client);
    char databuf[MXC6225_AXES_NUM] = {0};
    int acc[MXC6225_AXES_NUM] = {0};
    int res = 0;

    memset(acc, 0, sizeof(u8)*MXC6225_AXES_NUM);

    if (NULL == buf)
    {
        return -1;
    }
    if (NULL == client)
    {
        *buf = 0;
        return -2;
    }

	if(sensor_power == FALSE)
	{
		res = MXC6225_SetPowerMode(client, true);
		if(res)
		{
			GSE_ERR("Power on mxc6225 error %d!\n", res);
		}
	}

    if (res = MXC6225_ReadData(client, obj->data))
    {
        GSE_ERR("I2C error: ret value=%d", res);
        return -3;
    }
    else
    {
        mm_segment_t old_fs = get_fs();
        char strbuf[MXC6225_BUFSIZE];
        s16   data[MXC6225_AXES_NUM+1];
        struct file *fp;
        set_fs(KERNEL_DS);

        fp = filp_open(FTM_CUST_ACC,O_RDONLY,0);

        if (fp != 0xfffffffe)//open file suscess
        {
            //fp->f_op->llseek(fp, 0, 0);
            fp->f_pos = 0;
            fp->f_op->read(fp,
                           strbuf,
                           MXC6225_BUFSIZE,
                           &fp->f_pos);
            filp_close(fp, NULL);
            set_fs(old_fs);
            sscanf(strbuf, "%02x %02x %02x", &data[MXC6225_AXIS_X], &data[MXC6225_AXIS_Y], &data[MXC6225_AXIS_Z]);
            obj->data[MXC6225_AXIS_X] -= data[MXC6225_AXIS_X];
            obj->data[MXC6225_AXIS_Y] -= data[MXC6225_AXIS_Y];
            obj->data[MXC6225_AXIS_Z] -= (data[MXC6225_AXIS_Z]-(obj->cvt.sign[MXC6225_AXIS_Z]*obj->reso->sensitivity));
            //printk("BMA222E_SET_CALIBRATION value is %x %x %x -> %d %d %d\r\n",data[MXC6225_AXIS_X], data[MXC6225_AXIS_Y], data[MXC6225_AXIS_Z],obj->data[MXC6225_AXIS_X],obj->data[MXC6225_AXIS_Y],obj->data[MXC6225_AXIS_Z]);
        }
        //printk("raw data x=%d, y=%d, z=%d \n",obj->data[MXC6225_AXIS_X],obj->data[MXC6225_AXIS_Y],obj->data[MXC6225_AXIS_Z]);
        obj->data[MXC6225_AXIS_X] += obj->cali_sw[MXC6225_AXIS_X];
        obj->data[MXC6225_AXIS_Y] += obj->cali_sw[MXC6225_AXIS_Y];
        obj->data[MXC6225_AXIS_Z] += obj->cali_sw[MXC6225_AXIS_Z];
        //printk("cali_sw x=%d, y=%d, z=%d \n",obj->cali_sw[MXC6225_AXIS_X],obj->cali_sw[MXC6225_AXIS_Y],obj->cali_sw[MXC6225_AXIS_Z]);
        /*remap coordinate*/
        acc[obj->cvt.map[MXC6225_AXIS_X]] = obj->cvt.sign[MXC6225_AXIS_X]*obj->data[MXC6225_AXIS_X];
        acc[obj->cvt.map[MXC6225_AXIS_Y]] = obj->cvt.sign[MXC6225_AXIS_Y]*obj->data[MXC6225_AXIS_Y];
        acc[obj->cvt.map[MXC6225_AXIS_Z]] = obj->cvt.sign[MXC6225_AXIS_Z]*obj->data[MXC6225_AXIS_Z];
        //printk("cvt x=%d, y=%d, z=%d \n",obj->cvt.sign[MXC6225_AXIS_X],obj->cvt.sign[MXC6225_AXIS_Y],obj->cvt.sign[MXC6225_AXIS_Z]);

        //GSE_LOG("Mapped gsensor data: %d, %d, %d!\n", acc[MXC6225_AXIS_X], acc[MXC6225_AXIS_Y], acc[MXC6225_AXIS_Z]);

        //Out put the mg
        //printk("mg acc=%d, GRAVITY=%d, sensityvity=%d \n",acc[MXC6225_AXIS_X],GRAVITY_EARTH_1000,obj->reso->sensitivity);

        //acc[MXC6225_AXIS_X] = (int)obj->data[MXC6225_AXIS_X] * GRAVITY_EARTH_1000 / 64;
        //acc[MXC6225_AXIS_Y] = (int)obj->data[MXC6225_AXIS_Y] * GRAVITY_EARTH_1000 / 64;
        //acc[MXC6225_AXIS_Z] = 32;
        acc[MXC6225_AXIS_X] = acc[MXC6225_AXIS_X] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
        acc[MXC6225_AXIS_Y] = acc[MXC6225_AXIS_Y] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
        acc[MXC6225_AXIS_Z] = 32;
        sprintf(buf, "%04x %04x %04x", acc[MXC6225_AXIS_X], acc[MXC6225_AXIS_Y], acc[MXC6225_AXIS_Z]);
        if (atomic_read(&obj->trace) & ADX_TRC_IOCTL)
        {
            GSE_LOG("gsensor data: %s!\n", buf);
        }
    }
    return 0;
}

/*----------------------------------------------------------------------------*/
static int MXC6225_ReadSensorData_factory_autotest(struct i2c_client *client, char *buf, int bufsize)
{
    struct mxc6225_i2c_data *obj = (struct mxc6225_i2c_data*)i2c_get_clientdata(client);
    char databuf[MXC6225_AXES_NUM] = {0};
    int acc[MXC6225_AXES_NUM] = {0};
    int res = 0;

    memset(acc, 0, sizeof(u8)*MXC6225_AXES_NUM);

    printk("MXC6225_ReadSensorData_factory_autotest\n");

    if (NULL == buf)
    {
        return -1;
    }
    if (NULL == client)
    {
        *buf = 0;
        return -2;
    }

	if(sensor_power == FALSE)
	{
		res = MXC6225_SetPowerMode(client, true);
		if(res)
		{
			GSE_ERR("Power on mxc6225 error %d!\n", res);
		}
	}

    if (res = MXC6225_ReadData(client, obj->data))
    {
        GSE_ERR("I2C error: ret value=%d", res);
        return -3;
    }
    else
    {
        mm_segment_t old_fs = get_fs();
        char strbuf[MXC6225_BUFSIZE];
        s16   data[MXC6225_AXES_NUM+1];
        struct file *fp;
        set_fs(KERNEL_DS);

        fp = filp_open(FTM_CUST_ACC,O_RDONLY,0);

        if (fp != 0xfffffffe)//open file suscess
        {
            //fp->f_op->llseek(fp, 0, 0);
            fp->f_pos = 0;
            fp->f_op->read(fp,
                           strbuf,
                           MXC6225_BUFSIZE,
                           &fp->f_pos);
            filp_close(fp, NULL);
            set_fs(old_fs);
            sscanf(strbuf, "%02x %02x %02x", &data[MXC6225_AXIS_X], &data[MXC6225_AXIS_Y], &data[MXC6225_AXIS_Z]);
            obj->data[MXC6225_AXIS_X] -= data[MXC6225_AXIS_X];
            obj->data[MXC6225_AXIS_Y] -= data[MXC6225_AXIS_Y];
            obj->data[MXC6225_AXIS_Z] -= (data[MXC6225_AXIS_Z]-(obj->cvt.sign[MXC6225_AXIS_Z]*obj->reso->sensitivity));
            //printk("BMA222E_SET_CALIBRATION value is %x %x %x -> %d %d %d\r\n",data[MXC6225_AXIS_X], data[MXC6225_AXIS_Y], data[MXC6225_AXIS_Z],obj->data[MXC6225_AXIS_X],obj->data[MXC6225_AXIS_Y],obj->data[MXC6225_AXIS_Z]);
        }
        //printk("raw data x=%d, y=%d, z=%d \n",obj->data[MXC6225_AXIS_X],obj->data[MXC6225_AXIS_Y],obj->data[MXC6225_AXIS_Z]);
        obj->data[MXC6225_AXIS_X] += obj->cali_sw[MXC6225_AXIS_X];
        obj->data[MXC6225_AXIS_Y] += obj->cali_sw[MXC6225_AXIS_Y];
        obj->data[MXC6225_AXIS_Z] += obj->cali_sw[MXC6225_AXIS_Z];
        //printk("cali_sw x=%d, y=%d, z=%d \n",obj->cali_sw[MXC6225_AXIS_X],obj->cali_sw[MXC6225_AXIS_Y],obj->cali_sw[MXC6225_AXIS_Z]);
        /*remap coordinate*/
        acc[obj->cvt.map[MXC6225_AXIS_X]] = obj->cvt.sign[MXC6225_AXIS_X]*obj->data[MXC6225_AXIS_X];
        acc[obj->cvt.map[MXC6225_AXIS_Y]] = obj->cvt.sign[MXC6225_AXIS_Y]*obj->data[MXC6225_AXIS_Y];
        acc[obj->cvt.map[MXC6225_AXIS_Z]] = obj->cvt.sign[MXC6225_AXIS_Z]*obj->data[MXC6225_AXIS_Z];
        //printk("cvt x=%d, y=%d, z=%d \n",obj->cvt.sign[MXC6225_AXIS_X],obj->cvt.sign[MXC6225_AXIS_Y],obj->cvt.sign[MXC6225_AXIS_Z]);

        //GSE_LOG("Mapped gsensor data: %d, %d, %d!\n", acc[MXC6225_AXIS_X], acc[MXC6225_AXIS_Y], acc[MXC6225_AXIS_Z]);

        //Out put the mg
        //printk("mg acc=%d, GRAVITY=%d, sensityvity=%d \n",acc[MXC6225_AXIS_X],GRAVITY_EARTH_1000,obj->reso->sensitivity);

        //acc[MXC6225_AXIS_X] = (int)obj->data[MXC6225_AXIS_X] * GRAVITY_EARTH_1000 / 64;
        //acc[MXC6225_AXIS_Y] = (int)obj->data[MXC6225_AXIS_Y] * GRAVITY_EARTH_1000 / 64;
        //acc[MXC6225_AXIS_Z] = 32;
        acc[MXC6225_AXIS_X] = acc[MXC6225_AXIS_X] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
        acc[MXC6225_AXIS_Y] = acc[MXC6225_AXIS_Y] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
        acc[MXC6225_AXIS_Z] = 32;

        //add by huangrunming 20120907
#ifdef FACTORY_AUTOTEST_GSENSOR_VALUE
        auto_now_data[MXC6225_AXIS_X]= acc[MXC6225_AXIS_X];
        auto_now_data[MXC6225_AXIS_Y]= acc[MXC6225_AXIS_Y];
        auto_now_data[MXC6225_AXIS_Z]= 32;
        printk("haha\n");
        printk("acc[MXC6225_AXIS_X] = %d,acc[MXC6225_AXIS_Y] = %d\n",acc[MXC6225_AXIS_X],acc[MXC6225_AXIS_Y]);
        if ((acc[MXC6225_AXIS_X] > 5000) || (acc[MXC6225_AXIS_X] < -5000)){
            printk("auto_now_data x fail: \n");
            return -4;
        }
        if (acc[MXC6225_AXIS_Y] > 5000 || acc[MXC6225_AXIS_Y] < -5000){
            printk("auto_now_data y fail: \n");
            return -5;
        }
#endif
        sprintf(buf, "%04x %04x %04x", acc[MXC6225_AXIS_X], acc[MXC6225_AXIS_Y], acc[MXC6225_AXIS_Z]);
        if (atomic_read(&obj->trace) & ADX_TRC_IOCTL)
        {
            GSE_LOG("gsensor data: %s!\n", buf);
        }
    }
    return 0;
}

/*----------------------------------------------------------------------------*/
static int MXC6225_ReadRawData(struct i2c_client *client, char *buf)
{
    struct mxc6225_i2c_data *obj = (struct mxc6225_i2c_data*)i2c_get_clientdata(client);
    int res = 0;

    if (!buf || !client)
    {
        return EINVAL;
    }

    if (res = MXC6225_ReadData(client, obj->data))
    {
        GSE_ERR("I2C error: ret value=%d", res);
        return EIO;
    }
    else
    {
        sprintf(buf, "%04x %04x %04x", obj->data[MXC6225_AXIS_X],
                obj->data[MXC6225_AXIS_Y], obj->data[MXC6225_AXIS_Z]);

    }

    return 0;
}

static int MXC6225_SET_CALIBRATION(struct i2c_client *client)
{
    struct mxc6225_i2c_data *obj = (struct mxc6225_i2c_data*)i2c_get_clientdata(client);
    char strbuf[MXC6225_BUFSIZE];
    s16   data[MXC6225_AXES_NUM+1];
    struct file *fp;
    MXC6225_ReadData(client, data);
    sprintf(strbuf, "%02x %02x %02x", data[MXC6225_AXIS_X], data[MXC6225_AXIS_Y], data[MXC6225_AXIS_Z]);
    //write to nvram
    mm_segment_t old_fs = get_fs();
    set_fs(KERNEL_DS);

    fp = filp_open(FTM_CUST_ACC,O_WRONLY|O_CREAT, 0644);
    fp->f_pos = 0;
    fp->f_op->write(fp,
                    strbuf,
                    MXC6225_BUFSIZE,
                    &fp->f_pos);
    filp_close(fp, NULL);
    printk("MXC6225_SET_CALIBRATION value is %x %x %x\r\n",data[MXC6225_AXIS_X], data[MXC6225_AXIS_Y], data[MXC6225_AXIS_Z]);

    set_fs(old_fs);
    return 0;
}

/*----------------------------------------------------------------------------*/
static ssize_t show_chipinfo_value(struct device_driver *ddri, char *buf)
{
    struct i2c_client *client = mxc6225_i2c_client;
    char strbuf[MXC6225_BUFSIZE];
    if (NULL == client)
    {
        GSE_ERR("i2c client is null!!\n");
        return 0;
    }

    MXC6225_ReadChipInfo(client, strbuf, MXC6225_BUFSIZE);
    return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);
}

static ssize_t gsensor_init(struct device_driver *ddri, char *buf, size_t count)
{
    struct i2c_client *client = mxc6225_i2c_client;
    char strbuf[MXC6225_BUFSIZE];

    if (NULL == client)
    {
        GSE_ERR("i2c client is null!!\n");
        return 0;
    }
    mxc6225_init_client(client, 1);
    return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);
}



/*----------------------------------------------------------------------------*/
static ssize_t show_sensordata_value(struct device_driver *ddri, char *buf)
{
    struct i2c_client *client = mxc6225_i2c_client;
    char strbuf[MXC6225_BUFSIZE];

    if (NULL == client)
    {
        GSE_ERR("i2c client is null!!\n");
        return 0;
    }
    MXC6225_ReadSensorData(client, strbuf, MXC6225_BUFSIZE);
    //MXC6225_ReadRawData(client, strbuf);
    return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);
}

static ssize_t show_sensorrawdata_value(struct device_driver *ddri, char *buf, size_t count)
{
    struct i2c_client *client = mxc6225_i2c_client;
    char strbuf[MXC6225_BUFSIZE];

    if (NULL == client)
    {
        GSE_ERR("i2c client is null!!\n");
        return 0;
    }
    //MXC6225_ReadSensorData(client, strbuf, MXC6225_BUFSIZE);
    MXC6225_ReadRawData(client, strbuf);
    return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);
}

/*----------------------------------------------------------------------------*/
static ssize_t show_cali_value(struct device_driver *ddri, char *buf)
{
    struct i2c_client *client = mxc6225_i2c_client;
    struct mxc6225_i2c_data *obj;
    int err, len = 0, mul;
    int tmp[MXC6225_AXES_NUM];

    if (NULL == client)
    {
        GSE_ERR("i2c client is null!!\n");
        return 0;
    }

    obj = i2c_get_clientdata(client);



    if (err = MXC6225_ReadOffset(client, obj->offset))
    {
        return -EINVAL;
    }
    else if (err = MXC6225_ReadCalibration(client, tmp))
    {
        return -EINVAL;
    }
    else
    {
        mul = obj->reso->sensitivity/mxc6225_offset_resolution.sensitivity;
        len += snprintf(buf+len, PAGE_SIZE-len, "[HW ][%d] (%+3d, %+3d, %+3d) : (0x%02X, 0x%02X, 0x%02X)\n", mul,
                        obj->offset[MXC6225_AXIS_X], obj->offset[MXC6225_AXIS_Y], obj->offset[MXC6225_AXIS_Z],
                        obj->offset[MXC6225_AXIS_X], obj->offset[MXC6225_AXIS_Y], obj->offset[MXC6225_AXIS_Z]);
        len += snprintf(buf+len, PAGE_SIZE-len, "[SW ][%d] (%+3d, %+3d, %+3d)\n", 1,
                        obj->cali_sw[MXC6225_AXIS_X], obj->cali_sw[MXC6225_AXIS_Y], obj->cali_sw[MXC6225_AXIS_Z]);

        len += snprintf(buf+len, PAGE_SIZE-len, "[ALL]    (%+3d, %+3d, %+3d) : (%+3d, %+3d, %+3d)\n",
                        obj->offset[MXC6225_AXIS_X]*mul + obj->cali_sw[MXC6225_AXIS_X],
                        obj->offset[MXC6225_AXIS_Y]*mul + obj->cali_sw[MXC6225_AXIS_Y],
                        obj->offset[MXC6225_AXIS_Z]*mul + obj->cali_sw[MXC6225_AXIS_Z],
                        tmp[MXC6225_AXIS_X], tmp[MXC6225_AXIS_Y], tmp[MXC6225_AXIS_Z]);

        return len;
    }
}
/*----------------------------------------------------------------------------*/
static ssize_t store_cali_value(struct device_driver *ddri, char *buf, size_t count)
{
    struct i2c_client *client = mxc6225_i2c_client;
    int err, x, y, z;
    int dat[MXC6225_AXES_NUM];

    if (!strncmp(buf, "rst", 3))
    {
        if (err = MXC6225_ResetCalibration(client))
        {
            GSE_ERR("reset offset err = %d\n", err);
        }
    }
    else if (3 == sscanf(buf, "0x%02X 0x%02X 0x%02X", &x, &y, &z))
    {
        dat[MXC6225_AXIS_X] = x;
        dat[MXC6225_AXIS_Y] = y;
        dat[MXC6225_AXIS_Z] = z;
        if (err = MXC6225_WriteCalibration(client, dat))
        {
            GSE_ERR("write calibration err = %d\n", err);
        }
    }
    else
    {
        GSE_ERR("invalid format\n");
    }

    return count;
}


/*----------------------------------------------------------------------------*/
static ssize_t show_firlen_value(struct device_driver *ddri, char *buf)
{
#ifdef CONFIG_MXC6225_LOWPASS
    struct i2c_client *client = mxc6225_i2c_client;
    struct mxc6225_i2c_data *obj = i2c_get_clientdata(client);
    if (atomic_read(&obj->firlen))
    {
        int idx, len = atomic_read(&obj->firlen);
        GSE_LOG("len = %2d, idx = %2d\n", obj->fir.num, obj->fir.idx);

        for (idx = 0; idx < len; idx++)
        {
            GSE_LOG("[%5d %5d %5d]\n", obj->fir.raw[idx][MXC6225_AXIS_X], obj->fir.raw[idx][MXC6225_AXIS_Y], obj->fir.raw[idx][MXC6225_AXIS_Z]);
        }

        GSE_LOG("sum = [%5d %5d %5d]\n", obj->fir.sum[MXC6225_AXIS_X], obj->fir.sum[MXC6225_AXIS_Y], obj->fir.sum[MXC6225_AXIS_Z]);
        GSE_LOG("avg = [%5d %5d %5d]\n", obj->fir.sum[MXC6225_AXIS_X]/len, obj->fir.sum[MXC6225_AXIS_Y]/len, obj->fir.sum[MXC6225_AXIS_Z]/len);
    }
    return snprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&obj->firlen));
#else
    return snprintf(buf, PAGE_SIZE, "not support\n");
#endif
}
/*----------------------------------------------------------------------------*/
static ssize_t store_firlen_value(struct device_driver *ddri, char *buf, size_t count)
{
#ifdef CONFIG_MXC6225_LOWPASS
    struct i2c_client *client = mxc6225_i2c_client;
    struct mxc6225_i2c_data *obj = i2c_get_clientdata(client);
    int firlen;

    if (1 != sscanf(buf, "%d", &firlen))
    {
        GSE_ERR("invallid format\n");
    }
    else if (firlen > C_MAX_FIR_LENGTH)
    {
        GSE_ERR("exceeds maximum filter length\n");
    }
    else
    {
        atomic_set(&obj->firlen, firlen);
        if (NULL == firlen)
        {
            atomic_set(&obj->fir_en, 0);
        }
        else
        {
            memset(&obj->fir, 0x00, sizeof(obj->fir));
            atomic_set(&obj->fir_en, 1);
        }
    }
#endif
    return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t show_trace_value(struct device_driver *ddri, char *buf)
{
    ssize_t res;
    struct mxc6225_i2c_data *obj = obj_i2c_data;
    if (obj == NULL)
    {
        GSE_ERR("i2c_data obj is null!!\n");
        return 0;
    }

    res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&obj->trace));
    return res;
}
/*----------------------------------------------------------------------------*/
static ssize_t store_trace_value(struct device_driver *ddri, char *buf, size_t count)
{
    struct mxc6225_i2c_data *obj = obj_i2c_data;
    int trace;
    if (obj == NULL)
    {
        GSE_ERR("i2c_data obj is null!!\n");
        return 0;
    }

    if (1 == sscanf(buf, "0x%x", &trace))
    {
        atomic_set(&obj->trace, trace);
    }
    else
    {
        GSE_ERR("invalid content: '%s', length = %d\n", buf, count);
    }

    return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t show_status_value(struct device_driver *ddri, char *buf)
{
    ssize_t len = 0;
    struct mxc6225_i2c_data *obj = obj_i2c_data;
    if (obj == NULL)
    {
        GSE_ERR("i2c_data obj is null!!\n");
        return 0;
    }

    if (obj->hw)
    {
        len += snprintf(buf+len, PAGE_SIZE-len, "CUST: %d %d (%d %d)\n",
                        obj->hw->i2c_num, obj->hw->direction, obj->hw->power_id, obj->hw->power_vol);
    }
    else
    {
        len += snprintf(buf+len, PAGE_SIZE-len, "CUST: NULL\n");
    }
    return len;
}
/*----------------------------------------------------------------------------*/
static ssize_t show_power_status_value(struct device_driver *ddri, char *buf)
{
    if (sensor_power)
        printk("G sensor is in work mode, sensor_power = %d\n", sensor_power);
    else
        printk("G sensor is in standby mode, sensor_power = %d\n", sensor_power);

    return 0;
}
/*----------------------------------------------------------------------------*/
static DRIVER_ATTR(chipinfo,   S_IWUSR | S_IRUGO, show_chipinfo_value,      NULL);
static DRIVER_ATTR(sensordata, S_IWUSR | S_IRUGO, show_sensordata_value,    NULL);
static DRIVER_ATTR(cali,       S_IWUSR | S_IRUGO, show_cali_value,          store_cali_value);
static DRIVER_ATTR(firlen,     S_IWUSR | S_IRUGO, show_firlen_value,        store_firlen_value);
static DRIVER_ATTR(trace,      S_IWUSR | S_IRUGO, show_trace_value,         store_trace_value);
static DRIVER_ATTR(status,               S_IRUGO, show_status_value,        NULL);
static DRIVER_ATTR(powerstatus,               S_IRUGO, show_power_status_value,        NULL);

/*----------------------------------------------------------------------------*/
static struct driver_attribute *mxc6225_attr_list[] = {
            &driver_attr_chipinfo,     /*chip information*/
            &driver_attr_sensordata,   /*dump sensor data*/
            &driver_attr_cali,         /*show calibration data*/
            &driver_attr_firlen,       /*filter length: 0: disable, others: enable*/
            &driver_attr_trace,        /*trace log*/
            &driver_attr_status,
            &driver_attr_powerstatus,
        };
/*----------------------------------------------------------------------------*/
static int mxc6225_create_attr(struct device_driver *driver)
{
    int idx, err = 0;
    int num = (int)(sizeof(mxc6225_attr_list)/sizeof(mxc6225_attr_list[0]));
    if (driver == NULL)
    {
        return -EINVAL;
    }

    for (idx = 0; idx < num; idx++)
    {
        if (err = driver_create_file(driver, mxc6225_attr_list[idx]))
        {
            GSE_ERR("driver_create_file (%s) = %d\n", mxc6225_attr_list[idx]->attr.name, err);
            break;
        }
    }
    return err;
}
/*----------------------------------------------------------------------------*/
static int mxc6225_delete_attr(struct device_driver *driver)
{
    int idx ,err = 0;
    int num = (int)(sizeof(mxc6225_attr_list)/sizeof(mxc6225_attr_list[0]));

    if (driver == NULL)
    {
        return -EINVAL;
    }


    for (idx = 0; idx < num; idx++)
    {
        driver_remove_file(driver, mxc6225_attr_list[idx]);
    }


    return err;
}

/*----------------------------------------------------------------------------*/
int mxc6225_operate(void* self, uint32_t command, void* buff_in, int size_in,
                    void* buff_out, int size_out, int* actualout)
{
    int err = 0;
    int value, sample_delay;
    struct mxc6225_i2c_data *priv = (struct mxc6225_i2c_data*)self;
    hwm_sensor_data* gsensor_data;
    char buff[MXC6225_BUFSIZE];

    //GSE_FUN(f);
    switch (command)
    {
    case SENSOR_DELAY:
        if ((buff_in == NULL) || (size_in < sizeof(int)))
        {
            GSE_ERR("Set delay parameter error!\n");
            err = -EINVAL;
        }
        else
        {
            value = *(int *)buff_in;
            if (value <= 5)
            {
                sample_delay = MXC6225_BW_200HZ;
            }
            else if (value <= 10)
            {
                sample_delay = MXC6225_BW_100HZ;
            }
            else
            {
                sample_delay = MXC6225_BW_50HZ;
            }

            err = MXC6225_SetBWRate(priv->client, sample_delay);
            if (err != MXC6225_SUCCESS ) //0x2C->BW=100Hz
            {
                GSE_ERR("Set delay parameter error!\n");
            }

            if (value >= 50)
            {
                atomic_set(&priv->filter, 0);
            }
            else
            {
#if defined(CONFIG_MXC6225_LOWPASS)
                priv->fir.num = 0;
                priv->fir.idx = 0;
                priv->fir.sum[MXC6225_AXIS_X] = 0;
                priv->fir.sum[MXC6225_AXIS_Y] = 0;
                priv->fir.sum[MXC6225_AXIS_Z] = 0;
                atomic_set(&priv->filter, 1);
#endif
            }
        }
        break;

    case SENSOR_ENABLE:
        if ((buff_in == NULL) || (size_in < sizeof(int)))
        {
            GSE_ERR("Enable sensor parameter error!\n");
            err = -EINVAL;
        }
        else
        {
            value = *(int *)buff_in;
            if (((value == 0) && (sensor_power == false)) ||((value == 1) && (sensor_power == true)))
            {
                GSE_LOG("Gsensor device have updated!\n");
            }
            else
            {
                err = MXC6225_SetPowerMode( priv->client, !sensor_power);
            }
        }
        break;

    case SENSOR_GET_DATA:
        if ((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
        {
            GSE_ERR("get sensor data parameter error!\n");
            err = -EINVAL;
        }
        else
        {
            gsensor_data = (hwm_sensor_data *)buff_out;
            MXC6225_ReadSensorData(priv->client, buff, MXC6225_BUFSIZE);
            sscanf(buff, "%x %x %x", &gsensor_data->values[0],
                   &gsensor_data->values[1], &gsensor_data->values[2]);
            gsensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
            gsensor_data->value_divide = 1000;
        }
        break;
    default:
        GSE_ERR("gsensor operate function no this parameter %d!\n", command);
        err = -1;
        break;
    }

    return err;
}

/******************************************************************************
 * Function Configuration
******************************************************************************/
static int mxc6225_open(struct inode *inode, struct file *file)
{
    file->private_data = mxc6225_i2c_client;

    if (file->private_data == NULL)
    {
        GSE_ERR("null pointer!!\n");
        return -EINVAL;
    }
    return nonseekable_open(inode, file);
}
/*----------------------------------------------------------------------------*/
static int mxc6225_release(struct inode *inode, struct file *file)
{
    file->private_data = NULL;
    return 0;
}
/*----------------------------------------------------------------------------*/
//static int mxc6225_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
//       unsigned long arg)
static int mxc6225_unlocked_ioctl(struct file *file, unsigned int cmd,
                                  unsigned long arg)
{
    struct i2c_client *client = (struct i2c_client*)file->private_data;
    struct mxc6225_i2c_data *obj = (struct mxc6225_i2c_data*)i2c_get_clientdata(client);
    char strbuf[MXC6225_BUFSIZE];
    void __user *data;
    SENSOR_DATA sensor_data;
    int err = 0;
    int cali[3];
    signed char data_temp =0;

    //GSE_FUN(f);
    if (_IOC_DIR(cmd) & _IOC_READ)
    {
        err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
    }
    else if (_IOC_DIR(cmd) & _IOC_WRITE)
    {
        err = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
    }

    if (err)
    {
        GSE_ERR("access error: %08X, (%2d, %2d)\n", cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd));
        return -EFAULT;
    }

    switch (cmd)
    {
    case GSENSOR_IOCTL_INIT:
        mxc6225_init_client(client, 0);
        break;

    case GSENSOR_IOCTL_READ_CHIPINFO:
        data = (void __user *) arg;
        if (data == NULL)
        {
            err = -EINVAL;
            break;
        }

        MXC6225_ReadChipInfo(client, strbuf, MXC6225_BUFSIZE);
        if (copy_to_user(data, strbuf, strlen(strbuf)+1))
        {
            err = -EFAULT;
            break;
        }
        break;

    case GSENSOR_IOCTL_READ_SENSORDATA:
        data = (void __user *) arg;
        if (data == NULL)
        {
            err = -EINVAL;
            break;
        }

        MXC6225_ReadSensorData(client, strbuf, MXC6225_BUFSIZE);
        if (copy_to_user(data, strbuf, strlen(strbuf)+1))
        {
            err = -EFAULT;
            break;
        }
        break;

    case GSENSOR_IOCTL_READ_GAIN:
        data = (void __user *) arg;
        if (data == NULL)
        {
            err = -EINVAL;
            break;
        }

        if (copy_to_user(data, &gsensor_gain, sizeof(GSENSOR_VECTOR3D)))
        {
            err = -EFAULT;
            break;
        }
        break;

    case GSENSOR_IOCTL_READ_RAW_DATA:
        data = (void __user *) arg;
        if (data == NULL)
        {
            err = -EINVAL;
            break;
        }
        MXC6225_ReadRawData(client, strbuf);
        if (copy_to_user(data, &strbuf, strlen(strbuf)+1))
        {
            err = -EFAULT;
            break;
        }
        break;

    case GSENSOR_IOCTL_SET_CALI:
        data = (void __user*)arg;
        if (data == NULL)
        {
            err = -EINVAL;
            break;
        }
        if (copy_from_user(&sensor_data, data, sizeof(sensor_data)))
        {
            err = -EFAULT;
            break;
        }
        if (atomic_read(&obj->suspend))
        {
            GSE_ERR("Perform calibration in suspend state!!\n");
            err = -EINVAL;
        }
        else
        {
            cali[MXC6225_AXIS_X] = sensor_data.x * obj->reso->sensitivity / GRAVITY_EARTH_1000;
            cali[MXC6225_AXIS_Y] = sensor_data.y * obj->reso->sensitivity / GRAVITY_EARTH_1000;
            cali[MXC6225_AXIS_Z] = sensor_data.z * obj->reso->sensitivity / GRAVITY_EARTH_1000;
            err = MXC6225_WriteCalibration(client, cali);
        }
        break;

    case GSENSOR_IOCTL_CLR_CALI:
        err = MXC6225_ResetCalibration(client);
        break;

    case GSENSOR_IOCTL_GET_CALI:
        data = (void __user*)arg;
        if (data == NULL)
        {
            err = -EINVAL;
            break;
        }
        if (err = MXC6225_ReadCalibration(client, cali))
        {
            break;
        }

        sensor_data.x = cali[MXC6225_AXIS_X] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
        sensor_data.y = cali[MXC6225_AXIS_Y] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
        sensor_data.z = cali[MXC6225_AXIS_Z] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
        if (copy_to_user(data, &sensor_data, sizeof(sensor_data)))
        {
            err = -EFAULT;
            break;
        }
        break;
        /*
        case GSENSOR_IOCTL_TEST:							//modify by wangwei for auto test
        	
        	data = (void __user *) arg;
        	if(data == NULL)
        	{
        		err = -EINVAL;
        		break;	 
        	}
        	
        	//add by huangrunming 20120907
        	#ifndef FACTORY_AUTOTEST_GSENSOR_VALUE
        	    err = MXC6225_ReadSensorData(client, strbuf, MXC6225_BUFSIZE);
        	#else
        	    printk("mxc6225,FACTORY_AUTOTEST_GSENSOR_VALUE,_factory_autotest\n");
        	#endif

        	if (err < 0) 
        	{

        		data_temp = 1;
        	}
        	else
        	{
        		data_temp = 2;
        	}	
        	
        	if(copy_to_user((unsigned char*)arg,&data_temp,1)!=0)
        	{
        		printk("copy_to error\n");
        		return -EFAULT;
        	}
        	break;
        case GSENSOR_IOCTL_SET_CALIBRATION:
        		
        			err = MXC6225_SET_CALIBRATION(client);

        			break;		
*/
        
    default:
        GSE_ERR("unknown IOCTL: 0x%08x\n", cmd);
        err = -ENOIOCTLCMD;
        break;

    }

    return err;
}


/*----------------------------------------------------------------------------*/
static struct file_operations mxc6225_fops = {
            //.owner = THIS_MODULE,
            .open = mxc6225_open,
                    .release = mxc6225_release,
                               .unlocked_ioctl = mxc6225_unlocked_ioctl,
                                             };
/*----------------------------------------------------------------------------*/
static struct miscdevice mxc6225_device = {
            .minor = MISC_DYNAMIC_MINOR,
                     .name = "gsensor",
                             .fops = &mxc6225_fops,
                                 };
/*----------------------------------------------------------------------------*/
#ifndef CONFIG_HAS_EARLYSUSPEND
/*----------------------------------------------------------------------------*/
static int mxc6225_suspend(struct i2c_client *client, pm_message_t msg)
{
    struct mxc6225_i2c_data *obj = i2c_get_clientdata(client);
    int err = 0;
    GSE_FUN();

    if (msg.event == PM_EVENT_SUSPEND)
    {
        if (obj == NULL)
        {
            GSE_ERR("null pointer!!\n");
            return -EINVAL;
        }
        atomic_set(&obj->suspend, 1);
        if (err = MXC6225_SetPowerMode(obj->client, false))
        {
            GSE_ERR("write power control fail!!\n");
            return;
        }
        MXC6225_power(obj->hw, 0);
    }
    return err;
}
/*----------------------------------------------------------------------------*/
static int mxc6225_resume(struct i2c_client *client)
{
    struct mxc6225_i2c_data *obj = i2c_get_clientdata(client);
    int err;
    GSE_FUN();

    if (obj == NULL)
    {
        GSE_ERR("null pointer!!\n");
        return -EINVAL;
    }

    MXC6225_power(obj->hw, 1);
    //if(err = mxc6225_init_client(client, 0))
    //{
    //	GSE_ERR("initialize client fail!!\n");
    //	return err;
    //}
    atomic_set(&obj->suspend, 0);

    return 0;
}
/*----------------------------------------------------------------------------*/
#else /*CONFIG_HAS_EARLY_SUSPEND is defined*/
/*----------------------------------------------------------------------------*/
static void mxc6225_early_suspend(struct early_suspend *h)
{
    struct mxc6225_i2c_data *obj = container_of(h, struct mxc6225_i2c_data, early_drv);
    int err;
    GSE_FUN();

    if (obj == NULL)
    {
        GSE_ERR("null pointer!!\n");
        return;
    }
    atomic_set(&obj->suspend, 1);

    suspend_lock = 1;//add by libo 20130702
    if (err = MXC6225_SetPowerMode_ext(obj->client, false))
    {
        GSE_ERR("write power control fail!!\n");
        return;
    }

    sensor_power = false;

    MXC6225_power(obj->hw, 0);
}
/*----------------------------------------------------------------------------*/
static void mxc6225_late_resume(struct early_suspend *h)
{
    struct mxc6225_i2c_data *obj = container_of(h, struct mxc6225_i2c_data, early_drv);
    int err;
    GSE_FUN();

    if (obj == NULL)
    {
        GSE_ERR("null pointer!!\n");
        return;
    }

    MXC6225_power(obj->hw, 1);
    //if(err = mxc6225_init_client(obj->client, 0))
    //{
    //	GSE_ERR("initialize client fail!!\n");
    //	return;
    //}
    atomic_set(&obj->suspend, 0);

    suspend_lock = 0;	//add by libo 20130702

}
/*----------------------------------------------------------------------------*/
#endif /*CONFIG_HAS_EARLYSUSPEND*/
/*----------------------------------------------------------------------------*/
static int mxc6225_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info)
{
    strcpy(info->type, "MXC6225");
    return 0;
}

/*----------------------------------------------------------------------------*/
static int mxc6225_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct i2c_client *new_client;
    struct mxc6225_i2c_data *obj;
    struct hwmsen_object sobj;
    int err = 0;
    GSE_FUN();
    mdelay(100);

    if (!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
    {
        err = -ENOMEM;
        goto exit;
    }

    memset(obj, 0, sizeof(struct mxc6225_i2c_data));

    obj->hw = get_cust_acc_hw();
    /*add by zhuyaozhong 20120925*/
#ifdef BIRD_GSENSOR_MXC6225_DIR
    obj->hw->direction = simple_strtol(BIRD_GSENSOR_MXC6225_DIR, NULL, 0);
#endif
    if (err = hwmsen_get_convert(obj->hw->direction, &obj->cvt))
    {
        GSE_ERR("invalid direction: %d\n", obj->hw->direction);
        goto exit;
    }

    obj_i2c_data = obj;
    obj->client = client;
    new_client = obj->client;
    i2c_set_clientdata(new_client,obj);

    atomic_set(&obj->trace, 0);
    atomic_set(&obj->suspend, 0);

#ifdef CONFIG_MXC6225_LOWPASS
    if (obj->hw->firlen > C_MAX_FIR_LENGTH)
    {
        atomic_set(&obj->firlen, C_MAX_FIR_LENGTH);
    }
    else
    {
        atomic_set(&obj->firlen, obj->hw->firlen);
    }

    if (atomic_read(&obj->firlen) > 0)
    {
        atomic_set(&obj->fir_en, 1);
    }

#endif

    mxc6225_i2c_client = new_client;

    if (err = mxc6225_init_client(new_client, 1))
    {
        goto exit_init_failed;
    }


    if (err = misc_register(&mxc6225_device))
    {
        GSE_ERR("MYCAT mxc6225_device register failed\n");
        goto exit_misc_device_register_failed;
    }
#if !defined(MTK_AUTO_DETECT_ACCELEROMETER)
    if (err = mxc6225_create_attr(&mxc6225_gsensor_driver.driver))
    {
        GSE_ERR("MYCAT create attribute err = %d\n", err);
        goto exit_create_attr_failed;
    }
#else
    if ((err = mxc6225_create_attr(&mxc6225_init_info.platform_diver_addr->driver)))
    {
        GSE_ERR("MYCAT create attribute err = %d\n", err);
        goto exit_create_attr_failed;
    }
#endif
    sobj.self = obj;
    sobj.polling = 1;
    sobj.sensor_operate = mxc6225_operate;
    if (err = hwmsen_attach(ID_ACCELEROMETER, &sobj))
    {
        GSE_ERR("MYCAT attach fail = %d\n", err);
        goto exit_kfree;
    }

#ifdef CONFIG_HAS_EARLYSUSPEND
    obj->early_drv.level    = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,
                              obj->early_drv.suspend  = mxc6225_early_suspend,
                                                        obj->early_drv.resume   = mxc6225_late_resume,
                                                                                  register_early_suspend(&obj->early_drv);
#endif

#if defined(MTK_AUTO_DETECT_ACCELEROMETER)
    /* Add for auto detect feature */
    mxc6225_init_flag = 0;
#endif

    GSE_LOG("%s: OK\n", __func__);
    return 0;

exit_create_attr_failed:
    misc_deregister(&mxc6225_device);
exit_misc_device_register_failed:
exit_init_failed:
    //i2c_detach_client(new_client);
exit_kfree:
    kfree(obj);
exit:
    GSE_ERR("%s: err = %d\n", __func__, err);

#if defined(MTK_AUTO_DETECT_ACCELEROMETER)
    /* Add for auto detect feature */
    mxc6225_init_flag = -1;
#endif
    return err;
}

/*----------------------------------------------------------------------------*/
static int mxc6225_i2c_remove(struct i2c_client *client)
{
    int err = 0;
#if !defined(MTK_AUTO_DETECT_ACCELEROMETER)
    if (err = mxc6225_delete_attr(&mxc6225_gsensor_driver.driver))
    {
        GSE_ERR("mxc6225_delete_attr fail: %d\n", err);
    }
#else
    if ((err = mxc6225_delete_attr(&mxc6225_init_info.platform_diver_addr->driver)))
    {
        GSE_ERR("mxc6225_delete_attr fail: %d\n", err);
    }
#endif
    if (err = misc_deregister(&mxc6225_device))
    {
        GSE_ERR("misc_deregister fail: %d\n", err);
    }

    if (err = hwmsen_detach(ID_ACCELEROMETER))


        mxc6225_i2c_client = NULL;
    i2c_unregister_device(client);
    kfree(i2c_get_clientdata(client));
    return 0;
}

#if !defined(MTK_AUTO_DETECT_ACCELEROMETER)
/*----------------------------------------------------------------------------*/
static int mxc6225_probe(struct platform_device *pdev)
{
    struct acc_hw *hw = get_cust_acc_hw();
    GSE_FUN();

    MXC6225_power(hw, 1);
    //mxc6225_force[0] = hw->i2c_num;
    if (i2c_add_driver(&mxc6225_i2c_driver))
    {
        printk("MYCAT add driver error\n");
        return -1;
    }
    printk("MYCAT add driver OK\n");
    return 0;
}
/*----------------------------------------------------------------------------*/
static int mxc6225_remove(struct platform_device *pdev)
{
    struct acc_hw *hw = get_cust_acc_hw();

    GSE_FUN();
    MXC6225_power(hw, 0);
    i2c_del_driver(&mxc6225_i2c_driver);
    return 0;
}
/*----------------------------------------------------------------------------*/
static struct platform_driver mxc6225_gsensor_driver = {
            .probe      = mxc6225_probe,
                          .remove     = mxc6225_remove,
                                        .driver     = {
                                                          .name  = "gsensor",
                                                          //.owner = THIS_MODULE,
                                                      }
                                                  };
#else
static int  mxc6225_local_init(void)
{
    struct acc_hw *hw = get_cust_acc_hw();
    GSE_FUN();

    MXC6225_power(hw, 1);
    //mxc6225_force[0] = hw->i2c_num;
    if (i2c_add_driver(&mxc6225_i2c_driver))
    {
        GSE_ERR("add driver error\n");
        return -1;
    }

    if (-1 == mxc6225_init_flag)
    {
        return -1;
    }
    return 0;
}

static int mxc6225_remove(void)
{
    struct acc_hw *hw = get_cust_acc_hw();

    GSE_FUN();
    MXC6225_power(hw, 0);
    i2c_del_driver(&mxc6225_i2c_driver);
    return 0;
}

#endif

/*----------------------------------------------------------------------------*/
static int __init mxc6225_init(void)
{
    GSE_FUN();
#if !defined(MTK_AUTO_DETECT_ACCELEROMETER)
    i2c_register_board_info(1, &i2c_mxc6225, 1);
    if (platform_driver_register(&mxc6225_gsensor_driver))
    {
        GSE_ERR("failed to register driver");
        return -ENODEV;
    }
#else
    i2c_register_board_info(1, &i2c_mxc6225, 1);
    hwmsen_gsensor_add(&mxc6225_init_info);
#endif
    return 0;
}
/*----------------------------------------------------------------------------*/
static void __exit mxc6225_exit(void)
{
    GSE_FUN();
#if !defined(MTK_AUTO_DETECT_ACCELEROMETER)
    platform_driver_unregister(&mxc6225_gsensor_driver);
#endif
}
/*----------------------------------------------------------------------------*/
module_init(mxc6225_init);
module_exit(mxc6225_exit);
/*----------------------------------------------------------------------------*/
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MXC6225 I2C driver");
MODULE_AUTHOR("liejun.ma@nbbsw.com");
