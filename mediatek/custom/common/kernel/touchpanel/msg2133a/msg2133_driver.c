#include "tpd.h"
#include <linux/interrupt.h>
#include <cust_eint.h>
#include <linux/i2c.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/rtpm_prio.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/delay.h>

#include <linux/dma-mapping.h>


//#define TPD_PROXIMITY
#ifdef TPD_PROXIMITY
#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#endif

#include "tpd_custom_msg2133.h"
#include <mach/mt_pm_ldo.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_boot.h>
#include <cust_eint.h>
#include <linux/jiffies.h>

#include "cust_gpio_usage.h"

#ifdef TPD_HAVE_BUTTON 
static int tpd_keys_local[TPD_KEY_COUNT] = TPD_KEYS;
static int tpd_keys_dim_local[TPD_KEY_COUNT][4] = TPD_KEYS_DIM;
#endif
#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))
static int tpd_wb_start_local[TPD_WARP_CNT] = TPD_WARP_START;
static int tpd_wb_end_local[TPD_WARP_CNT]   = TPD_WARP_END;
#endif
#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
static int tpd_calmat_local[8]     = TPD_CALIBRATION_MATRIX;
static int tpd_def_calmat_local[8] = TPD_CALIBRATION_MATRIX;
#endif

extern struct tpd_device *tpd;

static struct i2c_client *i2c_client = NULL;
static struct task_struct *thread = NULL;

static DECLARE_WAIT_QUEUE_HEAD(waiter);


static void tpd_eint_interrupt_handler(void);
static int tpd_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int tpd_detect(struct i2c_client *client, struct i2c_board_info *info);
static int tpd_remove(struct i2c_client *client);
static int touch_event_handler(void *unused);

static int tpd_flag = 0;
static int point_num = 0;
static int p_point_num = 0;

//#define TP_PROXIMITY_SENSOR


#define TPD_OK 0
// debug macros
#if defined(TP_DEBUG)
#define SSL_PRINT(x...)		printk("MSG2133:"x)
#else
#define SSL_PRINT(format, args...)  do {} while (0)
#endif

#ifdef TP_PROXIMITY_SENSOR
char ps_data_state[1] = {0};
enum
{
    DISABLE_CTP_PS,
    ENABLE_CTP_PS,
    RESET_CTP_PS
};
char tp_proximity_state = DISABLE_CTP_PS;  //add by ningzhiyu
#endif
//////////////////////////////////////////////////////////////////////////////add by lan
#ifdef TPD_PROXIMITY
#define APS_ERR(fmt,arg...)           	printk("<<proximity>> "fmt"\n",##arg)
#define TPD_PROXIMITY_DEBUG(fmt,arg...) printk("<<proximity>> "fmt"\n",##arg)
#define TPD_PROXIMITY_DMESG(fmt,arg...) printk("<<proximity>> "fmt"\n",##arg)

static u8 tpd_proximity_flag 			= 0;
static u8 tpd_proximity_flag_one 		= 0; //add for tpd_proximity by wangdongfang
static u8 tpd_proximity_detect 			= 1;//0-->close ; 1--> far away

enum
{
    DISABLE_CTP_PS,
    ENABLE_CTP_PS,
    RESET_CTP_PS
};
#endif



#define TOUCH_ADDR_MSG20XX   0x4C
#define TOUCH_ADDR_MSG21XX   0x26

#define FW_ADDR_MSG20XX      0xC4
#define FW_UPDATE_ADDR_MSG20XX   0x92

#define GPIO_CTP_MSG2133_EN_PIN           GPIO_CTP_RST_PIN
#define GPIO_CTP_MSG2133_EN_PIN_M_GPIO    GPIO_MODE_00

#define GPIO_CTP_MSG2133_EINT_PIN           GPIO_CTP_EINT_PIN
#define GPIO_CTP_MSG2133_EINT_PIN_M_GPIO   GPIO_CTP_EINT_PIN_M_EINT

#define TPD_POINT_INFO_LEN      8
#define TPD_MAX_POINTS          2


struct touch_info
{
    unsigned short y[3];
    unsigned short x[3];
    unsigned short p[3];
    unsigned short count;
};

typedef struct
{
    unsigned short pos_x;
    unsigned short pos_y;
    unsigned short pos_x2;
    unsigned short pos_y2;
    unsigned short temp2;
    unsigned short temp;
    short dst_x;
    short dst_y;
    unsigned char checksum;

} SHORT_TOUCH_STATE;

struct msg_ts_priv 
{
    unsigned int buttons;
    int      prev_x[4];
    int	prev_y[4];
};

struct msg_ts_priv msg2133_priv;


static const struct i2c_device_id tpd_id[] = {{"msg2133", 0}, {}};
static struct i2c_board_info __initdata i2c_tpd={ I2C_BOARD_INFO("msg2133", (TOUCH_ADDR_MSG20XX>>1))};

static struct i2c_driver tpd_i2c_driver =
{
.driver = {
	 .name = "msg2133",
//	 .owner = THIS_MODULE,
  },
    .probe = tpd_probe,
    .remove = tpd_remove,
    .detect = tpd_detect,
    .driver.name = "msg2133",
    .id_table = tpd_id,

};


static void MSG2133_En_Pin_Out(u8 flag)
{
	if(flag==1)
	{
#ifdef EN_PIN_1_8_V
		mt_set_gpio_mode(GPIO_CTP_MSG2133_EN_PIN, GPIO_CTP_MSG2133_EN_PIN_M_GPIO);
		mt_set_gpio_dir(GPIO_CTP_MSG2133_EN_PIN, GPIO_DIR_IN);
		mt_set_gpio_pull_enable(GPIO_CTP_MSG2133_EN_PIN, GPIO_PULL_DISABLE);
		msleep(10);
		if(mt_get_gpio_in(GPIO_CTP_MSG2133_EN_PIN)==0)
		{
			mt_set_gpio_dir(GPIO_CTP_MSG2133_EN_PIN, GPIO_DIR_OUT);
			mt_set_gpio_out(GPIO_CTP_MSG2133_EN_PIN, GPIO_OUT_ONE);
			mt_set_gpio_pull_enable(GPIO_CTP_MSG2133_EN_PIN, GPIO_PULL_ENABLE);
			mt_set_gpio_pull_select(GPIO_CTP_MSG2133_EN_PIN, GPIO_PULL_UP);
		}
#else
		mt_set_gpio_mode(GPIO_CTP_MSG2133_EN_PIN, GPIO_CTP_MSG2133_EN_PIN_M_GPIO);
		mt_set_gpio_dir(GPIO_CTP_MSG2133_EN_PIN, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_CTP_MSG2133_EN_PIN, GPIO_OUT_ONE);
#endif
	}
	else
	{
		mt_set_gpio_mode(GPIO_CTP_MSG2133_EN_PIN, GPIO_CTP_MSG2133_EN_PIN_M_GPIO);
		mt_set_gpio_dir(GPIO_CTP_MSG2133_EN_PIN, GPIO_DIR_OUT);
#ifdef EN_PIN_1_8_V
		mt_set_gpio_pull_enable(GPIO_CTP_MSG2133_EN_PIN, GPIO_PULL_ENABLE);
		mt_set_gpio_pull_select(GPIO_CTP_MSG2133_EN_PIN, GPIO_PULL_DOWN);
#endif
		mt_set_gpio_out(GPIO_CTP_MSG2133_EN_PIN, GPIO_OUT_ZERO);
	}
}


#ifdef TP_FIRMWARE_UPDATE
/*FW UPDATE*/
#define DBG 		//printk
#define FW_ADDR_MSG2133   0x62//device address of msg2133
#define FW_UPDATE_ADDR_MSG2133   0x49
#define   DOWNLOAD_FIRMWARE_BUF_SIZE   59*1024
static U8 *download_firmware_buf = NULL;
static int FwDataCnt;
struct class *firmware_class;
struct device *firmware_cmd_dev;
static  char *fw_version;
//static int update_switch = 0;

#define MAX_CMD_LEN 255
static int i2c_master_send(struct i2c_client *client, char *buf ,int count)
{
	u8 *buf_dma = NULL;
	u32 old_addr = 0;
	int ret = 0;
	int retry = 3;

	if (count > MAX_CMD_LEN) {
		printk("[i2c_master_send_ext] exceed the max write length \n");
		return -1;
	}

	buf_dma= kmalloc(count,GFP_KERNEL);

	old_addr = client->addr;
	client->addr |= I2C_ENEXT_FLAG ;

	memcpy(buf_dma, buf, count);

	do {
		ret = i2c_master_send(client, (u8*)buf_dma, count);
		retry --;
		if (ret != count) {
			printk("[i2c_master_send_ext] Error sent I2C ret = %d\n", ret);
		}
	}while ((ret != count) && (retry > 0));

	client->addr = old_addr;
	kfree(buf_dma);

	return ret;

}


static int i2c_master_recv_ext(struct i2c_client *client, char *buf ,int count)
{
	u32 phyAddr = 0;
	u8  buf_dma[8] = {0};
	u32 old_addr = 0;
	int ret = 0;
	int retry = 3;
	int i = 0;
	u8  *buf_test ;
	buf_test = &buf_dma[0];

	old_addr = client->addr;
	client->addr |= I2C_ENEXT_FLAG ;

	DBG("[i2c_master_recv_ext] client->addr = %x\n", client->addr);

	do {
		ret = i2c_master_recv(client, buf_dma, count);
		retry --;
		if (ret != count) {
			DBG("[i2c_master_recv_ext] Error sent I2C ret = %d\n", ret);
		}
	}while ((ret != count) && (retry > 0));

	memcpy(buf, buf_dma, count);

	client->addr = old_addr;

	return ret;
}
void HalTscrCDevWriteI2CSeq(u8 addr, u8* data, u16 size)
{
	u8 addr_bak;

	addr_bak = i2c_client->addr;
	i2c_client->addr = addr;
	i2c_client->addr |= I2C_ENEXT_FLAG;
	i2c_master_send(i2c_client,data,size);
	i2c_client->addr = addr_bak;
}


void HalTscrCReadI2CSeq(u8 addr, u8* read_data, u8 size)
{
	u8 addr_bak;

	addr_bak = i2c_client->addr;
	i2c_client->addr = addr;
	i2c_client->addr |= I2C_ENEXT_FLAG;
	i2c_master_recv_ext(i2c_client,read_data,size);
	i2c_client->addr = addr_bak;
}


void Get_Chip_Version(void)
{

	unsigned char dbbus_tx_data[3];
	unsigned char dbbus_rx_data[2];

	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x1E;
	dbbus_tx_data[2] = 0xCE;
	HalTscrCDevWriteI2CSeq(FW_ADDR_MSG2133, &dbbus_tx_data[0], 3);
	HalTscrCReadI2CSeq(FW_ADDR_MSG2133, &dbbus_rx_data[0], 2);
	if (dbbus_rx_data[1] == 0)
	{
		// it is Catch2
		DBG("*** Catch2 ***\n");
		//FwVersion  = 2;// 2 means Catch2
	}
	else
	{
		// it is catch1
		DBG("*** Catch1 ***\n");
		//FwVersion  = 1;// 1 means Catch1
	}

}

void dbbusDWIICEnterSerialDebugMode()
{
	U8 data[5];

	// Enter the Serial Debug Mode
	data[0] = 0x53;
	data[1] = 0x45;
	data[2] = 0x52;
	data[3] = 0x44;
	data[4] = 0x42;
	HalTscrCDevWriteI2CSeq(FW_ADDR_MSG2133, data, 5);
}

void dbbusDWIICStopMCU()
{
	U8 data[1];

	// Stop the MCU
	data[0] = 0x37;
	HalTscrCDevWriteI2CSeq(FW_ADDR_MSG2133, data, 1);
}

void dbbusDWIICIICUseBus()
{
	U8 data[1];

	// IIC Use Bus
	data[0] = 0x35;
	HalTscrCDevWriteI2CSeq(FW_ADDR_MSG2133, data, 1);
}

void dbbusDWIICIICReshape()
{
	U8 data[1];

	// IIC Re-shape
	data[0] = 0x71;
	HalTscrCDevWriteI2CSeq(FW_ADDR_MSG2133, data, 1);
}

void dbbusDWIICIICNotUseBus()
{
	U8 data[1];

	// IIC Not Use Bus
	data[0] = 0x34;
	HalTscrCDevWriteI2CSeq(FW_ADDR_MSG2133, data, 1);
}

void dbbusDWIICNotStopMCU()
{
	U8 data[1];

	// Not Stop the MCU
	data[0] = 0x36;
	HalTscrCDevWriteI2CSeq(FW_ADDR_MSG2133, data, 1);
}

void dbbusDWIICExitSerialDebugMode()
{
	U8 data[1];

	// Exit the Serial Debug Mode
	data[0] = 0x45;
	HalTscrCDevWriteI2CSeq(FW_ADDR_MSG2133, data, 1);

	// Delay some interval to guard the next transaction
	//udelay ( 200 );        // delay about 0.2ms
}

void drvISP_EntryIspMode(void)
{
	U8 bWriteData[5] =
	{
		0x4D, 0x53, 0x54, 0x41, 0x52
	};

	HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG2133, bWriteData, 5);

	//udelay ( 1000 );        // delay about 1ms
}

U8 drvISP_Read(U8 n, U8* pDataToRead)    //First it needs send 0x11 to notify we want to get flash data back.
{
	U8 Read_cmd = 0x11;
	unsigned char dbbus_rx_data[2] = {0};
	HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG2133, &Read_cmd, 1);
	udelay(100);//10 zzf
	if (n == 1)
	{
		HalTscrCReadI2CSeq(FW_UPDATE_ADDR_MSG2133, &dbbus_rx_data[0], 2);
		*pDataToRead = dbbus_rx_data[0];
	}
	else
		HalTscrCReadI2CSeq(FW_UPDATE_ADDR_MSG2133, pDataToRead, n);

}

void drvISP_WriteEnable(void)
{
	U8 bWriteData[2] =
	{
		0x10, 0x06
	};
	U8 bWriteData1 = 0x12;
	HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG2133, bWriteData, 2);
	HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG2133, &bWriteData1, 1);
}


void drvISP_ExitIspMode(void)
{
	U8 bWriteData = 0x24;
	HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG2133, &bWriteData, 1);
}

U8 drvISP_ReadStatus()
{
	U8 bReadData = 0;
	U8 bWriteData[2] =
	{
		0x10, 0x05
	};
	U8 bWriteData1 = 0x12;

	udelay(100);
	HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG2133, bWriteData, 2);
	udelay(100);
	drvISP_Read(1, &bReadData);
	mdelay(10);//3->10 zzf
	HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG2133, &bWriteData1, 1);
	return bReadData;
}


void drvISP_BlockErase(U32 addr)
{
	U8 bWriteData[5] = { 0x00, 0x00, 0x00, 0x00, 0x00 };
	U8 bWriteData1 = 0x12;
	u32 timeOutCount=0;
	drvISP_WriteEnable();

	//Enable write status register
	bWriteData[0] = 0x10;
	bWriteData[1] = 0x50;
	HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG2133, bWriteData, 2);
	HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG2133, &bWriteData1, 1);

	//Write Status
	bWriteData[0] = 0x10;
	bWriteData[1] = 0x01;
	bWriteData[2] = 0x00;
	HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG2133, bWriteData, 3);
	HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG2133, &bWriteData1, 1);

	//Write disable
	bWriteData[0] = 0x10;
	bWriteData[1] = 0x04;
	HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG2133, bWriteData, 2);
	HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG2133, &bWriteData1, 1);

	udelay(100);
	timeOutCount=0;
	while ( ( drvISP_ReadStatus() & 0x01 ) == 0x01 )
	{
		timeOutCount++;
		if ( timeOutCount >= 10000 )
			break; /* around 1 sec timeout */
	}
	drvISP_WriteEnable();

	drvISP_ReadStatus();//zzf

	bWriteData[0] = 0x10;
	bWriteData[1] = 0xc7;//0xD8;        //Block Erase
	HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG2133, bWriteData, 2);
	HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG2133, &bWriteData1, 1);

	udelay(100);
	timeOutCount=0;
	while ( ( drvISP_ReadStatus() & 0x01 ) == 0x01 )
	{
		timeOutCount++;
		if ( timeOutCount >= 50000 )
			break; /* around 5 sec timeout */
	}
}

void drvISP_Program(U16 k, U8* pDataToWrite)
{
	U16 i = 0;
	U16 j = 0;
	//U16 n = 0;
	U8 TX_data[133];
	U8 bWriteData1 = 0x12;
	U32 addr = k * 1024;
	U32 timeOutCount = 0;

	for (j = 0; j < 8; j++)   //256*4 cycle
	{
		TX_data[0] = 0x10;
		TX_data[1] = 0x02;// Page Program CMD
		TX_data[2] = (addr + 128 * j) >> 16;
		TX_data[3] = (addr + 128 * j) >> 8;
		TX_data[4] = (addr + 128 * j);
		for (i = 0; i < 128; i++)
		{
			TX_data[5 + i] = pDataToWrite[j * 128 + i];
		}

		udelay(100);
		timeOutCount=0;
		while ( ( drvISP_ReadStatus() & 0x01 ) == 0x01 )
		{
			timeOutCount++;
			if ( timeOutCount >= 10000 )
				break; /* around 1 sec timeout */
		}

		drvISP_WriteEnable();
		HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG2133, TX_data, 133);   //write 256 byte per cycle
		HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG2133, &bWriteData1, 1);
	}
}

static ssize_t firmware_update_show(struct device *dev,
struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", fw_version);
}


static void tpd_hw_enable(unsigned int on);
static void tpd_hw_power(unsigned int on);
static ssize_t firmware_update_store(struct device *dev,
struct device_attribute *attr, const char *buf, size_t size)
{
	U8 i;
	U8 dbbus_tx_data[4];
	unsigned char dbbus_rx_data[2] = {0};
	int fd;
	//add mask interrupt
	//update_switch = 1;
	mt65xx_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
	mt65xx_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_EN, CUST_EINT_TOUCH_PANEL_POLARITY, 			NULL, 1);

	MSG2133_En_Pin_Out(0);
	mdelay(10);
	MSG2133_En_Pin_Out(1);
	mdelay(300);
	drvISP_EntryIspMode();
	drvISP_BlockErase(0x00000);
	mdelay(300);
	MSG2133_En_Pin_Out(0);
	mdelay(10);
	MSG2133_En_Pin_Out(1);

	mdelay(300);

#if 1
	// Enable slave's ISP ECO mode
	dbbusDWIICEnterSerialDebugMode();
	dbbusDWIICStopMCU();
	dbbusDWIICIICUseBus();
	dbbusDWIICIICReshape();

	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x08;
	dbbus_tx_data[2] = 0x0c;
	dbbus_tx_data[3] = 0x08;

	// Disable the Watchdog
	HalTscrCDevWriteI2CSeq(FW_ADDR_MSG2133, dbbus_tx_data, 4);


	//Get_Chip_Version();

	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x11;
	dbbus_tx_data[2] = 0xE2;
	dbbus_tx_data[3] = 0x00;

	HalTscrCDevWriteI2CSeq(FW_ADDR_MSG2133, dbbus_tx_data, 4);

	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x3C;
	dbbus_tx_data[2] = 0x60;
	dbbus_tx_data[3] = 0x55;
	HalTscrCDevWriteI2CSeq(FW_ADDR_MSG2133, dbbus_tx_data, 4);
	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x3C;
	dbbus_tx_data[2] = 0x61;
	dbbus_tx_data[3] = 0xAA;
	HalTscrCDevWriteI2CSeq(FW_ADDR_MSG2133, dbbus_tx_data, 4);

	//Stop MCU
	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x0F;
	dbbus_tx_data[2] = 0xE6;
	dbbus_tx_data[3] = 0x01;
	HalTscrCDevWriteI2CSeq(FW_ADDR_MSG2133, dbbus_tx_data, 4);

	//Enable SPI Pad
	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x1E;
	dbbus_tx_data[2] = 0x02;
	HalTscrCDevWriteI2CSeq(FW_ADDR_MSG2133, dbbus_tx_data, 3);
	HalTscrCReadI2CSeq(FW_ADDR_MSG2133, &dbbus_rx_data[0], 2);
	DBG("dbbus_rx_data[0]=0x%x", dbbus_rx_data[0]);
	dbbus_tx_data[3] = (dbbus_rx_data[0] | 0x20);  //Set Bit 5
	HalTscrCDevWriteI2CSeq(FW_ADDR_MSG2133, dbbus_tx_data, 4);

	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x1E;
	dbbus_tx_data[2] = 0x25;
	HalTscrCDevWriteI2CSeq(FW_ADDR_MSG2133, dbbus_tx_data, 3);
	dbbus_rx_data[0] = 0;
	dbbus_rx_data[1] = 0;
	HalTscrCReadI2CSeq(FW_ADDR_MSG2133, &dbbus_rx_data[0], 2);
	DBG("dbbus_rx_data[0]=0x%x", dbbus_rx_data[0]);
	dbbus_tx_data[3] = dbbus_rx_data[0] & 0xFC;  //Clear Bit 1,0
	HalTscrCDevWriteI2CSeq(FW_ADDR_MSG2133, dbbus_tx_data, 4);

	//WP overwrite
	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x1E;
	dbbus_tx_data[2] = 0x0E;
	dbbus_tx_data[3] = 0x02;
	HalTscrCDevWriteI2CSeq(FW_ADDR_MSG2133, dbbus_tx_data, 4);

	//set pin high
	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x1E;
	dbbus_tx_data[2] = 0x10;
	dbbus_tx_data[3] = 0x08;
	HalTscrCDevWriteI2CSeq(FW_ADDR_MSG2133, dbbus_tx_data, 4);

	//set FRO to 50M
	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x11;
	dbbus_tx_data[2] = 0xE2;
	HalTscrCDevWriteI2CSeq(FW_ADDR_MSG2133, dbbus_tx_data, 3);
	dbbus_rx_data[0] = 0;
	dbbus_rx_data[1] = 0;
	HalTscrCReadI2CSeq(FW_ADDR_MSG2133, &dbbus_rx_data[0], 2);
	//printk("dbbus_rx_data[0]=0x%x", dbbus_rx_data[0]);
	dbbus_tx_data[3] = dbbus_rx_data[0] & 0xF7;  //Clear Bit 1,0
	HalTscrCDevWriteI2CSeq(FW_ADDR_MSG2133, dbbus_tx_data, 4);

	dbbusDWIICIICNotUseBus();
	dbbusDWIICNotStopMCU();
	dbbusDWIICExitSerialDebugMode();

	///////////////////////////////////////
	// Start to load firmware
	///////////////////////////////////////
	drvISP_EntryIspMode();
	//drvISP_BlockErase(0x00000);//////zzf
	//DBG("FwVersion=2");
	for (i = 0; i < 59; i++)   // total  94 KB : 1 byte per R/W
	{
		if (download_firmware_buf == NULL)
			return 0;
		drvISP_Program(i,&download_firmware_buf[i*1024]);
		mdelay(1);
	}
#endif
	drvISP_ExitIspMode();
	FwDataCnt = 0;
	if (download_firmware_buf != NULL)
	{
		kfree(download_firmware_buf);
		download_firmware_buf = NULL;
	}

	MSG2133_En_Pin_Out(0);
	msleep(100);
	MSG2133_En_Pin_Out(1);
	msleep(500);
	//update_switch = 0;
	mt65xx_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_EN, CUST_EINT_POLARITY_HIGH, tpd_eint_interrupt_handler,1);
	mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);


	return size;
}

static DEVICE_ATTR(update, 0777, firmware_update_show, firmware_update_store);

static ssize_t firmware_version_show(struct device *dev,
struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", fw_version);
}

static ssize_t firmware_version_store(struct device *dev,
struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned char dbbus_tx_data[3];
	unsigned char dbbus_rx_data[4] ;
	unsigned short major=0, minor=0;
	fw_version = kzalloc(sizeof(char), GFP_KERNEL);

	//Get_Chip_Version();
	dbbus_tx_data[0] = 0x53;
	dbbus_tx_data[1] = 0x00;
	dbbus_tx_data[2] = 0x74;
	HalTscrCDevWriteI2CSeq(TOUCH_ADDR_MSG21XX, &dbbus_tx_data[0], 3);
	HalTscrCReadI2CSeq(TOUCH_ADDR_MSG21XX, &dbbus_rx_data[0], 4);
	major = (dbbus_rx_data[1]<<8)+dbbus_rx_data[0];
	minor = (dbbus_rx_data[3]<<8)+dbbus_rx_data[2];
	DBG("***major = %d ***\n", major);
	DBG("***minor = %d ***\n", minor);
	sprintf(fw_version,"%03d%03d", major, minor);
	DBG("***fw_version = %s ***\n", fw_version);

	return size;

}
static DEVICE_ATTR(version, 0777, firmware_version_show, firmware_version_store);


static ssize_t firmware_data_show(struct device *dev,
struct device_attribute *attr, char *buf)
{
	return FwDataCnt;
}

static ssize_t firmware_data_store(struct device *dev,
struct device_attribute *attr, const char *buf, size_t size)
{
	DBG("***FwDataCnt = %d ***\n", FwDataCnt);
	int i;
	if (download_firmware_buf == NULL) {
		download_firmware_buf = kzalloc(DOWNLOAD_FIRMWARE_BUF_SIZE, GFP_KERNEL);
		if (download_firmware_buf == NULL)
			return NULL;
	}
	if(FwDataCnt<59)
	{
		memcpy(&download_firmware_buf[FwDataCnt*1024], buf, 1024);
	}
	FwDataCnt++;
	return size;
}
static DEVICE_ATTR(data, 0777, firmware_data_show, firmware_data_store);

#endif





#ifdef TP_PROXIMITY_SENSOR
static ssize_t show_proximity_sensor(struct device *dev, struct device_attribute *attr, char *buf)
{
    static char temp=2;
        if (buf != NULL)
	    *buf = ps_data_state[0];
    if(temp!=*buf)
    {
    printk("proximity_sensor_show: buf=%d\n\n", *buf);
    temp=*buf;
    }    return 1;
}

static ssize_t store_proximity_sensor(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    U8 ps_store_data[4];

    if(buf != NULL && size != 0)
    {
        if(DISABLE_CTP_PS == *buf)
        {
            printk("DISABLE_CTP_PS buf=%d,size=%d\n", *buf, size);
            ps_store_data[0] = 0x52;
            ps_store_data[1] = 0x00;
	    ps_store_data[2] = 0x62;
            ps_store_data[3] = 0xa1;
            i2c_write(TOUCH_ADDR_MSG20XX, &ps_store_data[0], 4);
            msleep(2000);
            printk("RESET_CTP_PS buf=%d\n", *buf);
            mt65xx_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
            mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
            mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
            mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
            msleep(100);
            mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
            mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
            mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
            mdelay(500);
            mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
        }
        else if(ENABLE_CTP_PS == *buf)
        {
            printk("ENABLE_CTP_PS buf=%d,size=%d\n", *buf, size);
            ps_store_data[0] = 0x52;
            ps_store_data[1] = 0x00;
			ps_store_data[2] = 0x62;
            ps_store_data[3] = 0xa0;
            i2c_write(TOUCH_ADDR_MSG20XX, &ps_store_data[0], 4);
        }
                tp_proximity_state = *buf;
    }

    return size;
}
static DEVICE_ATTR(proximity_sensor, 0777, show_proximity_sensor, store_proximity_sensor);
#endif
////////////////////////////////////////////////////////////////////////////////////////////add by lan
#ifdef TPD_PROXIMITY

void i2c_write(u8 addr, u8* data, u16 size)
{
	u8 addr_bak;

	addr_bak = i2c_client->addr;
	i2c_client->addr = addr;
	i2c_client->addr |= I2C_ENEXT_FLAG;
	i2c_master_send(i2c_client,data,size);
	i2c_client->addr = addr_bak;
}

int tpd_read_ps(void)
{
	tpd_proximity_detect;
	return 0;    
}

static int tpd_get_ps_value(void)
{
	return tpd_proximity_detect;
}

static int tpd_enable_ps(int enable)
{
	u8 ps_store_data[4];
	if (enable)
	{
		ps_store_data[0] = 0x52;
		ps_store_data[1] = 0x00;
		ps_store_data[2] = 0x62;
		ps_store_data[3] = 0xa0;//0xa2//0xa4
		tpd_proximity_flag = 1;
		TPD_PROXIMITY_DEBUG("xxxxx mycat tpd_enable_ps function is on\n");
	}
	else
	{
		ps_store_data[0] = 0x52;
		ps_store_data[1] = 0x00;
		ps_store_data[2] = 0x62;
		ps_store_data[3] = 0xa1;
		tpd_proximity_flag = 0;
		TPD_PROXIMITY_DEBUG("xxxxx mycat tpd_enable_ps function is off\n");
	}
	i2c_write(TOUCH_ADDR_MSG21XX, &ps_store_data[0], 4);
	return 0;
}


static ssize_t show_proximity_sensor(struct device *dev, struct device_attribute *attr, char *buf)
{
	static char temp=2;
	int err = 0;
if((err = tpd_read_ps()))
{
	err = -1;
	return err;
}
else
{
       if (buf != NULL)
	    *buf = tpd_get_ps_value();
    if(temp!=*buf)
    {
    printk("xxxxx mycat proximity_sensor_show: buf=%d\n\n", *buf);
    temp=*buf;
    }
	return 1;
}

}

static ssize_t store_proximity_sensor(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{

    if(buf != NULL && size != 0)
    {
        if(DISABLE_CTP_PS == *buf)
        {
        
		if((tpd_enable_ps(0) != 0))
			{
			APS_ERR("enable ps fail: %d\n"); 
			return -1;
			}

        }
        else if(ENABLE_CTP_PS == *buf)
        {
        
		if((tpd_enable_ps(1) != 0))
			{
			APS_ERR("disable ps fail: %d\n"); 
			return -1;
			}
		
        }


	printk("xxxxx mycat tpd_proximity_state_on_off: buf=%d\n\n", *buf);
    }

    return size;
}
static DEVICE_ATTR(proximity_sensor, 0777, show_proximity_sensor, store_proximity_sensor);


#endif
////////////////////////////////////////////////////////////////////////////////////////////////////////////



static void tpd_down(int x, int y)
{
    input_report_abs(tpd->dev, ABS_PRESSURE, 128);
    input_report_key(tpd->dev, BTN_TOUCH, 1);
    input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 128);
    input_report_abs(tpd->dev, ABS_MT_WIDTH_MAJOR, 128);
    input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
    input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
    input_mt_sync(tpd->dev);
SSL_PRINT("MSG2133 D[%4d %4d %4d] ", x, y,1);

if (FACTORY_BOOT == get_boot_mode()|| RECOVERY_BOOT == get_boot_mode())
{ 
#ifdef TPD_HAVE_BUTTON
tpd_button(x,y,1);
#endif
if(y > TPD_RES_Y) //virtual key debounce to avoid android ANR issue
{
msleep(50);
printk("D virtual key \n");
}
}
}

static void tpd_up(int x, int y)
{
input_report_abs(tpd->dev, ABS_PRESSURE, 0);
input_report_key(tpd->dev, BTN_TOUCH, 0);
input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 0);
input_report_abs(tpd->dev, ABS_MT_WIDTH_MAJOR, 0);
input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
input_mt_sync(tpd->dev);
SSL_PRINT("MSG2133 U[%4d %4d %4d] ", x, y, 0);

if (FACTORY_BOOT == get_boot_mode()|| RECOVERY_BOOT == get_boot_mode())
{
#ifdef TPD_HAVE_BUTTON
tpd_button(x,y,0);
#endif
} 
}

unsigned char tpd_check_sum(unsigned char *pval)
{
    int i, sum = 0;

    for(i = 0; i < 7; i++)
    {
        sum += pval[i];
    }

    return (unsigned char)((-sum) & 0xFF);
}

static bool msg2033_i2c_read(char *pbt_buf, int dw_lenth)
{
    int ret;
    i2c_client->addr|=I2C_ENEXT_FLAG;
	
    ret = i2c_master_recv(i2c_client, pbt_buf, dw_lenth);

    if(ret <= 0)
    {
        printk("MYCAT msg_i2c_read_interface error ret=%d\n",ret);
        return false;
    }

    return true;
}

static int tpd_touchinfo(struct touch_info *cinfo)
{
    SHORT_TOUCH_STATE ShortTouchState;
    BYTE reg_val[8] = {0};
    unsigned int  temp = 0;
    u32 bitmask;
    int i;
#ifdef TPD_PROXIMITY
	int err;
	hwm_sensor_data sensor_data;
#endif
    msg2033_i2c_read(reg_val, 8);
    SSL_PRINT("received raw data from touch panel as following in fun %s\n", __func__);

    ShortTouchState.pos_x = ((reg_val[1] & 0xF0) << 4) | reg_val[2];
    ShortTouchState.pos_y = ((reg_val[1] & 0x0F) << 8) | reg_val[3];
    ShortTouchState.dst_x = ((reg_val[4] & 0xF0) << 4) | reg_val[5];
    ShortTouchState.dst_y = ((reg_val[4] & 0x0F) << 8) | reg_val[6];

    if((ShortTouchState.dst_x) & 0x0800)
    {
        ShortTouchState.dst_x |= 0xF000;
    }

    if((ShortTouchState.dst_y) & 0x0800)
    {
        ShortTouchState.dst_y |= 0xF000;
    }

    ShortTouchState.pos_x2 = ShortTouchState.pos_x + ShortTouchState.dst_x;
    ShortTouchState.pos_y2 = ShortTouchState.pos_y + ShortTouchState.dst_y;

    temp = tpd_check_sum(reg_val);
    SSL_PRINT("check_sum=%d,reg_val_7=%d\n", temp, reg_val[7]);

    if(temp == reg_val[7])
    {
        SSL_PRINT("TP_PS \nreg_val[1]=0x%x\nreg_val[2]=0x%x\nreg_val[3]=0x%x\nreg_val[4]=0x%x\nreg_val[5]=0x%x\nreg_val[6]=0x%x\nreg_val[7]=0x%x by cheehwa\n", reg_val[1], reg_val[2], reg_val[3], reg_val[4], reg_val[5], reg_val[6], reg_val[7]);

        if(reg_val[0] == 0x52) //CTP  ID
        {
        

#if (defined(TPD_HAVE_BUTTON) && !defined(SIMULATED_BUTTON))
	     if(reg_val[1] == 0xFF && reg_val[2] == 0xFF&& reg_val[3] == 0xFF&& reg_val[4] == 0xFF&& reg_val[6] == 0xFF)
	     {
		 	if(reg_val[5] == 0x0||reg_val[5] == 0xFF)
			{
				i = TPD_KEY_COUNT;
			}
		#ifdef TP_PROXIMITY_SENSOR
				else if(reg_val[5] == 0x80) // close to
				{
					SSL_PRINT("TP_PROXIMITY_SENSOR VVV by cheehwa\n");
					ps_data_state[0] = 1;
				}
				else if(reg_val[5] == 0x40) // leave
				{
					SSL_PRINT("TP_PROXIMITY_SENSOR XXX by cheehwa\n");
					ps_data_state[0] = 0;
				}
		#endif				
			else
			{
				bitmask=0;
				for(i=0;i<TPD_KEY_COUNT;i++)
				{
					bitmask=1<<i;
					if(reg_val[5] & bitmask) 
					{ 
						//button[i] down
							//printk("MYCAT Button%d down.\n",i);
							cinfo->x[0]  = tpd_keys_dim_local[i][0];
							cinfo->y[0]=  tpd_keys_dim_local[i][1];
							point_num = 1;
							break;
					}
					else
					{
						if(msg2133_priv.buttons & bitmask)
						{
							//button[i] up
							SSL_PRINT("Button%d up.\n",i);
						}
					}
				}
			}
			if(i == TPD_KEY_COUNT)
			{
				point_num = 0;
			}
			
			msg2133_priv.buttons = reg_val[5];
///////////////////////////////////////////////////////////////////////////add by lan
#ifdef TPD_PROXIMITY
TPD_PROXIMITY_DEBUG("mycat tpd_touchinfo reg_val[5] = %d\n", reg_val[5]);

				if (tpd_proximity_flag == 1)
				{
					if(reg_val[5]==0x40||reg_val[5]==0x80)
					{
						if (reg_val[5]==0x40)// leave
						{
							tpd_proximity_detect = 1;
						}
						else if (reg_val[5]==0x80)// close to
						{
							tpd_proximity_detect = 0;
						}
						//get raw data
						TPD_PROXIMITY_DEBUG(" ps change\n");
						//map and store data to hwm_sensor_data
						sensor_data.values[0] = tpd_get_ps_value();
						sensor_data.value_divide = 1;
						sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
						//let up layer to know
						if((err = hwmsen_get_interrupt_data(ID_PROXIMITY, &sensor_data)))
						{
							TPD_PROXIMITY_DEBUG("call hwmsen_get_interrupt_data fail = %d\n", err);
						}
						return  false;
					}
					else if(reg_val[5]==0xC0)
					{
						tpd_enable_ps(1);
						return  false;
					}
				}
#endif




	     }
#else
	     if(reg_val[1] == 0xFF && reg_val[2] == 0xFF&& reg_val[3] == 0xFF&& reg_val[4] == 0xFF&& reg_val[5] == 0xFF && reg_val[6] == 0xFF)
            {
		  	point_num = 0;
	     }
#endif
            else if((ShortTouchState.dst_x == 0) && (ShortTouchState.dst_y == 0))
            {
                #if defined(TPD_XY_INVERT)

		  #if defined(TPD_X_INVERT)
                cinfo->x[0] = (2048-ShortTouchState.pos_y) * TPD_RES_X / 2048;
		  #else
		  cinfo->x[0] = (ShortTouchState.pos_y) * TPD_RES_X / 2048;
		  #endif

		  #if defined(TPD_Y_INVERT)
                cinfo->y[0] = (2048-ShortTouchState.pos_x) * TPD_RES_Y / 2048;	
		  #else
                cinfo->y[0] = ShortTouchState.pos_x * TPD_RES_Y / 2048;	
		  #endif
		  
		  #else

		  #if defined(TPD_X_INVERT)
                cinfo->x[0] = (2048-ShortTouchState.pos_x) * TPD_RES_X / 2048;
		  #else
                cinfo->x[0] = ShortTouchState.pos_x * TPD_RES_X / 2048;
		  #endif

		  #if defined(TPD_Y_INVERT)
                cinfo->y[0] = (2048-ShortTouchState.pos_y) * TPD_RES_Y / 2048;
		  #else
                cinfo->y[0] = ShortTouchState.pos_y * TPD_RES_Y / 2048;
		  #endif
		  
		  #endif
                point_num = 1;
            }
            else
            {
                #if defined(TPD_XY_INVERT)

		  #if defined(TPD_X_INVERT)
		   cinfo->x[0] = (2048-ShortTouchState.pos_y) * TPD_RES_X / 2048;
		  #else
                cinfo->x[0] = ShortTouchState.pos_y * TPD_RES_X / 2048;
		  #endif

		  #if defined(TPD_Y_INVERT)
                cinfo->y[0] = (2048-ShortTouchState.pos_x) * TPD_RES_Y / 2048;
		  #else
                cinfo->y[0] = ShortTouchState.pos_x * TPD_RES_Y / 2048;
		  #endif

		  #if defined(TPD_X_INVERT)
                cinfo->x[1] = (2048-ShortTouchState.pos_y2) * TPD_RES_X / 2048;
		  #else
                cinfo->x[1] = ShortTouchState.pos_y2 * TPD_RES_X / 2048;
		  #endif

		  #if defined(TPD_Y_INVERT)
                cinfo->y[1] = (2048-ShortTouchState.pos_x2) * TPD_RES_Y / 2048;
		  #else
                cinfo->y[1] = ShortTouchState.pos_x2 * TPD_RES_Y / 2048;
		  #endif
		  
		  #else
		  
		  #if defined(TPD_X_INVERT)
		  cinfo->x[0] = (2048-ShortTouchState.pos_x) * TPD_RES_X / 2048;
		  #else
                cinfo->x[0] = ShortTouchState.pos_x * TPD_RES_X / 2048;
		  #endif

		  #if defined(TPD_Y_INVERT)
                cinfo->y[0] = (2048-ShortTouchState.pos_y) * TPD_RES_Y / 2048;
		  #else
                cinfo->y[0] = ShortTouchState.pos_y * TPD_RES_Y / 2048;
		  #endif

		  #if defined(TPD_X_INVERT)
                cinfo->x[1] = (2048-ShortTouchState.pos_x2) * TPD_RES_X / 2048;
		  #else
                cinfo->x[1] = ShortTouchState.pos_x2 * TPD_RES_X / 2048;
		  #endif

		  #if defined(TPD_Y_INVERT)
                cinfo->y[1] = (2048-ShortTouchState.pos_y2) * TPD_RES_Y / 2048;
		  #else
                cinfo->y[1] = ShortTouchState.pos_y2 * TPD_RES_Y / 2048;
		  #endif

		  #endif
                point_num = 2;
            }
        }
        return true;
    }
    else
    {
        SSL_PRINT("tpd_check_sum_ XXXX\n");
        return  false;
    }

}

static int touch_event_handler(void *unused)
{
    struct touch_info cinfo;
    int touch_state = 3;
    unsigned long time_eclapse;
    struct sched_param param = { .sched_priority = RTPM_PRIO_TPD };
    sched_setscheduler(current, SCHED_RR, &param);

    do
    {
        mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
        set_current_state(TASK_INTERRUPTIBLE);
        wait_event_interruptible(waiter, tpd_flag != 0);
        tpd_flag = 0;
        set_current_state(TASK_RUNNING);

        if(tpd_touchinfo(&cinfo))
        {
            //printk("MYCAT MSG_X = %d,MSG_Y = %d\n", cinfo.x[0], cinfo.y[0]);

            if(point_num == 1)
            {
                tpd_down(cinfo.x[0], cinfo.y[0]);
                SSL_PRINT("msg_press 1 point--->\n");
                input_sync(tpd->dev);
            }
            else if(point_num == 2)
            {
                //printk("MYCAT MSG_X2 = %d,MSG_Y2 = %d\n", cinfo.x[1], cinfo.y[1]);
                tpd_down(cinfo.x[0], cinfo.y[0]);
                tpd_down(cinfo.x[1], cinfo.y[1]);
                SSL_PRINT("msg_press 2 points--->\n");
                input_sync(tpd->dev);
            }
            else if(point_num == 0)
            {
            	//huangrunming
            	if (FACTORY_BOOT == get_boot_mode()|| RECOVERY_BOOT == get_boot_mode())
		{
                    SSL_PRINT("release --->\n");
                    tpd_up(cinfo.x[0], cinfo.y[0]);
                    //input_mt_sync(tpd->dev);
                    input_sync(tpd->dev);
                }else{
                    SSL_PRINT("release --->\n");
                    input_mt_sync(tpd->dev);
                    input_sync(tpd->dev);                
                }
            }
        }
    }
    while(!kthread_should_stop());

    return 0;
}

static int tpd_detect(struct i2c_client *client , struct i2c_board_info *info)
{
    strcpy(info->type, "msg2133");
    return 0;
}

static void tpd_eint_interrupt_handler(void)
{
    //printk("MYCAT MSG2133 TPD interrupt has been triggered\n");
    tpd_flag = 1;
    wake_up_interruptible(&waiter);
}

static int tpd_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int retval = TPD_OK;
	BYTE reg_testval[8] = {0};

    SSL_PRINT(" func: %s\n", __func__);
      char data;
        client->timing = 200;
    client->addr |= I2C_ENEXT_FLAG;
    i2c_client = client;
    printk("MYCAT In tpd_probe_ ,the i2c addr=0x%x", client->addr);

    if(TPD_POWER_SOURCE != MT65XX_POWER_NONE)
    {
	    hwPowerDown(TPD_POWER_SOURCE,"TP");
	    hwPowerOn(TPD_POWER_SOURCE,VOL_2800,"TP");
	    msleep(100);	
    }	
    mt_set_gpio_mode(GPIO_CTP_RST_PIN, 0);		//GPIO_CTP_RST_PIN
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);		
	mt_set_gpio_pull_enable(GPIO_CTP_RST_PIN, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO_CTP_RST_PIN, GPIO_PULL_UP);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
   
	msleep(100);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);


    mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT);
    mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_IN);
    mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO_CTP_EINT_PIN, GPIO_PULL_UP);
    mt65xx_eint_set_sens(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_SENSITIVE);
    mt65xx_eint_set_hw_debounce(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN);
    mt65xx_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_EN, CUST_EINT_TOUCH_PANEL_POLARITY, tpd_eint_interrupt_handler, 1);
    mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
    msleep(100);

    tpd_load_status = 1;
    thread = kthread_run(touch_event_handler, 0, TPD_DEVICE);

    if(IS_ERR(thread))
    {
        retval = PTR_ERR(thread);
        SSL_PRINT(TPD_DEVICE " failed to create kernel thread: %d\n", retval);
    }

    
        if (TRUE==msg2033_i2c_read(data,1))
        {
            //SSL_PRINT("Mstar 's TP\n");
            //SMC_SWITCH=1;
			 return -1;
        }
    
/*
		firmware_class = class_create(THIS_MODULE, "ms-touchscreen-msg20xx");
	
		if(IS_ERR(firmware_class))
		{
			pr_err("Failed to create class(firmware)!\n");
		}
	
		firmware_cmd_dev = device_create(firmware_class,NULL, 0, NULL, "device");
	
		if(IS_ERR(firmware_cmd_dev))
		{
			pr_err("Failed to create device(firmware_cmd_dev)!\n");
		}
	
		// version
		if(device_create_file(firmware_cmd_dev, &dev_attr_version) < 0)
		{
			pr_err("Failed to create device file(%s)!\n", dev_attr_version.attr.name);
		}
	
		// update
		if(device_create_file(firmware_cmd_dev, &dev_attr_update) < 0)
		{
			pr_err("Failed to create device file(%s)!\n", dev_attr_update.attr.name);
		}
	
		// data
		if(device_create_file(firmware_cmd_dev, &dev_attr_data) < 0)
		{
			pr_err("Failed to create device file(%s)!\n", dev_attr_data.attr.name);
		}
	
		dev_set_drvdata(firmware_cmd_dev, NULL);
#endif
//////////////////////////////////////////////////////////////////////add by lan
#ifdef TPD_PROXIMITY

	if(device_create_file(firmware_cmd_dev, &dev_attr_proximity_sensor) < 0) // /sys/class/mtk-tpd/device/proximity_sensor
		{
 			printk("xxxxx mycat Failed to create device file(%s)!\n", dev_attr_proximity_sensor.attr.name);
		}
#endif
*/
    printk("MYCAT Touch Panel Device Probe %s\n", (retval < TPD_OK) ? "FAIL" : "PASS");

    return 0;
}

static int tpd_remove(struct i2c_client *client)

{
/////////////////////////////////////////////////////////////////////////add by lan
#ifdef TPD_PROXIMITY	
	if (tpd_proximity_flag == 1)
	{
		if(tpd_proximity_flag_one == 1)
		{
			tpd_proximity_flag_one = 0;	
			TPD_DMESG(TPD_DEVICE " tpd_proximity_flag_one \n"); 
			return;
		}
	}
#endif	
#ifdef TPD_PROXIMITY	
	TPD_PROXIMITY_DEBUG("xxxxx mycat tpd_proximity_flag = %d\n", tpd_proximity_flag);
#endif


	
    SSL_PRINT(" func: %s\n", __func__);
    return 0;
}


static int tpd_local_init(void)
{
    SSL_PRINT(" func: %s\n", __func__);
    printk(" MSG2133 I2C Touchscreen Driver (Built %s @ %s)\n", __DATE__, __TIME__);

    if(i2c_add_driver(&tpd_i2c_driver) != 0)
    {
        printk("unable to add i2c driver.\n");
        return -1;
    }

#ifdef TPD_HAVE_BUTTON
    tpd_button_setting(TPD_KEY_COUNT, tpd_keys_local, tpd_keys_dim_local);// initialize tpd button data
#endif
#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))
    TPD_DO_WARP = 1;
    memcpy(tpd_wb_start, tpd_wb_start_local, TPD_WARP_CNT * 4);
    memcpy(tpd_wb_end, tpd_wb_start_local, TPD_WARP_CNT * 4);
#endif
#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
    memcpy(tpd_calmat, tpd_def_calmat_local, 8 * 4);
    memcpy(tpd_def_calmat, tpd_def_calmat_local, 8 * 4);
#endif
    SSL_PRINT("end %s, %d\n", __FUNCTION__, __LINE__);
    tpd_type_cap = 1;
    msg2133_priv.buttons=0;
    return 0;
}

static int tpd_resume(struct i2c_client *client)
{
    int retval = TPD_OK;
    SSL_PRINT(" func: %s\n", __func__);

#ifdef TP_FIRMWARE_UPDATE
	if(update_switch==1)
	{
		return 0;
	}
#endif

	
#ifdef TPD_CLOSE_POWER_IN_SLEEP
    hwPowerOn(TPD_POWER_SOURCE, VOL_3300, "TP");
#else
      mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
      mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
      mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
#endif
    msleep(200);
    mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
    return retval;
}

static int tpd_suspend(struct i2c_client *client, pm_message_t message)
{
    int retval = TPD_OK;
    static char data = 0x3;
    printk("TPD enter sleep\n");
/////////////////////////////////////////////////////////////////add by lan
#ifdef TPD_PROXIMITY
	if (tpd_proximity_flag == 1)
	{
		tpd_proximity_flag_one = 1;	
		return;
	}
#endif
#ifdef TPD_PROXIMITY	 
	TPD_PROXIMITY_DEBUG("xxxxx mycat tpd_proximity_flag = %d\n", tpd_proximity_flag);
#endif


#ifdef TP_FIRMWARE_UPDATE
		if(update_switch==1)
		{
			return 0;
		}
#endif

	
#ifdef TP_PROXIMITY_SENSOR
        if (tp_proximity_state == ENABLE_CTP_PS){
            // resume ......
            return retval;
        }
#endif
    mt65xx_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
    mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
    
#ifdef TPD_CLOSE_POWER_IN_SLEEP
    hwPowerDown(TPD_POWER_SOURCE, "TP");
#else
    //i2c_smbus_write_i2c_block_data(i2c_client, 0xA5, 1, &data);  //TP enter sleep mode
    //mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
    //mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    //mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
#endif
    return retval;
}


static struct tpd_driver_t tpd_device_driver =
{
    .tpd_device_name = "MSG2133",
    .tpd_local_init = tpd_local_init,
    .suspend = tpd_suspend,
    .resume = tpd_resume,
#ifdef TPD_HAVE_BUTTON
    .tpd_have_button = 1,
#else
    .tpd_have_button = 0,
#endif
};
/* called when loaded into kernel */
static int __init tpd_driver_init(void)
{
    SSL_PRINT(" func: %s\n", __func__);
    i2c_register_board_info(1, &i2c_tpd, 1);
    if(tpd_driver_add(&tpd_device_driver) < 0)
    {
        printk("add MSG2133 driver failed\n");
    }

    return 0;
}

/* should never be called */
static void __exit tpd_driver_exit(void)
{
    SSL_PRINT(" func: %s\n", __func__);
    tpd_driver_remove(&tpd_device_driver);
}

module_init(tpd_driver_init);
module_exit(tpd_driver_exit);


