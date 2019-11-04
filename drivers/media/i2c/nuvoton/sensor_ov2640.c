#include <linux/delay.h>
#include <linux/module.h>

#include "nuc970_cap.h"

#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-event.h>
#include <media/i2c-addr.h>



static struct nuvoton_vin_sensor ov2640;

struct OV_RegValue{
	__u16	uRegAddr;
	__u8	uValue;
};

#define VAL_SET(x, mask, rshift, lshift)  \
		((((x) >> rshift) & mask) << lshift)
/*
 * DSP registers
 * register offset for BANK_SEL == BANK_SEL_DSP
 */
#define R_BYPASS    0x05 /* Bypass DSP */
#define   R_BYPASS_DSP_BYPAS    0x01 /* Bypass DSP, sensor out directly */
#define   R_BYPASS_USE_DSP      0x00 /* Use the internal DSP */
#define QS          0x44 /* Quantization Scale Factor */
#define CTRLI       0x50
#define   CTRLI_LP_DP           0x80
#define   CTRLI_ROUND           0x40
#define   CTRLI_V_DIV_SET(x)    VAL_SET(x, 0x3, 0, 3)
#define   CTRLI_H_DIV_SET(x)    VAL_SET(x, 0x3, 0, 0)
#define HSIZE       0x51 /* H_SIZE[7:0] (real/4) */
#define   HSIZE_SET(x)          VAL_SET(x, 0xFF, 2, 0)
#define VSIZE       0x52 /* V_SIZE[7:0] (real/4) */
#define   VSIZE_SET(x)          VAL_SET(x, 0xFF, 2, 0)
#define XOFFL       0x53 /* OFFSET_X[7:0] */
#define   XOFFL_SET(x)          VAL_SET(x, 0xFF, 0, 0)
#define YOFFL       0x54 /* OFFSET_Y[7:0] */
#define   YOFFL_SET(x)          VAL_SET(x, 0xFF, 0, 0)
#define VHYX        0x55 /* Offset and size completion */
#define   VHYX_VSIZE_SET(x)     VAL_SET(x, 0x1, (8+2), 7)
#define   VHYX_HSIZE_SET(x)     VAL_SET(x, 0x1, (8+2), 3)
#define   VHYX_YOFF_SET(x)      VAL_SET(x, 0x3, 8, 4)
#define   VHYX_XOFF_SET(x)      VAL_SET(x, 0x3, 8, 0)
#define DPRP        0x56
#define TEST        0x57 /* Horizontal size completion */
#define   TEST_HSIZE_SET(x)     VAL_SET(x, 0x1, (9+2), 7)
#define ZMOW        0x5A /* Zoom: Out Width  OUTW[7:0] (real/4) */
#define   ZMOW_OUTW_SET(x)      VAL_SET(x, 0xFF, 2, 0)
#define ZMOH        0x5B /* Zoom: Out Height OUTH[7:0] (real/4) */
#define   ZMOH_OUTH_SET(x)      VAL_SET(x, 0xFF, 2, 0)
#define ZMHH        0x5C /* Zoom: Speed and H&W completion */
#define   ZMHH_ZSPEED_SET(x)    VAL_SET(x, 0x0F, 0, 4)
#define   ZMHH_OUTH_SET(x)      VAL_SET(x, 0x1, (8+2), 2)
#define   ZMHH_OUTW_SET(x)      VAL_SET(x, 0x3, (8+2), 0)
#define BPADDR      0x7C /* SDE Indirect Register Access: Address */
#define BPDATA      0x7D /* SDE Indirect Register Access: Data */
#define CTRL2       0x86 /* DSP Module enable 2 */
#define   CTRL2_DCW_EN          0x20
#define   CTRL2_SDE_EN          0x10
#define   CTRL2_UV_ADJ_EN       0x08
#define   CTRL2_UV_AVG_EN       0x04
#define   CTRL2_CMX_EN          0x01
#define CTRL3       0x87 /* DSP Module enable 3 */
#define   CTRL3_BPC_EN          0x80
#define   CTRL3_WPC_EN          0x40
#define SIZEL       0x8C /* Image Size Completion */
#define   SIZEL_HSIZE8_11_SET(x) VAL_SET(x, 0x1, 11, 6)
#define   SIZEL_HSIZE8_SET(x)    VAL_SET(x, 0x7, 0, 3)
#define   SIZEL_VSIZE8_SET(x)    VAL_SET(x, 0x7, 0, 0)
#define HSIZE8      0xC0 /* Image Horizontal Size HSIZE[10:3] */
#define   HSIZE8_SET(x)         VAL_SET(x, 0xFF, 3, 0)
#define VSIZE8      0xC1 /* Image Vertical Size VSIZE[10:3] */
#define   VSIZE8_SET(x)         VAL_SET(x, 0xFF, 3, 0)
#define CTRL0       0xC2 /* DSP Module enable 0 */
#define   CTRL0_AEC_EN       0x80
#define   CTRL0_AEC_SEL      0x40
#define   CTRL0_STAT_SEL     0x20
#define   CTRL0_VFIRST       0x10
#define   CTRL0_YUV422       0x08
#define   CTRL0_YUV_EN       0x04
#define   CTRL0_RGB_EN       0x02
#define   CTRL0_RAW_EN       0x01
#define CTRL1       0xC3 /* DSP Module enable 1 */
#define   CTRL1_CIP          0x80
#define   CTRL1_DMY          0x40
#define   CTRL1_RAW_GMA      0x20
#define   CTRL1_DG           0x10
#define   CTRL1_AWB          0x08
#define   CTRL1_AWB_GAIN     0x04
#define   CTRL1_LENC         0x02
#define   CTRL1_PRE          0x01
 /*      REG 0xC7 (unknown name): affects Auto White Balance (AWB)
  *	  AWB_OFF            0x40
  *	  AWB_SIMPLE         0x10
  *	  AWB_ON             0x00	(Advanced AWB ?) */
#define R_DVP_SP    0xD3 /* DVP output speed control */
#define   R_DVP_SP_AUTO_MODE 0x80
#define   R_DVP_SP_DVP_MASK  0x3F /* DVP PCLK = sysclk (48)/[6:0] (YUV0);
				   *          = sysclk (48)/(2*[6:0]) (RAW);*/
#define IMAGE_MODE  0xDA /* Image Output Format Select */
#define   IMAGE_MODE_Y8_DVP_EN   0x40
#define   IMAGE_MODE_JPEG_EN     0x10
#define   IMAGE_MODE_YUV422      0x00
#define   IMAGE_MODE_RAW10       0x04 /* (DVP) */
#define   IMAGE_MODE_RGB565      0x08
#define   IMAGE_MODE_HREF_VSYNC  0x02 /* HREF timing select in DVP JPEG output
					   * mode (0 for HREF is same as sensor) */
#define   IMAGE_MODE_LBYTE_FIRST 0x01 /* Byte swap enable for DVP
					   *    1: Low byte first UYVY (C2[4] =0)
					   *        VYUY (C2[4] =1)
					   *    0: High byte first YUYV (C2[4]=0)
					   *        YVYU (C2[4] = 1) */
#define RESET       0xE0 /* Reset */
#define   RESET_MICROC       0x40
#define   RESET_SCCB         0x20
#define   RESET_JPEG         0x10
#define   RESET_DVP          0x04
#define   RESET_IPU          0x02
#define   RESET_CIF          0x01
#define REGED       0xED /* Register ED */
#define   REGED_CLK_OUT_DIS  0x10
#define MS_SP       0xF0 /* SCCB Master Speed */
#define SS_ID       0xF7 /* SCCB Slave ID */
#define SS_CTRL     0xF8 /* SCCB Slave Control */
#define   SS_CTRL_ADD_AUTO_INC  0x20
#define   SS_CTRL_EN            0x08
#define   SS_CTRL_DELAY_CLK     0x04
#define   SS_CTRL_ACC_EN        0x02
#define   SS_CTRL_SEN_PASS_THR  0x01
#define MC_BIST     0xF9 /* Microcontroller misc register */
#define   MC_BIST_RESET           0x80 /* Microcontroller Reset */
#define   MC_BIST_BOOT_ROM_SEL    0x40
#define   MC_BIST_12KB_SEL        0x20
#define   MC_BIST_12KB_MASK       0x30
#define   MC_BIST_512KB_SEL       0x08
#define   MC_BIST_512KB_MASK      0x0C
#define   MC_BIST_BUSY_BIT_R      0x02
#define   MC_BIST_MC_RES_ONE_SH_W 0x02
#define   MC_BIST_LAUNCH          0x01
#define BANK_SEL    0xFF /* Register Bank Select */
#define   BANK_SEL_DSP     0x00
#define   BANK_SEL_SENS    0x01



#define GAIN        0x00 /* AGC - Gain control gain setting */
#define COM1        0x03 /* Common control 1 */
#define   COM1_1_DUMMY_FR          0x40
#define   COM1_3_DUMMY_FR          0x80
#define   COM1_7_DUMMY_FR          0xC0
#define   COM1_VWIN_LSB_UXGA       0x0F
#define   COM1_VWIN_LSB_SVGA       0x0A
#define   COM1_VWIN_LSB_CIF        0x06
#define REG04       0x04 /* Register 04 */
#define   REG04_DEF             0x20 /* Always set */
#define   REG04_HFLIP_IMG       0x80 /* Horizontal mirror image ON/OFF */
#define   REG04_VFLIP_IMG       0x40 /* Vertical flip image ON/OFF */
#define   REG04_VREF_EN         0x10
#define   REG04_HREF_EN         0x08
#define   REG04_AEC_SET(x)      VAL_SET(x, 0x3, 0, 0)
#define REG08       0x08 /* Frame Exposure One-pin Control Pre-charge Row Num */
#define COM2        0x09 /* Common control 2 */
#define   COM2_SOFT_SLEEP_MODE  0x10 /* Soft sleep mode */
/* Output drive capability */
#define   COM2_OCAP_Nx_SET(N)   (((N) - 1) & 0x03) /* N = [1x .. 4x] */
#define PID         0x0A /* Product ID Number MSB */
#define VER         0x0B /* Product ID Number LSB */
#define COM3        0x0C /* Common control 3 */
#define   COM3_BAND_50H        0x04 /* 0 For Banding at 60H */
#define   COM3_BAND_AUTO       0x02 /* Auto Banding */
#define   COM3_SING_FR_SNAPSH  0x01 /* 0 For enable live video output after the
					 * snapshot sequence*/
#define AEC         0x10 /* AEC[9:2] Exposure Value */
#define CLKRC       0x11 /* Internal clock */
#define   CLKRC_EN             0x80
#define   CLKRC_DIV_SET(x)     (((x) - 1) & 0x1F) /* CLK = XVCLK/(x) */
#define COM7        0x12 /* Common control 7 */
#define   COM7_SRST            0x80 /* Initiates system reset. All registers are
					 * set to factory default values after which
					 * the chip resumes normal operation */
#define   COM7_RES_UXGA        0x00 /* Resolution selectors for UXGA */
#define   COM7_RES_SVGA        0x40 /* SVGA */
#define   COM7_RES_CIF         0x20 /* CIF */
#define   COM7_ZOOM_EN         0x04 /* Enable Zoom mode */
#define   COM7_COLOR_BAR_TEST  0x02 /* Enable Color Bar Test Pattern */
#define COM8        0x13 /* Common control 8 */
#define   COM8_DEF             0xC0
#define   COM8_BNDF_EN         0x20 /* Banding filter ON/OFF */
#define   COM8_AGC_EN          0x04 /* AGC Auto/Manual control selection */
#define   COM8_AEC_EN          0x01 /* Auto/Manual Exposure control */
#define COM9        0x14 /* Common control 9
			  * Automatic gain ceiling - maximum AGC value [7:5]*/
#define   COM9_AGC_GAIN_2x     0x00 /* 000 :   2x */
#define   COM9_AGC_GAIN_4x     0x20 /* 001 :   4x */
#define   COM9_AGC_GAIN_8x     0x40 /* 010 :   8x */
#define   COM9_AGC_GAIN_16x    0x60 /* 011 :  16x */
#define   COM9_AGC_GAIN_32x    0x80 /* 100 :  32x */
#define   COM9_AGC_GAIN_64x    0xA0 /* 101 :  64x */
#define   COM9_AGC_GAIN_128x   0xC0 /* 110 : 128x */
#define COM10       0x15 /* Common control 10 */
#define   COM10_PCLK_HREF      0x20 /* PCLK output qualified by HREF */
#define   COM10_PCLK_RISE      0x10 /* Data is updated at the rising edge of
					 * PCLK (user can latch data at the next
					 * falling edge of PCLK).
					 * 0 otherwise. */
#define   COM10_HREF_INV       0x08 /* Invert HREF polarity:
					 * HREF negative for valid data*/
#define   COM10_VSINC_INV      0x02 /* Invert VSYNC polarity */
#define HSTART      0x17 /* Horizontal Window start MSB 8 bit */
#define HEND        0x18 /* Horizontal Window end MSB 8 bit */
#define VSTART      0x19 /* Vertical Window start MSB 8 bit */
#define VEND        0x1A /* Vertical Window end MSB 8 bit */
#define MIDH        0x1C /* Manufacturer ID byte - high */
#define MIDL        0x1D /* Manufacturer ID byte - low  */
#define AEW         0x24 /* AGC/AEC - Stable operating region (upper limit) */
#define AEB         0x25 /* AGC/AEC - Stable operating region (lower limit) */
#define VV          0x26 /* AGC/AEC Fast mode operating region */
#define   VV_HIGH_TH_SET(x)      VAL_SET(x, 0xF, 0, 4)
#define   VV_LOW_TH_SET(x)       VAL_SET(x, 0xF, 0, 0)
#define REG2A       0x2A /* Dummy pixel insert MSB */
#define FRARL       0x2B /* Dummy pixel insert LSB */
#define ADDVFL      0x2D /* LSB of insert dummy lines in Vertical direction */
#define ADDVFH      0x2E /* MSB of insert dummy lines in Vertical direction */
#define YAVG        0x2F /* Y/G Channel Average value */
#define REG32       0x32 /* Common Control 32 */
#define   REG32_PCLK_DIV_2    0x80 /* PCLK freq divided by 2 */
#define   REG32_PCLK_DIV_4    0xC0 /* PCLK freq divided by 4 */
#define ARCOM2      0x34 /* Zoom: Horizontal start point */
#define REG45       0x45 /* Register 45 */
#define FLL         0x46 /* Frame Length Adjustment LSBs */
#define FLH         0x47 /* Frame Length Adjustment MSBs */
#define COM19       0x48 /* Zoom: Vertical start point */
#define ZOOMS       0x49 /* Zoom: Vertical start point */
#define COM22       0x4B /* Flash light control */
#define COM25       0x4E /* For Banding operations */
#define   COM25_50HZ_BANDING_AEC_MSBS_MASK      0xC0 /* 50Hz Bd. AEC 2 MSBs */
#define   COM25_60HZ_BANDING_AEC_MSBS_MASK      0x30 /* 60Hz Bd. AEC 2 MSBs */
#define   COM25_50HZ_BANDING_AEC_MSBS_SET(x)    VAL_SET(x, 0x3, 8, 6)
#define   COM25_60HZ_BANDING_AEC_MSBS_SET(x)    VAL_SET(x, 0x3, 8, 4)
#define BD50        0x4F /* 50Hz Banding AEC 8 LSBs */
#define   BD50_50HZ_BANDING_AEC_LSBS_SET(x)     VAL_SET(x, 0xFF, 0, 0)
#define BD60        0x50 /* 60Hz Banding AEC 8 LSBs */
#define   BD60_60HZ_BANDING_AEC_LSBS_SET(x)     VAL_SET(x, 0xFF, 0, 0)
#define REG5A       0x5A /* 50/60Hz Banding Maximum AEC Step */
#define   BD50_MAX_AEC_STEP_MASK         0xF0 /* 50Hz Banding Max. AEC Step */
#define   BD60_MAX_AEC_STEP_MASK         0x0F /* 60Hz Banding Max. AEC Step */
#define   BD50_MAX_AEC_STEP_SET(x)       VAL_SET((x - 1), 0x0F, 0, 4)
#define   BD60_MAX_AEC_STEP_SET(x)       VAL_SET((x - 1), 0x0F, 0, 0)
#define REG5D       0x5D /* AVGsel[7:0],   16-zone average weight option */
#define REG5E       0x5E /* AVGsel[15:8],  16-zone average weight option */
#define REG5F       0x5F /* AVGsel[23:16], 16-zone average weight option */
#define REG60       0x60 /* AVGsel[31:24], 16-zone average weight option */
#define HISTO_LOW   0x61 /* Histogram Algorithm Low Level */
#define HISTO_HIGH  0x62 /* Histogram Algorithm High Level */
#define ENDMARKER { 0xff, 0xff }
					 /*
					  * ID
					  */
#define MANUFACTURER_ID	0x7FA2
#define PID_OV2640	0x2642
#define VERSION(pid, ver) ((pid << 8) | (ver & 0xFF))



#define OV2640_MID				0X7FA2
#define OV2640_PID				0X2642




#define SCCB_WR_Reg(A,B) i2c_smbus_write_byte_data(pi2c,A,B)
#define SCCB_RD_Reg(A) i2c_smbus_read_byte_data(pi2c,A)

struct regval_list {
	u8 reg_num;
	u8 value;
};




#define _REG_TABLE_SIZE(nTableName)	sizeof(nTableName)/sizeof(struct OV_RegValue)
/*
static struct OV_RegValue Init_RegValue[] = 
{	
};

*/
/************  I2C  *****************/
static struct i2c_client *save_client;
static char sensor_inited = 0;

#define UXGA_WIDTH (1600)
#define UXGA_HEIGHT (1200)

#define VGA_WIDTH (640)
#define VGA_HEIGHT (480)

#if 0
#define WIDTH_INIT VGA_WIDTH
#define HEIGTH_INIT VGA_HEIGHT
#else
#define WIDTH_INIT UXGA_WIDTH
#define HEIGTH_INIT UXGA_HEIGHT
#endif

static int sensor_probe(struct i2c_client *client,const struct i2c_device_id *did)
{
	printk("i2c sensor probe......\n");
	ENTRY();
	sensor_inited = 1;
	client->flags = I2C_CLIENT_SCCB;
	save_client = client;
	LEAVE();
	return 0;
}
static int sensor_remove(struct i2c_client *client)
{	
	ENTRY();
	LEAVE();
	return 0;
}

static int ov2640_init(struct nuvoton_vin_device* cam)
{
	int err = 0;
	ENTRY();
	LEAVE();		
	return err;
}

static struct nuvoton_vin_sensor ov2640 = {
	.name = "ov2640",
	.init = &ov2640_init,
	.infmtord = (INORD_YUYV | INFMT_YCbCr | INTYPE_CCIR601),
	//.polarity = (VSP_HI | HSP_LO | PCLKP_HI),
	.polarity = (0 | 0 | PCLKP_HI),
	.cropstart = (0 | 0 << 16), /*( Vertical | Horizontal<<16 ) */
	//.cropstart = (1 | 0 << 16), /*( Vertical | Horizontal<<16 ) */

	.cropcap = {
		.bounds = {
			.left = 0,
			.top = 0,
			.width = WIDTH_INIT,
			.height = HEIGTH_INIT,
		},
		.defrect = {
			.left = 0,
			.top = 0,
			.width = WIDTH_INIT,
			.height = HEIGTH_INIT,
		},
	},
	.pix_format = {
		.width = WIDTH_INIT,
		.height = HEIGTH_INIT,
		.pixelformat = V4L2_PIX_FMT_YUYV,
		.priv = 16,
		//.colorspace = V4L2_COLORSPACE_JPEG,
		.colorspace = V4L2_COLORSPACE_470_SYSTEM_BG,
	},
};


static const struct regval_list ov2640_init_regs[] = {
	{ BANK_SEL, BANK_SEL_DSP },
	{ 0x2c,   0xff },
	{ 0x2e,   0xdf },
	{ BANK_SEL, BANK_SEL_SENS },
	{ 0x3c,   0x32 },
	{ CLKRC,  CLKRC_DIV_SET(1) },
	{ COM2,   COM2_OCAP_Nx_SET(3) },
	{ REG04,  REG04_DEF | REG04_HREF_EN },
	{ COM8,   COM8_DEF | COM8_BNDF_EN | COM8_AGC_EN | COM8_AEC_EN },
	{ COM9,   COM9_AGC_GAIN_8x | 0x08},
	{ 0x2c,   0x0c },
	{ 0x33,   0x78 },
	{ 0x3a,   0x33 },
	{ 0x3b,   0xfb },
	{ 0x3e,   0x00 },
	{ 0x43,   0x11 },
	{ 0x16,   0x10 },
	{ 0x39,   0x02 },
	{ 0x35,   0x88 },
	{ 0x22,   0x0a },
	{ 0x37,   0x40 },
	{ 0x23,   0x00 },
	{ ARCOM2, 0xa0 },
	{ 0x06,   0x02 },
	{ 0x06,   0x88 },
	{ 0x07,   0xc0 },
	{ 0x0d,   0xb7 },
	{ 0x0e,   0x01 },
	{ 0x4c,   0x00 },
	{ 0x4a,   0x81 },
	{ 0x21,   0x99 },
	{ AEW,    0x40 },
	{ AEB,    0x38 },
	{ VV,     VV_HIGH_TH_SET(0x08) | VV_LOW_TH_SET(0x02) },
	{ 0x5c,   0x00 },
	{ 0x63,   0x00 },
	{ FLL,    0x22 },
	{ COM3,   0x38 | COM3_BAND_AUTO },
	{ REG5D,  0x55 },
	{ REG5E,  0x7d },
	{ REG5F,  0x7d },
	{ REG60,  0x55 },
	{ HISTO_LOW,   0x70 },
	{ HISTO_HIGH,  0x80 },
	{ 0x7c,   0x05 },
	{ 0x20,   0x80 },
	{ 0x28,   0x30 },
	{ 0x6c,   0x00 },
	{ 0x6d,   0x80 },
	{ 0x6e,   0x00 },
	{ 0x70,   0x02 },
	{ 0x71,   0x94 },
	{ 0x73,   0xc1 },
	{ 0x3d,   0x34 },
	{ COM7,   COM7_RES_UXGA | COM7_ZOOM_EN },
	{ REG5A,  BD50_MAX_AEC_STEP_SET(6)
		   | BD60_MAX_AEC_STEP_SET(8) },		/* 0x57 */
	{ COM25,  COM25_50HZ_BANDING_AEC_MSBS_SET(0x0bb)
		   | COM25_60HZ_BANDING_AEC_MSBS_SET(0x09c) },	/* 0x00 */
	{ BD50,   BD50_50HZ_BANDING_AEC_LSBS_SET(0x0bb) },	/* 0xbb */
	{ BD60,   BD60_60HZ_BANDING_AEC_LSBS_SET(0x09c) },	/* 0x9c */
	{ BANK_SEL,  BANK_SEL_DSP },
	{ 0xe5,   0x7f },
	{ MC_BIST,  MC_BIST_RESET | MC_BIST_BOOT_ROM_SEL },
	{ 0x41,   0x24 },
	{ RESET,  RESET_JPEG | RESET_DVP },
	{ 0x76,   0xff },
	{ 0x33,   0xa0 },
	{ 0x42,   0x20 },
	{ 0x43,   0x18 },
	{ 0x4c,   0x00 },
	{ CTRL3,  CTRL3_BPC_EN | CTRL3_WPC_EN | 0x10 },
	{ 0x88,   0x3f },
	{ 0xd7,   0x03 },
	{ 0xd9,   0x10 },
	{ R_DVP_SP,  R_DVP_SP_AUTO_MODE | 0x2 },
	{ 0xc8,   0x08 },
	{ 0xc9,   0x80 },
	{ BPADDR, 0x00 },
	{ BPDATA, 0x00 },
	{ BPADDR, 0x03 },
	{ BPDATA, 0x48 },
	{ BPDATA, 0x48 },
	{ BPADDR, 0x08 },
	{ BPDATA, 0x20 },
	{ BPDATA, 0x10 },
	{ BPDATA, 0x0e },
	{ 0x90,   0x00 },
	{ 0x91,   0x0e },
	{ 0x91,   0x1a },
	{ 0x91,   0x31 },
	{ 0x91,   0x5a },
	{ 0x91,   0x69 },
	{ 0x91,   0x75 },
	{ 0x91,   0x7e },
	{ 0x91,   0x88 },
	{ 0x91,   0x8f },
	{ 0x91,   0x96 },
	{ 0x91,   0xa3 },
	{ 0x91,   0xaf },
	{ 0x91,   0xc4 },
	{ 0x91,   0xd7 },
	{ 0x91,   0xe8 },
	{ 0x91,   0x20 },
	{ 0x92,   0x00 },
	{ 0x93,   0x06 },
	{ 0x93,   0xe3 },
	{ 0x93,   0x03 },
	{ 0x93,   0x03 },
	{ 0x93,   0x00 },
	{ 0x93,   0x02 },
	{ 0x93,   0x00 },
	{ 0x93,   0x00 },
	{ 0x93,   0x00 },
	{ 0x93,   0x00 },
	{ 0x93,   0x00 },
	{ 0x93,   0x00 },
	{ 0x93,   0x00 },
	{ 0x96,   0x00 },
	{ 0x97,   0x08 },
	{ 0x97,   0x19 },
	{ 0x97,   0x02 },
	{ 0x97,   0x0c },
	{ 0x97,   0x24 },
	{ 0x97,   0x30 },
	{ 0x97,   0x28 },
	{ 0x97,   0x26 },
	{ 0x97,   0x02 },
	{ 0x97,   0x98 },
	{ 0x97,   0x80 },
	{ 0x97,   0x00 },
	{ 0x97,   0x00 },
	{ 0xa4,   0x00 },
	{ 0xa8,   0x00 },
	{ 0xc5,   0x11 },
	{ 0xc6,   0x51 },
	{ 0xbf,   0x80 },
	{ 0xc7,   0x10 },	/* simple AWB */
	{ 0xb6,   0x66 },
	{ 0xb8,   0xA5 },
	{ 0xb7,   0x64 },
	{ 0xb9,   0x7C },
	{ 0xb3,   0xaf },
	{ 0xb4,   0x97 },
	{ 0xb5,   0xFF },
	{ 0xb0,   0xC5 },
	{ 0xb1,   0x94 },
	{ 0xb2,   0x0f },
	{ 0xc4,   0x5c },
	{ 0xa6,   0x00 },
	{ 0xa7,   0x20 },
	{ 0xa7,   0xd8 },
	{ 0xa7,   0x1b },
	{ 0xa7,   0x31 },
	{ 0xa7,   0x00 },
	{ 0xa7,   0x18 },
	{ 0xa7,   0x20 },
	{ 0xa7,   0xd8 },
	{ 0xa7,   0x19 },
	{ 0xa7,   0x31 },
	{ 0xa7,   0x00 },
	{ 0xa7,   0x18 },
	{ 0xa7,   0x20 },
	{ 0xa7,   0xd8 },
	{ 0xa7,   0x19 },
	{ 0xa7,   0x31 },
	{ 0xa7,   0x00 },
	{ 0xa7,   0x18 },
	{ 0x7f,   0x00 },
	{ 0xe5,   0x1f },
	{ 0xe1,   0x77 },
	{ 0xdd,   0x7f },
	{ CTRL0,  CTRL0_YUV422 | CTRL0_YUV_EN | CTRL0_RGB_EN },
	ENDMARKER,
};

static int ov2640_write_array(struct i2c_client *client,
	const struct regval_list *vals)
{
	int ret;

	while ((vals->reg_num != 0xff) || (vals->value != 0xff)) {
		ret = i2c_smbus_write_byte_data(client,
			vals->reg_num, vals->value);
		dev_vdbg(&client->dev, "array: 0x%02x, 0x%02x",
			vals->reg_num, vals->value);

		if (ret < 0)
			return ret;
		vals++;
	}
	return 0;
}



#define PER_SIZE_REG_SEQ(x, y, v_div, h_div, pclk_div)	\
	{ CTRLI, CTRLI_LP_DP | CTRLI_V_DIV_SET(v_div) |	\
		 CTRLI_H_DIV_SET(h_div)},		\
	{ ZMOW, ZMOW_OUTW_SET(x) },			\
	{ ZMOH, ZMOH_OUTH_SET(y) },			\
	{ ZMHH, ZMHH_OUTW_SET(x) | ZMHH_OUTH_SET(y) },	\
	{ R_DVP_SP, pclk_div },				\
	{ RESET, 0x00}


static const struct regval_list ov2640_uxga_regs[] = {
	PER_SIZE_REG_SEQ(UXGA_WIDTH, UXGA_HEIGHT, 0, 0, 0),
	{ CTRLI,    0x00},
	{ R_DVP_SP, 0 | R_DVP_SP_AUTO_MODE },
	ENDMARKER,
};

static const struct regval_list ov2640_vga_regs[] = {
	PER_SIZE_REG_SEQ(VGA_WIDTH, VGA_HEIGHT, 0, 0, 2),
	ENDMARKER,
};


static const struct regval_list ov2640_yuyv_regs[] = {
	{ IMAGE_MODE, IMAGE_MODE_YUV422 },
	{ 0xd7, 0x03 },
	{ 0x33, 0xa0 },
	{ 0xe5, 0x1f },
	{ 0xe1, 0x67 },
	{ RESET,  0x00 },
	{ R_BYPASS, R_BYPASS_USE_DSP },
	ENDMARKER,
};

static const struct regval_list ov2640_size_change_preamble_regs[] = {
	{ BANK_SEL, BANK_SEL_DSP },
	{ RESET, RESET_DVP },
	{ SIZEL, SIZEL_HSIZE8_11_SET(UXGA_WIDTH) |
		 SIZEL_HSIZE8_SET(UXGA_WIDTH) |
		 SIZEL_VSIZE8_SET(UXGA_HEIGHT) },
	{ HSIZE8, HSIZE8_SET(UXGA_WIDTH) },
	{ VSIZE8, VSIZE8_SET(UXGA_HEIGHT) },
	{ CTRL2, CTRL2_DCW_EN | CTRL2_SDE_EN |
		 CTRL2_UV_AVG_EN | CTRL2_CMX_EN | CTRL2_UV_ADJ_EN },
	{ HSIZE, HSIZE_SET(UXGA_WIDTH) },
	{ VSIZE, VSIZE_SET(UXGA_HEIGHT) },
	{ XOFFL, XOFFL_SET(0) },
	{ YOFFL, YOFFL_SET(0) },
	{ VHYX, VHYX_HSIZE_SET(UXGA_WIDTH) | VHYX_VSIZE_SET(UXGA_HEIGHT) |
		VHYX_XOFF_SET(0) | VHYX_YOFF_SET(0)},
	{ TEST, TEST_HSIZE_SET(UXGA_WIDTH) },
	ENDMARKER,
};

static const struct regval_list ov2640_format_change_preamble_regs[] = {
	{ BANK_SEL, BANK_SEL_DSP },
	{ R_BYPASS, R_BYPASS_USE_DSP },
	ENDMARKER,
};

static int ov2640_mask_set(struct i2c_client *client,
	u8  reg, u8  mask, u8  set)
{
	s32 val = i2c_smbus_read_byte_data(client, reg);
	if (val < 0)
		return val;

	val &= ~mask;
	val |= set & mask;

	dev_vdbg(&client->dev, "masks: 0x%02x, 0x%02x", reg, val);

	return i2c_smbus_write_byte_data(client, reg, val);
}

struct ov2640_priv {
	struct v4l2_subdev		subdev;
#if defined(CONFIG_MEDIA_CONTROLLER)
	struct media_pad pad;
#endif
	struct v4l2_ctrl_handler	hdl;
	u32	cfmt_code;
	struct clk			*clk;
	const struct ov2640_win_size	*win;

	struct gpio_desc *resetb_gpio;
	struct gpio_desc *pwdn_gpio;

	struct mutex lock; /* lock to protect streaming and power_count */
	bool streaming;
	int power_count;
};


static struct ov2640_priv *to_ov2640(const struct i2c_client *client)
{
	return container_of(i2c_get_clientdata(client), struct ov2640_priv,
		subdev);
}


static int ov2640_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd =
		&container_of(ctrl->handler, struct ov2640_priv, hdl)->subdev;
	struct i2c_client  *client = v4l2_get_subdevdata(sd);
	struct ov2640_priv *priv = to_ov2640(client);
	u8 val;
	int ret;

	/* v4l2_ctrl_lock() locks our own mutex */

	/*
	 * If the device is not powered up by the host driver, do not apply any
	 * controls to H/W at this time. Instead the controls will be restored
	 * when the streaming is started.
	 */
#if 0
	if (!priv->power_count)
		return 0;
#endif

	ret = i2c_smbus_write_byte_data(client, BANK_SEL, BANK_SEL_SENS);
	if (ret < 0)
		return ret;
	printk("ctrl->id = %d\n", ctrl->id);
	switch (ctrl->id) {
	case V4L2_CID_VFLIP:
		val = ctrl->val ? REG04_VFLIP_IMG | REG04_VREF_EN : 0x00;
		return ov2640_mask_set(client, REG04,
			REG04_VFLIP_IMG | REG04_VREF_EN, val);
		/* NOTE: REG04_VREF_EN: 1 line shift / even/odd line swap */
	case V4L2_CID_HFLIP:
		val = ctrl->val ? REG04_HFLIP_IMG : 0x00;
		return ov2640_mask_set(client, REG04, REG04_HFLIP_IMG, val);

		//自动增益
	case V4L2_CID_EXPOSURE_AUTO:

		return 0;
		//自动增益
	case V4L2_CID_AUTOGAIN:
		return 0;
		//增益
	case V4L2_CID_GAIN:
		return 0;
		//曝光
	case V4L2_CID_EXPOSURE:
		return 0;

	case V4L2_CID_TEST_PATTERN:
		val = ctrl->val ? COM7_COLOR_BAR_TEST : 0x00;
		return ov2640_mask_set(client, COM7, COM7_COLOR_BAR_TEST, val);
	}

	return -EINVAL;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int ov2640_g_register(struct v4l2_subdev *sd,
	struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;

	reg->size = 1;
	if (reg->reg > 0xff)
		return -EINVAL;

	ret = i2c_smbus_read_byte_data(client, reg->reg);
	if (ret < 0)
		return ret;

	reg->val = ret;

	return 0;
}
static int ov2640_s_register(struct v4l2_subdev *sd,
	const struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	if (reg->reg > 0xff ||
		reg->val > 0xff)
		return -EINVAL;

	return i2c_smbus_write_byte_data(client, reg->reg, reg->val);
}
#endif





static int ov2640initREG(struct nuvoton_vin_device* cam, struct i2c_client *pi2c)
{
	u16 reg;
	u8 val;

#if 1
	u8 pid, ver, midh, midl;
	u8 ret;
	const char *devname;
	i2c_smbus_write_byte_data(pi2c, BANK_SEL, BANK_SEL_SENS);
	pid = i2c_smbus_read_byte_data(pi2c, PID);
	ver = i2c_smbus_read_byte_data(pi2c, VER);
	midh = i2c_smbus_read_byte_data(pi2c, MIDH);
	midl = i2c_smbus_read_byte_data(pi2c, MIDL);
	
	switch (VERSION(pid, ver)) {
	case PID_OV2640:
		devname = "ov2640";
		break;
	default:
		dev_err(&pi2c->dev,
			"Product ID error %x:%x\n", pid, ver);
		ret = -ENODEV;
		return ret;
	}

	dev_info(&pi2c->dev,
		"%s Product ID %0x:%0x Manufacturer ID %x:%x\n",
		devname, pid, ver, midh, midl);

#else

	SCCB_WR_Reg(OV2640_DSP_RA_DLMT, 0x01);	//操作sensor寄存器
	SCCB_WR_Reg(OV2640_SENSOR_COM7, 0x80);	//软复位OV2640
	msleep(50);
	reg = SCCB_RD_Reg(OV2640_SENSOR_MIDH);	//读取厂家ID 高八位
	reg <<= 8;
	reg |= SCCB_RD_Reg(OV2640_SENSOR_MIDL);	//读取厂家ID 低八位

	if (reg != OV2640_MID) {
		printk("MID:%d\n", reg);
		return -1;
	} else {
		printk("MID:%d check ok\n", reg);
	}


	reg = SCCB_RD_Reg(OV2640_SENSOR_PIDH);	//读取厂家ID 高八位
	reg <<= 8;
	reg |= SCCB_RD_Reg(OV2640_SENSOR_PIDL);	//读取厂家ID 低八位
	if (reg != OV2640_PID) {
		printk("HID:%d\n", reg);
		return 2;
	} else {
		printk("HID:%d check ok\n", reg);
	}
#endif
	ov2640_write_array(pi2c, ov2640_init_regs);
	ov2640_write_array(pi2c, ov2640_size_change_preamble_regs);
	//ov2640_write_array(save_client, ov2640_vga_regs);
	ov2640_write_array(pi2c, ov2640_uxga_regs);
	ov2640_write_array(pi2c, ov2640_format_change_preamble_regs);
	ov2640_write_array(pi2c, ov2640_yuyv_regs);

	//val = (code == MEDIA_BUS_FMT_YVYU8_2X8)
	//	|| (code == MEDIA_BUS_FMT_VYUY8_2X8) ? CTRL0_VFIRST : 0x00;
	val = 0 ? CTRL0_VFIRST : 0x00;

	reg = ov2640_mask_set(save_client, CTRL0, CTRL0_VFIRST, val);

	return 0;
}

static const struct v4l2_ctrl_ops ov2640_ctrl_ops = {
	.s_ctrl = ov2640_s_ctrl,
};

static const char * const ov2640_test_pattern_menu[] = {
	"Disabled",
	"Eight Vertical Colour Bars",
};

static int ov2640_s_power(struct v4l2_subdev *sd, int on)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov2640_priv *priv = to_ov2640(client);

	mutex_lock(&priv->lock);
#if 0
	/*
	 * If the power count is modified from 0 to != 0 or from != 0 to 0,
	 * update the power state.
	 */
	if (priv->power_count == !on)
		ov2640_set_power(priv, on);
	priv->power_count += on ? 1 : -1;
	WARN_ON(priv->power_count < 0);
#endif
	mutex_unlock(&priv->lock);

	return 0;
}

static const struct v4l2_subdev_core_ops ov2640_subdev_core_ops = {
	.log_status = v4l2_ctrl_subdev_log_status,
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = ov2640_g_register,
	.s_register = ov2640_s_register,
#endif
	.s_power = ov2640_s_power,
};


static const struct v4l2_subdev_ops ov2640_subdev_ops = {
	.core = &ov2640_subdev_core_ops,
	//.pad = &ov2640_subdev_pad_ops,
	//.video = &ov2640_subdev_video_ops,
};


static int ov2640_pri_init(struct i2c_client *client)
{
	struct ov2640_priv	*priv;
	int ret;
	priv = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		return -1;
	}
	v4l2_i2c_subdev_init(&priv->subdev, client, &ov2640_subdev_ops);

	mutex_init(&priv->lock);

	v4l2_ctrl_handler_init(&priv->hdl, 6);
	priv->hdl.lock = &priv->lock;
	v4l2_ctrl_new_std(&priv->hdl, &ov2640_ctrl_ops,
		V4L2_CID_VFLIP, 0, 1, 1, 0);
	printk("err = %d\n", priv->hdl.error);
	v4l2_ctrl_new_std(&priv->hdl, &ov2640_ctrl_ops,
		V4L2_CID_HFLIP, 0, 1, 1, 0);
	printk("err = %d\n", priv->hdl.error);
	//自动曝光
	v4l2_ctrl_new_std(&priv->hdl, &ov2640_ctrl_ops,
		V4L2_CID_EXPOSURE_AUTO, 0, 1, 1, 0);
	printk("err = %d\n", priv->hdl.error);
	//自动增益
	v4l2_ctrl_new_std(&priv->hdl, &ov2640_ctrl_ops,
		V4L2_CID_AUTOGAIN, 0, 1, 1, 0);
	printk("err = %d\n", priv->hdl.error);
	//增益
	v4l2_ctrl_new_std(&priv->hdl, &ov2640_ctrl_ops,
		V4L2_CID_GAIN, 0, 32, 1, 0);
	printk("err = %d\n", priv->hdl.error);
	//曝光
	v4l2_ctrl_new_std(&priv->hdl, &ov2640_ctrl_ops,
		V4L2_CID_EXPOSURE, 0, 1024, 1, 0);
#if 0
	v4l2_ctrl_new_std_menu_items(&priv->hdl, &ov2640_ctrl_ops,
		V4L2_CID_TEST_PATTERN,
		ARRAY_SIZE(ov2640_test_pattern_menu) - 1, 0, 0,
		ov2640_test_pattern_menu);
#endif
	priv->subdev.ctrl_handler = &priv->hdl;
	if (priv->hdl.error) {
		ret = priv->hdl.error;
		goto err_hdl;
	}
	return 0;
err_hdl:
	v4l2_ctrl_handler_free(&priv->hdl);
	mutex_destroy(&priv->lock);
	return ret;
}


int nuvoton_vin_probe_ov2640(struct nuvoton_vin_device* cam)
{
	int ret = 0;
	ENTRY();
	nuvoton_vin_attach_sensor(cam, &ov2640);

	// if i2c module isn't loaded at this time
	if (!sensor_inited) {
		printk("sensor_inited not init\n");
		return -1;
	}

	ret = ov2640initREG(cam, save_client);

	if (!ret) {
		ret = ov2640_pri_init(save_client);
		if (ret)printk("error pri init RET = %d\n", ret);
	} else {
		printk("error init RET = %d\n", ret);
	}


	LEAVE();
	return ret;
}

static const struct i2c_device_id sensor_id[] = {
	{"OV2640",0},
	{},
};
MODULE_DEVICE_TABLE(i2c, sensor_id);

static struct i2c_driver sensor_i2c_driver = {
	.driver = { .name  = "OV2640", 
				.owner = THIS_MODULE,
						},
	.probe    = sensor_probe,
	.remove   = sensor_remove,	
	.id_table = sensor_id,
};

module_i2c_driver(sensor_i2c_driver);
MODULE_LICENSE("GPL");


