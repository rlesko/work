/** @file
	ST LIS3DSH definitions
	Digital output motion sensor
	@author pbleyer
*/

#ifndef LIS3DSH_H_
#define LIS3DSH_H_

#include <stdint.h>

/** @defgroup lis3dshdef LIS3DSH definitions */
///@{

#define LIS3DSH_ADDR 0x3c ///< Device I2C address
#define LIS3DSH_ABIT 0x04 ///< SAD bit (SDO/SA1 state)
#define LIS3DSH_SEQ 0x80 ///< Read sequential (multiple)

#define LIS3DSH_INF1 0x0d ///< Info 1
#define LIS3DSH_INF2 0x0e ///< Info 2
#define LIS3DSH_ID 0x0f ///< Identification: 0x3f (RO)
#define LIS3DSH_CTL3 0x23 ///< Control register 3 (RW)
#define LIS3DSH_CTL4 0x20 ///< Control register 4 (RW)
#define LIS3DSH_CTL5 0x24 ///< Control register 5 (RW)
#define LIS3DSH_CTL6 0x25 ///< Control register 6 (RW)
#define LIS3DSH_STS 0x27 ///< Status register (RO)

#define LIS3DSH_TMP 0x0c ///< Temperature
#define LIS3DSH_XOFS 0x10 ///< X offset
#define LIS3DSH_YOFS 0x11 ///< Y offset
#define LIS3DSH_ZOFS 0x12 ///< Z offset
#define LIS3DSH_XCSH 0x13 ///< X constant shift
#define LIS3DSH_YCSH 0x14 ///< Y constant shift
#define LIS3DSH_ZCSH 0x15 ///< Z constant shift
#define LIS3DSH_LC0 0x16 ///< Long counter [7:0]
#define LIS3DSH_LC1 0x17 ///< Long counter [15:8]
#define LIS3DSH_STAT 0x18 ///< Interrupt synchronization (RO)

#define LIS3DSH_VCF1 0x1b ///< Vector filter coefficient 1
#define LIS3DSH_VCF2 0x1c ///< Vector filter coefficient 2
#define LIS3DSH_VCF3 0x1d ///< Vector filter coefficient 3
#define LIS3DSH_VCF4 0x1e ///< Vector filter coefficient 4
#define LIS3DSH_THR3 0x1f ///< Threshold 3

#define LIS3DSH_X0 0x28 ///< X data [7:0]
#define LIS3DSH_X1 0x29 ///< X data [15:8]
#define LIS3DSH_Y0 0x2a ///< Y data [7:0]
#define LIS3DSH_Y1 0x2b ///< Y data [15:8]
#define LIS3DSH_Z0 0x2c ///< Z data [7:0]
#define LIS3DSH_Z1 0x2d ///< Z data [15:8]

#define LIS3DSH_FCTL 0x2e ///< FIFO control (RW)
#define LIS3DSH_FSRC 0x2f ///< FIFO source (RO)

#define LIS3DSH_CTL1 0x21 ///< SM1 control register (RW)
#define LIS3DSH_S1C 0x40 ///< SM1 code register (WO)
#define LIS3DSH_S1C_LEN 16 ///< SM1 code register length (0x40-0x4f)
#define LIS3DSH_S1TM4 0x50 ///< SM1 timer 4
#define LIS3DSH_S1TM3 0x51 ///< SM1 timer 3
#define LIS3DSH_S1TM20 0x52 ///< SM1 timer 2 [7:0]
#define LIS3DSH_S1TM21 0x53 ///< SM1 timer 2 [15:8]
#define LIS3DSH_S1TM10 0x54 ///< SM1 timer 1 [7:0]
#define LIS3DSH_S1TM11 0x55 ///< SM1 timer 1 [15:8]
#define LIS3DSH_S1TH2 0x56 ///< SM1 threshold 2
#define LIS3DSH_S1TH1 0x57 ///< SM1 threshold 1
#define LIS3DSH_S1MKA 0x59 ///< SM1 axis and sign mask
#define LIS3DSH_S1MKB 0x5a ///< SM1 axis and sign mask
#define LIS3DSH_S1SET 0x5b ///< SM1 settings
#define LIS3DSH_S1PR 0x5c ///< SM1 program reset pointer
#define LIS3DSH_S1TC0 0x5d ///< SM1 timer counter [7:0]
#define LIS3DSH_S1TC1 0x5e ///< SM1 timer counter [15:8]
#define LIS3DSH_S1OUT 0x5f ///< SM1 main set flag
#define LIS3DSH_S1PK 0x19 ///< SM1 peak value

#define LIS3DSH_CTL2 0x22 ///< SM2 control register (RW)
#define LIS3DSH_S2C 0x60 ///< SM2 code register (WO)
#define LIS3DSH_S2C_LEN 16 ///< SM2 code register length (0x60-0x6f)
#define LIS3DSH_S2TM4 0x70 ///< SM2 timer 4
#define LIS3DSH_S2TM3 0x71 ///< SM2 timer 3
#define LIS3DSH_S2TM20 0x72 ///< SM2 timer 2 [7:0]
#define LIS3DSH_S2TM21 0x73 ///< SM2 timer 2 [15:8]
#define LIS3DSH_S2TM10 0x74 ///< SM2 timer 1 [7:0]
#define LIS3DSH_S2TM11 0x75 ///< SM2 timer 1 [15:8]
#define LIS3DSH_S2TH2 0x76 ///< SM2 threshold 2
#define LIS3DSH_S2TH1 0x77 ///< SM2 threshold 1
#define LIS3DSH_S2MKA 0x79 ///< SM2 axis and sign mask
#define LIS3DSH_S2MKB 0x7a ///< SM2 axis and sign mask
#define LIS3DSH_S2SET 0x7b ///< SM2 settings
#define LIS3DSH_S2PR 0x7c ///< SM2 program reset pointer
#define LIS3DSH_S2TC0 0x7d ///< SM2 timer counter [7:0]
#define LIS3DSH_S2TC1 0x7e ///< SM2 timer counter [15:8]
#define LIS3DSH_S2OUT 0x7f ///< SM2 main set flag
#define LIS3DSH_S2PK 0x1a ///< SM2 peak value
#define LIS3DSH_S2DEC 0x78 ///< SM2 decimation factor (WO)

/** INF1 values */
enum Lis3dshInf1Value
{
	lis3dsh_INF1_Val = 0x21
};

/** INF2 values */
enum Lis3dshInf2Value
{
	lis3dsh_INF2_Val = 0x00
};

/** WHOAMI values */
enum Lis3dshIdValue
{
	lis3dsh_ID_Val = 0x3f
};

/** CTL3 values */
enum Lis3dshCtl3Value
{
	lis3dsh_STRT_Pos = 0, ///< Soft reset
	lis3dsh_STRT_Len = 1,
	lis3dsh_STRT_Off = 0,
	lis3dsh_STRT_On,

	lis3dsh_VFLT_Pos = 2, ///< Vector filter
	lis3dsh_VFLT_Len = 1,
	lis3dsh_VFLT_Off = 0,
	lis3dsh_VFLT_On,

	lis3dsh_INT1EN_Pos = 3, ///< Interrupt 1 enable
	lis3dsh_INT1EN_Len = 1,
	lis3dsh_INT1EN_Off = 0,
	lis3dsh_INT1EN_On,

	lis3dsh_INT2EN_Pos = 4, ///< Interrupt 2 enable
	lis3dsh_INT2EN_Len = 1,
	lis3dsh_INT2EN_Off = 0,
	lis3dsh_INT2EN_On,

	lis3dsh_IEL_Pos = 5, ///< Interrupt signal latching
	lis3dsh_IEL_Len = 1,
	lis3dsh_IEL_On = 0,
	lis3dsh_IEL_Off,

	lis3dsh_IEA_Pos = 6, ///< Interrupt active level
	lis3dsh_IEA_Len = 1,
	lis3dsh_IEA_Low = 0,
	lis3dsh_IEA_High,

	lis3dsh_DRDE_Pos = 7, ///< DRDY enable to INT1
	lis3dsh_DRDE_Len = 1,
	lis3dsh_DRDE_Off = 0,
	lis3dsh_DRDE_On,
};

/** CTRL4 (conversion) values */
enum Lis3dshCtl4Value
{
	lis3dsh_XEN_Pos = 0, ///< X axis enable
	lis3dsh_XEN_Len = 1,
	lis3dsh_XEN_Off = 0,
	lis3dsh_XEN_On,

	lis3dsh_YEN_Pos = 1, ///< Y axis enable
	lis3dsh_YEN_Len = 1,
	lis3dsh_YEN_Off = 0,
	lis3dsh_YEN_On,

	lis3dsh_ZEN_Pos = 2, ///< Z axis enable
	lis3dsh_ZEN_Len = 1,
	lis3dsh_ZEN_Off = 0,
	lis3dsh_ZEN_On,

	lis3dsh_BDU_Pos = 3, ///< Block data update
	lis3dsh_BDU_Len = 1,
	lis3dsh_BDU_Off = 0,
	lis3dsh_BDU_On,

	lis3dsh_ODR_Pos = 4, ///< Output data rate
	lis3dsh_ODR_Len = 5,
	lis3dsh_ODR_Off = 0, ///< Power down
	lis3dsh_ODR_3_125Hz,
	lis3dsh_ODR_6_25Hz,
	lis3dsh_ODR_12_5Hz,
	lis3dsh_ODR_25Hz,
	lis3dsh_ODR_50Hz,
	lis3dsh_ODR_100Hz,
	lis3dsh_ODR_400Hz,
	lis3dsh_ODR_800Hz,
	lis3dsh_ODR_1600Hz,
};

/** CTRL5 values */
enum Lis3dshCtl5Value
{
	lis3dsh_SIM_Pos = 0, ///< SPI mode
	lis3dsh_SIM_Len = 1,
	lis3dsh_SIM_4Wire = 0,
	lis3dsh_SIM_3Wire,

	lis3dsh_ST_Pos = 1, ///< Self test
	lis3dsh_ST_Len = 2,
	lis3dsh_ST_Off = 0, ///< Normal mode
	lis3dsh_ST_Positive, ///< Positive sign test
	lis3dsh_ST_Negative, ///< Negative sign test

	lis3dsh_FS_Pos = 3, ///< Full scale
	lis3dsh_FS_Len = 3,
	lis3dsh_FS_2 = 0, ///< +/- 2G
	lis3dsh_FS_4,
	lis3dsh_FS_6,
	lis3dsh_FS_8,
	lis3dsh_FS_16,

	lis3dsh_BW_Pos = 6, ///< Anti-aliasing filter bandwidth
	lis3dsh_BW_Len = 2,
	lis3dsh_BW_800Hz = 0,
	lis3dsh_BW_400Hz,
	lis3dsh_BW_200Hz,
	lis3dsh_BW_50Hz,
};

/** CTRL6 values */
enum Lis3dshCtl6Value
{
	lis3dsh_P2BOOT_Pos = 0, ///< Boot interrupt on INT2
	lis3dsh_P2BOOT_Len = 1,
	lis3dsh_P2BOOT_Off = 0,
	lis3dsh_P2BOOT_On,

	lis3dsh_P1OVR_Pos = 1, ///< FIFO overrun on INT1
	lis3dsh_P1OVR_Len = 1,
	lis3dsh_P1OVR_Off = 0,
	lis3dsh_P1OVR_On,

	lis3dsh_P1WTM_Pos = 2, ///< FIFO watermark on INT1
	lis3dsh_P1WTM_Len = 1,
	lis3dsh_P1WTM_Off = 0,
	lis3dsh_P1WTM_On,

	lis3dsh_P1EMP_Pos = 3, ///< FIFO empty on INT1
	lis3dsh_P1EMP_Len = 1,
	lis3dsh_P1EMP_Off = 0,
	lis3dsh_P1EMP_On,

	lis3dsh_ADDI_Pos = 4, ///< Address automatically incremented
	lis3dsh_ADDI_Len = 1,
	lis3dsh_ADDI_Off = 0,
	lis3dsh_ADDI_On,

	lis3dsh_WTME_Pos = 5, ///< FIFO watermark enable
	lis3dsh_WTME_Len = 1,
	lis3dsh_WTME_Off = 0,
	lis3dsh_WTME_On,

	lis3dsh_FEN_Pos = 6, ///< FIFO enable
	lis3dsh_FEN_Len = 1,
	lis3dsh_FEN_Off = 0,
	lis3dsh_FEN_On,

	lis3dsh_BOOT_Pos = 7, ///< Force reboot (self-cleared)
	lis3dsh_BOOT_Len = 1,
	lis3dsh_BOOT_Off = 0,
	lis3dsh_BOOT_On,
};

/** CTRL4 values */
/*
enum Lis3dshCtl4Value
{
	lis3dsh_BLE_Pos = 1, ///< Big/little endian selection
	lis3dsh_BLE_Len = 1,
	lis3dsh_BLE_Little = 0,
	lis3dsh_BLE_Big,

	lis3dsh_ZOM_Pos = 2, ///< Z-axis operating mode
	lis3dsh_ZOM_Len = 2,
	lis3dsh_ZOM_Low = 0,
	lis3dsh_ZOM_Medium,
	lis3dsh_ZOM_High,
	lis3dsh_ZOM_Ultra,
};
*/
/** CTRL5 values */
/*
enum Lis3dshCtl5Value
{
	lis3dsh_BDU_Pos = 6, ///< Block data update
	lis3dsh_BDU_Len = 1,
	lis3dsh_BDU_Cont = 0, ///< Continuous
	lis3dsh_BDU_Seq, ///< Sequential
};
*/
/** STATUS values */
/*
enum Lis3dshStsValue
{
	lis3dsh_XDA_Pos = 0, ///< X data available
	lis3dsh_XDA_Len = 1,
	lis3dsh_XDA_Off = 0,
	lis3dsh_XDA_On,

	lis3dsh_YDA_Pos = 1, ///< Y data available
	lis3dsh_YDA_Len = 1,
	lis3dsh_YDA_Off = 0,
	lis3dsh_YDA_On,

	lis3dsh_ZDA_Pos = 2, ///< Z data available
	lis3dsh_ZDA_Len = 1,
	lis3dsh_ZDA_Off = 0,
	lis3dsh_ZDA_On,

	lis3dsh_XYZDA_Pos = 3, ///< XYZ data available
	lis3dsh_XYZDA_Len = 1,
	lis3dsh_XYZDA_Off = 0,
	lis3dsh_XYZDA_On,

	lis3dsh_XOR_Pos = 4, ///< X OR data available
	lis3dsh_XOR_Len = 1,
	lis3dsh_XOR_Off = 0,
	lis3dsh_XOR_On,

	lis3dsh_YOR_Pos = 5, ///< Y OR data available
	lis3dsh_YOR_Len = 1,
	lis3dsh_YOR_Off = 0,
	lis3dsh_YOR_On,

	lis3dsh_ZOR_Pos = 6, ///< Z OR data available
	lis3dsh_ZOR_Len = 1,
	lis3dsh_ZOR_Off = 0,
	lis3dsh_ZOR_On,

	lis3dsh_XYZOR_Pos = 7, ///< XYZ OR data available
	lis3dsh_XYZOR_Len = 1,
	lis3dsh_XYZOR_Off = 0,
	lis3dsh_XYZOR_On,
};
*/
/** Interrupt configuration values */
enum Lis3dshIcfgValue
{
	lis3dsh_IEN_Pos = 0, ///< Interrupt enable on INT pin
	lis3dsh_IEN_Len = 1,
	lis3dsh_IEN_Off = 0,
	lis3dsh_IEN_On,

	lis3dsh_LIR_Pos = 1, ///< Latch interrupt request (read ISRC register to clear)
	lis3dsh_LIR_Len = 1,
	lis3dsh_LIR_On = 0, ///< Latch (default)
	lis3dsh_LIR_Off,

//	lis3dsh_IEA_Pos = 2, ///< Interrupt active configuration
//	lis3dsh_IEA_Len = 1,
//	lis3dsh_IEA_Low = 0,
//	lis3dsh_IEA_High,

	lis3dsh_ZIEN_Pos = 5, ///< Z-axis interrupt enable
	lis3dsh_ZIEN_Len = 1,
	lis3dsh_ZIEN_Off = 0,
	lis3dsh_ZIEN_On,

	lis3dsh_YIEN_Pos = 6, ///< Y-axis interrupt enable
	lis3dsh_YIEN_Len = 1,
	lis3dsh_YIEN_Off = 0,
	lis3dsh_YIEN_On,

	lis3dsh_XIEN_Pos = 7, ///< X-axis interrupt enable
	lis3dsh_XIEN_Len = 1,
	lis3dsh_XIEN_Off = 0,
	lis3dsh_XIEN_On,

};

/** Interrupt source values */
enum Lis3dshIsrcValue
{
	lis3dsh_IA_Pos = 0, ///< Interrupt active
	lis3dsh_IA_Len = 1,
	lis3dsh_IA_Off = 0,
	lis3dsh_IA_On,

	lis3dsh_MROI_Pos = 1, ///< Internal measurement overflow
	lis3dsh_MROI_Len = 1,
	lis3dsh_MROI_Off = 0,
	lis3dsh_MROI_On,

	lis3dsh_ZNTH_Pos = 2, ///< Z negative threshold exceeded
	lis3dsh_ZNTH_Len = 1,
	lis3dsh_ZNTH_Off = 0,
	lis3dsh_ZNTH_On,

	lis3dsh_YNTH_Pos = 3, ///< Y negative threshold exceeded
	lis3dsh_YNTH_Len = 1,
	lis3dsh_YNTH_Off = 0,
	lis3dsh_YNTH_On,

	lis3dsh_XNTH_Pos = 4, ///< X negative threshold exceeded
	lis3dsh_XNTH_Len = 1,
	lis3dsh_XNTH_Off = 0,
	lis3dsh_XNTH_On,

	lis3dsh_ZPTH_Pos = 5, ///< Z positive threshold exceeded
	lis3dsh_ZPTH_Len = 1,
	lis3dsh_ZPTH_Off = 0,
	lis3dsh_ZPTH_On,

	lis3dsh_YPTH_Pos = 6, ///< Y positive threshold exceeded
	lis3dsh_YPTH_Len = 1,
	lis3dsh_YPTH_Off = 0,
	lis3dsh_YPTH_On,

	lis3dsh_XPTH_Pos = 7, ///< X positive threshold exceeded
	lis3dsh_XPTH_Len = 1,
	lis3dsh_XPTH_Off = 0,
	lis3dsh_XPTH_On,
};

/** Build LIS3DSH register value */
#define lis3dsh_(x,y) (lis3dsh_##x##_##y << lis3dsh_##x##_Pos)

///@}

#endif // LIS3DSH_H_
