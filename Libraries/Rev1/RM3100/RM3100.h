/**
* @file         RM3100.h
*
* @brief        Sample interface for RM3100.
*
* @authors      Betty Zhang
* @date         05/21/2018
* @copyright    (C) 2018 PNI Corp, Protonex LLC
*
* @copyright    Disclosure to third parties or reproduction in any form
*               whatsoever, without prior written consent, is strictly forbidden
*
*/
#include "arduino.h"
// extern DigitalOut cs; //ssel

// Global typedef
typedef signed char     s8;
typedef char            u8;    
typedef short           s16;   
typedef unsigned short  u16;   
typedef int             s32;   
typedef unsigned int    u32;   
typedef unsigned long   u64;

//RM3100 Registers 
#define RM3100_POLL_REG     0x00 /**< RW; Poll for a single measurement*/
#define RM3100_CMM_REG      0x01 /**< RW; Initiate continuous measurement mode*/
#define RM3100_CCXLSB_REG   0x04 /**< RW; Cycle count X-axis */
#define RM3100_CCXMSB_REG   0x05
#define RM3100_CCYLSB_REG   0x06 /**< RW; Cycle count Y-axis */
#define RM3100_CCYMSB_REG   0x07
#define RM3100_CCZLSB_REG   0x08 /**< RW; Cycle count Z-axis */
#define RM3100_CCZMSB_REG   0x09
#define RM3100_TMRC_REG     0x0B /**< RW; Set data rate in continuous measurement mode*/
#define RM3100_MX_REG       0x24 /**< RW; Measurement Result X-axis, Signed 24-bit */
#define RM3100_BIST_REG     0x33 /**< RW; Built-in self test */
#define RM3100_STATUS_REG   0x34 /**< R; Status of DRDY */
#define RM3100_REVID_REG    0x36 /**< R; Revision ID, default 0x22 */

//RM3100 Default values
#define DEFAULTCCOUNT 200 //200
#define DEFAULTGAIN 75    //200 Cycle Count, Gain 75

//Other Settings
#define UPPERCYCLECOUNT 400
#define LOWERCYCLECOUNT 30


 /**
 * \brief Host interface control
 */
#pragma pack(push, 1)
typedef union RegCMM
{
    /**
    * \brief Direct access to the complete 8bit register
    */
    u8 reg;
    /**
    * \brief Access to individual bits in the register.
    */
    struct
    {
        u8 Start : 1;             /**< bit 0: Initiate Continuous Measurement Mode */
        u8 Alarm : 1;             /**< bit 1: Go high if measurment outside a prefefined range */
        u8 Drdm :  2;             /**< bit 2: DRDM bits establish required condition to trigger DRDY */
                                  /**< 0 = drdy high when Alarm = 1 and complete of full measurement set by CMX CMY CMZ */
                                  /**< 1 = drdy high after complete of measurement on any axis */
                                  /**< 2 = drdy high after complete of full measurement set by CMX CMY CMZ */
                                  /**< 3 = drdy high when Alarm = 1 */
        u8 CMX : 1;               /**< bit 4: 1 means measurement on this Axis */
        u8 CMY : 1;               /**< bit 5: 1 means measurement on this Axis */
        u8 CMZ : 1;               /**< bit 6: 1 means measurement on this Axis */
        u8 LDM : 1;               /**< bit 7: 0 absolute alarm mode, 1 relative alarm mode */
    }
    bits;
}
RegCMM;                          /**< typedef for storing CMM register values */
#pragma pack(pop)

#pragma pack(push, 1)
typedef union RegTMRC
{
    /**
    * \brief Direct access to the complete 8bit register
    */
    u8 reg;
    /**
    * \brief Access to individual bits in the register.
    */
    struct
    {
        u8 TMRC : 4;             /**< bit 0 - 3: Time between measurement in CCM moden 1-axis */
                                 /**< value 0b0010 to 0b1111, 0x2 to 0xF, bigger value longer interval */
        u8 Resv : 4;             /**< bit 4 - 7: Fixed as 0b1001, 0x9 */
    }
    bits;
}
RegTMRC;                          /**< typedef for storing TMRC register values */
#pragma pack(pop)

#pragma pack(push, 1)
typedef union RegBIST
{
    /**
    * \brief Direct access to the complete 8bit register
    */
    u8 reg;
    /**
    * \brief Access to individual bits in the register.
    */
    struct
    {
        u8 BP : 2;             /**< bit 0-1: Define number of LR periods for measurement */
                               /**< 0 = unused */
                               /**< 1 = 1 LR Period */
                               /**< 2 = 2 LR Periods */
                               /**< 3 = 4 LR Periods */
        u8 BW : 2;             /**< bit 2-3: Define timeout period for LR oscillator perriod */
                               /**< 0 = unused */
                               /**< 1 = 1 sleep oscillation cycle,  30us */
                               /**< 2 = 2 sleep oscillation cycles,  60us */
                               /**< 3 = 4 sleep oscillation cycles,  120us */
        u8 XYZOK : 3;          /**< bit 4-6: Read only */
                               /**< bit 4: 1 X ok, 0 not ok */
                               /**< bit 5: 1 Y ok, 0 not ok */
                               /**< bit 6: 1 Z ok, 0 not ok */
        u8 STE : 1;            /**< bit 7: write 1 to enable the self test */
    }
    bits;
}
RegBIST;                          /**< typedef for storing TMRC register values */
#pragma pack(pop)

#pragma pack(push, 1)
typedef union RegPOLL
{
    /**
    * \brief Direct access to the complete 8bit register
    */
    u8 reg;
    /**
    * \brief Access to individual bits in the register.
    */
    struct
    {
        u8 LowNibble : 4;      /**< bit 0-3: Reserved */
        u8 PMX : 1;            /**< bit 4: Measure X axis */
        u8 PMY : 1;            /**< bit 5: Measure Y axis */
        u8 PMZ : 1;            /**< bit 6: Measure Z axis */
        u8 MSB : 1;            /**< bit 7: Reserved */
    }
    bits;
}
RegPOLL;                          /**< typedef for storing TMRC register values */
#pragma pack(pop)


//RM3100 class
class RM3100
{

public:
    void ClearDrdyInt(void);
    
    void RunCMM(int flag);
    
    void ChangeSampleRate(int flag);
    
    void SetSampleRateReg(int rate); 
    
    int GetSampleRate(int * reg_val); 
    
    void ChangeCycleCount(int flag);
    
    void SetCycleCountReg();

    void ReadRM3100(); 
    
    void SetDrdyIntFlag(u8 flag);
    
    u8 GetDrdyIntFlag(void);
    
    void DrdyCallBack(void);
    
    void ProcessDrdyInt(void);
    
    void DisplayCycleCount();
    
    void DisplayREVIDReg();
        
    void SelfTest();
    
    //constructor
    RM3100();
    
private:
    u8 rm3100_service_flag;
    int current_gain[3];
    int current_ccount[3];
    int sample_rate;
    int prev_rate;
    
//    DigitalOut cs; //ssel
//    InterruptIn drdy; //Drdy pin D9
    
//    SPI spi;

};
