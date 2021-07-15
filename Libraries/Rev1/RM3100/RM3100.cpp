/**
* @file         RM3100.cpp
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
#include "RM3100.h"
// #include "main.h"
#include <Arduino.h>
#include <Spi.h>

#define csPin 10      // SPI CS
#define i2cenPin 8   // low to use SPI
#define datardyPin 9

extern void dataRDYIRQ();

void RM3100::ClearDrdyInt()
{
    //Clear Interrupt first
 //   cs = 0;
    digitalWrite(SS, LOW);
    SPI.transfer(RM3100_STATUS_REG);
    int status = SPI.transfer(0x00);
//    cs = 1;
    delay(10);
    digitalWrite(SS, HIGH);
    if (status & 0x80)
    {
        ReadRM3100();
        status = 0;
    }
}
    
void RM3100::RunCMM(int flag)
{
    //Set current sample rate if it changes
    if (sample_rate != prev_rate)
    {
        SetSampleRateReg(sample_rate);
        prev_rate = sample_rate;
    }
    
    if (flag) {
        //Set Cycle Count if it changes
        SetCycleCountReg();

        //Confirm new setting and update "gain"
        DisplayCycleCount();
    }

    RegCMM cmm_reg;
    cmm_reg.bits.LDM = 0;
    cmm_reg.bits.CMX = 1;
    cmm_reg.bits.CMY = 1;
    cmm_reg.bits.CMZ = 1;
    cmm_reg.bits.Drdm = 2; //0 on any axis, 2 Drdy after completion of XYZ
    cmm_reg.bits.Alarm = 0;

    if (flag) {
        //Start CMM run
        cmm_reg.bits.Start = 1;
        cmm_reg.bits.CMX = 1;
        cmm_reg.bits.CMY = 1;
        cmm_reg.bits.CMZ = 1;
    } else {
        //Stop CMM run
        cmm_reg.bits.Start = 0;
        cmm_reg.bits.CMX = 0;
        cmm_reg.bits.CMY = 0;
        cmm_reg.bits.CMZ = 0;
    }
    
//    cs = 0;
    digitalWrite(SS, LOW);
    
    SPI.transfer(RM3100_CMM_REG);
    SPI.transfer(cmm_reg.reg);
    delay(10);
    digitalWrite(SS, HIGH);
    
//    cs = 1;
}

void RM3100::ChangeSampleRate(int flag)
{
    if (flag > 0)
        sample_rate *= 2; //increase 2 times
    if (flag < 0)
        sample_rate /= 2; //decrease 2 times
    
    if (sample_rate < 1)
        sample_rate = 1; //1hz
    else if (sample_rate > 600)
        sample_rate = 600;
}

void RM3100::SetSampleRateReg(int rate)
{
    int value;
    
    sample_rate = rate;
        
    if ((rate <= 600) && (rate >= 300))
        value = 0x92;
    else if ((rate < 300) && (rate >= 150))
        value = 0x93;
    else if ((rate < 150) && (rate >= 75))
        value = 0x94;
    else if ((rate < 75) && (rate >= 37))
        value = 0x95;
    else if ((rate < 37) && (rate >= 18))
        value = 0x96;
    else if ((rate < 18) && (rate >= 9))
        value = 0x97;
    else if ((rate < 9) && (rate >= 4))
        value = 0x98;
    else if ((rate < 4) && (rate >= 3))
        value = 0x99;
    else if ((rate < 3) && (rate >= 2))
        value = 0x9A;
    else if ((rate < 2) && (rate >= 1))
        value = 0x9B;   //About 1Hz
    else 
        value = 0x9C;   //About 0.6Hz
        
//    cs = 0;
    digitalWrite(SS, LOW);
    
    //set sample rate
    SPI.transfer(RM3100_TMRC_REG);
    SPI.transfer(value); //about 1Hz
    delay(10);
    digitalWrite(SS, HIGH);
    
//    cs = 1;
}

int RM3100::GetSampleRate(int *tmrc_reg_val)
{
//    cs = 0;
    digitalWrite(SS, LOW);
	
    //set sample rate
    SPI.transfer(RM3100_TMRC_REG | 0x80); //Read TMRC reg
    * tmrc_reg_val = SPI.transfer(0);
//    cs = 1;
    delay(10);
    digitalWrite(SS, HIGH);
    
    return sample_rate;
}
    
void RM3100::ReadRM3100()
{
	int magX[9];
	int magY[9];
	int magZ[9];
	
    int count[3];

//    __disable_irq();    // Disable Interrupts
//    cs = 0;
    noInterrupts();
    digitalWrite(SS, LOW);
    
    SPI.transfer(RM3100_MX_REG);

    Serial.print("MagX = 0x");
    for (int i = 0; i < 9; i++) {
        magX[i] = SPI.transfer(0x00);
        Serial.print(magX[i], HEX);
        if ((i < 8) && ((i+1) % 3) == 0)
            Serial.print(" 0x");
    }
    Serial.print("\n\r");
//    cs = 1;
    delay(10);
    digitalWrite(SS, HIGH);
    delay(100);
    digitalWrite(SS, LOW);
    
    SPI.transfer(RM3100_MY_REG);

    Serial.print("MagY = 0x");
    for (int i = 0; i < 9; i++) {
        magY[i] = SPI.transfer(0x00);
        Serial.print(magY[i], HEX);
        if ((i < 8) && ((i+1) % 3) == 0)
            Serial.print(" 0x");
    }
    Serial.print("\n\r");
//    cs = 1;
    delay(10);
    digitalWrite(SS, HIGH);
    delay(100);   
    digitalWrite(SS, LOW);
    
    SPI.transfer(RM3100_MZ_REG);

    Serial.print("MagZ = 0x");
    for (int i = 0; i < 9; i++) {
        magZ[i] = SPI.transfer(0x00);
        Serial.print(magZ[i], HEX);
        if ((i < 8) && ((i+1) % 3) == 0)
            Serial.print(" 0x");
    }
//    cs = 1;
    delay(10);
    digitalWrite(SS, HIGH);
       
       
    Serial.print(", ");
    //Process the 24-bit signed measurement in count
    int measurement = 0;
    int index = 0;
    for (int j = 0; j < 9; j += 3) {
        if (magX[j] & 0x80)
            measurement = 0xFF;
        measurement <<= 24; //left shift 24-bit
        measurement |= (magX[j+2] | (magX[j+1] | (magX[j] << 8)) << 8);
        Serial.print(measurement, HEX);
        count[index] = measurement;
        measurement = 0;
        index++;
    }
    Serial.print("\n\r");  
    Serial.print(", ");
    //Convert to uT (microTesla)
    for (int k = 0; k < 3; k++) {
	    Serial.print("X - Axis: uT: ");
	    Serial.print((float)count[k]/current_gain[k]);
    }
    Serial.print("\n\r");  
    Serial.print(", ");
    //Process the 24-bit signed measurement in count
    measurement = 0;
    index = 0;
    for (int j = 0; j < 9; j += 3) {
        if (magY[j] & 0x80)
            measurement = 0xFF;
        measurement <<= 24; //left shift 24-bit
        measurement |= (magY[j+2] | (magY[j+1] | (magY[j] << 8)) << 8);
        Serial.print(measurement, HEX);
        count[index] = measurement;
        measurement = 0;
        index++;
    }
    Serial.print("\n\r");  
    Serial.print(", ");
    //Convert to uT (microTesla)
    for (int k = 0; k < 3; k++) {
	    Serial.print("Y - Axis: uT: ");
	    Serial.print((float)count[k]/current_gain[k]);
    }

    Serial.print("\n\r");  
    Serial.print(", ");
    //Process the 24-bit signed measurement in count
    measurement = 0;
    index = 0;
    for (int j = 0; j < 9; j += 3) {
        if (magZ[j] & 0x80)
            measurement = 0xFF;
        measurement <<= 24; //left shift 24-bit
        measurement |= (magZ[j+2] | (magZ[j+1] | (magZ[j] << 8)) << 8);
        Serial.print(measurement, HEX);
        count[index] = measurement;
        measurement = 0;
        index++;
    }
    Serial.print("\n\r");  
    Serial.print(", ");
    //Convert to uT (microTesla)
    for (int k = 0; k < 3; k++) {
	    Serial.print("Z - Axis: uT: ");
	    Serial.print((float)count[k]/current_gain[k]);
    }


    Serial.print("\n\r");
    
//    __enable_irq();     // Enable Interrupts
    interrupts();

}

void RM3100::SetDrdyIntFlag(u8 flag)
{
	rm3100_service_flag = flag;
#if 1	
    if (flag)
//        drdy.disable_irq();
//	    noInterrupts();
	    detachInterrupt(digitalPinToInterrupt(datardyPin));
//	    attachInterrupt(digitalPinToInterrupt(datardyPin), dataRDYIRQ, RISING);
  
    else
//	    drdy.enable_irq();
	//    interrupts();
	    attachInterrupt(digitalPinToInterrupt(datardyPin), dataRDYIRQ, RISING);
//    detachInterrupt(digitalPinToInterrupt(datardyPin));
#endif    
}

u8 RM3100::GetDrdyIntFlag(void)
{
    return rm3100_service_flag;
}

void RM3100::ProcessDrdyInt()
{
    SetDrdyIntFlag(1);
}

void RM3100::DrdyCallBack(void)
{
    // attach ProcessDrdyInt function of this RM3100 instance
//    drdy.rise(callback(this, &RM3100::ProcessDrdyInt));
	attachInterrupt(digitalPinToInterrupt(datardyPin), dataRDYIRQ, RISING); 
}

void RM3100::DisplayCycleCount()
{
    int cc[6];
    int c_count[3];
    float maxrate;

    //Read CC reg
//    __disable_irq();    // Disable Interrupts
    noInterrupts();
//    cs = 0;
    digitalWrite(SS, LOW);
    int cc_reg = RM3100_CCXLSB_REG | 0x80; //"| 0x80" to read CC Reg
    SPI.transfer(cc_reg);

    Serial.print("CC = 0x");
    for (int i = 0; i < 6; i++) {
        cc[i] = SPI.transfer(0);
        Serial.print(cc[i], HEX);
        if ((i < 5) && ((i+1) % 2) == 0)
            Serial.print(" 0x");
    }
//    cs = 1;
    delay(10);
    digitalWrite(SS, HIGH);

    Serial.print(", ");
    //Process the 16-bit unsigned cycle count
    int temp = 0;
    int index = 0;
    for (int j = 0; j < 6; j += 2) {
        if (cc[j] & 0x80)
            temp = 0xFF;
        temp <<= 8; //left shift 8-bit
        temp |= (cc[j+1] | (cc[j] << 8));
        c_count[index++] = temp; //save CC values 
///        Serial.print("%d ", temp);
	Serial.print("temp: ");
	Serial.print(temp);
        temp = 0;
    }

    //Calculate gain from CC, gain = 0.375 * cc;
    Serial.print(", Gain = ");
    for (int k = 0; k < 3; k++) {
        if (c_count[k])
            current_gain[k] = c_count[k] * DEFAULTGAIN/DEFAULTCCOUNT;
///        Serial.print("%d ", current_gain[k]);
	Serial.print("current gain:");
	Serial.print(current_gain[k]);
    }
    
    //Calculate max sample rate, assume CC same for 3 axes
    if (c_count[0] == 50)
        maxrate = 1600.0 / 3.0;
    else if (c_count[0] == 100)
        maxrate = 850.0 / 3.0;
    else if (c_count[0] == 200)
        maxrate = 440.0 / 3.0;
    else if (c_count[0] == 225)
        maxrate = 370.0 / 3.0;
    else if (c_count[0] == 250)
        maxrate = 350.0 / 3.0;
    else if (c_count[0] < 100)
        maxrate = (-15.0 * c_count[0] + 2350.0) / 3.0;
    else if (c_count[0] < 225)
        maxrate = (-14.0 * c_count[0]/5.0 + 1000.0) / 3.0;
    else if (c_count[0] < 250)
        maxrate = (-4.0 * c_count[0]/5.0 + 550.0) / 3.0;
    else if (c_count[0] <= 400)
        maxrate = (-4.0 * c_count[0] /5.0 + 550.0) / 3.0;

///    Serial.print("MaxRate = %3.2f Hz\n\r", maxrate);
    Serial.print("MaxRate = ");
    Serial.print(maxrate);
    Serial.print(" Hz");

//    __enable_irq();     // Enable Interrupts
    interrupts();
}

void RM3100::DisplayREVIDReg()
{
    //Read REVID reg
//    __disable_irq();    // Disable Interrupts
	noInterrupts();
//    cs = 0;
    digitalWrite(SS, LOW);
    int reg = RM3100_REVID_REG;
    SPI.transfer(reg);
    int revid = SPI.transfer(0);
//    __enable_irq();    // Enable Interrupts
    interrupts();
//    cs = 1;
    delay(10);
    digitalWrite(SS, HIGH);

///    Serial.print("RM3100 REVID = %2D\n\r", revid);
    Serial.print("RM3100 REVID = 0x");
    Serial.print(revid, HEX);
}

void RM3100::SelfTest()
{
    Serial.print("SelfTest \n\r");

    RegBIST regbist;
    RegPOLL regpoll;

    //Set BIST
    //bit#7 STE=1, bit#6#5#4 = 0, bit#3 BW1=1, bit#2 BW0=1, bit#1 BP1=1, bit#0 BP0=1
    regbist.bits.BP = 3;
    regbist.bits.BW = 3;
    regbist.bits.XYZOK = 0;
    regbist.bits.STE = 1;

    //Set POLL reg
    regpoll.bits.LowNibble = 0;
    regpoll.bits.PMX = 1;
    regpoll.bits.PMY = 1;
    regpoll.bits.PMZ = 1;
    regpoll.bits.MSB = 0;

    //Start Self Test
//    __disable_irq();    // Disable Interrupts
    noInterrupts();
//    cs = 0;
    digitalWrite(SS, LOW);

    //stop continuous mode
    SPI.transfer(RM3100_CMM_REG);
    SPI.transfer(0);
//    cs = 1;
    delay(10);
    digitalWrite(SS, HIGH);
    delay(10);
    digitalWrite(SS, LOW);

//    cs = 0;
    SPI.transfer(RM3100_BIST_REG);
    SPI.transfer(regbist.reg);
///    Serial.print("bist val= 0x%X, poll val = 0x%X \n\r",
///    regbist.reg, regpoll.reg);
    Serial.print("bist val= 0x");
    Serial.print(regbist.reg, HEX);
    Serial.print("poll val= 0x");
    Serial.print(regpoll.reg, HEX);
    delay(10);
     digitalWrite(SS, HIGH);

//    cs = 1;

//    cs = 0;
    //POLL 1 measurement on XYZ
    digitalWrite(SS, LOW);
    SPI.transfer(RM3100_POLL_REG);
    SPI.transfer(regpoll.reg);
    delay(10);
    digitalWrite(SS, HIGH);
    delay(10);
//    cs = 1;

    //Get result
//    cs = 0;
    digitalWrite(SS, LOW);

    SPI.transfer(RM3100_STATUS_REG | 0x80);
    int value = SPI.transfer(0);
///    Serial.print("Poll a measurement and Check status reg val = 0x%X
///    \n\r", value);
    Serial.print("Poll a measurement and Check status reg val = 0x");
    Serial.print(value, HEX);
    delay(10);
    digitalWrite(SS, HIGH);

//    cs = 1;

    if (value) {
        //Read RM3100_BIST_REG;
//        cs = 0;
    digitalWrite(SS, LOW);

        SPI.transfer(RM3100_BIST_REG | 0x80);
	value = SPI.transfer(0);
    delay(10);
    digitalWrite(SS, HIGH);
	
//        cs = 1;
#if 1
        //Check result here
    Serial.print("Check BIST reg 0x");
    Serial.print(value,HEX);
        if (value & 0x70) {
		Serial.print("Result = 0x");
	        Serial.print(value,HEX);
	} else {
		Serial.print("Result = 0x");
		Serial.print(value, HEX);
		Serial.print(" FAIL ");
	}
#endif
    } else
        Serial.print("Measurement not Ready\n\r");

    //It's important to Reset SeltTest reg
//    cs = 0;
    digitalWrite(SS, LOW);
    SPI.transfer(RM3100_BIST_REG);
    SPI.transfer(0);
    delay(10);
    digitalWrite(SS, HIGH);
//    cs = 1;

//    __enable_irq();    // Enable Interrupts
    interrupts();

}

void RM3100::ChangeCycleCount(int flag)
{    
    for (int i = 0; i < 3; i++)
    {
        if (flag > 0) 
        {
            current_ccount[i] += 10;
        } 
        else if (flag < 0) 
        {
            current_ccount[i] -= 10;
        }
        
        if (current_ccount[i] < LOWERCYCLECOUNT)
            current_ccount[i] = LOWERCYCLECOUNT;
            
        if (current_ccount[i] > UPPERCYCLECOUNT)
            current_ccount[i] = UPPERCYCLECOUNT;
          
    }
}


//Set Cycle Count Reg
void RM3100::SetCycleCountReg()
{   
    static int prev_ccount = DEFAULTCCOUNT;
    
    //Check any changes on CC. Should check 3 axes, just X-axis as example
    if (prev_ccount != current_ccount[0]) {
        
        prev_ccount = current_ccount[0];

        //Read CC reg
//        __disable_irq();    // Disable Interrupts
	noInterrupts();
//        cs = 0;
    digitalWrite(SS, LOW);
        int cc_reg = RM3100_CCXLSB_REG;
        SPI.transfer(cc_reg);

        for (int i = 0; i < 3; i++) {
            SPI.transfer((current_ccount[i] & 0xFF00)>>8); //MSB
            SPI.transfer(current_ccount[i] & 0x00FF); //LSB
        }
    delay(10);
    digitalWrite(SS, HIGH);
//        cs = 1;

//        __enable_irq();     // Enable Interrupts
    interrupts();
    }
}


//constructor
RM3100::RM3100(): rm3100_service_flag(0),   //init to 0
    sample_rate(1), //init to 1 Hz
    prev_rate(1)  //init to 1 Hz
//    cs(10),     //init to SPI_CS
//    drdy(9),        //init to D9
// spi(SPI_MOSI, SPI_MISO, SPI_SCK) //spi pins init
// SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
{ 
    //Init to default
    for (int i = 0; i < 3; i++)
    {
        current_ccount[i] = DEFAULTCCOUNT;
        current_gain[i] = DEFAULTGAIN;
    }
        
};
