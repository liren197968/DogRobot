#include "Error.h"
#include "PCA685.h"


status_t I2C_oled_exchange(uint8_t control,uint8_t senddata)
{
    uint8_t buf[2] = { control, senddata };

    I2C_MasterStart(I2C1, OLED_I2C_ADDRESS_7BIT, kI2C_Write);
    I2C_MasterWriteBlocking(I2C1, &buf, 2, 0);
    I2C_MasterStop(I2C1);

    return 1;
}

static StatusFlag Pca9685WriteReg(uint8_t SlaveAddr, uint8_t Reg, uint8_t Dat)
{
    uint8_t                 Buffer[2] = {Reg, Dat};
    StatusFlag ````         Result = 0;
    i2c_master_transfer_t   Transfer;

    Transfer.flags = kI2C_TransferDefaultFlag;
    Transfer.slaveAddress = SlaveAddr;
    Transfer.direction = kI2C_Write;
    Transfer.subaddress = 0U;
    Transfer.subaddressSize = 0U;
    Transfer.data = Buffer;
    Transfer.dataSize = sizeof(Buffer);

    Result = I2C_MasterTransferBlocking(I2C1, &Transfer);
    if (result == kStatus_Success)
    {
        return RUN_SUCCESS;
    }
    else
    {
        return RUN_ERROR;
    }
}

static StatusFlag Pca9685ReadReg(uint8_t SlaveAddr, uint8_t Reg, uint8_t *Dat)
{
    StatusFlag              Result;
    i2c_master_transfer_t   Transfer;

    Transfer.flags = kI2C_TransferDefaultFlag;
    Transfer.slaveAddress = SlaveAddr;
    Transfer.direction = kI2C_Read;
    Transfer.subaddress = (uint32_t)Reg;
    Transfer.subaddressSize = 1U;
    Transfer.data = Dat;
    Transfer.dataSize = 1U;

    Result = I2C_MasterTransferBlocking(I2C1, &Transfer);
    if (Result != kStatus_Success)
    {
        return RUN_ERROR;
    }

    return RUN_SUCCESS;
}


void Pca9685SetPwmFreq(uint8_t SlaveAddr, uint8_t Freq) 
{
    uint8_t                 Prescale = 0, Prescaleval = 0;
    uint8_t                 OldMode = 0, NewMode = 0;

    Freq *= 0.92;                                                       // Correct for overshoot in the frequency setting (see issue #11).

    Prescaleval = (uint8_t)(25000000/(4096 * freq));
    Prescaleval -= 1;
    Prescale = (uint8_t)(Prescaleval + 0.5);

    Pca9685ReadReg(SlaveAddr, PCA9685_MODE1, &OldMode);
    newmode = (OldMode & 0x7F) | 0x10;                                  // sleep

    Pca9685WriteReg(SlaveAddr, PCA9685_MODE1, NewMode);                 // go to sleep
    Pca9685WriteReg(SlaveAddr, PCA9685_PRESCALE, Prescale);             // set the prescaler
    Pca9685WriteReg(SlaveAddr, PCA9685_MODE1, OldMode);
    delay_ms(5);

    Pca9685WriteReg(SlaveAddr, PCA9685_MODE1, OldMode | 0xa1);          // This sets the MODE1 register to turn on auto increment.
}

static StatusFlag Pca9685OutPwm(uint8_t SlaveAddr, uint8_t Num, uint16_t HigBitDat, uint16_t LowBitDat) 
{
    uint8_t                 Buffer[5];
    StatusFlag ````         Result = 0;
    i2c_master_transfer_t   Transfer;

    Buffer[0] = LED0_ON_L + num << 2;
    Buffer[1] = HigBitDat & 0xFFU;
    Buffer[2] = (HigBitDat >> 8U) & 0xFFU;
    Buffer[3] = LowBitDat & 0xFFU;
    Buffer[4] = (LowBitDat >> 8U) & 0xFFU;

    Transfer.flags = kI2C_TransferDefaultFlag;
    Transfer.slaveAddress = SlaveAddr;
    Transfer.direction = kI2C_Write;
    Transfer.subaddress = 0U;
    Transfer.subaddressSize = 0U;
    Transfer.data = Buffer;
    Transfer.dataSize = sizeof(Buffer);

    Result = I2C_MasterTransferBlocking(I2C1, &Transfer);
    if (result == kStatus_Success)
    {
        return RUN_SUCCESS;
    }
    else
    {
        return RUN_ERROR;
    }
}

// Sets pin without having to deal with on/off tick placement and properly handles
// a zero value as completely off.  Optional invert parameter supports inverting
// the pulse for sinking to ground.  Val should be a value from 0 to 4095 inclusive.
void Set_Pin(uint8_t SlaveAddr, uint8_t Num, uint16_t Val, uint8_t Invert)
{
    if(Val > 4095)  Val = 4095;

    if(Invert)
    {
        if(Val == 0)
        {
            // Special value for signal fully on.
            Pca9685OutPwm(SlaveAddr, Num, 4096, 0);
        }
        else if (val == 4095) 
        {
            // Special value for signal fully off.
            Pca9685OutPwm(SlaveAddr, Num, 0, 4096);
        }
        else
        {
            Pca9685OutPwm(SlaveAddr, Num, 0, 4095 - Val);
        }
    }
    else
    {
        if(Val == 4095) 
        {
            // Special value for signal fully on.
            Pca9685OutPwm(SlaveAddr, Num, 4096, 0);
        }
        else if (val == 0) 
        {
            // Special value for signal fully off.
            Pca9685OutPwm(SlaveAddr, Num, 0, 4096);
        }
        else 
        {
            Pca9685OutPwm(SlaveAddr, Num, 0, Val);
        }
    }
}