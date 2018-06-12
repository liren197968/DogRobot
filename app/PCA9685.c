#include "pin_mux.h"

#include "fsl_i2c.h"
#include "fsl_gpio.h"
#include "fsl_iocon.h"

#include "app_interrupt.h"
#include "PCA9685.h"


#define PCA9685_IIC     I2C1


static void Pca9685GpioInit(void)
{
    i2c_slave_config_t      slaveConfig;
    i2c_master_config_t     masterConfig;
    gpio_pin_config_t       config={kGPIO_DigitalOutput,0};

    CLOCK_AttachClk(kFRO_HF_to_FLEXCOMM1);
    RESET_PeripheralReset(kFC1_RST_SHIFT_RSTn);

    IOCON_PinMuxSet(IOCON, 0, 23, IOCON_FUNC1 | IOCON_DIGITAL_EN | IOCON_INPFILT_OFF);
    IOCON_PinMuxSet(IOCON, 0, 24, IOCON_FUNC1 | IOCON_DIGITAL_EN | IOCON_INPFILT_OFF);

    I2C_MasterGetDefaultConfig(&masterConfig);
    masterConfig.baudRate_Bps = 100000U;
    masterConfig.enableMaster = 1;
    I2C_MasterInit((I2C_Type *)PCA9685_IIC, &masterConfig, 12000000);

    GPIO_PinInit(GPIO, 0U, 15U, &config);
    GPIO_PinInit(GPIO, 0U, 19U, &config);
}

static StatusFlag Pca9685WriteReg(uint8_t SlaveAddr, uint8_t Reg, uint8_t Dat)
{
    uint8_t                 Buffer[2] = {Reg, Dat};
    StatusFlag              Result = 0;
    i2c_master_transfer_t   Transfer;

    Transfer.flags = kI2C_TransferDefaultFlag;
    Transfer.slaveAddress = SlaveAddr;
    Transfer.direction = kI2C_Write;
    Transfer.subaddress = 0U;
    Transfer.subaddressSize = 0U;
    Transfer.data = Buffer;
    Transfer.dataSize = sizeof(Buffer);

    Result = I2C_MasterTransferBlocking(PCA9685_IIC, &Transfer);

    if (Result == kStatus_Success)
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

    Result = I2C_MasterTransferBlocking(PCA9685_IIC, &Transfer);
    if(Result != kStatus_Success)
    {
        return RUN_ERROR;
    }

    return RUN_SUCCESS;
}

static void Pca9685SetPwmFreq(uint8_t SlaveAddr, uint8_t Freq)
{
    uint8_t                 Prescale = 0, Prescaleval = 0;
    uint8_t                 OldMode = 0, NewMode = 0;

    Freq *= 0.92;                                                       // Correct for overshoot in the frequency setting (see issue #11).

    Prescaleval = (uint8_t)(25000000/(4096 * Freq));
    Prescaleval -= 1;
    Prescale = (uint8_t)(Prescaleval + 0.5);

    Pca9685ReadReg(SlaveAddr, PCA9685_MODE1, &OldMode);
    NewMode = (OldMode & 0x7F) | 0x10;                                  // sleep

    Pca9685WriteReg(SlaveAddr, PCA9685_MODE1, NewMode);                 // go to sleep
    Pca9685WriteReg(SlaveAddr, PCA9685_PRESCALE, Prescale);             // set the prescaler
    Pca9685WriteReg(SlaveAddr, PCA9685_MODE1, OldMode);
    HalDelayMs(5);

    Pca9685WriteReg(SlaveAddr, PCA9685_MODE1, OldMode | 0xA1);          // This sets the MODE1 register to turn on auto increment.
}

StatusFlag Pca9685OutPwm(uint8_t SlaveAddr, uint8_t Num, uint16_t HigBitDat, uint16_t LowBitDat)
{
    uint8_t                 Buffer[5];
    StatusFlag              Result = 0;
    i2c_master_transfer_t   Transfer;

    Buffer[0] = LED0_ON_L + Num * 4;
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

    Result = I2C_MasterTransferBlocking(PCA9685_IIC, &Transfer);
    if (Result == kStatus_Success)
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
void Pca9685SetPin(uint8_t SlaveAddr, uint8_t Num, uint16_t Val, uint8_t Invert)
{
    if(Val > 4095)  Val = 4095;

    if(Invert)
    {
        if(Val == 0)
        {
            // Special value for signal fully on.
            Pca9685OutPwm(SlaveAddr, Num, 4096, 0);
        }
        else if (Val == 4095) 
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
        else if (Val == 0) 
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

void Pca9685Enable(void)
{
    GPIO_WritePinOutput(GPIO, 0U, 15U, 0U);
    GPIO_WritePinOutput(GPIO, 0U, 19U, 0U);
}

void Pca9685Disable(void)
{
    GPIO_WritePinOutput(GPIO, 0U, 15U, 1U);
    GPIO_WritePinOutput(GPIO, 0U, 19U, 1U);
}

void Pca9685Init(void)
{
    Pca9685GpioInit();
    Pca9685Enable();

    Pca9685WriteReg(PWM_ADDRESS_L, PCA9685_MODE1, 0x00);
    Pca9685WriteReg(PWM_ADDRESS_H, PCA9685_MODE1, 0x00);

    Pca9685SetPwmFreq(PWM_ADDRESS_L, 50);
    Pca9685SetPwmFreq(PWM_ADDRESS_H, 50);
}
