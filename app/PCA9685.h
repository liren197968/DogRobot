/* I2C bus Functions */
#ifndef _PCA9685_H
#define _PCA9685_H


#define PCA9685_SUBADR1     0x2
#define PCA9685_SUBADR2     0x3
#define PCA9685_SUBADR3     0x4

#define PCA9685_MODE1       0x0
#define PCA9685_PRESCALE    0xFE

#define LED0_ON_L           0x6
#define LED0_ON_H           0x7
#define LED0_OFF_L          0x8
#define LED0_OFF_H          0x9

#define ALLLED_ON_L         0xFA
#define ALLLED_ON_H         0xFB
#define ALLLED_OFF_L        0xFC
#define ALLLED_OFF_H        0xFD

#define PWM_Address_0       0x80
#define PWM_Address_1       0xC0



#endif
