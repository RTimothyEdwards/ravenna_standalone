#include "../ravenna_defs.h"

// command register bits
// bit 7 = start
// bit 6 = stop
// bit 5 = read
// bit 4 = write

// control register bits
// bit 27 = acknowledge
// bit 24 = interrupt acknowledge
// bit 23 = enable
// bit 22 = interrupt enable

// bits 15-0:  clock prescaler

#define     I2C_CMD_STA         0x80
#define     I2C_CMD_STO         0x40
#define     I2C_CMD_RD          0x20
#define     I2C_CMD_WR          0x10
#define     I2C_CMD_ACK         0x08
#define     I2C_CMD_IACK        0x01

#define     I2C_CTRL_EN         0x80
#define     I2C_CTRL_IEN        0x40

// status regiter bits:
// bit 7 = receive acknowledge
// bit 6 = busy (start signal detected)
// bit 5 = arbitration lost
// bit 1 = transfer in progress
// bit 0 = interrupt flag

#define     I2C_STAT_RXACK      0x80
#define     I2C_STAT_BUSY       0x40
#define     I2C_STAT_AL         0x20
#define     I2C_STAT_TIP        0x02
#define     I2C_STAT_IF         0x01

// --------------------------------------------------------

void i2c_init(unsigned int pre) {

    reg_i2c_control = (uint16_t)(I2C_CTRL_EN | I2C_CTRL_IEN);
    reg_i2c_prescale = (uint16_t)pre;
}

// --------------------------------------------------------

int i2c_send(unsigned char saddr, unsigned char sdata) {

    int volatile y;
    reg_i2c_data = saddr;
    reg_i2c_command = I2C_CMD_STA | I2C_CMD_WR;

    while ((reg_i2c_status & I2C_STAT_TIP) != 0);

    if ((reg_i2c_status & I2C_STAT_RXACK)  == 1) {
        reg_i2c_command = I2C_CMD_STO;
        return 0;
    }

    reg_i2c_data = sdata;
    reg_i2c_command = I2C_CMD_WR;

    while (reg_i2c_status & I2C_STAT_TIP);
    reg_i2c_command = I2C_CMD_STO;

    if ((reg_i2c_status & I2C_STAT_RXACK) == 1)
        return 0;
    else
        return 1;
}

// --------------------------------------------------------

void main()
{
    int r;

    // Testbench to be completed:  Set up configuration
    // then read and write a few bytes to confirm signalling.

    reg_i2c_config = 0;
    reg_i2c_data = 0;

    // Enable I2C with prescaler set to 5
    i2c_init(5);

    // Send command 6, data byte 0xfa
    r = i2c_send(0x6, 0xfa);
}
