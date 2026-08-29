#ifndef APP_I2C_H
#define APP_I2C_H

#include <stdbool.h>
#include <stdint.h>

/* I2C1 is application-owned because production firmware intentionally keeps
 * lib/ unchanged. Pins match the robot PCB: PB8=SCL, PB7=SDA, AF4. */
void AppI2c1_Init(void);

/* addr7 is the unshifted 7-bit address. Transactions have bounded polling
 * timeouts and abort/recover the peripheral after NACK or timeout. */
bool AppI2c1_WriteReg(uint8_t addr7, uint8_t reg, uint8_t value);
bool AppI2c1_ReadRegs(uint8_t addr7, uint8_t reg, uint8_t *buffer, uint8_t length);

#endif /* APP_I2C_H */
