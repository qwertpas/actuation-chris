#pragma once

#include "i2c.h"

#define PCA9534_OUTPUT_1 0x20
#define PCA9534_OUTPUT_2 0x21
#define PCA9534_INPUT_1 0x22

#define PCA9534_REG_INPUT    0x00
#define PCA9534_REG_OUTPUT   0x01
#define PCA9534_REG_POLARITY 0x02
#define PCA9534_REG_CONFIG   0x03

uint8_t pca9534_read_reg(I2C_HandleTypeDef *hi2c, uint8_t addr, uint8_t reg);
void pca9534_write_reg(I2C_HandleTypeDef *hi2c, uint8_t addr, uint8_t reg, uint8_t write_data);

void pca9534_init_output(I2C_HandleTypeDef *hi2c, uint8_t addr);
void pca9534_init_input(I2C_HandleTypeDef *hi2c, uint8_t addr);
void pca9534_set_channel(I2C_HandleTypeDef *hi2c, uint8_t addr, uint8_t channel, uint8_t state);
uint8_t pca9534_read_channel(I2C_HandleTypeDef *hi2c, uint8_t addr, uint8_t channel);