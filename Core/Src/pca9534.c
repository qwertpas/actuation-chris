#include "pca9534.h"

uint8_t pca9534_read_reg(I2C_HandleTypeDef *hi2c, uint8_t addr, uint8_t reg) {
    addr <<= 1;

    uint8_t read_data = 0;
    HAL_I2C_Mem_Read(hi2c, addr, reg, 1, &read_data, 1, 10);

    return read_data;
}

void pca9534_write_reg(I2C_HandleTypeDef *hi2c, uint8_t addr, uint8_t reg, uint8_t write_data) {
    addr <<= 1;

    HAL_I2C_Mem_Write(hi2c, addr, reg, 1, &write_data, 1, 10);
}

void pca9534_init_output(I2C_HandleTypeDef *hi2c, uint8_t addr) {
    // configure all ports to be outputs
    pca9534_write_reg(hi2c, addr, PCA9534_REG_CONFIG, 0x00);
}

void pca9534_init_input(I2C_HandleTypeDef *hi2c, uint8_t addr) {
    // configure all ports to be inputs
    pca9534_write_reg(hi2c, addr, PCA9534_REG_CONFIG, 0xFF);
}

void pca9534_set_channel(I2C_HandleTypeDef *hi2c, uint8_t addr, uint8_t channel, uint8_t set) {
    uint8_t state = pca9534_read_reg(hi2c, addr, PCA9534_REG_OUTPUT);

    uint8_t n = channel;

    if (set) {
        state |= (1 << n);
    } else {
        state &= ~(1 << n);
    }
    
    pca9534_write_reg(hi2c, addr, PCA9534_REG_OUTPUT, state);
}