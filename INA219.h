#ifndef INA219_H
#define INA219_H

#include <stdint.h>
#include <stdbool.h>

#define INA219_I2C_ADDR_DEF 0b1000000u // I2C slave address (default)

#define INA219_REG_UPDATE_DELAY_US 4

#define INA219_BUS_VOLTAGE_RANGE_DEF INA219_BUS_RANGE_32V
#define INA219_PGA_DEF INA219_PGA_8_320mv
#define INA219_ADC_DEF INA219_ADC_12BIT_532us
#define INA219_MODE_DEF INA219_MODE_SHUNT_AND_BUS_CONT

enum INA219_BusVoltageRange {
    INA219_BUS_RANGE_16V = 0b00,
    INA219_BUS_RANGE_32V = 0b01
};

enum INA219_PGAGainRange {
    INA219_PGA_1_40mv = 0b00,
    INA219_PGA_2_80mv = 0b01,
    INA219_PGA_4_160mv = 0b10,
    INA219_PGA_8_320mv = 0b11
};

enum INA219_ADCSettings {
    INA219_ADC_9BIT_84us = 0b0000,
    INA219_ADC_10BIT_148us = 0b0001,
    INA219_ADC_11BIT_276us = 0b0010,
    INA219_ADC_12BIT_532us = 0b0011,
    INA219_ADC_AVER_1_532us = 0b1000,
    INA219_ADC_AVER_2_1_06ms = 0b1001,
    INA219_ADC_AVER_4_2_13ms = 0b1010,
    INA219_ADC_AVER_8_4_26ms = 0b1011,
    INA219_ADC_AVER_16_8_51ms = 0b1100,
    INA219_ADC_AVER_32_17_02ms = 0b1101,
    INA219_ADC_AVER_64_34_05ms = 0b1110,
    INA219_ADC_AVER_128_68_10ms = 0b1111
};

enum INA219_OperatingMode {
    INA219_MODE_POWERDOWN = 0b000,
    INA219_MODE_SHUNT_V_TRIG = 0b001,
    INA219_MODE_BUS_V_TRIG = 0b010,
    INA219_MODE_SHUNT_AND_BUS_TRIG = 0b011,
    INA219_MODE_DISABLED = 0b100,
    INA219_MODE_SHUNT_V_CONT = 0b101,
    INA219_MODE_BUS_V_CONT = 0b110,
    INA219_MODE_SHUNT_AND_BUS_CONT = 0b111
};

struct INA219_Platform {
    int (*i2cWriteReg)(uint8_t addr7bit, uint8_t regNum, const uint8_t *data, uint8_t length, uint8_t wait);
    int (*i2cReadReg)(uint8_t addr7bit, uint8_t regNum, uint8_t *data, uint8_t length, int timeout);
    void (*debugPrint)(const char *fmt, ...);

    uint8_t i2cAddress;
};

void INA219_Init(struct INA219_Platform *platform);

bool INA219_Reset(void);
bool INA219_SetConfiguration(enum INA219_BusVoltageRange brng, enum INA219_PGAGainRange pg,
    enum INA219_ADCSettings busAdc, enum INA219_ADCSettings shuntAdc, enum INA219_OperatingMode mode);
bool INA219_SetCalibration(float maxExpectedCurrent, float rShunt);

bool INA219_ReadShuntVoltage(float *voltage);

// Read bus voltage
// voltage - outputs measured voltage
// convReady - outputs true if conversion is ready (clears on mode change or power register read), optional
// Returns true on success, false on failure (I2C error or math overflow).
bool INA219_ReadVoltage(float *voltage, bool *convReady);

bool INA219_ReadPower(float *power);
bool INA219_ReadCurrent(float *current);

#endif // INA219_H
