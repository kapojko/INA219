#include "INA219.h"

#define I2C_READ_TIMEOUT_MS 5

#define INA219_REG_CONFIGURATION 0x00
#define INA219_REG_SHUNT_VOLTAGE 0x01
#define INA219_REG_BUS_VOLTAGE 0x02
#define INA219_REG_POWER 0x03
#define INA219_REG_CURRENT 0x04
#define INA219_REG_CALIBRATION 0x05

static struct INA219_Platform *platform = { 0 };

static float Current_LSB = 0.001; // default value 1 mA, will be overwritten by calibration set

void INA219_Init(struct INA219_Platform *platformPtr) {
    platform = platformPtr;
}

bool INA219_Reset(void) {
    int ret;

    // reset INA using configuration register
    // note: set only reset bit (bit 15)
    // datasheet: Setting this bit to '1' generates a system reset that is the same as power-on reset. 
    // Resets all registers to default values; this bit self-clears.
    uint8_t configReg[2] = { 0x80, 0x00 };
    ret = platform->i2cWriteReg(platform->i2cAddress, INA219_REG_CONFIGURATION, configReg, sizeof(configReg), 1);
    if (ret < 0) {
        platform->debugPrint("Error writing INA219 configuration register to reset INA: %d\r\n", -ret);
        return false;
    }

    return true;
}

bool INA219_SetConfiguration(enum INA219_BusVoltageRange brng, enum INA219_PGAGainRange pg,
    enum INA219_ADCSettings busAdc, enum INA219_ADCSettings shuntAdc, enum INA219_OperatingMode mode) {
    int ret;

    uint16_t config = (brng << 13) | (pg << 11) | (busAdc << 7) | (shuntAdc << 3) | mode;

    uint8_t configReg[2] = { (config >> 8) & 0xFF, config & 0xFF };
    ret = platform->i2cWriteReg(platform->i2cAddress, INA219_REG_CONFIGURATION, configReg, sizeof(configReg), 1);
    if (ret < 0) {
        platform->debugPrint("Error writing INA219 configuration register: %d\r\n", -ret);
        return false;
    }

    return true;
}

bool INA219_SetCalibration(float maxExpectedCurrent, float rShunt) {
    int err;

    // Calculate calibration (see datasheet)
    float Current_LSB_base = maxExpectedCurrent / 32768.0;
    float CAL_float = 0.04096 / Current_LSB_base / rShunt;
    uint16_t CAL = (uint16_t)(CAL_float);

    // Calculate current LSB from actual rounded CAL value
    Current_LSB = 0.04096 / (float)CAL / rShunt;

    // Write calibration
    // NOTE: CALIBRATION is the value stored in FS15:FS1
    // uint8_t calReg[2] = { (uint8_t)((CAL >> 7) & 0xFF), (uint8_t)((CAL << 1) & 0b11111110) };
    // NOTE: In spite of the datasheet note "(1) FS0 is a void bit and will always be 0. It is not possible to write a 1 to FS0. CALIBRATION is the value stored in FS15:FS1."
    // it seems that actually FS15:FS0 are actually used, but FS0 must be zeroed (possibly)
    uint8_t calReg[2] = { (uint8_t)((CAL >> 8) & 0xFF), (uint8_t)(CAL & 0b11111110) };
    err = platform->i2cWriteReg(platform->i2cAddress, INA219_REG_CALIBRATION, calReg, sizeof(calReg), 1);
    if (err < 0) {
        platform->debugPrint("Error writing INA219 calibration: %d\r\n", -err);
        return false;
    }

    platform->debugPrint("INA219 Calibration: %x (Current LSB: %.6f)\r\n", CAL, Current_LSB);
    return true;
}

bool INA219_ReadShuntVoltage(float *voltage) {
    int err;

    // Read shunt voltage register
    uint8_t shuntVoltageReg[2];
    err = platform->i2cReadReg(platform->i2cAddress, INA219_REG_SHUNT_VOLTAGE, shuntVoltageReg, sizeof(shuntVoltageReg), I2C_READ_TIMEOUT_MS);
    if (err < 0) {
        platform->debugPrint("Error reading INA219 shunt voltage: %d\r\n", -err);
        *voltage = -99.0;
        return false;
    }

    // Convert from twos complement
    uint16_t shuntVoltage_raw = (shuntVoltageReg[0] << 8) | shuntVoltageReg[1];
    int16_t shuntVoltage_signed = *(int16_t *)(&shuntVoltage_raw);
    
    // Convert to V (LSB = 10uV independent on PGA gain)
    float shuntVoltage_float = (float)shuntVoltage_signed * 0.00001f;

    *voltage = shuntVoltage_float;
    return true;
}

bool INA219_ReadVoltage(float *voltage, bool *convReady) {
    int err;

    // Read bus voltage register
    uint8_t busVoltageReg[2];
    err = platform->i2cReadReg(platform->i2cAddress, INA219_REG_BUS_VOLTAGE, busVoltageReg, sizeof(busVoltageReg), I2C_READ_TIMEOUT_MS);
    if (err < 0) {
        platform->debugPrint("Error reading INA219 bus voltage: %d\r\n", -err);
        *voltage = -1.0;
        *convReady = false;
        return false;
    }

    // Extract overflow bit (0)
    int ovf = busVoltageReg[1] & 0b1;
    if (ovf != 0) {
        platform->debugPrint("INA219 bus voltage overflow: %d\r\n", ovf);
        *voltage = -1.0;
        *convReady = false;
        return false;
    }

    // Calculate voltage
    const float BusVoltage_LSB = 0.004; // 4 mV
    uint16_t busVoltage_raw = (busVoltageReg[0] << 5) | (busVoltageReg[1] >> 3); // bits 15:3
    float busVoltage_float = (float)busVoltage_raw * BusVoltage_LSB;
    *voltage = busVoltage_float;

    // Extract conversion ready bit
    int cnvr = busVoltageReg[1] & 0b10;
    if (convReady != 0) {
        *convReady = cnvr != 0;
    }

    return true;
}

bool INA219_ReadPower(float *power) {
    int err;

    // Read power register
    uint8_t powerReg[2];
    err = platform->i2cReadReg(platform->i2cAddress, INA219_REG_POWER, powerReg, sizeof(powerReg), I2C_READ_TIMEOUT_MS);
    if (err < 0) {
        platform->debugPrint("Error reading INA219 power: %d\r\n", -err);
        *power = -1.0;
        return false;
    }


    // Calculate power
    uint16_t power_raw = (powerReg[0] << 8) | powerReg[1];
    float Power_LSB = 20.0f * Current_LSB; // see datasheet
    float power_float = (float)power_raw * Power_LSB;
    *power = power_float;

    return true;
}
bool INA219_ReadCurrent(float *current) {
    int err;

    // Read current register
    uint8_t current_reg[2];
    err = platform->i2cReadReg(platform->i2cAddress, INA219_REG_CURRENT, current_reg, sizeof(current_reg), I2C_READ_TIMEOUT_MS);
    if (err < 0) {
        platform->debugPrint("Error reading INA219 current: %d\r\n", -err);
        *current = -99.0;
        return false;
    }

    // Calculate current
    // NOTE: encoding is two's complement, see datasheet
    uint16_t current_raw = (current_reg[0] << 8) | current_reg[1];
    int16_t current_signed = *(int16_t *)(&current_raw);
    float current_float = (float)current_signed * Current_LSB;
    *current = current_float;

    return true;
}
