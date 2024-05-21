# INA219

Platform-agnostic library for INA219 current sensor for embedded systems.

## Usage

See the following base example:

```c
#include "INA219.h"

int main() {
    // Initialize the INA219 with your platform-specific I2C functions
    struct INA219_Platform platform = {
        .i2cWriteReg = my_i2c_write_reg,
        .i2cReadReg = my_i2c_read_reg,
        .debugPrint = printf,
        .i2cAddress = INA219_I2C_ADDR_DEF,
    };
    INA219_Init(&platform);

    // Set the configuration
    INA219_SetConfiguration(INA219_BUS_RANGE_32V, INA219_PGA_8_320mv, INA219_ADC_12BIT_532us, INA219_ADC_12BIT_532us, INA219_MODE_SHUNT_AND_BUS_CONT);

    // Set the calibration for a 0.1 ohm shunt resistor and a maximum expected current of 3.2A
    INA219_SetCalibration(3.2f, 0.1f);

    float voltage, current, power;
    bool convReady;

    // Read the bus voltage
    if (INA219_ReadVoltage(&voltage, &convReady)) {
        printf("Bus Voltage: %.3f V (conversion ready: %d)\n", voltage, convReady);
    }

    // Read the current
    if (INA219_ReadCurrent(&current)) {
        printf("Current: %.3f A\n", current);
    }

    // Read the power
    if (INA219_ReadPower(&power)) {
        printf("Power: %.3f W\n", power);
    }

    return 0;
}
```

## Author

Yuriy Kapoyko - ykapoyko@vk.com
