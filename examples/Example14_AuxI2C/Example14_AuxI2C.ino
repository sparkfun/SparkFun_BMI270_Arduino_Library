#include <Wire.h>
#include "SparkFun_BMI270_Arduino_Library.h"

// Create a new sensor object
BMI270 imu;

// I2C address selection
uint8_t i2cAddress = BMI2_I2C_PRIM_ADDR; // 0x68
//uint8_t i2cAddress = BMI2_I2C_SEC_ADDR; // 0x69

void setup()
{
    // Start serial
    Serial.begin(115200);
    Serial.println("BMI270 Example 14 - Auxiliary Sensor");

    // Initialize the I2C library
    Wire.begin();

    // Check if sensor is connected and initialize
    // Address is optional (defaults to 0x68)
    while(imu.beginI2C(i2cAddress) != BMI2_OK)
    {
        // Not connected, inform user
        Serial.println("Error: BMI270 not connected, check wiring and I2C address!");

        // Wait a bit to see if connection is established
        delay(1000);
    }

    Serial.println("BMI270 connected!");
    
    // Here we enable the auxiliary interface of the BMI270, which can be used
    // to attach an additional I2C sensor. The BMI270 can then automatically
    // acquire data from the auxiliary sensor without input from the host
    // microcontroller
    imu.enableFeature(BMI2_AUX);

    // We can configure the parameters for the auxiliary I2C bus. The following
    // config options are available:
    // 
    // .aux_en          - Whether to enableor disable the aux interface
    // .manual_en       - Manual or automatic mode
    // .man_rd_burst    - Number of bytes to burst read in manual mode
    // .aux_rd_burst    - Number of bytes to burst read in automatic mode
    // .odr             - Measurement rate in automatic mode
    // .i2c_device_addr - Auxiliary sensor I2C address (7-bit unshifted)
    // .read_addr       - First register address to be read from auxiliary
    //                    sensor in automatic mode
    // .fcu_write_en    - Some sensors require a trigger command to be sent by
    //                    writing a certain value to a register in order to take
    //                    a measurement. This is supported by enabling this
    // .offset          - When fcu_write_en is enabled, this specifies the delay
    //                    between sending the trigger command and reading the
    //                    data (2.5ms resolution). However if the offset is
    //                    zero, the trigger command is instead sent immediately
    //                    after the previous read
    // 
    // Here we configure the aux bus in manual mode. This example assumes a
    // BMP581 is attached to the aux bus, with an I2C address of 0x47 and 6 data
    // registers starting at address 0x1D
    bmi2_sens_config auxConfig;
    auxConfig.type = BMI2_AUX;
    auxConfig.cfg.aux.aux_en = BMI2_ENABLE;
    auxConfig.cfg.aux.manual_en = BMI2_ENABLE;
    auxConfig.cfg.aux.man_rd_burst = BMI2_AUX_RD_BURST_FRM_LEN_1;
    auxConfig.cfg.aux.aux_rd_burst = BMI2_AUX_RD_BURST_FRM_LEN_6;
    auxConfig.cfg.aux.odr = BMI2_AUX_ODR_1_56HZ;
    auxConfig.cfg.aux.i2c_device_addr = 0x47;
    auxConfig.cfg.aux.read_addr = 0x1D;
    auxConfig.cfg.aux.fcu_write_en = BMI2_DISABLE;
    auxConfig.cfg.aux.offset = 0;
    imu.setConfig(auxConfig);

    // If needed, pullup resistors can be enabled on the BMI270's aux pins
    imu.setAuxPullUps(BMI2_ASDA_PUPSEL_2K);

    // This example assumes a BMP581 is attached to the aux bus. We'll first
    // read the CHIP_ID register to confirm it's correct (should be 0x50)
    imu.readAux(0x01, 1);
    Serial.print("Aux sensor chip ID (hex): ");
    Serial.println(imu.data.auxData[0], HEX);

    // Send reset command to BMP581
    imu.writeAux(0x7E, 0xB6);

    // BMP581 takes 2ms to reset
    delay(2);

    // Enable BMP581 by setting the pwr_mode and press_en bits
    imu.writeAux(0x37, 0b00101001);
    imu.writeAux(0x36, 0b01000000);
    
    // Switch to automatic mode
    auxConfig.cfg.aux.manual_en = BMI2_DISABLE;
    imu.setConfig(auxConfig);
}

void loop()
{
    // In automatic mode, auxiliary sensor data is automatically read by the
    // BMI270, but we still need to read it from the BMI270. Calling
    // getSensorData() will get the auxiliary data as well as normal sensor data
    imu.getSensorData();

    // If the aux bus is in manual mode, use readAux() to get data from the
    // external sensor
    // imu.readAux(0x1D, 6);

    // Combine bytes into raw data
    uint32_t tempRaw = (imu.data.auxData[2] << 16) | (imu.data.auxData[1] << 8) | imu.data.auxData[0];
    uint32_t presRaw = (imu.data.auxData[5] << 16) | (imu.data.auxData[4] << 8) | imu.data.auxData[3];

    // Convert raw data to have proper units
    float temperature = tempRaw / 65536.0;
    float pressure = presRaw / 64.0;

    // Print data
    Serial.print("Temperature (C): ");
    Serial.print(temperature);
    Serial.print("\t\t");
    Serial.print("Pressure (Pa): ");
    Serial.println(pressure);

    // Wait for next measurement
    delay(1000);
}