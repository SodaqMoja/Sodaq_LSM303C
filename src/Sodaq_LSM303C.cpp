#include <Arduino.h>
#include <math.h>
#include "Sodaq_LSM303C.h"

#define _BV(bit) (1 << (bit))

double mapDouble(double x, double in_min, double in_max, double out_min, double out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

Sodaq_LSM303C::Sodaq_LSM303C(TwoWire& wire, uint8_t accelAddress, uint8_t magAddress) :
    _wire(wire),
    _accelAddress(accelAddress),
    _magAddress(magAddress),
    _accelScale(Scale2g)
{

}

int16_t Sodaq_LSM303C::getTemperature()
{
    // return readMagRegister16Bits(TEMP_L_M);
    uint8_t data_h = 0x00;
    uint8_t data_l = 0x00;

    data_h = readMagRegister(TEMP_H_M);
    data_l = readMagRegister(TEMP_L_M);

    uint16_t temp = (int16_t)((data_h << 8) | data_l);
    temp = temp/8;

    return temp;
}

// int8_t Sodaq_LSM303C::getTemperature()
// {
//     int16_t value = readMagRegister16Bits(TEMP_L_M);
//     SerialUSB.println(value);

//     if (_accelMode == AccelerometerMode::HighResMode || _accelMode == AccelerometerMode::NormalMode) {
//         value /= pow(2, 6); // 12-bit value

//         return value / 4.0f + 25.0f;
//     }
//     else if (_accelMode == AccelerometerMode::LowPowerMode) {
//         value /= pow(2, 8); // 8-bit value

//         return value + 25.0f;
//     }

//     return 0.0f;
// }

double Sodaq_LSM303C::getGsFromScaledValue(int16_t value)
{
    if (_accelMode == AccelerometerMode::HighResMode) {
        value /= pow(2, 4); // 12-bit value

        switch (_accelScale)
        {
            case Scale::Scale2g: return value * 1.0f / 1000.0f;
            case Scale::Scale4g: return value * 2.0f / 1000.0f;
            case Scale::Scale8g: return value * 4.0f / 1000.0f;
            case Scale::Scale16g: return value * 12.0f / 1000.0f;
            default:
                break;
        }
    }
    else if (_accelMode == AccelerometerMode::NormalMode) {
        value /= pow(2, 6); // 10-bit value

        switch (_accelScale)
        {
            case Scale::Scale2g: return value * 4.0f / 1000.0f;
            case Scale::Scale4g: return value * 8.0f / 1000.0f;
            case Scale::Scale8g: return value * 16.0f / 1000.0f;
            case Scale::Scale16g: return value * 48.0f / 1000.0f;
            default:
                break;
        }
    }
    else if (_accelMode == AccelerometerMode::LowPowerMode) {
        value /= pow(2, 8); // 8-bit value

        switch (_accelScale)
        {
            case Scale::Scale2g: return value * 16.0f / 1000.0f;
            case Scale::Scale4g: return value * 32.0f / 1000.0f;
            case Scale::Scale8g: return value * 64.0f / 1000.0f;
            case Scale::Scale16g: return value * 192.0f / 1000.0f;
            default:
                break;
        }
    }

    return 0.0f;
}

double Sodaq_LSM303C::getMagFromScaledValue(int16_t value)
{
    return value * 1.5;
}

bool Sodaq_LSM303C::checkWhoAmI()
{
    SerialUSB.println(readAccelRegister(WHO_AM_I_A));
    SerialUSB.println(readAccelRegister(WHO_AM_I_M));

    return (readAccelRegister(WHO_AM_I_A) == 0b01000001) &&
            (readMagRegister(WHO_AM_I_M) == 0b00111101);
}

void Sodaq_LSM303C::enableAccelerometer(AccelerometerMode mode, AccelerometerODR odr, Axes axes, Scale scale)
{
    // set odr, mode, enabled axes
    // Note: the values of AccelerometerMode are 0b(LPen,HR)
    uint8_t ctrlReg1A = (odr << ODR0) | (((mode & 0b10) == 0b10) << LP) | axes;
    writeAccelRegister(CTRL_REG1_A, ctrlReg1A);
    uint8_t ctrlReg4A = readAccelRegister(CTRL_REG4_A);
    if ((mode & 0b01) == 0b01) {
        ctrlReg4A |= _BV(HR);
    }
    else {
        ctrlReg4A &= ~_BV(HR);
    }

    // enable BDU
    ctrlReg4A |= _BV(BDU_A);

    // set scale
    ctrlReg4A &= ~(0b11 << FS0A); // first unset all FS bits
    ctrlReg4A |= (scale << FS0A);

    // write the value to CTRL_REG4_A
    writeAccelRegister(CTRL_REG4_A, ctrlReg4A);

    _accelScale = scale;
    _accelMode = mode;
}

void Sodaq_LSM303C::enableMagnetometer(MagnetometerMode mode, MagnetometerODR odr, MagnetometerSystemMode systemMode, bool enableLPF, bool isTemperatureOn)
{
    // set odr, mode, systemMode
    writeMagRegister(CTRL_REG1_M, (odr << DO0) );
    writeMagRegister(CTRL_REG3_M, (systemMode << MD0) | 0b10000000);

    if (mode == MagLowPowerMode) {
        setMagRegisterBits(CTRL_REG3_M, _BV(LP));
    }
    else {
        unsetMagRegisterBits(CTRL_REG3_M, _BV(LP));
    }

    // disable offset cancellation
    // unsetMagRegisterBits(CFG_REG_B_M, _BV(OFF_CANC));

    setLPF(enableLPF);

    if (isTemperatureOn) {
        // enable aux ADC and temperature sensor
        setMagRegisterBits(CTRL_REG1_M, _BV(TEMP_EN));
    }
    else {
        // disable aux ADC and temperature sensor
        unsetMagRegisterBits(CTRL_REG1_M, _BV(TEMP_EN));
    }
    unsetMagRegisterBits(CTRL_REG1_M, _BV(1));
}

void Sodaq_LSM303C::setLPF(bool enabled)
{
    if (enabled) {
        setMagRegisterBits(CTRL_REG3_M, _BV(LP));
    }
    else {
        unsetMagRegisterBits(CTRL_REG3_M, _BV(LP));
    }
}

void Sodaq_LSM303C::disableAccelerometer()
{
    enableAccelerometer(LowPowerMode, PowerDown_A, NoAxis, _accelScale);
}

void Sodaq_LSM303C::disableMagnetometer()
{
    enableMagnetometer(MagLowPowerMode, Hz10, PowerDown_M, false, true);
}

void Sodaq_LSM303C::rebootAccelerometer()
{
    writeAccelRegister(CTRL_REG6_A, _BV(BOOT));
}

void Sodaq_LSM303C::rebootMagnetometer()
{
    writeMagRegister(CTRL_REG2_M, _BV(REBOOT));
}

void Sodaq_LSM303C::setRegisterBits(uint8_t deviceAddress, Register reg, uint8_t byteValue)
{
    uint8_t value = readRegister(deviceAddress, reg);
    value |= byteValue;
    writeRegister(deviceAddress, reg, value);
}

void Sodaq_LSM303C::unsetRegisterBits(uint8_t deviceAddress, Register reg, uint8_t byteValue)
{
    uint8_t value = readRegister(deviceAddress, reg);
    value &= ~byteValue;
    writeRegister(deviceAddress, reg, value);
}

uint8_t Sodaq_LSM303C::getScaledInterruptThreshold(double threshold)
{
    uint8_t divider = 0; // divider in mg

    switch (_accelScale)
    {
    case Scale::Scale2g: divider = 16;
        break;
    case Scale::Scale4g: divider = 32;
        break;
    case Scale::Scale8g: divider = 62;
        break;
    case Scale::Scale16g: divider = 186;
        break;
    default:
        break;
    }

    return trunc(threshold * 1000.0f / divider);
}

void Sodaq_LSM303C::enableInterrupt1(uint8_t axesEvents, double threshold, uint8_t duration, InterruptMode interruptMode)
{
    // setup the interrupt
    writeAccelRegister(IG_CFG1_A, interruptMode | (axesEvents & 0b00111111));
    // writeAccelRegister(INT1_THS_A, getScaledInterruptThreshold(threshold));
    writeAccelRegister(IG_DUR1_A, duration); // time duration is INT1_DURATION_A/ODR

    // disable latching
    // unsetAccelRegisterBits(CTRL_REG5_A, _BV(LIR_IG1));

    // enable interrupt generator 1 on INT1
    setAccelRegisterBits(CTRL_REG3_A, _BV(INT_XL_IG1));
}

void Sodaq_LSM303C::disableInterrupt1()
{
    // disable interrupt generator 1
    unsetAccelRegisterBits(CTRL_REG3_A, _BV(INT_XL_IG1));
}

void Sodaq_LSM303C::enableInterrupt2(uint8_t axesEvents, double threshold, uint8_t duration, InterruptMode interruptMode)
{
    // setup the interrupt
    writeAccelRegister(IG_CFG2_A, interruptMode | (axesEvents & 0b00111111));
    // writeAccelRegister(INT2_THS_A, getScaledInterruptThreshold(threshold));
    writeAccelRegister(IG_DUR2_A, duration);  // time duration is INT2_DURATION_A/ODR

    // disable latching
    // unsetAccelRegisterBits(CTRL_REG3_A, _BV(LIR_IG2));

    // enable interrupt generator 2 on INT2
    setAccelRegisterBits(CTRL_REG3_A, _BV(INT_XL_IG2));
}

void Sodaq_LSM303C::disableInterrupt2()
{
    // disable interrupt generator 2 on INT2
    unsetAccelRegisterBits(CTRL_REG3_A, _BV(INT_XL_IG2));
}

void Sodaq_LSM303C::enableMagnetometerInterrupt(uint8_t magAxesEvents, double threshold, bool highOnInterrupt)
{
    // threshold needs to be positive, because mag checks interrupts for -threshold and +threshold always
    if (threshold < 0) {
        threshold = -threshold;
    }

    // set axes
    writeMagRegister(INT_CFG_M, (magAxesEvents << ZIEN));

    // interrupt mode
    if (highOnInterrupt) {
        setMagRegisterBits(INT_CFG_M, _BV(IEA));
    }
    else {
        unsetMagRegisterBits(INT_CFG_M, _BV(IEA));
    }

    // set threshold registers
    int16_t ths = trunc(threshold / 1.5);
    writeMagRegister(INT_THS_L_M, ths & 0x00FF);
    writeMagRegister(INT_THS_H_M, (ths & 0xFF00) >> 8);

    // disable latching
    unsetMagRegisterBits(INT_CFG_M, _BV(IEL));

    // enable mag interrupt
    setMagRegisterBits(INT_CFG_M, _BV(IEN));

    // enable DRDYpin as output
    // setMagRegisterBits(CFG_REG_C_M, _BV(INT_MAG));

    // set mag interrupt to INT_MAG_PIN
    // setMagRegisterBits(CFG_REG_C_M, _BV(INT_MAG_PIN));
}

void Sodaq_LSM303C::disableMagnetometerInterrupt()
{
    // disable mag interrupt
    unsetMagRegisterBits(INT_CFG_M, _BV(IEN));
}

uint8_t Sodaq_LSM303C::readRegister(uint8_t deviceAddress, uint8_t reg)
{
    _wire.beginTransmission(deviceAddress);
    _wire.write((uint8_t)reg);
    _wire.endTransmission();

    _wire.requestFrom(deviceAddress, 1);

    return _wire.read();
}

uint16_t Sodaq_LSM303C::readRegister16Bits(uint8_t deviceAddress, uint8_t reg)
{
    // TODO replace with request of 2 bytes?
    // TODO: don't we need BDU Here?
    uint16_t result = readRegister(deviceAddress, reg);
    result |= readRegister(deviceAddress, reg + 1) << 8;

    return result;
}

void Sodaq_LSM303C::writeRegister(uint8_t deviceAddress, uint8_t reg, uint8_t value)
{
    _wire.beginTransmission(deviceAddress);

    _wire.write((uint8_t)reg);
    _wire.write((uint8_t)value);

    _wire.endTransmission();
}
