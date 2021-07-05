#include <Sodaq_LSM303C.h>

Sodaq_LSM303C accel;

volatile bool magInterruptFlag = true;

void magInterrupt()
{
    magInterruptFlag = true;
}

void setup()
{
    SerialUSB.begin(57600);
    delay(1000);

    SerialUSB.println("BEGIN");
    Wire.begin();
    delay(1000);

    if (accel.checkWhoAmI()) {
        SerialUSB.println("FOUND ACCEL!");
    }
    else {
        SerialUSB.println("NO ACCEL!");
    }

    accel.rebootMagnetometer();
    delay(1000);

    accel.enableMagnetometer(Sodaq_LSM303C::MagLowPowerMode, Sodaq_LSM303C::Hz80, Sodaq_LSM303C::Continuous);

    // uint8_t axes = Sodaq_LSM303C::MagX;
    // accel.enableMagnetometerInterrupt(axes, -400);

    // pinMode(MAG_INT, INPUT_PULLDOWN);
    // attachInterrupt(MAG_INT, magInterrupt, RISING);
}

void loop()
{
    delay(1000);

    SerialUSB.println(accel.getTemperature());

    // SerialUSB.print(accel.getMagX());
    // SerialUSB.print(" ");
    // SerialUSB.print(accel.getMagY());
    // SerialUSB.print(" ");
    // SerialUSB.println(accel.getMagZ());

    // if (magInterruptFlag) {
    //     SerialUSB.println("INTERRUPT");
    //     magInterruptFlag = false;
    // }
}
