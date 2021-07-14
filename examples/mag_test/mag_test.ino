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

    while ((!SerialUSB) && (millis() < 10000)) {
        // Wait 10 seconds for the Serial Monitor
    }

    SerialUSB.println("BEGIN");
    Wire.begin();
    delay(1000);

    accel.rebootAccelerometer();
    delay(1000);
    
    accel.enableAccelerometer();

     if (accel.checkWhoAmI()) {
        SerialUSB.println("FOUND ACCEL!");
    }
    else {
        SerialUSB.println("NO ACCEL!");
    }

    accel.enableMagnetometer();

    // uint8_t axes = Sodaq_LSM303C::MagX;
    // accel.enableMagnetometerInterrupt(axes, -400);

    // pinMode(MAG_INT, INPUT_PULLDOWN);
    // attachInterrupt(MAG_INT, magInterrupt, RISING);
}

void loop()
{
    delay(1000);

    SerialUSB.println(accel.getTemperature());

    SerialUSB.println(accel.getTemperature());

    if (accel.checkWhoAmI()) {
        SerialUSB.println("FOUND ACCEL!");
    }
    else {
        SerialUSB.println("NO ACCEL!");
    }

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
