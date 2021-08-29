# Sensorbox-2
Sensorbox2 measures the current(A) and current direction for up to three phases with CT's and a MAINS voltage input.
It can also receive these measurements directly from a (D)SMR5 smart meters P1 port.
These measurements are then sent every two seconds to the connected SmartEVSE(s).

The Sensorbox2 uses a PIC microcontroller which currently does all the processing. It takes the measurements from the CT inputs and P1 port and can send the data over the RS485 bus.
It will be programmed initially by the ESP32 module. At powerup the ESP32 looks for a Sensorbox2.hex file in the /data folder and, when found programs the PIC18F26K40 microcontroller.

Please note that this also means that the PIC has control over the RS485 bus. If you want full controll, keep the PIC in reset by holding the PIC reset line low.

I'll release a version of the PIC firmware soon that will only do the CT measurements, and will not use the RS485 bus, nor use the P1 port.
On this version the ESP32 will handle everything else.



