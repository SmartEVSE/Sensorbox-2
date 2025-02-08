# Sensorbox-2
Sensorbox2 measures the current(A) and current direction for up to three phases with CT's and a MAINS voltage input.
It can also receive these measurements directly from a (D)SMR5 smart meters P1 port.
These measurements are then sent every two seconds to the connected SmartEVSE(s).

The Sensorbox2 uses a PIC microcontroller which does the CT measurements and sends this information to the ESP32.
The ESP32 sends both the P1 port (Smart Meter connection) and CT measurement data over the RS485 bus to the SmartEVSE.



## Connecting to Wifi (and updating firmware)

To upgrade your Sensorbox-2 firmware, first upgrade your SmartEVSE v3 firmware to version v3.7.2 or higher.

Once your SmartEVSE v3 detects your Sensorbox-2's old firmware, it will present the LCD screen with a new option: **SB2 WIFI**, which is set to `Disabled` by default.
Select `Setup` here, and the Sensorbox will presents itself as a Wifi Acces Point "smartevse-xxxx"; the password is shown on the LCD screen.
Connect with your phone to that access point, go to http://192.168.4.1/ and configure your Wifi password.

```
Please note that if you don't see the **SB2 WIFI** option it is possible you have a sensorbox hardware version 1.0.6 with an even older firmware. 
You can check the hardware version by removing the P1 cable from the sensorbox, and looking inside. the version is printed on the circuit board.
In this case you will need to flash the firmware with a USB cable.
```

Once the Sensorbox2 is connected to your Wifi, you can find it's IP address on the SmartEVSE's **SB2 WIFI** menu option.
Browse to this ip address, and you will be presented with the old Sensorbox2's wifi status page.<br>

If you want to upgrade your Sensorbox2's firmware, browse to http://ip-address/update where "ip-address" is the ip address of the sensorbox2.<br>
First update the spiffs partition, by uploading [spiffs.bin](https://github.com/SmartEVSE/Sensorbox-2/releases/download/v2.1.0/spiffs.bin)<br>
Then proceed by uploading the [main firmware](https://github.com/SmartEVSE/Sensorbox-2/releases/download/v2.1.0/firmware.bin)<br>

After you updated your Sensorbox-2 to v2.1.0 or higher, the SB2 WIFI option will not be shown anymore on the SmartEVSE menu.<br>

On firmware version 2.1.0 or newer the procedure to configure WiFi has changed:<br>
See [configuration](docs/configuration.md)

## Status Led

The status Led sequence depends on the working mode of the Sensorbox:

1 blink:
- Green: P1 measurement. Orange: CT measurement (no direction of currents)

2 blinks:
- Green: P1 measurement. Orange: CT measurement (no direction of currents)
- Green: connected to WiFi, Red: waiting to connect 

3 blinks:
1. Phase L1: Green = export, Orange = import
2. Phase L2: Green = export, Orange = import
3. Phase L3: Green = export, Orange = import

4 blinks:
1. Phase L1: Green = export, Orange = import
2. Phase L2: Green = export, Orange = import
3. Phase L3: Green = export, Orange = import
4. Green: connected to WiFi, Red: waiting to connect




## Modbus Registers

Modbus RTU address is fixed to 0x0A, speed is 9600 bps 

   
Input Registers (FC=04)(Read Only):

    Register  Register  
     Address   length (16 bits)
     0x0000      1       Sensorbox version 2             = 0x0114 (2 lsb mirror the 3/4 Wire and Rotation configuration data)
                                                           0x0115 = 4 wire, CCW rotation
                                                           0x0116 = 3 wire, CW rotation      
                                                           0x0117 = 3 wire, CCW rotation  
     0x0001      1       DSMR Version(MSB),CT's or P1(LSB) 0x3283 = DSMR version 50, P1 port connected (0x80) CT's Used (0x03)                    
     0x0002      2       Volts L1 (32 bit floating point), Smartmeter P1 data
     0x0004      2       Volts L2 (32 bit floating point), Smartmeter P1 data
     0x0006      2       Volts L3 (32 bit floating point), Smartmeter P1 data
     0x0008      2       Amps L1 (32 bit floating point), Smartmeter P1 data
     0x000A      2       Amps L2 (32 bit floating point), Smartmeter P1 data
     0x000C      2       Amps L3 (32 bit floating point), Smartmeter P1 data
     0x000E      2       Amps L1 (32 bit floating point), CT imput 1
     0x0010      2       Amps L2 (32 bit floating point), CT input 2
     0x0012      2       Amps L3 (32 bit floating point), CT input 3
     
     with sensorbox software version >= 0x01xx, the following extra registers are available

     0x0014      1       WiFi Connection Status  xxxxxACL xxxxxxWW = WiFi mode (00=Wifi Off, 01=On, 10=portal started)
                                                      ||\_ Local Time Set
                                                      |\__ Connected to WiFi
                                                      \___ AP_STA mode (portal) active
     0x0015      1       Time Hour(msb) Minute(lsb)
     0x0016      1       Time Month(msb) Day(lsb)
     0x0017      1       Time Year(msb) Weekday(lsb)
     0x0018      2       IP address Sensorbox
     0x001A      2       MAC Sensorbox (4 LSB bytes) http://SmartEVSE-012345.local can be derived from this
     0x001C      4       Password portal (8 bytes)
 
    
    Holding Registers (FC=06)(Write):
    
     Register  Register  
     Address   length (16 bits) 
     0x0800      1       Field rotation setting (bit 0)      00000000 0000000x -> 0= Rotation right 1= Rotation Left
                         3/4 wire configuration (bit 1)      00000000 000000x0 -> 0= 4Wire, 1= 3Wire
     0x0801      1       Set WiFi mode                       00000000 000000xx -> 00 = disabled, 01 = enabled, 02 = start portal.   

## Example:

    Request Sensorbox2 data:
    0a 04 00 00 00 14 f1 7e
    
    Sensorbox2 replies with:
    0a 04 28 00 14 32 83 43 66 80 00 43 66 4c cd 43 69 19 9a 3e 12 9a 61 3f 9f 83 80 be 41 4a 5a 3b ea 00 3b bb 6f f3 6b 3a 45 8d e3 ca cb
    4 wire CW --^^
    DSMR ver(50) --^^
    P1 + CT connected ^^
    Volts L1 ------------| 230.5 V  |			 
    					 
    Request 3 wire grid setting:
    0a 06 08 00 00 02 0b 10		
    
    Next sensorbox2 data:
    0a 04 28 00 16 32 03 43 66 e6 66 43 66 80 00 43 69 19 9a 3d 66 9c 53 3f 97 99 d2 be a2 8a 28 bb 59 73 10 bb 08 29 4d 3b 8f 23 79 ca e6 			 
    ------------^^ changed to 3 wire, CW setting.






