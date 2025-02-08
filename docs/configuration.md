
# How to configure WIFI
* WIFI is used to control your Sensorbox through the builtin webserver, (optionally) to export its measurements to an MQTT broker, and (optionally) to send its measurements to the SmartEVSE. It is preferred to send the measurements over modbus RS485 to the SmartEVSE for reliability reasons...
* In order to configure WIFI, the following conditions have to be met:
  - the device has to be DISCONNECTED from any WIFI network; if you want to reconfigure and existing WIFI connection, use the /erasesettings to wipe out the existing WIFI configuration.
  - the P1 input has to be disconnected;
  - we are in the first 180 seconds after startup of the device; after this period the device assumes you don't want to configure WIFI

* If the above conditions are met, you can configure the WIFI as follows:
  - connect your smartphone to the wifi network you want your Sensorbox connected to.
  - download and run the ESPTouch app from your favourite appstore
  [Android](https://play.google.com/store/apps/details?id=com.fyent.esptouch.android&hl=en_US:)
  (please ignore the strange Author name) or
  [Apple](https://apps.apple.com/us/app/espressif-esptouch/id1071176700) or
  [Github](https://github.com/EspressifApp/EsptouchForAndroid) (for source code).
  - choose EspTouch V2,
  - fill in the password of the wifi network,
  - fill in "1" in device count for provisioning,
  - fill in "0123456789abcdef" in the AES Key field
  - leave Custom Data empty
  - press "Confirm", within 30 seconds the app will confirm a MAC address and an IP address
  You are connected now. If you want special stuff (static IP address, special DNS address), configure them on your AP/router.

* If you don't get it to work with the ESPTouch app, there is
  a backup procedure:
  - make your Sensorbox powerless, so it will be off
  - connect your Sensorbox with a USB cable to your PC; it will be powered through your USB port now.
  - install the USB driver (Windows) or not (linux) for WCH ch340 chipset.
  - connect your favorite serial terminal to the appropriate port.
  - use the following settings: 115200bps, 8 bits, no parity, 1 stopbit
  - make sure the status led blinks two or four times, the last blink should be Red. This indicates it's waiting for wifi configuration.
  - press enter on your serial terminal; you will be prompted for your WiFi network name and password.
  - the sensorbox should now connect to WiFi.
  - and it's IP address is displayed on your terminal.
 
# Webserver
After configuration of your Wifi parameters, your Sensorbox will present itself on your LAN via a webserver. This webserver can be accessed through:
* http://ip-address/ where "ip-address" is the ip address of the Sensorbox.
* http://sensorbox-xxxx.local/ where xxxx is the serial number of your Sensorbox. It might be necessary that mDNS is configured on your LAN.
* http://sensorbox-xxxx.lan/ where xxxx is the serial number of your Sensorbox. It might be necessary that mDNS is configured on your LAN.
* OTA update of your firmware:
    - surf to http://ip-address/update or press the UPDATE button on the webserver
    - select the firmware.bin from this archive, OR if you want the debug version (via telnet over your wifi), select firmware.debug.bin
    - after OK, wait 10-30 seconds and your new firmware including the webserver should be online!
* Added wifi-debugging: if you flashed the debug version, telnet http://ip-address/ will bring you to a debugger that shows you whats going on!

# REST API

For the specification of the REST API, see [REST API](REST_API.md)

# MQTT API
Your Sensorbox can now export the most important data to your MQTT-server. Just fill in the configuration data on the webserver and the data will automatically be announced to your MQTT server. Note that because the configuration data is transported to the Sensorbox via the URL, special characters are not allowed.

You can easily show all the MQTT topics published:
```
mosquitto_sub -v -h ip-of-mosquitto-server -u username -P password  -t '#'
```

# Building the firmware
You can get the latest release off of https://github.com/dingo35/Sensorbox-2/releases, but if you want to build it yourself:
* Install platformio-core https://docs.platformio.org/en/latest/core/installation/methods/index.html
* Clone this github project, cd to the `Sensorbox2_ESP` directory where platformio.ini is located
* Compile firmware.bin: platformio run

If you are not using the webserver /update endpoint:
* Windows users: install USB drivers https://wch-ic.com/downloads/CH341SER_EXE.html
* Upload via USB configured in platformio.ini: platformio run --target upload

# I think I bricked my Sensorbox
Luckily for you, there are no known instances of people who actually bricked their Sensorbox.
But if all else fails, connect your Sensorbox via USB to your laptop and use the following web usb flasher:<br>
https://esp.huhn.me/

Remember to flash the firmware.bin to both partitions, 0x10000 and 0x1c0000<br>
If you also want to flash the spiffs.bin, it's start address is 0x370000
