
# How to configure WIFI
* WIFI is used to control your Sensorbox through the builtin webserver, (optionally) to export its measurements to an MQTT broker, and (optionally) to send its measurements to the Sensorbox. It is preferred to send the measurements over modbus RS485 to the Sensorbox for reliability reasons...
* In order to configure the WIFI, the following conditions have to be met:
  - the device has to be DISCONNECTED from any WIFI network; if you want to reconfigure and existing WIFI connection, use the /erasesettings to wipe out the existing WIFI configuration.
  - the P1 input has to be disconnected;
  - we are in the first 180 seconds after startup of the device; after this period the device assumes you don't want to configure WIFI

* If the above conditions are met, you can configure the WIFI as follows:
  - connect your smartphone to the wifi network you want your Sensorbox connected to
  - download and run the ESPTouch app from your favourite appstore
.   [Android](https://play.google.com/store/apps/details?id=com.fyent.esptouch.android&hl=en_US:)
.   (please ignore the strange Author name) or
.   [Apple](https://apps.apple.com/us/app/espressif-esptouch/id1071176700) or
.   [Github](https://github.com/EspressifApp/EsptouchForAndroid) (for source code).
  - choose EspTouch V2,
  - fill in the password of the wifi network,
  - fill in "1" in device count for provisioning,
  - fill in "0123456789abcdef" in the AES Key field
  - leave Custom Data empty
  - press "Confirm", within 30 seconds the app will confirm a MAC address and an IP address
  You are connected now. If you want special stuff (static IP address, special DNS address),
  configure them on your AP/router.

* If you don't get it to work with the ESPTouch app, there is
  a backup procedure:
  - make your Sensorbox powerless, so it will be off
  - connect your Sensorbox with a USB cable to your PC; it will feed through your USB port now
  - install the USB driver (Windows) or not (linux) for ESP32 chipset
  - connect your favorite serial terminal to the appropriate port
  - unplug and replug the USB cable so the device is freshly started and enters the 180 seconds WIFI configuration period
  - press enter on your serial terminal; you will be prompted for your WiFi network name and password.

 
# Webserver
After configuration of your Wifi parameters, your Sensorbox will present itself on your LAN via a webserver. This webserver can be accessed through:
* http://ip-address/
* http://sensorbox-xxxx.local/ where xxxx is the serial number of your Sensorbox. It can be found on a sticker on the bottom of your Sensorbox. It might be necessary that mDNS is configured on your LAN.
* http://sensorbox-xxxx.lan/ where xxxx is the serial number of your Sensorbox. It can be found on a sticker on the bottom of your Sensorbox. It might be necessary that mDNS is configured on your LAN.
* OTA update of your firmware:
    - surf to http://your-sensorbox/update or press the UPDATE button on the webserver
    - select the firmware.bin from this archive, OR if you want the debug version (via telnet over your wifi),
 select firmware.debug.bin
    - after OK, wait 10-30 seconds and your new firmware including the webserver should be online!
* Added wifi-debugging: if you flashed the debug version, telnet http://your-sensorbox/ will bring you to a debugger that shows you whats going on!

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
* Clone this github project, cd to the smartevse directory where platformio.ini is located
* Compile firmware.bin: platformio run

If you are not using the webserver /update endpoint:
* Windows users: install USB drivers https://www.silabs.com/de...o-uart-bridge-vcp-drivers
* Upload via USB configured in platformio.ini: platformio run --target upload

# I think I bricked my Sensorbox
Luckily for you, there are no known instances of people who actually bricked their Sensorbox.
But if all else fails, connect your Sensorbox via USB to your laptop and follow the instruction https://github.com/dingo35/Sensorbox-3.5/issues/79

Another tool can be found here: https://github.com/marcelstoer/nodemcu-pyflasher

Remember to flash to both partitions, 0x10000 and 0x1c0000  !!!
