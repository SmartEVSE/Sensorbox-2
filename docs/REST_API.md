# REST API

The REST API can be accessed through any http tool, here as an example CURL will be used.

# GET: /settings

curl -X GET http://ipaddress/settings

will give output like:
```
{"version":"09:09:41 @Oct  3 2024","serialnr":101110,"smartevse_host":"","wifi":{"status":"WL_CONNECTED","ssid":"wifi_nomap","rssi":-57,"bssid":"94:A6:7E:F6:AE:CC"},"settings":{"mains_meter":"Sensorbox"},"evse":{"temp":0},"ev_meter":{"description":"Disabled","currents":{"TOTAL":0,"L1":0,"L2":0,"L3":0}},"mqtt":{"host":"laptop-hans.lan","port":1883,"topic_prefix":"Sensorbox/101110","username":"","password_set":false,"status":"Connected"},"phase_currents":{"TOTAL":0,"L1":0,"L2":0,"L3":0,"last_data_update":1727961993}}
```

This output is often used to add to your bug report, so the developers can see your configuration.

NOTE:
In the http world, GET parameters are passed like this:
curl -X GET http://ipaddress/endpoint?param1=value1&param2=value2
and POST parameters are passed like this:
curl -X POST http://ipaddress/endpoint -d 'param1=value1' -d 'param2=value2' -d ''

Now in the ESP world, we all have picked up the habit of using the GET way of passing parameters also for POST commands. SmartEVSE development not excluded....
From version v3.6.0 on, instead of using the Arduino Core webserver libraries, we are now using the Mongoose webserver, which is broadly used. This webserver however sticks to the "normal" http standards.

This means that if you POST a request to SmartEVSE or Sensorbox, the webserver will be waiting for the -d data until it times out (or you ctrl-C your curl command). -d ''
You can prevent this by adding
'''
-d ''
'''

to your curl POST command. -d ''

# POST: /erasesettings

&emsp;&emsp;Note: no parameters, resets your device to factory settings.

# POST: /reboot

&emsp;&emsp;Note: no parameters, reboots your device.
