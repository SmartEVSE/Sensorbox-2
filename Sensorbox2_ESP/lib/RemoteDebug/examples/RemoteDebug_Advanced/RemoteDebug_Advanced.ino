////////
// Library: Remote debug - debug over WiFi - for Esp8266 (NodeMCU) or ESP32
// Author : Joao Lopes
// File   : RemoteDebug_Advanced.ino
// Notes  :
//
// Attention: This library is only for help development. Please not use this in production
//
// Sample to show how to use advanced features of Arduino and RemoteDebug library
//
// Example of use:
//
//#ifndef DEBUG_DISABLED
//        if (Debug.isActive(Debug.<level>)) { // <--- This is very important to reduce overheads and work of debug levels
//            Debug.printf("bla bla bla: %d %s\n", number, str);
//            Debug.println("bla bla bla");
//        }
//#endif
//
// Or short way (prefered if only one debug at time)
//
//		debugA("This is a any (always showed) - var %d", var);
//
//		debugV("This is a verbose - var %d", var);
//		debugD("This is a debug - var %d", var);
//		debugI("This is a information - var %d", var);
//		debugW("This is a warning - var %d", var);
//		debugE("This is a error - var %d", var);
//
//		debugV("This is println");
//
///////

////// Defines

// Host name (please change it)

#define HOST_NAME "remotedebug"

// Board especific libraries

#if defined ESP8266 || defined ESP32

// Use mDNS ? (comment this do disable it)

#define USE_MDNS true

// Arduino OTA (uncomment this to enable)

//#define USE_ARDUINO_OTA true

#else

// RemoteDebug library is now only to Espressif boards,
// as ESP32 and ESP82266,
// If need for another WiFi boards,
// please add an issue about this
// and we will see if it is possible made the port for your board.
// access: https://github.com/JoaoLopesF/RemoteDebug/issues

#error "The board must be ESP8266 or ESP32"

#endif // ESP

// Web server (uncomment this to need this)

//#define WEB_SERVER_ENABLED true

////// Includes

#if defined ESP8266

// Includes of ESP8266

#include <ESP8266WiFi.h>

#ifdef USE_MDNS
#include <DNSServer.h>
#include <ESP8266mDNS.h>
#endif

#ifdef WEB_SERVER_ENABLED
#include <ESP8266WebServer.h>
#endif

#elif defined ESP32

// Includes of ESP32

#include <WiFi.h>

#ifdef USE_MDNS
#include <DNSServer.h>
#include "ESPmDNS.h"
#endif

#ifdef WEB_SERVER_ENABLED
#include <WebServer.h>
#endif

#else

#error "For now, RemoteDebug support only boards Espressif, as ESP8266 and ESP32"

#endif // ESP

// Arduino OTA

#ifdef USE_ARDUINO_OTA
#include <ArduinoOTA.h>
#endif

// HTTP Web server

#ifdef WEB_SERVER_ENABLED

#if defined ESP8266

ESP8266WebServer HTTPServer(80);

#elif defined ESP32

WebServer HTTPServer(80);

#endif

#endif // WEB_SERVER_ENABLED

///// Remote debug over WiFi - not recommended for production/release, only for development

// Options for RemoteDebug of this project

// Attention: read this, before you change any option
//
// If yot changed it and not works, the compiler is using catching old compiled files
// To workaround this:
// - If have a clean project option in IDE (as Eclipse/Platformio), do it
// - or force compiler to compiler all (changing any configuration of board)
// - or to this change globally in RemoteDebugCfg.h (on library directory)
// - And upload again
//
// thanks to @22MarioZ for added this issue

// Disable all debug ?
// Important to compile for prodution/release
// Disable all debug ? Good to release builds (production)
// as nothing of RemoteDebug is compiled, zero overhead :-)
// Uncomment the line below, to do it:
//#define DEBUG_DISABLED true

// Disable te auto function feature of RemoteDebug
// Good if your code already have func name on debug messages
// Uncomment the line below, to do it:
//#define DEBUG_DISABLE_AUTO_FUNC true

// Disable Websocket? This is used with RemoteDebugApp connection
// Uncomment the line below, to do it:
//#define WEBSOCKET_DISABLED true

#ifndef WEBSOCKET_DISABLED // Only if Web socket enabled (RemoteDebugApp)
// If enabled, you can change the port here (8232 is default)
// Uncomment the line below, to do it:
//#define WEBSOCKET_PORT 8232

// Internally, the RemoteDebug uses a local copy of the arduinoWebSockets library (https://github.com/Links2004/arduinoWebSockets)
// Due it not in Arduino Library Manager
// If your project already use this library,
// Uncomment the line below, to do it:
//#define USE_LIB_WEBSOCKET true
#endif

// Include libraries

#include "RemoteDebug.h"        //https://github.com/JoaoLopesF/RemoteDebug

#ifndef DEBUG_DISABLED // Only if debug is not disabled (for production/release)

// Instance of RemoteDebug

RemoteDebug Debug;

#endif

// WiFi credentials
// Note: if commented, is used the smartConfig
// That allow to it in mobile app
// See more details in http://www.iotsharing.com/2017/05/how-to-use-smartconfig-on-esp32.html

//#define WIFI_SSID "..."  // your network SSID (name)
//#define WIFI_PASS "..."  // your network key

/////// Variables

// Time

uint32_t mTimeToSec = 0;
uint32_t mTimeSeconds = 0;

////// Setup

void setup() {

	// Initialize the Serial (use only in setup codes)

	Serial.begin(230400);

	// Buildin led

#ifdef LED_BUILTIN
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
#endif

	// Connect WiFi

	connectWiFi();

	// Host name of WiFi

#ifdef ESP8266
	WiFi.hostname(HOST_NAME);
#endif

#ifdef USE_ARDUINO_OTA
	// Update over air (OTA)

	initializeOTA();
#endif

	// Register host name in mDNS

#if defined USE_MDNS && defined HOST_NAME

	if (MDNS.begin(HOST_NAME)) {
		Serial.print("* MDNS responder started. Hostname -> ");
		Serial.println(HOST_NAME);
	}

	// Register the services

#ifdef WEB_SERVER_ENABLED
	MDNS.addService("http", "tcp", 80);   // Web server
#endif

#ifndef DEBUG_DISABLED
	MDNS.addService("telnet", "tcp", 23); // Telnet server of RemoteDebug, register as telnet
#endif

#endif // MDNS

	// HTTP web server

#ifdef WEB_SERVER_ENABLED
	 HTTPServer.on("/", handleRoot);

	 HTTPServer.onNotFound(handleNotFound);

	 HTTPServer.begin();

	 Serial.println("* HTTP server started");
#endif

#ifndef DEBUG_DISABLED // Only for development

	// Initialize RemoteDebug

	Debug.begin(HOST_NAME); // Initialize the WiFi server

	//Debug.setPassword("r3m0t0."); // Password for WiFi client connection (telnet or webapp)  ?

	Debug.setResetCmdEnabled(true); // Enable the reset command

	Debug.showProfiler(true); // Profiler (Good to measure times, to optimize codes)

	Debug.showColors(true); // Colors

	// Debug.setSerialEnabled(true); // if you wants serial echo - only recommended if ESP is plugged in USB

	// Project commands

	String helpCmd = "bench1 - Benchmark 1\n";
	helpCmd.concat("bench2 - Benchmark 2");

	Debug.setHelpProjectsCmds(helpCmd);
	Debug.setCallBackProjectCmds(&processCmdRemoteDebug);

	// End of setup - show IP

	Serial.println("* Arduino RemoteDebug Library");
	Serial.println("*");
	Serial.print("* WiFI connected. IP address: ");
	Serial.println(WiFi.localIP());
	Serial.println("*");
	Serial.println("* Please use the telnet client (telnet for Mac/Unix or putty and others for Windows)");
	Serial.println("* or the RemoteDebugApp (in browser: http://joaolopesf.net/remotedebugapp)");
	Serial.println("*");
	Serial.println("* This sample will send messages of debug in all levels.");
	Serial.println("*");
	Serial.println("* Please try change debug level in client (telnet or web app), to see how it works");
	Serial.println("*");

#endif

}

void loop() {

#ifndef DEBUG_DISABLED
	// Time of begin of this loop
	uint32_t timeBeginLoop = millis();
#endif


	// Each second

	if (millis() >= mTimeToSec) {

		// Time

		mTimeToSec = millis() + 1000;

		mTimeSeconds++;

		// Blink the led

#ifdef LED_BUILTIN
		digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
#endif

		// Debug the time (verbose level)

		debugV("* Time: %u seconds (VERBOSE)", mTimeSeconds);

		if (mTimeSeconds % 5 == 0) { // Each 5 seconds

			// Debug levels

			debugV("* This is a message of debug level VERBOSE");
			debugD("* This is a message of debug level DEBUG");
			debugI("* This is a message of debug level INFO");
			debugW("* This is a message of debug level WARNING");
			debugE("* This is a message of debug level ERROR");


			// RemoteDebug isActive? Use this RemoteDebug sintaxe if you need process anything only for debug
			// It is good to avoid overheads (this is only use that is suggest to use isActive)
			// Note this need be surrounded by DEBUG_DISABLED precompiler condition to not compile for production/release

#ifndef DEBUG_DISABLED

			if (Debug.isActive(Debug.VERBOSE)) {

				debugV("Calling a foo function");
				debugV("At time of %d sec.\n", mTimeSeconds);

				// Call a function

				foo();
			}
#endif

		}
	}

	////// Services on Wifi

#ifdef USE_ARDUINO_OTA
	// Update over air (OTA)

	ArduinoOTA.handle();
#endif

#ifdef WEB_SERVER_ENABLED
	// Web server

	HTTPServer.handleClient();
#endif

#ifndef DEBUG_DISABLED
	// RemoteDebug handle (for WiFi connections)

	Debug.handle();
#endif

	// Give a time for ESP

	yield();

#ifndef DEBUG_DISABLED
	// Show a debug - warning if time of these loop is over 50 (info) or 100 ms (warning)

	uint32_t time = (millis() - timeBeginLoop);

	if (time > 100) {
		debugI("* Time elapsed for the loop: %u ms.", time);
	} else if (time > 200) {
		debugW("* Time elapsed for the loop: %u ms.", time);
	}
#endif

}


// Function example to show a new auto function name of debug* macros

void foo() {

  uint8_t var = 1;

  debugV("this is a debug - var %u", var);
  debugV("This is a println");
}

#ifndef DEBUG_DISABLED

// Process commands from RemoteDebug

void processCmdRemoteDebug() {

	String lastCmd = Debug.getLastCommand();

	if (lastCmd == "bench1") {

		// Benchmark 1 - Printf

		debugA("* Benchmark 1 - one Printf");


		uint32_t timeBegin = millis();
		uint8_t times = 50;

		for (uint8_t i = 1; i <= times; i++) {
			debugA("%u - 1234567890 - AAAA", i);

		}

		debugA("* Time elapsed for %u printf: %ld ms.\n", times,
					(millis() - timeBegin));

	} else if (lastCmd == "bench2") {

		// Benchmark 2 - Print/println

		debugA("* Benchmark 2 - Print/Println");

		uint32_t timeBegin = millis();
		uint8_t times = 50;

		for (uint8_t i = 1; i <= times; i++) {
			if (Debug.isActive(Debug.ANY)) {
				Debug.print(i);
				Debug.print(" - 1234567890");
				Debug.println(" - AAAA");
			}
		}

		debugA("* Time elapsed for %u printf: %ld ms.\n", times,
					(millis() - timeBegin));
	}
}
#endif

////// WiFi

void connectWiFi() {

	////// Connect WiFi

#ifdef EM_DEPURACAO
	Serial.println("*** connectWiFi: begin conection ...");
#endif

#ifdef ESP32
	// ESP32 // TODO: is really necessary ?
	WiFi.enableSTA(true);
	delay(100);
#endif

	// Connect with SSID and password stored

#ifndef WIFI_SSID
	WiFi.begin();
#else
	WiFi.begin(WIFI_SSID, WIFI_PASS);
#endif

	// Wait connection

	uint32_t timeout = millis() + 20000; // Time out

	while (WiFi.status() != WL_CONNECTED && millis() < timeout) {
		delay(250);
		Serial.print(".");
	}

	// Not connected yet?

	if (WiFi.status() != WL_CONNECTED) {

#ifndef WIFI_SSID
		// SmartConfig

		WiFi.beginSmartConfig();

		// Wait for SmartConfig packet from mobile

		Serial.println("connectWiFi: Waiting for SmartConfig.");

		while (!WiFi.smartConfigDone()) {
			delay(500);
			Serial.print(".");
		}

		Serial.println("");
		Serial.println("connectWiFi: SmartConfig received.");

		// Wait for WiFi to connect to AP

		Serial.println("connectWiFi: Waiting for WiFi");

		while (WiFi.status() != WL_CONNECTED) {
			delay(500);
			Serial.print(".");
		}
#else
		Serial.println("Not possible connect to WiFi, rebooting");
		ESP.restart();
#endif
	}

	// End

	Serial.println("");
	Serial.print("connectWiFi: connect a ");
	Serial.println(WiFi.SSID());
	Serial.print("IP: ");
	Serial.println(WiFi.localIP().toString());

}

#ifdef USE_ARDUINO_OTA

// Initialize o Arduino OTA

void initializeOTA() {

	// TODO: option to authentication (password)

#if defined ESP8266

	ArduinoOTA.onStart([]() {
		Serial.println("* OTA: Start");
	});
	ArduinoOTA.onEnd([]() {
		Serial.println("\n*OTA: End");
	});
	ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
		Serial.printf("*OTA: Progress: %u%%\r", (progress / (total / 100)));
	});
	ArduinoOTA.onError([](ota_error_t error) {
		Serial.printf("*OTA: Error[%u]: ", error);
		if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
		else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
		else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
		else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
		else if (error == OTA_END_ERROR) Serial.println("End Failed");
	});

#elif defined ESP32

	// ArduinoOTA

	ArduinoOTA.onStart([]() {
		String type;
		if (ArduinoOTA.getCommand() == U_FLASH)
			type = "sketch";
		else // U_SPIFFS
			type = "filesystem";
			Serial.println("Start updating " + type);
		}).onEnd([]() {
		Serial.println("\nEnd");
	}).onProgress([](unsigned int progress, unsigned int total) {
		Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
	}).onError([](ota_error_t error) {
		Serial.printf("Error[%u]: ", error);
		if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
		else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
		else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
		else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
		else if (error == OTA_END_ERROR) Serial.println("End Failed");
	});

#endif

	// Begin

	ArduinoOTA.begin();

}

#endif

#ifdef WEB_SERVER_ENABLED

/////////// Handles

 void handleRoot() {

     // Root web page

     HTTPServer.send(200, "text/plain", "hello from esp - RemoteDebug Sample!");
 }

 void handleNotFound(){

     // Page not Found

     String message = "File Not Found\n\n";
     message.concat("URI: ");
     message.concat(HTTPServer.uri());
     message.concat("\nMethod: ");
     message.concat((HTTPServer.method() == HTTP_GET)?"GET":"POST");
     message.concat("\nArguments: ");
     message.concat(HTTPServer.args());
     message.concat("\n");
     for (uint8_t i=0; i<HTTPServer.args(); i++){
         message.concat(" " + HTTPServer.argName(i) + ": " + HTTPServer.arg(i) + "\n");
     }
     HTTPServer.send(404, "text/plain", message);
 }

#endif

/////////// End
