/*

Sensorbox2 ESP32 code

Version 1.0

Use
pio run -t uploadfs
to upload the SPIFFS files in the /data folder to Flash

(C) 2019-2021  Michael Stegen / Stegen Electronics

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/



#include <Arduino.h>
#include "FS.h"
#include "SPIFFS.h"
#include "radix.h"


#define PIN_LED_GREEN 32
#define PIN_LED_RED   33
#define PIN_PGD 23
#define PIN_PGC 22
#define PIN_RS485_RX 19
#define PIN_RS485_TX 16
#define PIN_RS485_DE 17
#define PIN_RS485_RE 18
#define PIN_RX 27
#define PIN_TX 14


uint16_t DeviceID, Revision, UserID0, FlashADR, FlashData ;
File file;
uint32_t filesize;
const char* PICfirmware = "/Sensorbox2.hex";

String s, ledState;


void setup() {
  // put your setup code here, to run once:

  PicSetup();

  pinMode(PIN_LED_GREEN, OUTPUT_OPEN_DRAIN);
  pinMode(PIN_LED_RED, OUTPUT_OPEN_DRAIN);

  pinMode (PIN_RS485_RX, INPUT);
  pinMode (PIN_RS485_TX, INPUT);
  pinMode (PIN_RS485_DE, INPUT);
  pinMode (PIN_RS485_RE, INPUT);
  pinMode (PIN_RX, INPUT);
  pinMode (PIN_RX, INPUT);


// Serup Serial port
  Serial.begin(115200);
  Serial.printf("\nSensorbox 2 powerup\n");

 // Initialize SPIFFS
  if(!SPIFFS.begin(true)){
      Serial.printf("An Error has occurred while mounting SPIFFS\n");
      return;
  }
  Serial.printf("Total SPIFFS bytes: %u, Bytes used: %u\n",SPIFFS.totalBytes(),SPIFFS.usedBytes());


  PicReadConfigs();
  if (DeviceID==0x6980) Serial.printf("PIC18F26K40 found\n");

  if (SPIFFS.exists(PICfirmware))
  {
      Serial.printf("Sensorbox2.hex found on SPIFFS\n");
      file = SPIFFS.open(PICfirmware, "r");
      if (!file) {
        Serial.println("file open failed\n");
      } else {
        ProgramPIC(file);                                                       // Program PIC
        file.close();                                                           // close file after use
        SPIFFS.remove(PICfirmware);                                             // erase hexfile, so we only program once
      }
  } else Serial.printf("Sensorbox2.hex -not- found on SPIFFS\n");

  // Set PGC and PGD pins to inputs so we can monitor them.
  pinMode(PIN_PGC, INPUT);
  pinMode(PIN_PGD, INPUT);

  //RESET_PIN_INPUT();

}

void loop() {
  // put your main code here, to run repeatedly:
  // Show status of PIC CPU on LED
  if (digitalRead(PIN_PGC)==HIGH) digitalWrite(PIN_LED_GREEN,1); else digitalWrite(PIN_LED_GREEN,0);
  if (digitalRead(PIN_PGD)==HIGH) digitalWrite(PIN_LED_RED,1); else digitalWrite(PIN_LED_RED,0);

}
