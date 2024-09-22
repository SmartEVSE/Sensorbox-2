/*
;  Use
;  pio run -t uploadfs
;  to upload the SPIFFS files in the /data folder to flash
;
;   Modbus RTU slave address is fixed to 0x0A, speed is 9600 bps 
:   
;   Input Registers (FC=04)(Read Only):
;  
;   Register  Register  
;   Address   length (16 bits)
;   0x0000      1       Sensorbox version 2             = 0x0014 (2 lsb mirror the 3/4 Wire and Rotation configuration data)
;                                                         0x0015 = 4 wire, CCW rotation
;                                                         0x0016 = 3 wire, CW rotation      
;                                                         0x0017 = 3 wire, CCW rotation  
;   0x0001      1       DSMR Version(MSB),CT's or P1(LSB) 0x3283 = DSMR version 50, P1 port connected (0x80) CT's Used (0x03)                    
;   0x0002      2       Volts L1 (32 bit floating point), Smartmeter P1 data
;   0x0004      2       Volts L2 (32 bit floating point), Smartmeter P1 data
;   0x0006      2       Volts L3 (32 bit floating point), Smartmeter P1 data
;   0x0008      2       Amps L1 (32 bit floating point), Smartmeter P1 data
;   0x000A      2       Amps L2 (32 bit floating point), Smartmeter P1 data
;   0x000C      2       Amps L3 (32 bit floating point), Smartmeter P1 data
;   0x000E      2       Amps L1 (32 bit floating point), CT imput 1
;   0x0010      2       Amps L2 (32 bit floating point), CT input 2
;   0x0012      2       Amps L3 (32 bit floating point), CT input 3
;
;   If the sensorbox software version >= 0x01xx, the following extra registers are available
;
;   0x0014      1       WiFi Connection Status  xxxxxACL xxxxxxWW = WiFi mode (00=Wifi Off, 01=On, 10=portal started)
;                                                    ||\_ Local Time Set
;                                                    |\__ Connected to WiFi
;                                                    \___ AP_STA mode (portal) active
;   0x0015      1       Time Hour(msb) Minute(lsb)
;   0x0016      1       Time Month(msb) Day(lsb)
;   0x0017      1       Time Year(msb) Weekday(lsb)
;   0x0018      2       IP address Sensorbox
;   0x001A      2       MAC Sensorbox (4 LSB bytes) http://SmartEVSE-012345.local can be derived from this
;   0x001C      4       Password portal (8 bytes)
;
;
;   Holding Registers (FC=06)(Write):
;
;   Register  Register  
;   Address   length (16 bits) 
;   0x0800      1       Field rotation setting (bit 0)      00000000 0000000x -> 0= Rotation right 1= Rotation Left
;                       3/4 wire configuration (bit 1)      00000000 000000x0 -> 0= 4Wire, 1= 3Wire
;   0x0801      1       Set WiFi mode                       00000000 000000xx -> 00 = disabled, 01 = enabled, 02 = start portal.
;                        
*/

#include <Arduino.h>
#include <Preferences.h>

#include <SPIFFS.h>

#include <WiFi.h>
#include "network.h"
//#include <AsyncTCP.h>
#include <Update.h>

#include <esp_task_wdt.h>

#include "Logging.h"
#include "ModbusServerRTU.h"        // Slave/node eModbus
#include "ModbusClientRTU.h"        // Master eModbus

#include "time.h"

#include "main.h"
#include "radix.h"
#include "prg_pic.h"
#include <SPI.h>

extern struct tm timeinfo;
extern String TZinfo;

//String APhostname = "SmartEVSE-" + String( MacId() & 0xffff, 10);           // SmartEVSE access point Name = SmartEVSE-xxxxx
extern String APhostname;
extern void network_loop(void);

// SSID and PW for your Router
//String Router_SSID;
//String Router_Pass;

// Create a ModbusRTU server and client instance on Serial1 
ModbusServerRTU MBserver(Serial1, 2000, ToggleRS485);                        // TCP timeout set to 2000 ms
//ModbusClientRTU MBclient(Serial1, ToggleRS485);  

Preferences preferences;

uint16_t DeviceID, Revision, UserID0, FlashADR, FlashData ;
File file;
uint32_t filesize;
const char* PICfirmware = "/PIC18F26K40.hex";

uint8_t P1data[2000];
uint16_t ModbusData[50];    // 50 registers

// data that will be stored in 'preferences'
extern uint8_t WIFImode;
String APpassword = "00000000";
// end of data that will be stored in 'preferences'
uint32_t serialnr;

unsigned long ModbusTimer=0;
unsigned char dataready=0, CTcount, DSMRver, IrmsMode = 0;
unsigned char LedCnt, LedState, LedSeq[3] = {0,0,0};
float Irms[3], Volts[3], IrmsCT[3];                                           // float is 32 bits
float MainsMeterIrms[3], EVMeterIrms[3];                                         // float is 32 bits
uint8_t datamemory = 0;
unsigned char led = 0, Wire = WIRES4 + CW;
uint16_t blinkram = 0, P1taskram = 0;
extern bool LocalTimeSet;
int phasesLastUpdate = 0;

// ------------------------------------------------ Settings -----------------------------------------------------
// 
// ---------------------------------------------------------------------------------------------------------------


// Generate random password for AP if not initialized
void CheckAPpassword(void) {
  uint8_t i, c;
  // Set random password if not yet initialized.
  if (strcmp(APpassword.c_str(), "00000000") == 0) {
    for (i=0; i<8 ;i++) {
      c = random(16) + '0';
      if (c > '9') c += 'a'-'9'-1;
      APpassword[i] = c;
    }
  }
  Serial.print("APpassword: ");
  Serial.println(APpassword);
}


void write_settings(void) {
  // Check if stored data is valid
  if (WIFImode > 2) WIFImode = WIFI_MODE;

  if (preferences.begin("settings", false) ) {

    preferences.putUChar("WIFImode", WIFImode);
    preferences.putString("APpassword", APpassword);
    preferences.end();

  } else Serial.print("Can not open preferences!\n");
}


void read_settings(bool write) {    
  if (preferences.begin("settings", false) == true) {
    WIFImode = preferences.getUChar("WIFImode", WIFI_MODE);
    APpassword = preferences.getString("APpassword", AP_PASSWORD);
    TZinfo = preferences.getString("TimezoneInfo","");
    if (TZinfo != "") {
        setenv("TZ",TZinfo.c_str(),1);
        tzset();
    }
    preferences.end();       

    // Check if AP password is unitialized. 
    // Create random AP password.
    CheckAPpassword(); 

    if (write) write_settings();

  } else Serial.print("Can not open preferences!\n");
}


/*
void StartwebServer(void) {

    webServer.on("/update", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(200, "text/html", "spiffs.bin updates the SPIFFS partition<br>firmware.bin updates the main firmware<br><form method='POST' action='/update' enctype='multipart/form-data'><input type='file' name='update'><input type='submit' value='Update'></form>");
    });

    webServer.on("/update", HTTP_POST, [](AsyncWebServerRequest *request) {
       bool shouldReboot = !Update.hasError();
        AsyncWebServerResponse *response = request->beginResponse(200, "text/plain", shouldReboot?"OK":"FAIL");
        response->addHeader("Connection", "close");
        request->send(response);
        delay(500);
        if (shouldReboot) ESP.restart();
    },[](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final){
        if(!index) {
            Serial.printf("\nUpdate Start: %s\n", filename.c_str());
                if (filename == "spiffs.bin" ) {
                    Serial.print("\nSPIFFS partition write\n");
                    // Partition size is 0x90000
                    if(!Update.begin(0x90000, U_SPIFFS)) {
                        Update.printError(Serial);
                    }    
                } else if(!Update.begin((ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000), U_FLASH) {
                    Update.printError(Serial);
                }    
        }
        if(!Update.hasError()) {
            if(Update.write(data, len) != len) {
                Update.printError(Serial);
            } else Serial.printf("bytes written %u\r", index+len);
        }
        if(final) {
            if(Update.end(true)) {
                Serial.print("\nUpdate Success\n");
            } else {
                Update.printError(Serial);
            }
        }
    });
}
*/
/*
// Setup Wifi 
void WiFiSetup(void) {



    // Init and get the time
    // See https://github.com/nayarsystems/posix_tz_db/blob/master/zones.csv for timezone codes for your region
    configTzTime(TZ_INFO, NTP_SERVER);
    
}
*/
/*
void SetupNetworkTask(void * parameter) {

  WiFiSetup();
  StartwebServer();
  while(1) {

    // retrieve time from NTP server
    LocalTimeSet = getLocalTime(&timeinfo, 1000U);
    
    // Cleanup old websocket clients
    ws.cleanupClients();

    if (WIFImode == 2 && WiFi.getMode() != WIFI_AP_STA) {
        Serial.print("Start Portal...\n");
        StopwebServer();
        //Init WiFi as Station, start SmartConfig
        WiFi.mode(WIFI_AP_STA);
        char SmartConfigKey[] = "0123456789abcdef";
        WiFi.beginSmartConfig(SC_TYPE_ESPTOUCH_V2, SmartConfigKey);
        //WiFi.beginSmartConfig(SC_TYPE_ESPTOUCH_V2, "0123456789abcdef");

        //Wait for SmartConfig packet from mobile.
        _LOG_V("Waiting for SmartConfig.\n");
        Serial.end();
        Serial.begin(115200);
        //Serial.begin(115200, SERIAL_8N1, PIN_RXD, PIN_TXD, false);                    // Input from TX of PIC, and debug output to USB

        while (!WiFi.smartConfigDone() && (WIFImode == 2) && (WiFi.status() != WL_CONNECTED)) {
        // Also start Serial CLI for entering AP and password.
            ProvisionCli();
            delay(10);
        }                       // loop until connected or Wifi setup menu is exited.

        delay(2000);            // give smartConfig time to send provision status back to the users phone.

        if (WiFi.status() == WL_CONNECTED) {
            _LOG_V("\nWiFi Connected, IP Address:%s.\n", WiFi.localIP().toString().c_str());
            WIFImode = 1;
            write_settings();
            //LCDNav = 0;
        }

        CliState = 0;
        Serial.end();
        Serial.begin(115200, SERIAL_8N1, PIN_PGD, PIN_TXD, false);                    // Input from TX of PIC, and debug output to USB
        WiFi.stopSmartConfig(); // this makes sure repeated SmartConfig calls are succesfull

        StartwebServer();       //restart webserver
    }

    if (WIFImode == 1 && WiFi.getMode() == WIFI_OFF) {
      Serial.print("Starting WiFi..\n");
      WiFi.mode(WIFI_STA);
      WiFi.begin();
    }    

    if (WIFImode == 0 && WiFi.getMode() != WIFI_OFF) {
      Serial.print("Stopping WiFi..\n");
      WiFi.disconnect(true);
    }   

    vTaskDelay(1000 / portTICK_PERIOD_MS);
  } // while(1)

}
*/

// ------------------------------------------------ Webserver End ------------------------------------------------
// 
// ---------------------------------------------------------------------------------------------------------------



// Poly used is x^16+x^15+x^2+x
unsigned int CRC16(unsigned int crc, unsigned char *buf, int len)
{
	for (int pos = 0; pos < len; pos++)
	{
		crc ^= (unsigned int)buf[pos];        // XOR byte into least sig. byte of crc

		for (int i = 8; i != 0; i--) {        // Loop over each bit
			if ((crc & 0x0001) != 0) {          // If the LSB is set
				crc >>= 1;                        // Shift right and XOR 0xA001
				crc ^= 0xA001;
			}
			else                                // Else LSB is not set
				crc >>= 1;                        // Just shift right
		}
	}
	return crc;
}



// ----------------------------------------------------------------------------------------------------------------
// CT data is measured by the PIC, is send on a serial line to Serial0 of the ESP32
// The final calculations and phase angle corrections are done by the ESP32.  
// We use / as start line char, ! is end of line, just like a P1 message the data is followed by a CRC16 checksum
// Called by P1Task every 100ms
//
void CTReceive() {
	char *ret, ch;
  int Samples = 0;
  unsigned char x, CTwire = 0;
  static unsigned char CTstring[100], CTeot = 0;
  static unsigned char CTlength = 0, CTptr = 0;
  uint16_t crccal, crcdata;
  int Power1A = 0, Power1B = 0, Power2A = 0, Power2B = 0, Power3A = 0, Power3B = 0;
  const float y1=-3.114637, x1=4.043093, y2=-3.057741, x2=3.988535, y3=-3.000724, x3=3.933821; //60 samples 19.00 Ilead


  while (Serial.available()) {
    ch = Serial.read();

    //Serial.printf("%c",ch);

    if (ch == '/') {                                                            // Start character
      CTptr = 0;
      CTeot = 0;                                                                // start from beginning of buffer
    }

    if (CTeot) {                                                                // end of transmission?
      if (CTeot > 4) ch = 0;                                                    // we have also received the CRC, null terminate
      CTeot++;
    }

    CTstring[CTptr] = ch;                                                       // Store in buffer
    if (CTptr < 90) CTptr++;                                                    // prevent overflow of buffer

    if (ch == '!' && CTstring[0] == '/') {
      CTlength = CTptr;                                                         // store pointer of start of CRC
      CTeot = 1;
    }  

    if (CTeot > 5) {
      
      crcdata = (uint16_t) strtol((const char *)CTstring+CTlength, NULL, 16);   // get crc from data, convert to int
      crccal = CRC16(0, CTstring, CTlength);                                    // calculate CRC16 from data
      //Serial.printf(" length: %u, CRC16: %04x : %04x\n\r",CTlength, crccal, crcdata);

      if (crcdata == crccal) {

        ret = strstr((const char *)CTstring,(const char *)"1A:");               // Extract CT measurements from the buffer.
        if (ret != NULL) Power1A = atoi((const char *)ret+3);                   
        ret = strstr((const char *)CTstring,(const char *)"1B:");
        if (ret != NULL) Power1B = atoi((const char *)ret+3);
        ret = strstr((const char *)CTstring,(const char *)"2A:");
        if (ret != NULL) Power2A = atoi((const char *)ret+3);
        ret = strstr((const char *)CTstring,(const char *)"2B:");
        if (ret != NULL) Power2B = atoi((const char *)ret+3);
        ret = strstr((const char *)CTstring,(const char *)"3A:");
        if (ret != NULL) Power3A = atoi((const char *)ret+3);
        ret = strstr((const char *)CTstring,(const char *)"3B:");
        if (ret != NULL) Power3B = atoi((const char *)ret+3);
        ret = strstr((const char *)CTstring,(const char *)"SA:");
        if (ret != NULL) Samples = atoi((const char *)ret+3);
        // as we divide by Samples, it can not be 0!
        if (Samples < 1) Samples = 1;

        ret = strstr((const char *)CTstring,(const char *)"WI:");               // CT wire setting 4Wire=0, 3Wire=1 (bit1)
        if (ret != NULL) CTwire = atoi((const char *)ret+3);                    // and phase rotation CW=0, CCW=1 (bit0)
        
        // Irms data when there is no mains plug connected.
        // There is no way of knowing the direction of the current.
        ret = strstr((const char *)CTstring,(const char *)"1R:");
        if (ret != NULL) Power1A = atoi((const char *)ret+3);
        ret = strstr((const char *)CTstring,(const char *)"2R:");
        if (ret != NULL) Power2A = atoi((const char *)ret+3);
        ret = strstr((const char *)CTstring,(const char *)"3R:");
        if (ret != NULL) {
          Power3A = atoi((const char *)ret+3);
        
          IrmsCT[0] = sqrt((float)Power1A / Samples) * CALRMS;
          IrmsCT[1] = sqrt((float)Power2A / Samples) * CALRMS;
          IrmsCT[2] = sqrt((float)Power3A / Samples) * CALRMS;

          // CT Measurement with no current direction information
          IrmsMode = 1; 

        } else {

          // We do have enough data to calculate the Irms and direction of current for each phase
          IrmsCT[0] = (x1 * ((float)Power1A / Samples) + y1 * ((float)Power1B / Samples) ) / CAL;
          IrmsCT[1] = (x2 * ((float)Power2A / Samples) + y2 * ((float)Power2B / Samples) ) / CAL;
          IrmsCT[2] = (x3 * ((float)Power3A / Samples) + y3 * ((float)Power3B / Samples) ) / CAL;

          IrmsMode = 0;      
        }

        // very small values will be displayed as 0.0A
        for (x=0; x<3 ;x++) {
          if ((IrmsCT[x] > -0.05) && (IrmsCT[x] < 0.05)) IrmsCT[x] = 0.0;
        }
        
        // if selected Wire setting (3-Wire or 4-Wire) and CW and CCW phase rotation are not correctly set, we can toggle the PGC pin to set it.
        if ((CTwire != Wire) && IrmsMode == 0) {
          x = (4 + Wire - CTwire) % 4;
          Serial.printf("\nWire:%u CTwire:%u pulses %u\n", Wire, CTwire, x);
          do {
            digitalWrite (PIN_PGC, HIGH); 
            digitalWrite (PIN_PGC, LOW); 
            vTaskDelay(1 / portTICK_PERIOD_MS);
          } while (--x);
        }
        
        // update dataready, so the Master knows the CT's have been read with new data
        dataready |= 0x03;
        //Serial.printf("\rCT1: %2.1f A CT2: %2.1f A CT3: %2.1f A  ",IrmsCT[0],IrmsCT[1],IrmsCT[2] );

      } else LOG_E("CRC error in CTdata\n");
      
      CTeot = 0;
      CTptr = 0;
      memset(CTstring, 0u, 100u);
    }
  }

}

// ----------------------------------------------------------------------------------------------------------------
// Extracts Voltage and Current measurement data from the P1data buffer.
// updates variables Irms[] and the modbus Ireg registers.
//
void P1Extract() {

  char *ret;
  float L1Power = 0, L2Power = 0, L3Power = 0;
  float L1PowerReturn = 0, L2PowerReturn = 0, L3PowerReturn = 0;
  bool fluvius = false;
  float TotalPower, TotalPowerReturn;

  DSMRver = 0;

  ret = strstr((const char *)P1data,(const char *)"/FLU5\\");                 // Belgium Fluvius SMR 5 meter
  if (ret != NULL) fluvius = true;

  ret = strstr((const char *)P1data,(const char *)":0.2.8");                  // DSMR version
  if (ret != NULL) DSMRver = atoi((const char *)ret+7);
  
  // Voltage on the phases (Volt*1)
  ret = strstr((const char *)P1data,(const char *)":32.7.0");                 // Phase 1
  if (ret != NULL) Volts[0] = atof((const char *)ret+8);                      // Some Grid types have 0.0 volts on one of the phases.
  if (Volts[0] < 1.0) Volts[0] = 1;                                           // We divide by Volts later on, so it can not be 0.0
  ret = strstr((const char *)P1data,(const char *)":52.7.0");                 // Phase 2
  if (ret != NULL) Volts[1] = atof((const char *)ret+8);
  if (Volts[1] < 1.0) Volts[1] = 1;
  ret = strstr((const char *)P1data,(const char *)":72.7.0");                 // Phase 3
  if (ret != NULL) Volts[2] = atof((const char *)ret+8);
  if (Volts[2] < 1.0) Volts[2] = 1;
  
  // Power received from Grid (Watt*1)
  ret = strstr((const char *)P1data,(const char *)":21.7.0");                 // Phase 1
  if (ret != NULL) {
      L1Power = atof((const char *)ret+8)*1000;
      if (DSMRver == 0) DSMRver = 50;                                         // Sagemcom T211-D does not send DSMR version
  }                                                                           // but it does send Power and Volts per phase.        
  ret = strstr((const char *)P1data,(const char *)":41.7.0");                 // Phase 2
  if (ret != NULL) L2Power = atof((const char *)ret+8)*1000;
  ret = strstr((const char *)P1data,(const char *)":61.7.0");                 // Phase 3
  if (ret != NULL) L3Power = atof((const char *)ret+8)*1000;
  
  // Power delivered to Grid (Watt*1)
  ret = strstr((const char *)P1data,(const char *)":22.7.0");                 
  if (ret != NULL) L1PowerReturn = atof((const char *)ret+8)*1000;
  ret = strstr((const char *)P1data,(const char *)":42.7.0");
  if (ret != NULL) L2PowerReturn = atof((const char *)ret+8)*1000;
  ret = strstr((const char *)P1data,(const char *)":62.7.0");
  if (ret != NULL) L3PowerReturn = atof((const char *)ret+8)*1000;

  ret = strstr((const char *)P1data,(const char *)":1.7.0");                  // Total Power from Grid
  TotalPower = atof((const char *)ret+7)*1000;
  ret = strstr((const char *)P1data,(const char *)":2.7.0");                  // Total Power delivered to Grid (Watt)
  TotalPowerReturn = atof((const char *)ret+7)*1000;

  // Fluvius meter, but no voltage on phase 2 and 3
  if (fluvius && DSMRver == 0 && Volts[1] == 1 && Volts[2] == 1) {
    // So it's a one phase meter without :21.7.0 data (some models of the Sagemcom S211)
    // we can use the Total power measurements instead.
    L1Power = TotalPower;
    L1PowerReturn = TotalPowerReturn;
    DSMRver = 50;
  }

  Irms[0] = (L1Power-L1PowerReturn)/Volts[0];                              		// Irms (Amps *1)
  Irms[1] = (L2Power-L2PowerReturn)/Volts[1];
  Irms[2] = (L3Power-L3PowerReturn)/Volts[2];
  

  if (DSMRver >= 50) dataready |= 0x80;                                     	// P1 dataready
  else dataready |= 0x40;                                                   	// DSMR version not 5.0 !!

  //Serial.printf("L1: %3d V L2: %3d V L3: %3d V  ",(int)(Volts[0]),(int)(Volts[1]),(int)(Volts[2]) );
  //Serial.printf("L1: %2.1f A L2: %2.1f A L3: %2.1f A  \r\n",Irms[0],Irms[1],Irms[2] );
  //Serial.printf("Total Power %5d W  ",(int)(Total_power-Total_power_return) );
}


// ----------------------------------------------------------------------------------------------------------------
//  Reads P1 data from Serial2, checks crc, and stores in P1data buffer
//  Called by P1Task every 100ms
//
void P1Receive() {
  uint8_t RXbyte;
  static uint16_t P1ptr = 0, P1length = 0;
  static uint8_t P1Eot = 0;
  static unsigned long P1LastMillis = 0;
  

  while (Serial2.available()) {                                                 // Uart2 data available? P1 PORT

    RXbyte = Serial2.read();
    if (RXbyte == '/') {                                                        // Start character
      P1ptr = 0;                                                                // start from beginning of buffer
      P1Eot = 0;                                                                // reset End of Transmission flag
    }
    if (P1Eot) {
      if (P1Eot > 4) break;                                                     // if EOT>4 the CRC16 has been received, and the message can be verified.
      P1Eot++;                                                                  // count bytes after End of transmission
    }
    P1data[P1ptr] = RXbyte;                                                     // Store in buffer
    if (P1ptr < 1990) P1ptr++;                                                  // prevent overflow of buffer, reserve some bytes for CRC

    if (RXbyte == '!' && P1data[0] == '/') {
      P1length = P1ptr;                                                         // store start of CRC.
      P1Eot = 1;                                                                // end character, flag end of transmission

    }
  }

  if (P1Eot > 4) {

    uint16_t csP1 = CRC16(0, P1data, P1length);                                      // calculate CRC16 from data
    uint16_t crcP1 = (uint16_t) strtol((const char *)P1data + P1length, NULL, 16);   // get crc from data, convert to int

    if (millis() > P1LastMillis + 1500) LOG_W("Missed P1 message\n");

    P1LastMillis = millis();

    if (crcP1 == csP1) {                                                        // check if crc's match
      // Send telegram to debug output
      for (uint16_t x=0; x<P1length+4; x++) Serial.printf("%c",P1data[x]);
      Serial.print("\n");

      P1Extract();                                                              // Extract Voltage and Current values from Smart Meter telegram
    } else {
      LOG_E("P1 crc error, length %d\n", P1length);
      //for (uint16_t x=0;x<P1length;x++) Serial.printf("%c",P1data[x]);
      //Serial.print("\n");
    }
    P1Eot = 0;                                                                  // Mark as processed
    P1ptr = 0;
    // clear P1buffer
    memset(P1data, 0u, 2000u);
  }
}


// ----------------------------------------------------------------------------------------------------------------
// Task that handles incoming P1 and CT data
// Will call P1Receive and CTreceive every 100ms 
//
void P1Task(void * parameter) {
  uint8_t socketupdate = 0;
  char buffer[30];

  while(1) {
  
    // Check if there is new P1 data.
    P1Receive();
    if (!heap_caps_check_integrity_all(true)) Serial.print("\nheap error after P1 receive\n");

    // Check if there is a new measurement from the PIC (CT measurements)
    CTReceive();
    if (!heap_caps_check_integrity_all(true)) Serial.print("\nheap error after CT receive\n");

    // remember state of dataready, as it will be cleared after sending the data to the modbus Master.
    if (dataready > datamemory) datamemory = dataready;

    if (!(datamemory & 0x80 ) && !(datamemory & 0x03) && WiFi.status() != WL_CONNECTED && esp_timer_get_time() / 1000000 > 5)
        // if both P1 and CT are not connected,
        // and we have no wifi
        // we go to wifimode 2 smartconfig
        // but we wait 5 seconds to give the existing wifi time to connect
        WIFImode = 2;

    // Every ~2 seconds send measurement data over websockets to browser(s)
    if (socketupdate == 20 && WiFi.status() == WL_CONNECTED) {
    //if (socketupdate == 20 && ws.count() && WiFi.status() == WL_CONNECTED) { TODO DINGO

      // Smart meter measurement?
      if (datamemory & 0x80 ) {
        snprintf(buffer, sizeof(buffer), "I:%3.2f,%3.2f,%3.2f",Irms[0], Irms[1], Irms[2]);
        for (int x = 0; x < 3; x++)
            MainsMeterIrms[x] = Irms[x];
        phasesLastUpdate = time(NULL);

        ///ws.textAll(buffer);
        snprintf(buffer, sizeof(buffer), "V:%3d,%3d,%3d",(int)(Volts[0]),(int)(Volts[1]),(int)(Volts[2]) );
        ///ws.textAll(buffer);
        //ws.printfAll_P("I:%3.2f,%3.2f,%3.2f",Irms[0],Irms[1],Irms[2]);
        //ws.printfAll_P("V:%3d,%3d,%3d",(int)(Volts[0]),(int)(Volts[1]),(int)(Volts[2]) );

      // CT measurement  
      } else if (datamemory & 0x03) { 
        snprintf(buffer, sizeof(buffer), "I:%3.2f,%3.2f,%3.2f", IrmsCT[0], IrmsCT[1], IrmsCT[2]);
        for (int x = 0; x < 3; x++)
            MainsMeterIrms[x] = Irms[x];
        phasesLastUpdate = time(NULL);
        ///ws.textAll(buffer);
        //ws.printfAll_P("I:%3.2f,%3.2f,%3.2f",IrmsCT[0],IrmsCT[1],IrmsCT[2]);
      }
      else {
        for (int x = 0; x < 3; x++)
            MainsMeterIrms[x] = 0;                                              // better send 0 data then incorrect data?
      }

      datamemory = 0;
      socketupdate = 0;
    }

    socketupdate++;
    // keep track of available stack ram
    P1taskram = uxTaskGetStackHighWaterMark( NULL );

    if (!heap_caps_check_integrity_all(true)) Serial.print("\nheap error after printfAll\n");

    // delay task for 100mS
    vTaskDelay(100 / portTICK_PERIOD_MS);

  } // while(1) loop
}  


// ----------------------------------------------------------------------------------------------------------------
//
// Task BlinkLed
//
void BlinkLed(void * parameter) {
  static uint8_t LedTimer = 0;

    while(1) {
      
      while ((LedTimer++ > 1) && LedState) {                                        // start with state 6

        if ((LedState-- %2) == 0) {
          // check for Orange LED blink
          if (LedSeq[LedCnt] == 1) {
            digitalWrite(PIN_LED_RED,LOW);                                          // LED_RED_ON
            digitalWrite(PIN_LED_GREEN,LOW);                                        // Led green on
          // check for Red LED blink  
          } else if (LedSeq[LedCnt] == 2) {
            digitalWrite(PIN_LED_RED,LOW);                                          // LED_RED_ON
            // blink LED Green
          } else digitalWrite(PIN_LED_GREEN,LOW);                                   // Led green on    
          if (LedCnt < 2) LedCnt++;  

        } else {
          digitalWrite(PIN_LED_GREEN,HIGH);                                         // LED_GREEN_OFF;
          digitalWrite(PIN_LED_RED,HIGH);                                           // LED_RED_OFF;
        }
        LedTimer = 0;
      }

      // keep track of available stack ram
      blinkram = uxTaskGetStackHighWaterMark( NULL );
      // delay task for 100mS
      vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}    





// ----------------------------------------------------------------------------------------------------------------
// callback from Modbus register read
//
ModbusMessage MBReadFC04(ModbusMessage request) {
  ModbusMessage response;     // response message to be sent back
  uint16_t addr = 0, words = 0; 
  uint8_t n, x;
  char *pBytes;
    
  request.get(2, addr);   // 00 00
  request.get(4, words);  // 00 14

  // address valid and length not zero?
  if ((addr + words > 32) || (words == 0)) {
    // No. Return error response
    response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_ADDRESS);
    return response;
  }
  
  // Prepare start of response
  response.add(request.getServerID(), request.getFunctionCode(), (uint8_t)(words * 2));

  // Set Led sequence for the next 2 seconds
  // reset to first part of sequence
  LedCnt = 0;

  // When we have received a valid measurement from the SmartMeter, the led will blink only once.
  if (dataready & 0xC0) {
	  LedState = 2;
    // DSMR version not 5.x! Blink RED
	  if (DSMRver < 50) LedSeq[0] = 2;
      // DSMR version OK, Blink GREEN
	    else LedSeq[0] = 0;

  // No SmartMeter connected, we use the CT's
  // CT measurement available?
	} else if (dataready & 0x03) {
    // CT measurements with current direction?
    if (IrmsMode == 0) {
      if (IrmsCT[0] < -0.1 ) LedSeq[0]=0; else LedSeq[0]=1;
      if (IrmsCT[1] < -0.1 ) LedSeq[1]=0; else LedSeq[1]=1;
      if (IrmsCT[2] < -0.1 ) LedSeq[2]=0; else LedSeq[2]=1;
      // 6 States, ON/OFF (3 CT's)
  	  LedState = 6;                                                           		
    } else {
      // Blink 1x Orange when not using MAINS input
      LedState = 2;
      LedSeq[0] = 1;                
    }
  // LED_RED_ON (PIC chip not programmed?)
	} else {
    LedState = 0;
    digitalWrite(PIN_LED_RED,LOW);
  }

  // Set Modbus Sensorbox version 2.0 (20) + Wire settings.
  // Set Software version in MSB
  ModbusData[0] = (SENSORBOX_SWVER << 8) + SENSORBOX_VERSION + Wire;
  // Set DSMR version + dataready
  ModbusData[1] = (uint16_t)(DSMRver<<8) + dataready;

  n = 2;
  // Volts P1
  for (x=0; x<3 ;x++) {
    pBytes = (char*)&Volts[x];
    ModbusData[n++] = (uint16_t) (pBytes[3]<<8)+pBytes[2];
    ModbusData[n++] = (uint16_t) (pBytes[1]<<8)+pBytes[0];
  }
  // Current P1
  for (x=0; x<3 ;x++) {
    pBytes = (char*)&Irms[x];
    ModbusData[n++] = (uint16_t) (pBytes[3]<<8)+pBytes[2];
    ModbusData[n++] = (uint16_t) (pBytes[1]<<8)+pBytes[0];
  }
  // Current CT inputs
  for (x=0; x<3 ;x++) {
    pBytes = (char*)&IrmsCT[x];
    ModbusData[n++] = (uint16_t) (pBytes[3]<<8)+pBytes[2];
    ModbusData[n++] = (uint16_t) (pBytes[1]<<8)+pBytes[0];
  }

  bool wifimodeAPSTA = false;
  bool wificonnected = false;
  if (WiFi.getMode() == WIFI_AP_STA) wifimodeAPSTA = true;
  if (WiFi.status() == WL_CONNECTED) wificonnected = true;
  
  ModbusData[n++] = (uint16_t) ((LocalTimeSet << 8) | (wificonnected << 9) | (wifimodeAPSTA << 10)) + WIFImode;
  ModbusData[n++] = (uint16_t) (timeinfo.tm_hour << 8) + timeinfo.tm_min;   // hours since midnight	0-23, minutes after the hour 0-59
  ModbusData[n++] = (uint16_t) (timeinfo.tm_mday << 8) + timeinfo.tm_mon;   // day of the month	1-31, months since January 0-11
  ModbusData[n++] = (uint16_t) (timeinfo.tm_year << 8) + timeinfo.tm_wday;  // years since 1900, days since Sunday 0-6
  IPAddress localIp = WiFi.localIP();
  ModbusData[n++] = (uint16_t) (localIp[0] << 8) + localIp[1];
  ModbusData[n++] = (uint16_t) (localIp[2] << 8) + localIp[3];
  ModbusData[n++] = (uint16_t) (MacId() >> 16);
  ModbusData[n++] = (uint16_t) (MacId() & 0xFFFF);
  ModbusData[n++] = (uint16_t) (APpassword[7]<<8) + APpassword[6];
  ModbusData[n++] = (uint16_t) (APpassword[5]<<8) + APpassword[4];
  ModbusData[n++] = (uint16_t) (APpassword[3]<<8) + APpassword[2];
  ModbusData[n++] = (uint16_t) (APpassword[1]<<8) + APpassword[0];

  dataready = 0;                                                                // reset dataready and DSMRversion
  DSMRver = 0;
	
  if ((millis() - ModbusTimer) > 2500) LOG_W("Missed modbus response\n");
	ModbusTimer = millis();

  // Loop over all words to be sent
  for (uint16_t i = 0; i < words; i++) {
    // Add word to response buffer
    response.add(ModbusData[addr + i]);
  }
    
  // Return the data response
  return response;
}    


// Write Modbus data FC06
//
// Wire settings @ 0x0800
// Field rotation setting (bit 0)      00000000 0000000x -> 0= Rotation right 1= Rotation Left
// 3/4 wire configuration (bit 1)      00000000 000000x0 -> 0= 4Wire, 1= 3Wire
// 
ModbusMessage MBWriteFC06(ModbusMessage request) {
  ModbusMessage response;     // response message to be sent back

  uint16_t addr, value; 
    
  request.get(2, addr);   // 08 00 (address 0x800)
  request.get(4, value);  // 00 02 (4 wire, rotation right)

  // address valid?
  if (addr == 0x800) {
    // Set Wire bits 
    Wire = value & 3u;
    write_settings();
    return ECHO_RESPONSE;  

  } else if (addr == 0x801) {
    // Set WiFimode
    WIFImode = value & 3u;
    write_settings();
    return ECHO_RESPONSE;  
  }
  
  // No. Return error response
  response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_ADDRESS);
  return response;
}


//
// Toggle DE and /RE pins of RS485 transceiver
// (Called by eModbus)
void ToggleRS485(bool level) {

  digitalWrite (PIN_RS485_DE, level);
  digitalWrite (PIN_RS485_RE, level);
}

// ----------------------------------------------------------------------------------------------------------------
// Setup everything
//
//
void setup() {
  
	PicSetup();

  //lower the CPU frequency to 160 MHz
  //setCpuFrequencyMhz(160);

  pinMode(PIN_LED_GREEN, OUTPUT_OPEN_DRAIN);
  pinMode(PIN_LED_RED, OUTPUT_OPEN_DRAIN);

  pinMode (PIN_RS485_RX, INPUT);                                                // RS485 RX input
  pinMode (PIN_RS485_TX, OUTPUT);                                               // RS485 TX output
  pinMode (PIN_RS485_DE, OUTPUT);
  pinMode (PIN_RS485_RE, OUTPUT);
  pinMode (PIN_RX, INPUT);                                                      // P1 data input
  pinMode (PIN_TX, INPUT);                                                      // extra debug output on P1 port (unused)
  digitalWrite (PIN_RS485_RE, LOW);                                             // Read enabled


  // Setup Serial ports
  Serial.begin(115200, SERIAL_8N1, PIN_PGD, PIN_TXD, false);                    // Input from TX of PIC, and debug output to USB
  Serial1.begin(9600, SERIAL_8N1, PIN_RS485_RX, PIN_RS485_TX, false);           // Modbus connection
  Serial2.setRxBufferSize(2048);                                                // Important! first increase buffer, then setup Uart2
  Serial2.begin(115200, SERIAL_8N1, PIN_RX, -1, true);                          // P1 smartmeter connection, TX pin unused (RX inverted)
  
  while(!Serial); 
  Serial.print("\nSensorbox 2 powerup\n");

	// Initialize SPIFFS
	if(!SPIFFS.begin(true)){
		LOG_E("SPIFFS failed! already tried formatting. HALT\n");
    while (true) {
      delay(1);
    }
	}
	Serial.printf("Total SPIFFS bytes: %u, Bytes used: %u\n",SPIFFS.totalBytes(),SPIFFS.usedBytes());


	// Check if the PIC needs updating..
  if (Pic18ReadConfigs() == 0x6980) {
		Serial.printf("PIC18F26K40 found\n");

		if (SPIFFS.exists(PICfirmware))	{
			Serial.printf("%s found on SPIFFS\n", PICfirmware);
			file = SPIFFS.open(PICfirmware, "r");
			if (!file) {
				Serial.println("file open failed\n");
			} else {
				ProgramPIC(file);                                                       	// Program PIC
				file.close();                                                           	// close file after use
				SPIFFS.remove(PICfirmware);                                             	// erase hexfile, so we only program once
			}
		} else Serial.printf("%s -not- found on SPIFFS\n", PICfirmware);

	} else if (Pic16ReadConfigs() == 0x3043) {
		Serial.printf("PIC16F1704 found\n");
    PICfirmware = "/PIC16F1704.hex";
    
    if (SPIFFS.exists(PICfirmware))	{
			Serial.printf("%s found on SPIFFS\n", PICfirmware);
			file = SPIFFS.open(PICfirmware, "r");
			if (!file) {
				Serial.println("file open failed\n");
			} else {
				ProgramPIC16F(file);                                                      // Program PIC
				file.close();                                                           	// close file after use
				SPIFFS.remove(PICfirmware);                                             	// erase hexfile, so we only program once
			}
		} else Serial.printf("%s -not- found on SPIFFS\n", PICfirmware);
  } else Serial.printf("No PIC found, Not possible to do CT measurements!\n");

	pinMode(PIN_PGD, INPUT);
  pinMode(PIN_PGC, OUTPUT);


  // Read all settings from non volatile memory
  // store default values if uninitialized
  read_settings(true);

  
  
  // Setup Modbus Server/Node
  // Set eModbus LogLevel to 1, to suppress possible E5 errors
  MBUlogLvl = LOG_LEVEL_CRITICAL;
  // Register worker. at address '10', function code 03 & 04
  MBserver.registerWorker(10U, READ_HOLD_REGISTER, &MBReadFC04);  
  MBserver.registerWorker(10U, READ_INPUT_REGISTER, &MBReadFC04);
  // Register worker for address '10', function code 06
  MBserver.registerWorker(10U, WRITE_HOLD_REGISTER, &MBWriteFC06);
  // Start ModbusRTU Node background task
  MBserver.start();
  
  
  // Create Task that handles P1 and CT data
  xTaskCreate(
    P1Task,               // Function that should be called
    "P1Task",             // Name of the task (for debugging)
    3000,                 // Stack size (bytes) 
    NULL,                 // Parameter to pass
    1,                    // Task priority
    NULL                  // Task handle
  );

// Create Task that blinks the LED, and cleans up network clients
  xTaskCreate(
    BlinkLed,             // Function that should be called
    "BlinkLed",           // Name of the task (for debugging)
    2000,                 // Stack size (bytes)
    NULL,                 // Parameter to pass
    1,                    // Task priority
    NULL                  // Task handle
  );
/*
  // Create Task that setups the network, and the webserver 
  xTaskCreate(
    SetupNetworkTask,     // Function that should be called
    "SetupNetworkTask",   // Name of the task (for debugging)
    10000,                // Stack size (bytes)
    NULL,                 // Parameter to pass
    1,                    // Task priority
    NULL                  // Task handle
  );
*/

  Serial.print("Configuring WDT...\n");
  esp_task_wdt_init(WDT_TIMEOUT, true);     // Setup watchdog
  esp_task_wdt_add(NULL);                   // add current thread to WDT watch


  // We might need some sort of authentication in the future.
  // Sensorbox 2 (hwver 1.10) have programmed ECDSA-256 keys stored in nvs
  // Unused for now.
  if (preferences.begin("KeyStorage", true) == true) {        // readonly
    uint16_t hwversion = preferences.getUShort("hwversion");   // 0x020A (02 = Sensorbox-2 0A = hwver 1.10)
    serialnr = preferences.getUInt("serialnr");
    String ec_private = preferences.getString("ec_private");
    String ec_public = preferences.getString("ec_public");
    preferences.end(); 
//    Serial.printf("hwversion %04x serialnr:%u \n",hwversion, serialnr);
//    Serial.print(ec_public);

  } else Serial.print("No KeyStorage found in nvs!\n");

  WiFiSetup();

}


//make mongoose 7.14 compatible with 7.13
#define mg_http_match_uri(X,Y) mg_match(X->uri, mg_str(Y), NULL)

// handles URI, returns true if handled, false if not
bool handle_URI(struct mg_connection *c, struct mg_http_message *hm) {
//    if (mg_match(hm->uri, mg_str("/settings"), NULL)) {               // REST API call?
    if (mg_http_match_uri(hm, "/settings")) {                            // REST API call?
          if (!memcmp("GET", hm->method.buf, hm->method.len)) {                     // if GET
/*            String mode = "N/A";
            int modeId = -1;
            if(Access_bit == 0)  {
                mode = "OFF";
                modeId=0;
            } else {
                switch(Mode) {
                    case MODE_NORMAL: mode = "NORMAL"; modeId=1; break;
                    case MODE_SOLAR: mode = "SOLAR"; modeId=2; break;
                    case MODE_SMART: mode = "SMART"; modeId=3; break;
                }
            }
            String backlight = "N/A";
            switch(BacklightSet) {
                case 0: backlight = "OFF"; break;
                case 1: backlight = "ON"; break;
                case 2: backlight = "DIMMED"; break;
            }
            String evstate = StrStateNameWeb[State];
            String error = getErrorNameWeb(ErrorFlags);
            int errorId = getErrorId(ErrorFlags);

            if (ErrorFlags & NO_SUN) {
                evstate += " - " + error;
                error = "None";
                errorId = 0;
            }

            boolean evConnected = pilot != PILOT_12V;                    //when access bit = 1, p.ex. in OFF mode, the STATEs are no longer updated
*/
            DynamicJsonDocument doc(1600); // https://arduinojson.org/v6/assistant/
            doc["version"] = String(SENSORBOX_SWVER);
            doc["serialnr"] = serialnr;

            if(WiFi.isConnected()) {
                switch(WiFi.status()) {
                    case WL_NO_SHIELD:          doc["wifi"]["status"] = "WL_NO_SHIELD"; break;
                    case WL_IDLE_STATUS:        doc["wifi"]["status"] = "WL_IDLE_STATUS"; break;
                    case WL_NO_SSID_AVAIL:      doc["wifi"]["status"] = "WL_NO_SSID_AVAIL"; break;
                    case WL_SCAN_COMPLETED:     doc["wifi"]["status"] = "WL_SCAN_COMPLETED"; break;
                    case WL_CONNECTED:          doc["wifi"]["status"] = "WL_CONNECTED"; break;
                    case WL_CONNECT_FAILED:     doc["wifi"]["status"] = "WL_CONNECT_FAILED"; break;
                    case WL_CONNECTION_LOST:    doc["wifi"]["status"] = "WL_CONNECTION_LOST"; break;
                    case WL_DISCONNECTED:       doc["wifi"]["status"] = "WL_DISCONNECTED"; break;
                    default:                    doc["wifi"]["status"] = "UNKNOWN"; break;
                }

                doc["wifi"]["ssid"] = WiFi.SSID();
                doc["wifi"]["rssi"] = WiFi.RSSI();
                doc["wifi"]["bssid"] = WiFi.BSSIDstr();
            }

            // stuff to keep compatibility with SmartEVSE
            doc["settings"]["mains_meter"] = "Sensorbox";
            doc["evse"]["temp"] = 0;
            //doc["evse"]["temp"] = TempEVSE; TODO
            doc["ev_meter"]["description"] = "Disabled"; //TODO
            doc["ev_meter"]["currents"]["TOTAL"] = 0; //TODO
            doc["ev_meter"]["currents"]["L1"] = 0;
            doc["ev_meter"]["currents"]["L2"] = 0;
            doc["ev_meter"]["currents"]["L3"] = 0;


    #if MQTT
            doc["mqtt"]["host"] = MQTTHost;
            doc["mqtt"]["port"] = MQTTPort;
            doc["mqtt"]["topic_prefix"] = MQTTprefix;
            doc["mqtt"]["username"] = MQTTuser;
            doc["mqtt"]["password_set"] = MQTTpassword != "";

            if (MQTTclient.connected) {
                doc["mqtt"]["status"] = "Connected";
            } else {
                doc["mqtt"]["status"] = "Disconnected";
            }
    #endif
  /*
            doc["ev_meter"]["description"] = EMConfig[EVMeter.Type].Desc;
            doc["ev_meter"]["address"] = EVMeter.Address;
            doc["ev_meter"]["import_active_power"] = round((float)EVMeter.PowerMeasured / 100)/10; //in kW, precision 1 decimal
            doc["ev_meter"]["total_kwh"] = round((float)EVMeter.Energy / 100)/10; //in kWh, precision 1 decimal
            doc["ev_meter"]["charged_kwh"] = round((float)EVMeter.EnergyCharged / 100)/10; //in kWh, precision 1 decimal
            doc["ev_meter"]["currents"]["TOTAL"] = EVMeter.Irms[0] + EVMeter.Irms[1] + EVMeter.Irms[2];
            doc["ev_meter"]["currents"]["L1"] = EVMeter.Irms[0];
            doc["ev_meter"]["currents"]["L2"] = EVMeter.Irms[1];
            doc["ev_meter"]["currents"]["L3"] = EVMeter.Irms[2];
            doc["ev_meter"]["import_active_energy"] = round((float)EVMeter.Import_active_energy / 100)/10; //in kWh, precision 1 decimal
            doc["ev_meter"]["export_active_energy"] = round((float)EVMeter.Export_active_energy / 100)/10; //in kWh, precision 1 decimal

            doc["mains_meter"]["import_active_energy"] = round((float)MainsMeter.Import_active_energy / 100)/10; //in kWh, precision 1 decimal
            doc["mains_meter"]["export_active_energy"] = round((float)MainsMeter.Export_active_energy / 100)/10; //in kWh, precision 1 decimal
   */
            doc["phase_currents"]["TOTAL"] = MainsMeterIrms[0] + MainsMeterIrms[1] + MainsMeterIrms[2];
            doc["phase_currents"]["L1"] = MainsMeterIrms[0];
            doc["phase_currents"]["L2"] = MainsMeterIrms[1];
            doc["phase_currents"]["L3"] = MainsMeterIrms[2];
            doc["phase_currents"]["last_data_update"] = phasesLastUpdate;

            String json;
            serializeJson(doc, json);
            mg_http_reply(c, 200, "Content-Type: application/json\r\n", "%s\n", json.c_str());    // Yes. Respond JSON
          } else {
            mg_http_reply(c, 404, "", "Not Found\n");
          }
          return true;
    }
    return false;
}

//
// This code will run forever
//
void loop() {
    network_loop();
    static unsigned long lastCheck = 0;
    if (millis() - lastCheck >= 1000) {
        lastCheck = millis();
        //this block is for non-time critical stuff that needs to run approx 1 / second
        // reset the WDT every second
        esp_task_wdt_reset();

        _LOG_A("Status: %04x Time: %02u:%02u Date: %02u/%02u/%02u Day:%u ", WIFImode + (LocalTimeSet << 8), timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_mday, timeinfo.tm_mon+1, timeinfo.tm_year%100, timeinfo.tm_wday);
        _LOG_A("Connected to AP: %s Local IP: %s\n", WiFi.SSID().c_str(), WiFi.localIP().toString().c_str());
    }
    //vTaskDelay(1000 / portTICK_PERIOD_MS);
}
