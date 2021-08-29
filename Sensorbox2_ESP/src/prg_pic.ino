#include <Arduino.h>

#include "radix.h"

#define SWAP16(x) (((x & 0x00ff) << 8) | ((x & 0xff00) >> 8))

#define FLASHSIZE       0xFFFF
#define PICMAXWORDS     0xFFFF
#define PICBLANKWORD    0xFFFF
#define WRITE_LATCHES   64
#define ROW_SIZE        32
#define ADR_PROGRAM     0x000000
#define ADR_USER_IDS    0x200000
#define ADR_CONFIG      0x300000
#define ADR_EEPROM      0x310000
#define ADR_CHIP_IDS    0x3FFFFC

#define DLY1  1      // 1 microsecond for toggling and stuff
#define DLY2  6000   // 6  millisecond for command delay

#define PIN_RESET 21
#define PIN_DAT   23
#define PIN_CLK   22

#define DAT_INPUT   pinMode(PIN_DAT, INPUT);
#define DAT_OUTPUT  pinMode(PIN_DAT, OUTPUT);
#define DAT_LOW     digitalWrite(PIN_DAT,   LOW)
#define DAT_HIGH    digitalWrite(PIN_DAT,   HIGH)
#define DAT_GET     digitalRead(PIN_DAT)

#define CLK_INPUT   pinMode(PIN_CLK, INPUT);
#define CLK_OUTPUT  pinMode(PIN_CLK, OUTPUT);
#define CLK_LOW     digitalWrite(PIN_CLK,   LOW)
#define CLK_HIGH    digitalWrite(PIN_CLK,   HIGH)

#define RESET_INPUT  pinMode(PIN_RESET, INPUT);
#define RESET_OUTPUT pinMode(PIN_RESET, OUTPUT);
void RESET_LOW(){   digitalWrite(PIN_RESET, LOW);}
void RESET_HIGH(){  digitalWrite(PIN_RESET, HIGH);}

void RESET_PIN_INPUT() { RESET_INPUT;}

uint16_t currentAddress;


//
//
//
void PicSetup() {
  RESET_OUTPUT;
  DAT_INPUT;
  CLK_INPUT;
  RESET_HIGH();
}


//
// Clock out 8 bits of data to the PIC.
//
void Send(uint8_t data) {
  DAT_OUTPUT;
  delayMicroseconds(DLY1);
  for (uint8_t i=0; i<8; i++) {
    if (data&0x80) {
      DAT_HIGH;
    } else {
      DAT_LOW;
    }
    delayMicroseconds(DLY1);
    CLK_HIGH;
    delayMicroseconds(DLY1);
    CLK_LOW;
    delayMicroseconds(DLY1);
    data = data << 1;
  }
  DAT_LOW;
}

//
// Send the magic 32 bits to get into the LVP programming
// mode as long as the RESET line is kept low.
//
void EnterLVPmode() {
  DAT_OUTPUT;
  CLK_OUTPUT;
  CLK_LOW;
  DAT_LOW;
  delayMicroseconds(500);
  RESET_LOW();
  delayMicroseconds(500);
  Send('M');
  Send('C');
  Send('H');
  Send('P');
  delayMicroseconds(DLY1);
  Serial.printf("\nPIC entered LVP mode\n");
 }

//
// Exit LVP programming mode of PIC microcontroller
//
void ExitLVPmode() {
     RESET_HIGH();
     DAT_INPUT;
     CLK_INPUT;
     Serial.printf("\nPIC exited LVP mode\n");
 }


//
// Clock in 16 bits of data from the PIC
//
uint16_t Read16(void){
  uint16_t data=0;

  DAT_INPUT;
  delayMicroseconds(DLY1);
  for (uint8_t i=0; i<24; i++) {
    data=data << 1;
    if (DAT_GET) data = data | 0x0001;
    CLK_HIGH;
    delayMicroseconds(DLY1);
    CLK_LOW;
    delayMicroseconds(DLY1);
  }
  return data;
}


//
// Send the Load PC ADDRESS command to the PIC
//
void CmdLoadPCAddress(uint32_t address) {
  Send(0x80);                     // Command= set PC address
  address = address << 1;         // shift the address to the left, as bit 0 is not used
  Send((address>>16) & 0xff);     // MSB first
  Send((address>>8) & 0xff);
  Send(address & 0xfe);
  delayMicroseconds(DLY1);
}

//
// Send the INCREMENT ADDRESS command to the PIC
//
void CmdIncAddress(void) {
  currentAddress++;
  Send(0xf8);
  delayMicroseconds(DLY1);
}

//
// Send the BEGIN PROGRAMMING INTERNAL TIMED command to the PIC
//
void CmdBeginProgramI(void) {
  Send(0xE0);
  delayMicroseconds(DLY2);                            // 6ms delay for program memory
}

//
// Send the READ DATA command to the PIC
//
uint16_t CmdReadData() {
  Send(0xFE);                                         // Read data from Flash, and increase PC
  delayMicroseconds(DLY1);
  return Read16();
}


//
// Send 16 bit data to program buffer, (and increase PC)
//
void CmdLoadData(uint16_t data, uint8_t n) {
  if (n == (WRITE_LATCHES-2) ) {
    Send(0x00);                                                                 // don't increase PC
    //Serial.printf("last word..");
  } else Send(0x02);                                         // Load data to program buffer, and increase PC
  delayMicroseconds(DLY1);
  Send((data>>15) & 0xff);                            // MSB is on bit 16
  Send((data>>7) & 0xff);
  Send((data<<1) & 0xfe);                             // LSB is on bit 1, bit 0 is always zero
}



//
// Send the BULK ERASE command to the PIC
//
void CmdBulkErase(void) {
  Send(0x18);
  delay(26);                                      // wait 26ms for bulk erase to complete
}


//
//
//
void PicReadConfigs() {
  EnterLVPmode();
  CmdLoadPCAddress(ADR_CHIP_IDS);
  Revision= CmdReadData();
  DeviceID= CmdReadData();
  Serial.printf("%04X %04X",Revision, DeviceID);
  ExitLVPmode();
}




uint8_t ProgramPIC(File file) {

  uint32_t filesize, filepointer=0;
  uint8_t n, p, checksum, bytecount, recordtype, startoffset;
  uint16_t fileaddress, configdata, FlashADR_MSB=0, PrevFlashADR=0;
  char filebuffer[90];                                                          // holds one row of the hex file
  char programbuffer[WRITE_LATCHES];                                            // block of flash data for the PIC

  filesize = file.size();
  Serial.printf("filesize %u bytes\n", filesize);

  EnterLVPmode();
  CmdLoadPCAddress(ADR_CONFIG);         // Program Flash Memory
  CmdBulkErase();                     // Erase Program Flash, ID and Configuration
  FlashADR = ADR_PROGRAM;
  CmdLoadPCAddress(FlashADR);         // set PC to 0


  memset(programbuffer, 0xff, sizeof(programbuffer));                           // clear programbuffer

  while (1) {                                                                   // loop till end of file
    file.seek(filepointer);
    file.readBytes(filebuffer,89);

    n = 0;
    while ((filebuffer[n++]!=':') && (n<90)) { }                                  // find the startcode ':' in buffer
    startoffset = n;

    checksum = 0;
    while ( (filebuffer[n]!=0x0a) && (filebuffer[n]!=0x0d) && (n<45)) {            // verify checksum for each line in the hex file
      checksum = checksum + HexDec2(filebuffer[n], filebuffer[n+1]);
    //  Serial.printf("%c%c", filebuffer[n],filebuffer[n+1]);
      n=n+2;
    }
  //  Serial.printf("\n");
    if (checksum!=0) {
      Serial.printf("checksum error in HEX file %u\n", checksum);
      ExitLVPmode();
      return 1;
    }
    filepointer = filepointer + n;                                                // set filepointer to end of line
    //Serial.printf("\nfilepointer set to %u\n", filepointer);

    // checksum verified OK
    // now decode the line

    n = startoffset;                                                            // begin of hex data, bytecount
    bytecount = HexDec2(filebuffer[n], filebuffer[n+1]);
    n=n+2;
    fileaddress = HexDec4(filebuffer[n], filebuffer[n+1], filebuffer[n+2], filebuffer[n+3]);
    n=n+4;
    recordtype = HexDec2(filebuffer[n], filebuffer[n+1]);
    n=n+2;
    if (recordtype == 0x01) {                                                     // End of file
      // it would be wise to do a verification of the programmed data at this point, but we ignore this for now
      Serial.printf("\nEnd of Hexfile\n");
      ExitLVPmode();
      return 0;

    } else if (recordtype == 0x04) {                                              // Change FLASHaddress MSB
      FlashADR_MSB = HexDec4(filebuffer[n], filebuffer[n+1], filebuffer[n+2], filebuffer[n+3]);
      n=n+4;

      // flash any unprogrammed data into PIC
      if (FlashADR!=0)
      {
        CmdLoadPCAddress(FlashADR);                                           // load address to program into PIC
        Serial.printf("FlashADR %08X: ",FlashADR);
        PrevFlashADR = FlashADR;
        for (p=0; p<WRITE_LATCHES; p=p+2) {
          CmdLoadData((programbuffer[p+1]<<8) + programbuffer[p], p);           // Convert endiness
        //  Serial.printf("%02X%02X",programbuffer[p+1], programbuffer[p]);
        }
        Serial.printf("\r");
        CmdBeginProgramI();
        memset(programbuffer, 0xff, sizeof(programbuffer));
      }
    //
    // Recordtype 00
    //
    } else if (recordtype == 0x00) {                                            // DATA
      FlashADR = fileaddress & 0x00ffffc0;                     //~(WRITE_LATCHES-1);

      // Check if there is a skip in the address , outside of the WRITE_LATCHES size
      // If so, program the flash with the already processed data
      // make sure the FlashADR is set to the previous address
      if (FlashADR > (PrevFlashADR+WRITE_LATCHES)) {                              // outside the boundary?

        PrevFlashADR+=WRITE_LATCHES;                                            // fix address
        CmdLoadPCAddress(PrevFlashADR);                                             // load address to program into PIC
        Serial.printf("PrevFlashADR %08X: ",PrevFlashADR);
        PrevFlashADR = FlashADR;
        for (p=0; p<WRITE_LATCHES; p=p+2) {
          CmdLoadData((programbuffer[p+1]<<8) + programbuffer[p], p);           // Convert endiness
        //  Serial.printf("%02X%02X",programbuffer[p+1], programbuffer[p]);
        }
        Serial.printf("\n");
        CmdBeginProgramI();
        memset(programbuffer, 0xff, sizeof(programbuffer));
      }

      // Configuration or UserID area
      if (FlashADR_MSB) {
        CmdLoadPCAddress(FlashADR_MSB<<16);                                     // load address to program into PIC
        Serial.printf("ExtFlashADR %08X: ",(FlashADR_MSB<<16));
        for (n=0; n<(bytecount*2); n=n+4) {
          configdata = HexDec4(filebuffer[startoffset+n+10], filebuffer[startoffset+n+11],filebuffer[startoffset+n+8], filebuffer[startoffset+n+9]);
        //  Serial.printf("%04X ",configdata);
          CmdLoadData(configdata, WRITE_LATCHES-2);
          CmdBeginProgramI();
          CmdIncAddress();
        }
        Serial.printf("\r");
      } else {
        // process latest row of data
        for (n=0; n<(bytecount*2); n=n+2) {
          programbuffer[fileaddress % WRITE_LATCHES] = HexDec2(filebuffer[startoffset+n+8], filebuffer[startoffset+n+9]);
          fileaddress ++;

          // Normal program flash
          if ((fileaddress % WRITE_LATCHES) == 0 ) {                              // Write latch boundary?
            CmdLoadPCAddress(FlashADR);                                           // load address to program into PIC
            Serial.printf("FlashADR %08X: ",FlashADR);
            PrevFlashADR = FlashADR;
            for (p=0; p<WRITE_LATCHES; p=p+2) {
              CmdLoadData((programbuffer[p+1]<<8) + programbuffer[p], p);       // Convert endiness
          //    Serial.printf("%02X%02X",programbuffer[p+1], programbuffer[p]);
            }
            Serial.printf("\r");
            CmdBeginProgramI();
            memset(programbuffer, 0xff, sizeof(programbuffer));
          }

        }
      } //else
    } // recordtype 00

  } // while loop



}
