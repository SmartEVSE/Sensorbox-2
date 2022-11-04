/*
;   Project:       Sensorbox 2, for use with Smart EVSE
;
;   Changes:
;   2.0  Initial release
;   2.1  Added support for switching between 4/3 wire mode, and Field rotation
;   2.1.1 added support for Sagemcom T-211D
;
;   (C) 2013-2022  Michael Stegen / Stegen Electronics
;
;	Power/Current measurement calculations, from openenergymonitor.org
;
;
;   Note: Errata for this chip suggests that the following put into a
;    "powerup" assembly file
;    
;    asm("  BSF NVMCON1, 7");
;
;   Note that, if anywhere in the code, you need to change NVMCON1<7:6>
;   to anything other than 0b10, be sure to change it back afterwards!
;   In other words:
;   You will have to change NVMCON1bits.NVMREG to access User ID,
;   configuration bits, Rev ID, Device ID or Data EEPROM.
;   If you do any of things, be sure to change it back to 0b10 (!))
;
; 
;   set XC8 linker memory model settings to: double 32 bit, float 32 bit
;   extended instruction set is not used on XC8
;
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
;   Holding Registers (FC=06)(Write):
;
;   Register  Register  
;   Address   length (16 bits) 
;   0x0800      1       Field rotation setting (bit 0)      00000000 0000000x -> 0= Rotation right 1= Rotation Left
;                       3/4 wire configuration (bit 1)      00000000 000000x0 -> 0= 4Wire, 1= 3Wire
; 
; 
; Permission is hereby granted, free of charge, to any person obtaining a copy
; of this software and associated documentation files (the "Software"), to deal
; in the Software without restriction, including without limitation the rights
; to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
; copies of the Software, and to permit persons to whom the Software is
; furnished to do so, subject to the following conditions:
;
; The above copyright notice and this permission notice shall be included in
; all copies or substantial portions of the Software.
;
; THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
; IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
; FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
; AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
; LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
; OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
; THE SOFTWARE.
*/

#include <pic18f26k40.h>




// PIC18F26K40 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1L
#pragma config FEXTOSC = OFF    // External Oscillator mode Selection bits (Oscillator not enabled)
#pragma config RSTOSC = HFINTOSC_64MHZ// Power-up default value for COSC bits (HFINTOSC with HFFRQ = 64 MHz and CDIV = 1:1)

// CONFIG1H
#pragma config CLKOUTEN = OFF   // Clock Out Enable bit (CLKOUT function is disabled)
#pragma config CSWEN = ON       // Clock Switch Enable bit (Writing to NOSC and NDIV is allowed)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor enabled)

// CONFIG2L
#pragma config MCLRE = INTMCLR  // Master Clear Enable bit (If LVP = 0, MCLR pin function is port defined function; If LVP =1, RE3 pin fuction is MCLR)
#pragma config PWRTE = ON       // Power-up Timer Enable bit (Power up timer enabled)
#pragma config LPBOREN = OFF    // Low-power BOR enable bit (ULPBOR disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable bits (Brown-out Reset enabled according to SBOREN)

// CONFIG2H
#pragma config BORV = VBOR_2P45 // Brown Out Reset Voltage selection bits (Brown-out Reset Voltage (VBOR) set to 2.45V)
#pragma config ZCD = OFF        // ZCD Disable bit (ZCD disabled. ZCD can be enabled by setting the ZCDSEN bit of ZCDCON)
#pragma config PPS1WAY = ON     // PPSLOCK bit One-Way Set Enable bit (PPSLOCK bit can be cleared and set only once; PPS registers remain locked after one clear/set cycle)
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config DEBUG = OFF      // Debugger Enable bit (Background debugger disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Extended Instruction Set and Indexed Addressing Mode disabled)

// CONFIG3L
#pragma config WDTCPS = WDTCPS_31// WDT Period Select bits (Divider ratio 1:65536; software control of WDTPS)
#pragma config WDTE = OFF       // WDT operating mode (WDT Disabled)

// CONFIG3H
#pragma config WDTCWS = WDTCWS_7// WDT Window Select bits (window always open (100%); software control; keyed access not required)
#pragma config WDTCCS = SC      // WDT input clock selector (Software Control)

// CONFIG4L
#pragma config WRT0 = OFF       // Write Protection Block 0 (Block 0 (000800-003FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection Block 1 (Block 1 (004000-007FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection Block 2 (Block 2 (008000-00BFFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection Block 3 (Block 3 (00C000-00FFFFh) not write-protected)

// CONFIG4H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-30000Bh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)
#pragma config SCANE = ON       // Scanner Enable bit (Scanner module is available for use, SCANMD bit can control the module)
#pragma config LVP = ON         // Low Voltage Programming Enable bit (Enabled)

// CONFIG5L
#pragma config CP = ON          // UserNVM Program Memory Code Protection bit (UserNVM code protection enabled)
#pragma config CPD = OFF        // DataNVM Memory Code Protection bit (DataNVM code protection disabled)

// CONFIG5H

// CONFIG6L
#pragma config EBTR0 = OFF      // Table Read Protection Block 0 (Block 0 (000800-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection Block 1 (Block 1 (004000-007FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection Block 2 (Block 2 (008000-00BFFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection Block 3 (Block 3 (00C000-00FFFFh) not protected from table reads executed in other blocks)

// CONFIG6H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot Block (000000-0007FFh) not protected from table reads executed in other blocks)

// ID stores the version nr
#pragma config IDLOC0 = 1       // version, unused


// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.


#include <xc.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include "Sensorbox2.h"

// Global data
char Modbus_TX[100], Modbus_RX[50], P1_data[2000];
double Irms[3], Volts[3], IrmsCT[3], realPower[3], Vrms;
double x1, x2, x3, y1, y2, y3;                                                  // phase shift coefficients

double i1Lead = 19.00;     // degrees that the voltage input phase error leads the ct1 phase error by
double i2Lead = 19.00;     // degrees that the voltage input phase error leads the ct2 phase error by
double i3Lead = 19.00;     // degrees that the voltage input phase error leads the ct3 phase error by

double i1phaseshift, i2phaseshift, i3phaseshift;
double apparentPower1,powerFactor1;
double apparentPower2,powerFactor2;
double apparentPower3,powerFactor3;

char LedSeq[3] = {0,0,0};
unsigned long Timer, LedTimer, ModbusTimer=0, P1Timer=0;        				// mS counter
char RXbyte, dataready=0, P1_EOT=0, spd;
unsigned char ModbusIdx=0, ModbusTXlen=0, ModbusTXidx=0;
unsigned int P1_ptr=0, P1_length=0, T1Period=1, T1PLL=2500, sumPeriodSamples=0, PLLLock=0; 
unsigned char DSMRver;
unsigned char newsumCycle=0;
unsigned char Phase1 = PHASE0, Phase2 = PHASE120, Phase3 = PHASE240, version = 0x14+WIRES+ROTATION;
//unsigned int T1PeriodReadout=0;


//int ADCresult, ADCmax,ADCmaxidx, ADCpeak=0, ADCidx=0;
int newV, lastV, result ,sampleI[3];
volatile unsigned long sumVsq, sumI1sq, sumI2sq, sumI3sq;
volatile long sumVavg, sumI1avg, sumI2avg, sumI3avg, debugIavg;
volatile long sumPower1A, sumPower1B, sumPower2A, sumPower2B, sumPower3A, sumPower3B; 
//volatile long sumI1dir, sumI2dir, sumI3dir;
volatile unsigned int sumSamples, Samples;

unsigned long sumPeriodVsq, sumPeriodI1sq, sumPeriodI2sq, sumPeriodI3sq;
long sumPeriodVavg, sumPeriodI1avg, sumPeriodI2avg, sumPeriodI3avg;
long sumPeriodPower1A, sumPeriodPower1B, sumPeriodPower2A, sumPeriodPower2B, sumPeriodPower3A, sumPeriodPower3B; 

//int voltage[NUMSAMPLES*2];

const signed char voltage[]= {0,6,13,19,25,31,37,42,46,50,54,57,59,61,62,62,62,61,59,57,54,50,46,42,37,31,25,19,13,6,
                              0,-6,-13,-19,-25,-31,-37,-42,-46,-50,-54,-57,-59,-61,-62,-62,-62,-61,-59,-57,-54,-50,-46,-42,-37,-31,-25,-19,-13,-6,
                              0,6,13,19,25,31,37,42,46,50,54,57,59,61,62,62,62,61,59,57,54,50,46,42,37,31,25,19,13,6,
                              0,-6,-13,-19,-25,-31,-37,-42,-46,-50,-54,-57,-59,-61,-62,-62,-62,-61,-59,-57,-54,-50,-46,-42,-37,-31,-25,-19,-13,-6 };


volatile int lastSampleI[3], tempI;      // sample holds the raw analog read value, lastSample holds the last sample
volatile long filteredI[3], filtI_div4, tempL;
volatile unsigned long sqI, sumI[] = {0,0,0};
volatile unsigned int SumSamples2 = 0;
volatile unsigned char TotalSumReady = 0, NoLock = 0;
volatile unsigned long TotalSumI[3];


void interrupt high_isr(void)
{
   // Determine what caused the interrupt
    
    while(PIR4bits.TMR1IF)                                                      // Timer 1 interrupt, called 3200 times/sec for ADC measurement
	{
        ADPCH = 0;                                                              // Select ADC channel CT1

        T1Period++;
        TMR1 = ~(T1PLL);                                                        // reset Timer 1

        if (T1Period<=NUMSAMPLES || NoLock) {                                   // start ADC conversion on the selected channel.
            ADCON0bits.ADGO = 1;                                                // ADC will generate interrupt when finished.
        } 
                                     
		PIR4bits.TMR1IF = 0;                                                    // clear Timer 1 interrupt flag
	}
    
    while(IOCBFbits.IOCBF0)                                                     // External pin interrupt triggered 50 times/sec (50Hz) (determined by line frequency)
    {
        ADPCH = 0;                                                              // Select ADC channel CT1

        if (T1Period>NUMSAMPLES) T1PLL+=10;                                     // we might have to adjust the T1 PLL value 
        else if (T1Period<NUMSAMPLES) T1PLL--;                                  // to make sure we sample 'NUMSAMPLES' times per period (0.02sec)

        TMR1 = ~(T1PLL);                                                        // reset Timer 1
        
        if (sumSamples<=NUMSAMPLES) {
            ADCON0bits.ADGO = 1;                                                // start ADC conversion on the selected channel.        
        } 
        newPeriod();                                                            // Last sample in this period
        
        T1Period = 1;                                                           // Start with 1
        PLLLock = 0;
        NoLock = 0;                                                             // reset flag
        IOCBFbits.IOCBF6 = 0;                                                   // Clear external Interrupt flag
        IOCBFbits.IOCBF0 = 0;                                                   // Clear interrupt flag
    }
    
#ifdef ESP_DIRECT    
    while(IOCBFbits.IOCBF6 )                                                    // External pin (RB6) interrupt
    {
        version ++;                                                             // increase version, this will change phase rotation and 3/4 Wire
        if (version == 0x14) version = 0x10;
        IOCBFbits.IOCBF6 = 0;                                                   // Clear Interrupt flag
    }
#endif
    
    while(PIR1bits.ADIF)                                                        // ADC conversion done interrupt
    {
    //  ADPCH = 0x3C;                                                           // select RA0 (GND)
        
        result = ADRES;                                                         // Get ADC result
        if (NoLock == 0) result -= 512;                                         // remove nominal offset

        if (ADPCH == 0)                                                         // CT1
        {
            ADPCH = 1;                                                          // Select ADC channel CT2
            lastSampleI[0] = sampleI[0];
            ADCON0bits.ADGO = 1;                                                // start ADC conversion on CT2
            sampleI[0] = result;                                                // process CT1 sample
            sumI1sq += (long)sampleI[0] * sampleI[0];
            sumI1avg += sampleI[0];
        }
        else if (ADPCH == 1)                                                    // CT2
        {
            ADPCH = 2;                                                          // Select ADC channel CT3
            lastSampleI[1] = sampleI[1];
            ADCON0bits.ADGO = 1;                                                // start ADC conversion on CT3
            sampleI[1] = result;
            sumI2sq += (long)sampleI[1] * sampleI[1];
            sumI2avg += sampleI[1];
        }
        else if (ADPCH == 2)                                                    // CT 3
        {
            ADPCH = 0;                                                          // Select first ADC channel (CT1)
            lastSampleI[2] = sampleI[2];
            sampleI[2] = result;
            sumI3sq += (long)sampleI[2] * sampleI[2];
            sumI3avg += sampleI[2];
            
            if (NoLock) {
                for (uint8_t i=0; i<3; i++) {
                
                    tempL = (long)(sampleI[i] - lastSampleI[i])<<8;             // re-scale the input change (x256)
                    tempL += filteredI[i];                                      // combine with the previous filtered value
                    filteredI[i] = tempL-(tempL>>8);                            // subtract 1/256, same as x255/256

                    filtI_div4 = filteredI[i]>>2;                               // now x64
                    // Root-mean-square method current
                    // 1) square current values
                    sqI = (unsigned long)(filtI_div4 * filtI_div4);
                    sqI = sqI >>12;                                             // scale back
                    // 2) sum
                    sumI[i] += sqI;
                }

                if (++SumSamples2 == SAMPLES) {
                
                    for (uint8_t i=0 ;i<3 ;i++) {
                        TotalSumI[i] = sumI[i];                                 // copy to variable to be used in main loop
                        sumI[i] = 0;  
                    }
                    SumSamples2 = 0;
                    TotalSumReady = 1;
                }
              
            } else {

                newV = voltage[T1Period-1+NUMSAMPLES-Phase1];
                sumVsq += ((long)newV * newV);
                sumVavg += newV;
                sumPower1A += (long)voltage[T1Period-1+NUMSAMPLES-Phase1] * sampleI[0]; // Use stored & delayed voltage for power calculation phase 1
                sumPower1B += (long)voltage[T1Period-1+NUMSAMPLES-Phase1-1] * sampleI[0];
                sumPower2A += (long)voltage[T1Period-1+NUMSAMPLES-Phase2] * sampleI[1]; // Use stored & delayed voltage for power calculation phase 2
                sumPower2B += (long)voltage[T1Period-1+NUMSAMPLES-Phase2-1] * sampleI[1];
                sumPower3A += (long)voltage[T1Period-1+NUMSAMPLES-Phase3] * sampleI[2]; // Use stored & delayed voltage for power calculation phase 3
                sumPower3B += (long)voltage[T1Period-1+NUMSAMPLES-Phase3-1] * sampleI[2];

                sumSamples++;
            }    
        }
         PIR1bits.ADIF = 0;                                                     // Clear ADC interrupt flag
    }    
    
#ifndef ESP_DIRECT
	while(PIR3bits.RC2IF)                                                       // Uart2 receive interrupt? P1 PORT
	{
		RXbyte = RC2REG;                                                        // copy received byte
        if (RXbyte == '/')                                                      // Start character
        {
            P1_ptr=0;                                                           // start from beginning of buffer
            P1_EOT=0;                                                           // reset End of Transmission flag
        }    
        if (P1_EOT) 
        {
            if (P1_EOT>4) RXbyte=0;                                             // if EOT>4 the CRC16 has been received, and the message can be verified.
            P1_EOT++;                                                           // count bytes after End of transmission
        }   
        P1_data[P1_ptr++] = RXbyte;                                             // Store in buffer
		if (P1_ptr==2000) P1_ptr--;                                             // prevent overflow of buffer
        
        if (RXbyte == '!')
        {
            P1_length = P1_ptr;    
            P1_EOT=1;                                                           // end character, flag end of transmission
        }
	}
    
    while(PIR3bits.RC1IF)                                                       // Uart1 receive interrupt? RS485 @9600 bps
	{
		RXbyte = RC1REG;                                                        // copy received byte	

        if (Timer>(ModbusTimer+3))                                              // last reception more then 3ms ago? 
        {
             ModbusIdx =0;                                                      // clear idx in RS485 RX handler
        }  
        if (ModbusIdx == 50) ModbusIdx--;                                       // max 50 bytes in buffer
  		Modbus_RX[ModbusIdx++] = RXbyte;                                        // Store received byte in buffer
        ModbusTimer=Timer;
	}
#endif
    
    while(PIR3bits.TX1IF && PIE3bits.TX1IE)                                     // Uart1 transmit interrupt? RS485 @9600 bps
	{
		TX1REG = Modbus_TX[ModbusTXidx++];
        if (ModbusTXidx == ModbusTXlen) PIE3bits.TX1IE = 0;                     // disable interrupts
	}
    
    while(PIR4bits.TMR4IF)                                                      // Timer 4 interrupt, called 1000 times/sec
	{
        Timer++;                                                                // mSec counter (overflows in 1193 hours)
        PLLLock++;
        if (LedTimer) LedTimer--;
		PIR4bits.TMR4IF = 0;                                                    // clear interrupt flag
	}
}

// low level on pin RB0, new Mains period has started
void newPeriod(void)                    
{
  
    if (newsumCycle==0)                                                         // Reset by main loop
    {    
        sumPeriodVsq      += sumVsq;
        sumPeriodVavg     += sumVavg;
        sumPeriodI1sq     += sumI1sq;
        sumPeriodI1avg    += sumI1avg; 
        sumPeriodI2sq     += sumI2sq;
        sumPeriodI2avg    += sumI2avg; 
        sumPeriodI3sq     += sumI3sq;
        sumPeriodI3avg    += sumI3avg; 

        sumPeriodPower1A  += sumPower1A;
        sumPeriodPower1B  += sumPower1B;
        sumPeriodPower2A  += sumPower2A;
        sumPeriodPower2B  += sumPower2B;
        sumPeriodPower3A  += sumPower3A;
        sumPeriodPower3B  += sumPower3B;
        
        sumPeriodSamples += sumSamples;                                         // add the sumsamples from this period (0.02sec) to the 1 second counter
                                                                                // should be around 3000 (60 samples per period * 50 cycles) per second.
                                                                                // if a period ends early, it might be lower, this has no effect on the calculations
        if (++Samples >= SUPPLY_FREQUENCY)                                      // 50 Cycles = 1 sec
        {
            newsumCycle=1;                                                      // flag to main loop to process the data
            Samples=0;
        }
    }
    
    sumVsq = 0;
    sumVavg = 0;
    sumI1sq	= 0;
    sumI1avg = 0;
    sumI2sq	= 0;
    sumI2avg = 0;
    sumI3sq	= 0;
    sumI3avg = 0; 
    
    sumPower1A	= 0;
    sumPower1B	= 0;
    sumPower2A	= 0;
    sumPower2B	= 0;
    sumPower3A	= 0;
    sumPower3B	= 0;
          
    sumSamples=0;
    
}    


double removeRMSOffset(unsigned long sumSquared, long sum, unsigned long numSamples)
{
    double x = ((double)sumSquared / numSamples) - ((double)sum * sum / numSamples / numSamples);
 
    //printf("sumsq: %lu sum: %ld samples %lu \r\n",sumSquared, sum, numSamples );
    
    return (x<0.0 ? 0.0 : sqrt(x));
}

double removePowerOffset(long power, long sumV, long sumI, unsigned long numSamples)
{
    return (((double)power / numSamples) - ((double)sumV * sumI / numSamples / numSamples));
}

double deg_rad(double a)
{
    return (0.01745329*a);
}


void calculateTiming(void)
{
  // Pre-calculate the constants for phase/timing correction
  i1phaseshift = (0 * ADC_RATE * 3.6e-4 * SUPPLY_FREQUENCY - i1Lead); // in degrees
  i2phaseshift = (1 * ADC_RATE * 3.6e-4 * SUPPLY_FREQUENCY - i2Lead);
  i3phaseshift = (2 * ADC_RATE * 3.6e-4 * SUPPLY_FREQUENCY - i3Lead);  
    
  y1 = sin(deg_rad(i1phaseshift)) / sin(deg_rad(SAMPLERATE));
  x1 = cos(deg_rad(i1phaseshift)) - y1 * cos(deg_rad(SAMPLERATE));
  y2 = sin(deg_rad(i2phaseshift)) / sin(deg_rad(SAMPLERATE));
  x2 = cos(deg_rad(i2phaseshift)) - y2 * cos(deg_rad(SAMPLERATE));
  y3 = sin(deg_rad(i3phaseshift)) / sin(deg_rad(SAMPLERATE));
  x3 = cos(deg_rad(i3phaseshift)) - y3 * cos(deg_rad(SAMPLERATE));
  
}



void delay(unsigned int d)
{
	unsigned long x;
	x = Timer;                                      							// read Timer value (increased every ms)
    while (Timer < (x+d)) { }           
}

// Poly used is x^16+x^15+x^2+x
unsigned int CRC16(unsigned int crc, unsigned char *buf, int len)
{
	for (int pos = 0; pos < len; pos++)
	{
		crc ^= (unsigned int)buf[pos];                                          // XOR byte into least sig. byte of crc

		for (int i = 8; i != 0; i--) {                                          // Loop over each bit
			if ((crc & 0x0001) != 0) {                                          // If the LSB is set
				crc >>= 1;                                                      // Shift right and XOR 0xA001
				crc ^= 0xA001;
			}
			else                                                                // Else LSB is not set
				crc >>= 1;                                                      // Just shift right
		}
	}
	return crc;
}

#ifndef ESP_DIRECT
void P1_extract(void)
{
    char *ret;
    double L1_power = 0, L2_power = 0, L3_power = 0;
    double L1_power_return = 0, L2_power_return = 0, L3_power_return = 0;
    double Total_power, Total_power_return;
        
    DSMRver = 0;
    
    ret = strstr((const char *)P1_data,(const far char *)":0.2.8");             // DSMR version
    if (ret != NULL) DSMRver = atoi((const char *)ret+7);
    
    //ret = strstr((const char *)P1_data,(const far char *)":1.0.0");           // Date-time stamp of the P1 message
    
    // Voltage on the phases (Volt*1)
    ret = strstr((const char *)P1_data,(const far char *)":32.7.0");            // Phase 1
    if (ret != NULL) Volts[0] = atof((const char *)ret+8);                      // Some Grid types have 0.0 volts on one of the phases.
    if (Volts[0] < 1.0) Volts[0] = 1;                                           // We divide by Volts later on, so it can not be 0.0
    ret = strstr((const char *)P1_data,(const far char *)":52.7.0");            // Phase 2
    if (ret != NULL) Volts[1] = atof((const char *)ret+8);
    if (Volts[1] < 1.0) Volts[1] = 1;
    ret = strstr((const char *)P1_data,(const far char *)":72.7.0");            // Phase 3
    if (ret != NULL) Volts[2] = atof((const char *)ret+8);
    if (Volts[2] < 1.0) Volts[2] = 1;
    
    // Power received from Grid (Watt*1)
    ret = strstr((const char *)P1_data,(const far char *)":21.7.0");            // Phase 1
    if (ret != NULL) {
        L1_power = atof((const char *)ret+8)*1000;
        if (DSMRver == 0) DSMRver = 50;                                         // Sagemcom T211-D does not send DSMR version
    }                                                                           // but it does send Power and Volts per phase.        
    ret = strstr((const char *)P1_data,(const far char *)":41.7.0");            // Phase 2
    if (ret != NULL) L2_power = atof((const char *)ret+8)*1000;
    ret = strstr((const char *)P1_data,(const far char *)":61.7.0");            // Phase 3
    if (ret != NULL) L3_power = atof((const char *)ret+8)*1000;
    
    // Power delivered to Grid (Watt*1)
    ret = strstr((const char *)P1_data,(const far char *)":22.7.0");            // Phase 1
    if (ret != NULL) L1_power_return = atof((const char *)ret+8)*1000;
    ret = strstr((const char *)P1_data,(const far char *)":42.7.0");            // Phase 2
    if (ret != NULL) L2_power_return = atof((const char *)ret+8)*1000;
    ret = strstr((const char *)P1_data,(const far char *)":62.7.0");            // Phase 3
    if (ret != NULL) L3_power_return = atof((const char *)ret+8)*1000;
    
   // ret = strstr((const char *)P1_data,(const far char *)":1.7.0");           // Total Power from Grid
   // Total_power = atof((const char *)ret+7)*1000;
  //  ret = strstr((const char *)P1_data,(const far char *)":2.7.0");           // Total Power delivered to Grid (Watt)
  //  Total_power_return = atof((const char *)ret+7)*1000;
    
    // Calculate Irms (Amps*1)
    Irms[0] = (L1_power-L1_power_return)/Volts[0];                              // Irms  (Amps *1)
    Irms[1] = (L2_power-L2_power_return)/Volts[1];
    Irms[2] = (L3_power-L3_power_return)/Volts[2];
    
    //printf("L1: %5d V L2: %5d V L3: %5d V   ",(int)(Volts[0]),(int)(Volts[1]),(int)(Volts[2]) );
    //printf("L1: %5d W L2: %5d W L3: %5d W  \r\n",(int)(L1_power-L1_power_return),(int)(L2_power-L2_power_return),(int)(L3_power-L3_power_return) );
    //printf("Total Power %5d W  ",(int)(Total_power-Total_power_return) );

    P1Timer = Timer;                                                            // Counts miliseconds since last reception
}
#endif


//  Send buffer over RS485.
void RS485_Start(void)
{
	RS485_TRANSMIT;                                         					// set RS485 transceiver to transmit
	delay(1);
    PIE3bits.TX1IE = 1;                                                         // Enable interrupts, this will enable transmission
}


// User defined printf support on uart2
void putch(unsigned char byte)      
{
	// output one byte on UART2
	while(!PIR3bits.TX2IF);                                                     // set when register is empty 
	TXREG2 = byte;

}


void init(void)
{
	// Peripheral pin select
    PPSLOCK = 0x55;
    PPSLOCK = 0xAA;
    PPSLOCKbits.PPSLOCKED = 0x00;       // unlock PPS

    RX1PPSbits.RXPPS = 0x17;            //RC7->EUSART1:RX1;
#ifdef ESP_DIRECT
    RB7PPS = 0x09;                      //RB7->EUSART1:TX1;
#else    
    RC4PPS = 0x09;                      //RC4->EUSART1:TX1;
#endif
    
    // Choose Pin RB3 (inverted) or RB5 (non inverted) for P1 signal input
    // RX2PPSbits.RXPPS = 0x0D;         //RB5->EUSART2:RX2;
    RX2PPSbits.RXPPS = 0x0B;            //RB3->EUSART2:RX2;
    RB4PPS = 0x0B;                      //RB4->EUSART2:TX2;
    
    PPSLOCK = 0x55;
    PPSLOCK = 0xAA;
    PPSLOCKbits.PPSLOCKED = 0x01;       // lock PPS
    
    
    OSCCON1 = 0b01100000;
    OSCFRQ = 0b00000110;                // setup 32Mhz internal oscillator (max 64Mhz)
    
	PORTA = 0;                          // Init PortA
 	LATA = 0;
	ANSELA = 0b00000111;                // RA0-RA2 are analog inputs (CT_1, CT_2, CT_3)
    WPUA = 0b00000000;                  // weak pullups disabled
    TRISA = 0b00000111;                 // RA0-RA2 inputs

    PORTB = 0;                          // Init PortB
    LATB = 0;   
    ANSELB = 0;                         // portB digital inputs/outputs
#ifdef ESP_DIRECT    
    TRISB = 0b01101001;                 // RB0,RB3,RB5,RB6 input, others outputs
                                        // TX2 (RB4) is active, make sure it's set to input on ESP32
#else        
    TRISB = 0b00101001;                 // RB0,RB3,RB5 input, others outputs
#endif    
    IOCBN = 0b00000001;                 // Interrupt on negative edge of RB0
    
#ifdef ESP_DIRECT    
    IOCBP = 0b01000000;                 // interrupt on positive edge of RB6
#endif
    
    
	PORTC = 0;
    LATC = 0;
    ANSELC = 0b00000000;                // no analog inputs       
    WPUC = 0b00000000;                  // no weak pull up on RC0
#ifdef ESP_DIRECT
    TRISC = 0b11110000;                 // RC4-7 input, others output
#else    
	TRISC = 0b10000000;                 // RC7 input, others output
#endif    
     
#ifdef BAUD9K6
	SP1BRGH = 0x03;                     // Initialize UART 1
	SP1BRGL = 0x40;                     // Baudrate 9600 
#endif
#ifdef BAUD4K8
	SP1BRGH = 6;                        // Initialize UART 1
	SP1BRGL = 130;                      // Baudrate 4800
#endif	
#ifdef BAUD1K2
	SP1BRGH = 0x1A;                     // Initialize UART 1
	SP1BRGL = 0x0A;                     // Baudrate 1200
#endif
#ifdef ESP_DIRECT
    SP1BRGH = 0x00;                     // Baud Rate = 115200; SP2BRGH 0; 
    SP1BRGL = 0x44;                     // Baud Rate = 115200; SP2BRGL 68; 
#endif	
	BAUD1CON = 0x08;                    // 16 bit Baudrate register is used
	TX1STA = 0x24;                      // Enable TX, 8 bit, Asynchronous mode
	RC1STA = 0x90;                      // Enable serial port TX and RX, 8 bit. 

    // UART 2
    SP2BRGH = 0x00;                     // Baud Rate = 115200; SP2BRGH 0; 
    SP2BRGL = 0x44;                     // Baud Rate = 115200; SP2BRGL 68; 
    BAUD2CON = 0x08;                    // 16 bit Baudrate register is used
    RC2STA = 0x90;                      // Enable serial port TX and RX, 8 bit. 
    TX2STA = 0x24;                      // Enable TX, 8 bit, Asynchronous mode

    //ADC 
    ADCLK = 0x0F;                       // ADCS FOSC/32; 
    ADCON0 = 0x84;                      // ADGO stop; ADFM right; ADON enabled; ADCONT disabled; ADCS FOSC/ADCLK; 
    ADACQ = 0x28;                       // 40uS Acquisition time
    //ADACT = 0x03;                     // ADC auto conversion trigger at TMR1 overflow
	
    //Timer0
	T0CON0 = 0b00010000;                // Setup timer0, no postscaler, 16 bit timer
    T0CON1 = 0b01011000;                // Fosc/4 @ 8Mhz, 1:256 prescaler = 32 uS Period
                                        // Do not disable T0ASYNC bit as it will not update TMR0H !!!! (hardware bug?)
    PIR0bits.TMR0IF = 0;
    T0CON0bits.T0EN = 1;                // Enable Timer0
    
    //Timer1
    T1GCON = 0;
    T1CLK = 0x01;                       // Clock source Fosc/4
    TMR1 = ~(T1PLL);                    // TMR1 counter value. 8Mhz * 3200 = 400uS
                                        // 400us * 50 samples = 0.02s (50 samples per 50Hz period)
    T1CON = 0b00000011;                 // Timer 1, prescale 1:1 , Read/write 16bit, Timer 1 ON
    
    
    //Timer 4
    T4PR = 250;                         // Timer 4 frequency value -> 1Khz @ 8 Mhz
	T4HLT = 0b10100000;                 // Prescaler synced,ON register synced to TMR4_clk
    T4CLKCON = 0b0000001;               // Clock Source Fosc/4 
    T4CON = 0b11010000;                 // Timer 4 ON, prescaler 1:32, Postscaler 1:1
    
    INTCON = 0b10100000;                // High priority interrupts enabled, Low priority interrupts disabled.
                                        // Priority levels enabled
#ifndef ESP_DIRECT
    PIE3bits.RC2IE = 1;                 // Enable receive interrupt for UART2  
    PIE3bits.RC1IE = 1;                 // Enable receive interrupt for UART1  
#endif
    
    PIE4bits.TMR4IE = 1;                // enable high priority Timer4 Interrupt
    PIE4bits.TMR1IE = 1;                // enable high priority Timer1 Interrupt
	PIE1bits.ADIE = 1;                  // enable ADC interrupts
    PIE0bits.IOCIE = 1;                 // enable interrupt on IO pin
}

void main(void)
{
	char *pBytes;
	unsigned char n;
	unsigned int x, cs,crc, P1, len;
    unsigned char LedState, LedCnt;

	init();                             // initialize ports, ADC, UART
    
    LED_GREEN_ON;                       // Green LED on
    LED_RED_ON;                         // Red LED on
    
    calculateTiming();

#ifndef ESP_DIRECT    
    printf("Sensorbox v2 powerup\n\r");
#endif  
    
/*  Calculate voltage sine wave.
 *  Replaced with pre calculated array.
 *  for (n=0;n<NUMSAMPLES*2;n++) {
 *       voltage[n]= (int)(sin(n*(360/NUMSAMPLES)*CONST)*101);
 *       printf("%i,",(int)voltage[n]);
 *   }
 */    	
    delay(500);
    LED_GREEN_OFF;                      // Green LED off
    LED_RED_OFF;                        // Red LED off
   
    ADPCH = 0;                          // Select ADC channel RA0(CT1)
    
	while(1)
	{
        
#ifndef ESP_DIRECT          
        while (!LedTimer && LedState)   // start with state 6
        {
            if ((LedState-- %2) ==0)
            {   
                if (!(LedSeq[LedCnt] & 2)) LED_GREEN_ON;                        // LedSeq[] 0 = GREEN, 1 = ORANGE, 2 = RED
                if (LedSeq[LedCnt++]) LED_RED_ON;
                LedTimer = 200;
            } else {
                LED_GREEN_OFF;
                LED_RED_OFF;
                LedTimer = 200;    
            }
        }

#endif
        
        if (PLLLock >= 75) {                                                    // Check if we have a lock on the 50 Hz mains input.
            PLLLock = 75;                                                       // No Lock (Mains not connected)
                
            // set T1PLL to fixed value. 
            T1PLL = 2500;                                                       // 312uS
            NoLock = 1;                                                         // indicates the MAINS connection is not connected,
                                                                                // switch to alternative Irms calculations
            if (TotalSumReady) {
   
#ifdef ESP_DIRECT
                sprintf(Modbus_TX,"/1R:%ld 2R:%ld 3R:%ld SA:%u!", TotalSumI[0], TotalSumI[1], TotalSumI[2], SAMPLES);
#else

                for (x=0 ;x<3; x++) {
                    IrmsCT[x] = sqrt((double)TotalSumI[x]/SAMPLES) * 0.3 ;
                }
                LedSeq[0] = 1;                                                  // Led Orange
#endif                
                dataready |= 3;                                                 // Mark as processed
                TotalSumReady = 0;
            }           
        }
        
        // sumPeriodxxx variables are updated by the ISR, and when ready the newsumCycle is set.
        // do not use the sumPeriodxxx variables anywhere else, as the data might be updated at any moment.
        if (newsumCycle)
        {
#ifndef ESP_DIRECT            
            Vrms = removeRMSOffset(sumPeriodVsq, sumPeriodVavg, sumPeriodSamples);  // We don't really measure the Voltage, we just use a sine wave lookuptable.
            
            //printf("sumPeriodVavg: %ld \r\n",sumPeriodVavg);
            
                                                                                // Calculate RMS Current for all Phases    
            IrmsCT[0] = ICAL * removeRMSOffset(sumPeriodI1sq, sumPeriodI1avg, sumPeriodSamples);
            IrmsCT[1] = ICAL * removeRMSOffset(sumPeriodI2sq, sumPeriodI2avg, sumPeriodSamples);
            IrmsCT[2] = ICAL * removeRMSOffset(sumPeriodI3sq, sumPeriodI3avg, sumPeriodSamples);

                                                                                // Calculate Real Power for all Phases
                                                                                // As the Vrms is based on a sine wave, and uncalibrated, the realPower
                                                                                // value is only used to calculate the real current (including phase shift).
            realPower[0] = ICAL * (x1 * removePowerOffset(sumPeriodPower1A, sumPeriodVavg, sumPeriodI1avg, sumPeriodSamples) 
                                  + y1 * removePowerOffset(sumPeriodPower1B, sumPeriodVavg, sumPeriodI1avg, sumPeriodSamples));
  
            realPower[1] = ICAL * (x2 * removePowerOffset(sumPeriodPower2A, sumPeriodVavg, sumPeriodI2avg, sumPeriodSamples)
                                   + y2 * removePowerOffset(sumPeriodPower2B, sumPeriodVavg, sumPeriodI2avg, sumPeriodSamples));
  
            realPower[2] = ICAL * (x3 * removePowerOffset(sumPeriodPower3A, sumPeriodVavg, sumPeriodI3avg, sumPeriodSamples)
                                   + y3 * removePowerOffset(sumPeriodPower3B, sumPeriodVavg, sumPeriodI3avg, sumPeriodSamples));

                                                                                // calculate apparent Power for all Phases
                                                                                // the Vrms is uncalibrated, this is only used to calculate the Power factor
            apparentPower1 = IrmsCT[CT1] * Vrms;                                 
            if (apparentPower1 > 0.1) powerFactor1 = realPower[CT1] / apparentPower1;
            else powerFactor1 = 0.0;
            
            apparentPower2 = IrmsCT[CT2] * Vrms;
            if (apparentPower2 > 0.1) powerFactor2 = realPower[CT2] / apparentPower2;
            else powerFactor2 = 0.0;
            
            apparentPower3 = IrmsCT[CT3] * Vrms;
            if (apparentPower3 > 0.1) powerFactor3 = realPower[CT3] / apparentPower3;
            else powerFactor3 = 0.0;
            
            IrmsCT[0]= realPower[CT1] / Vrms;
            IrmsCT[1]= realPower[CT2] / Vrms;
            IrmsCT[2]= realPower[CT3] / Vrms;
            
            if (IrmsCT[0] < -0.1 ) LedSeq[0]=0; else LedSeq[0]=1;
            if (IrmsCT[1] < -0.1 ) LedSeq[1]=0; else LedSeq[1]=1;
            if (IrmsCT[2] < -0.1 ) LedSeq[2]=0; else LedSeq[2]=1;

#else
            // We send the raw measurements to the ESP, for faster processing and more flexibility.
            //
            sprintf(Modbus_TX,"/1A:%ld 1B:%ld 2A:%ld 2B:%ld 3A:%ld 3B:%ld SA:%u WI:%u!", sumPeriodPower1A, sumPeriodPower1B, 
                    sumPeriodPower2A, sumPeriodPower2B, sumPeriodPower3A, sumPeriodPower3B, sumPeriodSamples, version & 3u);
#endif
    /*         
            printf("SumPeriodSamples %u T1PLL %u \r\n", sumPeriodSamples, T1PLL );
            printf("\r\nCT1: "); for (n=1;n<45;n++) printf("%i ",debugI1[n]); 
            printf("\r\nCT2: "); for (n=1;n<45;n++) printf("%i ",debugI2[n]);
            printf("\r\nCT3: "); for (n=1;n<45;n++) printf("%i ",debugI3[n]);
            printf(" sumPeriodI3dir %ld Iavg %ld\r\n", sumPeriodI3dir, sumPeriodI3avg/sumPeriodSamples);
    */        
            
            sumPeriodVsq = 0;
            sumPeriodVavg = 0;
            sumPeriodI1sq = 0;
            sumPeriodI1avg = 0; 
            sumPeriodI2sq = 0;
            sumPeriodI2avg = 0; 
            sumPeriodI3sq = 0;
            sumPeriodI3avg = 0; 
            
            sumPeriodPower1A  = 0;
            sumPeriodPower1B  = 0;
            sumPeriodPower2A  = 0;
            sumPeriodPower2B  = 0;
            sumPeriodPower3A  = 0;
            sumPeriodPower3B  = 0;
            sumPeriodSamples = 0;
            
            // here we set the phase shifts for the next cycles
            switch (version & 3) {
                case 0:                                                         // 4 Wires, CW rotation
                    Phase1 = PHASE0;
                    Phase2 = PHASE120;
                    Phase3 = PHASE240;
                    break;
                case 1:                                                         // 4 Wires, CCW rotation
                    Phase1 = PHASE0;
                    Phase2 = PHASE240;
                    Phase3 = PHASE120; 
                    break;
                case 2:                                                         // 3 Wires, CW rotation
                    Phase1 = PHASE30;
                    Phase2 = PHASE150;
                    Phase3 = PHASE270;
                    break;
                case 3:                                                         // 3 Wires, CCW rotation
                    Phase1 = PHASE30;
                    Phase2 = PHASE270;
                    Phase3 = PHASE150;
                    break;
               default:
                    break;
            }
            
            dataready |= 3;                                                     // we have processed all 3 CT inputs. 
            newsumCycle=0;                                                      // flag ready for next cycle
        }
        
#ifndef ESP_DIRECT      
        //
        // Check P1 port for new telegram
        //
        if (P1_EOT > 4) {                                                       // a new telegram has been received from smart meter

            cs = CRC16(0,P1_data,P1_length);// calculate CRC16 from data
            crc = (unsigned int) strtol((const char *)(P1_data+P1_length), NULL, 16); // get crc from data, convert to int

            //printf("length: %u, CRC16: %04x crc16from file: %04x\n\r",P1_length, cs, crc);
            //for (x=0;x<P1_length;x++) printf("%c",P1_data[x]);
   
            if (crc == cs) {                                                    // check if crc's match
                P1_extract();                                                   // Extract Voltage and Current values from Smart Meter telegram
                if (DSMRver >= 50) dataready |= 0x80;                           // P1 dataready
                else dataready |= 0x40;                                         // DSMR version not 5.0 !!   

                P1_EOT = 0;                                                     // Mark as processed
            } 
        }    
#endif

        
        if (RC1STAbits.OERR)			// Uart1 Overrun Error?
        {
            RC1STAbits.CREN = 0;
            RC1STAbits.CREN = 1;		// Restart Uart
        }
        
        if (RC2STAbits.OERR)			// Uart2 Overrun Error?
        {
            RC2STAbits.CREN = 0;
            RC2STAbits.CREN = 1;		// Restart Uart
        }
        
#ifndef ESP_DIRECT
        if (ModbusTXlen && TX1STAbits.TRMT && PIE3bits.TX1IE==0)                // transmit register empty and interrupt disabled ?
        {
            RS485_RECEIVE;                                          			// set RS485 transceiver to receive
                                
            ModbusTXlen = 0;            // reset length
        }
#endif
        
        // Transmit measurement data to master / host
        //
#ifdef ESP_DIRECT
        if (dataready) {
            // In direct mode we send the data over a serial line
            len = strlen(Modbus_TX);
            cs = CRC16(0, Modbus_TX, len);                                      // calculate CRC16 from data
            sprintf(Modbus_TX+len,"%04x\r\n", cs);                              // add crc to the end of the string

            ModbusTXlen = len+6;                                                // length of data to send
            ModbusTXidx = 0;                                                    // points to first character to send.

            RS485_Start();                                                      // send buffer to RS485 port, using interrupts
            dataready = 0;
        }
#else   
        // We use modbus to send the data
        //        
        if ( (ModbusIdx > 6) && (Timer>(ModbusTimer+3)) )                       // last reception more then 3ms ago? 
        {
            crc = CRC16(0xffff, Modbus_RX, ModbusIdx);                          // calculate checksum over all data (including crc16)
        
            // Write to Holding registers and set Wire and rotation settings (modbus register address 0x800)
            if (Modbus_RX[0]==0x0a && Modbus_RX[1]==0x06 && Modbus_RX[2]==0x08 && Modbus_RX[3]==0x00 && Modbus_RX[4]==0x00 && !crc)
            {
                version = 0x14 + (Modbus_RX[5] & 3u);                           // Set 3 or 4 Wire, and phase Rotation.
                for (n=0; n<8; n++) Modbus_TX[n] = Modbus_RX[n];                // echo back the Write request.
                ModbusTXlen = n;                                                // length of modbus packet
                ModbusTXidx = 0;                                                // points to first character to send.
                RS485_Start();                                                  // send buffer to RS485 port, using interrupts
            }
            
            else if (Modbus_RX[0]==0x0a && Modbus_RX[1]==0x04 && Modbus_RX[5]<=0x14 && !crc && dataready)   // check CRC
            {
                //we have received a valid request for sensorbox data
                                                            // Setup Modbus data
                Modbus_TX[0]= 0x0a;             			// Fixed Address 0x0A
                Modbus_TX[1]= 0x04;                         // function byte
                Modbus_TX[2]= Modbus_RX[5] * 2;             // takes the bytes from the request. 28h bytes will follow
                Modbus_TX[3]= 0x00;                         // 
                Modbus_TX[4]= version;                      // Sensorbox2 (new version) 0x14 (old was 0x00)
                Modbus_TX[5]= DSMRver;                      // DSMR version
                Modbus_TX[6]= dataready;                    // 0x80 = P1, 0x03= 3CT's ,0x83 = P1+ 3CT
                
                n=7;
                for (x=0; x<3 ;x++)
                {	
                    pBytes = (char*)&Volts[x];              // get raw 4 byte Double 
                    Modbus_TX[n++] = pBytes[3];
                    Modbus_TX[n++] = pBytes[2];
                    Modbus_TX[n++] = pBytes[1];
                    Modbus_TX[n++] = pBytes[0];
                }
                for (x=0; x<3 ;x++)
                {	
                    pBytes = (char*)&Irms[x];               // get raw 4 byte Double 
                    Modbus_TX[n++] = pBytes[3];
                    Modbus_TX[n++] = pBytes[2];
                    Modbus_TX[n++] = pBytes[1];
                    Modbus_TX[n++] = pBytes[0];
                }
                for (x=0; x<3 ;x++)
                {	
                    pBytes = (char*)&IrmsCT[x];             // get raw 4 byte Double 
                    Modbus_TX[n++] = pBytes[3];
                    Modbus_TX[n++] = pBytes[2];
                    Modbus_TX[n++] = pBytes[1];
                    Modbus_TX[n++] = pBytes[0];
                }
                cs = CRC16(0xffff, Modbus_TX, n);		// calculate CRC16 from data			
                Modbus_TX[n++] = ((unsigned char)(cs));
                Modbus_TX[n++] = ((unsigned char)(cs>>8));	

                ModbusTXlen = n;                // length of modbus packet
                ModbusTXidx = 0;                // points to first character to send.
                
                RS485_Start();                                                  // send buffer to RS485 port, using interrupts

                LedCnt = 0;                                                     // reset to first part of sequence

                if (dataready > 3) {                                            // P1 data?        
                    LedState = 2;                                               // Blink only once
                    if (DSMRver < 50) LedSeq[0] = 2;                            // DSMR version not 5.x !! Blink Red
                    else LedSeq[0] = 0;                                         // DSMR version OK, Blink Green
                } else if (NoLock) {
                    LedState = 2;                                               // Blink only once
                } else {
                    LedState = 6;                                               // 6 States, ON/OFF (3 CT's)
                } 
                                              
                printf("CT1: %2.1f A CT2: %2.1f A CT3: %2.1f A | L1: %2.1f A  L2: %2.1f A  L3: %2.1f A Sum: %3.1f A \r\n",
                IrmsCT[0], IrmsCT[1], IrmsCT[2],        
                Irms[0], Irms[1], Irms[2], Irms[0]+Irms[1]+Irms[2] );
                printf("PF1: %1.2f   PF2: %1.2f   PF3: %1.2f\r\n", powerFactor1, powerFactor2, powerFactor3 );
                
                DSMRver = 0;
                dataready = 0;
            }
            
            ModbusIdx=0;
        }

#endif 
 
	} // while(1)
 

}


