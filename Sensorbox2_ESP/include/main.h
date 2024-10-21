
#define SENSORBOX_VERSION 20                                                    // 2 LSB should be 0, as these are WIRE settings
#define SENSORBOX_SWVER 2                                                       // 0 = Original software
                                                                                // 1 = Supports extra modbus registers (Time, IP, Hostname etc)       
                                                                                // 2 = Software version 2.1.0
#define WDT_TIMEOUT 3                                                           // 3 seconds WDT timeout

#define PIN_LED_GREEN 32
#define PIN_LED_RED   33    													// LED outputs
#define PIN_PGD 23			    												// PIC connection to PGD, serial measurements input
#define PIN_PGC 22				    											// PIC connection to PGC, sets PIC wire mode (output)

#define PIN_RXD 3   															// connects to TTL-USB converter
#define PIN_TXD 1																// connects to TTL-USB converter
#define PIN_RS485_RX 19 														// 
#define PIN_RS485_TX 16															//  > connects to RS485 tranceiver
#define PIN_RS485_DE 17	    													// The DE and RE pins of the transceiver are both connected.
#define PIN_RS485_RE 18															//
#define PIN_RX 27																// P1 port input
#define PIN_TX 14																// serial debug data from PIC (unused, also available on P1 port)

#define CAL 151                                                                 // CT calibration value (normal mode)
#define CALRMS 0.3                                                              // CT calibration value (RMS mode)
#define CW 0                                                                    // Rotation Right (CW)
#define CCW 1                                                                   // Rotation Left (CCW)
#define WIRES4 0                                                                //
#define WIRES3 2                                                                //

#define LED_OFF 0
#define LED_RED 1
#define LED_GREEN 2
#define LED_ORANGE 3

// Settings
#define WIFI_MODE 0                                                             // WiFi disabled
#define AP_PASSWORD "00000000"                                                  // Will be overwritten by random 8 char password.

#include "debug.h"

// Toggle pins of the RS485 transceiver.
void ToggleRS485(bool level);

