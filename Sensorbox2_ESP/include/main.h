

#define SENSORBOX_VERSION 20                                                    // 2 LSB should be 0, as these are WIRE settings
#define SENSORBOX_SWVER 1                                                       // 0 = Original software
                                                                                // 1 = Supports extra modbus registers (Time, IP, Hostname etc)       

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


// Settings
#define WIFI_MODE 0                                                             // WiFi disabled
#define AP_PASSWORD "00000000"                                                  // Will be overwritten by random 8 char password.

// Toggle pins of the RS485 transceiver.
void ToggleRS485(bool level);

#define FREE(x) free(x); x = NULL;

#if DBG == 0
//used to steer RemoteDebug
#define DEBUG_DISABLED 1
#define _LOG_W( ... ) //dummy
#define _LOG_I( ... ) //dummy
#define _LOG_D( ... ) //dummy
#define _LOG_V( ... ) //dummy
#define _LOG_A( ... ) //dummy
#define _LOG_W_NO_FUNC( ... ) //dummy
#define _LOG_I_NO_FUNC( ... ) //dummy
#define _LOG_D_NO_FUNC( ... ) //dummy
#define _LOG_V_NO_FUNC( ... ) //dummy
#define _LOG_A_NO_FUNC( ... ) //dummy
#endif

#if DBG == 1
#define _LOG_A(fmt, ...) if (Debug.isActive(Debug.ANY))                Debug.printf("(%s)(C%d) " fmt, __func__, xPortGetCoreID(), ##__VA_ARGS__) //Always = Errors!!!
#define _LOG_P(fmt, ...) if (Debug.isActive(Debug.PROFILER))   Debug.printf("(%s)(C%d) " fmt, __func__, xPortGetCoreID(), ##__VA_ARGS__)
#define _LOG_V(fmt, ...) if (Debug.isActive(Debug.VERBOSE))    Debug.printf("(%s)(C%d) " fmt, __func__, xPortGetCoreID(), ##__VA_ARGS__)         //Verbose
#define _LOG_D(fmt, ...) if (Debug.isActive(Debug.DEBUG))              Debug.printf("(%s)(C%d) " fmt, __func__, xPortGetCoreID(), ##__VA_ARGS__) //Debug
#define _LOG_I(fmt, ...) if (Debug.isActive(Debug.INFO))               Debug.printf("(%s)(C%d) " fmt, __func__, xPortGetCoreID(), ##__VA_ARGS__) //Info
#define _LOG_W(fmt, ...) if (Debug.isActive(Debug.WARNING))    Debug.printf("(%s)(C%d) " fmt, __func__, xPortGetCoreID(), ##__VA_ARGS__)         //Warning
#define _LOG_E(fmt, ...) if (Debug.isActive(Debug.ERROR))              Debug.printf("(%s)(C%d) " fmt, __func__, xPortGetCoreID(), ##__VA_ARGS__) //Error not used!
#define _LOG_A_NO_FUNC(fmt, ...) if (Debug.isActive(Debug.ANY))                Debug.printf(fmt, ##__VA_ARGS__)
#define _LOG_P_NO_FUNC(fmt, ...) if (Debug.isActive(Debug.PROFILER))   Debug.printf(fmt, ##__VA_ARGS__)
#define _LOG_V_NO_FUNC(fmt, ...) if (Debug.isActive(Debug.VERBOSE))    Debug.printf(fmt, ##__VA_ARGS__)
#define _LOG_D_NO_FUNC(fmt, ...) if (Debug.isActive(Debug.DEBUG))              Debug.printf(fmt, ##__VA_ARGS__)
#define _LOG_I_NO_FUNC(fmt, ...) if (Debug.isActive(Debug.INFO))               Debug.printf(fmt, ##__VA_ARGS__)
#define _LOG_W_NO_FUNC(fmt, ...) if (Debug.isActive(Debug.WARNING))    Debug.printf(fmt, ##__VA_ARGS__)
#define _LOG_E_NO_FUNC(fmt, ...) if (Debug.isActive(Debug.ERROR))              Debug.printf(fmt, ##__VA_ARGS__)
#include "RemoteDebug.h"  //https://github.com/JoaoLopesF/RemoteDebug
extern RemoteDebug Debug;
#endif

#define EVSE_LOG_FORMAT(letter, format) "[%6u][" #letter "][%s:%u] %s(): " format , (uint32_t) (esp_timer_get_time() / 1000ULL), pathToFileName(__FILE__), __LINE__, __FUNCTION__

#if DBG == 2
#define DEBUG_DISABLED 1
#if LOG_LEVEL >= 1  // Errors
#define _LOG_A(fmt, ... ) Serial.printf(EVSE_LOG_FORMAT(E, fmt), ##__VA_ARGS__)
#define _LOG_A_NO_FUNC( ... ) Serial.printf ( __VA_ARGS__ )
#else
#define _LOG_A( ... )
#define _LOG_A_NO_FUNC( ... )
#endif
#if LOG_LEVEL >= 2  // Warnings
#define _LOG_W(fmt, ... ) Serial.printf(EVSE_LOG_FORMAT(W, fmt), ##__VA_ARGS__)
#define _LOG_W_NO_FUNC( ... ) Serial.printf ( __VA_ARGS__ )
#else
#define _LOG_W( ... )
#define _LOG_W_NO_FUNC( ... )
#endif
#if LOG_LEVEL >= 3  // Info
#define _LOG_I(fmt, ... ) Serial.printf(EVSE_LOG_FORMAT(I, fmt), ##__VA_ARGS__)
#define _LOG_I_NO_FUNC( ... ) Serial.printf ( __VA_ARGS__ )
#else
#define _LOG_I( ... )
#define _LOG_I_NO_FUNC( ... )
#endif
#if LOG_LEVEL >= 4  // Debug
#define _LOG_D(fmt, ... ) Serial.printf(EVSE_LOG_FORMAT(D, fmt), ##__VA_ARGS__)
#define _LOG_D_NO_FUNC( ... ) Serial.printf ( __VA_ARGS__ )
#else
#define _LOG_D( ... )
#define _LOG_D_NO_FUNC( ... )
#endif
#if LOG_LEVEL >= 5  // Verbose
#define _LOG_V(fmt, ... ) Serial.printf(EVSE_LOG_FORMAT(V, fmt), ##__VA_ARGS__)
#define _LOG_V_NO_FUNC( ... ) Serial.printf ( __VA_ARGS__ )
#else
#define _LOG_V( ... )
#define _LOG_V_NO_FUNC( ... )
#endif
#endif  // if DBG == 2
