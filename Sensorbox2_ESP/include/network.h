/*
;    Project: Smart EVSE v3
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

#ifndef __EVSE_NETWORK

#define __EVSE_NETWORK

#include "mongoose.h"
#include <ArduinoJson.h>

#ifndef VERSION
//please note that this version will only be displayed with the correct time/date if the program is recompiled
//so the webserver will show correct version if evse.cpp is recompiled
//the lcd display will show correct version if glcd.cpp is recompiled
#define VERSION (__TIME__ " @" __DATE__)
#endif

#ifndef MQTT
#define MQTT 1  // Uncomment or set to 0 to disable MQTT support in code
#endif

#if MQTT
// MQTT connection info
extern String MQTTuser;
extern String MQTTpassword;
extern String MQTTprefix;
extern String MQTTHost;
extern uint16_t MQTTPort;
//mg_timer *MQTTtimer;
extern uint8_t lastMqttUpdate;
extern bool shouldReboot;

class MQTTclient_t {
private:
    struct mg_mqtt_opts default_opts;
public:
    //constructor
    MQTTclient_t () {
        memset(&default_opts, 0, sizeof(default_opts));
        default_opts.qos = 0;
        default_opts.retain = false;
    }
    void publish(const String &topic, const int32_t &payload, bool retained, int qos) { publish(topic, String(payload), retained, qos); };
    void publish(const String &topic, const String &payload, bool retained, int qos);
    void subscribe(const String &topic, int qos);
    bool connected;
    struct mg_connection *s_conn;
    void disconnect(void) { mg_mqtt_disconnect(s_conn, &default_opts); };
};

extern MQTTclient_t MQTTclient;

#endif

#ifndef ENABLE_OCPP
#define ENABLE_OCPP 0
#endif

#if ENABLE_OCPP
#include <MicroOcpp/Model/ConnectorBase/Notification.h>
#endif

extern void WiFiSetup(void);

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

#define FW_DOWNLOAD_PATH "http://smartevse-3.s3.eu-west-2.amazonaws.com"

#define OWNER_FACT "SmartEVSE"
#define REPO_FACT "SmartEVSE-3"
#define OWNER_COMM "dingo35"
#define REPO_COMM "SmartEVSE-3.5"

#define WIFI_MODE 0

#endif