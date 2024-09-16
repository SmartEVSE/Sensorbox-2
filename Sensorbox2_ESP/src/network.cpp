#include "network.h"
#include "main.h"
#include <WiFi.h>
//#include "esp_ota_ops.h"
#include "mbedtls/md_internal.h"
#include "radix.h"

#include <HTTPClient.h>
#include <ESPmDNS.h>
#include <Update.h>

//#include <Logging.h>
//#include <ModbusServerRTU.h>        // Slave/node
//#include <ModbusClientRTU.h>        // Master
//#include <time.h>

//#include "evse.h"
//#include "glcd.h"
//#include "utils.h"
//#include "OneWire.h"
//#include "modbus.h"
//#include "meter.h"

#ifndef DEBUG_DISABLED
RemoteDebug Debug;
#endif

#define SNTP_GET_SERVERS_FROM_DHCP 1
#include <esp_sntp.h>

struct tm timeinfo;
bool LocalTimeSet = false;

//mongoose stuff
#include "mongoose.h"
#include "esp_log.h"
struct mg_mgr mgr;  // Mongoose event manager. Holds all connections
// end of mongoose stuff

//OCPP includes
#if ENABLE_OCPP
#include <MicroOcpp.h>
#include <MicroOcppMongooseClient.h>
#include <MicroOcpp/Core/Configuration.h>
#include <MicroOcpp/Core/Context.h>
#endif //ENABLE_OCPP

String APhostname = "SmartEVSE-" + String( MacId() & 0xffff, 10);           // SmartEVSE access point Name = SmartEVSE-xxxxx

#if MQTT
// MQTT connection info
String MQTTuser;
String MQTTpassword;
String MQTTprefix;
String MQTTHost = "";
uint16_t MQTTPort;
mg_timer *MQTTtimer;
uint8_t lastMqttUpdate = 0;
#endif

// SSID and PW for your Router
String Router_SSID;
String Router_Pass;

mg_connection *HttpListener80, *HttpListener443;

bool shouldReboot = false;

extern void write_settings(void);
extern void StopwebServer(void); //TODO or move over to network.cpp?
extern void StartwebServer(void); //TODO or move over to network.cpp?

extern uint32_t serialnr;
// Global data


// The following data will be updated by eeprom/storage data at powerup:
uint8_t WIFImode = WIFI_MODE;                                               // WiFi Mode (0:Disabled / 1:Enabled / 2:Start Portal)
char SmartConfigKey[] = "0123456789abcdef";                                 // SmartConfig / EspTouch AES key, used to encyrypt the WiFi password.
String TZinfo = "";                                                         // contains POSIX time string

char *downloadUrl = NULL;
int downloadProgress = 0;
int downloadSize = 0;
//#define FW_UPDATE_DELAY 30        //DINGO TODO                                            // time between detection of new version and actual update in seconds
#define FW_UPDATE_DELAY 3600                                                    // time between detection of new version and actual update in seconds
uint16_t firmwareUpdateTimer = 0;                                               // timer for firmware updates in seconds, max 0xffff = approx 18 hours
                                                                                // 0 means timer inactive
                                                                                // 0 < timer < FW_UPDATE_DELAY means we are in countdown for an actual update
                                                                                // FW_UPDATE_DELAY <= timer <= 0xffff means we are in countdown for checking
                                                                                //                                              whether an update is necessary

#if ENABLE_OCPP
uint8_t OcppMode = OCPP_MODE; //OCPP Client mode. 0:Disable / 1:Enable

unsigned char OcppRfidUuid [7];
size_t OcppRfidUuidLen;
unsigned long OcppLastRfidUpdate;
unsigned long OcppTrackLastRfidUpdate;

bool OcppForcesLock = false;
std::shared_ptr<MicroOcpp::Configuration> OcppUnlockConnectorOnEVSideDisconnect; // OCPP Config for RFID-based transactions: if false, demand same RFID card again to unlock connector
std::shared_ptr<MicroOcpp::Transaction> OcppLockingTx; // Transaction which locks connector until same RFID card is presented again

bool OcppTrackPermitsCharge = false;
bool OcppTrackAccessBit = false;
uint8_t OcppTrackCPvoltage = PILOT_NOK; //track positive part of CP signal for OCPP transaction logic
MicroOcpp::MOcppMongooseClient *OcppWsClient;

float OcppCurrentLimit = -1.f; // Negative value: no OCPP limit defined

unsigned long OcppStopReadingSyncTime; // Stop value synchronization: delay StopTransaction by a few seconds so it reports an accurate energy reading

bool OcppDefinedTxNotification;
MicroOcpp::TxNotification OcppTrackTxNotification;
unsigned long OcppLastTxNotification;
#endif //ENABLE_OCPP


/**
 * Get name of a state
 *
 * @param uint8_t State
 * @return uint8_t[] Name
 *//*
const char * getStateName(uint8_t StateCode) {
    if(StateCode < 15) return StrStateName[StateCode];
    else return "NOSTATE";
}
*/
/*
const char * getStateNameWeb(uint8_t StateCode) {
    if(StateCode < 15) return StrStateNameWeb[StateCode];
    else return "NOSTATE";    
}


uint8_t getErrorId(uint8_t ErrorCode) {
    uint8_t count = 0;
    //find the error bit that is set
    while (ErrorCode) {
        count++;
        ErrorCode = ErrorCode >> 1;
    }    
    return count;
}


const char * getErrorNameWeb(uint8_t ErrorCode) {
    uint8_t count = 0;
    count = getErrorId(ErrorCode);
    if(count < 9) return StrErrorNameWeb[count];
    else return "Multiple Errors";
}
*/

bool isValidInput(String input) {
  // Check if the input contains only alphanumeric characters, underscores, and hyphens
  for (char c : input) {
    if (!isalnum(c) && c != '_' && c != '-') {
      return false;
    }
  }
  return true;
}


static uint8_t CliState = 0;
void ProvisionCli() {

    static char CliBuffer[64];
    static uint8_t idx = 0;
    static bool entered = false;
    char ch;

    if (CliState == 0) {
        Serial.println("Enter WiFi access point name:");
        CliState++;

    } else if (CliState == 1 && entered) {
        Router_SSID = String(CliBuffer);
        Router_SSID.trim();
        if (!isValidInput(Router_SSID)) {
            Serial.println("Invalid characters in SSID.");
            Router_SSID = "";
            CliState = 0;
        } else CliState++;              // All OK, now request password.
        idx = 0;
        entered = false;

    } else if (CliState == 2) {
        Serial.println("Enter WiFi password:");
        CliState++;

    } else if (CliState == 3 && entered) {
        Router_Pass = String(CliBuffer);
        Router_Pass.trim();
        if (idx < 8) {
            Serial.println("Password should be min 8 characters.");
            Router_Pass = "";
            CliState = 2;
        } else CliState++;             // All OK
        idx = 0;
        entered = false;

    } else if (CliState == 4) {
        Serial.println("WiFi credentials stored.");
        CliState++;

    } else if (CliState == 5) {

        //WiFi.stopSmartConfig();             // Stop SmartConfig //TODO necessary?
        WiFi.mode(WIFI_STA);                // Set Station Mode
        WiFi.begin(Router_SSID, Router_Pass);   // Configure Wifi with credentials
        CliState++;
    }


    // read input, and store in buffer until we read a \n
    while (Serial.available()) {
        ch = Serial.read();

        // When entering a password, replace last character with a *
        if (CliState == 3 && idx) Serial.printf("\b*");
        Serial.print(ch);

        // check for CR/LF, and make sure the contents of the buffer is atleast 1 character
        if (ch == '\n' || ch == '\r') {
            if (idx) {
                CliBuffer[idx] = 0;         // null terminate
                entered = true;
            } else if (CliState == 1 || CliState == 3) CliState--; // Reprint the last message
        } else if (idx < 63) {              // Store in buffer
            if (ch == '\b' && idx) {
                idx--;
                Serial.print(" \b");        // erase character from terminal
            } else {
                CliBuffer[idx++] = ch;
            }
        }
    }
}


#if MQTT
void mqtt_receive_callback(const String topic, const String payload) {
    if (topic == MQTTprefix + "/Set/Mode") {
        if (payload == "Off") {
            ToModemWaitStateTimer = 0;
            ToModemDoneStateTimer = 0;
            LeaveModemDoneStateTimer = 0;
            setAccess(0);
        } else if (payload == "Normal") {
            setMode(MODE_NORMAL);
        } else if (payload == "Solar") {
            OverrideCurrent = 0;
            setMode(MODE_SOLAR);
        } else if (payload == "Smart") {
            OverrideCurrent = 0;
            setMode(MODE_SMART);
        }
    } else if (topic == MQTTprefix + "/Set/CurrentOverride") {
        uint16_t RequestedCurrent = payload.toInt();
        if (RequestedCurrent == 0) {
            OverrideCurrent = 0;
        } else if (LoadBl < 2 && (Mode == MODE_NORMAL || Mode == MODE_SMART)) { // OverrideCurrent not possible on Slave
            if (RequestedCurrent >= (MinCurrent * 10) && RequestedCurrent <= (MaxCurrent * 10)) {
                OverrideCurrent = RequestedCurrent;
            }
        }
    } else if (topic == MQTTprefix + "/Set/CurrentMaxSumMains" && LoadBl < 2) {
        uint16_t RequestedCurrent = payload.toInt();
        if (RequestedCurrent == 0) {
            MaxSumMains = 0;
        } else if (RequestedCurrent == 0 || (RequestedCurrent >= 10 && RequestedCurrent <= 600)) {
                MaxSumMains = RequestedCurrent;
        }
    } else if (topic == MQTTprefix + "/Set/CPPWMOverride") {
        int pwm = payload.toInt();
        if (pwm == -1) {
            SetCPDuty(1024);
            CP_ON;
            CPDutyOverride = false;
        } else if (pwm == 0) {
            SetCPDuty(0);
            CP_OFF;
            CPDutyOverride = true;
        } else if (pwm <= 1024) {
            SetCPDuty(pwm);
            CP_ON;
            CPDutyOverride = true;
        }
    } else if (topic == MQTTprefix + "/Set/MainsMeter") {
        if (MainsMeter.Type != EM_API || LoadBl >= 2)
            return;

        int32_t L1, L2, L3;
        int n = sscanf(payload.c_str(), "%d:%d:%d", &L1, &L2, &L3);

        // MainsMeter can measure -200A to +200A per phase
        if (n == 3 && (L1 > -2000 && L1 < 2000) && (L2 > -2000 && L2 < 2000) && (L3 > -2000 && L3 < 2000)) {
            if (LoadBl < 2)
                MainsMeter.Timeout = COMM_TIMEOUT;
            MainsMeter.Irms[0] = L1;
            MainsMeter.Irms[1] = L2;
            MainsMeter.Irms[2] = L3;
            CalcIsum();
        }
    } else if (topic == MQTTprefix + "/Set/EVMeter") {
        if (EVMeter.Type != EM_API)
            return;

        int32_t L1, L2, L3, W, WH;
        int n = sscanf(payload.c_str(), "%d:%d:%d:%d:%d", &L1, &L2, &L3, &W, &WH);

        // We expect 5 values (and accept -1 for unknown values)
        if (n == 5) {
            if ((L1 > -1 && L1 < 1000) && (L2 > -1 && L2 < 1000) && (L3 > -1 && L3 < 1000)) {
                // RMS currents
                EVMeter.Irms[0] = L1;
                EVMeter.Irms[1] = L2;
                EVMeter.Irms[2] = L3;
                EVMeter.CalcImeasured();
                EVMeter.Timeout = COMM_EVTIMEOUT;
            }

            if (W > -1) {
                // Power measurement
                EVMeter.PowerMeasured = W;
            }

            if (WH > -1) {
                // Energy measurement
                EVMeter.Import_active_energy = WH;
                EVMeter.Export_active_energy = 0;
                EVMeter.UpdateEnergies();
            }
        }
    } else if (topic == MQTTprefix + "/Set/HomeBatteryCurrent") {
        if (LoadBl >= 2)
            return;
        homeBatteryCurrent = payload.toInt();
        homeBatteryLastUpdate = time(NULL);
    } else if (topic == MQTTprefix + "/Set/RequiredEVCCID") {
        strncpy(RequiredEVCCID, payload.c_str(), sizeof(RequiredEVCCID));
        if (preferences.begin("settings", false) ) {                        //false = write mode
            preferences.putString("RequiredEVCCID", String(RequiredEVCCID));
            preferences.end();
        }
    }

    // Make sure MQTT updates directly to prevent debounces
    lastMqttUpdate = 10;
}

//wrapper so MQTTClient::Publish works
static struct mg_connection *s_conn;              // Client connection
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
    void disconnect(void) { mg_mqtt_disconnect(s_conn, &default_opts); };
};

void MQTTclient_t::publish(const String &topic, const String &payload, bool retained, int qos) {
  if (s_conn && connected) {
    struct mg_mqtt_opts opts = default_opts;
    opts.topic = mg_str(topic.c_str());
    opts.message = mg_str(payload.c_str());
    opts.qos = qos;
    opts.retain = retained;
    mg_mqtt_pub(s_conn, &opts);
  }
}

void MQTTclient_t::subscribe(const String &topic, int qos) {
  if (s_conn && connected) {
    struct mg_mqtt_opts opts = default_opts;
    opts.topic = mg_str(topic.c_str());
    opts.qos = qos;
    mg_mqtt_sub(s_conn, &opts);
  }
}

MQTTclient_t MQTTclient;

void SetupMQTTClient() {
    // Set up subscriptions
    MQTTclient.subscribe(MQTTprefix + "/Set/#",1);
    MQTTclient.publish(MQTTprefix+"/connected", "online", true, 0);

    //publish MQTT discovery topics
    //we need something to make all this JSON stuff readable, without doing all this assign and serialize stuff
#define jsn(x, y) String(R"(")") + x + R"(" : ")" + y + R"(")"
    //jsn(device_class, current) expands to:
    // R"("device_class" : "current")"

#define jsna(x, y) String(R"(, )") + jsn(x, y)
    //json add expansion, same as above but now with a comma prepended

    //first all device stuff:
    const String device_payload = String(R"("device": {)") + jsn("model","SmartEVSE v3") + jsna("identifiers", MQTTprefix) + jsna("name", MQTTprefix) + jsna("manufacturer","Stegen") + jsna("configuration_url", "http://" + WiFi.localIP().toString().c_str()) + jsna("sw_version", String(VERSION)) + "}";
    //a device SmartEVSE-1001 consists of multiple entities, and an entity can be in the domains sensor, number, select etc.
    String entity_suffix, entity_name, optional_payload;

    //some self-updating variables here:
#define entity_id String(MQTTprefix + "-" + entity_suffix)
#define entity_path String(MQTTprefix + "/" + entity_suffix)
#define entity_name(x) entity_name = x; entity_suffix = entity_name; entity_suffix.replace(" ", "");

    //create template to announce an entity in it's own domain:
#define announce(x, entity_domain) entity_name(x); \
    MQTTclient.publish("homeassistant/" + String(entity_domain) + "/" + entity_id + "/config", \
     "{" \
        + jsn("name", entity_name) \
        + jsna("object_id", entity_id) \
        + jsna("unique_id", entity_id) \
        + jsna("state_topic", entity_path) \
        + jsna("availability_topic",String(MQTTprefix+"/connected")) \
        + ", " + device_payload + optional_payload \
        + "}", \
    true, 0); // Retain + QoS 0

    //set the parameters for and announce sensors with device class 'current':
    optional_payload = jsna("device_class","current") + jsna("unit_of_measurement","A") + jsna("value_template", R"({{ value | int / 10 }})");
    announce("Charge Current", "sensor");
    announce("Max Current", "sensor");
    if (MainsMeter.Type) {
        announce("Mains Current L1", "sensor");
        announce("Mains Current L2", "sensor");
        announce("Mains Current L3", "sensor");
    }
    if (EVMeter.Type) {
        announce("EV Current L1", "sensor");
        announce("EV Current L2", "sensor");
        announce("EV Current L3", "sensor");
    }
    if (homeBatteryLastUpdate) {
        announce("Home Battery Current", "sensor");
    }

#if MODEM
        //set the parameters for modem/SoC sensor entities:
        optional_payload = jsna("unit_of_measurement","%") + jsna("value_template", R"({{ none if (value | int == -1) else (value | int) }})");
        announce("EV Initial SoC", "sensor");
        announce("EV Full SoC", "sensor");
        announce("EV Computed SoC", "sensor");
        announce("EV Remaining SoC", "sensor");

        optional_payload = jsna("device_class","duration") + jsna("unit_of_measurement","m") + jsna("value_template", R"({{ none if (value | int == -1) else (value | int / 60) | round }})");
        announce("EV Time Until Full", "sensor");

        optional_payload = jsna("device_class","energy") + jsna("unit_of_measurement","Wh") + jsna("value_template", R"({{ none if (value | int == -1) else (value | int) }})");
        announce("EV Energy Capacity", "sensor");
        announce("EV Energy Request", "sensor");

        optional_payload = jsna("value_template", R"({{ none if (value == '') else value }})");
        announce("EVCCID", "sensor");
        optional_payload = jsna("state_topic", String(MQTTprefix + "/RequiredEVCCID")) + jsna("command_topic", String(MQTTprefix + "/Set/RequiredEVCCID"));
        announce("Required EVCCID", "text");
#endif

    if (EVMeter.Type) {
        //set the parameters for and announce other sensor entities:
        optional_payload = jsna("device_class","power") + jsna("unit_of_measurement","W");
        announce("EV Charge Power", "sensor");
        optional_payload = jsna("device_class","energy") + jsna("unit_of_measurement","Wh");
        announce("EV Energy Charged", "sensor");
        optional_payload = jsna("device_class","energy") + jsna("unit_of_measurement","Wh") + jsna("state_class","total_increasing");
        announce("EV Total Energy Charged", "sensor");
    }

    //set the parameters for and announce sensor entities without device_class or unit_of_measurement:
    optional_payload = "";
    announce("EV Plug State", "sensor");
    announce("Access", "sensor");
    announce("State", "sensor");
    announce("RFID", "sensor");
    announce("RFIDLastRead", "sensor");
#if ENABLE_OCPP
    announce("OCPP", "sensor");
    announce("OCPPConnection", "sensor");
#endif //ENABLE_OCPP

    optional_payload = jsna("device_class","duration") + jsna("unit_of_measurement","s");
    announce("SolarStopTimer", "sensor");
    //set the parameters for and announce diagnostic sensor entities:
    optional_payload = jsna("entity_category","diagnostic");
    announce("Error", "sensor");
    announce("WiFi SSID", "sensor");
    announce("WiFi BSSID", "sensor");
    optional_payload = jsna("entity_category","diagnostic") + jsna("device_class","signal_strength") + jsna("unit_of_measurement","dBm");
    announce("WiFi RSSI", "sensor");
    optional_payload = jsna("entity_category","diagnostic") + jsna("device_class","temperature") + jsna("unit_of_measurement","°C");
    announce("ESP Temp", "sensor");
    optional_payload = jsna("entity_category","diagnostic") + jsna("device_class","duration") + jsna("unit_of_measurement","s") + jsna("entity_registry_enabled_default","False");
    announce("ESP Uptime", "sensor");

#if MODEM
        optional_payload = jsna("unit_of_measurement","%") + jsna("value_template", R"({{ (value | int / 1024 * 100) | round(0) }})");
        announce("CP PWM", "sensor");

        optional_payload = jsna("value_template", R"({{ none if (value | int == -1) else (value | int / 1024 * 100) | round }})");
        optional_payload += jsna("command_topic", String(MQTTprefix + "/Set/CPPWMOverride")) + jsna("min", "-1") + jsna("max", "100") + jsna("mode","slider");
        optional_payload += jsna("command_template", R"({{ (value | int * 1024 / 100) | round }})");
        announce("CP PWM Override", "number");
#endif
    //set the parameters for and announce select entities, overriding automatic state_topic:
    optional_payload = jsna("state_topic", String(MQTTprefix + "/Mode")) + jsna("command_topic", String(MQTTprefix + "/Set/Mode"));
    optional_payload += String(R"(, "options" : ["Off", "Normal", "Smart", "Solar"])");
    announce("Mode", "select");

    //set the parameters for and announce number entities:
    optional_payload = jsna("command_topic", String(MQTTprefix + "/Set/CurrentOverride")) + jsna("min", "0") + jsna("max", MaxCurrent ) + jsna("mode","slider");
    optional_payload += jsna("value_template", R"({{ value | int / 10 if value | is_number else none }})") + jsna("command_template", R"({{ value | int * 10 }})");
    announce("Charge Current Override", "number");
}
#endif
/*
//github.com L1
    const char* root_ca_github = R"ROOT_CA(
-----BEGIN CERTIFICATE-----
MIID0zCCArugAwIBAgIQVmcdBOpPmUxvEIFHWdJ1lDANBgkqhkiG9w0BAQwFADB7
MQswCQYDVQQGEwJHQjEbMBkGA1UECAwSR3JlYXRlciBNYW5jaGVzdGVyMRAwDgYD
VQQHDAdTYWxmb3JkMRowGAYDVQQKDBFDb21vZG8gQ0EgTGltaXRlZDEhMB8GA1UE
AwwYQUFBIENlcnRpZmljYXRlIFNlcnZpY2VzMB4XDTE5MDMxMjAwMDAwMFoXDTI4
MTIzMTIzNTk1OVowgYgxCzAJBgNVBAYTAlVTMRMwEQYDVQQIEwpOZXcgSmVyc2V5
MRQwEgYDVQQHEwtKZXJzZXkgQ2l0eTEeMBwGA1UEChMVVGhlIFVTRVJUUlVTVCBO
ZXR3b3JrMS4wLAYDVQQDEyVVU0VSVHJ1c3QgRUNDIENlcnRpZmljYXRpb24gQXV0
aG9yaXR5MHYwEAYHKoZIzj0CAQYFK4EEACIDYgAEGqxUWqn5aCPnetUkb1PGWthL
q8bVttHmc3Gu3ZzWDGH926CJA7gFFOxXzu5dP+Ihs8731Ip54KODfi2X0GHE8Znc
JZFjq38wo7Rw4sehM5zzvy5cU7Ffs30yf4o043l5o4HyMIHvMB8GA1UdIwQYMBaA
FKARCiM+lvEH7OKvKe+CpX/QMKS0MB0GA1UdDgQWBBQ64QmG1M8ZwpZ2dEl23OA1
xmNjmjAOBgNVHQ8BAf8EBAMCAYYwDwYDVR0TAQH/BAUwAwEB/zARBgNVHSAECjAI
MAYGBFUdIAAwQwYDVR0fBDwwOjA4oDagNIYyaHR0cDovL2NybC5jb21vZG9jYS5j
b20vQUFBQ2VydGlmaWNhdGVTZXJ2aWNlcy5jcmwwNAYIKwYBBQUHAQEEKDAmMCQG
CCsGAQUFBzABhhhodHRwOi8vb2NzcC5jb21vZG9jYS5jb20wDQYJKoZIhvcNAQEM
BQADggEBABns652JLCALBIAdGN5CmXKZFjK9Dpx1WywV4ilAbe7/ctvbq5AfjJXy
ij0IckKJUAfiORVsAYfZFhr1wHUrxeZWEQff2Ji8fJ8ZOd+LygBkc7xGEJuTI42+
FsMuCIKchjN0djsoTI0DQoWz4rIjQtUfenVqGtF8qmchxDM6OW1TyaLtYiKou+JV
bJlsQ2uRl9EMC5MCHdK8aXdJ5htN978UeAOwproLtOGFfy/cQjutdAFI3tZs4RmY
CV4Ks2dH/hzg1cEo70qLRDEmBDeNiXQ2Lu+lIg+DdEmSx/cQwgwp+7e9un/jX9Wf
8qn0dNW44bOwgeThpWOjzOoEeJBuv/c=
-----END CERTIFICATE-----
)ROOT_CA";


// get version nr. of latest release of off github
// input:
// owner_repo format: dingo35/SmartEVSE-3.5
// asset name format: one of firmware.bin, firmware.debug.bin, firmware.signed.bin, firmware.debug.signed.bin
// output:
// version -- null terminated string with latest version of this repo
// downloadUrl -- global pointer to null terminated string with the url where this version can be downloaded
bool getLatestVersion(String owner_repo, String asset_name, char *version) {
    HTTPClient httpClient;
    String useURL = "https://api.github.com/repos/" + owner_repo + "/releases/latest";
    httpClient.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);

    const char* url = useURL.c_str();
    _LOG_A("Connecting to: %s.\n", url );
    if( String(url).startsWith("https") ) {
        httpClient.begin(url, root_ca_github);
    } else {
        httpClient.begin(url);
    }
    httpClient.addHeader("User-Agent", "SmartEVSE-v3");
    httpClient.addHeader("Accept", "application/vnd.github+json");
    httpClient.addHeader("X-GitHub-Api-Version", "2022-11-28" );
    const char* get_headers[] = { "Content-Length", "Content-type", "Accept-Ranges" };
    httpClient.collectHeaders( get_headers, sizeof(get_headers)/sizeof(const char*) );
    int httpCode = httpClient.GET();  //Make the request

    // only handle 200/301, fail on everything else
    if( httpCode != HTTP_CODE_OK && httpCode != HTTP_CODE_MOVED_PERMANENTLY ) {
        // This error may be a false positive or a consequence of the network being disconnected.
        // Since the network is controlled from outside this class, only significant error messages are reported.
        _LOG_A("Error on HTTP request (httpCode=%i)\n", httpCode);
        httpClient.end();
        return false;
    }
    // The filter: it contains "true" for each value we want to keep
    DynamicJsonDocument  filter(100);
    filter["tag_name"] = true;
    filter["assets"][0]["browser_download_url"] = true;
    filter["assets"][0]["name"] = true;

    // Deserialize the document
    DynamicJsonDocument doc2(1500);
    DeserializationError error = deserializeJson(doc2, httpClient.getStream(), DeserializationOption::Filter(filter));

    if (error) {
        _LOG_A("deserializeJson() failed: %s\n", error.c_str());
        httpClient.end();  // We're done with HTTP - free the resources
        return false;
    }
    const char* tag_name = doc2["tag_name"]; // "v3.6.1"
    if (!tag_name) {
        //no version found
        _LOG_A("ERROR: LatestVersion of repo %s not found.\n", owner_repo.c_str());
        httpClient.end();  // We're done with HTTP - free the resources
        return false;
    }
    else
        //duplicate value so it won't get lost out of scope
        strlcpy(version, tag_name, 32);
        //strlcpy(version, tag_name, sizeof(version));
    _LOG_V("Found latest version:%s.\n", version);

    httpClient.end();  // We're done with HTTP - free the resources
    return true;
/*    for (JsonObject asset : doc2["assets"].as<JsonArray>()) {
        String name = asset["name"] | "";
        if (name == asset_name) {
            const char* asset_browser_download_url = asset["browser_download_url"];
            if (!asset_browser_download_url) {
                // no download url found
                _LOG_A("ERROR: Downloadurl of asset %s in repo %s not found.\n", asset_name.c_str(), owner_repo.c_str());
                httpClient.end();  // We're done with HTTP - free the resources
                return false;
            } else {
                asprintf(&downloadUrl, "%s", asset_browser_download_url);        //will be freed in FirmwareUpdate()
                _LOG_V("Found asset: name=%s, url=%s.\n", name.c_str(), downloadUrl);
                httpClient.end();  // We're done with HTTP - free the resources
                return true;
            }
        }
    }
    _LOG_A("ERROR: could not find asset %s in repo %s at version %s.\n", asset_name.c_str(), owner_repo.c_str(), version);
    httpClient.end();  // We're done with HTTP - free the resources
    return false;*//*
}


unsigned char *signature = NULL;
#define SIGNATURE_LENGTH 512

// SHA-Verify the OTA partition after it's been written
// https://techtutorialsx.com/2018/05/10/esp32-arduino-mbed-tls-using-the-sha-256-algorithm/
// https://github.com/ARMmbed/mbedtls/blob/development/programs/pkey/rsa_verify.c
bool validate_sig( const esp_partition_t* partition, unsigned char *signature, int size )
{
    const char* rsa_key_pub = R"RSA_KEY_PUB(
-----BEGIN PUBLIC KEY-----
MIICIjANBgkqhkiG9w0BAQEFAAOCAg8AMIICCgKCAgEAtjEWhkfKPAUrtX1GueYq
JmDp4qSHBG6ndwikAHvteKgWQABDpwaemZdxh7xVCuEdjEkaecinNOZ0LpSCF3QO
qflnXkvpYVxjdTpKBxo7vP5QEa3I6keJfwpoMzGuT8XOK7id6FHJhtYEXcaufALi
mR/NXT11ikHLtluATymPdoSscMiwry0qX03yIek91lDypBNl5uvD2jxn9smlijfq
9j0lwtpLBWJPU8vsU0uzuj7Qq5pWZFKsjiNWfbvNJXuLsupOazf5sh0yeQzL1CBL
RUsBlYVoChTmSOyvi6kO5vW/6GLOafJF0FTdOQ+Gf3/IB6M1ErSxlqxQhHq0pb7Y
INl7+aFCmlRjyLlMjb8xdtuedlZKv8mLd37AyPAihrq9gV74xq6c7w2y+h9213p8
jgcmo/HvOlGaXEIOVCUu102teOckXjTni2yhEtFISCaWuaIdb5P9e0uBIy1e+Bi6
/7A3aut5MQP07DO99BFETXyFF6EixhTF8fpwVZ5vXeIDvKKEDUGuzAziUEGIZpic
UQ2fmTzIaTBbNlCMeTQFIpZCosM947aGKNBp672wdf996SRwg9E2VWzW2Z1UuwWV
BPVQkHb1Hsy7C9fg5JcLKB9zEfyUH0Tm9Iur1vsuA5++JNl2+T55192wqyF0R9sb
YtSTUJNSiSwqWt1m0FLOJD0CAwEAAQ==
-----END PUBLIC KEY-----
)RSA_KEY_PUB";

    if( !partition ) {
        _LOG_A( "Could not find update partition!.\n");
        return false;
    }
    _LOG_D("Creating mbedtls context.\n");
    mbedtls_pk_context pk;
    mbedtls_md_context_t rsa;
    mbedtls_pk_init( &pk );
    _LOG_D("Parsing public key.\n");

    int ret;
    if( ( ret = mbedtls_pk_parse_public_key( &pk, (const unsigned char*)rsa_key_pub, strlen(rsa_key_pub)+1 ) ) != 0 ) {
        _LOG_A( "Parsing public key failed! mbedtls_pk_parse_public_key %d (%d bytes)\n%s", ret, strlen(rsa_key_pub)+1, rsa_key_pub);
        return false;
    }
    if( !mbedtls_pk_can_do( &pk, MBEDTLS_PK_RSA ) ) {
        _LOG_A( "Public key is not an rsa key -0x%x", -ret );
        return false;
    }
    _LOG_D("Initing mbedtls.\n");
    const mbedtls_md_info_t *mdinfo = mbedtls_md_info_from_type( MBEDTLS_MD_SHA256 );
    mbedtls_md_init( &rsa );
    mbedtls_md_setup( &rsa, mdinfo, 0 );
    mbedtls_md_starts( &rsa );
    int bytestoread = SPI_FLASH_SEC_SIZE;
    int bytesread = 0;
    uint8_t *_buffer = (uint8_t*)malloc(SPI_FLASH_SEC_SIZE);
    if(!_buffer){
        _LOG_A( "malloc failed.\n");
        return false;
    }
    _LOG_D("Parsing content.\n");
    _LOG_V( "Reading partition (%i sectors, sec_size: %i)", size, bytestoread );
    while( bytestoread > 0 ) {
        _LOG_V( "Left: %i (%i)               \r", size, bytestoread );

        if( ESP.partitionRead( partition, bytesread, (uint32_t*)_buffer, bytestoread ) ) {
            mbedtls_md_update( &rsa, (uint8_t*)_buffer, bytestoread );
            bytesread = bytesread + bytestoread;
            size = size - bytestoread;
            if( size <= SPI_FLASH_SEC_SIZE ) {
                bytestoread = size;
            }
        } else {
            _LOG_A( "partitionRead failed!.\n");
            return false;
        }
    }
    free( _buffer );

    unsigned char *hash = (unsigned char*)malloc( mdinfo->size );
    if(!hash){
        _LOG_A( "malloc failed.\n");
        return false;
    }
    mbedtls_md_finish( &rsa, hash );
    ret = mbedtls_pk_verify( &pk, MBEDTLS_MD_SHA256, hash, mdinfo->size, (unsigned char*)signature, SIGNATURE_LENGTH );
    free( hash );
    mbedtls_md_free( &rsa );
    mbedtls_pk_free( &pk );
    if( ret == 0 ) {
        return true;
    }

    // validation failed, overwrite the first few bytes so this partition won't boot!
    log_w( "Validation failed, erasing the invalid partition.\n");
    ESP.partitionEraseRange( partition, 0, ENCRYPTED_BLOCK_SIZE);
    return false;
}


bool forceUpdate(const char* firmwareURL, bool validate) {
    HTTPClient httpClient;
    //WiFiClientSecure _client;
    int partition = U_FLASH;

    httpClient.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
    _LOG_A("Connecting to: %s.\n", firmwareURL );
    if( String(firmwareURL).startsWith("https") ) {
        //_client.setCACert(root_ca_github); // OR
        //_client.setInsecure(); //not working for github
        httpClient.begin(firmwareURL, root_ca_github);
    } else {
        httpClient.begin(firmwareURL);
    }
    httpClient.addHeader("User-Agent", "SmartEVSE-v3");
    httpClient.addHeader("Accept", "application/vnd.github+json");
    httpClient.addHeader("X-GitHub-Api-Version", "2022-11-28" );
    const char* get_headers[] = { "Content-Length", "Content-type", "Accept-Ranges" };
    httpClient.collectHeaders( get_headers, sizeof(get_headers)/sizeof(const char*) );

    int updateSize = 0;
    int httpCode = httpClient.GET();
    String contentType;

    if( httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_MOVED_PERMANENTLY ) {
        updateSize = httpClient.getSize();
        contentType = httpClient.header( "Content-type" );
        String acceptRange = httpClient.header( "Accept-Ranges" );
        if( acceptRange == "bytes" ) {
            _LOG_V("This server supports resume!\n");
        } else {
            _LOG_V("This server does not support resume!\n");
        }
    } else {
        _LOG_A("ERROR: Server responded with HTTP Status %i.\n", httpCode );
        return false;
    }

    _LOG_D("updateSize : %i, contentType: %s.\n", updateSize, contentType.c_str());
    Stream * stream = httpClient.getStreamPtr();
    if( updateSize<=0 || stream == nullptr ) {
        _LOG_A("HTTP Error.\n");
        return false;
    }

    // some network streams (e.g. Ethernet) can be laggy and need to 'breathe'
    if( ! stream->available() ) {
        uint32_t timeout = millis() + 10000;
        while( ! stream->available() ) {
            if( millis()>timeout ) {
                _LOG_A("Stream timed out.\n");
                return false;
            }
            vTaskDelay(1);
        }
    }

    if( validate ) {
        if( updateSize == UPDATE_SIZE_UNKNOWN || updateSize <= SIGNATURE_LENGTH ) {
            _LOG_A("Malformed signature+fw combo.\n");
            return false;
        }
        updateSize -= SIGNATURE_LENGTH;
    }

    if( !Update.begin(updateSize, partition) ) {
        _LOG_A("ERROR Not enough space to begin OTA, partition size mismatch? Update failed!\n");
        Update.abort();
        return false;
    }

    Update.onProgress( [](uint32_t progress, uint32_t size) {
      _LOG_V("Firmware update progress %i/%i.\n", progress, size);
      //move this data to global var
      downloadProgress = progress;
      downloadSize = size;
      //give background tasks some air
      //vTaskDelay(100 / portTICK_PERIOD_MS);
    });

    // read signature
    if( validate ) {
        signature = (unsigned char *) malloc(SIGNATURE_LENGTH);                       //tried to free in in all exit scenarios, RISK of leakage!!!
        stream->readBytes( signature, SIGNATURE_LENGTH );
    }

    _LOG_I("Begin %s OTA. This may take 2 - 5 mins to complete. Things might be quiet for a while.. Patience!\n", partition==U_FLASH?"Firmware":"Filesystem");

    // Some activity may appear in the Serial monitor during the update (depends on Update.onProgress)
    int written = Update.writeStream(*stream);                                 // although writeStream returns size_t, we don't expect >2Gb

    if ( written == updateSize ) {
        _LOG_D("Written : %d successfully", written);
        updateSize = written; // flatten value to prevent overflow when checking signature
    } else {
        _LOG_A("Written only : %u/%u Premature end of stream?", written, updateSize);
        Update.abort();
        FREE(signature);
        return false;
    }

    if (!Update.end()) {
        _LOG_A("An Update Error Occurred. Error #: %d", Update.getError());
        FREE(signature);
        return false;
    }

    if( validate ) { // check signature
        _LOG_I("Checking partition %d to validate", partition);

        //getPartition( partition ); // updated partition => '_target_partition' pointer
        const esp_partition_t* _target_partition = esp_ota_get_next_update_partition(NULL);

        #define CHECK_SIG_ERROR_PARTITION_NOT_FOUND -1
        #define CHECK_SIG_ERROR_VALIDATION_FAILED   -2

        if( !_target_partition ) {
            _LOG_A("Can't access partition #%d to check signature!", partition);
            FREE(signature);
            return false;
        }

        _LOG_D("Checking signature for partition %d...", partition);

        const esp_partition_t* running_partition = esp_ota_get_running_partition();

        if( partition == U_FLASH ) {
            // /!\ An OTA partition is automatically set as bootable after being successfully
            // flashed by the Update library.
            // Since we want to validate before enabling the partition, we need to cancel that
            // by temporarily reassigning the bootable flag to the running-partition instead
            // of the next-partition.
            esp_ota_set_boot_partition( running_partition );
            // By doing so the ESP will NOT boot any unvalidated partition should a reset occur
            // during signature validation (crash, oom, power failure).
        }

        if( !validate_sig( _target_partition, signature, updateSize ) ) {
            FREE(signature);
            // erase partition
            esp_partition_erase_range( _target_partition, _target_partition->address, _target_partition->size );
            _LOG_A("Signature check failed!.\n");
            return false;
        } else {
            FREE(signature);
            _LOG_D("Signature check successful!.\n");
            if( partition == U_FLASH ) {
                // Set updated partition as bootable now that it's been verified
                esp_ota_set_boot_partition( _target_partition );
            }
        }
    }
    _LOG_D("OTA Update complete!.\n");
    if (Update.isFinished()) {
        _LOG_V("Update succesfully completed at %s partition\n", partition==U_SPIFFS ? "spiffs" : "firmware" );
        return true;
    } else {
        _LOG_A("ERROR: Update not finished! Something went wrong!.\n");
    }
    return false;
}


// put firmware update in separate task so we can feed progress to the html page
void FirmwareUpdate(void *parameter) {
    //_LOG_A("DINGO: url=%s.\n", downloadUrl);
    if (forceUpdate(downloadUrl, 1)) {
        _LOG_A("Firmware update succesfull; rebooting as soon as no EV is connected.\n");
        downloadProgress = -1;
        shouldReboot = true;
    } else {
        _LOG_A("ERROR: Firmware update failed.\n");
        //_http.end();
        downloadProgress = -2;
    }
    if (downloadUrl) free(downloadUrl);
    vTaskDelete(NULL);                                                        //end this task so it will not take up resources
}

void RunFirmwareUpdate(void) {
    _LOG_V("Starting firmware update from downloadUrl=%s.\n", downloadUrl);
    downloadProgress = 0;                                                       // clear errors, if any
    xTaskCreate(
        FirmwareUpdate, // Function that should be called
        "FirmwareUpdate",// Name of the task (for debugging)
        4096,           // Stack size (bytes)
        NULL,           // Parameter to pass
        3,              // Task priority - high
        NULL            // Task handle
    );
}


/* Takes TimeString in format
 * String = "2023-04-14T11:31"
 * and store it in the DelayedTimeStruct
 * returns 0 on success, 1 on failure
*//*
int StoreTimeString(String DelayedTimeStr, DelayedTimeStruct *DelayedTime) {
    // Parse the time string
    tm delayedtime_tm = {};
    if (strptime(DelayedTimeStr.c_str(), "%Y-%m-%dT%H:%M", &delayedtime_tm)) {
        delayedtime_tm.tm_isdst = -1;                 //so mktime is going to figure out whether DST is there or not
        DelayedTime->epoch2 = mktime(&delayedtime_tm) - EPOCH2_OFFSET;
        // Compare the times
        time_t now = time(nullptr);             //get current local time
        DelayedTime->diff = DelayedTime->epoch2 - (mktime(localtime(&now)) - EPOCH2_OFFSET);
        return 0;
    }
    //error TODO not sure whether we keep the old time or reset it to zero?
    //DelayedTime.epoch2 = 0;
    //DelayedTime.diff = 0;
    return 1;
}


void setTimeZone(void) {
    HTTPClient httpClient;
    // lookup current timezone
    httpClient.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
    httpClient.begin("http://worldtimeapi.org/api/ip");
    int httpCode = httpClient.GET();  //Make the request

    // only handle 200/301, fail on everything else
    if( httpCode != HTTP_CODE_OK && httpCode != HTTP_CODE_MOVED_PERMANENTLY ) {
        _LOG_A("Error on HTTP request (httpCode=%i)\n", httpCode);
        httpClient.end();
        return;
    }

    // The filter: it contains "true" for each value we want to keep
    DynamicJsonDocument  filter(16);
    filter["timezone"] = true;
    DynamicJsonDocument doc2(80);
    DeserializationError error = deserializeJson(doc2, httpClient.getStream(), DeserializationOption::Filter(filter));
    httpClient.end();
    if (error) {
        _LOG_A("deserializeJson() failed: %s\n", error.c_str());
        return;
    }
    String tzname = doc2["timezone"];
    if (tzname == "") {
        _LOG_A("Could not detect Timezone.\n");
        return;
    }
    _LOG_A("Timezone detected: tz=%s.\n", tzname.c_str());

    // takes TZname (format: Europe/Berlin) , gets TZ_INFO (posix string, format: CET-1CEST,M3.5.0,M10.5.0/3) and sets and stores timezonestring accordingly
    //httpClient.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
    WiFiClient * stream = httpClient.getStreamPtr();
    String l;
    char *URL;
    asprintf(&URL, "%s/zones.csv", FW_DOWNLOAD_PATH); //will be freed
    httpClient.begin(URL);
    httpCode = httpClient.GET();  //Make the request

    // only handle 200/301, fail on everything else
    if( httpCode != HTTP_CODE_OK && httpCode != HTTP_CODE_MOVED_PERMANENTLY ) {
        _LOG_A("Error on HTTP request (httpCode=%i)\n", httpCode);
        httpClient.end();
        FREE(URL);
        return;
    }

    stream = httpClient.getStreamPtr();
    while(httpClient.connected() && stream->available()) {
        l = stream->readStringUntil('\n');
        if (l.indexOf(tzname) > 0) {
            int from = l.indexOf("\",\"") + 3;
            TZinfo = l.substring(from, l.length() - 1);
            _LOG_A("Detected Timezone info: TZname = %s, tz_info=%s.\n", tzname.c_str(), TZinfo.c_str());
            setenv("TZ",TZinfo.c_str(),1);
            tzset();
            if (preferences.begin("settings", false) ) {
                preferences.putString("TimezoneInfo", TZinfo);
                preferences.end();
            }
            break;
        }
    }
    httpClient.end();
    FREE(URL);
}
*/
/*
// wrapper so hasParam and getParam still work
class webServerRequest {
private:
    struct mg_http_message *hm_internal;
    String _value;
    char temp[64];

public:
    void setMessage(struct mg_http_message *hm);
    bool hasParam(const char *param);
    webServerRequest* getParam(const char *param); // Return pointer to self
    const String& value(); // Return the string value
};

void webServerRequest::setMessage(struct mg_http_message *hm) {
    hm_internal = hm;
}

bool webServerRequest::hasParam(const char *param) {
    return (mg_http_get_var(&hm_internal->query, param, temp, sizeof(temp)) > 0);
}

webServerRequest* webServerRequest::getParam(const char *param) {
    _value = ""; // Clear previous value
    if (mg_http_get_var(&hm_internal->query, param, temp, sizeof(temp)) > 0) {
        _value = temp;
    }
    return this; // Return pointer to self
}

const String& webServerRequest::value() {
    return _value; // Return the string value
}
//end of wrapper

struct mg_str empty = mg_str_n("", 0UL);

#if MQTT
char s_mqtt_url[80];
//TODO perhaps integrate multiple fn callback functions?
static void fn_mqtt(struct mg_connection *c, int ev, void *ev_data) {
    if (ev == MG_EV_OPEN) {
        _LOG_V("%lu CREATED\n", c->id);
        // c->is_hexdumping = 1;
    } else if (ev == MG_EV_ERROR) {
        // On error, log error message
        _LOG_A("%lu ERROR %s\n", c->id, (char *) ev_data);
    } else if (ev == MG_EV_CONNECT) {
        // If target URL is SSL/TLS, command client connection to use TLS
        if (mg_url_is_ssl(s_mqtt_url)) {
            struct mg_tls_opts opts = {.ca = empty, .cert = empty, .key = empty, .name = mg_url_host(s_mqtt_url), .skip_verification = 0};
            //struct mg_tls_opts opts = {.ca = empty};
            mg_tls_init(c, &opts);
        }
    } else if (ev == MG_EV_MQTT_OPEN) {
        // MQTT connect is successful
        _LOG_V("%lu CONNECTED to %s\n", c->id, s_mqtt_url);
        MQTTclient.connected = true;
        SetupMQTTClient();
    } else if (ev == MG_EV_MQTT_MSG) {
        // When we get echo response, print it
        struct mg_mqtt_message *mm = (struct mg_mqtt_message *) ev_data;
        _LOG_V("%lu RECEIVED %.*s <- %.*s\n", c->id, (int) mm->data.len, mm->data.buf, (int) mm->topic.len, mm->topic.buf);
        //somehow topic is not null terminated
        String topic2 = String(mm->topic.buf).substring(0,mm->topic.len);
        mqtt_receive_callback(topic2, mm->data.buf);
    } else if (ev == MG_EV_CLOSE) {
        _LOG_V("%lu CLOSED\n", c->id);
        MQTTclient.connected = false;
        s_conn = NULL;  // Mark that we're closed
    }
}

// Timer function - recreate client connection if it is closed
static void timer_fn(void *arg) {
    struct mg_mgr *mgr = (struct mg_mgr *) arg;
    struct mg_mqtt_opts opts;
    memset(&opts, 0, sizeof(opts));
    opts.clean = false;
    // set will topic
    String temp = MQTTprefix + "/connected";
    opts.topic = mg_str(temp.c_str());
    opts.message = mg_str("offline");
    opts.retain = true;
    opts.keepalive = 15;                                                          // so we will timeout after 15s
    opts.version = 4;
    opts.client_id=mg_str(MQTTprefix.c_str());
    opts.user=mg_str(MQTTuser.c_str());
    opts.pass=mg_str(MQTTpassword.c_str());

    //prepare MQTT url
    //mqtt[s]://[username][:password]@host.domain[:port]
    snprintf(s_mqtt_url, sizeof(s_mqtt_url), "mqtt://%s:%i", MQTTHost.c_str(), MQTTPort);

    if (s_conn == NULL) s_conn = mg_mqtt_connect(mgr, s_mqtt_url, &opts, fn_mqtt, NULL);
}
#endif
/*
// Connection event handler function
// indenting lower level two spaces to stay compatible with old StartWebServer
// We use the same event handler function for HTTP and HTTPS connections
// fn_data is NULL for plain HTTP, and non-NULL for HTTPS
static void fn_http_server(struct mg_connection *c, int ev, void *ev_data) {
  if (ev == MG_EV_ACCEPT && c->fn_data != NULL) {
    struct mg_tls_opts opts = { .ca = empty, .cert = mg_unpacked("/data/cert.pem"), .key = mg_unpacked("/data/key.pem"), .name = empty, .skip_verification = 0};
    mg_tls_init(c, &opts);
  } else if (ev == MG_EV_CLOSE) {
    if (c == HttpListener80) {
        _LOG_A("Free HTTP port 80");
        HttpListener80 = nullptr;
    }
    if (c == HttpListener443) {
        _LOG_A("Free HTTP port 443");
        HttpListener443 = nullptr;
    }
  } else if (ev == MG_EV_HTTP_MSG) {  // New HTTP request received
    struct mg_http_message *hm = (struct mg_http_message *) ev_data;            // Parsed HTTP request
    webServerRequest* request = new webServerRequest();
    request->setMessage(hm);
//make mongoose 7.14 compatible with 7.13
#define mg_http_match_uri(X,Y) mg_match(X->uri, mg_str(Y), NULL)
    if (mg_match(hm->uri, mg_str("/erasesettings"), NULL)) {
        mg_http_reply(c, 200, "Content-Type: text/plain\r\n", "Erasing settings, rebooting");
        if ( preferences.begin("settings", false) ) {         // our own settings
          preferences.clear();
          preferences.end();
        }
        if (preferences.begin("nvs.net80211", false) ) {      // WiFi settings used by ESP
          preferences.clear();
          preferences.end();       
        }
        ESP.restart();
    } else if (mg_http_match_uri(hm, "/autoupdate")) {
        char owner[40];
        char buf[8];
        int debug;
        mg_http_get_var(&hm->query, "owner", owner, sizeof(owner));
        mg_http_get_var(&hm->query, "debug", buf, sizeof(buf));
        debug = strtol(buf, NULL, 0);
        if (!memcmp(owner, OWNER_FACT, sizeof(OWNER_FACT)) || (!memcmp(owner, OWNER_COMM, sizeof(OWNER_COMM)))) {
            asprintf(&downloadUrl, "%s/%s_firmware.%ssigned.bin", FW_DOWNLOAD_PATH, owner, debug ? "debug.": ""); //will be freed in FirmwareUpdate() ; format: http://s3.com/fact_firmware.debug.signed.bin
            RunFirmwareUpdate();
        }                                                                       // after the first call we just report progress
        DynamicJsonDocument doc(64); // https://arduinojson.org/v6/assistant/
        doc["progress"] = downloadProgress;
        doc["size"] = downloadSize;
        String json;
        serializeJson(doc, json);
        mg_http_reply(c, 200, "Content-Type: application/json\r\n", "%s\n", json.c_str());    // Yes. Respond JSON
    } else if (mg_http_match_uri(hm, "/update")) {
        //modified version of mg_http_upload
        char buf[20] = "0", file[40];
        size_t max_size = 0x1B0000;                                             //from partition_custom.csv
        long res = 0, offset, size;
        mg_http_get_var(&hm->query, "offset", buf, sizeof(buf));
        mg_http_get_var(&hm->query, "file", file, sizeof(file));
        offset = strtol(buf, NULL, 0);
        buf[0] = '0';
        mg_http_get_var(&hm->query, "size", buf, sizeof(buf));
        size = strtol(buf, NULL, 0);
        if (hm->body.len == 0) {
          struct mg_http_serve_opts opts = {.root_dir = "/data", .ssi_pattern = NULL, .extra_headers = NULL, .mime_types = NULL, .page404 = NULL, .fs = &mg_fs_packed };
          mg_http_serve_file(c, hm, "/data/update2.html", &opts);
        } else if (file[0] == '\0') {
          mg_http_reply(c, 400, "", "file required");
          res = -1;
        } else if (offset < 0) {
          mg_http_reply(c, 400, "", "offset required");
          res = -3;
        } else if ((size_t) offset + hm->body.len > max_size) {
          mg_http_reply(c, 400, "", "over max size of %lu", (unsigned long) max_size);
          res = -4;
        } else if (size <= 0) {
          mg_http_reply(c, 400, "", "size required");
          res = -5;
        } else {
            if (!memcmp(file,"firmware.bin", sizeof("firmware.bin")) || !memcmp(file,"firmware.debug.bin", sizeof("firmware.debug.bin"))) {
                if(!offset) {
                    _LOG_A("Update Start: %s\n", file);
                    if(!Update.begin((ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000), U_FLASH) {
                            Update.printError(Serial);
                    }
                }
                if(!Update.hasError()) {
                    if(Update.write((uint8_t*) hm->body.buf, hm->body.len) != hm->body.len) {
                        Update.printError(Serial);
                    } else {
                        _LOG_A("bytes written %lu\r", offset + hm->body.len);
                    }
                }
                if (offset + hm->body.len >= size) {                                           //EOF
                    if(Update.end(true)) {
                        _LOG_A("\nUpdate Success\n");
                        delay(1000);
                        ESP.restart();
                    } else {
                        Update.printError(Serial);
                    }
                }
            } else //end of firmware.bin
            if (!memcmp(file,"firmware.signed.bin", sizeof("firmware.signed.bin")) || !memcmp(file,"firmware.debug.signed.bin", sizeof("firmware.debug.signed.bin"))) {
#define dump(X)   for (int i= 0; i< SIGNATURE_LENGTH; i++) _LOG_A_NO_FUNC("%02x", X[i]); _LOG_A_NO_FUNC(".\n");
                if(!offset) {
                    _LOG_A("Update Start: %s\n", file);
                    signature = (unsigned char *) malloc(SIGNATURE_LENGTH);                       //tried to free in in all exit scenarios, RISK of leakage!!!
                    memcpy(signature, hm->body.buf, SIGNATURE_LENGTH);          //signature is prepended to firmware.bin
                    hm->body.buf = hm->body.buf + SIGNATURE_LENGTH;
                    hm->body.len = hm->body.len - SIGNATURE_LENGTH;
                    _LOG_A("Firmware signature:");
                    dump(signature);
                    if(!Update.begin((ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000), U_FLASH) {
                            Update.printError(Serial);
                    }
                }
                if(!Update.hasError()) {
                    if(Update.write((uint8_t*) hm->body.buf, hm->body.len) != hm->body.len) {
                        Update.printError(Serial);
                        FREE(signature);
                    } else {
                        _LOG_A("bytes written %lu\r", offset + hm->body.len);
                    }
                }
                if (offset + hm->body.len >= size) {                                           //EOF
                    //esp_err_t err;
                    const esp_partition_t* target_partition = esp_ota_get_next_update_partition(NULL);              // the newly updated partition
                    if (!target_partition) {
                        _LOG_A("ERROR: Can't access firmware partition to check signature!");
                        mg_http_reply(c, 400, "", "firmware.signed.bin update failed!");
                    }
                    const esp_partition_t* running_partition = esp_ota_get_running_partition();
                    _LOG_V("Running off of partition %s, trying to update partition %s.\n", running_partition->label, target_partition->label);
                    esp_ota_set_boot_partition( running_partition );            // make sure we have not switched boot partitions

                    bool verification_result = false;
                    if(Update.end(true)) {
                        verification_result = validate_sig( target_partition, signature, size - SIGNATURE_LENGTH);
                        FREE(signature);
                        if (verification_result) {
                            _LOG_A("Signature is valid!\n");
                            esp_ota_set_boot_partition( target_partition );
                            _LOG_A("\nUpdate Success\n");
                            shouldReboot = true;
                            //ESP.restart(); does not finish the call to fn_http_server, so the last POST of apps.js gets no response....
                            //which results in a "verify failed" message on the /update screen AFTER the reboot :-)
                        }
                    }
                    if (!verification_result) {
                        _LOG_A("Update failed!\n");
                        Update.printError(Serial);
                        //Update.abort(); //not sure this does anything in this stage
                        //Update.rollBack();
                        _LOG_V("Running off of partition %s, erasing partition %s.\n", running_partition->label, target_partition->label);
                        esp_partition_erase_range( target_partition, target_partition->address, target_partition->size );
                        esp_ota_set_boot_partition( running_partition );
                        mg_http_reply(c, 400, "", "firmware.signed.bin update failed!");
                    }
                    FREE(signature);
                }
            } else //end of firmware.signed.bin
            if (!memcmp(file,"rfid.txt", sizeof("rfid.txt"))) {
                if (offset != 0) {
                    mg_http_reply(c, 400, "", "rfid.txt too big, only 100 rfid's allowed!");
                }
                else {
                    //we are overwriting all stored RFID's with the ones uploaded
                    DeleteAllRFID();
                    res = offset + hm->body.len;
                    unsigned int RFID_UID[8] = {1, 0, 0, 0, 0, 0, 0, 0};
                    char RFIDtxtstring[20];                                     // 17 characters + NULL terminator
                    int r, pos = 0;
                    int beginpos = 0;
                    while (pos <= hm->body.len) {
                        char c;
                        c = *(hm->body.buf + pos);
                        //_LOG_A_NO_FUNC("%c", c);
                        if (c == '\n' || pos == hm->body.len) {
                            strncpy(RFIDtxtstring, hm->body.buf + beginpos, 19);         // in case of DOS the 0x0D is stripped off here
                            RFIDtxtstring[19] = '\0';
                            r = sscanf(RFIDtxtstring,"%02X%02x%02x%02x%02x%02x%02x", &RFID_UID[0], &RFID_UID[1], &RFID_UID[2], &RFID_UID[3], &RFID_UID[4], &RFID_UID[5], &RFID_UID[6]);
                            RFID_UID[7]=crc8((unsigned char *) RFID_UID,7);
                            if (r == 7) {
                                _LOG_A("Store RFID_UID %02x%02x%02x%02x%02x%02x%02x, crc=%02x.\n", RFID_UID[0], RFID_UID[1], RFID_UID[2], RFID_UID[3], RFID_UID[4], RFID_UID[5], RFID_UID[6], RFID_UID[7]);
                                LoadandStoreRFID(RFID_UID);
                            } else {
                                strncpy(RFIDtxtstring, hm->body.buf + beginpos, 17);         // in case of DOS the 0x0D is stripped off here
                                RFIDtxtstring[17] = '\0';
                                RFID_UID[0] = 0x01;
                                r = sscanf(RFIDtxtstring,"%02x%02x%02x%02x%02x%02x", &RFID_UID[1], &RFID_UID[2], &RFID_UID[3], &RFID_UID[4], &RFID_UID[5], &RFID_UID[6]);
                                RFID_UID[7]=crc8((unsigned char *) RFID_UID,7);
                                if (r == 6) {
                                    _LOG_A("Store RFID_UID %02x%02x%02x%02x%02x%02x, crc=%02x.\n", RFID_UID[1], RFID_UID[2], RFID_UID[3], RFID_UID[4], RFID_UID[5], RFID_UID[6], RFID_UID[7]);
                                    LoadandStoreRFID(RFID_UID);
                                }
                            }
                            beginpos = pos + 1;
                        }
                        pos++;
                    }
                }
            } else //end of rfid.txt
                mg_http_reply(c, 400, "", "only allowed to flash firmware.bin, firmware.debug.bin, firmware.signed.bin, firmware.debug.signed.bin or rfid.txt");
            mg_http_reply(c, 200, "", "%ld", res);
        }
    } else if (mg_http_match_uri(hm, "/settings")) {                            // REST API call?
      if (!memcmp("GET", hm->method.buf, hm->method.len)) {                     // if GET
        String mode = "N/A";
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

        DynamicJsonDocument doc(1600); // https://arduinojson.org/v6/assistant/
        doc["version"] = String(VERSION);
        doc["serialnr"] = serialnr;
        doc["mode"] = mode;
        doc["mode_id"] = modeId;
        doc["car_connected"] = evConnected;

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
        
        doc["evse"]["temp"] = TempEVSE;
        doc["evse"]["temp_max"] = maxTemp;
        doc["evse"]["connected"] = evConnected;
        doc["evse"]["access"] = Access_bit == 1;
        doc["evse"]["mode"] = Mode;
        doc["evse"]["loadbl"] = LoadBl;
        doc["evse"]["pwm"] = CurrentPWM;
        doc["evse"]["solar_stop_timer"] = SolarStopTimer;
        doc["evse"]["state"] = evstate;
        doc["evse"]["state_id"] = State;
        doc["evse"]["error"] = error;
        doc["evse"]["error_id"] = errorId;
        doc["evse"]["rfid"] = !RFIDReader ? "Not Installed" : RFIDstatus >= 8 ? "NOSTATUS" : StrRFIDStatusWeb[RFIDstatus];
        if (RFIDReader && RFIDReader != 6) { //RFIDLastRead not updated in Remote/OCPP mode
            char buf[15];
            if (RFID[0] == 0x01) {  // old reader 6 byte UID starts at RFID[1]
                sprintf(buf, "%02X%02X%02X%02X%02X%02X", RFID[1], RFID[2], RFID[3], RFID[4], RFID[5], RFID[6]);
            } else {
                sprintf(buf, "%02X%02X%02X%02X%02X%02X%02X", RFID[0], RFID[1], RFID[2], RFID[3], RFID[4], RFID[5], RFID[6]);
            }
            doc["evse"]["rfid_lastread"] = buf;
        }

        doc["settings"]["charge_current"] = Balanced[0];
        doc["settings"]["override_current"] = OverrideCurrent;
        doc["settings"]["current_min"] = MinCurrent;
        doc["settings"]["current_max"] = MaxCurrent;
        doc["settings"]["current_main"] = MaxMains;
        doc["settings"]["current_max_circuit"] = MaxCircuit;
        doc["settings"]["current_max_sum_mains"] = MaxSumMains;
        doc["settings"]["max_sum_mains_time"] = MaxSumMainsTime;
        doc["settings"]["solar_max_import"] = ImportCurrent;
        doc["settings"]["solar_start_current"] = StartCurrent;
        doc["settings"]["solar_stop_time"] = StopTime;
        doc["settings"]["enable_C2"] = StrEnableC2[EnableC2];
        doc["settings"]["mains_meter"] = EMConfig[MainsMeter.Type].Desc;
        doc["settings"]["starttime"] = (DelayedStartTime.epoch2 ? DelayedStartTime.epoch2 + EPOCH2_OFFSET : 0);
        doc["settings"]["stoptime"] = (DelayedStopTime.epoch2 ? DelayedStopTime.epoch2 + EPOCH2_OFFSET : 0);
        doc["settings"]["repeat"] = DelayedRepeat;
#if MODEM
            doc["settings"]["required_evccid"] = RequiredEVCCID;
            doc["settings"]["modem"] = "Experiment";

            doc["ev_state"]["initial_soc"] = InitialSoC;
            doc["ev_state"]["remaining_soc"] = RemainingSoC;
            doc["ev_state"]["full_soc"] = FullSoC;
            doc["ev_state"]["energy_capacity"] = EnergyCapacity > 0 ? round((float)EnergyCapacity / 100)/10 : -1; //in kWh, precision 1 decimal;
            doc["ev_state"]["energy_request"] = EnergyRequest > 0 ? round((float)EnergyRequest / 100)/10 : -1; //in kWh, precision 1 decimal
            doc["ev_state"]["computed_soc"] = ComputedSoC;
            doc["ev_state"]["evccid"] = EVCCID;
            doc["ev_state"]["time_until_full"] = TimeUntilFull;
#endif

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

#if ENABLE_OCPP
        doc["ocpp"]["mode"] = OcppMode ? "Enabled" : "Disabled";
        doc["ocpp"]["backend_url"] = OcppWsClient ? OcppWsClient->getBackendUrl() : "";
        doc["ocpp"]["cb_id"] = OcppWsClient ? OcppWsClient->getChargeBoxId() : "";
        doc["ocpp"]["auth_key"] = OcppWsClient ? OcppWsClient->getAuthKey() : "";

        {
            auto freevendMode = MicroOcpp::getConfigurationPublic(MO_CONFIG_EXT_PREFIX "FreeVendActive");
            doc["ocpp"]["auto_auth"] = freevendMode && freevendMode->getBool() ? "Enabled" : "Disabled";
            auto freevendIdTag = MicroOcpp::getConfigurationPublic(MO_CONFIG_EXT_PREFIX "FreeVendIdTag");
            doc["ocpp"]["auto_auth_idtag"] = freevendIdTag ? freevendIdTag->getString() : "";
        }

        if (OcppWsClient && OcppWsClient->isConnected()) {
            doc["ocpp"]["status"] = "Connected";
        } else {
            doc["ocpp"]["status"] = "Disconnected";
        }
#endif //ENABLE_OCPP

        doc["home_battery"]["current"] = homeBatteryCurrent;
        doc["home_battery"]["last_update"] = homeBatteryLastUpdate;

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

        doc["phase_currents"]["TOTAL"] = MainsMeter.Irms[0] + MainsMeter.Irms[1] + MainsMeter.Irms[2];
        doc["phase_currents"]["L1"] = MainsMeter.Irms[0];
        doc["phase_currents"]["L2"] = MainsMeter.Irms[1];
        doc["phase_currents"]["L3"] = MainsMeter.Irms[2];
        doc["phase_currents"]["last_data_update"] = phasesLastUpdate;
        doc["phase_currents"]["original_data"]["TOTAL"] = IrmsOriginal[0] + IrmsOriginal[1] + IrmsOriginal[2];
        doc["phase_currents"]["original_data"]["L1"] = IrmsOriginal[0];
        doc["phase_currents"]["original_data"]["L2"] = IrmsOriginal[1];
        doc["phase_currents"]["original_data"]["L3"] = IrmsOriginal[2];
        
        doc["backlight"]["timer"] = BacklightTimer;
        doc["backlight"]["status"] = backlight;

        String json;
        serializeJson(doc, json);
        mg_http_reply(c, 200, "Content-Type: application/json\r\n", "%s\n", json.c_str());    // Yes. Respond JSON
      } else if (!memcmp("POST", hm->method.buf, hm->method.len)) {                     // if POST
        DynamicJsonDocument doc(512); // https://arduinojson.org/v6/assistant/

        if(request->hasParam("backlight")) {
            int backlight = request->getParam("backlight")->value().toInt();
            BacklightTimer = backlight * BACKLIGHT;
            doc["Backlight"] = backlight;
        }

        if(request->hasParam("current_min")) {
            int current = request->getParam("current_min")->value().toInt();
            if(current >= MIN_CURRENT && current <= 16 && LoadBl < 2) {
                MinCurrent = current;
                doc["current_min"] = MinCurrent;
                write_settings();
            } else {
                doc["current_min"] = "Value not allowed!";
            }
        }

        if(request->hasParam("current_max_sum_mains")) {
            int current = request->getParam("current_max_sum_mains")->value().toInt();
            if((current == 0 || (current >= 10 && current <= 600)) && LoadBl < 2) {
                MaxSumMains = current;
                doc["current_max_sum_mains"] = MaxSumMains;
                write_settings();
            } else {
                doc["current_max_sum_mains"] = "Value not allowed!";
            }
        }

        if(request->hasParam("max_sum_mains_timer")) {
            int time = request->getParam("max_sum_mains_timer")->value().toInt();
            if(time >= 0 && time <= 60 && LoadBl < 2) {
                MaxSumMainsTime = time;
                doc["max_sum_mains_time"] = MaxSumMainsTime;
                write_settings();
            } else {
                doc["max_sum_mains_time"] = "Value not allowed!";
            }
        }

        if(request->hasParam("disable_override_current")) {
            OverrideCurrent = 0;
            doc["disable_override_current"] = "OK";
        }

        if(request->hasParam("mode")) {
            String mode = request->getParam("mode")->value();

            //first check if we have a delayed mode switch
            if(request->hasParam("starttime")) {
                String DelayedStartTimeStr = request->getParam("starttime")->value();
                //string time_str = "2023-04-14T11:31";
                if (!StoreTimeString(DelayedStartTimeStr, &DelayedStartTime)) {
                    //parse OK
                    if (DelayedStartTime.diff > 0)
                        setAccess(0);                         //switch to OFF, we are Delayed Charging
                    else {//we are in the past so no delayed charging
                        DelayedStartTime.epoch2 = DELAYEDSTARTTIME;
                        DelayedStopTime.epoch2 = DELAYEDSTOPTIME;
                        DelayedRepeat = 0;
                    }
                }
                else {
                    //we couldn't parse the string, so we are NOT Delayed Charging
                    DelayedStartTime.epoch2 = DELAYEDSTARTTIME;
                    DelayedStopTime.epoch2 = DELAYEDSTOPTIME;
                    DelayedRepeat = 0;
                }

                // so now we might have a starttime and we might be Delayed Charging
                if (DelayedStartTime.epoch2) {
                    //we only accept a DelayedStopTime if we have a valid DelayedStartTime
                    if(request->hasParam("stoptime")) {
                        String DelayedStopTimeStr = request->getParam("stoptime")->value();
                        //string time_str = "2023-04-14T11:31";
                        if (!StoreTimeString(DelayedStopTimeStr, &DelayedStopTime)) {
                            //parse OK
                            if (DelayedStopTime.diff <= 0 || DelayedStopTime.epoch2 <= DelayedStartTime.epoch2)
                                //we are in the past or DelayedStopTime before DelayedStartTime so no DelayedStopTime
                                DelayedStopTime.epoch2 = DELAYEDSTOPTIME;
                        }
                        else
                            //we couldn't parse the string, so no DelayedStopTime
                            DelayedStopTime.epoch2 = DELAYEDSTOPTIME;
                        doc["stoptime"] = (DelayedStopTime.epoch2 ? DelayedStopTime.epoch2 + EPOCH2_OFFSET : 0);
                        if(request->hasParam("repeat")) {
                            int Repeat = request->getParam("repeat")->value().toInt();
                            if (Repeat >= 0 && Repeat <= 1) {                                   //boundary check
                                DelayedRepeat = Repeat;
                                doc["repeat"] = Repeat;
                            }
                        }
                    }

                }
                doc["starttime"] = (DelayedStartTime.epoch2 ? DelayedStartTime.epoch2 + EPOCH2_OFFSET : 0);
            } else
                DelayedStartTime.epoch2 = DELAYEDSTARTTIME;


            switch(mode.toInt()) {
                case 0: // OFF
                    ToModemWaitStateTimer = 0;
                    ToModemDoneStateTimer = 0;
                    LeaveModemDoneStateTimer = 0;
                    LeaveModemDeniedStateTimer = 0;
                    setAccess(0);
                    break;
                case 1:
                    setMode(MODE_NORMAL);
                    break;
                case 2:
                    setMode(MODE_SOLAR);
                    break;
                case 3:
                    setMode(MODE_SMART);
                    break;
                default:
                    mode = "Value not allowed!";
            }
            doc["mode"] = mode;
        }

        if(request->hasParam("enable_C2")) {
            EnableC2 = (EnableC2_t) request->getParam("enable_C2")->value().toInt();
            write_settings();
            doc["settings"]["enable_C2"] = StrEnableC2[EnableC2];
        }

        if(request->hasParam("stop_timer")) {
            int stop_timer = request->getParam("stop_timer")->value().toInt();

            if(stop_timer >= 0 && stop_timer <= 60) {
                StopTime = stop_timer;
                doc["stop_timer"] = true;
                write_settings();
            } else {
                doc["stop_timer"] = false;
            }

        }

        if(Mode == MODE_NORMAL || Mode == MODE_SMART) {
            if(request->hasParam("override_current")) {
                int current = request->getParam("override_current")->value().toInt();
                if (LoadBl < 2 && (current == 0 || (current >= ( MinCurrent * 10 ) && current <= ( MaxCurrent * 10 )))) { //OverrideCurrent not possible on Slave
                    OverrideCurrent = current;
                    doc["override_current"] = OverrideCurrent;
                } else {
                    doc["override_current"] = "Value not allowed!";
                }
            }
        }

        if(request->hasParam("solar_start_current")) {
            int current = request->getParam("solar_start_current")->value().toInt();
            if(current >= 0 && current <= 48) {
                StartCurrent = current;
                doc["solar_start_current"] = StartCurrent;
                write_settings();
            } else {
                doc["solar_start_current"] = "Value not allowed!";
            }
        }

        if(request->hasParam("solar_max_import")) {
            int current = request->getParam("solar_max_import")->value().toInt();
            if(current >= 0 && current <= 48) {
                ImportCurrent = current;
                doc["solar_max_import"] = ImportCurrent;
                write_settings();
            } else {
                doc["solar_max_import"] = "Value not allowed!";
            }
        }

        //special section to post stuff for experimenting with an ISO15118 modem
        if(request->hasParam("override_pwm")) {
            int pwm = request->getParam("override_pwm")->value().toInt();
            if (pwm == 0){
                CP_OFF;
                CPDutyOverride = true;
            } else if (pwm < 0){
                CP_ON;
                CPDutyOverride = false;
                pwm = 100; // 10% until next loop, to be safe, corresponds to 6A
            } else{
                CP_ON;
                CPDutyOverride = true;
            }

            SetCPDuty(pwm);
            doc["override_pwm"] = pwm;
        }

        //allow basic plug 'n charge based on evccid
        //if required_evccid is set to a value, SmartEVSE will only allow charging requests from said EVCCID
        if(request->hasParam("required_evccid")) {
            if (request->getParam("required_evccid")->value().length() <= 32) {
                strncpy(RequiredEVCCID, request->getParam("required_evccid")->value().c_str(), sizeof(RequiredEVCCID));
                doc["required_evccid"] = RequiredEVCCID;
                write_settings();
            } else {
                doc["required_evccid"] = "EVCCID too long (max 32 char)";
            }
        }

#if MQTT
        if(request->hasParam("mqtt_update")) {
            if (request->getParam("mqtt_update")->value().toInt() == 1) {

                if(request->hasParam("mqtt_host")) {
                    MQTTHost = request->getParam("mqtt_host")->value();
                    doc["mqtt_host"] = MQTTHost;
                }

                if(request->hasParam("mqtt_port")) {
                    MQTTPort = request->getParam("mqtt_port")->value().toInt();
                    if (MQTTPort == 0) MQTTPort = 1883;
                    doc["mqtt_port"] = MQTTPort;
                }

                if(request->hasParam("mqtt_topic_prefix")) {
                    MQTTprefix = request->getParam("mqtt_topic_prefix")->value();
                    if (!MQTTprefix || MQTTprefix == "") {
                        MQTTprefix = APhostname;
                    }
                    doc["mqtt_topic_prefix"] = MQTTprefix;
                }

                if(request->hasParam("mqtt_username")) {
                    MQTTuser = request->getParam("mqtt_username")->value();
                    if (!MQTTuser || MQTTuser == "") {
                        MQTTuser.clear();
                    }
                    doc["mqtt_username"] = MQTTuser;
                }

                if(request->hasParam("mqtt_password")) {
                    MQTTpassword = request->getParam("mqtt_password")->value();
                    if (!MQTTpassword || MQTTpassword == "") {
                        MQTTpassword.clear();
                    }
                    doc["mqtt_password_set"] = (MQTTpassword != "");
                }
                write_settings();
            }
        }
#endif

#if ENABLE_OCPP
        if(request->hasParam("ocpp_update")) {
            if (request->getParam("ocpp_update")->value().toInt() == 1) {

                if(request->hasParam("ocpp_mode")) {
                    OcppMode = request->getParam("ocpp_mode")->value().toInt();
                    doc["ocpp_mode"] = OcppMode;
                }

                if(request->hasParam("ocpp_backend_url")) {
                    if (OcppWsClient) {
                        OcppWsClient->setBackendUrl(request->getParam("ocpp_backend_url")->value().c_str());
                        doc["ocpp_backend_url"] = OcppWsClient->getBackendUrl();
                    } else {
                        doc["ocpp_backend_url"] = "Can only update when OCPP enabled";
                    }
                }

                if(request->hasParam("ocpp_cb_id")) {
                    if (OcppWsClient) {
                        OcppWsClient->setChargeBoxId(request->getParam("ocpp_cb_id")->value().c_str());
                        doc["ocpp_cb_id"] = OcppWsClient->getChargeBoxId();
                    } else {
                        doc["ocpp_cb_id"] = "Can only update when OCPP enabled";
                    }
                }

                if(request->hasParam("ocpp_auth_key")) {
                    if (OcppWsClient) {
                        OcppWsClient->setAuthKey(request->getParam("ocpp_auth_key")->value().c_str());
                        doc["ocpp_auth_key"] = OcppWsClient->getAuthKey();
                    } else {
                        doc["ocpp_auth_key"] = "Can only update when OCPP enabled";
                    }
                }

                if(request->hasParam("ocpp_auto_auth")) {
                    auto freevendMode = MicroOcpp::getConfigurationPublic(MO_CONFIG_EXT_PREFIX "FreeVendActive");
                    if (freevendMode) {
                        freevendMode->setBool(request->getParam("ocpp_auto_auth")->value().toInt());
                        doc["ocpp_auto_auth"] = freevendMode->getBool() ? 1 : 0;
                    } else {
                        doc["ocpp_auto_auth"] = "Can only update when OCPP enabled";
                    }
                }

                if(request->hasParam("ocpp_auto_auth_idtag")) {
                    auto freevendIdTag = MicroOcpp::getConfigurationPublic(MO_CONFIG_EXT_PREFIX "FreeVendIdTag");
                    if (freevendIdTag) {
                        freevendIdTag->setString(request->getParam("ocpp_auto_auth_idtag")->value().c_str());
                        doc["ocpp_auto_auth_idtag"] = freevendIdTag->getString();
                    } else {
                        doc["ocpp_auto_auth_idtag"] = "Can only update when OCPP enabled";
                    }
                }

                // Apply changes in OcppWsClient
                if (OcppWsClient) {
                    OcppWsClient->reloadConfigs();
                }
                MicroOcpp::configuration_save();
                write_settings();
            }
        }
#endif //ENABLE_OCPP

        String json;
        serializeJson(doc, json);
        mg_http_reply(c, 200, "Content-Type: application/json\r\n", "%s\n", json.c_str());    // Yes. Respond JSON
      } else {
        mg_http_reply(c, 404, "", "Not Found\n");
      }
    } else if (mg_http_match_uri(hm, "/currents") && !memcmp("POST", hm->method.buf, hm->method.len)) {
        DynamicJsonDocument doc(200);

        if(request->hasParam("battery_current")) {
            if (LoadBl < 2) {
                homeBatteryCurrent = request->getParam("battery_current")->value().toInt();
                homeBatteryLastUpdate = time(NULL);
                doc["battery_current"] = homeBatteryCurrent;
            } else
                doc["battery_current"] = "not allowed on slave";
        }

        if(MainsMeter.Type == EM_API) {
            if(request->hasParam("L1") && request->hasParam("L2") && request->hasParam("L3")) {
                if (LoadBl < 2) {
                    MainsMeter.Irms[0] = request->getParam("L1")->value().toInt();
                    MainsMeter.Irms[1] = request->getParam("L2")->value().toInt();
                    MainsMeter.Irms[2] = request->getParam("L3")->value().toInt();

                    CalcIsum();
                    for (int x = 0; x < 3; x++) {
                        doc["original"]["L" + x] = IrmsOriginal[x];
                        doc["L" + x] = MainsMeter.Irms[x];
                    }
                    doc["TOTAL"] = Isum;

                    MainsMeter.Timeout = COMM_TIMEOUT;

                } else
                    doc["TOTAL"] = "not allowed on slave";
            }
        }

        String json;
        serializeJson(doc, json);
        mg_http_reply(c, 200, "Content-Type: application/json\r\n", "%s\r\n", json.c_str());    // Yes. Respond JSON

    } else if (mg_http_match_uri(hm, "/ev_meter") && !memcmp("POST", hm->method.buf, hm->method.len)) {
        DynamicJsonDocument doc(200);

        if(EVMeter.Type == EM_API) {
            if(request->hasParam("L1") && request->hasParam("L2") && request->hasParam("L3")) {

                EVMeter.Irms[0] = request->getParam("L1")->value().toInt();
                EVMeter.Irms[1] = request->getParam("L2")->value().toInt();
                EVMeter.Irms[2] = request->getParam("L3")->value().toInt();
                EVMeter.CalcImeasured();
                EVMeter.Timeout = COMM_EVTIMEOUT;
                for (int x = 0; x < 3; x++)
                    doc["ev_meter"]["currents"]["L" + x] = EVMeter.Irms[x];
                doc["ev_meter"]["currents"]["TOTAL"] = EVMeter.Irms[0] + EVMeter.Irms[1] + EVMeter.Irms[2];
            }

            if(request->hasParam("import_active_energy") && request->hasParam("export_active_energy") && request->hasParam("import_active_power")) {

                EVMeter.Import_active_energy = request->getParam("import_active_energy")->value().toInt();
                EVMeter.Export_active_energy = request->getParam("export_active_energy")->value().toInt();

                EVMeter.PowerMeasured = request->getParam("import_active_power")->value().toInt();
                EVMeter.UpdateEnergies();
                doc["ev_meter"]["import_active_power"] = EVMeter.PowerMeasured;
                doc["ev_meter"]["import_active_energy"] = EVMeter.Import_active_energy;
                doc["ev_meter"]["export_active_energy"] = EVMeter.Export_active_energy;
                doc["ev_meter"]["total_kwh"] = EVMeter.Energy;
                doc["ev_meter"]["charged_kwh"] = EVMeter.EnergyCharged;
            }
        }

        String json;
        serializeJson(doc, json);
        mg_http_reply(c, 200, "Content-Type: application/json\r\n", "%s\r\n", json.c_str());    // Yes. Respond JSON

    } else if (mg_http_match_uri(hm, "/reboot") && !memcmp("POST", hm->method.buf, hm->method.len)) {
        DynamicJsonDocument doc(20);

        ESP.restart();
        doc["reboot"] = true;

        String json;
        serializeJson(doc, json);
        mg_http_reply(c, 200, "Content-Type: application/json\r\n", "%s\r\n", json.c_str());    // Yes. Respond JSON

#if MODEM
    } else if (mg_http_match_uri(hm, "/ev_state") && !memcmp("POST", hm->method.buf, hm->method.len)) {
        DynamicJsonDocument doc(200);

        //State of charge posting
        int current_soc = request->getParam("current_soc")->value().toInt();
        int full_soc = request->getParam("full_soc")->value().toInt();

        // Energy requested by car
        int energy_request = request->getParam("energy_request")->value().toInt();

        // Total energy capacity of car's battery
        int energy_capacity = request->getParam("energy_capacity")->value().toInt();

        // Update EVCCID of car
        if (request->hasParam("evccid")) {
            if (request->getParam("evccid")->value().length() <= 32) {
                strncpy(EVCCID, request->getParam("evccid")->value().c_str(), sizeof(EVCCID));
                doc["evccid"] = EVCCID;
            }
        }

        if (full_soc >= FullSoC) // Only update if we received it, since sometimes it's there, sometimes it's not
            FullSoC = full_soc;

        if (energy_capacity >= EnergyCapacity) // Only update if we received it, since sometimes it's there, sometimes it's not
            EnergyCapacity = energy_capacity;

        if (energy_request >= EnergyRequest) // Only update if we received it, since sometimes it's there, sometimes it's not
            EnergyRequest = energy_request;

        if (current_soc >= 0 && current_soc <= 100) {
            // We set the InitialSoC for our own calculations
            InitialSoC = current_soc;

            // We also set the ComputedSoC to allow for app integrations
            ComputedSoC = current_soc;

            // Skip waiting, charge since we have what we've got
            if (State == STATE_MODEM_REQUEST || State == STATE_MODEM_WAIT || State == STATE_MODEM_DONE){
                _LOG_A("Received SoC via REST. Shortcut to State Modem Done\n");
                setState(STATE_MODEM_DONE); // Go to State B, which means in this case setting PWM
            }
        }

        RecomputeSoC();

        doc["current_soc"] = current_soc;
        doc["full_soc"] = full_soc;
        doc["energy_capacity"] = energy_capacity;
        doc["energy_request"] = energy_request;

        String json;
        serializeJson(doc, json);
        mg_http_reply(c, 200, "Content-Type: application/json\r\n", "%s\r\n", json.c_str());    // Yes. Respond JSON
#endif

#if FAKE_RFID
    //this can be activated by: http://smartevse-xxx.lan/debug?showrfid=1
    } else if (mg_http_match_uri(hm, "/debug") && !memcmp("GET", hm->method.buf, hm->method.len)) {
        if(request->hasParam("showrfid")) {
            Show_RFID = strtol(request->getParam("showrfid")->value().c_str(),NULL,0);
        }
        _LOG_A("DEBUG: Show_RFID=%u.\n",Show_RFID);
        mg_http_reply(c, 200, "Content-Type: application/json\r\n", "%s\r\n", ""); //json request needs json response
#endif

#if AUTOMATED_TESTING
    //this can be activated by: http://smartevse-xxx.lan/automated_testing?current_max=100
    //WARNING: because of automated testing, no limitations here!
    //THAT IS DANGEROUS WHEN USED IN PRODUCTION ENVIRONMENT
    //FOR SMARTEVSE's IN A TESTING BENCH ONLY!!!!
    } else if (mg_http_match_uri(hm, "/automated_testing") && !memcmp("POST", hm->method.buf, hm->method.len)) {
        if(request->hasParam("current_max")) {
            MaxCurrent = strtol(request->getParam("current_max")->value().c_str(),NULL,0);
        }
        if(request->hasParam("current_main")) {
            MaxMains = strtol(request->getParam("current_main")->value().c_str(),NULL,0);
        }
        if(request->hasParam("current_max_circuit")) {
            MaxCircuit = strtol(request->getParam("current_max_circuit")->value().c_str(),NULL,0);
        }
        if(request->hasParam("mainsmeter")) {
            MainsMeter.Type = strtol(request->getParam("mainsmeter")->value().c_str(),NULL,0);
        }
        if(request->hasParam("evmeter")) {
            EVMeter.Type = strtol(request->getParam("evmeter")->value().c_str(),NULL,0);
        }
        if(request->hasParam("config")) {
            Config = strtol(request->getParam("config")->value().c_str(),NULL,0);
            setState(STATE_A);                                                  // so the new value will actually be read
        }
        if(request->hasParam("loadbl")) {
            int LBL = strtol(request->getParam("loadbl")->value().c_str(),NULL,0);
            ConfigureModbusMode(LBL);
            LoadBl = LBL;
        }
        mg_http_reply(c, 200, "Content-Type: application/json\r\n", "%s\r\n", ""); //json request needs json response
#endif
    } else {                                                                    // if everything else fails, serve static page
        struct mg_http_serve_opts opts = {.root_dir = "/data", .ssi_pattern = NULL, .extra_headers = NULL, .mime_types = NULL, .page404 = NULL, .fs = &mg_fs_packed };
        //opts.fs = NULL;
        mg_http_serve_dir(c, hm, &opts);
    }
    delete request;
  }
}
*/
void onWifiEvent(WiFiEvent_t event, WiFiEventInfo_t info) {
    switch (event) {
        case WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_GOT_IP:
#if LOG_LEVEL >= 1
            _LOG_A("Connected to AP: %s Local IP: %s\n", WiFi.SSID().c_str(), WiFi.localIP().toString().c_str());
#else
            Serial.printf("Connected to AP: %s Local IP: %s\n", WiFi.SSID().c_str(), WiFi.localIP().toString().c_str());
#endif            
            //load dhcp dns ip4 address into mongoose
            static char dns4url[]="udp://123.123.123.123:53";
            sprintf(dns4url, "udp://%s:53", WiFi.dnsIP().toString().c_str());
            mgr.dns4.url = dns4url;
/*            if (TZinfo == "") {
                setTimeZone();
            }*/

            // Start the mDNS responder so that the SmartEVSE can be accessed using a local hostame: http://SmartEVSE-xxxxxx.local
            if (!MDNS.begin(APhostname.c_str())) {
                _LOG_A("Error setting up MDNS responder!\n");
            } else {
                _LOG_A("mDNS responder started. http://%s.local\n",APhostname.c_str());
                MDNS.addService("http", "tcp", 80);   // announce Web server
            }

            break;
        case WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_CONNECTED:
            _LOG_A("Connected or reconnected to WiFi\n");

#if MQTT
            if (!MQTTtimer) {
               MQTTtimer = mg_timer_add(&mgr, 3000, MG_TIMER_REPEAT | MG_TIMER_RUN_NOW, timer_fn, &mgr);
            }
#endif
/*            mg_log_set(MG_LL_NONE);
            //mg_log_set(MG_LL_VERBOSE);

            if (!HttpListener80) {
                HttpListener80 = mg_http_listen(&mgr, "http://0.0.0.0:80", fn_http_server, NULL);  // Setup listener
            }
            if (!HttpListener443) {
                HttpListener443 = mg_http_listen(&mgr, "http://0.0.0.0:443", fn_http_server, (void *) 1);  // Setup listener
            }
            _LOG_A("HTTP server started\n");
*/
#if DBG == 1
            // if we start RemoteDebug with no wifi credentials installed we get in a bootloop
            // so we start it here
            // Initialize the server (telnet or web socket) of RemoteDebug
            Debug.begin(APhostname, 23, 1);
            Debug.showColors(true); // Colors
#endif
            break;
        case WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
            if (WIFImode == 1) {
#if MQTT
                //mg_timer_free(&mgr);
#endif
                WiFi.reconnect();                                               // recommended reconnection strategy by ESP-IDF manual
            }
            break;
        // for some reason this is not necessary in the SmartEVSEv3 code, but it is for Sensorbox v2:
        case ARDUINO_EVENT_SC_GOT_SSID_PSWD:
        {
            Serial.println("Got SSID and password");

            uint8_t ssid[33] = { 0 };
            uint8_t password[65] = { 0 };

            uint8_t rvd_data[33] = { 0 };

            memcpy(ssid, info.sc_got_ssid_pswd.ssid, sizeof(info.sc_got_ssid_pswd.ssid));
            memcpy(password, info.sc_got_ssid_pswd.password, sizeof(info.sc_got_ssid_pswd.password));

/*          Serial.printf("SSID:%s\n", ssid);
            Serial.printf("PASSWORD:%s\n", password);

            if (info.sc_got_ssid_pswd.type == SC_TYPE_ESPTOUCH_V2) {
                ESP_ERROR_CHECK( esp_smartconfig_get_rvd_data(rvd_data, sizeof(rvd_data)) );

                Serial.println("RVD_DATA");
                Serial.write(rvd_data, 33);
                Serial.printf("\n");

                for (int i = 0; i < 33; i++) {
                    Serial.printf("%02x ", rvd_data[i]);
                }
                Serial.printf("\n");
            }*/
            WiFi.begin((char*)ssid, (char *)password);
        }
        break;
        default: break;
  }
}

// turns out getLocalTime only checks if the current year > 2016, and if so, decides NTP must have synced;
// this callback function actually checks if we are synced!
void timeSyncCallback(struct timeval *tv)
{
    LocalTimeSet = true;
    _LOG_A("Synced clock to NTP server!");    // somehow adding a \n here hangs the telnet server after printing this message ?!?
}


void SetupPortalTask(void * parameter) {
    _LOG_A("Start Portal...\n");
    WiFi.disconnect(true);

#if SENSORBOX_VERSION == 20
    StopwebServer();
#else
    // Close Mongoose HTTP Server
    if (HttpListener80) {
        HttpListener80->is_closing = 1;
    }
    if (HttpListener443) {
        HttpListener443->is_closing = 1;
    }

    while (HttpListener80 || HttpListener443) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        _LOG_A("Waiting for Mongoose Server to terminate\n");
    }
#endif
/*    WiFi.mode(WIFI_STA);
    WiFi.begin("iot_nomap", "goodbye:saidEASTERBUNNY");
    delay(1000);*/

    //Init WiFi as Station, start SmartConfig
    WiFi.mode(WIFI_AP_STA);
    WiFi.beginSmartConfig(SC_TYPE_ESPTOUCH_V2, SmartConfigKey);
 
    //Wait for SmartConfig packet from mobile.
    _LOG_V("Waiting for SmartConfig.\n");
    while (!WiFi.smartConfigDone() && (WIFImode == 2) && (WiFi.status() != WL_CONNECTED)) {
        // Also start Serial CLI for entering AP and password.
#if SENSORBOX_VERSION != 20                                                     // ProvisionCli does not work on Sensorbox
        ProvisionCli();
#endif
        delay(100);
    }                       // loop until connected or Wifi setup menu is exited.
    delay(2000);            // give smartConfig time to send provision status back to the users phone.
        
    if (WiFi.status() == WL_CONNECTED) {
        _LOG_V("\nWiFi Connected, IP Address:%s.\n", WiFi.localIP().toString().c_str());
        WIFImode = 1;
        write_settings();
#if SENSORBOX_VERSION != 20                                                     //so we are not on a sensorbox but on a smartevse
        LCDNav = 0;
#endif
    }  

    CliState = 0;
    WiFi.stopSmartConfig(); // this makes sure repeated SmartConfig calls are succesfull
#if SENSORBOX_VERSION == 20
    StartwebServer();
#endif
    vTaskDelete(NULL);                                                          //end this task so it will not take up resources
}


void handleWIFImode() {

    if (WIFImode == 2 && WiFi.getMode() != WIFI_AP_STA)
        //now start the portal in the background, so other tasks keep running
        xTaskCreate(
            SetupPortalTask,     // Function that should be called
            "SetupPortalTask",   // Name of the task (for debugging)
            10000,                // Stack size (bytes)                              // printf needs atleast 1kb
            NULL,                 // Parameter to pass
            1,                    // Task priority
            NULL                  // Task handleCTReceive
        );

    if (WIFImode == 1 && WiFi.getMode() == WIFI_OFF) {
        _LOG_A("Starting WiFi..\n");
        WiFi.mode(WIFI_STA);
        WiFi.begin();
    }    

    if (WIFImode == 0 && WiFi.getMode() != WIFI_OFF) {
        _LOG_A("Stopping WiFi..\n");
        WiFi.disconnect(true);
    }    
}

// Setup Wifi 
void WiFiSetup(void) {
    mg_mgr_init(&mgr);  // Initialise event manager

    WiFi.setAutoReconnect(false);                                                //actually does nothing since this is the default value
    //WiFi.setAutoReconnect(true);                                                //actually does nothing since this is the default value
    //WiFi.persistent(true);
    WiFi.onEvent(onWifiEvent);
//WIFImode = 2; //TODO for testing only
    handleWIFImode();                                                           //go into the mode that was saved in nonvolatile memory

    // Init and get the time
    // First option to get time from local ntp server blocks the second fallback option since 2021:
    // See https://github.com/espressif/arduino-esp32/issues/4964
    //sntp_servermode_dhcp(1);                                                    //try to get the ntp server from dhcp
    sntp_setservername(1, "europe.pool.ntp.org");                               //fallback server
    sntp_set_time_sync_notification_cb(timeSyncCallback);
    sntp_init();

    // Set random AES Key for SmartConfig provisioning, first 8 positions are 0
    // This key is displayed on the LCD, and should be entered when using the EspTouch app.
#if SENSORBOX_VERSION != 20
    for (uint8_t i=0; i<8 ;i++) {
        SmartConfigKey[i+8] = random(9) + '1';
    }
#else
    StartwebServer();
#endif
}


/*
 * OCPP-related function definitions
 */
#if ENABLE_OCPP

void ocppUpdateRfidReading(const unsigned char *uuid, size_t uuidLen) {
    if (!uuid || uuidLen > sizeof(OcppRfidUuid)) {
        _LOG_W("OCPP: invalid UUID\n");
        return;
    }
    memcpy(OcppRfidUuid, uuid, uuidLen);
    OcppRfidUuidLen = uuidLen;
    OcppLastRfidUpdate = millis();
}

bool ocppIsConnectorPlugged() {
    return OcppTrackCPvoltage >= PILOT_9V && OcppTrackCPvoltage <= PILOT_3V;
}

bool ocppHasTxNotification() {
    return OcppDefinedTxNotification && millis() - OcppLastTxNotification <= 3000;
}

MicroOcpp::TxNotification ocppGetTxNotification() {
    return OcppTrackTxNotification;
}

bool ocppLockingTxDefined() {
    return OcppLockingTx != nullptr;
}

void ocppInit() {

    //load OCPP library modules: Mongoose WS adapter and Core OCPP library

    auto filesystem = MicroOcpp::makeDefaultFilesystemAdapter(
            MicroOcpp::FilesystemOpt::Use_Mount_FormatOnFail // Enable FS access, mount LittleFS here, format data partition if necessary
            );

    OcppWsClient = new MicroOcpp::MOcppMongooseClient(
            &mgr,
            nullptr,    // OCPP backend URL (factory default)
            nullptr,    // ChargeBoxId (factory default)
            nullptr,    // WebSocket Basic Auth token (factory default)
            nullptr,    // CA cert (cert string must outlive WS client)
            filesystem);

    mocpp_initialize(
            *OcppWsClient, //WebSocket adapter for MicroOcpp
            ChargerCredentials("SmartEVSE", "Stegen Electronics", VERSION, String(serialnr).c_str(), NULL, (char *) EMConfig[EVMeter.Type].Desc),
            filesystem);

    //setup OCPP hardware bindings

    setEnergyMeterInput([] () { //Input of the electricity meter register in Wh
        return EVMeter.Energy;
    });

    setPowerMeterInput([] () { //Input of the power meter reading in W
        return EVMeter.PowerMeasured;
    });

    setConnectorPluggedInput([] () { //Input about if an EV is plugged to this EVSE
        return ocppIsConnectorPlugged();
    });

    setEvReadyInput([] () { //Input if EV is ready to charge (= J1772 State C)
        return OcppTrackCPvoltage >= PILOT_6V && OcppTrackCPvoltage <= PILOT_3V;
    });

    setEvseReadyInput([] () { //Input if EVSE allows charge (= PWM signal on)
        return GetCurrent() > 0; //PWM is enabled
    });

    addMeterValueInput([] () {
            return (float) (EVMeter.Irms[0] + EVMeter.Irms[1] + EVMeter.Irms[2]);
        },
        "Current.Import",
        "A");

    addMeterValueInput([] () {
            return (float) EVMeter.Irms[0];
        },
        "Current.Import",
        "A",
        nullptr, // Location defaults to "Outlet"
        "L1");

    addMeterValueInput([] () {
            return (float) EVMeter.Irms[1];
        },
        "Current.Import",
        "A",
        nullptr, // Location defaults to "Outlet"
        "L2");

    addMeterValueInput([] () {
            return (float) EVMeter.Irms[2];
        },
        "Current.Import",
        "A",
        nullptr, // Location defaults to "Outlet"
        "L3");

    addMeterValueInput([] () {
            return (float)GetCurrent() * 0.1f;
        },
        "Current.Offered",
        "A");

    addMeterValueInput([] () {
            return (float)TempEVSE;
        },
        "Temperature",
        "Celsius");

#if MODEM
        addMeterValueInput([] () {
                return (float)ComputedSoC;
            },
            "SoC",
            "Percent");
#endif

    addErrorCodeInput([] () {
        return (ErrorFlags & TEMP_HIGH) ? "HighTemperature" : (const char*)nullptr;
    });

    addErrorCodeInput([] () {
        return (ErrorFlags & RCM_TRIPPED) ? "GroundFailure" : (const char*)nullptr;
    });

    addErrorDataInput([] () -> MicroOcpp::ErrorData {
        if (ErrorFlags & CT_NOCOMM) {
            MicroOcpp::ErrorData error = "PowerMeterFailure";
            error.info = "Communication with mains meter lost";
            return error;
        }
        return nullptr;
    });

    addErrorDataInput([] () -> MicroOcpp::ErrorData {
        if (ErrorFlags & EV_NOCOMM) {
            MicroOcpp::ErrorData error = "PowerMeterFailure";
            error.info = "Communication with EV meter lost";
            return error;
        }
        return nullptr;
    });

    // If SmartEVSE load balancer is turned off, then enable OCPP Smart Charging
    // This means after toggling LB, OCPP must be disabled and enabled for changes to become effective
    if (!LoadBl) {
        setSmartChargingCurrentOutput([] (float currentLimit) {
            OcppCurrentLimit = currentLimit; // Can be negative which means that no limit is defined

            // Re-evaluate charge rate and apply
            if (!LoadBl) { // Execute only if LB is still disabled

                CalcBalancedCurrent(0);
                if (IsCurrentAvailable()) {
                    // OCPP is the exclusive LB, clear LESS_6A error if set
                    ErrorFlags &= ~LESS_6A;
                    ChargeDelay = 0;
                }
                if ((State == STATE_B || State == STATE_C) && !CPDutyOverride) {
                    if (IsCurrentAvailable()) {
                        SetCurrent(ChargeCurrent);
                    } else {
                        setStatePowerUnavailable();
                    }
                }
            }
        });
    }

    setOnUnlockConnectorInOut([] () -> UnlockConnectorResult {
        // MO also stops transaction which should toggle OcppForcesLock false
        OcppLockingTx.reset();
        if (Lock == 0 || digitalRead(PIN_LOCK_IN) == lock2) {
            // Success
            return UnlockConnectorResult_Unlocked;
        }

        // No result yet, wait (MO eventually times out)
        return UnlockConnectorResult_Pending;
    });

    setOccupiedInput([] () -> bool {
        // Keep Finishing state while LockingTx effectively blocks new transactions
        return OcppLockingTx != nullptr;
    });

    setStopTxReadyInput([] () {
        // Stop value synchronization: block StopTransaction for 5 seconds to give the Modbus readings some time to come through
        return millis() - OcppStopReadingSyncTime >= 5000;
    });

    setTxNotificationOutput([] (MicroOcpp::Transaction*, MicroOcpp::TxNotification event) {
        OcppDefinedTxNotification = true;
        OcppTrackTxNotification = event;
        OcppLastTxNotification = millis();
    });

    OcppUnlockConnectorOnEVSideDisconnect = MicroOcpp::declareConfiguration<bool>("UnlockConnectorOnEVSideDisconnect", true);

    endTransaction(nullptr, "PowerLoss"); // If a transaction from previous power cycle is still running, abort it here
}

void ocppDeinit() {

    // Record stop value for transaction manually (normally MO would wait until `mocpp_loop()`, but that's too late here)
    if (auto& tx = getTransaction()) {
        if (tx->getMeterStop() < 0) {
            // Stop value not defined yet
            tx->setMeterStop(EVMeter.Import_active_energy); // Use same reading as in `setEnergyMeterInput()`
            tx->setStopTimestamp(getOcppContext()->getModel().getClock().now());
        }
    }

    endTransaction(nullptr, "Other"); // If a transaction is running, shut it down forcefully. The StopTx request will be sent when OCPP runs again.

    OcppUnlockConnectorOnEVSideDisconnect.reset();
    OcppLockingTx.reset();
    OcppForcesLock = false;

    if (OcppTrackPermitsCharge) {
        _LOG_A("OCPP unset Access_bit\n");
        setAccess(false);
    }

    OcppTrackPermitsCharge = false;
    OcppTrackAccessBit = false;
    OcppTrackCPvoltage = PILOT_NOK;
    OcppCurrentLimit = -1.f;

    mocpp_deinitialize();

    delete OcppWsClient;
    OcppWsClient = nullptr;
}

void ocppLoop() {

    // Update pilot tracking variable (last measured positive part)
    auto pilot = Pilot();
    if (pilot >= PILOT_12V && pilot <= PILOT_3V) {
        OcppTrackCPvoltage = pilot;
    }

    mocpp_loop();

    //handle RFID input

    if (OcppTrackLastRfidUpdate != OcppLastRfidUpdate) {
        // New RFID card swiped

        char uuidHex [2 * sizeof(OcppRfidUuid) + 1];
        uuidHex[0] = '\0';
        for (size_t i = 0; i < OcppRfidUuidLen; i++) {
            snprintf(uuidHex + 2*i, 3, "%02X", OcppRfidUuid[i]);
        }

        if (OcppLockingTx) {
            // Connector is still locked by earlier transaction

            if (!strcmp(uuidHex, OcppLockingTx->getIdTag())) {
                // Connector can be unlocked again
                OcppLockingTx.reset();
                endTransaction(uuidHex, "Local");
            } // else: Connector remains blocked for now
        } else if (getTransaction()) {
            //OCPP lib still has transaction (i.e. transaction running or authorization pending) --> swiping card again invalidates idTag
            endTransaction(uuidHex, "Local");
        } else {
            //OCPP lib has no idTag --> swiped card is used for new transaction
            OcppLockingTx = beginTransaction(uuidHex);
        }
    }
    OcppTrackLastRfidUpdate = OcppLastRfidUpdate;

    // Set / unset Access_bit
    // Allow to set Access_bit only once per OCPP transaction because other modules may override the Access_bit
    // Doesn't apply if SmartEVSE built-in RFID store is enabled
    if (RFIDReader == 6 || RFIDReader == 0) {
        // RFID reader in OCPP mode or RFID fully disabled - OCPP controls Access_bit
        if (!OcppTrackPermitsCharge && ocppPermitsCharge()) {
            _LOG_A("OCPP set Access_bit\n");
            setAccess(true);
        } else if (Access_bit && !ocppPermitsCharge()) {
            _LOG_A("OCPP unset Access_bit\n");
            setAccess(false);
        }
        OcppTrackPermitsCharge = ocppPermitsCharge();

        // Check if OCPP charge permission has been revoked by other module
        if (OcppTrackPermitsCharge && // OCPP has set Acess_bit and still allows charge
                !Access_bit) { // Access_bit is not active anymore
            endTransaction(nullptr, "Other");
        }
    } else {
        // Built-in RFID store enabled - OCPP does not control Access_bit, but starts transactions when Access_bit is set
        if (Access_bit && !OcppTrackAccessBit && !getTransaction() && isOperative()) {
            // Access_bit has been set
            OcppTrackAccessBit = true;
            _LOG_A("OCPP detected Access_bit set\n");
            char buf[13];
            sprintf(buf, "%02X%02X%02X%02X%02X%02X", RFID[1], RFID[2], RFID[3], RFID[4], RFID[5], RFID[6]);
            beginTransaction_authorized(buf);
        } else if (!Access_bit && (OcppTrackAccessBit || (getTransaction() && getTransaction()->isActive()))) {
            OcppTrackAccessBit = false;
            _LOG_A("OCPP detected Access_bit unset\n");
            char buf[13];
            sprintf(buf, "%02X%02X%02X%02X%02X%02X", RFID[1], RFID[2], RFID[3], RFID[4], RFID[5], RFID[6]);
            endTransaction_authorized(buf);
        }
    }

    // Stop value synchronization: block StopTransaction for a short period as long as charging is permitted
    if (ocppPermitsCharge()) {
        OcppStopReadingSyncTime = millis();
    }

    auto& transaction = getTransaction(); // Common tx which OCPP is currently processing (or nullptr if no tx is ongoing)

    // Check if Locking Tx has been invalidated by something other than RFID swipe
    if (OcppLockingTx) {
        if (OcppUnlockConnectorOnEVSideDisconnect->getBool() && !OcppLockingTx->isActive()) {
            // No LockingTx mode configured (still, keep LockingTx until end of transaction because the config could be changed in the middle of tx)
            OcppLockingTx.reset();
        } else if (OcppLockingTx->isAborted()) {
            // LockingTx hasn't successfully started
            OcppLockingTx.reset();
        } else if (transaction && transaction != OcppLockingTx) {
            // Another Tx has already started
            OcppLockingTx.reset();
        } else if (digitalRead(PIN_LOCK_IN) == lock2 && !OcppLockingTx->isActive()) {
            // Connector is has been unlocked and LockingTx has already run
            OcppLockingTx.reset();
        } // There may be further edge cases
    }

    OcppForcesLock = false;

    if (transaction && transaction->isAuthorized() && (transaction->isActive() || transaction->isRunning()) && // Common tx ongoing
            (OcppTrackCPvoltage >= PILOT_9V && OcppTrackCPvoltage <= PILOT_3V)) { // Connector plugged
        OcppForcesLock = true;
    }

    if (OcppLockingTx && OcppLockingTx->getStartSync().isRequested()) { // LockingTx goes beyond tx completion
        OcppForcesLock = true;
    }

}
#endif //ENABLE_OCPP


void setup2() {

    // overwrite APhostname if serialnr is programmed
    APhostname = "SmartEVSE-" + String( serialnr & 0xffff, 10);                 // SmartEVSE access point Name = SmartEVSE-xxxxx
    WiFi.setHostname(APhostname.c_str());

    // Setup WiFi, webserver and firmware OTA
    // Please be aware that after doing a OTA update, its possible that the active partition is set to OTA1.
    // Uploading a new firmware through USB will however update OTA0, and you will not notice any changes...
    WiFiSetup();

    firmwareUpdateTimer = random(FW_UPDATE_DELAY, 0xffff);
    //firmwareUpdateTimer = random(FW_UPDATE_DELAY, 120); // DINGO TODO debug max 2 minutes
}
/*
// returns true if current and latest version can be detected correctly and if the latest version is newer then current
// this means that ANY home compiled version, which has version format "11:20:03@Jun 17 2024", will NEVER be automatically updated!!
// same goes for current version with an -RC extension: this will NEVER be automatically updated!
// same goes for latest version with an -RC extension: this will NEVER be automatically updated! This situation should never occur since
// we only update from the "stable" repo !!
bool fwNeedsUpdate(char * version) {
    // version NEEDS to be in the format: vx.y.z[-RCa] where x, y, z, a are digits, multiple digits are allowed.
    // valid versions are v3.6.10   v3.17.0-RC13
    int latest_major, latest_minor, latest_patch, latest_rc, cur_major, cur_minor, cur_patch, cur_rc;
    int hit = sscanf(version, "v%i.%i.%i-RC%i", &latest_major, &latest_minor, &latest_patch, &latest_rc);
    _LOG_A("Firmware version detection hit=%i, LATEST version detected=v%i.%i.%i-RC%i.\n", hit, latest_major, latest_minor, latest_patch, latest_rc);
    int hit2 = sscanf(VERSION, "v%i.%i.%i-RC%i", &cur_major, &cur_minor, &cur_patch, &cur_rc);
    _LOG_A("Firmware version detection hit=%i, CURRENT version detected=v%i.%i.%i-RC%i.\n", hit2, cur_major, cur_minor, cur_patch, cur_rc);
    if (hit != 3 || hit2 != 3)                                                  // we couldnt detect simple vx.y.z version nrs, either current or latest
        return false;
    if (cur_major > latest_major)
        return false;
    if (cur_major < latest_major)
        return true;
    if (cur_major == latest_major) {
        if (cur_minor > latest_minor)
            return false;
        if (cur_minor < latest_minor)
            return true;
        if (cur_minor == latest_minor)
            return (cur_patch < latest_patch);
    }
    return false;
}
*/
void loop2() {

    static unsigned long lastCheck = 0;
    if (millis() - lastCheck >= 1000) {
        lastCheck = millis();
        //this block is for non-time critical stuff that needs to run approx 1 / second
/*
        //_LOG_A("DINGO: firmwareUpdateTimer just before decrement=%i.\n", firmwareUpdateTimer);
        if (AutoUpdate && !shouldReboot) {                                      // we don't want to autoupdate if we are on the verge of rebooting
            firmwareUpdateTimer--;
            char version[32];
            if (firmwareUpdateTimer == FW_UPDATE_DELAY) {                       // we now have to check for a new version
                //timer is not reset, proceeds to 65535 which is approx 18h from now
                if (getLatestVersion(String(String(OWNER_FACT) + "/" + String(REPO_FACT)), "", version)) {
                    if (fwNeedsUpdate(version)) {
                        _LOG_A("Firmware reports it needs updating, will update in %i seconds\n", FW_UPDATE_DELAY);
                        asprintf(&downloadUrl, "%s/fact_firmware.signed.bin", FW_DOWNLOAD_PATH); //will be freed in FirmwareUpdate() ; format: http://s3.com/fact_firmware.debug.signed.bin
                    } else {
                        _LOG_A("Firmware reports it needs NO update!\n");
                        firmwareUpdateTimer = random(FW_UPDATE_DELAY + 36000, 0xffff);  // at least 10 hours in between checks
                    }
                }
            } else if (firmwareUpdateTimer == 0) {                              // time to download & flash!
                if (getLatestVersion(String(String(OWNER_FACT) + "/" + String(REPO_FACT)), "", version)) { // recheck version info
                    if (fwNeedsUpdate(version)) {
                        _LOG_A("Firmware reports it needs updating, starting update NOW!\n");
                        asprintf(&downloadUrl, "%s/fact_firmware.signed.bin", FW_DOWNLOAD_PATH); //will be freed in FirmwareUpdate() ; format: http://s3.com/fact_firmware.debug.signed.bin
                        RunFirmwareUpdate();
                    } else {
                        _LOG_A("Firmware changed its mind, NOW it reports it needs NO update!\n");
                    }
                    //note: the firmwareUpdateTimer will decrement to 65535s so next check will be in 18hours or so....
                }
            }
        } // AutoUpdate */
        /////end of non-time critical stuff
    }

    mg_mgr_poll(&mgr, 100);                                                     // TODO increase this parameter to up to 1000 to make loop() less greedy

    //OCPP lifecycle management
#if ENABLE_OCPP
    if (OcppMode && !getOcppContext()) {
        ocppInit();
    } else if (!OcppMode && getOcppContext()) {
        ocppDeinit();
    }

    if (OcppMode) {
        ocppLoop();
    }
#endif //ENABLE_OCPP

#ifndef DEBUG_DISABLED
    // Remote debug over WiFi
    Debug.handle();
#endif

}