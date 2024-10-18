#include "main.h"
#include <WiFi.h>
//#include "esp_ota_ops.h"
#include "mbedtls/md_internal.h"
#include "radix.h"
#include "network.h"

#include <HTTPClient.h>
#include <ESPmDNS.h>
#include <Update.h>
#include <ArduinoJson.h>
#include <Preferences.h>

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

mg_connection *HttpListener80, *HttpListener443;

bool shouldReboot = false;

extern void write_settings(void);
extern void StopwebServer(void); //TODO or move over to network.cpp?
extern void StartwebServer(void); //TODO or move over to network.cpp?
extern bool handle_URI(struct mg_connection *c, struct mg_http_message *hm,  webServerRequest* request);

extern uint32_t serialnr;
extern Preferences preferences;


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
void ProvisionCli(HardwareSerial &s) {
    // SSID and PW for your Router
    static String Router_SSID, Router_Pass;
    static char CliBuffer[64];
    static uint8_t idx = 0;
    static bool entered = false;
    char ch;

    if (CliState == 0) {
        s.println("Enter WiFi access point name:");
        CliState++;

    } else if (CliState == 1 && entered) {
        Router_SSID = String(CliBuffer);
        Router_SSID.trim();
        if (!isValidInput(Router_SSID)) {
            s.println("Invalid characters in SSID.");
            Router_SSID = "";
            CliState = 0;
        } else CliState++;              // All OK, now request password.
        idx = 0;
        entered = false;

    } else if (CliState == 2) {
        s.println("Enter WiFi password:");
        CliState++;

    } else if (CliState == 3 && entered) {
        Router_Pass = String(CliBuffer);
        Router_Pass.trim();
        if (idx < 8) {
            s.println("Password should be min 8 characters.");
            Router_Pass = "";
            CliState = 2;
        } else CliState++;             // All OK
        idx = 0;
        entered = false;

    } else if (CliState == 4) {
        s.println("WiFi credentials stored.");
        WiFi.mode(WIFI_STA);                // Set Station Mode
        WiFi.begin(Router_SSID, Router_Pass);   // Configure Wifi with credentials
        CliState++;
    }


    // read input, and store in buffer until we read a \n
    while (s.available()) {
        ch = s.read();

        // When entering a password, replace last character with a *
        if (CliState == 3 && idx) s.printf("\b*");
        s.print(ch);

        // check for CR/LF, and make sure the contents of the buffer is atleast 1 character
        if (ch == '\n' || ch == '\r') {
            if (idx) {
                CliBuffer[idx] = 0;         // null terminate
                entered = true;
            } else if (CliState == 1 || CliState == 3) CliState--; // Reprint the last message
        } else if (idx < 63) {              // Store in buffer
            if (ch == '\b' && idx) {
                idx--;
                s.print(" \b");        // erase character from terminal
            } else {
                CliBuffer[idx++] = ch;
            }
        }
    }
}


#if MQTT
void mqtt_receive_callback(const String topic, const String payload) {
    // Make sure MQTT updates directly to prevent debounces
    lastMqttUpdate = 10;
}


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
//    if (MainsMeter.Type) {
        announce("Mains Current L1", "sensor");
        announce("Mains Current L2", "sensor");
        announce("Mains Current L3", "sensor");
//    }
/*    if (EVMeter.Type) {
        announce("EV Current L1", "sensor");
        announce("EV Current L2", "sensor");
        announce("EV Current L3", "sensor");
    }*/
/*
    if (EVMeter.Type) {
        //set the parameters for and announce other sensor entities:
        optional_payload = jsna("device_class","power") + jsna("unit_of_measurement","W");
        announce("EV Charge Power", "sensor");
        optional_payload = jsna("device_class","energy") + jsna("unit_of_measurement","Wh");
        announce("EV Energy Charged", "sensor");
        optional_payload = jsna("device_class","energy") + jsna("unit_of_measurement","Wh") + jsna("state_class","total_increasing");
        announce("EV Total Energy Charged", "sensor");
    }
*/

    //set the parameters for and announce diagnostic sensor entities:
    optional_payload = jsna("entity_category","diagnostic");
    announce("WiFi SSID", "sensor");
    announce("WiFi BSSID", "sensor");
    optional_payload = jsna("entity_category","diagnostic") + jsna("device_class","signal_strength") + jsna("unit_of_measurement","dBm");
    announce("WiFi RSSI", "sensor");
    optional_payload = jsna("entity_category","diagnostic") + jsna("device_class","duration") + jsna("unit_of_measurement","s") + jsna("entity_registry_enabled_default","False");
    announce("ESP Uptime", "sensor");

    MQTTclient.publish(MQTTprefix + "/WiFiSSID", String(WiFi.SSID()), true, 0);
    MQTTclient.publish(MQTTprefix + "/WiFiBSSID", String(WiFi.BSSIDstr()), true, 0);
}


#if MQTT_ESP == 1
/*
 * @brief Event handler registered to receive MQTT events
 *
 *  This function is called by the MQTT client event loop.
 *
 * @param handler_args user data registered to the event.
 * @param base Event base for the handler(always MQTT Base in this example).
 * @param event_id The id for the received event.
 * @param event_data The data for the event, esp_mqtt_event_handle_t.
 */
void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, esp_mqtt_event_t *event) {
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        MQTTclient.connected = true;
        SetupMQTTClient();
        break;
    case MQTT_EVENT_DISCONNECTED:
        MQTTclient.connected = false;
        break;
    case MQTT_EVENT_DATA:
        {
        String topic2 = String(event->topic).substring(0,event->topic_len);
        String payload2 = String(event->data).substring(0,event->data_len);
        //_LOG_A("Received MQTT EVENT DATA: topic=%s, payload=%s.\n", topic2.c_str(), payload2.c_str());
        mqtt_receive_callback(topic2, payload2);
        }
        break;
    case MQTT_EVENT_ERROR:
        _LOG_I("MQTT_EVENT_ERROR; Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));
        break;
    default:
        break;
    }
}


void MQTTclient_t::connect(void) {
    if (MQTTHost != "") {
        char s_mqtt_url[80];
        snprintf(s_mqtt_url, sizeof(s_mqtt_url), "mqtt://%s:%i", MQTTHost.c_str(), MQTTPort);
        String lwtTopic = MQTTprefix + "/connected";
        esp_mqtt_client_config_t mqtt_cfg = { .uri = s_mqtt_url, .client_id=MQTTprefix.c_str(), .username=MQTTuser.c_str(), .password=MQTTpassword.c_str(), .lwt_topic=lwtTopic.c_str(), .lwt_msg="offline", .lwt_qos=0, .lwt_retain=1, .lwt_msg_len=7, .keepalive=15 };
        MQTTclient.client = esp_mqtt_client_init(&mqtt_cfg);
        /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
        esp_mqtt_client_register_event(MQTTclient.client, (esp_mqtt_event_id_t) ESP_EVENT_ANY_ID, (esp_event_handler_t) mqtt_event_handler, NULL);
        if (WiFi.status() == WL_CONNECTED)
            esp_mqtt_client_start(MQTTclient.client);
    }
}
#endif


//wrapper so MQTTClient::Publish works
void MQTTclient_t::publish(const String &topic, const String &payload, bool retained, int qos) {
#if MQTT_ESP == 0
    if (s_conn && connected) {
        struct mg_mqtt_opts opts = default_opts;
        opts.topic = mg_str(topic.c_str());
        opts.message = mg_str(payload.c_str());
        opts.qos = qos;
        opts.retain = retained;
        mg_mqtt_pub(s_conn, &opts);
    }
#else
    //esp_mqtt_client_enqueue(client, topic.c_str(), payload.c_str(), payload.length(), qos, retained, 1);
    if (connected)
        esp_mqtt_client_publish(client, topic.c_str(), payload.c_str(), payload.length(), qos, retained);
#endif
}

void MQTTclient_t::subscribe(const String &topic, int qos) {
#if MQTT_ESP == 0
    if (s_conn && connected) {
        struct mg_mqtt_opts opts = default_opts;
        opts.topic = mg_str(topic.c_str());
        opts.qos = qos;
        mg_mqtt_sub(s_conn, &opts);
    }
#else
    if (connected)
        esp_mqtt_client_subscribe(client, topic.c_str(), qos);
#endif
}

MQTTclient_t MQTTclient;

#endif

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
    return false;*/
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
#ifndef SENSORBOX_VERSION
        _LOG_A("Firmware update succesfull; rebooting as soon as no EV is connected.\n");
#else
        _LOG_A("Firmware update succesfull; rebooting.\n");
#endif
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
        3,              // Task priority - medium
        NULL            // Task handle
    );
}


void setTimeZone(void * parameter) {
    HTTPClient httpClient;
    // lookup current timezone
    httpClient.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
    httpClient.begin("http://worldtimeapi.org/api/ip");
    int httpCode = httpClient.GET();  //Make the request

    // only handle 200/301, fail on everything else
    if( httpCode != HTTP_CODE_OK && httpCode != HTTP_CODE_MOVED_PERMANENTLY ) {
        _LOG_A("Error on HTTP request (httpCode=%i)\n", httpCode);
        httpClient.end();
        vTaskDelete(NULL);                                                          //end this task so it will not take up resources
    }

    // The filter: it contains "true" for each value we want to keep
    DynamicJsonDocument  filter(16);
    filter["timezone"] = true;
    DynamicJsonDocument doc2(80);
    DeserializationError error = deserializeJson(doc2, httpClient.getStream(), DeserializationOption::Filter(filter));
    httpClient.end();
    if (error) {
        _LOG_A("deserializeJson() failed: %s\n", error.c_str());
        vTaskDelete(NULL);                                                          //end this task so it will not take up resources
    }
    String tzname = doc2["timezone"];
    if (tzname == "") {
        _LOG_A("Could not detect Timezone.\n");
        vTaskDelete(NULL);                                                          //end this task so it will not take up resources
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
        vTaskDelete(NULL);                                                          //end this task so it will not take up resources
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
    vTaskDelete(NULL);                                                          //end this task so it will not take up resources
}


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

#if MQTT && MQTT_ESP == 0
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
        MQTTclient.s_conn = NULL;  // Mark that we're closed
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

    if (MQTTclient.s_conn == NULL) MQTTclient.s_conn = mg_mqtt_connect(mgr, s_mqtt_url, &opts, fn_mqtt, NULL);
}
#endif

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
    // handles URI and response, returns true if handled, false if not
    if (!handle_URI(c, hm, request)) {
        if (mg_match(hm->uri, mg_str("/erasesettings"), NULL)) {
            if ( preferences.begin("settings", false) ) {         // our own settings
              preferences.clear();
              preferences.end();
            }
            if (preferences.begin("nvs.net80211", false) ) {      // WiFi settings used by ESP
              preferences.clear();
              preferences.end();       
            }
            shouldReboot = true;
            mg_http_reply(c, 200, "Content-Type: text/plain\r\n", "Erasing settings, rebooting");
        } else if (mg_http_match_uri(hm, "/autoupdate")) {
            char owner[40];
            char buf[8];
            int debug;
            mg_http_get_var(&hm->query, "owner", owner, sizeof(owner));
            mg_http_get_var(&hm->query, "debug", buf, sizeof(buf));
            debug = strtol(buf, NULL, 0);
            if (!memcmp(owner, OWNER_FACT, sizeof(OWNER_FACT)) || (!memcmp(owner, OWNER_COMM, sizeof(OWNER_COMM)))) {
                asprintf(&downloadUrl, "%s/%s_sensorboxv2_firmware.%ssigned.bin", FW_DOWNLOAD_PATH, owner, debug ? "debug.": ""); //will be freed in FirmwareUpdate() ; format: http://s3.com/fact_sensorboxv2_firmware.debug.signed.bin
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
#ifndef SENSORBOX_VERSION
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
#else
                    mg_http_reply(c, 400, "", "only allowed to flash firmware.bin, firmware.debug.bin, firmware.signed.bin, firmware.debug.signed.bin");
#endif
                mg_http_reply(c, 200, "", "%ld", res);
            }
/*        } else if (mg_http_match_uri(hm, "/currents") && !memcmp("POST", hm->method.buf, hm->method.len)) {
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
    
    */    } else if (mg_http_match_uri(hm, "/reboot") && !memcmp("POST", hm->method.buf, hm->method.len)) {
            shouldReboot = true;
            mg_http_reply(c, 200, "", "Rebooting....");
          } else if (mg_http_match_uri(hm, "/settings") && !memcmp("POST", hm->method.buf, hm->method.len)) {
            DynamicJsonDocument doc(64);
#if MQTT
            if (request->hasParam("mqtt_update") && request->getParam("mqtt_update")->value().toInt() == 1) {

                if(request->hasParam("mqtt_host")) {
                    MQTTHost = request->getParam("mqtt_host")->value();
#if MQTT_ESP == 1
                    MQTTclient.connect();
#endif
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
                if (preferences.begin("settings", false) ) {
                    preferences.putString("MQTTpassword", MQTTpassword);
                    preferences.putString("MQTTuser", MQTTuser);
                    preferences.putString("MQTTprefix", MQTTprefix);
                    preferences.putString("MQTTHost", MQTTHost);
                    preferences.putUShort("MQTTPort", MQTTPort);
                    preferences.end();
                }
            }
#endif
            String json;
            serializeJson(doc, json);
            mg_http_reply(c, 200, "Content-Type: application/json\r\n", "%s\r\n", json.c_str());    // Yes. Respond JSON
    /*
    
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
    */    } else {                                                                    // if everything else fails, serve static page
            struct mg_http_serve_opts opts = {.root_dir = "/data", .ssi_pattern = NULL, .extra_headers = NULL, .mime_types = NULL, .page404 = NULL, .fs = &mg_fs_packed };
            //opts.fs = NULL;
            mg_http_serve_dir(c, hm, &opts);
        }
    } // handle_URI
    delete request;
  } //HTTP request received
}

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
            if (TZinfo == "") {
                xTaskCreate(
                    setTimeZone, // Function that should be called
                    "setTimeZone",// Name of the task (for debugging)
                    4096,           // Stack size (bytes)
                    NULL,           // Parameter to pass
                    1,              // Task priority - low
                    NULL            // Task handle
                );
            }

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
#if MQTT_ESP == 0
            if (!MQTTtimer) {
               MQTTtimer = mg_timer_add(&mgr, 3000, MG_TIMER_REPEAT | MG_TIMER_RUN_NOW, timer_fn, &mgr);
            }
#else
            if (MQTTHost != "")
                esp_mqtt_client_start(MQTTclient.client);
#endif
#endif //MQTT
            mg_log_set(MG_LL_NONE);
            //mg_log_set(MG_LL_VERBOSE);

            if (!HttpListener80) {
                HttpListener80 = mg_http_listen(&mgr, "http://0.0.0.0:80", fn_http_server, NULL);  // Setup listener
            }
            if (!HttpListener443) {
                HttpListener443 = mg_http_listen(&mgr, "http://0.0.0.0:443", fn_http_server, (void *) 1);  // Setup listener
            }
            _LOG_A("HTTP server started\n");

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
            _LOG_A("Got SSID and password.\n");

            uint8_t ssid[33] = { 0 };
            uint8_t password[65] = { 0 };
            memcpy(ssid, info.sc_got_ssid_pswd.ssid, sizeof(info.sc_got_ssid_pswd.ssid));
            memcpy(password, info.sc_got_ssid_pswd.password, sizeof(info.sc_got_ssid_pswd.password));
            WiFi.begin((char*)ssid, (char *)password);
        }
        break;
        default: break;                                                         // prevent compiler warnings
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
#ifdef SENSORBOX_VERSION
    HardwareSerial *s1 = *((HardwareSerial **)parameter);
    HardwareSerial &s = *s1;
#endif
    _LOG_A("Start Portal...\n");
    WiFi.disconnect(true);

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

    //Init WiFi as Station, start SmartConfig
    WiFi.mode(WIFI_AP_STA);
    WiFi.beginSmartConfig(SC_TYPE_ESPTOUCH_V2, SmartConfigKey);
 
    //Wait for SmartConfig packet from mobile.
    _LOG_V("Waiting for SmartConfig.\n");
#ifdef SENSORBOX_VERSION
    s.end();
    s.begin(115200, SERIAL_8N1, PIN_RXD, PIN_TXD, false);                    // Input from TX of PIC, and debug output to USB
#endif
    unsigned long configTimer = millis();
    while (!WiFi.smartConfigDone() && (WIFImode == 2) && (WiFi.status() != WL_CONNECTED) && millis() - configTimer < 180000) {
        // Also start Serial CLI for entering AP and password.
        ProvisionCli(s);
        delay(100);
    }                       // loop until connected or Wifi setup menu is exited.
    delay(2000);            // give smartConfig time to send provision status back to the users phone.
        
    if (WiFi.status() == WL_CONNECTED) {
        _LOG_V("\nWiFi Connected, IP Address:%s.\n", WiFi.localIP().toString().c_str());
        WIFImode = 1;                                                           // we are already connected so don't call handleWIFImode
    } else {
        _LOG_V("\nCould not connect to WiFi, giving up.\n");
        WIFImode = 0;
        handleWIFImode();
    }
    write_settings();
    CliState= 0;
#ifndef SENSORBOX_VERSION                                                       //so we are not on a sensorbox but on a smartevse
    LCDNav = 0;
#else
    s.end();
    if (s == Serial2) {
        Serial2.setRxBufferSize(2048);                                                // Important! first increase buffer, then setup Uart2
        Serial2.begin(115200, SERIAL_8N1, PIN_RX, -1, true);
        Serial.begin(115200, SERIAL_8N1, PIN_PGD, PIN_TXD, false);
    } else
        s.begin(115200, SERIAL_8N1, PIN_PGD, PIN_TXD, false);                    // Input from TX of PIC, and debug output to USB
#endif
    WiFi.stopSmartConfig(); // this makes sure repeated SmartConfig calls are succesfull
    vTaskDelete(NULL);                                                          //end this task so it will not take up resources
}


void handleWIFImode(HardwareSerial *s) {
    if (WIFImode == 2 && WiFi.getMode() != WIFI_AP_STA) {
        //now start the portal in the background, so other tasks keep running
        xTaskCreate(
            SetupPortalTask,     // Function that should be called
            "SetupPortalTask",   // Name of the task (for debugging)
            10000,                // Stack size (bytes)                              // printf needs atleast 1kb
            (void *) &s,          // Parameter: which serial interface to use for ProvisionCli
            3,                    // Task priority - medium
            NULL                  // Task handleCTReceive
        );
    }

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
    // We might need some sort of authentication in the future.
    // SmartEVSE v3 have programmed ECDSA-256 keys stored in nvs
    // Unused for now.
    if (preferences.begin("KeyStorage", true) ) {                               // true = readonly
//prevent compiler warning
#if DBG == 1 || (DBG == 2 && LOG_LEVEL != 0)
        uint16_t hwversion = preferences.getUShort("hwversion");                // 0x0101 (01 = SmartEVSE,  01 = hwver 01)
#endif
        serialnr = preferences.getUInt("serialnr");
        String ec_private = preferences.getString("ec_private");
        String ec_public = preferences.getString("ec_public");
        preferences.end();

        _LOG_A("hwversion %04x serialnr:%u \n",hwversion, serialnr);
        //_LOG_A(ec_public);
    } else {
        _LOG_A("No KeyStorage found in nvs!\n");
        if (!serialnr) serialnr = MacId() & 0xffff;                             // when serialnr is not programmed (anymore), we use the Mac address
    }
    // overwrite APhostname if serialnr is programmed
#ifndef SENSORBOX_VERSION
    APhostname = "SmartEVSE-" + String( serialnr);                              // SmartEVSE access point Name = SmartEVSE-xxxxx
#else
    APhostname = "Sensorbox-" + String( serialnr);
#endif
    WiFi.setHostname(APhostname.c_str());

    mg_mgr_init(&mgr);  // Initialise event manager

    WiFi.setAutoReconnect(true);                                                //actually does nothing since this is the default value
    //WiFi.persistent(true);
    WiFi.onEvent(onWifiEvent);
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
#ifndef SENSORBOX_VERSION
    for (uint8_t i=0; i<8 ;i++) {
        SmartConfigKey[i+8] = random(9) + '1';
    }
#endif
    firmwareUpdateTimer = random(FW_UPDATE_DELAY, 0xffff);
    //firmwareUpdateTimer = random(FW_UPDATE_DELAY, 120); // DINGO TODO debug max 2 minutes

    if (preferences.begin("settings", false) ) {
#if MQTT == 0
        preferences.end();
    }
#else
        MQTTpassword = preferences.getString("MQTTpassword");
        MQTTuser = preferences.getString("MQTTuser");
#ifdef SENSORBOX_VERSION
        MQTTprefix = preferences.getString("MQTTprefix", "Sensorbox/" + String(serialnr));
#else
        MQTTprefix = preferences.getString("MQTTprefix", "SmartEVSE/" + String(serialnr));
#endif
        MQTTHost = preferences.getString("MQTTHost", "");
        MQTTPort = preferences.getUShort("MQTTPort", 1883);
        preferences.end();
    }

#if MQTT_ESP == 1
    MQTTclient.connect();
#endif

#endif //MQTT
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
}


// called by loop() in the main program
void network_loop() {
    static unsigned long lastCheck_net = 0;
    if (millis() - lastCheck_net >= 1000) {
        lastCheck_net = millis();
        //this block is for non-time critical stuff that needs to run approx 1 / second
        getLocalTime(&timeinfo, 1000U);
        if (!LocalTimeSet && WIFImode == 1) {
            _LOG_A("Time not synced with NTP yet.\n");
        }
#if MQTT
        MQTTclient.publish(MQTTprefix + "/ESPUptime", esp_timer_get_time() / 1000000, false, 0);
        MQTTclient.publish(MQTTprefix + "/WiFiRSSI", String(WiFi.RSSI()), false, 0);
#endif
    }

    mg_mgr_poll(&mgr, 100);                                                     // TODO increase this parameter to up to 1000 to make loop() less greedy

#ifndef DEBUG_DISABLED
    // Remote debug over WiFi
    Debug.handle();
#endif
}
