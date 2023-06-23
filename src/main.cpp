//////////////////////////////////////////////////////////////////////
//////////////////// REDTALLY by Martin Mittrenga ////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////// PoE - Transmitter ///////////////////////////
//////////////////////////////////////////////////////////////////////

#include <Arduino.h>
#include <U8g2lib.h>
#include <LoRa.h>
#include <Preferences.h>                //lib for flashstoreage
#include <bitmaps.h>
#include <ETH.h>
#include <SPI.h>
#include <SD.h>
#include <SPIFFS.h>                     //lib for filesystem
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>
#include <WiFi.h>
#include <rom/rtc.h>
#include <mbedtls/md.h>
#include <ctype.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif

#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

WiFiUDP udp;
int udpPort = 5727;
int udpPortState;
int udpPortNew;

String mode = "discover";
String mode_s = "dis";
String name_html = "REDTALLY";              // Device Name
String name = "REDTALLY Transmitter";       // Device Name
String version = "T0.03";                   // Frimeware Version
String rx_adr, tx_adr, rssi, bL;
String tx_adr_bb, tx_adr_cc, tx_adr_dd, tx_adr_ee;
String rssi_bb, rssi_cc, rssi_dd, rssi_ee;
String bL_bb, bL_cc, bL_dd, bL_ee;
String html_state_bb, html_state_cc, html_state_dd, html_state_ee;
String html_state_dhcp, html_state_wlan, html_state_dns, html_state_esm, html_state_tsl;
String html_state_ip, html_state_gw, html_state_snm, html_state_dns1, html_state_dns2;
String html_state_username, html_state_password, html_state_wifipw;
String ipOctet1, ipOctet2, ipOctet3, ipOctet4;
String gwOctet1, gwOctet2, gwOctet3, gwOctet4;
String snOctet1, snOctet2, snOctet3, snOctet4;
String dns1Octet1, dns1Octet2, dns1Octet3, dns1Octet4;
String dns2Octet1, dns2Octet2, dns2Octet3, dns2Octet4;
String ssid = "mySSID";
String wifipassword = "myPASSWORD";
String www_username = "admin";
String www_password = "admin";

char buf_rssi[4];
char buf_version[5];
char buf_localAddress[5];
char buf_mode[4];
char buf_ip[16];
char buf_ipip[16];
char buf_gw[16];
char buf_sn[16];
char buf_dns[16];
char buf_ls[8];
char buf_sdInit[32];
char buf_loraInit[32];
char buf_oledInit[32];
char buf_spiffsInit[32];
char buf_mdnsInit[32];
char buf_httpInit[32];
char buf_input_ip[100];
char buf_input_gw[100];
char buf_input_sn[100];
char buf_input_dns1[100];
char buf_input_dns2[100];
char buf_ipOctet1[4];
char buf_ipOctet2[4];
char buf_ipOctet3[4];
char buf_ipOctet4[4];
char buf_gwOctet1[4];
char buf_gwOctet2[4];
char buf_gwOctet3[4];
char buf_gwOctet4[4];
char buf_snOctet1[4];
char buf_snOctet2[4];
char buf_snOctet3[4];
char buf_snOctet4[4];
char buf_dns1Octet1[4];
char buf_dns1Octet2[4];
char buf_dns1Octet3[4];
char buf_dns1Octet4[4];
char buf_dns2Octet1[4];
char buf_dns2Octet2[4];
char buf_dns2Octet3[4];
char buf_dns2Octet4[4];

char buf_loraFrequenz[12];
char buf_loraTxPower[4];
char buf_loraSpreadingFactor[4];
char buf_loraSignalBandwidth[12];
char buf_loraCodingRate[4];
char buf_loraPreambleLength[8];
char buf_txpower[4];
char buf_connectedTallys[4];
char buf_udpPort[5];
char buf_udpPort_state[5];
char buf_discoverTime[4];
char buf_discoverTime_state [4];
char buf_tslbbassi[2];
char buf_tslbbassi_state[2];
char buf_tslccassi[2];
char buf_tslccassi_state[2];
char buf_tslddassi[2];
char buf_tslddassi_state[2];
char buf_tsleeassi[2];
char buf_tsleeassi_state[2];

char buf_html_ip[32];
char buf_html_gw[32];
char buf_html_snm[32];
char buf_html_dns1[32];
char buf_html_dns2[32];

int udp_seq[32] = {};
int udp_seq_old[32] = {};

const char* param_ip = "input1";
const char* param_gw = "input2";
const char* param_sn = "input3";
const char* param_dns1 = "input4";
const char* param_dns2 = "input5";
const char* param_txp = "input6";
const char* param_user = "inputUser";
const char* param_password = "inputPassword";
const char* param_new_user = "inputNewUser";
const char* param_new_password = "inputNewPassword";
const char* param_ssid = "inputSSID";
const char* param_wifipassword = "inputWLANPASSWORD";
const char* param_udpport = "inputUdpPort";
const char* param_distime = "inputDiscoverTime";
const char* param_bbassi = "inputtslbbassi";
const char* param_ccassi = "inputtslccassi";
const char* param_ddassi = "inputtslddassi";
const char* param_eeassi = "inputtsleeassi";

///////////////////////////////////////////////
///////// Setup Transmitter Values ////////////
///////////////////////////////////////////////

byte localAddress = 0xaa;                 // Address of this device                         
byte destination = 0xff;                  // Destination to send to                       

///////////////////////////////////////////////
///////////////////////////////////////////////
///////////////////////////////////////////////

byte msgKey1 = 0x2a;                          // Key of outgoing messages
byte msgKey2 = 0x56;
byte resFlag = 0x00;                          // 0x00 -> OFF  0x01 -> ON
byte esmFlag = 0x00;                          // 0x00 -> OFF  0x01 -> ON
byte transmissionPower;
byte receiverMode = 0x00;                     // 0x00 -> NOTHING        0x01 -> DISCOVER       0x02 -> OFFER       0x03 -> REQUEST     0x04 -> ACKNOWLEDGE      0x05 -> CONTROL
byte receiverState = 0x00;                    // 0x00 -> OFF            0x01 -> ON
byte receiverColor = 0x00;                    // 0x00 -> OFF            0x01 -> RED            0x02 -> GREEN       0x03 -> AMBER
byte msgCount = 0;                            // Count of outgoing messages

unsigned long lastDiscoverTimebb = 0;              // Last send time
unsigned long lastDiscoverTimecc = 0;              // Last send time
unsigned long lastDiscoverTimedd = 0;              // Last send time
unsigned long lastDiscoverTimeee = 0;              // Last send time
unsigned long lastOfferTime = 0;                   // Last send time
unsigned long lastOfferTimeRef = 0;
unsigned long lastOfferTimeEnd = 0;
unsigned long lastControlTime = 0;
unsigned long lastAckTime = 0;    
unsigned long lastAckTimeEnd = 0; 
unsigned long lastSwitchTime = 0;       
unsigned long lastAnalogReadTime = 0;
unsigned long lastDisplayPrint = 0;
unsigned long lastTslReadTime = 0;
unsigned long lastAuthentication = 0;
unsigned long lastSerialPrint = 0;

int defaultBrightnessDisplay = 255;   // value from 1 to 255
int counterSend = 0;
int counterSendTsl = 0;
int counterSendMax = 2;
int counterTallys = 0;
int gpioP1 = 36, gpioP2 = 39, gpioP3 = 34, gpioP4 = 35;
int gpioV1, gpioV2, gpioV3, gpioV4;
int gpioV1Map, gpioV2Map, gpioV3Map, gpioV4Map;
int missed_bb, missed_cc, missed_dd, missed_ee;
int useIPOctet1, useIPOctet2, useIPOctet3, useIPOctet4;
int useGWOctet1, useGWOctet2, useGWOctet3, useGWOctet4;
int useSNOctet1, useSNOctet2, useSNOctet3, useSNOctet4;
int useDNS1Octet1, useDNS1Octet2, useDNS1Octet3, useDNS1Octet4;
int useDNS2Octet1, useDNS2Octet2, useDNS2Octet3, useDNS2Octet4;
int int_ipOctet1, int_ipOctet2, int_ipOctet3, int_ipOctet4;
int int_gwOctet1, int_gwOctet2, int_gwOctet3, int_gwOctet4;
int int_snOctet1, int_snOctet2, int_snOctet3, int_snOctet4;
int int_dns1Octet1, int_dns1Octet2, int_dns1Octet3, int_dns1Octet4;
int int_dns2Octet1, int_dns2Octet2, int_dns2Octet3, int_dns2Octet4;
int buf_rssi_bb_int = 0;
int udp_len;
int discoverTime = 120;
int discoverTimeNew;
int tslbbassi = 1;
int tslccassi = 2;
int tslddassi = 3;
int tsleeassi = 4;
int tslbbassiNew, tslccassiNew, tslddassiNew, tsleeassiNew;

///////////////////////////////////////////////
//////////// Setup LORA Values ////////////////
///////////////////////////////////////////////

int loraTxPower = 17;                   //2-20 default if eeprom is empty 17
int loraTxPowerNew;
int loraSpreadingFactor = 7;            //6-12 default  7
double loraSignalBandwidth = 125E3;     //7.8E3, 10.4E3, 15.6E3, 20.8E3, 31.25E3, 41.7E3, 62.5E3, 125E3, 250E3, and 500E3 default 125E3
int loraCodingRate = 5;                 //5-8 default 5
int loraPreambleLength = 8;             //6-65535 default 8
double loraFrequenz = 868E6;            //set Frequenz 915E6 or 868E6

///////////////////////////////////////////////
///////////////////////////////////////////////
///////////////////////////////////////////////

float gpioV1Cal, gpioV2Cal, gpioV3Cal, gpioV4Cal;

bool gpioC1 = true;
bool gpioC2 = true;
bool gpioC3 = true;
bool gpioC4 = true;
bool tally_bb = false; 
bool tally_cc = false; 
bool tally_dd = false; 
bool tally_ee = false; 
bool tally_bb_init = false; 
bool tally_cc_init = false; 
bool tally_dd_init = false; 
bool tally_ee_init = false;
bool initBattery = true;
bool batteryAttention = false;
bool batteryAttentionState = false;
bool ethConnected = false;
bool useSTATIC = true;
bool useWLAN = false;
bool useTSL = true;
bool bool_esm = false;
bool ethState = false;
bool loraInit = false;
bool sdInit = false;
bool oledInit = false;
bool authenticated = false;
bool errorip = false;
bool notempty = false;
bool errortxp = false;
bool errorport = false;
bool errorDiscoverTime = false;
bool errorTslBBAssi= false;
bool errorTslCCAssi= false;
bool errorTslDDAssi= false;
bool errorTslEEAssi= false;
bool first_udp_seq = true;
bool tslAckReceived = true;
bool initUdpSteam = false;
bool blockTsl = false;
bool sendSequence = false;
bool shutdown = false;

///////////////////////////////////////////////
///////////////////////////////////////////////
///////////////////////////////////////////////

#define ETH_CLK_MODE ETH_CLOCK_GPIO17_OUT       //ETH_CLOCK_GPIO17_OUT - 50MHz clock from internal APLL inverted output on GPIO17 - tested with LAN8720
#define ETH_POWER_PIN                       16  // Pin# of the enable signal for the external crystal oscillator (-1 to disable for internal APLL source)
#define ETH_TYPE ETH_PHY_LAN8720                // Type of the Ethernet PHY (LAN8720 or TLK110)
#define ETH_ADDR                             0  // I²C-address of Ethernet PHY (0 or 1 for LAN8720, 31 for TLK110)
#define ETH_MDC_PIN                         23  // Pin# of the I²C clock signal for the Ethernet PHY
#define ETH_MDIO_PIN                        18  // Pin# of the I²C IO signal for the Ethernet PHY
#define NRST                                 5

#define LORA_MISO                            2  //Pinout Section
#define LORA_MOSI                           15
#define LORA_SCLK                           14
#define LORA_CS                             33
#define LORA_RST                            12
#define LORA_IRQ                             4  //Must be a Hardware Interrupt Pin

#define DISPLAY_MISO                         2
#define DISPLAY_MOSI                        15
#define DISPLAY_SCLK                        14
#define DISPLAY_CS                          16
#define DISPLAY_RST                         32

#define SD_MISO                              2
#define SD_MOSI                             15
#define SD_SCLK                             14
#define SD_CS                               13

#define loadWidth                           50  //Bitmap Declaration
#define loadHeight                          50
#define logoWidth                          128
#define logoHeight                          64
#define loraWidth                          128
#define loraHeight                          64
#define lineWidth                            2
#define lineHeight                          10

U8G2_SSD1306_128X64_NONAME_F_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/ DISPLAY_CS, /* dc=*/ DISPLAY_MISO, /* reset=*/ DISPLAY_RST);

AsyncWebServer server(80);

Preferences eeprom;            //Initiate Flash Memory

//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////

String proc_state(const String& state){

    if(state == "STATE_BB"){
        if(tally_bb == true) {
            html_state_bb = "ONLINE";
            }
            else{
            html_state_bb = "OFFLINE";
            }
            return html_state_bb;
    }

    if(state == "STATE_CC"){
        if(tally_cc == true) {
            html_state_cc = "ONLINE";
            }
            else{
            html_state_cc = "OFFLINE";
            }
            return html_state_cc;
    }

    if(state == "STATE_DD"){
        if(tally_dd == true) {
            html_state_dd = "ONLINE";
            }
            else{
            html_state_dd = "OFFLINE";
            }
            return html_state_dd;
    }

    if(state == "STATE_EE"){
        if(tally_ee == true) {
            html_state_ee = "ONLINE";
            }
            else{
            html_state_ee = "OFFLINE";
            }
            return html_state_ee;
    }

    if(state == "RSSI_BB"){        
            return rssi_bb;
    }

    if(state == "RSSI_CC"){        
            return rssi_cc;
    }

    if(state == "RSSI_DD"){        
            return rssi_dd;
    }

    if(state == "RSSI_EE"){        
            return rssi_ee;
    }

    if(state == "BL_BB"){        
            return bL_bb;
    }

    if(state == "BL_CC"){        
            return bL_cc;
    }

    if(state == "BL_DD"){        
            return bL_dd;
    }

    if(state == "BL_EE"){        
            return bL_ee;
    }

    if(state == "STATE_ADDRESS"){        
            return "aa";
    }

    if(state == "STATE_MODE"){        
            return mode_s;
    }

    if(state == "STATE_CONECTEDTALLYS"){      
            sprintf(buf_connectedTallys, "%d", counterTallys);  
            return buf_connectedTallys;
    }

    if(state == "STATE_DISCOVER_TIME"){    
            sprintf(buf_discoverTime_state, "%d", discoverTime);  
            return buf_discoverTime_state;
    }

    if(state == "STATE_ASSI_BB"){    
            sprintf(buf_tslbbassi_state, "%d", tslbbassi);  
            return buf_tslbbassi_state;
    }

    if(state == "STATE_ASSI_CC"){    
            sprintf(buf_tslccassi_state, "%d", tslccassi);  
            return buf_tslccassi_state;
    }

    if(state == "STATE_ASSI_DD"){    
            sprintf(buf_tslddassi_state, "%d", tslddassi);  
            return buf_tslddassi_state;
    }

    if(state == "STATE_ASSI_EE"){    
            sprintf(buf_tsleeassi_state, "%d", tsleeassi);  
            return buf_tsleeassi_state;
    }

    if(state == "STATE_DHCP"){
        if(useSTATIC == true){
            html_state_dhcp = "OFF";
        }
        else{
            html_state_dhcp = "ON";
        }
        return html_state_dhcp;
    }

    if(state == "STATE_WLAN"){
        if(useWLAN == true){
            html_state_wlan = "ON";
        }
        else{
            html_state_wlan = "OFF";
        }
        return html_state_wlan;
    }

    if(state == "STATE_SSID"){
        eeprom.begin("network", false); 
        ssid = eeprom.getString("ssid", ssid);
        eeprom.end();
        return ssid;
    }

    if(state == "STATE_WLANPASSWORD"){
        eeprom.begin("network", false); 
        html_state_wifipw = eeprom.getString("wifipassword", wifipassword);
        eeprom.end();

        const char* char_html_state_wifipw = html_state_wifipw.c_str();                           //convert string to char

        int m = strlen(char_html_state_wifipw) - 1;
        String sendbull = "&#x95";
            
        for(int i=0; i<m; i++) {
            sendbull = sendbull + "&#x95";  
        } 

        return sendbull;
    }

    if(state == "STATE_ESM"){
        if(bool_esm == true){
            html_state_esm = "ON";
        }
        else{
            html_state_esm = "OFF";
        }
        return html_state_esm;
    }

    if(state == "STATE_TSL"){
        if(useTSL == true){
            html_state_tsl = "TSL";
        }
        else{
            html_state_tsl = "GPIO";
        }
        return html_state_tsl;
    }

    if(state == "STATE_UDP_PORT"){  
        sprintf(buf_udpPort_state, "%d", udpPort);  
        return buf_udpPort_state;
    }

    if(state == "STATE_VERSION"){
            return version;
    }

    if(state == "STATE_LORA_FREQ"){
            sprintf(buf_loraFrequenz, "%f", loraFrequenz);
            return buf_loraFrequenz;
    }

    if(state == "STATE_LORA_TXP"){
            sprintf(buf_loraTxPower, "%d", loraTxPower);
            return buf_loraTxPower;
    }

    if(state == "STATE_LORA_SF"){
            sprintf(buf_loraSpreadingFactor, "%d", loraSpreadingFactor);
            return buf_loraSpreadingFactor;
    }

    if(state == "STATE_LORA_SB"){
            sprintf(buf_loraSignalBandwidth, "%f", loraSignalBandwidth);
            return buf_loraSignalBandwidth;
    }

    if(state == "STATE_LORA_CR"){
            sprintf(buf_loraCodingRate, "%d", loraCodingRate);
            return buf_loraCodingRate;
    }

    if(state == "STATE_LORA_PL"){
            sprintf(buf_loraPreambleLength, "%d", loraPreambleLength);
            return buf_loraPreambleLength;
    }

    if(state == "STATE_IP"){
            sprintf(buf_html_ip, "%d.%d.%d.%d", useIPOctet1, useIPOctet2, useIPOctet3, useIPOctet4);
            return buf_html_ip;
    }

    if(state == "STATE_GW"){
            sprintf(buf_html_gw, "%d.%d.%d.%d", useGWOctet1, useGWOctet2, useGWOctet3, useGWOctet4);
            return buf_html_gw;
    }

    if(state == "STATE_SNM"){
            sprintf(buf_html_snm, "%d.%d.%d.%d", useSNOctet1, useSNOctet2, useSNOctet3, useSNOctet4);
            return buf_html_snm;
    }

    if(state == "STATE_DNS1"){
            sprintf(buf_html_dns1, "%d.%d.%d.%d", useDNS1Octet1, useDNS1Octet2, useDNS1Octet3, useDNS1Octet4);
            return buf_html_dns1;
    }

    if(state == "STATE_DNS2"){
            sprintf(buf_html_dns2, "%d.%d.%d.%d", useDNS2Octet1, useDNS2Octet2, useDNS2Octet3, useDNS2Octet4);
            return buf_html_dns2;
    }

    if(state == "STATE_USERNAME"){
            eeprom.begin("configuration", false); 
            html_state_username = eeprom.getString("user", www_username);
            eeprom.end();
            return html_state_username;
    }

    if(state == "STATE_PASSWORD"){
            eeprom.begin("configuration", false); 
            html_state_password = eeprom.getString("password", www_password);
            eeprom.end();

            const char* char_html_state_password = html_state_password.c_str();                           //convert string to char

            int n = strlen(char_html_state_password) - 1;
            String sendbull = "&#x95";
            
            for(int i=0; i<n; i++) {
            sendbull = sendbull + "&#x95";  
            } 

            return sendbull;
    }

  return String();
}

//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////

void printLogo(int color, int wait) {
    
    u8g2.setDrawColor(color);
    
    // logo 1
    u8g2.firstPage();
    do { u8g2.drawXBM(0, 0, logoWidth, logoHeight, logo1); }
    while (u8g2.nextPage());
    delay(wait);
    // logo 2
    u8g2.firstPage();
    do { u8g2.drawXBM(0, 0, logoWidth, logoHeight, logo2); }
    while (u8g2.nextPage());
    delay(wait);
    // logo 3
    u8g2.firstPage();
    do { u8g2.drawXBM(0, 0,logoWidth, logoHeight, logo3); }
    while (u8g2.nextPage());
    delay(wait);
    // logo 4
    u8g2.firstPage();
    do { u8g2.drawXBM(0, 0, logoWidth, logoHeight, logo4); }
    while (u8g2.nextPage());
    delay(wait);
    // logo 5
    u8g2.firstPage();
    do { u8g2.drawXBM(0, 0, logoWidth, logoHeight, logo5); }
    while (u8g2.nextPage());
    delay(wait);
    // logo 6
    u8g2.firstPage();
    do { u8g2.drawXBM(0, 0, logoWidth, logoHeight, logo6); }
    while (u8g2.nextPage());
    delay(wait);
    // logo 7
    u8g2.firstPage();
    do { u8g2.drawXBM(0, 0, logoWidth, logoHeight, logo7); }
    while (u8g2.nextPage());
    delay(wait);
    // logo 8
    u8g2.firstPage();
    do { u8g2.drawXBM(0, 0, logoWidth, logoHeight, logo8); }
    while (u8g2.nextPage());
    delay(wait);
    // logo 9
    u8g2.firstPage();
    do { u8g2.drawXBM(0, 0, logoWidth, logoHeight, logo9); }
    while (u8g2.nextPage());
    delay(wait);
    // logo 10
    u8g2.firstPage();
    do { u8g2.drawXBM(0, 0, logoWidth, logoHeight, logo10); }
    while (u8g2.nextPage());
    delay(wait);
    
}

void printLoad(int color, int wait, int count) {
    
    u8g2.setDrawColor(color);

    for (int i=0; i < count; i++) {
        // load 1
        u8g2.firstPage();
        do { u8g2.drawXBM(39, 12, loadWidth, loadHeight, load1); }
        while (u8g2.nextPage());
        delay(wait);
        // load 2
        u8g2.firstPage();
        do { u8g2.drawXBM(39, 12, loadWidth, loadHeight, load2); }
        while (u8g2.nextPage());
        delay(wait);
        // load 3
        u8g2.firstPage();
        do { u8g2.drawXBM(39, 12, loadWidth, loadHeight, load3); }
        while (u8g2.nextPage());
        delay(wait);
        // load 4
        u8g2.firstPage();
        do { u8g2.drawXBM(39, 12, loadWidth, loadHeight, load4); }
        while (u8g2.nextPage());
        delay(wait);
        // load 5
        u8g2.firstPage();
        do { u8g2.drawXBM(39, 12, loadWidth, loadHeight, load5); }
        while (u8g2.nextPage());
        delay(wait);
        // load 6
        u8g2.firstPage();
        do { u8g2.drawXBM(39, 12, loadWidth, loadHeight, load6); }
        while (u8g2.nextPage());
        delay(wait);
        // load 7
        u8g2.firstPage();
        do { u8g2.drawXBM(39, 12, loadWidth, loadHeight, load7); }
        while (u8g2.nextPage());
        delay(wait);
        // load 8
        u8g2.firstPage();
        do { u8g2.drawXBM(39, 12, loadWidth, loadHeight, load8); }
        while (u8g2.nextPage());
        delay(wait);
        }
        
}

void printLora(int color) {
    
    u8g2.setDrawColor(color);
    u8g2.firstPage();
    do { u8g2.drawXBM(0, 0, loraWidth, loraHeight, lora); }
    while (u8g2.nextPage());
    
}

//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////

void startSPI_SD() {

    SPI.begin(SD_SCLK, SD_MISO, SD_MOSI);
    //SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));         
    digitalWrite(SD_CS, false);

}

void closeSPI_SD() {

    digitalWrite(SD_CS, true);
    SPI.end();
    //SPI.endTransaction();

}

//////////////////////////////////////////////////////////////////////

void startSPI_LORA() {

    SPI.begin(LORA_SCLK, LORA_MISO, LORA_MOSI);   
    //SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));      
    digitalWrite(LORA_CS, false);

    LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ);
    LoRa.setTxPower(loraTxPower);
    LoRa.setSpreadingFactor(loraSpreadingFactor);    
    LoRa.setSignalBandwidth(loraSignalBandwidth);
    LoRa.setCodingRate4(loraCodingRate);
    LoRa.setPreambleLength(loraPreambleLength);
    LoRa.begin(loraFrequenz);

}

void closeSPI_LORA() {

    digitalWrite(LORA_CS, true);
    SPI.end();
    //SPI.endTransaction();

}

//////////////////////////////////////////////////////////////////////

void startSPI_DISPLAY() {

    SPI.begin(DISPLAY_SCLK, DISPLAY_MISO, DISPLAY_MOSI); 
    //SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));    
    digitalWrite(DISPLAY_CS, false);

    u8g2.begin();
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.setContrast(defaultBrightnessDisplay);                  
    u8g2.setFlipMode(0);

}

void closeSPI_DISPLAY() {

    digitalWrite(DISPLAY_CS, true);
    SPI.end();
    //SPI.endTransaction();

}

//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////

void sendMessage() {
    LoRa.beginPacket();                   // start packet
    LoRa.write(destination);              // add destination address
    LoRa.write(localAddress);             // add sender address
    LoRa.write(msgKey1);                  // add message KEY
    LoRa.write(msgKey2);                  // add message KEY
    LoRa.write(resFlag);                  // add restart
    LoRa.write(esmFlag);                  // add energy save mode
    LoRa.write(transmissionPower);        // add txpower
    LoRa.write(receiverMode);             // add led color
    LoRa.write(receiverState);            // add led color
    LoRa.write(receiverColor);            // add led color
    LoRa.write(msgCount);                 // add message ID
    LoRa.endPacket();                     // finish packet and send it
    msgCount++;                           // increment message ID

    Serial.print("LORA TxD:");
    Serial.print(" DST: "); Serial.print(destination);
    Serial.print(" SOURCE: "); Serial.print(localAddress);
    Serial.print(" MSGKEY1: "); Serial.print(msgKey1);
    Serial.print(" MSGKEY2: "); Serial.print(msgKey2);
    Serial.print(" RESFLAG: "); Serial.print(resFlag);
    Serial.print(" ESMFLAG: "); Serial.print(esmFlag);
    Serial.print(" TXPOWER: "); Serial.print(transmissionPower);
    Serial.print(" RMODE: "); Serial.print(receiverMode);
    Serial.print(" RSTATE: "); Serial.print(receiverState);
    Serial.print(" RCOLOR: "); Serial.print(receiverColor);
    Serial.print(" MSGCOUNT: "); Serial.println(msgCount);
    
}

//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////

void onReceive(int packetSize, String *ptr_rx_adr, String *ptr_tx_adr, String *ptr_rssi, String *ptr_bL, byte *ptr_mode, byte *ptr_state, byte *ptr_color) { 
    if (packetSize == 0) return;          // if there's no packet, return

    //Clear the variables
    *ptr_rx_adr = "";
    *ptr_tx_adr = "";
    *ptr_rssi = "";
    *ptr_bL = "";
    *ptr_mode = 0x00;
    *ptr_state = 0x00;
    *ptr_color = 0x00;

    rx_adr = "";
    tx_adr = "";
    rssi = "";
    bL = "";
    receiverMode = 0x00;
    receiverState = 0x00;
    receiverColor = 0x00;

    // read packet header bytes:
    int recipient = LoRa.read();          // recipient address
    byte sender = LoRa.read();            // sender address
    byte incomingMsgKey1 = LoRa.read();   // incoming msg KEY1
    byte incomingMsgKey2 = LoRa.read();   // incoming msg KEY2
    byte incomingRSSI = LoRa.read();      // incoming incomingRSSI
    byte incomingBL = LoRa.read();        // incoming incomingBL
    byte incomingMode = LoRa.read();      
    byte incomingState = LoRa.read();     
    byte incomingColor = LoRa.read();   
    byte incomingMsgId = LoRa.read();     // incoming msg ID

    // if the recipient isn't this device or broadcast,
    if (recipient != localAddress && recipient != 0xff) {
        Serial.println("Message not for me.");
        return;                             // skip rest of function
    }

    if (incomingMsgKey1 != msgKey1 && incomingMsgKey2 != msgKey2) {
        Serial.println("Error: Message key false.");
        return;                             // skip rest of function
    }

    *ptr_rx_adr = String(recipient, HEX);
    *ptr_tx_adr = String(sender, HEX);
    *ptr_rssi = String(incomingRSSI - 256);
    *ptr_bL = String(incomingBL);
    *ptr_mode = (incomingMode);
    *ptr_state = (incomingState);
    *ptr_color = (incomingColor);

    Serial.print("LORA RxD:");
    Serial.print(" SOURCE: "); Serial.print(sender);
    Serial.print(" RECIPIENT: "); Serial.print(recipient);
    Serial.print(" MSGKEY1: "); Serial.print(incomingMsgKey1);
    Serial.print(" MSGKEY2: "); Serial.print(incomingMsgKey2);
    Serial.print(" RSSI: "); Serial.print(incomingRSSI);
    Serial.print(" BL: "); Serial.print(incomingBL);
    Serial.print(" RMODE: "); Serial.print(incomingMode);
    Serial.print(" RSTATE: "); Serial.print(incomingState);
    Serial.print(" RCOLOR: "); Serial.print(incomingColor);
    Serial.print(" MSGCOUNT: "); Serial.println(incomingMsgId);

    return;
}

//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////

String convertAddress(IPAddress address)
{
 return String(address[0]) + "." + 
        String(address[1]) + "." + 
        String(address[2]) + "." + 
        String(address[3]);
}

//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////

void clearValues() {
    rx_adr = "";
    tx_adr = "";
    rssi = "";
    receiverMode = 0x00;
    receiverState = 0x00;
    receiverColor = 0x00;
}

//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////

void printDisplay() {   
    
    if (ethConnected == true) {
        String MyIpAddress = convertAddress(ETH.localIP());
        MyIpAddress.toCharArray(buf_ipip, 16);

        String MyGwAddress = convertAddress(ETH.gatewayIP());
        sprintf(buf_gw, "%s", MyGwAddress);

        String MySnAddress = convertAddress(ETH.subnetMask());
        sprintf(buf_sn, "%s", MySnAddress);

        String MyDnsAddress = convertAddress(ETH.dnsIP());
        sprintf(buf_dns, "%s", MyDnsAddress);

        sprintf(buf_ls, "%d", ETH.linkSpeed());
    }

    sprintf(buf_localAddress, "%x", localAddress);          // byte
    sprintf(buf_mode, "%s", mode_s);                        // string  //%d for int

    u8g2.clearBuffer();					      // clear the internal memory
    
    //Address Indicator
    u8g2.setFont(u8g2_font_6x13_tf);
    u8g2.setDrawColor(1);
    u8g2.drawXBM(20, 3, lineWidth, lineHeight, line1);
    u8g2.setDrawColor(1);
    u8g2.drawStr(3,12,buf_localAddress);

    //Mode Indicator
    u8g2.setFont(u8g2_font_6x13_tf);
    u8g2.setDrawColor(1);
    u8g2.drawXBM(81, 3, lineWidth, lineHeight, line1);
    u8g2.setDrawColor(1);

    if (useSTATIC == true) {
        u8g2.setFont(u8g2_font_6x13_tf);
        u8g2.setDrawColor(1);
        u8g2.drawStr(29,12,"dhcp off");
    }else {
        u8g2.setFont(u8g2_font_6x13_tf);
        u8g2.setDrawColor(1);
        u8g2.drawStr(29,12,"dhcp on");
    }

    //IP Indicator
    if (ethConnected == true) {
        u8g2.setFont(u8g2_font_6x13_tf);
        u8g2.setDrawColor(1);
        u8g2.drawStr(0,35,"IP:");
        u8g2.drawStr(25,35,buf_ipip);
        u8g2.drawStr(0,50,"SN:");
        u8g2.drawStr(25,50,buf_sn);
    }else {
        u8g2.setFont(u8g2_font_6x13_tf);
        u8g2.setDrawColor(0);
        u8g2.drawBox(0,35,80,40);
    }
    
    u8g2.sendBuffer();

    lastDisplayPrint = millis();

}

//////////////////////////////////////////////////////////////////////

// Function for handeling the eth communication
void WiFiEvent(WiFiEvent_t event) {
    switch (event) {
    case ARDUINO_EVENT_ETH_START:
        Serial.println("ETH started.");
        ETH.setHostname("REDTALLY");     //set eth hostname here
        break;
    case ARDUINO_EVENT_ETH_CONNECTED:
        Serial.println("ETH connected.");
        break;
    case ARDUINO_EVENT_ETH_GOT_IP:
        Serial.print("ETH MAC: ");
        Serial.print(ETH.macAddress());
        Serial.print(", IPv4: ");
        Serial.print(ETH.localIP());
        if (ETH.fullDuplex()) {
            Serial.print(", FULL_DUPLEX");
        }
        Serial.print(", ");
        Serial.print(ETH.linkSpeed());
        Serial.println("Mbps.");
        ethConnected = true;

        //initializes the UDP state
        //This initializes the transfer buffer
        Serial.print("Start UDP receiver to ");
        Serial.print(udpPort);
        Serial.println(" Port");
        udp.begin(WiFi.localIP(), udpPort);

        break;
    case ARDUINO_EVENT_ETH_DISCONNECTED:
        Serial.println("ETH disconnected.");
        ethConnected = false;
        break;
    case ARDUINO_EVENT_ETH_STOP:
        Serial.println("ETH stopped.");
        ethConnected = false;
        break;
    default:
        break;
    }
}

//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////

void startWIFI() {

    eeprom.begin("network", false); 
    ssid = eeprom.getString("ssid", ssid);
    wifipassword = eeprom.getString("wifipassword", wifipassword);
    eeprom.end();

    const char* char_ssid = ssid.c_str();                           //convert string to char
    const char* char_wifipassword = wifipassword.c_str();
    const char* char_name_html = name_html.c_str();

    WiFi.setHostname(char_name_html);
    WiFi.mode(WIFI_STA);
    WiFi.begin(char_ssid, char_wifipassword);
    Serial.println("Connecting to WIFI.");
    int tryConnectWLAN = 0;
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        tryConnectWLAN++;
        if (tryConnectWLAN == 3){
            tryConnectWLAN = 0;
            break;
        }
    }
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("Could not connect to WIFI.");
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.print("WIFI MAC: "); Serial.print(WiFi.macAddress()); Serial.print(", IPv4: "); Serial.println(WiFi.localIP());
    }
}

void endWIFI() {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    Serial.println("Diconnected from WIFI.");
}

//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////

String sha1(String payloadStr){

    const char *payload = payloadStr.c_str();
    int size = 20;
    byte shaResult[size];
    mbedtls_md_context_t ctx;
    mbedtls_md_type_t md_type = MBEDTLS_MD_SHA1;
    const size_t payloadLength = strlen(payload);
    mbedtls_md_init(&ctx);
    mbedtls_md_setup(&ctx, mbedtls_md_info_from_type(md_type), 0);
    mbedtls_md_starts(&ctx);
    mbedtls_md_update(&ctx, (const unsigned char *) payload, payloadLength);
    mbedtls_md_finish(&ctx, shaResult);
    mbedtls_md_free(&ctx);
    String hashStr = "";

    for(uint16_t i = 0; i < size; i++) {

        String hex = String(shaResult[i], HEX);
        if(hex.length() < 2) {
            hex = "0" + hex;
        }
        
        hashStr += hex;
    }

    return hashStr;
}

//////////////////////////////////////////////////////////////////////

void handleLogin(AsyncWebServerRequest *request) {

    Serial.println("AUTH handle login.");
    String msg;


    eeprom.begin("configuration", false); 

    www_username = eeprom.getString("user", www_username);
    Serial.print("www_username: "); Serial.println(www_username);
    
    www_password = eeprom.getString("password", www_password);
    Serial.print("www_password: "); Serial.println(www_password); 

    eeprom.end();


    if (request->hasHeader("Cookie")) {
        // Print cookies
        Serial.print("AUTH found cookie: ");
        String cookie = request->header("Cookie");
        Serial.println(cookie);
    }

    if (request->hasArg("username") && request->hasArg("password")) {
        Serial.print("AUTH found parameter ");

        if (request->arg("username") == www_username && request->arg("password") == www_password) {
        AsyncWebServerResponse *response = request->beginResponse(301); //Sends 301 redirect
        response->addHeader("Location", "/");
        response->addHeader("Cache-Control", "no-cache");
        String token = sha1(www_username + ":" + www_password + ":" + request->client()->remoteIP().toString());
        Serial.print("Token: ");
        Serial.println(token);
        response->addHeader("Set-Cookie", "ESPSESSIONID=" + token);
        request->send(response);
        Serial.println("AUTH login successful.");
        lastAuthentication = millis();
        authenticated = true;
        return;
        }

        msg = "Wrong Username or Password! Try again.";
        Serial.println("AUTH login failed.");
        AsyncWebServerResponse *response = request->beginResponse(301); //Sends 301 redirect
        response->addHeader("Location", "/login.html?msg=" + msg);
        response->addHeader("Cache-Control", "no-cache");
        //request->send(response);
        request->send(SPIFFS, "/authentificationfalse.html", String(), false, proc_state);
        authenticated = false;
        return;
    }
}

//////////////////////////////////////////////////////////////////////

void handleLogout(AsyncWebServerRequest *request) {
    Serial.println("AUTH disconnection.");
    AsyncWebServerResponse *response = request->beginResponse(301); //Sends 301 redirect
    response->addHeader("Location", "/login.html?msg=User disconnected");
    response->addHeader("Cache-Control", "no-cache");
    response->addHeader("Set-Cookie", "ESPSESSIONID=0");
    request->send(response);
    authenticated = false;
    return;
}

//////////////////////////////////////////////////////////////////////

void handleNotFound(AsyncWebServerRequest *request) {
    request->send(SPIFFS, "/notfound.html", String(), false, proc_state);
}

//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////

void setup() {

    Serial.begin(115200);
    while (!Serial);

    Serial.println("");
    Serial.println(name);
    Serial.println("Version: " + version);

    pinMode(gpioP1, INPUT);
    pinMode(gpioP2, INPUT);
    pinMode(gpioP3, INPUT);
    pinMode(gpioP4, INPUT);

    pinMode(SD_CS, OUTPUT);                                     //CS SD
    pinMode(LORA_CS, OUTPUT);                                   //CS LORA
    pinMode(DISPLAY_CS, OUTPUT);                                //CS Display

    digitalWrite(SD_CS, true);
    digitalWrite(LORA_CS, true);
    digitalWrite(DISPLAY_CS, true);

    //pinMode(SD_MISO, INPUT_PULLUP);

//////////////////////////////////////////////////////////////////////

    //1=POWERON_RESET   3=SW_RESET  4=OWDT_RESET    5=DEEPSLEEP_RESET   6=SDIO_RESET    7=TG0WDT_SYS_RESET  8=TG1WDT_SYS_RESET  9=RTCWDT_SYS_RESET  
    //10=INTRUSION_RESET  11=TGWDT_CPU_RESET  12=SW_CPU_RESET 13=RTCWDT_CPU_RESET   14=EXT_CPU_RESET    15=RTCWDT_BROWN_OUT_RESET   16=RTCWDT_RTC_RESET     default=NO_MEAN

    //When upload some programs, esp clear the flash with the reason 1 and 14
    //When restart, only reason 12 is excecuted

    eeprom.begin("configuration", false);                //false mean use read/write mode

    shutdown = eeprom.getBool("shut", false);    
    Serial.print("shutdown: "); Serial.println(shutdown); 
    eeprom.end();

    Serial.print("CPU0 reset reason: ");
    Serial.println(rtc_get_reset_reason(0));

    Serial.print("CPU1 reset reason: ");
    Serial.println(rtc_get_reset_reason(1));

    if (rtc_get_reset_reason(0) == 1 && rtc_get_reset_reason(1) == 14 && shutdown == false) {
            eeprom.begin("network", false);
            eeprom.clear();             //Clear the eeprom when the reset button is pushed
            eeprom.end();
            eeprom.begin("configuration", false); 
            eeprom.clear();
            eeprom.end();
    }

    if (shutdown == true){
        shutdown = false;
        eeprom.begin("configuration", false);                //false mean use read/write mode
        eeprom.putBool("shut", shutdown);    
        Serial.print("shutdown: "); Serial.println(shutdown); 
        eeprom.end();
    }

//////////////////////////////////////////////////////////////////////

    eeprom.begin("network", false);                //false mean use read/write mode

    useSTATIC = eeprom.getBool("dhcp", true);     //false mean default value if nothing returned
    Serial.print("useSTATIC: "); Serial.println(useSTATIC);

    udpPort = eeprom.getInt("udpport", udpPort);     
    Serial.print("udpPort: "); Serial.println(udpPort);

    ssid = eeprom.getString("ssid", ssid);
    Serial.print("ssid: "); Serial.println(ssid);

    wifipassword = eeprom.getString("wifipassword", wifipassword);
    Serial.print("wifipassword: "); Serial.println(wifipassword);

    useIPOctet1 = eeprom.getInt("ipOctet1", 192);
    Serial.print("useIPOctet1: "); Serial.println(useIPOctet1);

    useIPOctet2 = eeprom.getInt("ipOctet2", 168);
    Serial.print("useIPOctet2: "); Serial.println(useIPOctet2);

    useIPOctet3 = eeprom.getInt("ipOctet3", 178);
    Serial.print("useIPOctet3: "); Serial.println(useIPOctet3);

    useIPOctet4 = eeprom.getInt("ipOctet4", 100);
    Serial.print("useIPOctet4: "); Serial.println(useIPOctet4);

    useGWOctet1 = eeprom.getInt("gwOctet1", 192);
    useGWOctet2 = eeprom.getInt("gwOctet2", 168);
    useGWOctet3 = eeprom.getInt("gwOctet3", 178);
    useGWOctet4 = eeprom.getInt("gwOctet4", 1);  
    useSNOctet1 = eeprom.getInt("snOctet1", 255);
    useSNOctet2 = eeprom.getInt("snOctet2", 255);
    useSNOctet3 = eeprom.getInt("snOctet3", 255);
    useSNOctet4 = eeprom.getInt("snOctet4", 0);  
    useDNS1Octet1 = eeprom.getInt("dns1Octet1", 1);
    useDNS1Octet2 = eeprom.getInt("dns1Octet2", 1);
    useDNS1Octet3 = eeprom.getInt("dns1Octet3", 1);
    useDNS1Octet4 = eeprom.getInt("dns1Octet4", 1);  
    useDNS2Octet1 = eeprom.getInt("dns2Octet1", 2);
    useDNS2Octet2 = eeprom.getInt("dns2Octet2", 2);
    useDNS2Octet3 = eeprom.getInt("dns2Octet3", 2);
    useDNS2Octet4 = eeprom.getInt("dns2Octet4", 2);
    eeprom.end();

    eeprom.begin("configuration", false); 

    useTSL = eeprom.getBool("tsl", true);                         //change default tsl
    Serial.print("useTSL: "); Serial.println(useTSL);

    tslbbassi = eeprom.getInt("tslbbid", tslbbassi);
    Serial.print("tslbbassi: "); Serial.println(tslbbassi);

    tslccassi = eeprom.getInt("tslccid", tslccassi);
    Serial.print("tslccassi: "); Serial.println(tslccassi);

    tslddassi = eeprom.getInt("tslddid", tslddassi);
    Serial.print("tslddassi: "); Serial.println(tslddassi);

    tsleeassi = eeprom.getInt("tsleeid", tsleeassi);
    Serial.print("tsleeassi: "); Serial.println(tsleeassi);

    discoverTime = eeprom.getInt("distime", discoverTime);   
    Serial.print("discoverTime: "); Serial.println(discoverTime);

    bool_esm = eeprom.getBool("esm", false);
    Serial.print("bool_esm: "); Serial.println(bool_esm);

    loraTxPower = eeprom.getInt("txpower", loraTxPower);            //if the eeprom is never written, then give the default value back
    Serial.print("loraTxPower: "); Serial.println(loraTxPower);
    
    www_username = eeprom.getString("user", www_username);
    Serial.print("www_username: "); Serial.println(www_username);
    
    www_password = eeprom.getString("password", www_password);
    Serial.print("www_password: "); Serial.println(www_password);

    transmissionPower = loraTxPower;  

    eeprom.end();

//////////////////////////////////////////////////////////////////////

    SPI.begin(SD_SCLK, SD_MISO, SD_MOSI);     
    //SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));    
    digitalWrite(SD_CS, false);

    if (!SD.begin(SD_CS)) {                                                 // initialize sd card
        sdInit = false;   
    }else {     
        sdInit = true; 
    }

    digitalWrite(SD_CS, true);
    SPI.end();
    //SPI.endTransaction();

//////////////////////////////////////////////////////////////////////

    SPI.begin(LORA_SCLK, LORA_MISO, LORA_MOSI);     
    //SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));    
    digitalWrite(LORA_CS, false);

    LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ);
    LoRa.setTxPower(loraTxPower);
    LoRa.setSpreadingFactor(loraSpreadingFactor);    
    LoRa.setSignalBandwidth(loraSignalBandwidth);
    LoRa.setCodingRate4(loraCodingRate);
    LoRa.setPreambleLength(loraPreambleLength);
    LoRa.begin(loraFrequenz);

    if (!LoRa.begin(loraFrequenz)) {                                        // initialize lora frequenz
        loraInit = false; 
    }
    if (LoRa.begin(loraFrequenz)) {    
        loraInit = true; 
    }

    digitalWrite(LORA_CS, true);
    SPI.end();  
    //SPI.endTransaction();

//////////////////////////////////////////////////////////////////////

    SPI.begin(DISPLAY_SCLK, DISPLAY_MISO, DISPLAY_MOSI);     
    //SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));    
    digitalWrite(DISPLAY_CS, false); 

    u8g2.begin();
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.setContrast(defaultBrightnessDisplay);                  
    u8g2.setFlipMode(0);

    if (!u8g2.begin()) {                                                      // initialize oled
        oledInit = false;   
    }else {     
        oledInit = true; 
    }

    //        Color, Delay, Runs
    printLogo(0, 70);
    delay(1000);
    printLoad(1, 80, 2);
    printLogo(0, 25);
    delay(500);
    printLoad(1, 80, 4);
    delay(500);

    sprintf(buf_version, "%s", version);
    u8g2.drawStr(99,60,buf_version);
    u8g2.sendBuffer();

    if (sdInit == true) {
        Serial.println("SD-Card init succeeded.");
        sprintf(buf_sdInit, "%s", "SD init");
        u8g2.drawStr(0,10,buf_sdInit);
        u8g2.sendBuffer();
        delay(300);
    }else{
        Serial.println("SD-Card init failed. Check your connections."); 
        sprintf(buf_sdInit, "%s", "SD failed");
        u8g2.drawStr(0,10,buf_sdInit);
        u8g2.sendBuffer();
        delay(300);
    }

    if (loraInit == true) {
        Serial.println("LORA init succeeded.");
        sprintf(buf_loraInit, "%s", "LORA init");
        u8g2.drawStr(0,20,buf_loraInit);
        u8g2.sendBuffer();
        delay(300);
    }else{
        Serial.println("LORA init failed. Check your connections.");
        sprintf(buf_loraInit, "%s", "LORA failed");
        u8g2.drawStr(0,20,buf_loraInit);
        u8g2.sendBuffer();
        delay(300);
        while (true);
    }

    if (oledInit == true) {
        Serial.println("OLED init succeeded.");
        sprintf(buf_oledInit, "%s", "OLED init");
        u8g2.drawStr(0,30,buf_oledInit);
        u8g2.sendBuffer();
        delay(300);
    }else{
        Serial.println("OLED init failed. Check your connections."); 
        sprintf(buf_oledInit, "%s", "OLED failed");
        u8g2.drawStr(0,30,buf_oledInit);
        u8g2.sendBuffer();
        delay(300);
    }
    
//////////////////////////////////////////////////////////////////////  

    if(!SPIFFS.begin(true)){
        Serial.println("SPIFFS init failed. An Error has occurred while mounting SPIFFS");
        sprintf(buf_spiffsInit, "%s", "SPIFFS failed");
        u8g2.drawStr(0,35,buf_spiffsInit);
        u8g2.sendBuffer();
        delay(300);
        while (true);
    }
    Serial.println("SPIFFS init succeeded.");
    sprintf(buf_spiffsInit, "%s", "SPIFFS init");
    u8g2.drawStr(0,40,buf_spiffsInit);
    u8g2.sendBuffer();
    delay(300);

//////////////////////////////////////////////////////////////////////

    WiFi.onEvent(WiFiEvent);

    pinMode(NRST, OUTPUT);

    digitalWrite(NRST, 0);
    delay(200);
    digitalWrite(NRST, 1);
    delay(200);
    digitalWrite(NRST, 0);
    delay(200);
    digitalWrite(NRST, 1);

    ETH.begin(ETH_ADDR, ETH_POWER_PIN, ETH_MDC_PIN, ETH_MDIO_PIN, ETH_TYPE, ETH_CLK_MODE);

    IPAddress local_IP (useIPOctet1, useIPOctet2, useIPOctet3, useIPOctet4);
    IPAddress gateway (useGWOctet1, useGWOctet2, useGWOctet3, useGWOctet4);
    IPAddress subnet (useSNOctet1, useSNOctet2, useSNOctet3, useSNOctet4);
    IPAddress primaryDNS (useDNS1Octet1, useDNS1Octet2, useDNS1Octet3, useDNS1Octet4);
    IPAddress secondaryDNS (useDNS2Octet1, useDNS2Octet2, useDNS2Octet3, useDNS2Octet4);
    
    if (useSTATIC == true) {
        ETH.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS);
        WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS);
    }
    
//////////////////////////////////////////////////////////////////////

    if (MDNS.begin("redtally")) {
        Serial.println("MDNS responder started.");
        sprintf(buf_mdnsInit, "%s", "MDNS started");
        u8g2.drawStr(0,50,buf_mdnsInit);
        u8g2.sendBuffer();
        delay(300);
    }

//////////////////////////////////////////////////////////////////////
 

    // Route for root / web page
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        if (authenticated == true) {
            request->send(SPIFFS, "/index.html", String(), false, proc_state);
        } else {
            request->send(SPIFFS, "/login.html", String(), false, proc_state);
        }
    });
    

    // Route to load style.css file
    server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(SPIFFS, "/style.css", "text/css");
    });


    // Route to load popup.js file
    server.on("/popup.js", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(SPIFFS, "/popup.js", "text/js");
    });

    
    server.on("/network", HTTP_GET, [](AsyncWebServerRequest *request){
        if (authenticated == true) {
            request->send(SPIFFS, "/network.html", String(), false, proc_state);
        } else {
            request->send(SPIFFS, "/login.html", String(), false, proc_state);
        }
    });

    server.on("/dhcp-on", HTTP_GET, [](AsyncWebServerRequest *request){
        if (authenticated == true) {
            useSTATIC = false;
            eeprom.begin("network", false);                //false mean use read/write mode
            eeprom.putBool("dhcp", useSTATIC);     
            eeprom.end();
            Serial.print("useSTATIC: "); Serial.println(useSTATIC);
            resFlag = 0x01;

            if (tally_bb == true){
                destination = 0xbb;                                                                     //if tx power changed via webterminal, then send message to receivers and change the txpower with restart
                receiverMode = 0x05;            // Send a message
                sendMessage();         
                delay(500);
            }
            if (tally_cc == true){
                destination = 0xcc;
                receiverMode = 0x05;
                sendMessage();
                delay(500);
            }
            if (tally_dd == true){
                destination = 0xdd;
                receiverMode = 0x05;
                sendMessage();
                delay(500);
            }
            if (tally_ee == true){
                destination = 0xee;
                receiverMode = 0x05;
                sendMessage();
            }
            resFlag = 0x00;
            request->send(SPIFFS, "/network.html", String(), false, proc_state);
            delay(2000);
            ESP.restart();
        } else {
            request->send(SPIFFS, "/login.html", String(), false, proc_state);
        }
    });

    server.on("/dhcp-off", HTTP_GET, [](AsyncWebServerRequest *request){
        if (authenticated == true) {
            useSTATIC = true;
            eeprom.begin("network", false);                //false mean use read/write mode
            eeprom.putBool("dhcp", useSTATIC);     
            eeprom.end();
            Serial.print("useSTATIC: "); Serial.println(useSTATIC);
            resFlag = 0x01;

            if (tally_bb == true){
                destination = 0xbb;                                                                     //if tx power changed via webterminal, then send message to receivers and change the txpower with restart
                receiverMode = 0x05;
                sendMessage();
                delay(500);
            }
            if (tally_cc == true){
                destination = 0xcc;
                receiverMode = 0x05;
                sendMessage();
                delay(500);
            }
            if (tally_dd == true){
                destination = 0xdd;
                receiverMode = 0x05;
                sendMessage();
                delay(500);
            }
            if (tally_ee == true){
                destination = 0xee;
                receiverMode = 0x05;
                sendMessage();
            }
            resFlag = 0x00;
            request->send(SPIFFS, "/network.html", String(), false, proc_state);
            delay(2000);
            ESP.restart();
        } else {
            request->send(SPIFFS, "/login.html", String(), false, proc_state);
        }
    });

    server.on("/wlan-on", HTTP_GET, [](AsyncWebServerRequest *request){
        if (authenticated == true) {
            useWLAN = true;

            startWIFI();
          
            request->send(SPIFFS, "/network.html", String(), false, proc_state);
        } else {
            request->send(SPIFFS, "/login.html", String(), false, proc_state);
        }
    });

    server.on("/wlan-off", HTTP_GET, [](AsyncWebServerRequest *request){
        if (authenticated == true) {
            useWLAN = false;

            endWIFI();
            
            request->send(SPIFFS, "/network.html", String(), false, proc_state);
        } else {
            request->send(SPIFFS, "/login.html", String(), false, proc_state);
        }
    });

    // Send a GET request to <ESP_IP>/get?input1=<inputMessage>
    server.on("/wlan-change", HTTP_GET, [] (AsyncWebServerRequest *request) {
        if (authenticated == true) {
            String input_ssid, input_wlanpassword;
            String input_param_ssid, input_param_wlanpassword;

            if (request->hasParam(param_ssid)) {
            input_ssid = request->getParam(param_ssid)->value();
            input_param_ssid = param_ssid;
            }
            if (request->hasParam(param_wifipassword)) {
            input_wlanpassword = request->getParam(param_wifipassword)->value();
            input_param_wlanpassword = param_wifipassword;
            }
            // If empty, print no message
            if (request->hasParam("")) {
            input_ssid = "No message sent";
            input_param_ssid = "none";
            }
            if (request->hasParam("")) {
            input_wlanpassword = "No message sent";
            input_param_wlanpassword = "none";
            }

            eeprom.begin("network", false);                //false mean use read/write mode
            eeprom.putString("ssid", input_ssid);
            Serial.print("input_ssid: "); Serial.println(input_ssid);
            eeprom.putString("wifipassword", input_wlanpassword);
            Serial.print("input_wlanpassword: "); Serial.println(input_wlanpassword);
            eeprom.end();

            request->send(SPIFFS, "/network.html", String(), false, proc_state);
            
        } else {
            request->send(SPIFFS, "/login.html", String(), false, proc_state);
        }
    });

    server.on("/configuration", HTTP_GET, [](AsyncWebServerRequest *request){
        if (authenticated == true) {
            request->send(SPIFFS, "/configuration.html", String(), false, proc_state);
        } else {
            request->send(SPIFFS, "/login.html", String(), false, proc_state);
        }
    });

    server.on("/tsl", HTTP_GET, [](AsyncWebServerRequest *request){
        if (authenticated == true) {
            useTSL = true;
            eeprom.begin("configuration", false);                //false mean use read/write mode
            eeprom.putBool("tsl", useTSL);     
            Serial.print("useTSL: "); Serial.println(useTSL);
            eeprom.end();
            request->send(SPIFFS, "/configuration.html", String(), false, proc_state);
        } else {
            request->send(SPIFFS, "/login.html", String(), false, proc_state);
        }
    });

    server.on("/gpio", HTTP_GET, [](AsyncWebServerRequest *request){
        if (authenticated == true) {
            useTSL = false;
            eeprom.begin("configuration", false);                //false mean use read/write mode
            eeprom.putBool("tsl", useTSL); 
            Serial.print("useTSL: "); Serial.println(useTSL);    
            eeprom.end();
            request->send(SPIFFS, "/configuration.html", String(), false, proc_state);
        } else {
            request->send(SPIFFS, "/login.html", String(), false, proc_state);
        }
    });

    server.on("/esm-on", HTTP_GET, [](AsyncWebServerRequest *request){
        if (authenticated == true) {
            bool_esm = true;
            if (bool_esm == true){ esmFlag = 0x01; }
            if (bool_esm == false){ esmFlag = 0x00; }
            eeprom.begin("configuration", false);                //false mean use read/write mode
            eeprom.putBool("esm", bool_esm);    
            Serial.print("bool_esm: "); Serial.println(bool_esm); 
            eeprom.end();
            request->send(SPIFFS, "/configuration.html", String(), false, proc_state);
        } else {
            request->send(SPIFFS, "/login.html", String(), false, proc_state);
        }
    });

    server.on("/esm-off", HTTP_GET, [](AsyncWebServerRequest *request){
        if (authenticated == true) {
            bool_esm = false;
            if (bool_esm == true){ esmFlag = 0x01; }
            if (bool_esm == false){ esmFlag = 0x00; }
            eeprom.begin("configuration", false);                //false mean use read/write mode
            eeprom.putBool("esm", bool_esm);    
            Serial.print("bool_esm: "); Serial.println(bool_esm); 
            eeprom.end();
            request->send(SPIFFS, "/configuration.html", String(), false, proc_state);
        } else {
            request->send(SPIFFS, "/login.html", String(), false, proc_state);
        }
    });

    server.on("/savedata", HTTP_GET, [](AsyncWebServerRequest *request){
        if (authenticated == true) {

            shutdown = true;
            eeprom.begin("configuration", false);                //false mean use read/write mode
            eeprom.putBool("shut", shutdown);    
            Serial.print("shutdown: "); Serial.println(shutdown); 
            eeprom.end();
            request->send(SPIFFS, "/configuration.html", String(), false, proc_state);

        } else {
            request->send(SPIFFS, "/login.html", String(), false, proc_state);
        }
    });

    server.on("/restart", HTTP_GET, [](AsyncWebServerRequest *request){
        if (authenticated == true) {
            resFlag = 0x01;

            if (tally_bb == true){
                destination = 0xbb;                                                                     //if tx power changed via webterminal, then send message to receivers and change the txpower with restart
                receiverMode = 0x05;
                sendMessage();
                delay(500);
            }
            if (tally_cc == true){
                destination = 0xcc;
                receiverMode = 0x05;      
                sendMessage();
                delay(500);
            }
            if (tally_dd == true){
                destination = 0xdd;
                receiverMode = 0x05;
                sendMessage();
                delay(500);
            }
            if (tally_ee == true){
                destination = 0xee;
                receiverMode = 0x05;
                sendMessage();
            }
            resFlag = 0x00;
            request->send(SPIFFS, "/configuration.html", String(), false, proc_state);
            delay(2000);
            ESP.restart();

        } else {
            request->send(SPIFFS, "/login.html", String(), false, proc_state);
        }
    });

    server.on("/factoryreset", HTTP_GET, [](AsyncWebServerRequest *request){
        if (authenticated == true) {
            resFlag = 0x01;

            if (tally_bb == true){
                destination = 0xbb;                                                                     //if tx power changed via webterminal, then send message to receivers and change the txpower with restart
                receiverMode = 0x05;
                sendMessage();
                delay(500);
            }
            if (tally_cc == true){
                destination = 0xcc;
                receiverMode = 0x05;      
                sendMessage();
                delay(500);
            }
            if (tally_dd == true){
                destination = 0xdd;
                receiverMode = 0x05;
                sendMessage();
                delay(500);
            }
            if (tally_ee == true){
                destination = 0xee;
                receiverMode = 0x05;
                sendMessage();
            }
            resFlag = 0x00;

            request->send(SPIFFS, "/configuration.html", String(), false, proc_state);
            
            eeprom.begin("network", false);
            eeprom.clear();             //Clear the eeprom when the reset button is pushed
            eeprom.end();
            eeprom.begin("configuration", false); 
            eeprom.clear();
            eeprom.end();

            delay(2000);
            ESP.restart();

        } else {
            request->send(SPIFFS, "/login.html", String(), false, proc_state);
        }
    });

    server.on("/info", HTTP_GET, [](AsyncWebServerRequest *request){
        if (authenticated == true) {
            request->send(SPIFFS, "/info.html", String(), false, proc_state);
        } else {
            request->send(SPIFFS, "/login.html", String(), false, proc_state);
        }
    });

    // Send a GET request to <ESP_IP>/get?input1=<inputMessage>
    server.on("/get-txp", HTTP_GET, [] (AsyncWebServerRequest *request) {
        if (authenticated == true) {
            String input_txp;
            String input_param_txp;

            // GET input6 value on <ESP_IP>/get?input6=<inputMessage>
            if (request->hasParam(param_txp)) {
            input_txp = request->getParam(param_txp)->value();
            input_param_txp = param_txp;
            }
            // If empty, print no message
            if (request->hasParam("")) {
            input_txp = "No message sent";
            input_param_txp = "none";
            }

            errortxp = false;

            if(input_txp != "") {
                sprintf(buf_txpower, "%s", input_txp);
                loraTxPowerNew = atoi(buf_txpower);

                Serial.print("LoraTxPowerNew: "); Serial.println(loraTxPowerNew);

                if ((loraTxPowerNew < 2) || (loraTxPowerNew > 20)) {
                    Serial.println("ERROR!");
                    errortxp = true;
                }
                else {

                    if (loraTxPowerNew != loraTxPower) {

                        loraTxPower = loraTxPowerNew;              //value for eeprom
                        transmissionPower = loraTxPowerNew;             //send via lora

                        eeprom.begin("configuration", false);                //false mean use read/write mode
                        eeprom.putInt("txpower", loraTxPower);    
                        Serial.print("loraTxPower: "); Serial.println(loraTxPower);  
                        eeprom.end(); 

                        if (tally_bb == true){
                            destination = 0xbb;                                                                     //if tx power changed via webterminal, then send message to receivers and change the txpower with restart
                            receiverMode = 0x05;
                            sendMessage();
                            delay(500);
                        }
                        if (tally_cc == true){
                            destination = 0xcc;
                            receiverMode = 0x05;
                            sendMessage();
                            delay(500);
                        }
                        if (tally_dd == true){
                            destination = 0xdd;
                            receiverMode = 0x05;
                            sendMessage();
                            delay(500);
                        }
                        if (tally_ee == true){
                            destination = 0xee;
                            receiverMode = 0x05;
                            sendMessage();
                        }

                        request->send(SPIFFS, "/configuration.html", String(), false, proc_state);
                        delay(2000);
                        ESP.restart();
                    } else{
                        request->send(SPIFFS, "/configuration.html", String(), false, proc_state);
                    }

                }
            }

            if(input_txp == "" || errortxp == true) {
                request->send(SPIFFS, "/formaterror.html", String(), false, proc_state);
            }

        } else {
            request->send(SPIFFS, "/login.html", String(), false, proc_state);
        }
    });

    // Send a GET request to <ESP_IP>/get?input1=<inputMessage>
    server.on("/get-port", HTTP_GET, [] (AsyncWebServerRequest *request) {
        if (authenticated == true) {
            String input_port;
            String input_param_port;

            // GET input6 value on <ESP_IP>/get?input6=<inputMessage>
            if (request->hasParam(param_udpport)) {
            input_port = request->getParam(param_udpport)->value();
            input_param_port = param_udpport;
            }
            // If empty, print no message
            if (request->hasParam("")) {
            input_port = "No message sent";
            input_param_port = "none";
            }

            errorport = false;

            if(input_port != "") {
                sprintf(buf_udpPort, "%s", input_port);
                udpPortNew = atoi(buf_udpPort);

                if ((udpPortNew < 999) || (udpPortNew > 10000)) {
                    Serial.println("ERROR!");
                    errorport = true;
                }
                else {

                    if (udpPortNew != udpPort) {

                        eeprom.begin("network", false);                //false mean use read/write mode
                        eeprom.putInt("udpport", udpPortNew);
                        Serial.print("udpPortNew: "); Serial.println(udpPortNew);     
                        eeprom.end(); 

                        udpPort = udpPortNew;

                        Serial.print("New UDP-Port: "); Serial.println(udpPortNew);

                        request->send(SPIFFS, "/network.html", String(), false, proc_state);
                    } else{
                        request->send(SPIFFS, "/network.html", String(), false, proc_state);
                    }

                }
            }

            if(input_port == "" || errorport == true) {
                request->send(SPIFFS, "/formaterror.html", String(), false, proc_state);
            }

        } else {
            request->send(SPIFFS, "/login.html", String(), false, proc_state);
        }
    });

    // Send a GET request to <ESP_IP>/get?input1=<inputMessage>
    server.on("/discover-time", HTTP_GET, [] (AsyncWebServerRequest *request) {
        if (authenticated == true) {
            String input_discoverTime;
            String input_param_discoverTime;

            // GET input6 value on <ESP_IP>/get?input6=<inputMessage>
            if (request->hasParam(param_distime)) {
            input_discoverTime = request->getParam(param_distime)->value();
            input_param_discoverTime = param_distime;
            }
            // If empty, print no message
            if (request->hasParam("")) {
            input_discoverTime = "No message sent";
            input_param_discoverTime = "none";
            }

            errorDiscoverTime = false;

            if(input_discoverTime != "") {
                sprintf(buf_discoverTime, "%s", input_discoverTime);
                discoverTimeNew = atoi(buf_discoverTime);

                if ((discoverTimeNew < 30) || (discoverTimeNew > 600)) {
                    Serial.println("ERROR!");
                    errorDiscoverTime = true;
                }
                else {

                    if (discoverTimeNew != discoverTime) {

                        eeprom.begin("configuration", false);                //false mean use read/write mode
                        eeprom.putInt("distime", discoverTimeNew);    
                        Serial.print("discoverTimeNew: "); Serial.println(discoverTimeNew); 
                        eeprom.end(); 

                        discoverTime = discoverTimeNew;

                        Serial.print("New Discover Time: "); Serial.println(discoverTimeNew);

                        request->send(SPIFFS, "/configuration.html", String(), false, proc_state);
                    } else{
                        request->send(SPIFFS, "/configuration.html", String(), false, proc_state);
                    }

                }
            }

            if(input_discoverTime == "" || errorDiscoverTime == true) {
                request->send(SPIFFS, "/formaterror.html", String(), false, proc_state);
            }

        } else {
            request->send(SPIFFS, "/login.html", String(), false, proc_state);
        }
    });


    // Send a GET request to <ESP_IP>/get?input1=<inputMessage>
    server.on("/tsl-assignment", HTTP_GET, [] (AsyncWebServerRequest *request) {
        if (authenticated == true) {
            String input_tslbbassi, input_tslccassi, input_tslddassi, input_tsleeassi;
            String input_param_tslbbassi, input_param_tslccassi, input_param_tslddassi, input_param_tsleeassi;

            if (request->hasParam(param_bbassi)) {
            input_tslbbassi = request->getParam(param_bbassi)->value();
            input_param_tslbbassi = param_bbassi;
            }
            if (request->hasParam(param_ccassi)) {
            input_tslccassi = request->getParam(param_ccassi)->value();
            input_param_tslccassi = param_ccassi;
            }
            if (request->hasParam(param_ddassi)) {
            input_tslddassi = request->getParam(param_ddassi)->value();
            input_param_tslddassi = param_ddassi;
            }
            if (request->hasParam(param_eeassi)) {
            input_tsleeassi = request->getParam(param_eeassi)->value();
            input_param_tsleeassi = param_eeassi;
            }
            // If empty, print no message
            if (request->hasParam("")) {
            input_tslbbassi = "No message sent";
            input_param_tslbbassi = "none";
            }
            // If empty, print no message
            if (request->hasParam("")) {
            input_tslccassi = "No message sent";
            input_param_tslccassi = "none";
            }
            // If empty, print no message
            if (request->hasParam("")) {
            input_tslddassi = "No message sent";
            input_param_tslddassi = "none";
            }
            // If empty, print no message
            if (request->hasParam("")) {
            input_tsleeassi = "No message sent";
            input_param_tsleeassi = "none";
            }

            errorTslBBAssi = false;

            if(input_tslbbassi != "") {
                sprintf(buf_tslbbassi, "%s", input_tslbbassi);
                tslbbassiNew = atoi(buf_tslbbassi);

                if ((tslbbassiNew < 1) || (tslbbassiNew > 8)) {
                    Serial.println("ERROR!");
                    errorTslBBAssi = true;
                }
                else {

                    if (tslbbassiNew != tslbbassi) {

                        eeprom.begin("configuration", false);                //false mean use read/write mode
                        eeprom.putInt("tslbbid", tslbbassiNew);    
                        Serial.print("tslbbassiNew: "); Serial.println(tslbbassiNew); 
                        eeprom.end(); 

                        tslbbassi = tslbbassiNew;

                        Serial.print("tslbbassiNew: "); Serial.println(tslbbassiNew);
                    } 
                }
            }

            errorTslCCAssi = false;

            if(input_tslccassi != "") {
                sprintf(buf_tslccassi, "%s", input_tslccassi);
                tslccassiNew = atoi(buf_tslccassi);

                if ((tslccassiNew < 1) || (tslccassiNew > 8)) {
                    Serial.println("ERROR!");
                    errorTslCCAssi = true;
                }
                else {

                    if (tslccassiNew != tslccassi) {

                        eeprom.begin("configuration", false);                //false mean use read/write mode
                        eeprom.putInt("tslccid", tslccassiNew);    
                        Serial.print("tslccassiNew: "); Serial.println(tslccassiNew); 
                        eeprom.end(); 

                        tslccassi = tslccassiNew;

                        Serial.print("tslccassiNew: "); Serial.println(tslccassiNew);
                    } 
                }
            }

            errorTslDDAssi = false;

            if(input_tslddassi != "") {
                sprintf(buf_tslddassi, "%s", input_tslddassi);
                tslddassiNew = atoi(buf_tslddassi);

                if ((tslddassiNew < 1) || (tslddassiNew > 8)) {
                    Serial.println("ERROR!");
                    errorTslDDAssi = true;
                }
                else {

                    if (tslddassiNew != tslddassi) {

                        eeprom.begin("configuration", false);                //false mean use read/write mode
                        eeprom.putInt("tslddid", tslddassiNew);    
                        Serial.print("tslddassiNew: "); Serial.println(tslddassiNew); 
                        eeprom.end(); 

                        tslddassi = tslddassiNew;

                        Serial.print("tslddassiNew: "); Serial.println(tslddassiNew);
                    } 
                }
            }

            errorTslEEAssi = false;

            if(input_tsleeassi != "") {
                sprintf(buf_tsleeassi, "%s", input_tsleeassi);
                tsleeassiNew = atoi(buf_tsleeassi);

                if ((tsleeassiNew < 1) || (tsleeassiNew > 8)) {
                    Serial.println("ERROR!");
                    errorTslEEAssi = true;
                }
                else {

                    if (tsleeassiNew != tsleeassi) {

                        eeprom.begin("configuration", false);                //false mean use read/write mode
                        eeprom.putInt("tsleeid", tsleeassiNew);    
                        Serial.print("tsleeassiNew: "); Serial.println(tsleeassiNew); 
                        eeprom.end(); 

                        tsleeassi = tsleeassiNew;

                        Serial.print("tsleeassiNew: "); Serial.println(tsleeassiNew);
                    }
                }
            }

            if(input_tslbbassi == "" || input_tslccassi == "" || input_tslddassi == "" || input_tsleeassi == "" || errorTslBBAssi == true || errorTslCCAssi == true || errorTslDDAssi == true || errorTslEEAssi == true) {
                request->send(SPIFFS, "/formaterror.html", String(), false, proc_state);
            } else {
                request->send(SPIFFS, "/configuration.html", String(), false, proc_state);
            }

        } else {
            request->send(SPIFFS, "/login.html", String(), false, proc_state);
        }
    });


    // Send a GET request to <ESP_IP>/get?input1=<inputMessage>
    server.on("/user-change", HTTP_GET, [] (AsyncWebServerRequest *request) {
        if (authenticated == true) {
            String input_new_username, input_new_password;
            String input_param_new_username, input_param_new_password;

            // GET input6 value on <ESP_IP>/get?input6=<inputMessage>
            if (request->hasParam(param_new_user)) {
            input_new_username = request->getParam(param_new_user)->value();
            input_param_new_username = param_new_user;
            }
            // GET input6 value on <ESP_IP>/get?input6=<inputMessage>
            if (request->hasParam(param_new_password)) {
            input_new_password = request->getParam(param_new_password)->value();
            input_param_new_password = param_new_password;
            }
            // If empty, print no message
            if (request->hasParam("")) {
            input_new_username = "No message sent";
            input_param_new_username = "none";
            input_new_password = "No message sent";
            input_param_new_password = "none";
            }

            eeprom.begin("configuration", false);                //false mean use read/write mode
            eeprom.putString("user", input_new_username);  
            Serial.print("input_new_username: "); Serial.println(input_new_username); 
            eeprom.putString("password", input_new_password);   
            Serial.print("input_new_password: "); Serial.println(input_new_password);  
            eeprom.end(); 

            request->send(SPIFFS, "/configuration.html", String(), false, proc_state);

        } else {
            request->send(SPIFFS, "/login.html", String(), false, proc_state);
        }
    });

    // Send a GET request to <ESP_IP>/get?input1=<inputMessage>
    server.on("/get-ip", HTTP_GET, [] (AsyncWebServerRequest *request) {
        if (authenticated == true) {
            String input_ip, input_gw, input_sn, input_dns1, input_dns2;
            String input_param_ip, input_param_gw, input_param_sn, input_param_dns1, input_param_dns2;

            // GET input1 value on <ESP_IP>/get?input1=<inputMessage>
            if (request->hasParam(param_ip)) {
            input_ip = request->getParam(param_ip)->value();
            input_param_ip = param_ip;
            }
            // GET input2 value on <ESP_IP>/get?input2=<inputMessage>
            if (request->hasParam(param_gw)) {
            input_gw = request->getParam(param_gw)->value();
            input_param_gw = param_gw;
            }
            // GET input3 value on <ESP_IP>/get?input3=<inputMessage>
            if (request->hasParam(param_sn)) {
            input_sn = request->getParam(param_sn)->value();
            input_param_sn = param_sn;
            }
            // GET input4 value on <ESP_IP>/get?input4=<inputMessage>
            if (request->hasParam(param_dns1)) {
            input_dns1 = request->getParam(param_dns1)->value();
            input_param_dns1 = param_dns1;
            }
            // GET input5 value on <ESP_IP>/get?input5=<inputMessage>
            if (request->hasParam(param_dns2)) {
            input_dns2 = request->getParam(param_dns2)->value();
            input_param_dns2 = param_dns2;
            }
            // If empty, print no message
            if (request->hasParam("")) {
            input_ip = "No message sent";
            input_param_ip = "none";
            input_gw = "No message sent";
            input_param_gw = "none";
            input_sn = "No message sent";
            input_param_sn = "none";
            input_dns1 = "No message sent";
            input_param_dns1 = "none";
            input_dns2 = "No message sent";
            input_param_dns2 = "none";
            }

            sprintf(buf_input_ip, "%s", "");
            sprintf(buf_input_gw, "%s", "");
            sprintf(buf_input_sn, "%s", "");
            sprintf(buf_input_dns1, "%s", "");
            sprintf(buf_input_dns2, "%s", "");

            sprintf(buf_input_ip, "%s", input_ip);
            sprintf(buf_input_gw, "%s", input_gw);
            sprintf(buf_input_sn, "%s", input_sn);
            sprintf(buf_input_dns1, "%s", input_dns1);
            sprintf(buf_input_dns2, "%s", input_dns2);

            errorip = false;
            notempty = false;

            if(input_ip != "") {
                notempty = true;
                char* ptr_ip = strtok(buf_input_ip, ".");
                int_ipOctet1 = atoi(ptr_ip);
                ptr_ip = strtok(NULL, ".");
                int_ipOctet2 = atoi(ptr_ip);
                ptr_ip = strtok(NULL, ".");
                int_ipOctet3 = atoi(ptr_ip);
                ptr_ip = strtok(NULL, ".");
                int_ipOctet4 = atoi(ptr_ip);
                Serial.print(int_ipOctet1); Serial.print("."); Serial.print(int_ipOctet2); Serial.print("."); Serial.print(int_ipOctet3); Serial.print("."); Serial.println(int_ipOctet4);

                if ((int_ipOctet1 < 0) || (int_ipOctet1 > 255) || (int_ipOctet2 < 0) || (int_ipOctet2 > 255) || (int_ipOctet3 < 0) || (int_ipOctet3 > 255) || (int_ipOctet4 < 0) || (int_ipOctet4 > 255)) {
                    Serial.println("ERROR 1!");
                    errorip = true;
                }
                else {
                    eeprom.begin("network", false);                //false mean use read/write mode
                    eeprom.putInt("ipOctet1", int_ipOctet1);  
                    Serial.print("int_ipOctet1: "); Serial.println(int_ipOctet1); 
                    eeprom.putInt("ipOctet2", int_ipOctet2);   
                    Serial.print("int_ipOctet2: "); Serial.println(int_ipOctet2);  
                    eeprom.putInt("ipOctet3", int_ipOctet3);
                    Serial.print("int_ipOctet3: "); Serial.println(int_ipOctet3); 
                    eeprom.putInt("ipOctet4", int_ipOctet4);  
                    Serial.print("int_ipOctet4: "); Serial.println(int_ipOctet4);           
                    eeprom.end();
                }
            }

            if(input_gw != "") {
                notempty = true;
                char* ptr_gw = strtok(buf_input_gw, ".");
                int_gwOctet1 = atoi(ptr_gw);
                ptr_gw = strtok(NULL, ".");
                int_gwOctet2 = atoi(ptr_gw);
                ptr_gw = strtok(NULL, ".");
                int_gwOctet3 = atoi(ptr_gw);
                ptr_gw = strtok(NULL, ".");
                int_gwOctet4 = atoi(ptr_gw);
                Serial.print(int_gwOctet1); Serial.print("."); Serial.print(int_gwOctet2); Serial.print("."); Serial.print(int_gwOctet3); Serial.print("."); Serial.println(int_gwOctet4);

                if ((int_gwOctet1 < 0) || (int_gwOctet1 > 255) || (int_gwOctet2 < 0) || (int_gwOctet2 > 255) || (int_gwOctet3 < 0) || (int_gwOctet3 > 255) || (int_gwOctet4 < 0) || (int_gwOctet4 > 255)) {
                    Serial.println("ERROR 2!");
                    errorip = true;
                }
                else {
                    eeprom.begin("network", false);                //false mean use read/write mode     
                    eeprom.putInt("gwOctet1", int_gwOctet1);  
                    eeprom.putInt("gwOctet2", int_gwOctet2);    
                    eeprom.putInt("gwOctet3", int_gwOctet3);
                    eeprom.putInt("gwOctet4", int_gwOctet4);           
                    eeprom.end();
                }
            }

            if(input_sn != "") {
                notempty = true;
                char* ptr_sn = strtok(buf_input_sn, ".");
                int_snOctet1 = atoi(ptr_sn);
                ptr_sn = strtok(NULL, ".");
                int_snOctet2 = atoi(ptr_sn);
                ptr_sn = strtok(NULL, ".");
                int_snOctet3 = atoi(ptr_sn);
                ptr_sn = strtok(NULL, ".");
                int_snOctet4 = atoi(ptr_sn);
                Serial.print(int_snOctet1); Serial.print("."); Serial.print(int_snOctet2); Serial.print("."); Serial.print(int_snOctet3); Serial.print("."); Serial.println(int_snOctet4);

                if ((int_snOctet1 < 0) || (int_snOctet1 > 255) || (int_snOctet2 < 0) || (int_snOctet2 > 255) || (int_snOctet3 < 0) || (int_snOctet3 > 255) || (int_snOctet4 < 0) || (int_snOctet4 > 255)) {
                    Serial.println("ERROR 3!");
                    errorip = true;
                }
                else {
                    eeprom.begin("network", false);                //false mean use read/write mode          
                    eeprom.putInt("snOctet1", int_snOctet1);  
                    eeprom.putInt("snOctet2", int_snOctet2);    
                    eeprom.putInt("snOctet3", int_snOctet3);
                    eeprom.putInt("snOctet4", int_snOctet4);           
                    eeprom.end();
                }
            }

            if(input_dns1 != "") {
                notempty = true;
                char* ptr_dns1 = strtok(buf_input_dns1, ".");
                int_dns1Octet1 = atoi(ptr_dns1);
                ptr_dns1 = strtok(NULL, ".");
                int_dns1Octet2 = atoi(ptr_dns1);
                ptr_dns1 = strtok(NULL, ".");
                int_dns1Octet3 = atoi(ptr_dns1);
                ptr_dns1 = strtok(NULL, ".");
                int_dns1Octet4 = atoi(ptr_dns1);
                Serial.print(int_dns1Octet1); Serial.print("."); Serial.print(int_dns1Octet2); Serial.print("."); Serial.print(int_dns1Octet3); Serial.print("."); Serial.println(int_dns1Octet4);

                if ((int_dns1Octet1 < 0) || (int_dns1Octet1 > 255) || (int_dns1Octet2 < 0) || (int_dns1Octet2 > 255) || (int_dns1Octet3 < 0) || (int_dns1Octet3 > 255) || (int_dns1Octet4 < 0) || (int_dns1Octet4 > 255)) {
                    Serial.println("ERROR 4!");
                    errorip = true;
                }
                else {
                    eeprom.begin("network", false);                //false mean use read/write mode          
                    eeprom.putInt("dns1Octet1", int_dns1Octet1);  
                    eeprom.putInt("dns1Octet2", int_dns1Octet2);    
                    eeprom.putInt("dns1Octet3", int_dns1Octet3);
                    eeprom.putInt("dns1Octet4", int_dns1Octet4);           
                    eeprom.end();
                }
            }

            if(input_dns2 != "") {
                notempty = true;
                char* ptr_dns2 = strtok(buf_input_dns2, ".");
                int_dns2Octet1 = atoi(ptr_dns2);
                ptr_dns2 = strtok(NULL, ".");
                int_dns2Octet2 = atoi(ptr_dns2);
                ptr_dns2 = strtok(NULL, ".");
                int_dns2Octet3 = atoi(ptr_dns2);
                ptr_dns2 = strtok(NULL, ".");
                int_dns2Octet4 = atoi(ptr_dns2);
                Serial.print(int_dns2Octet1); Serial.print("."); Serial.print(int_dns2Octet2); Serial.print("."); Serial.print(int_dns2Octet3); Serial.print("."); Serial.println(int_dns2Octet4);
                
                if ((int_dns2Octet1 < 0) || (int_dns2Octet1 > 255) || (int_dns2Octet2 < 0) || (int_dns2Octet2 > 255) || (int_dns2Octet3 < 0) || (int_dns2Octet3 > 255) || (int_dns2Octet4 < 0) || (int_dns2Octet4 > 255)) {
                    Serial.println("ERROR 5!");
                    errorip = true;
                }
                else {
                    eeprom.begin("network", false);                //false mean use read/write mode         
                    eeprom.putInt("dns2Octet1", int_dns2Octet1);  
                    eeprom.putInt("dns2Octet2", int_dns2Octet2);    
                    eeprom.putInt("dns2Octet3", int_dns2Octet3);
                    eeprom.putInt("dns2Octet4", int_dns2Octet4);      
                    eeprom.end();
                }
            }

            if (errorip == true || notempty == false) {
                request->send(SPIFFS, "/formaterror.html", String(), false, proc_state);
            }

            if (errorip == false && notempty == true){
                request->send(SPIFFS, "/network.html", String(), false, proc_state);
                delay(2000);
                ESP.restart();
            }

        
        } else {
                request->send(SPIFFS, "/login.html", String(), false, proc_state);
        }
    });


    server.on("/login", HTTP_POST, handleLogin);
    server.on("/logout", HTTP_GET, handleLogout);


    server.onNotFound([](AsyncWebServerRequest *request) {               // If the client requests any URI
        Serial.println("WEBS: Website not founded!");
        handleNotFound(request); // otherwise, respond with a 404 (Not Found) error
    });


    Serial.println("AUTH set cache.");
    // Serve a file with no cache so every tile It's downloaded
    server.serveStatic("/configuration.json", SPIFFS, "/configuration.json", "no-cache, no-store, must-revalidate");
    // Server all other page with long cache so browser chaching they
    // Comment this line for esp8266
    server.serveStatic("/", SPIFFS, "/", "max-age=31536000");


    server.begin();
    Serial.println("HTTP server started.");
    sprintf(buf_httpInit, "%s", "HTTP started");
    u8g2.drawStr(0,60,buf_httpInit);
    u8g2.sendBuffer();
    delay(500);

//////////////////////////////////////////////////////////////////////
 
    printLora(1);
    delay(2500);

    clearValues();
    printDisplay();

    digitalWrite(DISPLAY_CS, true);
    SPI.end();
    //SPI.endTransaction();

    startSPI_LORA();

}

//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////

void loop() {

    // Discover Mode
    if ((mode == "discover")) {
        destination = 0xff;
        receiverMode = 0x01;
        receiverState = 0x00;
        receiverColor = 0x00;
        sendMessage();
        lastOfferTime = millis();
        lastOfferTimeRef = millis();
        lastDiscoverTimebb = millis();
        lastDiscoverTimecc = millis();
        lastDiscoverTimedd = millis();
        lastDiscoverTimeee = millis();
        mode = "offer";
        mode_s = "off";
        clearValues();
    }

    // Offer Mode
    while (mode == "offer") {
        onReceive(LoRa.parsePacket(), &rx_adr, &tx_adr, &rssi, &bL, &receiverMode, &receiverState, &receiverColor);    // Parse Packets and Read it

        if ((receiverMode = 0x02) && (tx_adr == "bb") && (tally_bb == false)) {
            tally_bb = true;
            tally_bb_init = true;
            tx_adr_bb = tx_adr;
            rssi_bb = rssi;

            sprintf(buf_rssi, "%s", rssi);          //Register value string rssi convert into buffer char rssi
            buf_rssi_bb_int = atoi(buf_rssi);              //Convert char rssi in int rssi

            bL_bb = bL;
            counterTallys++;
            lastOfferTime = millis();
            lastOfferTimeEnd = millis();
            clearValues();
        }
        if ((receiverMode = 0x02) && (tx_adr == "cc") && (tally_cc == false)) {
            tally_cc = true;
            tally_cc_init = true;
            tx_adr_cc = tx_adr;
            rssi_cc = rssi;
            bL_cc = bL;
            counterTallys++;
            lastOfferTime = millis();
            lastOfferTimeEnd = millis();
            clearValues();
        }
        if ((receiverMode = 0x02) && (tx_adr == "dd") && (tally_dd == false)) {
            tally_dd = true;
            tally_dd_init = true;
            tx_adr_dd = tx_adr;
            rssi_dd = rssi;
            bL_dd = bL;
            counterTallys++;
            lastOfferTime = millis();
            lastOfferTimeEnd = millis();
            clearValues();
        }
        if ((receiverMode = 0x02) && (tx_adr == "ee") && (tally_ee == false)) {
            tally_ee = true;
            tally_ee_init = true;
            tx_adr_ee = tx_adr;
            rssi_ee = rssi;
            bL_ee = bL;
            counterTallys++;
            lastOfferTime = millis();
            lastOfferTimeEnd = millis();
            clearValues();
        }

        if ((millis() - lastOfferTimeRef > 2500)) {      // every 2.5 s clear display
            clearValues();
            lastOfferTimeRef = millis();
        }

        if ((millis() - lastOfferTime > 7500) && ((counterTallys >= 0))) {      // every 7.5 s on discover message
            mode = "discover"; 
            mode_s = "dis";
            break;
        }

        if ((millis() - lastOfferTimeEnd > discoverTime * 1000) && (counterTallys >= 1)) {    // after 120 s of no new receivers, request mode
            mode = "request"; 
            mode_s = "req";
            break;
        }
        }
  
    // Request Mode GPIO
    if ((mode == "request") && (millis() - lastAnalogReadTime > 300) && (useTSL == false)) {

        gpioV1 = analogRead(gpioP1);
        gpioV2 = analogRead(gpioP2);
        gpioV3 = analogRead(gpioP3);
        gpioV4 = analogRead(gpioP4);
        gpioV1Map = map(gpioV1, 0, 4095, 0, 4095);
        gpioV2Map = map(gpioV2, 0, 4095, 0, 4095);
        gpioV3Map = map(gpioV3, 0, 4095, 0, 4095);
        gpioV4Map = map(gpioV4, 0, 4095, 0, 4095); 
        gpioV1Cal = gpioV1Map * (3.3 / 4095.0);
        gpioV2Cal = gpioV2Map * (3.3 / 4095.0);
        gpioV3Cal = gpioV3Map * (3.3 / 4095.0);
        gpioV4Cal = gpioV4Map * (3.3 / 4095.0);

        
        Serial.print("gpioV1Cal: "); Serial.println(gpioV1Cal);
        Serial.print("gpioV2Cal: "); Serial.println(gpioV2Cal);
        Serial.print("gpioV3Cal: "); Serial.println(gpioV3Cal);
        Serial.print("gpioV4Cal: "); Serial.println(gpioV4Cal);
        Serial.print("tally_bb: "); Serial.println(tally_bb);
        Serial.print("tally_cc: "); Serial.println(tally_cc);
        Serial.print("tally_dd: "); Serial.println(tally_dd);
        Serial.print("tally_ee: "); Serial.println(tally_ee);
        Serial.print("gpioC1: "); Serial.println(gpioC1);
        Serial.print("gpioC2: "); Serial.println(gpioC2);
        Serial.print("gpioC3: "); Serial.println(gpioC3);
        Serial.print("gpioC4: "); Serial.println(gpioC4);
        

        if ((gpioV1Cal > 2.8 && tally_bb == true && gpioC1 == true && (millis() - lastSwitchTime > 100))) {
            destination = 0xbb;
            receiverMode = 0x03;
            receiverState = 0x01;
            receiverColor = 0x01;
            sendMessage();
            gpioC1 = !gpioC1;
            mode = "acknowledge";
            mode_s = "ack";
            lastAckTime = millis();
            lastSwitchTime = millis();
            clearValues();
        }

        if ((gpioV1Cal < 2.8 && tally_bb == true && gpioC1 == false && (millis() - lastSwitchTime > 100))) {
            destination = 0xbb;
            receiverMode = 0x03;
            receiverState = 0x00;
            receiverColor = 0x00;
            sendMessage();
            gpioC1 = !gpioC1;
            mode = "acknowledge";
            mode_s = "ack";
            lastAckTime = millis();
            lastSwitchTime = millis();
            clearValues();
        }

        if ((gpioV2Cal > 2.8 && tally_cc == true && gpioC2 == true && (millis() - lastSwitchTime > 100))) {
            destination = 0xcc;
            receiverMode = 0x03;
            receiverState = 0x01;
            receiverColor = 0x01;
            sendMessage();
            gpioC2 = !gpioC2;
            mode = "acknowledge";
            mode_s = "ack";
            lastAckTime = millis();
            lastSwitchTime = millis();
            clearValues();
        }

        if ((gpioV2Cal < 2.8 && tally_cc == true && gpioC2 == false && (millis() - lastSwitchTime > 100))) {
            destination = 0xcc;
            receiverMode = 0x03;
            receiverState = 0x00;
            receiverColor = 0x00;
            sendMessage();
            gpioC2 = !gpioC2;
            mode = "acknowledge";
            mode_s = "ack";
            lastAckTime = millis();
            lastSwitchTime = millis();
            clearValues();
        }

        if ((gpioV3Cal > 2.8 && tally_dd == true && gpioC3 == true && (millis() - lastSwitchTime > 100))) {
            destination = 0xdd;
            receiverMode = 0x03;
            receiverState = 0x01;
            receiverColor = 0x01;
            sendMessage();
            gpioC3 = !gpioC3;
            mode = "acknowledge";
            mode_s = "ack";
            lastAckTime = millis();
            lastSwitchTime = millis();
            clearValues();
        }

        if ((gpioV3Cal < 2.8 && tally_dd == true && gpioC3 == false && (millis() - lastSwitchTime > 100))) {
            destination = 0xdd;
            receiverMode = 0x03;
            receiverState = 0x00;
            receiverColor = 0x00;
            sendMessage();
            gpioC3 = !gpioC3;
            mode = "acknowledge";
            mode_s = "ack";
            lastAckTime = millis();
            lastSwitchTime = millis();
            clearValues();
        }

        if ((gpioV4Cal > 2.8 && tally_ee == true && gpioC4 == true && (millis() - lastSwitchTime > 100))) {
            destination = 0xee;
            receiverMode = 0x03;
            receiverState = 0x01;
            receiverColor = 0x01;
            sendMessage();
            gpioC4 = !gpioC4;
            mode = "acknowledge";
            mode_s = "ack";
            lastAckTime = millis();
            lastSwitchTime = millis();
            clearValues();
        }

        if ((gpioV4Cal < 2.8 && tally_ee == true && gpioC4 == false && (millis() - lastSwitchTime > 100))) {
            destination = 0xee;
            receiverMode = 0x03;
            receiverState = 0x00;
            receiverColor = 0x00;
            sendMessage();
            gpioC4 = !gpioC4;
            mode = "acknowledge";
            mode_s = "ack";
            lastAckTime = millis();
            lastSwitchTime = millis();
            clearValues();
        }
        lastAnalogReadTime = millis();
    }



    // Request Mode TSL
    if ((mode == "request") && (millis() - lastTslReadTime > 50) && (useTSL == true)) {

        if (ethConnected) {
            
            if (tslAckReceived == true) {
                    
                if (udp.parsePacket() > 0) {

                    if (udp.available()) {

                        udp_len = udp.available();

                        for(int i = 0; i < udp_len; i++) {
                            int udp_dec = udp.read();
                            
                            udp_seq[i] = {udp_dec};

                            if (initUdpSteam == false) {
                                udp_seq_old[i] = {udp_dec};
                            }

                            if (first_udp_seq == true) {
                                Serial.print("UDP DATAGRAM: ");
                                first_udp_seq = false;
                            }
                                
                            Serial.print(udp_seq[i]); Serial.print(" ");
                        }

                        Serial.println(" ");


                        //init first old udp sequence
                        if (initUdpSteam == false) {
                            initUdpSteam = true;
                        }

                        //if someone changed i in the udp stream, dont block tsl
                        for(int i = 0; i < udp_len; i++) {

                            if (udp_seq[i] != udp_seq_old[i]) {
                                blockTsl = false;
                                break;
                            } else {
                                blockTsl = true;
                            }
                        }

                        //give the old udp sequence the new sequence
                        for(int i = 0; i < udp_len; i++) {
                            udp_seq_old[i] = udp_seq[i];
                        }


                        first_udp_seq = true;
                        sendSequence = true;
                    } 
                } 
            }
            

            
            if ((millis() - lastSerialPrint > 2000)) {      // print
                lastSerialPrint = millis();
            }
            


            //tally bb off
            if ((((tslbbassi == 1) && (tally_bb == true)) || ((tslccassi == 1) && (tally_cc == true)) || ((tslddassi == 1) && (tally_dd == true)) || ((tsleeassi == 1) && (tally_ee == true))) && (sendSequence == true) && (blockTsl == false) && (udp_seq[0] == 129) && (udp_seq[1] == 48) && (udp_seq[2] == 49) && (millis() - lastSwitchTime > 25)) {
                Serial.println("=bb off=");
                if ((tslbbassi == 1) && (tally_bb == true)){destination = 0xbb;}
                if ((tslccassi == 1) && (tally_cc == true)){destination = 0xcc;}
                if ((tslddassi == 1) && (tally_dd == true)){destination = 0xdd;}
                if ((tsleeassi == 1) && (tally_ee == true)){destination = 0xee;}
                receiverMode = 0x03;
                receiverState = 0x00;
                receiverColor = 0x00;
                sendMessage();
                mode = "acknowledge";
                mode_s = "ack";
                lastAckTime = millis();
                lastSwitchTime = millis();
                clearValues();
            }

            //tally bb green
            if ((((tslbbassi == 1) && (tally_bb == true)) || ((tslccassi == 1) && (tally_cc == true)) || ((tslddassi == 1) && (tally_dd == true)) || ((tsleeassi == 1) && (tally_ee == true))) && (sendSequence == true) && (blockTsl == false) && (udp_seq[0] == 166) && (udp_seq[1] == 49) && (udp_seq[2] == 49) && (millis() - lastSwitchTime > 25)) {
                Serial.println("=bb green=");
                if ((tslbbassi == 1) && (tally_bb == true)){destination = 0xbb;}
                if ((tslccassi == 1) && (tally_cc == true)){destination = 0xcc;}
                if ((tslddassi == 1) && (tally_dd == true)){destination = 0xdd;}
                if ((tsleeassi == 1) && (tally_ee == true)){destination = 0xee;}
                receiverMode = 0x03;
                receiverState = 0x01;
                receiverColor = 0x02;
                sendMessage();
                mode = "acknowledge";
                mode_s = "ack";
                lastAckTime = millis();
                lastSwitchTime = millis();
                clearValues();
            }

            //tally bb red
            if ((((tslbbassi == 1) && (tally_bb == true)) || ((tslccassi == 1) && (tally_cc == true)) || ((tslddassi == 1) && (tally_dd == true)) || ((tsleeassi == 1) && (tally_ee == true))) && (sendSequence == true) && (blockTsl == false) && (udp_seq[0] == 165) && (udp_seq[1] == 50) && (udp_seq[2] == 49) && (millis() - lastSwitchTime > 25)) {
                Serial.println("=bb red=");
                if ((tslbbassi == 1) && (tally_bb == true)){destination = 0xbb;}
                if ((tslccassi == 1) && (tally_cc == true)){destination = 0xcc;}
                if ((tslddassi == 1) && (tally_dd == true)){destination = 0xdd;}
                if ((tsleeassi == 1) && (tally_ee == true)){destination = 0xee;}
                destination = 0xbb;
                receiverMode = 0x03;
                receiverState = 0x01;
                receiverColor = 0x01;
                sendMessage();
                mode = "acknowledge";
                mode_s = "ack";
                lastAckTime = millis();
                lastSwitchTime = millis();
                clearValues();
            }

            //tally bb red wip
            if ((((tslbbassi == 1) && (tally_bb == true)) || ((tslccassi == 1) && (tally_cc == true)) || ((tslddassi == 1) && (tally_dd == true)) || ((tsleeassi == 1) && (tally_ee == true))) && (sendSequence == true) && (blockTsl == false) && (udp_seq[0] == 166) && (udp_seq[1] == 51) && (udp_seq[2] == 49) && (millis() - lastSwitchTime > 25)) {
                Serial.println("=bb red wip=");
                if ((tslbbassi == 1) && (tally_bb == true)){destination = 0xbb;}
                if ((tslccassi == 1) && (tally_cc == true)){destination = 0xcc;}
                if ((tslddassi == 1) && (tally_dd == true)){destination = 0xdd;}
                if ((tsleeassi == 1) && (tally_ee == true)){destination = 0xee;}
                receiverMode = 0x03;
                receiverState = 0x01;
                receiverColor = 0x01;
                sendMessage();
                mode = "acknowledge";
                mode_s = "ack";
                lastAckTime = millis();
                lastSwitchTime = millis();
                clearValues();
            }
            
            //tally cc off
            if ((((tslbbassi == 2) && (tally_bb == true)) || ((tslccassi == 2) && (tally_cc == true)) || ((tslddassi == 2) && (tally_dd == true)) || ((tsleeassi == 2) && (tally_ee == true))) && (sendSequence == true) && (blockTsl == false) && (udp_seq[0] == 130) && (udp_seq[1] == 48) && (udp_seq[2] == 50) && (millis() - lastSwitchTime > 25)) {
                Serial.println("=cc off=");
                if ((tslbbassi == 2) && (tally_bb == true)){destination = 0xbb;}
                if ((tslccassi == 2) && (tally_cc == true)){destination = 0xcc;}
                if ((tslddassi == 2) && (tally_dd == true)){destination = 0xdd;}
                if ((tsleeassi == 2) && (tally_ee == true)){destination = 0xee;}
                receiverMode = 0x03;
                receiverState = 0x00;
                receiverColor = 0x00;
                sendMessage();
                mode = "acknowledge";
                mode_s = "ack";
                lastAckTime = millis();
                lastSwitchTime = millis();
                clearValues();
            }

            //tally cc green
            if ((((tslbbassi == 2) && (tally_bb == true)) || ((tslccassi == 2) && (tally_cc == true)) || ((tslddassi == 2) && (tally_dd == true)) || ((tsleeassi == 2) && (tally_ee == true))) && (sendSequence == true) && (blockTsl == false) && (udp_seq[0] == 166) && (udp_seq[1] == 49) && (udp_seq[2] == 50) && (millis() - lastSwitchTime > 25)) {
                Serial.println("=cc green=");
                if ((tslbbassi == 2) && (tally_bb == true)){destination = 0xbb;}
                if ((tslccassi == 2) && (tally_cc == true)){destination = 0xcc;}
                if ((tslddassi == 2) && (tally_dd == true)){destination = 0xdd;}
                if ((tsleeassi == 2) && (tally_ee == true)){destination = 0xee;}
                receiverMode = 0x03;
                receiverState = 0x01;
                receiverColor = 0x02;
                sendMessage();
                mode = "acknowledge";
                mode_s = "ack";
                lastAckTime = millis();
                lastSwitchTime = millis();
                clearValues();
            }

            //tally cc red
            if ((((tslbbassi == 2) && (tally_bb == true)) || ((tslccassi == 2) && (tally_cc == true)) || ((tslddassi == 2) && (tally_dd == true)) || ((tsleeassi == 2) && (tally_ee == true))) && (sendSequence == true) && (blockTsl == false) && (udp_seq[0] == 165) && (udp_seq[1] == 50) && (udp_seq[2] == 50) && (millis() - lastSwitchTime > 25)) {
                Serial.println("=cc red=");
                if ((tslbbassi == 2) && (tally_bb == true)){destination = 0xbb;}
                if ((tslccassi == 2) && (tally_cc == true)){destination = 0xcc;}
                if ((tslddassi == 2) && (tally_dd == true)){destination = 0xdd;}
                if ((tsleeassi == 2) && (tally_ee == true)){destination = 0xee;}
                receiverMode = 0x03;
                receiverState = 0x01;
                receiverColor = 0x01;
                sendMessage();
                mode = "acknowledge";
                mode_s = "ack";
                lastAckTime = millis();
                lastSwitchTime = millis();
                clearValues();
            }

            //tally cc red wip
            if ((((tslbbassi == 2) && (tally_bb == true)) || ((tslccassi == 2) && (tally_cc == true)) || ((tslddassi == 2) && (tally_dd == true)) || ((tsleeassi == 2) && (tally_ee == true))) && (sendSequence == true) && (blockTsl == false) && (udp_seq[0] == 166) && (udp_seq[1] == 51) && (udp_seq[2] == 50) && (millis() - lastSwitchTime > 25)) {
                Serial.println("=cc red wip=");
                if ((tslbbassi == 2) && (tally_bb == true)){destination = 0xbb;}
                if ((tslccassi == 2) && (tally_cc == true)){destination = 0xcc;}
                if ((tslddassi == 2) && (tally_dd == true)){destination = 0xdd;}
                if ((tsleeassi == 2) && (tally_ee == true)){destination = 0xee;}
                receiverMode = 0x03;
                receiverState = 0x01;
                receiverColor = 0x01;
                sendMessage();
                mode = "acknowledge";
                mode_s = "ack";
                lastAckTime = millis();
                lastSwitchTime = millis();
                clearValues();
            }
            
            //tally dd off
            if ((((tslbbassi == 3) && (tally_bb == true)) || ((tslccassi == 3) && (tally_cc == true)) || ((tslddassi == 3) && (tally_dd == true)) || ((tsleeassi == 3) && (tally_ee == true))) && (sendSequence == true) && (blockTsl == false) && (udp_seq[0] == 131) && (udp_seq[1] == 48) && (udp_seq[2] == 51) && (millis() - lastSwitchTime > 25)) {
                Serial.println("=dd off=");
                if ((tslbbassi == 3) && (tally_bb == true)){destination = 0xbb;}
                if ((tslccassi == 3) && (tally_cc == true)){destination = 0xcc;}
                if ((tslddassi == 3) && (tally_dd == true)){destination = 0xdd;}
                if ((tsleeassi == 3) && (tally_ee == true)){destination = 0xee;}
                receiverMode = 0x03;
                receiverState = 0x00;
                receiverColor = 0x00;
                sendMessage();
                mode = "acknowledge";
                mode_s = "ack";
                lastAckTime = millis();
                lastSwitchTime = millis();
                clearValues();
            }

            //tally dd green
            if ((((tslbbassi == 3) && (tally_bb == true)) || ((tslccassi == 3) && (tally_cc == true)) || ((tslddassi == 3) && (tally_dd == true)) || ((tsleeassi == 3) && (tally_ee == true))) && (sendSequence == true) && (blockTsl == false) && (udp_seq[0] == 166) && (udp_seq[1] == 49) && (udp_seq[2] == 51) && (millis() - lastSwitchTime > 25)) {
                Serial.println("=dd green=");
                if ((tslbbassi == 3) && (tally_bb == true)){destination = 0xbb;}
                if ((tslccassi == 3) && (tally_cc == true)){destination = 0xcc;}
                if ((tslddassi == 3) && (tally_dd == true)){destination = 0xdd;}
                if ((tsleeassi == 3) && (tally_ee == true)){destination = 0xee;}
                receiverMode = 0x03;
                receiverState = 0x01;
                receiverColor = 0x02;
                sendMessage();
                mode = "acknowledge";
                mode_s = "ack";
                lastAckTime = millis();
                lastSwitchTime = millis();
                clearValues();
            }

            //tally dd red
            if ((((tslbbassi == 3) && (tally_bb == true)) || ((tslccassi == 3) && (tally_cc == true)) || ((tslddassi == 3) && (tally_dd == true)) || ((tsleeassi == 3) && (tally_ee == true))) && (sendSequence == true) && (blockTsl == false) && (udp_seq[0] == 165) && (udp_seq[1] == 50) && (udp_seq[2] == 51) && (millis() - lastSwitchTime > 25)) {
                Serial.println("=dd red=");
                if ((tslbbassi == 3) && (tally_bb == true)){destination = 0xbb;}
                if ((tslccassi == 3) && (tally_cc == true)){destination = 0xcc;}
                if ((tslddassi == 3) && (tally_dd == true)){destination = 0xdd;}
                if ((tsleeassi == 3) && (tally_ee == true)){destination = 0xee;}
                receiverMode = 0x03;
                receiverState = 0x01;
                receiverColor = 0x01;
                sendMessage();
                mode = "acknowledge";
                mode_s = "ack";
                lastAckTime = millis();
                lastSwitchTime = millis();
                clearValues();
            }

            //tally dd red wip
            if ((((tslbbassi == 3) && (tally_bb == true)) || ((tslccassi == 3) && (tally_cc == true)) || ((tslddassi == 3) && (tally_dd == true)) || ((tsleeassi == 3) && (tally_ee == true))) && (sendSequence == true) && (blockTsl == false) && (udp_seq[0] == 166) && (udp_seq[1] == 51) && (udp_seq[2] == 51) && (millis() - lastSwitchTime > 25)) {
                Serial.println("=dd red wip=");
                if ((tslbbassi == 3) && (tally_bb == true)){destination = 0xbb;}
                if ((tslccassi == 3) && (tally_cc == true)){destination = 0xcc;}
                if ((tslddassi == 3) && (tally_dd == true)){destination = 0xdd;}
                if ((tsleeassi == 3) && (tally_ee == true)){destination = 0xee;}
                receiverMode = 0x03;
                receiverState = 0x01;
                receiverColor = 0x01;
                sendMessage();
                mode = "acknowledge";
                mode_s = "ack";
                lastAckTime = millis();
                lastSwitchTime = millis();
                clearValues();
            }

            //tally ee off
            if ((((tslbbassi == 4) && (tally_bb == true)) || ((tslccassi == 4) && (tally_cc == true)) || ((tslddassi == 4) && (tally_dd == true)) || ((tsleeassi == 4) && (tally_ee == true))) && (sendSequence == true) && (blockTsl == false) && (udp_seq[0] == 132) && (udp_seq[1] == 48) && (udp_seq[2] == 52) && (millis() - lastSwitchTime > 25)) {
                Serial.println("=ee off=");
                if ((tslbbassi == 4) && (tally_bb == true)){destination = 0xbb;}
                if ((tslccassi == 4) && (tally_cc == true)){destination = 0xcc;}
                if ((tslddassi == 4) && (tally_dd == true)){destination = 0xdd;}
                if ((tsleeassi == 4) && (tally_ee == true)){destination = 0xee;}
                receiverMode = 0x03;
                receiverState = 0x00;
                receiverColor = 0x00;
                sendMessage();
                mode = "acknowledge";
                mode_s = "ack";
                lastAckTime = millis();
                lastSwitchTime = millis();
                clearValues();
            }

            //tally ee green
            if ((((tslbbassi == 4) && (tally_bb == true)) || ((tslccassi == 4) && (tally_cc == true)) || ((tslddassi == 4) && (tally_dd == true)) || ((tsleeassi == 4) && (tally_ee == true))) && (sendSequence == true) && (blockTsl == false) && (udp_seq[0] == 166) && (udp_seq[1] == 49) && (udp_seq[2] == 52) && (millis() - lastSwitchTime > 25)) {
                Serial.println("=ee green=");
                if ((tslbbassi == 4) && (tally_bb == true)){destination = 0xbb;}
                if ((tslccassi == 4) && (tally_cc == true)){destination = 0xcc;}
                if ((tslddassi == 4) && (tally_dd == true)){destination = 0xdd;}
                if ((tsleeassi == 4) && (tally_ee == true)){destination = 0xee;}
                receiverMode = 0x03;
                receiverState = 0x01;
                receiverColor = 0x02;
                sendMessage();
                mode = "acknowledge";
                mode_s = "ack";
                lastAckTime = millis();
                lastSwitchTime = millis();
                clearValues();
            }

            //tally ee red
            if ((((tslbbassi == 4) && (tally_bb == true)) || ((tslccassi == 4) && (tally_cc == true)) || ((tslddassi == 4) && (tally_dd == true)) || ((tsleeassi == 4) && (tally_ee == true))) && (sendSequence == true) && (blockTsl == false) && (udp_seq[0] == 165) && (udp_seq[1] == 50) && (udp_seq[2] == 52) && (millis() - lastSwitchTime > 25)) {
                Serial.println("=ee red=");
                if ((tslbbassi == 4) && (tally_bb == true)){destination = 0xbb;}
                if ((tslccassi == 4) && (tally_cc == true)){destination = 0xcc;}
                if ((tslddassi == 4) && (tally_dd == true)){destination = 0xdd;}
                if ((tsleeassi == 4) && (tally_ee == true)){destination = 0xee;}
                receiverMode = 0x03;
                receiverState = 0x01;
                receiverColor = 0x01;
                sendMessage();
                mode = "acknowledge";
                mode_s = "ack";
                lastAckTime = millis();
                lastSwitchTime = millis();
                clearValues();
            }

            //tally ee red wip
            if ((((tslbbassi == 4) && (tally_bb == true)) || ((tslccassi == 4) && (tally_cc == true)) || ((tslddassi == 4) && (tally_dd == true)) || ((tsleeassi == 4) && (tally_ee == true))) && (sendSequence == true) && (blockTsl == false) && (udp_seq[0] == 166) && (udp_seq[1] == 51) && (udp_seq[2] == 52) && (millis() - lastSwitchTime > 25)) {
                Serial.println("=ee red wip=");
                if ((tslbbassi == 4) && (tally_bb == true)){destination = 0xbb;}
                if ((tslccassi == 4) && (tally_cc == true)){destination = 0xcc;}
                if ((tslddassi == 4) && (tally_dd == true)){destination = 0xdd;}
                if ((tsleeassi == 4) && (tally_ee == true)){destination = 0xee;}
                receiverMode = 0x03;
                receiverState = 0x01;
                receiverColor = 0x01;
                sendMessage();
                mode = "acknowledge";
                mode_s = "ack";
                lastAckTime = millis();
                lastSwitchTime = millis();
                clearValues();
            }

            //tally ww off
            if ((((tslbbassi == 5) && (tally_bb == true)) || ((tslccassi == 5) && (tally_cc == true)) || ((tslddassi == 5) && (tally_dd == true)) || ((tsleeassi == 5) && (tally_ee == true))) && (sendSequence == true) && (blockTsl == false) && (udp_seq[0] == 133) && (udp_seq[1] == 48) && (udp_seq[2] == 53) && (millis() - lastSwitchTime > 25)) {
                Serial.println("=ww off=");
                if ((tslbbassi == 5) && (tally_bb == true)){destination = 0xbb;}
                if ((tslccassi == 5) && (tally_cc == true)){destination = 0xcc;}
                if ((tslddassi == 5) && (tally_dd == true)){destination = 0xdd;}
                if ((tsleeassi == 5) && (tally_ee == true)){destination = 0xee;}
                receiverMode = 0x03;
                receiverState = 0x00;
                receiverColor = 0x00;
                sendMessage();
                mode = "acknowledge";
                mode_s = "ack";
                lastAckTime = millis();
                lastSwitchTime = millis();
                clearValues();
            }

            //tally ww green
            if ((((tslbbassi == 5) && (tally_bb == true)) || ((tslccassi == 5) && (tally_cc == true)) || ((tslddassi == 5) && (tally_dd == true)) || ((tsleeassi == 5) && (tally_ee == true))) && (sendSequence == true) && (blockTsl == false) && (udp_seq[0] == 166) && (udp_seq[1] == 49) && (udp_seq[2] == 53) && (millis() - lastSwitchTime > 25)) {
                Serial.println("=ww green=");
                if ((tslbbassi == 5) && (tally_bb == true)){destination = 0xbb;}
                if ((tslccassi == 5) && (tally_cc == true)){destination = 0xcc;}
                if ((tslddassi == 5) && (tally_dd == true)){destination = 0xdd;}
                if ((tsleeassi == 5) && (tally_ee == true)){destination = 0xee;}
                receiverMode = 0x03;
                receiverState = 0x01;
                receiverColor = 0x02;
                sendMessage();
                mode = "acknowledge";
                mode_s = "ack";
                lastAckTime = millis();
                lastSwitchTime = millis();
                clearValues();
            }

            //tally ww red
            if ((((tslbbassi == 5) && (tally_bb == true)) || ((tslccassi == 5) && (tally_cc == true)) || ((tslddassi == 5) && (tally_dd == true)) || ((tsleeassi == 5) && (tally_ee == true))) && (sendSequence == true) && (blockTsl == false) && (udp_seq[0] == 165) && (udp_seq[1] == 50) && (udp_seq[2] == 53) && (millis() - lastSwitchTime > 25)) {
                Serial.println("=ww red=");
                if ((tslbbassi == 5) && (tally_bb == true)){destination = 0xbb;}
                if ((tslccassi == 5) && (tally_cc == true)){destination = 0xcc;}
                if ((tslddassi == 5) && (tally_dd == true)){destination = 0xdd;}
                if ((tsleeassi == 5) && (tally_ee == true)){destination = 0xee;}
                receiverMode = 0x03;
                receiverState = 0x01;
                receiverColor = 0x01;
                sendMessage();
                mode = "acknowledge";
                mode_s = "ack";
                lastAckTime = millis();
                lastSwitchTime = millis();
                clearValues();
            }

            //tally ww red wip
            if ((((tslbbassi == 5) && (tally_bb == true)) || ((tslccassi == 5) && (tally_cc == true)) || ((tslddassi == 5) && (tally_dd == true)) || ((tsleeassi == 5) && (tally_ee == true))) && (sendSequence == true) && (blockTsl == false) && (udp_seq[0] == 166) && (udp_seq[1] == 51) && (udp_seq[2] == 53) && (millis() - lastSwitchTime > 25)) {
                Serial.println("=ww red wip=");
                if ((tslbbassi == 5) && (tally_bb == true)){destination = 0xbb;}
                if ((tslccassi == 5) && (tally_cc == true)){destination = 0xcc;}
                if ((tslddassi == 5) && (tally_dd == true)){destination = 0xdd;}
                if ((tsleeassi == 5) && (tally_ee == true)){destination = 0xee;}
                receiverMode = 0x03;
                receiverState = 0x01;
                receiverColor = 0x01;
                sendMessage();
                mode = "acknowledge";
                mode_s = "ack";
                lastAckTime = millis();
                lastSwitchTime = millis();
                clearValues();
            }

            //tally xx off
            if ((((tslbbassi == 6) && (tally_bb == true)) || ((tslccassi == 6) && (tally_cc == true)) || ((tslddassi == 6) && (tally_dd == true)) || ((tsleeassi == 6) && (tally_ee == true))) && (sendSequence == true) && (blockTsl == false) && (udp_seq[0] == 134) && (udp_seq[1] == 48) && (udp_seq[2] == 54) && (millis() - lastSwitchTime > 25)) {
                Serial.println("=xx off=");
                if ((tslbbassi == 6) && (tally_bb == true)){destination = 0xbb;}
                if ((tslccassi == 6) && (tally_cc == true)){destination = 0xcc;}
                if ((tslddassi == 6) && (tally_dd == true)){destination = 0xdd;}
                if ((tsleeassi == 6) && (tally_ee == true)){destination = 0xee;}
                receiverMode = 0x03;
                receiverState = 0x00;
                receiverColor = 0x00;
                sendMessage();
                mode = "acknowledge";
                mode_s = "ack";
                lastAckTime = millis();
                lastSwitchTime = millis();
                clearValues();
            }

            //tally xx green
            if ((((tslbbassi == 6) && (tally_bb == true)) || ((tslccassi == 6) && (tally_cc == true)) || ((tslddassi == 6) && (tally_dd == true)) || ((tsleeassi == 6) && (tally_ee == true))) && (sendSequence == true) && (blockTsl == false) && (udp_seq[0] == 166) && (udp_seq[1] == 49) && (udp_seq[2] == 54) && (millis() - lastSwitchTime > 25)) {
                Serial.println("=xx green=");
                if ((tslbbassi == 6) && (tally_bb == true)){destination = 0xbb;}
                if ((tslccassi == 6) && (tally_cc == true)){destination = 0xcc;}
                if ((tslddassi == 6) && (tally_dd == true)){destination = 0xdd;}
                if ((tsleeassi == 6) && (tally_ee == true)){destination = 0xee;}
                receiverMode = 0x03;
                receiverState = 0x01;
                receiverColor = 0x02;
                sendMessage();
                mode = "acknowledge";
                mode_s = "ack";
                lastAckTime = millis();
                lastSwitchTime = millis();
                clearValues();
            }

            //tally xx red
            if ((((tslbbassi == 6) && (tally_bb == true)) || ((tslccassi == 6) && (tally_cc == true)) || ((tslddassi == 6) && (tally_dd == true)) || ((tsleeassi == 6) && (tally_ee == true))) && (sendSequence == true) && (blockTsl == false) && (udp_seq[0] == 165) && (udp_seq[1] == 50) && (udp_seq[2] == 54) && (millis() - lastSwitchTime > 25)) {
                Serial.println("=xx red=");
                if ((tslbbassi == 6) && (tally_bb == true)){destination = 0xbb;}
                if ((tslccassi == 6) && (tally_cc == true)){destination = 0xcc;}
                if ((tslddassi == 6) && (tally_dd == true)){destination = 0xdd;}
                if ((tsleeassi == 6) && (tally_ee == true)){destination = 0xee;}
                receiverMode = 0x03;
                receiverState = 0x01;
                receiverColor = 0x01;
                sendMessage();
                mode = "acknowledge";
                mode_s = "ack";
                lastAckTime = millis();
                lastSwitchTime = millis();
                clearValues();
            }

            //tally xx red wip
            if ((((tslbbassi == 6) && (tally_bb == true)) || ((tslccassi == 6) && (tally_cc == true)) || ((tslddassi == 6) && (tally_dd == true)) || ((tsleeassi == 6) && (tally_ee == true))) && (sendSequence == true) && (blockTsl == false) && (udp_seq[0] == 166) && (udp_seq[1] == 51) && (udp_seq[2] == 54) && (millis() - lastSwitchTime > 25)) {
                Serial.println("=xx red wip=");
                if ((tslbbassi == 6) && (tally_bb == true)){destination = 0xbb;}
                if ((tslccassi == 6) && (tally_cc == true)){destination = 0xcc;}
                if ((tslddassi == 6) && (tally_dd == true)){destination = 0xdd;}
                if ((tsleeassi == 6) && (tally_ee == true)){destination = 0xee;}
                receiverMode = 0x03;
                receiverState = 0x01;
                receiverColor = 0x01;
                sendMessage();
                mode = "acknowledge";
                mode_s = "ack";
                lastAckTime = millis();
                lastSwitchTime = millis();
                clearValues();
            }

            //tally yy off
            if ((((tslbbassi == 7) && (tally_bb == true)) || ((tslccassi == 7) && (tally_cc == true)) || ((tslddassi == 7) && (tally_dd == true)) || ((tsleeassi == 7) && (tally_ee == true))) && (sendSequence == true) && (blockTsl == false) && (udp_seq[0] == 229) && (udp_seq[1] == 48) && (udp_seq[2] == 77) && (udp_seq[3] == 49)  && (millis() - lastSwitchTime > 25)) {
                Serial.println("=yy off=");
                if ((tslbbassi == 7) && (tally_bb == true)){destination = 0xbb;}
                if ((tslccassi == 7) && (tally_cc == true)){destination = 0xcc;}
                if ((tslddassi == 7) && (tally_dd == true)){destination = 0xdd;}
                if ((tsleeassi == 7) && (tally_ee == true)){destination = 0xee;}
                receiverMode = 0x03;
                receiverState = 0x00;
                receiverColor = 0x00;
                sendMessage();
                mode = "acknowledge";
                mode_s = "ack";
                lastAckTime = millis();
                lastSwitchTime = millis();
                clearValues();
            }

            //tally yy green
            if ((((tslbbassi == 7) && (tally_bb == true)) || ((tslccassi == 7) && (tally_cc == true)) || ((tslddassi == 7) && (tally_dd == true)) || ((tsleeassi == 7) && (tally_ee == true))) && (sendSequence == true) && (blockTsl == false) && (udp_seq[0] == 166) && (udp_seq[1] == 49) && (udp_seq[2] == 77) && (udp_seq[3] == 49) && (millis() - lastSwitchTime > 25)) {
                Serial.println("=yy green=");
                if ((tslbbassi == 7) && (tally_bb == true)){destination = 0xbb;}
                if ((tslccassi == 7) && (tally_cc == true)){destination = 0xcc;}
                if ((tslddassi == 7) && (tally_dd == true)){destination = 0xdd;}
                if ((tsleeassi == 7) && (tally_ee == true)){destination = 0xee;}
                receiverMode = 0x03;
                receiverState = 0x01;
                receiverColor = 0x02;
                sendMessage();
                mode = "acknowledge";
                mode_s = "ack";
                lastAckTime = millis();
                lastSwitchTime = millis();
                clearValues();
            }

            //tally yy red
            if ((((tslbbassi == 7) && (tally_bb == true)) || ((tslccassi == 7) && (tally_cc == true)) || ((tslddassi == 7) && (tally_dd == true)) || ((tsleeassi == 7) && (tally_ee == true))) && (sendSequence == true) && (blockTsl == false) && (udp_seq[0] == 165) && (udp_seq[1] == 50) && (udp_seq[2] == 77) && (udp_seq[3] == 49) && (millis() - lastSwitchTime > 25)) {
                Serial.println("=yy red=");
                if ((tslbbassi == 7) && (tally_bb == true)){destination = 0xbb;}
                if ((tslccassi == 7) && (tally_cc == true)){destination = 0xcc;}
                if ((tslddassi == 7) && (tally_dd == true)){destination = 0xdd;}
                if ((tsleeassi == 7) && (tally_ee == true)){destination = 0xee;}
                receiverMode = 0x03;
                receiverState = 0x01;
                receiverColor = 0x01;
                sendMessage();
                mode = "acknowledge";
                mode_s = "ack";
                lastAckTime = millis();
                lastSwitchTime = millis();
                clearValues();
            }

            //tally yy red wip
            if ((((tslbbassi == 7) && (tally_bb == true)) || ((tslccassi == 7) && (tally_cc == true)) || ((tslddassi == 7) && (tally_dd == true)) || ((tsleeassi == 7) && (tally_ee == true))) && (sendSequence == true) && (blockTsl == false) && (udp_seq[0] == 166) && (udp_seq[1] == 51) && (udp_seq[2] == 77) && (udp_seq[3] == 49) && (millis() - lastSwitchTime > 25)) {
                Serial.println("=yy red wip=");
                if ((tslbbassi == 7) && (tally_bb == true)){destination = 0xbb;}
                if ((tslccassi == 7) && (tally_cc == true)){destination = 0xcc;}
                if ((tslddassi == 7) && (tally_dd == true)){destination = 0xdd;}
                if ((tsleeassi == 7) && (tally_ee == true)){destination = 0xee;}
                receiverMode = 0x03;
                receiverState = 0x01;
                receiverColor = 0x01;
                sendMessage();
                mode = "acknowledge";
                mode_s = "ack";
                lastAckTime = millis();
                lastSwitchTime = millis();
                clearValues();
            }

            //tally zz off
            if ((((tslbbassi == 8) && (tally_bb == true)) || ((tslccassi == 8) && (tally_cc == true)) || ((tslddassi == 8) && (tally_dd == true)) || ((tsleeassi == 8) && (tally_ee == true))) && (sendSequence == true) && (blockTsl == false) && (udp_seq[0] == 230) && (udp_seq[1] == 48) && (udp_seq[2] == 77) && (udp_seq[3] == 50) && (millis() - lastSwitchTime > 25)) {
                Serial.println("=zz off=");
                if ((tslbbassi == 8) && (tally_bb == true)){destination = 0xbb;}
                if ((tslccassi == 8) && (tally_cc == true)){destination = 0xcc;}
                if ((tslddassi == 8) && (tally_dd == true)){destination = 0xdd;}
                if ((tsleeassi == 8) && (tally_ee == true)){destination = 0xee;}
                receiverMode = 0x03;
                receiverState = 0x00;
                receiverColor = 0x00;
                sendMessage();
                mode = "acknowledge";
                mode_s = "ack";
                lastAckTime = millis();
                lastSwitchTime = millis();
                clearValues();
            }

            //tally zz green
            if ((((tslbbassi == 8) && (tally_bb == true)) || ((tslccassi == 8) && (tally_cc == true)) || ((tslddassi == 8) && (tally_dd == true)) || ((tsleeassi == 8) && (tally_ee == true))) && (sendSequence == true) && (blockTsl == false) && (udp_seq[0] == 166) && (udp_seq[1] == 49) && (udp_seq[2] == 77) && (udp_seq[3] == 50) && (millis() - lastSwitchTime > 25)) {
                Serial.println("=zz green=");
                if ((tslbbassi == 8) && (tally_bb == true)){destination = 0xbb;}
                if ((tslccassi == 8) && (tally_cc == true)){destination = 0xcc;}
                if ((tslddassi == 8) && (tally_dd == true)){destination = 0xdd;}
                if ((tsleeassi == 8) && (tally_ee == true)){destination = 0xee;}
                receiverMode = 0x03;
                receiverState = 0x01;
                receiverColor = 0x02;
                sendMessage();
                mode = "acknowledge";
                mode_s = "ack";
                lastAckTime = millis();
                lastSwitchTime = millis();
                clearValues();
            }

            //tally zz red
            if ((((tslbbassi == 8) && (tally_bb == true)) || ((tslccassi == 8) && (tally_cc == true)) || ((tslddassi == 8) && (tally_dd == true)) || ((tsleeassi == 8) && (tally_ee == true))) && (sendSequence == true) && (blockTsl == false) && (udp_seq[0] == 165) && (udp_seq[1] == 50) && (udp_seq[2] == 77) && (udp_seq[3] == 50) && (millis() - lastSwitchTime > 25)) {
                Serial.println("=zz red=");
                if ((tslbbassi == 8) && (tally_bb == true)){destination = 0xbb;}
                if ((tslccassi == 8) && (tally_cc == true)){destination = 0xcc;}
                if ((tslddassi == 8) && (tally_dd == true)){destination = 0xdd;}
                if ((tsleeassi == 8) && (tally_ee == true)){destination = 0xee;}
                receiverMode = 0x03;
                receiverState = 0x01;
                receiverColor = 0x01;
                sendMessage();
                mode = "acknowledge";
                mode_s = "ack";
                lastAckTime = millis();
                lastSwitchTime = millis();
                clearValues();
            }

            //tally zz red wip
            if ((((tslbbassi == 8) && (tally_bb == true)) || ((tslccassi == 8) && (tally_cc == true)) || ((tslddassi == 8) && (tally_dd == true)) || ((tsleeassi == 8) && (tally_ee == true))) && (sendSequence == true) && (blockTsl == false) && (udp_seq[0] == 166) && (udp_seq[1] == 51) && (udp_seq[2] == 77) && (udp_seq[3] == 50) && (millis() - lastSwitchTime > 25)) {
                Serial.println("=zz red wip=");
                if ((tslbbassi == 8) && (tally_bb == true)){destination = 0xbb;}
                if ((tslccassi == 8) && (tally_cc == true)){destination = 0xcc;}
                if ((tslddassi == 8) && (tally_dd == true)){destination = 0xdd;}
                if ((tsleeassi == 8) && (tally_ee == true)){destination = 0xee;}
                receiverMode = 0x03;
                receiverState = 0x01;
                receiverColor = 0x01;
                sendMessage();
                mode = "acknowledge";
                mode_s = "ack";
                lastAckTime = millis();
                lastSwitchTime = millis();
                clearValues();
            }

        }

        lastTslReadTime = millis();
    }




    // Acknowledge Mode
    while (mode == "acknowledge") {
        onReceive(LoRa.parsePacket(), &rx_adr, &tx_adr, &rssi, &bL, &receiverMode, &receiverState, &receiverColor);    // Parse Packets and Read it    
    
        // Back to Request Mode
        if ((receiverMode = 0x04) && ((tx_adr == "bb") || (tx_adr == "cc") || (tx_adr == "dd") ||  (tx_adr == "ee"))) {
            mode = "request";
            mode_s = "req";
            clearValues();

            if (useTSL == true) {
                tslAckReceived = true;
                sendSequence = false;
            }

            break;
        }

    // Toggel and Resend Message, if ACK not arrived after 2 secounds
    if ((millis() - lastAckTime > 2000) && (counterSend < counterSendMax) && (useTSL == false)) {
        clearValues();
        mode = "request";
        mode_s = "req";
        counterSend++;
        gpioC1 = !gpioC1;
        gpioC2 = !gpioC2;
        gpioC3 = !gpioC3;
        gpioC4 = !gpioC4;
        break;
    }

    // Aborting the routine after 3 failed trys
    if ((counterSend == counterSendMax) && (useTSL == false)) {
        clearValues();
        mode = "request";
        mode_s = "req";
        counterSend = 0;
        break;
    }

    // Toggel and Resend Message, if ACK not arrived after 2 secounds
    if ((millis() - lastAckTime > 2000) && (counterSendTsl < counterSendMax) && (useTSL == true)) {
        clearValues();
        mode = "request";
        mode_s = "req";
        counterSendTsl++;

        tslAckReceived = false;
        sendSequence = true;

        break;
    }

    // Aborting the routine after 3 failed trys
    if ((counterSendTsl == counterSendMax) && (useTSL == true)) {
        clearValues();
        mode = "request";
        mode_s = "req";
        counterSendTsl = 0;
        tslAckReceived = true;
        sendSequence = false;

        break;
    }

    }

    // Control Mode BB after discover and 3 - 3.5 minutes or if BB offline, control after 9 minutes
    if (((millis() - lastDiscoverTimebb > 180000) && ((tally_bb == true) || (tally_bb_init == true))) || ((millis() - lastDiscoverTimebb > 540000) && ((tally_bb == false) || (tally_bb_init == false)))) {
        destination = 0xbb;
        receiverMode = 0x05;
        receiverState = 0x00;
        receiverColor = 0x00;
        sendMessage();
        lastDiscoverTimebb = millis();
        lastControlTime = millis();
        mode = "control";
        mode_s = "con";
        clearValues();

        while (mode == "control") {
            onReceive(LoRa.parsePacket(), &rx_adr, &tx_adr, &rssi, &bL, &receiverMode, &receiverState, &receiverColor);    // Parse Packets and Read it

            if ((receiverMode = 0x05) && (tx_adr == "bb")) {
                if (tally_bb_init == false || tally_bb == false) {
                counterTallys++;
                }
                tally_bb = true;
                tally_bb_init = true;
                tx_adr_bb = tx_adr;
                rssi_bb = rssi;

                sprintf(buf_rssi, "%s", rssi);          //Register value string rssi convert into buffer char rssi
                buf_rssi_bb_int = atoi(buf_rssi);              //Convert char rssi in int rssi

                bL_bb = bL;
                missed_bb = 0;
                mode = "request";
                mode_s = "req";
                clearValues();
                break;
            }

            if (millis() - lastControlTime > 3000) {
                missed_bb++;
            
                if (missed_bb >= 2 && tally_bb == true) {
                    tally_bb = false;
                    counterTallys--;
                }
                mode = "request"; 
                mode_s = "req";
                clearValues();
                break;
            }
        }
    }
  
    // Control Mode CC after discover and 3 - 3.5 minutes or if BB offline, control after 9.5 minutes
    if (((millis() - lastDiscoverTimecc > 190000) && ((tally_cc == true) || (tally_cc_init == true))) || ((millis() - lastDiscoverTimecc > 570000) && ((tally_cc == false) || (tally_cc_init == false)))) {
        destination = 0xcc;
        receiverMode = 0x05;
        receiverState = 0x00;
        receiverColor = 0x00;
        sendMessage();
        lastDiscoverTimecc = millis();
        lastControlTime = millis();
        mode = "control";
        mode_s = "con";
        clearValues();

        while (mode == "control") {
            onReceive(LoRa.parsePacket(), &rx_adr, &tx_adr, &rssi, &bL, &receiverMode, &receiverState, &receiverColor);    // Parse Packets and Read it
      
            if ((receiverMode = 0x05) && (tx_adr == "cc")) {
                if (tally_cc_init == false || tally_cc == false) {
                counterTallys++;
                }
                tally_cc = true;
                tally_cc_init = true;
                tx_adr_cc = tx_adr;
                rssi_cc = rssi;
                bL_cc = bL;
                missed_cc = 0;
                mode = "request";
                mode_s = "req";
                clearValues();
                break;
            }

            if (millis() - lastControlTime > 3000) {
                missed_cc++;
                
                if (missed_cc >= 2 && tally_cc == true) {
                tally_cc = false;
                counterTallys--;
                }
                mode = "request"; 
                mode_s = "req";
                clearValues();
                break;
            }
        }
    }

    // Control Mode DD after discover and 3 - 3.5 minutes or if BB offline, control after 10 minutes
    if (((millis() - lastDiscoverTimedd > 200000) && ((tally_dd == true) || (tally_dd_init == true))) || ((millis() - lastDiscoverTimedd > 600000) && ((tally_dd == false) || (tally_dd_init == false)))) {
        destination = 0xdd;
        receiverMode = 0x05;
        receiverState = 0x00;
        receiverColor = 0x00;
        sendMessage();
        lastDiscoverTimedd = millis();
        lastControlTime = millis();
        mode = "control";
        mode_s = "con";
        clearValues();

        while (mode == "control") {
            onReceive(LoRa.parsePacket(), &rx_adr, &tx_adr, &rssi, &bL, &receiverMode, &receiverState, &receiverColor);    // Parse Packets and Read it
            
            if ((receiverMode = 0x05) && (tx_adr == "dd")) {
                if (tally_dd_init == false || tally_dd == false) {
                counterTallys++;
                }
                tally_dd = true;
                tally_dd_init = true;
                tx_adr_dd = tx_adr;
                rssi_dd = rssi;
                bL_dd = bL;
                missed_dd = 0;
                mode = "request";
                mode_s = "req";
                clearValues();
                break;
            }

            if (millis() - lastControlTime > 3000) {
                missed_dd++;
                
                if (missed_dd >= 2 && tally_dd == true) {
                tally_dd = false;
                counterTallys--;
                }
                mode = "request"; 
                mode_s = "req"; 
                clearValues();
                break;
            }
        }
    }

    // Control Mode EE after discover and 3 - 3.5 minutes or if BB offline, control after 10.5 minutes
    if (((millis() - lastDiscoverTimeee > 210000) && ((tally_ee == true) || (tally_ee_init == true))) || ((millis() - lastDiscoverTimeee > 630000) && ((tally_ee == false) || (tally_ee_init == false)))) {
        destination = 0xee;
        receiverMode = 0x05;
        receiverState = 0x00;
        receiverColor = 0x00;
        sendMessage();
        lastDiscoverTimeee = millis();
        lastControlTime = millis();
        mode = "control";
        mode_s = "con";
        clearValues();

        while (mode == "control") {
            onReceive(LoRa.parsePacket(), &rx_adr, &tx_adr, &rssi, &bL, &receiverMode, &receiverState, &receiverColor);    // Parse Packets and Read it
            
            if ((receiverMode = 0x05) && (tx_adr == "ee")) {
                if (tally_ee_init == false || tally_ee == false) {
                counterTallys++;
                }
                tally_ee = true;
                tally_ee_init = true;
                tx_adr_ee = tx_adr;
                rssi_ee = rssi;
                bL_ee = bL;
                missed_ee = 0;
                mode = "request";
                mode_s = "req";
                clearValues();
                break;
            }

            if (millis() - lastControlTime > 3000) {
                missed_ee++;
                
                if (missed_ee >= 2 && tally_ee == true) {
                tally_ee = false;
                counterTallys--;
                }
                mode = "request"; 
                mode_s = "req";
                clearValues();
                break;
            }
        }
    }

    // Function Print Display if Eth State is changed
    if (ethConnected == true && ethState == false) {
        closeSPI_LORA();
        startSPI_DISPLAY();
        clearValues();
        printDisplay();
        closeSPI_DISPLAY();
        startSPI_LORA();
        ethState = !ethState;
    }

    if (ethConnected == false && ethState == true) {
        closeSPI_LORA();
        startSPI_DISPLAY();
        clearValues();
        printDisplay();
        closeSPI_DISPLAY();
        startSPI_LORA();
        ethState = !ethState;
    }
    // Function for Logout after 10 minutes
    if ((millis() - lastAuthentication > 600000)) {
        authenticated = false;
    }

}

//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////