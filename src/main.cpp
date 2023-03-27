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

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif

#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

String mode = "discover";
String mode_s = "dis";
String name_html = "REDTALLY";              // Device Name
String name = "REDTALLY Transmitter";       // Device Name
String version = "T0.02";                   // Frimeware Version
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
char buf_input_ip[32];
char buf_input_gw[32];
char buf_input_sn[32];
char buf_input_dns1[32];
char buf_input_dns2[32];
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

char buf_html_ip[32];
char buf_html_gw[32];
char buf_html_snm[32];
char buf_html_dns1[32];
char buf_html_dns2[32];

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
unsigned long lastAnalogReadTime = 0;
unsigned long lastDisplayPrint = 0;
unsigned long lastTslReadTime = 0;
unsigned long lastAuthentication = 0;

int defaultBrightnessDisplay = 255;   // value from 1 to 255
int counterSend = 0;
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

bool gpioC1 = HIGH;
bool gpioC2 = HIGH;
bool gpioC3 = HIGH;
bool gpioC4 = HIGH;
bool tally_bb = LOW; 
bool tally_cc = LOW; 
bool tally_dd = LOW; 
bool tally_ee = LOW; 
bool tally_bb_init = LOW; 
bool tally_cc_init = LOW; 
bool tally_dd_init = LOW; 
bool tally_ee_init = LOW;
bool initBattery = HIGH;
bool batteryAttention = LOW;
bool batteryAttentionState = LOW;
bool ethConnected = false;
bool useSTATIC = false;
bool useWLAN = false;
bool useTSL = false;
bool bool_res = false;
bool bool_esm = false;
bool bool_tsl = false;
bool ethState = false;
bool loraInit = false;
bool sdInit = false;
bool oledInit = false;
bool authenticated = false;
bool error = false;

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
        if(tally_bb == HIGH) {
            html_state_bb = "ONLINE";
            }
            else{
            html_state_bb = "OFFLINE";
            }
            return html_state_bb;
    }

    if(state == "STATE_CC"){
        if(tally_cc == HIGH) {
            html_state_cc = "ONLINE";
            }
            else{
            html_state_cc = "OFFLINE";
            }
            return html_state_cc;
    }

    if(state == "STATE_DD"){
        if(tally_dd == HIGH) {
            html_state_dd = "ONLINE";
            }
            else{
            html_state_dd = "OFFLINE";
            }
            return html_state_dd;
    }

    if(state == "STATE_EE"){
        if(tally_ee == HIGH) {
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
    digitalWrite(SD_CS, LOW);

}

void closeSPI_SD() {

    digitalWrite(SD_CS, HIGH);
    SPI.end();
    //SPI.endTransaction();

}

//////////////////////////////////////////////////////////////////////

void startSPI_LORA() {

    SPI.begin(LORA_SCLK, LORA_MISO, LORA_MOSI);   
    //SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));      
    digitalWrite(LORA_CS, LOW);

    LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ);
    LoRa.setTxPower(loraTxPower);
    LoRa.setSpreadingFactor(loraSpreadingFactor);    
    LoRa.setSignalBandwidth(loraSignalBandwidth);
    LoRa.setCodingRate4(loraCodingRate);
    LoRa.setPreambleLength(loraPreambleLength);
    LoRa.begin(loraFrequenz);

}

void closeSPI_LORA() {

    digitalWrite(LORA_CS, HIGH);
    SPI.end();
    //SPI.endTransaction();

}

//////////////////////////////////////////////////////////////////////

void startSPI_DISPLAY() {

    SPI.begin(DISPLAY_SCLK, DISPLAY_MISO, DISPLAY_MOSI); 
    //SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));    
    digitalWrite(DISPLAY_CS, LOW);

    u8g2.begin();
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.setContrast(defaultBrightnessDisplay);                  
    u8g2.setFlipMode(0);

}

void closeSPI_DISPLAY() {

    digitalWrite(DISPLAY_CS, HIGH);
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

    /*
    Serial.print("DST: "); Serial.print(destination);
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
    */
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
        sprintf(buf_ip, "%s", MyIpAddress);

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
        u8g2.drawStr(25,35,buf_ip);
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

    pinMode(gpioP1, INPUT_PULLDOWN);
    pinMode(gpioP2, INPUT_PULLDOWN);
    pinMode(gpioP3, INPUT_PULLDOWN);
    pinMode(gpioP4, INPUT_PULLDOWN);

    pinMode(SD_CS, OUTPUT);                                     //CS SD
    pinMode(LORA_CS, OUTPUT);                                   //CS LORA
    pinMode(DISPLAY_CS, OUTPUT);                                //CS Display

    digitalWrite(SD_CS, HIGH);
    digitalWrite(LORA_CS, HIGH);
    digitalWrite(DISPLAY_CS, HIGH);

    //pinMode(SD_MISO, INPUT_PULLUP);

//////////////////////////////////////////////////////////////////////

    //1=POWERON_RESET   3=SW_RESET  4=OWDT_RESET    5=DEEPSLEEP_RESET   6=SDIO_RESET    7=TG0WDT_SYS_RESET  8=TG1WDT_SYS_RESET  9=RTCWDT_SYS_RESET  
    //10=INTRUSION_RESET  11=TGWDT_CPU_RESET  12=SW_CPU_RESET 13=RTCWDT_CPU_RESET   14=EXT_CPU_RESET    15=RTCWDT_BROWN_OUT_RESET   16=RTCWDT_RTC_RESET     default=NO_MEAN

    if (rtc_get_reset_reason(0) == 1 || rtc_get_reset_reason(0) == 14 || rtc_get_reset_reason(1) == 1 || rtc_get_reset_reason(1) == 14) {
            eeprom.begin("network", false);
            eeprom.clear();             //Clear the eeprom when the reset button is pushed
            eeprom.end();
            eeprom.begin("configuration", false); 
            eeprom.clear();
            eeprom.end();
    }

//////////////////////////////////////////////////////////////////////

    eeprom.begin("network", false);                //false mean use read/write mode
    useSTATIC = eeprom.getBool("dhcp", false);     //false mean default value if nothing returned
    ssid = eeprom.getString("ssid", ssid);
    wifipassword = eeprom.getString("wifipassword", wifipassword);
    useIPOctet1 = eeprom.getInt("ipOctet1", 192);
    useIPOctet2 = eeprom.getInt("ipOctet2", 168);
    useIPOctet3 = eeprom.getInt("ipOctet3", 1);
    useIPOctet4 = eeprom.getInt("ipOctet4", 100);
    useGWOctet1 = eeprom.getInt("gwOctet1", 192);
    useGWOctet2 = eeprom.getInt("gwOctet2", 168);
    useGWOctet3 = eeprom.getInt("gwOctet3", 1);
    useGWOctet4 = eeprom.getInt("gwOctet4", 1);  
    useSNOctet1 = eeprom.getInt("snOctet1", 255);
    useSNOctet2 = eeprom.getInt("snOctet2", 255);
    useSNOctet3 = eeprom.getInt("snOctet3", 0);
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
    bool_tsl = eeprom.getBool("tsl", false);        
    bool_esm = eeprom.getBool("esm", false);
    loraTxPower = eeprom.getInt("txpower", loraTxPower);            //if the eeprom is never written, then give the default value back
    
    www_username = eeprom.getString("user", www_username);
    www_password = eeprom.getString("password", www_password);

    transmissionPower = loraTxPower;  

    eeprom.end();

//////////////////////////////////////////////////////////////////////

    SPI.begin(SD_SCLK, SD_MISO, SD_MOSI);     
    //SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));    
    digitalWrite(SD_CS, LOW);

    if (!SD.begin(SD_CS)) {                                                 // initialize sd card
        sdInit = false;   
    }else {     
        sdInit = true; 
    }

    digitalWrite(SD_CS, HIGH);
    SPI.end();
    //SPI.endTransaction();

//////////////////////////////////////////////////////////////////////

    SPI.begin(LORA_SCLK, LORA_MISO, LORA_MOSI);     
    //SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));    
    digitalWrite(LORA_CS, LOW);

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

    digitalWrite(LORA_CS, HIGH);
    SPI.end();  
    //SPI.endTransaction();

//////////////////////////////////////////////////////////////////////

    SPI.begin(DISPLAY_SCLK, DISPLAY_MISO, DISPLAY_MOSI);     
    //SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));    
    digitalWrite(DISPLAY_CS, LOW); 

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
            //Serial.print("useSTATIC: "); Serial.print(useSTATIC);
            resFlag = 0x01;

            if (tally_bb == HIGH){
                destination = 0xbb;                                                                     //if tx power changed via webterminal, then send message to receivers and change the txpower with restart
                receiverMode = 0x05;            // Send a message
                sendMessage();         
                Serial.println("LORA TxD: 0x05 CONTROL");
                delay(500);
            }
            if (tally_cc == HIGH){
                destination = 0xcc;
                receiverMode = 0x05;
                sendMessage();
                Serial.println("LORA TxD: 0x05 CONTROL");
                delay(500);
            }
            if (tally_dd == HIGH){
                destination = 0xdd;
                receiverMode = 0x05;
                sendMessage();
                Serial.println("LORA TxD: 0x05 CONTROL");
                delay(500);
            }
            if (tally_ee == HIGH){
                destination = 0xee;
                receiverMode = 0x05;
                sendMessage();
                Serial.println("LORA TxD: 0x05 CONTROL");
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
            //Serial.print("useSTATIC: "); Serial.print(useSTATIC);
            resFlag = 0x01;

            if (tally_bb == HIGH){
                destination = 0xbb;                                                                     //if tx power changed via webterminal, then send message to receivers and change the txpower with restart
                receiverMode = 0x05;
                sendMessage();
                Serial.println("LORA TxD: 0x05 CONTROL");
                delay(500);
            }
            if (tally_cc == HIGH){
                destination = 0xcc;
                receiverMode = 0x05;
                sendMessage();
                Serial.println("LORA TxD: 0x05 CONTROL");
                delay(500);
            }
            if (tally_dd == HIGH){
                destination = 0xdd;
                receiverMode = 0x05;
                sendMessage();
                Serial.println("LORA TxD: 0x05 CONTROL");
                delay(500);
            }
            if (tally_ee == HIGH){
                destination = 0xee;
                receiverMode = 0x05;
                sendMessage();
                Serial.println("LORA TxD: 0x05 CONTROL");
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
            eeprom.putString("wifipassword", input_wlanpassword);
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
            eeprom.putBool("tsl", bool_tsl);     
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
            eeprom.putBool("tsl", bool_tsl);     
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
            eeprom.end();
            request->send(SPIFFS, "/configuration.html", String(), false, proc_state);
        } else {
            request->send(SPIFFS, "/login.html", String(), false, proc_state);
        }
    });

    server.on("/restart", HTTP_GET, [](AsyncWebServerRequest *request){
        if (authenticated == true) {
            resFlag = 0x01;

            if (tally_bb == HIGH){
                destination = 0xbb;                                                                     //if tx power changed via webterminal, then send message to receivers and change the txpower with restart
                receiverMode = 0x05;
                sendMessage();
                Serial.println("LORA TxD: 0x05 CONTROL");
                delay(500);
            }
            if (tally_cc == HIGH){
                destination = 0xcc;
                receiverMode = 0x05;      
                sendMessage();
                Serial.println("LORA TxD: 0x05 CONTROL");
                delay(500);
            }
            if (tally_dd == HIGH){
                destination = 0xdd;
                receiverMode = 0x05;
                sendMessage();
                Serial.println("LORA TxD: 0x05 CONTROL");
                delay(500);
            }
            if (tally_ee == HIGH){
                destination = 0xee;
                receiverMode = 0x05;
                sendMessage();
                Serial.println("LORA TxD: 0x05 CONTROL");
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

            if (tally_bb == HIGH){
                destination = 0xbb;                                                                     //if tx power changed via webterminal, then send message to receivers and change the txpower with restart
                receiverMode = 0x05;
                sendMessage();
                Serial.println("LORA TxD: 0x05 CONTROL");
                delay(500);
            }
            if (tally_cc == HIGH){
                destination = 0xcc;
                receiverMode = 0x05;      
                sendMessage();
                Serial.println("LORA TxD: 0x05 CONTROL");
                delay(500);
            }
            if (tally_dd == HIGH){
                destination = 0xdd;
                receiverMode = 0x05;
                sendMessage();
                Serial.println("LORA TxD: 0x05 CONTROL");
                delay(500);
            }
            if (tally_ee == HIGH){
                destination = 0xee;
                receiverMode = 0x05;
                sendMessage();
                Serial.println("LORA TxD: 0x05 CONTROL");
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

            sprintf(buf_txpower, "%s", input_txp);
            loraTxPowerNew = atoi(buf_txpower);

            if (loraTxPowerNew != loraTxPower) {

                loraTxPower = loraTxPowerNew;              //value for eeprom
                transmissionPower = loraTxPowerNew;             //send via lora

                eeprom.begin("configuration", false);                //false mean use read/write mode
                eeprom.putInt("txpower", loraTxPower);     
                eeprom.end(); 

                if (tally_bb == HIGH){
                    destination = 0xbb;                                                                     //if tx power changed via webterminal, then send message to receivers and change the txpower with restart
                    receiverMode = 0x05;
                    sendMessage();
                Serial.println("LORA TxD: 0x05 CONTROL");
                    delay(500);
                }
                if (tally_cc == HIGH){
                    destination = 0xcc;
                    receiverMode = 0x05;
                    sendMessage();
                Serial.println("LORA TxD: 0x05 CONTROL");
                    delay(500);
                }
                if (tally_dd == HIGH){
                    destination = 0xdd;
                    receiverMode = 0x05;
                    sendMessage();
                Serial.println("LORA TxD: 0x05 CONTROL");
                    delay(500);
                }
                if (tally_ee == HIGH){
                    destination = 0xee;
                    receiverMode = 0x05;
                    sendMessage();
                Serial.println("LORA TxD: 0x05 CONTROL");
                }

                request->send(SPIFFS, "/configuration.html", String(), false, proc_state);
                delay(2000);
                ESP.restart();
            } else{
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
            eeprom.putString("password", input_new_password);    
            eeprom.end(); 

            request->send(SPIFFS, "/configuration.html", String(), false, proc_state);
            delay(2000);
            ESP.restart();

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
            Serial.println(input_ip);
            Serial.println(input_gw);
            Serial.println(input_sn);
            Serial.println(input_dns1);
            Serial.println(input_dns2);

            sprintf(buf_input_ip, "%s", input_ip);
            sprintf(buf_input_gw, "%s", input_gw);
            sprintf(buf_input_sn, "%s", input_sn);
            sprintf(buf_input_dns1, "%s", input_dns1);
            sprintf(buf_input_dns2, "%s", input_dns2);

            error = false;

            char* ptr_ip = strtok(buf_input_ip, ".");
            int_ipOctet1 = atoi(ptr_ip);
            ptr_ip = strtok(NULL, ".");
            int_ipOctet2 = atoi(ptr_ip);
            ptr_ip = strtok(NULL, ".");
            int_ipOctet3 = atoi(ptr_ip);
            ptr_ip = strtok(NULL, ".");
            int_ipOctet4 = atoi(ptr_ip);

            if ((scanf("%i", &int_ipOctet1) != 3) || (scanf("%i", &int_ipOctet2) != 3) || (scanf("%i", &int_ipOctet3) != 3) || (scanf("%i", &int_ipOctet4) != 3)) {
                Serial.println("ERROR!");
                error = true;
            }

            char* ptr_gw = strtok(buf_input_gw, ".");
            int_gwOctet1 = atoi(ptr_gw);
            ptr_gw = strtok(NULL, ".");
            int_gwOctet2 = atoi(ptr_gw);
            ptr_gw = strtok(NULL, ".");
            int_gwOctet3 = atoi(ptr_gw);
            ptr_gw = strtok(NULL, ".");
            int_gwOctet4 = atoi(ptr_gw);

            if ((scanf("%i", &int_gwOctet1) != 3) || (scanf("%i", &int_gwOctet2) != 3) || (scanf("%i", &int_gwOctet3) != 3) || (scanf("%i", &int_gwOctet4) != 3)) {
                Serial.println("ERROR!");
                error = true;
            }

            char* ptr_sn = strtok(buf_input_sn, ".");
            int_snOctet1 = atoi(ptr_sn);
            ptr_sn = strtok(NULL, ".");
            int_snOctet2 = atoi(ptr_sn);
            ptr_sn = strtok(NULL, ".");
            int_snOctet3 = atoi(ptr_sn);
            ptr_sn = strtok(NULL, ".");
            int_snOctet4 = atoi(ptr_sn);

            if ((scanf("%i", &int_snOctet1) != 3) || (scanf("%i", &int_snOctet2) != 3) || (scanf("%i", &int_snOctet3) != 3) || (scanf("%i", &int_snOctet4) != 3)) {
                Serial.println("ERROR!");
                error = true;
            }

            char* ptr_dns1 = strtok(buf_input_dns1, ".");
            int_dns1Octet1 = atoi(ptr_dns1);
            ptr_dns1 = strtok(NULL, ".");
            int_dns1Octet2 = atoi(ptr_dns1);
            ptr_dns1 = strtok(NULL, ".");
            int_dns1Octet3 = atoi(ptr_dns1);
            ptr_dns1 = strtok(NULL, ".");
            int_dns1Octet4 = atoi(ptr_dns1);

            if ((scanf("%i", &int_dns1Octet1) != 3) || (scanf("%i", &int_dns1Octet2) != 3) || (scanf("%i", &int_dns1Octet3) != 3) || (scanf("%i", &int_dns1Octet4) != 3)) {
                Serial.println("ERROR!");
                error = true;
            }

            char* ptr_dns2 = strtok(buf_input_dns2, ".");
            int_dns2Octet1 = atoi(ptr_dns2);
            ptr_dns2 = strtok(NULL, ".");
            int_dns2Octet2 = atoi(ptr_dns2);
            ptr_dns2 = strtok(NULL, ".");
            int_dns2Octet3 = atoi(ptr_dns2);
            ptr_dns2 = strtok(NULL, ".");
            int_dns2Octet4 = atoi(ptr_dns2);

            if ((scanf("%i", &int_dns2Octet1) != 3) || (scanf("%i", &int_dns2Octet2) != 3) || (scanf("%i", &int_dns2Octet3) != 3) || (scanf("%i", &int_dns2Octet4) != 3)) {
                Serial.println("ERROR!");
                error = true;
            }

            //if (error == false) {

                eeprom.begin("network", false);                //false mean use read/write mode
                eeprom.putInt("ipOctet1", int_ipOctet1);  
                eeprom.putInt("ipOctet2", int_ipOctet2);    
                eeprom.putInt("ipOctet3", int_ipOctet3);
                eeprom.putInt("ipOctet4", int_ipOctet4);      
                eeprom.putInt("gwOctet1", int_gwOctet1);  
                eeprom.putInt("gwOctet2", int_gwOctet2);    
                eeprom.putInt("gwOctet3", int_gwOctet3);
                eeprom.putInt("gwOctet4", int_gwOctet4);      
                eeprom.putInt("snOctet1", int_snOctet1);  
                eeprom.putInt("snOctet2", int_snOctet2);    
                eeprom.putInt("snOctet3", int_snOctet3);
                eeprom.putInt("snOctet4", int_snOctet4);      
                eeprom.putInt("dns1Octet1", int_dns1Octet1);  
                eeprom.putInt("dns1Octet2", int_dns1Octet2);    
                eeprom.putInt("dns1Octet3", int_dns1Octet3);
                eeprom.putInt("dns1Octet4", int_dns1Octet4);      
                eeprom.putInt("dns2Octet1", int_dns2Octet1);  
                eeprom.putInt("dns2Octet2", int_dns2Octet2);    
                eeprom.putInt("dns2Octet3", int_dns2Octet3);
                eeprom.putInt("dns2Octet4", int_dns2Octet4);      
                eeprom.end();
            //}

            request->send(SPIFFS, "/network.html", String(), false, proc_state);
            delay(2000);
            ESP.restart();
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

    digitalWrite(DISPLAY_CS, HIGH);
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
        //Serial.println("LORA TxD: 0x01 DISCOVER");
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

        if ((receiverMode = 0x02) && (tx_adr == "bb") && (tally_bb == LOW)) {
            Serial.println("LORA RxD: 0x02 OFFER");
            tally_bb = HIGH;
            tally_bb_init = HIGH;
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
        if ((receiverMode = 0x02) && (tx_adr == "cc") && (tally_cc == LOW)) {
            Serial.println("LORA RxD: 0x02 OFFER");
            tally_cc = HIGH;
            tally_cc_init = HIGH;
            tx_adr_cc = tx_adr;
            rssi_cc = rssi;
            bL_cc = bL;
            counterTallys++;
            lastOfferTime = millis();
            lastOfferTimeEnd = millis();
            clearValues();
        }
        if ((receiverMode = 0x02) && (tx_adr == "dd") && (tally_dd == LOW)) {
            Serial.println("LORA RxD: 0x02 OFFER");
            tally_dd = HIGH;
            tally_dd_init = HIGH;
            tx_adr_dd = tx_adr;
            rssi_dd = rssi;
            bL_dd = bL;
            counterTallys++;
            lastOfferTime = millis();
            lastOfferTimeEnd = millis();
            clearValues();
        }
        if ((receiverMode = 0x02) && (tx_adr == "ee") && (tally_ee == LOW)) {
            Serial.println("LORA RxD: 0x02 OFFER");
            tally_ee = HIGH;
            tally_ee_init = HIGH;
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

        if ((millis() - lastOfferTimeEnd > 120000) && (counterTallys >= 1)) {    // after 120 s of no new receivers, request mode
            mode = "request"; 
            mode_s = "req";
            break;
        }
        }
  
    // Request Mode GPIO
    if ((mode == "request") && (millis() - lastAnalogReadTime > 250) && (useTSL == false)) {

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

        if ((gpioV1Cal > 2.2 && tally_bb == HIGH && gpioC1 == HIGH)) {
            destination = 0xbb;
            receiverMode = 0x03;
            receiverState = 0x01;
            receiverColor = 0x01;
            sendMessage();
            Serial.println("LORA TxD: 0x03 REQUEST");
            gpioC1 = !gpioC1;
            mode = "acknowledge";
            mode_s = "ack";
            lastAckTime = millis();
            clearValues();
        }

        if ((gpioV1Cal < 2.2 && tally_bb == HIGH && gpioC1 == LOW)) {
            destination = 0xbb;
            receiverMode = 0x03;
            receiverState = 0x00;
            receiverColor = 0x00;
            sendMessage();
            Serial.println("LORA TxD: 0x03 REQUEST");
            gpioC1 = !gpioC1;
            mode = "acknowledge";
            mode_s = "ack";
            lastAckTime = millis();
            clearValues();
        }

        if ((gpioV2Cal > 2.2 && tally_cc == HIGH && gpioC2 == HIGH)) {
            destination = 0xcc;
            receiverMode = 0x03;
            receiverState = 0x01;
            receiverColor = 0x01;
            sendMessage();
            Serial.println("LORA TxD: 0x03 REQUEST");
            gpioC2 = !gpioC2;
            mode = "acknowledge";
            mode_s = "ack";
            lastAckTime = millis();
            clearValues();
        }

        if ((gpioV2Cal < 2.2 && tally_cc == HIGH && gpioC2 == LOW)) {
            destination = 0xcc;
            receiverMode = 0x03;
            receiverState = 0x00;
            receiverColor = 0x00;
            sendMessage();
            Serial.println("LORA TxD: 0x03 REQUEST");
            gpioC2 = !gpioC2;
            mode = "acknowledge";
            mode_s = "ack";
            lastAckTime = millis();
            clearValues();
        }

        if ((gpioV3Cal > 2.2 && tally_dd == HIGH && gpioC3 == HIGH)) {
            destination = 0xdd;
            receiverMode = 0x03;
            receiverState = 0x01;
            receiverColor = 0x01;
            sendMessage();
            Serial.println("LORA TxD: 0x03 REQUEST");
            gpioC3 = !gpioC3;
            mode = "acknowledge";
            mode_s = "ack";
            lastAckTime = millis();
            clearValues();
        }

        if ((gpioV3Cal < 2.2 && tally_dd == HIGH && gpioC3 == LOW)) {
            destination = 0xdd;
            receiverMode = 0x03;
            receiverState = 0x00;
            receiverColor = 0x00;
            sendMessage();
            Serial.println("LORA TxD: 0x03 REQUEST");
            gpioC3 = !gpioC3;
            mode = "acknowledge";
            mode_s = "ack";
            lastAckTime = millis();
            clearValues();
        }

        if ((gpioV4Cal > 2.2 && tally_ee == HIGH && gpioC4 == HIGH)) {
            destination = 0xee;
            receiverMode = 0x03;
            receiverState = 0x01;
            receiverColor = 0x01;
            sendMessage();
            Serial.println("LORA TxD: 0x03 REQUEST");
            gpioC4 = !gpioC4;
            mode = "acknowledge";
            mode_s = "ack";
            lastAckTime = millis();
            clearValues();
        }

        if ((gpioV4Cal < 2.2 && tally_ee == HIGH && gpioC4 == LOW)) {
            destination = 0xee;
            receiverMode = 0x03;
            receiverState = 0x00;
            receiverColor = 0x00;
            sendMessage();
            Serial.println("LORA TxD: 0x03 REQUEST");
            gpioC4 = !gpioC4;
            mode = "acknowledge";
            mode_s = "ack";
            lastAckTime = millis();
            clearValues();
        }
        lastAnalogReadTime = millis();
    }

    // Request Mode TSL
    if ((mode == "request") && (millis() - lastTslReadTime > 250) && (useTSL == true)) {
        lastTslReadTime = millis();
    }

    // Acknowledge Mode
    while (mode == "acknowledge") {
        onReceive(LoRa.parsePacket(), &rx_adr, &tx_adr, &rssi, &bL, &receiverMode, &receiverState, &receiverColor);    // Parse Packets and Read it    
    
        // Back to Request Mode
        if ((receiverMode = 0x04) && ((tx_adr == "bb") || (tx_adr == "cc") || (tx_adr == "dd") ||  (tx_adr == "ee"))) {
            Serial.println("LORA RxD: 0x04 ACKNOWLEDGE");
            mode = "request";
            mode_s = "req";
            clearValues();
            break;
        }

    // Toggel and Resend Message, if ACK not arrived after 2 secounds
    if ((millis() - lastAckTime > 2000) && (counterSend < counterSendMax)) {
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
    if ((counterSend == counterSendMax)) {
        clearValues();
        mode = "request";
        mode_s = "req";
        counterSend = 0;
        break;
    }
    }

    // Control Mode BB after discover and 3 - 3.5 minutes or if BB offline, control after 9 minutes
    if (((millis() - lastDiscoverTimebb > 180000) && ((tally_bb == HIGH) || (tally_bb_init == HIGH))) || ((millis() - lastDiscoverTimebb > 540000) && ((tally_bb == LOW) || (tally_bb_init == LOW)))) {
        destination = 0xbb;
        receiverMode = 0x05;
        receiverState = 0x00;
        receiverColor = 0x00;
        sendMessage();
        Serial.println("LORA TxD: 0x05 CONTROL");
        lastDiscoverTimebb = millis();
        lastControlTime = millis();
        mode = "control";
        mode_s = "con";
        clearValues();

        while (mode == "control") {
            onReceive(LoRa.parsePacket(), &rx_adr, &tx_adr, &rssi, &bL, &receiverMode, &receiverState, &receiverColor);    // Parse Packets and Read it

            if ((receiverMode = 0x05) && (tx_adr == "bb")) {
                Serial.println("LORA RxD: 0x05 CONTROL");
                if (tally_bb_init == LOW || tally_bb == LOW) {
                counterTallys++;
                }
                tally_bb = HIGH;
                tally_bb_init = HIGH;
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
            
                if (missed_bb >= 2 && tally_bb == HIGH) {
                    tally_bb = LOW;
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
    if (((millis() - lastDiscoverTimecc > 190000) && ((tally_cc == HIGH) || (tally_cc_init == HIGH))) || ((millis() - lastDiscoverTimecc > 570000) && ((tally_cc == LOW) || (tally_cc_init == LOW)))) {
        destination = 0xcc;
        receiverMode = 0x05;
        receiverState = 0x00;
        receiverColor = 0x00;
        sendMessage();
        Serial.println("LORA TxD: 0x05 CONTROL");
        lastDiscoverTimecc = millis();
        lastControlTime = millis();
        mode = "control";
        mode_s = "con";
        clearValues();

        while (mode == "control") {
            onReceive(LoRa.parsePacket(), &rx_adr, &tx_adr, &rssi, &bL, &receiverMode, &receiverState, &receiverColor);    // Parse Packets and Read it
      
            if ((receiverMode = 0x05) && (tx_adr == "cc")) {
                Serial.println("LORA RxD: 0x05 CONTROL");
                if (tally_cc_init == LOW || tally_cc == LOW) {
                counterTallys++;
                }
                tally_cc = HIGH;
                tally_cc_init = HIGH;
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
                
                if (missed_cc >= 2 && tally_cc == HIGH) {
                tally_cc = LOW;
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
    if (((millis() - lastDiscoverTimedd > 200000) && ((tally_dd == HIGH) || (tally_dd_init == HIGH))) || ((millis() - lastDiscoverTimedd > 600000) && ((tally_dd == LOW) || (tally_dd_init == LOW)))) {
        destination = 0xdd;
        receiverMode = 0x05;
        receiverState = 0x00;
        receiverColor = 0x00;
        sendMessage();
        Serial.println("LORA TxD: 0x05 CONTROL");
        lastDiscoverTimedd = millis();
        lastControlTime = millis();
        mode = "control";
        mode_s = "con";
        clearValues();

        while (mode == "control") {
            onReceive(LoRa.parsePacket(), &rx_adr, &tx_adr, &rssi, &bL, &receiverMode, &receiverState, &receiverColor);    // Parse Packets and Read it
            
            if ((receiverMode = 0x05) && (tx_adr == "dd")) {
                Serial.println("LORA RxD: 0x05 CONTROL");
                if (tally_dd_init == LOW || tally_dd == LOW) {
                counterTallys++;
                }
                tally_dd = HIGH;
                tally_dd_init = HIGH;
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
                
                if (missed_dd >= 2 && tally_dd == HIGH) {
                tally_dd = LOW;
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
    if (((millis() - lastDiscoverTimeee > 210000) && ((tally_ee == HIGH) || (tally_ee_init == HIGH))) || ((millis() - lastDiscoverTimeee > 630000) && ((tally_ee == LOW) || (tally_ee_init == LOW)))) {
        destination = 0xee;
        receiverMode = 0x05;
        receiverState = 0x00;
        receiverColor = 0x00;
        sendMessage();
        Serial.println("LORA TxD: 0x05 CONTROL");
        lastDiscoverTimeee = millis();
        lastControlTime = millis();
        mode = "control";
        mode_s = "con";
        clearValues();

        while (mode == "control") {
            onReceive(LoRa.parsePacket(), &rx_adr, &tx_adr, &rssi, &bL, &receiverMode, &receiverState, &receiverColor);    // Parse Packets and Read it
            
            if ((receiverMode = 0x05) && (tx_adr == "ee")) {
                Serial.println("LORA RxD: 0x05 CONTROL");
                if (tally_ee_init == LOW || tally_ee == LOW) {
                counterTallys++;
                }
                tally_ee = HIGH;
                tally_ee_init = HIGH;
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
                
                if (missed_ee >= 2 && tally_ee == HIGH) {
                tally_ee = LOW;
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