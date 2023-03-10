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
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif

#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

String mode = "discover";
String mode_s = "dis";
String name = "REDTALLY Transmitter";      // Device Name
String version = "T0.01";                   // Frimeware Version
String bb = "bb";
String cc = "cc";
String dd = "dd";
String ee = "ee";
String lost = "x";
String rx_adr, tx_adr, incoming, outgoing, rssi, bL, snr;
String tx_adr_bb, tx_adr_cc, tx_adr_dd, tx_adr_ee;
String incoming_bb, incoming_cc, incoming_dd, incoming_ee;
String rssi_bb, rssi_cc, rssi_dd, rssi_ee;
String bL_bb, bL_cc, bL_dd, bL_ee;
String html_state_bb, html_state_cc, html_state_dd, html_state_ee;
String html_state_dhcp, html_state_dns, html_state_esm, html_state_tsl;
String ipOctet1, ipOctet2, ipOctet3, ipOctet4;
String gwOctet1, gwOctet2, gwOctet3, gwOctet4;
String snOctet1, snOctet2, snOctet3, snOctet4;
String dns1Octet1, dns1Octet2, dns1Octet3, dns1Octet4;
String dns2Octet1, dns2Octet2, dns2Octet3, dns2Octet4;

char buf_tx[12];
char buf_rx[12];
char buf_bb[3];
char buf_cc[3];
char buf_dd[3];
char buf_ee[3];
char buf_lost[2];
char buf_version[5];
char buf_localAddress[5];
char buf_mode[4];
char buf_rxAdr[5];
char buf_txAdr[5];
char buf_rssi_bb[4];
char buf_rssi_cc[4];
char buf_rssi_dd[4];
char buf_rssi_ee[4];
char buf_bL_bb[4];
char buf_bL_cc[4];
char buf_bL_dd[4];
char buf_bL_ee[4];
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

const char* username = "admin";
const char* password = "admin";

const char* param_ip = "input1";
const char* param_gw = "input2";
const char* param_sn = "input3";
const char* param_dns1 = "input4";
const char* param_dns2 = "input5";
const char* param_txp = "input6";

///////////////////////////////////////////////
///////// Setup Transmitter Values ////////////
///////////////////////////////////////////////

byte localAddress = 0xaa;                 // Address of this device  
String string_localAddress = "aa";                                    
byte destination = 0xff;                  // Destination to send to              
String string_destinationAddress = "ff";            

///////////////////////////////////////////////
///////////////////////////////////////////////
///////////////////////////////////////////////

byte msgKey1 = 0x2a;                      // Key of outgoing messages
byte msgKey2 = 0x56;
byte msgCount = 0;                        // Count of outgoing messages
byte esm = 0x00;                          // 0x00 -> OFF  0x01 -> ON
byte byte_txpower = 0x17;
      
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
unsigned long lastTestTime = 0;
unsigned long lastDisplayPrint = 0;
unsigned long lastTslReadTime = 0;

int defaultBrightnessDisplay = 255;   // value from 1 to 255
int counterSend = 0;
int counterSendMax = 2;
int counterTallys = 0;
int gpioP1 = 36, gpioP2 = 39, gpioP3 = 34, gpioP4 = 35;
int gpioV1, gpioV2, gpioV3, gpioV4;
int gpioV1Map, gpioV2Map, gpioV3Map, gpioV4Map;
int missed_bb, missed_cc, missed_dd, missed_ee;
int buf_rssi_bb_int, buf_rssi_cc_int, buf_rssi_dd_int, buf_rssi_ee_int;
int useIPOctet1, useIPOctet2, useIPOctet3, useIPOctet4;
int useGWOctet1, useGWOctet2, useGWOctet3, useGWOctet4;
int useSNOctet1, useSNOctet2, useSNOctet3, useSNOctet4;
int useDNS1Octet1, useDNS1Octet2, useDNS1Octet3, useDNS1Octet4;
int useDNS2Octet1, useDNS2Octet2, useDNS2Octet3, useDNS2Octet4;
int int_txpower;
int int_ipOctet1, int_ipOctet2, int_ipOctet3, int_ipOctet4;
int int_gwOctet1, int_gwOctet2, int_gwOctet3, int_gwOctet4;
int int_snOctet1, int_snOctet2, int_snOctet3, int_snOctet4;
int int_dns1Octet1, int_dns1Octet2, int_dns1Octet3, int_dns1Octet4;
int int_dns2Octet1, int_dns2Octet2, int_dns2Octet3, int_dns2Octet4;

///////////////////////////////////////////////
//////////// Setup LORA Values ////////////////
///////////////////////////////////////////////

int loraTxPower = 17;                   //2-20 default 17
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
bool useDNS = false;
bool useTSL = false;
bool bool_esm = false;
bool bool_tsl = false;
bool ethState = false;
bool loraInit = false;
bool sdInit = false;
bool oledInit = false;

///////////////////////////////////////////////
/////////// Setup Network Values //////////////
///////////////////////////////////////////////
    
// Use static ip address config
IPAddress local_ip (192,168,0,50);       //uint32_t
IPAddress gateway (192, 168, 0, 1);
IPAddress subnet (255, 255, 255, 0);
IPAddress dns1 (0, 1, 2, 3);
IPAddress dns2 (0, 1, 2, 3);

///////////////////////////////////////////////
///////////////////////////////////////////////
///////////////////////////////////////////////

#define ETH_CLK_MODE ETH_CLOCK_GPIO17_OUT       //ETH_CLOCK_GPIO17_OUT - 50MHz clock from internal APLL inverted output on GPIO17 - tested with LAN8720
#define ETH_POWER_PIN                       16  // Pin# of the enable signal for the external crystal oscillator (-1 to disable for internal APLL source)
#define ETH_TYPE ETH_PHY_LAN8720                // Type of the Ethernet PHY (LAN8720 or TLK110)
#define ETH_ADDR                             0  // I??C-address of Ethernet PHY (0 or 1 for LAN8720, 31 for TLK110)
#define ETH_MDC_PIN                         23  // Pin# of the I??C clock signal for the Ethernet PHY
#define ETH_MDIO_PIN                        18  // Pin# of the I??C IO signal for the Ethernet PHY
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

#define image_width                         32  //Bitmap Declaration
#define image_height                        32
#define loadWidth                           50
#define loadHeight                          50
#define logoWidth                          128
#define logoHeight                          64
#define loraWidth                          128
#define loraHeight                          64
#define batteryWidth                        29
#define batteryHeight                       15
#define signalWidth                         21
#define signalHeight                        18
#define lineWidth                            2
#define lineHeight                          10

U8G2_SSD1306_128X64_NONAME_F_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/ DISPLAY_CS, /* dc=*/ DISPLAY_MISO, /* reset=*/ DISPLAY_RST);
//U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ 22, /* data=*/ 21);   // ESP32 Thing, HW I2C with pin remapping

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

    if(state == "STATE_ADDRESS"){        
            return string_localAddress;
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

    if(state == "STATE_DNS"){
        if(useDNS == true){
            html_state_dns = "ON";
        }
        else{
            html_state_dns = "OFF";
        }
        return html_state_dns;
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

void sendMessage(String message) {
    LoRa.beginPacket();                   // start packet
    LoRa.write(destination);              // add destination address
    LoRa.write(localAddress);             // add sender address
    LoRa.write(msgKey1);                  // add message KEY
    LoRa.write(msgKey2);                  // add message KEY
    LoRa.write(esm);                      // add energy save mode
    LoRa.write(byte_txpower);             // add txpower
    LoRa.write(msgCount);                 // add message ID
    LoRa.write(message.length());         // add payload length
    LoRa.print(message);                  // add payload
    LoRa.endPacket();                     // finish packet and send it
    msgCount++;                           // increment message ID
}

//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////

void onReceive(int packetSize, String *ptr_rx_adr, String *ptr_tx_adr, String *ptr_incoming, String *ptr_rssi, String *ptr_bL, String *ptr_snr) { 
    if (packetSize == 0) return;          // if there's no packet, return

    //Clear the variables
    *ptr_rx_adr = "";
    *ptr_tx_adr = "";
    *ptr_incoming = "";
    *ptr_rssi = "";
    *ptr_bL = "";
    *ptr_snr = "";

    string_destinationAddress = "";
    rx_adr = "";
    outgoing = "";
    tx_adr = "";
    incoming = "";
    rssi = "";
    bL = "";
    snr = "";

    // read packet header bytes:
    int recipient = LoRa.read();          // recipient address
    byte sender = LoRa.read();            // sender address
    byte incomingMsgKey1 = LoRa.read();   // incoming msg KEY1
    byte incomingMsgKey2 = LoRa.read();   // incoming msg KEY2
    byte byte_rssi = LoRa.read();         // incoming byte_rssi
    //byte byte_snr = LoRa.read();          // incoming byte_snr
    byte byte_bL = LoRa.read();           // incoming byte_bL
    byte incomingMsgId = LoRa.read();     // incoming msg ID
    byte incomingLength = LoRa.read();    // incoming msg length

    while (LoRa.available()) {
        incoming += (char)LoRa.read();
    }

    if (incomingMsgKey1 != msgKey1 && incomingMsgKey2 != msgKey2) {
        Serial.println("Error: Message key false.");
        return;                             // skip rest of function
    }

    if (incomingLength != incoming.length()) {   // check length for error
        Serial.println("Error: Message length false.");
        return;                             // skip rest of function
    }

    // if the recipient isn't this device or broadcast,
    if (recipient != localAddress && recipient != 0xff) {
        Serial.println("Message not for me.");
        return;                             // skip rest of function
    }

    *ptr_rx_adr = String(recipient, HEX);
    *ptr_tx_adr = String(sender, HEX);
    *ptr_incoming = incoming;
    *ptr_rssi = String(byte_rssi - 256);
    *ptr_bL = String(byte_bL);
    *ptr_snr = String(LoRa.packetSnr());

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

void emptyDisplay() {
    string_destinationAddress = "";
    rx_adr = "";
    outgoing = "";
    tx_adr = "";
    incoming = "";
    rssi = "";
    snr = "";
}

//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////

void printDisplay() {   

    sprintf(buf_bb, "%s", bb);
    sprintf(buf_cc, "%s", cc);
    sprintf(buf_dd, "%s", dd);
    sprintf(buf_ee, "%s", ee);
    sprintf(buf_lost, "%s", lost);
    sprintf(buf_rssi_bb, "%s", rssi_bb);
    sprintf(buf_rssi_cc, "%s", rssi_cc);
    sprintf(buf_rssi_dd, "%s", rssi_dd);
    sprintf(buf_rssi_ee, "%s", rssi_ee);
    sprintf(buf_bL_bb, "%s", bL_bb);
    sprintf(buf_bL_cc, "%s", bL_cc);
    sprintf(buf_bL_dd, "%s", bL_dd);
    sprintf(buf_bL_ee, "%s", bL_ee);
    
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

    buf_rssi_bb_int = atoi(buf_rssi_bb);
    buf_rssi_cc_int = atoi(buf_rssi_cc);
    buf_rssi_dd_int = atoi(buf_rssi_dd);
    buf_rssi_ee_int = atoi(buf_rssi_ee);

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

    //u8g2.drawStr(0,54,buf_ls);
    //u8g2.drawStr(30,54,"Mbps");
    
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

void notFound(AsyncWebServerRequest *request) {
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

    eeprom.begin("network", false);                //false mean use read/write mode
    useSTATIC = eeprom.getBool("dhcp", false);     //false mean default value if nothing returned
    useDNS = eeprom.getBool("dns", false);
    useIPOctet1 = eeprom.getInt("ipOctet1", false);
    useIPOctet2 = eeprom.getInt("ipOctet2", false);
    useIPOctet3 = eeprom.getInt("ipOctet3", false);
    useIPOctet4 = eeprom.getInt("ipOctet4", false);
    useGWOctet1 = eeprom.getInt("gwOctet1", false);
    useGWOctet2 = eeprom.getInt("gwOctet2", false);
    useGWOctet3 = eeprom.getInt("gwOctet3", false);
    useGWOctet4 = eeprom.getInt("gwOctet4", false);  
    useSNOctet1 = eeprom.getInt("snOctet1", false);
    useSNOctet2 = eeprom.getInt("snOctet2", false);
    useSNOctet3 = eeprom.getInt("snOctet3", false);
    useSNOctet4 = eeprom.getInt("snOctet4", false);  
    useDNS1Octet1 = eeprom.getInt("dns1Octet1", false);
    useDNS1Octet2 = eeprom.getInt("dns1Octet2", false);
    useDNS1Octet3 = eeprom.getInt("dns1Octet3", false);
    useDNS1Octet4 = eeprom.getInt("dns1Octet4", false);  
    useDNS2Octet1 = eeprom.getInt("dns2Octet1", false);
    useDNS2Octet2 = eeprom.getInt("dns2Octet2", false);
    useDNS2Octet3 = eeprom.getInt("dns2Octet3", false);
    useDNS2Octet4 = eeprom.getInt("dns2Octet4", false);  
    eeprom.end();

    eeprom.begin("configuration", false); 
    bool_tsl = eeprom.getBool("tsl", false);        
    bool_esm = eeprom.getBool("esm", false);
    loraTxPower = eeprom.getInt("txpower", false);

    Serial.print("loraTxPower: "); Serial.println(loraTxPower);
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

    IPAddress local_ip (useIPOctet1, useIPOctet2, useIPOctet3, useIPOctet4);
    IPAddress gateway (useGWOctet1, useGWOctet2, useGWOctet3, useGWOctet4);
    IPAddress subnet (useSNOctet1, useSNOctet2, useSNOctet3, useSNOctet4);
    IPAddress dns1 (useDNS1Octet1, useDNS1Octet2, useDNS1Octet3, useDNS1Octet4);
    IPAddress dns2 (useDNS2Octet1, useDNS2Octet2, useDNS2Octet3, useDNS2Octet4);
    
    if ((useSTATIC == true) && (useDNS = false)) {
        ETH.config(local_ip, gateway, subnet);
    }

    if ((useSTATIC == true) && (useDNS = true)) {
        ETH.config(local_ip, gateway, subnet, dns1, dns2);
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
        request->send(SPIFFS, "/index.html", String(), false, proc_state);
    });
    
    // Route to load style.css file
    server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(SPIFFS, "/style.css", "text/css");
    });

    server.on("/network", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(SPIFFS, "/network.html", String(), false, proc_state);
    });

    server.on("/dhcp-on", HTTP_GET, [](AsyncWebServerRequest *request){
        useSTATIC = false;
        eeprom.begin("network", false);                //false mean use read/write mode
        eeprom.putBool("dhcp", useSTATIC);     
        eeprom.end();
        //Serial.print("useSTATIC: "); Serial.print(useSTATIC);
        //Serial.print("useDNS: "); Serial.print(useDNS);
        request->send(SPIFFS, "/network.html", String(), false, proc_state);
        delay(2000);
        ESP.restart();
    });

    server.on("/dhcp-off", HTTP_GET, [](AsyncWebServerRequest *request){
        useSTATIC = true;
        eeprom.begin("network", false);                //false mean use read/write mode
        eeprom.putBool("dhcp", useSTATIC);     
        eeprom.end();
        //Serial.print("useSTATIC: "); Serial.print(useSTATIC);
        //Serial.print("useDNS: "); Serial.print(useDNS);
        request->send(SPIFFS, "/network.html", String(), false, proc_state);
        delay(2000);
        ESP.restart();
    });

    server.on("/dns-on", HTTP_GET, [](AsyncWebServerRequest *request){
        useDNS = true;
        eeprom.begin("network", false);                //false mean use read/write mode
        eeprom.putBool("dns", useDNS);     
        eeprom.end();
        //Serial.print("useSTATIC: "); Serial.print(useSTATIC);
        //Serial.print("useDNS: "); Serial.print(useDNS);
        request->send(SPIFFS, "/network.html", String(), false, proc_state);
        delay(2000);
        ESP.restart();
    });

    server.on("/dns-off", HTTP_GET, [](AsyncWebServerRequest *request){
        useDNS = false;
        eeprom.begin("network", false);                //false mean use read/write mode
        eeprom.putBool("dns", useDNS);     
        eeprom.end();
        //Serial.print("useSTATIC: "); Serial.print(useSTATIC);
        //Serial.print("useDNS: "); Serial.print(useDNS);
        request->send(SPIFFS, "/network.html", String(), false, proc_state);
        delay(2000);
        ESP.restart();
    });

    server.on("/configuration", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(SPIFFS, "/configuration.html", String(), false, proc_state);
    });

    server.on("/tsl", HTTP_GET, [](AsyncWebServerRequest *request){
        useTSL = true;
        eeprom.begin("configuration", false);                //false mean use read/write mode
        eeprom.putBool("tsl", bool_tsl);     
        eeprom.end();
        request->send(SPIFFS, "/configuration.html", String(), false, proc_state);
    });

    server.on("/gpio", HTTP_GET, [](AsyncWebServerRequest *request){
        useTSL = false;
        eeprom.begin("configuration", false);                //false mean use read/write mode
        eeprom.putBool("tsl", bool_tsl);     
        eeprom.end();
        request->send(SPIFFS, "/configuration.html", String(), false, proc_state);
    });

    server.on("/esm-on", HTTP_GET, [](AsyncWebServerRequest *request){
        bool_esm = true;
        if (bool_esm == true){ esm = 0x01; }
        if (bool_esm == false){ esm = 0x00; }
        eeprom.begin("configuration", false);                //false mean use read/write mode
        eeprom.putBool("esm", bool_esm);     
        eeprom.end();
        request->send(SPIFFS, "/configuration.html", String(), false, proc_state);
    });

    server.on("/esm-off", HTTP_GET, [](AsyncWebServerRequest *request){
        bool_esm = false;
        if (bool_esm == true){ esm = 0x01; }
        if (bool_esm == false){ esm = 0x00; }
        eeprom.begin("configuration", false);                //false mean use read/write mode
        eeprom.putBool("esm", bool_esm);     
        eeprom.end();
        request->send(SPIFFS, "/configuration.html", String(), false, proc_state);
    });

    server.on("/info", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(SPIFFS, "/info.html", String(), false, proc_state);
    });

    // Send a GET request to <ESP_IP>/get?input1=<inputMessage>
    server.on("/get-txp", HTTP_GET, [] (AsyncWebServerRequest *request) {
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
        int_txpower = atoi(buf_txpower);

        loraTxPower = int_txpower;              //value for eeprom
        byte_txpower = int_txpower;             //send via lora

        eeprom.begin("configuration", false);                //false mean use read/write mode
        eeprom.putInt("txpower", loraTxPower);     
        eeprom.end();

        request->send(SPIFFS, "/configuration.html", String(), false, proc_state);   

        if (tally_bb == HIGH){
            destination = 0xbb;                                                                     //if tx power changed via webterminal, then send message to receivers and change the txpower with restart
            string_destinationAddress = "bb";
            outgoing = "con-rec?";         // Send a message
            sendMessage(outgoing);
            Serial.println("LORA TxD: " + outgoing);
            delay(100);
        }
        if (tally_cc == HIGH){
            destination = 0xcc;
            string_destinationAddress = "cc";
            outgoing = "con-rec?";         // Send a message
            sendMessage(outgoing);
            Serial.println("LORA TxD: " + outgoing);
            delay(100);
        }
        if (tally_dd == HIGH){
            destination = 0xdd;
            string_destinationAddress = "dd";
            outgoing = "con-rec?";         // Send a message
            sendMessage(outgoing);
            Serial.println("LORA TxD: " + outgoing);
            delay(100);
        }
        if (tally_ee == HIGH){
            destination = 0xee;
            string_destinationAddress = "ee";
            outgoing = "con-rec?";         // Send a message
            sendMessage(outgoing);
            Serial.println("LORA TxD: " + outgoing);
        }

        delay(2000);
        ESP.restart();
    });

    // Send a GET request to <ESP_IP>/get?input1=<inputMessage>
    server.on("/get-ip", HTTP_GET, [] (AsyncWebServerRequest *request) {
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
        //Serial.println(input_ip);
        //Serial.println(input_gw);
        //Serial.println(input_sn);
        //Serial.println(input_dns1);
        //Serial.println(input_dns2);

        sprintf(buf_input_ip, "%s", input_ip);
        sprintf(buf_input_gw, "%s", input_gw);
        sprintf(buf_input_sn, "%s", input_sn);
        sprintf(buf_input_dns1, "%s", input_dns1);
        sprintf(buf_input_dns2, "%s", input_dns2);

        char* ptr_ip = strtok(buf_input_ip, ".");
        int_ipOctet1 = atoi(ptr_ip);
        ptr_ip = strtok(NULL, ".");
        int_ipOctet2 = atoi(ptr_ip);
        ptr_ip = strtok(NULL, ".");
        int_ipOctet3 = atoi(ptr_ip);
        ptr_ip = strtok(NULL, ".");
        int_ipOctet4 = atoi(ptr_ip);

        char* ptr_gw = strtok(buf_input_gw, ".");
        int_gwOctet1 = atoi(ptr_gw);
        ptr_gw = strtok(NULL, ".");
        int_gwOctet2 = atoi(ptr_gw);
        ptr_gw = strtok(NULL, ".");
        int_gwOctet3 = atoi(ptr_gw);
        ptr_gw = strtok(NULL, ".");
        int_gwOctet4 = atoi(ptr_gw);

        char* ptr_sn = strtok(buf_input_sn, ".");
        int_snOctet1 = atoi(ptr_sn);
        ptr_sn = strtok(NULL, ".");
        int_snOctet2 = atoi(ptr_sn);
        ptr_sn = strtok(NULL, ".");
        int_snOctet3 = atoi(ptr_sn);
        ptr_sn = strtok(NULL, ".");
        int_snOctet4 = atoi(ptr_sn);

        char* ptr_dns1 = strtok(buf_input_dns1, ".");
        int_dns1Octet1 = atoi(ptr_dns1);
        ptr_dns1 = strtok(NULL, ".");
        int_dns1Octet2 = atoi(ptr_dns1);
        ptr_dns1 = strtok(NULL, ".");
        int_dns1Octet3 = atoi(ptr_dns1);
        ptr_dns1 = strtok(NULL, ".");
        int_dns1Octet4 = atoi(ptr_dns1);

        char* ptr_dns2 = strtok(buf_input_dns2, ".");
        int_dns2Octet1 = atoi(ptr_dns2);
        ptr_dns2 = strtok(NULL, ".");
        int_dns2Octet2 = atoi(ptr_dns2);
        ptr_dns2 = strtok(NULL, ".");
        int_dns2Octet3 = atoi(ptr_dns2);
        ptr_dns2 = strtok(NULL, ".");
        int_dns2Octet4 = atoi(ptr_dns2);

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

        request->send(SPIFFS, "/network.html", String(), false, proc_state);
        delay(2000);
        ESP.restart();
    });

    server.onNotFound(notFound);

    server.begin();
    Serial.println("HTTP server started.");
    sprintf(buf_httpInit, "%s", "HTTP started");
    u8g2.drawStr(0,60,buf_httpInit);
    u8g2.sendBuffer();
    delay(500);

//////////////////////////////////////////////////////////////////////
 
    printLora(1);
    delay(2500);

    emptyDisplay();
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
        string_destinationAddress = "ff";
        outgoing = "dis-anyrec?";         // Send a message
        sendMessage(outgoing);
        Serial.println("LORA TxD: " + outgoing);
        lastOfferTime = millis();
        lastOfferTimeRef = millis();
        lastDiscoverTimebb = millis();
        lastDiscoverTimecc = millis();
        lastDiscoverTimedd = millis();
        lastDiscoverTimeee = millis();
        mode = "offer";
        mode_s = "off";
        emptyDisplay();
    }

    // Offer Mode
    while (mode == "offer") {
        onReceive(LoRa.parsePacket(), &rx_adr, &tx_adr, &incoming, &rssi, &bL, &snr);    // Parse Packets and Read it

        if ((incoming == "off") && (tx_adr == "bb") && (tally_bb == LOW)) {
            Serial.println("LORA RxD: " + incoming);
            tally_bb = HIGH;
            tally_bb_init = HIGH;
            incoming_bb = incoming;
            tx_adr_bb = tx_adr;
            rssi_bb = rssi;
            bL_bb = bL;
            counterTallys++;
            lastOfferTime = millis();
            lastOfferTimeEnd = millis();
            emptyDisplay();
        }
        if ((incoming == "off") && (tx_adr == "cc") && (tally_cc == LOW)) {
            Serial.println("LORA RxD: " + incoming);
            tally_cc = HIGH;
            tally_cc_init = HIGH;
            incoming_cc = incoming;
            tx_adr_cc = tx_adr;
            rssi_cc = rssi;
            bL_cc = bL;
            counterTallys++;
            lastOfferTime = millis();
            lastOfferTimeEnd = millis();
            emptyDisplay();
        }
        if ((incoming == "off") && (tx_adr == "dd") && (tally_dd == LOW)) {
            Serial.println("LORA RxD: " + incoming);
            tally_dd = HIGH;
            tally_dd_init = HIGH;
            incoming_dd = incoming;
            tx_adr_dd = tx_adr;
            rssi_dd = rssi;
            bL_dd = bL;
            counterTallys++;
            lastOfferTime = millis();
            lastOfferTimeEnd = millis();
            emptyDisplay();
        }
        if ((incoming == "off") && (tx_adr == "ee") && (tally_ee == LOW)) {
            Serial.println("LORA RxD: " + incoming);
            tally_ee = HIGH;
            tally_ee_init = HIGH;
            incoming_ee = incoming;
            tx_adr_ee = tx_adr;
            rssi_ee = rssi;
            bL_ee = bL;
            counterTallys++;
            lastOfferTime = millis();
            lastOfferTimeEnd = millis();
            emptyDisplay();
        }

        if ((millis() - lastOfferTimeRef > 2500)) {      // every 2.5 s clear display
            emptyDisplay();
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

        if (gpioV1Cal > 2.2 && tally_bb == HIGH && gpioC1 == HIGH) {
            destination = 0xbb;
            string_destinationAddress = "bb";
            outgoing = "req-high";         // Send a message
            sendMessage(outgoing);
            Serial.println("LORA TxD: " + outgoing);
            gpioC1 = !gpioC1;
            mode = "acknowledge";
            mode_s = "ack";
            lastAckTime = millis();
            emptyDisplay();
        }

        if (gpioV1Cal < 2.2 && tally_bb == HIGH && gpioC1 == LOW) {
            destination = 0xbb;
            string_destinationAddress = "bb";
            outgoing = "req-low";         // Send a message
            sendMessage(outgoing);
            Serial.println("LORA TxD: " + outgoing);
            gpioC1 = !gpioC1;
            mode = "acknowledge";
            mode_s = "ack";
            lastAckTime = millis();
            emptyDisplay();
        }

        if (gpioV2Cal > 2.2 && tally_cc == HIGH && gpioC2 == HIGH) {
            destination = 0xcc;
            string_destinationAddress = "cc";
            outgoing = "req-high";         // Send a message
            sendMessage(outgoing);
            Serial.println("LORA TxD: " + outgoing);
            gpioC2 = !gpioC2;
            mode = "acknowledge";
            mode_s = "ack";
            lastAckTime = millis();
            emptyDisplay();
        }

        if (gpioV2Cal < 2.2 && tally_cc == HIGH && gpioC2 == LOW) {
            destination = 0xcc;
            string_destinationAddress = "cc";
            outgoing = "req-low";         // Send a message
            sendMessage(outgoing);
            Serial.println("LORA TxD: " + outgoing);
            gpioC2 = !gpioC2;
            mode = "acknowledge";
            mode_s = "ack";
            lastAckTime = millis();
            emptyDisplay();
        }

        if (gpioV3Cal > 2.2 && tally_dd == HIGH && gpioC3 == HIGH) {
            destination = 0xdd;
            string_destinationAddress = "dd";
            outgoing = "req-high";         // Send a message
            sendMessage(outgoing);
            Serial.println("LORA TxD: " + outgoing);
            gpioC3 = !gpioC3;
            mode = "acknowledge";
            mode_s = "ack";
            lastAckTime = millis();
            emptyDisplay();
        }

        if (gpioV3Cal < 2.2 && tally_dd == HIGH && gpioC3 == LOW) {
            destination = 0xdd;
            string_destinationAddress = "dd";
            outgoing = "req-low";         // Send a message
            sendMessage(outgoing);
            Serial.println("LORA TxD: " + outgoing);
            gpioC3 = !gpioC3;
            mode = "acknowledge";
            mode_s = "ack";
            lastAckTime = millis();
            emptyDisplay();
        }

        if (gpioV4Cal > 2.2 && tally_ee == HIGH && gpioC4 == HIGH) {
            destination = 0xee;
            string_destinationAddress = "ee";
            outgoing = "req-high";         // Send a message
            sendMessage(outgoing);
            Serial.println("LORA TxD: " + outgoing);
            gpioC4 = !gpioC4;
            mode = "acknowledge";
            mode_s = "ack";
            lastAckTime = millis();
            emptyDisplay();
        }

        if (gpioV4Cal < 2.2 && tally_ee == HIGH && gpioC4 == LOW) {
            destination = 0xee;
            string_destinationAddress = "ee";
            outgoing = "req-low";         // Send a message
            sendMessage(outgoing);
            Serial.println("LORA TxD: " + outgoing);
            gpioC4 = !gpioC4;
            mode = "acknowledge";
            mode_s = "ack";
            lastAckTime = millis();
            emptyDisplay();
        }
        lastAnalogReadTime = millis();
    }

    // Request Mode TSL
    if ((mode == "request") && (millis() - lastTslReadTime > 250) && (useTSL == true)) {
        lastTslReadTime = millis();
    }

    // Acknowledge Mode
    while (mode == "acknowledge") {
        onReceive(LoRa.parsePacket(), &rx_adr, &tx_adr, &incoming, &rssi, &bL, &snr);    // Parse Packets and Read it    
    
        // Back to Request Mode
        if ((incoming == "ack") && ((tx_adr == "bb") || (tx_adr == "cc") || (tx_adr == "dd") ||  (tx_adr == "ee"))) {
            Serial.println("LORA RxD: " + incoming);
            mode = "request";
            mode_s = "req";
            emptyDisplay();
            break;
        }

    // Toggel and Resend Message, if ACK not arrived after 2 secounds
    if ((millis() - lastAckTime > 2000) && (counterSend < counterSendMax)) {
        emptyDisplay();
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
        emptyDisplay();
        mode = "request";
        mode_s = "req";
        counterSend = 0;
        break;
    }
    }

    // Control Mode BB after discover and 3 - 3.5 minutes or if BB offline, control after 9 minutes
    if (((millis() - lastDiscoverTimebb > 180000) && ((tally_bb == HIGH) || (tally_bb_init == HIGH))) || ((millis() - lastDiscoverTimebb > 540000) && ((tally_bb == LOW) || (tally_bb_init == LOW)))) {
        destination = 0xbb;
        string_destinationAddress = "bb";
        outgoing = "con-rec?";         // Send a message
        sendMessage(outgoing);
        Serial.println("LORA TxD: " + outgoing);
        lastDiscoverTimebb = millis();
        lastControlTime = millis();
        mode = "control";
        mode_s = "con";
        emptyDisplay();

    while (mode == "control") {
        onReceive(LoRa.parsePacket(), &rx_adr, &tx_adr, &incoming, &rssi, &bL, &snr);    // Parse Packets and Read it

        if ((incoming == "con") && (tx_adr == "bb")) {
            Serial.println("LORA RxD: " + incoming);
            if (tally_bb_init == LOW) {
            counterTallys++;
            }
            tally_bb = HIGH;
            tally_bb_init = HIGH;
            incoming_bb = incoming;         // reguster values for rssi measuring
            tx_adr_bb = tx_adr;
            rssi_bb = rssi;
            bL_bb = bL;
            missed_bb = 0;
            mode = "request";
            mode_s = "req";
            emptyDisplay();
            break;
        }

        if (millis() - lastControlTime > 3000) {
            missed_bb++;
        
            if (missed_bb >= 2) {
                tally_bb = LOW;
                counterTallys--;
            }
            mode = "request"; 
            mode_s = "req";
            emptyDisplay();
            break;
        }
        }
    }
  
    // Control Mode CC after discover and 3 - 3.5 minutes or if BB offline, control after 9.5 minutes
    if (((millis() - lastDiscoverTimecc > 190000) && ((tally_cc == HIGH) || (tally_cc_init == HIGH))) || ((millis() - lastDiscoverTimecc > 570000) && ((tally_cc == LOW) || (tally_cc_init == LOW)))) {
        destination = 0xcc;
        string_destinationAddress = "cc";
        outgoing = "con-rec?";         // Send a message
        sendMessage(outgoing);
        Serial.println("LORA TxD: " + outgoing);
        lastDiscoverTimecc = millis();
        lastControlTime = millis();
        mode = "control";
        mode_s = "con";
        emptyDisplay();

        while (mode == "control") {
            onReceive(LoRa.parsePacket(), &rx_adr, &tx_adr, &incoming, &rssi, &bL, &snr);    // Parse Packets and Read it
      
            if ((incoming == "con") && (tx_adr == "cc")) {
                Serial.println("LORA RxD: " + incoming);
                if (tally_cc_init == LOW) {
                counterTallys++;
                }
                tally_cc = HIGH;
                tally_cc_init = HIGH;
                incoming_cc = incoming;
                tx_adr_cc = tx_adr;
                rssi_cc = rssi;
                bL_cc = bL;
                missed_cc = 0;
                mode = "request";
                mode_s = "req";
                emptyDisplay();
                break;
            }

            if (millis() - lastControlTime > 3000) {
                missed_cc++;
                
                if (missed_cc >= 2) {
                tally_cc = LOW;
                counterTallys--;
                }
                mode = "request"; 
                mode_s = "req";
                emptyDisplay();
                break;
            }
            }
        }

    // Control Mode DD after discover and 3 - 3.5 minutes or if BB offline, control after 10 minutes
    if (((millis() - lastDiscoverTimedd > 200000) && ((tally_dd == HIGH) || (tally_dd_init == HIGH))) || ((millis() - lastDiscoverTimedd > 600000) && ((tally_dd == LOW) || (tally_dd_init == LOW)))) {
        destination = 0xdd;
        string_destinationAddress = "dd";
        outgoing = "con-rec?";         // Send a message
        sendMessage(outgoing);
        Serial.println("LORA TxD: " + outgoing);
        lastDiscoverTimedd = millis();
        lastControlTime = millis();
        mode = "control";
        mode_s = "con";
        emptyDisplay();

        while (mode == "control") {
            onReceive(LoRa.parsePacket(), &rx_adr, &tx_adr, &incoming, &rssi, &bL, &snr);    // Parse Packets and Read it
            
            if ((incoming == "con") && (tx_adr == "dd")) {
                Serial.println("LORA RxD: " + incoming);
                if (tally_dd_init == LOW) {
                counterTallys++;
                }
                tally_dd = HIGH;
                tally_dd_init = HIGH;
                incoming_dd = incoming;
                tx_adr_dd = tx_adr;
                rssi_dd = rssi;
                bL_dd = bL;
                missed_dd = 0;
                mode = "request";
                mode_s = "req";
                emptyDisplay();
                break;
            }

            if (millis() - lastControlTime > 3000) {
                missed_dd++;
                
                if (missed_dd >= 2) {
                tally_dd = LOW;
                counterTallys--;
                }
                mode = "request"; 
                mode_s = "req";
                emptyDisplay();
                break;
            }
            }
        }

    // Control Mode EE after discover and 3 - 3.5 minutes or if BB offline, control after 10.5 minutes
    if (((millis() - lastDiscoverTimeee > 210000) && ((tally_ee == HIGH) || (tally_ee_init == HIGH))) || ((millis() - lastDiscoverTimeee > 630000) && ((tally_ee == LOW) || (tally_ee_init == LOW)))) {
        destination = 0xee;
        string_destinationAddress = "ee";
        outgoing = "con-rec?";         // Send a message
        sendMessage(outgoing);
        Serial.println("LORA TxD: " + outgoing);
        lastDiscoverTimeee = millis();
        lastControlTime = millis();
        mode = "control";
        mode_s = "con";
        emptyDisplay();

        while (mode == "control") {
            onReceive(LoRa.parsePacket(), &rx_adr, &tx_adr, &incoming, &rssi, &bL, &snr);    // Parse Packets and Read it
            
            if ((incoming == "con") && (tx_adr == "ee")) {
                Serial.println("LORA RxD: " + incoming);
                if (tally_ee_init == LOW) {
                counterTallys++;
                }
                tally_ee = HIGH;
                tally_ee_init = HIGH;
                incoming_ee = incoming;
                tx_adr_ee = tx_adr;
                rssi_ee = rssi;
                bL_ee = bL;
                missed_ee = 0;
                mode = "request";
                mode_s = "req";
                emptyDisplay();
                break;
            }

            if (millis() - lastControlTime > 3000) {
                missed_ee++;
                
                if (missed_ee >= 2) {
                tally_ee = LOW;
                counterTallys--;
                }
                mode = "request"; 
                mode_s = "req";
                emptyDisplay();
                break;
            }
            }
        }

        // Function Print Display if nothing work
        if (ethConnected == true && ethState == false) {
            
            closeSPI_LORA();
            startSPI_DISPLAY();
            emptyDisplay();
            printDisplay();
            closeSPI_DISPLAY();
            startSPI_LORA();
            ethState = !ethState;
        }
        if (ethConnected == false && ethState == true) {
            
            closeSPI_LORA();
            startSPI_DISPLAY();
            emptyDisplay();
            printDisplay();
            closeSPI_DISPLAY();
            startSPI_LORA();
            ethState = !ethState;
        }
}

//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////