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
String version = "0.0a";                   // Frimeware Version
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

String oledInit;
String loraInit;
String outputInit;

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
char buf_oledInit[12];
char buf_loraInit[12];
char buf_outputInit[12];
char buf_rssi_bb[4];
char buf_rssi_cc[4];
char buf_rssi_dd[4];
char buf_rssi_ee[4];
char buf_bL_bb[4];
char buf_bL_cc[4];
char buf_bL_dd[4];
char buf_bL_ee[4];

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

int defaultBrightnessDisplay = 150;   // value from 1 to 255
int counterSend = 0;
int counterSendMax = 2;
int counterTallys = 0;
int gpioP1 = 36, gpioP2 = 39, gpioP3 = 34, gpioP4 = 35;
int gpioV1, gpioV2, gpioV3, gpioV4;
int gpioV1Map, gpioV2Map, gpioV3Map, gpioV4Map;
int missed_bb, missed_cc, missed_dd, missed_ee;
int buf_rssi_bb_int, buf_rssi_cc_int, buf_rssi_dd_int, buf_rssi_ee_int;

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

static bool ethConnected = false;
static bool useSTATIC = false;
static bool useDNS = false;

///////////////////////////////////////////////
/////////// Setup Network Values //////////////
///////////////////////////////////////////////
    
// Use static ip address config
IPAddress local_ip (192, 168, 0, 55);       //uint32_t
IPAddress gateway (192, 168, 0, 1);
IPAddress subnet (255, 255, 255, 0);
IPAddress dns1 (0, 0, 0, 0);
IPAddress dns2 (0, 0, 0, 0);

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

//U8G2_SSD1306_128X64_NONAME_F_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/ DISPLAY_CS, /* dc=*/ DISPLAY_MISO, /* reset=*/ DISPLAY_RST);
//U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ 22, /* data=*/ 21);   // ESP32 Thing, HW I2C with pin remapping

AsyncWebServer server(80);

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

  return String();
}

//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////

void printLogo(int color, int wait) {
    /*
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
    */
}

void printLoad(int color, int wait, int count) {
    /*
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
        */
}

void printLora(int color) {
    /*
    u8g2.setDrawColor(color);
    u8g2.firstPage();
    do { u8g2.drawXBM(0, 0, loraWidth, loraHeight, lora); }
    while (u8g2.nextPage());
    */
}

//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////

void sendMessage(String message) {
    digitalWrite(LORA_CS, LOW);           //Select LORA SPI Device
    LoRa.beginPacket();                   // start packet
    LoRa.write(destination);              // add destination address
    LoRa.write(localAddress);             // add sender address
    LoRa.write(msgKey1);                  // add message KEY
    LoRa.write(msgKey2);                  // add message KEY
    LoRa.write(esm);                      // add energy save mode
    LoRa.write(msgCount);                 // add message ID
    LoRa.write(message.length());         // add payload length
    LoRa.print(message);                  // add payload
    LoRa.endPacket();                     // finish packet and send it
    msgCount++;                           // increment message ID
    digitalWrite(LORA_CS, HIGH);           //Select LORA SPI Device
}

//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////

void onReceive(int packetSize, String *ptr_rx_adr, String *ptr_tx_adr, String *ptr_incoming, String *ptr_rssi, String *ptr_bL, String *ptr_snr) {  
    digitalWrite(LORA_CS, LOW);           //Select LORA SPI Device
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
        Serial.println("Error: Message key is false.");
        return;                             // skip rest of function
    }

    if (incomingLength != incoming.length()) {   // check length for error
        Serial.println("Error: Message length does not match length.");
        return;                             // skip rest of function
    }

    // if the recipient isn't this device or broadcast,
    if (recipient != localAddress && recipient != 0xff) {
        Serial.println("This Message is not for me.");
        return;                             // skip rest of function
    }

    *ptr_rx_adr = String(recipient, HEX);
    *ptr_tx_adr = String(sender, HEX);
    *ptr_incoming = incoming;
    *ptr_rssi = String(byte_rssi - 256);
    *ptr_bL = String(byte_bL);
    *ptr_snr = String(LoRa.packetSnr());

    digitalWrite(LORA_CS, HIGH);
    return;
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

void printDisplay() {   //tx Transmit Message,  rx Receive Message,   txAdr Receive Address
    /*
    Serial.println("");
    Serial.print("Mode: "); Serial.println(mode);
    Serial.print("TxD Adr: "); Serial.println(string_destinationAddress);
    Serial.print("TxD: "); Serial.println(outgoing);
    Serial.print("RxD Adr: "); Serial.println(tx_adr);
    Serial.print("RxD: "); Serial.println(incoming);
    Serial.print("RxD Adr bb: "); Serial.println(tx_adr_bb);
    Serial.print("RxD bb: "); Serial.println(incoming_bb);
    Serial.print("Rssi: "); Serial.println(rssi_bb);
    Serial.print("Tally bb: "); Serial.println(tally_bb);
    Serial.print("Tally bb init: "); Serial.println(tally_bb_init);
    Serial.print("Tally cc: "); Serial.println(tally_cc);
    Serial.print("Tally cc init: "); Serial.println(tally_cc_init);
    Serial.print("Tally dd: "); Serial.println(tally_dd);
    Serial.print("Tally dd init: "); Serial.println(tally_dd_init);
    Serial.print("Tally ee: "); Serial.println(tally_ee);
    Serial.print("Tally ee init: "); Serial.println(tally_ee_init);
    */

    digitalWrite(DISPLAY_CS, LOW);      //Select Display SPI Device
    
    sprintf(buf_tx, "%s", outgoing);
    sprintf(buf_rx, "%s", incoming);

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

    sprintf(buf_localAddress, "%x", localAddress);          // byte
    sprintf(buf_mode, "%s", mode_s);                        // string  //%d for int
    sprintf(buf_rxAdr, "%s", string_destinationAddress);            
    sprintf(buf_txAdr, "%s", tx_adr);

    buf_rssi_bb_int = atoi(buf_rssi_bb);
    buf_rssi_cc_int = atoi(buf_rssi_cc);
    buf_rssi_dd_int = atoi(buf_rssi_dd);
    buf_rssi_ee_int = atoi(buf_rssi_ee);

    //u8g2.clearBuffer();					      // clear the internal memory

    //TxD and RxD Indicator
    //u8g2.setFont(u8g2_font_6x10_tf);
    //u8g2.setDrawColor(1);
    //u8g2.drawStr(0,26,"TxD:");
    //u8g2.drawStr(30,26,buf_tx);
    //u8g2.drawStr(115,26,buf_rxAdr);
    //u8g2.drawStr(0,36,"RxD:");
    //u8g2.drawStr(30,36,buf_rx);
    //u8g2.drawStr(115,36,buf_txAdr);
    
    //Address Indicator
    //u8g2.setFont(u8g2_font_6x13_tf);
    //u8g2.setDrawColor(1);
    //u8g2.drawXBM(20, 3, lineWidth, lineHeight, line1);
    //u8g2.setDrawColor(1);
    //u8g2.drawStr(3,12,buf_localAddress);

    //Mode Indicator
    //u8g2.setFont(u8g2_font_6x13_tf);
    //u8g2.setDrawColor(1);
    //u8g2.drawXBM(51, 3, lineWidth, lineHeight, line1);
    //u8g2.setDrawColor(1);
    //u8g2.drawStr(29,12,buf_mode);

    //u8g2.setFont(u8g2_font_6x13_tf);
    //u8g2.setDrawColor(0);

    //Power Mode Indicator
    //u8g2.drawXBM(99, 0, batteryWidth, batteryHeight, battery0);

    //Signal Strength Indicator bb
    if (((tally_bb == HIGH) || (tally_bb_init == HIGH)) && (buf_rssi_bb_int <= -80) && (tx_adr_bb == "bb") && ((incoming_bb == "off") || (incoming_bb == "con"))) {
        //u8g2.drawXBM(8, 46, signalWidth, signalHeight, signal1);
    }
    if (((tally_bb == HIGH) || (tally_bb_init == HIGH)) && (buf_rssi_bb_int <= -60 ) && (buf_rssi_bb_int >= -79) && (tx_adr_bb == "bb") && ((incoming_bb == "off") || (incoming_bb == "con"))) {
        //u8g2.drawXBM(8, 46, signalWidth, signalHeight, signal2);
    }
    if (((tally_bb == HIGH) || (tally_bb_init == HIGH)) && (buf_rssi_bb_int <= -40 ) && (buf_rssi_bb_int >= -59) && (tx_adr_bb == "bb") && ((incoming_bb == "off") || (incoming_bb == "con"))) {
        //u8g2.drawXBM(8, 46, signalWidth, signalHeight, signal3);
    }
    if (((tally_bb == HIGH) || (tally_bb_init == HIGH)) && (buf_rssi_bb_int <= -20 ) && (buf_rssi_bb_int >= -39) && (tx_adr_bb == "bb") && ((incoming_bb == "off") || (incoming_bb == "con"))) {
        //u8g2.drawXBM(8, 46, signalWidth, signalHeight, signal4);
    }
    if (((tally_bb == HIGH) || (tally_bb_init == HIGH)) && (buf_rssi_bb_int >= -19) && (tx_adr_bb == "bb") && ((incoming_bb == "off") || (incoming_bb == "con"))) {
        //u8g2.drawXBM(8, 46, signalWidth, signalHeight, signal5);
    }

    //Signal Strength Indicator cc
    if (((tally_cc == HIGH) || (tally_cc_init == HIGH)) && (buf_rssi_cc_int <= -80) && (tx_adr_cc == "cc") && ((incoming_cc == "off") || (incoming_cc == "con"))) {
        //u8g2.drawXBM(40, 46, signalWidth, signalHeight, signal1);
    }
    if (((tally_cc == HIGH) || (tally_cc_init == HIGH)) && (buf_rssi_cc_int <= -60 ) && (buf_rssi_cc_int >= -79) && (tx_adr_cc == "cc") && ((incoming_cc == "off") || (incoming_cc == "con"))) {
        //u8g2.drawXBM(40, 46, signalWidth, signalHeight, signal2);
    }
    if (((tally_cc == HIGH) || (tally_cc_init == HIGH)) && (buf_rssi_cc_int <= -40 ) && (buf_rssi_cc_int >= -59) && (tx_adr_cc == "cc") && ((incoming_cc == "off") || (incoming_cc == "con"))) {
        //u8g2.drawXBM(40, 46, signalWidth, signalHeight, signal3);
    }
    if (((tally_cc == HIGH) || (tally_cc_init == HIGH)) && (buf_rssi_cc_int <= -20 ) && (buf_rssi_cc_int >= -39) && (tx_adr_cc == "cc") && ((incoming_cc == "off") || (incoming_cc == "con"))) {
        //u8g2.drawXBM(40, 46, signalWidth, signalHeight, signal4);
    }
    if (((tally_cc == HIGH) || (tally_cc_init == HIGH)) && (buf_rssi_cc_int >= -19) && (tx_adr_cc == "cc") && ((incoming_cc == "off") || (incoming_cc == "con"))) {
        //u8g2.drawXBM(40, 46, signalWidth, signalHeight, signal5);
    }

    //Signal Strength Indicator dd
    if (((tally_dd == HIGH) || (tally_dd_init == HIGH)) && (buf_rssi_dd_int <= -80) && (tx_adr_dd == "dd") && ((incoming_dd == "off") || (incoming_dd == "con"))) {
        //u8g2.drawXBM(72, 46, signalWidth, signalHeight, signal1);
    }
    if (((tally_dd == HIGH) || (tally_dd_init == HIGH)) && (buf_rssi_dd_int <= -60 ) && (buf_rssi_dd_int >= -79) && (tx_adr_dd == "dd") && ((incoming_dd == "off") || (incoming_dd == "con"))) {
        //u8g2.drawXBM(72, 46, signalWidth, signalHeight, signal2);
    }
    if (((tally_dd == HIGH) || (tally_dd_init == HIGH)) && (buf_rssi_dd_int <= -40 ) && (buf_rssi_dd_int >= -59) && (tx_adr_dd == "dd") && ((incoming_dd == "off") || (incoming_dd == "con"))) {
        //u8g2.drawXBM(72, 46, signalWidth, signalHeight, signal3);
    }
    if (((tally_dd == HIGH) || (tally_dd_init == HIGH)) && (buf_rssi_dd_int <= -20 ) && (buf_rssi_dd_int >= -39) && (tx_adr_dd == "dd") && ((incoming_dd == "off") || (incoming_dd == "con"))) {
        //u8g2.drawXBM(72, 46, signalWidth, signalHeight, signal4);
    }
    if (((tally_dd == HIGH) || (tally_dd_init == HIGH)) && (buf_rssi_dd_int >= -19) && (tx_adr_dd == "dd") && ((incoming_dd == "off") || (incoming_dd == "con"))) {
        //u8g2.drawXBM(72, 46, signalWidth, signalHeight, signal5);
    }

    //Signal Strength Indicator ee
    if (((tally_ee == HIGH) || (tally_ee_init == HIGH)) && (buf_rssi_ee_int <= -80) && (tx_adr_ee == "ee") && ((incoming_ee == "off") || (incoming_ee == "con"))) {
        //u8g2.drawXBM(104, 46, signalWidth, signalHeight, signal1);
    }
    if (((tally_ee == HIGH) || (tally_ee_init == HIGH)) && (buf_rssi_ee_int <= -60 ) && (buf_rssi_ee_int >= -79) && (tx_adr_ee == "ee") && ((incoming_ee == "off") || (incoming_ee == "con"))) {
        //u8g2.drawXBM(104, 46, signalWidth, signalHeight, signal2);
    }
    if (((tally_ee == HIGH) || (tally_ee_init == HIGH)) && (buf_rssi_ee_int <= -40 ) && (buf_rssi_ee_int >= -59) && (tx_adr_ee == "ee") && ((incoming_ee == "off") || (incoming_ee == "con"))) {
        //u8g2.drawXBM(104, 46, signalWidth, signalHeight, signal3);
    }
    if (((tally_ee == HIGH) || (tally_ee_init == HIGH)) && (buf_rssi_ee_int <= -20 ) && (buf_rssi_ee_int >= -39) && (tx_adr_ee == "ee") && ((incoming_ee == "off") || (incoming_ee == "con"))) {
        //u8g2.drawXBM(104, 46, signalWidth, signalHeight, signal4);
    }
    if (((tally_ee == HIGH) || (tally_ee_init == HIGH)) && (buf_rssi_ee_int >= -19) && (tx_adr_ee == "ee") && ((incoming_ee == "off") || (incoming_ee == "con"))) {
        //u8g2.drawXBM(104, 46, signalWidth, signalHeight, signal5);
    }

    //Signal Lost Indicator bb
    if ((tally_bb == LOW) && (tally_bb_init == HIGH)) {
        //u8g2.setDrawColor(1);
        //u8g2.setFont(u8g2_font_6x10_tf);
        //u8g2.drawStr(0,47,buf_rssi_bb);
        //u8g2.drawStr(6,55,buf_bb);
        //u8g2.setFont(u8g2_font_10x20_tf);
        //u8g2.drawStr(19,47,buf_lost);
    }

    //Signal Lost Indicator cc
    if ((tally_cc == LOW) && (tally_cc_init == HIGH)) {
        //u8g2.setDrawColor(1);
        //u8g2.setFont(u8g2_font_6x10_tf);
        //u8g2.drawStr(32,47,buf_rssi_cc);
        //u8g2.drawStr(38,55,buf_cc);
        //u8g2.setFont(u8g2_font_10x20_tf);
        //u8g2.drawStr(51,47,buf_lost);
    }

    //Signal Lost Indicator dd
    if ((tally_dd == LOW) && (tally_dd_init == HIGH)) {
        //u8g2.setDrawColor(1);
        //u8g2.setFont(u8g2_font_6x10_tf);
        //u8g2.drawStr(64,47,buf_rssi_dd);
        //u8g2.drawStr(70,55,buf_dd);
        //u8g2.setFont(u8g2_font_10x20_tf);
        //u8g2.drawStr(83,47,buf_lost);
    }

    //Signal Lost Indicator ee
    if ((tally_ee == LOW) && (tally_ee_init == HIGH)) {
        //u8g2.setDrawColor(1);
        //u8g2.setFont(u8g2_font_6x10_tf);
        //u8g2.drawStr(96,47,buf_rssi_ee);
        //u8g2.drawStr(102,55,buf_ee);
        //u8g2.setFont(u8g2_font_10x20_tf);
        //u8g2.drawStr(115,47,buf_lost);
    }

    //Signal High Indicator bb
    if ((tally_bb == HIGH) && (tally_bb_init == HIGH)) {
        //u8g2.setDrawColor(1);
        //u8g2.setFont(u8g2_font_6x10_tf);
        //u8g2.drawStr(0,47,buf_rssi_bb);
        //u8g2.drawStr(6,55,buf_bb);
    }

    //Signal High Indicator cc
    if ((tally_cc == HIGH) && (tally_cc_init == HIGH)) {
        //u8g2.setDrawColor(1);
        //u8g2.setFont(u8g2_font_6x10_tf);
        //u8g2.drawStr(32,47,buf_rssi_cc);
        //u8g2.drawStr(38,55,buf_cc);
    }

    //Signal High Indicator dd
    if ((tally_dd == HIGH) && (tally_dd_init == HIGH)) {
        //u8g2.setDrawColor(1);
        //u8g2.setFont(u8g2_font_6x10_tf);
        //u8g2.drawStr(64,47,buf_rssi_dd);
        //u8g2.drawStr(70,55,buf_dd);
    }

    //Signal High Indicator ee
    if ((tally_ee == HIGH) && (tally_ee_init == HIGH)) {
        //u8g2.setDrawColor(1);
        //u8g2.setFont(u8g2_font_6x10_tf);
        //u8g2.drawStr(96,47,buf_rssi_ee);
        //u8g2.drawStr(102,55,buf_ee);
    }

    //u8g2.sendBuffer();

    lastDisplayPrint = millis();
    digitalWrite(DISPLAY_CS, HIGH);
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

void setup() {

    Serial.begin(115200);
    while (!Serial);

    Serial.println("");
    Serial.println(name);
    Serial.println("Version: " + version);

    pinMode(DISPLAY_CS, OUTPUT);                                //CS Display
    pinMode(LORA_CS, OUTPUT);                                   //CS LORA
    pinMode(SD_CS, OUTPUT);                                     //CS SD-Reader
    pinMode(SD_MISO, INPUT_PULLUP);

    digitalWrite(LORA_CS, HIGH);
    digitalWrite(DISPLAY_CS, HIGH);
    digitalWrite(SD_CS, LOW);

//////////////////////////////////////////////////////////////////////

    SPI.begin(SD_SCLK, SD_MISO, SD_MOSI);                      

    while (true) {
            if (SD.begin(SD_CS)) {
                Serial.println("SDCard init succeeded.");
                delay(300);
                break;
            }
            Serial.println("SDCard init failed. Check your connections.");
            delay(300);
            break;
        }

    digitalWrite(LORA_CS, HIGH);
    digitalWrite(DISPLAY_CS, LOW);
    digitalWrite(SD_CS, HIGH);         //Select Display SPI Device

//////////////////////////////////////////////////////////////////////

    //u8g2.setBusClock(1000000);
    //u8g2.begin();
    //u8g2.clearBuffer();
    //u8g2.setFont(u8g2_font_6x10_tf);
    //u8g2.setContrast(defaultBrightnessDisplay);                  
    //u8g2.setFlipMode(1);

    //        Color, Delay, Runs
    printLogo(0, 50);
    delay(1000);
    printLoad(1, 60, 2);
    printLogo(0, 25);
    delay(500);
    printLoad(1, 60, 4);

    sprintf(buf_version, "%s", version);
    //u8g2.drawStr(99,60,buf_version);
    //u8g2.sendBuffer();

    Serial.println("OLED init succeeded.");
    oledInit = "OLED init";
    sprintf(buf_oledInit, "%s", oledInit);
    //u8g2.drawStr(0,15,buf_oledInit);
    //u8g2.sendBuffer();
    delay(300);

    digitalWrite(LORA_CS, LOW);
    digitalWrite(DISPLAY_CS, HIGH);
    digitalWrite(SD_CS, HIGH);                                        //Select Display SPI Device 

//////////////////////////////////////////////////////////////////////  

    LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ);
    LoRa.setTxPower(loraTxPower);
    LoRa.setSpreadingFactor(loraSpreadingFactor);    
    LoRa.setSignalBandwidth(loraSignalBandwidth);
    LoRa.setCodingRate4(loraCodingRate);
    LoRa.setPreambleLength(loraPreambleLength);
    LoRa.begin(loraFrequenz);

    if (!LoRa.begin(loraFrequenz)) {                                 // initialize lora frequenz
        Serial.println("LORA init failed. Check your connections.");
        loraInit = "LORA failed";
        digitalWrite(LORA_CS, HIGH);
        digitalWrite(DISPLAY_CS, LOW);
        digitalWrite(SD_CS, HIGH);                                  //Select Display SPI Device    
        sprintf(buf_loraInit, "%s", loraInit);   
        //u8g2.drawStr(0,35,buf_loraInit);
        //u8g2.sendBuffer();
        digitalWrite(DISPLAY_CS, HIGH);     
        while (true);                                               // if failed, do nothing
    }

    digitalWrite(LORA_CS, HIGH);
    digitalWrite(DISPLAY_CS, LOW);
    digitalWrite(SD_CS, HIGH);                                       //Select Display SPI Device   

//////////////////////////////////////////////////////////////////////

    Serial.println("LORA init succeeded.");
    loraInit = "LORA init";
    sprintf(buf_loraInit, "%s", loraInit);   
    //u8g2.drawStr(0,35,buf_loraInit);
    //u8g2.sendBuffer();
    delay(300);

    pinMode(gpioP1, INPUT_PULLDOWN);
    pinMode(gpioP2, INPUT_PULLDOWN);
    pinMode(gpioP3, INPUT_PULLDOWN);
    pinMode(gpioP4, INPUT_PULLDOWN);

    Serial.println("Outputs init succeeded.");
    outputInit = "Outputs init";
    sprintf(buf_outputInit, "%s", outputInit);   
    //u8g2.drawStr(0,45,buf_outputInit);
    //u8g2.sendBuffer();
    delay(500);

    printLora(1);
    delay(2500);

    emptyDisplay();
    printDisplay();

    digitalWrite(LORA_CS, HIGH);
    digitalWrite(DISPLAY_CS, HIGH);
    digitalWrite(SD_CS, HIGH);         //Select Display SPI Device   

//////////////////////////////////////////////////////////////////////

    if(!SPIFFS.begin(true)){
        Serial.println("SPIFFS init failed. An Error has occurred while mounting SPIFFS");
        return;
    }
    Serial.println("SPIFFS init succeeded.");

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

    if ((useSTATIC == true) && (useDNS = false)) {
        ETH.config(local_ip, gateway, subnet);
    }

    if ((useSTATIC == true) && (useDNS = true)) {
        ETH.config(local_ip, gateway, subnet, dns1, dns2);
    }

//////////////////////////////////////////////////////////////////////

    if (MDNS.begin("redtally")) {
        Serial.println("MDNS responder started.");
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

    // Route to set GPIO to HIGH
    server.on("/network", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(SPIFFS, "/network.html");
    });
    
    // Route to set GPIO to LOW
    server.on("/info", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(SPIFFS, "/info.html");
    });

    server.begin();
    Serial.println("HTTP server started.");

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
        printDisplay();
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
            printDisplay();
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
            printDisplay();
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
            printDisplay();
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
            printDisplay();
            lastOfferTime = millis();
            lastOfferTimeEnd = millis();
            emptyDisplay();
        }

        if ((millis() - lastOfferTimeRef > 2500)) {      // every 2.5 s clear display
            emptyDisplay();
            printDisplay();
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
  
    // Request Mode
    if ((mode == "request") && (millis() - lastAnalogReadTime > 250)) {

        gpioV1 = analogRead(gpioP1);
        gpioV2 = analogRead(gpioP2);
        gpioV3 = analogRead(gpioP3);
        gpioV4 = analogRead(gpioP4);
        gpioV1Map = map(gpioV1, 400, 4095, 0, 3695);
        gpioV2Map = map(gpioV2, 400, 4095, 0, 3695);
        gpioV3Map = map(gpioV3, 400, 4095, 0, 3695);
        gpioV4Map = map(gpioV4, 400, 4095, 0, 3695); 
        gpioV1Cal = gpioV1Map * (3.3 / 3695.0);
        gpioV2Cal = gpioV2Map * (3.3 / 3695.0);
        gpioV3Cal = gpioV3Map * (3.3 / 3695.0);
        gpioV4Cal = gpioV4Map * (3.3 / 3695.0);

        if (gpioV1Cal > 2.0 && tally_bb == HIGH && gpioC1 == HIGH) {
            destination = 0xbb;
            string_destinationAddress = "bb";
            outgoing = "req-high";         // Send a message
            sendMessage(outgoing);
            printDisplay();
            Serial.println("LORA TxD: " + outgoing);
            gpioC1 = !gpioC1;
            mode = "acknowledge";
            mode_s = "ack";
            lastAckTime = millis();
            emptyDisplay();
        }

        if (gpioV1Cal < 2.0 && tally_bb == HIGH && gpioC1 == LOW) {
            destination = 0xbb;
            string_destinationAddress = "bb";
            outgoing = "req-low";         // Send a message
            sendMessage(outgoing);
            printDisplay();
            Serial.println("LORA TxD: " + outgoing);
            gpioC1 = !gpioC1;
            mode = "acknowledge";
            mode_s = "ack";
            lastAckTime = millis();
            emptyDisplay();
        }

        if (gpioV2Cal > 2.0 && tally_cc == HIGH && gpioC2 == HIGH) {
            destination = 0xcc;
            string_destinationAddress = "cc";
            outgoing = "req-high";         // Send a message
            sendMessage(outgoing);
            printDisplay();
            Serial.println("LORA TxD: " + outgoing);
            gpioC2 = !gpioC2;
            mode = "acknowledge";
            mode_s = "ack";
            lastAckTime = millis();
            emptyDisplay();
        }

        if (gpioV2Cal < 2.0 && tally_cc == HIGH && gpioC2 == LOW) {
            destination = 0xcc;
            string_destinationAddress = "cc";
            outgoing = "req-low";         // Send a message
            sendMessage(outgoing);
            printDisplay();
            Serial.println("LORA TxD: " + outgoing);
            gpioC2 = !gpioC2;
            mode = "acknowledge";
            mode_s = "ack";
            lastAckTime = millis();
            emptyDisplay();
        }

        if (gpioV3Cal > 2.0 && tally_dd == HIGH && gpioC3 == HIGH) {
            destination = 0xdd;
            string_destinationAddress = "dd";
            outgoing = "req-high";         // Send a message
            sendMessage(outgoing);
            printDisplay();
            Serial.println("LORA TxD: " + outgoing);
            gpioC3 = !gpioC3;
            mode = "acknowledge";
            mode_s = "ack";
            lastAckTime = millis();
            emptyDisplay();
        }

        if (gpioV3Cal < 2.0 && tally_dd == HIGH && gpioC3 == LOW) {
            destination = 0xdd;
            string_destinationAddress = "dd";
            outgoing = "req-low";         // Send a message
            sendMessage(outgoing);
            printDisplay();
            Serial.println("LORA TxD: " + outgoing);
            gpioC3 = !gpioC3;
            mode = "acknowledge";
            mode_s = "ack";
            lastAckTime = millis();
            emptyDisplay();
        }

        if (gpioV4Cal > 2.0 && tally_ee == HIGH && gpioC4 == HIGH) {
            destination = 0xee;
            string_destinationAddress = "ee";
            outgoing = "req-high";         // Send a message
            sendMessage(outgoing);
            printDisplay();
            Serial.println("LORA TxD: " + outgoing);
            gpioC4 = !gpioC4;
            mode = "acknowledge";
            mode_s = "ack";
            lastAckTime = millis();
            emptyDisplay();
        }

        if (gpioV4Cal < 2.0 && tally_ee == HIGH && gpioC4 == LOW) {
            destination = 0xee;
            string_destinationAddress = "ee";
            outgoing = "req-low";         // Send a message
            sendMessage(outgoing);
            printDisplay();
            Serial.println("LORA TxD: " + outgoing);
            gpioC4 = !gpioC4;
            mode = "acknowledge";
            mode_s = "ack";
            lastAckTime = millis();
            emptyDisplay();
        }
        lastAnalogReadTime = millis();
    }

    // Acknowledge Mode
    while (mode == "acknowledge") {
        onReceive(LoRa.parsePacket(), &rx_adr, &tx_adr, &incoming, &rssi, &bL, &snr);    // Parse Packets and Read it    
    
        // Back to Request Mode
        if ((incoming == "ack") && ((tx_adr == "bb") || (tx_adr == "cc") || (tx_adr == "dd") ||  (tx_adr == "ee"))) {
            Serial.println("LORA RxD: " + incoming);
            printDisplay();
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
        printDisplay();
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
        printDisplay();
        break;
    }
    }

    // Control Mode BB after discover and 3 - 3.5 minutes or if BB offline, control after 9 minutes
    if (((millis() - lastDiscoverTimebb > 180000) && ((tally_bb == HIGH) || (tally_bb_init == HIGH))) || ((millis() - lastDiscoverTimebb > 540000) && ((tally_bb == LOW) || (tally_bb_init == LOW)))) {
        destination = 0xbb;
        string_destinationAddress = "bb";
        outgoing = "con-rec?";         // Send a message
        sendMessage(outgoing);
        printDisplay();
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
            printDisplay();
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
            printDisplay();
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
        printDisplay();
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
                printDisplay();
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
                printDisplay();
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
        printDisplay();
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
                printDisplay();
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
                printDisplay();
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
        printDisplay();
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
                printDisplay();
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
                printDisplay();
                mode = "request"; 
                mode_s = "req";
                emptyDisplay();
                break;
            }
            }
        }

        // Function Print Display if nothing work
        if (millis() - lastDisplayPrint > 10000) {
            emptyDisplay();
            printDisplay();
        }
  
}

//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////