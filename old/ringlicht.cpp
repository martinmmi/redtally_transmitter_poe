/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// Neopixel WebService ////////////////////////////////////////////
////////////////////////// Development partnership for the MCI AB-Link //////////////////////////////
///////////////////////////// In the year 2021 von Martin Mittrenga /////////////////////////////////
/////////////////////////////////////// and Jan Vollmer /////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////

//#include <EEPROM.h>   // Import of libraries
#include <SPI.h>
#include <Ethernet.h>
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h>
#endif

/////////////////////////////////////////////////////////////////////////////////////////////////////

#define fiveV_PIN 3         // PIN for +5V
#define LED_PIN 4           // PIN for Digital-In
#define LED_COUNT 120       // LED Count
#define DHCP true           // DHCP ON or OFF

/////////////////////////////////////////////////////////////////////////////////////////////////////

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRBW + NEO_KHZ800);       // NEO_GRB for WS2812 and NEO_GRBW for WS6812

/////////////////////////////////////////////////////////////////////////////////////////////////////


IPAddress ip(192, 168, 50, 200);    // Static IP address if DHCP is deactivated

byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};    // MAC-Address

EthernetServer server(80);     // Initialization of Ethernet Server on port 80

/////////////////////////////////////////////////////////////////////////////////////////////////////

/*
   EEPROM Belegungen:
   100 = neopixelState
   101 = brightness
   102 = zaehlerWhiteBalance
*/

/////////////////////////////////////////////////////////////////////////////////////////////////////

String neopixelState = "on";                 // Auxiliary variables for saving the current output status
String header;                               // Temporary memory for HTTP requests

int brightness = 10;                         // Memory for brightness and whitebalance
int zaehlerWhiteBalance = 0;

bool authenticated = false;

bool start = true;

unsigned long lastauthentication = millis();
unsigned long currentTime = millis();        // Current Time
unsigned long previousTime = 0;              // Previous Time

const long timeoutTime = 2000;               // Timeout Time

const char *info = "This surface controls the Neopixel ring light from MCI AB-Link.";     // Text in Help Pop-up
const char *info2 = "Developed in 2020 by Martin Mittrenga (MMG) and Jan Vollmer (MCI).";
const char *hashtag_begin_on = "\n#################";       // Often used strings
const char *hashtag_end_on = "#################\n";
const char *hashtag_begin_off = "\n##################";
const char *hashtag_end_off = "##################\n";
const char *hashtag_begin_wb = "\n###############";
const char *hashtag_end_wb = "###############\n";
const char *hashtag_begin_b = "\n##############";
const char *hashtag_end_b = "##############\n";
const char *gleichlinie = "\n========================================\n";
const char *password = "neopixel";

/////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  //EEPROM.get(100,neopixelState);
  //EEPROM.get(101,brightness);
  //EEPROM.get(102,zaehlerWhiteBalance);
  pinMode(fiveV_PIN, OUTPUT);
  digitalWrite(fiveV_PIN, HIGH);
  Serial.begin(9600);
  while (!Serial) {
    ;
  }
  Serial.println(gleichlinie);
  Serial.println("Neopixel WebService.");

/////////////////////////////////////////////////////////////////////////////////////////////////////

  if (DHCP) {                                         // Start of Ethernet Connection and Server
    Ethernet.begin(mac);                              // With DHCP
  } else {
    Ethernet.begin(mac, ip);                          // With Static IP
  }

/////////////////////////////////////////////////////////////////////////////////////////////////////

  /*

  if (Ethernet.hardwareStatus() == EthernetNoHardware) {            // Check whether the Ethernet shield are connected
    Serial.println("Ethernet Shield was not found. ");
    Serial.println(gleichlinie);
    while (true) {
      delay(1);
    }
  } else if (Ethernet.linkStatus() == LinkOFF) {                    // Check whether the network cable are connected
    Serial.println("No ethernet connection possible. ");
    Serial.println(gleichlinie);
    while (true) {
      delay(1);
    }
  }
  
  */

/////////////////////////////////////////////////////////////////////////////////////////////////////

  server.begin();     //Start of Server
  Serial.print("Server runs with the IP: ");
  Serial.println(Ethernet.localIP());
  Serial.println(gleichlinie);

/////////////////////////////////////////////////////////////////////////////////////////////////////

#if defined(__AVR_ATtiny85__) && (F_CPU == 16000000)   // CPU-Optimization for better addressing of the Neopixel
  clock_prescale_set(clock_div_1);
#endif

/////////////////////////////////////////////////////////////////////////////////////////////////////

  strip.begin();                                                //Initialization of the neopixels
  strip.setBrightness(brightness * 5);                          // Set the initial birghtness
  strip.show();                                                 //All pixel off

  //strip.fill(strip.Color(230, 210, 150, 255), 0, LED_COUNT);
  //strip.show();  

  if (start == true) {
    for(int i=0; i<LED_COUNT; i++) {  // For each pixel in strip...
  
        strip.setPixelColor(i, strip.Color(255, 200, 100, 255));            //  Set pixel's color (in RAM)
        strip.show();                                                       //  Update strip to match
        delay(5);
          
      if (i==strip.numPixels()) {
        start = false;
        break;
      }
    }
  }

}

/////////////////////////////////////////////////////////////////////////////////////////////////////

void loop() {

  EthernetClient client = server.available();         // listen for incoming clients
  if (client) {                                       // If a new client connects,
    Serial.println("New Client.\n");                  // print a message out in the serial port
    String currentLine = "";                          // make a String to hold incoming data from the client
    currentTime = millis();
    previousTime = currentTime;
    while (client.connected() && currentTime - previousTime <= timeoutTime) { // loop while the client's connected
      currentTime = millis();
      if (currentTime - lastauthentication >= 600000) { // Lock Arduino again 10 minutes after last unlock
        authenticated=false;
      }
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        header += c;
        if (c == '\n') {                    // if the byte is a newline character
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println("Refresh: 30");  // refresh the page automatically every 30 sec
            client.println();

/////////////////////////////////////////////////////////////////////////////////////////////////////

            for (int i = -10; i < 21; i++) {                    // Iterate numbers from -10 up to 20
              // Schaltet die GPIOs an und aus
              if (header.indexOf("POST /neopixel/on") >= 0) {
                Serial.println(hashtag_begin_on);
                Serial.println("## Neopixel on ##");
                Serial.println(hashtag_end_on);
                Serial.print("\nWhitebalance: "); Serial.println(zaehlerWhiteBalance);
                
                neopixelState = "on";                           // State set to ON
                //EEPROM.put(100,neopixelState);
                auswahl();                                      // Apply the changes
                break;
              }

/////////////////////////////////////////////////////////////////////////////////////////////////////

             else if (header.indexOf("POST /neopixel/off") >= 0) {
                Serial.println(hashtag_begin_off);
                Serial.println("## Neopixel off ##");
                Serial.println(hashtag_end_off);
                Serial.print("\nWhitebalance: "); Serial.println(zaehlerWhiteBalance);
                neopixelState = "off";                                    // State set to OFF
                //EEPROM.put(100,neopixelState);
                strip.fill(strip.Color(0, 0, 0, 0), 0, LED_COUNT);        // LED strip gets filled with no color (OFF)
                strip.show(); // Refresh LEDs
                break;
              }

/////////////////////////////////////////////////////////////////////////////////////////////////////
              
              else if (header.indexOf("POST /neopixel/wb"+String(i)+" ") >= 0) {

                if ((neopixelState == "on")) {

                  zaehlerWhiteBalance = i;                               // Set new value of whitebalance
                  //EEPROM.put(102,zaehlerWhiteBalance);
                  Serial.println(hashtag_begin_wb);
                  Serial.println("## WB Change ##");
                  Serial.println(hashtag_end_wb);
                  Serial.print("Whitebalance: "); Serial.println(zaehlerWhiteBalance);

                  auswahl();                                                                // Apply change of whitebalance
                  break;
                }
              }

/////////////////////////////////////////////////////////////////////////////////////////////////////
                
              else if (header.indexOf("POST /neopixel/b"+String(i)+" ") >= 0 && neopixelState == "on") {

                brightness = i; //Set brightness value
                
                //EEPROM.put(101,brightness);
                Serial.println(hashtag_begin_b);
                Serial.println("## B Change ##");
                Serial.println(hashtag_end_b);
                
                Serial.print("Brightness: "); Serial.println(brightness);
                
                updateBrightness();                                             // Brightness of the LEDs refreshes
                
                break;
                
              }
              
/////////////////////////////////////////////////////////////////////////////////////////////////////
              
              else if (header.indexOf("POST /neopixel/authenticated") >= 0) {  // Unlock Arduino UI

                  //Serial.println("Authenticated with password. ");
                  //Serial.println("");

                authenticated = true;
                lastauthentication = millis();
                              
              } else if (header.indexOf("POST /neopixel/logout")>= 0) {       // Lock Arduino UI

                  //Serial.println("Logged out. ");
                  //Serial.println("");
                  
                authenticated = false;
              }
            }

/////////////////////////////////////////////////////////////////////////////////////////////////////
            
            // Shows the HTML website
            client.println(F("<!DOCTYPE html><html>"));
            client.println(F("<head><title>Neopixel WebService</title><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">"));
            client.println(F("<link rel=\"icon\" href=\"data:,\">"));

/////////////////////////////////////////////////////////////////////////////////////////////////////

            // CSS to style the on/off buttons
            // Feel free to change the background-color and font-size attributes to fit your preferences
            client.println(F("<style>html { font-family: Helvetica; display: inline-block; margin: auto; text-align: center;}"));
            if (authenticated==false) { // When the user hasn't been authenticated yet.
                    client.println(F(".button {background-color: grey; border: none; color: white; font-size: 12px; padding: 5px 10px; outline: none; border-radius: 6px; text-align: center;margin: 12px 0px;"));                                  // CSS "Login" Button1
                    client.println(F("text-decoration: none; font-size: 20px; margin: 2px; cursor: pointer; border-radius: 6px}"));
                    client.println(F(".button2 {background-color: #5ADD57; border: none; color: white; padding: 6px 20px; outline: none;"));      // CSS "ON" Button1
                    client.println(F("text-decoration: none; font-size: 28px; margin: 2px; cursor: default; border-radius: 5px}"));
                    client.println(F(".button3 {background-color: #77878A; border: none; color: white; padding: 6px 20px; outline: none;"));
                    client.println(F("text-decoration: none; font-size: 28px; margin: 2px; cursor: default; border-radius: 5px}"));               // CSS "OFF" Button
                    client.println(F("</style></head><body><h1>Neopixel WebService"));

                        if (neopixelState == "on") {                                                               // If the state is ON/OFF ...
                          client.print(F("<button class=\"button button2\">ON</button></h1>"));      // ... the ON Button is displayed
                        } else {
                          client.print(F("<button class=\"button button3\">OFF</button></h1>"));     //... the OFF Button is displayed 
                        }
                    
                    client.println(F("<p><b>Password:  </b><input name=\"password\" size=\"5\" maxlength=\"30\" id=\"password\" type=\"password\" value=\"\"></p><p><button id=\"submit\" type=\"button\" autofocus onclick=\"redirect()\" class=\"button button\">Login</button></p><script>")); // Text field and Button
                    client.println(F("var http = new XMLHttpRequest();")); // HTTP Sender/Receiver Object
                    client.println(F("var field = document.getElementById(\"password\");")); // Text field Object   
                    client.println(F("function redirect() {"));  // Function executed when Button is getting pressed
                    client.print(F("if (field.value == \""));
                    client.print(password);
                    client.println(F("\") {")); // If the right password is submitted
                    client.println(F("http.open(\"POST\",\"/neopixel/authenticated\");http.send();setTimeout(function(){location.reload();},3000);} else {alert(\"Password is incorrect.\");}}")); // Message will be sent to Arduino and the site will be reloaded after 3 seconds, else a pop-up will tell that the password was incorrect
                    client.println("</script></body></html>");
            } else {
                    client.println(F(".button {background-color: #5ADD57; border: none; color: white; padding: 6px 20px; outline: none;"));                                  // CSS "ON" Button1
                    client.println(F("text-decoration: none; font-size: 28px; margin: 2px; cursor: pointer; border-radius: 5px}"));
                    client.println(F(".button2 {background-color: #77878A; border: none; color: white; padding: 6px 20px; outline: none;"));
                    client.println(F("text-decoration: none; font-size: 28px; margin: 2px; cursor: pointer; border-radius: 5px}"));       // CSS "OFF" Button
                    client.println(F(".button3 {background-color: grey; color:white; font-size: 12px; padding: 5px 10px; border-radius: 6px; text-align: center;margin: 15px 15px; outline: none;}"));
                    client.println(F(".slidecontainer {width: 100%;}"));                                                                                                                                // With this line the CSS-Properties of the Sliders start
                    client.println(F(".slider {-webkit-appearance: none;width: 65%;height: 10px;background: #d3d3d3;outline: none;opacity: 0.7;-webkit-transition: .2s;transition: opacity .2s;}"));
                    client.println(F(".slider:hover {opacity: 1;}"));
                    client.println(F(".slider::-webkit-slider-thumb {-webkit-appearance: none;appearance: none;width: 30px;height: 35px;background: #4CAF50;cursor: pointer;}"));
                    client.println(F(".slider::-moz-range-thumb {width: 30px;height: 35px;background: #4CAF50;cursor: pointer;}"));
                    client.println(F(".popup {position: relative;display: inline-block;cursor: pointer;-webkit-user-select: none;-moz-user-select: none;-ms-user-select: none;user-select: none;}"));   // CSS Properties of Help Pop-Up
                    client.println(F(".popup .popuptext {visibility: hidden;width: 160px;background-color: #555; font-size: 9px; color: #fff; text-align: center;border-radius: 6px;padding: 5px 2px;position: absolute;z-index: 1;margin: 15px 2px;}"));
                    client.println(F(".popup .show {visibility: visible;}</style>"));
                   
            

/////////////////////////////////////////////////////////////////////////////////////////////////////

            // Web Page Heading
            client.print(F("<body><h1>Neopixel WebService  "));

/////////////////////////////////////////////////////////////////////////////////////////////////////

            // If the neopixelState is ON, it displays the ON button
            if (neopixelState == "on") {                                                               // If the state is ON/OFF ...
              client.print(F("<button  id=\"switch\" class=\"button button\">ON</button></h1>"));      // ... the ON Button is displayed
            } else {
              client.print(F("<button id=\"switch\" class=\"button button2\">OFF</button></h1>"));     //... the OFF Button is displayed 
            }

/////////////////////////////////////////////////////////////////////////////////////////////////////

            // Display current state, and ON/OFF buttons for Neopixel
            client.println();

            // Regulator
            client.print("<h3><p>Brightness: <span id=\"brightness\">");     //Helligkeits-Regler
            client.print(String(brightness));
            client.print(F("</span></p><div class=\"slidecontainer\"><input type=\"range\" min=\"1\" max=\"20\" value=\""));
            client.print(String(brightness));
            client.print("\" class=\"slider\" id=\"rs_brightness\"");
            if (neopixelState == "off") {
              client.print(" disabled");
            }
            client.println("></div></h3>");
            client.print("<h3><p>Whitebalance: <span id=\"whitebalance\">");  // WhiteBalance-Regler
            client.print(String(zaehlerWhiteBalance));
            client.print(F("</span></p><div class=\"slidecontainer\"><input type=\"range\" min=\"-10\" max=\"10\" value=\""));
            client.print(String(zaehlerWhiteBalance));
            client.print("\" class=\"slider\" id=\"rs_wb\"");
            if (neopixelState == "off") {
              client.print(" disabled");
            }

            client.println("></div></h3>");
            
/////////////////////////////////////////////////////////////////////////////////////////////////////

            
            client.println("<button class=\"button button3\" id=\"network\">Network</button>");                //Network-Button
            client.println("<button class=\"button button3\" id=\"logout\">Logout</button>");                 //Logout-Button
            
            client.print(F("<div class=\"popup\" onclick=\"popup()\"><button class=\"button button3\">Help</button><span class=\"popuptext\" id=\"hilfe\">")); // Help-Button
            client.print(info);
            client.print(info2);
            client.println("</span></div>");          //Limits the pop-up field

/////////////////////////////////////////////////////////////////////////////////////////////////////

            // Script that is executed by the browser when used
            client.println(F("<script>var rs_brightness = document.getElementById(\"rs_brightness\");"));         // Brightness-Slider Object
            client.println(F("var onoffbutton = document.getElementById(\"switch\");"));                          // ON/OFF Button
            client.println(F("var logout = document.getElementById(\"logout\");"));
            client.println(F("var network = document.getElementById(\"network\");"));
            client.println(F("var rs_whitebalance = document.getElementById(\"rs_wb\");"));                       // WhiteBalance-Slider Object
            client.println(F("var brightness = document.getElementById(\"brightness\");"));                       // Brightness Span Object
            client.println(F("var whitebalance = document.getElementById(\"whitebalance\");"));                   // WhiteBalance Span Object
            client.println(F("var http = new XMLHttpRequest();"));                                                // HTTP-Sender/Receiver Object
            client.println(F("rs_brightness.value=brightness.innerHTML;rs_whitebalance.value=whitebalance.innerHTML;"));
            client.println(F("onoffbutton.onclick=function(){"));                                                  // ON/OFF Button clickevent
            client.println(F("if (onoffbutton.innerHTML==\"ON\") {"));                                             // If the ON Button is visible,
            client.println(F("http.open(\"POST\",\"/neopixel/off\");http.send();"));                                 // an "OFF" message is sent to the Arduino
            client.println(F("onoffbutton.innerHTML= \"OFF\"; onoffbutton.style.backgroundColor=\"#77878A\";rs_whitebalance.disabled=true;rs_brightness.disabled=true;}"));     // and the Button and the Slider refresh
            client.print(F("else {"));                                                                               // Otherwise the OFF Button must be visible,
            client.println(F("http.open(\"POST\",\"/neopixel/on\");http.send();"));                                  // so that an "ON" message is sent to the Arduino
            client.println(F("onoffbutton.innerHTML= \"ON\"; onoffbutton.style.backgroundColor=\"#5ADD57\";rs_whitebalance.disabled=false;rs_brightness.disabled=false;}}"));      // and the Button and the Slider refresh
            client.println(F(""));
            client.println(F("rs_brightness.oninput=function(){brightness.innerHTML = rs_brightness.value;"));                    // Event on change of Brightness; Displayed value refreshes
            client.println(F("http.open(\"POST\",\"/neopixel/b\"+brightness.innerHTML.toString());http.send();}"));                // Send new brightness value to the Arduino
            client.println(F("rs_whitebalance.oninput=function(){whitebalance.innerHTML = rs_whitebalance.value;"));              // Event on change of Whitebalance-Slider; Displayed value refreshes
            client.println(F("http.open(\"POST\",\"/neopixel/wb\"+whitebalance.innerHTML.toString());http.send();}"));    // Send new whitebalance value to the Arduino
            client.println(F("logout.onclick=function(){http.open(\"POST\",\"/neopixel/logout\");http.send();setTimeout(function(){location.reload();},3000);}")); // Send Logut-Message to Arduino and refesh site after 3 seconds when the logout Button gets clicked
            client.println(F("function popup() { var hilfe = document.getElementById(\"hilfe\"); hilfe.classList.toggle(\"show\");}</script>")); // Help is visible after click on the "Help" Button

/////////////////////////////////////////////////////////////////////////////////////////////////////

            client.println("</body></html>");
            }
            // The HTTP response ends with another blank line
            client.println();
            // Break out of the while loop
            break;
          } else { // if you got a newline, then clear currentLine
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
      }
    }

/////////////////////////////////////////////////////////////////////////////////////////////////////

    // Clear the header variable
    header = "";

/////////////////////////////////////////////////////////////////////////////////////////////////////

    //cclose the connection:
    client.stop();
    Serial.println("Client disconnected.");
    Serial.println(gleichlinie);
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////

void updateBrightness() {
  if (1 < brightness < 20) {
    strip.setBrightness(brightness * 5);                     // Set new brightness 
    strip.show();                                            // Refresh LEDs
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////

void auswahl() {


  if (zaehlerWhiteBalance < 0) {													// If whitebalance < 0

    strip.fill(strip.Color(230, 210, 150 - 10 * zaehlerWhiteBalance, 255), 0, LED_COUNT);

  } else {																	            	// If whitebalance >= 0
    strip.fill(strip.Color(230, 210, 150 - 10 * zaehlerWhiteBalance, 255), 0, LED_COUNT);
  }

  strip.show();

}

/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////