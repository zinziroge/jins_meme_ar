/*
  Verify that your Qwiic Transparent Grahical OLED is connected correctly and working.

  By: Owen Lyke
  SparkFun Electronics
  Date: February 26, 2019
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.
  Feel like supporting open source hardware?
  Buy a board from SparkFun! https://www.sparkfun.com/products/15173



  The graphics library is like a 3-layer cake. Here they are from top-down
  https://github.com/sparkfun/SparkFun_HyperDisplay
  https://github.com/sparkfun/HyperDisplay_SSD1309_ArduinoLibrary
  https://github.com/sparkfun/HyperDisplay_UG2856KLBAG01_ArduinoLibrary

  Hardware Compatibility
    - The IO pins on this board are designed for use with 3.3V so if you are using a 5V microcontroller
      please use a level shifter. Note: Qwiic connectors on SparkFun dev boards are already at 3.3V
    - This display relies on a copy of graphics data in your microcontroller, a total of 1024 bytes.
      That is half the RAM available on an Uno so it is easy to run into sinister low-memory related
      bugs. We reccomend using a micro with more memory like a SAMD21, Esp32, Teensy, etc.

  Hardware Connections:
  Option 1 (I2C):
    Connect using a Qwiic jumper if you have a Qwiic compatible board and you plan to use I2C

  Option 2 (SPI):
    Connect SCLK and MOSI to the SPI port of your choice (13 and 11 for SPI on Uno-like boards)
    Also connect D/C and CS to two unused GPIO pins of your choice (and set the proper pin definitions below)
    Don't forget power - connect 3.3V and GND
*/

#include <Arduino.h>
//#include <M5StickC.h>
#include <M5StickCPlus.h>

// at C:\Users\xxx\AppData\Local\Arduino15\packages\esp32\hardware\esp32\1.0.5\libraries\WiFi
#include <WiFi.h>
#include <WiFiMulti.h>
#include <WiFiClientSecure.h>
#include <WebSocketsServer.h>
/* https://rikoubou.hatenablog.com/entry/2021/04/16/162258 */
#include <ArduinoJson.h>

// Your library can be installed here: http://librarymanager/All#SparkFun_Transparent_Graphical_OLED
// The rest of the Layer Cake:         http://librarymanager/All#SparkFun_HyperDisplay_SSD1309
//                                     http://librarymanager/All#SparkFun_HyperDisplay
#include "HyperDisplay_UG2856KLBAG01.h" 

#include <ssid_define.h>

//////////////////////////
//      User Setup      //
//////////////////////////
#define SERIAL_PORT Serial
#define WIRE_PORT Wire // Used if USE_SPI == 0
#define SPI_PORT SPI // Used if USE_SPI == 1

#define RES_PIN 2 // Optional
#define CS_PIN 4 // Used only if USE_SPI == 1
#define DC_PIN 5 // Used only if USE_SPI == 1

#define USE_SPI 0 // Choose your interface. 0 = I2C, 1 = SPI

// define at <ssid_define.h>
#define SSID_AP        HOME_SSID_AP
#define SSID_PASSWORD  HOME_SSID_PASSWORD

#define MIN_INTERVAL_MS     1000        // 3sec
// END USER SETUP

typedef struct {
    // invalid
    uint16_t    blinkSpeed; // ????????????	Int	Number	?????????????????????????????????(mSec)	0-400(??????90???180??????)
    uint16_t    blinkStrength; // ????????????	Int	Number	??????????????????(uV-equiv)	0-1000(??????30???150??????)
    uint16_t    eyeMoveUp; // ????????????	Int	Number	??????????????????????????????????????????	0: ???????????? 1: ??????-7: ??????
    uint8_t     eyeMoveDown; // ????????????	Int	Number	??????????????????????????????????????????	0: ???????????? 1: ??????-7: ??????
    uint8_t     eyeMoveLeft; // ????????????	Int	Number	??????????????????????????????????????????	0: ???????????? 1: ??????-7: ??????
    uint8_t     eyeMoveRight; // ????????????	Int	Number	??????????????????????????????????????????	0: ???????????? 1: ??????-7: ??????
    float       roll; //	Float	Number	?????????????????????????????????????????????????????????	-180.00 - 180.00
    float       pitch; //	Float	Number	?????????????????????????????????????????????????????????	-180.00 - 180.00
    float       yaw; //	Float	Number	???????????????????????????????????????????????????	0.00 - 360.00
    // valid
    float       accX; //	Float	Number	????????????X????????????????????????1G=16	-128(-8G) - 127(7.9375G)
    float       accY; //	Float	Number	????????????Y????????????????????????1G=16	-128(-8G) - 127(7.9375G)
    float       accZ; //	Float	Number	????????????Z????????????????????????1G=16	-128(-8G) - 127(7.9375G)
    // invalid ?
    int8_t      walking; //(isWalking) ????????????	boolean	Number	???????????????????????????????????????????????????(?????????0.15~0.25s???????????????)	0/false: ???????????? 1/true: ????????????
    // valid
    int8_t      noiseStatus; //	boolean	Number	???????????????????????????????????????????????????	0/false: ??????????????? 1/true: ???????????????
    int8_t      fitError; //	Int	Number	JINS MEME?????????????????????????????????????????????????????????5??????1?????????	0: ????????? 1: ?????????
    // valid
    int8_t      powerLeft; //	Int	Number	??????????????????????????????	0: ????????? 1: ???-5: ?????????
    uint8_t     sequenceNumber; //(seqNo)	Int	Number	0-255???????????????????????????	0-255
} currentData_T;

typedef struct {
    // valid
    char        date[20];	//date	String	????????????	2000-01-01T00:00:00 - 2099-12-31T23:59:59
    uint8_t     stepCount;  //????????????	stp	Number	??????	0-255
    // invalid ?  
    float       stepCadence;    //????????????;  //	cad	Number(float)	???????????????(?????????)	0-255
    // valid
    uint8_t     isStill;    //	isl	Boolean	????????????????????????????????????	true: ????????????????????? false: ????????????????????????
    float       noiseTime;    //	nis_time	Number(float)	???????????????	0.00 - 15.00
    // valid ?
    uint8_t     isValid;    //	vld	Boolean	?????????????????????????????????????????????3?????????????????????5????????????	true: ?????? false: ??????
    // valid
    float       yMean;    //	tl_yav	Number(float)	????????????Y (???)	-180.00-180.00
    float       ySD;    //	tl_ysd	umber(float)	??????????????????Y (???)	0-655.36
    float       xMean;    //	tl_xav	Number(float)	????????????X (???)	-180.00-180.00
    float       xSD;    //	tl_xsd	Number(float)	??????????????????X (???)	0-655.36
    // invalid
    uint8_t     pitchOnewayCount;    //	hm_po	umber	??????????????????	0-255
    uint8_t     pitchRoundCount;    //	hm_pr	Number	???????????????????????????????????????	0-255
    uint8_t     yawOnewayCount;    //	hm_yo	Number	??????????????????	0-255
    uint8_t     yawRoundCount;    //	hm_yr	Number	???????????????????????????????????????	0-255
    float       xRightStepAmplitude;    //????????????	sa_xr	Number(float)	????????????X???cm,?????????	0.00-16.00
    float       xLeftStepAmplitude;    //????????????	sa_xl	Number(float)	????????????X???cm,?????????	0.00-16.00
    float       yRightStepAmplitude;    //????????????	sa_yr	Number(float)	????????????Y???cm,?????????	0.00-16.00
    float       yLeftStepAmplitude;    //????????????	sa_yl	Number(float)	????????????Y???cm,?????????	0.00-16.00
    float       zRightStepAmplitude;    //????????????	sa_zr	Number(float)	????????????Z???cm,?????????	0.00-16.00
    float       zLeftStepAmplitude;    //????????????	sa_zl	Number(float)	????????????Z???cm,?????????	0.00-16.00
     float       zRightStepAmplitudeCal;    //????????????	sa_zrc	Number(float)	????????????Z????????????cm,?????????	0.00-20.00
    float       zLeftStepAmplitudeCal;    //????????????	sa_zlc	Number(float)	????????????Z????????????cm,?????????	0.00-20.00
    float       maxRightStepAcceleration;    //????????????	st_r	Number(float)	???????????????????????? (G, ??????)	0.00-8.00
    float       maxLeftStepAcceleration;    //????????????	st_l	Number(float)	???????????????????????? (G, ??????)	0.00-8.00
    // valid
    float       sleepScoreStandard;    //????????????	sc_slp_std	Number(float)	??????????????????(?????????)????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????	?????????: 0(????????????)-100(??????)?????????: -1
    float       sleepScore;    //????????????	sc_slp	Number(float)	??????????????????(?????????)?????????????????????(???????????????????????????????????????)???????????????????????????????????????????????????	?????????: 0(????????????)-100(??????)?????????: -1
    float       focusScore;    //????????????	sc_fcs	Number(float)	??????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????	?????????: 0(??????????????????)-100(??????????????????)?????????: -1
    float       tensionScore;    //????????????	sc_tsn	Number(float)	?????????????????????????????????????????????????????????????????????????????????????????????????????????????????????	?????????: 0(???????????????)-100(???????????????)?????????: -1
    float       calmScore;    //????????????	sc_clm	Number(float)	??????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????	?????????: 0(????????????????????????)-100(?????????????????????)?????????: -1
    // none
    float       distance;    //	distance	Number(float)	???????????????????????? ??????????????????????????????(m)	0-5000
    float       latitude;    //	lat	Number(float)	???????????????????????? ??????	-180 - 180
    float       longitude;    //	lng	Number(float)	???????????????????????? ??????	-90 - 90
    uint8_t     appMeasurementStatus;    //	app_measurement_status	Number	???????????????????????????????????????	0: ???APP?????? 2: Run????????? 3: Run??????????????? 8: Drive????????? 12: Drive??????????????? 32: Focus????????? 48: Focus???????????????
    // valid
    float       nptMean;    //????????????	npt_av	Number(float)	NPT????????????????????????????????????	-0.999 - 0.999
    float       nptMedian;    //????????????	npt_med	Number(float)	NPT???????????????????????????????????????	-0.999 - 0.999
    float       nptSD;    //????????????	npt_sd	Number(float)	NPT????????????	0-0.999
    float       blinkWidthMean;    //????????????	bkw_av	Number(float)	????????????????????????(mSec)	0-300
    float       blinkStrengthTotal;    //????????????	bkh_sum	Number(float)	????????????????????????(uV-equiv)	0-10000.0
    float       blinkStrengthMax;    //????????????	bkh_max	Number(float)	????????????????????????(uV-equiv)	0-1000.0
    float       blinkStrengthSD;    //????????????	bkh_sd	Number(float)	??????????????????????????????(uV-equiv)	0.00-1000.0
    float       blinkStrengthMean;    //????????????	bkh_av	Number(float)	????????????????????????	0-1000.0
    float       blinkIntervalTotal;    //????????????	bki_sum	Number(float)	????????????????????????(s)	0-120.0
    uint8_t     blinkIntervalCount;    //????????????	bki_n	Number	?????????????????????	0-120
    float       blinkIntervalMean;    //????????????	bki_av	Number(float)	????????????????????????	0.00-60.00
    uint8_t     blinkCount;    //????????????	bk_n	Number	??????????????????	0-120
    uint8_t     blinkCountRaw;    //????????????	rbk_n	Number	????????????????????????	0-255
    uint8_t     eyeMoveUpCount;    //????????????	re_u	Number	???????????????????????????	0-255
    uint8_t     eyeMoveDownCount;    //????????????	re_d	Number	???????????????????????????	0-255
    uint8_t     eyeMoveRightCount;    //????????????	re_r	Number	???????????????????????????	0-255
    uint8_t     eyeMoveLeftCount;    //????????????	re_l	Number	???????????????????????????	0-255
    // none
    uint32_t    cummulativeTime;    //	cum_time	Number	?????????????????????(s)	0-4294967296
    float       blinkIntervalMeanWA;    //????????????	bki_av_wa	Number(float)	???????????????????????? ????????????	0.00-60.00
    float       blinkStrengtnSDWA;    //????????????	bkh_sd_wa	Number(float)	?????????????????????????????? ????????????	0.00-1000.0
    float       blinkStrengthMeanWA;    //????????????	bkh_av_wa	Number(float)	???????????????????????? ????????????	0-1000.0
    float       nptMeanWA;    //????????????	npt_av_wa	Number(float)	NPT???????????????????????????????????? ????????????	-0.999 - 0.999
    float       nptSDWA;    //????????????	npt_sd_wa	Number(float)	NPT?????????????????????????????????????????? ????????????	0-0.999
    float       blinkWidthMeanWA;    //????????????	bkw_av_wa	Number(float)	???????????????????????? ????????????	0-300
    float       nptScore;    //????????????	sc_npt	Number(float)	?????????????????? NPT?????????	?????????: 0-100?????????: -1
    float       btsScore;    //????????????	sc_bts	Number(float)	?????????????????? BTS?????????	?????????: 0-100?????????: -1
    float       lbsScore;    //????????????	sc_lbs	Number(float)	?????????????????? LBS?????????	?????????: 0-100?????????: -1
    int8_t      legacyZone;    //????????????	zone	Number	RT??????????????????????????? zone???	?????????: 0-100?????????: -1
    int8_t      legacyFocus;    //????????????	focus	Number	RT??????????????????????????? focus???	?????????: 0-100?????????: -1
    int8_t      legacyCalm;    //????????????	calm	Number	RT??????????????????????????? calm???	?????????: 0-100?????????: -1
    int8_t      legacyPosture;    //????????????	posture	Number	RT??????????????????????????? posture???	?????????: 0-100?????????: -1
} logicIndexData_T;

// wifi
WiFiMulti WiFiMulti;
WebSocketsServer webSocket_c = WebSocketsServer(8080);
WebSocketsServer webSocket_l = WebSocketsServer(8081);

IPAddress ip(192, 168, 0, 20); //ip
IPAddress gateway(192, 168, 0, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress DNS(192, 168, 0, 1);

StaticJsonDocument<1024> json_buf;  // ??????????????????(??????)

currentData_T       jins_meme_currentData;
logicIndexData_T    jins_meme_logicIndexData;

// Object Declaration. A class exists for each interface option
#if USE_SPI
UG2856KLBAG01_SPI myTOLED; // Declare a SPI-based Transparent OLED object called myTOLED
#else
UG2856KLBAG01_I2C myTOLED; // Declare a I2C-based Transparent OLED object called myTOLED
#endif /* USE_SPI */

//
uint32_t m5stick_lcd_cur_y;
char m5stick_lcd_buf[8*24];
DeserializationError g_err_json;
static uint32_t g_c_cnt=0;
static uint32_t g_cnt_l=0;
char g_buf[64];

//////////////////////////////////////////////////////////////////////////////
void print_both(const char* s) {
    USE_SERIAL.println(s);

    int16_t y = M5.Lcd.getCursorY();
    if(240 - y < 40) {
        y = 0;
        M5.Lcd.fillScreen(BLACK);
        M5.Lcd.setCursor(0, y);
    } else {
        //M5.Lcd.setCursor(0, y);
        M5.Lcd.print("\n");
    }
    M5.Lcd.print(s);
}

void setup_uart(const int baudrate) {
    // USE_SERIAL.begin(921600);
    USE_SERIAL.begin(baudrate);

    //Serial.setDebugOutput(true);
    USE_SERIAL.setDebugOutput(true);

    //USE_SERIAL.begin(9600);
    //USE_SERIAL.println(F("Example1_DisplayTest: Transparent Graphical OLED"));
}

void setup_wifi() {
    USE_SERIAL.println();
    USE_SERIAL.println();
    USE_SERIAL.println();

    for(uint8_t t = 4; t > 0; t--) {
        USE_SERIAL.printf("[SETUP] BOOT WAIT %d...\n", t);
        USE_SERIAL.flush();
        delay(1000);
    }

    WiFi.config(ip, gateway, subnet, DNS); //static_ip
    delay(100);
    WiFiMulti.addAP(SSID_AP, SSID_PASSWORD);

    while(WiFiMulti.run() != WL_CONNECTED) {
        delay(100);
        USE_SERIAL.println("connect to WiFi ...");
    }
    IPAddress ip = WiFi.localIP();

    int16_t y = M5.Lcd.getCursorY();
    M5.Lcd.setCursor(0, y);
    M5.Lcd.print("\n");
    M5.Lcd.print("IP Adr: ");
    M5.Lcd.print(ip);
    Serial.print("IP Adr: ");
    Serial.println(ip);

    print_both("WiFi Ready");
    /*
    USE_SERIAL.println("WiFi Ready !");

    M5.Lcd.setCursor(0, M5.Lcd.getCursorY());
    M5.Lcd.print("\n");
    M5.Lcd.print("WiFi Ready !");
    */
}

void setup_OLED() {
#if USE_SPI
    SPI_PORT.begin();
    myTOLED.begin(CS_PIN, DC_PIN, SPI_PORT); // Begin for SPI requires that you provide the CS and DC pin numbers
#else
    WIRE_PORT.begin(32, 33); // 2pin:SDA(White), 1pin:SCL(Yellow)
    myTOLED.begin(WIRE_PORT, false, SSD1309_ARD_UNUSED_PIN); // Begin for I2C has default values for every argument
    Wire.setClock(400000);
#endif /* USSE_SPI */
}

void i2c_scanner()
{
    byte error, address; // variable for error and I2C address
    int nDevices;

    Serial.println("Scanning...");

    nDevices = 0;
    for (address = 1; address < 127; address++) {
        // The i2c_scanner uses the return value of
        // the Write.endTransmisstion to see if
        // a device did acknowledge to the address.
        Wire.beginTransmission(address);
        error = Wire.endTransmission();

        if (error == 0) {
            Serial.print("I2C device found at address 0x");
            if (address < 16)
                Serial.print("0");
            Serial.print(address, HEX);
            Serial.println("  !");
            nDevices++;
        } else if (error == 4) {
            Serial.print("Unknown error at address 0x");
            if (address < 16)
                Serial.print("0");
            Serial.println(address, HEX);
        }
    }
    if (nDevices == 0) {
        //Serial.println("No I2C devices found\n");
        print_both("No I2C devices found");
    } else {
        //Serial.println("I2C scan is done\n");
        print_both("I2C scan is done");
    }

    delay(5000); // wait 5 seconds for the next I2C scan
}

void setup()
{
    setup_uart(115200);

    // Initialize the M5StickC object
    M5.begin();
    // LCD display
    M5.Lcd.setCursor(0, 0, 4);
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setTextColor(WHITE);
    M5.Lcd.print("jins meme logger");

    setup_wifi();
    setup_OLED();

// Don't show the logo on boards with small memory
#if !defined(__AVR_ATmega328P__) && !defined(__AVR_ATmega168__)
    //showLogo(); // The showLogo function is a hacky way to get a large bitmap into program space without using <avr/pgspace.h>
#endif

    webSocket_c.begin();
    webSocket_l.begin();
    webSocket_c.onEvent(webSocket_c_Event);
    webSocket_l.onEvent(webSocket_l_Event);

    i2c_scanner();
}

//////////////////////////////////////////////////////////////////////////////
void loop()
{
    /*
    static bool inverted = false;
    if(inverted){
        // Black Text Blue Background
        myTOLED.windowSet();
        myTOLED.setWindowColorClear();
    }else{
        // Blue Text Black Background
        myTOLED.windowClear();
        myTOLED.setWindowColorSet();
    }
    myTOLED.setTextCursor(0,0);
    myTOLED.print("Hello world!!");
    delay(3000);
    inverted = !inverted;

    M5.update();    //You need to add M5.update () to read the status of the keystroke. For details, please see System. 
    if (M5.BtnA.isPressed()) {    //If the key is pressed. 
        which_log = (which_log==0) ? 1 : 0;
        myTOLED.clearDisplay();
    }
    */

    static uint32_t prev_draw_t;
    static uint32_t prev_loop_t;
    uint32_t cur_t = millis();
    uint32_t dt_draw = cur_t - prev_draw_t;
    uint32_t dt_loop = cur_t - prev_loop_t;

    // flush
    //do {
    //    webSocket.loop();
    //    Serial.println(g_err_json.c_str());
    //} while(g_err_json == DeserializationError::Ok);
    int i;
    for(i=0; i < 100; i++) {
        webSocket_c.loop();
    }
    sprintf(g_buf, "%3d,%2.1f,%2.1f", 
        g_c_cnt, 
        dt_loop/1000.0,
        dt_draw/1000.0);
    print_both(g_buf);
    g_c_cnt = 0;
    webSocket_l.loop();

    //// @note because it has much time to draw.
    if(dt_draw > MIN_INTERVAL_MS) {
        draw_logicIndexData_2();
        draw_currentData_2();
        prev_draw_t = cur_t;
    }

    prev_loop_t = cur_t;
}

void print_logicIndexData() {
    //deserializeJson(json_buf, json);       // json???????????????
    jins_meme_logicIndexData.calmScore = json_buf["calmScore"]; // ??????????????????
    USE_SERIAL.printf("calmScore: ");
    USE_SERIAL.printf("%f\n", jins_meme_logicIndexData.calmScore);
}

void hexdump(const void *mem, uint32_t len, uint8_t cols = 16) {
	const uint8_t* src = (const uint8_t*) mem;
	USE_SERIAL.printf("\n[HEXDUMP] Address: 0x%08X len: 0x%X (%d)", (ptrdiff_t)src, len, len);
	for(uint32_t i = 0; i < len; i++) {
		if(i % cols == 0) {
			USE_SERIAL.printf("\n[0x%08X] 0x%08X: ", (ptrdiff_t)src, i);
		}
		USE_SERIAL.printf("%02X ", *src);
		src++;
	}
	USE_SERIAL.printf("\n");
}

void webSocket_c_Event(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
    float f;
    char buf[32];
    //g_err_json = DeserializationError::InvalidInput;
    //USE_SERIAL.printf("c_type: %d\n", type);

    switch(type) {
        case WStype_DISCONNECTED:
            USE_SERIAL.printf("[%u] Disconnected!\n", num);
                print_both("disconn_c");
            break;
        case WStype_CONNECTED:
            {
                IPAddress ip = webSocket_c.remoteIP(num);
                USE_SERIAL.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);
                print_both("conn_c");

				// send message to client
				webSocket_c.sendTXT(num, "Connected");
            }
            break;
        case WStype_TEXT:
            //USE_SERIAL.printf("c::[%u] get Text: %s\n", num, payload);
            g_err_json = deserializeJson(json_buf, payload);
            //USE_SERIAL.printf("c:%s\n", g_err_json.c_str());
            if(g_err_json != DeserializationError::Ok) {
                break;
            }
            read_currentData();
            g_c_cnt += 1;

            // send message to client
            // webSocket.sendTXT(num, "message here");

            // send data to all connected clients
            // webSocket.broadcastTXT("message here");
            break;
        case WStype_BIN:
            USE_SERIAL.printf("[%u] get binary length: %u\n", num, length);
            hexdump(payload, length);

            // send message to client
            // webSocket.sendBIN(num, payload, length);
            break;
		case WStype_ERROR:
		case WStype_FRAGMENT_TEXT_START:
		case WStype_FRAGMENT_BIN_START:
		case WStype_FRAGMENT:
		case WStype_FRAGMENT_FIN:
			break;
    }

}

void webSocket_l_Event(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
    float f;
    char buf[32];
    USE_SERIAL.printf("l_type: %d\n", type);

    switch(type) {
        case WStype_DISCONNECTED:
            USE_SERIAL.printf("[%u] Disconnected!\n", num);
                print_both("disconn_l");
            break;
        case WStype_CONNECTED:
            {
                IPAddress ip = webSocket_l.remoteIP(num);
                USE_SERIAL.printf("l::[%u] Connected from %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);
                print_both("conn_l");
				// send message to client
				webSocket_l.sendTXT(num, "Connected");
            }
            break;
        case WStype_TEXT:
            USE_SERIAL.printf("[%u] get Text: %s\n", num, payload);
            g_err_json = deserializeJson(json_buf, payload);
            USE_SERIAL.printf("l:%s\n", g_err_json.c_str());
            if(g_err_json != DeserializationError::Ok) {
                break;
            }
            read_logicIndexData();
            g_cnt_l = (g_cnt_l + 1)%256;

            // send message to client
            // webSocket.sendTXT(num, "message here");

            // send data to all connected clients
            // webSocket.broadcastTXT("message here");
            break;
        case WStype_BIN:
            USE_SERIAL.printf("[%u] get binary length: %u\n", num, length);
            hexdump(payload, length);

            // send message to client
            // webSocket.sendBIN(num, payload, length);
            break;
		case WStype_ERROR:
		case WStype_FRAGMENT_TEXT_START:
		case WStype_FRAGMENT_BIN_START:
		case WStype_FRAGMENT:
		case WStype_FRAGMENT_FIN:
			break;
    }

}

float flimit(const float v, const float _min, const float _max) {
    if(v < _min) {
        return _min;
    } else if(v > _max) {
        return _max;
    } else {
        return v;
    }
}

void read_currentData() {
    jins_meme_currentData.sequenceNumber = json_buf["sequenceNumber"];
    jins_meme_currentData.accX = json_buf["accX"];
    jins_meme_currentData.accY = json_buf["accY"];
    jins_meme_currentData.accZ = json_buf["accZ"];
}

void read_logicIndexData() {
    jins_meme_logicIndexData.sleepScoreStandard = json_buf["sleepScoreStandard"];
    jins_meme_logicIndexData.focusScore = json_buf["focusScore"];
    jins_meme_logicIndexData.tensionScore = json_buf["tensionScore"];
    jins_meme_logicIndexData.calmScore = json_buf["calmScore"];
}

void draw_currentData_2() {
    // valid
    // float       accX; //	Float	Number	????????????X????????????????????????1G=16	-128(-8G) - 127(7.9375G)
    // float       accY; //	Float	Number	????????????Y????????????????????????1G=16	-128(-8G) - 127(7.9375G)
    // float       accZ; //	Float	Number	????????????Z????????????????????????1G=16	-128(-8G) - 127(7.9375G)
    const int step_y = 12;
    char buf[32];
    char *param_short[] = {
        "sn",
        "aX",
        "aY",
        "aZ",
    };
    static float prev_v[4] = {0};
    float cur_v[4] = {
        (float)jins_meme_currentData.sequenceNumber,
        jins_meme_currentData.accX,
        jins_meme_currentData.accY,
        jins_meme_currentData.accZ,
    };
    int y = 0;

    /// param name
    myTOLED.rectangleClear(107, y, 128, y + step_y, true);
    myTOLED.setTextCursor(107, 0);
    sprintf(buf, "%3.0f", cur_v[0]);
    myTOLED.print(buf);
    y += step_y;

    int i;
    for(i=1; i < 4; i++) {
        float v = flimit(cur_v[i] / 128.0 * 8.0 * 25.0/2.0, -25.0, 25.0); // max 4g

        // clear
        myTOLED.setTextCursor(0, y);
        myTOLED.rectangleClear(
            89, y,
            89 + prev_v[i], y + step_y - 2,
            false);

        // set
        myTOLED.setTextCursor(114, y);
        sprintf(buf, "%s", param_short[i]);
        myTOLED.print(buf);
        myTOLED.rectangleSet(
            89, y,
            89 + v, y + step_y - 2,
            false);

        //USE_SERIAL.printf("%s: %f\n", param_short[i], v);
        y += step_y;
        prev_v[i] = v;
    }
    myTOLED.lineSet(89, 12, 89, 48, 1);
}

void draw_logicIndexData_2() {
    // valid
    // float       sleepScoreStandard;    //????????????	sc_slp_std	Number(float)	??????????????????(?????????)????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????	?????????: 0(????????????)-100(??????)?????????: -1
    // float       sleepScore;    //????????????	sc_slp	Number(float)	??????????????????(?????????)?????????????????????(???????????????????????????????????????)???????????????????????????????????????????????????	?????????: 0(????????????)-100(??????)?????????: -1
    // float       focusScore;    //????????????	sc_fcs	Number(float)	??????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????	?????????: 0(??????????????????)-100(??????????????????)?????????: -1
    // float       tensionScore;    //????????????	sc_tsn	Number(float)	?????????????????????????????????????????????????????????????????????????????????????????????????????????????????????	?????????: 0(???????????????)-100(???????????????)?????????: -1
    // float       calmScore;    //????????????	sc_clm	Number(float)	??????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????	?????????: 0(????????????????????????)-100(?????????????????????)?????????: -1
    const int step_y = 12;
    char buf[32];
    char *param_short[] = {
        "sleep",
        "focus",
        "tens ",
        "calm ",
    };
    static float prev_v[4] = {0};
    float cur_v[4] = {
        jins_meme_logicIndexData.sleepScoreStandard,
        jins_meme_logicIndexData.focusScore,
        jins_meme_logicIndexData.tensionScore,
        jins_meme_logicIndexData.calmScore,
    };
    int y = 0;

    /// param name
    myTOLED.rectangleClear(66, 0, 106, step_y-2, true);
    myTOLED.setTextCursor(66, 0);
    sprintf(buf, "%3d", g_cnt_l);
    myTOLED.print(buf);
    myTOLED.setTextCursor(0, 0);

    int i;
    for(i=0; i < 4; i++) {
        float v = cur_v[i] * 0.34; // 0<=cur_v[i]<=100

        // clear
        myTOLED.rectangleClear(
            30, y,
            30 + prev_v[i], y + step_y - 2,
            false);

        // set
        myTOLED.setTextCursor(0, y);
        sprintf(buf, "%s", param_short[i]);
        myTOLED.print(buf);
        myTOLED.rectangleSet(
            30, y,
            30 + v, y + step_y - 2,
            false);

        //USE_SERIAL.printf("%s: %f\n", param_short[i], v);
        y += step_y;
        prev_v[i] = v;
    }
    myTOLED.lineSet(64, 0, 64, 48, 1);
}

void draw_currentData() {
    // valid
    // float       accX; //	Float	Number	????????????X????????????????????????1G=16	-128(-8G) - 127(7.9375G)
    // float       accY; //	Float	Number	????????????Y????????????????????????1G=16	-128(-8G) - 127(7.9375G)
    // float       accZ; //	Float	Number	????????????Z????????????????????????1G=16	-128(-8G) - 127(7.9375G)
    const int step_y = 12;
    int y = 0;
    char buf[32];
    char *param[] = {
        "fitError",
        "accX",
        "accY",
        "accZ",
    };
    char *param_short[] = {
        "fit",
        "accX",
        "accY",
        "accZ",
    };
    static float prev_v[3] = {0};

    //myTOLED.clearDisplay();

    myTOLED.setTextCursor(0, y);
    y += step_y;

    int i;
    for(i=0; i < 3; i++) {
        float v = json_buf[param[i]]; // ??????????????????
        v = flimit(v / 128.0 * 80, -40.0, 40.0);

        // clear
        myTOLED.setTextCursor(0, y);
        myTOLED.rectangleClear(
            28 + 100/2, y + 2,
            28 + 100/2 + prev_v[i], y + step_y - 2,
            false);

        // set
        //sprintf(buf, "%s: %3.0f/100", param_short[i], v);
        myTOLED.setTextCursor(0, y);
        sprintf(buf, "%s", param_short[i]);
        myTOLED.print(buf);
        //myTOLED.setTextCursor(0, y);
        myTOLED.setTextCursor(0, y);
        myTOLED.lineSet(28 + v, y + 2, 28 + v, y + step_y - 2, 1);
        myTOLED.rectangleSet(
            28 + 100/2, y + 2,
            28 + 100/2 + v, y + step_y - 2,
            false);

        USE_SERIAL.printf("%s: %f\n", param[i], v);
        y += step_y;
        prev_v[i] = v;
    }
}

void draw_logicIndexData() {
    // valid
    // float       sleepScoreStandard;    //????????????	sc_slp_std	Number(float)	??????????????????(?????????)????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????	?????????: 0(????????????)-100(??????)?????????: -1
    // float       sleepScore;    //????????????	sc_slp	Number(float)	??????????????????(?????????)?????????????????????(???????????????????????????????????????)???????????????????????????????????????????????????	?????????: 0(????????????)-100(??????)?????????: -1
    // float       focusScore;    //????????????	sc_fcs	Number(float)	??????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????	?????????: 0(??????????????????)-100(??????????????????)?????????: -1
    // float       tensionScore;    //????????????	sc_tsn	Number(float)	?????????????????????????????????????????????????????????????????????????????????????????????????????????????????????	?????????: 0(???????????????)-100(???????????????)?????????: -1
    // float       calmScore;    //????????????	sc_clm	Number(float)	??????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????	?????????: 0(????????????????????????)-100(?????????????????????)?????????: -1
    const int step_y = 12;
    int y = 0;
    char buf[32];
    char *param[] = {
        "sleepScoreStandard",
        "focusScore",
        "tensionScore",
        "calmScore"
    };
    char *param_short[] = {
        "sleep",
        "focus",
        "tens ",
        "calm "
    };
    static float prev_v[4] = {0};

    //myTOLED.clearDisplay();

    myTOLED.setTextCursor(0, y);
    y += step_y;
    //bool vb = json_buf["isStill"];
    //sprintf(buf, "still: %d", vb);
    //myTOLED.print(buf);

    int i;
    for(i=0; i < 4; i++) {
        float v = json_buf[param[i]]; // ??????????????????

        // clear
        myTOLED.setTextCursor(0, y);
        myTOLED.rectangleClear(
            28, y + 2,
            28 + prev_v[i], y + step_y - 2,
            false);

        // set
        //sprintf(buf, "%s: %3.0f/100", param_short[i], v);
        myTOLED.setTextCursor(0, y);
        sprintf(buf, "%s", param_short[i]);
        myTOLED.print(buf);
        //myTOLED.setTextCursor(0, y);
        myTOLED.setTextCursor(0, y);
        myTOLED.lineSet(28 + v, y + 2, 28 + v, y + step_y - 2, 1);
        myTOLED.rectangleSet(
            28, y + 2,
            28 + v, y + step_y - 2,
            false);

        USE_SERIAL.printf("%s: %f\n", param_short[i], v);
        y += step_y;
        prev_v[i] = v;
    }
}

#if !defined(__AVR_ATmega328P__) && !defined(__AVR_ATmega168__)
void showLogo(void)
{
    myTOLED.setContrastControl(0);

    myTOLED.pixelSet(84, 3);
    myTOLED.pixelSet(85, 3);
    myTOLED.pixelSet(86, 3);
    myTOLED.pixelSet(87, 3);
    myTOLED.pixelSet(88, 3);
    myTOLED.pixelSet(83, 4);
    myTOLED.pixelSet(84, 4);
    myTOLED.pixelSet(85, 4);
    myTOLED.pixelSet(86, 4);
    myTOLED.pixelSet(87, 4);
    myTOLED.pixelSet(82, 5);
    myTOLED.pixelSet(83, 5);
    myTOLED.pixelSet(84, 5);
    myTOLED.pixelSet(85, 5);
    myTOLED.pixelSet(86, 5);
    myTOLED.pixelSet(82, 6);
    myTOLED.pixelSet(83, 6);
    myTOLED.pixelSet(84, 6);
    myTOLED.pixelSet(85, 6);
    myTOLED.pixelSet(86, 6);
    myTOLED.pixelSet(82, 7);
    myTOLED.pixelSet(83, 7);
    myTOLED.pixelSet(84, 7);
    myTOLED.pixelSet(85, 7);
    myTOLED.pixelSet(86, 7);
    myTOLED.pixelSet(82, 8);
    myTOLED.pixelSet(83, 8);
    myTOLED.pixelSet(84, 8);
    myTOLED.pixelSet(85, 8);
    myTOLED.pixelSet(86, 8);
    myTOLED.pixelSet(87, 8);
    myTOLED.pixelSet(92, 8);
    myTOLED.pixelSet(83, 9);
    myTOLED.pixelSet(84, 9);
    myTOLED.pixelSet(85, 9);
    myTOLED.pixelSet(86, 9);
    myTOLED.pixelSet(87, 9);
    myTOLED.pixelSet(88, 9);
    myTOLED.pixelSet(89, 9);
    myTOLED.pixelSet(91, 9);
    myTOLED.pixelSet(92, 9);
    myTOLED.pixelSet(93, 9);
    myTOLED.pixelSet(84, 10);
    myTOLED.pixelSet(85, 10);
    myTOLED.pixelSet(86, 10);
    myTOLED.pixelSet(87, 10);
    myTOLED.pixelSet(88, 10);
    myTOLED.pixelSet(89, 10);
    myTOLED.pixelSet(90, 10);
    myTOLED.pixelSet(91, 10);
    myTOLED.pixelSet(92, 10);
    myTOLED.pixelSet(93, 10);
    myTOLED.pixelSet(85, 11);
    myTOLED.pixelSet(86, 11);
    myTOLED.pixelSet(87, 11);
    myTOLED.pixelSet(88, 11);
    myTOLED.pixelSet(89, 11);
    myTOLED.pixelSet(90, 11);
    myTOLED.pixelSet(91, 11);
    myTOLED.pixelSet(92, 11);
    myTOLED.pixelSet(93, 11);
    myTOLED.pixelSet(78, 12);
    myTOLED.pixelSet(79, 12);
    myTOLED.pixelSet(85, 12);
    myTOLED.pixelSet(86, 12);
    myTOLED.pixelSet(87, 12);
    myTOLED.pixelSet(88, 12);
    myTOLED.pixelSet(89, 12);
    myTOLED.pixelSet(90, 12);
    myTOLED.pixelSet(91, 12);
    myTOLED.pixelSet(92, 12);
    myTOLED.pixelSet(93, 12);
    myTOLED.pixelSet(94, 12);
    myTOLED.pixelSet(77, 13);
    myTOLED.pixelSet(78, 13);
    myTOLED.pixelSet(79, 13);
    myTOLED.pixelSet(85, 13);
    myTOLED.pixelSet(86, 13);
    myTOLED.pixelSet(87, 13);
    myTOLED.pixelSet(88, 13);
    myTOLED.pixelSet(89, 13);
    myTOLED.pixelSet(90, 13);
    myTOLED.pixelSet(91, 13);
    myTOLED.pixelSet(92, 13);
    myTOLED.pixelSet(93, 13);
    myTOLED.pixelSet(94, 13);
    myTOLED.pixelSet(77, 14);
    myTOLED.pixelSet(78, 14);
    myTOLED.pixelSet(79, 14);
    myTOLED.pixelSet(85, 14);
    myTOLED.pixelSet(86, 14);
    myTOLED.pixelSet(87, 14);
    myTOLED.pixelSet(88, 14);
    myTOLED.pixelSet(89, 14);
    myTOLED.pixelSet(90, 14);
    myTOLED.pixelSet(91, 14);
    myTOLED.pixelSet(92, 14);
    myTOLED.pixelSet(93, 14);
    myTOLED.pixelSet(94, 14);
    myTOLED.pixelSet(76, 15);
    myTOLED.pixelSet(77, 15);
    myTOLED.pixelSet(78, 15);
    myTOLED.pixelSet(79, 15);
    myTOLED.pixelSet(80, 15);
    myTOLED.pixelSet(84, 15);
    myTOLED.pixelSet(85, 15);
    myTOLED.pixelSet(86, 15);
    myTOLED.pixelSet(87, 15);
    myTOLED.pixelSet(88, 15);
    myTOLED.pixelSet(89, 15);
    myTOLED.pixelSet(90, 15);
    myTOLED.pixelSet(91, 15);
    myTOLED.pixelSet(92, 15);
    myTOLED.pixelSet(93, 15);
    myTOLED.pixelSet(94, 15);
    myTOLED.pixelSet(76, 16);
    myTOLED.pixelSet(77, 16);
    myTOLED.pixelSet(78, 16);
    myTOLED.pixelSet(79, 16);
    myTOLED.pixelSet(80, 16);
    myTOLED.pixelSet(81, 16);
    myTOLED.pixelSet(82, 16);
    myTOLED.pixelSet(83, 16);
    myTOLED.pixelSet(84, 16);
    myTOLED.pixelSet(85, 16);
    myTOLED.pixelSet(86, 16);
    myTOLED.pixelSet(87, 16);
    myTOLED.pixelSet(88, 16);
    myTOLED.pixelSet(89, 16);
    myTOLED.pixelSet(90, 16);
    myTOLED.pixelSet(91, 16);
    myTOLED.pixelSet(92, 16);
    myTOLED.pixelSet(93, 16);
    myTOLED.pixelSet(94, 16);
    myTOLED.pixelSet(76, 17);
    myTOLED.pixelSet(77, 17);
    myTOLED.pixelSet(78, 17);
    myTOLED.pixelSet(79, 17);
    myTOLED.pixelSet(80, 17);
    myTOLED.pixelSet(81, 17);
    myTOLED.pixelSet(82, 17);
    myTOLED.pixelSet(83, 17);
    myTOLED.pixelSet(84, 17);
    myTOLED.pixelSet(85, 17);
    myTOLED.pixelSet(86, 17);
    myTOLED.pixelSet(87, 17);
    myTOLED.pixelSet(88, 17);
    myTOLED.pixelSet(89, 17);
    myTOLED.pixelSet(90, 17);
    myTOLED.pixelSet(91, 17);
    myTOLED.pixelSet(92, 17);
    myTOLED.pixelSet(93, 17);
    myTOLED.pixelSet(76, 18);
    myTOLED.pixelSet(77, 18);
    myTOLED.pixelSet(78, 18);
    myTOLED.pixelSet(79, 18);
    myTOLED.pixelSet(80, 18);
    myTOLED.pixelSet(81, 18);
    myTOLED.pixelSet(82, 18);
    myTOLED.pixelSet(83, 18);
    myTOLED.pixelSet(84, 18);
    myTOLED.pixelSet(85, 18);
    myTOLED.pixelSet(86, 18);
    myTOLED.pixelSet(87, 18);
    myTOLED.pixelSet(88, 18);
    myTOLED.pixelSet(89, 18);
    myTOLED.pixelSet(90, 18);
    myTOLED.pixelSet(91, 18);
    myTOLED.pixelSet(92, 18);
    myTOLED.pixelSet(93, 18);
    myTOLED.pixelSet(76, 19);
    myTOLED.pixelSet(77, 19);
    myTOLED.pixelSet(78, 19);
    myTOLED.pixelSet(79, 19);
    myTOLED.pixelSet(80, 19);
    myTOLED.pixelSet(81, 19);
    myTOLED.pixelSet(82, 19);
    myTOLED.pixelSet(83, 19);
    myTOLED.pixelSet(84, 19);
    myTOLED.pixelSet(85, 19);
    myTOLED.pixelSet(86, 19);
    myTOLED.pixelSet(87, 19);
    myTOLED.pixelSet(88, 19);
    myTOLED.pixelSet(89, 19);
    myTOLED.pixelSet(90, 19);
    myTOLED.pixelSet(91, 19);
    myTOLED.pixelSet(92, 19);
    myTOLED.pixelSet(76, 20);
    myTOLED.pixelSet(77, 20);
    myTOLED.pixelSet(78, 20);
    myTOLED.pixelSet(79, 20);
    myTOLED.pixelSet(80, 20);
    myTOLED.pixelSet(81, 20);
    myTOLED.pixelSet(82, 20);
    myTOLED.pixelSet(83, 20);
    myTOLED.pixelSet(84, 20);
    myTOLED.pixelSet(85, 20);
    myTOLED.pixelSet(86, 20);
    myTOLED.pixelSet(87, 20);
    myTOLED.pixelSet(88, 20);
    myTOLED.pixelSet(89, 20);
    myTOLED.pixelSet(90, 20);
    myTOLED.pixelSet(91, 20);
    myTOLED.pixelSet(92, 20);
    myTOLED.pixelSet(76, 21);
    myTOLED.pixelSet(77, 21);
    myTOLED.pixelSet(78, 21);
    myTOLED.pixelSet(79, 21);
    myTOLED.pixelSet(80, 21);
    myTOLED.pixelSet(81, 21);
    myTOLED.pixelSet(82, 21);
    myTOLED.pixelSet(83, 21);
    myTOLED.pixelSet(84, 21);
    myTOLED.pixelSet(85, 21);
    myTOLED.pixelSet(86, 21);
    myTOLED.pixelSet(87, 21);
    myTOLED.pixelSet(88, 21);
    myTOLED.pixelSet(89, 21);
    myTOLED.pixelSet(90, 21);
    myTOLED.pixelSet(91, 21);
    myTOLED.pixelSet(76, 22);
    myTOLED.pixelSet(77, 22);
    myTOLED.pixelSet(78, 22);
    myTOLED.pixelSet(79, 22);
    myTOLED.pixelSet(80, 22);
    myTOLED.pixelSet(81, 22);
    myTOLED.pixelSet(82, 22);
    myTOLED.pixelSet(83, 22);
    myTOLED.pixelSet(84, 22);
    myTOLED.pixelSet(85, 22);
    myTOLED.pixelSet(86, 22);
    myTOLED.pixelSet(87, 22);
    myTOLED.pixelSet(88, 22);
    myTOLED.pixelSet(89, 22);
    myTOLED.pixelSet(76, 23);
    myTOLED.pixelSet(77, 23);
    myTOLED.pixelSet(78, 23);
    myTOLED.pixelSet(79, 23);
    myTOLED.pixelSet(80, 23);
    myTOLED.pixelSet(81, 23);
    myTOLED.pixelSet(82, 23);
    myTOLED.pixelSet(83, 23);
    myTOLED.pixelSet(84, 23);
    myTOLED.pixelSet(85, 23);
    myTOLED.pixelSet(86, 23);
    myTOLED.pixelSet(87, 23);
    myTOLED.pixelSet(76, 24);
    myTOLED.pixelSet(77, 24);
    myTOLED.pixelSet(78, 24);
    myTOLED.pixelSet(79, 24);
    myTOLED.pixelSet(80, 24);
    myTOLED.pixelSet(81, 24);
    myTOLED.pixelSet(76, 25);
    myTOLED.pixelSet(77, 25);
    myTOLED.pixelSet(78, 25);
    myTOLED.pixelSet(79, 25);
    myTOLED.pixelSet(80, 25);
    myTOLED.pixelSet(76, 26);
    myTOLED.pixelSet(77, 26);
    myTOLED.pixelSet(78, 26);
    myTOLED.pixelSet(79, 26);
    myTOLED.pixelSet(76, 27);
    myTOLED.pixelSet(77, 27);
    myTOLED.pixelSet(78, 27);
    myTOLED.pixelSet(76, 28);
    myTOLED.pixelSet(77, 28);
    myTOLED.pixelSet(76, 29);
    myTOLED.pixelSet(88, 29);
    myTOLED.pixelSet(89, 29);
    myTOLED.pixelSet(90, 29);
    myTOLED.pixelSet(91, 29);
    myTOLED.pixelSet(92, 29);
    myTOLED.pixelSet(69, 30);
    myTOLED.pixelSet(70, 30);
    myTOLED.pixelSet(76, 30);
    myTOLED.pixelSet(87, 30);
    myTOLED.pixelSet(88, 30);
    myTOLED.pixelSet(89, 30);
    myTOLED.pixelSet(90, 30);
    myTOLED.pixelSet(91, 30);
    myTOLED.pixelSet(92, 30);
    myTOLED.pixelSet(67, 31);
    myTOLED.pixelSet(68, 31);
    myTOLED.pixelSet(69, 31);
    myTOLED.pixelSet(70, 31);
    myTOLED.pixelSet(86, 31);
    myTOLED.pixelSet(87, 31);
    myTOLED.pixelSet(88, 31);
    myTOLED.pixelSet(89, 31);
    myTOLED.pixelSet(90, 31);
    myTOLED.pixelSet(91, 31);
    myTOLED.pixelSet(92, 31);
    myTOLED.pixelSet(67, 32);
    myTOLED.pixelSet(68, 32);
    myTOLED.pixelSet(69, 32);
    myTOLED.pixelSet(70, 32);
    myTOLED.pixelSet(86, 32);
    myTOLED.pixelSet(87, 32);
    myTOLED.pixelSet(88, 32);
    myTOLED.pixelSet(89, 32);
    myTOLED.pixelSet(67, 33);
    myTOLED.pixelSet(68, 33);
    myTOLED.pixelSet(69, 33);
    myTOLED.pixelSet(70, 33);
    myTOLED.pixelSet(86, 33);
    myTOLED.pixelSet(87, 33);
    myTOLED.pixelSet(88, 33);
    myTOLED.pixelSet(67, 34);
    myTOLED.pixelSet(68, 34);
    myTOLED.pixelSet(69, 34);
    myTOLED.pixelSet(70, 34);
    myTOLED.pixelSet(86, 34);
    myTOLED.pixelSet(87, 34);
    myTOLED.pixelSet(88, 34);
    myTOLED.pixelSet(6, 35);
    myTOLED.pixelSet(7, 35);
    myTOLED.pixelSet(8, 35);
    myTOLED.pixelSet(9, 35);
    myTOLED.pixelSet(10, 35);
    myTOLED.pixelSet(11, 35);
    myTOLED.pixelSet(12, 35);
    myTOLED.pixelSet(13, 35);
    myTOLED.pixelSet(22, 35);
    myTOLED.pixelSet(26, 35);
    myTOLED.pixelSet(27, 35);
    myTOLED.pixelSet(28, 35);
    myTOLED.pixelSet(29, 35);
    myTOLED.pixelSet(30, 35);
    myTOLED.pixelSet(31, 35);
    myTOLED.pixelSet(41, 35);
    myTOLED.pixelSet(42, 35);
    myTOLED.pixelSet(43, 35);
    myTOLED.pixelSet(44, 35);
    myTOLED.pixelSet(45, 35);
    myTOLED.pixelSet(46, 35);
    myTOLED.pixelSet(47, 35);
    myTOLED.pixelSet(48, 35);
    myTOLED.pixelSet(61, 35);
    myTOLED.pixelSet(62, 35);
    myTOLED.pixelSet(63, 35);
    myTOLED.pixelSet(64, 35);
    myTOLED.pixelSet(67, 35);
    myTOLED.pixelSet(68, 35);
    myTOLED.pixelSet(69, 35);
    myTOLED.pixelSet(70, 35);
    myTOLED.pixelSet(77, 35);
    myTOLED.pixelSet(78, 35);
    myTOLED.pixelSet(79, 35);
    myTOLED.pixelSet(80, 35);
    myTOLED.pixelSet(84, 35);
    myTOLED.pixelSet(85, 35);
    myTOLED.pixelSet(86, 35);
    myTOLED.pixelSet(87, 35);
    myTOLED.pixelSet(88, 35);
    myTOLED.pixelSet(89, 35);
    myTOLED.pixelSet(90, 35);
    myTOLED.pixelSet(91, 35);
    myTOLED.pixelSet(92, 35);
    myTOLED.pixelSet(94, 35);
    myTOLED.pixelSet(95, 35);
    myTOLED.pixelSet(96, 35);
    myTOLED.pixelSet(97, 35);
    myTOLED.pixelSet(105, 35);
    myTOLED.pixelSet(106, 35);
    myTOLED.pixelSet(107, 35);
    myTOLED.pixelSet(108, 35);
    myTOLED.pixelSet(112, 35);
    myTOLED.pixelSet(113, 35);
    myTOLED.pixelSet(114, 35);
    myTOLED.pixelSet(118, 35);
    myTOLED.pixelSet(119, 35);
    myTOLED.pixelSet(120, 35);
    myTOLED.pixelSet(121, 35);
    myTOLED.pixelSet(122, 35);
    myTOLED.pixelSet(123, 35);
    myTOLED.pixelSet(5, 36);
    myTOLED.pixelSet(6, 36);
    myTOLED.pixelSet(7, 36);
    myTOLED.pixelSet(8, 36);
    myTOLED.pixelSet(9, 36);
    myTOLED.pixelSet(10, 36);
    myTOLED.pixelSet(11, 36);
    myTOLED.pixelSet(12, 36);
    myTOLED.pixelSet(13, 36);
    myTOLED.pixelSet(14, 36);
    myTOLED.pixelSet(20, 36);
    myTOLED.pixelSet(21, 36);
    myTOLED.pixelSet(22, 36);
    myTOLED.pixelSet(24, 36);
    myTOLED.pixelSet(25, 36);
    myTOLED.pixelSet(26, 36);
    myTOLED.pixelSet(27, 36);
    myTOLED.pixelSet(28, 36);
    myTOLED.pixelSet(29, 36);
    myTOLED.pixelSet(30, 36);
    myTOLED.pixelSet(31, 36);
    myTOLED.pixelSet(32, 36);
    myTOLED.pixelSet(39, 36);
    myTOLED.pixelSet(40, 36);
    myTOLED.pixelSet(41, 36);
    myTOLED.pixelSet(42, 36);
    myTOLED.pixelSet(43, 36);
    myTOLED.pixelSet(44, 36);
    myTOLED.pixelSet(45, 36);
    myTOLED.pixelSet(46, 36);
    myTOLED.pixelSet(47, 36);
    myTOLED.pixelSet(48, 36);
    myTOLED.pixelSet(49, 36);
    myTOLED.pixelSet(50, 36);
    myTOLED.pixelSet(55, 36);
    myTOLED.pixelSet(56, 36);
    myTOLED.pixelSet(57, 36);
    myTOLED.pixelSet(58, 36);
    myTOLED.pixelSet(60, 36);
    myTOLED.pixelSet(61, 36);
    myTOLED.pixelSet(62, 36);
    myTOLED.pixelSet(63, 36);
    myTOLED.pixelSet(64, 36);
    myTOLED.pixelSet(67, 36);
    myTOLED.pixelSet(68, 36);
    myTOLED.pixelSet(69, 36);
    myTOLED.pixelSet(70, 36);
    myTOLED.pixelSet(76, 36);
    myTOLED.pixelSet(77, 36);
    myTOLED.pixelSet(78, 36);
    myTOLED.pixelSet(79, 36);
    myTOLED.pixelSet(83, 36);
    myTOLED.pixelSet(84, 36);
    myTOLED.pixelSet(85, 36);
    myTOLED.pixelSet(86, 36);
    myTOLED.pixelSet(87, 36);
    myTOLED.pixelSet(88, 36);
    myTOLED.pixelSet(89, 36);
    myTOLED.pixelSet(90, 36);
    myTOLED.pixelSet(91, 36);
    myTOLED.pixelSet(92, 36);
    myTOLED.pixelSet(94, 36);
    myTOLED.pixelSet(95, 36);
    myTOLED.pixelSet(96, 36);
    myTOLED.pixelSet(97, 36);
    myTOLED.pixelSet(105, 36);
    myTOLED.pixelSet(106, 36);
    myTOLED.pixelSet(107, 36);
    myTOLED.pixelSet(108, 36);
    myTOLED.pixelSet(112, 36);
    myTOLED.pixelSet(113, 36);
    myTOLED.pixelSet(114, 36);
    myTOLED.pixelSet(117, 36);
    myTOLED.pixelSet(118, 36);
    myTOLED.pixelSet(119, 36);
    myTOLED.pixelSet(120, 36);
    myTOLED.pixelSet(121, 36);
    myTOLED.pixelSet(122, 36);
    myTOLED.pixelSet(123, 36);
    myTOLED.pixelSet(124, 36);
    myTOLED.pixelSet(4, 37);
    myTOLED.pixelSet(5, 37);
    myTOLED.pixelSet(6, 37);
    myTOLED.pixelSet(7, 37);
    myTOLED.pixelSet(12, 37);
    myTOLED.pixelSet(13, 37);
    myTOLED.pixelSet(14, 37);
    myTOLED.pixelSet(15, 37);
    myTOLED.pixelSet(20, 37);
    myTOLED.pixelSet(21, 37);
    myTOLED.pixelSet(22, 37);
    myTOLED.pixelSet(23, 37);
    myTOLED.pixelSet(24, 37);
    myTOLED.pixelSet(25, 37);
    myTOLED.pixelSet(26, 37);
    myTOLED.pixelSet(27, 37);
    myTOLED.pixelSet(28, 37);
    myTOLED.pixelSet(29, 37);
    myTOLED.pixelSet(30, 37);
    myTOLED.pixelSet(31, 37);
    myTOLED.pixelSet(32, 37);
    myTOLED.pixelSet(33, 37);
    myTOLED.pixelSet(38, 37);
    myTOLED.pixelSet(39, 37);
    myTOLED.pixelSet(40, 37);
    myTOLED.pixelSet(41, 37);
    myTOLED.pixelSet(42, 37);
    myTOLED.pixelSet(47, 37);
    myTOLED.pixelSet(48, 37);
    myTOLED.pixelSet(49, 37);
    myTOLED.pixelSet(50, 37);
    myTOLED.pixelSet(51, 37);
    myTOLED.pixelSet(55, 37);
    myTOLED.pixelSet(56, 37);
    myTOLED.pixelSet(57, 37);
    myTOLED.pixelSet(58, 37);
    myTOLED.pixelSet(59, 37);
    myTOLED.pixelSet(60, 37);
    myTOLED.pixelSet(61, 37);
    myTOLED.pixelSet(62, 37);
    myTOLED.pixelSet(63, 37);
    myTOLED.pixelSet(64, 37);
    myTOLED.pixelSet(67, 37);
    myTOLED.pixelSet(68, 37);
    myTOLED.pixelSet(69, 37);
    myTOLED.pixelSet(70, 37);
    myTOLED.pixelSet(75, 37);
    myTOLED.pixelSet(76, 37);
    myTOLED.pixelSet(77, 37);
    myTOLED.pixelSet(78, 37);
    myTOLED.pixelSet(82, 37);
    myTOLED.pixelSet(83, 37);
    myTOLED.pixelSet(84, 37);
    myTOLED.pixelSet(85, 37);
    myTOLED.pixelSet(86, 37);
    myTOLED.pixelSet(87, 37);
    myTOLED.pixelSet(88, 37);
    myTOLED.pixelSet(89, 37);
    myTOLED.pixelSet(90, 37);
    myTOLED.pixelSet(91, 37);
    myTOLED.pixelSet(92, 37);
    myTOLED.pixelSet(94, 37);
    myTOLED.pixelSet(95, 37);
    myTOLED.pixelSet(96, 37);
    myTOLED.pixelSet(97, 37);
    myTOLED.pixelSet(105, 37);
    myTOLED.pixelSet(106, 37);
    myTOLED.pixelSet(107, 37);
    myTOLED.pixelSet(108, 37);
    myTOLED.pixelSet(112, 37);
    myTOLED.pixelSet(113, 37);
    myTOLED.pixelSet(114, 37);
    myTOLED.pixelSet(115, 37);
    myTOLED.pixelSet(116, 37);
    myTOLED.pixelSet(117, 37);
    myTOLED.pixelSet(118, 37);
    myTOLED.pixelSet(119, 37);
    myTOLED.pixelSet(120, 37);
    myTOLED.pixelSet(121, 37);
    myTOLED.pixelSet(122, 37);
    myTOLED.pixelSet(123, 37);
    myTOLED.pixelSet(124, 37);
    myTOLED.pixelSet(125, 37);
    myTOLED.pixelSet(3, 38);
    myTOLED.pixelSet(4, 38);
    myTOLED.pixelSet(5, 38);
    myTOLED.pixelSet(6, 38);
    myTOLED.pixelSet(13, 38);
    myTOLED.pixelSet(14, 38);
    myTOLED.pixelSet(15, 38);
    myTOLED.pixelSet(16, 38);
    myTOLED.pixelSet(20, 38);
    myTOLED.pixelSet(21, 38);
    myTOLED.pixelSet(22, 38);
    myTOLED.pixelSet(23, 38);
    myTOLED.pixelSet(24, 38);
    myTOLED.pixelSet(30, 38);
    myTOLED.pixelSet(31, 38);
    myTOLED.pixelSet(32, 38);
    myTOLED.pixelSet(33, 38);
    myTOLED.pixelSet(38, 38);
    myTOLED.pixelSet(39, 38);
    myTOLED.pixelSet(40, 38);
    myTOLED.pixelSet(41, 38);
    myTOLED.pixelSet(48, 38);
    myTOLED.pixelSet(49, 38);
    myTOLED.pixelSet(50, 38);
    myTOLED.pixelSet(51, 38);
    myTOLED.pixelSet(55, 38);
    myTOLED.pixelSet(56, 38);
    myTOLED.pixelSet(57, 38);
    myTOLED.pixelSet(58, 38);
    myTOLED.pixelSet(59, 38);
    myTOLED.pixelSet(60, 38);
    myTOLED.pixelSet(61, 38);
    myTOLED.pixelSet(64, 38);
    myTOLED.pixelSet(67, 38);
    myTOLED.pixelSet(68, 38);
    myTOLED.pixelSet(69, 38);
    myTOLED.pixelSet(70, 38);
    myTOLED.pixelSet(74, 38);
    myTOLED.pixelSet(75, 38);
    myTOLED.pixelSet(76, 38);
    myTOLED.pixelSet(77, 38);
    myTOLED.pixelSet(86, 38);
    myTOLED.pixelSet(87, 38);
    myTOLED.pixelSet(88, 38);
    myTOLED.pixelSet(94, 38);
    myTOLED.pixelSet(95, 38);
    myTOLED.pixelSet(96, 38);
    myTOLED.pixelSet(97, 38);
    myTOLED.pixelSet(105, 38);
    myTOLED.pixelSet(106, 38);
    myTOLED.pixelSet(107, 38);
    myTOLED.pixelSet(108, 38);
    myTOLED.pixelSet(112, 38);
    myTOLED.pixelSet(113, 38);
    myTOLED.pixelSet(114, 38);
    myTOLED.pixelSet(115, 38);
    myTOLED.pixelSet(116, 38);
    myTOLED.pixelSet(121, 38);
    myTOLED.pixelSet(122, 38);
    myTOLED.pixelSet(123, 38);
    myTOLED.pixelSet(124, 38);
    myTOLED.pixelSet(125, 38);
    myTOLED.pixelSet(3, 39);
    myTOLED.pixelSet(4, 39);
    myTOLED.pixelSet(5, 39);
    myTOLED.pixelSet(6, 39);
    myTOLED.pixelSet(20, 39);
    myTOLED.pixelSet(21, 39);
    myTOLED.pixelSet(22, 39);
    myTOLED.pixelSet(23, 39);
    myTOLED.pixelSet(31, 39);
    myTOLED.pixelSet(32, 39);
    myTOLED.pixelSet(33, 39);
    myTOLED.pixelSet(34, 39);
    myTOLED.pixelSet(38, 39);
    myTOLED.pixelSet(39, 39);
    myTOLED.pixelSet(40, 39);
    myTOLED.pixelSet(41, 39);
    myTOLED.pixelSet(48, 39);
    myTOLED.pixelSet(49, 39);
    myTOLED.pixelSet(50, 39);
    myTOLED.pixelSet(51, 39);
    myTOLED.pixelSet(55, 39);
    myTOLED.pixelSet(56, 39);
    myTOLED.pixelSet(57, 39);
    myTOLED.pixelSet(58, 39);
    myTOLED.pixelSet(59, 39);
    myTOLED.pixelSet(67, 39);
    myTOLED.pixelSet(68, 39);
    myTOLED.pixelSet(69, 39);
    myTOLED.pixelSet(70, 39);
    myTOLED.pixelSet(73, 39);
    myTOLED.pixelSet(74, 39);
    myTOLED.pixelSet(75, 39);
    myTOLED.pixelSet(76, 39);
    myTOLED.pixelSet(86, 39);
    myTOLED.pixelSet(87, 39);
    myTOLED.pixelSet(88, 39);
    myTOLED.pixelSet(94, 39);
    myTOLED.pixelSet(95, 39);
    myTOLED.pixelSet(96, 39);
    myTOLED.pixelSet(97, 39);
    myTOLED.pixelSet(105, 39);
    myTOLED.pixelSet(106, 39);
    myTOLED.pixelSet(107, 39);
    myTOLED.pixelSet(108, 39);
    myTOLED.pixelSet(112, 39);
    myTOLED.pixelSet(113, 39);
    myTOLED.pixelSet(114, 39);
    myTOLED.pixelSet(115, 39);
    myTOLED.pixelSet(122, 39);
    myTOLED.pixelSet(123, 39);
    myTOLED.pixelSet(124, 39);
    myTOLED.pixelSet(125, 39);
    myTOLED.pixelSet(3, 40);
    myTOLED.pixelSet(4, 40);
    myTOLED.pixelSet(5, 40);
    myTOLED.pixelSet(6, 40);
    myTOLED.pixelSet(7, 40);
    myTOLED.pixelSet(20, 40);
    myTOLED.pixelSet(21, 40);
    myTOLED.pixelSet(22, 40);
    myTOLED.pixelSet(23, 40);
    myTOLED.pixelSet(31, 40);
    myTOLED.pixelSet(32, 40);
    myTOLED.pixelSet(33, 40);
    myTOLED.pixelSet(34, 40);
    myTOLED.pixelSet(48, 40);
    myTOLED.pixelSet(49, 40);
    myTOLED.pixelSet(50, 40);
    myTOLED.pixelSet(51, 40);
    myTOLED.pixelSet(55, 40);
    myTOLED.pixelSet(56, 40);
    myTOLED.pixelSet(57, 40);
    myTOLED.pixelSet(58, 40);
    myTOLED.pixelSet(59, 40);
    myTOLED.pixelSet(67, 40);
    myTOLED.pixelSet(68, 40);
    myTOLED.pixelSet(69, 40);
    myTOLED.pixelSet(70, 40);
    myTOLED.pixelSet(72, 40);
    myTOLED.pixelSet(73, 40);
    myTOLED.pixelSet(74, 40);
    myTOLED.pixelSet(75, 40);
    myTOLED.pixelSet(86, 40);
    myTOLED.pixelSet(87, 40);
    myTOLED.pixelSet(88, 40);
    myTOLED.pixelSet(94, 40);
    myTOLED.pixelSet(95, 40);
    myTOLED.pixelSet(96, 40);
    myTOLED.pixelSet(97, 40);
    myTOLED.pixelSet(105, 40);
    myTOLED.pixelSet(106, 40);
    myTOLED.pixelSet(107, 40);
    myTOLED.pixelSet(108, 40);
    myTOLED.pixelSet(112, 40);
    myTOLED.pixelSet(113, 40);
    myTOLED.pixelSet(114, 40);
    myTOLED.pixelSet(115, 40);
    myTOLED.pixelSet(122, 40);
    myTOLED.pixelSet(123, 40);
    myTOLED.pixelSet(124, 40);
    myTOLED.pixelSet(125, 40);
    myTOLED.pixelSet(4, 41);
    myTOLED.pixelSet(5, 41);
    myTOLED.pixelSet(6, 41);
    myTOLED.pixelSet(7, 41);
    myTOLED.pixelSet(8, 41);
    myTOLED.pixelSet(9, 41);
    myTOLED.pixelSet(10, 41);
    myTOLED.pixelSet(11, 41);
    myTOLED.pixelSet(20, 41);
    myTOLED.pixelSet(21, 41);
    myTOLED.pixelSet(22, 41);
    myTOLED.pixelSet(31, 41);
    myTOLED.pixelSet(32, 41);
    myTOLED.pixelSet(33, 41);
    myTOLED.pixelSet(34, 41);
    myTOLED.pixelSet(46, 41);
    myTOLED.pixelSet(47, 41);
    myTOLED.pixelSet(48, 41);
    myTOLED.pixelSet(49, 41);
    myTOLED.pixelSet(50, 41);
    myTOLED.pixelSet(51, 41);
    myTOLED.pixelSet(55, 41);
    myTOLED.pixelSet(56, 41);
    myTOLED.pixelSet(57, 41);
    myTOLED.pixelSet(58, 41);
    myTOLED.pixelSet(67, 41);
    myTOLED.pixelSet(68, 41);
    myTOLED.pixelSet(69, 41);
    myTOLED.pixelSet(70, 41);
    myTOLED.pixelSet(71, 41);
    myTOLED.pixelSet(72, 41);
    myTOLED.pixelSet(73, 41);
    myTOLED.pixelSet(74, 41);
    myTOLED.pixelSet(75, 41);
    myTOLED.pixelSet(86, 41);
    myTOLED.pixelSet(87, 41);
    myTOLED.pixelSet(88, 41);
    myTOLED.pixelSet(94, 41);
    myTOLED.pixelSet(95, 41);
    myTOLED.pixelSet(96, 41);
    myTOLED.pixelSet(97, 41);
    myTOLED.pixelSet(105, 41);
    myTOLED.pixelSet(106, 41);
    myTOLED.pixelSet(107, 41);
    myTOLED.pixelSet(108, 41);
    myTOLED.pixelSet(112, 41);
    myTOLED.pixelSet(113, 41);
    myTOLED.pixelSet(114, 41);
    myTOLED.pixelSet(115, 41);
    myTOLED.pixelSet(122, 41);
    myTOLED.pixelSet(123, 41);
    myTOLED.pixelSet(124, 41);
    myTOLED.pixelSet(125, 41);
    myTOLED.pixelSet(5, 42);
    myTOLED.pixelSet(6, 42);
    myTOLED.pixelSet(7, 42);
    myTOLED.pixelSet(8, 42);
    myTOLED.pixelSet(9, 42);
    myTOLED.pixelSet(10, 42);
    myTOLED.pixelSet(11, 42);
    myTOLED.pixelSet(12, 42);
    myTOLED.pixelSet(13, 42);
    myTOLED.pixelSet(14, 42);
    myTOLED.pixelSet(20, 42);
    myTOLED.pixelSet(21, 42);
    myTOLED.pixelSet(22, 42);
    myTOLED.pixelSet(32, 42);
    myTOLED.pixelSet(33, 42);
    myTOLED.pixelSet(34, 42);
    myTOLED.pixelSet(41, 42);
    myTOLED.pixelSet(42, 42);
    myTOLED.pixelSet(43, 42);
    myTOLED.pixelSet(44, 42);
    myTOLED.pixelSet(45, 42);
    myTOLED.pixelSet(46, 42);
    myTOLED.pixelSet(47, 42);
    myTOLED.pixelSet(48, 42);
    myTOLED.pixelSet(49, 42);
    myTOLED.pixelSet(50, 42);
    myTOLED.pixelSet(51, 42);
    myTOLED.pixelSet(55, 42);
    myTOLED.pixelSet(56, 42);
    myTOLED.pixelSet(57, 42);
    myTOLED.pixelSet(58, 42);
    myTOLED.pixelSet(67, 42);
    myTOLED.pixelSet(68, 42);
    myTOLED.pixelSet(69, 42);
    myTOLED.pixelSet(70, 42);
    myTOLED.pixelSet(71, 42);
    myTOLED.pixelSet(72, 42);
    myTOLED.pixelSet(73, 42);
    myTOLED.pixelSet(74, 42);
    myTOLED.pixelSet(75, 42);
    myTOLED.pixelSet(76, 42);
    myTOLED.pixelSet(86, 42);
    myTOLED.pixelSet(87, 42);
    myTOLED.pixelSet(88, 42);
    myTOLED.pixelSet(94, 42);
    myTOLED.pixelSet(95, 42);
    myTOLED.pixelSet(96, 42);
    myTOLED.pixelSet(97, 42);
    myTOLED.pixelSet(105, 42);
    myTOLED.pixelSet(106, 42);
    myTOLED.pixelSet(107, 42);
    myTOLED.pixelSet(108, 42);
    myTOLED.pixelSet(112, 42);
    myTOLED.pixelSet(113, 42);
    myTOLED.pixelSet(114, 42);
    myTOLED.pixelSet(115, 42);
    myTOLED.pixelSet(122, 42);
    myTOLED.pixelSet(123, 42);
    myTOLED.pixelSet(124, 42);
    myTOLED.pixelSet(125, 42);
    myTOLED.pixelSet(7, 43);
    myTOLED.pixelSet(8, 43);
    myTOLED.pixelSet(9, 43);
    myTOLED.pixelSet(10, 43);
    myTOLED.pixelSet(11, 43);
    myTOLED.pixelSet(12, 43);
    myTOLED.pixelSet(13, 43);
    myTOLED.pixelSet(14, 43);
    myTOLED.pixelSet(15, 43);
    myTOLED.pixelSet(20, 43);
    myTOLED.pixelSet(21, 43);
    myTOLED.pixelSet(22, 43);
    myTOLED.pixelSet(32, 43);
    myTOLED.pixelSet(33, 43);
    myTOLED.pixelSet(34, 43);
    myTOLED.pixelSet(39, 43);
    myTOLED.pixelSet(40, 43);
    myTOLED.pixelSet(41, 43);
    myTOLED.pixelSet(42, 43);
    myTOLED.pixelSet(43, 43);
    myTOLED.pixelSet(44, 43);
    myTOLED.pixelSet(45, 43);
    myTOLED.pixelSet(48, 43);
    myTOLED.pixelSet(49, 43);
    myTOLED.pixelSet(50, 43);
    myTOLED.pixelSet(51, 43);
    myTOLED.pixelSet(55, 43);
    myTOLED.pixelSet(56, 43);
    myTOLED.pixelSet(57, 43);
    myTOLED.pixelSet(58, 43);
    myTOLED.pixelSet(67, 43);
    myTOLED.pixelSet(68, 43);
    myTOLED.pixelSet(69, 43);
    myTOLED.pixelSet(70, 43);
    myTOLED.pixelSet(71, 43);
    myTOLED.pixelSet(72, 43);
    myTOLED.pixelSet(73, 43);
    myTOLED.pixelSet(74, 43);
    myTOLED.pixelSet(75, 43);
    myTOLED.pixelSet(76, 43);
    myTOLED.pixelSet(86, 43);
    myTOLED.pixelSet(87, 43);
    myTOLED.pixelSet(88, 43);
    myTOLED.pixelSet(94, 43);
    myTOLED.pixelSet(95, 43);
    myTOLED.pixelSet(96, 43);
    myTOLED.pixelSet(97, 43);
    myTOLED.pixelSet(105, 43);
    myTOLED.pixelSet(106, 43);
    myTOLED.pixelSet(107, 43);
    myTOLED.pixelSet(108, 43);
    myTOLED.pixelSet(112, 43);
    myTOLED.pixelSet(113, 43);
    myTOLED.pixelSet(114, 43);
    myTOLED.pixelSet(115, 43);
    myTOLED.pixelSet(122, 43);
    myTOLED.pixelSet(123, 43);
    myTOLED.pixelSet(124, 43);
    myTOLED.pixelSet(125, 43);
    myTOLED.pixelSet(11, 44);
    myTOLED.pixelSet(12, 44);
    myTOLED.pixelSet(13, 44);
    myTOLED.pixelSet(14, 44);
    myTOLED.pixelSet(15, 44);
    myTOLED.pixelSet(16, 44);
    myTOLED.pixelSet(20, 44);
    myTOLED.pixelSet(21, 44);
    myTOLED.pixelSet(22, 44);
    myTOLED.pixelSet(31, 44);
    myTOLED.pixelSet(32, 44);
    myTOLED.pixelSet(33, 44);
    myTOLED.pixelSet(34, 44);
    myTOLED.pixelSet(38, 44);
    myTOLED.pixelSet(39, 44);
    myTOLED.pixelSet(40, 44);
    myTOLED.pixelSet(41, 44);
    myTOLED.pixelSet(48, 44);
    myTOLED.pixelSet(49, 44);
    myTOLED.pixelSet(50, 44);
    myTOLED.pixelSet(51, 44);
    myTOLED.pixelSet(55, 44);
    myTOLED.pixelSet(56, 44);
    myTOLED.pixelSet(57, 44);
    myTOLED.pixelSet(58, 44);
    myTOLED.pixelSet(67, 44);
    myTOLED.pixelSet(68, 44);
    myTOLED.pixelSet(69, 44);
    myTOLED.pixelSet(70, 44);
    myTOLED.pixelSet(71, 44);
    myTOLED.pixelSet(73, 44);
    myTOLED.pixelSet(74, 44);
    myTOLED.pixelSet(75, 44);
    myTOLED.pixelSet(76, 44);
    myTOLED.pixelSet(77, 44);
    myTOLED.pixelSet(86, 44);
    myTOLED.pixelSet(87, 44);
    myTOLED.pixelSet(88, 44);
    myTOLED.pixelSet(94, 44);
    myTOLED.pixelSet(95, 44);
    myTOLED.pixelSet(96, 44);
    myTOLED.pixelSet(97, 44);
    myTOLED.pixelSet(105, 44);
    myTOLED.pixelSet(106, 44);
    myTOLED.pixelSet(107, 44);
    myTOLED.pixelSet(108, 44);
    myTOLED.pixelSet(112, 44);
    myTOLED.pixelSet(113, 44);
    myTOLED.pixelSet(114, 44);
    myTOLED.pixelSet(115, 44);
    myTOLED.pixelSet(122, 44);
    myTOLED.pixelSet(123, 44);
    myTOLED.pixelSet(124, 44);
    myTOLED.pixelSet(125, 44);
    myTOLED.pixelSet(13, 45);
    myTOLED.pixelSet(14, 45);
    myTOLED.pixelSet(15, 45);
    myTOLED.pixelSet(16, 45);
    myTOLED.pixelSet(20, 45);
    myTOLED.pixelSet(21, 45);
    myTOLED.pixelSet(22, 45);
    myTOLED.pixelSet(23, 45);
    myTOLED.pixelSet(31, 45);
    myTOLED.pixelSet(32, 45);
    myTOLED.pixelSet(33, 45);
    myTOLED.pixelSet(34, 45);
    myTOLED.pixelSet(37, 45);
    myTOLED.pixelSet(38, 45);
    myTOLED.pixelSet(39, 45);
    myTOLED.pixelSet(40, 45);
    myTOLED.pixelSet(48, 45);
    myTOLED.pixelSet(49, 45);
    myTOLED.pixelSet(50, 45);
    myTOLED.pixelSet(51, 45);
    myTOLED.pixelSet(55, 45);
    myTOLED.pixelSet(56, 45);
    myTOLED.pixelSet(57, 45);
    myTOLED.pixelSet(58, 45);
    myTOLED.pixelSet(67, 45);
    myTOLED.pixelSet(68, 45);
    myTOLED.pixelSet(69, 45);
    myTOLED.pixelSet(70, 45);
    myTOLED.pixelSet(74, 45);
    myTOLED.pixelSet(75, 45);
    myTOLED.pixelSet(76, 45);
    myTOLED.pixelSet(77, 45);
    myTOLED.pixelSet(78, 45);
    myTOLED.pixelSet(86, 45);
    myTOLED.pixelSet(87, 45);
    myTOLED.pixelSet(88, 45);
    myTOLED.pixelSet(94, 45);
    myTOLED.pixelSet(95, 45);
    myTOLED.pixelSet(96, 45);
    myTOLED.pixelSet(97, 45);
    myTOLED.pixelSet(105, 45);
    myTOLED.pixelSet(106, 45);
    myTOLED.pixelSet(107, 45);
    myTOLED.pixelSet(108, 45);
    myTOLED.pixelSet(112, 45);
    myTOLED.pixelSet(113, 45);
    myTOLED.pixelSet(114, 45);
    myTOLED.pixelSet(115, 45);
    myTOLED.pixelSet(122, 45);
    myTOLED.pixelSet(123, 45);
    myTOLED.pixelSet(124, 45);
    myTOLED.pixelSet(125, 45);
    myTOLED.pixelSet(3, 46);
    myTOLED.pixelSet(4, 46);
    myTOLED.pixelSet(5, 46);
    myTOLED.pixelSet(6, 46);
    myTOLED.pixelSet(13, 46);
    myTOLED.pixelSet(14, 46);
    myTOLED.pixelSet(15, 46);
    myTOLED.pixelSet(16, 46);
    myTOLED.pixelSet(20, 46);
    myTOLED.pixelSet(21, 46);
    myTOLED.pixelSet(22, 46);
    myTOLED.pixelSet(23, 46);
    myTOLED.pixelSet(31, 46);
    myTOLED.pixelSet(32, 46);
    myTOLED.pixelSet(33, 46);
    myTOLED.pixelSet(34, 46);
    myTOLED.pixelSet(37, 46);
    myTOLED.pixelSet(38, 46);
    myTOLED.pixelSet(39, 46);
    myTOLED.pixelSet(40, 46);
    myTOLED.pixelSet(48, 46);
    myTOLED.pixelSet(49, 46);
    myTOLED.pixelSet(50, 46);
    myTOLED.pixelSet(51, 46);
    myTOLED.pixelSet(55, 46);
    myTOLED.pixelSet(56, 46);
    myTOLED.pixelSet(57, 46);
    myTOLED.pixelSet(58, 46);
    myTOLED.pixelSet(67, 46);
    myTOLED.pixelSet(68, 46);
    myTOLED.pixelSet(69, 46);
    myTOLED.pixelSet(70, 46);
    myTOLED.pixelSet(75, 46);
    myTOLED.pixelSet(76, 46);
    myTOLED.pixelSet(77, 46);
    myTOLED.pixelSet(78, 46);
    myTOLED.pixelSet(86, 46);
    myTOLED.pixelSet(87, 46);
    myTOLED.pixelSet(88, 46);
    myTOLED.pixelSet(94, 46);
    myTOLED.pixelSet(95, 46);
    myTOLED.pixelSet(96, 46);
    myTOLED.pixelSet(97, 46);
    myTOLED.pixelSet(98, 46);
    myTOLED.pixelSet(104, 46);
    myTOLED.pixelSet(105, 46);
    myTOLED.pixelSet(106, 46);
    myTOLED.pixelSet(107, 46);
    myTOLED.pixelSet(108, 46);
    myTOLED.pixelSet(112, 46);
    myTOLED.pixelSet(113, 46);
    myTOLED.pixelSet(114, 46);
    myTOLED.pixelSet(115, 46);
    myTOLED.pixelSet(122, 46);
    myTOLED.pixelSet(123, 46);
    myTOLED.pixelSet(124, 46);
    myTOLED.pixelSet(125, 46);
    myTOLED.pixelSet(3, 47);
    myTOLED.pixelSet(4, 47);
    myTOLED.pixelSet(5, 47);
    myTOLED.pixelSet(6, 47);
    myTOLED.pixelSet(13, 47);
    myTOLED.pixelSet(14, 47);
    myTOLED.pixelSet(15, 47);
    myTOLED.pixelSet(16, 47);
    myTOLED.pixelSet(20, 47);
    myTOLED.pixelSet(21, 47);
    myTOLED.pixelSet(22, 47);
    myTOLED.pixelSet(23, 47);
    myTOLED.pixelSet(24, 47);
    myTOLED.pixelSet(30, 47);
    myTOLED.pixelSet(31, 47);
    myTOLED.pixelSet(32, 47);
    myTOLED.pixelSet(33, 47);
    myTOLED.pixelSet(34, 47);
    myTOLED.pixelSet(37, 47);
    myTOLED.pixelSet(38, 47);
    myTOLED.pixelSet(39, 47);
    myTOLED.pixelSet(40, 47);
    myTOLED.pixelSet(47, 47);
    myTOLED.pixelSet(48, 47);
    myTOLED.pixelSet(49, 47);
    myTOLED.pixelSet(50, 47);
    myTOLED.pixelSet(51, 47);
    myTOLED.pixelSet(55, 47);
    myTOLED.pixelSet(56, 47);
    myTOLED.pixelSet(57, 47);
    myTOLED.pixelSet(58, 47);
    myTOLED.pixelSet(67, 47);
    myTOLED.pixelSet(68, 47);
    myTOLED.pixelSet(69, 47);
    myTOLED.pixelSet(70, 47);
    myTOLED.pixelSet(75, 47);
    myTOLED.pixelSet(76, 47);
    myTOLED.pixelSet(77, 47);
    myTOLED.pixelSet(78, 47);
    myTOLED.pixelSet(79, 47);
    myTOLED.pixelSet(86, 47);
    myTOLED.pixelSet(87, 47);
    myTOLED.pixelSet(88, 47);
    myTOLED.pixelSet(95, 47);
    myTOLED.pixelSet(96, 47);
    myTOLED.pixelSet(97, 47);
    myTOLED.pixelSet(98, 47);
    myTOLED.pixelSet(104, 47);
    myTOLED.pixelSet(105, 47);
    myTOLED.pixelSet(106, 47);
    myTOLED.pixelSet(107, 47);
    myTOLED.pixelSet(108, 47);
    myTOLED.pixelSet(112, 47);
    myTOLED.pixelSet(113, 47);
    myTOLED.pixelSet(114, 47);
    myTOLED.pixelSet(115, 47);
    myTOLED.pixelSet(122, 47);
    myTOLED.pixelSet(123, 47);
    myTOLED.pixelSet(124, 47);
    myTOLED.pixelSet(125, 47);
    myTOLED.pixelSet(4, 48);
    myTOLED.pixelSet(5, 48);
    myTOLED.pixelSet(6, 48);
    myTOLED.pixelSet(7, 48);
    myTOLED.pixelSet(12, 48);
    myTOLED.pixelSet(13, 48);
    myTOLED.pixelSet(14, 48);
    myTOLED.pixelSet(15, 48);
    myTOLED.pixelSet(16, 48);
    myTOLED.pixelSet(20, 48);
    myTOLED.pixelSet(21, 48);
    myTOLED.pixelSet(22, 48);
    myTOLED.pixelSet(23, 48);
    myTOLED.pixelSet(24, 48);
    myTOLED.pixelSet(25, 48);
    myTOLED.pixelSet(26, 48);
    myTOLED.pixelSet(27, 48);
    myTOLED.pixelSet(28, 48);
    myTOLED.pixelSet(29, 48);
    myTOLED.pixelSet(30, 48);
    myTOLED.pixelSet(31, 48);
    myTOLED.pixelSet(32, 48);
    myTOLED.pixelSet(33, 48);
    myTOLED.pixelSet(38, 48);
    myTOLED.pixelSet(39, 48);
    myTOLED.pixelSet(40, 48);
    myTOLED.pixelSet(41, 48);
    myTOLED.pixelSet(46, 48);
    myTOLED.pixelSet(47, 48);
    myTOLED.pixelSet(48, 48);
    myTOLED.pixelSet(49, 48);
    myTOLED.pixelSet(50, 48);
    myTOLED.pixelSet(51, 48);
    myTOLED.pixelSet(55, 48);
    myTOLED.pixelSet(56, 48);
    myTOLED.pixelSet(57, 48);
    myTOLED.pixelSet(58, 48);
    myTOLED.pixelSet(67, 48);
    myTOLED.pixelSet(68, 48);
    myTOLED.pixelSet(69, 48);
    myTOLED.pixelSet(70, 48);
    myTOLED.pixelSet(76, 48);
    myTOLED.pixelSet(77, 48);
    myTOLED.pixelSet(78, 48);
    myTOLED.pixelSet(79, 48);
    myTOLED.pixelSet(80, 48);
    myTOLED.pixelSet(86, 48);
    myTOLED.pixelSet(87, 48);
    myTOLED.pixelSet(88, 48);
    myTOLED.pixelSet(95, 48);
    myTOLED.pixelSet(96, 48);
    myTOLED.pixelSet(97, 48);
    myTOLED.pixelSet(98, 48);
    myTOLED.pixelSet(99, 48);
    myTOLED.pixelSet(100, 48);
    myTOLED.pixelSet(101, 48);
    myTOLED.pixelSet(102, 48);
    myTOLED.pixelSet(103, 48);
    myTOLED.pixelSet(104, 48);
    myTOLED.pixelSet(105, 48);
    myTOLED.pixelSet(106, 48);
    myTOLED.pixelSet(107, 48);
    myTOLED.pixelSet(108, 48);
    myTOLED.pixelSet(112, 48);
    myTOLED.pixelSet(113, 48);
    myTOLED.pixelSet(114, 48);
    myTOLED.pixelSet(115, 48);
    myTOLED.pixelSet(122, 48);
    myTOLED.pixelSet(123, 48);
    myTOLED.pixelSet(124, 48);
    myTOLED.pixelSet(125, 48);
    myTOLED.pixelSet(4, 49);
    myTOLED.pixelSet(5, 49);
    myTOLED.pixelSet(6, 49);
    myTOLED.pixelSet(7, 49);
    myTOLED.pixelSet(8, 49);
    myTOLED.pixelSet(9, 49);
    myTOLED.pixelSet(10, 49);
    myTOLED.pixelSet(11, 49);
    myTOLED.pixelSet(12, 49);
    myTOLED.pixelSet(13, 49);
    myTOLED.pixelSet(14, 49);
    myTOLED.pixelSet(15, 49);
    myTOLED.pixelSet(20, 49);
    myTOLED.pixelSet(21, 49);
    myTOLED.pixelSet(22, 49);
    myTOLED.pixelSet(23, 49);
    myTOLED.pixelSet(24, 49);
    myTOLED.pixelSet(25, 49);
    myTOLED.pixelSet(26, 49);
    myTOLED.pixelSet(27, 49);
    myTOLED.pixelSet(28, 49);
    myTOLED.pixelSet(29, 49);
    myTOLED.pixelSet(30, 49);
    myTOLED.pixelSet(31, 49);
    myTOLED.pixelSet(32, 49);
    myTOLED.pixelSet(38, 49);
    myTOLED.pixelSet(39, 49);
    myTOLED.pixelSet(40, 49);
    myTOLED.pixelSet(41, 49);
    myTOLED.pixelSet(42, 49);
    myTOLED.pixelSet(43, 49);
    myTOLED.pixelSet(44, 49);
    myTOLED.pixelSet(45, 49);
    myTOLED.pixelSet(46, 49);
    myTOLED.pixelSet(47, 49);
    myTOLED.pixelSet(48, 49);
    myTOLED.pixelSet(49, 49);
    myTOLED.pixelSet(50, 49);
    myTOLED.pixelSet(51, 49);
    myTOLED.pixelSet(55, 49);
    myTOLED.pixelSet(56, 49);
    myTOLED.pixelSet(57, 49);
    myTOLED.pixelSet(58, 49);
    myTOLED.pixelSet(67, 49);
    myTOLED.pixelSet(68, 49);
    myTOLED.pixelSet(69, 49);
    myTOLED.pixelSet(70, 49);
    myTOLED.pixelSet(77, 49);
    myTOLED.pixelSet(78, 49);
    myTOLED.pixelSet(79, 49);
    myTOLED.pixelSet(80, 49);
    myTOLED.pixelSet(86, 49);
    myTOLED.pixelSet(87, 49);
    myTOLED.pixelSet(88, 49);
    myTOLED.pixelSet(95, 49);
    myTOLED.pixelSet(96, 49);
    myTOLED.pixelSet(97, 49);
    myTOLED.pixelSet(98, 49);
    myTOLED.pixelSet(99, 49);
    myTOLED.pixelSet(100, 49);
    myTOLED.pixelSet(101, 49);
    myTOLED.pixelSet(102, 49);
    myTOLED.pixelSet(103, 49);
    myTOLED.pixelSet(105, 49);
    myTOLED.pixelSet(106, 49);
    myTOLED.pixelSet(107, 49);
    myTOLED.pixelSet(108, 49);
    myTOLED.pixelSet(112, 49);
    myTOLED.pixelSet(113, 49);
    myTOLED.pixelSet(114, 49);
    myTOLED.pixelSet(115, 49);
    myTOLED.pixelSet(122, 49);
    myTOLED.pixelSet(123, 49);
    myTOLED.pixelSet(124, 49);
    myTOLED.pixelSet(125, 49);
    myTOLED.pixelSet(6, 50);
    myTOLED.pixelSet(7, 50);
    myTOLED.pixelSet(8, 50);
    myTOLED.pixelSet(9, 50);
    myTOLED.pixelSet(10, 50);
    myTOLED.pixelSet(11, 50);
    myTOLED.pixelSet(12, 50);
    myTOLED.pixelSet(13, 50);
    myTOLED.pixelSet(20, 50);
    myTOLED.pixelSet(21, 50);
    myTOLED.pixelSet(22, 50);
    myTOLED.pixelSet(23, 50);
    myTOLED.pixelSet(26, 50);
    myTOLED.pixelSet(27, 50);
    myTOLED.pixelSet(28, 50);
    myTOLED.pixelSet(29, 50);
    myTOLED.pixelSet(30, 50);
    myTOLED.pixelSet(31, 50);
    myTOLED.pixelSet(39, 50);
    myTOLED.pixelSet(40, 50);
    myTOLED.pixelSet(41, 50);
    myTOLED.pixelSet(42, 50);
    myTOLED.pixelSet(43, 50);
    myTOLED.pixelSet(44, 50);
    myTOLED.pixelSet(45, 50);
    myTOLED.pixelSet(48, 50);
    myTOLED.pixelSet(49, 50);
    myTOLED.pixelSet(50, 50);
    myTOLED.pixelSet(51, 50);
    myTOLED.pixelSet(55, 50);
    myTOLED.pixelSet(56, 50);
    myTOLED.pixelSet(57, 50);
    myTOLED.pixelSet(58, 50);
    myTOLED.pixelSet(67, 50);
    myTOLED.pixelSet(68, 50);
    myTOLED.pixelSet(69, 50);
    myTOLED.pixelSet(70, 50);
    myTOLED.pixelSet(77, 50);
    myTOLED.pixelSet(78, 50);
    myTOLED.pixelSet(79, 50);
    myTOLED.pixelSet(80, 50);
    myTOLED.pixelSet(81, 50);
    myTOLED.pixelSet(86, 50);
    myTOLED.pixelSet(87, 50);
    myTOLED.pixelSet(88, 50);
    myTOLED.pixelSet(96, 50);
    myTOLED.pixelSet(97, 50);
    myTOLED.pixelSet(98, 50);
    myTOLED.pixelSet(99, 50);
    myTOLED.pixelSet(100, 50);
    myTOLED.pixelSet(101, 50);
    myTOLED.pixelSet(102, 50);
    myTOLED.pixelSet(105, 50);
    myTOLED.pixelSet(106, 50);
    myTOLED.pixelSet(107, 50);
    myTOLED.pixelSet(108, 50);
    myTOLED.pixelSet(112, 50);
    myTOLED.pixelSet(113, 50);
    myTOLED.pixelSet(114, 50);
    myTOLED.pixelSet(115, 50);
    myTOLED.pixelSet(122, 50);
    myTOLED.pixelSet(123, 50);
    myTOLED.pixelSet(124, 50);
    myTOLED.pixelSet(125, 50);
    myTOLED.pixelSet(20, 51);
    myTOLED.pixelSet(21, 51);
    myTOLED.pixelSet(22, 51);
    myTOLED.pixelSet(23, 51);
    myTOLED.pixelSet(20, 52);
    myTOLED.pixelSet(21, 52);
    myTOLED.pixelSet(22, 52);
    myTOLED.pixelSet(23, 52);
    myTOLED.pixelSet(20, 53);
    myTOLED.pixelSet(21, 53);
    myTOLED.pixelSet(22, 53);
    myTOLED.pixelSet(23, 53);
    myTOLED.pixelSet(20, 54);
    myTOLED.pixelSet(21, 54);
    myTOLED.pixelSet(22, 54);
    myTOLED.pixelSet(23, 54);
    myTOLED.pixelSet(20, 55);
    myTOLED.pixelSet(21, 55);
    myTOLED.pixelSet(22, 55);
    myTOLED.pixelSet(23, 55);
    myTOLED.pixelSet(20, 56);
    myTOLED.pixelSet(21, 56);
    myTOLED.pixelSet(22, 56);
    myTOLED.pixelSet(23, 56);
    myTOLED.pixelSet(21, 57);
    myTOLED.pixelSet(22, 57);
    myTOLED.pixelSet(23, 57);
    myTOLED.pixelSet(22, 58);
    myTOLED.pixelSet(23, 58);
    myTOLED.pixelSet(23, 59);

    for (uint8_t indi = 0; indi < 254; indi++) {
        myTOLED.setContrastControl(indi);
        delay(5);
    }
    delay(5000);
    for (uint8_t indi = 255; indi > 1; indi--) {
        myTOLED.setContrastControl(indi);
        delay(5);
    }
    myTOLED.setContrastControl(0);
    myTOLED.clearDisplay();
    myTOLED.setContrastControl(128);
}
#endif
