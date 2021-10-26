

// Meter colour schemes
#define RED2RED 0
#define GREEN2GREEN 1
#define BLUE2BLUE 2
#define BLUE2RED 3
#define GREEN2RED 4
#define RED2GREEN 5

#define TFT_GREY 0x2104 // Dark grey 16 bit colour

#define _TASK_SLEEP_ON_IDLE_RUN
#define _TASK_PRIORITY
#define _TASK_WDT_IDS
#define _TASK_TIMECRITICAL


#define WIFI_AP ""
#define WIFI_PASSWORD ""

#include <ADS1X15.h>

#include <Adafruit_MLX90614.h>


#include <WiFi.h>

#include <DNSServer.h>
#include <WiFiManager.h>
#include <WiFiClientSecure.h>
#include <TaskScheduler.h>
#include <PubSubClient.h>


#include <ArduinoOTA.h>


#include "Alert.h" // Out of range alert icon
#include "wifi1.h" // Signal Streng WiFi
#include "wifi2.h" // Signal Streng WiFi
#include "wifi3.h" // Signal Streng WiFi
#include "wifi4.h" // Signal Streng WiFi
#include "isolation.h" // Signal Streng WiFi
#include <TFT_eSPI.h> // Hardware-specific library
#include <SPI.h>


#include <BME280I2C.h>
#include <Wire.h>

#include "Free_Fonts.h" // Include the header file attached to this sketch


#include "Adafruit_SGP30.h"
#include "Logo.h"
#include "Splash2.h"

#include "RTClib.h"
#include "Free_Fonts.h"
#include "EEPROM.h"

#define HOSTNAME "GreenIO"
#define PASSWORD "green7650"

int PORT = 8883;
String deviceToken = "";
uint64_t chipId = 0;

String wifiName = "@IS01";
float DP = 0.0;
unsigned long ms;
struct Settings
{
  char TOKEN[40] = "";
  char SERVER[40] = "mqtt.thingcontrol.io";
  int PORT = 8883;
  uint32_t ip;
} sett;



//unsigned long drawTime = 0;
TFT_eSPI tft = TFT_eSPI(); // Invoke custom library with default width and height
TFT_eSprite tempSprite = TFT_eSprite(&tft); // Invoke custom library with default width and height
TFT_eSprite co2Sprite = TFT_eSprite(&tft); // Invoke custom library with default width and height
TFT_eSprite humSprite = TFT_eSprite(&tft); // Invoke custom library with default width and height
TFT_eSprite headerSprite = TFT_eSprite(&tft); // Invoke custom library with default width and height
TFT_eSprite wifiSprite = TFT_eSprite(&tft); // Invoke custom library with default width and height
TFT_eSprite timeSprite = TFT_eSprite(&tft); // Invoke custom library with default width and height
TFT_eSprite updateSprite = TFT_eSprite(&tft); // Invoke custom library with default width and height




// Instantiate eeprom objects with parameter/argument names and sizes

EEPROMClass  TVOCBASELINE("eeprom1", 0x200);
EEPROMClass  eCO2BASELINE("eeprom2", 0x100);



ADS1115 ads;  /* Use this for the 16-bit version */

int16_t adc0, adc1, adc2, adc3;
float val0, val1, val2, val3;
Scheduler runner;

BME280I2C bme;    // Default : forced mode, standby time = 1000 ms

String json = "";


WiFiClientSecure wifiClient;
PubSubClient client(wifiClient);


Adafruit_MLX90614 mlx = Adafruit_MLX90614();


int rssi = 0; ;

float temp(NAN), hum(NAN), pres(NAN);
Adafruit_SGP30 sgp;
uint32_t getAbsoluteHumidity(float temperature, float humidity) {
  // approximation formula from Sensirion SGP30 Driver Integration chapter 3.15
  const float absoluteHumidity = 216.7f * ((humidity / 100.0f) * 6.112f * exp((17.62f * temperature) / (243.12f + temperature)) / (273.15f + temperature)); // [g/m^3]
  const uint32_t absoluteHumidityScaled = static_cast<uint32_t>(1000.0f * absoluteHumidity); // [mg/m^3]
  return absoluteHumidityScaled;
}
//
//// Update these with values suitable for your network.
//
#define tvoc_topic "sensor/tvoc"
#define eco2_topic "sensor/eco2"


// # Add On
#include <TimeLib.h>
#include <ArduinoJson.h>
#include "time.h"

const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 3600 * 7;

const int   daylightOffset_sec = 3600;

bool connectWifi = false;
StaticJsonDocument<400> doc;
bool validEpoc = false;
String dataJson = "";
unsigned long _epoch = 0;
WiFiManager wifiManager;
unsigned long time_s = 0;

struct tm timeinfo;




int reading = 0; // Value to be displayed
int d = 0; // Variable used for the sinewave test waveform
boolean range_error = 0;
int8_t ramp = 1;

#define TEMP_WIDTH  120
#define TEMP_HEIGHT 50

#define CO2_WIDTH  260
#define CO2_HEIGHT 50

#define HUM_WIDTH  120
#define HUM_HEIGHT 50


#define HEADER_WIDTH  270
#define HEADER_HEIGHT 30

#define TIME_WIDTH  130
#define TIME_HEIGHT 55

#define WiFi_WIDTH  30
#define WiFi_HEIGHT 30


// Pause in milliseconds between screens, change to 0 to time font rendering
#define WAIT 100

// Callback methods prototypes
void tCallback();
void t1CallGetProbe();
void t2CallShowEnv();
void t3CallSendData();
//void t4CallPrintPMS7003();
void t5CallSendAttribute();
void t7showTime();
// Tasks
Task t1(2000, TASK_FOREVER, &t1CallGetProbe);  //adding task to the chain on creation
Task t2(5000, TASK_FOREVER, &t2CallShowEnv);
Task t3(60000, TASK_FOREVER, &t3CallSendData);
//
//Task t4(2000, TASK_FOREVER, &t4CallPrintPMS7003);  //adding task to the chain on creation
Task t5(10400000, TASK_FOREVER, &t5CallSendAttribute);  //adding task to the chain on creation
Task t7(60000, TASK_FOREVER, &t7showTime);

//flag for saving data
bool shouldSaveConfig = false;


class IPAddressParameter : public WiFiManagerParameter
{
  public:
    IPAddressParameter(const char *id, const char *placeholder, IPAddress address)
      : WiFiManagerParameter("")
    {
      init(id, placeholder, address.toString().c_str(), 16, "", WFM_LABEL_BEFORE);
    }

    bool getValue(IPAddress &ip)
    {
      return ip.fromString(WiFiManagerParameter::getValue());
    }
};

class IntParameter : public WiFiManagerParameter
{
  public:
    IntParameter(const char *id, const char *placeholder, long value, const uint8_t length = 10)
      : WiFiManagerParameter("")
    {
      init(id, placeholder, String(value).c_str(), length, "", WFM_LABEL_BEFORE);
    }

    long getValue()
    {
      return String(WiFiManagerParameter::getValue()).toInt();
    }
};



char  char_to_byte(char c)
{
  if ((c >= '0') && (c <= '9'))
  {
    return (c - 0x30);
  }
  if ((c >= 'A') && (c <= 'F'))
  {
    return (c - 55);
  }
}

String read_String(char add)
{
  int i;
  char data[100]; //Max 100 Bytes
  int len = 0;
  unsigned char k;
  k = EEPROM.read(add);
  while (k != '\0' && len < 500) //Read until null character
  {
    k = EEPROM.read(add + len);
    data[len] = k;
    len++;
  }
  data[len] = '\0';

  return String(data);
}


void getChipID() {
  chipId = ESP.getEfuseMac(); //The chip ID is essentially its MAC address(length: 6 bytes).
  Serial.printf("ESP32ChipID=%04X", (uint16_t)(chipId >> 32)); //print High 2 bytes
  Serial.printf("%08X\n", (uint32_t)chipId); //print Low 4bytes.

}

void setupOTA()
{
  //Port defaults to 8266
  //ArduinoOTA.setPort(8266);

  //Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname(HOSTNAME);

  //No authentication by default
  ArduinoOTA.setPassword(PASSWORD);

  //Password can be set with it's md5 value as well
  //MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  //ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA.onStart([]()
  {
    Serial.println("Start Updating....");

    Serial.printf("Start Updating....Type:%s\n", (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem");
  });

  ArduinoOTA.onEnd([]()
  {

    Serial.println("Update Complete!");

    ESP.restart();
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
  {
    String pro = String(progress / (total / 100)) + "%";
    int progressbar = (progress / (total / 100));
    //int progressbar = (progress / 5) % 100;
    //int pro = progress / (total / 100);

    drawUpdate(progressbar, 430, 290);


    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });

  ArduinoOTA.onError([](ota_error_t error)
  {
    Serial.printf("Error[%u]: ", error);
    String info = "Error Info:";
    switch (error)
    {
      case OTA_AUTH_ERROR:
        info += "Auth Failed";
        Serial.println("Auth Failed");
        break;

      case OTA_BEGIN_ERROR:
        info += "Begin Failed";
        Serial.println("Begin Failed");
        break;

      case OTA_CONNECT_ERROR:
        info += "Connect Failed";
        Serial.println("Connect Failed");
        break;

      case OTA_RECEIVE_ERROR:
        info += "Receive Failed";
        Serial.println("Receive Failed");
        break;

      case OTA_END_ERROR:
        info += "End Failed";
        Serial.println("End Failed");
        break;
    }


    Serial.println(info);
    ESP.restart();
  });

  ArduinoOTA.begin();
}

String uint64ToString(uint64_t input) {
  String result = "";
  uint8_t base = 10;

  do {
    char c = input % base;
    input /= base;

    if (c < 10)
      c += '0';
    else
      c += 'A' - 10;
    result = c + result;
  } while (input);
  return result;
}


void setupWIFI()
{
  WiFi.setHostname(uint64ToString(chipId).c_str());

  //等待5000ms，如果没有连接上，就继续往下
  //不然基本功能不可用
  byte count = 0;
  while (WiFi.status() != WL_CONNECTED && count < 10)
  {
    count ++;
    delay(500);
    Serial.print(".");
  }


  if (WiFi.status() == WL_CONNECTED)
    Serial.println("Connecting...OK.");
  else
    Serial.println("Connecting...Failed");

}

void writeString(char add, String data)
{
  int _size = data.length();
  int i;
  for (i = 0; i < _size; i++)
  {
    EEPROM.write(add + i, data[i]);
  }
  EEPROM.write(add + _size, '\0'); //Add termination null character for String Data
  EEPROM.commit();
}

void _writeEEPROM(String data) {
  //Serial.print("Writing Data:");
  //Serial.println(data);
  writeString(10, data);  //Address 10 and String type data
  delay(10);
}


void splash() {
  int xpos =  0;
  int ypos = 40;
  tft.init();
  // Swap the colour byte order when rendering
  tft.setSwapBytes(true);
  tft.setRotation(1);  // landscape

  tft.fillScreen(TFT_BLACK);
  // Draw the icons
  tft.pushImage(tft.width() / 2 - logoWidth / 2, 39, logoWidth, logoHeight, logo);
  tft.setTextColor(TFT_WHITE);
  tft.setTextDatum(TC_DATUM); // Centre text on x,y position

  tft.setFreeFont(FSB12);
  xpos = tft.width() / 2; // Half the screen width
  ypos = 160;
  tft.drawString("Isolation Room Monitoring", xpos, ypos, GFXFF);  // Draw the text string in the selected GFX free font
  tft.drawString("GreenIO", xpos, ypos + 60, GFXFF); // Draw the text string in the selected GFX free font


  delay(4000);
  tft.setTextColor(TFT_BLACK);
  tft.setFreeFont(FSB9);
  //  tft.drawString(nccidStr, xpos, ypos+20, GFXFF);
  //  tft.drawString(imsiStr, xpos, ypos+40, GFXFF);
  delay(3000);

  tft.setTextFont(GLCD);
  tft.setRotation(1);
  tft.fillScreen(TFT_WHITE);
  // Select the font
  ypos += tft.fontHeight(GFXFF);                      // Get the font height and move ypos down
  tft.setFreeFont(FSB9);
  tft.pushImage(tft.width() / 2 - (isolationWidth / 2), 10, isolationWidth, isolationHeight, isolation);



  delay(5000);
  tft.setTextPadding(180);
  tft.setTextColor(TFT_GREEN);
  tft.setTextDatum(MC_DATUM);
  Serial.println(F("Start..."));
  for ( int i = 0; i < 250; i++)
  {
    tft.drawString(".", 1 + 2 * i, 300, GFXFF);
    delay(10);
    //    Serial.println(i);
  }
  Serial.println("end");
}


void _initLCD() {
  tft.fillScreen(TFT_BLACK);
  // TFT
  splash();
  // MLX
  mlx.begin();
}

void _initBME280()
{
  while (!Serial) {} // Wait

  delay(200);

  Wire.begin(21, 22);

  while (!bme.begin())
  {
    Serial.println("Could not find BME280 sensor!");
    delay(1000);
  }

  // bme.chipID(); // Deprecated. See chipModel().
  switch (bme.chipModel())
  {
    case BME280::ChipModel_BME280:
      //      Serial.println(F("Found BME280 sensor! Success."));
      break;
    case BME280::ChipModel_BMP280:
      //      Serial.println(F("Found BMP280 sensor! No Humidity available."));
      break;
    default:
      Serial.println(F("Found UNKNOWN sensor! Error!"));
  }
}

void calibrate() {
  uint16_t readTvoc = 0;
  uint16_t readCo2 = 0;
  Serial.println("Done Calibrate");
  TVOCBASELINE.get(0, readTvoc);
  eCO2BASELINE.get(0, readCo2);

  //  Serial.println("Calibrate");
  Serial.print("****Baseline values: eCO2: 0x"); Serial.print(readCo2, HEX);
  Serial.print(" & TVOC: 0x"); Serial.println(readTvoc, HEX);
  sgp.setIAQBaseline(readCo2, readTvoc);
}
void _initSGP30 () {
  if (! sgp.begin()) {
    Serial.println("Sensor not found :(");
    while (1);
  }
  Serial.print("Found SGP30 serial #");
  Serial.print(sgp.serialnumber[0], HEX);
  Serial.print(sgp.serialnumber[1], HEX);
  Serial.println(sgp.serialnumber[2], HEX);

  calibrate();
}



void configModeCallback (WiFiManager *myWiFiManager) {
  Serial.println("Entered config mode");
  Serial.println(WiFi.softAPIP());
  //if you used auto generated SSID, print it
  Serial.println(myWiFiManager->getConfigPortalSSID());
}

void initBaseLine() {
  //  Serial.println("Testing EEPROMClass\n");

  if (!TVOCBASELINE.begin(TVOCBASELINE.length())) {
    //    Serial.println("Failed to initialise eCO2BASELINE");
    //    Serial.println("Restarting...");
    delay(1000);
    ESP.restart();
  }
  if (!eCO2BASELINE.begin(eCO2BASELINE.length())) {
    //    Serial.println("Failed to initialise eCO2BASELINE");
    //    Serial.println("Restarting...");
    delay(1000);
    ESP.restart();
  }

}

void saveConfigCallback () {
  Serial.println("Should save config");
  shouldSaveConfig = true;
  Serial.print("saveConfigCallback:");
  Serial.println(sett.TOKEN);
}



void setup(void) {

  Serial.begin(115200);

  EEPROM.begin(512);
  _initLCD();
  if (EEPROM.read(10) == 255 ) {
    _writeEEPROM("147.50.151.130");
  }
  getChipID();

  tft.fillScreen(TFT_BLACK);
  tft.drawString("Wait for WiFi Setting (Timeout 120 Sec)", tft.width() / 2, tft.height() / 2, GFXFF);
  wifiManager.setTimeout(120);

  wifiManager.setAPCallback(configModeCallback);
  std::vector<const char *> menu = {"wifi", "info", "sep", "restart", "exit"};
  wifiManager.setMenu(menu);
  wifiManager.setClass("invert");
  wifiManager.setConfigPortalTimeout(180); // auto close configportal after n seconds
  wifiManager.setAPClientCheck(true); // avoid timeout if client connected to softap
  wifiManager.setBreakAfterConfig(true);   // always exit configportal even if wifi save fails

  WiFiManagerParameter blnk_Text("<b>Isolation Room setup.</b> <br>");
  sett.TOKEN[39] = '\0';   //add null terminator at the end cause overflow
  sett.SERVER[39] = '\0';   //add null terminator at the end cause overflow
  WiFiManagerParameter blynk_Token( "blynktoken", "device Token",  sett.TOKEN, 40);
  WiFiManagerParameter blynk_Server( "blynkserver", "Server",  sett.SERVER, 40);
  IntParameter blynk_Port( "blynkport", "Port",  sett.PORT);

  wifiManager.addParameter( &blnk_Text );
  wifiManager.addParameter( &blynk_Token );
  wifiManager.addParameter( &blynk_Server );
  wifiManager.addParameter( &blynk_Port );


  //set config save notify callback
  wifiManager.setSaveConfigCallback(saveConfigCallback);




  wifiName.concat(uint64ToString(chipId));
  if (!wifiManager.autoConnect(wifiName.c_str())) {
    Serial.println("failed to connect and hit timeout");
    //    ESP.reset();
    //delay(1000);
  }
  deviceToken = wifiName.c_str();


  configTime(gmtOffset_sec, 0, ntpServer);
  client.setServer( sett.SERVER, sett.PORT );


  setupWIFI();
  setupOTA();

  _initBME280();

  _initSGP30();
  initBaseLine();
  runner.init();
  Serial.println("Initialized scheduler");

  runner.addTask(t1);
  //  Serial.println("added t1");
  runner.addTask(t2);
  //  Serial.println("added t2");
  runner.addTask(t3);
  //  Serial.println("added t3");
  //  runner.addTask(t4);
  //  Serial.println("added t4");
  //  runner.addTask(t6);
  runner.addTask(t7);
  delay(2000);

  t1.enable();
  //  Serial.println("Enabled t1");
  t2.enable();
  //  Serial.println("Enabled t2");
  t3.enable();
  //  Serial.println("Enabled t3");
  //  t4.enable();
  //  Serial.println("Enabled t4");
  //  t6.enable();
  t7.enable();

  //  Serial.println("Initialized scheduler");
  tft.begin();

  tft.setRotation(1);
  tempSprite.createSprite(TEMP_WIDTH, TEMP_HEIGHT);
  tempSprite.fillSprite(TFT_BLACK);
  co2Sprite.createSprite(CO2_WIDTH, CO2_HEIGHT);
  co2Sprite.fillSprite(TFT_BLACK);
  humSprite.createSprite(HUM_WIDTH, HUM_HEIGHT);
  humSprite.fillSprite(TFT_BLACK);
  headerSprite.createSprite(HEADER_WIDTH, HEADER_HEIGHT);
  headerSprite.fillSprite(TFT_BLACK);
  wifiSprite.createSprite(WiFi_WIDTH, WiFi_HEIGHT);
  wifiSprite.fillSprite(TFT_BLACK);
  timeSprite.createSprite(TIME_WIDTH, TIME_HEIGHT);
  timeSprite.fillSprite(TFT_BLACK);

  tft.fillScreen(TFT_BLACK);

  drawTime();
  ads.begin();
  header("Isolation Room Monitor");
}


void printBME280Data()
{
  _initBME280();
  BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
  BME280::PresUnit presUnit(BME280::PresUnit_Pa);
  bme.read(pres, temp, hum, tempUnit, presUnit);
  temp = temp - 3;
}



void loop() {

  ArduinoOTA.handle();
  runner.execute();
  ms = millis();
  if (ms % 1440000 == 0)
  {

    Serial.println("Attach WiFi for，OTA "); Serial.println(WiFi.RSSI() );

    setupWIFI();

    setupOTA();
  }
}
String a0(int n) {
  return (n < 10) ? "0" + String(n) : String(n);
}

void drawTime() {
  unsigned long NowTime = _epoch + ((millis() - time_s) / 1000) + (7 * 3600);
  String dateS = "";
  String timeS = "";

  if (!getLocalTime(&timeinfo)) {
    //Serial.println("Failed to obtain time");
    return;
  }
  dateS = a0(timeinfo.tm_mday) + "/" + a0(timeinfo.tm_mon + 1) + "/" + String(timeinfo.tm_year + 1900);
  timeS = a0(timeinfo.tm_hour) + ":" + a0(timeinfo.tm_min) ;
  timeSprite.fillScreen(TFT_BLACK);
  timeSprite.setTextSize(1);
  timeSprite.setTextColor(TFT_ORANGE);
  timeSprite.drawString(dateS, 0, 2, 4); // Font 4 for fast drawing with background
  timeSprite.drawString(timeS, 60, 25, 4); // Font 4 for fast drawing with background
  timeSprite.pushSprite(315, 0);

}
int roundNumber(float n) {
  return round(n);
}
void drawUpdate(int num, int x, int y)
{
  updateSprite.createSprite(60, 20);
  updateSprite.fillScreen(TFT_BLACK);
  updateSprite.setFreeFont(FSB9);
  updateSprite.setTextColor(TFT_ORANGE);
  updateSprite.setTextSize(1);
  updateSprite.drawNumber(num, 0, 3);
  updateSprite.drawString("%", 27, 3, GFXFF);
  updateSprite.pushSprite(x, y);
  updateSprite.deleteSprite();
}

void drawTemp() {

  tempSprite.fillScreen(TFT_BLACK);
  tempSprite.setTextColor(TFT_YELLOW);

  tempSprite.setTextDatum(TC_DATUM); // Centre text on x,y position

  int xpos = tempSprite.width() / 2; // Half the screen width
  int ypos = 280;

  tempSprite.setFreeFont(FSB9);                              // Select the font
  tempSprite.drawString("T:", 0, 0, GFXFF);
  tempSprite.drawNumber(roundNumber(temp), 60, 0, 7);
  tempSprite.fillCircle(95, 2, 3, TFT_YELLOW);
  tempSprite.drawString("C", 105, 0, GFXFF);


  tempSprite.pushSprite(0, 220); delay(WAIT);
}


void drawCO2() {

  co2Sprite.fillScreen(TFT_BLACK);
  co2Sprite.setTextColor(TFT_WHITE, TFT_BLACK);
  co2Sprite.setTextDatum(TC_DATUM); // Centre text on x,y position
  int xpos = co2Sprite.width() / 2; // Half the screen width
  int ypos = 280;
  co2Sprite.setFreeFont(FSB9);                              // Select the font
  //  tft.drawString("Serif Bold 9pt", xpos, ypos, GFXFF);  // Draw the text string in the selected GFX free font
  //  ypos += tft.fontHeight(GFXFF);                      // Get the font height and move ypos down

  //  tft.setFreeFont(FSB12);
  //  tft.drawString("Serif Bold 12pt", xpos, ypos, GFXFF);
  //  ypos += tft.fontHeight(GFXFF);

  co2Sprite.setFreeFont(FSB12);
  co2Sprite.drawString("CO2:", 0, 0, GFXFF);
  String co2 = String(sgp.eCO2);
  co2Sprite.drawString( co2, 135, 0, 7);
  co2Sprite.drawString("ppm", 240, 0, GFXFF);


  co2Sprite.pushSprite(xpos, 270); delay(WAIT);
}

void drawHum() {

  humSprite.fillScreen(TFT_BLACK);
  humSprite.setTextColor(TFT_BLUE);

  humSprite.setTextDatum(TC_DATUM); // Centre text on x,y position

  int xpos = humSprite.width() / 2; // Half the screen width
  int ypos = 280;

  humSprite.setFreeFont(FSB9);                              // Select the font
  humSprite.drawString("H:", 0, 0, GFXFF);
  humSprite.drawNumber(roundNumber(hum), 55, 0, 7);
  humSprite.drawString("%", 95, 0, GFXFF);
  humSprite.pushSprite(370, 220);
}

// #########################################################################
//  Draw the meter on the screen, returns x coord of righthand side
// #########################################################################
int ringMeter(float value, int vmin, int vmax, int x, int y, int r, const char *units, byte scheme)
{
  // Minimum value of r is about 52 before value text intrudes on ring
  // drawing the text first is an option

  x += r; y += r;   // Calculate coords of centre of ring

  int w = r / 3;    // Width of outer ring is 1/4 of radius

  int angle = 150;  // Half the sweep angle of meter (300 degrees)

  int v = map(value, vmin, vmax, -angle, angle); // Map the value to an angle v

  byte seg = 3; // Segments are 3 degrees wide = 100 segments for 300 degrees
  byte inc = 6; // Draw segments every 3 degrees, increase to 6 for segmented ring

  // Variable to save "value" text colour from scheme and set default
  int colour = TFT_BLUE;

  // Draw colour blocks every inc degrees
  for (int i = -angle + inc / 2; i < angle - inc / 2; i += inc) {
    // Calculate pair of coordinates for segment start
    float sx = cos((i - 90) * 0.0174532925);
    float sy = sin((i - 90) * 0.0174532925);
    uint16_t x0 = sx * (r - w) + x;
    uint16_t y0 = sy * (r - w) + y;
    uint16_t x1 = sx * r + x;
    uint16_t y1 = sy * r + y;

    // Calculate pair of coordinates for segment end
    float sx2 = cos((i + seg - 90) * 0.0174532925);
    float sy2 = sin((i + seg - 90) * 0.0174532925);
    int x2 = sx2 * (r - w) + x;
    int y2 = sy2 * (r - w) + y;
    int x3 = sx2 * r + x;
    int y3 = sy2 * r + y;

    if (i < v) { // Fill in coloured segments with 2 triangles
      switch (scheme) {
        case 0: colour = TFT_RED; break; // Fixed colour
        case 1: colour = TFT_GREEN; break; // Fixed colour
        case 2: colour = TFT_BLUE; break; // Fixed colour
        case 3: colour = rainbow(map(i, -angle, angle, 0, 127)); break; // Full spectrum blue to red
        case 4: colour = rainbow(map(i, -angle, angle, 70, 127)); break; // Green to red (high temperature etc)
        case 5: colour = rainbow(map(i, -angle, angle, 127, 63)); break; // Red to green (low battery etc)
        default: colour = TFT_BLUE; break; // Fixed colour
      }
      tft.fillTriangle(x0, y0, x1, y1, x2, y2, colour);
      tft.fillTriangle(x1, y1, x2, y2, x3, y3, colour);
      //text_colour = colour; // Save the last colour drawn
    }
    else // Fill in blank segments
    {
      tft.fillTriangle(x0, y0, x1, y1, x2, y2, TFT_GREY);
      tft.fillTriangle(x1, y1, x2, y2, x3, y3, TFT_GREY);
    }
  }
  // Convert value to a string
  char buf[10];
  byte len = 3; if (value > 9) len = 5;
  //  if (value != 0) {
  //    value = value - (value * 2);
  //  }
  DP = value;
  Serial.println(value);
  dtostrf(value, len, 1, buf);
  buf[len] = ' '; buf[len + 1] = 0; // Add blanking space and terminator, helps to centre text too!
  // Set the text colour to default
  tft.setTextSize(1);
  //  Serial.println(value);
  //  Serial.println(vmin);
  //  Serial.println(vmax);
  if (value <= 3 || value >= 20) {
    drawAlert(x, y + 90, 50, 1);
  }
  else {
    drawAlert(x, y + 90, 50, 0);
  }

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  // Uncomment next line to set the text colour to the last segment value!
  tft.setTextColor(colour, TFT_BLACK);
  tft.setTextDatum(MC_DATUM);
  // Print value, if the meter is large then use big font 8, othewise use 4
  if (r > 84) {
    tft.setTextPadding(35 * 3); // Allow for 3 digits each 30 pixels wide
    tft.drawString(buf, x, y, 7); // Value in middle
  }
  else {
    tft.setTextPadding(3 * 14); // Allow for 3 digits each 14 pixels wide
    tft.drawString(buf, x, y, 7); // Value in middle
  }
  tft.setTextSize(1);
  tft.setTextPadding(0);
  // Print units, if the meter is large then use big font 4, othewise use 2
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  if (r > 84) tft.drawString(units, x, y + 60, 4); // Units display
  else tft.drawString(units, x, y + 15, 2); // Units display

  // Calculate and return right hand side x coordinate

  return x + r;
}

void drawAlert(int x, int y , int side, boolean draw)
{
  if (draw && !range_error) {
    drawIcon(alert, x - alertWidth / 2, y - alertHeight / 2, alertWidth, alertHeight);
    range_error = 1;
  }
  else if (!draw) {
    tft.fillRect(x - alertWidth / 2, y - alertHeight / 2, alertWidth, alertHeight, TFT_BLACK);
    range_error = 0;
  }
}

// #########################################################################
// Return a 16 bit rainbow colour
// #########################################################################
unsigned int rainbow(byte value)
{
  // Value is expected to be in range 0-127
  // The value is converted to a spectrum colour from 0 = blue through to 127 = red

  byte red = 0; // Red is the top 5 bits of a 16 bit colour value
  byte green = 0;// Green is the middle 6 bits
  byte blue = 0; // Blue is the bottom 5 bits

  byte quadrant = value / 32;

  if (quadrant == 0) {
    blue = 31;
    green = 2 * (value % 32);
    red = 0;
  }
  if (quadrant == 1) {
    blue = 31 - (value % 32);
    green = 63;
    red = 0;
  }
  if (quadrant == 2) {
    blue = 0;
    green = 63;
    red = value % 32;
  }
  if (quadrant == 3) {
    blue = 0;
    green = 63 - 2 * (value % 32);
    red = 31;
  }
  return (red << 11) + (green << 5) + blue;
}


//====================================================================================
// This is the function to draw the icon stored as an array in program memory (FLASH)
//====================================================================================

// To speed up rendering we use a 64 pixel buffer
#define BUFF_SIZE 64

// Draw array "icon" of defined width and height at coordinate x,y
// Maximum icon size is 255x255 pixels to avoid integer overflow

void drawIcon(const unsigned short* icon, int16_t x, int16_t y, int8_t width, int8_t height) {

  uint16_t  pix_buffer[BUFF_SIZE];   // Pixel buffer (16 bits per pixel)

  tft.startWrite();

  // Set up a window the right size to stream pixels into
  tft.setAddrWindow(x, y, width, height);

  // Work out the number whole buffers to send
  uint16_t nb = ((uint16_t)height * width) / BUFF_SIZE;

  // Fill and send "nb" buffers to TFT
  for (int i = 0; i < nb; i++) {
    for (int j = 0; j < BUFF_SIZE; j++) {
      pix_buffer[j] = pgm_read_word(&icon[i * BUFF_SIZE + j]);
    }
    tft.pushColors(pix_buffer, BUFF_SIZE);
  }

  // Work out number of pixels not yet sent
  uint16_t np = ((uint16_t)height * width) % BUFF_SIZE;

  // Send any partial buffer left over
  if (np) {
    for (int i = 0; i < np; i++) pix_buffer[i] = pgm_read_word(&icon[nb * BUFF_SIZE + i]);
    tft.pushColors(pix_buffer, np);
  }

  tft.endWrite();
}

// Print the header for a display screen
void header(const char *string)
{
  headerSprite.setTextSize(1);

  ////  headerSprite.setFreeFont(FSB4);
  headerSprite.setTextColor(TFT_MAGENTA);
  //  //  headerSprite.fillRect(0, 0, 480, 30, TFT_BLUE);
  //  headerSprite.setTextDatum(TC_DATUM);
  headerSprite.drawString(string, 5, 2, 4); // Font 4 for fast drawing with background
  //  headerSprite.drawString(string, 0, 2, 4); // Font 4 for fast drawing with background
  //  //  headerSprite.pushImage(445 , 2, wifi1Width, wifi1Height, wifi2);
  headerSprite.pushSprite(0, 0);

  //  headerSprite.setCursor(0, 0);    // Set cursor near top left corner of screen
  //
  //  headerSprite.setTextFont(GLCD);     // Select the orginal small GLCD font by using NULL or GLCD
  //  headerSprite.println();             // Move cursor down a line
  //  headerSprite.print("Original GLCD font");    // Print the font name onto the TFT screen
  //  headerSprite.println();
  //  headerSprite.println();
  //
  //  headerSprite.setFreeFont(FSB9);   // Select Free Serif 9 point font, could use:
  //  // tft.setFreeFont(&FreeSerif9pt7b);
  //  headerSprite.println();          // Free fonts plot with the baseline (imaginary line the letter A would sit on)
  //  // as the datum, so we must move the cursor down a line from the 0,0 position
  //  headerSprite.print(string);  // Print the font name onto the TFT screen
  // headerSprite.pushSprite(0, 0); delay(WAIT);


}

void drawWiFi( )
{

  wifiSprite.fillScreen(TFT_BLACK);
  Serial.print("RSSI:");
  Serial.println(rssi);
  if (rssi == 1)
    wifiSprite.pushImage(0 , 0, wifi1Width, wifi1Height, wifi4);
  if (rssi == 2)
    wifiSprite.pushImage(0 , 0, wifi1Width, wifi1Height, wifi3);
  if (rssi == 3)
    wifiSprite.pushImage(0 , 0, wifi1Width, wifi1Height, wifi2);
  if (rssi >= 4)
    wifiSprite.pushImage(0 , 0, wifi1Width, wifi1Height, wifi1);

  wifiSprite.pushSprite(445, 0);
}


// Draw a + mark centred on x,y
void drawDatum(int x, int y)
{
  tft.drawLine(x - 5, y, x + 5, y, TFT_GREEN);
  tft.drawLine(x, y - 5, x, y + 5, TFT_GREEN);
}

void composeJson() {


  json = "";

  json.concat(" {\"temp\":");

  json.concat(temp);
  json.concat(",\"hum\":");
  json.concat(hum);
  json.concat(",\"pres\":");
  json.concat(pres);
  json.concat(",\"DP\":");
  json.concat(DP);

  json.concat(",\"co2\":");
  json.concat(sgp.eCO2);
  json.concat(",\"voc\":");
  json.concat(sgp.TVOC);

  json.concat(",\"rssi\":");
  json.concat(WiFi.RSSI());

  json.concat("}");
  Serial.println(json);



}


void getDataSGP30 () {
  // put your main code here, to run repeatedly:
  // If you have a temperature / humidity sensor, you can set the absolute humidity to enable the humditiy compensation for the air quality signals
  float temperature = temp - 3; // [°C]
  float humidity = hum; // [%RH]
  sgp.setHumidity(getAbsoluteHumidity(temperature, humidity));

  if (! sgp.IAQmeasure()) {
    Serial.println("Measurement failed");
    return;
  }
  Serial.print("TVOC "); Serial.print(sgp.TVOC); Serial.print(" ppb\t");
  Serial.print("eCO2 "); Serial.print(sgp.eCO2); Serial.println(" ppm");


  if (! sgp.IAQmeasureRaw()) {
    Serial.println("Raw Measurement failed");
    return;
  }
  Serial.print("Raw H2 "); Serial.print(sgp.rawH2); Serial.print(" \t");
  Serial.print("Raw Ethanol "); Serial.print(sgp.rawEthanol); Serial.println("");



  uint16_t TVOC_base, eCO2_base;

  //  Serial.print("eCo2: ");   Serial.println(readCo2);
  //  Serial.print("voc: ");  Serial.println(readTvoc);

  //      sgp.setIAQBaseline(eCO2_base, TVOC_base);
  if (! sgp.getIAQBaseline(&eCO2_base, &TVOC_base)) {
    Serial.println("Failed to get baseline readings");
    return;
  }

  //  Serial.print("****Get Baseline values: eCO2: 0x"); Serial.print(eCO2_base, HEX);
  //  Serial.print(" & TVOC: 0x"); Serial.println(TVOC_base, HEX);
}


void tCallback() {
  Scheduler &s = Scheduler::currentScheduler();
  Task &t = s.currentTask();

  //  Serial.print("Task: "); Serial.print(t.getId()); Serial.print(":\t");
  //  Serial.print(millis()); Serial.print("\tStart delay = "); Serial.println(t.getStartDelay());
  //  delay(10);

  if (t1.isFirstIteration()) {
    runner.addTask(t2);
    t2.enable();
    //    Serial.println("t1CallgetProbe: enabled t2CallshowEnv and added to the chain");
  }


}
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  if ((in_max - in_min) + out_min != 0) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  } else {
    return 0;
  }
}



void getDataDP () {

  adc3 = ads.readADC(3);  
  val3 = mapfloat(adc3, 2096 , 9658, 0, 25);
  Serial.print("Read DP:\t");
  Serial.print(adc3); Serial.print(":");  Serial.println(val3);

}
void t1CallGetProbe() {
  tCallback();
  int xpos = 0, ypos = 0, gap = 0, radius = 0;
  xpos = 480 / 2 - 120, ypos = 30, gap = 55, radius = 120;
  reading += (ramp);
  if (reading > 0) ramp = -1;
  if (reading < 25) ramp = 26;
  // Comment out above meters, then uncomment the next line to show large meter
  ringMeter(val3, 0, 25, xpos, ypos, radius, " Pa", BLUE2RED); // Draw analogue meter

  printBME280Data();
  getDataSGP30();
  getDataDP();
}

void t2CallShowEnv() {
  drawWiFi();
  drawTemp();
  drawCO2();
  drawHum();
  Serial.print("deviceToken:");
  Serial.println(deviceToken);
}
void t3CallSendData() {
  composeJson();

  tft.setTextColor(0xFFFF);
  int mapX = 315;
  int mapY = 30;
  Serial.println(WL_CONNECTED); Serial.print("(WiFi.status():"); Serial.println(WiFi.status());

  rssi = map(WiFi.RSSI(), -100, -30, 1, 4);
  Serial.println(rssi);
  wifiSprite.setTextSize(1);
  wifiSprite.setTextColor(TFT_BLUE);
  //  headerSprite.fillRect(0, 0, 480, 30, TFT_BLUE);
  wifiSprite.pushImage(0 , 0, wifi1Width, wifi1Height, wifi4);
  wifiSprite.pushSprite(445, 0);



  if (client.connect("ISOLATIONROOM", deviceToken.c_str(), NULL)) {
    Serial.println("******************************************************Connected!");

    client.publish("v1/devices/me/telemetry", json.c_str());
  }

}
//void t4CallPrintPMS7003();
void t5CallSendAttribute() {}

void t7showTime() {


  drawTime();

}
