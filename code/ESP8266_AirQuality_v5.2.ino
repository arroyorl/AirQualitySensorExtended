/*
 *  This sketch runs an IoT for Air Quality and wind speed MQTT device.
 *  During the setup the device will act as AP mode so that you can set the
 *  Wifi connection, then it will try to connect to the WiFi. 
 *  - While serving the webserver it will also post
 *    the data to MQTT broker.
 */
///////////////////////////////////////////////////////////////
//           History                                         //
///////////////////////////////////////////////////////////////
//  0.1 first version using sds011
//      code based on measure.ino from Dirk O. Kaar <dok@dok-net.net>
//
//  0.1a updated sendDataWunderground()
//      eco mode working
//      added ADC_MODE(ADC_VCC) for read internal VCC
//      reviewed code for getBattLevel() (VCCINT)
//
//  0.2 added DHT22 to read Temp & Humid to be used in 
//      humidityCompensation funcion (based on hackAir github)
//
//  0.3 Moved SDS011 to GPIO12 and 14
//      included CCS811 gas sensor
//
//  0.4 Added Grove Multichannel Gas Sensor
//
//  0.4.1 added Weatherunderground PM2.5, PM10, NO2 and CO data
//
//  0.5 adjusted Vcc & batery level
//      added nominalBatt as nominal Batt (maximum) level
//      changed GetBattLevel to use nominalBatt
//
//  1.0 Added support for ThingSpeak
//
//  1.1 Using HTTP Get to submit ThingSpeak data
//
//  1.2 Back to ThingSpeak Library
//      error correction (when MQTT broker is down)
//
//  1.3 don't send Grove data if not defined
//
//  1.4 send dew point data to ThingSpeak (field 8)
//
//  1.5 activate/deactivate Grove sensor by configuration
//      show Vcc instead of Batt level in mainPage
//
//  1.5.1 reset device when temperature reading is NAN
//      send local IP by MQTT message
//
//  1.5.2 Initialize sensors after initialize WiFi
//      initialize DHT22 before SDS011  
//
//  1.5.3 Reset device every 24 hours
//
//  2.0 Added annemometer
//      changed DHT22 by BME280
//      removed echo mode
//
//  2.1 send windspeed data every n 5 secs. intervals
//      added maxwindspeed (max in send interval)
//
//  3.0 added TSL2561 luminosity sensor
//
//  4.0 added rain sensor
//      removed report of Vcc/battery (analog input is used for rain detector)
//
//  4.1 changed rain send frequency to 30 secs.
//      corrected an error in JSON message on handleRawData function
//
//  5.0 changed HTTPS access method to avoid fingerprint
//      removed fingerprint update web page
//
//  5.1 TSL2561 set to manual gain (1x) and 101 ms integration time
//
//  5.2 updated (corrected) formula for wind speed calculation
//      Wind speed as per https://pop.fsck.pl/hardware/wh-sp-ws01.html
//      is 0.34 * pulses/sec (in m/s)
//
///////////////////////////////////////////////////////////////


#include <ESP8266WiFi.h>    
#include <ESP8266WiFiMulti.h>
#include <ESP8266WebServer.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClientSecureBearSSL.h>

#include <PubSubClient.h>  // v 2.8

#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

#include <ThingSpeak.h>  // v 2.0.1

#define SENSORTYPE BME280
#define SENSORNAME "BME280"
#include <Wire.h>
#include <Adafruit_Sensor.h>  // Adafruit Unified Sensor v 1.1.12
#include <Adafruit_BME280.h>  // v 2.2.2
#include <Adafruit_TSL2561_U.h>  // Adafruit TSL 2561 v 1.1.0

#define FVERSION  "v5.2"

/////////////////SENSOR TYPE///////////////////////
// Uncomment whatever type you're using!         //
#define SDS011  // Air quality sensor
//#define CCS811  // CO2 sensor     
//#define GROVE   // Grove Multichannel Gas Sensor
///////////////////////////////////////////////////
// Comment for final release
#define RDEBUG
///////////////////////////////////////////////////

#include <SoftwareSerial.h> // EspSoftwareSerial v 8.1.0
#include <esp_sds011.h> // v 2.2.0

#ifdef CCS811
  #include <Adafruit_Sensor.h>  // Adafruit Unified Sensor v 1.1.12
  #include <Adafruit_CCS811.h>  // Adafruit CCS811 v 1.1.1
#endif

#ifdef GROVE
  #include <Wire.h>
  #include <MutichannelGasSensor.h> // arduino_916704 library v 1.0.0 (https://github.com/Seeed-Studio/Mutichannel_Gas_Sensor)
#endif

WiFiClient espClient;
PubSubClient client(espClient);

#define   NUM_SAMPLES 3
#define   TABLESIZE 20

#include "Rdebug.h"
#include "settings.h"
#include "mainPage.h"
#include "initPage.h"

#define GPIO0 0
#define GPIO1 1
#define GPIO2 2
#define GPIO3 3
#define GPIO4 4
#define GPIO5 5
#define GPIO6 6
#define GPIO7 7
#define GPIO8 8
#define GPIO9 9
#define GPIO10 10
#define GPIO11 11
#define GPIO12 12
#define GPIO13 13
#define GPIO14 14
#define GPIO15 15
#define GPIO16 16

#define LEDON LOW
#define LEDOFF HIGH
#define SETUP_PIN GPIO0
#define WAKEUP_PIN GPIO16


#define SDS_PIN_RX GPIO14  // D5
#define SDS_PIN_TX GPIO12  // D6
#define ANNEMOMETER_PIN GPIO13 // D7
#define I2C_SCL    GPIO5   // D1
#define I2C_SDA    GPIO4   // D2
#define RAIN_ANALOG A0     // A0

// Define Red and green LEDs
#ifdef ARDUINO_ESP8266_NODEMCU_ESP12E
  #define RED_LED GPIO12 // D6
  #define GREEN_LED GPIO10 // D12 - SD3
#endif
#ifdef ARDUINO_ESP8266_GENERIC
  #define RED_LED GPIO9 
  #define GREEN_LED GPIO10
#endif

ESP8266WebServer setupserver(80);
ESP8266WebServer server(80);

#ifdef ESP32
  HardwareSerial& serialSDS(Serial2);
  Sds011Async< HardwareSerial > sds011(serialSDS);
#else
  SoftwareSerial serialSDS;
  Sds011Async< SoftwareSerial > sds011(serialSDS);
#endif

//Struct to store setting in EEPROM
Settings    settings;

//Global Variables
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
int         inecomode = false; // is in eco mode?
String      data;
String      httpUpdateResponse;
const char* ap_ssid = "ESP-TEMP";
const char* ap_password = ""; //"12345678";
int         ap_setup_done = 0;
long        start;
long        timer;
long        timer2;
int         pm25_table[TABLESIZE];
int         pm10_table[TABLESIZE];
float       pm25f;
float       pm10f;
bool        is_SDS_running = true;
float       temperature=0;
float       humidity=0;
float       dewpoint=0;
float       pressure;
float       sealevelpressure;
float       tempccs811=0;
float       CO2ppm;
float       TVOCppb;
float       NH3ppm;
float       COppm;
float       NO2ppm;
float       C3H8ppm;
float       C4H10ppm;
float       CH4ppm;
float       H2ppm;
float       C2H5OHppm;
float       windspeed;
float       maxwindspeed=0.0;
float       sumwindspeed=0.0;
int         numsampleswind=0;
float       lux;
bool        TSL2561active = false;
bool        CCS811_active = false;
bool        send_TS = false;
int         rainA;
bool        rainD;
String      base_topic;
long        lastSecond; //The millis counter for wind
long        lastSecond2; // millis counter for rain 
rst_info    *myResetInfo;

long lastWindCheck = 0;
volatile long lastWindIRQ = 0;
volatile byte windClicks = 0;

Adafruit_BME280 bme; 
Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);

#ifdef CCS811
  Adafruit_CCS811 ccs;
#endif

void displayBusy(){
  digitalWrite(RED_LED, LEDON);
  delay(500);
  digitalWrite(RED_LED, LEDOFF);
  delay(500);
}

/**************************************************************************/
/*  Calculate Dew Point
/* reference: http://wahiduddin.net/calc/density_algorithms.htm
/**************************************************************************/
float computeDewPoint(float celsius, float humidity)
{
        double RATIO = 373.15 / (273.15 + celsius);  // RATIO was originally named A0, possibly confusing in Arduino context
        double SUM = -7.90298 * (RATIO - 1);
        SUM += 5.02808 * log10(RATIO);
        SUM += -1.3816e-7 * (pow(10, (11.344 * (1 - 1/RATIO ))) - 1) ;
        SUM += 8.1328e-3 * (pow(10, (-3.49149 * (RATIO - 1))) - 1) ;
        SUM += log10(1013.246);
        double VP = pow(10, SUM - 3) * humidity;
        double T = log(VP/0.61078);   // temp var
        return (241.88 * T) / (17.558 - T);
}

/**************************************************************************/
/*  Takes an average of readings on a given pin
/*  Returns the average
/**************************************************************************/
int averageAnalogRead(int pinToRead)
{
    byte numberOfReadings = 3;
    unsigned int runningValue = 0;

    for(int x = 0 ; x < numberOfReadings ; x++)
        runningValue += analogRead(pinToRead);
    runningValue /= numberOfReadings;

    return(runningValue);
}

/*************************************************
                  BME280
*************************************************/
void getBME280data() {

float runtemp=0;
float runhumid=0;
float runpress=0;

  DebugLn("get BME280 data");

  //////////////////////////////// BME280 ////////////////////////////////////////////////
  for(int i = 0 ; i < NUM_SAMPLES; i++){
    runtemp += bme.readTemperature();
    runhumid += bme.readHumidity();
    runpress += bme.readPressure();
  }

  temperature = (runtemp / NUM_SAMPLES) + settings.data.tempadjust;
  humidity = runhumid / NUM_SAMPLES;
  dewpoint = computeDewPoint(temperature, humidity);
  pressure = (runpress / NUM_SAMPLES) / 100.0F;
  sealevelpressure = bme.seaLevelForAltitude(settings.data.altitude, pressure);
  DebugLn("BME280 data read !");
  ////////////////////////////////////////////////////////////////////////////////

   if (isnan(humidity) || isnan(temperature) || isnan(pressure)) {
      DebugLn("Failed to read from sensor!");
   }

  
  DebugLn("Temp: "+String(temperature));
  DebugLn("Hum: "+String(humidity));
  DebugLn("DewPoint: "+String(dewpoint));
  DebugLn("Pres: " + String(pressure));
  DebugLn("SeaLevelPres: "+String(sealevelpressure));
}

/*************************************************
                  Annemometer
*************************************************/
//Interrupt routines (these are called by the hardware interrupts, not by the main code)
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void IRAM_ATTR wspeedIRQ()
// Activated by the magnet in the anemometer (2 ticks per rotation)
{
    if (millis() - lastWindIRQ > 15) // Ignore switch-bounce glitches less than 15ms (99.5 MPH max reading) after the reed switch closes
    {
        lastWindIRQ = millis(); //Grab the current time
        windClicks++; //There is 1.492MPH for each click per second.
    }
}


//Returns the instataneous wind speed
float get_wind_speed()
{
    float deltaTime = millis() - lastWindCheck; // elapsed time

    deltaTime /= 1000.0; //Convert to seconds

    float WSpeed = (float)windClicks / deltaTime; // clics/sec

    windClicks = 0; //Reset and start watching for new wind
    lastWindCheck = millis();

    WSpeed *= 0.34 * 3.6; // 0.34 m/s click * 3.6 = speed in Km/h

    return(WSpeed);
}

/*************************************************
          TSL 2561 luminosity sensor
*************************************************/
bool TSL2561setup()
{
  if(!tsl.begin())
  {
    // TSL2561 not detected
    DebugLn("No TSL2561 detected ... Check wiring or I2C ADDR !");
    return false;
  }

  // Display sensor details
  sensor_t sensor;
  tsl.getSensor(&sensor);
  DebugLn("***** Inicializing TSL2561 *****");
  DebugLn ("Sensor:       " + String(sensor.name));
  DebugLn ("Driver Ver:   " + String(sensor.version));
  DebugLn ("Unique ID:    " + String(sensor.sensor_id));
  DebugLn ("Max Value:    " + String(sensor.max_value) + " lux");
  DebugLn ("Min Value:    " + String(sensor.min_value) + " lux");
  DebugLn ("Resolution:   " + String(sensor.resolution) + " lux");  
  delay(500);

  /* You can also manually set the gain or enable auto-gain support */
  tsl.setGain(TSL2561_GAIN_1X);      /* No gain ... use in bright light to avoid sensor saturation */
  // tsl.setGain(TSL2561_GAIN_16X);     /* 16x gain ... use in low light to boost sensitivity */
  // tsl.enableAutoRange(true);            /* Auto-gain ... switches automatically between 1x and 16x */
  
  /* Changing the integration time gives you better sensor resolution (402ms = 16-bit data) */
  // tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);      /* fast but low resolution */
  tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_101MS);  /* medium resolution and speed   */
  // tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_402MS);  /* 16-bit data but slowest conversions */

  /* Update these values depending on what you've set above! */  
  DebugLn ("***** Initialized TSL 2561 *****");

  return true;
}

float getTSL2561lux() 
{
  /* Get a new sensor event */ 
  sensors_event_t event;
  sensor_t sensor;

  tsl.getSensor(&sensor);

  DebugLn ("Reading TSL2561 data");
  DebugLn ("Max Value:    " + String(sensor.max_value) + " lux");
  DebugLn ("Min Value:    " + String(sensor.min_value) + " lux");
  DebugLn ("Resolution:   " + String(sensor.resolution) + " lux");  

  tsl.getEvent(&event);

  return event.light;
 
}

#ifdef CCS811
// CCS811 functions
/**************************************************************************/
/*     CSS811 setup function 
/**************************************************************************/
bool CCS811setup()
{
  if(!ccs.begin()){
    DebugLn("Failed to start CCS811!");
    CCS811_active = false;
  } 
  else
  {
    DebugLn("Sensor CCS811 initiated");
    CCS811_active = true;
  }

  return CCS811_active;
}

/**************************************************************************/
/*     CSS811 get data function 
/**************************************************************************/
void getCCS811data()
{
  if(ccs.available()){
    tempccs811 = ccs.calculateTemperature();
    if(!ccs.readData()){
      CO2ppm = ccs.geteCO2();
      TVOCppb = ccs.getTVOC();
      DebugLn("Read CCS811 data");
      DebugLn("CO2: " + String(CO2ppm) + " ppm, TVOC: " + String(TVOCppb) + "ppb"); 
    }
    else{
      DebugLn("ERROR getting CCS811 data!");
    }
  }
}

#endif


#ifdef GROVE
void GroveSetup(){

    gas.begin(0x04);  //the default I2C address of the slave is 0x04
    DebugLn("Grove Gas sensor initialized");
    GrovePowerON();
    Debug("Grove firmware Version: ");
    DebugLn(gas.getVersion());
}

void GrovePowerON() {
    gas.powerOn();
    DebugLn("Grove sensor, power ON");
}

void GrovePowerOFF() {
    gas.powerOff();
    DebugLn("Grove sensor, power OFF");
}

void getGrovedata() {
  float c;

  c = gas.measure_NH3();
  if (c >= 0) {
    NH3ppm = c;
    DebugLn("NH3: " + String(c));
  }

  c = gas.measure_CO();
  if (c >= 0) {
    COppm = c;
    DebugLn("CO: " + String(c));
  }
  
  c = gas.measure_NO2();
  if (c >= 0) {
    NO2ppm = c;
    DebugLn("NO2: " + String(c));
  }
  
  c = gas.measure_C3H8();
  if (c >= 0) {
    C3H8ppm = c;
    DebugLn("C3H8: " + String(c));
  }

  c = gas.measure_C4H10();
  if (c >= 0) {
    C4H10ppm = c;
    DebugLn("C4H10: " + String(c));
  }

  c = gas.measure_CH4();
  if (c >= 0) {
    CH4ppm = c;
    DebugLn("CH4: " + String(c));
  }

  c = gas.measure_H2();
  if (c >= 0) {
    H2ppm = c;
    DebugLn("H2: " + String(c));
  }

  c = gas.measure_C2H5OH();
  if (c >= 0) {
    C2H5OHppm = c;
    DebugLn("C2H5OH: " + String(c));
  }

}

#endif

/**************************************************************************/
/*    humidityCompensation function
/*    based on same name function in hackAir.cpp 
/*    (https://github.com/hackair-project/hackAir-Arduino)
/**************************************************************************/
void  humidityCompensation() 
{
  pm25f = pm25f / (1.0 + 0.48756 * pow((humidity / 100.0), 8.60068));
  pm10f = pm10f / (1.0 + 0.81559 * pow((humidity / 100.0), 5.83411));
}


/**************************************************************************/
/*     sds011 setup function 
/**************************************************************************/
void SDS011setup()
{

#ifdef ESP32
    serialSDS.begin(9600, SERIAL_8N1, SDS_PIN_RX, SDS_PIN_TX);
    delay(100);
#else
    serialSDS.begin(9600, SWSERIAL_8N1, SDS_PIN_RX, SDS_PIN_TX, false, 192);
#endif

    DebugLn("SDS011 start/stop and reporting sample");

    Sds011::Report_mode report_mode;
    if (!sds011.get_data_reporting_mode(report_mode)) {
        DebugLn("Sds011::get_data_reporting_mode() failed");
    }
    if (Sds011::REPORT_ACTIVE != report_mode) {
        DebugLn("Turning on Sds011::REPORT_ACTIVE reporting mode");
        if (!sds011.set_data_reporting_mode(Sds011::REPORT_ACTIVE)) {
            DebugLn("Sds011::set_data_reporting_mode(Sds011::REPORT_ACTIVE) failed");
        }
    }
}

/**************************************************************************/
/*     sds011 send data to MQTT broker
/**************************************************************************/
void sendSDS011data () {
  mqtt_send("data/pm10", String(pm10f), false);
  mqtt_send("data/pm25", String(pm25f), false);

}

/**************************************************************************/
/*     registerSDS011capture function
/**************************************************************************/
void registerSDS011capture() {

    start_SDS();
    DebugLn("started SDS011 (is running = " + String(is_SDS_running) + ")");
  
    sds011.on_query_data_auto_completed([](int n) {
        DebugLn("Begin Handling SDS011 query data");
        digitalWrite(GREEN_LED, LEDON);
        int pm25;
        int pm10;
        DebugLn("n = " + String(n));
        if (sds011.filter_data(n, pm25_table, pm10_table, pm25, pm10) &&
            !isnan(pm10) && !isnan(pm25)) {
              pm10f = float(pm10) / 10;
              pm25f = float(pm25) / 10;
              DebugLn("PM10raw: " + String(pm10f));
              DebugLn("PM2.5raw: " + String(pm25f));
              humidityCompensation();
              DebugLn("PM10: " + String(pm10f));
              DebugLn("PM2.5: " + String(pm25f));
              send_TS = true;
        }
        stop_SDS();
        digitalWrite(GREEN_LED, LEDOFF);
        DebugLn("End Handling SDS011 query data");

        // Send data to Wunderground
        sendDataWunderground();

    });

    if (!sds011.query_data_auto_async(TABLESIZE, pm25_table, pm10_table)) {
        DebugLn("measurement capture start failed");
    }
  
}
/**************************************************************************/
/*     Start SDS
/**************************************************************************/
void start_SDS() {
    DebugLn("Start wakeup SDS011");

    if (sds011.set_sleep(false)) { is_SDS_running = true; }
}

/**************************************************************************/
/*     Stop SDS
/**************************************************************************/
void stop_SDS() {
    DebugLn("Start sleep SDS011");

    if (sds011.set_sleep(true)) { is_SDS_running = false; }
}


void firstSetup(){
  DebugLn("First Setup ->");
  DebugLn("Magic->"+String(settings.data.magic));
  DebugLn("Setting up AP");
  WiFi.mode(WIFI_AP);             //Only Access point
  WiFi.softAP(ap_ssid, NULL, 8);  //Start HOTspot removing password will disable security
  delay(50);
  setupserver.on("/", handleSetup);
  setupserver.on("/initForm", handleInitForm);
  DebugLn("Server Begin");
  setupserver.begin(); 
  delay(100);
  do {
    setupserver.handleClient(); 
    delay(500);
    Debug(".");
  }
  while (!ap_setup_done);
  settings.data.magic[0] = MAGIC[0];
  settings.data.magic[1] = MAGIC[1];
  settings.data.magic[2] = MAGIC[2];
  settings.data.magic[3] = MAGIC[3];
  setupserver.stop();
  WiFi.disconnect();
  settings.Save();
  DebugLn("First Setup Done");
}


void handleSetup() {
  DebugLn("handlesetup");
  String s = FPSTR(INIT_page);
  s.replace("@@SSID@@", settings.data.ssid);
  s.replace("@@PSK@@", settings.data.psk);
  s.replace("@@CLOCKNAME@@", settings.data.name);
  s.replace("@@VERSION@@",FVERSION);
  s.replace("@@UPDATERESPONSE@@", httpUpdateResponse);
  httpUpdateResponse = "";
  setupserver.send(200, "text/html", s);
}

void handleInitForm() {
  DebugLn("handleInitForm");
  DebugLn("Mode ="+String(WiFi.status()));

  String t_ssid = setupserver.arg("ssid");
  String t_psk = setupserver.arg("psk");
  String t_name = setupserver.arg("clockname");
  String(t_name).replace("+", " ");
  t_ssid.toCharArray(settings.data.ssid,SSID_LENGTH);
  t_psk.toCharArray(settings.data.psk,PSK_LENGTH);
  t_name.toCharArray(settings.data.name,NAME_LENGTH);
  httpUpdateResponse = "The configuration was updated.";
  setupserver.sendHeader("Location", "/");
  setupserver.send(302, "text/plain", "Moved");
  settings.Save();
  ap_setup_done = 1;
}

void handleRoot() {
  DebugLn("handleRoot");
 
  String s = FPSTR(MAIN_page);
  s.replace("@@SSID@@", settings.data.ssid);
  s.replace("@@PSK@@", settings.data.psk);
  s.replace("@@CLOCKNAME@@", settings.data.name);
  s.replace("@@VERSION@@",FVERSION);
  s.replace("@@PM10@@",String(pm10f));
  s.replace("@@PM25@@",String(pm25f));
  s.replace("@@TEMPERATURE@@",String(temperature));
  s.replace("@@HUMIDITY@@",String(humidity));
  s.replace("@@SEALEVELPRESSURE@@",String(sealevelpressure));
  s.replace("@@RAINA@@", String(rainA));
  s.replace("@@UPDATERESPONSE@@", httpUpdateResponse);
  s.replace("@@MQTTBROKER@@",settings.data.mqttbroker);
  s.replace("@@MQTTPORT@@",String(settings.data.mqttport));
  s.replace("@@MQTTUSER@@",settings.data.mqttuser);
  s.replace("@@MQTTPASSWD@@",settings.data.mqttpassword);
  s.replace("@@MQTTTOPIC@@",settings.data.mqtttopic);
  s.replace("@@TSCHANNEL@@",String(settings.data.ThingSpeakChannel));
  s.replace("@@TSKEY@@",settings.data.ThingSpeakKey);
  s.replace("@@POOLINT@@",String(settings.data.poolinterval));
  s.replace("@@NUMSAMPLESWIND@@",String(settings.data.windpoolinterval));
  s.replace("@@WINDINTERVAL@@",String(settings.data.windpoolinterval * 5));
  s.replace("@@RAINTHRESHOLD@@",String(settings.data.rainthreshold));
  s.replace("@@ALTITUDE@@",String(settings.data.altitude));
  s.replace("@@STATIONID@@",settings.data.stationID);
  s.replace("@@STATIONKEY@@",settings.data.stationKey);
  if (settings.data.groveactive) {
    s.replace("@@GROVE_ACTIVE@@",String(settings.data.groveactive)+String(" Checked"));
    DebugLn("GROVE " + String(settings.data.groveactive) + " Grove checked");
  }
  else {
    s.replace("@@GROVE_ACTIVE@@",String(settings.data.groveactive));
    DebugLn("GROVE =" + String(settings.data.groveactive));
  }
  httpUpdateResponse = "";
  server.send(200, "text/html", s);
}

void handleRawData() {
  DebugLn("handleRawData");

  windspeed = (sumwindspeed / (float)numsampleswind ) ;  //Wind speed in Km/h

  readRainSensor();
  
  httpUpdateResponse = "";
  server.send(200, "text/html", 
                   String("{\"Type\":\"AIRQUAL\",\"data\":") +
                        "{ \"pm10\":" + String(pm10f) + 
                        ", \"pm25\":" + String(pm25f) + 
                        ", \"NH3\":" + String(NH3ppm) + 
                        ", \"CO\":" + String(COppm) + 
                        ", \"NO2\":" + String(NO2ppm) + 
                        ", \"C3H8\":" + String(C3H8ppm) + 
                        ", \"C4H10\":" + String(C4H10ppm) + 
                        ", \"CH4\":" + String(CH4ppm) + 
                        ", \"H2\":" + String(H2ppm) + 
                        ", \"C2H5OH\":" + String(C2H5OHppm) + 
                        ", \"temperature\":" + String(temperature) +
                        ", \"humidity\":" + String(humidity) +
                        ", \"dewpoint\":" + String(dewpoint) +
                        ", \"sealevelpressure\":" + String(sealevelpressure) +
                        ", \"windspeed\":" + String(windspeed) +
                        ", \"maxwindspeed\":" + String(maxwindspeed) +
                        ", \"lux\":" + String(lux) +
                        ", \"running\":" + String(is_SDS_running) + 
                        ", \"CO2ppm\":" + String(CO2ppm) + 
                        ", \"TVOCppb\":" + String(TVOCppb) + 
                        ", \"RainA\":" + String(rainA) + 
                        ", \"RainD\":" + String(rainD) + 
                        "}}\r\n"
                   );
}

void sendHTTP (String httpPost){
  WiFiClient client;
  HTTPClient http;
  DebugLn(httpPost);
  http.begin(client, httpPost);
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");
  int httpCode = http.POST("");
  DebugLn(httpCode);    
  http.end();
}

void sendHTTPGet (String httpGet){
  WiFiClient client;
  HTTPClient http;
  DebugLn(httpGet);
  http.begin(client, httpGet);
  int httpCode = http.GET();
  DebugLn(httpCode);    
  http.end();
}

void sendHTTPsGet (String httpGet){
  HTTPClient https;
  std::unique_ptr<BearSSL::WiFiClientSecure>client(new BearSSL::WiFiClientSecure);

  // fingerprint not needed
  client->setInsecure();
  // send data
  DebugLn("HTTPsGet: " + httpGet);
  https.begin(*client, httpGet);
  // http.addHeader("Content-Type", "application/x-www-form-urlencoded");
  int httpCode = https.GET();
  String payload = https.getString();

  DebugLn("HTTPsGet return code: " + String(httpCode));    // this return 200 when success
  DebugLn(payload);     // this will get the response
  https.end();

}

void sendRainData()
{
  // read rain sensor and send data
  readRainSensor();
  mqtt_send("data/rain", 
    "{\"rainD\":" + String(rainD) + 
    ",\"rainA\":" + String(rainA) + 
    "}", false);
  
}

void handleIO() {
  DebugLn("handleIO");

  // Report Data
  mqtt_send("data/ipaddress", WiFi.localIP().toString(),false);

  // get Temperature, humidity and Pressure
  getBME280data();
  DebugLn("Temp: "+String(temperature)+" ºC, Humid: "+String(humidity)+"%");
  if ( isnan(temperature) ) {
    // temperature value is nan, reset device
    DebugLn("Failed temp reading [nan], reseting device");
    delay(2000);
    ESP.restart();
  }
  mqtt_send("data/temperature", String(temperature), false);
  mqtt_send("data/humidity", String(humidity), false);
  mqtt_send("data/dewpoint", String(dewpoint), false);
  mqtt_send("data/sealevelpressure", String(sealevelpressure), false);

  // get luminostity from TSL2561
  if (TSL2561active) {
    lux = getTSL2561lux();
    mqtt_send("data/lux", String(lux), false);
  }

  #ifdef CCS811
  // read CCS811
  if (CCS811_active) {
    getCCS811data();
    mqtt_send("data/CO2ppm", String(CO2ppm), false);
    mqtt_send("data/TVOCppb", String(TVOCppb), false);
  }
#endif

#ifdef GROVE
  if (settings.data.groveactive) {
    // Read Grove Gas Multisensor
    getGrovedata();
    mqtt_send("data/NH3ppm", String(NH3ppm), false);
    mqtt_send("data/COppm", String(COppm), false);
    mqtt_send("data/NO2ppm", String(NO2ppm), false);
    mqtt_send("data/C3H8ppm", String(C3H8ppm), false);
    mqtt_send("data/C4H10ppm", String(C4H10ppm), false);
    mqtt_send("data/CH4ppm", String(CH4ppm), false);
    mqtt_send("data/H2ppm", String(H2ppm), false);
    mqtt_send("data/C2H5OHppm", String(C2H5OHppm), false);
  }
#endif

}

void handleForm() {
  String aux;

  DebugLn("handleForm");
  DebugLn("Mode ="+String(WiFi.status()));
  String update_wifi = server.arg("update_wifi");
  String eco_mode = server.arg("eco_mode");
  DebugLn("update_wifi: " + update_wifi +", eco_mode: "+eco_mode);
  String t_ssid = server.arg("ssid");
  String t_psk = server.arg("psk");
  String t_name = server.arg("clockname");
  String(t_name).replace("+", " ");
  if (update_wifi == "1") {
    t_ssid.toCharArray(settings.data.ssid,SSID_LENGTH);
    t_psk.toCharArray(settings.data.psk,PSK_LENGTH);
    t_name.toCharArray(settings.data.name,NAME_LENGTH);
  }

  String pint = server.arg("poolint");
  if (pint.length()) {
    settings.data.poolinterval=constrain(pint.toInt(),10,3600);
  }
//  mqtt_send("setup/poolint", String(settings.data.poolinterval), false);

  aux = server.arg("rainthreshold");
  if (aux.length()) {
    settings.data.rainthreshold= constrain(aux.toInt(),0,1023);
  }
  aux = server.arg("altitude");
  if (aux.length()) {
    settings.data.altitude = aux.toFloat();
  }
  aux = server.arg("numsampleswind");
  if (aux.length()) {
    settings.data.windpoolinterval = aux.toInt();
  }

  bool grove_act_prev = settings.data.groveactive;  // store previous value
  aux = server.arg("grove_active");
  if (aux != "") {
    settings.data.groveactive=1;
  }
  else {
    settings.data.groveactive=0;
  }


  String brokerprev = String(settings.data.mqttbroker);
  int portprev = settings.data.mqttport;
  
  aux = server.arg("mqttbroker");
  aux.toCharArray(settings.data.mqttbroker,128);
  aux = server.arg("mqttuser");
  aux.toCharArray(settings.data.mqttuser,30);
  aux = server.arg("mqttpasswd");
  aux.toCharArray(settings.data.mqttpassword,30);
  aux = server.arg("mqtttopic");
  aux.toCharArray(settings.data.mqtttopic,30);
  aux = server.arg("mqttport");
  if (aux.length()) {
    settings.data.mqttport=aux.toInt();
  }

  aux = server.arg("tschannel");
  if (aux.length()) {
    settings.data.ThingSpeakChannel=aux.toInt();
  }
  else {
    settings.data.ThingSpeakChannel=0;
  }
  aux = server.arg("tskey");
  aux.toCharArray(settings.data.ThingSpeakKey,30);

  aux = server.arg("stationid");
  aux.toCharArray(settings.data.stationID,128);
  aux = server.arg("stationkey");
  aux.toCharArray(settings.data.stationKey,128);

  httpUpdateResponse = "The configuration was updated.";
  ap_setup_done = 1;
  server.sendHeader("Location", "/");
  server.send(302, "text/plain", "Moved");
  settings.Save();

  if ( grove_act_prev != settings.data.groveactive ) {
    // if Grove activation status change, then reset to init/no-init Grove sensor
    ESP.restart();
  }

  if (String(settings.data.mqttbroker) != brokerprev || settings.data.mqttport != portprev) {
    // MQTT broker or port has changed. restart
    client.disconnect();
    mqtt_init();
  }
  
  if (update_wifi != "") {
    delay(500);
    setupSTA();             // connect to Wifi
  }
}

/**************************************************
 * Callback function for MQTT
 * ***********************************************/
void callback(char* topic, byte* payload, unsigned int length) {
char* buff;
const char* nomsg="";

  Debug("MQTT message received [");
  Debug(String(topic));
  Debug("] :");

  buff = (char*)malloc(length+1);
  memcpy(buff, payload, length);
  buff[length] = '\0';
  DebugLn(buff);

  // is payload no NULL?
  if (length > 0) { 
    if (String(topic) == (base_topic + "/setup/poolint") ) {
      int poolintprev = settings.data.poolinterval;
      settings.data.poolinterval = atoi(buff);
      // save settings only if parameter has changed
      if (poolintprev != settings.data.poolinterval) settings.Save();  
      // remove retained message
      client.publish((base_topic + "/setup/poolint").c_str(), nomsg, true);
    }
    else if (String(topic) == (base_topic + "/setup/altitude") ) {
      int altitudeprev = settings.data.altitude;
      settings.data.altitude = atof(buff);
      // save settings only if parameter has changed
      if (altitudeprev!= settings.data.altitude) settings.Save();  
      // remove retained message
      client.publish((base_topic + "/setup/altitude").c_str(), nomsg, true);
    }
    else if (String(topic) == (base_topic + "/setup/rainthreshold") ) {
      int thresholdprev = settings.data.rainthreshold;
      settings.data.rainthreshold = atoi(buff);
      // save settings only if parameter has changed
      if (thresholdprev != settings.data.rainthreshold) settings.Save();  
      // remove retained message
      client.publish((base_topic + "/setup/rainthreshold").c_str(), nomsg, true);
    }
  }
  
  free(buff);
}

/*************************************************
 * MQTT init function
 * **********************************************/
void mqtt_init() {

  client.setServer(settings.data.mqttbroker, settings.data.mqttport);
  DebugLn("setServer: " + String(settings.data.mqttbroker) + ", port: " +String(settings.data.mqttport));
  client.setCallback(callback);
  mqtt_reconnect();

}

/*************************************************
 * MQTT reconnect function
 * **********************************************/
bool mqtt_reconnect() {
int res;

  if (!client.connected() ) {
    DebugLn("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = String(settings.data.name) + String(random(0xffff), HEX);
    // Attempt to connect

    if(strlen(settings.data.mqttuser)) {
      res = client.connect(clientId.c_str(), settings.data.mqttuser, settings.data.mqttpassword);
    }
    else {
      res = client.connect(clientId.c_str());
    }
    if (res) {
      DebugLn("MQTT connected");
      // once connect ....resubscribe
      base_topic = String(settings.data.mqtttopic) + "/" + String(settings.data.name);
      DebugLn("base topic: " + base_topic);
      client.subscribe((base_topic + "/setup/#").c_str());
      // return true;
    } else {
      Debug ("MQTT reconnection failed, rc=");
      DebugLn (client.state());
      // return false;
    }
  }
  else // already connected
    res = true;
  return res;
}

/*************************************************
 * MQTT send function
 * **********************************************/
void mqtt_send(String subtopic, String message, bool retain){

String topic = base_topic + "/" + subtopic;

  DebugLn("mqtt_send, topic: " + topic + ", payload: " + message);
  if(mqtt_reconnect() ) {
    // send data to topic
    client.publish(topic.c_str(), message.c_str(), retain);
    Debug("mqtt send [" );
    Debug(topic);
    Debug("]: ");
    DebugLn(message);
  }
}

/*************************************************
 * ThingSpeak send function
 * **********************************************/
void ts_send(unsigned long ts_channel, char* ts_key){
int result;

  if ( ts_channel != 0) {
    DebugLn("ThingSpeak, loading fields...");
#ifdef SDS011
    ThingSpeak.setField(1, pm10f);
    ThingSpeak.setField(2, pm25f);
#endif
#ifdef  GROVE
    if (settings.data.groveactive) {
      ThingSpeak.setField(3, NH3ppm);
      ThingSpeak.setField(4, COppm);
      ThingSpeak.setField(5, NO2ppm);
    }
#endif
    ThingSpeak.setField(6, temperature);
    ThingSpeak.setField(7, humidity);
    ThingSpeak.setField(8, dewpoint);

    result = ThingSpeak.writeFields(ts_channel, ts_key);
    Debug("ThingSpeak write, channel = ");
    Debug(ts_channel);
    Debug(", Key= ");
    Debug(ts_key);
    Debug(", result = ");
    DebugLn(result);
  }
}

int setupSTA()
{
  int timeOut=0;

  for (int retry=0; retry<=3; retry++) {
    WiFi.disconnect();
    WiFi.hostname("ESP_" + String(settings.data.name)) ;
    WiFi.mode(WIFI_STA);
    DebugLn("Connecting to "+String(settings.data.ssid)+" Retry:"+String(retry));
    DebugLn("Connecting to "+String(settings.data.psk));
    
    if (String(settings.data.psk).length()) {
      WiFi.begin(String(settings.data.ssid), String(settings.data.psk));
    } else {
      WiFi.begin(String(settings.data.ssid));
    }
    
    timeOut=0;
    while (WiFi.status() != WL_CONNECTED) {
      if ((timeOut < 10) && WiFi.status() != WL_CONNECT_FAILED){ // if not timeout or failure, keep trying
        //delay(100);
        Debug(String(WiFi.status()));
        displayBusy();
        timeOut ++;
      } 
      else{
        timeOut = 0;
        DebugLn("-Wifi connection timeout");
        displayBusy();
        if (retry == 2) 
          return 0;
        break;
      }
    }
    if (WiFi.status() == WL_CONNECTED)
      break;
  }  
  DebugLn(" Connected");
  // Print the IP address
  DebugLn(WiFi.localIP()); 
  DebugLn(WiFi.hostname().c_str());
  return 1;
}

void sendDataWunderground(){
float tempf;
float dewptf;
float baromin;

  if(strlen(settings.data.stationID) > 0 && strlen(settings.data.stationKey) > 0 ){
    // Send data to Wunderground
    DebugLn("--- Sending data to Wunderground ---");
    String  weatherData =  "https://rtupdate.wunderground.com/weatherstation/updateweatherstation.php?";
    weatherData += "ID=" + String(settings.data.stationID);
    weatherData += "&PASSWORD=" + String(settings.data.stationKey);
    weatherData += "&dateutc=now";
    weatherData += "&action=updateraw";
    tempf = (temperature * 9.0) / 5.0 + 32.0;
    weatherData += "&tempf=" + String (tempf);
    weatherData += "&humidity=" + String (humidity);
    dewptf = (dewpoint * 9.0) / 5.0 + 32.0;
    weatherData += "&dewptf=" + String(dewptf);
    baromin = sealevelpressure * 29.92 / 1013.25;
    weatherData += "&baromin=" + String(baromin);
  
  #ifdef SDS011
    // send PM10 and PM2.5 particles concentration
    weatherData += "&AqPM2.5=" + String (pm25f);
    weatherData += "&AqPM10=" + String (pm10f);
  #endif
  #ifdef GROVE
    if ( settings.data.groveactive ) {
      // send NO2 and CO concetrations
      weatherData += "&AqNO2=" + String (NO2ppm);
      weatherData += "&AqCO=" + String (COppm);
    }
  #endif
  
    // send to Wunderground
    sendHTTPsGet(weatherData);
  }
                
}

void readRainSensor()
{
  // Read rain sensor on analog input
  rainA = analogRead(A0);
  rainD = ( rainA < settings.data.rainthreshold );
  
}

void setup() {

  DebugStart();
  DebugLn("Setup ->");
  myResetInfo = ESP.getResetInfoPtr();
  DebugLn("myResetInfo->reason "+String(myResetInfo->reason)); // reason is uint32
                                                                 // 0 = power down
                                                                 // 6 = reset button
                                                                 // 5 = restart from deepsleep
                                                                 
  settings.Load();
  
  // ******** initiallize LEDs and wait for setup pin *****************
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);

  DebugLn("-> Check if SETUP_PIN is low");
  digitalWrite(RED_LED, LEDON);
  digitalWrite(GREEN_LED, LEDON);
  // Wait up to 5s for SETUP_PIN to go low to enter AP/setup mode.
  pinMode(SETUP_PIN, INPUT);      //Configure setup pin as input
  digitalWrite(SETUP_PIN, HIGH);  //Enable internal pooling
  delay(5000);  
  DebugLn("Magic ->"+String(settings.data.magic));

  // if NO MAGIC Or SETUP PIN enter hard config...
  if ((String(settings.data.magic) != MAGIC)   || !digitalRead(SETUP_PIN)){
    digitalWrite(GREEN_LED, LEDOFF);
    digitalWrite(RED_LED, LEDON);
    firstSetup();
  }

  // NO SETUP, switch off both LEDs
  digitalWrite(GREEN_LED, LEDOFF);
  digitalWrite(RED_LED, LEDOFF);

  // *********** setup STA mode and connect to WiFi ************
  if (setupSTA() == 0) { // SetupSTA mode
    DebugLn("Wifi no connected");
    // wait poolinterval and restart
    DebugLn("wait " + String(settings.data. poolinterval) + " sec. and restart");
    displayBusy();
    delay(1000*settings.data.poolinterval);
    ESP.restart();
  }

  //********* initialize sensors *****************
  // BME280, SDA=4 (D2), SCL=5 (D1)
  bme.begin(0x76);          
  DebugLn("-> BME begin");

  // TSL2561, SDA=4 (D2), SCL=5 (D1)
  TSL2561active = TSL2561setup();
  
#ifdef SDS011
  SDS011setup();
#endif
#ifdef CCS811
  CCS811setup();
#endif
#ifdef GROVE
  if ( settings.data.groveactive ) {
    GroveSetup();
  }
#endif

  // ********** initialize OTA *******************
  ArduinoOTA.begin();

  // ********* initialize MQTT ******************
  mqtt_init();
  
  // ********* initialize ThingSpeak *************
  ThingSpeak.begin(espClient);
  
  //********** switch ON GREEN LED, initialize web servers and switch OFF LEDs
  digitalWrite(GREEN_LED, LEDON);
  digitalWrite(RED_LED, LEDOFF);
  DebugLn("-> Initiate WebServer");
  server.on("/", handleRoot);
  server.on("/setup", handleRoot);
  server.on("/form", handleForm);
  server.on("/data",handleRawData);
  delay(100);
  server.begin(); 
   delay(3000);
  digitalWrite(GREEN_LED, LEDOFF);
  digitalWrite(RED_LED, LEDOFF);  
  
  // ********* initialize counters for WindSpeed and Rain
  lastSecond = millis();
  lastSecond2 = lastSecond;
    
  //*********** Annemometer initialization *************
  pinMode(ANNEMOMETER_PIN, FUNCTION_3);
  pinMode(ANNEMOMETER_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ANNEMOMETER_PIN), wspeedIRQ, FALLING);
  // reset value and counters
  sumwindspeed = 0.0;
  numsampleswind = 0;
  maxwindspeed = 0.0;

  start = millis();
  DebugLn("Started at "+String(start));

  // Send data to Jeedom
  handleIO();
  
  // initiate SDS011 capture
  registerSDS011capture();

  // check MQTT received messages
  for (int i=0; i < 10; i++) {
    client.loop();
    delay(10);
  }

  timer = 0;
  timer2 = millis();
  DebugLn("-> End Setup");
}

void loop() {

  if (timer > settings.data.poolinterval*9) {
      timer = 0;

      handleIO();

      // initiate SDS011 capture
      registerSDS011capture();
    }
    else {                  // handle http client while waiting
      timer++;
      server.handleClient();    

      //send data to ThingSpeak if data is ready
      if (send_TS) {
        DebugLn("Send ThingSpeak data");
        send_TS = false;
        // send sds011 data to MQTT
        sendSDS011data();
        ts_send(settings.data.ThingSpeakChannel, settings.data.ThingSpeakKey);
      }
      delay(100);
    }
 
  // Perform DSD011 read cycle
  if (is_SDS_running) {
    sds011.perform_work();
  }

  // handle OTA
  ArduinoOTA.handle();

  // handle MQTT
  client.loop();

  // calculate wind speed every 5 seconds
  if(millis() - lastSecond >= 5000)
  {
      lastSecond += 5000;

      //Get wind speed 
      windspeed = get_wind_speed(); //Wind speed in Km/h
      sumwindspeed += windspeed;
      maxwindspeed = max(windspeed, maxwindspeed);
      numsampleswind++;
      if (numsampleswind == settings.data.windpoolinterval) {
        windspeed = (sumwindspeed / (float)numsampleswind);  
        mqtt_send("data/windspeed", String(windspeed), false);
        mqtt_send("data/maxwindspeed", String(maxwindspeed), false);
        sumwindspeed = 0.0;
        maxwindspeed = 0.0;
        numsampleswind = 0;
      }
  }

  // send rain data every 30 seconds
  if( (millis() - lastSecond2) >= 30000)
  {
    lastSecond2 += 30000;
    sendRainData();
  }
  
  // reset every 24 h 
  if (millis() > (24*60*60*1000ul)) {
    ESP.restart();
  }
  
}
