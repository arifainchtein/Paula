#include "Arduino.h"
#include <LoRa.h>
#include <Timer.h>
#include <FastLED.h>
#include <SPI.h>
#include <LangleyData.h>
#include <DigitalStablesData.h>
#include <ChinampaData.h>
#include <GloriaTankFlowPumpData.h>
#include <SeedlingMonitoringData.h>
#include <SolarPowerData.h>
#include <PCF8563TimeManager.h>
#include <Esp32SecretManager.h>
//#include <Arduino_JSON.h>
#include <Wire.h>
#include <DataManager.h>
#include <LittleFS.h>
#include "ADS1X15.h"
bool gloriaTankFlowPumpNewData = false;
bool digitalStablesDataNewData = false;
bool chinampaDataNewData = false;
WeatherForecast storedForecasts[4];
bool weatherForecastNewData = false;
RTCInfoRecord weatherForecastReceivedTime;
String currentIpAddress = "No IP";
bool inPulse=false;

// Flow sensor qfactor test/calibration - moved here from Daffodil.ino (2026-09-10). Paula is
// the bench diagnostic unit: the sensor under test connects directly to Paula's own screw
// terminals (pins 18/33, same as Daffodil), so this runs locally, not over LoRa/USB to a
// separate device. qfactor = pulses / (liters * 60) - see runFlowCalibrationLedSequence and the
// CalibrateFlowSensorXStop handlers below for the derivation and one-person LED-guided workflow.
static volatile int flowMeterPulseCount;
static volatile int flowMeterPulseCount2;
static volatile unsigned long calibrationPulseCount1 = 0;
static volatile unsigned long calibrationPulseCount2 = 0;
static volatile bool calibratingFlow1 = false;
static volatile bool calibratingFlow2 = false;

// DYP-A02-V2.0 ultrasonic sensor, UART automatic output mode (datasheet: UlltraSonicUART.png,
// ~/Pictures/Screenshots) - 9600 baud 8N1, continuously streams 4-byte frames once its own
// RX/trigger lead is left floating/high (that's what selects "automatic"/processed output
// rather than needing a trigger pulse): 0xFF header, Data_H, Data_L, checksum = (header+Data_H+
// Data_L) & 0xFF. Distance in mm = Data_H*256 + Data_L. Pin 18 receives the sensor's TX - same
// physical screw terminal as Flow Sensor 1's interrupt input, so only one sensor type can be
// under test at a time (see SetTestMode below, which reconfigures pin 18 between the two).
HardwareSerial ultrasonicSerial(2);
enum TestMode { TEST_MODE_FLOW, TEST_MODE_ULTRASONIC };
TestMode currentTestMode = TEST_MODE_FLOW;
uint16_t lastUltrasonicDistanceMM = 0;
unsigned long lastUltrasonicReadMillis = 0;
uint8_t ultrasonicFrameBuf[4];
uint8_t ultrasonicFrameBufLen = 0;

#define SCK 14
#define MOSI 13
#define MISO 12
#define LoRa_SS 15
#define LORA_RESET 16
#define LORA_DI0 17
#define SENSOR_INPUT_2 33
#define SENSOR_INPUT_1 18

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "SHTSensor.h"

// Display settings for 2.4" SSD1309
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1        // Reset pin (-1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C  // Common I2C address (try 0x3D if this doesn't work)
#define SLEEP_SWITCH_26 26

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
// I2C temperature/humidity sensor test (SHT3x, address 0x44 - same part Daffodil uses on the
// TopTank/etc. units). Shares the same I2C bus (SDA/SCL) already begun for the OLED - no
// separate pins or mode switching needed, unlike the flow/ultrasonic tests on pin 18/33.
SHTSensor shtTest;

// Tank pressure sensor test (0.5V-4.5V/0-5psi transducer via the ADS1115 ADC, address 0x48) -
// same screw terminals as the flow sensors (pin 18/33), same math as Daffodil.ino's
// readTankPressure1()/2(): 0.5V=0psi, 4.5V=5psi (live-zero transducer, so ~0V reads as a fault,
// not "0psi"). Unlike flow/ultrasonic this needs no SetTestMode switch - it's the ANALOG jumper
// position on the same terminal (flow/ultrasonic need the DIGITAL position instead), a physical
// setting firmware can't detect, hence the reminder in PaulaDeployer's UI.
ADS1115 ADS(0x48);
DataManager dataManager(Serial, LittleFS);


#define LED_PIN 19
// Paula shield (Daffodil hardware) has 15 physical LEDs - the flow calibration countdown
// (runFlowCalibrationLedSequence) needs all 15 for its 15s red / 5s green sequence.
#define NUM_LEDS 15
#define STALE_INTERVAL 20
#define WEATHER_STALE_INTERVAL 7200

RTCInfoRecord currentTimerRecord;
#define TIME_RECORD_REFRESH_SECONDS 3
bool switchPositionLeft = true;
volatile bool clockTicked = false;
volatile bool lowVoltageAlert = false;
int lastLoraReceptionSeconds = 0;
int lastWeatherForecastSeconds = 0;

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

boolean debug = true;
String timezone = "AEST-10AEDT,M10.1.0,M4.1.0/3";
PCF8563TimeManager timeManager(Serial);
Esp32SecretManager secretManager(timeManager);
GeneralFunctions generalFunctions;


CRGBPalette16 currentPalette;
TBlendType currentBlending;


CRGB leds[NUM_LEDS];

LangleyData langleyData;
DigitalStablesData digitalStablesData;
SeedlingMonitorData seedlingMonitorData;
GloriaTankFlowPumpData gloriaTankFlowPumpData;
ChinampaData chinampaData;
boolean displayingChinampa = true;
uint8_t chinampaPageToDisplay = 0;
bool refreshChinampaPage = false;

#define RTC_CLK_OUT 4
bool loraActive = false;
volatile bool loraReceived = false;
volatile int loraPacketSize = 0;


TaskHandle_t ledShowTask = NULL;
bool runLedShow = true;
int ledShowDuration = 1000;

void ledShowTaskFunction(void* parameter) {
  while (true) {
    if (runLedShow) {
      performLedShow(ledShowDuration);
      runLedShow = false;  // Reset flag after completion
    }
    vTaskDelay(1);  // Small delay to prevent watchdog triggers
  }
}
void LoRa_rxMode() {
  LoRa.disableInvertIQ();  // normal mode
  LoRa.receive();          // set receive mode
}

void LoRa_txMode() {
  LoRa.idle();             // set standby mode
  
  LoRa.disableInvertIQ();  // normal mode
}

void FillLEDsFromPaletteColors(uint8_t colorIndex) {
  uint8_t brightness = 255;

  for (int i = 8; i < NUM_LEDS; ++i) {
    leds[i] = ColorFromPalette(currentPalette, colorIndex, brightness, currentBlending);
    colorIndex += 3;
  }
}

void performLedShow(int millisseconds) {
  long startmillis = millis();
  long currentMillis = startmillis;
  while (currentMillis < startmillis + (millisseconds)) {
    // currentPalette = RainbowColors_p;
    // currentBlending = LINEARBLEND;
    //  ChangePalettePeriodically();
    static uint8_t startIndex = 8;
    startIndex = startIndex + 1; /* motion speed */
    FillLEDsFromPaletteColors(startIndex);
    FastLED.show();
    currentMillis = millis();
  }
  for (int i = 1; i < 6; i++) {
    leds[i] = CRGB(0, 0, 0);
  }
  FastLED.show();
  //  delay(1000);
}


void IRAM_ATTR clockTick() {
  portENTER_CRITICAL_ISR(&mux);
  clockTicked = true;
  portEXIT_CRITICAL_ISR(&mux);
}

void IRAM_ATTR pulseCounter() {
  flowMeterPulseCount++;
  if (calibratingFlow1) calibrationPulseCount1++;
}

void IRAM_ATTR pulseCounter2() {
  flowMeterPulseCount2++;
  if (calibratingFlow2) calibrationPulseCount2++;
}

// Non-blocking - call every loop() while in TEST_MODE_ULTRASONIC. Scans for the 0xFF frame
// header (resyncs on any byte that doesn't complete a valid checksum, so it recovers on its own
// from a partial/garbled frame instead of getting permanently out of sync with the stream).
void pollUltrasonicSerial() {
  while (ultrasonicSerial.available()) {
    uint8_t b = ultrasonicSerial.read();
    if (ultrasonicFrameBufLen == 0) {
      if (b == 0xFF) ultrasonicFrameBuf[ultrasonicFrameBufLen++] = b;
    } else {
      ultrasonicFrameBuf[ultrasonicFrameBufLen++] = b;
      if (ultrasonicFrameBufLen == 4) {
        uint8_t sum = (ultrasonicFrameBuf[0] + ultrasonicFrameBuf[1] + ultrasonicFrameBuf[2]) & 0xFF;
        if (sum == ultrasonicFrameBuf[3]) {
          lastUltrasonicDistanceMM = (ultrasonicFrameBuf[1] << 8) | ultrasonicFrameBuf[2];
          lastUltrasonicReadMillis = millis();
        }
        ultrasonicFrameBufLen = 0;
      }
    }
  }
}

void onReceive(int packetSize) {

  loraReceived = true;
  loraPacketSize = packetSize;
}


void processLora(int packetSize) {

  lastLoraReceptionSeconds = 0;



  if (packetSize == 0) return;  // if there's no packet, return
  if (switchPositionLeft) displayingChinampa = false;
  if(debug)Serial.println("received " + String(packetSize));
  if (packetSize == sizeof(LangleyData)) {
    LoRa.readBytes((uint8_t*)&langleyData, sizeof(LangleyData));

    langleyData.rssi = LoRa.packetRssi();
    langleyData.snr = LoRa.packetSnr();




  } else if (packetSize == sizeof(GloriaTankFlowPumpData)) {
    memset(&gloriaTankFlowPumpData, 0, sizeof(GloriaTankFlowPumpData));
    LoRa.readBytes((uint8_t*)&gloriaTankFlowPumpData, sizeof(GloriaTankFlowPumpData));

    gloriaTankFlowPumpData.rssi = LoRa.packetRssi();
    gloriaTankFlowPumpData.snr = LoRa.packetSnr();


    gloriaTankFlowPumpData.rssi = LoRa.packetRssi();
    gloriaTankFlowPumpData.snr = LoRa.packetSnr();


  } else if (packetSize == sizeof(ChinampaData)) {
    memset(&chinampaData, 0, sizeof(ChinampaData));
    LoRa.readBytes((uint8_t*)&chinampaData, sizeof(ChinampaData));

    chinampaData.rssi = LoRa.packetRssi();
    chinampaData.snr = LoRa.packetSnr();
    //if(debug)
    if (debug) Serial.print("received chinampaData from ");
    if (debug) Serial.print(chinampaData.devicename);

    if (debug) Serial.print(" ");
    if (debug) Serial.println(TimeUtils::epochToString(chinampaData.secondsTime));
    dataManager.storeChinampaData(chinampaData);
   chinampaDataNewData = true;
  } else if (packetSize == sizeof(DigitalStablesData)) {
    memset(&digitalStablesData, 0, sizeof(DigitalStablesData));
    LoRa.readBytes((uint8_t*)&digitalStablesData, sizeof(DigitalStablesData));
    digitalStablesDataNewData = true;
    digitalStablesData.rssi = LoRa.packetRssi();
    digitalStablesData.snr = LoRa.packetSnr();
    dataManager.storeDigitalStablesData(digitalStablesData);
     dataManager.printDigitalStablesData(digitalStablesData);
    //if(debug)
    if (debug) Serial.print("received digitalStablesData from ");
    if (debug) Serial.print(digitalStablesData.devicename);

    if (debug) Serial.print(" ");
    if (debug) Serial.print(TimeUtils::epochToString(digitalStablesData.secondsTime));


    if (debug) Serial.print(" voltage=");
    if (debug) Serial.print(digitalStablesData.batteryVoltage);
    if (debug) Serial.print(" current=");
    if (debug) Serial.print(digitalStablesData.batteryCurrent);
    if (debug) Serial.print(" estimatedRuntime=");
    if (debug) Serial.print(digitalStablesData.estimatedRuntime);
    if (debug) Serial.print(" operatingStatus=");
    if (debug) Serial.println(digitalStablesData.operatingStatus);
  } else if (packetSize == sizeof(WeatherForecastUpdate)) {
    WeatherForecastUpdate wfu;
    LoRa.readBytes((uint8_t*)&wfu, sizeof(WeatherForecastUpdate));
    bool validCode = secretManager.checkCode(wfu.totpcode);
    if (validCode) {
      for (int i = 0; i < 4; i++) storedForecasts[i] = wfu.forecasts[i];
      weatherForecastReceivedTime = currentTimerRecord;
      weatherForecastNewData = true;
      lastWeatherForecastSeconds = 0;
      if (debug) Serial.println("WeatherForecastUpdate received");
    } else {
      if (debug) Serial.println("WeatherForecastUpdate: invalid TOTP");
    }
  } else if (packetSize == sizeof(SeedlingMonitorData)) {
    memset(&seedlingMonitorData, 0, sizeof(SeedlingMonitorData));
    LoRa.readBytes((uint8_t*)&seedlingMonitorData, sizeof(SeedlingMonitorData));



    seedlingMonitorData.rssi = LoRa.packetRssi();
    seedlingMonitorData.snr = LoRa.packetSnr();
    //if(debug)
    if (debug) Serial.print("received SeedlingMonitorData from ");
    if (debug) Serial.println(seedlingMonitorData.devicename);
  }
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Wire.begin();

  if(!LittleFS.begin(true)) { 
    if(debug)Serial.println("LittleFS Mount Failed, formatting..."); 
    LittleFS.format(); 
    if(!LittleFS.begin(false)) { 
      if(debug)Serial.println("LittleFS Mount Failed even after formatting"); 
      return; 
    }else{
      if(debug)Serial.println("LittleFS Mount Succces after formating"); 
    }
 }else{
   if(debug)Serial.println("LittleFS Mount Succces"); 
 }
  pinMode(SLEEP_SWITCH_26, OUTPUT);
  digitalWrite(SLEEP_SWITCH_26, HIGH);

  if(debug)Serial.print("DigitalStablesData size=");
  if(debug)Serial.println(sizeof(DigitalStablesData));

  if(debug)Serial.print("ChinampaData size=");
  if(debug)Serial.println(sizeof(ChinampaData));


  if(debug)Serial.print("seedlingMonitorData size=");
  if(debug)Serial.println(sizeof(SeedlingMonitorData));

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    if(debug)Serial.println(F("SSD1309 allocation failed"));
    for (;;)
      ;  // Loop forever if failed
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("Please Wait"));
  display.display();


  pinMode(SENSOR_INPUT_1, INPUT);
  pinMode(SENSOR_INPUT_2, INPUT);
  attachInterrupt(SENSOR_INPUT_1, pulseCounter, FALLING);
  attachInterrupt(SENSOR_INPUT_2, pulseCounter2, FALLING);

  pinMode(RTC_CLK_OUT, INPUT_PULLUP);  // set up interrupt pin
  digitalWrite(RTC_CLK_OUT, HIGH);     // turn on pullup resistors
  // attach interrupt to set_tick_tock callback on rising edge of INT0
  attachInterrupt(digitalPinToInterrupt(RTC_CLK_OUT), clockTick, RISING);

  TimeUtils::parseTimezone(timezone);
  setenv("TZ", timezone.c_str(), 1);
  tzset();
  timeManager.start();
  timeManager.PCF8563osc1Hz();
  currentTimerRecord = timeManager.now();
  timeManager.printTimeToSerial(currentTimerRecord);
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(10);

  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB(255, 255, 0);
  }
  FastLED.show();
  //
  // lora code
  //
  SPI.begin(SCK, MISO, MOSI);
  pinMode(LoRa_SS, OUTPUT);
  pinMode(LORA_RESET, OUTPUT);
  pinMode(LORA_DI0, INPUT);
  digitalWrite(LoRa_SS, HIGH);
  delay(100);
  LoRa.setPins(LoRa_SS, LORA_RESET, LORA_DI0);


  // Small delay
  delay(50);





  if (!LoRa.begin(433E6)) {

    while (1)
      ;
    leds[1] = CRGB(255, 0, 0);
    display.clearDisplay();
    display.setTextSize(1);
    centerText("LORA Failed", 0);

  } else {

    //leds[1] = CRGB(0, 0, 255);
    display.clearDisplay();
    display.setTextSize(1);
    centerText("LORA Active", 0);
    loraActive = true;
    LoRa.enableCrc(); 
     LoRa.setSpreadingFactor(9);
    LoRa.setSignalBandwidth(125E3);
    //  LoRa.setSpreadingFactor(12);
    // LoRa.setSignalBandwidth(125E3);
    //  LoRa.setCodingRate4(8);
  }
  display.display();
  if (loraActive) {
    leds[1] = CRGB(0, 255, 0);

    // LoRa_rxMode();
    // LoRa.setSyncWord(0xF3);
    LoRa.onReceive(onReceive);
    // put the radio into receive mode
    LoRa.receive();

  } else {
    leds[1] = CRGB(255, 0, 0);
  }
  leds[0] = CRGB(32, 0, 0);  // WiFi not yet connected
  leds[2] = CRGB(255, 0, 0);  // weather stale at startup
  FastLED.show();
  delay(1000);
  //   gpio_matrix_out(18, 0x100, false, false);
  //pinMode(18, OUTPUT);  // Reconfigure as regular GPIO
  //pinMode(5, OUTPUT);   // Also set pin 5

  //  xTaskCreatePinnedToCore(
  //         ledShowTaskFunction,    // Task function
  //         "ledShowTask",          // Name of task
  //         5000,                   // Stack size
  //         NULL,                   // Parameter
  //         1,                      // Priority
  //         &ledShowTask,          // Task handle
  //         0                      // Core ID (0)
  //     );
  //    runLedShow = true;
  leds[1] = CRGB(0, 0, 0);
  leds[2] = CRGB(0, 0, 0);
  FastLED.show();
  Serial.println("finished setup");
}

bool doit = true;



void centerText(String text, int y) {
  display.setTextSize(1);
  display.setTextColor(WHITE);

  int16_t x1, y1;
  uint16_t w, h;

  // Get text bounds
  display.getTextBounds(text, 0, 0, &x1, &y1, &w, &h);

  // Calculate center position
  int x = (SCREEN_WIDTH - w) / 2;

  display.setCursor(x, y);
  display.print(text);
}

// R display: < 1.0 → ".75" style decimal, >= 1.0 → rounded integer right-justified in 3 chars
void formatRain(char* out, int size, double val) {
  if (val < 1.0) {
    int frac = min((int)round(val * 100), 99);
    snprintf(out, size, ".%02d", frac);
  } else {
    snprintf(out, size, "%3d", (int)round(val));
  }
}

void showWeatherForecast() {
  // textSize=1 on 128px wide display = 21 chars per line (128/6=21)
  // Data rows use 17 chars; title uses up to 21.
  display.clearDisplay();

  // Title: "WF dd/m/yy hh:mm:ss" — timestamp of last reception
  char title[22];
  snprintf(title, sizeof(title), "WF %d/%d/%02d %02d:%02d:%02d",
           weatherForecastReceivedTime.date,
           weatherForecastReceivedTime.month,
           weatherForecastReceivedTime.year % 100,
           weatherForecastReceivedTime.hour,
           weatherForecastReceivedTime.minute,
           weatherForecastReceivedTime.second);
  display.setCursor(0, 0);
  display.print(title);

  display.setCursor(0, 9);
  char buf[22];

  // Cloudiness (C) — integer %, 2 forecasts per row with hour
  // e.g. "C  9h  30 12h  50"  = 17 chars
  snprintf(buf, sizeof(buf), "C %2dh %3d %2dh %3d",
           storedForecasts[0].hour, storedForecasts[0].cloudiness,
           storedForecasts[1].hour, storedForecasts[1].cloudiness);
  display.println(buf);
  snprintf(buf, sizeof(buf), "  %2dh %3d %2dh %3d",
           storedForecasts[2].hour, storedForecasts[2].cloudiness,
           storedForecasts[3].hour, storedForecasts[3].cloudiness);
  display.println(buf);

  // Temperature (T) — integer degrees
  // e.g. "T  9h  22 12h  24"  = 17 chars
  snprintf(buf, sizeof(buf), "T %2dh %3d %2dh %3d",
           storedForecasts[0].hour, (int)storedForecasts[0].temperature,
           storedForecasts[1].hour, (int)storedForecasts[1].temperature);
  display.println(buf);
  snprintf(buf, sizeof(buf), "  %2dh %3d %2dh %3d",
           storedForecasts[2].hour, (int)storedForecasts[2].temperature,
           storedForecasts[3].hour, (int)storedForecasts[3].temperature);
  display.println(buf);

  // Rain proxy (R) — <1 shows ".75", >=1 shows rounded integer
  // e.g. "R  9h .75 12h  11"  = 17 chars
  char r0[5], r1[5], r2[5], r3[5];
  formatRain(r0, sizeof(r0), storedForecasts[0].rain);
  formatRain(r1, sizeof(r1), storedForecasts[1].rain);
  formatRain(r2, sizeof(r2), storedForecasts[2].rain);
  formatRain(r3, sizeof(r3), storedForecasts[3].rain);
  snprintf(buf, sizeof(buf), "R %2dh %s %2dh %s",
           storedForecasts[0].hour, r0, storedForecasts[1].hour, r1);
  display.println(buf);
  snprintf(buf, sizeof(buf), "  %2dh %s %2dh %s",
           storedForecasts[2].hour, r2, storedForecasts[3].hour, r3);
  display.println(buf);

  display.display();
}

void showChinampaPage1() {
  display.clearDisplay();
  centerText(chinampaData.devicename, 0);

  display.setTextSize(1);    // Switch to smaller text
  display.setCursor(0, 10);  // x=0 (left), y=20 (below title)'
  if (chinampaData.alertstatus) {
    if (chinampaData.alertcode >0  && chinampaData.alertcode<10) {
      display.print("Alrt:");
      if (chinampaData.alertcode == 1) {
        display.println("Fish Data Stale");
      } else if (chinampaData.alertcode == 2) {
        display.println("Sump Stale");
      } else if (chinampaData.alertcode == 3) {
        display.println("Fish & Sump Stale");
      } else if (chinampaData.alertcode == 4) {
        display.println("Fish flow<2");
      } else if (chinampaData.alertcode == 5) {
        display.println("Sump too low");
      } else if (chinampaData.alertcode == 6) {
        display.println("Add H20");
      } else if (chinampaData.alertcode == 7) {
        display.println("Fish Sensor Bad");
      } else if (chinampaData.alertcode == 8) {
        display.println("Sump Sensor Bad");
      } else if (chinampaData.alertcode == 9) {
        display.println("F & S Sensors Bad");
      }else{
        display.println(chinampaData.alertcode);
      }
    }
  }
  // line 2
  display.print("Pump: ");
  if (chinampaData.pumprelaystatus) {
    display.print("ON");
  } else {
    display.print("OFF");
  }

  display.print(" FS:");
  if (chinampaData.fishtankoutflowsolenoidrelaystatus) {
    display.println("OPEN");
  } else {
    display.println("CLOSED");
  }


  // line 3
  display.print("F Last:");
  if(chinampaData.secondsSinceLastFishTankData>0 && chinampaData.secondsSinceLastFishTankData<1000){
    display.print(chinampaData.secondsSinceLastFishTankData);
  }else{
    display.print("Bad");
  }
  display.print(" Flow:");
  display.println(chinampaData.fishtankoutflowflowRate);

  display.print("S Last:");
  if(chinampaData.secondsSinceLastSumpTroughData>0 && chinampaData.secondsSinceLastSumpTroughData<1000){
    display.print(chinampaData.secondsSinceLastSumpTroughData);
  }else{
     display.print("Bad");
  }
  
  display.print(" H:");
  display.print((int)chinampaData.sumpTroughMeasuredHeight);
  

  if (chinampaData.sumpTroughMeasuredHeight >= (chinampaData.sumpTroughHeight - chinampaData.minimumSumpTroughLevel)){
    display.println(" Red");
  }else if (chinampaData.sumpTroughMeasuredHeight < (chinampaData.sumpTroughHeight - chinampaData.minimumSumpTroughLevel) && chinampaData.sumpTroughMeasuredHeight >= (chinampaData.sumpTroughHeight - chinampaData.maximumSumpTroughLevel)) {
    display.println(" Green");
  } else if (chinampaData.sumpTroughMeasuredHeight < (chinampaData.sumpTroughHeight - chinampaData.maximumSumpTroughLevel)) {
    display.println("  Blue");
  }

  // display.print("Mi:");
  // display.print((int)chinampaData.minimumSumpTroughLevel);
  // display.print(" Ma:");
  // display.print((int)chinampaData.maximumSumpTroughLevel);
  // display.print(" TH:");
  // display.println((int)chinampaData.sumpTroughHeight);
  display.print("uT:");
  display.print(chinampaData.microtemperature);
  display.print(" ");
  display.print(chinampaData.outdoortemperature);
  display.print(" ");
  display.print(chinampaData.rtcBatVolt);
  display.println("V");
   float rssi = chinampaData.rssi;
  float snr = chinampaData.snr;
  display.print("rssi:");
  display.print((int)rssi);
  display.print(" snr:");
  display.println(snr);
  display.display();
}

void showChinampaPage2() {
  display.clearDisplay();
  display.setTextSize(1);  // Switch to smaller text
  display.setCursor(0, 0);
  centerText(chinampaData.devicename, 0);
  display.setCursor(0, 10);
 
  if (chinampaData.sensorstatus[0]) {
    display.println("uTemp high");
  } else {
    display.println("uTemp ok");
  }
  if (chinampaData.sensorstatus[1]) {
    display.println("Fish Tank Change>10%");
  } else {
    display.println("Fish Tank Ok");
  }
  if (chinampaData.sensorstatus[2]) {
    display.println("Sump Height Change>10%");
  } else {
    display.println("Sump Height Ok");
  }
  display.display();
}

bool isSentByArraySet(uint8_t* arr, size_t size) {
  // Check if array is empty (all zeros)
  for (size_t i = 0; i < size; i++) {
    if (arr[i] != 0) {
      return true;  // Array has data
    }
  }
  return false;  // Array is empty
}

// One-person field flow-sensor test: LEDs do the timing so the operator can be at the sensor/
// jug (which may be meters from the console) instead of typing commands at the exact moment
// water needs catching. Moved here from Daffodil.ino (2026-09-10) - Paula is the dedicated
// bench/field diagnostic tool for this, so the calibration/testing feature lives here instead.
// Blocking (delay-based) is deliberate and fine here: this only runs when an operator
// explicitly requests a test, for ~20s.
//
// Phase 1 (red, 15s): one LED lights per second, all 15 - gives the operator time to walk to the
// sensor and position the collection container. No pulses counted yet - water may already be
// running but isn't being captured into anything measurable.
// Phase 2 (green, 5s): one of the first 5 LEDs lights per second - the operator should be
// actively catching water now. Pulses ARE counted during exactly this window, nothing before or
// after it.
// Returns the pulse count captured during phase 2. A 0 with water confirmed running is itself a
// useful field diagnostic - the sensor (or its wiring) likely isn't producing pulses at all,
// independent of whatever qfactor math would otherwise follow.
unsigned long runFlowCalibrationLedSequence(volatile bool &calibratingFlag, volatile unsigned long &pulseCounter) {
  uint8_t savedBrightness = 10;  // Paula's normal LED brightness, set once in setup()
  FastLED.setBrightness(255);

  for (int i = 0; i < NUM_LEDS; i++) leds[i] = CRGB(0, 0, 0);
  FastLED.show();

  Serial.println("Phase 1/2: RED countdown, 15s - position the collection container now.");
  for (uint8_t i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB(255, 0, 0);
    FastLED.show();
    delay(1000);
  }

  for (int i = 0; i < NUM_LEDS; i++) leds[i] = CRGB(0, 0, 0);
  FastLED.show();

  pulseCounter = 0;
  calibratingFlag = true;
  Serial.println("Phase 2/2: GREEN capture window, 5s - catch water now.");
  for (uint8_t i = 0; i < 5; i++) {
    leds[i] = CRGB(0, 255, 0);
    FastLED.show();
    delay(1000);
  }
  calibratingFlag = false;

  for (int i = 0; i < NUM_LEDS; i++) leds[i] = CRGB(0, 0, 0);
  FastLED.show();
  FastLED.setBrightness(savedBrightness);

  unsigned long captured = pulseCounter;
  Serial.println("Capture window complete - pulses=" + String(captured) + (captured == 0 ? " (0 pulses with water running suggests a bad sensor or connection, not a qfactor problem)" : " - measure the water you just caught, then send the Stop command with that volume in mL"));
  return captured;
}

void loop() {
  // put your main code here, to run repeatedly:

  if (currentTestMode == TEST_MODE_ULTRASONIC) pollUltrasonicSerial();

  if (clockTicked) {
    portENTER_CRITICAL(&mux);
    clockTicked = false;
    portEXIT_CRITICAL(&mux);
    currentTimerRecord = timeManager.now();
    lastLoraReceptionSeconds++;
    lastWeatherForecastSeconds++;
    leds[4] = CRGB(0, 0, 0);
    leds[5] = CRGB(0, 0, 0);
    // The physical left/right toggle switch (previously read here on pins 32/33) no longer
    // exists on this board - the Daffodil shield replacing the Paula shield wires pin 33 to
    // Flow Sensor 2 instead. switchPositionLeft keeps its fixed default (see declaration) and
    // is never toggled now.
  }



  if (!switchPositionLeft) {
    showWeatherForecast();
  } else if (displayingChinampa && refreshChinampaPage) {
    showChinampaPage1();
    //refreshChinampaPage=false;
  }

  leds[0] = (currentIpAddress != "No IP") ? CRGB(0, 255, 0) : CRGB(32, 0, 0);
  leds[1] = loraActive ? CRGB(0, 255, 0) : CRGB(255, 0, 0);
  leds[2] = (lastWeatherForecastSeconds > WEATHER_STALE_INTERVAL) ? CRGB(255, 0, 0) : CRGB(0, 255, 0);
  leds[3] = (lastWeatherForecastSeconds > WEATHER_STALE_INTERVAL) ? CRGB(255, 0, 0) : CRGB(0, 255, 0);
  FastLED.show();
  if (loraReceived) {
    //Serial.printf("lora recive Free Heap: %d \n", xPortGetFreeHeapSize());
    if (debug) Serial.printf("lora recive loraPacketSize: %d \n", loraPacketSize);
    if (debug) Serial.println("");
    processLora(loraPacketSize);
    loraReceived = false;
    leds[4] = leds[5] = CRGB(0, 255, 0);
    FastLED.show();
    bool show = false;
    currentPalette = RainbowStripeColors_p;
    currentBlending = NOBLEND;

    //  performLedShow(250);
    ledShowDuration = 250;  // Set desired duration in milliseconds
    runLedShow = true;      // Set flag to trigger the show


    if (loraPacketSize == sizeof(LangleyData)) {


    } else if (loraPacketSize == sizeof(SeedlingMonitorData)) {

      //    leds[2] = CRGB(0, 0, 0);
      //   FastLED.show();
      //  display.clearDisplay();
      //   centerText(seedlingMonitorData.devicename, 0);

      //   display.setTextSize(1);  // Switch to smaller text
      //   display.setCursor(0, 20);  // x=0 (left), y=20 (below title)
      //    float rssi = seedlingMonitorData.rssi;
      //    float snr = seedlingMonitorData.snr;
      //   display.print("rssi: ");
      //   display.print((int)rssi);
      //   display.print(" snr: ");
      //   display.println(snr);
      //   display.display();

      //  if(rssi<-100 && rssi>-120){
      //     leds[3] = CRGB(255, 0, 0);
      //     Serial.println("line 435");
      //   }else if(rssi<=-90 && rssi>=-100){
      //     leds[3] = CRGB(255, 255, 0);
      //     Serial.println("line 438");
      //   }else if(rssi<=-70 && rssi>=-90){
      //     leds[3] = CRGB(0, 255, 0);
      //     Serial.println("line 441");

      //   }else if(rssi<=-30 && rssi>=-70){
      //     leds[3] = CRGB(0, 0, 255);
      //     Serial.println("line 445");

      //   }


      //    if(snr<-10 ){
      //     leds[4] = CRGB(255, 0, 0);
      //   }else if(snr<=0 && snr>=-5){
      //     leds[4] = CRGB(255, 255, 0);
      //   }else if(snr>=0 && snr<=5){
      //     leds[4] = CRGB(0, 255, 0);
      //   }else if(snr>5){2
      //     leds[4] = CRGB(0, 0, 255);
      //   }

      //   FastLED.show();
    } else if (loraPacketSize == sizeof(GloriaTankFlowPumpData)) {

    } else if (loraPacketSize == sizeof(DigitalStablesData)) {
     // leds[2] = CRGB(0, 0, 0);
      FastLED.show();
      //if(debug)
      if (debug) Serial.print("line 462, received from ");
      //if(debug)
      if (debug) Serial.print(digitalStablesData.devicename);
      if (debug) Serial.print("  sleep time=");
      if (debug) Serial.println(digitalStablesData.sleepTime);
    //  if (strstr(digitalStablesData.devicename, "TopTank") != NULL) {
        Serial.println("");
        Serial.println("*******************************");

        dataManager.printDigitalStablesData(digitalStablesData);
        float rssi = digitalStablesData.rssi;
        float snr = digitalStablesData.snr;
        if (switchPositionLeft) {
          display.clearDisplay();
          centerText(digitalStablesData.devicename, 0);
          display.setTextSize(1);    // Switch to smaller text
          display.setCursor(0, 10);  // x=0 (left), y=20 (below title)

          if(isSentByArraySet(digitalStablesData.sentbyarray, sizeof(digitalStablesData.sentbyarray))){
            display.print("Sent by: ");
            char buffer[9];  // 8 chars + null terminator
            memcpy(buffer, digitalStablesData.sentbyarray, 8);
            buffer[8] = '\0';  // Force null termination
            display.print(buffer);
            display.print(" ");
          }

          display.println( TimeUtils::epochToString(digitalStablesData.secondsTime));

          display.print("B ");
          display.print(digitalStablesData.batteryVoltage);
          display.print("V ");
          display.print(digitalStablesData.batteryCurrent);
          display.print("mA ");
          display.println(digitalStablesData.estimatedRuntime);

          display.print("rssi:");
          display.print((int)rssi);
          display.print(" snr:");
          display.println(snr);
          display.print("H: ");
          display.print(digitalStablesData.measuredHeight);
          uint8_t red = 255;
          uint8_t green = 255;
          uint8_t blue = 255;

          if (digitalStablesData.measuredHeight >= (digitalStablesData.maximumScepticHeight - digitalStablesData.troughlevelminimumcm)) {
            red = 255;
            green = 0;
            blue = 0;
            display.println("  Red");
          } else if (digitalStablesData.measuredHeight < (digitalStablesData.maximumScepticHeight - digitalStablesData.troughlevelminimumcm) && digitalStablesData.measuredHeight >= (digitalStablesData.maximumScepticHeight - digitalStablesData.troughlevelmaximumcm)) {
            red = 0;
            green = 255;
            blue = 0;
            display.println("  Green");
          } else if (digitalStablesData.measuredHeight < (digitalStablesData.maximumScepticHeight - digitalStablesData.troughlevelmaximumcm)) {
            red = 0;
            green = 0;
            blue = 255;
            display.println("  Blue");
          }

          display.print("Mi:");
          display.print((int)digitalStablesData.troughlevelminimumcm);
          display.print(" Ma:");
          display.print((int)digitalStablesData.troughlevelmaximumcm);
          display.print(" TH: ");
          display.println((int)digitalStablesData.maximumScepticHeight);

          display.print("u Temp:");
          display.print(digitalStablesData.temperature);
          display.print(" Out T:");
          display.print(digitalStablesData.outdoortemperature);

          display.display();
        } // end if (switchPositionLeft)

        if(debug)Serial.print("rssi= ");
        if(debug)Serial.print(digitalStablesData.rssi);
        if(debug)Serial.print("  snr=");
        if(debug)Serial.println(digitalStablesData.snr);
   //   }
      //  delay(500);
    } else if (loraPacketSize == sizeof(WeatherForecastUpdate)) {
      if (weatherForecastNewData) {
        leds[4] = leds[5] = CRGB(0, 100, 255);
        FastLED.show();
        showWeatherForecast();
        weatherForecastNewData = false;
      }
    } else if (loraPacketSize == sizeof(ChinampaData)) {
      displayingChinampa = true;

      //if(debug)
      if (debug) Serial.print("line 513, received from ");
      //if(debug)
      if (debug) Serial.println(chinampaData.devicename);

      if (switchPositionLeft) showChinampaPage1();
      else showWeatherForecast();


      float rssi = chinampaData.rssi;
      float snr = chinampaData.snr;
//      if(chinampaData.microtemperature<chinampaData.maximumSumpTroughLevel){
//        leds[2] = CRGB(0,255, 0);
//      }else{
//        leds[2] = CRGB(255, 0, 0);
//      }
      
      if(debug)Serial.print("rssi= ");
      if(debug)Serial.print(chinampaData.rssi);
      if(debug)Serial.print("  snr=");
      if(debug)Serial.println(chinampaData.snr);
      //  delay(500);
    } else if (loraPacketSize > 0) {
      if (switchPositionLeft) {
        display.clearDisplay();
        centerText("Unidentified", 0);

        display.setTextSize(1);    // Switch to smaller text
        display.setCursor(0, 20);  // x=0 (left), y=20 (below title)
        display.print("loraPacketSize: ");
        display.println(loraPacketSize);

        float rssi = digitalStablesData.rssi;
        float snr = digitalStablesData.snr;

        display.print("rssi: ");
        display.print(rssi);
        display.print(" snr: ");
        display.println(snr);
        display.print("Height: ");

        display.display();
      }
    }
    // currentPalette = RainbowStripeColors_p;
    //  currentBlending = NOBLEND;
    //   if(!inSerial && show)performLedShow(100);
  }
  loraPacketSize = 0;

  if (Serial.available() != 0) {
  
    String command = Serial.readString();
    if (command.startsWith("SetTime")) {
      //SetTime#8#5#24#4#18#22#25
        timeManager.setTime(command);
      
    }else if(command.startsWith("debug")){
      int debugv = generalFunctions.getValue(command, '#', 1).toInt();
        if(debugv>0)debug=true;
        else debug=false;
    Serial.println("Ok-debug");
        Serial.flush();
    } else if (command.startsWith("SetFieldId")) {
      // fieldId= GeneralFunctions::getValue(command, '#', 1).toInt();
    } else if (command.startsWith("GetTime")) {
      timeManager.printTimeToSerial(currentTimerRecord);
      Serial.flush();
      Serial.flush();
    } else if (command.startsWith("GetCommandCode")) {
      long code = secretManager.generateCode();
      //
      // patch a bug in the totp library
      // if the first digit is a zero, it
      // returns a 5 digit number
      if (code < 100000) {
        Serial.print("0");
        Serial.println(code);
      }

      Serial.flush();
      // // delay(delayTime);
    }else if (command.startsWith("VerifyUserCode")) {
      String codeInString = generalFunctions.getValue(command, '#', 1);
      long userCode = codeInString.toInt();
      boolean validCode = true;  //secretManager.checkCode( userCode);
      String result = "Failure-Invalid Code";
      if (validCode) result = "Ok-Valid Code";
      Serial.println(result);
      Serial.flush();
      //// delay(delayTime);
    } else if (command.startsWith("GetSecret")) {
      //uint8_t switchState = digitalRead(OP_MODE);
      //if (switchState == LOW) {
        //  char secretCode[SHARED_SECRET_LENGTH];
        String secretCode = secretManager.readSecret();
        Serial.println(secretCode);
        Serial.println("Ok-GetSecret");
     // } else {
    //    Serial.println("Failure-GetSecret");
   //   }
      Serial.flush();
      // // delay(delayTime);
    } else if (command.startsWith("SetSecret")) {
      //uint8_t switchState = digitalRead(OP_MODE);
    //  if (switchState == LOW) {
        //SetSecret#IZQWS3TDNB2GK2LO#6#30
        String secret = generalFunctions.getValue(command, '#', 1);
        int numberDigits = generalFunctions.getValue(command, '#', 2).toInt();
        int periodSeconds = generalFunctions.getValue(command, '#', 3).toInt();
        secretManager.saveSecret(secret, numberDigits, periodSeconds);
        Serial.println("Ok-SetSecret");
        Serial.flush();
        //// delay(delayTime);
   //   } else {
    //    Serial.println("Failure-SetSecret");
    //  }


    } else if (command == "Flush") {
      while (Serial.read() >= 0)
        ;
      Serial.println("Ok-Flush");
      Serial.flush();
    } else if (command.startsWith("PulseStart")) {
      inPulse = true;
      Serial.println("Ok-PulseStart");
      Serial.flush();
      // delay(delayTime);

    } else if (command.startsWith("PulseFinished")) {
      
      inPulse = false;
      Serial.println("Ok-PulseFinished");
      Serial.flush();
      // delay(delayTime);

    } else if (command.startsWith("IPAddr")) {
      currentIpAddress = generalFunctions.getValue(command, '#', 1);
      Serial.println("Ok-IPAddr");
      Serial.flush();
      // delay(delayTime);
    } else if (command.startsWith("SSID")) {
      String currentSSID = generalFunctions.getValue(command, '#', 1);
      //wifiManager.setCurrentSSID(currentSSID.c_str());
      Serial.println("Ok-currentSSID");
      Serial.flush();
      // delay(delayTime);
    } else if (command.startsWith("GetIpAddress")) {
      //Serial.println(wifiManager.getIpAddress());
      Serial.println("Ok-GetIpAddress");
      Serial.flush();
      // delay(delayTime);
    } else if (command.startsWith("RestartWifi")) {
     // wifiManager.restartWifi();
      Serial.println("Ok-restartWifi");
      Serial.flush();
      // delay(delayTime);
    }else if (command.startsWith("GetSensorData")) {
        DigitalStablesDataSerializer digitalStablesDataSerializer;
        digitalStablesDataSerializer.pushToSerial(Serial,digitalStablesData );
        Serial.flush();  
    } else if (command.startsWith("AsyncData")) {

      if (gloriaTankFlowPumpNewData) {
        //gloriaTankFlowPumpSerializer.pushToSerial(Serial, gloriaTankFlowPumpData);
        dataManager.processGloriaQueue();
        gloriaTankFlowPumpNewData = false;
      }

      if (digitalStablesDataNewData) {
         // 00digitalStablesDataSerializer.pushToSerial(Serial, digitalStablesData);
         dataManager.processDigitalStablesDataQueue();
         digitalStablesDataNewData = false;
      }

      if (chinampaDataNewData) {
         // 00digitalStablesDataSerializer.pushToSerial(Serial, digitalStablesData);
         dataManager.processChinampaDataQueue();
         chinampaDataNewData = false;
      }
      
      Serial.println("Ok-AsyncData");
      Serial.flush();
      // delay(delayTime);
    } else if (command.startsWith("GetLifeCycleData")) {
      
      CRGB orange = CRGB::Amethyst;
      CRGB orangered = CRGB::Azure;
      //performLedShow(500);
      Serial.println("Ok-GetLifeCycleData");
      Serial.flush();
    } else if (command.startsWith("GetWPSSensorData")) {
      Serial.println("Ok-GetWPSSensorData");
      Serial.flush();
    } else if (command.startsWith("SetStatusText")) {
      // Field-uploader bridge (Pi 3B host): overrides the LoRa-hub display with a one-line
      // status message. Stays up until the next SetStatusText or the next LoRa packet redraw
      // elsewhere in loop() - no dedicated "clear" command needed for the single-job bridge flow.
      String statusText = generalFunctions.getValue(command, '#', 1);
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0, 0);
      display.println(statusText);
      display.display();
      Serial.println("Ok-SetStatusText");
      Serial.flush();
    } else if (command.startsWith("GetSwitchState")) {
      // Left/Right pins no longer host a physical switch on this board (33 is now Flow
      // Sensor 2 - see the shield swap note near SENSOR_INPUT_2's declaration).
      int sleep = digitalRead(SLEEP_SWITCH_26);
      Serial.print("Sleep=");
      Serial.println(sleep);
      Serial.println("Ok-GetSwitchState");
      Serial.flush();
    } else if (command.startsWith("SetTestMode")) {
      // Pin 18 is shared between Flow Sensor 1's interrupt and the DYP-A02 ultrasonic sensor's
      // UART TX line (see ultrasonicSerial's declaration comment) - only one can own the pin at
      // a time, so switching modes detaches/reattaches the interrupt and starts/stops the UART
      // port accordingly. Flow Sensor 2 (pin 33) is unaffected either way.
      String mode = generalFunctions.getValue(command, '#', 1);
      if (mode == "Ultrasonic") {
        if (currentTestMode != TEST_MODE_ULTRASONIC) {
          detachInterrupt(SENSOR_INPUT_1);
          ultrasonicFrameBufLen = 0;
          lastUltrasonicDistanceMM = 0;
          lastUltrasonicReadMillis = 0;
          ultrasonicSerial.begin(9600, SERIAL_8N1, SENSOR_INPUT_1, -1);
          currentTestMode = TEST_MODE_ULTRASONIC;
        }
        Serial.println("Ok-SetTestMode-Ultrasonic");
      } else if (mode == "Flow") {
        if (currentTestMode != TEST_MODE_FLOW) {
          ultrasonicSerial.end();
          attachInterrupt(SENSOR_INPUT_1, pulseCounter, FALLING);
          currentTestMode = TEST_MODE_FLOW;
        }
        Serial.println("Ok-SetTestMode-Flow");
      } else {
        Serial.println("Failure-SetTestMode-unknown mode " + mode + " (use Flow or Ultrasonic)");
      }
      Serial.flush();
    } else if (command.startsWith("GetUltrasonicReading")) {
      if (currentTestMode != TEST_MODE_ULTRASONIC) {
        Serial.println("Failure-GetUltrasonicReading-not in Ultrasonic mode (send SetTestMode#Ultrasonic first)");
      } else {
        unsigned long ageMs = lastUltrasonicReadMillis == 0 ? 0 : (millis() - lastUltrasonicReadMillis);
        Serial.println("distanceMM=" + String(lastUltrasonicDistanceMM) + " ageMs=" + String(ageMs) + (lastUltrasonicReadMillis == 0 ? " (no valid frame received yet - check wiring)" : ""));
        Serial.println("Ok-GetUltrasonicReading");
      }
      Serial.flush();
    } else if (command.startsWith("TestI2CTemperatureSensor")) {
      // One-shot: re-runs init() fresh every call rather than trusting a prior result, since the
      // whole point is diagnosing "not detected" (confirmed real-world case, 2026-09-12: TopTank
      // installed, its SHT3x not being detected) - a stale success from an earlier call would
      // hide exactly the failure this test exists to catch.
      if (!shtTest.init()) {
        Serial.println("Failure-TestI2CTemperatureSensor-not detected on I2C bus (expected at 0x44 - check wiring/address)");
      } else {
        shtTest.setAccuracy(SHTSensor::SHT_ACCURACY_MEDIUM);  // only supported by SHT3x
        if (shtTest.readSample()) {
          Serial.println("temperature=" + String(shtTest.getTemperature(), 2) + " humidity=" + String(shtTest.getHumidity(), 2));
          Serial.println("Ok-TestI2CTemperatureSensor");
        } else {
          Serial.println("Failure-TestI2CTemperatureSensor-detected but readSample() failed");
        }
      }
      Serial.flush();
    } else if (command.startsWith("TestTankPressureSensor1")) {
      // Same terminal as Flow Sensor 1 (pin 18), but the ANALOG jumper position instead of
      // digital - firmware can't verify the jumper is set right, only that the ADS1115 itself
      // answers on I2C. isConnected() is checked fresh here rather than relying on any boot-time
      // flag, same reasoning as TestI2CTemperatureSensor's init() above.
      if (!ADS.isConnected()) {
        Serial.println("Failure-TestTankPressureSensor1-ADS1115 not detected on I2C bus (expected at 0x48 - check wiring/address)");
      } else {
        ADS.setGain(0);
        int16_t raw = ADS.readADC(1);
        float volts = raw * ADS.toVoltage(1);
        float psi = (volts - 0.5f) * 1.25f;
        Serial.println("tank1 volts=" + String(volts, 3) + " psi=" + String(psi, 3) + (volts < 0.2f ? " (near 0V - a live-zero transducer reading ~0V usually means disconnected/faulty, not truly 0psi - also check the jumper is on the ANALOG position)" : ""));
        Serial.println("Ok-TestTankPressureSensor1");
      }
      Serial.flush();
    } else if (command.startsWith("TestTankPressureSensor2")) {
      if (!ADS.isConnected()) {
        Serial.println("Failure-TestTankPressureSensor2-ADS1115 not detected on I2C bus (expected at 0x48 - check wiring/address)");
      } else {
        ADS.setGain(0);
        int16_t raw = ADS.readADC(0);
        float volts = raw * ADS.toVoltage(1);
        float psi = (volts - 0.5f) * 1.25f;
        Serial.println("tank2 volts=" + String(volts, 3) + " psi=" + String(psi, 3) + (volts < 0.2f ? " (near 0V - a live-zero transducer reading ~0V usually means disconnected/faulty, not truly 0psi - also check the jumper is on the ANALOG position)" : ""));
        Serial.println("Ok-TestTankPressureSensor2");
      }
      Serial.flush();
    } else if (command.startsWith("CalibrateFlowSensor1Start")) {
      if (currentTestMode != TEST_MODE_FLOW) {
        Serial.println("Failure-CalibrateFlowSensor1Start-not in Flow mode (send SetTestMode#Flow first - pin 18 is currently set up for ultrasonic UART)");
        Serial.flush();
      } else {
      // One-person field test: turn the water on FIRST, then send this. It blocks for ~20s
      // running the LED-guided countdown/capture sequence (see runFlowCalibrationLedSequence's
      // comment) - no need to be at the console during that window, the LEDs do the timing.
      // Once it returns, measure the water you caught during the green phase and send
      // CalibrateFlowSensor1Stop#<measuredMilliLitres>.
      Serial.println("Ok-CalibrateFlowSensor1Start");
      Serial.flush();
      runFlowCalibrationLedSequence(calibratingFlow1, calibrationPulseCount1);
      Serial.flush();
      }
    } else if (command.startsWith("CalibrateFlowSensor1Stop")) {
      calibratingFlow1 = false;
      float measuredMilliLitres = generalFunctions.stringToDouble(generalFunctions.getValue(command, '#', 1));
      if (measuredMilliLitres <= 0 || calibrationPulseCount1 == 0) {
        Serial.println("Failure-CalibrateFlowSensor1Stop-need a positive measured volume and at least one pulse counted (pulses=" + String(calibrationPulseCount1) + ")");
      } else {
        // qfactor = pulses / (liters * 60) - the flow duration/rate cancels out of the algebra
        // entirely, so the operator doesn't need to time anything.
        float newQFactor1 = calibrationPulseCount1 / ((measuredMilliLitres / 1000.0f) * 60.0f);
        digitalStablesData.qfactor1 = newQFactor1;
        secretManager.saveQFactor1(newQFactor1);
        Serial.println("pulses=" + String(calibrationPulseCount1) + " measuredMilliLitres=" + String(measuredMilliLitres) + " newQFactor1=" + String(newQFactor1, 4));
        Serial.println("Ok-CalibrateFlowSensor1Stop");
      }
      Serial.flush();
    } else if (command.startsWith("CalibrateFlowSensor2Start")) {
      Serial.println("Ok-CalibrateFlowSensor2Start");
      Serial.flush();
      runFlowCalibrationLedSequence(calibratingFlow2, calibrationPulseCount2);
      Serial.flush();
    } else if (command.startsWith("CalibrateFlowSensor2Stop")) {
      calibratingFlow2 = false;
      float measuredMilliLitres2 = generalFunctions.stringToDouble(generalFunctions.getValue(command, '#', 1));
      if (measuredMilliLitres2 <= 0 || calibrationPulseCount2 == 0) {
        Serial.println("Failure-CalibrateFlowSensor2Stop-need a positive measured volume and at least one pulse counted (pulses=" + String(calibrationPulseCount2) + ")");
      } else {
        float newQFactor2 = calibrationPulseCount2 / ((measuredMilliLitres2 / 1000.0f) * 60.0f);
        digitalStablesData.qfactor2 = newQFactor2;
        secretManager.saveQFactor2(newQFactor2);
        Serial.println("pulses=" + String(calibrationPulseCount2) + " measuredMilliLitres=" + String(measuredMilliLitres2) + " newQFactor2=" + String(newQFactor2, 4));
        Serial.println("Ok-CalibrateFlowSensor2Stop");
      }
      Serial.flush();
    } else {
      //
      // call read to flush the incoming
      //
      Serial.println("Failure-Command Not Found-" + command);
      Serial.flush();
      // delay(delayTime);
    }
  }
}
