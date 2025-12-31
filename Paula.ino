
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
#include <PCF8563TimeManager.h>
#include <Esp32SecretManager.h>
//#include <Arduino_JSON.h>
#include <Wire.h>
#include <DataManager.h>
#include <LittleFS.h>
bool gloriaTankFlowPumpNewData = false;

bool digitalStablesDataNewData=false;
bool chinampaDataNewData=false;
String currentIpAddress = "No IP";
bool inPulse=false;
#define SCK 14
#define MOSI 13
#define MISO 12
#define LoRa_SS 15
#define LORA_RESET 16
#define LORA_DI0 17

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Display settings for 2.4" SSD1309
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1        // Reset pin (-1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C  // Common I2C address (try 0x3D if this doesn't work)
#define SLEEP_SWITCH_26 26

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
DataManager dataManager(Serial, LittleFS);


const int SWITCH_PIN_LEFT = 32;   // Connected to Pin 1
const int SWITCH_PIN_RIGHT = 33;  // Connected to Pin 3

#define LED_PIN 19
#define NUM_LEDS 5
#define STALE_INTERVAL 20

RTCInfoRecord currentTimerRecord;
#define TIME_RECORD_REFRESH_SECONDS 3
bool switchPositionLeft = true;
volatile bool clockTicked = false;
volatile bool lowVoltageAlert = false;
int lastLoraReceptionSeconds = 0;

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

boolean debug = true;
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

void onReceive(int packetSize) {

  loraReceived = true;
  loraPacketSize = packetSize;
}


void processLora(int packetSize) {

  lastLoraReceptionSeconds = 0;



  if (packetSize == 0) return;  // if there's no packet, return
  displayingChinampa = false;
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
    //if(debug)
    if (debug) Serial.print("received digitalStablesData from ");
    if (debug) Serial.print(digitalStablesData.devicename);

    if (debug) Serial.print(" ");
    if (debug) Serial.print(TimeUtils::epochToString(digitalStablesData.secondsTime));



    if (debug) Serial.print(" current=");
    if (debug) Serial.print(digitalStablesData.capacitorCurrent);
    if (debug) Serial.print(" operatingStatus=");
    if (debug) Serial.println(digitalStablesData.operatingStatus);
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


  pinMode(SWITCH_PIN_LEFT, INPUT_PULLUP);
  pinMode(SWITCH_PIN_RIGHT, INPUT_PULLUP);

  pinMode(RTC_CLK_OUT, INPUT_PULLUP);  // set up interrupt pin
  digitalWrite(RTC_CLK_OUT, HIGH);     // turn on pullup resistors
  // attach interrupt to set_tick_tock callback on rising edge of INT0
  attachInterrupt(digitalPinToInterrupt(RTC_CLK_OUT), clockTick, RISING);

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
    leds[0] = CRGB(255, 0, 0);
    display.clearDisplay();
    display.setTextSize(1);
    centerText("LORA Failed", 0);

  } else {

    //leds[1] = CRGB(0, 0, 255);
    display.clearDisplay();
    display.setTextSize(1);
    centerText("LORA Active", 0);
    loraActive = true;
    //  LoRa.setSpreadingFactor(12);
    // LoRa.setSignalBandwidth(125E3);
    //  LoRa.setCodingRate4(8);
  }
  display.display();
  if (loraActive) {
    leds[0] = CRGB(0, 255, 0);

    // LoRa_rxMode();
    // LoRa.setSyncWord(0xF3);
    LoRa.onReceive(onReceive);
    // put the radio into receive mode
    LoRa.receive();

  } else {
    leds[0] = CRGB(255, 0, 0);
  }
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

void showChinampaPage1() {
  display.clearDisplay();
  centerText(chinampaData.devicename, 0);

  display.setTextSize(1);    // Switch to smaller text
  display.setCursor(0, 10);  // x=0 (left), y=20 (below title)'
  if (chinampaData.alertstatus) {
    leds[2] = CRGB(255, 0, 0);
    if (chinampaData.alertcode >0  && chinampaData.alertcode<6) {
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
      }else{
        display.println(chinampaData.alertcode);
      }
    }
  } else {
    leds[2] = CRGB(0, 0, 0);
  }
  FastLED.show();
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
  display.print(" RTC:");
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

void loop() {
  // put your main code here, to run repeatedly:

  if (clockTicked) {
    portENTER_CRITICAL(&mux);
    clockTicked = false;
    portEXIT_CRITICAL(&mux);
    currentTimerRecord = timeManager.now();
    lastLoraReceptionSeconds++;
    int left = digitalRead(SWITCH_PIN_LEFT);
    int right = digitalRead(SWITCH_PIN_RIGHT);



    if (left == LOW && right == HIGH) {
      //Serial.println("  Position: LEFT");
      if (switchPositionLeft) {
        // no change
        refreshChinampaPage = false;
      } else {
        if (displayingChinampa) refreshChinampaPage = true;
      }
      switchPositionLeft = true;
    } else if (left == HIGH && right == LOW) {
      // Serial.println("  Position: RIGHT");
      if (switchPositionLeft) {
        // change
        if (displayingChinampa) refreshChinampaPage = true;
      } else {
        //  no chanfge
        refreshChinampaPage = false;
      }
      switchPositionLeft = false;
    }
  }



  if (displayingChinampa && refreshChinampaPage) {
    if (switchPositionLeft) showChinampaPage1();
    else showChinampaPage2();
    //refreshChinampaPage=false;
  }

  if (lastLoraReceptionSeconds > STALE_INTERVAL) {
    leds[1] = CRGB(255, 0, 0);
  } else {
    leds[1] = CRGB(0, 255, 0);
  }
  FastLED.show();
  if (loraReceived) {
    //Serial.printf("lora recive Free Heap: %d \n", xPortGetFreeHeapSize());
    if (debug) Serial.printf("lora recive loraPacketSize: %d \n", loraPacketSize);
    if (debug) Serial.println("");
    processLora(loraPacketSize);
    loraReceived = false;
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
      //   }else if(snr>5){
      //     leds[4] = CRGB(0, 0, 255);
      //   }

      //   FastLED.show();
    } else if (loraPacketSize == sizeof(GloriaTankFlowPumpData)) {

    } else if (loraPacketSize == sizeof(DigitalStablesData)) {
      leds[2] = CRGB(0, 0, 0);
      FastLED.show();
      //if(debug)
      if (debug) Serial.print("line 462, received from ");
      //if(debug)
      if (debug) Serial.print(digitalStablesData.devicename);
      if (debug) Serial.print("  sleep time=");
      if (debug) Serial.println(digitalStablesData.sleepTime);

      if (isSentByArraySet(digitalStablesData.sentbyarray, sizeof(digitalStablesData.sentbyarray))) {
        display.clearDisplay();
        centerText(digitalStablesData.devicename, 0);
        display.setTextSize(1);    // Switch to smaller text
        display.setCursor(0, 10);  // x=0 (left), y=20 (below title)
        display.print("Sent by: ");
        // Cast to char* for printing
        // Ensure null termination for safety
        char buffer[9];  // 8 chars + null terminator
        memcpy(buffer, digitalStablesData.sentbyarray, 8);
        buffer[8] = '\0';  // Force null termination
        display.println(buffer);
        float rssi = digitalStablesData.rssi;
        float snr = digitalStablesData.snr;
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

        leds[1] = CRGB(red, green, blue);
        FastLED.show();





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

        if (rssi < -100) {
          leds[3] = CRGB(255, 0, 0);
          if(debug)Serial.println("line 491");

        } else if (rssi <= -90 && rssi >= -100) {
          leds[3] = CRGB(255, 255, 0);
          if(debug)Serial.println("line 496");

        } else if (rssi <= -70 && rssi >= -90) {
          leds[3] = CRGB(0, 255, 0);
          if(debug)Serial.println("line 500");

        } else if (rssi <= -30 && rssi >= -70) {
          leds[3] = CRGB(0, 0, 255);
          if(debug)Serial.println("line 504");
        }
        FastLED.show();

        if (snr < -10) {
          leds[4] = CRGB(255, 0, 0);
        } else if (snr <= 0 && snr >= -5) {
          leds[4] = CRGB(255, 255, 0);
        } else if (snr >= 0 && snr <= 5) {
          leds[4] = CRGB(0, 255, 0);
        } else if (snr > 5) {
          leds[4] = CRGB(0, 0, 255);
        }
        FastLED.show();
        if(debug)Serial.print("rssi= ");
        if(debug)Serial.print(digitalStablesData.rssi);
        if(debug)Serial.print("  snr=");
        if(debug)Serial.println(digitalStablesData.snr);
      }
      //  delay(500);
    } else if (loraPacketSize == sizeof(ChinampaData)) {
      displayingChinampa = true;

      //if(debug)
      if (debug) Serial.print("line 513, received from ");
      //if(debug)
      if (debug) Serial.println(chinampaData.devicename);

      if (switchPositionLeft) showChinampaPage1();
      else showChinampaPage2();


      float rssi = chinampaData.rssi;
      float snr = chinampaData.snr;

      if (rssi < -100) {
        leds[3] = CRGB(255, 0, 0);
      } else if (rssi <= -90 && rssi >= -100) {
        leds[3] = CRGB(255, 255, 0);
      } else if (rssi <= -70 && rssi >= -90) {
        leds[3] = CRGB(0, 255, 0);
      } else if (rssi <= -30 && rssi >= -70) {
        leds[3] = CRGB(0, 0, 255);
      }
      FastLED.show();

      if (snr < -10) {
        leds[4] = CRGB(255, 0, 0);
      } else if (snr <= 0 && snr >= -5) {
        leds[4] = CRGB(255, 255, 0);
      } else if (snr >= 0 && snr <= 5) {
        leds[4] = CRGB(0, 255, 0);
      } else if (snr > 5) {
        leds[4] = CRGB(0, 0, 255);
      }
      FastLED.show();
     if(debug) Serial.print("rssi= ");
      if(debug)Serial.print(chinampaData.rssi);
      if(debug)Serial.print("  snr=");
      if(debug)Serial.println(chinampaData.snr);
      //
      // line 5 sump
      //


      uint8_t red = 255;
      uint8_t green = 255;
      uint8_t blue = 255;

      if (chinampaData.sumpTroughMeasuredHeight >= (chinampaData.sumpTroughHeight - chinampaData.minimumSumpTroughLevel)) {
        red = 255;
        green = 0;
        blue = 0;
      } else if (chinampaData.sumpTroughMeasuredHeight < (chinampaData.sumpTroughHeight - chinampaData.minimumSumpTroughLevel) && chinampaData.sumpTroughMeasuredHeight >= (chinampaData.sumpTroughHeight - chinampaData.maximumSumpTroughLevel)) {
        red = 0;
        green = 255;
        blue = 0;

      } else if (chinampaData.sumpTroughMeasuredHeight < (chinampaData.sumpTroughHeight - chinampaData.maximumSumpTroughLevel)) {
        red = 0;
        green = 0;
        blue = 255;
      }
      leds[1] = CRGB(red, green, blue);
      FastLED.show();

      //  delay(500);
    } else if (loraPacketSize > 0) {
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
