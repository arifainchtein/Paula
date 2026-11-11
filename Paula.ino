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
#include <VitalSignsRecord.h>
#include <DeviceCommandRecord.h>
#include <DeviceCommandQueue.h>
#include <LinkAckRecord.h>
#include <LinkHealthRecord.h>  // relayed (2026-11-10), see the relay section
//#include <Arduino_JSON.h>
#include <Wire.h>
#include <DataManager.h>
#include <LittleFS.h>
#include "ADS1X15.h"
#include "OneWire.h"
#include "DallasTemperature.h"
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
// Shown on the OLED (start-up and the "LORA Active" screen), answered to GetFirmware and in
// GetNearbyDevices (PaulaDeployer's Nearby Devices shows it) - like Annabelle. Add 1 to firmwareVersion
// with every flashed change; the build time tells two builds of one version apart.
// 1 = 2026-11-03: Nearby Devices map positions, Ping (signal survey), the first numbered build.
// 2 = 2026-11-03: a factory product like the rest - GetProductDefinition / SetProductDefinition /
//     GetSerialNumber (Inspect, deploy packages, Allocate Customer); plain "Ping" is Ok-Ping again.
// 3 = 2026-11-03: 150 ms for the power rails before the OLED is initialised (dark after a cold boot).
// 4 = 2026-11-07: ReadClipsal / StopClipsal - PaulaDeployer's "Clipsal Uart" test (ultrasonic on 18 +
//     DS18B20 water temperature on 33 through a ClipsallJunctionBoxWaterTrough board).
// 5 = 2026-11-07: ReadClipsal2US - "Clipsal 2US" test, two DYP-A02 UART ultrasonics (18 and 33) through a
//     ClipsallJunctionBoxWaterTrough_2US board; the second one on UART1 (ultrasonicSerial2).
// 6 = 2026-11-07: ReadVoltageMonitor - "Clipsal Voltage Monitor" test (INA228 at 0x4F + battery NTC on the
//     pin-18 terminal, ANALOG jumper, ADS1115 AIN1).
// 7 = 2026-11-07: ReadVoltageMonitor reads the NTC on AIN0 (pin-33 terminal, JP5) per the board's spec.
// 8 = 2026-11-10: Nearby Devices keeps each device's last packet as received (GetNearbyPacket - the
//     page's "Data" button, which also checks the packet's layout) and lists Chinampa too.
// 9 = 2026-11-10: relay mode (Relay#1/#0 - resends what it hears to Annabelle, for testing a relay
//     spot) and the channel's noise level where Paula stands (GetNoise, and in GetNearbyDevices).
//     Also (2026-11-11): ScanI2C (lists what answers on the I2C bus - to see whether the OLED is there)
//     and Help / ListCommands (lists the serial commands). OLED_SMALL: a 0.91" 128x32 screen that
//     shows the name of each device heard, big (showHeard). Relayed DigitalStablesData/ChinampaData
//     carry sentbyarray "Paula". LEDs show rotating status glyphs (L F R, green/red) like Daffodil.
// 10 = 2026-11-11: the LEDs show rotating status glyphs like Daffodil - L (LoRa), F (forecast), R (relay),
//      each green = good / on, red = bad / off; a packet heard = a short rainbow light show. Relay
//      ticker: the OLED's bottom line shows "> <name> relayed" for 5 s after each packet relayed.
// 11 = 2026-11-11: Relay / GetNearbyDevices report the last packet relayed ("last", "lastAgeSec") -
//      PaulaDeployer's relay ticker line on Nearby Devices.
// 12 = 2026-11-11: relays LangleyData too (it has a sentbyarray now, 164 bytes). Strong-signal rule: a
//      packet heard at -80 dBm or stronger isn't relayed (Relay#1#<dBm> changes it, 0 = relay
//      everything) - the ticker says "<name> strong -58". LEDs capped at 100 mA (FastLED power limit) and
//      the packet rainbow on the middle row only - V50 sagged to 3.96 V and the OLED blanked.
const int firmwareVersion = 12;
#define FIRMWARE_BUILD_TIME __DATE__ " " __TIME__

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

// "Clipsal Uart" test (2026-11-07, PaulaDeployer's yellow Clipsal Uart button): a
// ClipsallJunctionBoxWaterTrough board wired to Paula's terminals as it would be to a Daffodil in
// DAFFODIL_WATER_TROUGH_WATER_TEMP mode - DYP-A02 UART TX on pin 18, waterproof DS18B20 on pin 33.
// Same as Daffodil.ino's waterOneWire: no external pull-up, the shield's 2N7002 level shifter on the
// pin-33 terminal (10k to V50 on the terminal side) is the bus pull-up, so the jumper must be on
// DIGITAL. ReadClipsal takes pin 18 as ultrasonic UART and pin 33 off its flow interrupt;
// StopClipsal (or SetTestMode#Flow) gives both back to the flow sensors.
OneWire clipsalOneWire(33);  // SENSOR_INPUT_2, defined below
DallasTemperature clipsalTempSensor(&clipsalOneWire);
bool clipsalActive = false;

// "Clipsal 2US" (2026-11-07): the ClipsallJunctionBoxWaterTrough_2US board has two DYP-A02s, one per
// Grove socket - left socket (J1) on J2 "18", right socket (J7) on J2 "33", both RX unconnected so both
// stream in automatic mode. The second one is read on UART1 with RX on pin 33 (same digital-jumper
// level shifter as pin 18). Same 4-byte frame as ultrasonicSerial, parsed by its own buffer.
HardwareSerial ultrasonicSerial2(1);
bool clipsal2USActive = false;
uint16_t lastUltrasonic2DistanceMM = 0;
unsigned long lastUltrasonic2ReadMillis = 0;
uint8_t ultrasonic2FrameBuf[4];
uint8_t ultrasonic2FrameBufLen = 0;

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
// OLED_SMALL 1 = a 0.91" 128x32 OLED (2026-11-11: Paula's 0.96" 128x64 died, the 0.91" is what's on hand).
// Too small for the detailed pages, so it only shows who Paula just heard over LoRa - the device's
// name in big letters, then its RSSI/SNR and the time (showHeard). 0 = the 128x64 screens as before.
#define OLED_SMALL 1
#define SCREEN_WIDTH 128
#if OLED_SMALL
#define SCREEN_HEIGHT 32
#else
#define SCREEN_HEIGHT 64
#endif
const bool oledDetailed = !OLED_SMALL;  // the 128x64 pages (Chinampa, weather, DigitalStablesData, Unidentified)
#define OLED_RESET -1        // Reset pin (-1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C  // default; setup() uses 0x3D instead when that is where the OLED answers
#define SLEEP_SWITCH_26 26
#define LED_CONTROL 23

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
// Starts stale: LEDs 2-3 turn green only after a forecast with a valid code arrives (it started at 0,
// so they were green for 2 h after every boot whether or not Paula's clock was right).
int lastWeatherForecastSeconds = WEATHER_STALE_INTERVAL + 1;

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

// Nearby devices and WiFi commands (2026-10-14). Paula sends the same DeviceCommandRecord as
// Annabelle's web page, so a Daffodil's WiFi can be switched on standing next to it, with
// PaulaDeployer's "Nearby devices" list (serial commands GetNearbyDevices / DeviceWifi). Delivery
// works like Annabelle's (DeviceCommandQueue.h): resent after the device's VitalSigns until its
// data shows the change. Everything runs in loop() - onReceive() only sets flags.
#define NEARBY_MAX 8
// The last packet of each nearby device, byte for byte as received (2026-11-10): the page's "Data"
// button shows its values. Kept raw, not decoded here, because a device on old firmware sends an
// older layout of the same size - PaulaDeployer (PacketLayouts) works out which layout fits.
#define NEARBY_RAW_MAX 248
#define NEARBY_TYPE_DSD      1   // DigitalStablesData
#define NEARBY_TYPE_CHINAMPA 2   // ChinampaData
struct NearbyDevice {
  char name[17];
  uint8_t serial[8];
  uint8_t wifiStatus;   // 0 off, 1 own hotspot, 2 router, 3 router + internet
  long sleepTime;       // s, 0 = awake
  float batteryVoltage;
  int rssi;
  uint32_t lastMs;      // 0 = slot unused
  uint8_t type;         // NEARBY_TYPE_*
  float snr;
  uint16_t rawLen;
  uint8_t raw[NEARBY_RAW_MAX];
};
uint8_t loraRxRaw[NEARBY_RAW_MAX];  // the packet being processed, before rssi/snr are written into it
static_assert(sizeof(DigitalStablesData) <= NEARBY_RAW_MAX && sizeof(ChinampaData) <= NEARBY_RAW_MAX,
              "NEARBY_RAW_MAX must hold every packet the Nearby list keeps");

// ---- Relay mode (2026-11-10) ----
// For testing whether a spot would make a good relay: Paula, placed between a device and Annabelle,
// resends the data packets it hears so Annabelle gets a second chance at them. Chinampa first - its
// direct link to Annabelle has no margin (see the "LoRa Link-Health Diagnosis" doc). Off at every boot.
//   Relayed: ChinampaData, DigitalStablesData, VitalSignsRecord, LinkHealthRecord - byte for byte, so
//   Annabelle can't tell a relayed packet from the original (the Pi stores a reading once anyway).
//   Never twice: the last RELAY_SEEN packets are remembered by a hash of their bytes. For
//   DigitalStablesData the hash leaves out sentbyarray/rssi/snr, so Chinampa's own repeat of a Fish
//   Tank or Sump packet counts as the same packet and Paula and Chinampa can't keep bouncing it.
//   When: 150-650 ms after hearing it (random, so two relays don't collide), and not while the
//   channel reads more than RELAY_BUSY_DB over its noise level (up to 3 waits of 300 ms, then anyway).
#define RELAY_QUEUE 4
#define RELAY_SEEN 32
#define RELAY_SEEN_MS (10UL * 60UL * 1000UL)
#define RELAY_BUSY_DB 12
struct RelaySlot { uint16_t len; uint8_t waits; uint32_t dueMs; char name[17]; uint8_t bytes[NEARBY_RAW_MAX]; };
// The OLED's relay ticker (2026-11-11): the last packet relayed, on the bottom line for RELAY_TICKER_MS.
#define RELAY_TICKER_MS 5000
int relayJustQueued = -1;     // slot noteRelayCandidate just filled - named in loop() once processLora is done
char relayTickerName[17] = "";
uint32_t relayTickerAt = 0;   // millis() of the last ticker event, 0 = none
bool relayTickerShowing = false;
// What the ticker says (2026-11-11): RELAY_TICK_SENT = resent; RELAY_TICK_STRONG = not resent, heard
// so strongly here that the device doesn't need Paula (relayTickerRssi says how strong).
#define RELAY_TICK_SENT 1
#define RELAY_TICK_STRONG 2
uint8_t relayTickerAction = 0;
int relayTickerRssi = 0;
// Strong-signal rule (2026-11-11): a packet heard at relayStrongDbm or stronger is not relayed - a
// relay is for devices that are hard to hear, and Paula next to a device would otherwise resend all
// of it. Relay#1#<dBm> sets it (Relay#1#0 = relay everything); Relay#1 alone keeps the current one.
#define RELAY_STRONG_DBM_DEFAULT -80
int relayStrongDbm = RELAY_STRONG_DBM_DEFAULT;
uint32_t relayStrong = 0;     // packets not relayed for being strong
int relayPendingStrongRssi = 0;  // a strong skip in this processLora - named in loop() like relayJustQueued
bool relayPendingStrong = false;
bool relayOn = false;
RelaySlot relayQueue[RELAY_QUEUE];  // len 0 = free
uint32_t relaySeenHash[RELAY_SEEN];
uint32_t relaySeenMs[RELAY_SEEN];
uint8_t relaySeenNext = 0;
uint32_t relayHeard = 0, relaySent = 0, relayDup = 0, relayDropped = 0;
static_assert(sizeof(VitalSignsRecord) <= NEARBY_RAW_MAX && sizeof(LinkHealthRecord) <= NEARBY_RAW_MAX,
              "a relay slot must hold every packet type relayed");

// ---- Noise where Paula stands (2026-11-10) ----
// The channel's RSSI once a second while nothing is being received - the last NOISE_SAMPLES seconds.
// A rise here is interference (2026-11-08: Annabelle's noise rose ~10 dB and Chinampa dropped out);
// walking around with Paula and watching it finds the source.
#define NOISE_SAMPLES 60
int16_t noiseRing[NOISE_SAMPLES];
uint8_t noiseCount = 0, noiseNext = 0;
int16_t noiseNow = 0;
NearbyDevice nearby[NEARBY_MAX];
DeviceCommandQueue deviceCommandQueue;
// The LoRa "L" (LEDs 1, 6, 11, 12 - same glyph as Daffodil's drawLora) flashed for a WiFi command:
// blue when Paula sends one, green when the device's data shows it worked. While it shows, loop()
// leaves LEDs 0-3 alone (it rewrites them every pass).
#define COMMAND_FLASH_MS 3000
#define COMMAND_FLASH_BRIGHTNESS 80   // brighter than normal (50) so the L stands out (120 was too bright)
#define NORMAL_BRIGHTNESS 50
#define LED_MAX_MILLIAMPS 100  // FastLED's power cap - see setup()
unsigned long commandFlashUntil = 0;

// LoRa Sniffer (2026-10-24): after LoraSniffer#1 (LoraSniffer#0 stops it), one line per packet
// heard, for PaulaDeployer's "Lora Sniffer" terminal - what the original Paula showed on its OLED:
//   SNIFF {"type":"DigitalStablesData","size":81,"rssi":-87,"snr":9.50,"name":"Cleotilde","serial":"28FF...","ok":true}
// ok (checksum / code valid) only for the types already checked; a size Paula doesn't know comes
// as type "Unknown" with the bytes in "hex". Off after every boot.
bool loraSniffer = false;

// lat/lon (2026-11-03, PaulaDeployer's Signal Map): the position the sender broadcasts in its data
// packet, NAN for packet types that carry none. 0/0 (never configured) is left out too.
void printSniff(const char *type, int size, float rssi, float snr, const char *name, size_t nameLen, const uint8_t *serial, int ok, float lat, float lon) {
  Serial.printf("SNIFF {\"type\":\"%s\",\"size\":%d,\"rssi\":%d,\"snr\":%.2f", type, size, (int)rssi, snr);
  if (name) {
    Serial.print(",\"name\":\"");
    for (size_t i = 0; i < nameLen && name[i]; i++) {
      char c = name[i];
      Serial.print((c >= 32 && c < 127 && c != '"' && c != '\\') ? c : '?');
    }
    Serial.print("\"");
  }
  if (serial) {
    Serial.print(",\"serial\":\"");
    for (int b = 0; b < 8; b++) Serial.printf("%02X", serial[b]);
    Serial.print("\"");
  }
  if (ok >= 0) Serial.printf(",\"ok\":%s", ok ? "true" : "false");
  if (isfinite(lat) && isfinite(lon) && fabsf(lat) <= 90 && fabsf(lon) <= 180 && !(lat == 0 && lon == 0)) {
    Serial.printf(",\"lat\":%.6f,\"lon\":%.6f", lat, lon);
  }
  Serial.println("}");
}


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

void startUltrasonicMode();
void startFlowMode();

// "Clipsal Voltage Monitor" (2026-11-07): HardwareDesign/Clipsall_VoltageMonitor - an INA228 at 0x4F
// (A0 = A1 = SCL) on Paula's I2C terminal, measuring the battery on its J3 (VBUS, through 1k) and an
// external shunt in the battery negative lead on J4 (I+/I-, Kelvin sense), plus a battery 10k NTC
// divider whose output (J2 "TEMP") goes to the pin-33 terminal with the jumper on ANALOG (Daffodil
// JP5) -> ADS1115 AIN0, as the board's own spec says for Daffodil. The INA228 is read register by register
// at its power-on defaults (continuous conversions, ADCRANGE 0 = +-163.84 mV shunt range); the current
// is worked out on the Pi from the shunt value the page is given.
#define INA228_ADDRESS 0x4F
bool ina228Read(uint8_t reg, uint8_t len, uint32_t &value) {
  Wire.beginTransmission(INA228_ADDRESS);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)INA228_ADDRESS, len) != len) return false;
  value = 0;
  for (uint8_t i = 0; i < len; i++) value = (value << 8) | Wire.read();
  return true;
}

// 20-bit two's complement in bits 23..4 of a 24-bit register (VSHUNT, VBUS)
int32_t ina228Signed20(uint32_t raw24) {
  int32_t v = (int32_t)(raw24 >> 4);
  if (v & 0x80000) v -= 0x100000;
  return v;
}

void readVoltageMonitor() {
  uint32_t manufacturer = 0, device = 0, vbusRaw = 0, vshuntRaw = 0, dieRaw = 0;
  bool found = ina228Read(0x3E, 2, manufacturer) && ina228Read(0x3F, 2, device);
  float vbusV = -99, vshuntMV = -99, dieTempC = -99;
  if (found && ina228Read(0x05, 3, vbusRaw) && ina228Read(0x04, 3, vshuntRaw) && ina228Read(0x06, 2, dieRaw)) {
    vbusV = ina228Signed20(vbusRaw) * 195.3125e-6f;      // 195.3125 uV/LSB
    vshuntMV = ina228Signed20(vshuntRaw) * 312.5e-6f;    // 312.5 nV/LSB = 312.5e-6 mV, ADCRANGE 0
    dieTempC = (int16_t)dieRaw * 7.8125e-3f;             // 7.8125 m°C/LSB
  }
  float ntcV = -99, v50V = -99;
  if (ADS.isConnected()) {
    ADS.setGain(0);
    ntcV = ADS.readADC(0) * ADS.toVoltage(1);  // pin-33 terminal, ANALOG jumper (JP5) = AIN0
    v50V = ADS.readADC(3) * ADS.toVoltage(1);  // AIN3 = V50_I (pre-diode input), only a guide to V50
  }
  Serial.println("inaFound=" + String(found ? 1 : 0)
                 + " manufacturerId=" + String(manufacturer, HEX)
                 + " deviceId=" + String(device, HEX)
                 + " vbusV=" + String(vbusV, 4)
                 + " vshuntMV=" + String(vshuntMV, 5)
                 + " inaDieTempC=" + String(dieTempC, 2)
                 + " ntcV=" + String(ntcV, 4)
                 + " v50V=" + String(v50V, 4)
                 + " ntcAin=0");  // tells PaulaDeployer this is v7+, not v6 (which read AIN1)
  Serial.println("Ok-ReadVoltageMonitor");
  Serial.flush();
}

void pollUltrasonicSerial2() {
  while (ultrasonicSerial2.available()) {
    uint8_t b = ultrasonicSerial2.read();
    if (ultrasonic2FrameBufLen == 0) {
      if (b == 0xFF) ultrasonic2FrameBuf[ultrasonic2FrameBufLen++] = b;
    } else {
      ultrasonic2FrameBuf[ultrasonic2FrameBufLen++] = b;
      if (ultrasonic2FrameBufLen == 4) {
        uint8_t sum = (ultrasonic2FrameBuf[0] + ultrasonic2FrameBuf[1] + ultrasonic2FrameBuf[2]) & 0xFF;
        if (sum == ultrasonic2FrameBuf[3]) {
          lastUltrasonic2DistanceMM = (ultrasonic2FrameBuf[1] << 8) | ultrasonic2FrameBuf[2];
          lastUltrasonic2ReadMillis = millis();
        }
        ultrasonic2FrameBufLen = 0;
      }
    }
  }
}

// Gives pin 33 back to Flow Sensor 2, from either Clipsal test (OneWire or the second UART).
void endClipsal() {
  if (clipsal2USActive) {
    ultrasonicSerial2.end();
    clipsal2USActive = false;
  }
  if (!clipsalActive) return;
  pinMode(SENSOR_INPUT_2, INPUT);
  attachInterrupt(SENSOR_INPUT_2, pulseCounter2, FALLING);
  clipsalActive = false;
}

// One reading of both ultrasonics: waits up to 1.5 s for a frame from each that arrived after the
// command. distance1 = pin 18 (left Grove, J2 "18"), distance2 = pin 33 (right Grove, J2 "33").
void readClipsal2US() {
  if (currentTestMode != TEST_MODE_ULTRASONIC) startUltrasonicMode();
  if (!clipsalActive) {
    detachInterrupt(SENSOR_INPUT_2);
    clipsalActive = true;
  }
  if (!clipsal2USActive) {
    ultrasonic2FrameBufLen = 0;
    lastUltrasonic2DistanceMM = 0;
    lastUltrasonic2ReadMillis = 0;
    ultrasonicSerial2.begin(9600, SERIAL_8N1, SENSOR_INPUT_2, -1);
    clipsal2USActive = true;
  }
  unsigned long asked = millis();
  bool fresh1 = false, fresh2 = false;
  while (millis() - asked < 1500 && !(fresh1 && fresh2)) {
    pollUltrasonicSerial();
    pollUltrasonicSerial2();
    fresh1 = lastUltrasonicReadMillis != 0 && lastUltrasonicReadMillis >= asked;
    fresh2 = lastUltrasonic2ReadMillis != 0 && lastUltrasonic2ReadMillis >= asked;
    delay(5);
  }
  Serial.println("distance1MM=" + String(fresh1 ? lastUltrasonicDistanceMM : 0)
                 + " ultrasonic1Ok=" + String(fresh1 ? 1 : 0)
                 + " distance2MM=" + String(fresh2 ? lastUltrasonic2DistanceMM : 0)
                 + " ultrasonic2Ok=" + String(fresh2 ? 1 : 0));
  Serial.println("Ok-ReadClipsal2US");
  Serial.flush();
}

// One reading of both sensors, printed as one key=value line. Waits up to 1.5 s for an ultrasonic
// frame that arrived after the command (the DYP-A02 sends ~every 100-300 ms in automatic mode), and
// re-scans the 1-Wire bus every time so a probe plugged in mid-test is found.
void readClipsal() {
  if (clipsal2USActive) {  // pin 33 was the second ultrasonic's UART - give it to OneWire
    ultrasonicSerial2.end();
    clipsal2USActive = false;
  }
  if (currentTestMode != TEST_MODE_ULTRASONIC) startUltrasonicMode();
  if (!clipsalActive) {
    detachInterrupt(SENSOR_INPUT_2);
    clipsalActive = true;
  }
  unsigned long asked = millis();
  while (millis() - asked < 1500 && (lastUltrasonicReadMillis == 0 || lastUltrasonicReadMillis < asked)) {
    pollUltrasonicSerial();
    delay(5);
  }
  bool freshFrame = lastUltrasonicReadMillis != 0 && lastUltrasonicReadMillis >= asked;

  clipsalTempSensor.begin();
  int probes = clipsalTempSensor.getDeviceCount();
  float tempC = -99;
  if (probes > 0) {
    clipsalTempSensor.setResolution(10);  // 0.25°C, ~190 ms
    clipsalTempSensor.setWaitForConversion(true);
    clipsalTempSensor.requestTemperatures();
    float t = clipsalTempSensor.getTempCByIndex(0);
    if (t != DEVICE_DISCONNECTED) tempC = t;
  }
  pollUltrasonicSerial();  // drain what came in during the conversion

  Serial.println("distanceMM=" + String(freshFrame ? lastUltrasonicDistanceMM : 0)
                 + " ultrasonicOk=" + String(freshFrame ? 1 : 0)
                 + " tempC=" + String(tempC, 2)
                 + " tempProbes=" + String(probes));
  Serial.println("Ok-ReadClipsal");
  Serial.flush();
}

void onReceive(int packetSize) {

  loraReceived = true;
  loraPacketSize = packetSize;
}


// XOR of every byte except the one at checksumOffset - the checksum rule of the LoRa structs.
uint8_t xorChecksum(const void *data, size_t size, size_t checksumOffset) {
  const uint8_t *p = (const uint8_t *)data;
  uint8_t c = 0;
  for (size_t i = 0; i < size; i++) if (i != checksumOffset) c ^= p[i];
  return c;
}

// Who Paula heard last (2026-11-11) - set in processLora for every packet, drawn by showHeard() on
// the OLED_SMALL screen.
char lastHeardName[17] = "";
char lastHeardType[12] = "";
int lastHeardRssi = 0;
float lastHeardSnr = 0;
bool lastHeardNew = false;

// Name a packet without one (VitalSigns, LinkHealth) carries: the name its data packets gave, else
// the first 6 hex digits of its serial.
void noteHeard(const char *name, size_t nameLen, const uint8_t *serial, const char *type) {
  memset(lastHeardName, 0, sizeof(lastHeardName));
  if (name && name[0]) {
    size_t n = strnlen(name, nameLen);
    if (n > sizeof(lastHeardName) - 1) n = sizeof(lastHeardName) - 1;
    memcpy(lastHeardName, name, n);
  } else if (serial) {
    for (int i = 0; i < NEARBY_MAX; i++) {
      if (nearby[i].lastMs && memcmp(nearby[i].serial, serial, 8) == 0 && nearby[i].name[0]) {
        strncpy(lastHeardName, nearby[i].name, sizeof(lastHeardName) - 1);
        break;
      }
    }
    if (!lastHeardName[0]) snprintf(lastHeardName, sizeof(lastHeardName), "%02X%02X%02X", serial[0], serial[1], serial[2]);
  } else {
    strncpy(lastHeardName, "Unknown", sizeof(lastHeardName) - 1);
  }
  strncpy(lastHeardType, type, sizeof(lastHeardType) - 1);
  lastHeardType[sizeof(lastHeardType) - 1] = 0;
  lastHeardRssi = LoRa.packetRssi();
  lastHeardSnr = LoRa.packetSnr();
  lastHeardNew = true;
}

// The device's slot: its own, else a free one, else the least recently heard.
NearbyDevice &nearbySlot(const uint8_t *serial) {
  int slot = -1, oldest = 0;
  for (int i = 0; i < NEARBY_MAX; i++) {
    if (nearby[i].lastMs && memcmp(nearby[i].serial, serial, 8) == 0) { slot = i; break; }
    if (!nearby[i].lastMs) { if (slot < 0) slot = i; continue; }
    if (nearby[i].lastMs < nearby[oldest].lastMs) oldest = i;
  }
  // a free slot found before the device's own one would duplicate it - look again
  for (int i = 0; i < NEARBY_MAX; i++) {
    if (nearby[i].lastMs && memcmp(nearby[i].serial, serial, 8) == 0) { slot = i; break; }
  }
  if (slot < 0) slot = oldest;
  return nearby[slot];
}

// What every packet type records: name, serial, how it was heard, and the packet itself (loraRxRaw).
void noteNearbyCommon(NearbyDevice &n, const char *name, size_t nameLen, const uint8_t *serial, uint8_t type,
                      int rssi, float snr, size_t rawLen) {
  memset(n.name, 0, sizeof(n.name));
  for (size_t i = 0; i < 16 && i < nameLen && name[i]; i++) {
    n.name[i] = (name[i] >= 32 && name[i] < 127) ? name[i] : '?';
  }
  memcpy(n.serial, serial, 8);
  n.type = type;
  n.rssi = rssi;
  n.snr = snr;
  n.rawLen = rawLen > NEARBY_RAW_MAX ? NEARBY_RAW_MAX : rawLen;
  memcpy(n.raw, loraRxRaw, n.rawLen);
  n.lastMs = millis();
}

void noteNearby(const DigitalStablesData &d) {
  NearbyDevice &n = nearbySlot(d.serialnumberarray);
  n.wifiStatus = d.wifiStatus;
  n.sleepTime = d.sleepTime;
  n.batteryVoltage = d.batteryVoltage;
  noteNearbyCommon(n, d.devicename, sizeof(d.devicename), d.serialnumberarray, NEARBY_TYPE_DSD, (int)d.rssi, d.snr,
                   sizeof(DigitalStablesData));
}

// Chinampa (2026-11-10): listed so its Data can be seen - it is always awake, has no WiFi command and
// no battery, so those fields stay 0. ChinampaData carries no checksum: a packet with no serial is
// left out.
void noteNearbyChinampa(const ChinampaData &d) {
  bool hasSerial = false;
  for (int i = 0; i < 8; i++) if (d.serialnumberarray[i] != 0 && d.serialnumberarray[i] != 0xFF) hasSerial = true;
  if (!hasSerial) return;
  NearbyDevice &n = nearbySlot(d.serialnumberarray);
  n.wifiStatus = 0;
  n.sleepTime = 0;
  n.batteryVoltage = 0;
  noteNearbyCommon(n, d.devicename, sizeof(d.devicename), d.serialnumberarray, NEARBY_TYPE_CHINAMPA, (int)d.rssi, d.snr,
                   sizeof(ChinampaData));
}

// GetNearbyPacket#<serial, 16 hex digits> -> one line: PACKET {"serial","name","type","size","ageSec",
// "rssi","snr","hex"} - the device's last packet as received. PaulaDeployer decodes it.
bool printNearbyPacket(const String &serialHex) {
  uint8_t serial[8];
  if (serialHex.length() != 16) return false;
  for (int b = 0; b < 8; b++) serial[b] = (uint8_t)strtoul(serialHex.substring(b * 2, b * 2 + 2).c_str(), nullptr, 16);
  for (int i = 0; i < NEARBY_MAX; i++) {
    const NearbyDevice &n = nearby[i];
    if (!n.lastMs || memcmp(n.serial, serial, 8) != 0) continue;
    Serial.printf("PACKET {\"serial\":\"%s\",\"name\":\"%s\",\"type\":\"%s\",\"size\":%u,\"ageSec\":%lu,\"rssi\":%d,\"snr\":%.2f,\"hex\":\"",
                  serialHex.c_str(), n.name, n.type == NEARBY_TYPE_CHINAMPA ? "ChinampaData" : "DigitalStablesData", n.rawLen,
                  (unsigned long)((millis() - n.lastMs) / 1000), n.rssi, n.snr);
    for (int k = 0; k < n.rawLen; k++) Serial.printf("%02X", n.raw[k]);
    Serial.println("\"}");
    return true;
  }
  return false;
}

// ---- noise ----
void noteNoiseSample(int rssi) {
  noiseNow = rssi;
  noiseRing[noiseNext] = rssi;
  noiseNext = (noiseNext + 1) % NOISE_SAMPLES;
  if (noiseCount < NOISE_SAMPLES) noiseCount++;
}

// min / median / max of the last NOISE_SAMPLES seconds; false = no samples yet
bool noiseStats(int &mn, int &med, int &mx) {
  if (!noiseCount) return false;
  int16_t v[NOISE_SAMPLES];
  memcpy(v, noiseRing, noiseCount * sizeof(int16_t));
  for (int i = 1; i < noiseCount; i++) {  // insertion sort, 60 values
    int16_t x = v[i];
    int j = i - 1;
    while (j >= 0 && v[j] > x) { v[j + 1] = v[j]; j--; }
    v[j + 1] = x;
  }
  mn = v[0];
  mx = v[noiseCount - 1];
  med = v[noiseCount / 2];
  return true;
}

void printNoiseJson() {
  int mn, med, mx;
  if (noiseStats(mn, med, mx)) Serial.printf("{\"now\":%d,\"min\":%d,\"median\":%d,\"max\":%d,\"samples\":%u}", noiseNow, mn, med, mx, noiseCount);
  else Serial.print("{\"samples\":0}");
}

// ---- relay ----
uint32_t relayHash(const uint8_t *p, size_t len, bool isDsd) {
  uint32_t h = 2166136261u;
  bool isLangley = len == sizeof(LangleyData);
  for (size_t i = 0; i < len; i++) {
    // LangleyData the same way: sentby, checksum and the receiver's rssi/snr change between copies
    if (isLangley && (i == offsetof(LangleyData, checksum) ||
                      (i >= offsetof(LangleyData, sentbyarray) && i < offsetof(LangleyData, sentbyarray) + 8) ||
                      (i >= offsetof(LangleyData, rssi) && i < offsetof(LangleyData, snr) + sizeof(float)))) continue;
    // sentbyarray and so the checksum change at every hop (Chinampa "Chinamp", Paula "Paula") - both
    // left out, or Paula would take Chinampa's resend of her own relay for a new packet (2026-11-11)
    if (isDsd && i == offsetof(DigitalStablesData, checksum)) continue;
    if (isDsd && ((i >= offsetof(DigitalStablesData, sentbyarray) && i < offsetof(DigitalStablesData, sentbyarray) + 8) ||
                  (i >= offsetof(DigitalStablesData, rssi) && i < offsetof(DigitalStablesData, snr) + sizeof(float)))) continue;
    h ^= p[i];
    h *= 16777619u;
  }
  return h;
}

// A packet as received: queued for resending if relay mode is on and it hasn't been relayed already.
void noteRelayCandidate(const uint8_t *p, size_t len, bool isDsd, int rssi) {
  if (!relayOn || len > NEARBY_RAW_MAX) return;
  // Already relayed by someone (sentbyarray set - Chinampa's "Chinamp"): not sent again, same rule as
  // Chinampa's relay, so two relays can't bounce a packet between them (2026-11-11).
  if (len == sizeof(DigitalStablesData) || len == sizeof(ChinampaData) || len == sizeof(LangleyData)) {
    size_t at = len == sizeof(DigitalStablesData) ? offsetof(DigitalStablesData, sentbyarray)
              : len == sizeof(ChinampaData) ? offsetof(ChinampaData, sentbyarray) : offsetof(LangleyData, sentbyarray);
    for (int i = 0; i < 8; i++) if (p[at + i]) return;
  }
  relayHeard++;
  // Heard strongly here: the device doesn't need a relay (see relayStrongDbm). Shown on the ticker.
  if (relayStrongDbm < 0 && rssi >= relayStrongDbm) {
    relayStrong++;
    relayPendingStrong = true;
    relayPendingStrongRssi = rssi;
    return;
  }
  uint32_t h = relayHash(p, len, isDsd), now = millis();
  for (int i = 0; i < RELAY_SEEN; i++) {
    if (relaySeenMs[i] && relaySeenHash[i] == h && now - relaySeenMs[i] < RELAY_SEEN_MS) { relayDup++; return; }
  }
  relaySeenHash[relaySeenNext] = h;
  relaySeenMs[relaySeenNext] = now ? now : 1;
  relaySeenNext = (relaySeenNext + 1) % RELAY_SEEN;
  for (int i = 0; i < RELAY_QUEUE; i++) {
    if (relayQueue[i].len) continue;
    relayQueue[i].len = len;
    relayQueue[i].waits = 0;
    relayQueue[i].dueMs = now + random(150, 650);
    relayQueue[i].name[0] = 0;
    relayJustQueued = i;
    memcpy(relayQueue[i].bytes, p, len);
    // Mark the copy as relayed (2026-11-11): sentbyarray = "Paula", the field Chinampa already fills
    // ("Chinamp") when it resends a Daffodil - Annabelle shows it, so a relayed packet can be told
    // from one she heard directly. DigitalStablesData's checksum covers sentbyarray, so it is
    // recomputed; ChinampaData's checksum isn't used by anyone (Annabelle checks its name only).
    // VitalSigns / LinkHealth have no sentby field and go out unchanged.
    if (len == sizeof(DigitalStablesData)) {
      DigitalStablesData *d = (DigitalStablesData *)relayQueue[i].bytes;
      memset(d->sentbyarray, 0, sizeof(d->sentbyarray));
      memcpy(d->sentbyarray, "Paula", 5);
      d->checksum = xorChecksum(d, sizeof(DigitalStablesData), offsetof(DigitalStablesData, checksum));
    } else if (len == sizeof(LangleyData)) {
      // LangleyData has a sentbyarray since 2026-11-11; its checksum covers it, like DigitalStablesData's
      LangleyData *l = (LangleyData *)relayQueue[i].bytes;
      memset(l->sentbyarray, 0, sizeof(l->sentbyarray));
      memcpy(l->sentbyarray, "Paula", 5);
      l->checksum = xorChecksum(l, sizeof(LangleyData), offsetof(LangleyData, checksum));
    } else if (len == sizeof(ChinampaData)) {
      ChinampaData *c = (ChinampaData *)relayQueue[i].bytes;
      memset(c->sentbyarray, 0, sizeof(c->sentbyarray));
      memcpy(c->sentbyarray, "Paula", 5);
    }
    return;
  }
  relayDropped++;  // queue full
}

// From loop(): sends the queued packets whose time has come.
void serviceRelay() {
  for (int i = 0; i < RELAY_QUEUE; i++) {
    RelaySlot &r = relayQueue[i];
    if (!r.len || (int32_t)(millis() - r.dueMs) < 0) continue;
    if (loraReceived) return;  // a packet is waiting to be read - that first
    int mn, med, mx;
    if (r.waits < 3 && noiseStats(mn, med, mx) && LoRa.rssi() > med + RELAY_BUSY_DB) {
      r.waits++;
      r.dueMs = millis() + 300;  // someone is transmitting - wait
      continue;
    }
    LoRa_txMode();
    LoRa.beginPacket();
    LoRa.write(r.bytes, r.len);
    bool ok = LoRa.endPacket();
    LoRa_rxMode();
    if (ok) {
      relaySent++;
      strncpy(relayTickerName, r.name[0] ? r.name : "packet", sizeof(relayTickerName) - 1);
      relayTickerName[sizeof(relayTickerName) - 1] = 0;
      relayTickerAction = RELAY_TICK_SENT;
      relayTickerAt = millis();
      if (!relayTickerAt) relayTickerAt = 1;
      lastHeardNew = true;  // redraw the OLED with the ticker line
    }
    if (debug) Serial.printf("relayed %u bytes: %s\n", r.len, ok ? "ok" : "failed");
    r.len = 0;
  }
}

void printRelayJson() {
  int queued = 0;
  for (int i = 0; i < RELAY_QUEUE; i++) if (relayQueue[i].len) queued++;
  Serial.printf("{\"on\":%s,\"heard\":%lu,\"sent\":%lu,\"dup\":%lu,\"dropped\":%lu,\"queued\":%d", relayOn ? "true" : "false",
                (unsigned long)relayHeard, (unsigned long)relaySent, (unsigned long)relayDup, (unsigned long)relayDropped, queued);
  // the last packet relayed (2026-11-11) - PaulaDeployer's relay ticker line
  if (relayTickerAt) {
    char safe[17];
    int n = 0;
    for (const char *c = relayTickerName; *c && n < 16; c++) if (*c >= 32 && *c < 127 && *c != '"' && *c != '\\') safe[n++] = *c;
    safe[n] = 0;
    Serial.printf(",\"last\":\"%s\",\"lastAgeSec\":%lu,\"lastAction\":\"%s\",\"lastRssi\":%d", safe,
                  (unsigned long)((millis() - relayTickerAt) / 1000), relayTickerAction == RELAY_TICK_STRONG ? "strong" : "relayed",
                  relayTickerRssi);
  }
  Serial.printf(",\"strong\":%lu,\"strongDbm\":%d}", (unsigned long)relayStrong, relayStrongDbm);
}

// Glyphs on the 15 LEDs (2026-11-11): a 5-wide, 3-high grid, LED = row * 5 + column (the "L" below
// is 1, 6, 11 down and 12 across). Each row is a 5-bit mask, bit 4 = column 0 (left). Clears the
// rest of the grid.
void drawGlyph(const uint8_t rows[3], CRGB color) {
  for (int i = 0; i < NUM_LEDS; i++) leds[i] = CRGB(0, 0, 0);
  for (int r = 0; r < 3; r++)
    for (int c = 0; c < 5; c++)
      if (rows[r] & (0x10 >> c)) leds[r * 5 + c] = color;
}

// Status glyphs, shown one after another like Daffodil's display (showStatusGlyphs, from loop()):
// GLYPH_STEP_MS each with a short dark gap between, green = good / on, red = bad / off.
//   L  LoRa      green: the radio started. Red: LoRa.begin failed.
//   F  Forecast  green: Annabelle's weather forecast heard within WEATHER_STALE_INTERVAL. Red: stale.
//   r  Relay     green: relay mode on (Relay#1). Red: off - it is off after every boot.
// A packet heard: a short light show - a rainbow sweeping along the middle row for PACKET_SHOW_MS - then
// the glyphs carry on. (WiFi isn't shown: Paula talks to the Pi over USB, so it is always off.)
// (Before 2026-11-11: LED 0 WiFi, 1 LoRa, 2-3 forecast, 4-5 a packet, and a rainbow on 8-14.)
const uint8_t GLYPH_L[3] = { 0x08, 0x08, 0x0C };  // .X... / .X... / .XX..
const uint8_t GLYPH_F[3] = { 0x0E, 0x0C, 0x08 };  // .XXX. / .XX.. / .X...
const uint8_t GLYPH_R[3] = { 0x00, 0x0C, 0x08 };  // ..... / .XX.. / .X...  (a small r, bottom two rows)
#define GLYPH_STEP_MS 1500
#define GLYPH_GAP_MS 200
#define PACKET_SHOW_MS 600
unsigned long packetShowUntil = 0;
int lastGlyphKey = -1;  // what is on the LEDs now; -1 = redraw

void flashLoraL(CRGB color) {
  for (int i = 0; i < NUM_LEDS; i++) leds[i] = CRGB(0, 0, 0);
  leds[1] = leds[6] = leds[11] = leds[12] = color;
  FastLED.setBrightness(COMMAND_FLASH_BRIGHTNESS);
  FastLED.show();
  commandFlashUntil = millis() + COMMAND_FLASH_MS;
}

// Returns the senderTime it was stamped with (the ping's id - see the Ping command).
uint32_t sendDeviceCommand(DeviceCommandRecord c) {
  c.senderTime = (uint32_t)timeManager.getTimeForCodeGeneration();
  c.totpcode = secretManager.generateCodeAt(c.senderTime);
  c.sender = DEVICE_COMMAND_SENDER_PAULA;
  c.checksum = deviceCommandChecksum(c);
  LoRa_txMode();
  LoRa.beginPacket();
  LoRa.write((uint8_t *)&c, sizeof(c));
  bool sent = LoRa.endPacket();
  LoRa_rxMode();
  flashLoraL(sent ? CRGB(0, 0, 255) : CRGB(255, 0, 0));  // red L = the radio failed to send
  if (debug) Serial.printf("device command %u sent: %s\n", c.command, sent ? "ok" : "failed");
  return sent ? c.senderTime : 0;
}

// ---- Signal survey ping (2026-11-03, PaulaDeployer's Nearby Devices map) ----
// "Ping#<dBm>": one DEVICE_COMMAND_PING to every device, sent at that transmit power - 5 dBm by
// default, a Daffodil's (and Langley's, Gloria's) power, so the answers say what a device placed here
// would get, not what Paula's stronger 17 dBm would. Back to 17 dBm right after. Answers come as
// LinkAckRecords within LINK_ACK_LISTEN_MS, printed as SNIFF lines (printLinkAck).
// Replies "Ok-Ping#<senderTime>#<number>#<dBm>" (senderTime 0 = not sent).
#define PAULA_TX_POWER 17   // the LoRa library's default, which Paula has always used
uint16_t pingNumber = 0;

void sendPing(int power) {
  DeviceCommandRecord c;
  memset(c.serialnumberarray, 0xFF, 8);
  c.command = DEVICE_COMMAND_PING;
  pingNumber++;
  c.reserved[0] = pingNumber & 0xFF;
  c.reserved[1] = pingNumber >> 8;
  LoRa.setTxPower(power);
  uint32_t id = sendDeviceCommand(c);
  LoRa.setTxPower(PAULA_TX_POWER);
  Serial.printf("Ok-Ping#%lu#%u#%d\n", (unsigned long)id, pingNumber, power);
}

// One SNIFF line per answer: the usual fields (rssi/snr = how Paula heard the answer) plus
//   "probe","seq"          which ping it answers
//   "kind"                 1 Annabelle, 2 Daffodil
//   "prssi","psnr"         how the device heard the ping
//   "grssi","gsnr","gage"  how it hears Annabelle (absent when unknown, and for Annabelle itself)
void printLinkAck(const LinkAckRecord &a, int size, int rssi, float snr) {
  bool ok = linkAckValid(a);
  Serial.printf("SNIFF {\"type\":\"LinkAck\",\"size\":%d,\"rssi\":%d,\"snr\":%.2f,\"name\":\"", size, rssi, snr);
  for (size_t i = 0; i < sizeof(a.devicename) && a.devicename[i]; i++) {
    char ch = a.devicename[i];
    Serial.print((ch >= 32 && ch < 127 && ch != '"' && ch != '\\') ? ch : '?');
  }
  Serial.print("\",\"serial\":\"");
  for (int b = 0; b < 8; b++) Serial.printf("%02X", a.serialnumberarray[b]);
  Serial.printf("\",\"ok\":%s,\"probe\":%lu,\"seq\":%u,\"kind\":%u,\"prssi\":%d,\"psnr\":%.2f",
                ok ? "true" : "false", (unsigned long)a.probeTime, a.probeSeq, a.kind, a.probeRssi, a.probeSnrQ / 4.0);
  if (a.gatewayAgeSec != LINK_ACK_GATEWAY_UNKNOWN_AGE) {
    Serial.printf(",\"grssi\":%d,\"gsnr\":%.2f,\"gage\":%u", a.gatewayRssi, a.gatewaySnrQ / 4.0, a.gatewayAgeSec);
  }
  if (isfinite(a.latitude) && isfinite(a.longitude) && fabsf(a.latitude) <= 90 && fabsf(a.longitude) <= 180
      && !(a.latitude == 0 && a.longitude == 0)) {
    Serial.printf(",\"lat\":%.6f,\"lon\":%.6f", a.latitude, a.longitude);
  }
  Serial.println("}");
}

// One JSON line: {"now":<ms>,"devices":[{name,serial,wifi,sleep,battery,rssi,ageSec,cmd:{on,state,sends,ageSec}}]}
void printNearbyDevices() {
  uint32_t now = millis();
  // paulaTime: Paula's clock as code time (local fields as seconds, like the commands' senderTime) -
  // PaulaDeployer compares it with the Pi's clock, since a Daffodil rejects commands >15 min off.
  Serial.printf("{\"paulaTime\":%ld,\"firmware\":\"Version %d (%s)\",\"devices\":[", timeManager.getTimeForCodeGeneration(),
                firmwareVersion, FIRMWARE_BUILD_TIME);
  bool first = true;
  for (int i = 0; i < NEARBY_MAX; i++) {
    const NearbyDevice &n = nearby[i];
    if (!n.lastMs) continue;
    char sn[17];
    for (int b = 0; b < 8; b++) snprintf(sn + b * 2, 3, "%02X", n.serial[b]);
    if (!first) Serial.print(",");
    first = false;
    Serial.printf("{\"name\":\"%s\",\"serial\":\"%s\",\"wifi\":%u,\"sleep\":%ld,\"battery\":%.2f,\"rssi\":%d,\"ageSec\":%lu,\"type\":\"%s\"",
                  n.name, sn, n.wifiStatus, n.sleepTime, n.batteryVoltage, n.rssi, (unsigned long)((now - n.lastMs) / 1000),
                  n.type == NEARBY_TYPE_CHINAMPA ? "ChinampaData" : "DigitalStablesData");
    DeviceCommandQueue::Slot c = deviceCommandQueue.status(n.serial);
    if (c.state != DEVICE_COMMAND_STATE_NONE) {
      Serial.printf(",\"cmd\":{\"on\":%s,\"state\":%u,\"sends\":%u,\"ageSec\":%lu}", c.command == DEVICE_COMMAND_WIFI_ON ? "true" : "false",
                    c.state, c.sends, (unsigned long)((now - c.createdMs) / 1000));
    }
    Serial.print("}");
  }
  Serial.print("],\"noise\":");
  printNoiseJson();
  Serial.print(",\"relay\":");
  printRelayJson();
  Serial.println("}");
}

void processLora(int packetSize) {

  lastLoraReceptionSeconds = 0;



  if (packetSize == 0) return;  // if there's no packet, return
  if (switchPositionLeft) displayingChinampa = false;
  if(debug)Serial.println("received " + String(packetSize));
  if (packetSize == sizeof(LangleyData)) {
    LoRa.readBytes((uint8_t*)&langleyData, sizeof(LangleyData));
    // relayed too since 2026-11-11 (LangleyData has a sentbyarray) - checked on the bytes as received
    if (xorChecksum(&langleyData, sizeof(LangleyData), offsetof(LangleyData, checksum)) == langleyData.checksum)
      noteRelayCandidate((const uint8_t *)&langleyData, sizeof(LangleyData), false, LoRa.packetRssi());

    langleyData.rssi = LoRa.packetRssi();
    langleyData.snr = LoRa.packetSnr();
    noteHeard(langleyData.devicename, sizeof(langleyData.devicename), NULL, "Langley");
    if (loraSniffer) printSniff("LangleyData", packetSize, langleyData.rssi, langleyData.snr, langleyData.devicename, sizeof(langleyData.devicename), langleyData.serialnumberarray, -1, langleyData.latitude, langleyData.longitude);




  } else if (packetSize == sizeof(GloriaTankFlowPumpData)) {
    memset(&gloriaTankFlowPumpData, 0, sizeof(GloriaTankFlowPumpData));
    LoRa.readBytes((uint8_t*)&gloriaTankFlowPumpData, sizeof(GloriaTankFlowPumpData));

    gloriaTankFlowPumpData.rssi = LoRa.packetRssi();
    gloriaTankFlowPumpData.snr = LoRa.packetSnr();


    gloriaTankFlowPumpData.rssi = LoRa.packetRssi();
    gloriaTankFlowPumpData.snr = LoRa.packetSnr();
    noteHeard(gloriaTankFlowPumpData.devicename, sizeof(gloriaTankFlowPumpData.devicename), NULL, "Gloria");
    if (loraSniffer) printSniff("GloriaTankFlowPumpData", packetSize, gloriaTankFlowPumpData.rssi, gloriaTankFlowPumpData.snr, gloriaTankFlowPumpData.devicename, sizeof(gloriaTankFlowPumpData.devicename), gloriaTankFlowPumpData.serialnumberarray, -1, gloriaTankFlowPumpData.latitude, gloriaTankFlowPumpData.longitude);


  } else if (packetSize == sizeof(ChinampaData)) {
    memset(&chinampaData, 0, sizeof(ChinampaData));
    LoRa.readBytes((uint8_t*)&chinampaData, sizeof(ChinampaData));
    memcpy(loraRxRaw, &chinampaData, sizeof(ChinampaData));  // as received, for the Data button

    noteRelayCandidate(loraRxRaw, sizeof(ChinampaData), false, LoRa.packetRssi());
    chinampaData.rssi = LoRa.packetRssi();
    chinampaData.snr = LoRa.packetSnr();
    noteNearbyChinampa(chinampaData);
    noteHeard(chinampaData.devicename, sizeof(chinampaData.devicename), NULL, "Chinampa");
    if (loraSniffer) printSniff("ChinampaData", packetSize, chinampaData.rssi, chinampaData.snr, chinampaData.devicename, sizeof(chinampaData.devicename), chinampaData.serialnumberarray, -1, chinampaData.latitude, chinampaData.longitude);
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
    memcpy(loraRxRaw, &digitalStablesData, sizeof(DigitalStablesData));  // as received, for the Data button
    // Checked on the bytes as received, before rssi/snr (inside the checksummed range) are overwritten.
    bool dsChecksumOk = xorChecksum(&digitalStablesData, sizeof(DigitalStablesData), offsetof(DigitalStablesData, checksum))
                        == digitalStablesData.checksum;
    digitalStablesDataNewData = true;
    if (dsChecksumOk) noteRelayCandidate(loraRxRaw, sizeof(DigitalStablesData), true, LoRa.packetRssi());
    digitalStablesData.rssi = LoRa.packetRssi();
    digitalStablesData.snr = LoRa.packetSnr();
    noteHeard(digitalStablesData.devicename, sizeof(digitalStablesData.devicename), NULL, dsChecksumOk ? "Data" : "Data bad");
    if (loraSniffer) printSniff("DigitalStablesData", packetSize, digitalStablesData.rssi, digitalStablesData.snr, digitalStablesData.devicename, sizeof(digitalStablesData.devicename), digitalStablesData.serialnumberarray, dsChecksumOk, digitalStablesData.latitude, digitalStablesData.longitude);
    dataManager.storeDigitalStablesData(digitalStablesData);
     dataManager.printDigitalStablesData(digitalStablesData);
    if (dsChecksumOk) {
      noteNearby(digitalStablesData);
      uint8_t before = deviceCommandQueue.status(digitalStablesData.serialnumberarray).state;
      deviceCommandQueue.noteDataPacket(digitalStablesData.serialnumberarray, digitalStablesData.wifiStatus != 0,
                                        digitalStablesData.sleepTime > 0, millis());
      if (before == DEVICE_COMMAND_STATE_PENDING &&
          deviceCommandQueue.status(digitalStablesData.serialnumberarray).state == DEVICE_COMMAND_STATE_DONE) {
        flashLoraL(CRGB(0, 255, 0));  // the device's WiFi is now as requested
      }
    }
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
  } else if (packetSize == sizeof(VitalSignsRecord)) {
    // Only for command delivery: a sleeping Daffodil listens right after its VitalSigns.
    VitalSignsRecord vs;
    LoRa.readBytes((uint8_t*)&vs, sizeof(VitalSignsRecord));
    bool vsOk = xorChecksum(&vs, sizeof(vs), offsetof(VitalSignsRecord, checksum)) == vs.checksum;
    if (vsOk) {
      deviceCommandQueue.noteVitalSigns(vs.serialnumberarray, millis());
      noteRelayCandidate((const uint8_t *)&vs, sizeof(vs), false, LoRa.packetRssi());
    }
    noteHeard(NULL, 0, vs.serialnumberarray, "Vitals");
    if (loraSniffer) printSniff("VitalSignsRecord", packetSize, LoRa.packetRssi(), LoRa.packetSnr(), NULL, 0, vs.serialnumberarray, vsOk, NAN, NAN);
  } else if (packetSize == sizeof(VitalSignsRecordV3)) {
    // Version 3 (89 bytes) from a Daffodil not yet on v4 - the serial is in the same place.
    VitalSignsRecordV3 vs3;
    LoRa.readBytes((uint8_t*)&vs3, sizeof(VitalSignsRecordV3));
    bool vs3Ok = xorChecksum(&vs3, sizeof(vs3), offsetof(VitalSignsRecordV3, checksum)) == vs3.checksum;
    if (vs3Ok) {
      deviceCommandQueue.noteVitalSigns(vitalSignsFromV3(vs3).serialnumberarray, millis());
    }
    noteHeard(NULL, 0, vitalSignsFromV3(vs3).serialnumberarray, "Vitals");
    if (loraSniffer) printSniff("VitalSignsRecordV3", packetSize, LoRa.packetRssi(), LoRa.packetSnr(), NULL, 0, vitalSignsFromV3(vs3).serialnumberarray, vs3Ok, NAN, NAN);
  } else if (packetSize == sizeof(WeatherForecastUpdate)) {
    WeatherForecastUpdate wfu;
    LoRa.readBytes((uint8_t*)&wfu, sizeof(WeatherForecastUpdate));
    bool validCode = secretManager.checkCode(wfu.totpcode);
    noteHeard("Weather", 8, NULL, "Forecast");
    if (loraSniffer) printSniff("WeatherForecastUpdate", packetSize, LoRa.packetRssi(), LoRa.packetSnr(), NULL, 0, NULL, validCode, NAN, NAN);
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
    noteHeard(seedlingMonitorData.devicename, sizeof(seedlingMonitorData.devicename), NULL, "Seedling");
    if (loraSniffer) printSniff("SeedlingMonitorData", packetSize, seedlingMonitorData.rssi, seedlingMonitorData.snr, seedlingMonitorData.devicename, sizeof(seedlingMonitorData.devicename), seedlingMonitorData.serialnumberarray, -1, NAN, NAN);
    //if(debug)
    if (debug) Serial.print("received SeedlingMonitorData from ");
    if (debug) Serial.println(seedlingMonitorData.devicename);
  } else if (packetSize == sizeof(LinkHealthRecord)) {
    // A device's report on its own link (Chinampa) - Paula only relays it, so Annabelle still gets it
    // when the device can't reach her itself.
    LinkHealthRecord lh;
    LoRa.readBytes((uint8_t*)&lh, sizeof(LinkHealthRecord));
    bool lhOk = linkHealthValid(lh);
    if (lhOk) noteRelayCandidate((const uint8_t *)&lh, sizeof(lh), false, LoRa.packetRssi());
    noteHeard(NULL, 0, lh.serialnumberarray, "LinkHlth");
    if (loraSniffer) printSniff("LinkHealthRecord", packetSize, LoRa.packetRssi(), LoRa.packetSnr(), NULL, 0, lh.serialnumberarray, lhOk, NAN, NAN);
  } else if (packetSize == sizeof(LinkAckRecord)) {
    // An answer to Paula's ping (sendPing) - for the map; printed whether or not the sniffer is on.
    LinkAckRecord ack;
    LoRa.readBytes((uint8_t*)&ack, sizeof(LinkAckRecord));
    printLinkAck(ack, packetSize, LoRa.packetRssi(), LoRa.packetSnr());
    noteHeard(NULL, 0, ack.serialnumberarray, "PingAck");
  } else {
    char unknown[17];
    snprintf(unknown, sizeof(unknown), "%d bytes", packetSize);
    noteHeard(unknown, sizeof(unknown), NULL, "Unknown");
  }
  if (packetSize != sizeof(LangleyData) && packetSize != sizeof(GloriaTankFlowPumpData) && packetSize != sizeof(ChinampaData)
      && packetSize != sizeof(DigitalStablesData) && packetSize != sizeof(VitalSignsRecord) && packetSize != sizeof(VitalSignsRecordV3)
      && packetSize != sizeof(WeatherForecastUpdate) && packetSize != sizeof(SeedlingMonitorData) && packetSize != sizeof(LinkHealthRecord)
      && packetSize != sizeof(LinkAckRecord) && loraSniffer) {
    // A size Paula has no struct for - shown raw, so new/foreign senders are still visible.
    uint8_t raw[256];
    int n = LoRa.readBytes(raw, packetSize < (int)sizeof(raw) ? packetSize : (int)sizeof(raw));
    Serial.printf("SNIFF {\"type\":\"Unknown\",\"size\":%d,\"rssi\":%d,\"snr\":%.2f,\"hex\":\"", packetSize, LoRa.packetRssi(), LoRa.packetSnr());
    for (int i = 0; i < n; i++) Serial.printf("%02X", raw[i]);
    Serial.println("\"}");
  }
}


// Pin 18 as the DYP-A02's UART RX (see ultrasonicSerial) / back to Flow Sensor 1's interrupt.
void startUltrasonicMode() {
  detachInterrupt(SENSOR_INPUT_1);
  ultrasonicFrameBufLen = 0;
  lastUltrasonicDistanceMM = 0;
  lastUltrasonicReadMillis = 0;
  ultrasonicSerial.begin(9600, SERIAL_8N1, SENSOR_INPUT_1, -1);
  currentTestMode = TEST_MODE_ULTRASONIC;
}

void startFlowMode() {
  if (currentTestMode == TEST_MODE_FLOW) return;
  ultrasonicSerial.end();
  attachInterrupt(SENSOR_INPUT_1, pulseCounter, FALLING);
  currentTestMode = TEST_MODE_FLOW;
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
  // DEVICE_POWER on (ADS1115, sensor terminals) - always HIGH on Paula: it never sleeps, and
  // nothing else in this sketch touches pin 26. The hold release covers a board last flashed
  // with Daffodil.ino, which holds the pin LOW through deep sleep.
  gpio_hold_dis((gpio_num_t)SLEEP_SWITCH_26);
  pinMode(SLEEP_SWITCH_26, OUTPUT);
  digitalWrite(SLEEP_SWITCH_26, HIGH);
  // The Daffodil shield powers its WS2812 LEDs through a load switch on GPIO23 (Daffodil.ino
  // LED_CONTROL, HIGH = on). Paula never set it, so its LEDs stayed dark (found 2026-10-14).
  gpio_hold_dis((gpio_num_t)LED_CONTROL);
  pinMode(LED_CONTROL, OUTPUT);
  digitalWrite(LED_CONTROL, HIGH);
  // Let the switched rails settle before the OLED is probed and initialised (2026-11-03): after the
  // factory's flash (a cold boot) the screen stayed dark until a manual reset - an SSD1306 initialised
  // before its supply is up ignores the init sequence.
  delay(150);

  if(debug)Serial.print("DigitalStablesData size=");
  if(debug)Serial.println(sizeof(DigitalStablesData));

  if(debug)Serial.print("ChinampaData size=");
  if(debug)Serial.println(sizeof(ChinampaData));


  if(debug)Serial.print("seedlingMonitorData size=");
  if(debug)Serial.println(sizeof(SeedlingMonitorData));

  // The OLED answers at 0x3C or 0x3D (written 0x78 / 0x7A on the back of 0.96" modules, set by a
  // resistor) - use whichever is there, 0x3C if neither answers (2026-11-03).
  uint8_t oledAddress = SCREEN_ADDRESS;
  Wire.beginTransmission(0x3C);
  if (Wire.endTransmission() != 0) {
    Wire.beginTransmission(0x3D);
    if (Wire.endTransmission() == 0) oledAddress = 0x3D;
  }
  if (debug) Serial.printf("OLED at 0x%02X\n", oledAddress);
  if (!display.begin(SSD1306_SWITCHCAPVCC, oledAddress)) {
    if(debug)Serial.println(F("SSD1309 allocation failed"));
    for (;;)
      ;  // Loop forever if failed
  }
  showVersionBanner("Please Wait");


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
  FastLED.setBrightness(NORMAL_BRIGHTNESS);
  // LED current cap (2026-11-11): V50 measured 3.96 V on Paula's screw terminals and the OLED went blank
  // (still answering on I2C - a brown-out reset of its controller). FastLED dims every frame that would
  // draw more than LED_MAX_MILLIAMPS, so the LEDs can't drag the rail down.
  FastLED.setMaxPowerInVoltsAndMilliamps(5, LED_MAX_MILLIAMPS);

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
    showVersionBanner("LORA Active");  // stays until the first packet is shown
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
    // Yellow L for 2 s: the radio started and the command LEDs work (blue = command sent,
    // green = confirmed - see flashLoraL()).
    flashLoraL(CRGB(255, 200, 0));
    delay(2000);
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



// "Paula", the firmware version and build time - the way to see which build is flashed - under a
// one-line status. Drawn and shown at once.
void showVersionBanner(const char *status) {
#if OLED_SMALL
  // 128x32: status small on top, "Paula v9" big below
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  centerText(status, 0);
  display.setTextSize(2);
  {
    String big = "Paula v" + String(firmwareVersion);
    int16_t x1, y1;
    uint16_t w, h;
    display.getTextBounds(big, 0, 0, &x1, &y1, &w, &h);
    display.setCursor((SCREEN_WIDTH - w) / 2, 14);
    display.print(big);
  }
  display.display();
  return;
#endif
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  centerText(status, 0);
  display.setTextSize(2);
  int16_t x1, y1;
  uint16_t w, h;
  display.getTextBounds("Paula", 0, 0, &x1, &y1, &w, &h);
  display.setCursor((SCREEN_WIDTH - w) / 2, 14);
  display.print("Paula");
  centerText("Version " + String(firmwareVersion), 38);
  centerText(FIRMWARE_BUILD_TIME, 50);
  display.display();
}

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

// OLED_SMALL (128x32): who Paula just heard - the name as big as it fits, since that is what has to
// be read at arm's length. Rows 0-23 the name: text size 3 (18x24 px, 7 characters) when it fits,
// else size 2 (12x16, 10 characters), else size 1; row 24-31 one small line, RSSI and time:
//   CHINAMPA          <- size 3 up to 7 chars ("Chinamp" - "Chinampa" is 8, so size 2)
//   -118dBm 14:03:22
void showHeard() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextWrap(false);
  int16_t x1, y1;
  uint16_t w, h;
  size_t len = strlen(lastHeardName);
  uint8_t size = len <= 7 ? 3 : (len <= 10 ? 2 : 1);
  display.setTextSize(size);
  display.getTextBounds(lastHeardName, 0, 0, &x1, &y1, &w, &h);
  int y = (24 - (int)h) / 2;  // centred in rows 0-23
  if (y < 0) y = 0;
  display.setCursor((SCREEN_WIDTH - (int)w) / 2, y);
  display.print(lastHeardName);
  char line[22];
  relayTickerShowing = relayOn && relayTickerAt && millis() - relayTickerAt < RELAY_TICKER_MS;
  if (relayTickerShowing) {
    // relay ticker: the last packet Paula resent, e.g. "> Chinampa relayed"
    if (relayTickerAction == RELAY_TICK_STRONG) snprintf(line, sizeof(line), "%.8s strong %d", relayTickerName, relayTickerRssi);
    else snprintf(line, sizeof(line), "> %.10s relayed", relayTickerName);
  } else {
    snprintf(line, sizeof(line), "%ddBm %02d:%02d:%02d", lastHeardRssi, currentTimerRecord.hour,
             currentTimerRecord.minute, currentTimerRecord.second);
  }
  centerText(line, 24);
  display.display();
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
  uint8_t savedBrightness = NORMAL_BRIGHTNESS;  // Paula's normal LED brightness, set once in setup()
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

// One status glyph at a time - L, F, R (see GLYPH_L above). Redraws only when what should show
// changes; while a packet's light show runs, every loop draws its next frame instead.
void showStatusGlyphs() {
  unsigned long now = millis();
  if ((long)(packetShowUntil - now) > 0) {
    // rainbow sweep on the middle row only (2026-11-11: all 15 LEDs pulled V50 down - see LED_MAX_MILLIAMPS),
    // each LED a hue, the rainbow sliding right as the show runs
    uint8_t shift = (uint8_t)((PACKET_SHOW_MS - (packetShowUntil - now)) * 255UL / PACKET_SHOW_MS);
    for (int i = 0; i < NUM_LEDS; i++) leds[i] = CRGB(0, 0, 0);
    for (int c = 0; c < 5; c++) leds[5 + c] = CHSV((uint8_t)(c * 40 - shift), 255, 255);
    FastLED.show();
    lastGlyphKey = -1;  // redraw the glyph when the show ends
    return;
  }
  int step = (now / GLYPH_STEP_MS) % 3;
  bool gap = (now % GLYPH_STEP_MS) < GLYPH_GAP_MS;
  bool good = false;
  const uint8_t *glyph = GLYPH_L;
  switch (step) {
    case 0: glyph = GLYPH_L; good = loraActive; break;
    case 1: glyph = GLYPH_F; good = lastWeatherForecastSeconds <= WEATHER_STALE_INTERVAL; break;
    case 2: glyph = GLYPH_R; good = relayOn; break;
  }
  int key = (gap ? 3 : step) | (good ? 4 : 0);
  if (key == lastGlyphKey) return;
  lastGlyphKey = key;
  if (gap) {
    for (int i = 0; i < NUM_LEDS; i++) leds[i] = CRGB(0, 0, 0);
  } else {
    drawGlyph(glyph, good ? CRGB(0, 255, 0) : CRGB(255, 0, 0));
  }
  FastLED.show();
}

void loop() {
  // put your main code here, to run repeatedly:

  if (currentTestMode == TEST_MODE_ULTRASONIC) pollUltrasonicSerial();

  if (clockTicked) {
    portENTER_CRITICAL(&mux);
    clockTicked = false;
    portEXIT_CRITICAL(&mux);
    currentTimerRecord = timeManager.now();
    if (loraActive && !loraReceived) noteNoiseSample(LoRa.rssi());  // the channel's noise here
    lastLoraReceptionSeconds++;
    lastWeatherForecastSeconds++;
    // The physical left/right toggle switch (previously read here on pins 32/33) no longer
    // exists on this board - the Daffodil shield replacing the Paula shield wires pin 33 to
    // Flow Sensor 2 instead. switchPositionLeft keeps its fixed default (see declaration) and
    // is never toggled now.
  }



  if (!oledDetailed) {
    // OLED_SMALL: only the last device heard, redrawn when a new packet came in - and when the relay
    // ticker line runs out
    if (relayTickerShowing && millis() - relayTickerAt >= RELAY_TICKER_MS) lastHeardNew = true;
    if (lastHeardNew) {
      lastHeardNew = false;
      showHeard();
    }
  } else if (!switchPositionLeft) {
    showWeatherForecast();
  } else if (displayingChinampa && refreshChinampaPage) {
    showChinampaPage1();
    //refreshChinampaPage=false;
  }

  if (commandFlashUntil && (long)(millis() - commandFlashUntil) < 0) {
    // the command "L" is showing - see flashLoraL()
  } else {
    if (commandFlashUntil) {
      FastLED.setBrightness(NORMAL_BRIGHTNESS);
      commandFlashUntil = 0;
      lastGlyphKey = -1;  // the command's L is over - draw the status glyph again
    }
    showStatusGlyphs();
  }
  serviceRelay();
  if (loraReceived) {
    //Serial.printf("lora recive Free Heap: %d \n", xPortGetFreeHeapSize());
    if (debug) Serial.printf("lora recive loraPacketSize: %d \n", loraPacketSize);
    if (debug) Serial.println("");
    processLora(loraPacketSize);
    loraReceived = false;
    if (relayJustQueued >= 0) {  // name the packet just queued for relaying - processLora's noteHeard knows it
      strncpy(relayQueue[relayJustQueued].name, lastHeardName, sizeof(relayQueue[relayJustQueued].name) - 1);
      relayQueue[relayJustQueued].name[sizeof(relayQueue[relayJustQueued].name) - 1] = 0;
      relayJustQueued = -1;
    }
    if (relayPendingStrong) {  // not relayed for being strong - on the ticker with the name just heard
      strncpy(relayTickerName, lastHeardName, sizeof(relayTickerName) - 1);
      relayTickerName[sizeof(relayTickerName) - 1] = 0;
      relayTickerAction = RELAY_TICK_STRONG;
      relayTickerRssi = relayPendingStrongRssi;
      relayTickerAt = millis();
      if (!relayTickerAt) relayTickerAt = 1;
      relayPendingStrong = false;
      lastHeardNew = true;
    }
    packetShowUntil = millis() + PACKET_SHOW_MS;  // the packet light show - see showStatusGlyphs
    bool show = false;
    currentPalette = RainbowStripeColors_p;
    currentBlending = NOBLEND;

    //  performLedShow(250);
    // (the rainbow on LEDs 8-14 after each packet is off since 2026-11-11 - it drew over the glyphs)


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
        if (oledDetailed && switchPositionLeft) {
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
        if (oledDetailed) showWeatherForecast();
        weatherForecastNewData = false;
      }
    } else if (loraPacketSize == sizeof(ChinampaData)) {
      displayingChinampa = true;

      //if(debug)
      if (debug) Serial.print("line 513, received from ");
      //if(debug)
      if (debug) Serial.println(chinampaData.devicename);

      if (!oledDetailed) {
        // OLED_SMALL: showHeard() draws it
      } else if (switchPositionLeft) showChinampaPage1();
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
      if (oledDetailed && switchPositionLeft) {
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

  {
    DeviceCommandRecord command;
    if (deviceCommandQueue.takeDue(millis(), command)) sendDeviceCommand(command);
  }

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
    } else if (command.startsWith("GetSerialNumber")) {
      // Same form as serialnumberextractor (what the factory records): 12 lowercase hex of the efuse MAC, then Ok
      Serial.printf("%012llx", ESP.getEfuseMac());
      Serial.println("Ok");
      Serial.flush();
    } else if (command.startsWith("SetProductDefinition")) {
      // SetProductDefinition#<name>#<powerSource>#<battery>#<pcbs>#<firmware> - as Daffodil/Langley (2026-11-03)
      secretManager.saveProductDefinition(generalFunctions.getValue(command, '#', 1), generalFunctions.getValue(command, '#', 2),
                                          generalFunctions.getValue(command, '#', 3), generalFunctions.getValue(command, '#', 4),
                                          generalFunctions.getValue(command, '#', 5));
      Serial.println("Ok-SetProductDefinition");
      Serial.flush();
    } else if (command.startsWith("GetProductDefinition")) {
      // Same reply as Langley/Daffodil, so Inspect and the factory/PaulaDeployer parsers need no branch:
      // name#power#battery#pcbs#firmware#commission#ssid#wifipass#apssid#appass#host#mode#now#devicename#shortname#serial
      String pdName, pdPowerSource, pdBattery, pdPcbs, pdFirmware;
      secretManager.getProductDefinition(pdName, pdPowerSource, pdBattery, pdPcbs, pdFirmware);
      unsigned long commissionDate = secretManager.getCommissionDate();
      if (commissionDate < 1704067200UL) {  // before 2024-01-01 = never set (RTC default) - capture now, as Langley does
        commissionDate = timeManager.getCurrentTimeInSeconds(timeManager.now());
        secretManager.setCommissionDate(commissionDate);
      }
      char pdDeviceName[64] = {0}, pdDeviceShortName[64] = {0}, s1[64] = {0}, s2[64] = {0};
      String tz;
      double lat = 0, lon = 0, alt = 0;
      uint8_t minLed = 0, minWifi = 0;
      secretManager.getDeviceSensorConfig(pdDeviceName, pdDeviceShortName, s1, s2, tz, lat, lon, alt, minLed, minWifi);
      char pdSerial[13];
      snprintf(pdSerial, sizeof(pdSerial), "%012llx", ESP.getEfuseMac());
      Serial.println("Ok-GetProductDefinition#" + pdName + "#" + pdPowerSource + "#" + pdBattery + "#" + pdPcbs + "#" + pdFirmware + "#"
                     + String(commissionDate) + "#" + secretManager.getSSID() + "#" + secretManager.getWifiPassword() + "#"
                     + secretManager.getSoftAPSSID() + "#" + secretManager.getSoftAPPASS() + "#" + secretManager.getHostName() + "#"
                     + (secretManager.getStationMode() ? "Station" : "AccessPoint") + "#"
                     + String(timeManager.getCurrentTimeInSeconds(timeManager.now())) + "#" + String(pdDeviceName) + "#"
                     + String(pdDeviceShortName) + "#" + String(pdSerial));
      Serial.flush();
    } else if (command.startsWith("ScanI2C")) {
      // ScanI2C - probe every 7-bit address and list what answers, with the part Paula expects there
      // (2026-11-11: the OLED stayed dark; this tells a missing/unpowered screen from a firmware problem).
      // One line per device "I2C 0x3C OLED (SSD1306)", then "I2C found=<n>" and Ok-ScanI2C.
      int found = 0;
      for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() != 0) continue;
        found++;
        const char *what = "unknown";
        switch (addr) {
          case 0x3C: case 0x3D: what = "OLED (SSD1306)"; break;
          case 0x44: what = "SHT3x temperature/humidity"; break;
          case 0x48: case 0x49: case 0x4A: case 0x4B: what = "ADS1115 ADC"; break;
          case 0x4F: what = "INA228 voltage monitor"; break;
          case 0x51: what = "PCF8563 RTC"; break;
        }
        Serial.printf("I2C 0x%02X %s\n", addr, what);
      }
      Serial.printf("I2C found=%d%s\n", found, found == 0 ? " (nothing answered - check SDA/SCL and the 3.3V rail)" : "");
      Wire.beginTransmission(0x3C);
      bool oled = Wire.endTransmission() == 0;
      Wire.beginTransmission(0x3D);
      oled = oled || Wire.endTransmission() == 0;
      if (!oled) Serial.println("I2C OLED not found at 0x3C or 0x3D - the screen is unpowered, unplugged or dead");
      Serial.println("Ok-ScanI2C");
      Serial.flush();
    } else if (command.startsWith("Help") || command.startsWith("ListCommands")) {
      // Help / ListCommands - the serial commands, one per line (2026-11-11)
      static const char *const helpLines[] = {
        "GetFirmware                 version and build time",
        "GetSerialNumber             12-hex efuse MAC",
        "GetProductDefinition        factory product definition",
        "SetProductDefinition#name#power#battery#pcbs#firmware",
        "GetTime / SetTime#...       RTC",
        "Help / ListCommands         this list",
        "ScanI2C                     list devices on the I2C bus (OLED, RTC, ADS1115, SHT3x, INA228)",
        "debug#1 / debug#0           debug output on/off",
        "Ping                        alive check (Ok-Ping)",
        "Ping#<dBm>                  LoRa signal-survey ping, 2..17 dBm",
        "LoraSniffer#1 / #0          print every LoRa packet heard",
        "GetNearbyDevices            devices heard over LoRa",
        "GetNearbyPacket#<serial>    last packet from a device + layout check",
        "Relay#1 / Relay#0 / Relay   relay mode on/off/status",
        "Relay#1#<dBm>               relay on, skip packets heard at <dBm> or stronger (0 = relay all)",
        "GetNoise                    channel noise level here",
        "GetSensorData / AsyncData / GetLifeCycleData / GetWPSSensorData",
        "GetIpAddress / IPAddr / SSID / RestartWifi / DeviceWifi",
        "GetCommandCode / VerifyUserCode / GetSecret / SetSecret",
        "SetStatusText / GetSwitchState / SetTestMode",
        "GetUltrasonicReading / ReadClipsal / ReadClipsal2US / StopClipsal / ReadVoltageMonitor",
        "TestI2CTemperatureSensor / TestTankPressureSensor1 / TestTankPressureSensor2",
        "CalibrateFlowSensor1Start / CalibrateFlowSensor1Stop#<ml> (and 2)",
        "PulseStart / PulseFinished / SetFieldId",
      };
      for (const char *line : helpLines) Serial.println(line);
      Serial.println("Ok-Help");
      Serial.flush();
    } else if (command.startsWith("GetFirmware")) {
      Serial.println("Version " + String(firmwareVersion) + " " FIRMWARE_BUILD_TIME);
      Serial.println(F("Ok-GetFirmware"));
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
    } else if (command == "Ping") {
      // Plain "Ping": the factory's / flasher's "are you alive?" - answered like every other firmware.
      // (Before 2026-11-03 it fell into Ping#<dBm> below and sent a LoRa survey ping each time.)
      Serial.println(F("Ok-Ping"));
      Serial.flush();
    } else if (command.startsWith("Ping#")) {
      // Ping#<dBm> - see sendPing(). 2..17 dBm (the radio's PA_BOOST range); default 5.
      int power = generalFunctions.getValue(command, '#', 1).toInt();
      if (power < 2 || power > 17) power = 5;
      sendPing(power);
      Serial.flush();
    } else if (command.startsWith("LoraSniffer")) {
      // LoraSniffer#1 / LoraSniffer#0 - see printSniff()
      loraSniffer = generalFunctions.getValue(command, '#', 1).toInt() > 0;
      Serial.println("Ok-LoraSniffer");
      Serial.flush();
    } else if (command.startsWith("GetNearbyDevices")) {
      printNearbyDevices();
      Serial.println("Ok-GetNearbyDevices");
      Serial.flush();
    } else if (command.startsWith("Relay")) {
      // Relay#1 / Relay#0 - see the relay section; plain "Relay" just reports
      String arg = generalFunctions.getValue(command, '#', 1);
      if (arg.length()) {
        relayOn = arg.toInt() > 0;
        if (!relayOn) for (int i = 0; i < RELAY_QUEUE; i++) relayQueue[i].len = 0;
      }
      // Relay#1#<dBm>: the strong-signal limit (2026-11-11) - packets at or above it are not relayed;
      // 0 = relay everything. -120..-1 accepted.
      String limit = generalFunctions.getValue(command, '#', 2);
      limit.trim();
      if (limit.length()) {
        int v = limit.toInt();
        if (v == 0 || (v >= -120 && v < 0)) relayStrongDbm = v;
      }
      Serial.print("RELAY ");
      printRelayJson();
      Serial.println();
      Serial.println("Ok-Relay");
      Serial.flush();
    } else if (command.startsWith("GetNoise")) {
      Serial.print("NOISE ");
      printNoiseJson();
      Serial.println();
      Serial.println("Ok-GetNoise");
      Serial.flush();
    } else if (command.startsWith("GetNearbyPacket")) {
      // GetNearbyPacket#<serial> - see printNearbyPacket()
      String serialHex = generalFunctions.getValue(command, '#', 1);
      serialHex.trim();
      serialHex.toUpperCase();
      if (printNearbyPacket(serialHex)) Serial.println("Ok-GetNearbyPacket");
      else Serial.println("Failure-GetNearbyPacket-not heard");
      Serial.flush();
    } else if (command.startsWith("DeviceWifi")) {
      // DeviceWifi#<serial, 16 hex digits>#on|off - PaulaDeployer's Nearby devices button
      String sn = generalFunctions.getValue(command, '#', 1);
      String state = generalFunctions.getValue(command, '#', 2);
      state.trim();
      uint8_t serial[8];
      bool valid = sn.length() == 16 && (state == "on" || state == "off");
      for (int i = 0; valid && i < 8; i++) {
        char hex[3] = { sn[i * 2], sn[i * 2 + 1], 0 };
        char *end;
        serial[i] = (uint8_t)strtoul(hex, &end, 16);
        valid = *end == 0;
      }
      if (valid) {
        deviceCommandQueue.request(serial, state == "on" ? DEVICE_COMMAND_WIFI_ON : DEVICE_COMMAND_WIFI_OFF, 0, millis());
        Serial.println("Ok-DeviceWifi");
      } else {
        Serial.println("Failure-DeviceWifi: expected DeviceWifi#<16 hex digits>#on|off");
      }
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
        if (currentTestMode != TEST_MODE_ULTRASONIC) startUltrasonicMode();
        Serial.println("Ok-SetTestMode-Ultrasonic");
      } else if (mode == "Flow") {
        startFlowMode();
        endClipsal();
        Serial.println("Ok-SetTestMode-Flow");
      } else {
        Serial.println("Failure-SetTestMode-unknown mode " + mode + " (use Flow or Ultrasonic)");
      }
      Serial.flush();
    } else if (command.startsWith("ReadVoltageMonitor")) {
      readVoltageMonitor();
    } else if (command.startsWith("ReadClipsal2US")) {  // before ReadClipsal, which it starts with
      readClipsal2US();
    } else if (command.startsWith("ReadClipsal")) {
      readClipsal();
    } else if (command.startsWith("StopClipsal")) {
      startFlowMode();
      endClipsal();
      Serial.println("Ok-StopClipsal");
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
