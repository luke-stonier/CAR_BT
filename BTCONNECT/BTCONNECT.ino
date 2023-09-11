#include <FastLED.h>
#include <Adafruit_NeoPixel.h>
#include <ELMduino.h>
#include <map>
#include <BluetoothSerial.h>
#include <LiquidCrystal_I2C.h>
#include <Preferences.h>
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include"esp_gap_bt_api.h"
#include "esp_err.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif

#define BT_DISCOVER_TIME  10000
#define REMOVE_BONDED_DEVICES 1
#define PAIR_MAX_DEVICES 20

#define PIN 13
#define NUMPIXELS  100
//150
CRGB leds[NUMPIXELS];

int maxRPM = 6000;
int minRPM = 750;
int LED_BUILTIN = 2;

Preferences preferences;
BluetoothSerial SerialBT;
LiquidCrystal_I2C lcd(0x27, 16, 2); // I2C address 0x27, 16 column and 2 rows // BLUE -> D22 / GREEN -> D21
ELM327 myELM327;

boolean elmConnected = false;
int deviceCount=0;
int reconnectDelay = 0;
uint32_t rpm=0;
String _mode = "";

uint8_t pairedDeviceBtAddr[PAIR_MAX_DEVICES][6];
char bda_str[18];


esp_spp_sec_t sec_mask=ESP_SPP_SEC_NONE; // or ESP_SPP_SEC_ENCRYPT|ESP_SPP_SEC_AUTHENTICATE to request pincode confirmation
esp_spp_role_t role=ESP_SPP_ROLE_SLAVE; // or ESP_SPP_ROLE_MASTER

void LCDWrite(String line1, String line2) {
  
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(line1);
  lcd.setCursor(0,1);
  lcd.print(line2);
  
}

void setup() {
  lcd.init(); //initialize the lcd
  lcd.backlight(); //open the backlight 
  
  preferences.begin("suprabt", false); 
  FastLED.addLeds<NEOPIXEL, PIN>(leds, NUMPIXELS);
  FastLED.setBrightness(32);

  pinMode(4, INPUT);
  pinMode (LED_BUILTIN, OUTPUT);
  FastLED.setBrightness(100);

  FastLED.showColor(CRGB(255,0,0));
  delay(500);
  FastLED.showColor(CRGB(0,255,0));
  delay(500);
  FastLED.showColor(CRGB(0,0,255));
  delay(500);

  _mode = preferences.getString("mode", "")
  if (_mode == "rainbow") {
    preferences.putString("mode", "BT");
  } else if(_mode == "BT") {
    preferences.putString("mode", "rainbow");
  } else {
    _mode = "rainbow"
    preferences.putString("mode", "rainbow");
  }

  rainbowCycle(10);
  byte buttonState = digitalRead(4);
  if (_mode == "BT") {
    FastLED.showColor(CRGB(0,0,255));
    delay(5000);
    if (config()) {
      if (!SerialBT.hasClient()) {
        StartConnection();
      }
    }
  } else {
    FastLED.showColor(CRGB(0,255,255));
  }

  /*
  if (buttonState == HIGH) {
    FastLED.showColor(CRGB(0,0,0));
    delay(5000);
    if (config()) {
      if (!SerialBT.hasClient()) {
        StartConnection();
      }
    }
  }
  */
}

char *bda2str(const uint8_t* bda, char *str, size_t size)
{
  if (bda == NULL || str == NULL || size < 18) {
    return NULL;
  }
  sprintf(str, "%02x:%02x:%02x:%02x:%02x:%02x",
          bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
  return str;
}

boolean config() {
  if(! SerialBT.begin("Supra ESP32", true) ) {
    LCDWrite("Bluetooth setup", "Failed to start");
    FastLED.showColor(CRGB(255,0,0));
    delay(1000);
    return false;
  } else {
    LCDWrite("Bluetooth setup", "Started");
    FastLED.showColor(CRGB(0,255,0));
    delay(1000);
    return true;
  }
}

boolean OBDconnect(String address) {  
  uint8_t MAC[6];
  char* ptr;

  MAC[0] = strtol(address.c_str(), &ptr, HEX );
  for( uint8_t i = 1; i < 6; i++ )
  {
    MAC[i] = strtol(ptr+1, &ptr, HEX );
  }

  LCDWrite("Attempting", "Connection");

  if (SerialBT.connect(MAC, preferences.getInt("elm_channel", 0)))
    {
      LCDWrite("OBD CONNECT", "SUCCESS");
      StartELMCom();
      return true;
    } else {
      LCDWrite("OBD CON FAIL", "Restart OBDII");
      delay(1000);
      if (SerialBT.disconnect()) {
        LCDWrite("Disconnected Successfully", "");
      }
      delay(1500);
      return false;
    }
}

void StartELMCom() {
  if(!SerialBT.isClosed() && SerialBT.connected()) {
    myELM327.begin(SerialBT, true, 2000);
    elmConnected = true;
    delay(1000);
  } else {
    LCDWrite("Connect failed", "");
  }
}

void StartConnection() {
  boolean doPairing = true;
  int count = esp_bt_gap_get_bond_device_num();
  byte buttonState = digitalRead(4);
  LCDWrite(String(count) + " Paired devices", buttonState == LOW ? "Attempt OBDII" : "Skip OBDII");
  String existingAddress = preferences.getString("elm_address", "");
  delay(2000);
  if(buttonState == LOW && existingAddress != "" && (SerialBT.isClosed() || !SerialBT.connected())) {
    if (OBDconnect(existingAddress)) {
      doPairing = false;
    }
  }

  if (doPairing) { Pair(); }
}

void Pair() {
  LCDWrite("BT is open", "Start Connect...");
  
  deviceCount = 0;
  LCDWrite("Starting", "Discovery");
  BTScanResults* btDeviceList = SerialBT.getScanResults();  // maybe accessing from different threads!
  if (SerialBT.discoverAsync([](BTAdvertisedDevice* pDevice) {
      deviceCount++;
      String _name = pDevice->haveName() ? pDevice->getName().c_str() : pDevice->getAddress().toString().c_str();
      LCDWrite(_name, "Device #" + String(deviceCount) + " (" + pDevice->getRSSI() + ")");
  })){
    delay(BT_DISCOVER_TIME);
    SerialBT.discoverAsyncStop();
    LCDWrite("End discovery", String(btDeviceList->getCount()) + " Devices");
    if(btDeviceList->getCount() > 0) {
      BTAddress addr;
      int channel=0;
      for (int i=0; i < btDeviceList->getCount(); i++) {
        BTAdvertisedDevice *device=btDeviceList->getDevice(i);

        FastLED.showColor(CRGB(0, 255, 255));
        String _name = device->haveName() ? device->getName().c_str() : "No name";
        LCDWrite("Config: " + _name, device->getAddress().toString().c_str());
        
        std::map<int,std::string> channels=SerialBT.getChannels(device->getAddress());
        if(channels.size() > 0) {
          addr = device->getAddress();
          channel=channels.begin()->first;
        }

        delay(1000);
        FastLED.showColor(CRGB(0, 0, 0));
      }
      if(addr) {
        LCDWrite("Connecting...", addr.toString().c_str());
        SerialBT.connect(addr, channel, sec_mask, role);
        delay(1000);

        if(!SerialBT.isClosed() && SerialBT.connected()) {
          LCDWrite("Connected...", addr.toString().c_str());
          preferences.putString("elm_address", addr.toString().c_str());
          preferences.putInt("elm_channel", channel);
          StartELMCom();
          delay(1000);
        } else {
          LCDWrite("Connect failed", addr.toString().c_str());
          FastLED.showColor(CRGB(255, 0, 0));
          delay(500);
          FastLED.showColor(CRGB(0, 0, 0));
        }
      }
    } else {
      LCDWrite("No devices", "");
      FastLED.showColor(CRGB(255, 0, 0));
      delay(500);
      FastLED.showColor(CRGB(0, 0, 0));
    }
  } else {
    LCDWrite("Discovery error", "");
    FastLED.showColor(CRGB(255, 0, 0));
    delay(500);
    FastLED.showColor(CRGB(0, 0, 0));
    config();
  }
  
}

void loop() {
  byte buttonState = digitalRead(4);
  //if (buttonState == HIGH) {  // LIGHT = ON
  if (_mode == "BT" {
    dynamic_cycle(); 
  } else {
    rainbowCycle(10);
  }
}

void dynamic_cycle() {
  if(!SerialBT.isClosed() && SerialBT.connected()) {
    if (elmConnected) {
      float tempRPM = myELM327.rpm();
      if (myELM327.nb_rx_state == ELM_SUCCESS)
      {
        rpm = (uint32_t)tempRPM;
        LCDWrite("RPM: " + String(rpm), "");
        uint8_t hue = map(rpm, minRPM, maxRPM, 100, 0);
        FastLED.showColor(CHSV(hue, 255, 255));
      }
      else if (myELM327.nb_rx_state != ELM_GETTING_MSG) {
        LCDWrite("ELM COM FAIL", "requires debug");
        FastLED.showColor(CRGB(255, 0, 0));
      }
    } else {
      LCDWrite("NO ELM327 DEVICE", "CONNECTED...");
    }
  } else {
    LCDWrite(" Not connected", "Retry in: " + String(5 - reconnectDelay) + "s");
    if (reconnectDelay >= 5) {
      reconnectDelay = 0;
      FastLED.showColor(CRGB(0, 255, 0));
      delay(500);
      FastLED.showColor(CRGB(0, 0, 0));
      StartConnection();
      delay(100);
    } else {
      reconnectDelay++;
      FastLED.showColor(CRGB(255, 0, 0));
      delay(500);
      FastLED.showColor(CRGB(0, 0, 0));
      delay(500);
    }
  }
}

void rainbowCycle(int SpeedDelay) {
  LCDWrite("RAINBOW CYCLE", "FOR DAYS");
  
  byte *c;
  uint16_t i, j;

  for(j=0; j<256; j++) { // 5 cycles of all colors on wheel
    for(i=NUMPIXELS; i > 0; i--) {
      c=Wheel(((i * 256 / NUMPIXELS) + j) & 255);
      setPixel(i, *c, *(c+1), *(c+2));
    }
    FastLED.show();
    delay(SpeedDelay);
  }
}

byte * Wheel(byte WheelPos) {
  static byte c[3];
 
  if(WheelPos < 85) {
   c[0]=WheelPos * 3;
   c[1]=255 - WheelPos * 3;
   c[2]=0;
  } else if(WheelPos < 170) {
   WheelPos -= 85;
   c[0]=255 - WheelPos * 3;
   c[1]=0;
   c[2]=WheelPos * 3;
  } else {
   WheelPos -= 170;
   c[0]=0;
   c[1]=WheelPos * 3;
   c[2]=255 - WheelPos * 3;
  }

  return c;
}

void setPixel(int Pixel, byte red, byte green, byte blue) {
 leds[Pixel].r = red;
 leds[Pixel].g = green;
 leds[Pixel].b = blue;
}

void setAll(byte red, byte green, byte blue) {
  for(int i = 0; i < NUMPIXELS; i++ ) {
    setPixel(i, red, green, blue);
  }
  FastLED.show();
}
