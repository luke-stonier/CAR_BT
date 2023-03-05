#include <ELMduino.h>
#include <map>
#include <BluetoothSerial.h>
#include <LiquidCrystal_I2C.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif

BluetoothSerial SerialBT;
LiquidCrystal_I2C lcd(0x27, 16, 2); // I2C address 0x27, 16 column and 2 rows
ELM327 myELM327;

int deviceCount=0;
int reconnectDelay = 0;
uint32_t rpm=0;


#define BT_DISCOVER_TIME  10000
esp_spp_sec_t sec_mask=ESP_SPP_SEC_NONE; // or ESP_SPP_SEC_ENCRYPT|ESP_SPP_SEC_AUTHENTICATE to request pincode confirmation
esp_spp_role_t role=ESP_SPP_ROLE_SLAVE; // or ESP_SPP_ROLE_MASTER

// std::map<BTAddress, BTAdvertisedDeviceSet> btDeviceList;

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
  
  config();
  // SerialBT.setPin("1234"); // doesn't seem to change anything
  // SerialBT.enableSSP(); // doesn't seem to change anything

  delay(1000);
  StartConnection();
  delay(100);
  LCDWrite("Connect cycle", "complete");
  delay(1000);
}

void config() {
  if(! SerialBT.begin("Supra ESP32", true) ) {
    LCDWrite("Bluetooth setup", "Failed to start");
    abort();
  } else {
    LCDWrite("Bluetooth setup", "Started");
  }

  delay(1000);
}

boolean OBDconnect() {
  if (SerialBT.connect("OBDII"))
    {
      LCDWrite("OBD CONNECT", "SUCCESS");
      delay(1500);
      return true;
    } else {
      LCDWrite("OBD CONNECT", "FAIL");
      SerialBT.flush();  
      SerialBT.disconnect();
      SerialBT.end();
      delay(1500);
      return false;
    }
}

void StartConnection() {
  int count = esp_bt_gap_get_bond_device_num();
  LCDWrite(String(count) + " Paired", "devices...");
  delay(2000);
  if(SerialBT.isClosed() || !SerialBT.connected()) {
    LCDWrite("Initial Connect", "Attempting...");
    if (OBDconnect()) {
      return;
    }
  }
  
  LCDWrite("BT is open", "Start Connect...");

  delay(2000);
  deviceCount = 0;
  LCDWrite("Starting", "Discovery");
  BTScanResults* btDeviceList = SerialBT.getScanResults();  // maybe accessing from different threads!
  if (SerialBT.discoverAsync([](BTAdvertisedDevice* pDevice) {
      deviceCount++;
      LCDWrite("Found new device", "device: " + String(deviceCount));
  })){
    SerialBT.discoverAsyncStop();
    delay(BT_DISCOVER_TIME);
    LCDWrite("End discovery", String(btDeviceList->getCount()) + " Devices");
    if(btDeviceList->getCount() > 0) {
      BTAddress addr;
      int channel=0;
      for (int i=0; i < btDeviceList->getCount(); i++) {
        BTAdvertisedDevice *device=btDeviceList->getDevice(i);
        
        LCDWrite(device->getName().c_str(), device->getAddress().toString().c_str());
        
        std::map<int,std::string> channels=SerialBT.getChannels(device->getAddress());
        if(channels.size() > 0) {
          addr = device->getAddress();
          channel=channels.begin()->first;
        }

        delay(1000);
      }
      if(addr) {
        LCDWrite("Connecting...", addr.toString().c_str());
        SerialBT.connect(addr, channel, sec_mask, role);
        delay(1000);

        if(!SerialBT.isClosed() && SerialBT.connected()) {
          LCDWrite("Connected...", addr.toString().c_str());
          myELM327.begin(SerialBT, true, 2000);
          delay(1000);
        } else {
          LCDWrite("Connect failed", addr.toString().c_str());
        }
      }
    } else {
      LCDWrite("No devices", "");
    }
  } else {
    LCDWrite("Discovery error", "");
    delay(5000);
    config();
  }
  
}

void loop() {
  if(! SerialBT.isClosed() && SerialBT.connected()) {
    float tempRPM = myELM327.rpm();
    if (myELM327.nb_rx_state == ELM_SUCCESS)
    {
      rpm = (uint32_t)tempRPM;
      /*Serial.print("RPM: "); Serial.println(rpm);*/
      LCDWrite("RPM: " + String(rpm), "");
    }
    else if (myELM327.nb_rx_state != ELM_GETTING_MSG) {
      //myELM327.printError();
      LCDWrite("ELM COM FAIL", "requires debug");
    }
  } else {
    LCDWrite("Not connected...", "Retry in: " + String(10 - reconnectDelay) + "s");
    if (reconnectDelay >= 10) {
      reconnectDelay = 0;
      StartConnection();
      delay(100);
      LCDWrite("Connect cycle", "complete");
    }
    reconnectDelay++;
    delay(1000);
  }
}
