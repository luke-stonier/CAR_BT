#include <ELMduino.h>
#include <SoftwareSerial.h>
#include <LiquidCrystal_I2C.h>


LiquidCrystal_I2C lcd(0x27, 16, 2); // I2C address 0x27, 16 column and 2 rows
SoftwareSerial mySerial(2, 3); // RX, TX
ELM327 myELM327;
uint32_t rpm = 0;

void setup()
{
  Serial.begin(115200);
  mySerial.begin(115200);
  myELM327.begin(mySerial, true, 2000);
}


void loop()
{
  float tempRPM = myELM327.rpm();

  if (myELM327.nb_rx_state == ELM_SUCCESS)
  {
    rpm = (uint32_t)tempRPM;
    Serial.print("RPM: "); Serial.println(rpm);
  }
  else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
    myELM327.printError();
}
