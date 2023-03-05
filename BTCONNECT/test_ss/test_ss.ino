/*
  Software serial multple serial test

 Receives from the hardware serial, sends to software serial.
 Receives from software serial, sends to hardware serial.

 The circuit:
 * RX is digital pin 10 (connect to TX of other device)
 * TX is digital pin 11 (connect to RX of other device)

 Note:
 Not all pins on the Mega and Mega 2560 support change interrupts,
 so only the following can be used for RX:
 10, 11, 12, 13, 50, 51, 52, 53, 62, 63, 64, 65, 66, 67, 68, 69

 Not all pins on the Leonardo and Micro support change interrupts,
 so only the following can be used for RX:
 8, 9, 10, 11, 14 (MISO), 15 (SCK), 16 (MOSI).

 created back in the mists of time
 modified 25 May 2012
 by Tom Igoe
 based on Mikal Hart's example

 This example code is in the public domain.

 */
#include <SoftwareSerial.h>
SoftwareSerial mySerial(10, 11); // RX, TX

int ticks = 0;

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println("READY");

  mySerial.begin(9600);

  while (!mySerial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println("CONNECTED");
}

void loop() { // run over and over
  /*
  if (Serial.available()) {
    char x = Serial.read();
    mySerial.write(x);
    Serial.write(x);
  }
  */

  delay(1000);
  sendMessage(ticks);
  if (ticks >= 10) {
    ticks = 0;
  }
  ticks++;
}

void sendMessage(int tick) {
  if (tick == 0) {
    mySerial.write("Test Message");
  }

  if (tick == 2) {
    mySerial.write("Another Message");
  }

  if (tick == 4) {
    mySerial.write("Hey Athena");
  }

  if (tick == 6) {
    mySerial.write("Hi Luke");
  }
}
