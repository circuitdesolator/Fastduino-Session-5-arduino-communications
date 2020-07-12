/***************************************************
  This is an example for our Adafruit FONA Cellular Module

  Designed specifically to work with the Adafruit FONA
  ----> http://www.adafruit.com/products/1946
  ----> http://www.adafruit.com/products/1963
  ----> http://www.adafruit.com/products/2468
  ----> http://www.adafruit.com/products/2542

  These cellular modules use TTL Serial to communicate, 2 pins are
  required to interface
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ****************************************************/

/*
  THIS CODE IS STILL IN PROGRESS!

  Open up the serial console on the Arduino at 115200 baud to interact with FONA

  Note that if you need to set a GPRS APN, username, and password scroll down to
  the commented section below at the end of the setup() function.
*/
#include "Adafruit_FONA.h"

#define SMS_CHECK 4000            // Time alloted to check SMS receive
#define FONA_RX 2
#define FONA_TX 3
#define FONA_RST 6
#define SEND_BUTTON 7
#define BUZZER 8

// We default to using software serial. If you want to use hardware serial
// (because softserial isnt supported) comment out the following three lines
// and uncomment the HardwareSerial line
#include <SoftwareSerial.h>
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;

// Hardware serial is also possible!
//  HardwareSerial *fonaSerial = &Serial1;

// Use this for FONA 800 and 808s
Adafruit_FONA fona = Adafruit_FONA(FONA_RST);
// Use this one for FONA 3G
//Adafruit_FONA_3G fona = Adafruit_FONA_3G(FONA_RST);

char SMSbuffer[200]; // We'll store the SMS content in here
uint16_t SMSLength;
String SMSString = "";

uint8_t type;

uint32_t prev_time = 0;
uint32_t current_time = 0;
uint32_t prev_time_sms = 0;

void setup()
{
  Serial.begin(115200);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(SEND_BUTTON, INPUT_PULLUP);
  Serial.println(F("Basic Sending"));
  Serial.println(F("Initializing....(May take 3 seconds)"));

  digitalWrite(BUZZER, LOW);

  fonaSerial->begin(9600);
  if (! fona.begin(*fonaSerial)) {
    Serial.println(F("Couldn't find FONA"));
    while (1);
  }
  type = fona.type();
  Serial.println(F("FONA is OK"));
  Serial.print(F("Found "));
  switch (type) {
    case FONA800L:
      Serial.println(F("FONA 800L")); break;
    case FONA800H:
      Serial.println(F("FONA 800H")); break;
    case FONA808_V1:
      Serial.println(F("FONA 808 (v1)")); break;
    case FONA808_V2:
      Serial.println(F("FONA 808 (v2)")); break;
    case FONA3G_A:
      Serial.println(F("FONA 3G (American)")); break;
    case FONA3G_E:
      Serial.println(F("FONA 3G (European)")); break;
    default:
      Serial.println(F("???")); break;
  }

  // Print module IMEI number.
  char imei[16] = {0}; // MUST use a 16 character buffer for IMEI!
  uint8_t imeiLen = fona.getIMEI(imei);
  if (imeiLen > 0) {
    Serial.print("Module IMEI: "); Serial.println(imei);
  }

  if (!fona.sendSMS("09xxyyyzzzz", "demo 4"))
  {
    Serial.println(F("Failed"));
  } else {
    Serial.println(F("Sent!"));
  }

}

void loop()
{
  current_time = millis();

  if (current_time - prev_time_sms >= SMS_CHECK)
  {
    readsms_cmd();
  }


  if (digitalRead(SEND_BUTTON) == 0)
  {
    fona.sendSMS("09xxyyyzzzz", "EMERGENCY BUTTON IS PRESSED!");

    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
    digitalWrite(LED_BUILTIN, LOW);
    delay(200);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
    digitalWrite(LED_BUILTIN, LOW);
    delay(200);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
    digitalWrite(LED_BUILTIN, LOW);
    Serial.print("Sent!");
  }
}

void readsms_cmd()
{
  prev_time_sms = current_time;

  if (!fona.readSMS(1, SMSbuffer, 1000, &SMSLength))
  { // pass in buffer and max len!
    Serial.println("Failed!");
  }
  else
  {
    SMSString = String(SMSbuffer);
    Serial.print("SMS: "); Serial.println(SMSString);
  }

  boolean deleteSMSDone = fona.deleteSMS(1);

  if (deleteSMSDone == true)
  {
    Serial.println("OK!");
  }
  else
  {
    Serial.println("Couldn't delete, try again.");
  }

  if (SMSString == "ON" || SMSString == "On" || SMSString == "on")
  {
    digitalWrite(LED_BUILTIN, HIGH);
    digitalWrite(BUZZER, HIGH);
  }
  else if (SMSString == "OFF" || SMSString == "Off" || SMSString == "off")
  {
    digitalWrite(LED_BUILTIN, LOW);
    digitalWrite(BUZZER, LOW);
  }
  else
  {
    Serial.print("Invalid command.");
  }

  SMSString = "";
}
