
//  SerialIn_SerialOut_HM-10_01
//
//  Uses hardware serial to talk to the host computer and AltSoftSerial for communication with the bluetooth module
//
//  What ever is entered in the serial monitor is sent to the connected device
//  Anything received from the connected device is copied to the serial monitor
//  Does not send line endings to the HM-10
//
//  Pins
//  BT VCC to Arduino 5V out.
//  BT GND to GND
//  Arduino D8 (SS RX) - BT TX no need voltage divider
//  Arduino D9 (SS TX) - BT RX through a voltage divider (5v to 3.3v)
//dfx6t

// Quick Documentation
// To set this module up, the HM-10 BLE needs to be set up in CENTRAL mode,
// connect with the motion detector hardware using:
// AT+ROLE1 set to central mode
// AT+DISC? find devices
// AT+CON<Mac ID> connect, and AT+IMME0 (default) to reconnect on startup,
// then AT+RESET to reset the device with the new settings
// http://www.martyncurrey.com/hm-10-bluetooth-4ble-modules/#HM-10+scan

// Quick status
// This unit works now with LED
// Testing pushbutton with 12 light neopixel, will tie to BLE later

#include <Time.h>
#include <TimeLib.h>
#include <AltSoftSerial.h>
#include <Adafruit_NeoPixel.h>
#include <DS1307RTC.h>
#include "RTClib.h"

// Date and time functions using a DS1307 RTC connected via I2C and Wire lib
#include <Wire.h>

RTC_DS1307 rtc;

AltSoftSerial BTserial;
// https://www.pjrc.com/teensy/td_libs_AltSoftSerial.html

#define BUTTON_PIN 2    // Digital IO pin connected to the button.  This will be
// driven with a pull-up resistor so the switch should
// pull the pin to ground momentarily.  On a high -> low
// transition the button press logic will execute.

#define PIXEL_PIN 6    // Digital IO pin connected to the NeoPixels.

#define FORCE_TIME_UPDATE
#define BOARD_HAS_RTC 

const int pixelCount = 12;

// Parameter 1 = number of pixels in strip,  neopixel stick has 8
// Parameter 2 = pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_RGB     Pixels are wired for RGB bitstream
//   NEO_GRB     Pixels are wired for GRB bitstream, correct for neopixel stick
//   NEO_KHZ400  400 KHz bitstream (e.g. FLORA pixels)
//   NEO_KHZ800  800 KHz bitstream (e.g. High Density LED strip), correct for neopixel stick
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(pixelCount, PIXEL_PIN, NEO_GRB + NEO_KHZ800);

bool oldState = LOW;
int rate = 9600;
char c = ' ';
boolean NL = true;
int ledPin = 12;
int curPixel = 0;
int red = 1;
int green = 1;
int blue = 1;

int pixelBuckets[pixelCount] = { 0 };

int bucketsPerHalfDay(int numPixels) {
  int minutesPerHalfDay = 12 * 60;
  return minutesPerHalfDay / pixelCount;
}

void printDigits(int digits) {
  Serial.print(":");
  if (digits < 10)
    Serial.print('0');
  Serial.print(digits);
}

void digitalClockDisplay(time_t t) {
  // digital clock display of the time
  Serial.print(hour(t));
  printDigits(minute(t));
  printDigits(second(t));
  Serial.println();
}

void digitalClockDisplay() {
    return digitalClockDisplay(now());
}

void setupTime() {
#ifdef FORCE_TIME_UPDATE
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); // set clock to time of last compilation
#endif

#ifdef BOARD_HAS_RTC //real time clock installed
  if (!rtc.isrunning()) {
    Serial.println("RTC is NOT running!");
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    //rtc.adjust(DateTime(2017, 1, 6, 0, 0, 0));
  } else {
    Serial.println("RTC is running");
  }

  setSyncProvider(RTC.get); // the function to get the time from the RTC
  if (timeStatus() != timeSet) {
    Serial.println("Time set failed");
  }
  setTime(rtc.now().unixtime());
#else /// no real time clock, set clock to last compile time
  // manually set the time
  setTime(23, 50, 0, 1, 17, 18);
#endif
}

void setup()
{
  Serial.begin(rate);
  Serial.print("Sketch:   ");   Serial.println(__FILE__);
  Serial.print("Uploaded: ");   Serial.println(__DATE__);
  Serial.println(" ");

  BTserial.begin(rate);
  Serial.print("BTserial started at ");
  Serial.println(rate);
  pinMode(ledPin, OUTPUT);
  pixels.begin(); // This initializes the NeoPixel library.
  pixels.setBrightness(30);
  pixels.show();

  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }
  setupTime();
  
  digitalClockDisplay();
}

int getMinutesPerPixel() {
  return 60 * 12 / pixelCount;
}

int getCurrentPixelBucket() {
  int hour12 = hour();
  Serial.print("hour12: ");
  Serial.println(hour12);
  hour12 = hour12 > 11 ? hour12 - 12 : hour12;
  Serial.print("hour12 adj: ");
  Serial.println(hour12);
  int curMinute = hour12 * 60 + minute();
  Serial.print("minute: ");
  Serial.println(curMinute);
  int bucket = curMinute / getMinutesPerPixel();
  Serial.print("bucket: ");
  Serial.println(bucket);
  return bucket;
}

uint32_t getColorForCount(int count) {
  const int maxCount = 9;
  count = count > maxCount - 1 ? maxCount - 1: count;
  Serial.print("Count: ");
  Serial.println(count);
  static uint32_t countColors[maxCount] = {
    pixels.Color(0, 0, 0), // off
    pixels.Color(255, 0, 0), // red
    pixels.Color(255, 50, 0), // orange
    pixels.Color(225, 240, 0), // yellow
    pixels.Color(0, 128, 0), // green
    pixels.Color(0, 0, 255), // blue
    pixels.Color(75, 0, 130), // indigo
    pixels.Color(220, 120, 220),  // violet
    pixels.Color(255, 255, 255)  // white
  };
  Serial.println(countColors[count]);
  return countColors[count];
}

void updatePixelColor(int bucket) {
  uint32_t color = getColorForCount(pixelBuckets[bucket]);
  pixels.setPixelColor(bucket, color);
  pixels.show(); // This sends the updated pixel color to the hardware.
}

void incrementPixel(int bucket) {
  pixelBuckets[bucket] = pixelBuckets[bucket] + 1;
  updatePixelColor(bucket);
}

void registerMotion() {
  int bucket = getCurrentPixelBucket();
  incrementPixel(bucket);
}

void resetPixels() {
  for (int i = 0; i < pixelCount; i++) {
    updatePixelColor(i);
  }
}

void checkBT() {
  // Read from the Bluetooth module and send to the Arduino Serial Monitor
  if (BTserial.available())
  {
    c = BTserial.read();
    Serial.write(c);
    if (c == '1') {
      digitalWrite(ledPin, HIGH);
    }
    else {
      digitalWrite(ledPin, LOW);
    }
  }
}

void checkButton() {
  // Get current button state.
  bool newState = digitalRead(BUTTON_PIN);

  // Check if state changed from high to low (button press).
  if (newState == LOW && oldState == HIGH) {
    // Short delay to debounce button.
    delay(20);
    // Check if button is still low after debounce.
    newState = digitalRead(BUTTON_PIN);
    if (newState == LOW) {
      Serial.print(newState);
      registerMotion();
    }
  }

  // Set the last button state to the old state.
  oldState = newState;
}

void checkInput() {
  //checkBT();
  checkButton();

}
void loop()
{
  //    digitalWrite(ledPin, HIGH);
  checkInput();
}

