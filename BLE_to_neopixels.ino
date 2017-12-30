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
//
 
#include <AltSoftSerial.h>
#include <Adafruit_NeoPixel.h>

AltSoftSerial BTserial; 
// https://www.pjrc.com/teensy/td_libs_AltSoftSerial.html
 
#define BUTTON_PIN   2    // Digital IO pin connected to the button.  This will be
                          // driven with a pull-up resistor so the switch should
                          // pull the pin to ground momentarily.  On a high -> low
                          // transition the button press logic will execute.

#define PIXEL_PIN    6    // Digital IO pin connected to the NeoPixels.

#define PIXEL_COUNT 12

// Parameter 1 = number of pixels in strip,  neopixel stick has 8
// Parameter 2 = pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_RGB     Pixels are wired for RGB bitstream
//   NEO_GRB     Pixels are wired for GRB bitstream, correct for neopixel stick
//   NEO_KHZ400  400 KHz bitstream (e.g. FLORA pixels)
//   NEO_KHZ800  800 KHz bitstream (e.g. High Density LED strip), correct for neopixel stick
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(PIXEL_COUNT, PIXEL_PIN, NEO_GRB + NEO_KHZ800);

bool oldState = LOW;
int rate = 9600;
char c = ' ';
boolean NL = true;
int ledPin = 12;
int curPixel = 0;
int red = 1;
int green = 1;
int blue = 1;

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
      pixels.setPixelColor(curPixel++, pixels.Color(red, green++, blue)); // Moderately bright green color.
      blue += 2;
      red += 3;
      if (curPixel > PIXEL_COUNT - 1) {
        curPixel = 0;
        blue = 0;
      }
      pixels.show(); // This sends the updated pixel color to the hardware.
      
    }
  }

  // Set the last button state to the old state.
  oldState = newState;
}

void checkInput() {
//  checkBT();
  checkButton();
   
}
void loop()
{
//    digitalWrite(ledPin, HIGH);
  checkInput();
}

