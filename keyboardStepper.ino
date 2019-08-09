#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>
#include <Stepper.h>
#include <FastLED.h>
#include <PacketSerial.h>

#define  SLAVE_ADDRESS           0x29  //slave address,any number from 0x01 to 0x7F
/*********  Initialize Global Objects  *********/
Adafruit_NeoPixel aftBar = Adafruit_NeoPixel(27, 11, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel forBar = Adafruit_NeoPixel(30, 10, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel clRing = Adafruit_NeoPixel(12, 12, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel mcuBox = Adafruit_NeoPixel(3, 5, NEO_GRB + NEO_KHZ800);

Encoder       frontKnob(2, 3);
PacketSerial  Teensy;
Stepper       stepper(200, 6, 7, 8, 9);

/*********  Initialize Global Variables  *********/
int           doorPos;      //Position, in steps at the driver gear, of the door.  **NOT** angle of the door
int           frameCursor   =   0;
uint8_t       pos           =   0;
int           oldPos        =   0;
int           fade          =   0;
int           doorLimit     =   -183;
int           inc           =   -1;
int           behavoir      =   0;
uint8_t       toTeensy[]    =   {0, 0, 0, 0, 0, 0, 0, 0};
unsigned long elapsed;
word          sawTooth[128];
boolean       isOpen;  //True = Open, False = Closed
long          serMin        =   0; //Serial drop-in replacement for now.minute()
long          serTen        =   0;
long          serHour       =   0; //Serial drop-in replacement for now.hour()
byte          curMin        =   0;
byte          rollover[24]  =   {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
bool          colorSwitch;
byte          rnd           =   0;
CRGB          leds[30];
CRGB          fBar[30];
CRGB          rBar[27];
CRGB          mBox[3];

void setup()
{
  delay(1000);

  Teensy.setPacketHandler(&onTeensyPacket);
  Teensy.begin(38400);

  //Serial.begin(38400);
  //Serial.setTimeout(50);
  //  rtc.begin();
  //  // following line sets the RTC to the date & time this sketch was compiled
  //  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

  stepper.setSpeed(100);

  for (byte i = 0; i < 64; i++) {
    sawTooth[i] = i;
    sawTooth[63 + i] = 64 - i;
  }

  /*********  INPUTS  *********/
  pinMode(4, INPUT);  //Front Circular Toggle Button  *[Depressed = LOW = Door Open], [Raised = HIGH = Door Closed]
  pinMode(A0, INPUT); //Second Left Toggle Switch
  pinMode(A3, INPUT); //First Left Toggle Switch

  /*********  OUTPUTS  *********/
  pinMode(13, OUTPUT);
  pinMode(A1, OUTPUT);  //Front GREEN led
  pinMode(A2, OUTPUT);  //Front YELLOW led

  /*********  Start and Clear NeoPixel Objects  *********/
  aftBar.begin();
  forBar.begin();
  clRing.begin();
  mcuBox.begin();
  aftBar.show();
  forBar.show();
  clRing.show();
  mcuBox.show();

  FastLED.addLeds<NEOPIXEL, 10>(fBar, 30);
  FastLED.addLeds<NEOPIXEL, 11>(rBar, 27);
  FastLED.addLeds<NEOPIXEL,  5>(mBox,  3);
  //  FastLED.addLeds<NEOPIXEL, 10>(leds, 30);  //forBar
  //  FastLED.addLeds<NEOPIXEL, 11>(leds, 27);  //aftBar
  FastLED.setBrightness(255);

  /*
       If the main circular switch is low when the board receives power update the status
       Note: because of this code block, the board assumes the door is closed when the board receives power
  */
  testLEDs();

  if (digitalRead(4) == LOW) {
    isOpen = true;
    doorPos = doorLimit;
    for (int i = 0; i < 26; i++) {
      stepper.step(1);
      delay(20);
    }
  } else {
    isOpen = false;
    doorPos = 0;
  }

  colorSwitch = false;
}

void loop()
{
  bool dispTime;
  int newPos = frontKnob.read();
  if (newPos != pos) {
    oldPos = pos;
    if (newPos + 1 < pos) {
      if (pos <= 0) {
        pos = 0;
        frontKnob.write(0);
      } else {
        pos = constrain(pos - 1, 0, 255);
      }
    } else if (newPos - 1 > pos) {
      if (pos >= 255) {
        pos = 255;
        frontKnob.write(255);
      } else {
        pos = constrain(pos + 1, 0, 255);
      }
    }

    if (abs(oldPos - pos) >= 1) {
      elapsed = millis();
    }

    byte posRing = floor(pos / 22);
    for (byte i = 0; i < 12; i++) {
      if (i <= posRing) {
        clRing.setPixelColor(rollover[i + 12], constrain(pos, 0, 255), constrain(255 - pos, 0, 255), 0);
      } else {
        clRing.setPixelColor(rollover[i + 12], 0, 0, 0);
      }
    }
  }

  if (abs(millis() - elapsed) < 2500) {
    clRing.show();
    dispTime = false;
  } else {
    dispTime = true;
  }

  /*********  Door Stepper Control  ********/
  if (digitalRead(4) == LOW) {
    if (doorPos > doorLimit && doorPos <= 0) {
      digitalWrite(13, HIGH);
      if (doorPos % 40 == 0) {
        colorSwitch = !colorSwitch;
      }
      stepper.step(-1);
      delay(15);
      doorPos--;

      clRing.setPixelColor(map(doorPos, -152, 0, 0, 11), 255 + doorPos, 90 - doorPos, 60);
      hazardLights();
      showThenClearAll();
    } else {
      digitalWrite(13, LOW);
    }
  } else if (digitalRead(4) == HIGH) {
    if (doorPos < 0 && doorPos >= doorLimit) {
      digitalWrite(13, HIGH);
      if (doorPos % 40 == 0) {
        colorSwitch = !colorSwitch;
      }
      stepper.step(1);
      delay(15);
      doorPos++;

      clRing.setPixelColor(map(doorPos, -152, 0, 0, 11), 255 + doorPos, 90 - doorPos, 60);
      hazardLights();
      showThenClearAll();
    } else {
      digitalWrite(13, LOW);
    }
  }

  /*********  Set front green/yellow status LEDs  *********/
  if (doorPos == doorLimit) {
    digitalWrite(A1, HIGH);
    digitalWrite(A2, LOW);
  } else {
    digitalWrite(A1, LOW);
    digitalWrite(A2, HIGH);
  }

  /**********  'True' Main loop:  *********/
  if (doorPos == 0  ||  doorPos == doorLimit) {
    clearFor();
    clearAft();

    //DateTime now = rtc.now();
    //if (millis() % 500000) rtc.begin();

    //    if (now.minute() != curMin) {
    if (serMin != curMin) {
      clearRing();
      displayTime();
      //      curMin = now.minute();
      curMin = serMin;
    }

    if (digitalRead(A3) == HIGH) {
      if (digitalRead(A0) == HIGH)
      {
        confetti(fBar, 30, 3);
        confetti(rBar, 27, 0);
        toTeensy[0] = frameCursor;
        toTeensy[1] = pos;
        toTeensy[2] = 1;
        if (frameCursor % 8 == 0)
          Teensy.send(toTeensy, 8);

        if (frameCursor >= 512) {
          frameCursor = 0;
        } else if (frameCursor < 0) {
          frameCursor = 0;
        } else {
          frameCursor++;
        }
      }
      else
      {
        int offset = map(pos, 0, 255, 0, 128);
        int rB = constrain(8    + map(pos, 0, 255, 0, 64), 0, 255);
        int gB = constrain(24   + map(pos, 0, 255, 0, -43), 0, 255);
        int bB = constrain(16   + map(pos, 0, 255, 0, 128), 0, 255);
        int rP = constrain(255  + map(pos, 0, 255, 0, 64), 0, 255);
        int gP = constrain(109  + map(pos, 0, 255, 0, -43), 0, 255);
        int bP = constrain(16   + map(pos, 0, 255, 0, 128), 0, 255);

        pulse(fBar, 30, 3, 2, rB, gB, bB, rP, gP, bP);
        pulse(rBar, 27, 0, 2, rB, gB, bB, rP, gP, bP);

        toTeensy[0] = frameCursor;
        toTeensy[1] = pos;
        toTeensy[2] = 0;

         if (frameCursor == 0)
          Teensy.send(toTeensy, 8);

        if (frameCursor >= 512) {
          frameCursor = 0;
        } else if (frameCursor < 0) {
          frameCursor = 0;
        } else {
          frameCursor++;
        }
      }

      /*  Set Lights For Box  */
      if (doorPos == doorLimit) {
        if (digitalRead(A0) == HIGH)
        {
          fBar[0] = CHSV( pos, 255, 255);
          fBar[1] = CHSV( pos, 255, 255);
          fBar[2] = CHSV( pos, 255, 255);
          mBox[0] = CHSV( pos, 255, 255);
          mBox[1] = CHSV( pos, 255, 255);
          mBox[2] = CHSV( pos, 255, 255);
        }
        else
        {
          fBar[0] = CRGB( 255, 255, 255);
          fBar[1] = CRGB( 255, 255, 255);
          fBar[2] = CRGB( 255, 255, 255);
          mBox[0] = CRGB( 255, 255, 255);
          mBox[1] = CRGB( 255, 255, 255);
          mBox[2] = CRGB( 255, 255, 255);
        }
      }
      else
      {
        if (digitalRead(A0) == HIGH)
        {
          fBar[0] = CHSV( pos, 255, 64);
          fBar[1] = CHSV( pos, 255, 64);
          fBar[2] = CHSV( pos, 255, 64);
          mBox[0] = CHSV( pos, 255, 64);
          mBox[1] = CHSV( pos, 255, 64);
          mBox[2] = CHSV( pos, 255, 64);
        }
        else
        {
          fBar[0] = CRGB( 255, 64, 2);
          fBar[1] = CRGB( 255, 64, 2);
          fBar[2] = CRGB( 255, 64, 2);
          mBox[0] = CRGB( 255, 64, 2);
          mBox[1] = CRGB( 255, 64, 2);
          mBox[2] = CRGB( 255, 64, 2);
        }
      }
      FastLED.show();
      clearRing();
      if (dispTime) {
        displayTime();
      }
    }
    else {
      showThenClearAll();
    }
  }
  Teensy.update();
}

void onTeensyPacket(const uint8_t* buffer, size_t size)
{
  uint8_t tmp[size];
  memcpy(tmp, buffer, size);
  uint8_t m = tmp[0];
  uint8_t d = tmp[1];
  uint8_t h = tmp[2];
//  if ((m == 0) || (m == serMin+1))
    serMin  = tmp[0];
//  if ((d == 0) || (d == serTen+1))
    serTen  = tmp[1];
//  if ((h == 0) || (h == serHour+1))
    serHour = tmp[2];
}

void confetti(CRGB * strip, int num_leds, int offset_from_start) {
  int cursorFast = random16(offset_from_start, num_leds);
  fadeToBlackBy(strip, num_leds, 2);
  strip[cursorFast] += CHSV(pos + random(-32, 32), 255, 128);
  strip[num_leds - cursorFast] += CHSV(pos + random(-32, 32), 255, 128);
  delay(10);
}

//void rainDrop(int pixel, Adafruit_NeoPixel& o) {
//
//  uint32_t  red       = getRed(o.getPixelColor(pixel));
//  uint32_t  green     = getGreen(o.getPixelColor(pixel));
//  uint32_t  blue      = getBlue(o.getPixelColor(pixel));
//  uint32_t  composite;
//
//  if (frameCursor < 199 && frameCursor >= 0) {
//    //int offset = constrain(frameCursor-200, 0,
//    composite = o.Color(constrain(128 - frameCursor, 0, 128), constrain(128 - frameCursor, 0, 128), constrain(200 - frameCursor, 0, 200)) | red | green | blue;
//  } else if (frameCursor < 399 && frameCursor >= 200) {
//    composite = o.Color(constrain(328 - frameCursor, 0, 128), constrain(328 - frameCursor, 0, 128), constrain(400 - frameCursor, 0, 200)) | red | green | blue;
//  }
//
//  o.setPixelColor(pixel, composite);
//  //o.show();
//}

uint32_t getGreen(uint32_t c) {
  uint32_t green = clRing.Color(0, 255, 0);
  return (green & c);
}

uint32_t getRed(uint32_t c) {
  uint32_t red = clRing.Color(255, 0, 0);
  return red & c;
}

uint32_t getBlue(uint32_t c) {
  uint32_t blue = clRing.Color(0, 0, 255);
  return blue & c;
}

void pulse( CRGB * strip,
            int   num_leds,
            int   offset_from_start,
            int   behavior,
            int   rBase,
            int   gBase,
            int   bBase,
            int   rPulse,
            int   gPulse,
            int   bPulse
          )
{
  /*
     Description of behaviors:
     0: Out-In
     1: Downstream
     2: Mid-Out
     3: Upstream
  */
  num_leds = num_leds - offset_from_start;

  int rComp   = 0;
  int gComp   = 0;
  int bComp   = 0;

  int pixel_offset = int(512.0 / float(num_leds));
  int upper_limit = (behavior % 2 == 0 ? (num_leds / 2) + 1 : num_leds);

  for (int i = 0; i < upper_limit; i++) {
    int c = constrain(255 - abs(2 * frameCursor - (255 + i * pixel_offset) + 1), 0, 255);
    //    if(i == 0)
    //      Serial.println(c);
    rComp = (rBase == rPulse ? rBase : map(c, 0, 255, rBase, rPulse));
    gComp = (gBase == gPulse ? gBase : map(c, 0, 255, gBase, gPulse));
    bComp = (bBase == bPulse ? bBase : map(c, 0, 255, bBase, bPulse));

    switch (behavior)
    {
      // "Out-in" loop:
      case 0:
        {
          strip[offset_from_start + i] = CRGB(rComp, gComp, bComp);
          strip[offset_from_start + num_leds - 1 - i] = CRGB(rComp, gComp, bComp);
        }
        break;

      // "Mid-out" loop:
      case 2:
        {
          strip[offset_from_start + num_leds / 2 - i] = CRGB(rComp, gComp, bComp);
          strip[offset_from_start + num_leds / 2 + i] = CRGB(rComp, gComp, bComp);
        }
        break;

      // "Upstream" loop:
      case 1:
        strip[offset_from_start + i] = CRGB(rComp, gComp, bComp);
        break;

      // "Downstream" loop:
      case 3:
        strip[offset_from_start + num_leds - 1 - i] = CRGB(rComp, gComp, bComp);
        break;
    }
  }
  delay(5);
}

void hazardLights() {
  for (byte i = 0; i < 14; i++) {
    if (colorSwitch) {
      aftBar.setPixelColor(     i,  255,  255,  255 );
      aftBar.setPixelColor(i + 14,  255,  255,  16  );
      forBar.setPixelColor(i +  2,  32,   32,   255 );
      forBar.setPixelColor(i + 16,  255,  255,  255 );
      forBar.setPixelColor(     0,  0,    0,    0   );
      forBar.setPixelColor(     1,  0,    0,    0   );
      mcuBox.setPixelColor(     0,  255,  0,    0   );
      mcuBox.setPixelColor(     1,  255,  0,    0   );
      mcuBox.setPixelColor(     2,  255,  0,    0   );
    } else {
      aftBar.setPixelColor(     i,  32,   32,   255 );
      aftBar.setPixelColor(i + 14,  255,  255,  255 );
      forBar.setPixelColor(i +  2,  255,  255,  255 );
      forBar.setPixelColor(i + 16,  255,  255,  16  );
      forBar.setPixelColor(     0,  255,    0,  0   );
      forBar.setPixelColor(     1,  255,    0,  0   );
      mcuBox.setPixelColor(     0,  0,      0,  0   );
      mcuBox.setPixelColor(     1,  0,      0,  0   );
      mcuBox.setPixelColor(     2,  0,      0,  0   );
    }
  }
}

void showThenClearAll() {
  aftBar.show();
  forBar.show();
  clRing.show();
  mcuBox.show();
  clearmcuBox();
  clearRing();
  clearAft();
  clearFor();
}

void displayTime() {
  //DateTime  now     =   rtc.now();
  int       curOne  =   constrain(serMin, 0, 9);
  int       curTen  =   serTen * 2;
  int       inc     =   (curOne <= 4) ? 0 : 1;

  for (int i = 0; i < curOne - 4 * inc; i++)
    clRing.setPixelColor(rollover[curTen + i + 1], 128, 46, 0);

  clRing.setPixelColor(rollover[curTen + inc],  128,  0,      128 );
  clRing.setPixelColor(rollover[serHour],         0,    255,  128 );
  clRing.show();
}

void clearRing() {
  for (int i = 0; i < 12; i++)
    clRing.setPixelColor(i, 0, 0, 0);
}

void clearAft() {
  for (byte i = 0; i < 27; i++)
    aftBar.setPixelColor(i, 0, 0, 0);
}

void clearFor() {
  for (byte i = 0; i < 30; i++)
    forBar.setPixelColor(i, 0, 0, 0);
}

void clearmcuBox() {
  mcuBox.setPixelColor(0, 0, 0, 0);
  mcuBox.setPixelColor(1, 0, 0, 0);
  mcuBox.setPixelColor(2, 0, 0, 0);
}

void testLEDs() {
  for (int r = 0; r < 128; r++) {
    for (int j = 0; j < 30; j++) {
      aftBar.setPixelColor(j, r,  0,  0);
      forBar.setPixelColor(j, r,  0,  0);
      clRing.setPixelColor(j, r,  0,  0);
      mcuBox.setPixelColor(j, r,  0,  0);
    }
    aftBar.show();
    forBar.show();
    clRing.show();
    mcuBox.show();
  }
  for (int o = 0; o < 128; o++) {
    for (int j = 0; j < 30; j++) {
      aftBar.setPixelColor(j, o,  o / 3,  0);
      forBar.setPixelColor(j, o,  o / 3,  0);
      clRing.setPixelColor(j, o,  o / 3,  0);
      mcuBox.setPixelColor(j, o,  o / 3,  0);
    }
    aftBar.show();
    forBar.show();
    clRing.show();
    mcuBox.show();
  }
  for (int y = 0; y < 128; y++) {
    for (int j = 0; j < 30; j++) {
      aftBar.setPixelColor(j, y,  y,  0);
      forBar.setPixelColor(j, y,  y,  0);
      clRing.setPixelColor(j, y,  y,  0);
      mcuBox.setPixelColor(j, y,  y,  0);
    }
    aftBar.show();
    forBar.show();
    clRing.show();
    mcuBox.show();
  }
  for (int g = 0; g < 128; g++) {
    for (int j = 0; j < 30; j++) {
      aftBar.setPixelColor(j, 0,  g,  0);
      forBar.setPixelColor(j, 0,  g,  0);
      clRing.setPixelColor(j, 0,  g,  0);
      mcuBox.setPixelColor(j, 0,  g,  0);
    }
    aftBar.show();
    forBar.show();
    clRing.show();
    mcuBox.show();
  }
  for (int b = 0; b < 128; b++) {
    for (int j = 0; j < 30; j++) {
      aftBar.setPixelColor(j, 0,  0,  b);
      forBar.setPixelColor(j, 0,  0,  b);
      clRing.setPixelColor(j, 0,  0,  b);
      mcuBox.setPixelColor(j, 0,  0,  b);
    }
    aftBar.show();
    forBar.show();
    clRing.show();
    mcuBox.show();
  }
  for (int i = 0; i < 128; i++) {
    for (int j = 0; j < 30; j++) {
      aftBar.setPixelColor(j, 0,  i,  i);
      forBar.setPixelColor(j, 0,  i,  i);
      clRing.setPixelColor(j, 0,  i,  i);
      mcuBox.setPixelColor(j, 0,  i,  i);
    }
    aftBar.show();
    forBar.show();
    clRing.show();
    mcuBox.show();
  }
  for (int v = 0; v < 128; v++) {
    for (int j = 0; j < 30; j++) {
      aftBar.setPixelColor(j, v,  0,  v);
      forBar.setPixelColor(j, v,  0,  v);
      clRing.setPixelColor(j, v,  0,  v);
      mcuBox.setPixelColor(j, v,  0,  v);
    }
    aftBar.show();
    forBar.show();
    clRing.show();
    mcuBox.show();
  }
  for (int w = 0; w < 128; w++) {
    for (int j = 0; j < 308; j++) {
      aftBar.setPixelColor(j, w,  w,  w);
      forBar.setPixelColor(j, w,  w,  w);
      clRing.setPixelColor(j, w,  w,  w);
      mcuBox.setPixelColor(j, w,  w,  w);
    }
    aftBar.show();
    forBar.show();
    clRing.show();
    mcuBox.show();
  }
}

