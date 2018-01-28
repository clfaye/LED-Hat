#include "FastLED.h"

CRGBPalette16 gPal;
/* Rate of cooling. Play with to change fire from
   roaring (smaller values) to weak (larger values) */
#define COOLING 2

/* How hot is "hot"? Increase for brighter fire */
#define HOT 180
#define MAXHOT HOT*Height

boolean sinusoidOn = true;


const byte Width  = 32;
const byte Height = 8;
// y dimension  x=0, y=0 at lower left hand corner (pixel 8)

float size = 4.0;    // amplitude of the curves
const unsigned long millis_increment = 20; // millis increment is effectively "speed" and should be renamed
const float pi2 = PI * 2.0;

const byte radioButtonPin_A = 21;
const byte radioButtonPin_B = 20;
const byte radioButtonPin_C = 19;
const byte radioButtonPin_D = 18;

const byte auxInputA = 4;
const byte auxInputB = 5;

boolean changeInput = false;
boolean teensyOn = true;

// NUM_LEDS = Width * Height
#define NUM_LEDS_PER_STRIP 64
// Note: this can be 12 if you're using a teensy 3 and don't mind soldering the pads on the back
#define NUM_STRIPS 4
#define NUM_LEDS      256
int BRIGHTNESS  =  50;
#define FPS 25
#define FPS_DELAY 1000/FPS
CRGB leds[NUM_LEDS];

float centerRedWidth = Width / 5;
float centerRedHeight = Height / 2;
float centerBlueWidth = Width * 4 / 5;
float centerBlueHeight = Height / 2;
float centerGreenWidth = Width  / 2;
float centerGreenHeight = Height / 2;

float  changeCenterRedWidth = 0.11;
float  changeCenterRedHeight = 0.021;
float  changeCenterBlueWidth = 0.011;
float  changeCenterBlueHeight = 0.031;
float  changeCenterGreenWidth = 0.12;
float  changeCenterGreenHeight = 0.01;

float maxCenterWidth = Width;
float minCenterWidth = 0.0;
float maxCenterHeight = Height;
float minCenterHeight = 0.0;

byte centerRedWidthUp = true;
byte centerRedHeightUp = true;
byte centerBlueWidthUp = false;
byte centerBlueHeightUp = true;
byte centerGreenWidthUp = true;
byte centerGreenHeightUp = false;

elapsedMillis centerRedWidthChange;
elapsedMillis centerRedHeightChange;
elapsedMillis centerBlueWidthChange;
elapsedMillis centerBlueHeightChange;
elapsedMillis centerGreenWidthChange;
elapsedMillis centerGreenHeightChange;

float  speedRed1 = 0.0023;
float  speedRed2 = 0.0012;
float  speedBlue1 = 0.0021;
float  speedBlue2 = 0.0010;
float  speedGreen1 = 0.0018;
float  speedGreen2 = 0.004;

float  changeSpeedRed1 = 0.0001;
float  changeSpeedRed2 = 0.0005;
float  changeSpeedBlue1 = 0.0008;
float  changeSpeedBlue2 = 0.0001;
float  changeSpeedGreen1 = 0.0002;
float  changeSpeedGreen2 = 0.0004;

float maxSpeedColors = 0.005;
float minSpeedColors = 0.000;

byte  speedRed1Up = false;
byte  speedRed2Up = true;
byte  speedBlue1Up = false;
byte  speedBlue2Up = true;
byte  speedGreen1Up = true;
byte  speedGreen2Up = false;

float  sizeRed1 = 2.0;
float  sizeRed2 = 2.5;
float  sizeBlue1 = 3.0;
float  sizeBlue2 = 3.5;
float  sizeGreen1 = 4.0;
float  sizeGreen2 = 5.5;

float  changeSizeRed1 = 0.05;
float  changeSizeRed2 = 0.02;
float  changeSizeBlue1 = 0.03;
float  changeSizeBlue2 = 0.01;
float  changeSizeGreen1 = 0.02;
float  changeSizeGreen2 = 0.03;

float maxSizeColors = 5.0;
float minSizeColors = 0.5;

byte  sizeRed1Up = false;
byte  sizeRed2Up = false;
byte  sizeBlue1Up = true;
byte  sizeBlue2Up = false;
byte  sizeGreen1Up = true;
byte  sizeGreen2Up = false;


float phaseRedPaletteWidth = 0.0;
float phaseRedPaletteCenter = 0.0;
float phaseBluePaletteWidth = 0.0;
float phaseBluePaletteCenter = 0.0;
float phaseGreenPaletteWidth = 0.0;
float phaseGreenPaletteCenter = 0.0;

float  speedRedPaletteWidth = 0.0003;
float  speedRedPaletteCenter = 0.0002;
float  speedBluePaletteWidth = 0.00015;
float  speedBluePaletteCenter = 0.00034;
float  speedGreenPaletteWidth = 0.0008;
float  speedGreenPaletteCenter = 0.0004;

float  changeSpeedRedPaletteWidth = 0.0001;
float  changeSpeedRedPaletteCenter = 0.0005;
float  changeSpeedBluePaletteWidth = 0.0008;
float  changeSpeedBluePaletteCenter = 0.0001;
float  changeSpeedGreenPaletteWidth = 0.0002;
float  changeSpeedGreenPaletteCenter = 0.0004;

uint8_t RedPaletteWidth = 255;
uint8_t RedPaletteCenter = 127;
uint8_t BluePaletteWidth = 255;
uint8_t BluePaletteCenter = 127;
uint8_t GreenPaletteWidth = 255;
uint8_t GreenPaletteCenter = 127;

float phaseRed1 = 0.0;
float phaseRed2 = 0.0;
float phaseBlue1 = 0.0;
float phaseBlue2 = 0.0;
float phaseGreen1 = 0.0;
float phaseGreen2 = 0.0;

elapsedMillis timeButtonA;
elapsedMillis timeButtonB;
elapsedMillis timeButtonC;
elapsedMillis timeButtonD;

void setup()
{
  LEDS.addLeds<WS2811_PORTD, NUM_STRIPS, GRB>(leds, NUM_LEDS_PER_STRIP);
  FastLED.setBrightness(BRIGHTNESS);
  FastLED.setDither( 0 );

  pinMode(radioButtonPin_A, INPUT_PULLDOWN);
  pinMode(radioButtonPin_B, INPUT_PULLDOWN);
  pinMode(radioButtonPin_C, INPUT_PULLDOWN);
  pinMode(radioButtonPin_D, INPUT_PULLDOWN);

  attachInterrupt(digitalPinToInterrupt(radioButtonPin_A), buttonPressed_A, RISING);
  attachInterrupt(digitalPinToInterrupt(radioButtonPin_B), buttonPressed_B, RISING);

  pinMode(auxInputA, OUTPUT);
  pinMode(auxInputB, OUTPUT);

  digitalWrite(auxInputA, HIGH); // when this is set HIGH it latches to the Teensy
  digitalWrite(auxInputB, LOW); // when this is set HIGH it latches to the Aux Input
  delay(1000);
  digitalWrite(auxInputA, LOW); // when this is set HIGH it latches to the Teensy

  gPal = HeatColors_p;

}


void loop()
{
  checkInput();
  checkBrightness();
  if (teensyOn)
  {
    checkBrightness();
    if (sinusoidOn)
    {
      sinusoid();
    }
    else
    {
      random16_add_entropy( random() ); // We chew a lot of entropy
      Fireplace();
      FastLED.delay(FPS_DELAY); //
    }
    FastLED.show();
  }
}

void sinusoid()
{

  // the speed constants in these expressions are what to play with - each "phase" cycles through 0 to 2pi at a different rate
  // the constants dictate the relative speed that each one cycles from 0 to 2pi
  // the millis_increment dicates the overall speed of the visualization
  // play with the centers, make variables instead of Width/2 use redCenter etc
  //

  phaseRed1 += speedRed1 * millis_increment;
  phaseRed2 += speedRed2 * millis_increment;
  phaseBlue1 += speedBlue1 * millis_increment;
  phaseBlue2 += speedBlue2 * millis_increment;
  phaseGreen1 += speedGreen1 * millis_increment;
  phaseGreen2 += speedGreen2 * millis_increment;

  // this section makes sure that the individual phases don't exceed 2pi
  // for some reason, the sine and cosine functions have a real problem with anything over 2pi and get dramatically slower
  if (phaseRed1 > pi2) phaseRed1 -= pi2;
  if (phaseRed2 > pi2) phaseRed2 -= pi2;
  if (phaseBlue1 > pi2) phaseBlue1 -= pi2;
  if (phaseBlue2 > pi2) phaseBlue2 -= pi2;
  if (phaseGreen1 > pi2) phaseGreen1 -= pi2;
  if (phaseGreen2 > pi2) phaseGreen2 -= pi2;


  float sinRed1 = sinf(phaseRed1) * sizeRed1;
  float cosRed2 = cosf(phaseRed2) * sizeRed2;
  float sinBlue1 = sinf(phaseBlue1) * sizeBlue1;
  float cosBlue2 = cosf(phaseBlue2) * sizeBlue2;
  float sinGreen1 = sinf(phaseGreen1) * sizeGreen1;
  float cosGreen2 = cosf(phaseGreen2) * sizeGreen2;



  //This section dictates the center and width of the rgb palettes
  // max width is 255 and center is 127

  phaseRedPaletteWidth += speedRedPaletteWidth * millis_increment ;
  phaseRedPaletteCenter += speedRedPaletteCenter * millis_increment  ;
  phaseBluePaletteWidth += speedBluePaletteWidth * millis_increment ;
  phaseBluePaletteCenter += speedBluePaletteCenter * millis_increment ;
  phaseGreenPaletteWidth += speedGreenPaletteWidth * millis_increment ;
  phaseGreenPaletteCenter += speedGreenPaletteCenter * millis_increment ;


  if (phaseRedPaletteWidth > pi2) phaseRedPaletteWidth -= pi2;
  if (phaseRedPaletteCenter > pi2) phaseRedPaletteCenter -= pi2;
  if (phaseBluePaletteWidth > pi2) phaseBluePaletteWidth -= pi2;
  if (phaseBluePaletteCenter > pi2) phaseBluePaletteCenter -= pi2;
  if (phaseGreenPaletteWidth > pi2) phaseGreenPaletteWidth -= pi2;
  if (phaseGreenPaletteCenter > pi2) phaseGreenPaletteCenter -= pi2;

  RedPaletteWidth = ( ( sinf(phaseRedPaletteWidth) + 1 ) * 254 / 2 ) + 1 ;
  RedPaletteCenter = ( ( sinf(phaseRedPaletteCenter) + 1 ) * 254 / 2) + 1 ;
  BluePaletteWidth = ( ( sinf(phaseBluePaletteWidth) + 1 ) * 254 / 2 ) + 1 ;
  BluePaletteCenter = ( ( sinf(phaseBluePaletteCenter) + 1 ) * 254 / 2) + 1 ;
  GreenPaletteWidth = ( ( sinf(phaseGreenPaletteWidth) + 1 ) * 254 / 2 ) + 1 ;
  GreenPaletteCenter = ( ( sinf(phaseGreenPaletteCenter) + 1 ) * 254 / 2) + 1 ;



  if ((RedPaletteCenter + RedPaletteWidth / 2) > 254)
  {
    RedPaletteCenter = 255 - RedPaletteWidth / 2;
  }
  if ((RedPaletteCenter - RedPaletteWidth / 2) < 1)
  {
    RedPaletteCenter = 1 + RedPaletteWidth / 2;
  }

  if ((BluePaletteCenter + BluePaletteWidth / 2) > 254)
  {
    BluePaletteCenter = 255 - BluePaletteWidth / 2;
  }
  if ((BluePaletteCenter - BluePaletteWidth / 2) < 1)
  {
    BluePaletteCenter = 1 + BluePaletteWidth / 2;
  }

  if ((GreenPaletteCenter + GreenPaletteWidth / 2) > 254)
  {
    GreenPaletteCenter = 255 - GreenPaletteWidth / 2;
  }
  if ((GreenPaletteCenter - GreenPaletteWidth / 2) < 1)
  {
    GreenPaletteCenter = 1 + GreenPaletteWidth / 2;
  }


  for (uint8_t y = 0; y < Height; y++) {
    for (uint8_t x = 0; x < Width; x++) {

      float cx = x + sinRed1 - centerRedWidth;
      float cy = y + cosRed2 - centerRedHeight;
      float v = 127 * (1 + sinf ( sqrtf (cx * cx + cy * cy ) ) );
      uint8_t data = v;
      data = (data * RedPaletteWidth / 255) + (RedPaletteCenter - RedPaletteWidth / 2);
      leds[XY(x, y)].r = data;

      cx = x + sinBlue1 - centerBlueWidth;
      cy = y + cosBlue2 - centerBlueHeight;
      v = 127 * (1 + sinf ( sqrtf (cx * cx + cy * cy ) ) );
      data = v;
      data = (data * BluePaletteWidth / 255) + (BluePaletteCenter - BluePaletteWidth / 2);
      leds[XY(x, y)].b = data;

      cx = x + sinGreen1 - centerGreenWidth;
      cy = y + cosGreen2 - centerGreenHeight;
      v = 127 * (1 + sinf ( sqrtf (cx * cx + cy * cy ) ) );
      data = v;
      data = (data * GreenPaletteWidth / 255) + (GreenPaletteCenter - GreenPaletteWidth / 2);
      leds[XY(x, y)].g = data;

    }
  }

  // RED VARIABLES
  // RED CENTERS
  if (centerRedWidthUp)
  {
    centerRedWidth += changeCenterRedWidth;
    if (centerRedWidth > maxCenterWidth)
    {
      centerRedWidth = maxCenterWidth;
      centerRedWidthUp = false;
    }
  }
  else
  {
    centerRedWidth -= changeCenterRedWidth;
    if (centerRedWidth < minCenterWidth)
    {
      centerRedWidth = minCenterWidth;
      centerRedWidthUp = true;
    }
  }

  if (centerRedHeightUp)
  {
    centerRedHeight += changeCenterRedHeight;
    if (centerRedHeight > maxCenterHeight)
    {
      centerRedHeight = maxCenterHeight;
      centerRedHeightUp = false;
    }
  }
  else
  {
    centerRedHeight -= changeCenterRedHeight;
    if (centerRedHeight < minCenterHeight)
    {
      centerRedHeight = minCenterHeight;
      centerRedHeightUp = true;
    }
  }

  // RED SPEED
  if (speedRed1Up)
  {
    speedRed1 += changeSpeedRed1;
    if (speedRed1 > maxSpeedColors)
    {
      speedRed1 = maxSpeedColors;
      speedRed1Up = false;
    }
  }
  else
  {
    speedRed1 -= changeSpeedRed1;
    if (speedRed1 < minSpeedColors)
    {
      speedRed1 = minSpeedColors;
      speedRed1Up = true;
    }
  }

  if (speedRed2Up)
  {
    speedRed2 += changeSpeedRed2;
    if (speedRed2 > maxSpeedColors)
    {
      speedRed2 = maxSpeedColors;
      speedRed2Up = false;
    }
  }
  else
  {
    speedRed2 -= changeSpeedRed2;
    if (speedRed2 < minSpeedColors)
    {
      speedRed2 = minSpeedColors;
      speedRed2Up = true;
    }
  }

  // RED SIZE
  if (sizeRed1Up)
  {
    sizeRed1 += changeSizeRed1;
    if (sizeRed1 > maxSizeColors)
    {
      sizeRed1 = maxSizeColors;
      sizeRed1Up = false;
    }
  }
  else
  {
    sizeRed1 -= changeSizeRed1;
    if (sizeRed1 < minSizeColors)
    {
      sizeRed1 = minSizeColors;
      sizeRed1Up = true;
    }
  }

  if (sizeRed2Up)
  {
    sizeRed2 += changeSizeRed2;
    if (sizeRed2 > maxSizeColors)
    {
      sizeRed2 = maxSizeColors;
      sizeRed2Up = false;
    }
  }
  else
  {
    sizeRed2 -= changeSizeRed2;
    if (sizeRed2 < minSizeColors)
    {
      sizeRed2 = minSizeColors;
      sizeRed2Up = true;
    }
  }

  // BLUE VARIABLES
  // BLUE CENTERS
  if (centerBlueWidthUp)
  {
    centerBlueWidth += changeCenterBlueWidth;
    if (centerBlueWidth > maxCenterWidth)
    {
      centerBlueWidth = maxCenterWidth;
      centerBlueWidthUp = false;
    }
  }
  else
  {
    centerBlueWidth -= changeCenterBlueWidth;
    if (centerBlueWidth < minCenterWidth)
    {
      centerBlueWidth = minCenterWidth;
      centerBlueWidthUp = true;
    }
  }

  if (centerBlueHeightUp)
  {
    centerBlueHeight += changeCenterBlueHeight;
    if (centerBlueHeight > maxCenterHeight)
    {
      centerBlueHeight = maxCenterHeight;
      centerBlueHeightUp = false;
    }
  }
  else
  {
    centerBlueHeight -= changeCenterBlueHeight;
    if (centerBlueHeight < minCenterHeight)
    {
      centerBlueHeight = minCenterHeight;
      centerBlueHeightUp = true;
    }
  }

  // BLUE SPEED
  if (speedBlue1Up)
  {
    speedBlue1 += changeSpeedBlue1;
    if (speedBlue1 > maxSpeedColors)
    {
      speedBlue1 = maxSpeedColors;
      speedBlue1Up = false;
    }
  }
  else
  {
    speedBlue1 -= changeSpeedBlue1;
    if (speedBlue1 < minSpeedColors)
    {
      speedBlue1 = minSpeedColors;
      speedBlue1Up = true;
    }
  }

  if (speedBlue2Up)
  {
    speedBlue2 += changeSpeedBlue2;
    if (speedBlue2 > maxSpeedColors)
    {
      speedBlue2 = maxSpeedColors;
      speedBlue2Up = false;
    }
  }
  else
  {
    speedBlue2 -= changeSpeedBlue2;
    if (speedBlue2 < minSpeedColors)
    {
      speedBlue2 = minSpeedColors;
      speedBlue2Up = true;
    }
  }

  // BLUE SIZE
  if (sizeBlue1Up)
  {
    sizeBlue1 += changeSizeBlue1;
    if (sizeBlue1 > maxSizeColors)
    {
      sizeBlue1 = maxSizeColors;
      sizeBlue1Up = false;
    }
  }
  else
  {
    sizeBlue1 -= changeSizeBlue1;
    if (sizeBlue1 < minSizeColors)
    {
      sizeBlue1 = minSizeColors;
      sizeBlue1Up = true;
    }
  }

  if (sizeBlue2Up)
  {
    sizeBlue2 += changeSizeBlue2;
    if (sizeBlue2 > maxSizeColors)
    {
      sizeBlue2 = maxSizeColors;
      sizeBlue2Up = false;
    }
  }
  else
  {
    sizeBlue2 -= changeSizeBlue2;
    if (sizeBlue2 < minSizeColors)
    {
      sizeBlue2 = minSizeColors;
      sizeBlue2Up = true;
    }
  }

  // GREEN VARIABLES
  // GREEN CENTERS
  if (centerGreenWidthUp)
  {
    centerGreenWidth += changeCenterGreenWidth;
    if (centerGreenWidth > maxCenterWidth)
    {
      centerGreenWidth = maxCenterWidth;
      centerGreenWidthUp = false;
    }
  }
  else
  {
    centerGreenWidth -= changeCenterGreenWidth;
    if (centerGreenWidth < minCenterWidth)
    {
      centerGreenWidth = minCenterWidth;
      centerGreenWidthUp = true;
    }
  }

  if (centerGreenHeightUp)
  {
    centerGreenHeight += changeCenterGreenHeight;
    if (centerGreenHeight > maxCenterHeight)
    {
      centerGreenHeight = maxCenterHeight;
      centerGreenHeightUp = false;
    }
  }
  else
  {
    centerGreenHeight -= changeCenterGreenHeight;
    if (centerGreenHeight < minCenterHeight)
    {
      centerGreenHeight = minCenterHeight;
      centerGreenHeightUp = true;
    }
  }

  // GREEN SPEED
  if (speedGreen1Up)
  {
    speedGreen1 += changeSpeedGreen1;
    if (speedGreen1 > maxSpeedColors)
    {
      speedGreen1 = maxSpeedColors;
      speedGreen1Up = false;
    }
  }
  else
  {
    speedGreen1 -= changeSpeedGreen1;
    if (speedGreen1 < minSpeedColors)
    {
      speedGreen1 = minSpeedColors;
      speedGreen1Up = true;
    }
  }

  if (speedGreen2Up)
  {
    speedGreen2 += changeSpeedGreen2;
    if (speedGreen2 > maxSpeedColors)
    {
      speedGreen2 = maxSpeedColors;
      speedGreen2Up = false;
    }
  }
  else
  {
    speedGreen2 -= changeSpeedGreen2;
    if (speedGreen2 < minSpeedColors)
    {
      speedGreen2 = minSpeedColors;
      speedGreen2Up = true;
    }
  }

  // GREEN SIZE
  if (sizeGreen1Up)
  {
    sizeGreen1 += changeSizeGreen1;
    if (sizeGreen1 > maxSizeColors)
    {
      sizeGreen1 = maxSizeColors;
      sizeGreen1Up = false;
    }
  }
  else
  {
    sizeGreen1 -= changeSizeGreen1;
    if (sizeGreen1 < minSizeColors)
    {
      sizeGreen1 = minSizeColors;
      sizeGreen1Up = true;
    }
  }

  if (sizeGreen2Up)
  {
    sizeGreen2 += changeSizeGreen2;
    if (sizeGreen2 > maxSizeColors)
    {
      sizeGreen2 = maxSizeColors;
      sizeGreen2Up = false;
    }
  }
  else
  {
    sizeGreen2 -= changeSizeGreen2;
    if (sizeGreen2 < minSizeColors)
    {
      sizeGreen2 = minSizeColors;
      sizeGreen2Up = true;
    }
  }


}

void Fireplace ()
{
  static unsigned int spark[Width]; // base heat
  CRGB stack[Width][Height];        // stacks that are cooler

  // 1. Generate sparks to re-heat
  for ( int i = 0; i < Width; i++) {
    if (spark[i] < HOT ) {
      int base = HOT * 2;
      spark[i] = random16( base, MAXHOT );
    }
  }

  // 2. Cool all the sparks
  for ( int i = 0; i < Width; i++) {
    spark[i] = qsub8( spark[i], random8(0, COOLING) );
  }

  // 3. Build the stack
  /*    This works on the idea that pixels are "cooler"
        as they get further from the spark at the bottom */
  for ( int i = 0; i < Width; i++) {
    unsigned int heat = constrain(spark[i], HOT / 2, MAXHOT);
    for ( int j = Height - 1; j >= 0; j--) {
      /* Calculate the color on the palette from how hot this
         pixel is */
      byte index = constrain(heat, 0, HOT);
      stack[i][j] = ColorFromPalette( gPal, index );

      /* The next higher pixel will be "cooler", so calculate
         the drop */
      unsigned int drop = random8(0, HOT);
      if (drop > heat) heat = 0; // avoid wrap-arounds from going "negative"
      else heat -= drop;

      heat = constrain(heat, 0, MAXHOT);
    }
  }

  // 4. map stacks to led array
  for ( int i = 0; i < Width; i++)
  {
    for ( int j = 0; j < Height; j++)
    {
      leds[XY(i, j)] = stack[i][j];

    }
  }

}










// Helper function that translates from x, y into an index into the LED array
uint16_t XY( uint8_t x, uint8_t y)
{
  uint16_t ledNum;
  if ( x & 0x01)
  {
    // Odd rows run backwards
    ledNum = ((x + 1) * Height) - (y + 1);
  }
  else
  {
    // Even rows run forwards
    ledNum = ((x * Height) + y);
  }

  return ledNum;
}


void checkInput()
{
  if (changeInput)
  {
    changeInput = false;
    if (teensyOn)
    {
      digitalWrite(auxInputB, HIGH); // when this is set HIGH it latches to the Aux Input
      delay(50);
      digitalWrite(auxInputB, LOW); // when this is set HIGH it latches to the Aux Input
      teensyOn = false;
    }
    else
    {
      digitalWrite(auxInputA, HIGH); // when this is set HIGH it latches to the Teensy
      delay(50);
      digitalWrite(auxInputA, LOW); // when this is set HIGH it latches to the teensy
      teensyOn = true;
    }
  }
}

void checkBrightness()
{
  if (digitalRead(radioButtonPin_C) == HIGH)
  {
    if (timeButtonC > 100)
    {
      timeButtonC = 0;
      BRIGHTNESS += 10;

      if (BRIGHTNESS > 253)
      {
        BRIGHTNESS = 253;
      }
      FastLED.setBrightness(BRIGHTNESS);
    }
  }

  if (digitalRead(radioButtonPin_D) == HIGH)
  {
    if (timeButtonD > 100)
    {
      timeButtonD = 0;
      BRIGHTNESS -= 10;

      if (BRIGHTNESS < 10)
      {
        BRIGHTNESS = 10;
      }
      FastLED.setBrightness(BRIGHTNESS);
    }
  }
}



void buttonPressed_A ()
{
  if (timeButtonA > 1000)
  {
    changeInput = true;
    timeButtonA = 0;
  }
}

void buttonPressed_B ()
{
  if (timeButtonB > 1000)
  {
    if (sinusoidOn == true)
    {
      sinusoidOn = false;
    }
    else
    {
      sinusoidOn = true;
    }
    timeButtonB = 0;
  }
}



