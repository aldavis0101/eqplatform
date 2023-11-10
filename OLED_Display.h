/****************************************************************************
 OLED_Display.h
   Manager for display console of tracking platform.
   The display is a 1.3" 128x64 OLED with a SH1106 controller.
   We use the "ss_oled" library which is very lightweight compared to 
   the Adafruit library (with the Adafruit library this sketch is
   too bug for a Nano). 
   The library manages the display as having 8 rows of text, each 8 pixels
   high. There is a double-high 12x16 font which takes 2 rows. Our display
   layout is as follows:
      line 0   8x8    status
      line 2-3 12x16  current speed adjustment
      line 4-5 12x16  current motor speed
      line 6-7 12x16  current position 
 ***************************************************************************/
#include <ss_oled.h>

// OLED configuration
// if your system doesn't have enough RAM for a back buffer, comment out
// this line (e.g. ATtiny85)
//#define USE_BACKBUFFER

#ifdef USE_BACKBUFFER
static uint8_t ucBackBuffer[1024];
#else
static uint8_t *ucBackBuffer = NULL;
#endif

// Use -1 for the Wire library default pins
// or specify the pin numbers to use with the Wire library or bit banging on any GPIO pins
#define SDA_PIN 18
#define SCL_PIN 19
// Set this to -1 to disable or the GPIO pin number connected to the reset
// line of your display if it requires an external reset
#define RESET_PIN -1
// let ss_oled figure out the display address
#define OLED_ADDR -1
// don't rotate the display
#define FLIP180 0
// don't invert the display
#define INVERT 0
// Bit-Bang the I2C bus
#define USE_HW_I2C 0

// Change these if you're using a different OLED display
#define MY_OLED OLED_128x64
#define OLED_WIDTH 128
#define OLED_HEIGHT 64

/* Uncomment the initialize the I2C address , uncomment only one, If you get a totally blank screen try the other*/
#define i2c_Address 0x3c //initialize with the I2C addr 0x3C Typically eBay OLED's
//#define i2c_Address 0x3d //initialize with the I2C addr 0x3D Typically Adafruit OLED's

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1   //   QT-PY / XIAO
//Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

/*----------------------------------------------------------------------------
  Display class
  ----------------------------------------------------------------------------*/
class Display {
  public:
  #if 0
  Display(FastAccelStepper* const &stepper, const unsigned &speedFactor, 
     float mechAdvantage, unsigned microstepsPerRev)  : 
     stepper(stepper), speedFactor(speedFactor),
     mechAdvantage(mechAdvantage), steps(microstepsPerRev), 
     displayedSpeedFactor(0), displayedSpeedmHz(0), displayedPosition(0),
     lastDisplayTime(0) {}
      #else
     Display(unsigned (*getSpeedFactor)(), 
             uint32_t (*getSpeedmRPM)(),
             uint32_t (*getPositionArcsec)()) : 
             getSpeedFactor(getSpeedFactor), getSpeedmRPM(getSpeedmRPM), getPositionArcsec(getPositionArcsec),
             displayedSpeedFactor(0), displayedSpeed(0), displayedPosition(0),
             lastDisplayTime(0) {}
      #endif 
  
  void init() {
    int rc = oledInit(&oled, MY_OLED, OLED_ADDR, FLIP180, INVERT, 
                     USE_HW_I2C, SDA_PIN, SCL_PIN, RESET_PIN, 400000L); 
                     // use standard I2C bus at 400Khz
    // Nano lacks RAM for a backBuffer
    //oledSetBackBuffer(&oled, ucBackBuffer);

    // brightness value for night viewing
    oledSetContrast(&oled, 1);   // 0-255
    oledSetTextWrap(&oled, 0);

    oledFill(&oled, 0x0, 1);
  }

  // Called by each iteration of main loop(), to update display
  void display() {
    // only update each .25 second
    unsigned long currentTime = millis();
    if (currentTime - lastDisplayTime < 250) return;

    // Display the current speed ajustment, in percent. 1000 = 100%
    uint32_t speedFactor = getSpeedFactor();
    if (speedFactor != displayedSpeedFactor)
    {
      char buffer[9], *p = buffer; 
      itoa(speedFactor/10, p, 10);  p += strlen(p);
      *p++ = '.';
      itoa(speedFactor % 10, p, 10);  p += strlen(p); 

      oledSetCursor(&oled, 0, 2);
      oledWriteString(&oled, 0, -1, -1, buffer, FONT_12x16, 0, 1);
      superScript("%");
      clearToEOL();      
      displayedSpeedFactor = speedFactor;
    }

    // Display the motor speed. Callback returns speed in RPM/1000
    uint32_t mRPM = getSpeedmRPM();
    if (mRPM != displayedSpeed) {
      char buffer[10], *p = buffer;

      // Format the integral part, then 3 digits after the decimal
      ultoa(mRPM / 1000, buffer, 10);   p += strlen(buffer);
      *p++ = '.';
      *p++ = '0' + ((mRPM % 1000) / 100);
      *p++ = '0' + ((mRPM % 100) / 10);
      *p++ = '0' + (mRPM % 10);
      *p++ = '\0';
      // Chop to 4 siginifcant digits
      if (buffer[4] == '.') buffer[4] = '\0';
      else buffer[5] = '\0';
      
      oledSetCursor(&oled, 0, 4);
      oledWriteString(&oled, 0,  -1, -1, buffer, FONT_12x16, 0, 1);
      superScript("RPM");
      clearToEOL();
      displayedSpeed = mRPM;
    }
    
    // Display the platform position since reset. Callback returns position
    // in arcseconds.

    uint32_t arcsec = getPositionArcsec();
    if (arcsec != displayedPosition) {
      //uint32_t arcSec = (uint32_t)(pos * arcSecsPerMicrostep);
      unsigned deg = arcsec / (60 * 60);
      unsigned min = (arcsec - ((uint32_t)deg * 60 * 60)) / 60;
      unsigned sec = (arcsec % 60);
      char buffer[12];
    
      oledSetCursor(&oled, 0, 6);
      itoa(deg, buffer, 10);
      oledWriteString(&oled, 0,  -1, -1, buffer, FONT_12x16, 0, 1);
      // Display non-ASCII degree symbol as a bitmap
      degreeSymbol12x16();

      itoa(min, buffer, 10);
      strcat(buffer, "'");
      itoa(sec, buffer + strlen(buffer), 10);
      strcat(buffer, "\"");

      oledWriteString(&oled, 0,  -1, -1, buffer, FONT_12x16, 0, 1);
      clearToEOL();
      displayedPosition = arcsec;
    }  
    lastDisplayTime = currentTime;
  }
  
  // Display a status message on the top line of the console.
  void status(const char* msg) {
    oledSetCursor(&oled, 0, 0);
    oledWriteString(&oled, 0,-1,-1, (char *)msg, FONT_NORMAL, 0, 1);
    clearToEOL();  
  }
  
  // Clear the console message
  void clearStatus() {
    status("");
  }
  
  // Erase the current line from the cursor position to the right margin.
  void clearToEOL() {
    static const int charPixelWidth = 16;   // pixel width of FONT_16x16
    static const int charsInLine = (128/charPixelWidth) + 1;
    // We just write a whole line worth of blanks. With line wrapping
    // disabled, the library will just chop the excess.
    for (int i = 0; i < charsInLine; ++i)
      oledWriteString(&oled, 0, -1, -1, (char *)" ", FONT_16x16, 0, 1);
  }
  
  // Display a string as a superscript using the 8x8 font, within a double-
  // high line (12x16 or 16x16) of text.
  void superScript(const char *str) {
    int oldX = oled.iCursorX;
    int oldY = oled.iCursorY;
    // Write the superscipt string.
    oledWriteString(&oled, 0, -1, -1, (char *)str, FONT_NORMAL, 0, 1);
    // Since the superscript is only 8 pixels high, we need to clear the pixels
    // below it on the next line
    int newX = oled.iCursorX;
    int newY = oled.iCursorY;
    oledSetCursor(&oled, oldX, oldY + 1);
    for (int i = 0; i < strlen(str); ++i)
      oledWriteString(&oled, 0, -1, -1, (char *)" ", FONT_NORMAL, 0, 1);
    oledSetCursor(&oled, newX, newY);
  }
  
  // Display a degree symbol in 12x16 text (not part of the ASCII character set)
  void degreeSymbol12x16() {
    static const uint8_t bitmap[]PROGMEM = {
      0x00, 0x00,
      0B00001110, 0B00000000,
      0B00011111, 0B00000000,
      0B00011011, 0B00000000,
      0B00011111, 0B00000000,
      0B00001110, 0B00000000,
      0x00, 0x00,
      0x00, 0x00,
      0x00, 0x00,
      0x00, 0x00,
      0x00, 0x00,
      0x00, 0x00,
      0x00, 0x00,
      0x00, 0x00,
      0x00, 0x00,
      0x00, 0x00,
    };
    // Draws a 16x16 tile from the bitmap
    oledDrawTile(&oled, bitmap, oled.iCursorX, oled.iCursorY, 0, 0, 1);
    oled.iCursorX += 12;
  }

  private:
  SSOLED oled;
  unsigned (*getSpeedFactor)();
  uint32_t (*getSpeedmRPM)();
  uint32_t (*getPositionArcsec)();
  unsigned displayedSpeedFactor;
  uint32_t displayedSpeed;
  int32_t displayedPosition;
  unsigned long lastDisplayTime;
};