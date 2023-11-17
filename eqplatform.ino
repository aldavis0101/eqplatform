/****************************************************************************
  EQPlatform.ino - Arduino Sketch for Equatorial Platform Drive
   
  This sketch controls the stepper motor used to drive the platform
  and provides a user interface for the drive. The drive system has the 
  following elements:
  Hardware:
    - Stepper motor. We use a small NEMA 11 motor.
    - Mechanical advantage, from gears and the platform itself. The total
      advantage is the number of motor revolutions for a hypothetical
      full rotation of the platform around its polar axis.
    - Stepper motor driver - a circuit that pulses current to spin the 
      motor. We use a TMC2209 module from Trinamic.
    - Arduino Nano, for controlling the driver and handling user controls.
    - Speed controls. This allows the drive speed to be fine-tuned to match
      the sidereal rate, or to center objects in the eyepiece.
    - OLED display - shows current speed and position.
    Software:
    - TMCStepper library, for controlling the TMC2209
    - FastAccelStepper library, for sending timed step pulses
    - "SSEncoder" class - uses rotary encoder for persistent speed adjustment
    - "QPot" class - uses a joystick to set momentary speed adjustment
    - "Button" class - simple class to detect pushbutton switches, for pause/reset
    - "OLED_Display" - manages the OLED display console
  General notes:
    - This is a microcontroller application after all, so we try to keep 
      it lighweight and fast. In particular, we try to avoid floating point,
      and keep expensive operations like formatting output to a minimum.  
*****************************************************************************/
#include "SSEncoder.h"
#include "QPot.h"
#include "Button.h"
#include "OLED_Display.h"
#include <TMCStepper.h>
#include <FastAccelStepper.h>
#include <SoftwareSerial.h>

// Baseline motor speed and microstep frequency.
// Determined from platform geometry and mechanical advantage -- see spreadsheet.
// FastAccelStepper represents speed in milliHz (mHz); that is, microsteps/sec x 1000.
// This gives good resolution without using floating point.
// All the calculations below happen at compile time.
static const unsigned siderealMins = 1436;
static const double platformRatio = 4605.0;   // total mech advantage, motor axis to platform axis
static const double calibrationFactor = 0.97;  // empirically derived from field testing
static const double baselineRPM = (platformRatio * calibrationFactor) / siderealMins;
static const unsigned steps = 200;
static const unsigned microsteps = 16;
static const uint32_t baseline_mHz = (baselineRPM * steps * microsteps * 1000) / 60;
static const int trackingTimeMins = 60;
//static const int trackingTimeMins = 1;     // for testing
static unsigned long microstepLimit = (trackingTimeMins * baseline_mHz * 60) / 1000;

static const int RMSSetting = 400;         // Nema 11 670ma rated
//static const int RMSSetting = 800;         // Nema 17 1500ma rated

// Serial communication with TMC2209 driver via UART
// Nano has only 1 HW UART, used for PC communication, so we need SW UART
// Note: only the TX pin needs to be hooked up
#define swRXpin 8
#define swTXpin 7
SoftwareSerial driver_uart(swRXpin, swTXpin);

// TMC2209 Stepper Driver control
#define RSENSE 0.11f
#define CSADDR 0b00
TMC2209Stepper driver(&driver_uart, RSENSE, CSADDR);

// FastAccelStepper - Library-based class for stepper pulse generation
#define stepPin 9
#define dirPin 4
#define enablePin 11

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

// Controllable Speed adjustment - represented as a linear scale factor 
// where 1000 represents 1.0.
// Current speed = baseline speed * (adjustment/1000)
static const unsigned int initSpeedFactor = 1000;
unsigned int currentSpeedFactor = initSpeedFactor;
bool warpMode = false;     // 10x
bool pauseMode = true;

// Sense pin - senses whether hand controller is connected or not
#define sensePin 14

// PSC - Persistent Speed Control - lasting speed adjustment using rotary encoder.
// This allows sidereal tracking rate to be fine-tuned according to temperature or
// other factors. Also allows adjustment for lunar tracking, which is approximately 
// 3% slower than the astral rate.
#define encoderClkPin 2
#define encoderDTPin 3

SSEncoder PSCKnob(encoderClkPin, encoderDTPin, initSpeedFactor);

// Encoder button. Short press = pause; long press = position reset
#define PSCButtonPin 15
Button PSCButton(PSCButtonPin);

// MSC - Momentary speed control using joystick potentiometer.
// Allows tracking to me momentarily sped up or slowed down in order
// to center objects in the eyepiece.
// The pot is not very granular, so we just quantize it as a value from 
// [-2. +2], and adjust the speed as a power of 2 (from 1/4x to 4x).
#define MSCPotPin 3
QPot MSCPot(MSCPotPin, sensePin);

// Joystick button. Long press = "warp mode" -- 10x faster, for demo/testing
#define warpModePin 16
Button MSCButton(warpModePin);

// OLED display for current speed and position. The display class uses callback
// functions to retrieve current values.
unsigned getSpeedFactor() {     // current speed factor
  return currentSpeedFactor;
}

uint32_t getSpeedmRPM() {       // current speed in RPMx1000
  if (!stepper) return 0;
  return (stepper->getCurrentSpeedInMilliHz() * 60) / (steps * microsteps);
}

uint32_t getPositionArcsec() {  // current position in arcsrc
  if (!stepper) return 0;
  uint32_t pos = stepper->getCurrentPosition();
  return ((uint64_t)pos * 360 * 60 * 60) / (uint32_t)(steps * microsteps * platformRatio);
}

Display OLEDConsole(getSpeedFactor, getSpeedmRPM, getPositionArcsec);

// Forward declarations
void updateStepper();
void resetPlatform();

/*-----------------------------------------------------------------------------
// Setup
-----------------------------------------------------------------------------*/
void setup() {
  pinMode(sensePin, INPUT_PULLUP);


  // Serial console on PC, via USB
  Serial.begin(115200);
  while (!Serial)
    ;
  Serial.println(F("Serial initialized -----------------------------"));
  
  // UART interface to TMC2209. Max speed of SW UART is ~38400
  driver_uart.begin(19200);
  while (!driver_uart)
    ;
  Serial.println(F("UART initialized"));

  Serial.print(F("baseline RPM: ")); Serial.println(baselineRPM);
  Serial.print(F("baseline mHz: ")); Serial.println(baseline_mHz);

  Serial.print(F("Controller connected: "));
  Serial.println(MSCPot.connected());
  
  // Initialize the controls to establish initial values
  PSCKnob.read();
  Serial.print(F("Init PSC: ")); Serial.println(PSCKnob.value());
  MSCPot.read();
  Serial.print(F("Init MSC: ")); Serial.println(MSCPot.scale());

  // Shout out to Kyle
  OLEDConsole.init();
  OLEDConsole.status("KWD 08.03.98");
  
  // Wait a moment before sending commands to allow the TMC and UART
  // to initialize (may not need?)
  delay(1000);
  
  // Configure the TMC2209 stepper driver
  driver.begin();
  driver.toff(5);
  driver.rms_current(RMSSetting);    

  driver.microsteps(microsteps);
  driver.pwm_autoscale(true);     // Needed for stealthChop
  driver.en_spreadCycle(false);   // false = StealthChop / true = SpreadCycle

  // Make sure UART communication with TMC2209 is established. This
  // can be flaky especially when the 2209 is powered up separately from
  // the Arduino (e.g. when Arduino is powered by USB). 
  uint16_t msread=driver.microsteps();
  Serial.print(F("Read microsteps via UART: "));    
  Serial.println(msread);
  uint8_t result = driver.test_connection();
  Serial.print(F("Connection test: ")); Serial.println(result);
  if (result != 0 || msread != microsteps) {
     OLEDConsole.status("UART init error");
     delay(1000);
     abort();
  }  
  
  // Initialize the FastAccelStepper pulse controller.
  engine.init();
  stepper = engine.stepperConnectToPin(stepPin);
  if (!stepper) {
    Serial.println(F("FastAccelStepper init error"));
    OLEDConsole.status("Stepper init error");
    delay(1000);
    abort();
  }
  
  Serial.println(F("Stepper initialized"));      
  stepper->setDirectionPin(dirPin, false);   // false: HI = CCW 
  stepper->setEnablePin(enablePin, true);    // true: LO = enable
  stepper->setAutoEnable(false);
  stepper->setAcceleration(1000);
  // If auto enable/disable need delays, just add (one or both):
  //stepper->setDelayToEnable(50);
  //stepper->setDelayToDisable(1000);
  
  // Start the drive
  Serial.println(F("Starting..."));
  OLEDConsole.clearStatus();
  resetPlatform();
  OLEDConsole.display();
}

/*-----------------------------------------------------------------------------
  Loop
-----------------------------------------------------------------------------*/
void loop() {
  // Read speed controls and adjust speed factor, used to fine-tune the 
  // tracking rate, represented as a linear factor scaled by 1000. 
  //    speed = baseline speed * (speed factor/1000)
  // The PSC (Persistent Speed Control) is a rotary encoder that gives the 
  // baseline speed factor.
  currentSpeedFactor = PSCKnob.read();
  
  // The MSC (Momentary Speed Control) is a potentiometer (joystick) that returns
  // a value between -2 and +2, representing a factor from 1/4x to 4x.
  int pot = MSCPot.read();
  if      (pot < 0)  currentSpeedFactor >>= -pot;
  else if (pot > 0)  currentSpeedFactor <<= pot;

  // Warp mode speeds up by 10x, for demo and testing
  if (MSCButton.event() == Button::LONGPRESS) warpMode = !warpMode;
  if (warpMode) currentSpeedFactor *= 10;
  
  // A short press of the encoder button pauses the drive. A long press 
  // resets the computed position and restarts the drive, after the user 
  // manually moves the platform to the start position.
  switch (PSCButton.event()) {
    case Button::SHORTPRESS:
      pauseMode = !pauseMode;
      break;
    case Button::LONGPRESS:
      resetPlatform();
      break;
  }

  // Apply any changes and update the display
  updateStepper();
  OLEDConsole.display();

  delay(100);
}

/*-----------------------------------------------------------------------------
  updateStepper()
    Apply any control changes (speed, start, stop), and run drive 
-----------------------------------------------------------------------------*/
void updateStepper() {
  if (!stepper) return;

  // Compute adjusted target speed. FastAccelStepper represents speed as
  // "milliHz", that is Hz * 1000.
  uint32_t new_mHz = ((uint64_t)baseline_mHz * currentSpeedFactor) / 1000;

  // Retrieve current speed setting (may not be actual speed if accelerating)
  uint32_t current_mHz = stepper->getSpeedInMilliHz();
  static bool limitReported = false;

  // If we reached the tracking limit, stop
  if (stepper->getCurrentPosition() == microstepLimit) {
    if (!limitReported) {
      Serial.println(F("Limit reached, stopping"));
      OLEDConsole.status("Platform Limit");
    }
    limitReported = true;
  }
  // Pause via user control
  else if (pauseMode || new_mHz == 0) {
    if (stepper->isRunning()) {
      stepper->stopMove();
      //stepper->disableOutputs();
      Serial.println(F("Pausing"));
      OLEDConsole.status("Paused");
    }
  }
  // Restart or change speeds
  else if (!stepper->isRunning() || new_mHz != current_mHz) {
    //Serial.print("set speed "); Serial.print(current_mHz); Serial.print(" -> "); Serial.println(new_mHz);
    stepper->setSpeedInMilliHz(new_mHz);
    if (stepper->isRunning())
      stepper->applySpeedAcceleration();
    else {
      stepper->enableOutputs();
      stepper->moveTo(microstepLimit);
      Serial.print(F("restarting, speed is ")); 
      Serial.println(stepper->getSpeedInMilliHz());
      OLEDConsole.status("Running");
      limitReported = false;
    }
  }
}

/*-----------------------------------------------------------------------------
  resetPlatform()
    Reset platform position to starting position, and start drive
-----------------------------------------------------------------------------*/
void resetPlatform() {
  if (!stepper) return;
  stepper->setCurrentPosition(0);
  pauseMode = false;
  updateStepper();
}
