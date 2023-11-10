/****************************************************************************
  QPot.h - "Quantized Potentiometer"

  The cheap joystick we have does not have good granularity. So we "quantize"
  it by dividing the range into 5 positions from -2 to +2. This value
  is used to shift the speed adjustment, resulting in a scale factor
  from .25x (-2) to 4x (+2). 
***************************************************************************/
class QPot {
  static const int range = 1024;

public:
  QPot(int p, int s) : 
    pin(p), sensePin(s), 
    pinVal(0xFFFF), lastVal(pinVal), currentScale(0) {}
  // Read the potentiometer and update values
  int read() { 
    lastVal = pinVal;
    lastScale = currentScale;
    currentScale = 0;
    if (connected()) {
      pinVal = analogRead(pin);
      // Round the value and map it from 0..1023 
      if (pinVal != lastVal) {
          pinVal += range/8;
        // Then map the value into the range [-2, +2]. Negate so that "up" is faster
        currentScale =  -constrain(map(pinVal, 100, range-100, -2, +2), -2, +2);
      }
    }
    return currentScale;
  }
  // The sense pin is connected to ground via the hand controller. If the
  // controller is present, the pin will read low, or high if not. Sensing
  // the absense of the controller avoids bogus readings from the pot. 
  bool connected() { return digitalRead(sensePin) == LOW; }
  bool changed() { return currentScale != lastScale; }
  int scale() { return currentScale; }
  private:
  int pin;
  int sensePin; 
  int pinVal;
  int lastVal;
  int currentScale;
  int lastScale;
};
