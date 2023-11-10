/****************************************************************************
  Button.h - simple class for debouncing a pushbutton switch, with
             detection of short and long presses.
  ***************************************************************************/
class Button {
  static const int debounceTime = 100;
  static const int longPressTime = 2000;
public:
  typedef enum { LONGPRESS, SHORTPRESS, NONE } Event;
  Button(int p) : pin(p), lastState(HIGH), longDetected(false) {
    pinMode(p, INPUT_PULLUP);
  }

  Event event() {
    Event result = NONE;
    currentState = digitalRead(pin);
    if (lastState == HIGH && currentState == LOW) {     // pressed
      pressedTime = millis();
      longDetected = false;
    }
    else if (lastState == LOW && currentState == HIGH && !longDetected) {  // released
      releasedTime = millis();
      unsigned long elapsed = releasedTime - pressedTime;
      if (elapsed > debounceTime)
        result = SHORTPRESS;
    }
    else if (lastState == LOW && currentState == LOW && !longDetected) {    // held
      unsigned long elapsed = millis() - pressedTime;
      if (elapsed >= longPressTime) {
        result = LONGPRESS;
        longDetected = true;
      }
    }
    lastState = currentState;
    return result;
  }
  private:
  int pin;
  int currentState;
  int lastState;
  bool longDetected;
  unsigned long pressedTime;
  unsigned long releasedTime;
};
