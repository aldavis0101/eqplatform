/****************************************************************************
  SSEncoder - Speed-Sensitive Rotary Encoder
 ***************************************************************************/
//#define ENCODER_OPTIMIZE_INTERRUPTS
//#define ENCODER_DO_NOT_USE_INTERRUPTS
#include <Encoder.h>

/*-----------------------------------------------------------------------------
  DelayQueue - helper class for measuring encoder rotation speed.
  Implements a simple circular FIFO queue to store a history of N values.
-----------------------------------------------------------------------------*/
template <typename T, int N> class DelayQueue {
  public:
  DelayQueue() : head(0), tail(N-1) { for(int i=0; i<N; ++i) queue[i] = 0; }
  // push_back() - push a new value into the queue
  void push_back(T val) {
     if (++head == N) head = 0;
     if (++tail == N) tail = 0;
     queue[head] = val;
  }
  // front() - returns first (oldest) value
  T front() { return queue[head]; }
  private:
  T queue[N];
  int head;
  int tail;
};

/*-----------------------------------------------------------------------------
  SSEncoder - Speed-Sensitive Encoder.
  Implements two enhancements over the basic Encoder class.
  1. Scaling increment/decrement, so that each "click" adjusts by 1
     instead of the usual 4 provided by typical quadrature encoding
  2. Speed sensitivity, so that turning faster makes larger adjustments
-----------------------------------------------------------------------------*/
class SSEncoder {
  public:
  SSEncoder(int clkPin, int dirPin, int initialValue) : 
    encoder(clkPin, dirPin), 
    currentVal(initialValue), lastVal(currentVal), 
    currentPos(0xFFFFFFFF), lastPos(0), currentSpeed(0) {}
  
  // Poll the encoder and update values
  int read() { 
    // Record state from last call to read()
    lastPos = currentPos;
    lastVal = currentVal;
  
    // Get encoder position. If no movement, return
    currentPos = encoder.read();
    long diffPos = currentPos - lastPos;
    //Serial.print("SSEncoder: "); Serial.print(lastPos); Serial.print(" -> "); Serial.println(currentPos);
    if (lastPos == 0xFFFFFFFF || diffPos == 0) {
      currentSpeed = 0;
      return currentVal;
    }
    
    // Calculate speed, by measuring amount of position change over elapsed
    // time. Because one "click" of the encoder between the detents results
    // in a nearly instantateous 4-position delta, we amortize the changes
    // by recording timechecks in a 16-deep FIFO queue.
    unsigned long currentTime = millis();                      // time now
    unsigned long elapsed = currentTime - timeChecks.front();  // time at N changes ago
    timeChecks.push_back(currentTime); 
    if (elapsed == 0)  // avoid div/0
      currentSpeed = 999;
    else
      currentSpeed = (abs(diffPos) * 1000) / elapsed;        // approx clicks/sec

    // Adjust the represented value. Round to detent boundary: 4 positions == 1 value.
    // If "fast" movement, scale the adjustment.
    int factor = (currentSpeed > 200) ? 10 : 1;
    if (diffPos < 0)  // CW rotation
      currentVal = currentVal + ((-diffPos+2)/4) * factor;
    else  // CCW
      currentVal = currentVal - ((diffPos+2)/4) * factor;
    return currentVal;
  }
  bool changed() { return currentVal != lastVal; }
  long position() { return currentPos; }
  int speed() { return currentSpeed; }
  int value() { return currentVal; }
  
  private:
  Encoder encoder;                            // from Encoder library 
  DelayQueue<unsigned long, 16> timeChecks;   // FIFO queue of timecheck history
  long currentPos;                            // current position 
  long lastPos;                               // position at last read()
  int currentVal;                             // current result value
  int lastVal;                                // value at last read()
  int currentSpeed;                           // speed estimate
};