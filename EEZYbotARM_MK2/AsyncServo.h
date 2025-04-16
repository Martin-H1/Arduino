#include <Servo.h>

// Create a servo sub class that moves the servo asynchronously
// via polling from the main loop function. It uses pulse width
// as the unit of angular measure to allow for integer arithmetic.
class AsyncServo : public Servo
{
  protected:
    const uint16_t oneDegreeMS = 6;    // One degree is the increment or decrement amount.
    uint16_t minMS;                    // servo constraint data.
    uint16_t midMS;
    uint16_t maxMS;

    uint16_t target = 0;               // target angle in microseconds.
    uint16_t current = 0;              // current angle in microseconds.
    uint16_t startAngle = 0;           // what was the start angle in microseconds.

    const uint8_t startInterval = 10;  // slow speed for start
    uint8_t interval = startInterval;  // delay time
    uint32_t previousMillis = 0;       // last movement time.
    uint16_t minInterval = 5;          // interval after rampUp but before ramp down (=max Speed)
    uint8_t rampUp;                    // angle used for ramp up trigger.
    uint8_t rampDown;                  // angle used for ramp down trigger.

  public:
    /* Binds the servo to a pin and sets up limits of movement.
    */
    void init(uint8_t pin, uint16_t min, uint16_t mid, uint16_t max)
    {
      this->attach(pin);
      minMS = min;
      midMS = mid;
      maxMS = max;
      rampUp = (mid - min) / 3;
      rampDown = (mid - min) / 9;
    }

    /* Synchronously moves the servo to the home position.
     * This allows for a known starting point for timed movement.
     */
    void home()
    {
      this->writeMicroseconds(midMS);
      this->current = midMS;
      this->target = midMS;
    }

    /* Sets the destination and movement duration.
     */
    void setTarget(uint16_t target, uint16_t duration)
    {
      // convert duration into min interval
      this->target = constrain(target, minMS, maxMS);
      this->minInterval = 1;
      startAngle = current;
      interval = startInterval;
    }

    // Update is called in the loop function to iteratively move the servo into position.
    // Uses trapizoidal ramping to move servo smoothly.
    //      ____
    // ____/    \____
    //     u    d
    void update()
    {
      // Nothing to do if the servo is at the target position.
      if (current == target)
        return;

      uint32_t currentMillis = millis();

      // The servo requires update, but only when the interval is exceeded.
      // The interval contracts, remains constant, then contracts as the servo ramps.
      if (currentMillis - previousMillis > interval)
      {
        previousMillis = currentMillis;
        int16_t total = abs(startAngle - target);
        int16_t remaining = abs(current - target);

        if (remaining < rampDown)
        {
          DPRINTLNF("ramp down");
          if (interval < startInterval)
            interval++;
        }
        else if (remaining > rampUp)
        {
          DPRINTLNF("ramp Up");
          if (interval > minInterval)
            interval--;
        }
        DPRINT("interval="); DPRINTLN(interval);
        if (target < current)
        {
          current -= oneDegreeMS;
        }
        else if (target > current)
        {
          current += oneDegreeMS;
        }
        this->writeMicroseconds(current);
      }
    }
};