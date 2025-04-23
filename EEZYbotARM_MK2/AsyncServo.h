/* AsyncServo.h
 * A servo sub class that moves the servo asynchronously via polling
 * from the main loop function. It uses pulse width microseconds as
  * the unit of angular measure to allow for integer arithmetic.
 */
 #include <Servo.h>

class AsyncServo : public Servo
{
  protected:
    uint16_t minBr;                    // Min joint movement in brads
    uint16_t maxBr;                    // Max joint movement in brads

    uint16_t minMS;                    // servo constraint data.
    uint16_t maxMS;
    uint16_t homeMS;

    uint16_t target = 0;               // target angle in microseconds.
    uint16_t current = 0;              // current angle in microseconds.
    uint16_t startAngle = 0;           // what was the start angle in microseconds.

    uint8_t minInterval = 5;
    uint8_t startInterval;             // initial update time.
    uint8_t interval;                  // current update time.

    uint32_t previousMillis = 0;       // last movement time.

    uint8_t rampUp;                    // angle used for ramp up trigger.
    uint8_t rampDown;                  // angle used for ramp down trigger.

  public:
    /* Binds the servo to a pin and sets up limits of movement.
    */
    void init(uint8_t pin, uint16_t min, uint16_t max, uint16_t minB, uint16_t maxB, uint16_t home)
    {
      this->attach(pin);
      minBr = minB;
      maxBr = maxB;
      minMS = min;
      maxMS = max;
      homeMS = home;
      rampUp = (max - min) / 3;
      rampDown = (max - min) / 9;

      // Constrain the home position to the physical range of motion configured.
      // Convert that value into servo pulse widths and constrain.
      homeMS = map(constrain(home, minBr, maxBr), minBr, maxBr, minMS, maxMS);
    }

    /* Synchronously moves the servo to the home position.
     * This allows for a known starting point for timed movement.
     */
    void home()
    {
      this->writeMicroseconds(homeMS);
      this->current = homeMS;
      this->target = homeMS;
    }

    /* Gets the target in brads.
     */
    uint16_t getTarget()
    {
      // Convert that value into servo pulse widths and constrain.
      return map(target, minMS, maxMS,  minBr, maxBr);
    }

    /* Sets the target (in brads) and movement duration (in ms).
     */
    uint16_t setTarget(uint16_t target, uint16_t duration)
    {
      // Constrain the allowed input to the physical range of motion configured.
      int16_t value = constrain(target, minBr, maxBr);

      // Convert that value into servo pulse widths and constrain.
      value = map(value, minBr, maxBr, minMS, maxMS);
      this->target = constrain(value, minMS, maxMS);

      startAngle = current;
      interval = startInterval;

      DPRINT("startInterval="); DPRINTLN(startInterval);
      return this->target;
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
          current--;
        }
        else if (target > current)
        {
          current++;
        }
        this->writeMicroseconds(current);
      }
    }
};
