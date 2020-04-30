/**
 * ** Cosmac Elf control firmware **
 * The goal of this program is to allow an AVR to replace the debounce IC's and
 * glue logic in Cosmac Elf. However, it doesn't replace the hardware switches,
 * RAM, the 4016 bilateral switch, or the buffer latching for the display.
 *
 * I wrote this because I read about the Elf when it was first published, but
 * didn't have the skills to build one. While I could have built one since then,
 * not understanding how it worked would undermine the value of building it.
 * By controlling the 1802 with a program, I truly understand the schematic.
 *
 * Copyright (c) 2017 by Martin Heermance
 * MIT Licensed
 **/
 
// Output pins to control ELF.
const int clockPin = 15;
const int clrPin = 14;
const int ef4Pin = 13;
const int waitPin = 12;
const int displayLatchPin = 11;
const int inputLatchPin = 10;
const int dmaInPin = 9;

// Input pins that determine machine state.
const int inPin = 8;
const int loadPin = 7;
const int mreadPin = 6;
const int n2Pin = 5;
const int runPin = 4;
const int sc1Pin = 3;
const int tpbPin = 2;

void setup()
{
  pinMode(clockPin, OUTPUT);
  pinMode(clrPin, OUTPUT);
  pinMode(dmaInPin, OUTPUT);
  pinMode(waitPin, OUTPUT);
  pinMode(displayLatchPin, OUTPUT);
  pinMode(inputLatchPin, OUTPUT);

  pinMode(inPin, INPUT);
  pinMode(loadPin, INPUT);
  pinMode(mreadPin, INPUT);
  pinMode(n2Pin, INPUT);
  pinMode(runPin, INPUT);
  pinMode(sc1Pin, INPUT);
  pinMode(tpbPin, INPUT);
}

void loop()
{
  // set clock high
  digitalWrite(clockPin, HIGH);

  // Read all the input state.
  int notIn = !debouncedDigitalRead(inPin);
  int load = debouncedDigitalRead(loadPin);
  int notMread = digitalRead(mreadPin);
  int n2OrLoad = digitalRead(n2Pin) || load;
  int run = debouncedDigitalRead(runPin);
  int sc1OrNotLoad = digitalRead(sc1Pin) || !load;
  int tpb = digitalRead(tpbPin);

  // Compute flip flip state.
  int notQ = flipFlop(notIn, sc1OrNotLoad);

  // Set the output pins.
  digitalWrite(clrPin, run);
  digitalWrite(displayLatchPin, !(!notMread && tpb && n2OrLoad) );
  digitalWrite(dmaInPin, notQ);
  digitalWrite(ef4Pin, load);
  digitalWrite(inputLatchPin, notMread && n2OrLoad);
  digitalWrite(waitPin, !load);

  // Set clock low
  digitalWrite(clockPin, LOW);
}

int flipFlop(int notIn, int sc1OrNotLoad)
{
  static int priorNotIn = HIGH;

  // The DMA In pin is controlled by a D flip flip and is the
  // most complicated. D is tied high, while S is low. Only clk
  // and reset change value, with !Q used to set DMA In.
  int notQ = HIGH;

  // First, detect a notIn edge trigger on clk.
  if (priorNotIn == LOW && notIn != LOW)
  {
    // On clock rising edge Q is set to the value of D, while
    // notQ is set inverted. Since D is HIGH, notQ is LOW.
    notQ = LOW;
    priorNotIn = notIn;
  }

  // Now check the reset condition of the D flip flop.
  if (sc1OrNotLoad)
  {
    // On reset Q is set to LOW and notQ is set HIGH.
    notQ = HIGH;
  }

  return notQ;
}

int debouncedDigitalRead(int pin) {
  // the debounce time; increase if weirdness happens.
  const int debounceDelay = 50;

  // Do an intial read of button state.
  int lastButtonState = digitalRead(pin);

  // Wait the standard delay.
  delay(debounceDelay);

  // read the state of the button again.
  int reading = digitalRead(pin);

  // Wait until the reading stabilizes.
  while(reading != lastButtonState) {
    lastButtonState = reading;

    // read the state of the switch again.
    reading = digitalRead(pin);

    // Wait the standard delay.
    delay(debounceDelay);
  }

  return reading;
}

