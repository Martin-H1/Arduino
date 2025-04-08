/*
 * Reflects incoming serial data back to host PC without blocking.
 * Useful for connectivity testing scripts running on host PC.
 * Example of processing incoming serial data without blocking.
*/

// how much serial data we expect before a newline
const unsigned int BUFFER_SIZE = 80;
char buffer[BUFFER_SIZE];
unsigned int buffIdx = 0;

void setup() {
  Serial.begin (9600);
  Serial.println("serial setup complete\n");
}

void loop() {
  // if serial data available, process it
  if (Serial.available () > 0) {
    byte inByte = Serial.read();

    switch (inByte)
    {
      case '\n':   // end of line
        buffer[buffIdx] = 0;    // terminating null byte
        buffIdx = 0;            // reset buffer for next time
        Serial.println(buffer); // Reflect data a line at a time.
        break;

      case '\r':   // discard carriage return
        break;

      default:
        // keep adding if not full ... allow for terminating null byte
        if (buffIdx < (BUFFER_SIZE - 1))
          buffer[buffIdx++] = inByte;
        break;
    }
  }
}
