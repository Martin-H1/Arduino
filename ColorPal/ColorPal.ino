/* ColorPal Sensor Example
 * Author.... Martin Heeermance based upon Phil Pilgrim's PBASIC example
 *
 * This program drives the Parallax ColorPAL color sensor and provides
 * serial RGB data to the PC-hosted TCS230_ColorPAL_match.exe color
 * matching program.
 */

#include <SoftwareSerial.h>

// I/O Pin definitions.
const int sio = 2;
const int unused = 255; // non-existant pin value
const int sioBaud = 4800;

// Received RGB values from ColorPAL.
int red;
int grn;
int blu;

// Set up two software serials on the same pin.
// This mimic PBASIC's serin and serout function.
SoftwareSerial serin(sio, unused);
SoftwareSerial serout(unused, sio);

// -----[ Initialization ]--------------------------------------------------
void setup()
{
  // initialize serial communication:
  Serial.begin(9600);
  
  // Reset the ColorPAL and enter direct command mode.
  reset();
  
  // Program ColorPAL to send $ then color data.
  serout.begin(sioBaud);
  pinMode(sio, OUTPUT);
  serout.print("= (00 $ m) !"); // buffer commmands, loop print $ and data end_loop now execute
  // serout is unused from this point forwards
  serout.end();

  // Now turn the sio pin into an input to read data from the color pal.
  serin.begin(sioBaud);
  pinMode(sio, INPUT);
}

// -----[ Program Code ]----------------------------------------------------
// SERIN sio, baud, [WAIT("$"), HEX3 red, HEX3 grn, HEX3 blu] ' Receive RGB data back.
void loop()
{
  readData();
}  

// -----[ Subroutines ]-----------------------------------------------------

// reset: Sends a long break to reset ColorPAL and enter direct command mode.
void reset()
{
  pinMode(sio, OUTPUT);
  digitalWrite(sio, LOW);  // Pull sio low to eliminate any residual charge.
  pinMode(sio, INPUT);     // Return pin to input.
  while (digitalRead(sio) != HIGH); // Wait for pin to be pulled high by ColorPAL.
  pinMode(sio, OUTPUT);
  digitalWrite(sio, LOW);  // Pull pin low.
  delay(80);               // Keep low for 80ms to enter Direct mode.
  pinMode(sio, INPUT);     // Return pin to input.
  delay(10);               // Pause another 10ms
}

void readData()
{
  char buffer[32];
  
  if (serin.available() > 0)
  {
    // Wait for a $ and then read three 3 digit hex numbers
    buffer[0] = serin.read();
    if (buffer[0] == '$')
    {
      for(int i = 0; i < 9; i++)
      {
        // Wait for the next input character.
        while (serin.available() == 0);     
        buffer[i] = serin.read();

        // every so often the data terminates early.  If this happens return
        if (buffer[i] == '$')
          return;
      }
      parseAndPrint(buffer);
      delay(10);
    }
  }
}

void parseAndPrint(char * data)
{
  // parse the hex data into integers.
  sscanf (data, "%3x%3x%3x", &red, &grn, &blu);

  // format using the format expected by the windows program and output it.
  char buffer[32];
  sprintf(buffer, "R%4.4d G%4.4d B%4.4d", red, grn, blu);
  Serial.println(buffer);
}
