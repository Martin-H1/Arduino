/* This sketch controls the EEZYBotArm Mk2 using messages set over the serial monitor.
 * The messages are in the form of command, arg1, arg2, ..., argN where args are integers.
 * The follow three examples move joint servos from their current positon to a
 * new one in the specified duration:
 * azimuth, brads, duration
 * shoulder, brads, duration
 * elbow, brads, duration
 *
 * These commands move multiple servos concurrently:
 * home
 * slew azimuth_brads, azimuth_duration, shoulder_brad, shoulder_duration, elbow_brads, elbow_duration
 *
 * These commands use boolean arguments:
 * gripper, [0 | 1]
 * led, [0 | 1]
 *
 * This command plays a tone at the frequency for the duration.
 * tone, hertz, duration
 */

#include <Servo.h>

/* This sketch uses binary angular measurement and subdivides the circle into
 * 8192 brads per full rotation, and a servo has roughly 4096 brads of resolution.
 */
const int FULL_ROTATION = 8192;
const int RIGHT_ANGLE = FULL_ROTATION / 4;

// Precomputed hash values for commands.
const unsigned int H_AZIMUTH = 53895;
const unsigned int H_SHOULDER = 23051;
const unsigned int H_ELBOW = 37790;
const unsigned int H_GRIPPER = 34814;
const unsigned int H_HOME = 64910;
const unsigned int H_SLEW = 63488;
const unsigned int H_LED = 35770;
const unsigned int H_TONE = 37435;

// configuration constants used to convert brads to microseconds
const int GRIPPER_PIN = 5;
const int GRIPPER_CLOSED = 700;
const int GRIPPER_OPEN = 1500;

const int AZIMUTH_PIN = 2;
const int AZIMUTH_MAX = 2260;
const int AZIMUTH_CENTER = 1500;
const int AZIMUTH_MIN = 600;

const int SHOULDER_PIN = 3;
const int SHOULDER_MAX = 1750;
const int SHOULDER_CENTER = 1300;
const int SHOULDER_MIN = 850;

const int ELBOW_PIN = 4;
const int ELBOW_MAX = 700;
const int ELBOW_CENTER = 850;
const int ELBOW_MIN = 1000;

const int BUZZER_PIN = 12;

const int BRADS_IDX = 0;
const int DURATION_IDX = 1;
const int FREQUENCY_IDX = 0;

const char MISSING_ARGS[] = " - insuffcient arguments.";

// Create servo objects for each channel
Servo azimuthServo;
Servo shoulderServo;
Servo elbowServo;
Servo gripperServo;

void setup() {
  Serial.begin (9600);

  // Bind servo objects to pins.
  azimuthServo.attach(AZIMUTH_PIN);
  shoulderServo.attach(SHOULDER_PIN);
  elbowServo.attach(ELBOW_PIN);
  gripperServo.attach(GRIPPER_PIN);

  // initialize digital pin for buzzer and LED_BUILTIN as an outputs.
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  // Variables for receiving and sending serial data
  const unsigned int BUFFER_SIZE = 64; // how much serial data we expect before a newline
  static char input[BUFFER_SIZE];
  static char response[BUFFER_SIZE];
  static unsigned int buffIdx = 0;

  // if serial data available, process it
  if (Serial.available () > 0) {
    byte inByte = Serial.read();

    switch (inByte)
    {
      case '\n':   // end of line
        // Null terminate the input, process the command, and formulate the response.
        input[buffIdx] = 0;
        processCmd(input, response);

        // Reset the index for next command.
        buffIdx = 0;
        break;

      case '\r':   // discard carriage return
        break;

      default:
        // keep adding if not full ... allow for terminating null byte
        if (buffIdx < (BUFFER_SIZE - 1))
          input[buffIdx++] = tolower(inByte);
        break;
    }
  }
}

/* The DJB2 hashing function for the command string.
 */
unsigned int djb2Hash(char *str)
{
  unsigned int hash = 5381;
  char c;

  while (c = *str++)
    hash = ((hash << 5) + hash) + c; /* hash * 33 + c */

  return hash;
}

/* Extract the command and arguments.
 * Forumlates the response.
 */
void processCmd(char input[], char response[])
{
  const unsigned int MAX_ARGS = 6;     // Maxium number of arguments per command
  int argv[MAX_ARGS];
  unsigned int argc = 0;

  // split the command into its parts
  char * token = strtok(input, ",");

  // copy first token to the response buffer
  strcpy(response, token);

  unsigned int hash = djb2Hash(response);

  // Extract the remaining tokens into the args array.
  token = strtok(NULL, ",");
  while (token != NULL && argc < MAX_ARGS)
  {
    argv[argc++] = atoi(token);
    token = strtok(NULL, ",");
  }

  // Declare a function pointer that matches the processing routine signature.
  int (*fptr)(int, int [], char []);

  // Select a processing routine using the precomputed hashes.
  switch (hash)
  {
    case H_AZIMUTH:
      fptr = &azimuth;
      break;

    case H_SHOULDER:
      fptr = shoulder;
      break;

    case H_ELBOW:
      fptr = elbow;
      break;

    case H_GRIPPER:
      fptr = gripper;
      break;

    case H_HOME:
      fptr = home;
      break;

    case H_SLEW:
      fptr = slew;
      break;

    case H_LED:
      fptr = led;
      break;

    case H_TONE:
      fptr = buzzer;
      break;

    default:
      fptr = unsupported;
  }

  // Dispatch to the processing routine.
  fptr(argc, argv, response);

  // Send reponse to host.
  Serial.println(response);
}

// Command handlers here

void azimuth(int argc, int argv[], char response[])
{
  // arv[0] is angle in brads, argv[1] is duration of slew.
  if (argc > 1)
  {
    // The azimuth axis has a range of -(right angle/2) to +(right angle/2)
    // Convert this signed into unsigned value starting at zero to ease mapping.
    int value = constrain((argv[0] + (RIGHT_ANGLE >> 1) ), 0, RIGHT_ANGLE);

    // Map that value onto servo pulse widths and constrain.
    value = map(value, 0, RIGHT_ANGLE, AZIMUTH_MIN, AZIMUTH_MAX);
    value = constrain(value, AZIMUTH_MIN, AZIMUTH_MAX);

    // Move servo.
    azimuthServo.writeMicroseconds(value);
    sprintf(response, "azimuth - %d.", value);
  }
  else
    strcat(response, MISSING_ARGS);
}

void shoulder(int argc, int argv[], char response[])
{
  // arv[0] is angle in brads, argv[1] is duration of slew
}

void elbow(int argc, int argv[], char response[])
{
  // arv[0] is angle in brads, argv[1] is duration of slew
}

void gripper(int argc, int argv[], char response[])
{
  if (argc > 0)
  {
    // arv[0] is gripper state, 1 = close, 0 = open
    gripperServo.writeMicroseconds(argv[0] ? GRIPPER_CLOSED : GRIPPER_OPEN);
    strcat(response, argv[0] != 0 ? " - closed." : " - open.");
  }
  else
    strcat(response, MISSING_ARGS);
}

void home(int argc, int argv[], char response[])
{
  // set up arguments for each call.
  argc = 2;
  argv[BRADS_IDX] = AZIMUTH_CENTER;
  argv[DURATION_IDX] = 1000;
  azimuth(argc, argv, response);
  argv[BRADS_IDX] = ELBOW_CENTER;
  elbow(argc, argv, response);
  argv[BRADS_IDX] = 0;
  gripper(argc, argv, response);
  argv[BRADS_IDX] = SHOULDER_CENTER;
  shoulder(argc, argv, response);
}

void slew(int argc, int argv[], char response[])
{

}

void led(int argc, int argv[], char response[])
{
  if (argc > 0)
  {
    digitalWrite(LED_BUILTIN, argv[0]);
    strcat(response, argv[0] != 0 ? " - high." : " - low.");
  }
  else
    strcat(response, MISSING_ARGS);
}

void buzzer(int argc, int argv[], char response[])
{
  if (argc > 1)
  {
    tone(BUZZER_PIN, argv[FREQUENCY_IDX], argv[DURATION_IDX]);
    sprintf(response, "tone - frequency %d, duration %d.", argv[FREQUENCY_IDX], argv[DURATION_IDX]);
  }
  else
    strcat(response, MISSING_ARGS);
}

void unsupported(int argc, int argv[], char response[])
{
  strcat(response, " - unsupported command.");
}

void sweepAzimuth()
{
  for (int pos = 600; pos < 2200; pos++)
  {
    azimuthServo.writeMicroseconds(pos);
    delay(5);
  }

  for (int pos = 2200; pos > 600; pos--)
  {
    azimuthServo.writeMicroseconds(pos);
    delay(5);
  }
}

void sweepShoulder()
{
  for (int pos = 750; pos < 1600; pos++)
  {
    shoulderServo.writeMicroseconds(pos);
    delay(5);
  }

  for (int pos = 1600; pos > 750; pos--)
  {
    shoulderServo.writeMicroseconds(pos);
    delay(5);
  }
}