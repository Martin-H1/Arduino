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

//#define DEBUG  // if this line is commented, the debug macros are ignored.

#include "Debug.h"
#include "AsyncServo.h"

/* This sketch uses binary angular measurement and subdivides the circle into
 * 8192 brads per full rotation, and a servo has roughly 4096 brads of resolution.
 */
const int16_t FULL_ROTATION = 8192;
const int16_t RIGHT_ANGLE = FULL_ROTATION / 4;

// Precomputed hash values for commands.
const uint16_t H_AZIMUTH = 53895;
const uint16_t H_SHOULDER = 23051;
const uint16_t H_ELBOW = 37790;
const uint16_t H_GRIPPER = 34814;
const uint16_t H_HOME = 64910;
const uint16_t H_SLEW = 63488;
const uint16_t H_LED = 35770;
const uint16_t H_TONE = 37435;

// configuration constants used to convert brads to microseconds
const uint8_t GRIPPER_PIN = 5;
const uint16_t GRIPPER_CLOSED = 700;
const uint16_t GRIPPER_CENTER = 1100;
const uint16_t GRIPPER_OPEN = 1500;

const uint8_t AZIMUTH_PIN = 2;
const uint16_t AZIMUTH_MAX = 2260;
const uint16_t AZIMUTH_CENTER = 1500;
const uint16_t AZIMUTH_MIN = 600;

const uint8_t SHOULDER_PIN = 3;
const uint16_t SHOULDER_MAX = 1750;
const uint16_t SHOULDER_CENTER = 1300;
const uint16_t SHOULDER_MIN = 850;

const uint8_t ELBOW_PIN = 4;
const uint16_t ELBOW_MAX = 700;
const uint16_t ELBOW_CENTER = 850;
const uint16_t ELBOW_MIN = 1000;

const uint8_t BUZZER_PIN = 12;

const uint8_t BRADS_IDX = 0;
const uint8_t DURATION_IDX = 1;
const uint8_t FREQUENCY_IDX = 0;

const char MISSING_ARGS[] = " - insuffcient arguments.";

// Create servo objects for each channel
AsyncServo azimuthServo;
AsyncServo shoulderServo;
AsyncServo elbowServo;
AsyncServo gripperServo;

void setup() {
  Serial.begin (9600);

  // Bind servo objects to pins and constraints.
  azimuthServo.init(AZIMUTH_PIN, AZIMUTH_MIN, AZIMUTH_CENTER, AZIMUTH_MAX);
  shoulderServo.init(SHOULDER_PIN, SHOULDER_MIN, SHOULDER_CENTER, SHOULDER_MAX);
  elbowServo.init(ELBOW_PIN, ELBOW_MIN, ELBOW_CENTER, ELBOW_MAX);
  gripperServo.init(GRIPPER_PIN, GRIPPER_CLOSED, GRIPPER_CENTER, GRIPPER_OPEN);

  // initialize digital pin for buzzer and LED_BUILTIN as an outputs.
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  // Variables for receiving and sending serial data
  const uint8_t BUFFER_SIZE = 64; // how much serial data we expect before a newline
  static char input[BUFFER_SIZE];
  static char response[BUFFER_SIZE];
  static uint8_t buffIdx = 0;

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

  // Poll the servos to allow them to move to target positons.
  azimuthServo.update();
  shoulderServo.update();
  elbowServo.update();
  gripperServo.update();
}

/* The DJB2 hashing function for the command string.
 */
uint16_t djb2Hash(char *str)
{
  uint16_t hash = 5381;
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
  const uint8_t MAX_ARGS = 6;     // Maxium number of arguments per command
  int16_t argv[MAX_ARGS];
  uint8_t argc = 0;

  // split the command into its parts
  char * token = strtok(input, ",");

  // copy first token to the response buffer
  strcpy(response, token);

  uint16_t hash = djb2Hash(response);

  // Extract the remaining tokens into the args array.
  token = strtok(NULL, ",");
  while (token != NULL && argc < MAX_ARGS)
  {
    argv[argc++] = atoi(token);
    token = strtok(NULL, ",");
  }

  // Declare a function pointer that matches the processing routine signature.
  int16_t (*fptr)(int16_t, int16_t [], char []);

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

void azimuth(uint8_t argc, int16_t argv[], char response[])
{
  // arv[0] is angle in brads, argv[1] is duration of slew.
  if (argc > 1)
  {
    // The azimuth axis has a range of -(right angle/2) to +(right angle/2)
    // Convert this signed into unsigned value starting at zero to ease mapping.
    int16_t value = constrain((argv[0] + (RIGHT_ANGLE >> 1) ), 0, RIGHT_ANGLE);

    // Map that value onto servo pulse widths and constrain.
    value = map(value, 0, RIGHT_ANGLE, AZIMUTH_MIN, AZIMUTH_MAX);

    // Move servo.
    azimuthServo.setTarget(value, argv[DURATION_IDX]);
    sprintf(response, "azimuth - %d.", value);
  }
  else
    strcat(response, MISSING_ARGS);
}

void shoulder(uint8_t argc, int16_t argv[], char response[])
{
  // arv[0] is angle in brads, argv[1] is duration of slew
  if (argc > 1)
  {
    // The shoulder axis has a range of -(right angle/2) to +(right angle/2)
    // Convert this signed into unsigned value starting at zero to ease mapping.
    int16_t value = constrain((argv[0] + (RIGHT_ANGLE >> 1) ), 0, RIGHT_ANGLE);

    // Map that value onto servo pulse widths and constrain.
    value = map(value, 0, RIGHT_ANGLE, SHOULDER_MIN, SHOULDER_MAX);

    // Move servo.
    shoulderServo.setTarget(value, argv[DURATION_IDX]);
    sprintf(response, "shoulder - %d.", value);
  }
  else
    strcat(response, MISSING_ARGS);
}

void elbow(uint8_t argc, int16_t argv[], char response[])
{
  // arv[0] is angle in brads, argv[1] is duration of slew
  if (argc > 1)
  {
    // The shoulder axis has a range of -(right angle/2) to +(right angle/2)
    // Convert this signed into unsigned value starting at zero to ease mapping.
    int16_t value = constrain((argv[0] + (RIGHT_ANGLE >> 1) ), 0, RIGHT_ANGLE);

    // Map that value onto servo pulse widths and constrain.
    value = map(value, 0, RIGHT_ANGLE, ELBOW_MIN, ELBOW_MAX);

    // Move servo.
    elbowServo.setTarget(value, argv[DURATION_IDX]);
    sprintf(response, "elbow - %d.", value);
  }
  else
    strcat(response, MISSING_ARGS);
}

void gripper(uint8_t argc, int16_t argv[], char response[])
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

void home(uint8_t argc, int16_t argv[], char response[])
{
  // Set all servos to known starting positions.
  azimuthServo.home();
  shoulderServo.home();
  elbowServo.home();
  gripperServo.home();
}

void slew(uint8_t argc, int16_t argv[], char response[])
{

}

void led(uint8_t argc, int16_t argv[], char response[])
{
  if (argc > 0)
  {
    digitalWrite(LED_BUILTIN, argv[0]);
    strcat(response, argv[0] != 0 ? " - high." : " - low.");
  }
  else
    strcat(response, MISSING_ARGS);
}

void buzzer(uint8_t argc, int16_t argv[], char response[])
{
  if (argc > 1)
  {
    tone(BUZZER_PIN, argv[FREQUENCY_IDX], argv[DURATION_IDX]);
    sprintf(response, "tone - frequency %d, duration %d.", argv[FREQUENCY_IDX], argv[DURATION_IDX]);
  }
  else
    strcat(response, MISSING_ARGS);
}

void unsupported(uint8_t argc, int16_t argv[], char response[])
{
  strcat(response, " - unsupported command.");
}