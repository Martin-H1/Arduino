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
const int16_t ACUTE_ANGLE = FULL_ROTATION / 8;
const int16_t RIGHT_ANGLE = FULL_ROTATION / 4;
const int16_t OBTUSE_ANGLE = RIGHT_ANGLE + ACUTE_ANGLE;

// Precomputed hash values for commands.
const uint16_t H_AZIMUTH = 53895;
const uint16_t H_SHOULDER = 23051;
const uint16_t H_VARM = 28667;
const uint16_t H_GRIPPER = 34814;
const uint16_t H_HOME = 64910;
const uint16_t H_SLEW = 63488;
const uint16_t H_LED = 35770;
const uint16_t H_STATUS = 36169;
const uint16_t H_TONE = 37435;

// Constants used to constrain joints to safe movement ranges, put
// arm into a known good position, and convert brads to microseconds.
const uint8_t  GRIPPER_PIN = 5;
const uint16_t GRIPPER_CLOSED = 700;
const uint16_t GRIPPER_OPEN = 1500;
const uint16_t GRIPPER_HOME = 1500;

const uint8_t  AZIMUTH_PIN = 2;
const uint16_t AZIMUTH_MIN = 600;
const uint16_t AZIMUTH_MAX = 2200;
const uint16_t AZIMUTH_HOME = 1500;

const uint8_t  SHOULDER_PIN = 3;
const uint16_t SHOULDER_MIN = 850;
const uint16_t SHOULDER_MAX = 1750;
const uint16_t SHOULDER_HOME = 1350;

const uint8_t  VARM_PIN = 4;
const uint16_t VARM_MIN = 700;
const uint16_t VARM_MAX = 2000;
const uint16_t VARM_HOME = 1100;

const uint8_t  BUZZER_PIN = 12;

const uint8_t BRADS_IDX = 0;
const uint8_t DURATION_IDX = 1;
const uint8_t FREQUENCY_IDX = 0;

const char MISSING_ARGS[] = " - insuffcient arguments.";

// Create servo objects for each channel
AsyncServo azimuthServo;
AsyncServo shoulderServo;
AsyncServo varmServo;
AsyncServo gripperServo;

void setup()
{
  Serial.begin (9600);

  // Wait for serial communications to start before continuing
  while (!Serial)
    ; // delay for Leonardo

  // Bind servo objects to pins and constraints.
  azimuthServo.init( AZIMUTH_PIN,  AZIMUTH_MIN,    AZIMUTH_MAX,  AZIMUTH_HOME);
  shoulderServo.init(SHOULDER_PIN, SHOULDER_MIN,   SHOULDER_MAX, SHOULDER_HOME);
  varmServo.init(    VARM_PIN,     VARM_MIN,       VARM_MAX,     VARM_HOME);
  gripperServo.init( GRIPPER_PIN,  GRIPPER_CLOSED, GRIPPER_OPEN,  GRIPPER_HOME);

  // Put the robot in known starting state.
  home(0, NULL, NULL);

  // initialize digital pin for buzzer and LED_BUILTIN as an outputs.
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop()
{
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
  varmServo.update();
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

  DPRINT("command="); DPRINT(token); DPRINT(", hash="); DPRINTLN(hash);

  // Extract the remaining tokens into the args array.
  token = strtok(NULL, ",");
  while (token != NULL && argc < MAX_ARGS)
  {
    argv[argc++] = atoi(token);
    token = strtok(NULL, ",");
  }

  // Conditionally print the argument count and first two arguments.
  DPRINT("argc="); DPRINT(argc);
  DPRINT(", argv[0]="); DPRINT(argv[0]);
  DPRINT(", argv[1]="); DPRINTLN(argv[1]);

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

    case H_VARM:
      fptr = varm;
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

    case H_STATUS:
      fptr = status;
      break;

    case H_TONE:
      fptr = buzzer;
      break;

    default:
      argv[0] = response;
      argv[1] = hash;
      fptr = unsupported;
  }

  // Dispatch to the processing routine.
  fptr(argc, argv, response);

  // Send reponse to host.
  Serial.println(response);
}

// Command handlers here

// The azimuth angle is measured from the X axis and the range of motion
// is 45 to 135 degrees (1024 to 3072 brads).
void azimuth(uint8_t argc, int16_t argv[], char response[])
{
  // arv[0] is angle in brads, argv[1] is duration of slew.
  if (argc > 1)
  {
    // Constrain the allowed inputs to the physical range of motion of the arm.
    int16_t value = constrain(argv[0], ACUTE_ANGLE, OBTUSE_ANGLE);

    // Map that value onto servo pulse widths and constrain.
    value = map(value, ACUTE_ANGLE, OBTUSE_ANGLE, AZIMUTH_MIN, AZIMUTH_MAX);

    // Move servo.
    azimuthServo.setTarget(value, argv[DURATION_IDX]);
    sprintf(response, "azimuth - %d.", value);
  }
  else
    strcat(response, MISSING_ARGS);
}

// The shoulder angle is measure from the horizontal XY plane to the vertical Z axis.
// It has a range of 45 to 135 degrees (1024 to 3072 brads).
void shoulder(uint8_t argc, int16_t argv[], char response[])
{
  // arv[0] is angle in brads, argv[1] is duration of slew
  if (argc > 1)
  {
    // Constrain the allowed inputs to the physical range of motion of the arm.
    int16_t value = constrain(argv[0], ACUTE_ANGLE, OBTUSE_ANGLE);

    // Map that value onto servo pulse widths and constrain.
    value = map(value, ACUTE_ANGLE, OBTUSE_ANGLE, SHOULDER_MIN, SHOULDER_MAX);

    // Move servo.
    shoulderServo.setTarget(value, argv[DURATION_IDX]);
    sprintf(response, "shoulder - %d.", value);
  }
  else
    strcat(response, MISSING_ARGS);
}

// The elbow is driven indirectly through the V arm linkage, and its angle
// is the summation of the shoulder and V arm angles. The V arm angle is
// measured relative to the Z axis and ranges from 45 to 150 degrees.
void varm(uint8_t argc, int16_t argv[], char response[])
{
  // arv[0] is angle in brads, argv[1] is duration of slew
  if (argc > 1)
  {
    // Constrain the allowed inputs to the physical range of motion of the arm.
    // Note: Mechanical considerations limit the permissible angle as the shoulder
    // angle varies. Care must be taken to avoid burning out the V arm servo!
    int16_t value = constrain(argv[0], ACUTE_ANGLE, OBTUSE_ANGLE);

    // Map that value onto servo pulse widths and constrain.
    value = map(value, ACUTE_ANGLE, OBTUSE_ANGLE, VARM_MIN, VARM_MAX);

    // Move servo.
    varmServo.setTarget(value, argv[DURATION_IDX]);
    sprintf(response, "varm - %d.", value);
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
  varmServo.home();
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

void status(uint8_t argc, int16_t argv[], char response[])
{
  uint16_t azPos = azimuthServo.readMicroseconds();
  uint16_t shPos = shoulderServo.readMicroseconds();
  uint16_t elPos = varmServo.readMicroseconds();
  uint16_t grPos = gripperServo.readMicroseconds();

  sprintf(response, "status - az=%u, sh=%u, el=%u, gr=%u", azPos, shPos, elPos, grPos);
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
  sprintf(response, "%s - unsupported command, hash=%u", argv[0], argv[1]);
}