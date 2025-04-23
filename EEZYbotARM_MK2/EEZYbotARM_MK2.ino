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
const int16_t STRAIGHT_ANGLE = FULL_ROTATION / 2;

// Precomputed hash values for commands.
const uint16_t H_AZIMUTH = 53895;
const uint16_t H_SHOULDER = 23051;
const uint16_t H_VARM = 28667;
const uint16_t H_ELBOW = 37790;
const uint16_t H_GRIPPER = 34814;
const uint16_t H_HOME = 64910;
const uint16_t H_SLEW = 63488;
const uint16_t H_LED = 35770;
const uint16_t H_STATUS = 36169;
const uint16_t H_TONE = 37435;

const uint16_t GRIPPER_CLOSED = 700;
const uint16_t GRIPPER_OPEN = 1500;

const uint8_t  BUZZER_PIN = 12;

const uint8_t BRADS_IDX = 0;
const uint8_t DURATION_IDX = 1;
const uint8_t FREQUENCY_IDX = 0;

const char MISSING_ARGS[] = " - insuffcient arguments.";

// Create servo objects for each channel.
AsyncServo azimuthServo;
AsyncServo shoulderServo;
AsyncServo varmServo;
AsyncServo gripperServo;

/* All one time initialization goes here.
 */
void setup()
{
  Serial.begin (9600);

  // Wait for serial communications to start before continuing
  while (!Serial)
    ; // delay for Leonardo

  // Bind servo objects to pins, and set constraints.
  // Constraints are used to constrain joints to safe movement ranges, put
  // arm into a known good position, and convert brads to microseconds.
  //               pin, min ms, max ms,     min brads,     max brads, home
  azimuthServo.init( 2,   600,    2200,   ACUTE_ANGLE,  OBTUSE_ANGLE, RIGHT_ANGLE);
  shoulderServo.init(3,   850,    1750,   ACUTE_ANGLE,  OBTUSE_ANGLE, RIGHT_ANGLE);
  varmServo.init(    4,   700,    1500,   0,            RIGHT_ANGLE,  ACUTE_ANGLE);
  gripperServo.init( 5,   700,    1500,   0,            OBTUSE_ANGLE, RIGHT_ANGLE);

  // Put the robot in known starting state.
  home(0, NULL, NULL);

  // initialize digital pin for buzzer and LED_BUILTIN as an outputs.
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
}

/* Polling tasks all go here.
 */
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

  // Poll the joint servos to allow them to move to target positons.
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
    // Move servo.
    uint16_t target = azimuthServo.setTarget(argv[0], argv[DURATION_IDX]);
    sprintf(response, "azimuth - target = %d brads, %d ms.", argv[0], target);
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
    // Move servo.
    uint16_t target = shoulderServo.setTarget(argv[0], argv[DURATION_IDX]);
    sprintf(response, "shoulder - target = %d brads, %d ms.", argv[0], target);
  }
  else
    strcat(response, MISSING_ARGS);
}

// The elbow is driven indirectly through the V arm linkage, and its angle is
// measured relative to the XY plane and ranges from 0 to almost 90 degrees.
// The range of motion of this joint is constrained by the position of the
// shoulder joint, and care must be taken to avoid burning out the servo.
void varm(uint8_t argc, int16_t argv[], char response[])
{
  // arv[0] is angle in brads, argv[1] is duration of slew
  if (argc > 1)
  {
    // Move servo.
    uint16_t target = varmServo.setTarget(argv[0], argv[DURATION_IDX]);
    sprintf(response, "varm - target = %d brads, %d ms.", argv[0], target);
  }
  else
    strcat(response, MISSING_ARGS);
}

// The elbow angle is measured relative to the main arm. If the arm could
// fully extend the elbow would be at zero degrees. This method computes
// the required varm setting to achieve the desired eblow angle.
void elbow(uint8_t argc, int16_t argv[], char response[])
{
  // agrv[0] is angle in brads, argv[1] is duration of slew
  if (argc > 1)
  {
    // elbow angle = varm angle + shoulder angle, so
    // varm angle = elbow angle - shoulder angle
    argv[0] = argv[0] - shoulderServo.getTarget();

    // Move servo.
    uint16_t target = varmServo.setTarget(argv[0], argv[DURATION_IDX]);
    sprintf(response, "elbow - target = %d brads, %d ms.", argv[0], target);
  }
  else
    strcat(response, MISSING_ARGS);
}

// The gripper has two states settable by the user.
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

// Home all servos to return arm to known good state.
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
  uint16_t azPos = azimuthServo.getTarget();
  uint16_t shPos = shoulderServo.getTarget();
  uint16_t elPos = varmServo.getTarget();
  uint16_t grPos = gripperServo.getTarget();

  sprintf(response, "status - az=%u, sh=%u, va=%u, gr=%u", azPos, shPos, elPos, grPos);
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