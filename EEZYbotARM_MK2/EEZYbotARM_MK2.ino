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
 * slew azimuth_brads, shoulder_brad, elbow_brads, azimuth_duration, shoulder_duration, elbow_duration
 *
 * These commands use boolean arguments:
 * gripper, [0 | 1]
 * led, [0 | 1]
 *
 * This command plays a tone at the frequency for the duration.
 * tone, hertz, duration
 */

#include <Servo.h>

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

// Create servo objects for each channel
Servo azimuthServo;
Servo shoulderServo;
Servo elbowServo;
Servo gripperServo;

// Variables for receiving and sending serial data
const unsigned int BUFFER_SIZE = 64; // how much serial data we expect before a newline
char input[BUFFER_SIZE];
char response[BUFFER_SIZE];

bool respAvail = false;

void setup() {
  Serial.begin (9600);

  // Bind servo objects to pins.
  azimuthServo.attach(AZIMUTH_PIN);
  shoulderServo.attach(SHOULDER_PIN);
  elbowServo.attach(ELBOW_PIN);
  gripperServo.attach(GRIPPER_PIN);

  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  // if serial data available, process it
  if (Serial.available () > 0) {
    static unsigned int buffIdx = 0;
    byte inByte = Serial.read();

    switch (inByte)
    {
      case '\n':   // end of line
        input[buffIdx] = 0;    // terminating null byte.
        processCmd();          // process the command and formulate the response.
        buffIdx = 0;           // reset index for next time
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

  // Send response if one is available.
  if (respAvail)
  {
    Serial.println(response);
    respAvail = false;
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
void processCmd()
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
  while (token != NULL)
  {
    argv[argc++] = atoi(token);
    token = strtok(NULL, ",");
  }

  // Dispatch to the processing routine.
  switch (hash)
  {
    case H_AZIMUTH:
      azimuth(argc, argv);
      break;

    case H_SHOULDER:
      shoulder(argc, argv);
      break;

    case H_ELBOW:
      elbow(argc, argv);
      break;

    case H_GRIPPER:
      gripper(argc, argv);
      break;

    case H_HOME:
      home(argc, argv);
      break;

    case H_SLEW:
      slew(argc, argv);
      break;

    case H_LED:
      led(argc, argv);
      break;

    case H_TONE:
      buzzer(argc, argv);
      break;

    default:
      unsupported(argc, argv);
  } 
}

// Command handlers here

void azimuth(int argc, int argv[])
{
  // arv[0] is angle in brads, argv[1] is duration of slew.
}

void shoulder(int argc, int argv[])
{
  // arv[0] is angle in brads, argv[1] is duration of slew
}

void elbow(int argc, int argv[])
{
  // arv[0] is angle in brads, argv[1] is duration of slew
}

void gripper(int argc, int argv[])
{
  // arv[0] is gripper state, 1 = close, 0 = open
  gripperServo.writeMicroseconds(argv[0] ? GRIPPER_CLOSED : GRIPPER_OPEN);
}

void home(int argc, int argv[])
{
  // set up arguments for each call.
  argc = 2;
  argv[0] = AZIMUTH_CENTER;
  argv[1] = 1000;
  azimuth(argc, argv);
  argv[0] = ELBOW_CENTER;
  elbow(argc, argv);
  argv[0] = 0;
  gripper(argc, argv);
  argv[0] = SHOULDER_CENTER;
  shoulder(argc, argv);
}

void slew(int argc, int argv[])
{

}

void led(int argc, int argv[])
{
  digitalWrite(LED_BUILTIN, argv[0] ? HIGH : LOW);
}

void buzzer(int argc, int argv[])
{
  tone(BUZZER_PIN, argv[0], argv[1]);
}

void unsupported(int argc, int argv[])
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