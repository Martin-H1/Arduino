/* Debug.h - a set of macros to enable condition serial print lines.
 * Usage:
 * Include this file and add the line:
 * #define DEBUG
 * The macros below will then print to the serial terminal.
 * If you comment DEBUG the DPRINT & DPRINTLN lines are defined as blank
 * and ignored by the compiler. Examples:
 * DPRINTLN("Testing123");    
 * DPRINTLN(0xC0FFEEul,DEC);
 * DPRINTLN(12648430ul,HEX);
 * DPRINTLNF("This message came from your flash");  
 */

#ifdef DEBUG
#define DPRINT(...)    Serial.print(__VA_ARGS__)
//OR, #define DPRINT(args...)    Serial.print(args)
#define DPRINTLN(...)  Serial.println(__VA_ARGS__)
#define DRINTF(...)    Serial.print(F(__VA_ARGS__))
#define DPRINTLNF(...) Serial.println(F(__VA_ARGS__)) //printing text using the F macro
#define DBEGIN(...)    Serial.begin(__VA_ARGS__)

#else
#define DPRINT(...)     //blank line
#define DPRINTLN(...)   //blank line
#define DPRINTF(...)    //blank line
#define DPRINTLNF(...)  //blank line
#define DBEGIN(...)     //blank line

#endif