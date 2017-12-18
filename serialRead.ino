#include <AccelStepper.h>
#include <MultiStepper.h>
#include <QueueArray.h>
#include <Servo.h>



/****************************************************************/
/**** ASCII messaging scheme ************************************/
/****************************************************************/

// The message protocol is based on commands encoded as a sequence of string
// tokens and integers in a line of text.  One line is one message.  All the
// input message formats begin with a string naming a specific command or
// destination followed by one or two argument integers.  The output formats are
// similar but include more general debugging output with a variable number of
// tokens.

// The following message formats are recognized by this program.  Note that not
// all pins or channels are supported, e.g., servo output is only supported on a
// particular pin.

// Command  Arguments   Meaning
// led    <value>     controls the built-in LED, value is 0 or non-zero
// poll         <value>                 set the input polling rate, value is milliseconds 
// pwm    <pin> <value>   PWM control on given pin
// dig    <pin> <value>   digital output on given pin, value is 0 or non-zero
// svo    <pin> <value>   hobby servo PWM control signal on given pin, value is angle in degrees

// Additional messages can be added by inserting code in the user_message_#() functions below.

// This program generates the following messages:

// Command  Arguments   Meaning
// dbg    <value-or-token>+ debugging message to print for user
// clk    <microseconds>    Arduino clock time in microseconds
// led    <value>     reply with current LED state
// ana    <channel> <value> analog input value on given channel, value is 0 to 1023
// dig    <pin> <value>   digital input value on PIN8, value is 0 or 1

/****************************************************************/
/**** Library imports *******************************************/
/****************************************************************/

// Use the Servo library for generating control signals for hobby servomotors.
// Hobby servos require a specific form of pulse-width modulated control signal,
// usually with positive-going pulses between 1 and 2 milliseconds repeated at
// 50 Hz.  Note that this is a significantly different signal than the PWM
// usually required for powering a motor at variable torque.
//#include <Servo.h>

/****************************************************************/
/**** Global variables and constants ****************************/
/****************************************************************/

// The baud rate is the number of bits per second transmitted over the serial port.
#define BAUD_RATE 9600

// Interval in milliseconds between input samples.
static unsigned int hardware_polling_interval = 50; // 20 Hz samples to start
static unsigned int sol_polling_interval = 1000;
static unsigned long last_timeSol = 0;
unsigned long nowSol = millis();

// Create a hobby servo control signal generator.
//static Servo servo_output;
//static const int servo_output_pin = 4;

// The maximum message line length.
#define MAX_LINE_LENGTH 80

// The maximum number of tokens in a single message.
#define MAX_TOKENS 10

// Some version of the Arduino IDE don't correctly define this symbol for an
// Arduino Uno.
#ifndef LED_BUILTIN
#define LED_BUILTIN 13
#endif

struct Coordinates {
  int x;
  int y;
};
#define ENABLE 8

const int solPin = 12;
const int debugLed = 13;

static const int xStepPin = 2;
static const int xDirPin = 5;

static const int yStepPin = 3;
static const int yDirPin = 6;
// Define some steppers and the pins the will use
AccelStepper stepperY(AccelStepper::DRIVER, xStepPin, xDirPin);
AccelStepper stepperX(AccelStepper::DRIVER, yStepPin, yDirPin);
Servo triggerServo;
int triggerPos = 0;
int triggerSpeed = 10;

QueueArray <Coordinates> shotsQueue;

bool reached = false;

int triggerState = LOW;
int next_step_outputX = HIGH;
unsigned long step_intervalX = 100000.0;
long step_timerX = 0;
bool steppingX = true;
int currentPositionX = 0;
int directionY = 1;
bool movingX = false;
bool reachedX = false;

int next_step_outputY = HIGH;
unsigned long step_intervalY = 100000.0;
long step_timerY = 0;
bool steppingY = true;
int currentPositionY = 0;
int directionX = 1;
bool movingY = false;
bool reachedY = false;
bool lastShot = false;

Coordinates current = {-1,-1};


/****************************************************************/
/**** Utility functions *****************************************/
/****************************************************************/

/// Send a single debugging string to the console.
static void send_debug_message( const char *str )
{
  Serial.print("dbg ");
  Serial.println( str );
}

/****************************************************************/
/// Send a single debugging integer to the console.
static void send_debug_message( int i )
{
  Serial.print("dbg ");
  Serial.println( i );
}

/****************************************************************/
/// Send a single-argument message back to the host.
static void send_message( const char *command, long value )
{
  Serial.print( command );
  Serial.print( " " );
  Serial.println( value );
}

/****************************************************************/
/// Send a two-argument message back to the host.
static void send_message( const char *command, long value1, long value2 )
{
  Serial.print( command );
  Serial.print( " " );
  Serial.print( value1 );
  Serial.print( " " );
  Serial.println( value2 );
}

/****************************************************************/
// Wrapper on strcmp for clarity of code.  Returns true if strings are
// identical.
static int string_equal( char *str1, char *str2) 
{
  return !strcmp(str1, str2);
}

/****************************************************************/
/****************************************************************/
// Application-specific message processing.  You can customize these functions
// to add additional message types.

/// Convenience function provided to help with extending the messaging protocol;
/// this function receives zero-argument messages which just contain a token as
/// a string, e.g. "stop".  The protocol can also be extended by modifying
/// parse_input_message().
static void user_message_0( char *command )
{
  if (string_equal(command, "stop")) {
    // do something to set the stop state here

    send_debug_message("now stopped");

  } else  if (string_equal(command, "start")) {
    // do something to set the start state here

    send_debug_message("starting");
  }
  // ...
}

/// Similar to user_message_0; process one-argument messages with a single
/// value. E.g. "speed 33".
static void user_message_1( char *command, int value )
{
  if (string_equal(command, "speed")) {
    // do something to set the stop state using 'value'

  }
  // ...
}

/// Similar to user_message_0; process two-argument messages. E.g. "pantilt 0
/// 33".
static void user_message_2( char *command, int value1, int value2 )
{
  if (string_equal(command, "pantilt")) {
    // do something using value1 and value2
  }
  // ...
}

/****************************************************************/
/// Process an input message.  Unrecognized commands are silently ignored.
///   \param argc   number of argument tokens
///   \param argv   array of pointers to strings, one per token
static void parse_input_message(int argc, char *argv[])
{
  // Interpret the first token as a command symbol.
  char *command = argv[0];

  /* -- process zero-argument commands --------------------------- */
  if (argc == 1) {
    // just pass it along
    user_message_0( command );      
  }

  /* -- process one-argument commands --------------------------- */
  else if (argc == 2) {
    int value = atoi(argv[1] );

    // Process the 'led' command.
    if ( string_equal( command, "led" )) {
#ifdef LED_BUILTIN
      pinMode( LED_BUILTIN, OUTPUT );
      // turn on the LED if that value is true, then echo it back as a handshake
      digitalWrite(LED_BUILTIN, (value != 0) ? HIGH : LOW);
#endif
      send_message( "led", value );
    }
    else if ( string_equal( command, "poll" )) {
      if (value > 0)  hardware_polling_interval = value;
      else send_debug_message("invalid poll value");
    }

    // else just pass it along
    else { 
      user_message_1( command, value ); 
    }
  } 

  /* -- process two-argument commands --------------------------- */
  else if (argc == 3) {
    int x   = atoi(argv[1] );
    int y = atoi(argv[2] );


    // Process the 'pwm' command to generate a variable duty-cycle PWM signal on
    // any digital pin.  The value must be between 0 and 255.
    if ( string_equal( command, "pwm" )) {
      analogWrite( x, y );
      return;
    }

    // Process the 'dig' command to set a pin to output mode and control its level.
    else if ( string_equal( command, "dig" )) {
      pinMode( x, OUTPUT );
      digitalWrite( x, y );
      return;

    } else if( string_equal(command, "paint")) {
//      Serial.write(y);
      if(shotsQueue.count() <= 30) {
        Coordinates incoming = {x, y};
        shotsQueue.enqueue(incoming);
      }
    }

    // Process the 'svo' command to generate a hobby-servo PWM signal on a particular pin.
    // The value must be an angle between 0 and 180.
    else if ( string_equal( command, "svo" )) {
//      if (x == servo_output_pin) {
//        servo_output.write( y );
//      } else {
//        send_debug_message("unsupported servo pin");
//      }
      return;
    }

    // else just pass it along
    else user_message_2( command, x, y );
  }
}


/****************************************************************/
/// Polling function to process messages arriving over the serial port.  Each
/// iteration through this polling function processes at most one character.  It
/// records the input message line into a buffer while simultaneously dividing it
/// into 'tokens' delimited by whitespace.  Each token is a string of
/// non-whitespace characters, and might represent either a symbol or an integer.
/// Once a message is complete, parse_input_message() is called.

static void serial_input_poll(void)
{
  static char input_buffer[ MAX_LINE_LENGTH ];   // buffer for input characters
  static char *argv[MAX_TOKENS];                 // buffer for pointers to tokens
  static int chars_in_buffer = 0;  // counter for characters in buffer
  static int chars_in_token = 0;   // counter for characters in current partially-received token (the 'open' token)
  static int argc = 0;             // counter for tokens in argv
  static int error = 0;            // flag for any error condition in the current message

  // Check if at least one byte is available on the serial input.
  if (Serial.available()) {
    int input = Serial.read();
    // If the input is a whitespace character, end any currently open token.
    if ( isspace(input) ) {
      if ( !error && chars_in_token > 0) {
        if (chars_in_buffer == MAX_LINE_LENGTH) error = 1;
        else {
          input_buffer[chars_in_buffer++] = 0;  // end the current token
          argc++;                               // increase the argument count
          chars_in_token = 0;                   // reset the token state
        }
      }
      
      // If the whitespace input is an end-of-line character, then pass the message buffer along for interpretation.
      if (input == '\r' || input == '\n') {
           
          // if the message included too many tokens or too many characters, report an error
          if (error) send_debug_message("excessive input error");
        
          // else process any complete message
          else if (argc > 0) parse_input_message( argc, argv ); 
        
          // reset the full input state
          error = chars_in_token = chars_in_buffer = argc = 0;                     
      }
    }

    // Else the input is a character to store in the buffer at the end of the current token.
    else {
      // if beginning a new token
      if (chars_in_token == 0) {

        // if the token array is full, set an error state
        if (argc == MAX_TOKENS) error = 1;
      
        // otherwise save a pointer to the start of the token
        else argv[ argc ] = &input_buffer[chars_in_buffer];
      }

      // the save the input and update the counters
      if (!error) {
        if (chars_in_buffer == MAX_LINE_LENGTH) error = 1;
        else {
          input_buffer[chars_in_buffer++] = input;
          chars_in_token++;
        }
      }
    }
  }
}


static void moveToNewCurrent(void) {
  triggerPos = 90;
  triggerServo.write(90);
  reached = false;
  current = shotsQueue.dequeue();
//  Serial.write(shotsQueue.count());
  stepperX.moveTo(current.x);
  stepperY.moveTo(current.y);
}

/****************************************************************/
/// Polling function to read and send specific input values at periodic
/// intervals.

// N.B. The timing calculation could be improved to reduce jitter.

static void hardware_input_poll(void)
{
  static unsigned long last_time = 0;
  unsigned long now = millis();
  unsigned long interval = now - last_time;
  unsigned long intervalSol = nowSol - last_timeSol;
  if (interval > hardware_polling_interval) {
    last_time = now;
    if(!shotsQueue.isEmpty() ) {
      if(current.x == -1 && current.y == -1) {
        moveToNewCurrent();
      }
      if(stepperX.distanceToGo() == 0 && stepperY.distanceToGo() == 0) {
        if(reached == false) {
           nowSol = millis();
           last_timeSol = nowSol;
        }
        reached = true;
      }

      // Might need to change the order of the solPin write if the motors are moving before solenoid is triggered
      if(reached) {
        triggerPos = 135;
        triggerServo.write(triggerPos);
        intervalSol = last_timeSol - nowSol;
        last_timeSol = millis();
        if(intervalSol > sol_polling_interval ) {
          Serial.write(67);
            last_timeSol = nowSol;
            moveToNewCurrent();
        }
      } 
    } else {
      Serial.write(69);
    }
  }
}



void setup() {
        Serial.begin(9600);     // opens serial port, sets data rate to 9600 bps
        Serial.println("Setup");
        pinMode(solPin, OUTPUT);
        Serial.setTimeout(10); // set the timeout for parseInt
        shotsQueue.setPrinter (Serial);
        stepperX.setMaxSpeed(200.0);
        stepperX.setSpeed(200);
        stepperX.setAcceleration(100.0);
        
        stepperY.setMaxSpeed(200.0);
        stepperY.setSpeed(200);
        stepperY.setAcceleration(80.0);

        triggerServo.attach(12);
    
        pinMode(ENABLE,OUTPUT);
        digitalWrite(ENABLE,LOW);
        pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  serial_input_poll();
  stepperX.run();
  stepperY.run();
  hardware_input_poll();

}
