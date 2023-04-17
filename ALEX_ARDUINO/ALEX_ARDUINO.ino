#include <serialize.h>
#include <buffer.h>

#include <stdarg.h>
#include <math.h>
#include "packet.h"
#include "constants.h"

typedef enum {
  STOP = 0,
  FORWARD = 1,
  BACKWARD = 2,
  LEFT = 3,
  RIGHT = 4
} TDirection;

volatile TDirection dir = STOP;

/*
	 Alex's configuration constants
*/

// Number of ticks per revolution from the
// wheel encoder.

#define COUNTS_PER_REV      60

// Wheel circumference in cm.

#define WHEEL_CIRC          20.4

#define LEFT_FORWARD_OCR    OCR0A // pin6  // PD6
#define LEFT_REVERSE_OCR    OCR0B // pin5  // PD5
#define RIGHT_FORWARD_OCR   OCR2A // pin11 // PB3
#define RIGHT_REVERSE_OCR   OCR1B // pin10 // PB2

#define LEFT_FORWARD_MASK   1 << 6
#define LEFT_REVERSE_MASK   1 << 5
#define RIGHT_FORWARD_MASK  1 << 3
#define RIGHT_REVERSE_MASK  1 << 2

#define ALEX_LENGTH         19
#define ALEX_BREADTH        12.5

#define COLOUR_S2_MASK      1 << 0 // pin8 PB0 
#define COLOUR_S3_MASK      1 << 1 // pin9 PB1
#define COLOUR_OUT_MASK     1 << 4 // pin4 PD4 
#define COLOUR_OUT_PIN      4

#define TRIGGER_PIN         12
#define TRIGGER_MASK        1 << 4 // PB4
#define ECHO_PIN            13
#define ECHO_MASK           1 << 5 // PB5


float alexDiagonal;
float alexCirc;

/*
	 Alex's State Variables
*/

// Store the ticks from Alex's left and
// right encoders.
volatile unsigned long leftForwardTicks;
volatile unsigned long rightForwardTicks;
volatile unsigned long leftReverseTicks;
volatile unsigned long rightReverseTicks;

volatile unsigned long leftForwardTicksTurns;
volatile unsigned long rightForwardTicksTurns;
volatile unsigned long leftReverseTicksTurns;
volatile unsigned long rightReverseTicksTurns;

// Store the revolutions on Alex's left
// and right wheels
volatile unsigned long leftRevs;
volatile unsigned long rightRevs;

// Forward and backward distance traveled
volatile unsigned long forwardDist;
volatile unsigned long reverseDist;

unsigned long deltaDist;
unsigned long newDist;

unsigned long deltaTicks;
unsigned long targetTicks;

unsigned long prescaler = 64;

/*

	 Alex Communication Routines.

*/

// Size of temporary buffers 
#define TEMP_BUFFER_SIZE  180

TBuffer receiveBuffer;
TBuffer transmitBuffer;


TResult readPacket(TPacket *packet)
{
  // Reads in data from the serial port and
  // deserializes it. Returns deserialized
  // data in "packet".

  char buffer[PACKET_SIZE];
  int len = readSerial(buffer);
  if ( len == 0 ) {
    return PACKET_INCOMPLETE;
  }
  return deserialize (buffer, len, packet);

}

void sendStatus() {
  TPacket statusPacket;
  statusPacket.packetType = PACKET_TYPE_RESPONSE;
  statusPacket.command = RESP_STATUS;
  statusPacket.params[0] = leftForwardTicks;
  statusPacket.params[1] = rightForwardTicks;
  statusPacket.params[2] = leftReverseTicks;
  statusPacket.params[3] = rightReverseTicks;
  statusPacket.params[4] = leftForwardTicksTurns;
  statusPacket.params[5] = rightForwardTicksTurns;
  statusPacket.params[6] = leftReverseTicksTurns;
  statusPacket.params[7] = rightReverseTicksTurns;
  statusPacket.params[8] = forwardDist;
  statusPacket.params[9] = reverseDist;
  unsigned long duration = getDistance();
  statusPacket.params[10] = duration;
  sendResponse(&statusPacket);
}
 
unsigned long getDistance () {
  // Switch on trigger for 10 microseconds 
  PORTB |= TRIGGER_MASK;
  long endpoint = TCNT2 + 3;
  while ( TCNT2 != endpoint );
  PORTB &= ~( TRIGGER_MASK );

  unsigned long duration = pulseIn (ECHO_PIN, HIGH);
  return duration;
  //unsigned long ultrasonic_distance = duration * 0.034 / 2;  
  //return ultrasonic_distance;
}

/*
	 Arduino only sends RGB values over
	 Colour interpretation is done on rpi
*/
unsigned long find_colour () {
  unsigned long pulseWidth = pulseIn ( COLOUR_OUT_PIN, LOW );
  return pulseWidth;
}

void stopColourSensors() {
  PORTB |= COLOUR_S2_MASK;
  PORTB &= ~COLOUR_S3_MASK;
}

// Determine colour
void getColour( TPacket *colourPacket ) {

  /* Colour    S2       S3
  	 R        L        L
  	 G        H        H
  	 B        L        H
  	 Clear    H        L
  */

  // Start red sensor
  PORTB &= ~ ( COLOUR_S2_MASK | COLOUR_S3_MASK );
  unsigned long rVal = find_colour ();
  stopColourSensors();
  delay(200);

  // Start green sensor
  PORTB |= ( COLOUR_S2_MASK | COLOUR_S3_MASK );
  unsigned long gVal = find_colour ();
  stopColourSensors();
  delay(200);

  // Start blue sensor
  PORTB &= ( ~COLOUR_S2_MASK );
  PORTB |= COLOUR_S3_MASK;
  unsigned long bVal = find_colour ();
  stopColourSensors();
  delay(200);

  colourPacket->params[0] = rVal;
  colourPacket->params[1] = gVal;
  colourPacket->params[2] = bVal;
}

void sendColour () {
  TPacket colourPacket;
  colourPacket.packetType = PACKET_TYPE_RESPONSE;
  colourPacket.command = RESP_COLOUR;

  getColour( &colourPacket );

  sendResponse ( &colourPacket );
}

void sendMessage(const char *message)
{
  // Sends text messages back to the Pi. Useful
  // for debugging.

  TPacket messagePacket;
  messagePacket.packetType = PACKET_TYPE_MESSAGE;
  strncpy(messagePacket.data, message, MAX_STR_LEN);
  sendResponse(&messagePacket);
}

void dbprintf (char *format, ...) {
  va_list args;
  char buffer [128];
  va_start ( args, format );
  vsprintf ( buffer, format, args );
  sendMessage ( buffer );
}

void sendBadPacket()
{
  // Tell the Pi that it sent us a packet with a bad
  // magic number.

  TPacket badPacket;
  badPacket.packetType = PACKET_TYPE_ERROR;
  badPacket.command = RESP_BAD_PACKET;
  sendResponse(&badPacket);

}

void sendBadChecksum()
{
  // Tell the Pi that it sent us a packet with a bad
  // checksum.

  TPacket badChecksum;
  badChecksum.packetType = PACKET_TYPE_ERROR;
  badChecksum.command = RESP_BAD_CHECKSUM;
  sendResponse(&badChecksum);
}

void sendBadCommand()
{
  // Tell the Pi that we don't understand its
  // command sent to us.

  TPacket badCommand;
  badCommand.packetType = PACKET_TYPE_ERROR;
  badCommand.command = RESP_BAD_COMMAND;
  sendResponse(&badCommand);

}

void sendBadResponse()
{
  TPacket badResponse;
  badResponse.packetType = PACKET_TYPE_ERROR;
  badResponse.command = RESP_BAD_RESPONSE;
  sendResponse(&badResponse);
}

void sendOK()
{
  TPacket okPacket;
  okPacket.packetType = PACKET_TYPE_RESPONSE;
  okPacket.command = RESP_OK;
  sendResponse(&okPacket);
}

void sendResponse(TPacket *packet)
{
  // Takes a packet, serializes it then sends it out
  // over the serial port.
  char buffer[PACKET_SIZE];
  int len = serialize(buffer, packet, sizeof(TPacket));
  writeSerial(buffer, len);
}

/*
	 Setup and start codes for external interrupts and
	 pullup resistors.
*/
// Enable pull up resistors on pins 2 and 3

void enablePullups()
{
  // We set bits 2 and 3 in DDRD to 0 to make them inputs.
  DDRD &= 0b11110011;
  PORTD |= 0b00001100;
}

// Functions to be called by INT0 and INT1 ISRs.
void leftISR() {
  switch (dir) {
    case FORWARD:
      leftForwardTicks++;
      forwardDist = (unsigned long) ((float) leftForwardTicks / COUNTS_PER_REV * WHEEL_CIRC);
      break;
    case BACKWARD:
      leftReverseTicks++;
      reverseDist = (unsigned long) ((float) leftReverseTicks / COUNTS_PER_REV * WHEEL_CIRC);
      break;
    case LEFT:
      leftReverseTicksTurns++;
      break;
    case RIGHT:
      leftForwardTicksTurns++;
      break;
  }
}

void rightISR() {
  switch (dir) {
    case FORWARD:
      rightForwardTicks++;
      break;
    case BACKWARD:
      rightReverseTicks++;
      break;
    case RIGHT:
      rightReverseTicksTurns++;
      break;
    case LEFT:
      rightForwardTicksTurns++;
      break;
  }
}

// Set up the external interrupt pins INT0 and INT1
// for falling edge triggered.
void setupEINT()
{
  EIMSK |= 0b11;
  EICRA = 0b1010;
}

// Implement the external interrupt ISRs below.
// INT0 ISR should call leftISR while INT1 ISR
// should call rightISR.

ISR ( INT0_vect ) {
  leftISR();
}

ISR ( INT1_vect ) {
  rightISR();
}

/*
	 Setup and start codes for serial communications
*/

// Set up the serial connection. 
void setupSerial()
{
  UCSR0C = 0b00000110;
  UBRR0L = 103;
  UBRR0H = 0;
  UCSR0A = 0;
}

// Start the serial connection
void startSerial()
{
  UCSR0B = 0b10111000;
}

// Read the serial port. 
ISR ( USART_RX_vect ) {
  unsigned char data = UDR0;
  writeBuffer ( &receiveBuffer, data );
}

int readSerial(char *buffer)
{
  int count = 0;
  TBufferResult result = BUFFER_OK;
  
  while ( dataAvailable ( &receiveBuffer ) && result == BUFFER_OK ){
    result = readBuffer ( &receiveBuffer, &buffer[count] );
    if ( result == BUFFER_OK ) {
      count += 1;
    }
  } 
  
  return count;
}

// Write to the serial port. 
ISR ( USART_UDRE_vect ) {
  unsigned char data;
  TBufferResult result = readBuffer ( &transmitBuffer, ( unsigned char * ) &data );
  if ( result == BUFFER_OK ) {
    UDR0 = data;
  } else if ( result == BUFFER_EMPTY ) {
    UCSR0B &= 0b11011111;
  }
}

void writeSerial(const char *buffer, int len)
{
  TBufferResult result = BUFFER_OK;
  for ( int count = 1; count < len && result == BUFFER_OK; count += 1 ) {
    result = writeBuffer ( &transmitBuffer, buffer[count] );
  }

  UDR0 = buffer[0];

  UCSR0B |= 0b00100000;
}

/*
	 Alex's motor drivers.
*/

// Set up Alex's motors. 
void setupMotors()
{
  /* Our motor set up is:
  	 A1IN - Pin 5, PD5, OC0B
  	 A2IN - Pin 6, PD6, OC0A
  	 B1IN - Pin 10, PB2, OC1B
  	 B2In - pIN 11, PB3, OC2A
  */

  // PWM phase correct mode 1
  // No need to set interrupts since 
  // duty cycles do not change on compare match
  // motor pins clear on compare match 
  // OCR set to 0
  // Otherwise motors may start running

  DDRD |= ( LEFT_FORWARD_MASK | LEFT_REVERSE_MASK);
  DDRB |= ( RIGHT_FORWARD_MASK | RIGHT_REVERSE_MASK );

  TCCR0A = 0b10100001;
  TCNT0 = 0;
  OCR0A = 0;
  OCR0B = 0;

  TCCR1A = 0b00100001;
  TCNT1 = 0;
  OCR1B = 0;

  TCCR2A = 0b10000001;
  TCNT2 = 0;
  OCR2A = 0;
}

// Start the PWM for Alex's motors.
void startMotors()
{
  // Start timers with prescaler 64
  TCCR0B = 0b11;
  TCCR2B = 0b11;
  TCCR1B = 0b11;
}

// Convert percentages to PWM values
int pwmVal(float speed)
{
  if (speed < 0.0)
    speed = 0;

  if (speed > 100.0)
    speed = 100.0;

  return (int) ((speed / 100.0) * 255.0);
}

void setupUltrasonic()
{
  DDRB |= TRIGGER_MASK; // Set Trigger pin on Arduino pin 12 to output
  DDRB &= ~(ECHO_MASK); // Set Echo pin on Arduino pin 13 to input
}

// Move Alex forward "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// move forward at half speed.
// Specifying a distance of 0 means Alex will
// continue moving forward indefinitely.
void forward(float dist, float speed)
{
  dir = FORWARD;

  if ( dist > 0 ) {
    deltaDist = dist;
  } else {
    deltaDist = 9999999;
  }
  newDist = forwardDist + deltaDist;

  int val = pwmVal(speed);

  LEFT_FORWARD_OCR = val;
  RIGHT_FORWARD_OCR = val;
  LEFT_REVERSE_OCR = 0;
  RIGHT_REVERSE_OCR = 0;
}

// Reverse Alex "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// reverse at half speed.
// Specifying a distance of 0 means Alex will
// continue reversing indefinitely.
void reverse(float dist, float speed)
{
  dir = BACKWARD;

  if ( dist > 0 ) {
    deltaDist = dist;
  } else {
    deltaDist = 9999999;
  }
  newDist = reverseDist + deltaDist;

  int val = pwmVal(speed);

  LEFT_REVERSE_OCR = val;
  RIGHT_REVERSE_OCR = val;
  LEFT_FORWARD_OCR = 0;
  RIGHT_FORWARD_OCR = 0;
}

unsigned long computeDeltaTicks ( float ang ) {
  unsigned long ticks = ( unsigned long ) ( ang * alexCirc * COUNTS_PER_REV / ( 360.0 * WHEEL_CIRC) );
  return ticks;
}

// Turn Alex left "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Alex to
// turn left indefinitely.
void left(float ang, float speed)
{
  dir = LEFT;

  if ( ang == 0 ) {
    deltaTicks = 99999999;
  } else {
    deltaTicks = computeDeltaTicks (ang);
  }

  targetTicks = leftReverseTicksTurns + deltaTicks;

  int val = pwmVal(speed);

  LEFT_REVERSE_OCR = val;
  RIGHT_FORWARD_OCR = val;
  LEFT_FORWARD_OCR = 0;
  RIGHT_REVERSE_OCR = 0;
}

// Turn Alex right "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Alex to
// turn right indefinitely.
void right(float ang, float speed)
{
  dir = RIGHT;

  if ( ang == 0 ) {
    deltaTicks = 99999999;
  } else {
    deltaTicks = computeDeltaTicks (ang);
  }

  targetTicks = rightReverseTicksTurns + deltaTicks;

  int val = pwmVal(speed);

  RIGHT_REVERSE_OCR = val;
  LEFT_FORWARD_OCR = val;
  LEFT_REVERSE_OCR = 0;
  RIGHT_FORWARD_OCR = 0;

}

// Stop Alex. 
void stop()
{
  dir = STOP;

  LEFT_FORWARD_OCR = 0;
  LEFT_REVERSE_OCR = 0;
  RIGHT_FORWARD_OCR = 0;
  RIGHT_REVERSE_OCR = 0;
}

/*
	 Alex's setup and run codes
*/

// Clears all our counters
void clearCounters()
{

  leftForwardTicks = 0;
  rightForwardTicks = 0;
  leftReverseTicks = 0;
  rightReverseTicks = 0;

  leftForwardTicksTurns = 0;
  rightForwardTicksTurns = 0;
  leftReverseTicksTurns = 0;
  rightReverseTicksTurns = 0;

  forwardDist = 0;
  reverseDist = 0;
}

void setupColourSensor() {
  // S2 = PB0 ( PIN 8 ) | S3 = PB1 ( PIN 9 ) | OUT = PD4 ( PIN 4 )
  // S0 high and S1 low for 20% frequency
  // S2 and S3 are output pins, OUT is an input pin
  DDRB |= ( COLOUR_S2_MASK | COLOUR_S3_MASK);
  DDRD &= ( ~COLOUR_OUT_MASK );
  stopColourSensors();
}

// Clears one particular counter
void clearOneCounter(int which)
{
  switch (which) {
    case 0:
      clearCounters();
      break;
    case 1:
      leftForwardTicks = 0;
      break;
    case 2:
      rightForwardTicks = 0;
      break;
    case 3:
      leftReverseTicks = 0;
      break;
    case 4:
      rightReverseTicks = 0;
      break;
    case 5:
      leftForwardTicksTurns = 0;
      break;
    case 6:
      rightForwardTicksTurns = 0;
      break;
    case 7:
      leftReverseTicksTurns = 0;
      break;
    case 8:
      rightReverseTicksTurns = 0;
      break;
    case 9:
      forwardDist = 0;
      break;
    case 10:
      reverseDist = 0;
      break;
  }
}

// Intialize Vincent's internal states
void initializeState()
{
  clearCounters();
}

void handleCommand(TPacket *command) {
  switch (command->command) {
    // For movement commands, param[0] = distance, param[1] = speed.
    case COMMAND_FORWARD:
      sendOK();
      forward((float) command->params[0], (float) command->params[1]);
      break;

    case COMMAND_REVERSE:
      sendOK();
      reverse((float) command->params[0], (float) command->params[1]);
      break;

    case COMMAND_TURN_LEFT:
      sendOK();
      left((float) command->params[0], (float) command->params[1]);
      break;

    case COMMAND_TURN_RIGHT:
      sendOK();
      right((float) command->params[0], (float) command->params[1]);
      break;

    case COMMAND_STOP:
      sendOK();
      stop();
      break;

    case COMMAND_GET_STATS:
      sendStatus();
      break;

    case COMMAND_CLEAR_STATS:
      sendOK(); // Acknowledgement
      clearOneCounter(command->params[0]);
      break;

    case COMMAND_GET_COLOUR:
      sendColour();
      break;

    default:
      sendBadCommand();
  }
}

void waitForHello()
{
  int exit = 0;

  while (!exit)
  {
    TPacket hello;
    TResult result;

    do {
      result = readPacket(&hello);
    } while (result == PACKET_INCOMPLETE);

    if (result == PACKET_OK)
    {
      if (hello.packetType == PACKET_TYPE_HELLO) {
        sendOK();
        exit = 1;
      } else {
        sendBadResponse();
      }
    } else if (result == PACKET_BAD) {
      sendBadPacket();
    } else if (result == PACKET_CHECKSUM_BAD) {
      sendBadChecksum();
    }
  } // !exit
}

void disableADC () {
  // Shut down ADC first 
  ADCSRA = 0;

  // Write a 1 to bit 0 ( PRADC ) in PPR to disable power to ADC 
  PRR |= 0b00000001;
}

void onStandbyMode () {
  SMCR = 0b00001100;
}

void setup() {
  alexDiagonal = sqrt ( ALEX_LENGTH * ALEX_LENGTH + ALEX_BREADTH * ALEX_BREADTH );
  alexCirc = alexDiagonal * PI;
  
  cli();
  setupEINT();
  setupSerial();
  startSerial();
  setupMotors();
  startMotors();
  enablePullups();
  setupColourSensor();
  initializeState();
  setupUltrasonic();
  //disableADC();
  //onStandbyMode();
  sei();

  initBuffer ( &receiveBuffer, TEMP_BUFFER_SIZE);
  initBuffer ( &transmitBuffer, TEMP_BUFFER_SIZE);

}

void handlePacket(TPacket *packet)
{
  switch (packet->packetType)
  {
    case PACKET_TYPE_COMMAND:
      handleCommand(packet);
      break;

    case PACKET_TYPE_RESPONSE:
      break;

    case PACKET_TYPE_ERROR:
      break;

    case PACKET_TYPE_MESSAGE:
      break;

    case PACKET_TYPE_HELLO:
      break;
  }
}

void loop() {

  TPacket recvPacket; // This holds commands from the Pi

  TResult result = readPacket(&recvPacket);
  if (result == PACKET_OK) {
    handlePacket(&recvPacket);
  } else if (result == PACKET_BAD) {
    sendBadPacket();
  } else if (result == PACKET_CHECKSUM_BAD) {
    sendBadChecksum();
  }

  if ( deltaDist > 0 ) {
    if ( (dir == FORWARD && forwardDist > newDist) ||
         (dir == BACKWARD && reverseDist > newDist) ||
         (dir == STOP) ) {
      deltaDist = 0;
      newDist = 0;
      stop();
    }
  }

  if ( deltaTicks > 0 ) {
    if ( (dir == LEFT && leftReverseTicksTurns > targetTicks) ||
         (dir == RIGHT && rightReverseTicksTurns > targetTicks) ||
         (dir == STOP) ) {
      deltaTicks = 0;
      targetTicks = 0;
      stop();
    }
  }
}
