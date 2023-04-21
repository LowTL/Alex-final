#include <serialize.h>
#include <stdarg.h>
#include "packet.h"
#include "constants.h"
#include <math.h>

typedef enum 
{
  STOP = 0,
  FORWARD = 1,
  BACKWARD = 2,
  LEFT = 3,
  RIGHT = 4,
} TDirection;

volatile TDirection dir = STOP;

/*
 * Alex's configuration constants
 */

// Number of ticks per revolution from the 
// wheel encoder.

#define COUNTS_PER_REV      193

// Wheel circumference in cm.
// We will use this to calculate forward/backward distance traveled 
// by taking revs * WHEEL_CIRC

#define WHEEL_CIRC          20.4

// Motor control pins. You need to adjust these till
// Alex moves in the correct direction
#define RR                  6   // Left forward pin
#define RF                  5   // Left reverse pin
#define LR                  10  // Right forward pin
#define LF                  11  // Right reverse pin

// PI, for calculating turn circumference
#define PI 3.141592654

// Alex's length and breadth in cm
#define ALEX_LENGTH 17.2
#define ALEX_BREADTH 12.1

//Pins wiring
#define S0 4
#define S1 12
#define S2 8
#define S3 7
#define sensorOut 9

long redF = 0;
long greenF = 0;
long blueF = 0;

//to Calibrate
long redColor = 0;
long blueColor = 0;
long greenColor = 0;

// Alex's diagonal. We compute and store this once.
// Since it is expensive to compute and really doesn't change.
float alexDiagonal = 0.0; 

// Alex's turning circumference, calculated once
float alexCirc = 0.0;
/*
 *    Alex's State Variables
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
volatile unsigned long forwardDist = 0;
volatile unsigned long reverseDist = 0;

// variables to keep track of whether we have moved a commanded distance
unsigned long deltaDist;
unsigned long newDist; 

// variables to keep track of turning angle 
unsigned long deltaTicks;
unsigned long targetTicks;

/*
 * 
 * Alex Communication Routines.
 * 
 */
 
TResult readPacket(TPacket *packet)
{
    // Reads in data from the serial port and
    // deserializes it.Returns deserialized
    // data in "packet".
    
    char buffer[PACKET_SIZE];
    int len;

    len = readSerial(buffer);

    if(len == 0)
      return PACKET_INCOMPLETE;
    else
      return deserialize(buffer, len, packet);
    
}

void senseColor() {  
  //set frequency scaling to 20%
  PORTD |= 0b10000; //pin4/S0 is HIGH
  PORTB &= ~0b10000; //pin 12/S0 is LOW
  
  delay(500);
  //TODO: calibrate RGB values using map function

  PORTB &= ~1; //set pin 7 to LOW;
  PORTD &= ~128; //set pin 8 to LOW;
  redF = pulseIn(sensorOut, LOW);
  //measures red frequency
  //MAYBE DELAY
  PORTB |= 1; //sets pin 8 to HIGH
  PORTD |= 0b10000000; //seta pin 7 to HIGH
  greenF = pulseIn(sensorOut, LOW);
  
  PORTB &= 0b11111110; //sets pin 8 to LOW
  PORTD |= 0b10000000; //sets pin 7 to HIGH
  blueF = pulseIn(sensorOut, LOW);
  
  char *color;
  if (redF < 200 && greenF > 200) {
    color = "red";
    sendColor(color);
  } else if (greenF < 200 && redF > 200) {
    color = "green";
    sendColor(color);
    sendColor(color);
  } else {
    color = "retry";
    sendColor(color);
    sendColor(color);
    sendColor(color);
  }
  


}

void sendStatus()
{
  // Implement code to send back a packet containing key
  // information like leftTicks, rightTicks, leftRevs, rightRevs
  // forwardDist and reverseDist
  // Use the params array to store this information, and set the
  // packetType and command files accordingly, then use sendResponse
  // to send out the packet. See sendMessage on how to use sendResponse.
  //
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
  sendResponse(&statusPacket);
}

void sendMessage(const char *message)
{
  // Sends text messages back to the Pi. Useful
  // for debugging.
  
  TPacket messagePacket;
  messagePacket.packetType=PACKET_TYPE_MESSAGE;
  strncpy(messagePacket.data, message, MAX_STR_LEN);
  sendResponse(&messagePacket);
} 

void dbprintf(char *format, ...) {
  va_list args;
  char buffer[128];
  
  va_start(args, format);
  vsprintf(buffer, format, args);
  sendMessage(buffer);

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
  badCommand.packetType=PACKET_TYPE_ERROR;
  badCommand.command=RESP_BAD_COMMAND;
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

void sendColor(const char *color) {
  TPacket colorPacket;
  colorPacket.packetType = PACKET_TYPE_MESSAGE;
  strncpy(colorPacket.data, color, MAX_STR_LEN);
  sendResponse(&colorPacket);
} 
 
void sendResponse(TPacket *packet)
{
  // Takes a packet, serializes it then sends it out
  // over the serial port.
  char buffer[PACKET_SIZE];
  int len;

  len = serialize(buffer, packet, sizeof(TPacket));
  writeSerial(buffer, len);
}

/*
 * Setup and start codes for external interrupts and 
 * pullup resistors.
 * 
 */
// Enable pull up resistors on pins 2 and 3
void enablePullups()
{
  // Use bare-metal to enable the pull-up resistors on pins
  // 2 and 3. These are pins PD2 and PD3 respectively.
  // We set bits 2 and 3 in DDRD to 0 to make them inputs. 
  DDRD &= 0b11110011;
  PORTD |= 0b1100;
}

// Functions to be called by INT0 and INT1 ISRs.
void leftISR()
{
  if(dir == FORWARD){
    leftForwardTicks ++;
    forwardDist = (unsigned long) ((float) leftForwardTicks / COUNTS_PER_REV * WHEEL_CIRC);                                                                    
  }
  else if(dir == BACKWARD){
    leftReverseTicks++;
    reverseDist = (unsigned long) ((float) leftReverseTicks / COUNTS_PER_REV * WHEEL_CIRC);
  }
  else if (dir == LEFT) {
    leftReverseTicksTurns ++;
  }
  else if (dir == RIGHT) {
    leftForwardTicksTurns ++;
  }
}

void rightISR()
{
  if(dir == FORWARD){
    rightForwardTicks ++;
  }
  else if(dir == BACKWARD){
    rightReverseTicks++;
  }
  else if (dir == LEFT) {
    rightForwardTicksTurns ++;
  }
  else if (dir == RIGHT) {
    rightReverseTicksTurns ++;
  }
}

// Set up the external interrupt pins INT0 and INT1
// for falling edge triggered. Use bare-metal.
void setupEINT()
{
  // Use bare-metal to configure pins 2 and 3 to be
  // falling edge triggered. Remember to enable
  // the INT0 and INT1 interrupts.
  EIMSK = 0b11; 
  EICRA = 0b1010;
  DDRD &= 0b11110011;
}

void setupColorSensor() {
  DDRD |= 0b10010000; //sets pins 4 and 7 to OUTPUT
  DDRB |= 0b00010001; //sets pins 8 and 12 to OUTPUT
  DDRB &= 0b11111101; //sets Pn 9 to INPUT
  
  PORTD &= 0b11101111;
  PORTB &= 0b11101111; //turn off S0 & S1

}

// Implement the external interrupt ISRs below.
// INT0 ISR should call leftISR while INT1 ISR
// should call rightISR.
ISR(INT0_vect){
  leftISR();
}

ISR(INT1_vect){
  rightISR();
}



// Implement INT0 and INT1 ISRs above.

/*
 * Setup and start codes for serial communications
 * 
 */
// Set up the serial connection. For now we are using 
// Arduino Wiring, you will replace this later
// with bare-metal code.
void setupSerial()
{
  // To replace later with bare-metal.
  UBRR0L = 103;
  UBRR0H = 0;
  UCSR0C = 0b00000110;
  UCSR0A = 0;
}

// Start the serial connection. For now we are using
// Arduino wiring and this function is empty. We will
// replace this later with bare-metal code.

void startSerial()
{
  // Empty for now. To be replaced with bare-metal code
  // later on.
  UCSR0B = 0b10111000;
}

// Read the serial port. Returns the read character in
// ch if available. Also returns TRUE if ch is valid. 
// This will be replaced later with bare-metal code.

int readSerial(char *buffer)
{
/*  int count = 0;
  while (!(UCSR0A & (1 << RXC0)));
  while (1) {
    if (UCSR0A & (1 << RXC0)) {
      buffer[count++] = UDR0;
    }
  }*/ 
  int count=0;
  while(Serial.available())
    buffer[count++] = Serial.read();

  return count;
}

// Write to the serial port. Replaced later with
// bare-metal code

void writeSerial(const char *buffer, int len)
{
//  for (int i = 0; i < len; i++) {
//    while (!(UCSR0A & (1 << UDRE0)));
//    UDR0 = buffer[i];
//  }
  Serial.write(buffer, len);
}

/*
 * Alex's motor drivers.
 * 
 */

// Set up Alex's motors. Right now this is empty, but
// later you will replace it with code to set up the PWMs
// to drive the motors.
void setupMotors()
{
  TCNT0 = 0;
  TCNT1 = 0;
  TCNT2 = 0;
  DDRD |= 0b01100000;
  TCCR0A |= 0b10100001;
  TCCR2A |= 0b10000001;
  TCCR1A |= 0b00100001;
  DDRB |= 0b1100;
  /* Our motor set up is:  
   *    A1IN - Pin 5, PD5, OC0B
   *    A2IN - Pin 6, PD6, OC0A
   *    B1IN - Pin 10, PB2, OC1B
   *    B2In - pIN 11, PB3, OC2A
   */
//#define LF                  6   // Left forward pin
//#define LR                  5   // Left reverse pin
//#define RF                  10  // Right forward pin
//#define RR                  11  // Right reverse pin
}

// Start the PWM for Alex's motors.
// We will implement this later. For now it is
// blank.
void startMotors()
{
  TCCR0B |= 0b11;
  TCCR1B |= 0b11;
  TCCR2B |= 0b11; 
}

// Convert percentages to PWM values
int pwmVal(float speed)
{
  if(speed < 0.0)
    speed = 0;

  if(speed > 100.0)
    speed = 100.0;

  return (int) ((speed / 100.0) * 255.0);
}

void forward(float dist, float speed)
{
  /*if(dist == 0){
    deltaDist = 999999;
  }else{
    deltaDist = dist;
  }
  newDist = forwardDist + deltaDist;*/
  dir = FORWARD;
  int val = pwmVal(speed);
//  analogWrite(LF, val);
//  analogWrite(RF, val);
//  analogWrite(LR,0);
//  analogWrite(RR, 0);
  // Left Forward
  OCR0A = 0;
  // Right Forward
  OCR1B = 0;
  // Left Reverse
  OCR0B = val;
  // Right Reverse
  OCR2A = val * 0.8;
  delay(200);
  stop();
  
}

void newForward(float dist, float velo) {
  if(dist == 0){
    deltaDist = 999999;
  }else{
    deltaDist = dist;
  }
  newDist = forwardDist + deltaDist;
  dir = FORWARD;
  int val = pwmVal(velo);
//  analogWrite(LF, val);
//  analogWrite(RF, val);
//  analogWrite(LR,0);
//  analogWrite(RR, 0);
  // Left Forward
  OCR0A = 0;
  // Right Forward
  OCR1B = 0;
  // Left Reverse
  OCR0B = val;
  // Right Reverse
  OCR2A = val;
}

void reverse(float dist, float speed)
{
  /*if(dist == 0){
    deltaDist = 999999;
  }else{
    deltaDist = dist;
  }
  newDist = reverseDist + deltaDist;*/
  dir = BACKWARD;
  int val = pwmVal(speed);
//  analogWrite(LR, val);
//  analogWrite(RR, val);
//  analogWrite(LF, 0);
//  analogWrite(RF, 0);

    // Left Forward
  OCR0A = val;
  // Right Forward
  OCR1B = val;
  // Left Reverse
  OCR0B = 0;
  // Right Reverse
  OCR2A = 0;
  delay(200);
  stop();
}

unsigned long computeDeltaTicks(float ang){
  unsigned long ticks = (unsigned long) ((ang * alexCirc * COUNTS_PER_REV) / (360.0 * WHEEL_CIRC));
  return ticks; 
}

// Turn Alex left "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Alex to
// turn left indefinitely.
void left(float ang, float speed)
{
  ang /= 2;
  dir = LEFT;
  /*if(ang == 0){
    deltaTicks = 99999999;
  }else{
    deltaTicks = computeDeltaTicks(ang);
  }
  targetTicks = leftReverseTicksTurns + deltaTicks;*/
  int val = pwmVal(speed);
//  analogWrite(LR, val);
//  analogWrite(RF, val);
//  analogWrite(LF, 0);
//  analogWrite(RR, 0);

      // Left Forward
  OCR0A = 0;
  // Right Forward
  OCR1B = val;
  // Left Reverse
  OCR0B = val;
  // Right Reverse
  OCR2A = 0;
  delay(200);
  stop();
}

// Turn Alex right "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Alex to
// turn right indefinitely.
void right(float ang, float speed)
{
  ang /= 3;
  dir = RIGHT;
  /*if(ang == 0){
    deltaTicks = 99999999;
  }else{
    deltaTicks = computeDeltaTicks(ang);
  }
  targetTicks = rightReverseTicksTurns + deltaTicks;*/
  int val = pwmVal(speed);
//  analogWrite(RR, val);
//  analogWrite(LF, val);
//  analogWrite(LR, 0);
//  analogWrite(RF, 0);
  // Left Forward
  OCR0A = val;
  // Right Forward
  OCR1B = 0;
  // Left Reverse
  OCR0B = 0;
  // Right Reverse
  OCR2A = val;
  delay(200);
  stop();
  
}

// Stop Alex. To replace with bare-metal code later.
void stop()
{
  dir = STOP;
//  analogWrite(LF, 0);
//  analogWrite(LR, 0);
//  analogWrite(RF, 0);
//  analogWrite(RR, 0);
  // Left Forward
  OCR0A = 0;
  // Right Forward
  OCR1B = 0;
  // Left Reverse
  OCR0B = 0;
  // Right Reverse
  OCR2A = 0;
}

/*
 * Alex's setup and run codes
 * 
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
  
  leftRevs=0;
  rightRevs=0;
  forwardDist=0;
  reverseDist=0; 
}

// Clears one particular counter
void clearOneCounter(int which)
{
  clearCounters();
//  switch(which)
//  {
//    case 0:
//      clearCounters();
//      break;
//
//    case 1:
//      leftTicks=0;
//      break;
//
//    case 2:
//      rightTicks=0;
//      break;
//
//    case 3:
//      leftRevs=0;
//      break;
//
//    case 4:
//      rightRevs=0;
//      break;
//
//    case 5:
//      forwardDist=0;
//      break;
//
//    case 6:
//      reverseDist=0;
//      break;
//  }
}
// Intialize Vincet's internal states

void initializeState()
{
  clearCounters();
}

void handleCommand(TPacket *command)
{
  switch(command->command)
  {
    // For movement commands, param[0] = distance, param[1] = speed.

    case COMMAND_STOP:
        sendOK();
        stop();
      break;
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
    case COMMAND_GET_STATS:
        sendStatus();
      break;
    case COMMAND_CLEAR_STATS:
        clearOneCounter(command->params[0]);
        sendOK();
      break;
    case COMMAND_SENSE_COLOR:
        sendOK();
        senseColor();
      break;
    case COMMAND_FORWARD_2:
      sendOK();
      newForward((float) command->params[0], (float) command->params[1]);
      break;      
      
     default:
      sendBadCommand();
  }
}

void waitForHello()
{
  int exit=0;

  while(!exit)
  {
    TPacket hello;
    TResult result;
    
    do
    {
      result = readPacket(&hello);
    } while (result == PACKET_INCOMPLETE);

    if(result == PACKET_OK)
    {
      if(hello.packetType == PACKET_TYPE_HELLO)
      {
     

        sendOK();
        exit=1;
      }
      else
        sendBadResponse();
    }
    else
      if(result == PACKET_BAD)
      {
        sendBadPacket();
      }
      else
        if(result == PACKET_CHECKSUM_BAD)
          sendBadChecksum();
  } // !exit
}

void setup() {
  // put your setup code here, to run once:

  alexDiagonal = sqrt((ALEX_LENGTH * ALEX_LENGTH) + (ALEX_BREADTH * ALEX_BREADTH));
  alexCirc = PI * alexDiagonal; 
  cli();
  setupEINT();
  setupSerial();
  startSerial();
  setupMotors();
  setupColorSensor();
  startMotors();
  enablePullups();
  initializeState();
  sei();
  waitForHello();
  
}

void handlePacket(TPacket *packet)
{
  switch(packet->packetType)
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

// Uncomment the code below for Step 2 of Activity 3 in Week 8 Studio 2

//forward(0, 100);

// Uncomment the code below for Week 9 Studio 2

 // put your main code here, to run repeatedly:
  TPacket recvPacket; // This holds commands from the Pi

  TResult result = readPacket(&recvPacket);
  
  if(result == PACKET_OK)
    handlePacket(&recvPacket);
  else
    if(result == PACKET_BAD)
    {
      sendBadPacket();
    }
    else
      if(result == PACKET_CHECKSUM_BAD)
      {
        sendBadChecksum();
      }
  if (deltaDist > 0) 
  {
    if (dir==FORWARD) 
    {
      if(forwardDist > newDist)
      {
        deltaDist = 0;
        newDist = 0;
        stop();
      }
    }
    else 
    if (dir == BACKWARD) 
    {
    /*  if (reverseDist > newDist) 
      {
        deltaDist = 0;
        newDist = 0;
        stop();
      }*/
    }
    else
      if (dir==STOP)
      {
        deltaDist = 0;
        newDist = 0;
        stop();
      }
  }
  if(deltaTicks > 0){
    if(dir == LEFT){
      /*if(leftReverseTicksTurns >= targetTicks){
        deltaTicks = 0;
        targetTicks = 0; 
        stop();
      }*/
    }else
      if(dir == RIGHT){
        /*if(rightReverseTicksTurns >= targetTicks){
        deltaTicks = 0;
        targetTicks = 0; 
        stop();
      }*/
    }else
      if(dir == STOP){
        deltaTicks = 0;
        targetTicks = 0; 
        stop();
      }
  }
}
