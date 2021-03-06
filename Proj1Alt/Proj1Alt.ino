

/***
****  INSTRUCTIONS FOR USE OF THIS FILE
****  This file is meant for the AltSoftSerial library.  This library requires
****  that the transmit pin (green wire) be moved to pin *9* (displacing the camera servo),
****  and that the right wheel servo (pin 10) be moved to pin *6* (displacing the
****  gripper servo)
****
****  You also have to install the AltSoftSerial library which you can get here:
****  https://www.pjrc.com/teensy/arduino_libraries/AltSoftSerial.zip
****
****  After this change, you will find Robot 2 to work great.  Maybe other robots
****  as well!  -- Sean
****
*/



/* MUCH OF THIS CODE IS STOLEN FROM 
 * https://github.com/pickle27/arduino_cmucam2/blob/master/cmucam2/cmucam2.ino
 * http://kevinhughes.ca/2013/07/17/how-to-use-the-cmu-cam2-with-the-arduino/
 * by Kevin Hughes at Queen's University
 */

#include <SoftwareSerial.h>
#include <SoftwareServo.h>
#include <AltSoftSerial.h>

#define ALTSERIAL 1

// Microcontroller pin set up
const int RX = 8;
const int TX = 9;
const int RIGHT_ENCODER_PIN = 5;
const int LEFT_ENCODER_PIN = 4;
const int INTERRUPT_1 = 0;
const int INTERRUPT_2 = 1;
const int RIGHT_FRONT_FACING_IR_PIN = 0;
const int RIGHT_FACING_IR_PIN = 1;
const int CENTER_IR_PIN = 2;
const int LEFT_FACING_IR_PIN = 3;
const int LEFT_FRONT_FACING_IR_PIN = 4;

int wallFound = 0;

int left_turn_left = 99;
int left_turn_right = 40;

int right_turn_left = 140;
int right_turn_right = 81;

int move_foward_left = 100 ;
int move_foward_right = 80;

int rffIR = 0;
int rfIR = 0;
int cIR = 0;
int lfIR = 0;
int lffIR = 0;

// Global variables to count wheel watcher tics
int rightWW, leftWW;

int tickCounter, tickCounter2;

// Create servo objects for lw/rw
SoftwareServo rightWheel;
SoftwareServo leftWheel;

// Serial comm set up for CMU cam 2
#ifdef ALTSERIAL
AltSoftSerial cmucam; 
#else
SoftwareSerial cmucam(RX,TX);
#endif
unsigned char RcvData[8] = "";
unsigned char packet[8];





/* 
 * Function for sending commands to the CMU Cam2
 * where no return data aside from the 'ACK' 
 * command is expected. The verbose option
 * will print out exactly what is sent and 
 * recieved from the CMU Cam2
 */
boolean cmucam2_set(char* cmd, boolean verbose=false)
{
  if(verbose)
  {
    Serial.print("Sending CMU Cam2 Command: ");
    Serial.print(cmd);
    Serial.println();
  }
  // send the command
  cmucam.print(cmd);
  //cmucam.print("\r");
  cmucam.write(13);
  cmucam.flush();

  boolean ack = false;

    if(verbose)
      {
      Serial.print("++>");
      }


#define ___ 0
#define __A 1
#define __C 2
#define __K 3
#define __R 4
#define __O 5

  int v = ___;

  // get the response
  delay(25);
  while( cmucam.available() > 0 ) 
  {
    char inbyte = cmucam.read();

    if (inbyte == 'A')
      {
      v = __A;
      }
    else if (inbyte == 'C' && v == __A)
      {
      v = __C;
      }
    else if (inbyte == 'K' && v == __C)
      {
      v = __K;
      }
    else if (inbyte == '\r' && v == __K)
      {
      v = __R;
      }
    else if (inbyte == ':' && v == __R)
      {
      ack = true;
      v = __O ;
      }
    else if (v == __R)
      {
      }
    else 
      {
        //Serial.write("?");
        //Serial.print(inbyte, DEC);
        //Serial.write("!");
        v = ___;
      }
    
    if(verbose)
      {
      Serial.write(inbyte);
      Serial.print(v, DEC);
      }
   delay(5);
   }

  if(verbose)
    Serial.println();

  // flush
  while( cmucam.available() > 0 )
    cmucam.read();

 // if (cmucam.overflow())
 //  Serial.println("SoftwareSerial overflow!"); 

  return ack;
}

/* 
 * Function for sending commands to the CMU Cam2
 * where return data is expected. The packet type
 * must be specified, currently only S and T packets
 * are supported. This code expects the camera to be in
 * raw mode and still sending 'ACK'. The rtrn buffer 
 * must be at least 8 bytes for T packets and 
 * at least 6 bytes for S packets. 
 * The verbose option will print out exactly what 
 * is sent and recieved from the CMU Cam2
 */
boolean cmucam2_get(char* cmd, char packet, unsigned char *rtrn, boolean verbose=false)
{
  if(verbose)
  {
    Serial.print("Sending CMU Cam2 GET Command: ");
    Serial.print(cmd);
    Serial.println();
  }

  // send the command
  cmucam.print(cmd);
  cmucam.write(13);
  //cmucam.print("\r");
  cmucam.flush();
  //cmucam.listen();

  delay(25);

  // S-Packet
  // raw mode must be on
  if(packet == 'S')
  {
    while(cmucam.read() != 255); // wait for signal bit
    while(cmucam.read() != 83);
    while(cmucam.available() < 6); // wait for data
    for(int i = 0; i < 6; i++) // read the packet
    {
      rtrn[i] = cmucam.read();    
    }
  }

#define ___ (-2)
#define __T (-1)

  // T-Packet
  // raw mode must be on
  int v = ___;
  
  if(packet == 'T')
  {
    while( cmucam.available() > 0 ) 
    {
      char inbyte = cmucam.read();
      if (inbyte == 255 && v == ___)
        {
          v = __T;
        }
      else if (inbyte == 'T' && v == __T)
        {
          v = 0;
        }
      else if (v >= 0 && v < 8)
        {
        rtrn[v] = inbyte;
        v++;
        }
      else
        {
          v = ___;  // error
        }
    }
    if (v == 8)
      return true;
    else 
      return false;
  }
      
//    while(cmucam.read() != 255); // wait for signal bit
//    while(cmucam.read() != 84); 
//    while(cmucam.available() < 8); // wait for data
//    for(int i = 0; i < 8; i++) // read the packet
//    {
//      rtrn[i] = cmucam.read();    
//    }
//  }

  //if (cmucam.overflow())
  // Serial.println("SoftwareSerial overflow!"); 

  return true;  
  }



void _updateRightEncoder() {
  int8_t adjustment = (((digitalRead(RIGHT_ENCODER_PIN) ^ 0)<<1)-1);
  rightWW += adjustment;
}

void _updateLeftEncoder() {
  int8_t adjustment = ((((!digitalRead(LEFT_ENCODER_PIN)) ^ 0)<<1)-1);
  leftWW += adjustment;
}

void setup()
{
  Serial.begin(115200);
    
  // Attach the wheel watchers
  attachInterrupt(INTERRUPT_1, _updateLeftEncoder, FALLING); 
  attachInterrupt(INTERRUPT_2, _updateRightEncoder, FALLING);  
  
  // Attach servos
#ifdef ALTSERIAL
  rightWheel.attach(6);
#else
  rightWheel.attach(10);
#endif
  leftWheel.attach(11);
  
  // CMU cam 2 init  
  cmucam.begin(57600);
  cmucam.write("\xff") ;  // write a dummy character to fix setTX bug
  delay(500);
  cmucam.listen();
  cmucam.print("RS"); 
  cmucam.print("\r");
  cmucam.print("RS"); 
  cmucam.print("\r");
  
  delay(25);
  while( cmucam.available() > 0 ) 
  {
    cmucam.read();
    delay(5);
  }

  while(!cmucam2_set("RS", true));
  Serial.println("RS Done");
  // End Init CMU Cam2

  // Turn OFF Auto Gain
  while(!cmucam2_set("CR 19 33", true));
  Serial.println("CR 19 33 Done");
	
  // Turn OFF Auto White Balance (this is unnecessary since it's the default)
  while(!cmucam2_set("CR 18 40", true));
  Serial.println("CR 18 40 Done");
	
  // Turn ON Poll Mode
  while(!cmucam2_set("PM 1", true));
  Serial.println("PM 1 Done");

  // Turn ON Raw Mode
  while(!cmucam2_set("RM 1", true));
  Serial.println("RM 1 Done");

}

void loop()
{
  SoftwareServo::refresh();
  // You must have this. This function needs to be called every 50ms in order to write new values to the servos.
  // If your loop function gets to big you will need to add more refresh()

  // To get a blob tracking packet from the camera,
  // we send the following command which says to look
  // for blobs whose colors are between 200 and 240 RED,
  // between 0 (really 16) and 40 BLUE, and between
  // 0 (really 16) and 40 GREEN. 
  // The data comes back as eight bytes describing a rectangular
  // bounding box around the blob.
  // Mean x, Mean y, Min x, Min y, Max x, Max y, 
  // num pixels in the rectangle which belong to the blob, 
  // and "confidence" (basically a measure how much of the
  // rectangle is filled by pixels belonging to the blob).
        
  // The camera is rotated 90 degrees, so use the Y value.
  // Low Y values indicate the blob is to the left of the
  // view, high Y values indicate the blob is to the right of the view.
  // You want a blob with a moderate confidence (perhaps over 20?)
  // Else it's probably just noise.
        
  cmucam2_get("TC 200 240 0 40 0 40", 'T', packet, false);
  
  //WALL FOLLOWING
  rffIR = analogRead(RIGHT_FRONT_FACING_IR_PIN);
  rfIR = analogRead(RIGHT_FACING_IR_PIN);
  cIR = analogRead(CENTER_IR_PIN);
  lfIR = analogRead(LEFT_FACING_IR_PIN);
  lffIR = analogRead(LEFT_FRONT_FACING_IR_PIN);
  
  
  
  /*
  Serial.println(rffIR, DEC);
  Serial.println(rfIR, DEC);
  Serial.println(cIR, DEC);
  Serial.println(lfIR, DEC);
  Serial.println(lffIR, DEC);
  */

  if(rffIR >= 250 && lfIR < 150){
  //if the right front finds a wall and left doesnt
    Serial.print("rffIR: ");
    Serial.println(rffIR, DEC);
    Serial.print("lfIR: ");
    Serial.println(lfIR, DEC);
    Serial.println(" ");
  }
  if(lffIR >= 250 && rfIR < 150){
  //if the left front finds a wall and the right doesnt
    //Serial.println(" ");
  }

  
  if(wallFound == 1 && cIR >= 150){
   //if following a wall and encounters an inside curve 
      rightWheel.write(100);
      leftWheel.write(120);
    }
  
  if(rffIR >= 200 && lfIR >= 150 && cIR < 150){
  //if right front and the left find a wall
    //Serial.println(" ");
    wallFound = 1;
    rightWheel.write(move_foward_right);
    leftWheel.write(move_foward_left); 
  }
  if(lffIR >= 250 && rfIR >= 150){
  //if the left front and the right find a wall
    //Serial.println(" ");


  }
  if(rffIR < 250 && lfIR >= 150){
  //if on the left finds a wall and the right front doesnt
    //Serial.println(" ");
    
    rightWheel.write(move_foward_right);
    leftWheel.write(move_foward_left);

  }
  if(lffIR < 250 && rfIR >= 150){
  //if only the right finds a wall and the left front doesnt
    //Serial.println(" ");

  }

  
  if(rffIR > 250 && lfIR >= 150){
      wallFound = 1;
      rightWheel.write(move_foward_right);
      leftWheel.write(move_foward_left);
  }
  
  if(rffIR < 250 && lffIR < 250 && lfIR < 150 && rfIR < 150){
    //niether front finds a wall
    //Serial.println(" ");
    //rightWheel.write(60);
    //leftWheel.write(120); 
    
    //and hasnt found a wall yet
    if(wallFound == 0){
      //search for wall
      
      rightWheel.write(move_foward_right);
      leftWheel.write(move_foward_left);
    }
    
    //and has already found a wall
    if(wallFound == 1){
      //turn left
      
      tickCounter = rightWW;
      tickCounter2 = tickCounter + 15;
      
      while(tickCounter < tickCounter2){
        rightWheel.write(left_turn_right);
        leftWheel.write(left_turn_left);
        tickCounter++;
      }
      

    }
  }
   if(rffIR > 300 || lfIR > 200){
     
     tickCounter = rightWW;
     tickCounter2 = tickCounter + 15;
      
      while(tickCounter < tickCounter2){
        rightWheel.write(right_turn_right);
        leftWheel.write(right_turn_left);
        tickCounter++;
   }
        
  }
  
  /*
  if(rffIR < 250 && lffIR < 150 ){
    //niether front finds a wall
    //Serial.println(" ");
    //rightWheel.write(60);
    //leftWheel.write(120); 
    
    //and hasnt found a wall yet
    if(wallFound == 0){
      //search for wall
      
      rightWheel.write(60);
      leftWheel.write(120);
    }
    
    //and has already found a wall
    if(wallFound == 1){
      //turn left
      
      tickCounter = rightWW;
      tickCounter2 = tickCounter + 15;
      
      while(tickCounter < tickCounter2){
        rightWheel.write(40);
        leftWheel.write(99);
        tickCounter++;
      }
      

    }
        
  }*/
 

  // Read values from IR sensors
  rffIR = analogRead(RIGHT_FRONT_FACING_IR_PIN);
  rfIR = analogRead(RIGHT_FACING_IR_PIN);
  cIR = analogRead(CENTER_IR_PIN);
  lfIR = analogRead(LEFT_FACING_IR_PIN);
  lffIR = analogRead(LEFT_FRONT_FACING_IR_PIN);
  
  // Here is some debugging code which will print out the packets
  // received.
  /*
  
  
  Serial.print(packet[0], DEC);    // MEAN X
  Serial.print(" ");
  Serial.print(packet[1], DEC);    // MEAN Y
  Serial.print(" ");
  Serial.print(packet[2], DEC);    // MIN X
  Serial.print(" ");
  Serial.print(packet[3], DEC);    // MIN Y
  Serial.print(" ");
  Serial.print(packet[4], DEC);    // MAX X
  Serial.print(" ");
  Serial.print(packet[5], DEC);    // MAX Y
  Serial.print(" ");
  Serial.print(packet[6], DEC);    // NUM PIXELS
  Serial.print(" ");
  Serial.print(packet[7], DEC);    // CONFIDENCE
  Serial.print("LW ");
  Serial.print(leftWW, DEC);       // left wheel ticks
  Serial.print(" RW ");
  Serial.print(rightWW, DEC);    // right wheel ticks
  */
  //Serial.print("right front facing ir ");
  //Serial.println(rffIR, DEC);    // right front facing ir 
 /* Serial.print("right front ir ");
  Serial.print(rfIR, DEC);    // right front ir 
  Serial.print("center ir ");
  
  Serial.print(cIR, DEC);    // center ir 
  Serial.println("left facing ir ");

  Serial.print(lfIR, DEC);    // left facing ir 
  Serial.print("left front facing ir ");
  Serial.println(lffIR, DEC);    // left front facing ir 
  */
}


