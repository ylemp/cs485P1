/* MUCH OF THIS CODE IS STOLEN FROM 
 * https://github.com/pickle27/arduino_cmucam2/blob/master/cmucam2/cmucam2.ino
 * http://kevinhughes.ca/2013/07/17/how-to-use-the-cmu-cam2-with-the-arduino/
 * by Kevin Hughes at Queen's University
 */

#include <SoftwareSerial.h>
#include <SoftwareServo.h>


// Microcontroller pin set up
const int RX = 8;
const int TX = 7;
const int RIGHT_ENCODER_PIN = 5;
const int LEFT_ENCODER_PIN = 4;
const int INTERRUPT_1 = 0;
const int INTERRUPT_2 = 1;
const int RIGHT_FRONT_FACING_IR_PIN = 0;
const int RIGHT_FACING_IR_PIN = 1;
const int CENTER_IR_PIN = 2;
const int LEFT_FACING_IR_PIN = 3;
const int LEFT_FRONT_FACING_IR_PIN = 4;

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
SoftwareSerial cmucam(RX, TX);
unsigned char RcvData[8] = "";
unsigned char packet[8];

/* 
 * Function to print the data packet
 */
void print_packet(unsigned char * packet)
{
  for(int i = 0; i < 8; i++)
  {
     Serial.print( (int)packet[i] );
     Serial.print(" "); 
  }
  Serial.println();
}

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
  cmucam.print("\r");
  cmucam.listen();

  boolean ack = false;

  if(verbose)
    Serial.print("++> ");

  // get the response
  while( cmucam.available() > 0 ) 
  {
    char inbyte = cmucam.read();
    
    if(inbyte == ':')
      ack = true;  

    if(verbose)
      Serial.write(inbyte);
  }

  if(verbose)
    Serial.println();

  // flush
  while( cmucam.available() > 0 )
    cmucam.read();

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
  cmucam.print("\r");
  cmucam.listen();

  // Debug Packet
  // Change to true and
  // turn of raw mode "RM 0"
  // to see the packet
  boolean debug = false;
  if(debug) 
  {
    while(cmucam.available() > 0)
    {
      delay(100);
      Serial.print((char)cmucam.read());    
    }
    Serial.println();
    
    return true;
  }

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

  // T-Packet
  // raw mode must be on
  if(packet == 'T')
  {
    while(cmucam.read() != 255); // wait for signal bit
    while(cmucam.read() != 84); 
    while(cmucam.available() < 8); // wait for data
    for(int i = 0; i < 8; i++) // read the packet
    {
      rtrn[i] = cmucam.read();    
    }
  }

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
  rightWheel.attach(10);
  leftWheel.attach(11);
  
  // CMU cam 2 init  
  cmucam.begin(9600);
  cmucam.print("RS"); 
  cmucam.print("\r");
  cmucam.print("RS"); 
  cmucam.print("\r");
  cmucam.listen();
  while( cmucam.available() > 0 ) 
  {
    cmucam.read();
  }
  delay(100);
  while(!cmucam2_set("RS", true));
  // End Init CMU Cam2

  // Turn OFF Auto Gain
  while(!cmucam2_set("CR 19 33", true));
	
  // Turn OFF Auto White Balance (this is unnecessary since it's the default)
  while(!cmucam2_set("CR 18 40", true));
	
  // Turn ON Poll Mode
  while(!cmucam2_set("PM 1", true));

  // Turn ON Raw Mode
  while(!cmucam2_set("RM 1", true));

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
 
  //CODE FOR CONE TRACKING
  // Read incoming value from packet 6 (packet 6 = can I see ANY pixels I want?)
  cIR = analogRead(CENTER_IR_PIN);
  
   if (rightWheel.attached() == 1){
    if(packet[6] > 0 && cIR >= 250){
      //If the cone is immediately infront of me
      rightWheel.detach();
      leftWheel.detach();      
     }
    else if(packet[6] > 0 && cIR < 250){
      // If I can, drive straight
      rightWheel.write(60);
      leftWheel.write(125);   
    }
    else{
      // No blob found start looking for a blob
      tickCounter = rightWW;
      tickCounter2 = tickCounter + 15;
        while(tickCounter<tickCounter2){
          rightWheel.write(30);
          leftWheel.write(0);
          tickCounter++;
       }
    }      
  }
  else{
    if(packet[6] > 0 && cIR >= 250){
      rightWheel.detach();
      leftWheel.detach();
    }
    else if(packet[6] > 0 && cIR < 250){
      // If I can, drive straight with no cone infront

      // Attach servos
      rightWheel.attach(10);
      leftWheel.attach(11);
      
      rightWheel.write(60);
      leftWheel.write(125);   
    }
    else{
      // No blob found start looking for a blob
      tickCounter = rightWW;
      tickCounter2 = tickCounter + 15;    
      stopped = 0;
      // Attach servos
      rightWheel.attach(10);
      leftWheel.attach(11);
        while(tickCounter<tickCounter2){
          rightWheel.write(30);
          leftWheel.write(0);
          tickCounter++;
       }
    } 
  }



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
  Serial.print(" ");
  Serial.print(rffIR, DEC);    // right front facing ir 
  Serial.print(" ");
  Serial.print(rfIR, DEC);    // right front ir 
  Serial.print(" ");
  */
  Serial.print(cIR, DEC);    // center ir 
  Serial.println(" ");
  /*
  Serial.print(lfIR, DEC);    // left facing ir 
  Serial.print(" ");
  Serial.println(lffIR, DEC);    // left front facing ir 
  */
}


