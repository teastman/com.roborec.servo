#include <MaskNode.h>
#include <MultiServoController.h>
#include <PortMaskNode.h>
#include <Servo.h>
#include <ServoNode.h>


// Serial port communication variables;
char serialValue;
String serialString;

// Servos
int active_servos = 1;
int const NUM_SERVOS = 18;
Servo* servo[NUM_SERVOS];
int pattern = 1;

void setup()
{
  // Start listening on the serial port for commands
  Serial.begin(57600);
 
  DDRD = B11111100;
  DDRB = B00111111;
  DDRC = B00111111;
  
 // Servo s = new Servo();
  
  for(int i = 0; i < NUM_SERVOS; i++)
  {
    servo[i] = new Servo();
    
    if(i < 6)
    {
      attach(servo[i], &PORTD, 1<<(i+2));
    }
    else if(i < 12)
    {
      attach(servo[i], &PORTB, 1<<(i-6));
    }
    else if(i < 18)
    {
      attach(servo[i], &PORTC, 1<<(i-12));
    }
    
    servo[i]->rotateTo(0);
  }
  
  Serial.println("Setup Complete."); 
}

void printHex(int num, int precision) {
	char tmp[16];
	char format[128];

	sprintf(format, "0x%%.%dX", precision);

	sprintf(tmp, format, num);
	Serial.println(tmp);
}

void loop(){
 
  /* DEBUG INFO */
/*   
  Serial.print("current angle: "); Serial.println(servo1->currentAngle(), DEC);
  Serial.print("current pulse: "); printHex(servo1->currentPulse(), 8);
  Serial.print("destination angle: "); Serial.println(servo1->destinationAngle());
  Serial.print("next angle: "); Serial.println(servo1->nextAngle());
  Serial.print("max velocity: ");  printHex(servo1->maxVelocity(), 8);
  Serial.print("destination angle: "); Serial.println(servo1->destinationAngle());
  Serial.print("is limited: "); Serial.println(servo1->isLimited());
*/

// Rotates all the servos in unison
if(pattern == 1)
{
  for(int i=0; i<active_servos; i++)
  {
    if(servo[i]->currentAngle() <= 0)
      servo[i]->rotateTo(180);
    else if(servo[i]->currentAngle() >= 180)
      servo[i]->rotateTo(0);
  }
}

// Rotates all the servos at varying speeds.
if(pattern == 2)
{
  for(int i=0; i<active_servos; i++)
  {
    if(servo[i]->currentAngle() <= 0)
      servo[i]->rotateTo(180, 100/(i+1));
    else if(servo[i]->currentAngle() >= 180)
      servo[i]->rotateTo(0,100/(i+1));
  }
}
  
  
  if(Serial.available() > 0){
      serialValue = Serial.read();
      serialString += serialValue;
      
      if(serialValue == '\n')
      {
        Serial.print("Received command:"); 
        Serial.print(serialString);
        
        // commands
        // set the number of servos currently moving
        // usages include "n0" up to "n20"
       if(serialString.substring(0, 1) == "n")
       {
         char numServos[] = {' ', ' ', '\0'};
         serialString.substring(1, 3).toCharArray(numServos, 3);
         active_servos = atoi(numServos);
       }
       
       // set the pattern
       // see patterns above, currently valid are "p1" and "p2", 
       // "p3" will cause all servos to just stop moving.
       if(serialString.substring(0, 1) == "p")
       {
         char patternArr[] = {' ', ' ', '\0'};
         serialString.substring(1, 3).toCharArray(patternArr, 3);
         pattern = atoi(patternArr);
       }
       
       // Set the position of a given servo
       // usage ex: "s00180" set servo at index 0 (ie pin2) to position 180.
       // "s0790" set servo at index 7 to a position of 90 degrees.
       if(serialString.substring(0, 1) == "s")
       {
         char selectedServo[] = {' ', ' ', '\0'};
         serialString.substring(1, 3).toCharArray(selectedServo, 3);
         int selectedServoId = atoi(selectedServo);
         
         char angleChars[] = {' ', ' ', ' ', '\0'};
          serialString.substring(3, 6).toCharArray(angleChars, 4);
          int angle = atoi(angleChars);
          servo[selectedServoId]->rotateTo(angle);
          Serial.print("Moving to:");
          Serial.println(angle);
         
       }
        serialString = "";
      }
  } 
}

