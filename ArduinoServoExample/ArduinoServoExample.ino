#include <BitOps.h>
#include <MaskNode.h>
#include <MultiServoController.h>
#include <PortMaskNode.h>
#include <Servo.h>
#include <ServoNode.h>


// Serial port communication variables;
char serialValue;
String serialString;

// Servos
Servo *servo1;
Servo *servo2;
int pos = 0;

void setup()
{
  // Start listening on the serial port for commands
  Serial.begin(57600);
 
      servo1 = new Servo();
      servo2 = new Servo();

      // Attach a servo controller to pin 4.
      DDRD |= DDRD | 1<<4;
      attach(servo1, &PORTD, 1<<4);
      
      // Attach a servo controller to pins 5 and 6, (will control 2 servos as mirrors)
      DDRD |= DDRD | 1<<5 | 1<<6;
      attach(servo2, &PORTD, 1<<5 | 1<<6);
      
      servo1->rotateTo(90);
      servo2->rotateTo(180);
      
      Serial.println("Setup Complete."); 
      Serial.println(DDRD); 
}

void loop(){
  
  if(Serial.available() > 0){
      serialValue = Serial.read();
      serialString += serialValue;
      
      if(serialValue == '\n')
      {
        Serial.print("Received command:"); 
        Serial.print(serialString);
        
          Servo *selectedServo = NULL;
        //ex: "1:45" set servo1 to 45 degrees
          if(serialString.substring(0, 1) == "1")
            selectedServo = servo1;
          else if(serialString.substring(0, 1) == "2")
            selectedServo = servo2;
          
          if(selectedServo){
            char angleChars[] = {' ', ' ', ' ', '\0'};
            serialString.substring(2, 5).toCharArray(angleChars, 4);
            int angle = atoi(angleChars);
            selectedServo->rotateTo(angle);
            Serial.print("Moving to:");
            Serial.println(angle);
          }
        serialString = "";
      }
  } 
}
