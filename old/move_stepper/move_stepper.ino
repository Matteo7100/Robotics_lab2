// This version uses an explicit PWM generator (in function MoveMotor1)

#include <Stepper.h>

// teste de velocidade e direcção para o balancer
// script baseado em codigo online na reference da biblioteca stepper

#include <Wire.h>


#define STEPS 200
#define dirPin 9
#define stepPin 8
#define sw1Pin 11
#define sw2Pin 10
// Define stepper motor connections and motor interface type. Motor interface type must be set to 1 when using a driver:
Stepper stepper(STEPS, 8, 9);
#define motorInterfaceType 1

int left, right, middle;


void setup() {
  
  Wire.begin();

  Serial.begin(9600);
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(sw1Pin, INPUT);
  pinMode(sw2Pin, INPUT);


  left = 0;
  right = 0;

  Serial.println("Going left"); // when viewed from the robot frame
                                // It's the right when facing the front of the robot
  while (digitalRead(sw2Pin)==1) {
    // go up until the switch is reached
    moveMotor1( 10, sw2Pin );
    left +=10;
  }
  Serial.print("Left at ");
  Serial.print(left);
  Serial.print("   ");
  Serial.print(digitalRead(sw2Pin));
  Serial.print("  ");
  Serial.println(digitalRead(sw1Pin));
  // at this point the motor is at the highest position

  Serial.println("Going right");
  while (digitalRead(sw1Pin)==1) {
    // go down until the switch is reached
    moveMotor1( -10, sw1Pin );
    right +=10;
  }
  Serial.print("Right at ");
  Serial.print(right);
  Serial.print("   ");
  Serial.print(digitalRead(sw2Pin));
  Serial.print("  ");
  Serial.println(digitalRead(sw1Pin));
   
  middle = right / 2;
  // put the motor at the middle - if needed an offset can be used
  moveMotor1(middle, sw2Pin);
  Serial.print("Middle point at  ");
  Serial.println(middle);

  Serial.println("Setup completed");
  
}

// ------------------------- main loop ----------------------------
void loop() { 
/*
  Serial.print(digitalRead(sw1Pin));
  Serial.print("   ");
  Serial.println(digitalRead(sw2Pin));
*/

    moveMotor1(100, sw2Pin);
    Serial.println("move 1");
    delay(3000);
    moveMotor1(-100, sw1Pin);
    Serial.println("move 2");
    delay(3000);
/*
    // move looking only to 1 end switch
    if( digitalRead(sw1Pin)==0) {   // Left switch, seen from front
      moveMotor1(10, sw2Pin);       // Move right, seen from front
      Serial.println("move 1");
    }
    else if( digitalRead(sw2Pin)==0) {   // Right switch, seen from front
      moveMotor1(-10, sw1Pin);        // Move left, seen from front
      Serial.println("move 2");
    }
*/
}



// Motor movement
// This function returns only after the movement is completed

void moveMotor1(int steps, int safetyPin1) {

  digitalWrite(dirPin, steps > 0 ? HIGH : LOW); // Set direction

  for (int i=0; i<abs(steps); i++) {

    if (digitalRead(safetyPin1)==1) {
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(1000);    //2500);  // Adjust for speed
      digitalWrite(stepPin, LOW);
      delayMicroseconds(1000);    //1500);
    }
    else {
      // get out of the loop and function
      break;
    }
  }
}
