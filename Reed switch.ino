const int reedSwitchPin = 5;  // Pin connected to the reed switch

const int ledPin = 7;        // Pin connected to the LED

 void setup() {

  pinMode(reedSwitchPin, INPUT_PULLUP);  // Set reed switch pin as input with 

  internal pull-up resistor
  
  pinMode(ledPin, OUTPUT);               // Set LED pin as output

}


void loop() {

  // Read the state of the reed switch

  int reedSwitchState = digitalRead(reedSwitchPin);


  // If the reed switch is activated (magnet close to it), turn the LED on

  if (reedSwitchState == LOW) {  // The reed switch connects to ground when activated

  digitalWrite(ledPin, HIGH);  // Turn LED on


  } else 

{

    digitalWrite(ledPin, LOW);   // Turn LED off

  }

}
