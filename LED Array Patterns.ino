// Define the pins for the LEDs 

int ledPins[] = {2, 3, 4, 5, 6, 7, 8, 9}; 

void setup() {

// Set up the LED pins as outputs 

for (int i = 0; i < 8; i++) { pinMode(ledPins[i], OUTPUT);

} 

}

 void loop() {

// Call different functions to display patterns 
blinkAll();

delay(1000); 

alternateBlink(); 


delay(1000);

// Add more patterns as needed

}

// Function to blink all LEDs void blinkAll() { for (int i = 0; i < 8; i++) {

digitalWrite(ledPins[i], HIGH);

} delay(500); for (int i = 0; i < 8; i++) {

digitalWrite(ledPins[i], LOW);

} delay(500);

}

// Function to alternate blink LEDs 

void alternateBlink() {

 for (int i = 0; i < 8; i += 2) {

   digitalWrite(ledPins[i], HIGH); 

   delay(250); 

   digitalWrite(ledPins[i], LOW);

}

 delay(500);

for (int i = 1; i < 8; i += 2) {

 digitalWrite(ledPins[i], HIGH);

 delay(250); 

 digitalWrite(ledPins[i], LOW);

} 

delay(500);

}

// Add more functions for additional patterns as needed
