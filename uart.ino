void setup() {
  // Initialize serial communication
  Serial.begin(9600);

  // Wait for the serial connection to open (optional for some boards)
  while (!Serial);

  // Set LED pin as output
  pinMode(7, OUTPUT);

  // Feedback message
  Serial.println("UART Communication Initialized.");
}

void loop() {
  // Check if data is available to read
  if (Serial.available() > 0) {
    // Read incoming data
    char receivedChar = Serial.read();

    // Echo the received character back to the sender
    Serial.print("Received: ");
    Serial.println(receivedChar);

    // Perform actions based on received data
    if (receivedChar == '1') {
      // Turn on LED if '1' is received
      digitalWrite(7, HIGH);
      Serial.println("LED ON");
    } else if (receivedChar == '0') {
      // Turn off LED if '0' is received
      digitalWrite(7, LOW);
      Serial.println("LED OFF");
    } else {
      // Invalid input
      Serial.println("Invalid Command");
    }
  }

  // Optional delay for better readability
  delay(100);
}
