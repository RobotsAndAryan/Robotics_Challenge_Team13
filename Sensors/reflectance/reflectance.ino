void setup() {
  // Initialize the built-in LED pin as an output
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);  // Turn the LED on
  delay(10000);                     // Wait for a second
  digitalWrite(LED_BUILTIN, LOW);   // Turn the LED off
  delay(10000);                     // Wait for a second
}