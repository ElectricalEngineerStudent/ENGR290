//#define RELAY_PIN 11

const int RELAY_PIN = 5;  // the Arduino pin, which connects to the IN pin of relay
//this case we put it at pin P3 of the atmega328p

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin A5 as an output.
  pinMode(RELAY_PIN, OUTPUT);
  Serial.begin(9600);
}

// the loop function runs over and over again forever
void loop() {
  digitalWrite(RELAY_PIN, HIGH); // turn on fan 5 seconds
  delay(5000);
  digitalWrite(RELAY_PIN, LOW);  // turn off fan 5 seconds
  delay(5000);
}
