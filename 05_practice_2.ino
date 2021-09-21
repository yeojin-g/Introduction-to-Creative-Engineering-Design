#define PIN_LED 7
unsigned int count, toggle;

void setup() {
   pinMode(PIN_LED, OUTPUT);
  Serial.begin(115200); // Initialize serial port
  while (!Serial) {
    ; // wait for serial port to connect.
  }
  count = toggle = 1;
  digitalWrite(PIN_LED, 0);
  delay(1000);
  digitalWrite(PIN_LED, 1);
}

void loop() {
  while (count < 12) {
    toggle = toggle_state(toggle);
    Serial.println(toggle_state(toggle));
    Serial.println(count);
    digitalWrite(PIN_LED, toggle);
    delay(100);
    toggle = ++toggle;
    count = 1 + count;
  }
}

int toggle_state(int toggle) {
  return toggle%2;
}
