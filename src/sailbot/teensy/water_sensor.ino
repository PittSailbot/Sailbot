/*#define WAIT_TIME 10 // How many seconds to between sensor collections

#define POWER_PIN 12 // Connect to VCC for constant power or pulse on/off with digital out
#define SIGNAL_PIN0 A0 // Water level sensor 1
#define SIGNAL_PIN1 A1 // Water level sensor 2
#define SIGNAL_PIN2 A2 // Water level sensor 3

int val0 = 0;
int val1 = 0;
int val2 = 0;

void setup() {
  Serial.begin(9600);
  pinMode(POWER_PIN, OUTPUT);
  digitalWrite(POWER_PIN, LOW);
}

void loop() {
  digitalWrite(POWER_PIN, HIGH);
  delay(10);
  val0 = analogRead(SIGNAL_PIN0);
  digitalWrite(POWER_PIN, LOW);

  Serial.print("Water Level: ");
  Serial.println(val0);

  delay(WAIT_TIME * 1000);
}*/