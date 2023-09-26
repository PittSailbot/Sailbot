 //       Class description: Uses sensor to get current water level and show a 
 //       colored light depending on how high the level is
 //
 //       The Water Sensor website recommended that
 //       "the sensor be turned on only when taking readings."
 
 // Global Variables
 int waterLevel=0;

 int redLED = 2;
 int yellowLED = 3;
 int greenLED = 4;

 int lowerThreshold = 420; //a water level between 0 and lowerThreshold will show green light
 int upperThreshold = 520; //a water level between lowerThreshold and upperThreshold will show yellow light

// Sensor pins
#define sensorPower 7
#define sensorPin A0


void setup() {
  Serial.begin(9600);
	pinMode(sensorPower, OUTPUT);
	digitalWrite(sensorPower, LOW);
	
	// Set LED pins as an OUTPUT
	pinMode(redLED, OUTPUT);
	pinMode(yellowLED, OUTPUT);
	pinMode(greenLED, OUTPUT);

	// Initially turn off all LEDs
	digitalWrite(redLED, LOW);
	digitalWrite(yellowLED, LOW);
	digitalWrite(greenLED, LOW);
}

int getWaterLevel(){
  const DELAY=10; //delay time in ms between turning sensor on and off
  int level=0; //water level: ranges from 0 to 520 depending on how submerged sensor is

  digitalWrite(sensorPower, HIGH);
  delay(DELAY);
  int level= analogRead(sensorPin);
  digitalWrite(sensorPower, LOW);
  return level;
}

void turnOnLight(){ //Low:Green->Medium:Yellow->High:Red  
  const DELAY=1000; //time between sensor readings in ms
  
	if (level > 0 && level <= lowerThreshold) {
		Serial.println("Water Level: Low");
		digitalWrite(redLED, LOW);
		digitalWrite(yellowLED, LOW);
		digitalWrite(greenLED, HIGH);
	}
	else if (level > lowerThreshold && level <= upperThreshold) {
		Serial.println("Water Level: Medium");
		digitalWrite(redLED, LOW);
		digitalWrite(yellowLED, HIGH);
		digitalWrite(greenLED, LOW);
	}
	else if (level > upperThreshold) {
		Serial.println("Water Level: High");
		digitalWrite(redLED, HIGH);
		digitalWrite(yellowLED, LOW);
		digitalWrite(greenLED, LOW);
	}
	delay(DELAY);
}

void loop() { //main 
  waterLevel=getWaterLevel();
  turnOnLight();
}
