#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ================= OLED =================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ================= HC-SR04 =================
// Sensor 1 = FRONT
#define TRIG_FRONT 2
#define ECHO_FRONT 3

// Sensor 2 = RIGHT
#define TRIG_RIGHT 4
#define ECHO_RIGHT 5

// Sensor 3 = REAR
#define TRIG_REAR 6
#define ECHO_REAR 7

// Sensor 4 = LEFT
#define TRIG_LEFT 8
#define ECHO_LEFT 9

// ================= LEDs + Buzzer =================
#define RED_LED 10
#define BLUE_LED 11
#define BUZZER 12

unsigned long previousMillis = 0;
bool warningState = false;


// ================= Distance Function =================
float getDistance(int trigPin, int echoPin) {

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);

  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000);

  if (duration == 0) {
    return -1;
  }

  return duration * 0.0343 / 2;
}


// ================= Setup =================
void setup() {

  Serial.begin(9600);

  // FRONT
  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);

  // RIGHT
  pinMode(TRIG_RIGHT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);

  // REAR
  pinMode(TRIG_REAR, OUTPUT);
  pinMode(ECHO_REAR, INPUT);

  // LEFT
  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);

  // LEDs + Buzzer
  pinMode(RED_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);
  pinMode(BUZZER, OUTPUT);

  digitalWrite(RED_LED, LOW);
  digitalWrite(BLUE_LED, LOW);
  digitalWrite(BUZZER, LOW);


  // OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {

    Serial.println("OLED not found!");

    while (1);
  }

  display.clearDisplay();

  display.setTextColor(SSD1306_WHITE);

  display.setTextSize(2);
  display.setCursor(15, 20);
  display.println("Starting");

  display.display();

  delay(1000);
}


// ================= Main Loop =================
void loop() {

  // Read FRONT
  float front = getDistance(TRIG_FRONT, ECHO_FRONT);
  delay(50);

  // Read RIGHT
  float right = getDistance(TRIG_RIGHT, ECHO_RIGHT);
  delay(50);

  // Read REAR
  float rear = getDistance(TRIG_REAR, ECHO_REAR);
  delay(50);

  // Read LEFT
  float left = getDistance(TRIG_LEFT, ECHO_LEFT);


  // ================= Find Closest =================

  float closest = 999;

  if (front > 0 && front < closest)
    closest = front;

  if (right > 0 && right < closest)
    closest = right;

  if (rear > 0 && rear < closest)
    closest = rear;

  if (left > 0 && left < closest)
    closest = left;


  // ================= Serial Monitor =================

  Serial.print("FRONT: ");
  Serial.print(front);
  Serial.print(" cm | ");

  Serial.print("RIGHT: ");
  Serial.print(right);
  Serial.print(" cm | ");

  Serial.print("REAR: ");
  Serial.print(rear);
  Serial.print(" cm | ");

  Serial.print("LEFT: ");
  Serial.print(left);
  Serial.print(" cm | ");

  Serial.print("CLOSEST: ");
  Serial.println(closest);


  // ================= OLED =================

  display.clearDisplay();

  display.setTextSize(1);

  // FRONT
  display.setCursor(0, 0);
  display.print("F:");
  if (front < 0)
    display.print("--");
  else {
    display.print(front, 1);
    display.print("cm");
  }

  // RIGHT
  display.setCursor(65, 0);
  display.print("R:");
  if (right < 0)
    display.print("--");
  else {
    display.print(right, 1);
    display.print("cm");
  }

  // REAR
  display.setCursor(0, 16);
  display.print("REAR:");
  if (rear < 0)
    display.print("--");
  else {
    display.print(rear, 1);
    display.print("cm");
  }

  // LEFT
  display.setCursor(65, 16);
  display.print("L:");
  if (left < 0)
    display.print("--");
  else {
    display.print(left, 1);
    display.print("cm");
  }


  // ================= SAFETY STATUS =================

  display.setTextSize(2);
  display.setCursor(0, 35);


  // =================================================
  // SAFE: ABOVE 35 CM
  // =================================================

  if (closest > 35 || closest == 999) {

    digitalWrite(BLUE_LED, HIGH);
    digitalWrite(RED_LED, LOW);
    digitalWrite(BUZZER, LOW);

    warningState = false;
    previousMillis = millis();

    display.println("SAFE");

    display.setTextSize(1);
    display.setCursor(0, 54);
    display.print("> 35 cm");
  }


  // =================================================
  // WARNING: 15-35 CM
  // =================================================

  else if (closest >= 15 && closest <= 35) {

    digitalWrite(BLUE_LED, LOW);

    display.println("WARNING");

    display.setTextSize(1);
    display.setCursor(0, 54);
    display.print("15 - 35 cm");

    unsigned long currentMillis = millis();

    if (currentMillis - previousMillis >= 500) {

      previousMillis = currentMillis;

      warningState = !warningState;

      digitalWrite(RED_LED, warningState);
      digitalWrite(BUZZER, warningState);
    }
  }


  // =================================================
  // DANGER: 5-15 CM
  // =================================================

  else if (closest >= 5 && closest < 15) {

    digitalWrite(BLUE_LED, LOW);

    display.println("DANGER");

    display.setTextSize(1);
    display.setCursor(0, 54);
    display.print("5 - 15 cm");

    unsigned long currentMillis = millis();

    if (currentMillis - previousMillis >= 200) {

      previousMillis = currentMillis;

      warningState = !warningState;

      digitalWrite(RED_LED, warningState);
      digitalWrite(BUZZER, warningState);
    }
  }


  // =================================================
  // CRITICAL: BELOW 5 CM
  // =================================================

  else {

    digitalWrite(BLUE_LED, LOW);

    // Constant RED LED
    digitalWrite(RED_LED, HIGH);

    // Constant BUZZER
    digitalWrite(BUZZER, HIGH);

    warningState = true;

    display.println("CRITICAL");

    display.setTextSize(1);
    display.setCursor(0, 54);
    display.print("< 5 cm");
  }


  display.display();

  delay(20);
}void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
