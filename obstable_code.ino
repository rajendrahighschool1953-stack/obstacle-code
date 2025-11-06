#include <Servo.h>
#define trigPin 8
#define echoPin 11
#define ENA 9   // PWM pin for motor A
#define ENB 10  // PWM pin for motor B
#define IN1 7
#define IN2 6
#define IN3 5
#define IN4 4

Servo ultrasonicServo;

long duration;
int distance;
int servoAngle = 90;
int motorSpeed = 200; // Slightly reduced for smoother motion (0–255)

// --- Adjustable timing constants ---
const int SAFE_DISTANCE = 30;     // cm
const int BACKWARD_TIME = 600;    // ms
const int STOP_DELAY = 300;       // ms
const int TURN_DELAY = 700;       // ms
const int SERVO_LOOK_DELAY = 300; // ms
const int SENSOR_SAMPLE_DELAY = 5;
const int SENSOR_SAMPLES = 3;

void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  Serial.begin(9600);
  ultrasonicServo.attach(3);
  ultrasonicServo.write(servoAngle); // face forward
}

void loop() {
  distance = measureDistance();

  Serial.print("Distance: ");
  Serial.println(distance);

  if (distance > 0 && distance < SAFE_DISTANCE) {
    stopCar();
    delay(STOP_DELAY);

    // Move backward briefly
    moveBackward();
    delay(BACKWARD_TIME);
    stopCar();
    delay(STOP_DELAY);

    // Look left and right quickly
    int distanceLeft = lookLeft();
    int distanceRight = lookRight();
    ultrasonicServo.write(90); // reset forward

    // Choose best direction
    if (distanceLeft > distanceRight) {
      turnLeft();
    } else {
      turnRight();
    }
    delay(TURN_DELAY);
  } else {
    moveForward();
  }
}

// --- Ultrasonic measurement ---
int measureDistance() {
  long sum = 0;
  for (int i = 0; i < SENSOR_SAMPLES; i++) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    duration = pulseIn(echoPin, HIGH, 20000); // timeout in 20ms
    int measuredDistance = duration * 0.034 / 2;
    sum += measuredDistance;
    delay(SENSOR_SAMPLE_DELAY);
  }
  return sum / SENSOR_SAMPLES;
}

// --- Servo scanning ---
int lookRight() {
  ultrasonicServo.write(0);
  delay(SERVO_LOOK_DELAY);
  int distance = measureDistance();
  delay(100);
  return distance;
}

int lookLeft() {
  ultrasonicServo.write(180);
  delay(SERVO_LOOK_DELAY);
  int distance = measureDistance();
  delay(100);
  return distance;
}

// --- Motion control ---
void moveForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, motorSpeed);
  analogWrite(ENB, motorSpeed);
}

void moveBackward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, motorSpeed);
  analogWrite(ENB, motorSpeed);
}

void turnLeft() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, motorSpeed);
  analogWrite(ENB, motorSpeed);
}

void turnRight() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, motorSpeed);
  analogWrite(ENB, motorSpeed);
}

void stopCar() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}