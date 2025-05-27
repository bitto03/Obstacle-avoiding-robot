#include <Servo.h>
#include <NewPing.h>
#include <AFMotor.h>  // Include the AFMotor library for motor control

#define TRIGGER_PIN A0  // Use A0 as trigger pin on the ultrasonic sensor.
#define ECHO_PIN A1     // Use A1 as echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 200 // Maximum distance we want to ping for

int lt = 1;
int buzzer = 4;

Servo panMotor;  // create servo object to control a servo
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

AF_DCMotor motor1(1);  // Motor 1 (connected to M1 port of the motor shield)
AF_DCMotor motor2(4);  // Motor 2 (connected to M4 port of the motor shield)

void setup() {
  Serial.begin(9600);

  // Set the buzzer pin as output
  pinMode(lt, OUTPUT);
  pinMode(buzzer, OUTPUT);
  digitalWrite(lt, HIGH);

  panMotor.attach(9); // Attach servo motor to pin 9
  panMotor.write(90); // Set servo to center position
}

int obstacleFront, obstacleLeft, obstacleRight;
long randNumber;

void loop() {
  obstacleFront = HCSR04();
  Serial.print("Front Obstacle: ");
  Serial.println(obstacleFront);

  if (obstacleFront <= 30) {  // Obstacle detected in front
    moverStop();
    beep();
    delay(200);

    randNumber = random(100);

    // Randomize the direction of scanning (left or right first)
    if (randNumber % 2 == 0) {
      // Scan Right First
      panMotor.write(10); // Move servo to the right
      delay(500);
      obstacleRight = HCSR04();
      delay(300);
      Serial.print("Right Obstacle: ");
      Serial.println(obstacleRight);

      panMotor.write(90); // Move back to center
      delay(500);

      // Scan Left
      panMotor.write(180); // Move servo to the left
      delay(500);
      obstacleLeft = HCSR04();
      delay(300);
      Serial.print("Left Obstacle: ");
      Serial.println(obstacleLeft);

      panMotor.write(90); // Move back to center
      delay(500);
    } else {
      // Scan Left First
      panMotor.write(180); // Move servo to the left
      delay(500);
      obstacleLeft = HCSR04();
      delay(300);
      Serial.print("Left Obstacle: ");
      Serial.println(obstacleLeft);

      panMotor.write(90); // Move back to center
      delay(500);

      // Scan Right
      panMotor.write(10); // Move servo to the right
      delay(500);
      obstacleRight = HCSR04();
      delay(300);
      Serial.print("Right Obstacle: ");
      Serial.println(obstacleRight);

      panMotor.write(90); // Move back to center
      delay(500);
    }

    // Decision making based on the readings
    if (obstacleLeft >= obstacleRight) {
      Serial.println("Turn Left");
      moverLeft();
      delay(180);

      moverStop();
      beep();
      delay(200);
    } else {
      Serial.println("Turn Right");
      moverRight();
      delay(180);

      moverStop();
      beep();
      delay(200);
    }
  } else {
    // Move forward if no obstacle is detected
    moverRun();
  }
}

void beep() {
  digitalWrite(buzzer, HIGH);
  delay(5);
  digitalWrite(buzzer, LOW);
}

int HCSR04() {
  unsigned int time = sonar.ping(); // Send out a ping and store the time it took for it to come back
  int distance = time / US_ROUNDTRIP_CM; // Convert that time into a distance
  if (distance == 0 || distance == 5) {
    distance = 100; // Set the distance to max if no ping was received
  }

  delay(100);

  return distance;
}

void moverRun() {
  // Move forward
  motor1.setSpeed(255);  // Set speed for motor 1
  motor2.setSpeed(255);  // Set speed for motor 2
  motor1.run(FORWARD);   // Run motor 1 forward
  motor2.run(FORWARD);   // Run motor 2 forward
}

void moverLeft() {
  // Turn Left
  motor1.setSpeed(255);  // Set speed for motor 1
  motor2.setSpeed(255);  // Set speed for motor 2
  motor1.run(BACKWARD);  // Run motor 1 backward
  motor2.run(FORWARD);   // Run motor 2 forward
}

void moverRight() {
  // Turn Right
  motor1.setSpeed(255);  // Set speed for motor 1
  motor2.setSpeed(255);  // Set speed for motor 2
  motor1.run(FORWARD);   // Run motor 1 forward
  motor2.run(BACKWARD);  // Run motor 2 backward
}

void moverStop() {
  // Stop all motors
  motor1.run(RELEASE);
  motor2.run(RELEASE);
}