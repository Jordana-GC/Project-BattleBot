#include <Adafruit_NeoPixel.h>

Adafruit_NeoPixel strip(4, 4, NEO_RGB);

int sensorPins[] = {A0, A1, A2, A3, A4, A5, A6, A7};
int v[] = {0, 0, 0, 0, 0, 0, 0, 0};
const int Motor_A1 = 10;  // Left wheel, goes backwards
const int Motor_A2 = 6;  // Left wheel, goes forward
const int Motor_B1 = 9;  // Right wheel, goes forward
const int Motor_B2 = 5;  // Right wheel, goes backwards
bool lineMaze = false;
bool finished = false;

const int gripperPin = 2;
void moveGripper(int angle); // Function prototype declaration
void dropGripper(); // Function prototype declaration
bool stopGripper = false;

bool state = true;

int NeoPixels = 4;

int Echo = 7;
int Trigger = 8;

int repetitionsOfBlack = 0;
unsigned long startTime;

void setup() {

  for (int i = 0; i <= 7; i++) {
    pinMode(sensorPins[i], INPUT);
  }
  
//  pinMode(echoPin, INPUT);
//  pinMode(trigPin, OUTPUT);
  pinMode(Motor_A1, OUTPUT);
  pinMode(Motor_A2, OUTPUT);
  pinMode(Motor_B1, OUTPUT);
  pinMode(Motor_B2, OUTPUT);
  pinMode(NeoPixels, OUTPUT);

  pinMode(Echo, INPUT);
  pinMode(Trigger, OUTPUT);

  pinMode(gripperPin, OUTPUT);
  
  Serial.begin(9600); // Initialize serial communication
}

void loop() {

//  for (int i = 0; i <= 6; i++) {
//    Serial.println(" ");
//  }
//  for (int i = 0; i <= 7; i++) {
////    Serial.print("Value "+String(i)+" : ");
//    v[i] = analogRead(sensorPins[i]);
////    Serial.println(v[i]);
//  }
  if (lineMaze == false && finished == false) {
    lookMaybeStart();
  }
  if (lineMaze == true && finished == false) {
    if (analogRead(sensorPins[0]) > 840){
      turnRight();
    }
    if (isEverythingWhite()) {
      turnAround();
    }
    else if ((analogRead(sensorPins[2]) > 700 && analogRead(sensorPins[4]) > 700) || analogRead(sensorPins[2]) > 700 || analogRead(sensorPins[1]) > 700) {
      moveSlightlyRight();
    }
    else if ((analogRead(sensorPins[3]) > 700 && analogRead(sensorPins[5]) > 700) || analogRead(sensorPins[5]) > 700 || analogRead(sensorPins[6]) > 700 || analogRead(sensorPins[7]) > 700) {
      moveSlightlyLeft();
    } 
    else {
      forward();
    }
  } 

  if (lineMaze == true && finished == true) {
    setStrip("S");
    goBackwards();
    startTime = millis();
    while (millis() - startTime < 200) {
      continue;
    }
    stopMoving();
    int mapAngle = map(90, 0, 180, 544, 2400);
    // You can define the action to drop the gripper here
    // For example, open the gripper fully
    digitalWrite(gripperPin, HIGH); // Activate gripper (open)
    delayMicroseconds(mapAngle); // Delay to keep gripper open for 1 second (adjust as needed)
    digitalWrite(gripperPin, LOW); // Release gripper
    goBackwards();
    startTime = millis();
    while (millis() - startTime < 2500) {
      continue;
    }
    stopMoving();
    lineMaze = false;
    delay(20);
  }
  delay(1);
}

void forward() {
  setStrip("F");
  analogWrite(Motor_A1, 207);
  analogWrite(Motor_A2, 0);
  analogWrite(Motor_B1, 0);
  analogWrite(Motor_B2, 213);
}

void moveSlightlyLeft() {
  setStrip("F");
  analogWrite(Motor_A1, 150);
  analogWrite(Motor_A2, 0);
  analogWrite(Motor_B1, 0);
  analogWrite(Motor_B2, 220);
}

void moveSlightlyRight() {
  setStrip("F");
  analogWrite(Motor_A1, 221);
  analogWrite(Motor_A2, 0);
  analogWrite(Motor_B1, 0);
  analogWrite(Motor_B2, 160);
}

void stopMoving() {
  analogWrite(Motor_A1, 0);
  analogWrite(Motor_A2, 0);
  analogWrite(Motor_B1, 0);
  analogWrite(Motor_B2, 0);
  startTime = millis();
  while (millis() - startTime < 500) {
    continue;
  }
}

void goBackwards()
{
  analogWrite(Motor_A1, 0);// Left wheel, goes forward
  analogWrite(Motor_A2, 182);// Left wheel, goes backward
  analogWrite(Motor_B1, 171);//Right wheel, goes backward
  analogWrite(Motor_B2, 0);// Right wheel, goes forward
}


void turnAround() {
  startTime = millis();
  while (millis() - startTime < 274) {
    continue;
  }
  spinLeft();
  startTime = millis();
  while (millis() - startTime < 3000) {
    if (!isEverythingWhite()) {
      startTime = millis();
      while (millis() - startTime < 25) {
        continue;
      }
      forward();
      return;
    }
  }
}

//void turnAround() {
//  delay(320);
//  spinRight();
//  delay(470);
//  stopMoving();
//  if (!isEverythingWhite()) {
//    forward();
//    return;
//  }
//  spinLeft();
//  delay(920);
//  stopMoving();
//  if (!isEverythingWhite()) {
//    forward();
//    return;
//  }
//  spinLeft();
//  delay(500);
//}

void turnRight() {
  startTime = millis();
  while (millis() - startTime < 200) {
    continue;
  }
  if (analogRead(sensorPins[2]) > 800 && analogRead(sensorPins[5]) > 800) {
    setStrip("S");
    stopMoving();
    startTime = millis();
    finished = true;
    return;
  }
  startTime = millis();
  while (millis() - startTime < 180) {
    continue;
  }
  spinRight();
  startTime = millis();
  while (millis() - startTime < 480) {
      continue;
  }
  stopMoving();
  if (isEverythingWhite()) {
    spinLeft();
    startTime = millis();
    while (millis() - startTime < 480) {
      continue;
    }
    stopMoving();
  }
}

void spinLeft() {
  setStrip("L");
  analogWrite(Motor_A1, 0);
  analogWrite(Motor_A2, 219);
  analogWrite(Motor_B1, 0);
  analogWrite(Motor_B2, 201);  
}

void spinRight() {
  setStrip("R");
  analogWrite(Motor_A1, 190.0);
  analogWrite(Motor_A2, 0);
  analogWrite(Motor_B1, 188.0);
  analogWrite(Motor_B2, 0);  
}

//void turnRightOrLeftOrMoveForward() {
//  unsigned long startTime = millis();
//  while (millis() - startTime < 180) {
//    if (analogRead(sensorPins[7]) > 700 || analogRead(sensorPins[6]) > 700 || analogRead(sensorPins[5]) > 700) {
//      turnAround(0);
//      return;
//    }  
//  }
//  if (isEverythingWhite()) {
//    delay(100);
//    spinRight();
//    delay(420);
//  }
//  forward();
//  delay(50);
//}

bool isEverythingWhite() {
  for (int i = 0; i <= 7; i++) {
    if (analogRead(sensorPins[i]) > 720 ) {
      return false;
    }
  }
  return true;
}

bool black() {
  for (int i = 1; i <= 6; i++) {
    if (analogRead(sensorPins[i]) < 720 ) {
      return false;
    }
  }
  return true;
}

void setStrip(String dir) {
  strip.clear();
  if (dir == "L") {
    strip.setPixelColor(3, strip.Color(255, 120, 0));
  } else if (dir == "R") {
    strip.setPixelColor(2, strip.Color(255, 120, 0));
  } else if (dir == "B") {
    strip.setPixelColor(0, strip.Color(255, 0, 0));
    strip.setPixelColor(1, strip.Color(255, 0, 0));
  } else if (dir == "F") {
    strip.setPixelColor(3, strip.Color(0, 255, 0));
    strip.setPixelColor(2, strip.Color(0, 255, 0));
  } else if (dir == "S") {
    strip.setPixelColor(0, strip.Color(0, 0, 255));
    strip.setPixelColor(1, strip.Color(0, 0, 255));
    strip.setPixelColor(2, strip.Color(0, 0, 255));
    strip.setPixelColor(3, strip.Color(0, 0, 255));
  } else if (dir == "X") {
    strip.setPixelColor(0, strip.Color(255, 0, 0));
    strip.setPixelColor(1, strip.Color(255, 0, 0));
    strip.setPixelColor(2, strip.Color(255, 0, 0));
    strip.setPixelColor(3, strip.Color(255, 0, 0));
  }
  strip.show();
}

void moveGripper(int angle) {
  int mapAngle = map(angle, 0, 180, 544, 2400);

  unsigned long startTime = millis(); // Record the start time
  unsigned long duration = 700; // Duration for gripper action in milliseconds
  
  for (int i = 0; i < 10; i++) { // Loop the gripper movement 10 times
    if (millis() - startTime >= duration) {
      break; // Exit the loop if the duration is exceeded
    }

    digitalWrite(gripperPin, HIGH); // Activate gripper
    delayMicroseconds(mapAngle); // Hold gripper position
    digitalWrite(gripperPin, LOW); // Release gripper
    delay(20); // Delay between actions
  }
}

void dropGripper() {
  // Function to drop the gripper
  int mapAngle = map(90, 0, 180, 544, 2400);
  // You can define the action to drop the gripper here
  // For example, open the gripper fully
  digitalWrite(gripperPin, HIGH); // Activate gripper (open)
  delayMicroseconds(mapAngle); // Delay to keep gripper open for 1 second (adjust as needed)
  digitalWrite(gripperPin, LOW); // Release gripper
  delay(20);

  if (stopGripper) {
    return;
  }
}

void lookMaybeStart() {
  // Declare variables
  static int consecutiveDetections = 0;
  static boolean eyeState = false;
  long duration;
  long distance = 0; // Declare distance variable
  long prevDistance = 0; // Variable to store the previous distance

  // Loop until three consecutive detections are made
  while (consecutiveDetections < 10) {
    Serial.println("Untra");
    // Trigger the sensor
    digitalWrite(Trigger, LOW);
    delayMicroseconds(5);
    digitalWrite(Trigger, HIGH);
    delayMicroseconds(50);
    digitalWrite(Trigger, LOW);

    // Measure the duration of the pulse
    duration = pulseIn(Echo, HIGH);
    delay(100);
    
    // Calculate distance based on the duration
    distance = duration * 0.034 / 2;
    Serial.println(distance);
    // Check if the current distance matches the previous distance
    if (distance == prevDistance && distance <= 35) {
      // Increment consecutive detections counter
      consecutiveDetections++;
    } else {
      // Reset consecutive detections counter if the distance changes
      consecutiveDetections = 0;
    
    }
    // Store the current distance as the previous distance for the next iteration
    prevDistance = distance;
  }

  forward();
  
  startTime = millis();
  while (millis() - startTime < 1150) {
      continue;
  }

  moveGripper(58);
  
  spinLeft();

  startTime = millis();
  while (millis() - startTime < 550) {
      continue;
  }
  stopMoving();
  lineMaze = true;
}
