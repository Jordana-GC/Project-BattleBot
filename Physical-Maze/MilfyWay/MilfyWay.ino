//NeoPixel Library that allows for the control of the 4 neopixels on the bot//
#include <Adafruit_NeoPixel.h>

// Motor pins//
#define LEFT_MOTOR_FORWARD 10
#define LEFT_MOTOR_BACK 9
#define RIGHT_MOTOR_FORWARD 5
#define RIGHT_MOTOR_BACK 6
//Constants for motor speed to allow for consistency in motor speed through functions//
const int RIGHT_MOTOR_SPEED = 213;
const int LEFT_MOTOR_SPEED = 200;
//Rotation sensor pin and int for the rotation count to be used in functions//
#define ROTATION_SENSOR 3
volatile int ROTATION_COUNT = 0;

// Ultrasonic sensors//
#define TRIG_FRONT 4
#define ECHO_FRONT 7
#define TRIG_RIGHT 13
#define ECHO_RIGHT 8

//Line sensors //
#define IR_0 A0
#define IR_1 A1
#define IR_2 A2
#define IR_3 A3
#define IR_4 A4
#define IR_5 A5
#define IR_6 A6
#define IR_7 A7

int IR_SENSORS[8] = {IR_0, IR_1, IR_2, IR_3, IR_4, IR_5, IR_6, IR_7};
boolean IR_VALUES[8];

//Gripper//
#define SERVO_GRIP 11

// NeoPixel defines, declares and strip initialization//
#define LED_PIN 12
#define LED_COUNT 4
#define BRIGHTNESS 150
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_RGB);//Needs to be initialized as RGB as AdaFruit library defaults GRB//

// Define a flag variable to track whether the maze has started//
bool mazeStarted = true;
bool mazeFinished = false;

// Robot status//
enum RobotStatus {
  MOVE_FORWARD,
  TURN_RIGHT,
  TURN_LEFT,
  ADJUST_RIGHT,
  ADJUST_LEFT,
  STAY_IN_PLACE,//STAY_IN_PLACE has not been initialized in the switch case as it is being used to keep the robot idle whilst waiting for the race to start//
};//semicolon needs to be at the end of enum to terminate the statement//

RobotStatus status = STAY_IN_PLACE; //Status variable needs to be defined after the enum declaration//

void setup() {
  //Ultrasonic sensor initialization and setup//
  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);

  // NeoPixel initialization and setup//
  strip.begin();
  strip.show();
  strip.setBrightness(BRIGHTNESS);

  //Gripper//
  pinMode(SERVO_GRIP, OUTPUT);
  moveGripper(200);  // Open the gripper
  delay(1000); // Give the servo time to move to the open position

  // Motor setup and initialization//
  pinMode(LEFT_MOTOR_FORWARD, OUTPUT);
  pinMode(LEFT_MOTOR_BACK, OUTPUT);
  pinMode(RIGHT_MOTOR_BACK, OUTPUT);
  pinMode(RIGHT_MOTOR_FORWARD, OUTPUT);
  pinMode(ROTATION_SENSOR, INPUT);

  // attach Interrupts to allow for rotation sensor to be updated//
  attachInterrupt(digitalPinToInterrupt(ROTATION_SENSOR), updateRotations, CHANGE);
}
/*
   The following functions until stated otherwise pertain to the motor movement and functions of the robot.
*/
void moveForward() {
  //This function is the basis of all decisions made in the maze, when it reads threshholds it switches cases based on the readings eventually the case returns to this function//
  strip.clear();
  strip.setPixelColor(2, strip.Color(52, 235, 164));
  strip.setPixelColor(3, strip.Color(52, 235, 164));
  strip.show();

  //This is what alllows the robot to make choices based on the readings it receives//
  const int frontDistance = getObstacleDistance(TRIG_FRONT, ECHO_FRONT);
  const int rightDistance = getObstacleDistance(TRIG_RIGHT, ECHO_RIGHT);

  if (rightDistance > 35) {
    status = TURN_RIGHT;
    return;
  }
  else if (frontDistance <= 15) {
    status = TURN_LEFT;
    return;
  }
  if (rightDistance <= 6) {
    status = ADJUST_RIGHT;
    return;
  }
  else if (rightDistance >= 9) {
    status = ADJUST_LEFT;
    return;
  }
  else {
    analogWrite(RIGHT_MOTOR_FORWARD, RIGHT_MOTOR_SPEED);
    analogWrite(LEFT_MOTOR_FORWARD, LEFT_MOTOR_SPEED);
    analogWrite(RIGHT_MOTOR_BACK, 0);
    analogWrite(LEFT_MOTOR_BACK, 0);
  }
}

void moveBackward(int rotations) {
  strip.clear();
  strip.setPixelColor(0, strip.Color(52, 235, 164));
  strip.setPixelColor(1, strip.Color(52, 235, 164));
  strip.show();

  resetRotations();
  while (ROTATION_COUNT < rotations) {
    analogWrite(RIGHT_MOTOR_FORWARD, 0);
    analogWrite(LEFT_MOTOR_FORWARD, 0);
    analogWrite(RIGHT_MOTOR_BACK, RIGHT_MOTOR_SPEED);
    analogWrite(LEFT_MOTOR_BACK, LEFT_MOTOR_SPEED);
  }
}

void stopRobot() {
  analogWrite(LEFT_MOTOR_FORWARD, 0);
  analogWrite(LEFT_MOTOR_BACK, 0);
  analogWrite(RIGHT_MOTOR_FORWARD, 0);
  analogWrite(RIGHT_MOTOR_BACK, 0);
  delay(500);
}

void turnRight(int rotations) {
  Serial.println("Turning right");
  strip.clear();
  strip.setPixelColor(1, strip.Color(157, 86, 227));
  strip.setPixelColor(2, strip.Color(157, 86, 227));
  strip.show();

  resetRotations();
  while (ROTATION_COUNT < rotations) {
    analogWrite(LEFT_MOTOR_FORWARD, LEFT_MOTOR_SPEED);
    analogWrite(LEFT_MOTOR_BACK, 0);
    analogWrite(RIGHT_MOTOR_FORWARD, 0);
    analogWrite(RIGHT_MOTOR_BACK, RIGHT_MOTOR_SPEED);
  }
}

void turnLeft(int rotations) {
  strip.clear();
  strip.setPixelColor(0, strip.Color(137, 242, 39));
  strip.setPixelColor(3, strip.Color(137, 242, 39));
  strip.show();

  resetRotations();
  while (ROTATION_COUNT < rotations) {
    analogWrite(LEFT_MOTOR_FORWARD, 0);
    analogWrite(LEFT_MOTOR_BACK, LEFT_MOTOR_SPEED);
    analogWrite(RIGHT_MOTOR_FORWARD, RIGHT_MOTOR_SPEED);
    analogWrite(RIGHT_MOTOR_BACK, 0);
  }
}

void moveForwardBeforeTurn (int rotations) {
  strip.clear();
  strip.fill(strip.Color(237, 76, 210));
  strip.show();

  resetRotations();
  while (ROTATION_COUNT < rotations) {
    analogWrite(RIGHT_MOTOR_FORWARD, RIGHT_MOTOR_SPEED);
    analogWrite(LEFT_MOTOR_FORWARD, LEFT_MOTOR_SPEED);
    analogWrite(RIGHT_MOTOR_BACK, 0);
    analogWrite(LEFT_MOTOR_BACK, 0);
  }
}
void checkFrontForLeftTurn(int rotations) {
  const int frontDistance = getObstacleDistance(TRIG_FRONT, ECHO_FRONT);

  if (frontDistance <= 13) {
    moveBackward(random(8));
    turnLeft(rotations);
  } else {
    return;
  }
}

void adjustRight() {
  analogWrite(LEFT_MOTOR_FORWARD, 0);
  analogWrite(LEFT_MOTOR_BACK, LEFT_MOTOR_SPEED);
  analogWrite(RIGHT_MOTOR_FORWARD, RIGHT_MOTOR_SPEED);
  analogWrite(RIGHT_MOTOR_BACK, 0);

  status = MOVE_FORWARD;
}

void adjustLeft() {
  analogWrite(LEFT_MOTOR_FORWARD, LEFT_MOTOR_SPEED);
  analogWrite(LEFT_MOTOR_BACK, 0);
  analogWrite(RIGHT_MOTOR_FORWARD, 0);
  analogWrite(RIGHT_MOTOR_BACK, RIGHT_MOTOR_SPEED);

  status = MOVE_FORWARD;
}

//Function that allows for scalability reagrding ultrasonic sensors and readings as it takes the name of the ultrasonic pins as a parameter to obtain the distances from each sensor //
int getObstacleDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  int distance = duration * 0.0343 / 2;

  return distance;
}

//These functions are interrupt attached to allow for the rotations to be counted incremented and used to control how the motors run//
void updateRotations() {
  noInterrupts();
  ROTATION_COUNT++;
  interrupts();
}

void resetRotations() {
  ROTATION_COUNT = 0;
}

//Function that runs while the robot is idle and after the race is completed//
void updateNeoPixels() {
  // Circular lighting effect//
  static int pixelIndex = 0;
  static int colorIndex = 0;
  static unsigned long lastUpdateTime = 0;
  unsigned long currentTime = millis();

  //Update NeoPixels every 100 milliseconds//
  if (currentTime - lastUpdateTime >= 100) {
    strip.clear(); // Clear previous pixel colors//

    //Set current pixel color based on the color index//
    switch (colorIndex) {
      case 0:
        strip.setPixelColor(pixelIndex, strip.Color(255, 0, 0)); // Red //
        break;
      case 1:
        strip.setPixelColor(pixelIndex, strip.Color(255, 127, 0)); // Orange //
        break;
      case 2:
        strip.setPixelColor(pixelIndex, strip.Color(255, 255, 0)); // Yellow //
        break;
      case 3:
        strip.setPixelColor(pixelIndex, strip.Color(0, 255, 0)); // Green //
        break;
      case 4:
        strip.setPixelColor(pixelIndex, strip.Color(0, 0, 255)); // Blue //
        break;
      case 5:
        strip.setPixelColor(pixelIndex, strip.Color(75, 0, 130)); // Purple //
        break;
      case 6:
        strip.setPixelColor(pixelIndex, strip.Color(255, 0, 255)); // Pink //
        break;
    }

    strip.show(); //Update NeoPixels//

    //Increment pixel index for the next iteration//
    pixelIndex = (pixelIndex + 1) % LED_COUNT;

    //If a round is completed, switch to the next color//
    if (pixelIndex == 0) {
      colorIndex = (colorIndex + 1) % 7; //There are 7 colors//
    }

    lastUpdateTime = currentTime;
  }
}

//Function that is used to opne and close the gripper at the begining and end of the race//
void moveGripper(int pulseWidth) {
  int angle = map(pulseWidth, 0, 280, 0, 255);
  analogWrite(SERVO_GRIP, angle);
}

//The bellow functions pertain to th eline sensors and read its values to detarmine the end of the race//
void queryIRSensors()
{
  for (int i = 0; i < 8; i++)
  {
    IR_VALUES[i] = analogRead(IR_SENSORS[i]) > 800;
  }
}

boolean blackSquare()
{
  short sum = 0;
  queryIRSensors();
  for (int i = 0; i < 8; i++)
  {
    if (IR_VALUES[i])
    {
      sum++;
    }
  }

  return sum == 8;
}

//The bellow function is ran when the object has been detected and the maze starts//
void startMaze()
{
  // Move forward towards the object for 1 second //
  analogWrite(RIGHT_MOTOR_BACK, 0);
  analogWrite(LEFT_MOTOR_BACK, 0);
  analogWrite(LEFT_MOTOR_FORWARD, LEFT_MOTOR_SPEED);
  analogWrite(RIGHT_MOTOR_FORWARD, RIGHT_MOTOR_SPEED);
  delay(1000);
  moveGripper(130); // Close the gripper //

  // Moveforward lateral to the maze opening //
  resetRotations();
  while (ROTATION_COUNT < 15) {
    analogWrite(RIGHT_MOTOR_BACK, 0);
    analogWrite(LEFT_MOTOR_BACK, 0);
    analogWrite(LEFT_MOTOR_FORWARD, LEFT_MOTOR_SPEED);
    analogWrite(RIGHT_MOTOR_FORWARD, RIGHT_MOTOR_SPEED);
  }

  // Turn left towards the start of the maze //
  resetRotations();
  while (ROTATION_COUNT < 22) {
    analogWrite(LEFT_MOTOR_FORWARD, 0);
    analogWrite(LEFT_MOTOR_BACK, LEFT_MOTOR_SPEED);
    analogWrite(RIGHT_MOTOR_FORWARD, RIGHT_MOTOR_SPEED);
    analogWrite(RIGHT_MOTOR_BACK, 0);
  }

  //After turning in move straight into the maze with enough rotations so that there is a wall to its side to allow for the turnning algorithm to function correctly//
  resetRotations();
  while (ROTATION_COUNT < 55) {
    analogWrite(RIGHT_MOTOR_BACK, 0);
    analogWrite(LEFT_MOTOR_BACK, 0);
    analogWrite(LEFT_MOTOR_FORWARD, LEFT_MOTOR_SPEED);
    analogWrite(RIGHT_MOTOR_FORWARD, RIGHT_MOTOR_SPEED);
  }
  // After the start sequence is finished and the robot is securely in the maze switch the case to start the challenge //
  status = MOVE_FORWARD;
}


void loop() {
  // Update obstacle distances //
  const int frontDistance = getObstacleDistance(TRIG_FRONT, ECHO_FRONT);
  const int rightDistance = getObstacleDistance(TRIG_RIGHT, ECHO_RIGHT);

  // Check if the maze has not yet started //
  if (mazeStarted) {
    // Check if there is an obstacle in front //
    updateNeoPixels(); // While the robot is iddle have the lights showing //
    const int frontDistance = getObstacleDistance(TRIG_FRONT, ECHO_FRONT);
    if (frontDistance <= 25) {
      startMaze();  // Call the startMaze() function //
      mazeStarted = false;
    }
  }

  if (blackSquare())// If the black square is read the maze has finished //
  {
    mazeFinished = true;
  }

  if (mazeFinished)
  {
    stopRobot();
    moveGripper(200); // Open the  Gripper to drop the object //
    updateNeoPixels();

  } else { // If the maze has started and not yet finished the switch case will be executed //
    switch (status) {
      case MOVE_FORWARD:
        moveForward();
        break;

      case TURN_RIGHT:
        stopRobot();
        moveForwardBeforeTurn(20);
        stopRobot();
        turnRight(14);
        stopRobot();
        moveForwardBeforeTurn(20);
        stopRobot();
        status = MOVE_FORWARD;
        break;

      case TURN_LEFT:
        stopRobot();
        turnLeft(17);
        stopRobot();
        checkFrontForLeftTurn(16);
        stopRobot();
        moveForwardBeforeTurn(20);
        stopRobot();
        status = MOVE_FORWARD;
        break;

      case ADJUST_RIGHT:
        adjustRight();
        break;

      case ADJUST_LEFT:
        adjustLeft();
        break;
    }
  }
}
