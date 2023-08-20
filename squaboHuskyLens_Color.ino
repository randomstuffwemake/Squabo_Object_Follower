#include <HUSKYLENS.h>

#define RIGHT_BACK_EN 9
#define RIGHT_BACK_IN1 8
#define RIGHT_BACK_IN2 10

#define LEFT_BACK_EN 3
#define LEFT_BACK_IN1 4
#define LEFT_BACK_IN2 7

#define RIGHT_FRONT_EN 11
#define RIGHT_FRONT_IN1 12
#define RIGHT_FRONT_IN2 13

#define LEFT_FRONT_EN 5
#define LEFT_FRONT_IN1 2
#define LEFT_FRONT_IN2 1

#define MAX_SPEED 255
#define MED_SPEED 127
#define MIN_SPEED  80

#define HEADLIGHT 6
#define MAX_BRIGHTNESS 255

#define MIN_BLOCK_SIZE 35 // Example threshold for small block size
#define MAX_BLOCK_SIZE 170 // Example threshold for large block size


#define CAMERA_CENTER_X 160
#define TOLERANCE 35  // to give a range around the center for the object to be considered centered

#define DEBUG false

HUSKYLENS huskylens;

void stopRobot() {
  analogWrite(RIGHT_FRONT_EN, 0);
  analogWrite(RIGHT_BACK_EN, 0);
  analogWrite(LEFT_FRONT_EN, 0);
  analogWrite(LEFT_BACK_EN, 0);
}

void rightFrontCW(uint8_t speed) {
  analogWrite(RIGHT_FRONT_EN, speed);
  digitalWrite(RIGHT_FRONT_IN1, HIGH);
  digitalWrite(RIGHT_FRONT_IN2, LOW);
}

void rightBackCW(uint8_t speed) {
  analogWrite(RIGHT_BACK_EN, speed);
  digitalWrite(RIGHT_BACK_IN1, HIGH);
  digitalWrite(RIGHT_BACK_IN2, LOW);
}

void rightFrontACW(uint8_t speed) {
  analogWrite(RIGHT_FRONT_EN, speed);
  digitalWrite(RIGHT_FRONT_IN1, LOW);
  digitalWrite(RIGHT_FRONT_IN2, HIGH);
}

void rightBackACW(uint8_t speed) {
  analogWrite(RIGHT_BACK_EN, speed);
  digitalWrite(RIGHT_BACK_IN1, LOW);
  digitalWrite(RIGHT_BACK_IN2, HIGH);
}

void leftFrontCW(uint8_t speed) {
  analogWrite(LEFT_FRONT_EN, speed);
  digitalWrite(LEFT_FRONT_IN1, HIGH);
  digitalWrite(LEFT_FRONT_IN2, LOW);
}

void leftBackCW(uint8_t speed) {
  analogWrite(LEFT_BACK_EN, speed);
  digitalWrite(LEFT_BACK_IN1, HIGH);
  digitalWrite(LEFT_BACK_IN2, LOW);
}

void leftFrontACW(uint8_t speed) {
  analogWrite(LEFT_FRONT_EN, speed);
  digitalWrite(LEFT_FRONT_IN1, LOW);
  digitalWrite(LEFT_FRONT_IN2, HIGH);
}

void leftBackACW(uint8_t speed) {
  analogWrite(LEFT_BACK_EN, speed);
  digitalWrite(LEFT_BACK_IN1, LOW);
  digitalWrite(LEFT_BACK_IN2, HIGH);
}

void goForward(uint8_t speed) {
  rightFrontCW(speed);
  rightBackCW(speed);
  leftFrontACW(speed);
  leftBackACW(speed);
}

void goBackward(uint8_t speed) {
  rightFrontACW(speed);
  rightBackACW(speed);
  leftFrontCW(speed);
  leftBackCW(speed);
}

void goLeft(uint8_t speed) {
  rightFrontCW(speed);
  rightBackCW(speed);
  leftFrontCW(speed);
  leftBackCW(speed);
}

void goRight(uint8_t speed) {
  rightFrontACW(speed);
  rightBackACW(speed);
  leftFrontACW(speed);
  leftBackACW(speed);
}

void printResult(HUSKYLENSResult result) {
  if (result.command == COMMAND_RETURN_BLOCK) {
    Serial.println(String()+F("Block:xCenter=")+result.xCenter+F(",yCenter=")+result.yCenter+F(",width=")+result.width+F(",height=")+result.height+F(",ID=")+result.ID);
  } else if (result.command == COMMAND_RETURN_ARROW) {
    Serial.println(String()+F("Arrow:xOrigin=")+result.xOrigin+F(",yOrigin=")+result.yOrigin+F(",xTarget=")+result.xTarget+F(",yTarget=")+result.yTarget+F(",ID=")+result.ID);
  } else {
    Serial.println("Object unknown!");
  }
}

void setup() {
  pinMode(RIGHT_BACK_EN, OUTPUT);
  pinMode(RIGHT_BACK_IN1, OUTPUT);
  pinMode(RIGHT_BACK_IN2, OUTPUT);
  pinMode(LEFT_BACK_EN, OUTPUT);
  pinMode(LEFT_BACK_IN1, OUTPUT);
  pinMode(LEFT_BACK_IN2, OUTPUT);
  pinMode(RIGHT_FRONT_EN, OUTPUT);
  pinMode(RIGHT_FRONT_IN1, OUTPUT);
  pinMode(RIGHT_FRONT_IN2, OUTPUT);
  pinMode(LEFT_FRONT_EN, OUTPUT);
  pinMode(LEFT_FRONT_IN1, OUTPUT);
  pinMode(LEFT_FRONT_IN2, OUTPUT);
  pinMode(HEADLIGHT, OUTPUT);
  analogWrite(HEADLIGHT, 127);
  
  if (DEBUG) {
    Serial.begin(115200);
    while(!Serial);
    Serial.println("STARTING");
  }
  Wire.begin();
  while (!huskylens.begin(Wire)) {
    if (DEBUG) {
      Serial.println("Begin failed");
    }
  }
  huskylens.writeAlgorithm(ALGORITHM_OBJECT_TRACKING);
}

void loop() {

//   Serial.print("Run");
//   goForward(MAX_SPEED);
//   delay(5000);
//   Serial.print("Stop");
//   stopRobot();
//   delay(2000);
   

  if (!huskylens.request()) {
    if (DEBUG) {
      Serial.println("Fail to request data from HUSKYLENS, recheck the connection!");
    }
    stopRobot();
  } else if (!huskylens.isLearned()) {
    if (DEBUG) {
      Serial.println("Nothing learned, press learn button on HUSKYLENS to learn one!");
    }
    stopRobot();
  } else if(!huskylens.available()) {
    if (DEBUG) {
      Serial.println("No block or arrow appears on the screen!");
    }
    stopRobot();
  } else {
    while (huskylens.available()) {
      HUSKYLENSResult result = huskylens.read();
      if (DEBUG) {
        printResult(result);
      }
      if (result.ID == 0) {
        stopRobot();
      }
      else if (result.ID == 1) {
          if (result.xCenter > (CAMERA_CENTER_X + TOLERANCE)) {
              // Object is to the right of the center
              goRight(MIN_SPEED);
          } else if (result.xCenter < (CAMERA_CENTER_X - TOLERANCE)) {
              // Object is to the left of the center
              goLeft(MIN_SPEED);
          } else if (result.width < MIN_BLOCK_SIZE && result.height < MIN_BLOCK_SIZE) {
              // Object block size is smaller, indicating it's farther away
              goForward(MED_SPEED);
          } else if (result.width > MAX_BLOCK_SIZE && result.height > MAX_BLOCK_SIZE) {
              // Object block size is larger, indicating it's closer
              goBackward(MED_SPEED);
          } else {
              // Object is approximately centered and its size is between the thresholds
              stopRobot();
          }
      }
    }    
}

}
