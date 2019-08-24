#pragma region Sensor Library


#define MOTOR_ENABLE A5
#define SENSOR_LEFT 0
#define SENSOR_RIGHT 1

#define SENSOR_IN 0
#define SENSOR_S0 1
#define SENSOR_S1 2
#define SENSOR_S2 3
#define SENSOR_S3 4

#define SENSOR_SCALE_OFF 0
#define SENSOR_SCALE_2 1
#define SENSOR_SCALE_20 2
#define SENSOR_SCALE_100 3

const int SENSOR_SCALE[4][2] = {
  { LOW, LOW },
  { LOW, HIGH },
  { HIGH, LOW },
  { HIGH, HIGH }
};

#define SENSOR_FILTER_RED 0
#define SENSOR_FILTER_BLUE 1
#define SENSOR_FILTER_CLEAR 2
#define SENSOR_FILTER_GREEN 3

const int SENSOR_FILTER[4][2] = {
  { LOW, LOW },
  { LOW, HIGH },
  { HIGH, LOW },
  { HIGH, HIGH }
};

#define SENSOR_RED 0
#define SENSOR_GREEN 1
#define SENSOR_BLUE 2
#define SENSOR_BLACK 3
#define SENSOR_WHITE 4

const int SENSORS[2][5] = {
  {3,13,12,11,4}, //OUTPUT, S0,S1,S2,S3
  {A4,A0,A1,A2,A3}
};

void setupSensor(int sensor) {
  // Setup input pin
  pinMode(SENSORS[sensor][SENSOR_IN], INPUT);
  // Setup configuration pins
  for (int pin = 1; pin < 5; pin++) {
    pinMode(SENSORS[sensor][pin], OUTPUT);
  }
}

void setSensorScale(int sensor, int scale) {
  digitalWrite(SENSORS[sensor][SENSOR_S0], SENSOR_SCALE[scale][0]);
  digitalWrite(SENSORS[sensor][SENSOR_S1], SENSOR_SCALE[scale][1]);
}

void setSensorFilter(int sensor, int filter) {
  digitalWrite(SENSORS[sensor][SENSOR_S2], SENSOR_FILTER[filter][0]);
  digitalWrite(SENSORS[sensor][SENSOR_S3], SENSOR_FILTER[filter][1]);
}

int getSensor(int sensor, int filter) {
  setSensorFilter(sensor, filter);
  int frequency = pulseIn(SENSORS[sensor][SENSOR_IN], LOW);
  return frequency;
}

int getColour(int sensor) {
  int red = getSensor(sensor, SENSOR_FILTER_RED);
  int green = getSensor(sensor, SENSOR_FILTER_GREEN);
  int blue = getSensor(sensor, SENSOR_FILTER_BLUE);
  
  int deltaRG = abs(red-green);
  int deltaRB = abs(red-blue);
  int deltaGB = abs(blue-green);

  if (blue < 25 && green < 25 && red < 25) {
    return SENSOR_WHITE;
  }

  if (green > blue && blue > red) {
    return SENSOR_RED;
  }

  if (deltaRB < deltaRG && deltaRB < deltaGB) {
    return SENSOR_BLACK;
  }
  // red or green high here
    // if red high or green high
  // if delta between green and blue is around 10
  // green
  // if delta is around 20
  // blue
  if(deltaGB > 15) {
    return SENSOR_GREEN;
  }

  return SENSOR_BLUE;
}

#pragma endregion

#pragma region Motor Library
#define MOTOR_LEFT_FW 5
#define MOTOR_LEFT_BW 6
#define MOTOR_LEFT_ENABLE 7
#define MOTOR_RIGHT_FW 9
#define MOTOR_RIGHT_BW 10 
#define MOTOR_RIGHT_ENABLE 8


void setupMotors() {
  pinMode(MOTOR_LEFT_FW, OUTPUT);
  pinMode(MOTOR_LEFT_BW, OUTPUT);
  pinMode(MOTOR_LEFT_ENABLE, OUTPUT);
  digitalWrite(MOTOR_LEFT_ENABLE, HIGH);
  pinMode(MOTOR_RIGHT_FW, OUTPUT);
  pinMode(MOTOR_RIGHT_BW, OUTPUT);
  pinMode(MOTOR_RIGHT_ENABLE, OUTPUT);
  digitalWrite(MOTOR_RIGHT_ENABLE, HIGH);
}

void driveMotors(int speedLeft, int speedRight) {
  int leftValue = abs(speedLeft);
  int rightValue = abs(speedRight);
  
  if (speedLeft < 0) {
    digitalWrite(MOTOR_LEFT_FW, LOW);
    analogWrite(MOTOR_LEFT_BW, leftValue);
  } else {
    analogWrite(MOTOR_LEFT_FW, leftValue);
    digitalWrite(MOTOR_LEFT_BW, LOW);
  }
  
  if (speedRight < 0) {
    digitalWrite(MOTOR_RIGHT_FW, LOW);
    analogWrite(MOTOR_RIGHT_BW, rightValue);
  } else {
    analogWrite(MOTOR_RIGHT_FW, rightValue);
    digitalWrite(MOTOR_RIGHT_BW, LOW);
  }
}

#pragma endregion

void debugRawSensor(int sensor) {
  Serial.print("R= ");
  Serial.print(getSensor(sensor,SENSOR_FILTER_RED));
  Serial.print("  G= ");
  Serial.print(getSensor(sensor,SENSOR_FILTER_GREEN));
  Serial.print("  B= ");
  Serial.print(getSensor(sensor,SENSOR_FILTER_BLUE));
  Serial.println();
}

void debugSensorColour(int sensor) {
  int colour = getColour(sensor);
  String colourString;
  
  switch (colour)
  {
  case SENSOR_RED:
    colourString = "Red";
    break;
  case SENSOR_GREEN:
    colourString = "Green";
    break;
  case SENSOR_BLUE:
    colourString = "Blue";
    break;
  case SENSOR_BLACK:
    colourString = "Black";
    break;
  case SENSOR_WHITE:
    colourString = "White";
    break;
  default:
    colourString = "lol what help this is broken";
    break;
  }
  Serial.println(colourString);
}

#define TURN_FW_SPD 25
#define TURN_BW_SPD -10
#define CRAWL_SPD 30
void tierOne() {
  // if not white, correct course
  int leftColour = getColour(SENSOR_LEFT);
  int rightColour = getColour(SENSOR_RIGHT);


  if (leftColour != SENSOR_WHITE) {
    driveMotors(TURN_BW_SPD,TURN_FW_SPD);
  } else if(rightColour != SENSOR_WHITE ) {
    driveMotors(TURN_FW_SPD,TURN_BW_SPD);
  } else {
    driveMotors(CRAWL_SPD,CRAWL_SPD);
  }
}

double Kp = 1;
double Ki = 1;
double Kd = 1; 
void tierOnePID() {
  
}

void tierTwo() {
  int leftColour = getColour(SENSOR_LEFT);
  int rightColour = getColour(SENSOR_RIGHT);

  if (leftColour == SENSOR_WHITE && rightColour == SENSOR_WHITE) {
    driveMotors(25,25);
  }

  if (leftColour == SENSOR_GREEN || leftColour == SENSOR_RED || leftColour == SENSOR_BLUE ) {
    if (rightColour != SENSOR_GREEN && rightColour != SENSOR_RED && rightColour != SENSOR_BLUE) {
      driveMotors(-10,25);
    }
  } else if(rightColour == SENSOR_GREEN || rightColour == SENSOR_RED || rightColour == SENSOR_BLUE ) {
    driveMotors(25,-10);
  } else if (leftColour == SENSOR_BLACK) {
    driveMotors(-10,25);
  } else if (rightColour == SENSOR_BLACK) {
    driveMotors(25,-10);
  }
}

void tierThree() {
  // only follow green black priority??

}

void setup() {
  setupSensor(SENSOR_LEFT);
  setupSensor(SENSOR_RIGHT);
  setSensorScale(SENSOR_LEFT, SENSOR_SCALE_20);
  setSensorScale(SENSOR_RIGHT, SENSOR_SCALE_20);

  setupMotors();

  pinMode(MOTOR_ENABLE, INPUT_PULLUP);
  Serial.begin(9600);
  
}

void loop() {
  int powerState = digitalRead(MOTOR_ENABLE);
  if(powerState == LOW) {
    tierOne();
  } else {
    debugRawSensor(SENSOR_RIGHT);
    debugSensorColour(SENSOR_RIGHT);
    delay(100);
  }
  
  //tierOne();
  //driveMotors(255, 255);
}
