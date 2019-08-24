#pragma region Sensor Library
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

// The maximum reflection values on a clear filter over R, G, B, Bl, and W
const int SENSOR_MAX_VALUES[] = { 15, 16, 15, 16, 10 };

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

// Returns an approximate location of the line between -1 and 1
#define SENSOR_POS 0.5
double getLinePos(int lineColour) {
  // Naively assumes line is always between two sensors
  // -1 is at left sensor, 1 is at right sensor
  // TODO better positioning algorithm
  int leftReflect = getSensor(SENSOR_LEFT, SENSOR_FILTER_CLEAR) - SENSOR_MAX_VALUES[lineColour];
  int rightReflect = getSensor(SENSOR_RIGHT, SENSOR_FILTER_CLEAR) - SENSOR_MAX_VALUES[lineColour];

  int range = leftReflect + rightReflect;
  double pos = ((double)leftReflect / range * 2) - 1;
  return constrain(pos, -1, 1);
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
  Serial.print("  Y= ");
  Serial.print(getSensor(sensor,SENSOR_FILTER_CLEAR));
  Serial.println();
}

void debugSensorColour(int sensor) {
  int colour = getColour(sensor);
  String colourString;
  
  switch (colour) {
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

const double Kp = 1;
const double Ki = 0;
const double Kd = 0;
#define PID_MAX_SPD 25
#define PID_MIN_SPD -10
double integral = 0;
double lastError = 0;
double derivative = 0;
int lastColour = SENSOR_WHITE;
void tierOnePID() {
  int leftColour = getColour(SENSOR_LEFT);
  int rightColour = getColour(SENSOR_RIGHT);

  // Transition colour only from/to black. Block blue to green to smooth colour estimations
  if (leftColour != SENSOR_WHITE && (leftColour == SENSOR_BLACK || lastColour == SENSOR_BLACK)) {
    lastColour = leftColour;
  } else if (rightColour != SENSOR_WHITE && (rightColour == SENSOR_BLACK || lastColour == SENSOR_BLACK)) {
    lastColour = rightColour;
  }

  double error = getLinePos(lastColour);
  
  // PID algorithm
  integral += error;
  derivative = error - lastError;
  lastError = error;
  double turn = (Kp * error) + (Ki * integral) + (Kd * derivative);

  // Convert turn to drive
  double turnMagnitude = abs(turn);
  double lGradient = ((double)TURN_FW_SPD - TURN_BW_SPD) / 2;
  double rGradient = ((double)TURN_BW_SPD - TURN_FW_SPD) / 2;
  driveMotors(-TURN_BW_SPD + lGradient*(1+turn), TURN_FW_SPD + rGradient*(1+turn));
}

bool isColour(int colour) {
  if( colour != SENSOR_WHITE && colour != SENSOR_BLACK) {
    return true;
  }
  return false;
}

void tierTwo() {
  int leftColour = getColour(SENSOR_LEFT);
  int rightColour = getColour(SENSOR_RIGHT);

  if (leftColour == SENSOR_WHITE && rightColour == SENSOR_WHITE) {
    driveMotors(CRAWL_SPD,CRAWL_SPD); //straight
  }
  else if (leftColour == SENSOR_WHITE) {
    driveMotors(TURN_FW_SPD,TURN_BW_SPD);
    // right
  }
  else if (rightColour == SENSOR_WHITE) {
    driveMotors(TURN_BW_SPD,TURN_FW_SPD);
    //left
  }
  else if ( isColour(leftColour) && !isColour(rightColour) ) {
    driveMotors(TURN_BW_SPD,TURN_FW_SPD);
    //left
  }
  else if ( !isColour(leftColour) && isColour(rightColour)) {
    driveMotors(TURN_FW_SPD,TURN_BW_SPD);
    //right
  }
  else {
      driveMotors(CRAWL_SPD,CRAWL_SPD); //straight
    // forward, both 'coloured'
  }
}


void isTierThreeColour() {
  
}
void tierThree() { 
  int leftColour = getColour(SENSOR_LEFT);
  int rightColour = getColour(SENSOR_RIGHT);

  if (leftColour == SENSOR_WHITE && rightColour == SENSOR_WHITE) {
    driveMotors(CRAWL_SPD,CRAWL_SPD); //straight
  }
  else if (leftColour == SENSOR_WHITE) {
    driveMotors(TURN_FW_SPD,TURN_BW_SPD);
    // right
  }
  else if (rightColour == SENSOR_WHITE) {
    driveMotors(TURN_BW_SPD,TURN_FW_SPD);
    //left
  }
  else if ( isColour(leftColour) && !isColour(rightColour) ) {
    driveMotors(TURN_BW_SPD,TURN_FW_SPD);
    //left
  }
  else if ( !isColour(leftColour) && isColour(rightColour)) {
    driveMotors(TURN_FW_SPD,TURN_BW_SPD);
    //right
  }
  else {
      driveMotors(CRAWL_SPD,CRAWL_SPD); //straight
    // forward, both 'coloured'
  }
}

#define MOTOR_ENABLE A5
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
  if (powerState == LOW) {
    tierOne();
  } else {
    debugRawSensor(SENSOR_RIGHT);
    debugSensorColour(SENSOR_RIGHT);
    delay(100);
  }
  
  //driveMotors(25, 25);
}
