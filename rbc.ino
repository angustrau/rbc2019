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

const int SENSORS[2][5] = {
  {8,9,10,11,12},
  {8,9,10,11,12}
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

  if(blue < 25 && green < 25 && red < 25) {
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


void debugRawSensor() {
  Serial.print("R= ");
  Serial.print(getSensor(SENSOR_LEFT,SENSOR_FILTER_RED));
  Serial.print("  G= ");
  Serial.print(getSensor(SENSOR_LEFT,SENSOR_FILTER_GREEN));
  Serial.print("  B= ");
  Serial.print(getSensor(SENSOR_LEFT,SENSOR_FILTER_BLUE));
  Serial.println();
  delay(100);
}

void debugSensorColour() {
  int colour = getColour(SENSOR_LEFT);
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



void setup() {
  setupSensor(SENSOR_LEFT);
  setSensorScale(SENSOR_LEFT, SENSOR_SCALE_20);
  Serial.begin(9600);
  
}

void loop() {
  debugRawSensor();
  //debugSensorColour();
}


