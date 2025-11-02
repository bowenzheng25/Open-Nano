#include <Arduino.h>
#include <timers.h>
#define DEADBAND 1.0F

// MAGNETIC SENSOR INITIALIZATION
const int X_CS = A2;
const int X_CLK = A3;
const int X_DO = A4;
const int Y_CS = A5;
const int Y_CLK = 25;
const int Y_DO = 24;

// STATE MACHINE
const int CALIBRATE = 1;
const int READ = 2;
const int MOVE = 3;
const int OPEN = 4;
const int WAIT = 5;
int state = CALIBRATE;
int old_state = CALIBRATE;
bool axis_state = 0;       // 0 is X axis, 1 is Y axis
bool unusedXFlag = true;   // For tracking if offset is used
bool unusedYFlag = true;   // For tracking if offset is used
bool readComplete = false; // Checks if reading is complete before moving stages

// CALIBRATION
struct AxisStatus {
  volatile int piezoPosition;
  volatile int oldPiezoPosition;
  volatile int newPiezoPosition;
  volatile int offsetVal;
  bool calibrationDone;
  unsigned long startTime;
};

// INITIALIZING POSITION TRACKING
#define NUM_AXES 2  // X, Y
AxisStatus axes[NUM_AXES] = {
  { -15000, 0, 0, 0, false, 0 },  // X
  { -15000, 0, 0, 0, false, 0 },  // Y
};

const int calibrateRange = 2;
// unsigned long startTime         = 0;
const unsigned long duration = 2000;  // 2 seconds in milliseconds
const int startingFrequencyX = 700;
const int startingFrequencyY = -700;
const float countsToMicrons = 0.48828125;  // 2000 microns/2^12 counts

// FOR PID CONTROL
unsigned long executionDuration = 0;  // [microseconds] Time between this and the previous loop execution.  Variable used for integrals and derivatives
unsigned long lastExecutionTime = 0;  // [microseconds] System clock value at the moment the loop was started the last time
int targetXPosition = 0, targetYPosition = 0;
float positionErrorX = 0, positionErrorY = 0;
float integralErrorX = 0, integralErrorY = 0;
float velocityErrorX = 0, velocityErrorY = 0;
float desiredFrequencyX = 0, desiredFrequencyY = 0;
float piezoVelocityX = 0, piezoVelocityY = 0;
int previousPiezoPositionX = 0, previousPiezoPositionY = 0;
long previousVelCompTimeX = 0, previousVelCompTimeY = 0;
const int MIN_VEL_COMP_COUNT = 2;      // [encoder counts] Minimal change in piezo position that must happen between two velocity measurements
const long MIN_VEL_COMP_TIME = 10000;  // [microseconds] Minimal time that must pass between two velocity measurements
float KP = 1;                        // [Volt / encoder counts] P-Gain
float KD = 0.0005;                      // [Volt * seconds / encoder counts] D-Gain
float KI = 0;                      // [Volt / (encoder counts * seconds)] I-Gain

// Timing:
unsigned long startWaitTime;     // [microseconds] System clock value at the moment the WAIT state started
const long WAIT_TIME = 10000;  // [microseconds] Time waiting at each location
const int TARGET_BAND = 1;       // [encoder counts] "Close enough" range when moving towards a target.

// USER INPUT:
struct Coordinate {
  float xMicrons;
  float yMicrons;
  int xCounts;
  int yCounts;
};
int step_size_state = 0;
int targetX_open = 0;
int targetY_open = 0;
bool reached_pos = true;

#define NUM_COORDINATES 100
Coordinate coordinates[NUM_COORDINATES] = {
  {0, 0, 0, 0},
  {100, 100, 0, 0},
  {50, 50, 0, 0},
  {150, 150, 0, 0},
  {100, 100, 0, 0},
  {200, 200, 0, 0},
  {150, 150, 0, 0},
  {250, 250, 0, 0},
  {200, 200, 0, 0},
  {300, 300, 0, 0},
  {14000, 14000, 0, 0},
  {13950, 13950, 0, 0},
  {14450, 14450, 0, 0},
  {14400, 14400, 0, 0},
  {14900, 14900, 0, 0},
  {14850, 14850, 0, 0},
  {15350, 15350, 0, 0},
  {15300, 15300, 0, 0},
  {15800, 15800, 0, 0},
  {15750, 15750, 0, 0},
  {16250, 16250, 0, 0},
  {16200, 16200, 0, 0},
  {16700, 16700, 0, 0},
  {16650, 16650, 0, 0},
  {17150, 17150, 0, 0},
  {17100, 17100, 0, 0},
  {17600, 17600, 0, 0},
  {17550, 17550, 0, 0},
  {18050, 18050, 0, 0},
  {18000, 18000, 0, 0},
  {18500, 18500, 0, 0},
  {18450, 18450, 0, 0},
  {18950, 18950, 0, 0},
  {18900, 18900, 0, 0},
  {19400, 19400, 0, 0},
  {19350, 19350, 0, 0},
  {19850, 19850, 0, 0},
  {19800, 19800, 0, 0},
  {20300, 20300, 0, 0},
  {20250, 20250, 0, 0},
  {20750, 20750, 0, 0},
  {20700, 20700, 0, 0},
  {21200, 21200, 0, 0},
  {21150, 21150, 0, 0},
  {21650, 21650, 0, 0},
  {21600, 21600, 0, 0},
  {22100, 22100, 0, 0},
  {22050, 22050, 0, 0}
};
// Track which coordinate we're at:
int currentCoordinateIndex = 0; // Ignoring first one
// int currentCoordinateIndex = 1; // Ignoring first one

uint16_t read_bits(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder, uint8_t numBits) {
  uint16_t value = 0;
  uint16_t i;
  for (i = 0; i < numBits; i++) {
    digitalWrite(clockPin, HIGH);
    if (bitOrder == LSBFIRST) {
      delayMicroseconds(50);
      value |= digitalRead(dataPin) << i;
    } else {
      delayMicroseconds(50);
      value |= digitalRead(dataPin) << (numBits - 1 - i);
    }
    digitalWrite(clockPin, LOW);
  }
  return value;
}

int position_read(uint8_t csPin, uint8_t clockPin, uint8_t dataPin) {
  int position = 0;
  digitalWrite(clockPin, 1);
  digitalWrite(csPin, LOW);  // Start transfer
  digitalWrite(clockPin, 0);
  uint16_t read_position = read_bits(dataPin, clockPin, MSBFIRST, 12);
  uint8_t error = read_bits(dataPin, clockPin, MSBFIRST, 3);
  digitalWrite(csPin, HIGH);  // End transfer

  if (error == 4) {  // expect binary to be 100 to be good, which is 4 in decimal
    position = read_position;
    return position;
  } else {
    return position;
  }
}

int track_position(int current_pos, int new_pos, int old_pos, int index, bool offsetFlag, int offset) {
  int diff = new_pos - old_pos;
  if (offsetFlag & (unusedXFlag | unusedYFlag)) {
    current_pos = current_pos - offset;
    if (index == 0) {
      unusedXFlag = false;  // X is offsetted
    } else if (index == 1) {
      unusedYFlag = false;  // Y is offsetted
    }
  }
  if (abs(diff) > 4096 / 4) {  // wrap around occurred
    if (diff > 0) {            // backward motion occurred
      current_pos = current_pos - (4096 - new_pos) - old_pos;
    } else {  // forward motion occured
      current_pos = current_pos + (4096 - old_pos) + new_pos;
    }
  } else {  // no wrap around
    current_pos = current_pos + diff;
  }

  return current_pos;
}

void write_freqs(float hz_0, float hz_1) {
  // TODO: deadband... around zero...
  if (hz_0 < DEADBAND && hz_0 > -DEADBAND) {
    hz_0 = hz_0 < 0.0F ? -DEADBAND : DEADBAND;
  }
  if (hz_1 < DEADBAND && hz_1 > -DEADBAND) {
    hz_1 = hz_1 < 0.0F ? -DEADBAND : DEADBAND;
  }
  //
  float per_0 = 120000000.0F / (hz_0 * 4096.0F);
  float per_1 = 120000000.0F / (hz_1 * 4096.0F);
  //
  write_periods((int16_t)per_0, (int16_t)per_1);
}

void calibrateAxis(int axisIndex) {
  AxisStatus &axis = axes[axisIndex];
  if (axis.calibrationDone) return;

  if (axisIndex == 0) {
    axis.piezoPosition = position_read(X_CS, X_CLK, X_DO);
  } else if (axisIndex == 1) {
    axis.piezoPosition = position_read(Y_CS, Y_CLK, Y_DO);
  }
  if ((axis.piezoPosition >= axis.oldPiezoPosition - calibrateRange) && (axis.piezoPosition <= axis.oldPiezoPosition + calibrateRange)) {
    if (axis.startTime == 0) {
      axis.startTime = millis();
    } else if (millis() - axis.startTime >= duration) {
      // Serial.println("Positioner stopped moving");
      if (axisIndex == 0) {  // X-Axis
        write_freqs(0, startingFrequencyY);
        axes[0].offsetVal = axis.piezoPosition;
      } else if (axisIndex == 1) {  // Y-Axis
        write_freqs(0, 0);
        axes[1].offsetVal = axis.piezoPosition;
      }
      axis.calibrationDone = true;
      write_freqs(0, 0);  // Temporarily stops movements - calibration is done
      // Serial.print("Finished calibrating axis ");
      // Serial.println(axisIndex);
    }
  } else {
    axis.startTime = 0;
  }

  axis.oldPiezoPosition = axis.piezoPosition;
  delay(100);
}

bool receivingCoords = false;
int coordCount = 0;

void readSerialInput() {
  if (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    line.trim();

    if (line == "START_COORDS") {
      // Serial.println("Ready to receive coordinates");
      receivingCoords = true;
      coordCount = 0;
      return;
    }

    if (line == "END_COORDS") {
      // Serial.print("Received ");
      // Serial.print(coordCount);
      // Serial.println(" coordinates total.");
      receivingCoords = false;
      readComplete = true;  // Coordinate reading done 
      return;
    }
    if (receivingCoords) {
      int commaIndex = line.indexOf(',');
      if (commaIndex > 0) {
        String xStr = line.substring(0, commaIndex);
        String yStr = line.substring(commaIndex + 1);
        float xVal = xStr.toFloat();
        float yVal = yStr.toFloat();

        // Store or process coordinate
        coordinates[coordCount].xMicrons = xVal;
        coordinates[coordCount].yMicrons = yVal;
        coordinates[coordCount].xCounts = xVal / countsToMicrons;
        coordinates[coordCount].yCounts = yVal / countsToMicrons;

        coordCount++;
        // Serial.println("ACK");
      }
    }
    
  }
}

void display_position(){
  // TRACKING X POSITION
  axes[0].newPiezoPosition = position_read(X_CS, X_CLK, X_DO);
  axes[0].piezoPosition = track_position(axes[0].piezoPosition, axes[0].newPiezoPosition, axes[0].oldPiezoPosition, 0, true, axes[0].offsetVal);
  axes[0].oldPiezoPosition = axes[0].newPiezoPosition;

  // TRACKING Y POSITION
  axes[1].newPiezoPosition = position_read(Y_CS, Y_CLK, Y_DO);
  axes[1].piezoPosition = track_position(axes[1].piezoPosition, axes[1].newPiezoPosition, axes[1].oldPiezoPosition, 1, true, axes[1].offsetVal);
  axes[1].oldPiezoPosition = axes[1].newPiezoPosition;
  Serial.print("X");
  Serial.print(axes[0].piezoPosition * countsToMicrons);
  Serial.print(";");
  Serial.print("Y");
  Serial.println(axes[1].piezoPosition * countsToMicrons);  
}

bool closed_loop_positioning(int targetX, int targetY){
  int limit = 500;
  executionDuration = micros() - lastExecutionTime;
  lastExecutionTime = micros();

  // --------- X Axis ---------
  if ((abs(axes[0].piezoPosition - previousPiezoPositionX) > MIN_VEL_COMP_COUNT) || (micros() - previousVelCompTimeX) > MIN_VEL_COMP_TIME) {
    piezoVelocityX = (double)(axes[0].piezoPosition - previousPiezoPositionX) * 1000000 / (micros() - previousVelCompTimeX);
    previousPiezoPositionX = axes[0].piezoPosition;
    previousVelCompTimeX = micros();
  }
  positionErrorX = targetX - axes[0].piezoPosition;
  integralErrorX += positionErrorX * (float)(executionDuration) / 1000000;
  velocityErrorX = 0 - piezoVelocityX;
  desiredFrequencyX = KP * positionErrorX + KI * integralErrorX + KD * velocityErrorX;

  if (desiredFrequencyX >= limit) desiredFrequencyX = limit;
  else if (desiredFrequencyX <= -limit) desiredFrequencyX = -limit;
  else if ((desiredFrequencyX >= 0) && (desiredFrequencyX <= 10)) desiredFrequencyX = 3;
  else if ((desiredFrequencyX <= 0) && (desiredFrequencyX >= -10)) desiredFrequencyX = -3;

  // --------- Y Axis ---------
  if ((abs(axes[1].piezoPosition - previousPiezoPositionY) > MIN_VEL_COMP_COUNT) || (micros() - previousVelCompTimeY) > MIN_VEL_COMP_TIME) {
    piezoVelocityY = (double)(axes[1].piezoPosition - previousPiezoPositionY) * 1000000 / (micros() - previousVelCompTimeY);
    previousPiezoPositionY = axes[1].piezoPosition;
    previousVelCompTimeY = micros();
  }
  positionErrorY = targetY - axes[1].piezoPosition;
  integralErrorY += positionErrorY * (float)(executionDuration) / 1000000;
  velocityErrorY = 0 - piezoVelocityY;
  desiredFrequencyY = KP * positionErrorY + KI * integralErrorY + KD * velocityErrorY;

  if (desiredFrequencyY >= limit) desiredFrequencyY = limit;
  else if (desiredFrequencyY <= -limit) desiredFrequencyY = -limit;
  else if ((desiredFrequencyY >= 0) && (desiredFrequencyY <= 10)) desiredFrequencyY = 10;
  else if ((desiredFrequencyY <= 0) && (desiredFrequencyY >= -10)) desiredFrequencyY = -10;

  write_freqs(-desiredFrequencyX, desiredFrequencyY);

  // Target position check (both axes within TARGET_BAND)
  bool xReached = abs(axes[0].piezoPosition - targetX) <= TARGET_BAND;
  bool yReached = abs(axes[1].piezoPosition - targetY) <= TARGET_BAND;

  return xReached & yReached;
}

void setup() {
  setup_timers();

  // POSITION TRACKING
  pinMode(X_CS, OUTPUT);
  pinMode(X_CLK, OUTPUT);
  pinMode(X_DO, INPUT);
  pinMode(Y_CS, OUTPUT);
  pinMode(Y_CLK, OUTPUT);
  pinMode(Y_DO, INPUT);
  digitalWrite(X_CLK, 1);  //CLK
  digitalWrite(X_CS, 1);   //Csn
  digitalWrite(Y_CLK, 1);  //CLK
  digitalWrite(Y_CS, 1);   //Csn

  // INITIALIZE SERIAL MONITOR
  Serial.begin(115200);
  while (!Serial) {
    // wait until Serial is connected
  }

  // Start DAC
  // write_freqs(600, 600);
  write_freqs(0, 0);

  // Start position tracking
  axes[0].oldPiezoPosition = position_read(X_CS, X_CLK, X_DO);
  axes[1].oldPiezoPosition = position_read(Y_CS, Y_CLK, Y_DO);

  // Convert all positions from microns to encoder counts
  for (int i = 0; i < NUM_COORDINATES; i++) {
    coordinates[i].xCounts = coordinates[i].xMicrons / countsToMicrons;
    coordinates[i].yCounts = coordinates[i].yMicrons / countsToMicrons;
  }

  Serial.println("Connected");
}

unsigned long startTime = millis();
String incomingString;

void loop() {
  switch (state) {
    case CALIBRATE:
      {
        write_freqs(startingFrequencyX, startingFrequencyY);  // Or update to support more axes if needed
        bool allDone = true;
        for (int i = 0; i < NUM_AXES; i++) {
          calibrateAxis(i);
          allDone = allDone && axes[i].calibrationDone;
        }
        if (allDone) {
          write_freqs(0, 0);
          state = WAIT;
          startWaitTime = micros();
        }
        break;
      }
    case READ:
    {
      readSerialInput();
      // Wait until all coordinates have been received before moving on
      if (readComplete) {
        // Serial.println("All coordinates received, moving to MOVE state");
        readComplete = false;  // reset for next round, if needed
        // currentCoordinateIndex = 0; // reset index for movement
        old_state = READ;
        state = MOVE;
      }
      break;
    }
    case MOVE:
      {
        // Set targets
        targetXPosition = coordinates[currentCoordinateIndex].xCounts;
        targetYPosition = coordinates[currentCoordinateIndex].yCounts;

        // Serial.print("Coordinates: ");
        // Serial.println(currentCoordinateIndex);

        while (!reached_pos){
          display_position();
          reached_pos = closed_loop_positioning(targetXPosition, targetYPosition);
          delay(5);  // small delay for loop stability
        }
        if (reached_pos) {
          Serial.println("YOU REACHED THE POSITION");
          write_freqs(0, 0);
          startWaitTime = micros();
          old_state = MOVE;
          state = WAIT;
        }
        break;
      }

    case OPEN:
      {
        old_state = OPEN;
        while (true) {
          display_position();

          // Check for input from Python
          if (Serial.available() > 0) {
            char command = Serial.read(); 
            if (command == 'F'){
              Serial.println("Received F command");
              step_size_state = 0;
            }
            else if (command == 'G'){
              Serial.println("Received G command");
              step_size_state = 1000;
            }
            else if (command == 'H'){
              Serial.println("Received H command");
              step_size_state = 100;
            }
            else if (command == 'J'){
              Serial.println("Received J command");
              step_size_state = 10;
            }
            if (step_size_state > 0){
              // Serial.print("Target X: ");
              // Serial.println(targetX_open);
              // Serial.print("Target Y: ");
              // Serial.println(targetY_open);
              while (!reached_pos){
                display_position();
                reached_pos = closed_loop_positioning(targetX_open/countsToMicrons, targetY_open/countsToMicrons);
                delay(5);  // small delay for loop stability
              }
              if (reached_pos) {
                Serial.println("YOU REACHED THE POSITION");
                write_freqs(0, 0);
                startWaitTime = micros();
                integralErrorX = 0;
                integralErrorY = 0;
              }
              if (command == 'L') {
                if ((axes[0].piezoPosition - step_size_state) > 0){
                  reached_pos = false;
                  targetX_open = axes[0].piezoPosition*countsToMicrons - step_size_state;
                  targetY_open = axes[1].piezoPosition*countsToMicrons;
                }
              } 
              else if (command == 'R') {
                if ((axes[0].piezoPosition + step_size_state) < 25000/countsToMicrons){
                  reached_pos = false;
                  targetX_open = axes[0].piezoPosition*countsToMicrons + step_size_state;
                  targetY_open = axes[1].piezoPosition*countsToMicrons;
                }
              }
              else if (command == 'U') {
                if ((axes[1].piezoPosition + step_size_state) < 25000/countsToMicrons){
                  reached_pos = false;
                  targetY_open = axes[1].piezoPosition*countsToMicrons + step_size_state;
                  targetX_open = axes[0].piezoPosition*countsToMicrons;
                }
              } 
              else if (command == 'D') {
                if ((axes[1].piezoPosition - step_size_state) > 0){
                  reached_pos = false;
                  targetY_open = axes[1].piezoPosition*countsToMicrons - step_size_state;
                  targetX_open = axes[0].piezoPosition*countsToMicrons;
                }              
              }
            }
            else if (step_size_state == 0){
              if (command == 'S') {
                write_freqs(0, 0);
              }
              else if (command == 'L') {
                write_freqs(-700, 0); 
              } 
              else if (command == 'R') {
                write_freqs(700, 0); 
              }
              else if (command == 'U') {
                write_freqs(0, 700);
              } 
              else if (command == 'D') {
                write_freqs(0, -700);
              }
            }
          }
          delay(10); // small delay to prevent overload
        }
        break;
      }


    case WAIT:
      if (micros() - startWaitTime > WAIT_TIME) {  // enter WAIT after a certain amount of time
        if (old_state == CALIBRATE) {
          // state = MOVE;
          state = OPEN;
          
          Serial.println("State transition from CALIBRATE to READ");
        }
        if (old_state == OPEN){
          while(1);
          // Serial.println("STOP at OPEN");
        }
        if (old_state == READ) {
          state = MOVE;
          Serial.println("State transition from READ to MOVE");
        }
        if (old_state == MOVE) {
          integralErrorX = 0;
          integralErrorY = 0;
          currentCoordinateIndex++;
          reached_pos = false;
          if (currentCoordinateIndex < NUM_COORDINATES) {
            state = MOVE;  // Jump to next coordinate
            Serial.println("State transition to next coordinate");
            delay(1000);
          } else {
            Serial.println("All coordinates completed!");
            while (1);  // Stop program
          }
        }
      }
      break;

    default:
      // Serial.println("State machine reached a state that it cannot handle.  ABORT!!!!");
      // Serial.print("Found the following unknown state: ");
      // Serial.println(state);
      while (1)
        ;  // infinite loop to halt the program
      break;
  }

  if (state == MOVE) {
    display_position();
  }
}
