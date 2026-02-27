#include <Arduino.h>
#include <timers.h>
#define DEADBAND 1.0F

//// LOW PASS FILTER ////
template <int order> // order is 1 or 2
class LowPass
{
  private:
    float a[order];
    float b[order+1];
    float omega0;
    float dt;
    bool adapt;
    float tn1 = 0;
    float x[order+1]; // Raw values
    float y[order+1]; // Filtered values

  public:  
    LowPass(float f0, float fs, bool adaptive){
      // f0: cutoff frequency (Hz)
      // fs: sample frequency (Hz)
      // adaptive: boolean flag, if set to 1, the code will automatically set
      // the sample frequency based on the time history.
      
      omega0 = 6.28318530718*f0;
      dt = 1.0/fs;
      adapt = adaptive;
      tn1 = -dt;
      for(int k = 0; k < order+1; k++){
        x[k] = 0;
        y[k] = 0;        
      }
      setCoef();
    }

    void setCoef(){
      if(adapt){
        float t = micros()/1.0e6;
        dt = t - tn1;
        tn1 = t;
      }
      
      float alpha = omega0*dt;
      if(order==1){
        a[0] = -(alpha - 2.0)/(alpha+2.0);
        b[0] = alpha/(alpha+2.0);
        b[1] = alpha/(alpha+2.0);        
      }
      if(order==2){
        float alphaSq = alpha*alpha;
        float beta[] = {1, sqrt(2), 1};
        float D = alphaSq*beta[0] + 2*alpha*beta[1] + 4*beta[2];
        b[0] = alphaSq/D;
        b[1] = 2*b[0];
        b[2] = b[0];
        a[0] = -(2*alphaSq*beta[0] - 8*beta[2])/D;
        a[1] = -(beta[0]*alphaSq - 2*beta[1]*alpha + 4*beta[2])/D;      
      }
    }

    float filt(float xn){
      // Provide me with the current raw value: x
      // I will give you the current filtered value: y
      if(adapt){
        setCoef(); // Update coefficients if necessary      
      }
      y[0] = 0;
      x[0] = xn;
      // Compute the filtered values
      for(int k = 0; k < order; k++){
        y[0] += a[k]*y[k+1] + b[k]*x[k];
      }
      y[0] += b[order]*x[order];

      // Save the historical values
      for(int k = order; k > 0; k--){
        y[k] = y[k-1];
        x[k] = x[k-1];
      }
  
      // Return the filtered value    
      return y[0];
    }
};

LowPass<1> lp(4, 200, true);

//// PIN INITIALIZATION ////
#define TIMING_PIN SCL     // Synchronization with Keyence Sensor
#define X_CS 11            // Magnetic encoder setup
#define X_CLK 12           // Magnetic encoder setup
#define X_DO 13            // Magnetic encoder setup
#define X_CHANNEL_A 10     // Posic encoder setup
#define X_CHANNEL_B 6      // Posic encoder setup
#define X_CHANNEL_NOT_A 9  // Posic encoder setup
#define X_CHANNEL_NOT_B 5  // Posic encoder setup

//// KEYENCE SYNCHRONIZATION ////
void triggerMeasurement() {
  digitalWrite(TIMING_PIN, LOW);   // Trigger (short to ground)
  delay(1);           // Minimum 1000μs hold time
  digitalWrite(TIMING_PIN, HIGH);  // Release
  delay(1);           // Minimum 1000μs hold time
}

//// STATE MACHINE ////
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

//// ENCODER TRACKING ////
volatile int xEncoderStatus  = 0;      // [binary] Past and Current A&B values of the encoder  (Declared 'volatile', since it is updated in a function called by interrupts)
volatile long xEncoderCounts = 0;

struct AxisStatus { // units are in microns
  volatile int piezoPosition;
  volatile int oldPiezoPosition;
  volatile int newPiezoPosition;
  volatile int offsetVal;
  bool calibrationDone;
  unsigned long startTime;
};
#define NUM_AXES 2  // X, Y
AxisStatus axes[NUM_AXES] = {
  { 0, 0, 0, 0, false, 0 },  // X
  { 0, 0, 0, 0, false, 0 },  // Y
};

//// TIMING
unsigned long startWaitTime;     // [microseconds] System clock value at the moment the WAIT state started
const long WAIT_TIME = 10000;  // [microseconds] Time waiting at each location
const int TARGET_BAND = 0;       // [encoder counts] "Close enough" range when moving towards a target.

// int step_size_state = 0;
// int targetX_open = 0;
// int targetY_open = 0;
bool reached_pos = false;

//// INPUTS ////
struct Coordinate {
  float xMicrons;
  float yMicrons;
  int xCounts;
  int yCounts;
};
#define NUM_COORDINATES 19
Coordinate coordinates[NUM_COORDINATES] = {
  {-100, 0, 0, 0},
  {-200, 0, 0, 0},
  {-300, 0, 0, 0},
  {-400, 0, 0, 0},
  {-500, 0, 0, 0},
  {-600, 0, 0, 0},
  {-700, 0, 0, 0},
  {-800, 0, 0, 0},
  {-900, 0, 0, 0},
  {-1000, 0, 0, 0},
  {-900, 0, 0, 0},
  {-800, 0, 0, 0},
  {-700, 0, 0, 0},
  {-600, 0, 0, 0},
  {-500, 0, 0, 0},
  {-400, 0, 0, 0},
  {-300, 0, 0, 0},
  {-200, 0, 0, 0},
  {-100, 0, 0, 0}
};
// Track which coordinate we're at:
int currentCoordinateIndex = 0; // Ignoring first one

void display_position(){
  // TRACKING X POSITION
  // axes[0].newPiezoPosition = position_read(X_CS, X_CLK, X_DO);
  axes[0].newPiezoPosition = position_read(X_CS, X_CLK, X_DO);
  axes[1].newPiezoPosition = position_read(X_CS, X_CLK, X_DO);
  axes[0].newPiezoPosition = lp.filt(axes[0].newPiezoPosition);
  // axes[0].newPiezoPosition = axes[0].newPiezoPosition * 0.9459754389;
  axes[0].piezoPosition = track_position(axes[0].piezoPosition, axes[0].newPiezoPosition, axes[0].oldPiezoPosition, 0, true, axes[0].offsetVal);
  // axes[0].piezoPosition = lp.filt(axes[0].piezoPosition);
  axes[1].piezoPosition = track_position(axes[1].piezoPosition, axes[1].newPiezoPosition, axes[1].oldPiezoPosition, 0, true, axes[0].offsetVal);
  // Serial.print(", Current pos: ");
  // Serial.println(axes[0].piezoPosition);
  axes[0].oldPiezoPosition = axes[0].newPiezoPosition;
  axes[1].oldPiezoPosition = axes[1].newPiezoPosition;
  // Serial.print(", Old pos: ");
  // Serial.println(axes[0].oldPiezoPosition);
  // delay(2000);

  // // TRACKING Y POSITION
  // axes[1].newPiezoPosition = position_read(Y_CS, Y_CLK, Y_DO);
  // axes[1].piezoPosition = track_position(axes[1].piezoPosition, axes[1].newPiezoPosition, axes[1].oldPiezoPosition, 1, true, axes[1].offsetVal);
  // axes[1].oldPiezoPosition = axes[1].newPiezoPosition;
  // // Serial.print("X");
  // // Serial.print(axes[0].piezoPosition * countsToMicrons);
  // // Serial.print(";");
  // // Serial.print("Y");

  float xn = axes[0].piezoPosition;
  float yn = axes[1].piezoPosition;

  // Compute the filtered signal
  // float yn = lp.filt(xn);

  // Output
  // Serial.print(millis());
  // Serial.print(", ");
  // Serial.print(xn *  0.48828125 * 0.9459754389);
  Serial.print(", Filtered pos: ");
  Serial.print(xn * 0.9459754389);
  Serial.print(", Unfiltered pos: ");
  Serial.println(yn * 0.9459754389);

  // // Serial.print(", ");
  // // Serial.print(yn);
  // Serial.print(", ");
  // Serial.println(xEncoderCounts);

  // Serial.print(millis());
  // Serial.print(", ");
  // // Serial.println(axes[1].piezoPosition * countsToMicrons * 0.9472702863);  
  // Serial.println(axes[1].piezoPosition);  
}

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
  Serial.print("Diff: ");
  Serial.print(diff);
  Serial.print(", ");
  int max_float_limit = 4096;
  if (offsetFlag & (unusedXFlag | unusedYFlag)) {
    current_pos = current_pos - offset;
    if (index == 0) {
      unusedXFlag = false;  // X is offsetted
      unusedYFlag = false;  // Y is offsetted, will need to delete in the future
      index ++;
    } else if (index == 1) {
      unusedYFlag = false;  // Y is offsetted
      index ++;
    }
  }
  if (abs(diff) > max_float_limit / 4) {  // wrap around occurred
    if (diff > 0) {            // backward motion occurred
      current_pos = current_pos - (max_float_limit - new_pos) - old_pos;
    } else {  // forward motion occured
      current_pos = current_pos + (max_float_limit - old_pos) + new_pos;
    }
  } 
  else {  // no wrap around
    current_pos = current_pos + diff;
  }
  return current_pos;
}
void updateXPosition() {
  // isr_start_time = micros();
  xEncoderStatus <<= 1;   
  if (digitalRead(X_CHANNEL_A) != digitalRead(X_CHANNEL_NOT_A)){
    xEncoderStatus |= digitalRead(X_CHANNEL_A);   
  }
  xEncoderStatus <<= 1;
  if (digitalRead(X_CHANNEL_B) != digitalRead(X_CHANNEL_NOT_B)){
    xEncoderStatus |= digitalRead(X_CHANNEL_B);   
  }
  xEncoderStatus &= 15;
  if (xEncoderStatus==2 || xEncoderStatus==4 || xEncoderStatus==11 || xEncoderStatus==13) {
    xEncoderCounts ++;         // increase the encoder count by one
  } 
  else if (xEncoderStatus==1 || xEncoderStatus==7 || xEncoderStatus==8 || xEncoderStatus==14) {
    xEncoderCounts --;         // decrease the encoder count by one
  }
  // isr_end_time = micros();
  // last_isr_duration = isr_end_time - isr_start_time;
}

//// CALIBRATION ////
const int calibrateRange = 2;
// unsigned long startTime         = 0;
const unsigned long duration = 2000;       // 2 seconds in milliseconds
const int startingFrequencyX = -700;
const int startingFrequencyY = 0;
const float countsToMicrons = 0.48828125;  // 2000 microns/2^12 counts
void calibrateAxis(int axisIndex) {
  AxisStatus &axis = axes[axisIndex];
  if (axis.calibrationDone) return;

  if (axisIndex == 0) {
    axis.piezoPosition = position_read(X_CS, X_CLK, X_DO);
  } 
  // else if (axisIndex == 1) {
  //   axis.piezoPosition = position_read(Y_CS, Y_CLK, Y_DO);
  // }
  if ((axis.piezoPosition >= axis.oldPiezoPosition - calibrateRange) && (axis.piezoPosition <= axis.oldPiezoPosition + calibrateRange)) {
    if (axis.startTime == 0) {
      axis.startTime = millis();
    } else if (millis() - axis.startTime >= duration) {
      // Serial.println("Positioner stopped moving");
      if (axisIndex == 0) {  // X-Axis
        write_freqs(0, startingFrequencyY);
        axes[0].offsetVal = axis.piezoPosition;
        // Serial.print("Offset val for x is: ");
        // Serial.println(axes[0].offsetVal);
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

//// PID CONTROL ////
unsigned long executionDuration = 0;                        // [microseconds] Time between this and the previous loop execution.  Variable used for integrals and derivatives
unsigned long lastExecutionTime = 0;                        // [microseconds] System clock value at the moment the loop was started the last time
int targetXPosition = 0, targetYPosition = 0;
float positionErrorX = 0, positionErrorY = 0;
float integralErrorX = 0, integralErrorY = 0;
float velocityErrorX = 0, velocityErrorY = 0;
float desiredFrequencyX = 0, desiredFrequencyY = 0;
float piezoVelocityX = 0, piezoVelocityY = 0;
int previousPiezoPositionX = 0, previousPiezoPositionY = 0;
long previousVelCompTimeX = 0, previousVelCompTimeY = 0;
const int MIN_VEL_COMP_COUNT = 2;                            // [encoder counts] Minimal change in piezo position that must happen between two velocity measurements
const long MIN_VEL_COMP_TIME = 10000;                        // [microseconds] Minimal time that must pass between two velocity measurements
// float KP = 1;                                                // [Volt / encoder counts] P-Gain
// float KD = 0.0005;                                           // [Volt * seconds / encoder counts] D-Gain
// float KI = 0;                                                // [Volt / (encoder counts * seconds)] I-Gain
float KP = 4;                                                // [Volt / encoder counts] P-Gain
float KD = 1;                                           // [Volt * seconds / encoder counts] D-Gain
float KI = 0.01;                                                // [Volt / (encoder counts * seconds)] I-Gain

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
  // else if ((desiredFrequencyX >= 0) && (desiredFrequencyX <= 10)) desiredFrequencyX = 1;
  // else if ((desiredFrequencyX <= 0) && (desiredFrequencyX >= -10)) desiredFrequencyX = -1;

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
  else if ((desiredFrequencyY >= 0) && (desiredFrequencyY <= 10)) desiredFrequencyY = 1;
  else if ((desiredFrequencyY <= 0) && (desiredFrequencyY >= -10)) desiredFrequencyY = -1;

  // Serial.println(desiredFrequencyX);

  write_freqs(-desiredFrequencyX, desiredFrequencyY);

  // Serial.print("Target pos: ");
  // Serial.print(targetX);
  // Target position check (both axes within TARGET_BAND)
  bool xReached = abs(axes[0].piezoPosition - targetX) <= TARGET_BAND;
  // bool yReached = abs(axes[1].piezoPosition - targetY) <= TARGET_BAND;
  bool yReached = 1; // temporary until I get the y stage running too

  // Serial.print(axes[0].piezoPosition - targetX);
  // Serial.print(", ");

  return xReached & yReached;
}

//// SIGNAL GENERATION ////
void write_freqs(float hz_0, float hz_1) {
  int16_t per_0, per_1;
 
  // Handle hz_0
  if (hz_0 == 0.0F) {
    // Suspend DMAC channel 0 to stop DAC0 updates
    DMAC->Channel[0].CHCTRLA.bit.ENABLE = 0;
    // Optionally set DAC0 to a safe/neutral value
    analogWrite(DAC0, 2048);  // Mid-point
    per_0 = 100;  // Arbitrary value, won't be used
  } else {
    // Apply deadband for non-zero values
    if (hz_0 < DEADBAND && hz_0 > -DEADBAND) {
      hz_0 = hz_0 < 0.0F ? -DEADBAND : DEADBAND;
    }
    per_0 = (int16_t)(120000000.0F / (hz_0 * 4096.0F));
    
    // Re-enable DMAC channel 0 if it was disabled
    if (!DMAC->Channel[0].CHCTRLA.bit.ENABLE) {
      DMAC->Channel[0].CHCTRLA.bit.ENABLE = 1;
    }
  }
  
  // Handle hz_1
  if (hz_1 == 0.0F) {
    // Suspend DMAC channel 2 to stop DAC1 updates
    DMAC->Channel[2].CHCTRLA.bit.ENABLE = 0;
    // Optionally set DAC1 to a safe/neutral value
    analogWrite(DAC1, 2048);  // Mid-point
    per_1 = 100;  // Arbitrary value, won't be used
  } else {
    // Apply deadband for non-zero values
    if (hz_1 < DEADBAND && hz_1 > -DEADBAND) {
      hz_1 = hz_1 < 0.0F ? -DEADBAND : DEADBAND;
    }
    per_1 = (int16_t)(120000000.0F / (hz_1 * 4096.0F));
    
    // Re-enable DMAC channel 2 if it was disabled
    if (!DMAC->Channel[2].CHCTRLA.bit.ENABLE) {
      DMAC->Channel[2].CHCTRLA.bit.ENABLE = 1;
    }
  }
  
  // Write the periods (TCC timers keep running)
  write_periods(per_0, per_1);
}

void setup() {
  setup_timers();

  // POSITION TRACKING
  pinMode(X_CS, OUTPUT);
  pinMode(X_CLK, OUTPUT);
  pinMode(X_DO, INPUT);
  // pinMode(Y_CS, OUTPUT);
  // pinMode(Y_CLK, OUTPUT);
  // pinMode(Y_DO, INPUT);
  digitalWrite(X_CLK, 1);  //CLK
  digitalWrite(X_CS, 1);   //Csn
  // digitalWrite(Y_CLK, 1);  //CLK
  // digitalWrite(Y_CS, 1);   //Csn

  pinMode(TIMING_PIN, OUTPUT);
  digitalWrite(TIMING_PIN, HIGH);  // Idle LOW for NPN

  pinMode(X_CHANNEL_A, INPUT);
  pinMode(X_CHANNEL_B, INPUT);
  pinMode(X_CHANNEL_NOT_A, INPUT);
  pinMode(X_CHANNEL_NOT_B, INPUT);

  attachInterrupt(digitalPinToInterrupt(X_CHANNEL_A), updateXPosition, CHANGE);
  attachInterrupt(digitalPinToInterrupt(X_CHANNEL_B), updateXPosition, CHANGE);

  // INITIALIZE SERIAL MONITOR
  Serial.begin(115200);
  while (!Serial) {
    // wait until Serial is connected
  }

  // // Start position tracking
  axes[0].oldPiezoPosition = position_read(X_CS, X_CLK, X_DO);
  axes[1].oldPiezoPosition = position_read(X_CS, X_CLK, X_DO);
  // axes[1].oldPiezoPosition = position_read(Y_CS, Y_CLK, Y_DO);

  // Convert all positions from microns to encoder counts
  for (int i = 0; i < NUM_COORDINATES; i++) {
    coordinates[i].xCounts = coordinates[i].xMicrons / countsToMicrons / 0.9459754389;
    coordinates[i].yCounts = coordinates[i].yMicrons / countsToMicrons / 0.9459754389;
    // coordinates[i].xCounts = coordinates[i].xMicrons / countsToMicrons;
    // coordinates[i].yCounts = coordinates[i].yMicrons / countsToMicrons;
  }

  // Serial.println("Connected");
  write_freqs(0,0);
  delay(2000);
}

void loop() {
  switch (state) 
  {
    case CALIBRATE:
      {
        write_freqs(startingFrequencyX, startingFrequencyY);  // Or update to support more axes if needed
        bool allDone = true;
        // for (int i = 0; i < NUM_AXES; i++) {
        for (int i = 0; i < NUM_AXES-1; i++) {
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
    // case READ:
    // {
    //   readSerialInput();
    //   // Wait until all coordinates have been received before moving on
    //   if (readComplete) {
    //     // Serial.println("All coordinates received, moving to MOVE state");
    //     readComplete = false;  // reset for next round, if needed
    //     // currentCoordinateIndex = 0; // reset index for movement
    //     old_state = READ;
    //     state = MOVE;
    //   }
    //   break;
    // }
    case MOVE:
      {
        // Set targets
        targetXPosition = coordinates[currentCoordinateIndex].xCounts;
        targetYPosition = coordinates[currentCoordinateIndex].yCounts;

        // Serial.print("Coordinates: ");
        // Serial.println(currentCoordinateIndex);

        while (!reached_pos){
          triggerMeasurement(); 
          display_position();
          reached_pos = closed_loop_positioning(targetXPosition, targetYPosition);
          delay(5);  // small delay for loop stability
        }
        if (reached_pos) {
          // Serial.println("YOU REACHED THE POSITION");
          write_freqs(0, 0);
          delay(2000);
          startWaitTime = micros();
          old_state = MOVE;
          state = WAIT;
        }
        break;
      }

  //   case OPEN:
  //     {
  //       old_state = OPEN;
  //       while (true) {
  //         display_position();

  //         // Check for input from Python
  //         if (Serial.available() > 0) {
  //           char command = Serial.read(); 
  //           if (command == 'F'){
  //             Serial.println("Received F command");
  //             step_size_state = 0;
  //           }
  //           else if (command == 'G'){
  //             Serial.println("Received G command");
  //             step_size_state = 1000;
  //           }
  //           else if (command == 'H'){
  //             Serial.println("Received H command");
  //             step_size_state = 100;
  //           }
  //           else if (command == 'J'){
  //             Serial.println("Received J command");
  //             step_size_state = 10;
  //           }
  //           if (step_size_state > 0){
  //             // Serial.print("Target X: ");
  //             // Serial.println(targetX_open);
  //             // Serial.print("Target Y: ");
  //             // Serial.println(targetY_open);
  //             while (!reached_pos){
  //               display_position();
  //               reached_pos = closed_loop_positioning(targetX_open/countsToMicrons, targetY_open/countsToMicrons);
  //               delay(5);  // small delay for loop stability
  //             }
  //             if (reached_pos) {
  //               Serial.println("YOU REACHED THE POSITION");
  //               write_freqs(0, 0);
  //               startWaitTime = micros();
  //               integralErrorX = 0;
  //               integralErrorY = 0;
  //             }
  //             if (command == 'L') {
  //               if ((axes[0].piezoPosition - step_size_state) > 0){
  //                 reached_pos = false;
  //                 targetX_open = axes[0].piezoPosition*countsToMicrons - step_size_state;
  //                 targetY_open = axes[1].piezoPosition*countsToMicrons;
  //               }
  //             } 
  //             else if (command == 'R') {
  //               if ((axes[0].piezoPosition + step_size_state) < 25000/countsToMicrons){
  //                 reached_pos = false;
  //                 targetX_open = axes[0].piezoPosition*countsToMicrons + step_size_state;
  //                 targetY_open = axes[1].piezoPosition*countsToMicrons;
  //               }
  //             }
  //             else if (command == 'U') {
  //               if ((axes[1].piezoPosition + step_size_state) < 25000/countsToMicrons){
  //                 reached_pos = false;
  //                 targetY_open = axes[1].piezoPosition*countsToMicrons + step_size_state;
  //                 targetX_open = axes[0].piezoPosition*countsToMicrons;
  //               }
  //             } 
  //             else if (command == 'D') {
  //               if ((axes[1].piezoPosition - step_size_state) > 0){
  //                 reached_pos = false;
  //                 targetY_open = axes[1].piezoPosition*countsToMicrons - step_size_state;
  //                 targetX_open = axes[0].piezoPosition*countsToMicrons;
  //               }              
  //             }
  //           }
  //           else if (step_size_state == 0){
  //             if (command == 'S') {
  //               write_freqs(0, 0);
  //             }
  //             else if (command == 'L') {
  //               write_freqs(-700, 0); 
  //             } 
  //             else if (command == 'R') {
  //               write_freqs(700, 0); 
  //             }
  //             else if (command == 'U') {
  //               write_freqs(0, 700);
  //             } 
  //             else if (command == 'D') {
  //               write_freqs(0, -700);
  //             }
  //           }
  //         }
  //         delay(10); // small delay to prevent overload
  //       }
  //       break;
  //     }


    case WAIT:
    {
      if (micros() - startWaitTime > WAIT_TIME) {  // enter WAIT after a certain amount of time
        if (old_state == CALIBRATE) {
          // state = MOVE;
          state = MOVE;
          
          // Serial.println("State transition from CALIBRATE to MOVE");
        }
        if (old_state == OPEN){
          while(1);
          // Serial.println("STOP at OPEN");
        }
        if (old_state == READ) {
          state = MOVE;
          // Serial.println("State transition from READ to MOVE");
        }
        if (old_state == MOVE) {
          integralErrorX = 0;
          integralErrorY = 0;
          currentCoordinateIndex++;
          reached_pos = false;
          if (currentCoordinateIndex < NUM_COORDINATES) {
            state = MOVE;  // Jump to next coordinate
            // Serial.println("State transition to next coordinate");
            delay(1000);
          } else {
            // Serial.println("All coordinates completed!");
            while (1);  // Stop program
          }
        }
      }
      break;
    }

    default:
    {
      // Serial.println("State machine reached a state that it cannot handle.  ABORT!!!!");
      // Serial.print("Found the following unknown state: ");
      // Serial.println(state);
      while (1)
        ;  // infinite loop to halt the program
      break;
    }

    // if (state == MOVE) {
    //   display_position();
    // }
  }
}
