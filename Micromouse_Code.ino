#include <Wire.h>
#include <VL53L0X.h>

// ============================================================================
// HARDWARE PIN MAPPING (ESP32 Core 3.x)
// ============================================================================
#define PIN_STBY     19
#define PIN_PWMA     25
#define PIN_AIN1     26
#define PIN_AIN2     27
#define PIN_PWMB     32
#define PIN_BIN1     33
#define PIN_BIN2     23

#define PIN_SDA_MAIN 21
#define PIN_SCL_MAIN 22
#define TCA_ADDR     0x70
#define AS5600_ADDR  0x36

#define PIN_SDA_GYRO 16
#define PIN_SCL_GYRO 17
#define MPU_ADDR     0x68

#define CH_TOF_LEFT   0
#define CH_TOF_DIAG_L 1
#define CH_TOF_CENTER 2
#define CH_TOF_DIAG_R 3
#define CH_TOF_RIGHT  4
#define CH_ENC_LEFT   5
#define CH_ENC_RIGHT  6

// ============================================================================
// CONFIGURATION & TUNING
// ============================================================================
const int SAFE_DIST_MIN     = 45;  
const int SAFE_DIST_MAX     = 80;  
const int FRONT_TURN_DIST   = 100;  

const float BASE_RPM        = 200.0; 
const float MAX_STEER_RPM   = 50.0;  

const float DECEL_ZONE_START_MM = 300.0f; 
const float APPROACH_RPM        = 55.0f;  

const float SAFE_DIST_MID   = (SAFE_DIST_MIN + SAFE_DIST_MAX) / 2.0f;
const float KP_CENTER       = 0.5f;   
const float KD_CENTER       = 0.2f;  

const float KGYRO = 0.30f; 

const int WALL_APPEAR_DIST    = 150;
const int WALL_DISAPPEAR_DIST = 180;

const int   DIAG_DANGER_DIST     = 100;    
const int   DIAG_FLOOR_MM        = 25;   
const int   DIAG_CLEAR_DIST      = 110;   
const int   DIAG_TRIP_DEBOUNCE   = 2;    
const float DIAG_CORRECT_KP      = 3.0f;  
const float DIAG_CORRECT_RPM_MIN = 25.0f; 
const float DIAG_CORRECT_RPM_MAX = 90.0f; 

const float RAMP_STAGE_DIST_MM = 60.0f;                      
const float RAMP_STAGE_RPM[3]  = {60.0f, 80.0f, 100.0f};     
const int   RAMP_STAGE_COUNT   = 3;                          

const float KP_MOTOR = 0.35;
const float KI_MOTOR = 0.80;
const float FF_SLOPE_L = 168.0 / 324.0;
const float FF_SLOPE_R = 149.0 / 302.0;

const int PWM_OFFSET_L = 85;
const int PWM_OFFSET_R = 90;
const float MAX_INTEGRAL_PWM = 40.0;

const int OFFSET_LEFT   = 12;
const int OFFSET_CENTER = 15;
const int OFFSET_RIGHT  = 11;
const int OFFSET_DIAG_L = 16;  
const int OFFSET_DIAG_R = 16;  

const float WHEEL_DIAMETER_MM = 40.0f;
const float MM_PER_COUNT = (PI * WHEEL_DIAMETER_MM) / 4096.0f;

const float GYRO_SENS_LSB_PER_DPS  = 131.0f;  
const float TURN_TARGET_DEG        = 82.0f;
const float HEADING_TOLERANCE_DEG  = 2.0f;    
const float GYRO_RATE_THRESHOLD_DPS= 8.0f;    
const float KP_TURN                = 1.2f;    
const float MIN_PIVOT_RPM          = 45.0f;   
const float MAX_PIVOT_RPM          = 90.0f;
const unsigned long TURN_TIMEOUT_MS = 3000;   

// ============================================================================
// GLOBAL STATE VARIABLES
// ============================================================================
VL53L0X tofLeft, tofCenter, tofRight;
VL53L0X tofDiagL, tofDiagR; 

unsigned long prevMotorTime = 0;
uint8_t tofSequenceState    = 0; 

int16_t prevTickL = -1, prevTickR = -1;
float actualRpmL = 0.0, actualRpmR = 0.0;

float integralL = 0.0, integralR = 0.0;
float smoothSteerRPM = 0.0; 
float smoothForwardRpm = BASE_RPM; 

float targetRpmL = BASE_RPM;
float targetRpmR = BASE_RPM;

int dLeft = 8190, dCenter = 8190, dRight = 8190;
int dDiagL = 8190, dDiagR = 8190; 

bool leftWallPresent  = false;
bool rightWallPresent = false;
bool diagLDanger = false; 
bool diagRDanger = false; 
int diagLTripCount = 0, diagRTripCount = 0; 
unsigned long lastSteerMs = 0;
float prevSteerError = 0.0f;

float distanceSinceTurnMm  = 0.0f;
bool  postTurnRampActive   = false;

float gyroZBiasRaw = 0.0f; 

// ============================================================================
// I2C & HARDWARE UTILITIES
// ============================================================================
void selectMux(uint8_t channel) {
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

int16_t readEncoder(uint8_t channel) {
  selectMux(channel);
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(0x0C);
  if (Wire.endTransmission(false) != 0) return -1;
  Wire.requestFrom((int)AS5600_ADDR, 2);
  if (Wire.available() < 2) return -1;
  return ((Wire.read() << 8) | Wire.read()) & 0x0FFF;
}

bool tofDataReady(VL53L0X &sensor) {
  return (sensor.readReg(0x13) & 0x07) != 0;
}

// ============================================================================
// GYRO (MPU6050 on Wire1)
// ============================================================================
void mpuWriteReg(uint8_t reg, uint8_t val) {
  Wire1.beginTransmission(MPU_ADDR);
  Wire1.write(reg);
  Wire1.write(val);
  Wire1.endTransmission();
}

int16_t mpuReadGyroZRaw() {
  Wire1.beginTransmission(MPU_ADDR);
  Wire1.write(0x47); 
  if (Wire1.endTransmission(false) != 0) return 0;
  Wire1.requestFrom((int)MPU_ADDR, 2);
  if (Wire1.available() < 2) return 0;
  uint8_t hi = Wire1.read();
  uint8_t lo = Wire1.read();
  return (int16_t)((hi << 8) | lo);
}

void initGyro() {
  Wire1.begin(PIN_SDA_GYRO, PIN_SCL_GYRO);
  Wire1.setClock(400000);

  mpuWriteReg(0x6B, 0x00); 
  delay(50);
  mpuWriteReg(0x1B, 0x00); 

  const int N = 400;
  long sum = 0;
  for (int i = 0; i < N; i++) {
    sum += mpuReadGyroZRaw();
    delay(2);
  }
  gyroZBiasRaw = sum / (float)N;
}

float readGyroZDps() {
  int16_t raw = mpuReadGyroZRaw();
  return (raw - gyroZBiasRaw) / GYRO_SENS_LSB_PER_DPS;
}

// ============================================================================
// LAYER 1: MOTOR SPEED CONTROLLER
// ============================================================================
void initMotors() {
  pinMode(PIN_STBY, OUTPUT);
  pinMode(PIN_AIN1, OUTPUT); pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT); pinMode(PIN_BIN2, OUTPUT);
  digitalWrite(PIN_STBY, HIGH); 
  
  ledcAttach(PIN_PWMA, 20000, 8);
  ledcAttach(PIN_PWMB, 20000, 8);
}

void applyMotorPWM(int leftPWM, int rightPWM) {
  leftPWM = constrain(leftPWM, -255, 255);
  rightPWM = constrain(rightPWM, -255, 255);

  if (leftPWM >= 0) { digitalWrite(PIN_AIN1, LOW); digitalWrite(PIN_AIN2, HIGH); } 
  else              { digitalWrite(PIN_AIN1, HIGH); digitalWrite(PIN_AIN2, LOW); }
  ledcWrite(PIN_PWMA, abs(leftPWM));

  if (rightPWM >= 0) { digitalWrite(PIN_BIN1, LOW); digitalWrite(PIN_BIN2, HIGH); } 
  else               { digitalWrite(PIN_BIN1, HIGH); digitalWrite(PIN_BIN2, LOW); }
  ledcWrite(PIN_PWMB, abs(rightPWM));
}

void updateEncoders(float dt) {
  int16_t tickL = readEncoder(CH_ENC_LEFT);
  int16_t tickR = readEncoder(CH_ENC_RIGHT);

  float distL = 0.0f, distR = 0.0f;
  bool gotL = false, gotR = false;
  
  if (tickL >= 0 && prevTickL >= 0) {
    int16_t delta = tickL - prevTickL;
    if (delta > 2048) delta -= 4096;
    if (delta < -2048) delta += 4096;
    
    float rawRpmL = (1.0 * delta / 4096.0) * (60.0 / dt);
    actualRpmL = (0.3 * rawRpmL) + (0.7 * actualRpmL); 

    distL = (1.0f * delta) * MM_PER_COUNT; 
    gotL = true;
  }
  
  if (tickR >= 0 && prevTickR >= 0) {
    int16_t delta = tickR - prevTickR;
    if (delta > 2048) delta -= 4096;
    if (delta < -2048) delta += 4096;
    
    float rawRpmR = (-1.0 * delta / 4096.0) * (60.0 / dt);
    actualRpmR = (0.3 * rawRpmR) + (0.7 * actualRpmR); 

    distR = (-1.0f * delta) * MM_PER_COUNT;
    gotR = true;
  }
  
  if (tickL >= 0) prevTickL = tickL;
  if (tickR >= 0) prevTickR = tickR;

  if (postTurnRampActive && gotL && gotR) {
    distanceSinceTurnMm += (distL + distR) / 2.0f;
  }
}

void driveWheelsRPM(float targetL, float targetR, float dt) {
  targetL = constrain(targetL, -220.0, 220.0);
  targetR = constrain(targetR, -220.0, 220.0);

  float errorL = targetL - actualRpmL;
  integralL += errorL * dt;
  integralL = constrain(integralL, -MAX_INTEGRAL_PWM / KI_MOTOR, MAX_INTEGRAL_PWM / KI_MOTOR);

  float ffL = 0;
  if (targetL > 1)       ffL = PWM_OFFSET_L + targetL * FF_SLOPE_L;
  else if (targetL < -1) ffL = -(PWM_OFFSET_L + (-targetL) * FF_SLOPE_L);

  float pwmL = ffL + (KP_MOTOR * errorL) + (KI_MOTOR * integralL);
  pwmL = constrain(pwmL, -255, 255);

  float errorR = targetR - actualRpmR;
  integralR += errorR * dt;
  integralR = constrain(integralR, -MAX_INTEGRAL_PWM / KI_MOTOR, MAX_INTEGRAL_PWM / KI_MOTOR);

  float ffR = 0;
  if (targetR > 1)       ffR = PWM_OFFSET_R + targetR * FF_SLOPE_R;
  else if (targetR < -1) ffR = -(PWM_OFFSET_R + (-targetR) * FF_SLOPE_R);

  float pwmR = ffR + (KP_MOTOR * errorR) + (KI_MOTOR * integralR);
  pwmR = constrain(pwmR, -255, 255);

  applyMotorPWM((int)pwmL, (int)pwmR);
}

// ============================================================================
// LAYER 2 & 3: NAVIGATION & CORNERING
// ============================================================================

float computeRampedForwardRpm() {
  if (!postTurnRampActive) return BASE_RPM;

  int stage = (int)(distanceSinceTurnMm / RAMP_STAGE_DIST_MM);
  if (stage >= RAMP_STAGE_COUNT) {
    postTurnRampActive = false; 
    return BASE_RPM;
  }
  return RAMP_STAGE_RPM[stage];
}

float computeForwardRpm() {
  float target;
  if (dCenter <= 0 || dCenter >= DECEL_ZONE_START_MM) {
    target = BASE_RPM;
  } else if (dCenter <= FRONT_TURN_DIST) {
    target = APPROACH_RPM;
  } else {
    float t = (float)(dCenter - FRONT_TURN_DIST) / (float)(DECEL_ZONE_START_MM - FRONT_TURN_DIST);
    target = APPROACH_RPM + t * (BASE_RPM - APPROACH_RPM);
  }

  float rampTarget = computeRampedForwardRpm();
  if (rampTarget < target) target = rampTarget;

  smoothForwardRpm = 0.3f * target + 0.7f * smoothForwardRpm;
  return smoothForwardRpm;
}

void computeSteering() {
  if (dLeft > 0) {
    if (!leftWallPresent && dLeft < WALL_APPEAR_DIST)       leftWallPresent = true;
    else if (leftWallPresent && dLeft > WALL_DISAPPEAR_DIST) leftWallPresent = false;
  } else {
    leftWallPresent = false;
  }

  if (dRight > 0) {
    if (!rightWallPresent && dRight < WALL_APPEAR_DIST)        rightWallPresent = true;
    else if (rightWallPresent && dRight > WALL_DISAPPEAR_DIST) rightWallPresent = false;
  } else {
    rightWallPresent = false;
  }

  float effLeft  = leftWallPresent  ? (float)dLeft  : SAFE_DIST_MID;
  float effRight = rightWallPresent ? (float)dRight : SAFE_DIST_MID;
  float error = effRight - effLeft;

  unsigned long nowMs = millis();
  float dt = (lastSteerMs == 0) ? 0.02f : (nowMs - lastSteerMs) / 1000.0f;
  if (dt <= 0.0f) dt = 0.001f;
  lastSteerMs = nowMs;

  float derivative = (error - prevSteerError) / dt;
  prevSteerError = error;

  float yawRateDps = readGyroZDps();

  float rawSteerRPM = KP_CENTER * error + KD_CENTER * derivative - KGYRO * yawRateDps;
  rawSteerRPM = constrain(rawSteerRPM, -MAX_STEER_RPM, MAX_STEER_RPM);

  smoothSteerRPM = (0.3 * rawSteerRPM) + (0.7 * smoothSteerRPM);

  float baseRpm = computeForwardRpm();
  targetRpmL = baseRpm + smoothSteerRPM;
  targetRpmR = baseRpm - smoothSteerRPM;

  if (dDiagL > 0 && dDiagL < DIAG_DANGER_DIST) {
    diagLTripCount++;
    if (!diagLDanger && diagLTripCount >= DIAG_TRIP_DEBOUNCE) diagLDanger = true;
  } else {
    diagLTripCount = 0;
    if (diagLDanger && (dDiagL <= 0 || dDiagL > DIAG_CLEAR_DIST)) diagLDanger = false;
  }

  if (dDiagR > 0 && dDiagR < DIAG_DANGER_DIST) {
    diagRTripCount++;
    if (!diagRDanger && diagRTripCount >= DIAG_TRIP_DEBOUNCE) diagRDanger = true;
  } else {
    diagRTripCount = 0;
    if (diagRDanger && (dDiagR <= 0 || dDiagR > DIAG_CLEAR_DIST)) diagRDanger = false;
  }

  if (diagLDanger) {
    float correctRpm;
    if (dDiagL <= DIAG_FLOOR_MM) {
      correctRpm = DIAG_CORRECT_RPM_MAX; 
    } else {
      float penetration = (float)DIAG_DANGER_DIST - (float)dDiagL;
      float linear_rpm = DIAG_CORRECT_RPM_MIN + DIAG_CORRECT_KP * penetration;
      
      float safe_dist = max((float)dDiagL, 1.0f); 
      float proximity_ratio = (float)DIAG_DANGER_DIST / safe_dist;
      float multiplier = proximity_ratio * proximity_ratio;
      
      correctRpm = linear_rpm * multiplier;
      correctRpm = constrain(correctRpm, DIAG_CORRECT_RPM_MIN, DIAG_CORRECT_RPM_MAX);
    }
    targetRpmL = baseRpm + correctRpm; 
    targetRpmR = baseRpm - correctRpm; 
    
  } else if (diagRDanger) {
    float correctRpm;
    if (dDiagR <= DIAG_FLOOR_MM) {
      correctRpm = DIAG_CORRECT_RPM_MAX;
    } else {
      float penetration = (float)DIAG_DANGER_DIST - (float)dDiagR;
      float linear_rpm = DIAG_CORRECT_RPM_MIN + DIAG_CORRECT_KP * penetration;
      
      float safe_dist = max((float)dDiagR, 1.0f);
      float proximity_ratio = (float)DIAG_DANGER_DIST / safe_dist;
      float multiplier = proximity_ratio * proximity_ratio;
      
      correctRpm = linear_rpm * multiplier;
      correctRpm = constrain(correctRpm, DIAG_CORRECT_RPM_MIN, DIAG_CORRECT_RPM_MAX);
    }
    targetRpmL = baseRpm - correctRpm; 
    targetRpmR = baseRpm + correctRpm; 
  }
}

void executePivotTurn() {
  digitalWrite(PIN_AIN1, HIGH); digitalWrite(PIN_AIN2, HIGH);
  digitalWrite(PIN_BIN1, HIGH); digitalWrite(PIN_BIN2, HIGH);
  ledcWrite(PIN_PWMA, 255); ledcWrite(PIN_PWMB, 255);
  delay(60);
  applyMotorPWM(0, 0);
  delay(100); 

  integralL = 0;
  integralR = 0;
  smoothSteerRPM = 0;

  bool turnLeft = (dLeft > dRight);
  float currentHeadingDeg = 0.0f;
  float targetHeadingDeg = 999.0f; 
  bool centerCleared = false;

  float scanRpm = 45.0f; 

  unsigned long turnStartMs  = millis();
  unsigned long lastMotorMs  = millis();
  unsigned long lastGyroUs   = micros();

  while (true) {
    unsigned long nowMs = millis();
    unsigned long nowUs = micros();

    float gz = readGyroZDps();
    float gdt = (nowUs - lastGyroUs) / 1e6f;
    if (gdt > 0.0f) {
      currentHeadingDeg += gz * gdt;
      lastGyroUs = nowUs;
    }

    selectMux(CH_TOF_CENTER);
    if (tofDataReady(tofCenter)) {
      int raw = tofCenter.readRangeContinuousMillimeters();
      dCenter = (tofCenter.timeoutOccurred() || raw > 8000) ? 8190 : raw - OFFSET_CENTER;
    }

    float pivotRpm = 0.0f;

    if (!centerCleared) {
      if (dCenter > 160 && fabs(currentHeadingDeg) > 40.0f) { 
        centerCleared = true;
        
        targetHeadingDeg = currentHeadingDeg + (turnLeft ? 32.0f : -32.0f);
        
        Serial.printf("Opening found at %.1f deg. Squaring up to %.1f deg.\n", currentHeadingDeg, targetHeadingDeg);
      }
      
      pivotRpm = turnLeft ? scanRpm : -scanRpm; 
      
    } else {
      float headingError = targetHeadingDeg - currentHeadingDeg;
      pivotRpm = KP_TURN * headingError;
      pivotRpm = constrain(pivotRpm, -MAX_PIVOT_RPM, MAX_PIVOT_RPM);
      
      if (fabs(headingError) > HEADING_TOLERANCE_DEG && fabs(pivotRpm) < MIN_PIVOT_RPM) {
        pivotRpm = (pivotRpm >= 0) ? MIN_PIVOT_RPM : -MIN_PIVOT_RPM;
      }

      if (fabs(headingError) < HEADING_TOLERANCE_DEG && fabs(gz) < GYRO_RATE_THRESHOLD_DPS) {
        break; 
      }
    }

    if (nowMs - lastMotorMs >= 10) {
      float dt = (nowMs - lastMotorMs) / 1000.0f;
      lastMotorMs = nowMs;
      updateEncoders(dt);
      
      if (!centerCleared) {
          float leftSpeed = turnLeft ? -scanRpm : scanRpm;
          float rightSpeed = turnLeft ? scanRpm : -scanRpm;
          driveWheelsRPM(leftSpeed, rightSpeed, dt);
      } else {
          driveWheelsRPM(-pivotRpm, pivotRpm, dt);
      }
    }

    if (nowMs - turnStartMs > TURN_TIMEOUT_MS) break;
  }

  digitalWrite(PIN_AIN1, HIGH); digitalWrite(PIN_AIN2, HIGH);
  digitalWrite(PIN_BIN1, HIGH); digitalWrite(PIN_BIN2, HIGH);
  ledcWrite(PIN_PWMA, 255); ledcWrite(PIN_PWMB, 255);
  delay(60);
  applyMotorPWM(0, 0);
  delay(100);

  integralL = 0;
  integralR = 0;

  selectMux(CH_TOF_LEFT);
  int rL = tofLeft.readRangeContinuousMillimeters();
  dLeft = (tofLeft.timeoutOccurred() || rL > 8000) ? 8190 : rL - OFFSET_LEFT;

  selectMux(CH_TOF_CENTER);
  int rC = tofCenter.readRangeContinuousMillimeters();
  dCenter = (tofCenter.timeoutOccurred() || rC > 8000) ? 8190 : rC - OFFSET_CENTER;

  selectMux(CH_TOF_RIGHT);
  int rR = tofRight.readRangeContinuousMillimeters();
  dRight = (tofRight.timeoutOccurred() || rR > 8000) ? 8190 : rR - OFFSET_RIGHT;

  selectMux(CH_TOF_DIAG_L); 
  int rDL = tofDiagL.readRangeContinuousMillimeters();
  dDiagL = (tofDiagL.timeoutOccurred() || rDL > 8000) ? 8190 : rDL - OFFSET_DIAG_L;

  selectMux(CH_TOF_DIAG_R); 
  int rDR = tofDiagR.readRangeContinuousMillimeters();
  dDiagR = (tofDiagR.timeoutOccurred() || rDR > 8000) ? 8190 : rDR - OFFSET_DIAG_R;

  leftWallPresent  = (dLeft  > 0 && dLeft  < WALL_APPEAR_DIST);
  rightWallPresent = (dRight > 0 && dRight < WALL_APPEAR_DIST);
  diagLDanger = (dDiagL > 0 && dDiagL < DIAG_DANGER_DIST); 
  diagRDanger = (dDiagR > 0 && dDiagR < DIAG_DANGER_DIST); 
  diagLTripCount = 0; 
  diagRTripCount = 0; 
  prevSteerError = 0.0f;
  lastSteerMs = 0; 

  distanceSinceTurnMm = 0.0f;
  postTurnRampActive  = true; 
}

// ============================================================================
// SYSTEM BOOTSTRAP
// ============================================================================
void setup() {
  Serial.begin(115200);
  Wire.begin(PIN_SDA_MAIN, PIN_SCL_MAIN);
  Wire.setClock(400000); 

  initMotors();
  applyMotorPWM(0, 0);

  bool leftOk, centerOk, rightOk, diagLOk, diagROk;

  selectMux(CH_TOF_LEFT);
  leftOk = tofLeft.init();
  tofLeft.setTimeout(50);
  tofLeft.setMeasurementTimingBudget(20000);
  tofLeft.startContinuous(); 

  selectMux(CH_TOF_CENTER);
  centerOk = tofCenter.init();
  tofCenter.setTimeout(50);
  tofCenter.setMeasurementTimingBudget(20000);
  tofCenter.startContinuous();

  selectMux(CH_TOF_RIGHT);
  rightOk = tofRight.init();
  tofRight.setTimeout(50);
  tofRight.setMeasurementTimingBudget(20000);
  tofRight.startContinuous();

  selectMux(CH_TOF_DIAG_L);
  diagLOk = tofDiagL.init();
  tofDiagL.setTimeout(50);
  tofDiagL.setMeasurementTimingBudget(20000);
  tofDiagL.startContinuous();

  selectMux(CH_TOF_DIAG_R);
  diagROk = tofDiagR.init();
  tofDiagR.setTimeout(50);
  tofDiagR.setMeasurementTimingBudget(20000);
  tofDiagR.startContinuous();

  Serial.printf("ToF init -> Left:%s  Center:%s  Right:%s  DiagL:%s  DiagR:%s\n",
                leftOk ? "OK" : "FAILED",
                centerOk ? "OK" : "FAILED",
                rightOk ? "OK" : "FAILED",
                diagLOk ? "OK" : "FAILED",
                diagROk ? "OK" : "FAILED");
  if (!centerOk) {
    Serial.println("WARNING: front/center ToF failed to init - turns will never trigger.");
  }
  if (!diagLOk || !diagROk) {
    Serial.println("WARNING: a diagonal ToF failed to init - corner-clip override on that side is disabled.");
  }

  initGyro();

  delay(2000); 
  prevMotorTime = millis();
}

// ============================================================================
// MAIN LOOP: SCHEDULER
// ============================================================================
void loop() {
  unsigned long now = millis();

  if (now - prevMotorTime >= 10) {
    float dt = (now - prevMotorTime) / 1000.0;
    prevMotorTime = now;

    updateEncoders(dt);
    driveWheelsRPM(targetRpmL, targetRpmR, dt);
  }

  switch (tofSequenceState) {
    case 0: 
      selectMux(CH_TOF_LEFT);
      if (tofDataReady(tofLeft)) {
        int raw = tofLeft.readRangeContinuousMillimeters();
        dLeft = (tofLeft.timeoutOccurred() || raw > 8000) ? 8190 : raw - OFFSET_LEFT;
      }
      tofSequenceState = 1;
      break;
      
    case 1: 
      selectMux(CH_TOF_CENTER);
      if (tofDataReady(tofCenter)) {
        int raw = tofCenter.readRangeContinuousMillimeters();
        dCenter = (tofCenter.timeoutOccurred() || raw > 8000) ? 8190 : raw - OFFSET_CENTER;
      }
      tofSequenceState = 2;
      break;
      
    case 2: 
      selectMux(CH_TOF_RIGHT);
      if (tofDataReady(tofRight)) {
        int raw = tofRight.readRangeContinuousMillimeters();
        dRight = (tofRight.timeoutOccurred() || raw > 8000) ? 8190 : raw - OFFSET_RIGHT;
      }
      tofSequenceState = 3;
      break;

    case 3: 
      selectMux(CH_TOF_DIAG_L);
      if (tofDataReady(tofDiagL)) {
        int raw = tofDiagL.readRangeContinuousMillimeters();
        dDiagL = (tofDiagL.timeoutOccurred() || raw > 8000) ? 8190 : raw - OFFSET_DIAG_L;
      }
      tofSequenceState = 4;
      break;

    case 4: 
      selectMux(CH_TOF_DIAG_R);
      if (tofDataReady(tofDiagR)) {
        int raw = tofDiagR.readRangeContinuousMillimeters();
        dDiagR = (tofDiagR.timeoutOccurred() || raw > 8000) ? 8190 : raw - OFFSET_DIAG_R;
      }
      tofSequenceState = 0;
      break;
  }

  static unsigned long lastSteerUpdateMs = 0;
  const unsigned long STEER_UPDATE_PERIOD_MS = 20;
  if (now - lastSteerUpdateMs >= STEER_UPDATE_PERIOD_MS) {
    lastSteerUpdateMs = now;

    if (dCenter > 0 && dCenter < FRONT_TURN_DIST) {
      Serial.printf(">> TURN TRIGGERED  L:%d  C:%d  R:%d\n", dLeft, dCenter, dRight);
      executePivotTurn();
    } else {
      computeSteering();
    }
  }

  static unsigned long lastDebugMs = 0;
  if (now - lastDebugMs >= 200) {
    lastDebugMs = now;
    Serial.printf("L:%d  DL:%d  C:%d  DR:%d  R:%d  steer:%.1f  fwd:%.1f\n",
                  dLeft, dDiagL, dCenter, dDiagR, dRight, smoothSteerRPM, smoothForwardRpm);
  }
}
    Serial.printf("L:%d  DL:%d  C:%d  DR:%d  R:%d  steer:%.1f  fwd:%.1f\n",
                  dLeft, dDiagL, dCenter, dDiagR, dRight, smoothSteerRPM, smoothForwardRpm);
  }
}
