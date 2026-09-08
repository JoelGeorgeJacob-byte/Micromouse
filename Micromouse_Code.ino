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

// Gyro lives on its OWN I2C bus (Wire1), exactly as in the original hardware
// spec. It is never wired into the mux and never shares a bus with the ToF
// sensors or encoders, so it can't create bus contention with them either.
#define PIN_SDA_GYRO 16
#define PIN_SCL_GYRO 17
#define MPU_ADDR     0x68

#define CH_TOF_LEFT   0
#define CH_TOF_DIAG_L 1   // NEW: front-left diagonal, ~45 deg off center
#define CH_TOF_CENTER 2
#define CH_TOF_DIAG_R 3   // NEW: front-right diagonal, ~45 deg off center
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

// ----------------------------------------------------------------------------
// Pre-turn deceleration zone.
// Instead of cruising at full BASE_RPM right up to FRONT_TURN_DIST and then
// relying on a hard stop to kill all that momentum in one go, forward speed
// is ramped down proportionally as dCenter shrinks through this zone, so the
// robot is already slow by the time it reaches the hard trigger. This is
// what actually prevents wall clips at high BASE_RPM - the brake/stop still
// matters, but it now only has to kill a much smaller amount of momentum.
// ----------------------------------------------------------------------------
const float DECEL_ZONE_START_MM = 300.0f; // begin slowing down once center distance drops below this
const float APPROACH_RPM        = 55.0f;  // target speed to have reached by the time FRONT_TURN_DIST is hit

// ----------------------------------------------------------------------------
// Steering tuning (computeSteering)
// ----------------------------------------------------------------------------
// When BOTH walls are visible: error = dRight - dLeft, PD centering.
// When only ONE wall is visible: error = (that wall's distance) vs. this
// fixed target, using the exact same PD math — see computeSteering() for why
// this unifies cleanly into one formula instead of two separate branches.
const float SAFE_DIST_MID   = (SAFE_DIST_MIN + SAFE_DIST_MAX) / 2.0f; // 62.5mm target stand-off
const float KP_CENTER       = 0.5f;   // RPM per mm of centering error
const float KD_CENTER       = 0.2f;  // RPM per (mm/s) of error rate-of-change — damps sway

// ----------------------------------------------------------------------------
// Yaw-rate damping (NOT heading control — no angle is tracked or targeted
// here, only instantaneous rotation rate). This is a pure stabilizing term
// added on top of the existing PD: if the chassis is actually rotating,
// oppose that rotation directly, using the gyro's much cleaner rate signal
// instead of relying only on the derivative of a noisier ToF-distance error.
// It's added to the same single steer output — this is not a second
// controller competing with computeSteering(), it's one more term inside it.
// ----------------------------------------------------------------------------
const float KGYRO = 0.30f; // RPM per (deg/s) of yaw rate — tune on the bench.
// If enabling this makes oscillation WORSE rather than better, your gyro's Z
// axis is mounted with the opposite polarity relative to this file's existing
// +90°=left turn convention — flip the sign used in computeSteering() (i.e.
// add KGYRO*yawRate instead of subtracting) rather than changing anything else.

// Wall-presence hysteresis: a wall is considered PRESENT once distance drops
// below WALL_APPEAR_DIST, and ABSENT only once it rises above
// WALL_DISAPPEAR_DIST. This gap prevents the steering mode from flapping
// between "both walls" and "single wall" when a reading sits right at the
// boundary.
const int WALL_APPEAR_DIST    = 150;
const int WALL_DISAPPEAR_DIST = 180;

// ----------------------------------------------------------------------------
// NEW: Diagonal corner-clip override.
//
// L/C/R only look straight ahead and straight sideways. On a zigzag, the
// wall that actually clips the robot is the corridor corner coming in at
// ~45 deg to the chassis, right at the robot's front-left/front-right
// corner — a spot none of the three existing sensors ever point at. These
// two diagonal sensors watch exactly that spot.
//
// This is deliberately NOT blended into the PD sum like KGYRO is. Near a
// corner, the PD centering term's model of "where the walls are" is
// momentarily wrong, so adding a diagonal term into the same sum just makes
// it fight a misinformed centering term. Instead this is a hard override:
// below DIAG_DANGER_DIST it fully replaces the commanded differential for
// that loop iteration. Same hysteresis pattern as WALL_APPEAR/DISAPPEAR so
// it can't chatter at the threshold.
// ----------------------------------------------------------------------------
const int   DIAG_DANGER_DIST     = 100;    // mm — trigger the override below this (0 = disabled, sensors only reported)
// Sensor floor: VL53L0X clamps and returns ~42mm raw for anything closer,
// so distance is unreadable below it. Expressed here in distance-FROM-EDGE
// terms (after the OFFSET_DIAG subtraction) to match every other threshold
// in this file: with the sensor recessed 16mm, raw-floor 42mm corresponds
// to 42 - 16 = 26mm from the actual bot edge.
const int   DIAG_FLOOR_MM        = 25;   // treat readings at/near this as "unreadable, assume worst case"
const int   DIAG_CLEAR_DIST      = 110;   // mm — must rise above this to release
const int   DIAG_TRIP_DEBOUNCE   = 2;    // consecutive dangerous readings required before reacting at all — rejects a single noisy ping
// PROPORTIONAL ZONE (DIAG_DANGER_DIST down to DIAG_FLOOR_MM) — real distance
// data exists here, so grade the correction with it.
const float DIAG_CORRECT_KP      = 3.0f;  // RPM per mm of penetration past DIAG_DANGER_DIST
const float DIAG_CORRECT_RPM_MIN = 25.0f; // floor — weakest correction applied while tripped at all
const float DIAG_CORRECT_RPM_MAX = 90.0f; // ceiling — also the flat value used once floored (see below)

// ----------------------------------------------------------------------------
// Post-turn speed ramp. Rather than snapping back to full BASE_RPM the
// instant a turn ends, hold a stepped sequence of slower speeds for a fixed
// DISTANCE (not time) after the turn, using encoder-measured travel. Distance
// based means this behaves consistently regardless of how fast the ramp
// itself executes.
// ----------------------------------------------------------------------------
const float RAMP_STAGE_DIST_MM = 60.0f;                      // distance covered per stage
const float RAMP_STAGE_RPM[3]  = {60.0f, 80.0f, 100.0f};     // stepped speeds; BASE_RPM follows automatically after the last stage
const int   RAMP_STAGE_COUNT   = 3;                          // 3 stages * 60mm = 180mm total ramp distance (within the requested 150-200mm)

// Motor PI Gains
const float KP_MOTOR = 0.35;
const float KI_MOTOR = 0.80;
const float FF_SLOPE_L = 168.0 / 324.0;
const float FF_SLOPE_R = 149.0 / 302.0;

// Minimum PWM required to overcome motor stiction
const int PWM_OFFSET_L = 85;
const int PWM_OFFSET_R = 90;
const float MAX_INTEGRAL_PWM = 40.0;

const int OFFSET_LEFT   = 12;
const int OFFSET_CENTER = 15;
const int OFFSET_RIGHT  = 11;
const int OFFSET_DIAG_L = 16;  // NEW — sensor recessed 16mm inward from the edge; raw reading includes that extra path length
const int OFFSET_DIAG_R = 16;  // NEW — same recess on the right

// Wheel geometry, used only to convert encoder ticks to millimeters for the
// post-turn ramp's distance tracking.
const float WHEEL_DIAMETER_MM = 40.0f;
const float MM_PER_COUNT = (PI * WHEEL_DIAMETER_MM) / 4096.0f;

// ----------------------------------------------------------------------------
// Gyro-based turn tuning (ONLY used inside executePivotTurn())
// ----------------------------------------------------------------------------
const float GYRO_SENS_LSB_PER_DPS  = 131.0f;  // MPU6050 @ +/-250 dps range
const float TURN_TARGET_DEG        = 82.0f;
const float HEADING_TOLERANCE_DEG  = 2.0f;    // "close enough" heading error
const float GYRO_RATE_THRESHOLD_DPS= 8.0f;    // must also be nearly stopped rotating
const float KP_TURN                = 1.2f;    // pivot RPM per degree of heading error
const float MIN_PIVOT_RPM          = 45.0f;   // floor so the turn doesn't stall out early
const float MAX_PIVOT_RPM          = 90.0f;
const unsigned long TURN_TIMEOUT_MS = 3000;   // safety net if gyro ever misbehaves

// ============================================================================
// GLOBAL STATE VARIABLES
// ============================================================================
VL53L0X tofLeft, tofCenter, tofRight;
VL53L0X tofDiagL, tofDiagR; // NEW

unsigned long prevMotorTime = 0;
uint8_t tofSequenceState    = 0; 

int16_t prevTickL = -1, prevTickR = -1;
float actualRpmL = 0.0, actualRpmR = 0.0;

float integralL = 0.0, integralR = 0.0;
float smoothSteerRPM = 0.0; 
float smoothForwardRpm = BASE_RPM; // tracks current commanded cruise speed with deceleration zone applied

float targetRpmL = BASE_RPM;
float targetRpmR = BASE_RPM;

int dLeft = 8190, dCenter = 8190, dRight = 8190;
int dDiagL = 8190, dDiagR = 8190; // NEW

bool leftWallPresent  = false;
bool rightWallPresent = false;
bool diagLDanger = false; // NEW
bool diagRDanger = false; // NEW
int diagLTripCount = 0, diagRTripCount = 0; // NEW: debounce counters, consecutive dangerous reads
unsigned long lastSteerMs = 0;
float prevSteerError = 0.0f;

float distanceSinceTurnMm  = 0.0f;
bool  postTurnRampActive   = false;

float gyroZBiasRaw = 0.0f; // calibrated once at boot, while stationary

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

// ----------------------------------------------------------------------------
// Non-blocking "is a fresh continuous-mode measurement ready?" check.
// RESULT_INTERRUPT_STATUS (0x13), low 3 bits nonzero = new sample available.
// This is what lets the round-robin cycle through all sensors in well
// under a millisecond each when nothing new has arrived yet, instead of
// blocking a full conversion time on every single call like single-shot mode
// does.
// ----------------------------------------------------------------------------
bool tofDataReady(VL53L0X &sensor) {
  return (sensor.readReg(0x13) & 0x07) != 0;
}

// ============================================================================
// GYRO (MPU6050 on Wire1)
//
// Two distinct uses, and it's important they stay distinct:
//  - executePivotTurn() integrates yaw rate into a HEADING and steers toward
//    a target angle — this is the only place anything is done with heading.
//  - computeSteering() reads raw yaw RATE only (no integration, no target,
//    no drift-prone accumulation) purely to damp rotational oscillation on
//    top of the existing ToF-based PD. It never decides direction on its
//    own, it only opposes whatever rotation is actually happening.
// Only one of these runs at a time, and only one of them ever commands a
// heading — the rate-damping term never fights the turn maneuver's heading
// control, because it isn't heading control.
// ============================================================================
void mpuWriteReg(uint8_t reg, uint8_t val) {
  Wire1.beginTransmission(MPU_ADDR);
  Wire1.write(reg);
  Wire1.write(val);
  Wire1.endTransmission();
}

int16_t mpuReadGyroZRaw() {
  Wire1.beginTransmission(MPU_ADDR);
  Wire1.write(0x47); // GYRO_ZOUT_H
  if (Wire1.endTransmission(false) != 0) return 0;
  Wire1.requestFrom((int)MPU_ADDR, 2);
  if (Wire1.available() < 2) return 0;
  uint8_t hi = Wire1.read();
  uint8_t lo = Wire1.read();
  return (int16_t)((hi << 8) | lo);
}

// Blocking, boot-time only. The robot MUST be perfectly still while this runs.
void initGyro() {
  Wire1.begin(PIN_SDA_GYRO, PIN_SCL_GYRO);
  Wire1.setClock(400000);

  mpuWriteReg(0x6B, 0x00); // PWR_MGMT_1: wake from sleep
  delay(50);
  mpuWriteReg(0x1B, 0x00); // GYRO_CONFIG: +/-250 dps, highest sensitivity

  const int N = 400;
  long sum = 0;
  for (int i = 0; i < N; i++) {
    sum += mpuReadGyroZRaw();
    delay(2);
  }
  gyroZBiasRaw = sum / (float)N;
}

// Degrees/sec, bias-corrected. Called from computeSteering() (rate damping
// only) and from inside executePivotTurn() (integrated into heading there).
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

    distL = (1.0f * delta) * MM_PER_COUNT; // same forward-positive sign convention as actualRpmL
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

// ----------------------------------------------------------------------------
// Post-turn speed ramp lookup. Distance-gated, not time-gated: looks at how
// far the wheels have actually traveled since the last turn ended, not how
// long it's been.
// ----------------------------------------------------------------------------
float computeRampedForwardRpm() {
  if (!postTurnRampActive) return BASE_RPM;

  int stage = (int)(distanceSinceTurnMm / RAMP_STAGE_DIST_MM);
  if (stage >= RAMP_STAGE_COUNT) {
    postTurnRampActive = false; // ramp distance covered - hand back to normal cruise/decel-zone logic
    return BASE_RPM;
  }
  return RAMP_STAGE_RPM[stage];
}

// ----------------------------------------------------------------------------
// Forward cruise speed, with a deceleration ramp as the front wall approaches.
//   dCenter >= DECEL_ZONE_START_MM      -> full BASE_RPM
//   dCenter <= FRONT_TURN_DIST          -> APPROACH_RPM (slow, controlled)
//   in between                          -> linear taper
// Also respects the post-turn ramp as a floor: whichever of "close to the
// next wall" or "just came out of a turn" wants a slower speed wins.
// EMA-smoothed the same way smoothSteerRPM is, so a single noisy ToF sample
// near the zone boundary can't cause a sudden speed jump.
// ----------------------------------------------------------------------------
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

// ----------------------------------------------------------------------------
// Steering. Unified centering formula:
//
//   effLeft  = dLeft  if left wall present,  else SAFE_DIST_MID
//   effRight = dRight if right wall present, else SAFE_DIST_MID
//   error    = effRight - effLeft
//
// Both walls visible -> error is exactly "Right distance - Left distance",
// i.e. true corridor centering (the spec's "Left Distance - Right Distance"
// error, sign-flipped to match this file's existing steer convention where
// positive steer = turn right).
//
// Only one wall visible -> the missing side's "distance" is substituted with
// the target stand-off, which makes the same formula collapse into
// "hold this distance from the one wall I can see" automatically, with no
// separate branch or gain set needed.
//
// Neither wall visible -> error is 0 -> drive straight (no correction to
// fight the motor PID with).
//
// NEW, LAST STAGE: diagonal corner-clip override. dDiagL/dDiagR watch the
// front-left/front-right corners of the chassis specifically — the spot a
// zigzag corner can clip that none of L/C/R ever point at. If either gets
// dangerously close, this fully overrides targetRpmL/R for this cycle
// instead of blending into the PD sum above, so it can't fight a PD term
// that's currently working off a stale picture of where the walls are.
// ----------------------------------------------------------------------------
void computeSteering() {
  // --- Wall presence with hysteresis (prevents flapping between modes) ---
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

  // --- Derivative on error, based on actual elapsed time ---
  unsigned long nowMs = millis();
  float dt = (lastSteerMs == 0) ? 0.02f : (nowMs - lastSteerMs) / 1000.0f;
  if (dt <= 0.0f) dt = 0.001f;
  lastSteerMs = nowMs;

  float derivative = (error - prevSteerError) / dt;
  prevSteerError = error;

  // Pure rate damping - no heading tracked, no angle targeted. Opposes
  // whatever actual rotation the gyro sees, on top of the ToF-based PD.
  float yawRateDps = readGyroZDps();

  float rawSteerRPM = KP_CENTER * error + KD_CENTER * derivative - KGYRO * yawRateDps;
  rawSteerRPM = constrain(rawSteerRPM, -MAX_STEER_RPM, MAX_STEER_RPM);

  smoothSteerRPM = (0.3 * rawSteerRPM) + (0.7 * smoothSteerRPM);

  float baseRpm = computeForwardRpm();
  targetRpmL = baseRpm + smoothSteerRPM;
  targetRpmR = baseRpm - smoothSteerRPM;

  // --- NEW: diagonal corner-clip override ---
  // Entry is debounced (DIAG_TRIP_DEBOUNCE consecutive dangerous reads)
  // so one noisy ping can't snap the steering. Release still uses the
  // simple hysteresis gap against DIAG_CLEAR_DIST — a fluke high reading
  // releasing early just means it has to re-confirm danger to trip again,
  // which is the safe direction to be wrong in.
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

  // Front-left corner is threatened -> steer away from it (right).
  // Front-right corner is threatened -> steer away from it (left).
  // This replaces the PD-derived targets outright for this cycle; it does
  // not add on top of them. If both somehow trip at once (very tight
  // pinch), left takes priority arbitrarily rather than the two cancelling
  // out to zero correction.
  //
  // TWO-TIER magnitude, because the sensor itself has two regimes:
  //  - Above DIAG_FLOOR_MM, real graded distance data exists -> proportional
  //    correction, gentle near the trigger and climbing as it closes in.
  //  - At/below DIAG_FLOOR_MM the sensor is saturated and CANNOT tell you
  //    35mm from 5mm — there is no distance information left to be
  //    proportional about. Trying to grade a floored reading just means
  //    "confirmed danger" gets treated as "barely tripped." Once debounced
  //    and confirmed floored, go straight to max authority instead.
  if (diagLDanger) {
    float correctRpm;
    if (dDiagL <= DIAG_FLOOR_MM) {
      correctRpm = DIAG_CORRECT_RPM_MAX; 
    } else {
      // NEW: Applied Inverse Square Multiplier to Left Diagonal Sensor
      float penetration = (float)DIAG_DANGER_DIST - (float)dDiagL;
      float linear_rpm = DIAG_CORRECT_RPM_MIN + DIAG_CORRECT_KP * penetration;
      
      float safe_dist = max((float)dDiagL, 1.0f); 
      float proximity_ratio = (float)DIAG_DANGER_DIST / safe_dist;
      float multiplier = proximity_ratio * proximity_ratio;
      
      correctRpm = linear_rpm * multiplier;
      correctRpm = constrain(correctRpm, DIAG_CORRECT_RPM_MIN, DIAG_CORRECT_RPM_MAX);
    }
    // FIX: Speed up left, slow down right -> Steers RIGHT (away from left peg)
    targetRpmL = baseRpm + correctRpm; 
    targetRpmR = baseRpm - correctRpm; 
    
  } else if (diagRDanger) {
    float correctRpm;
    if (dDiagR <= DIAG_FLOOR_MM) {
      correctRpm = DIAG_CORRECT_RPM_MAX;
    } else {
      // NEW: Applied Inverse Square Multiplier to Right Diagonal Sensor
      float penetration = (float)DIAG_DANGER_DIST - (float)dDiagR;
      float linear_rpm = DIAG_CORRECT_RPM_MIN + DIAG_CORRECT_KP * penetration;
      
      float safe_dist = max((float)dDiagR, 1.0f);
      float proximity_ratio = (float)DIAG_DANGER_DIST / safe_dist;
      float multiplier = proximity_ratio * proximity_ratio;
      
      correctRpm = linear_rpm * multiplier;
      correctRpm = constrain(correctRpm, DIAG_CORRECT_RPM_MIN, DIAG_CORRECT_RPM_MAX);
    }
    // FIX: Speed up right, slow down left -> Steers LEFT (away from right peg)
    targetRpmL = baseRpm - correctRpm; 
    targetRpmR = baseRpm + correctRpm; 
  }
}

// ----------------------------------------------------------------------------
// ADAPTIVE SENSOR-GUIDED TURN (Replaces Hardcoded Pivot Turn)
//
// This turn operates in two phases to handle unknown maze angles dynamically:
// PHASE 1: Scans blindly at a steady RPM while polling the center ToF sensor.
//          It uses an ANGLE GATE (>40 deg) so it doesn't prematurely trigger 
//          by glancing off the inner corner peg.
// PHASE 2: The exact moment dCenter sees open space (> 160mm, safe for zigzags), 
//          it locks in the current gyro angle, adds a 15-degree offset to square 
//          up with the walls, and uses proportional braking to finish the turn.
// ----------------------------------------------------------------------------
void executePivotTurn() {
  // INLINE ACTIVE BRAKE: Kill forward momentum instantly
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
  float targetHeadingDeg = 999.0f; // Will be set dynamically when the opening is found
  bool centerCleared = false;

  // Steady scan speed allows high-resolution ToF polling without overshooting
  float scanRpm = 45.0f; 

  unsigned long turnStartMs  = millis();
  unsigned long lastMotorMs  = millis();
  unsigned long lastGyroUs   = micros();

  while (true) {
    unsigned long nowMs = millis();
    unsigned long nowUs = micros();

    // --- Gyro integration (as fast as the loop runs) ---
    float gz = readGyroZDps();
    float gdt = (nowUs - lastGyroUs) / 1e6f;
    if (gdt > 0.0f) {
      currentHeadingDeg += gz * gdt;
      lastGyroUs = nowUs;
    }

    // --- Fast-Poll the Center ToF ---
    selectMux(CH_TOF_CENTER);
    if (tofDataReady(tofCenter)) {
      int raw = tofCenter.readRangeContinuousMillimeters();
      dCenter = (tofCenter.timeoutOccurred() || raw > 8000) ? 8190 : raw - OFFSET_CENTER;
    }

    float pivotRpm = 0.0f;

    if (!centerCleared) {
      // PHASE 1: Rotate blindly until the center sensor sees down the new corridor
      
      // ZIGZAG FIX: The next wall in a zigzag is ~180mm away. 160mm is a safe "clear" threshold.
      // ANGLE GATE: Ignore all sensor data until we have physically swung past the corner peg (>40 deg).
      if (dCenter > 160 && fabs(currentHeadingDeg) > 40.0f) { 
        centerCleared = true;
        
        // Add a 15-degree "square up" to the current angle to finish perfectly parallel.
        // Change 15.0f to 20.0f if the bot exits too tight to the inner wall.
        targetHeadingDeg = currentHeadingDeg + (turnLeft ? 32.0f : -32.0f);
        
        Serial.printf("Opening found at %.1f deg. Squaring up to %.1f deg.\n", currentHeadingDeg, targetHeadingDeg);
      }
      
      pivotRpm = turnLeft ? scanRpm : -scanRpm; 
      
    } else {
      // PHASE 2: Proportional heading controller down to the squared-up target
      float headingError = targetHeadingDeg - currentHeadingDeg;
      pivotRpm = KP_TURN * headingError;
      pivotRpm = constrain(pivotRpm, -MAX_PIVOT_RPM, MAX_PIVOT_RPM);
      
      if (fabs(headingError) > HEADING_TOLERANCE_DEG && fabs(pivotRpm) < MIN_PIVOT_RPM) {
        pivotRpm = (pivotRpm >= 0) ? MIN_PIVOT_RPM : -MIN_PIVOT_RPM;
      }

      // Exit condition: Reached the squared-up angle and stopped rotating
      if (fabs(headingError) < HEADING_TOLERANCE_DEG && fabs(gz) < GYRO_RATE_THRESHOLD_DPS) {
        break; 
      }
    }

    // --- Motor PID at the same fixed cadence used everywhere else ---
    if (nowMs - lastMotorMs >= 10) {
      float dt = (nowMs - lastMotorMs) / 1000.0f;
      lastMotorMs = nowMs;
      updateEncoders(dt);
      
      if (!centerCleared) {
          // Manual RPM during scan phase
          float leftSpeed = turnLeft ? -scanRpm : scanRpm;
          float rightSpeed = turnLeft ? scanRpm : -scanRpm;
          driveWheelsRPM(leftSpeed, rightSpeed, dt);
      } else {
          // PID proportional stop
          driveWheelsRPM(-pivotRpm, pivotRpm, dt);
      }
    }

    // Safety net only — should never trigger in normal operation.
    if (nowMs - turnStartMs > TURN_TIMEOUT_MS) break;
  }

  // INLINE ACTIVE BRAKE: Kill rotational momentum instantly
  digitalWrite(PIN_AIN1, HIGH); digitalWrite(PIN_AIN2, HIGH);
  digitalWrite(PIN_BIN1, HIGH); digitalWrite(PIN_BIN2, HIGH);
  ledcWrite(PIN_PWMA, 255); ledcWrite(PIN_PWMB, 255);
  delay(60);
  applyMotorPWM(0, 0);
  delay(100);

  integralL = 0;
  integralR = 0;

  // Re-sync all ToF readings immediately so computeSteering() doesn't
  // act on stale pre-turn distances on the very next loop iteration.
  selectMux(CH_TOF_LEFT);
  int rL = tofLeft.readRangeContinuousMillimeters();
  dLeft = (tofLeft.timeoutOccurred() || rL > 8000) ? 8190 : rL - OFFSET_LEFT;

  selectMux(CH_TOF_CENTER);
  int rC = tofCenter.readRangeContinuousMillimeters();
  dCenter = (tofCenter.timeoutOccurred() || rC > 8000) ? 8190 : rC - OFFSET_CENTER;

  selectMux(CH_TOF_RIGHT);
  int rR = tofRight.readRangeContinuousMillimeters();
  dRight = (tofRight.timeoutOccurred() || rR > 8000) ? 8190 : rR - OFFSET_RIGHT;

  selectMux(CH_TOF_DIAG_L); // NEW
  int rDL = tofDiagL.readRangeContinuousMillimeters();
  dDiagL = (tofDiagL.timeoutOccurred() || rDL > 8000) ? 8190 : rDL - OFFSET_DIAG_L;

  selectMux(CH_TOF_DIAG_R); // NEW
  int rDR = tofDiagR.readRangeContinuousMillimeters();
  dDiagR = (tofDiagR.timeoutOccurred() || rDR > 8000) ? 8190 : rDR - OFFSET_DIAG_R;

  // Force wall-presence / diagonal-danger flags and the steering derivative
  // to re-latch fresh off the just-refreshed readings, instead of carrying
  // over pre-turn state.
  leftWallPresent  = (dLeft  > 0 && dLeft  < WALL_APPEAR_DIST);
  rightWallPresent = (dRight > 0 && dRight < WALL_APPEAR_DIST);
  diagLDanger = (dDiagL > 0 && dDiagL < DIAG_DANGER_DIST); // NEW
  diagRDanger = (dDiagR > 0 && dDiagR < DIAG_DANGER_DIST); // NEW
  diagLTripCount = 0; // NEW — force debounce to re-confirm fresh after a turn
  diagRTripCount = 0; // NEW
  prevSteerError = 0.0f;
  lastSteerMs = 0; // makes computeSteering() use its dt fallback on the next call

  distanceSinceTurnMm = 0.0f;
  postTurnRampActive  = true; // hold the stepped speed ramp for the next ~180mm instead of jumping straight back to BASE_RPM
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
  tofLeft.startContinuous(); // free-running; round-robin polls for fresh data instead of triggering a new conversion every read

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

  // NEW: diagonal corner sensors, same init pattern as the other three
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

  // Robot must be stationary through this whole call — it calibrates the
  // gyro's zero-rate bias. This runs once, at boot, on Wire1 only.
  initGyro();

  delay(2000); 
  prevMotorTime = millis();
}

// ============================================================================
// MAIN LOOP: SCHEDULER
// ============================================================================
void loop() {
  unsigned long now = millis();

  // ---------------------------------------------------------
  // FAST TRACK: Motor PID Loop (Every 10ms / 100Hz)
  // NOTE: no gyro anywhere in here — normal driving is exactly your
  // original working behavior.
  // ---------------------------------------------------------
  if (now - prevMotorTime >= 10) {
    float dt = (now - prevMotorTime) / 1000.0;
    prevMotorTime = now;

    updateEncoders(dt);
    driveWheelsRPM(targetRpmL, targetRpmR, dt);
  }

  // ---------------------------------------------------------
  // SLOW TRACK: Sensor Round-Robin (continuous mode, non-blocking)
  // Each case is now a cheap "is new data ready?" check plus a read only
  // when it is — no more blocking a full conversion time on every call.
  // Extended from 3 states to 5 to fold in the two diagonal sensors; each
  // case is still just a readiness check plus, at most, one register read,
  // so this does not meaningfully slow the round-robin cadence.
  // ---------------------------------------------------------
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

    case 3: // NEW
      selectMux(CH_TOF_DIAG_L);
      if (tofDataReady(tofDiagL)) {
        int raw = tofDiagL.readRangeContinuousMillimeters();
        dDiagL = (tofDiagL.timeoutOccurred() || raw > 8000) ? 8190 : raw - OFFSET_DIAG_L;
      }
      tofSequenceState = 4;
      break;

    case 4: // NEW
      selectMux(CH_TOF_DIAG_R);
      if (tofDataReady(tofDiagR)) {
        int raw = tofDiagR.readRangeContinuousMillimeters();
        dDiagR = (tofDiagR.timeoutOccurred() || raw > 8000) ? 8190 : raw - OFFSET_DIAG_R;
      }
      tofSequenceState = 0;
      break;
  }

  // ---------------------------------------------------------
  // STEERING TRACK: fixed cadence, decoupled from loop() speed
  //
  // computeSteering()'s derivative term is (error - prevError) / dt. The ToF
  // round-robin above only actually refreshes dLeft/dRight once every ~20ms
  // (their own measurement period) - but continuous mode made loop() itself
  // spin much faster than that. Calling computeSteering() on every loop pass
  // meant dt was sometimes just 1-2ms while the underlying reading hadn't
  // changed at all, so the one call that *did* land on a fresh sample got a
  // real error delta divided by a tiny dt - a derivative spike far larger
  // than KD_CENTER was ever tuned for. That's what turned into oscillation
  // once BASE_RPM went up (higher speed gives that spike more authority per
  // control cycle), and it hit hardest right after a turn because
  // prevSteerError/lastSteerMs get reset there, so the very next real update
  // was guaranteed to produce exactly this kind of kick.
  //
  // Gating this block to a fixed period restores a consistent, predictable
  // dt matching the sensors' real update rate, regardless of how fast the
  // outer loop() spins.
  // ---------------------------------------------------------
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

  // Throttled debug print (5Hz) so you can watch dCenter live on the Serial
  // monitor while approaching a corner.
  static unsigned long lastDebugMs = 0;
  if (now - lastDebugMs >= 200) {
    lastDebugMs = now;
    Serial.printf("L:%d  DL:%d  C:%d  DR:%d  R:%d  steer:%.1f  fwd:%.1f\n",
                  dLeft, dDiagL, dCenter, dDiagR, dRight, smoothSteerRPM, smoothForwardRpm);
  }
}
