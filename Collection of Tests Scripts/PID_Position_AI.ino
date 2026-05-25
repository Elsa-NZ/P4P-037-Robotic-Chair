// =============================================================================
//  Maxon BLDC — Angular Position Control
//  Hardware: Arduino Uno/Mega + ESCON 70/10 + Maxon BLDC with quadrature encoder
//
//  ESCON 70/10 wiring (configure in ESCON Studio first):
//    Pin 2  (ESCON) ← Enable output         → Arduino pin 7 (digital out)
//    Pin 3  (ESCON) ← Analog set-value in   → Arduino pin 9 (PWM) + RC low-pass filter
//    Pin 4  (ESCON) ← Direction / Sign in   → Arduino pin 8 (digital out)
//
//  Encoder wiring:
//    Encoder A  → Arduino pin 2  (INT0 — hardware interrupt)
//    Encoder B  → Arduino pin 3  (INT1 — hardware interrupt)
//    Encoder Z  → Arduino pin 4  (optional index / home pulse)
//    GND        → Arduino GND
//    Vcc (5 V)  → Arduino 5 V
//
//  NOTE: The ESCON expects an analog set-value in the range 0–5 V
//        representing 0 … +max speed. Direction is set separately via PIN_DIR.
//        Use an RC low-pass filter (10 kΩ + 100 nF) on the PWM output to
//        convert it to a smooth DC level.
//        Alternatively use an external DAC (MCP4725 etc.) for cleaner output.
//
//  Configure in ESCON Studio:
//    - Operation mode : Speed controller (closed-loop via encoder)
//    - Enable polarity: active high
//    - Analog input   : 0–5 V → 0 … +max speed  (unidirectional)
//    - Direction      : digital input (PIN_DIR) controls forward/reverse
//    - Max speed, current limits, ramp times → tune to your motor spec sheet
// =============================================================================

#include <Arduino.h>

// ── Pin definitions ──────────────────────────────────────────────────────────
const uint8_t PIN_ENABLE    = 7;   // ESCON Enable (active HIGH)
const uint8_t PIN_PWM_CMD   = 9;   // Speed command (PWM → RC filter → analog)
const uint8_t PIN_DIR       = 8;   // Direction (HIGH = forward)
const uint8_t PIN_ENC_A     = 2;   // Encoder channel A (INT0)
const uint8_t PIN_ENC_B     = 3;   // Encoder channel B (INT1)
const uint8_t PIN_ENC_Z     = 4;   // Encoder index / home

// ── Encoder & motion parameters ─────────────────────────────────────────────
// Check your encoder's datasheet for CPR (counts per revolution).
// Multiply by 4 for full quadrature (both edges of A and B).
const long   COUNTS_PER_REV = 500L*51*4;  // e.g. 500 CPR * 51:1 gearbox = 25500 counts/rev at motor shaft
const double DEG_PER_COUNT  = 360.0 / COUNTS_PER_REV;

// ── PID gains (tune these for your motor + load) ────────────────────────────
// Start with small Kp, Ki = 0, Kd = 0.  Increase Kp until oscillation,
// then add Kd to damp, then a small Ki to eliminate steady-state error.
double Kp = 1.5;
double Ki = 0.50;
double Kd = 0.00;

// ── PID state ────────────────────────────────────────────────────────────────
volatile long encoderCount = 0;    // Updated in ISR — always read with noInterrupts()
double  integral    = 0.0;
double  prevError   = 0.0;
double  targetAngle = 0.0;         // Degrees

// ── Timing ───────────────────────────────────────────────────────────────────
const unsigned long CONTROL_LOOP_MS = 10;   // 100 Hz control loop
unsigned long       lastLoopTime    = 0;

// ── Limits ───────────────────────────────────────────────────────────────────
const double MAX_INTEGRAL    = 50.0;   // Anti-windup clamp
const int    PWM_MAX         = 230;    // Arduino 8-bit PWM maximum
const int    PWM_DEADBAND    = 5;      // Ignore very small commands (reduces jitter)
const int    PWM_STOP        = 5;     // 10% of 255 — ESCON zero-speed threshold
const int    PWM_FULL        = 230;    // 90% of 255 — ESCON max-speed threshold
const double POSITION_TOL    = 0.5;   // Degrees — "close enough" tolerance

// =============================================================================
//  Interrupt service routines — quadrature decoding
// =============================================================================
void onEncoderA() {
  bool a = digitalRead(PIN_ENC_A);
  bool b = digitalRead(PIN_ENC_B);
  encoderCount += (a == b) ? 1 : -1;
}

void onEncoderB() {
  bool a = digitalRead(PIN_ENC_A);
  bool b = digitalRead(PIN_ENC_B);
  encoderCount += (a != b) ? 1 : -1;
}

// =============================================================================
//  Helper: read encoder count atomically
// =============================================================================
long readEncoder() {
  noInterrupts();
  long c = encoderCount;
  interrupts();
  return c;
}

// =============================================================================
//  Helper: convert encoder count → degrees (unwrapped, not modulo 360)
// =============================================================================
double countsToDegrees(long counts) {
  return (double)counts * DEG_PER_COUNT;
}

// =============================================================================
//  Helper: set motor output
//    output: -255 … +255  (negative = reverse)
// =============================================================================
void setMotorOutput(int output) {
  if (abs(output) < PWM_DEADBAND) {
    // Below deadband — hold position (keep enable, zero speed)
    analogWrite(PIN_PWM_CMD, PWM_STOP);  // 10% = zero speed on ESCON
    return;
  }

  if (output > 0) {
    digitalWrite(PIN_DIR, HIGH);
    // Map 0…255 → PWM_STOP…PWM_FULL (10%…90% = 0…max forward speed)
    int pwm = map(output, 0, 255, PWM_STOP, PWM_FULL);
    analogWrite(PIN_PWM_CMD, constrain(pwm, PWM_STOP, PWM_FULL));
  } else {
    digitalWrite(PIN_DIR, LOW);
    // Map 0…255 → PWM_STOP…PWM_FULL (10%…90% = 0…max reverse speed)
    int pwm = map(-output, 0, 255, PWM_STOP, PWM_FULL);
    analogWrite(PIN_PWM_CMD, constrain(pwm, PWM_STOP, PWM_FULL));
  }
}

// =============================================================================
//  Helper: disable motor (safe stop)
// =============================================================================
void disableMotor() {
  analogWrite(PIN_PWM_CMD, PWM_STOP);  // 10% = zero-speed command before disabling
  delay(10);
  digitalWrite(PIN_ENABLE, LOW);
}

// =============================================================================
//  Helper: enable motor
// =============================================================================
void enableMotor() {
  analogWrite(PIN_PWM_CMD, PWM_STOP);  // 10% = pre-set to zero before enabling
  digitalWrite(PIN_ENABLE, HIGH);
  delay(50);  // Short settle time
}

// =============================================================================
//  Move to absolute angle (blocking, with timeout)
//    angle_deg: target in degrees (unwrapped from home)
//    timeout_ms: give up after this many milliseconds (0 = no timeout)
// =============================================================================
void moveToAngle(double angle_deg, unsigned long timeout_ms = 0) {
  targetAngle = angle_deg;
  integral    = 0.0;
  prevError   = 0.0;

  unsigned long startTime = millis();
  unsigned long stableTime = 0;
  bool wasStable = false;

  Serial.print("Moving to ");
  Serial.print(angle_deg, 2);
  Serial.println(" deg...");

  while (true) {
    unsigned long now = millis();

    // Timeout guard
    if (timeout_ms > 0 && (now - startTime) > timeout_ms) {
      Serial.println("TIMEOUT — target not reached.");
      break;
    }

    if ((now - lastLoopTime) >= CONTROL_LOOP_MS) {
      double dt = (now - lastLoopTime) / 1000.0;
      lastLoopTime = now;

      double currentAngle = countsToDegrees(readEncoder());
      double error        = targetAngle - currentAngle;

      // Anti-windup integral accumulation
      integral += error * dt;
      integral  = constrain(integral, -MAX_INTEGRAL, MAX_INTEGRAL);

      double derivative = (error - prevError) / dt;
      prevError = error;

      double pidOut = (Kp * error) + (Ki * integral) + (Kd * derivative);
      int    output = (int)constrain(pidOut, -255.0, 255.0);

      setMotorOutput(output);

      // Serial telemetry (useful for Serial Plotter tuning)
      Serial.print("target:"); Serial.print(targetAngle, 2);
      Serial.print(" pos:");   Serial.print(currentAngle, 2);
      Serial.print(" err:");   Serial.print(error, 2);
      Serial.print(" out:");   Serial.println(output);

      // Stable within tolerance for 300 ms → done
      if (abs(error) < POSITION_TOL) {
        if (!wasStable) {
          stableTime = now;
          wasStable = true;
        } else if ((now - stableTime) > 300) {
          setMotorOutput(0);
          Serial.println("Position reached.");
          break;
        }
      } else {
        wasStable = false;
      }
    }
  }
}

// =============================================================================
//  Home routine: rotate slowly until encoder index (Z) goes HIGH
//  Call this once on startup if you need an absolute reference.
// =============================================================================
void homeMotor() {
  Serial.println("Homing...");
  // Slow forward until Z pulse
  analogWrite(PIN_PWM_CMD, 40);   // Very slow forward (~16% — just above zero-speed threshold)
  digitalWrite(PIN_DIR, HIGH);

  unsigned long startHome = millis();
  while (digitalRead(PIN_ENC_Z) == LOW) {
    if (millis() - startHome > 10000) {
      Serial.println("Homing FAILED — no index pulse within 10 s.");
      disableMotor();
      return;
    }
  }

  // Zero the encoder count at the index
  noInterrupts();
  encoderCount = 0;
  interrupts();

  setMotorOutput(0);
  Serial.println("Homed. Encoder zeroed.");
  delay(200);
}

// Test if encoders are working: prin

// =============================================================================
//  setup
// =============================================================================
void setup() {
  Serial.begin(115200);
  Serial.println("Maxon BLDC position controller starting...");

  // Pin modes
  pinMode(PIN_ENABLE,   OUTPUT);
  pinMode(PIN_PWM_CMD,  OUTPUT);
  pinMode(PIN_DIR,      OUTPUT);
  pinMode(PIN_ENC_A,    INPUT_PULLUP);
  pinMode(PIN_ENC_B,    INPUT_PULLUP);
  pinMode(PIN_ENC_Z,    INPUT_PULLUP);

  // Start disabled, zero command
  digitalWrite(PIN_ENABLE, LOW);
  analogWrite(PIN_PWM_CMD, PWM_STOP);

  // Attach encoder interrupts
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), onEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_B), onEncoderB, CHANGE);

  delay(500);
  enableMotor();

  // Optional: home to encoder index on startup
  homeMotor();
}

// =============================================================================
//  loop — example sequence; adapt to your application
// =============================================================================
void loop() {
  // Example: move through a sequence of angles
//   moveToAngle(90.0,  0);   delay(5000);
//   moveToAngle(180.0, 0);   delay(5000);
//   moveToAngle(45.0,  0);   delay(5000);
//   moveToAngle(0.0,   0);   delay(5000);

  // Or: read a target angle from Serial
  // homeMotor();
  if (Serial.available()) {
    double deg = Serial.parseFloat();
    moveToAngle(deg, 000);
  }
}
