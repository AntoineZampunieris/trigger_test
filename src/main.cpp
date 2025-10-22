#include <Arduino.h>

// ===== Pins =====
const uint8_t PIN_EDGE_A = 2;   // Comparator A -> D2 (master)
const uint8_t PIN_EDGE_B = 3;   // Comparator B -> D3 (slave)

const uint8_t MASTER_PWM = 5;   // Motor A PWM
const uint8_t MASTER_DIR = 4;   // Motor A DIR (forward level as wired)
const uint8_t SLAVE_PWM  = 6;   // Motor B PWM
const uint8_t SLAVE_DIR  = 7;   // Motor B DIR (forward level as wired)

// ===== Fixed settings =====
const uint8_t MASTER_PWM_BASE = 90;  // open-loop base for master (0..255)
const uint8_t SLAVE_PWM_BASE  = 80;  // nominal base for slave (0..255)
const uint8_t SLAVE_PWM_MIN   = 20;  // helps overcome static friction
const uint8_t SLAVE_PWM_MAX   = 255;

const uint16_t EDGE_LOCKOUT_US = 100;   // ignore edges that are too close
const uint32_t CTRL_DT_US      = 100;   // control loop period (µs)

// ===== Controller gains (tune these) =====
// PWM correction = Kp * (Δt in us) + Ki * ∫(Δt in us)*dt
// If slave lags (Δt>0), PWM will increase; if it leads (Δt<0), PWM will decrease.
float Kp = 0.0020f;      // counts per microsecond (e.g., 0.002 * 500us = 1 PWM count)
float Ki = 0.0005f;      // counts per microsecond per second

// ===== Desired phase offset =====
// Set the desired phase difference between A and B.
// You can set either cycles or degrees; both write the same variable.
volatile float phaseOffset_cycles = 0.25f;  // 0.0 = in-phase; +0.25 = +90°; +0.5 = 180°; range (-0.5..+0.5)

// Helper setters (call from elsewhere if you want to change at runtime)
inline void setPhaseOffsetCycles(float cyc) {
  noInterrupts();
  // wrap to (-0.5, +0.5]
  while (cyc >  0.5f) cyc -= 1.0f;
  while (cyc <= -0.5f) cyc += 1.0f;
  phaseOffset_cycles = cyc;
  interrupts();
}
inline void setPhaseOffsetDegrees(float deg) {
  setPhaseOffsetCycles(deg / 360.0f);
}

// ===== ISR state =====
volatile uint32_t tA = 0, pA = 0, perA = 0;
volatile uint32_t tB = 0, pB = 0, perB = 0;

void isrA() {
  uint32_t t = micros();
  if (t - tA < EDGE_LOCKOUT_US) return;
  pA = tA; tA = t;
  if (pA) perA = tA - pA;
}

void isrB() {
  uint32_t t = micros();
  if (t - tB < EDGE_LOCKOUT_US) return;
  pB = tB; tB = t;
  if (pB) perB = tB - pB;
}

// Wrap signed error into (-T/2, +T/2]
static inline int32_t wrapHalfPeriod(int32_t e_us, uint32_t T_us) {
  if (!T_us) return e_us;
  int32_t half = (int32_t)(T_us / 2u);
  while (e_us >  half) e_us -= (int32_t)T_us;
  while (e_us <= -half) e_us += (int32_t)T_us;
  return e_us;
}

void setup() {
  // Directions fixed (keep your chosen forward level)
  pinMode(MASTER_DIR, OUTPUT);
  pinMode(SLAVE_DIR,  OUTPUT);
  digitalWrite(MASTER_DIR, LOW);
  digitalWrite(SLAVE_DIR,  LOW);

  // PWM outputs
  pinMode(MASTER_PWM, OUTPUT);
  pinMode(SLAVE_PWM,  OUTPUT);
  analogWrite(MASTER_PWM, MASTER_PWM_BASE);
  analogWrite(SLAVE_PWM,  SLAVE_PWM_BASE);

  // Edge inputs (enable pull-ups for open-collector comparators)
  pinMode(PIN_EDGE_A, INPUT_PULLUP);
  pinMode(PIN_EDGE_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_EDGE_A), isrA, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_EDGE_B), isrB, RISING);

  // Example: set a 90° phase lead for A relative to B
  //setPhaseOffsetDegrees(90.0f);   // comment/uncomment as needed
}

void loop() {
  static uint32_t tNext = micros();
  static float integ = 0.0f;          // integral of Δt (us·s)
  static int16_t slaveCmd = SLAVE_PWM_BASE;

  if ((int32_t)(micros() - tNext) >= 0) {
    tNext += CTRL_DT_US;
    float dt = CTRL_DT_US * 1e-6f;    // seconds

    // Snapshot ISR vars
    noInterrupts();
    uint32_t TA = tA, TB = tB;
    uint32_t T  = perA ? perA : perB; // prefer master period if available
    float    off_cyc = phaseOffset_cycles;
    interrupts();

    // If we don't have valid edges yet, relax toward base and decay integral
    if (T == 0 || TA == 0 || TB == 0) {
      // gentle return to base
      slaveCmd += (int16_t)(0.05f * (SLAVE_PWM_BASE - slaveCmd));
      // light integral decay to avoid windup at startup
      integ *= 0.98f;
      if (slaveCmd < SLAVE_PWM_MIN) slaveCmd = SLAVE_PWM_MIN;
      if (slaveCmd > SLAVE_PWM_MAX) slaveCmd = SLAVE_PWM_MAX;
      analogWrite(SLAVE_PWM, (uint8_t)slaveCmd);
      return;
    }

    // Desired offset in microseconds based on current period
    int32_t desired_offset_us = (int32_t)(off_cyc * (float)T);

    // Phase error in microseconds (A leads positive), relative to desired offset
    int32_t e_us = (int32_t)(TA - TB) - desired_offset_us;
    e_us = wrapHalfPeriod(e_us, T);

    // PI on time error
    integ += (float)e_us * dt;                    // us·s
    // Anti-windup clamp (keeps integral reasonable)
    const float I_MAX = 100000.0f;                // ~1e5 us·s
    if (integ >  I_MAX) integ =  I_MAX;
    if (integ < -I_MAX) integ = -I_MAX;

    float corr = Kp * (float)e_us + Ki * integ;   // PWM counts (can be fractional)
    int16_t target = (int16_t)((float)SLAVE_PWM_BASE + corr);

    // Clamp and minimum
    if (target < SLAVE_PWM_MIN) target = SLAVE_PWM_MIN;
    if (target > SLAVE_PWM_MAX) target = SLAVE_PWM_MAX;

    // Small slew limit (optional) to avoid abrupt jumps
    const int16_t SLEW = 4; // max change per control step
    if (target > slaveCmd + SLEW)      slaveCmd += SLEW;
    else if (target < slaveCmd - SLEW) slaveCmd -= SLEW;
    else                               slaveCmd  = target;

    analogWrite(SLAVE_PWM, (uint8_t)slaveCmd);
  }
}
