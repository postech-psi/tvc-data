/* =====================================================================
   TVC Gimbal Characterization  —  ESP32-WROOM-32D (KS0413)
                                   2x PTK 8515MG-D
                                   Adafruit ICM-20948 on the motor plate
   ---------------------------------------------------------------------
   DESIGN RULE: this firmware MEASURES and DUMPS.  It does not average,
   filter, fit, or bias-correct anything.  Every sample leaves the chip
   raw, in LSB, with its own timestamp and health flags.  All decisions
   about windows, scaling, and models are made on the PC, where they can
   be changed without re-running the bench.

   WIRING
     Servo A (pitch) signal -> GPIO 18
     Servo B (roll)  signal -> GPIO 19
     Servo power  -> separate BEC 7.4V 3A+, 1000uF across V+/GND
     Common GND   -> BEC GND == ESP32 GND        (mandatory)
     ICM-20948    -> SDA GPIO 21, SCL GPIO 22, 3V3, addr 0x69

   *** RIG ORIENTATION ***
   The accelerometer measures tilt from gravity.  The axis being swept
   MUST be horizontal so the plate tilts up/down.  A vertical rotation
   axis reads nothing and the mapping test returns garbage.

   Mount the real motor (or a dummy of equal mass).  Load inertia moves
   both w_n and zeta; a servo characterized unloaded is not the servo
   you fly.

   SERIAL 460800, newline-terminated commands:
     <number>      jog selected servo to that pulse (us)
     + - ++ --     jog by step / 5x step
     s <us>        set jog step            n     both servos to neutral
     w <lo> <hi>   set soft window         x <0|1>  select servo A / B
     m             mark current pulse as a travel limit
     u             detach (limp)           v     re-attach
     ?             status                  h     help

     c   IMU health   : noise floor, bias, |g|, throughput, axis report
     z   live readout : 20 s of tilt + rate, for checking orientation
     a   TEST A       : pulse <-> angle mapping, 4-pass sweep
     b   TEST B       : step response, 1 kHz gyro burst
     k   TEST K       : deadband / resolution, 1 us increments
     p   TEST P       : static repeatability + backlash, 20 reps

   OUTPUT FORMAT — one schema for every test, so one parser:

     #META key=value ...        run manifest, emitted at the start of each test
     #COLS ...                  column header
     E,<t_us>,<phase>,<seq>,<axis>,<cmd_a>,<cmd_b>,,,,,,,<flags>   event
     S,<t_us>,<phase>,<seq>,<axis>,<cmd_a>,<cmd_b>,ax,ay,az,gx,gy,gz,<flags>

   t_us    micros() since the test started (E rows are the command instant)
   phase   what is happening: zero|dwell|pre|post|settle|hold
   seq     step index within the test
   axis    which servo is being driven this run (0=A, 1=B)
   ax..gz  RAW int16 LSB, straight off the chip.  Scale on the PC using
           acc_lsb_per_g / gyro_lsb_per_dps from #META.
   flags   bit0 = I2C read failed (sample values are stale/zero)
           bit1 = sample loop missed its deadline
   ===================================================================== */

#include <Wire.h>
#include <ESP32Servo.h>
#include <Preferences.h>

#define FW_VERSION "tvc_char 2.1"

// ---------------- servo ----------------
#define SERVO_A_PIN     18
#define SERVO_B_PIN     19
#define SERVO_HZ        333        // PTK 8515MG-D supports it; 3 ms frame
#define PULSE_MIN       500        // driver range, NOT a safety limit
#define PULSE_MAX       2500
#define PULSE_NEUTRAL   1520       // PTK/Futaba convention, not 1500

/* ---------------- MECHANICAL TRAVEL LIMITS ----------------
   YOU set these.  They are the numbers you read off the rig by jogging
   to each stop by hand, and nothing in this firmware overwrites them.

   Why they are enforced at all: a servo commanded past its mechanical
   stop does not fault or complain.  It holds full torque against the
   stop for as long as the command stands -- 500 ms per point, 56
   points, 4 passes in TEST A -- which strips gears, cooks the BEC, and
   produces data that looks perfectly plausible, because a plate held
   against a stop reports a beautifully stable angle.  So every test
   checks its whole intended range against these limits up front and
   ABORTS if it does not fit.  It never silently clamps: a clamped point
   is a stalled servo logged as good data.

   Three ways to set them, in order of preference:
     1. edit LIMIT_A_LO/HI and LIMIT_B_LO/HI below, set LIMITS_PRESET 1,
        re-flash.  The values live in source, in git, with the rig.
     2. 'w <lo> <hi>' at the prompt -- saved to NVS, survives reset
     3. 'l' (TEST L) suggests values by feeling for the stops, but it
        only PRINTS them.  It never writes.  Treat it as a second
        opinion on numbers you already measured by hand.

   Enter the USABLE range, i.e. the stop position with your safety
   margin already subtracted.  30-50 us is a sensible margin. */
#define LIMITS_PRESET   1         // set to 1 once the four values below
                                   // are the real measured travel
#define LIMIT_A_LO      1370       // servo A (GPIO 18) usable range
#define LIMIT_A_HI      1690
#define LIMIT_B_LO      1310       // servo B (GPIO 19) usable range
#define LIMIT_B_HI      2040

#define SAFE_START      1440       // the only range trusted while
#define SAFE_END        1600       // uncalibrated: +/- 80 us of neutral,
                                   // small enough that a mis-assembled
                                   // linkage buzzes rather than breaks
#define LIMIT_MARGIN    40         // us backed off the detected stop
#define LIMIT_STEP      5          // creep increment while searching
#define LIMIT_DWELL_MS  150        // short: this is time spent near a stop
#define LIMIT_MAX_TRAVEL 450       // us from neutral, hard search cap
#define STALL_WIN       4          // steps in the motion-detect window
#define STALL_FRAC      0.30f      // <30% of expected motion = stopped

struct AxisLimits {
  int  lo, hi;                     // usable range, margin already applied
  bool valid;
  float gain_deg_per_us;           // measured during the limit search
};
#if LIMITS_PRESET
AxisLimits lim[2] = {{LIMIT_A_LO, LIMIT_A_HI, true, 0.0f},
                     {LIMIT_B_LO, LIMIT_B_HI, true, 0.0f}};
#else
AxisLimits lim[2] = {{SAFE_START, SAFE_END, false, 0.0f},
                     {SAFE_START, SAFE_END, false, 0.0f}};
#endif
Preferences prefs;

// ---------------- TEST A : mapping ----------------
// A_START / A_END are NOT constants -- they are derived from the
// measured limits at run time.  See axStart() / axEnd().
#define A_STEP          10
#define A_DWELL_MS      500        // logged in full, start to finish
#define A_LOG_HZ        200
#define A_ZERO_MS       1500       // neutral reference before each sweep

// ---------------- TEST B : step ----------------
#define B_LOG_HZ        1000
#define B_PRE_MS        200        // baseline before the step
#define B_POST_MS       2000       // long tail: the drift fit needs it
#define B_LOG_N         ((B_PRE_MS + B_POST_MS) * B_LOG_HZ / 1000)
#define B_ARM_MS        1800       // settle at the start pulse before logging

// ---------------- TEST K : deadband ----------------
#define K_SPAN          40         // +/- us around each center
#define K_DWELL_MS      250
#define K_LOG_HZ        100

// ---------------- TEST P : repeatability ----------------
#define P_REPS          20
#define P_APPROACH      200        // us of run-up on each side
#define P_DWELL_MS      400
#define P_LOG_HZ        100

// ================= ICM-20948 =================
#define ICM_ADDR        0x69
#define REG_BANK_SEL    0x7F
// bank 0
#define B0_WHO_AM_I     0x00
#define B0_USER_CTRL    0x03
#define B0_PWR_MGMT_1   0x06
#define B0_PWR_MGMT_2   0x07
#define B0_ACCEL_XOUT   0x2D       // 0x2D..0x32 accel, 0x33..0x38 gyro,
                                   // 0x39..0x3A TEMP -- temp is AFTER the
                                   // gyro here, unlike the MPU6050.  Read
                                   // 12 contiguous bytes, skip nothing.
// bank 2
#define B2_GYRO_SMPLRT_DIV  0x00
#define B2_GYRO_CONFIG_1    0x01
#define B2_ACCEL_SMPLRT_1   0x10
#define B2_ACCEL_SMPLRT_2   0x11
#define B2_ACCEL_CONFIG     0x14

#define GYRO_LSB_PER_DPS  16.4f    // +/-2000 dps
#define ACC_LSB_PER_G     16384.0f // +/-2 g

#define FLAG_I2C_ERR    0x01
#define FLAG_LATE       0x02

// ---------------- state ----------------
Servo servoA, servoB;
int   posA = PULSE_NEUTRAL, posB = PULSE_NEUTRAL;
bool  attached = false;
int   sel  = 0;                    // selected servo: 0=A, 1=B
int   jogStep = 10;

uint32_t tRef = 0;                 // per-test time origin
uint32_t i2cErrors = 0;

/* ---- IMU health, measured automatically at boot ----
   Every field here is REPORTED and travels in the #META of every run.
   The gyro bias in particular is measured but never subtracted: the
   analysis recovers bias AND drift from each trace's own pre-step and
   settled-tail windows, which a one-shot startup calibration cannot
   see.  Storing it here gives the PC an independent cross-check -- if
   the per-trace fit disagrees wildly with the boot bias, something
   moved during the run. */
struct ImuHealth {
  bool  alive;                     // WHO_AM_I answered 0xEA
  bool  pass;                      // all criteria met
  float bias[3];                   // gyro bias, dps  (reported, not applied)
  float gnoise[3];                 // gyro sd, dps
  float anoise[3];                 // accel sd, g
  float gmag;                      // |g|
  float rate_hz;                   // 12-byte read throughput
  int   fails;
};
ImuHealth H = {false, false, {0,0,0}, {0,0,0}, {0,0,0}, 0, 0, 0};

// ================= I2C primitives =================
void wr(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(ICM_ADDR);
  Wire.write(reg); Wire.write(val);
  Wire.endTransmission();
}

uint8_t rd(uint8_t reg) {
  Wire.beginTransmission(ICM_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)ICM_ADDR, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0xFF;
}

void bank(uint8_t b) { wr(REG_BANK_SEL, b << 4); }

// Explicit hi-then-lo.  `(Wire.read()<<8)|Wire.read()` is UB: C++ does
// not specify operand evaluation order, and the two builds disagree.
static inline int16_t be16() {
  uint8_t hi = Wire.read();
  uint8_t lo = Wire.read();
  return (int16_t)(((uint16_t)hi << 8) | lo);
}

bool imuInit() {
  bank(0);
  if (rd(B0_WHO_AM_I) != 0xEA) return false;

  wr(B0_PWR_MGMT_1, 0x80);  delay(120);        // reset
  bank(0);
  wr(B0_PWR_MGMT_1, 0x01);  delay(20);         // auto clock, wake
  wr(B0_PWR_MGMT_2, 0x00);                     // accel + gyro on
  wr(B0_USER_CTRL,  0x00);                     // no I2C master, no FIFO/DMP

  bank(2);
  // GYRO_CONFIG_1: DLPFCFG=7 (BW 361 Hz, group delay 0.17 ms),
  //                FS_SEL=3 (+/-2000 dps), FCHOICE=1 (DLPF on)
  wr(B2_GYRO_CONFIG_1, (7 << 3) | (3 << 1) | 1);
  wr(B2_GYRO_SMPLRT_DIV, 0);                   // ODR 1125 Hz
  // ACCEL_CONFIG: DLPFCFG=7 (BW 473 Hz), FS=0 (+/-2 g), FCHOICE=1
  wr(B2_ACCEL_CONFIG, (7 << 3) | (0 << 1) | 1);
  wr(B2_ACCEL_SMPLRT_1, 0);
  wr(B2_ACCEL_SMPLRT_2, 0);                    // ODR 1125 Hz

  bank(0);
  delay(50);
  return true;
}

// 12 contiguous bytes: accel XYZ then gyro XYZ.  No temperature skip.
bool readAG(int16_t *a, int16_t *g) {
  Wire.beginTransmission(ICM_ADDR);
  Wire.write(B0_ACCEL_XOUT);
  if (Wire.endTransmission(false) != 0) { i2cErrors++; return false; }
  if (Wire.requestFrom((uint8_t)ICM_ADDR, (uint8_t)12) != 12) {
    i2cErrors++; return false;
  }
  for (int i = 0; i < 3; i++) a[i] = be16();
  for (int i = 0; i < 3; i++) g[i] = be16();
  return true;
}

// ================= servo primitives =================
void attachServos() {
  servoA.setPeriodHertz(SERVO_HZ);
  servoB.setPeriodHertz(SERVO_HZ);
  servoA.attach(SERVO_A_PIN, PULSE_MIN, PULSE_MAX);
  servoB.attach(SERVO_B_PIN, PULSE_MIN, PULSE_MAX);
  servoA.writeMicroseconds(posA);
  servoB.writeMicroseconds(posB);
  attached = true;
}

uint32_t clampCount = 0;           // clamps are never silent; see below

int clampWin(int axis, int us) {
  const AxisLimits &L = lim[axis];
  if (us < L.lo) { clampCount++; return L.lo; }
  if (us > L.hi) { clampCount++; return L.hi; }
  return us;
}

// Writes the pulse and returns the micros() instant of the write.
// Every caller in a test logs that instant as an E row; nothing in the
// analysis is allowed to assume when the command happened.
//
// A clamp here means a test asked for something outside the measured
// travel.  That must never pass quietly: the sample would look like a
// normal settled point while the servo is actually stalled against a
// stop.  Tests pre-check their whole range (see rangeOK) so this should
// be unreachable during a test; it is counted and reported anyway.
uint32_t cmdWrite(int axis, int us) {
  int want = clampWin(axis, us);
  if (want != us)
    Serial.printf("#WARN clamp axis=%d asked=%d gave=%d limits=[%d,%d]\n",
                  axis, us, want, lim[axis].lo, lim[axis].hi);
  uint32_t t;
  if (axis == 0) { t = micros(); servoA.writeMicroseconds(want); posA = want; }
  else           { t = micros(); servoB.writeMicroseconds(want); posB = want; }
  return t;
}

/* Bypasses the measured window, clamped only to what the driver can
   physically emit.  ONLY the limit search may use this -- it is how the
   search reaches past the current (unknown) window to find the stop.
   Every other command path goes through cmdWrite. */
uint32_t rawWrite(int axis, int us) {
  if (us < PULSE_MIN) us = PULSE_MIN;
  if (us > PULSE_MAX) us = PULSE_MAX;
  uint32_t t;
  if (axis == 0) { t = micros(); servoA.writeMicroseconds(us); posA = us; }
  else           { t = micros(); servoB.writeMicroseconds(us); posB = us; }
  return t;
}

// ---- travel accessors: every test derives its range from these ----
int axLo(int a)     { return lim[a].lo; }
int axHi(int a)     { return lim[a].hi; }
// Mechanical centre.  If the true travel is asymmetric about the
// nominal 1520, tests must centre on the middle of the ACTUAL travel or
// the large steps run off one end.
int axCenter(int a) {
  int c = PULSE_NEUTRAL;
  if (c < lim[a].lo || c > lim[a].hi) c = (lim[a].lo + lim[a].hi) / 2;
  return c;
}
int axHalf(int a) {
  int c = axCenter(a);
  int h = min(c - lim[a].lo, lim[a].hi - c);
  return h;
}

// ---- preflight gates ----
bool haveLimits(int axis) {
  if (lim[axis].valid) return true;
  Serial.printf("!! axis %d has no measured travel limits.\n", axis);
  Serial.println(F("!! run 'l' (TEST L) first, or set them manually with "
                   "'w <lo> <hi>'."));
  Serial.println(F("!! refusing to run: an uncalibrated sweep drives the "
                   "servo into its hard stop."));
  return false;
}

// Abort rather than clamp.  Called by every test with the widest pulse
// it intends to command.
bool rangeOK(int axis, int lo, int hi, const char *what) {
  if (lo >= lim[axis].lo && hi <= lim[axis].hi) return true;
  Serial.printf("!! %s wants [%d,%d] but axis %d travel is [%d,%d]\n",
                what, lo, hi, axis, lim[axis].lo, lim[axis].hi);
  Serial.println(F("!! ABORTED (not clamped -- clamped points would stall "
                   "the servo and log as valid data)"));
  return false;
}

void saveLimits() {
  prefs.begin("tvc", false);
  for (int a = 0; a < 2; a++) {
    char k[8];
    snprintf(k, sizeof k, "lo%d", a); prefs.putInt(k, lim[a].lo);
    snprintf(k, sizeof k, "hi%d", a); prefs.putInt(k, lim[a].hi);
    snprintf(k, sizeof k, "ok%d", a); prefs.putBool(k, lim[a].valid);
    snprintf(k, sizeof k, "gn%d", a); prefs.putFloat(k, lim[a].gain_deg_per_us);
  }
  prefs.end();
  Serial.println(F("# limits saved to NVS"));
}

void loadLimits() {
  prefs.begin("tvc", true);
  for (int a = 0; a < 2; a++) {
    char k[8];
    snprintf(k, sizeof k, "ok%d", a); bool ok = prefs.getBool(k, false);
    if (!ok) continue;
    snprintf(k, sizeof k, "lo%d", a); lim[a].lo = prefs.getInt(k, SAFE_START);
    snprintf(k, sizeof k, "hi%d", a); lim[a].hi = prefs.getInt(k, SAFE_END);
    snprintf(k, sizeof k, "gn%d", a); lim[a].gain_deg_per_us = prefs.getFloat(k, 0);
    // sanity: refuse anything the driver could not have produced
    if (lim[a].lo >= PULSE_MIN && lim[a].hi <= PULSE_MAX &&
        lim[a].lo < lim[a].hi)
      lim[a].valid = true;
    else
      lim[a] = {SAFE_START, SAFE_END, false, 0.0f};
  }
  prefs.end();
}

// ================= output =================
void emitCols() {
  Serial.println(F("#COLS rec,t_us,phase,seq,axis,cmd_a,cmd_b,"
                   "ax,ay,az,gx,gy,gz,flags"));
}

void emitMeta(const char *test, int axis, const char *extra) {
  bank(2);
  uint8_t gcfg = rd(B2_GYRO_CONFIG_1), acfg = rd(B2_ACCEL_CONFIG);
  uint8_t gdiv = rd(B2_GYRO_SMPLRT_DIV);
  bank(0);
  uint8_t who = rd(B0_WHO_AM_I);

  // Config registers are read BACK from the chip, not echoed from the
  // #defines.  If a write silently failed, the manifest shows it.
  Serial.printf("#META fw=%s test=%s axis=%d servo_hz=%d neutral_us=%d "
                "pulse_min=%d pulse_max=%d win_lo=%d win_hi=%d "
                "pin_a=%d pin_b=%d\n",
                FW_VERSION, test, axis, SERVO_HZ, PULSE_NEUTRAL,
                PULSE_MIN, PULSE_MAX, lim[axis].lo, lim[axis].hi,
                SERVO_A_PIN, SERVO_B_PIN);
  Serial.printf("#META imu_addr=0x%02X who_am_i=0x%02X gyro_cfg1=0x%02X "
                "accel_cfg=0x%02X gyro_div=%u "
                "gyro_lsb_per_dps=%.4f acc_lsb_per_g=%.1f\n",
                ICM_ADDR, who, gcfg, acfg, gdiv,
                GYRO_LSB_PER_DPS, ACC_LSB_PER_G);
  Serial.printf("#META i2c_err_at_start=%lu %s\n",
                (unsigned long)i2cErrors, extra ? extra : "");

  // Sensor health travels with the data.  A run whose health block says
  // FAIL is still captured -- it is just labelled, so the analysis can
  // refuse to fit it instead of quietly producing a number.
  Serial.printf("#META imu_alive=%d imu_pass=%d imu_rate_hz=%.0f "
                "g_mag=%.4f\n", H.alive, H.pass, H.rate_hz, H.gmag);
  Serial.printf("#META gyro_bias_dps=%.4f,%.4f,%.4f "
                "gyro_noise_dps=%.4f,%.4f,%.4f "
                "acc_noise_g=%.5f,%.5f,%.5f\n",
                H.bias[0], H.bias[1], H.bias[2],
                H.gnoise[0], H.gnoise[1], H.gnoise[2],
                H.anoise[0], H.anoise[1], H.anoise[2]);
  Serial.printf("#META limits_source=%s axis_lo=%d axis_hi=%d "
                "axis_center=%d axis_half=%d\n",
                lim[axis].valid ? "user" : "UNCALIBRATED",
                lim[axis].lo, lim[axis].hi, axCenter(axis), axHalf(axis));
  if (!H.pass)
    Serial.println(F("#WARN IMU health check did not pass -- see the health "
                     "block above; this run is suspect"));
}

static inline void emitEvent(uint32_t t, const char *phase, int seq, int axis) {
  Serial.printf("E,%lu,%s,%d,%d,%d,%d,,,,,,,0\n",
                (unsigned long)(t - tRef), phase, seq, axis, posA, posB);
}

static inline void emitSample(uint32_t t, const char *phase, int seq, int axis,
                              const int16_t *a, const int16_t *g, uint8_t flags) {
  Serial.printf("S,%lu,%s,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%u\n",
                (unsigned long)(t - tRef), phase, seq, axis, posA, posB,
                a[0], a[1], a[2], g[0], g[1], g[2], flags);
}

// Log for ms milliseconds at hz, printing as we go.  Only used where
// the sample rate is low enough that printf fits inside the period;
// TEST B uses the RAM-buffered path instead.
void logWindow(const char *phase, int seq, int axis, uint32_t ms, int hz) {
  const uint32_t period = 1000000UL / hz;
  uint32_t t0 = micros(), next = t0;
  while ((micros() - t0) < ms * 1000UL) {
    uint8_t flags = 0;
    if ((int32_t)(micros() - next) > (int32_t)period) flags |= FLAG_LATE;
    while ((int32_t)(micros() - next) < 0) { }
    next += period;

    int16_t a[3] = {0, 0, 0}, g[3] = {0, 0, 0};
    if (!readAG(a, g)) flags |= FLAG_I2C_ERR;
    emitSample(micros(), phase, seq, axis, a, g, flags);
  }
}

// ================= TEST L : LIMIT SUGGESTION (advisory only) =========
/* You set the limits yourself (LIMIT_A_* / LIMIT_B_*, or 'w').  This
   test is an optional SECOND OPINION: it feels for the hard stops and
   PRINTS suggested numbers, but it never writes them.  Use it to
   sanity-check the values you measured by hand, not to replace them.

   Principle: below the stop, commanding +5 us moves the plate by
   roughly gain*5 degrees.  At the stop, the command still changes but
   the plate does not move at all.  So: creep outward in 5 us steps and
   watch the motion over a sliding 4-step window.  When the plate has
   moved less than 30% of what those 4 steps should have produced, the
   linkage is against something.  Back off immediately and subtract a
   40 us margin.

   The motion metric is the angle between the current gravity unit
   vector and the reference one.  That is axis-agnostic: it does not
   care which way the rig is lying or which IMU axis is which, so it
   cannot be defeated by a wrong AXIS_MAIN.

   Time spent against the stop is bounded by STALL_WIN * LIMIT_DWELL_MS
   = 600 ms, once per direction, and the servo is commanded away the
   instant detection fires. */

bool accUnit(int n, float *u) {
  double s[3] = {0, 0, 0};
  int ok = 0;
  for (int i = 0; i < n; i++) {
    int16_t a[3], g[3];
    if (readAG(a, g)) { for (int k = 0; k < 3; k++) s[k] += a[k]; ok++; }
    delay(2);
  }
  if (!ok) return false;
  double m = sqrt(s[0]*s[0] + s[1]*s[1] + s[2]*s[2]);
  if (m < 1e-6) return false;
  for (int k = 0; k < 3; k++) u[k] = (float)(s[k] / m);
  return true;
}

float angBetween(const float *u, const float *v) {
  float d = u[0]*v[0] + u[1]*v[1] + u[2]*v[2];
  d = constrain(d, -1.0f, 1.0f);
  return acosf(d) * 57.29578f;
}

// Returns the last pulse that definitely still produced motion.
int findStop(int axis, int center, int dir, float gain, const float *uCenter) {
  float hist[64];
  int   pulse[64];
  int   n = 0;
  int   lastGood = center;

  float u[3];
  if (!accUnit(30, u)) return center;
  hist[n] = 0.0f; pulse[n] = center; n++;

  for (int d = LIMIT_STEP; d <= LIMIT_MAX_TRAVEL && n < 63; d += LIMIT_STEP) {
    int us = center + dir * d;
    if (us <= PULSE_MIN + 10 || us >= PULSE_MAX - 10) {
      Serial.printf("#   hit driver rail at %d us\n", us);
      lastGood = us; break;
    }
    uint32_t t = rawWrite(axis, us);
    emitEvent(t, "lsearch", d, axis);
    delay(LIMIT_DWELL_MS);
    logWindow("lsearch", d, axis, 60, 100);

    if (!accUnit(20, u)) continue;
    hist[n] = angBetween(uCenter, u);
    pulse[n] = us;
    n++;

    if (n > STALL_WIN) {
      float moved = fabsf(hist[n-1] - hist[n-1-STALL_WIN]);
      float expect = fabsf(gain) * STALL_WIN * LIMIT_STEP;
      Serial.printf("#   %d us  ang %6.3f deg  moved %6.4f / %6.4f expected\n",
                    us, hist[n-1], moved, expect);
      if (expect > 1e-4 && moved < STALL_FRAC * expect) {
        // The last pulse we can PROVE moved the plate is the one at the
        // start of the stalled window, not the one we just tried.
        lastGood = pulse[n-1-STALL_WIN];
        Serial.printf("#   STOP detected at %d us -> last good %d us\n",
                      us, lastGood);
        rawWrite(axis, center);          // off the stop immediately
        delay(600);
        return lastGood;
      }
      lastGood = pulse[n-1];
    }
  }
  Serial.println(F("#   search cap reached without a stop (travel is "
                   "larger than the cap, or the gain estimate is off)"));
  rawWrite(axis, center);
  delay(600);
  return lastGood;
}

void testL(int axis) {
  emitMeta("L", axis, "limit_search");
  Serial.printf("#META limit_step=%d dwell_ms=%d margin=%d max_travel=%d "
                "stall_win=%d stall_frac=%.2f\n",
                LIMIT_STEP, LIMIT_DWELL_MS, LIMIT_MARGIN, LIMIT_MAX_TRAVEL,
                STALL_WIN, STALL_FRAC);
  emitCols();

  Serial.println(F("# TEST L: finding mechanical travel limits."));
  Serial.println(F("# WATCH AND LISTEN. Any buzzing that does not stop "
                   "within ~1 s: cut power."));

  int center = PULSE_NEUTRAL;
  uint32_t t = rawWrite(axis, center);
  emitEvent(t, "lcenter", -1, axis);
  delay(1200);

  float uC[3];
  if (!accUnit(60, uC)) { Serial.println(F("!! IMU read failed")); return; }

  // --- probe the gain over a deliberately small, safe excursion ---
  const int probe = 30;
  rawWrite(axis, center + probe); delay(700);
  float uP[3];
  if (!accUnit(40, uP)) { Serial.println(F("!! IMU read failed")); return; }
  float dAng = angBetween(uC, uP);
  rawWrite(axis, center); delay(700);

  float gain = dAng / probe;                     // deg per us, magnitude
  Serial.printf("# probe: %d us -> %.3f deg  => gain %.5f deg/us\n",
                probe, dAng, gain);
  if (gain < 0.002f) {
    Serial.println(F("!! plate barely moved over the probe."));
    Serial.println(F("!! either the servo is not driving the linkage, or "
                     "the sweep axis is VERTICAL (accel cannot see it)."));
    Serial.println(F("!! ABORTED -- fix the rig before searching for stops."));
    return;
  }

  Serial.println(F("# searching UP..."));
  int hi = findStop(axis, center, +1, gain, uC);
  Serial.println(F("# searching DOWN..."));
  int lo = findStop(axis, center, -1, gain, uC);

  int usableLo = lo + LIMIT_MARGIN;
  int usableHi = hi - LIMIT_MARGIN;

  // Advisory ONLY.  Nothing is written to lim[] or NVS.  If you want
  // these numbers to take effect, type them yourself with 'w', or put
  // them in LIMIT_A_*/LIMIT_B_* and re-flash -- that keeps you in the
  // loop, which is the whole point.
  rawWrite(axis, PULSE_NEUTRAL); delay(300);
  Serial.printf("#META limit_suggestion axis=%d stop_lo=%d stop_hi=%d "
                "suggest_lo=%d suggest_hi=%d margin=%d gain=%.5f\n",
                axis, lo, hi, usableLo, usableHi, LIMIT_MARGIN, gain);
  Serial.printf("# AXIS %d SUGGESTED: stops felt at %d / %d us.\n",
                axis, lo, hi);
  Serial.printf("# with %d us margin -> usable %d..%d us "
                "(%d us, ~%.1f deg)\n",
                LIMIT_MARGIN, usableLo, usableHi, usableHi - usableLo,
                (usableHi - usableLo) * gain);
  Serial.printf("# to APPLY these, type:  w %d %d\n", usableLo, usableHi);
  Serial.println(F("# (they are only suggestions; nothing was saved)"));
  Serial.println(F("# TEST_L_END"));
}

// ================= TEST A : MAPPING =================
/* Four passes: up, down, up, down.  Two full cycles separate one-off
   settling from real hysteresis, and the repeated visits to the same
   pulse give the backlash estimate a second opinion.

   Every sample of every dwell is emitted, from the command instant
   onward -- not a mean.  The PC picks the settled sub-window, and the
   scatter within it IS the repeatability number. */
void sweep(int axis, int from, int to, int step, const char *tag) {
  uint32_t t = cmdWrite(axis, from);
  emitEvent(t, tag, -1, axis);
  delay(1500);

  int seq = 0;
  for (int us = from; (step > 0) ? (us <= to) : (us >= to); us += step) {
    t = cmdWrite(axis, us);
    emitEvent(t, tag, seq, axis);
    logWindow(tag, seq, axis, A_DWELL_MS, A_LOG_HZ);
    seq++;
  }
}

void testA(int axis) {
  if (!haveLimits(axis)) return;
  const int aStart = axLo(axis), aEnd = axHi(axis);
  if (!rangeOK(axis, aStart, aEnd, "TEST A sweep")) return;

  emitMeta("A", axis, "sweep=up,dn,up2,dn2");
  Serial.printf("#META a_start=%d a_end=%d a_step=%d dwell_ms=%d log_hz=%d "
                "limits_from=measured\n",
                aStart, aEnd, A_STEP, A_DWELL_MS, A_LOG_HZ);
  emitCols();

  uint32_t t = cmdWrite(axis, axCenter(axis));
  emitEvent(t, "zero", -1, axis);
  delay(A_ZERO_MS);
  logWindow("zero", -1, axis, 1000, A_LOG_HZ);

  sweep(axis, aStart, aEnd,    A_STEP, "up");
  sweep(axis, aEnd,   aStart, -A_STEP, "dn");
  sweep(axis, aStart, aEnd,    A_STEP, "up2");
  sweep(axis, aEnd,   aStart, -A_STEP, "dn2");

  t = cmdWrite(axis, axCenter(axis));
  emitEvent(t, "zero", -2, axis);
  delay(A_ZERO_MS);
  logWindow("zero", -2, axis, 1000, A_LOG_HZ);   // closing zero: drift check

  Serial.printf("#META i2c_err_at_end=%lu\n", (unsigned long)i2cErrors);
  Serial.println(F("# TEST_A_END"));
}

// ================= TEST B : STEP =================
/* 1 kHz into RAM.  Printing inside the loop would blow the deadline, so
   nothing is emitted until the burst is over.

   The buffer holds the pre-step baseline AND a 2 s settled tail in one
   trace.  That is deliberate: fitting drift to the 200 ms pre-window
   alone gives a slope standard error ~20x the drift itself, which
   integrates into a degree-scale fake ramp.  Both ends together give a
   1.1 s+ lever arm and a usable fit. */
uint32_t tbuf[B_LOG_N];
int16_t  abuf[B_LOG_N][3];
int16_t  gbuf[B_LOG_N][3];
uint8_t  fbuf[B_LOG_N];

void oneStep(int axis, int fromUs, int toUs, int seq) {
  uint32_t t = cmdWrite(axis, fromUs);
  emitEvent(t, "arm", seq, axis);
  delay(B_ARM_MS);

  const uint32_t period = 1000000UL / B_LOG_HZ;
  uint32_t t0 = micros(), next = t0;
  uint32_t tCmd = 0;
  bool fired = false;
  int nPre = 0;

  for (int i = 0; i < B_LOG_N; i++) {
    uint8_t flags = 0;
    if ((int32_t)(micros() - next) > (int32_t)period) flags |= FLAG_LATE;
    while ((int32_t)(micros() - next) < 0) { }
    next += period;

    if (!fired && (micros() - t0) >= (uint32_t)B_PRE_MS * 1000UL) {
      tCmd = cmdWrite(axis, toUs);            // <-- the step
      fired = true;
      nPre = i;
    }

    tbuf[i] = micros();
    if (!readAG(abuf[i], gbuf[i])) flags |= FLAG_I2C_ERR;
    fbuf[i] = flags;
  }

  // Dump.  tRef is moved to the burst origin so t_us is relative to the
  // trace, and the command event carries its true measured instant.
  uint32_t saveRef = tRef;
  tRef = t0;
  emitEvent(tCmd, "cmd", seq, axis);
  Serial.printf("#META step seq=%d from_us=%d to_us=%d amp_us=%d "
                "t_cmd_us=%lu n_pre=%d\n",
                seq, fromUs, toUs, toUs - fromUs,
                (unsigned long)(tCmd - t0), nPre);
  for (int i = 0; i < B_LOG_N; i++) {
    const char *ph = (tbuf[i] < tCmd) ? "pre" : "post";
    Serial.printf("S,%lu,%s,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%u\n",
                  (unsigned long)(tbuf[i] - t0), ph, seq, axis, posA, posB,
                  abuf[i][0], abuf[i][1], abuf[i][2],
                  gbuf[i][0], gbuf[i][1], gbuf[i][2], fbuf[i]);
  }
  tRef = saveRef;

  cmdWrite(axis, axCenter(axis));
  delay(1200);
}

void testB(int axis) {
  if (!haveLimits(axis)) return;

  // Amplitudes as fractions of half-travel about the MECHANICAL centre,
  // which is not necessarily 1520 if the travel is asymmetric.  Sizing
  // from the measured half-travel is what keeps the 100% step inside
  // the stops instead of slamming into one.
  const int N = axCenter(axis);
  const int half = axHalf(axis);
  const float frac[] = {0.05f, 0.10f, 0.25f, 0.50f, 1.00f};

  if (!rangeOK(axis, N - half, N + half, "TEST B steps")) return;
  if (half < 30) {
    Serial.printf("!! half-travel is only %d us -- steps would be smaller "
                  "than the deadband. ABORTED.\n", half);
    return;
  }

  emitMeta("B", axis, "buffered=1");
  Serial.printf("#META log_hz=%d pre_ms=%d post_ms=%d n=%d arm_ms=%d "
                "center_us=%d half_travel_us=%d\n",
                B_LOG_HZ, B_PRE_MS, B_POST_MS, B_LOG_N, B_ARM_MS, N, half);
  emitCols();

  int seq = 0;
  for (int i = 0; i < 5; i++) {
    int amp = (int)(frac[i] * half);
    if (amp < 5) amp = 5;                     // deadband is 2 us; stay above
    oneStep(axis, N, N + amp, seq++);
    oneStep(axis, N, N - amp, seq++);
  }

  Serial.printf("#META i2c_err_at_end=%lu\n", (unsigned long)i2cErrors);
  Serial.println(F("# TEST_B_END"));
}

// ================= TEST K : DEADBAND / RESOLUTION =================
/* 1 us increments across a center point.  The spec says 2 us deadband;
   this measures what it actually is with the linkage and load attached,
   which is the real resolution floor of the whole actuator.

   Run at three centers: the response is not the same at neutral as it
   is near the travel limits where the linkage geometry is worst. */
void deadbandAt(int axis, int center, int seq0) {
  int seq = seq0;
  uint32_t t = cmdWrite(axis, center - K_SPAN);
  emitEvent(t, "hold", seq, axis);
  delay(1200);

  for (int us = center - K_SPAN; us <= center + K_SPAN; us += 1) {
    t = cmdWrite(axis, us);
    emitEvent(t, "kup", seq, axis);
    logWindow("kup", seq, axis, K_DWELL_MS, K_LOG_HZ);
    seq++;
  }
  for (int us = center + K_SPAN; us >= center - K_SPAN; us -= 1) {
    t = cmdWrite(axis, us);
    emitEvent(t, "kdn", seq, axis);
    logWindow("kdn", seq, axis, K_DWELL_MS, K_LOG_HZ);
    seq++;
  }
}

void testK(int axis) {
  if (!haveLimits(axis)) return;

  // Centres pulled in by a full K_SPAN plus slack, so the +/-K_SPAN
  // excursion around the outermost centres still lands inside travel.
  const int centers[3] = {axCenter(axis),
                          axLo(axis) + K_SPAN + 10,
                          axHi(axis) - K_SPAN - 10};
  if (!rangeOK(axis, centers[1] - K_SPAN, centers[2] + K_SPAN,
               "TEST K deadband")) return;
  if (centers[1] >= centers[2]) {
    Serial.println(F("!! travel too narrow for three deadband centres. "
                     "ABORTED."));
    return;
  }

  emitMeta("K", axis, "deadband");
  Serial.printf("#META k_span=%d dwell_ms=%d log_hz=%d\n",
                K_SPAN, K_DWELL_MS, K_LOG_HZ);
  emitCols();

  for (int i = 0; i < 3; i++) {
    Serial.printf("#META deadband_center=%d idx=%d\n", centers[i], i);
    deadbandAt(axis, centers[i], i * 1000);
  }

  cmdWrite(axis, axCenter(axis));
  Serial.printf("#META i2c_err_at_end=%lu\n", (unsigned long)i2cErrors);
  Serial.println(F("# TEST_K_END"));
}

// ================= TEST P : REPEATABILITY / BACKLASH =================
/* Return to the SAME pulse from below and from above, 20 times each.
   The mean split between the two approach directions is backlash; the
   scatter within one direction is repeatability.  A sweep alone cannot
   separate those two -- it sees only their sum. */
void testP(int axis) {
  if (!haveLimits(axis)) return;

  // The run-up must fit inside the travel on BOTH sides of the
  // outermost target, so it is sized from the measured half-travel
  // rather than being a fixed 200 us that may not fit.
  const int C = axCenter(axis);
  const int approach = min(P_APPROACH, axHalf(axis) / 2);
  if (approach < 20) {
    Serial.println(F("!! travel too narrow for a meaningful approach "
                     "run-up. ABORTED."));
    return;
  }
  const int targets[3] = {C, C - approach / 2, C + approach / 2};
  if (!rangeOK(axis, targets[1] - approach, targets[2] + approach,
               "TEST P approaches")) return;

  emitMeta("P", axis, "repeatability");
  Serial.printf("#META reps=%d approach_us=%d dwell_ms=%d log_hz=%d "
                "center_us=%d\n",
                P_REPS, approach, P_DWELL_MS, P_LOG_HZ, C);
  emitCols();

  int seq = 0;
  for (int ti = 0; ti < 3; ti++) {
    int target = targets[ti];
    Serial.printf("#META repeat_target=%d idx=%d\n", target, ti);

    for (int r = 0; r < P_REPS; r++) {
      // from below
      cmdWrite(axis, target - approach); delay(700);
      uint32_t t = cmdWrite(axis, target);
      emitEvent(t, "frombelow", seq, axis);
      logWindow("frombelow", seq, axis, P_DWELL_MS, P_LOG_HZ);
      seq++;

      // from above
      cmdWrite(axis, target + approach); delay(700);
      t = cmdWrite(axis, target);
      emitEvent(t, "fromabove", seq, axis);
      logWindow("fromabove", seq, axis, P_DWELL_MS, P_LOG_HZ);
      seq++;
    }
  }

  cmdWrite(axis, axCenter(axis));
  Serial.printf("#META i2c_err_at_end=%lu\n", (unsigned long)i2cErrors);
  Serial.println(F("# TEST_P_END"));
}

// ================= IMU HEALTH =================
/* Pass criteria, from the bench procedure:
     all three gyro axes within +/-1 dps, gyro sd < 0.5 dps,
     accel sd < 0.01 g, |g| ~ 1.00, 12-byte read > 1000 Hz
   The bias is reported for information only.  It is NOT stored and NOT
   subtracted anywhere -- the analysis recovers it from each trace's own
   pre/tail windows, which also captures the drift that a startup
   calibration cannot see. */
bool imuHealth(bool quiet) {
  if (!quiet) Serial.println(F("# IMU HEALTH - keep the rig perfectly still"));
  delay(quiet ? 200 : 400);

  bank(0);
  H.alive = (rd(B0_WHO_AM_I) == 0xEA);
  if (!H.alive) {
    Serial.println(F("!! ICM-20948 did not answer WHO_AM_I=0xEA."));
    Serial.println(F("!! check I2C wiring, 3V3, and address (0x69 vs 0x68)."));
    H.pass = false;
    return false;
  }

  const int N = 2000;
  double sg[3] = {0, 0, 0}, sg2[3] = {0, 0, 0};
  double sa[3] = {0, 0, 0}, sa2[3] = {0, 0, 0};
  int n = 0, bad = 0;
  uint32_t t0 = micros();

  for (int i = 0; i < N; i++) {
    int16_t a[3], g[3];
    if (readAG(a, g)) {
      for (int k = 0; k < 3; k++) {
        double gv = g[k] / GYRO_LSB_PER_DPS, av = a[k] / ACC_LSB_PER_G;
        sg[k] += gv; sg2[k] += gv * gv;
        sa[k] += av; sa2[k] += av * av;
      }
      n++;
    } else bad++;
  }
  uint32_t dt = micros() - t0;
  H.rate_hz = N * 1e6f / dt;
  H.fails = bad;

  if (!n) {
    Serial.println(F("!! no valid reads - check wiring/address"));
    H.pass = false;
    return false;
  }

  bool ok = true;
  double gmag = 0;
  for (int k = 0; k < 3; k++) {
    double gm = sg[k] / n, gsd = sqrt(sg2[k] / n - gm * gm);
    double am = sa[k] / n, asd = sqrt(sa2[k] / n - am * am);
    H.bias[k] = gm; H.gnoise[k] = gsd; H.anoise[k] = asd;
    gmag += am * am;
    bool gok = (fabs(gm) < 1.0 && gsd < 0.5), aok = (asd < 0.01);
    ok &= gok && aok;
    Serial.printf("# axis %d  gyro bias %+7.3f dps  sd %5.3f %s   |   "
                  "accel %+7.4f g  sd %6.4f %s\n",
                  k, gm, gsd, gok ? "OK " : "BAD", am, asd, aok ? "OK " : "BAD");
  }
  gmag = sqrt(gmag);
  H.gmag = gmag;
  bool magok = fabs(gmag - 1.0) < 0.03;
  bool rateok = H.rate_hz > 1000;
  ok &= magok && rateok && (bad == 0);

  Serial.printf("# |g| = %.4f %s   throughput %.0f Hz %s   i2c failures %d\n",
                gmag, magok ? "OK" : "BAD",
                H.rate_hz, rateok ? "OK" : "BAD (need >1000)", bad);
  Serial.printf("# IMU %s\n", ok ? "PASS" : "*** FAIL ***");
  if (!ok) {
    Serial.println(F("!! high gyro sd or |g| off 1.00 usually means the IMU "
                     "is not rigidly attached, or the rig is moving."));
    Serial.println(F("!! run 't' (tap test) to check mount rigidity."));
  }
  H.pass = ok;
  if (!quiet) Serial.println(F("# HEALTH_END"));
  return ok;
}

/* Mount rigidity.  Tap the plate: a rigidly mounted IMU rings down
   within ~0.2 s.  A long tail means a soft mount (thick hot glue, a
   loose screw, a flexing bracket), which puts a resonance right where
   the step response is being measured and will corrupt Test B. */
void tapTest() {
  Serial.println(F("# TAP TEST - tap the motor plate once, sharply."));
  Serial.println(F("# rigid mount: decays within ~0.2 s."));
  uint32_t t0 = millis();
  float peak = 0; uint32_t tPeak = 0; bool armed = false;

  while (millis() - t0 < 8000) {
    int16_t a[3], g[3];
    if (!readAG(a, g)) continue;
    float m = 0;
    for (int k = 0; k < 3; k++) {
      float v = g[k] / GYRO_LSB_PER_DPS;
      m += v * v;
    }
    m = sqrtf(m);
    if (!armed && m > 20.0f) { armed = true; peak = m; tPeak = millis();
                               Serial.println(F("# tap detected...")); }
    if (armed) {
      if (m > peak) { peak = m; tPeak = millis(); }
      if (m < 0.05f * peak && millis() - tPeak > 20) {
        uint32_t decay = millis() - tPeak;
        Serial.printf("# peak %.1f dps, decayed to 5%% in %lu ms  %s\n",
                      peak, (unsigned long)decay,
                      decay < 200 ? "OK - rigid" : "!! SOFT MOUNT");
        Serial.println(F("# TAP_END"));
        return;
      }
    }
  }
  Serial.println(F("# no tap detected (or it never decayed)"));
  Serial.println(F("# TAP_END"));
}

// ================= LIVE READOUT =================
/* For checking rig orientation before a run.  The sweep axis must be
   HORIZONTAL: jog the servo and confirm the tilt number actually moves.
   If it barely changes, the axis is vertical and the mapping test will
   produce nonsense. */
void liveReadout() {
  Serial.println(F("# live readout, 20 s. Jog by hand / check axes."));
  uint32_t t = millis();
  while (millis() - t < 20000) {
    int16_t a[3], g[3];
    if (readAG(a, g)) {
      float v[3] = {a[0] / ACC_LSB_PER_G, a[1] / ACC_LSB_PER_G, a[2] / ACC_LSB_PER_G};
      Serial.printf("acc %+6.3f %+6.3f %+6.3f g | gyro %+7.2f %+7.2f %+7.2f dps"
                    " | tiltXZ %+7.2f  tiltYZ %+7.2f deg\n",
                    v[0], v[1], v[2],
                    g[0] / GYRO_LSB_PER_DPS, g[1] / GYRO_LSB_PER_DPS,
                    g[2] / GYRO_LSB_PER_DPS,
                    atan2f(v[0], v[2]) * 57.29578f,
                    atan2f(v[1], v[2]) * 57.29578f);
    }
    delay(100);
    if (Serial.available()) { Serial.read(); break; }
  }
  Serial.println(F("# LIVE_END"));
}

// ================= COMMAND SHELL =================
void status() {
  Serial.printf("# sel=%c posA=%d posB=%d step=%d hz=%d %s i2c_err=%lu "
                "clamps=%lu\n",
                sel ? 'B' : 'A', posA, posB, jogStep, SERVO_HZ,
                attached ? "attached" : "DETACHED",
                (unsigned long)i2cErrors, (unsigned long)clampCount);
  for (int a = 0; a < 2; a++)
    Serial.printf("#   axis %d travel [%d,%d] %s  centre %d  half %d  "
                  "gain %.5f deg/us\n",
                  a, lim[a].lo, lim[a].hi,
                  lim[a].valid ? "MEASURED" : "*** NOT CALIBRATED ***",
                  axCenter(a), axHalf(a), lim[a].gain_deg_per_us);
}

void help() {
  Serial.println(F("# jog: <us> | + - ++ -- | s <us> | n | x <0|1> | m"));
  Serial.println(F("#      w <lo> <hi> set limits manually | W wipe limits"));
  Serial.println(F("#      u detach | v attach | ? status | h help"));
  Serial.println(F("# test: l=TRAVEL LIMITS (run first)  c=imu health  z=live"));
  Serial.println(F("#       a=mapping  b=step  k=deadband  p=repeatability"));
}

void jog(int us, const char *how) {
  if (!attached) { Serial.println(F("!! detached - press 'v' first")); return; }
  // Jogging is how you sanity-check a rig, so it is allowed to move
  // freely inside the measured travel -- but not outside it.  Before
  // calibration that means the deliberately tiny SAFE window.
  int want = clampWin(sel, us);
  if (want != us && !lim[sel].valid)
    Serial.println(F("!! axis not calibrated: jog is restricted to a narrow "
                     "safe window until you run 'l'"));
  cmdWrite(sel, want);
  Serial.printf("%s  servo%c -> %d us%s\n", how, sel ? 'B' : 'A', want,
                (want != us) ? "   [CLAMPED]" : "");
}

int selPos() { return sel ? posB : posA; }

void handle(String line) {
  line.trim();
  if (!line.length()) return;
  char c0 = line.charAt(0);

  if (isDigit(c0)) { jog(line.toInt(), "abs"); return; }
  if (line == "+")  { jog(selPos() + jogStep,     "step +"); return; }
  if (line == "-")  { jog(selPos() - jogStep,     "step -"); return; }
  if (line == "++") { jog(selPos() + jogStep * 5, "jump +"); return; }
  if (line == "--") { jog(selPos() - jogStep * 5, "jump -"); return; }
  if (line == "n")  { cmdWrite(0, axCenter(0)); cmdWrite(1, axCenter(1));
                      Serial.println(F("# both to centre")); return; }

  if (c0 == 's' && line.length() > 1) {
    int v = line.substring(1).toInt();
    if (v >= 1 && v <= 200) { jogStep = v; Serial.printf("# step = %d us\n", v); }
    else Serial.println(F("!! step must be 1..200"));
    return;
  }

  if (c0 == 'x' && line.length() > 1) {
    sel = line.substring(1).toInt() ? 1 : 0;
    Serial.printf("# selected servo %c\n", sel ? 'B' : 'A');
    return;
  }

  // Manual limit entry, for when you have found the stops by hand and
  // would rather not let the automatic search touch them again.  These
  // are the SAME numbers TEST L writes, so enter the USABLE range with
  // your margin already subtracted.
  if (c0 == 'w' && line.length() > 1) {
    int sp = line.indexOf(' ');
    int sp2 = (sp > 0) ? line.indexOf(' ', sp + 1) : -1;
    if (sp2 < 0) { Serial.println(F("!! usage: w <lo> <hi>  (usable range, "
                                    "margin already subtracted)")); return; }
    int lo = line.substring(sp + 1, sp2).toInt();
    int hi = line.substring(sp2 + 1).toInt();
    if (lo < PULSE_MIN || hi > PULSE_MAX || lo >= hi || hi - lo < 60) {
      Serial.printf("!! need %d <= lo < hi <= %d and at least 60 us of span\n",
                    PULSE_MIN, PULSE_MAX);
      return;
    }
    lim[sel].lo = lo; lim[sel].hi = hi; lim[sel].valid = true;
    saveLimits();
    Serial.printf("# axis %d travel set MANUALLY to [%d,%d]\n", sel, lo, hi);
    Serial.println(F("# these were not measured -- if they are wrong, the "
                     "servo will stall against a stop for the whole run"));
    return;
  }

  if (line == "W") {
    for (int a = 0; a < 2; a++) lim[a] = {SAFE_START, SAFE_END, false, 0.0f};
    saveLimits();
    Serial.println(F("# limits wiped; back to the narrow safe window"));
    return;
  }

  if (line == "m") {
    int p = selPos();
    Serial.printf("LIMIT,servo%c,%d,%+d,margin%d=%d\n", sel ? 'B' : 'A',
                  p, p - PULSE_NEUTRAL, LIMIT_MARGIN,
                  (p > PULSE_NEUTRAL) ? p - LIMIT_MARGIN : p + LIMIT_MARGIN);
    return;
  }

  if (line == "u") { servoA.detach(); servoB.detach(); attached = false;
                     Serial.println(F("# DETACHED - servos limp")); return; }
  if (line == "v") { attachServos(); Serial.println(F("# attached")); return; }
  if (line == "?") { status(); return; }
  if (line == "h") { help();   return; }

  // tests -- tRef is reset so every trace starts at t_us = 0
  if (line == "l") { tRef = micros(); testL(sel); return; }
  if (line == "c") { imuHealth(false); return; }
  if (line == "t") { tapTest();  return; }
  if (line == "z") { liveReadout(); return; }
  if (line == "a") { tRef = micros(); testA(sel); return; }
  if (line == "b") { tRef = micros(); testB(sel); return; }
  if (line == "k") { tRef = micros(); testK(sel); return; }
  if (line == "p") { tRef = micros(); testP(sel); return; }

  Serial.printf("!! unknown: %s\n", line.c_str());
  help();
}

void setup() {
  Serial.begin(460800);
  delay(2000);

  Wire.begin(21, 22);
  Wire.setClock(400000);

  bool imuOK = imuInit();
  if (!imuOK)
    Serial.println(F("!! ICM-20948 not found. Check addr (0x69 vs 0x68) "
                     "and I2C wiring."));

  // Servos start at the safe centre.  If a preset/NVS limit exists it is
  // loaded first, so 'centre' is the centre of the real travel.
#if !LIMITS_PRESET
  loadLimits();                    // preset in source wins over NVS
#endif
  posA = axCenter(0);
  posB = axCenter(1);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  attachServos();
  delay(1500);

  Serial.printf("# %s ready. step buffer %d samples (%d ms @ %d Hz)\n",
                FW_VERSION, B_LOG_N, B_PRE_MS + B_POST_MS, B_LOG_HZ);

  // Automatic IMU health at boot: alive? attached? noise floor? -- so a
  // bad sensor is caught before any test, not discovered in analysis.
  Serial.println(F("# --- automatic IMU health check (hold the rig still) ---"));
  imuHealth(false);
  if (imuOK && H.pass)
    Serial.println(F("# IMU OK. Recommend 't' (tap test) once to confirm "
                     "mount rigidity."));

  for (int a = 0; a < 2; a++)
    if (!lim[a].valid)
      Serial.printf("# NOTE axis %d has no travel limits: set LIMIT_%c_LO/HI "
                    "+ LIMITS_PRESET, or type 'w', before running a test.\n",
                    a, a ? 'B' : 'A');

  help();
  status();
}

void loop() {
  static String buf;
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') { handle(buf); buf = ""; }
    else if (buf.length() < 40)   { buf += ch; }
  }
}
