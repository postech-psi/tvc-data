/*
 * TVC automatic system-identification firmware
 * Hardware: ESP32-WROOM-32D, one rear-plate ICM-20948, two PWM servos.
 *
 * The firmware only performs deterministic motion and emits raw samples.
 * Calibration, fitting, acceptance decisions and reports are done on the PC.
 */
#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>

// ============================================================================
// USER TEST PARAMETERS
// Edit the experiment and hardware settings only in this block.
// Each PC session saves a copy of this file for traceability.
// ============================================================================

// Serial / hardware. Binary records need less than half of this UART capacity.
constexpr uint32_t SERIAL_BAUD = 921600;
constexpr size_t SERIAL_TX_BUFFER_BYTES = 4096;
constexpr int OUTER_GIMBAL_AXIS = 0;  // Servo A, fixed outer frame
constexpr int INNER_GIMBAL_AXIS = 1;  // Servo B, moving inner frame
constexpr int OUTER_GIMBAL_PIN = 18;
constexpr int INNER_GIMBAL_PIN = 19;
constexpr int SERVO_HZ = 333;
constexpr int PULSE_MIN_US = 500;   // Servo library driver range, not test limit
constexpr int PULSE_MAX_US = 2500;  // Servo library driver range, not test limit
constexpr int NEUTRAL_US = 1520;
constexpr int OUTER_GIMBAL_LIMIT_LO_US = 1370;
constexpr int OUTER_GIMBAL_LIMIT_HI_US = 1690;
constexpr int INNER_GIMBAL_LIMIT_LO_US = 1350;
constexpr int INNER_GIMBAL_LIMIT_HI_US = 2040;

// One rear-plate Adafruit ICM-20948
constexpr int I2C_SDA_PIN = 21;
constexpr int I2C_SCL_PIN = 22;
constexpr uint32_t I2C_CLOCK_HZ = 200000;
constexpr uint8_t IMU_ADDR = 0x69;
constexpr uint8_t GYRO_DLPF_CONFIG = 7;
constexpr uint8_t GYRO_FULL_SCALE_SETTING = 3;   // 3 = +/-2000 dps
constexpr uint8_t ACCEL_DLPF_CONFIG = 7;
constexpr uint8_t ACCEL_FULL_SCALE_SETTING = 0; // 0 = +/-2 g
constexpr float ACC_LSB_PER_G = 16384.0f;        // Must match accel FS above
constexpr float GYRO_LSB_PER_DPS = 16.4f;        // Must match gyro FS above
constexpr uint8_t GYRO_SAMPLE_RATE_DIV = 0;
constexpr uint16_t ACCEL_SAMPLE_RATE_DIV = 0;
constexpr int SENSOR_INTERNAL_ODR_HZ = 1125;     // ICM nominal ODR with div=0

// ESP32 acquisition, serial output and PC storage are all exactly 1,000 Hz.
// The ICM-20948 cannot generate exactly 1,000 Hz internally, so the ESP32
// reads the latest value every 1 ms from its 1,125 Hz register stream.
constexpr int DATA_RATE_HZ = 1000;
constexpr uint32_t DATA_PERIOD_US = 1000000UL / DATA_RATE_HZ;

// Experiment 1: static health check
constexpr int HEALTH_MS = 4000;
constexpr float RAW_GYRO_BIAS_MAX_DPS = 5.0f;
constexpr float RAW_GYRO_NOISE_MAX_DPS = 0.5f;
constexpr float RAW_ACCEL_NOISE_MAX_G = 0.01f;
constexpr float RAW_GRAVITY_MAG_TOLERANCE_G = 0.08f;

// Experiment 2: static PWM-to-angle mapping
constexpr int MAP_STEP_US = 10;
constexpr int MAP_DWELL_MS = 1000;
constexpr int MAP_ZERO_MS = 1200;
constexpr int MAP_NEUTRAL_SETTLE_MS = 500;

// Experiment 3: step response (both directions for every percentage)
constexpr int STEP_PRE_MS = 200;
constexpr int STEP_POST_MS = 2000;
constexpr int STEP_ARM_MS = 1600;
constexpr int STEP_RETURN_SETTLE_MS = 500;
constexpr int STEP_MIN_AMPLITUDE_US = 4;
constexpr int STEP_PERCENT[] = {5, 10, 25, 50, 90};
constexpr size_t STEP_PERCENT_COUNT = sizeof(STEP_PERCENT) / sizeof(STEP_PERCENT[0]);
constexpr int STEP_SAMPLES =
    (STEP_PRE_MS + STEP_POST_MS) * DATA_RATE_HZ / 1000;

// Experiment 4: chirp (frequency sweep) — optional, opened by recommend_chirp.
// Small-amplitude log sweep around neutral; input PWM is logged per sample.
constexpr float CHIRP_F0_HZ = 0.5f;
constexpr float CHIRP_F1_HZ = 25.0f;
constexpr int CHIRP_SWEEP_MS = 20000;
constexpr int CHIRP_PRE_MS = 300;
constexpr int CHIRP_SETTLE_MS = 500;
constexpr int CHIRP_AMPLITUDE_PCT = 15;   // % of the smaller travel side
constexpr int CHIRP_MIN_AMP_US = 8;

// Experiment 5: deadband/backlash — optional, opened by recommend_deadband_test.
// Fine ascending/descending staircase across neutral; backlash = hysteresis-loop
// width at the mid angle.
constexpr int DB_RANGE_US = 60;    // +/- around neutral
constexpr int DB_STEP_US = 3;
constexpr int DB_DWELL_MS = 400;
constexpr int DB_ZERO_MS = 800;
constexpr int DB_NEUTRAL_SETTLE_MS = 500;

// Experiment 6: joint 2D PWM_A x PWM_B -> angle grid map — optional.
// Both servos are commanded together over a grid so the coupled tip pointing
// surface (and cross-axis coupling) can be measured, unlike the per-axis A test.
// Time-optimized: a serpentine (boustrophedon) path keeps every move to one grid
// step, so a short settle is enough. 11x11 @ these settings is ~1 min.
constexpr int GRID_POINTS_PER_AXIS = 11;   // cells per servo (N x N total)
constexpr int GRID_DWELL_MS = 300;         // logged window per cell
constexpr int GRID_SETTLE_MS = 150;        // settle after each one-step move
constexpr int GRID_ROW_SETTLE_MS = 250;    // extra settle when the outer servo steps
constexpr int GRID_NEUTRAL_SETTLE_MS = 500;
constexpr int GRID_ZERO_MS = 1000;         // neutral reference window

// Unmounted-bench smoke test: small, slow motion only. This never runs the
// mapping or step ranges and always returns to neutral before detaching.
constexpr int SMOKE_DELTA_US = 20;
constexpr int SMOKE_NEUTRAL_SETTLE_MS = 1000;
constexpr int SMOKE_RAMP_STEP_US = 1;
constexpr int SMOKE_RAMP_INTERVAL_MS = 50;
constexpr int SMOKE_ENDPOINT_HOLD_MS = 300;
constexpr int SMOKE_FAST_HOLD_MS = 700;

// ============================================================================
// INTERNAL IMPLEMENTATION CONSTANTS -- normally do not edit below this line.
// ============================================================================

constexpr char FW_VERSION[] = "tvc_sid_2.3_gimbal_grid";
constexpr char BINARY_FORMAT[] = "tvc_sid_gimbal_bin_v2";

constexpr uint8_t REG_BANK_SEL = 0x7F;
constexpr uint8_t B0_WHO_AM_I = 0x00;
constexpr uint8_t B0_USER_CTRL = 0x03;
constexpr uint8_t B0_PWR_MGMT_1 = 0x06;
constexpr uint8_t B0_PWR_MGMT_2 = 0x07;
constexpr uint8_t B0_ACCEL_XOUT = 0x2D;
constexpr uint8_t B2_GYRO_SMPLRT_DIV = 0x00;
constexpr uint8_t B2_GYRO_CONFIG_1 = 0x01;
constexpr uint8_t B2_ACCEL_SMPLRT_1 = 0x10;
constexpr uint8_t B2_ACCEL_SMPLRT_2 = 0x11;
constexpr uint8_t B2_ACCEL_CONFIG = 0x14;

constexpr uint8_t FLAG_I2C = 0x01;
constexpr uint8_t FLAG_LATE = 0x02;
constexpr uint8_t RECORD_SAMPLE = 0x01;
constexpr uint8_t RECORD_EVENT = 0x02;
constexpr uint8_t RECORD_MAGIC_0 = 0xA5;
constexpr uint8_t RECORD_MAGIC_1 = 0x5A;

enum PhaseCode : uint8_t {
  PHASE_HEALTH = 0,
  PHASE_ZERO,
  PHASE_UP,
  PHASE_DN,
  PHASE_UP2,
  PHASE_DN2,
  PHASE_PRE,
  PHASE_POST,
  PHASE_ARM,
  PHASE_CMD,
  PHASE_CHIRP,
  PHASE_GRID,
};

struct __attribute__((packed)) BinaryRecord {
  uint8_t magic[2];
  uint8_t type;
  uint32_t packetSeq;
  uint32_t tUs;
  uint8_t phase;
  int16_t seq;
  int8_t axis;
  uint16_t cmdOuter;
  uint16_t cmdInner;
  int16_t ax;
  int16_t ay;
  int16_t az;
  int16_t gx;
  int16_t gy;
  int16_t gz;
  uint8_t flags;
  int16_t jitterUs;
  uint16_t crc;
};

static_assert(sizeof(BinaryRecord) == 36, "binary protocol size changed");

Servo gimbalServos[2];
int gimbalUs[2] = {NEUTRAL_US, NEUTRAL_US};
const int gimbalLo[2] = {OUTER_GIMBAL_LIMIT_LO_US, INNER_GIMBAL_LIMIT_LO_US};
const int gimbalHi[2] = {OUTER_GIMBAL_LIMIT_HI_US, INNER_GIMBAL_LIMIT_HI_US};
bool gimbalsAttached = false;
bool imuReady = false;
bool healthPass = false;
uint32_t i2cErrors = 0;
uint32_t tRef = 0;
uint32_t packetSeq = 0;
uint32_t serialShortWrites = 0;

struct ImuRaw {
  int16_t a[3];
  int16_t g[3];
};

void writeReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  if (Wire.endTransmission() != 0) i2cErrors++;
}

uint8_t readReg(uint8_t reg) {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    i2cErrors++;
    return 0xFF;
  }
  if (Wire.requestFrom(IMU_ADDR, static_cast<uint8_t>(1)) != 1) {
    i2cErrors++;
    return 0xFF;
  }
  return Wire.read();
}

void selectBank(uint8_t bank) { writeReg(REG_BANK_SEL, bank << 4); }

int16_t readBe16() {
  const uint8_t hi = Wire.read();
  const uint8_t lo = Wire.read();
  return static_cast<int16_t>((static_cast<uint16_t>(hi) << 8) | lo);
}

bool readImu(ImuRaw &out) {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(B0_ACCEL_XOUT);
  if (Wire.endTransmission(false) != 0) {
    i2cErrors++;
    return false;
  }
  if (Wire.requestFrom(IMU_ADDR, static_cast<uint8_t>(12)) != 12) {
    i2cErrors++;
    return false;
  }
  for (int i = 0; i < 3; ++i) out.a[i] = readBe16();
  for (int i = 0; i < 3; ++i) out.g[i] = readBe16();
  return true;
}

bool initImu() {
  selectBank(0);
  if (readReg(B0_WHO_AM_I) != 0xEA) return false;
  writeReg(B0_PWR_MGMT_1, 0x80);
  delay(120);
  selectBank(0);
  writeReg(B0_PWR_MGMT_1, 0x01);
  delay(20);
  writeReg(B0_PWR_MGMT_2, 0x00);
  writeReg(B0_USER_CTRL, 0x00);

  selectBank(2);
  writeReg(B2_GYRO_CONFIG_1,
           (GYRO_DLPF_CONFIG << 3) | (GYRO_FULL_SCALE_SETTING << 1) | 1);
  writeReg(B2_GYRO_SMPLRT_DIV, GYRO_SAMPLE_RATE_DIV);
  writeReg(B2_ACCEL_CONFIG,
           (ACCEL_DLPF_CONFIG << 3) | (ACCEL_FULL_SCALE_SETTING << 1) | 1);
  writeReg(B2_ACCEL_SMPLRT_1, (ACCEL_SAMPLE_RATE_DIV >> 8) & 0x0F);
  writeReg(B2_ACCEL_SMPLRT_2, ACCEL_SAMPLE_RATE_DIV & 0xFF);
  selectBank(0);
  delay(50);
  return readReg(B0_WHO_AM_I) == 0xEA;
}

const char *gimbalName(int axis) {
  if (axis == OUTER_GIMBAL_AXIS) return "outer";
  if (axis == INNER_GIMBAL_AXIS) return "inner";
  return "none";
}

void attachGimbals() {
  if (gimbalsAttached) return;
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  gimbalServos[OUTER_GIMBAL_AXIS].setPeriodHertz(SERVO_HZ);
  gimbalServos[INNER_GIMBAL_AXIS].setPeriodHertz(SERVO_HZ);
  gimbalServos[OUTER_GIMBAL_AXIS].attach(
      OUTER_GIMBAL_PIN, PULSE_MIN_US, PULSE_MAX_US);
  gimbalServos[INNER_GIMBAL_AXIS].attach(
      INNER_GIMBAL_PIN, PULSE_MIN_US, PULSE_MAX_US);
  gimbalsAttached = true;
  for (int axis = 0; axis < 2; ++axis)
    gimbalServos[axis].writeMicroseconds(gimbalUs[axis]);
}

void detachGimbals() {
  if (!gimbalsAttached) return;
  gimbalServos[OUTER_GIMBAL_AXIS].detach();
  gimbalServos[INNER_GIMBAL_AXIS].detach();
  gimbalsAttached = false;
}

uint32_t writeGimbal(int axis, int pulse) {
  pulse = constrain(pulse, gimbalLo[axis], gimbalHi[axis]);
  gimbalUs[axis] = pulse;
  if (gimbalsAttached) gimbalServos[axis].writeMicroseconds(pulse);
  return micros();
}

void neutral() {
  writeGimbal(OUTER_GIMBAL_AXIS, NEUTRAL_US);
  writeGimbal(INNER_GIMBAL_AXIS, NEUTRAL_US);
}

uint16_t crc16Ccitt(const uint8_t *data, size_t length) {
  uint16_t crc = 0xFFFF;
  while (length--) {
    crc ^= static_cast<uint16_t>(*data++) << 8;
    for (int bit = 0; bit < 8; ++bit)
      crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                           : static_cast<uint16_t>(crc << 1);
  }
  return crc;
}

uint8_t phaseCode(const char *phase) {
  if (!strcmp(phase, "health")) return PHASE_HEALTH;
  if (!strcmp(phase, "zero")) return PHASE_ZERO;
  if (!strcmp(phase, "up")) return PHASE_UP;
  if (!strcmp(phase, "dn")) return PHASE_DN;
  if (!strcmp(phase, "up2")) return PHASE_UP2;
  if (!strcmp(phase, "dn2")) return PHASE_DN2;
  if (!strcmp(phase, "pre")) return PHASE_PRE;
  if (!strcmp(phase, "post")) return PHASE_POST;
  if (!strcmp(phase, "arm")) return PHASE_ARM;
  if (!strcmp(phase, "cmd")) return PHASE_CMD;
  if (!strcmp(phase, "chirp")) return PHASE_CHIRP;
  if (!strcmp(phase, "grid")) return PHASE_GRID;
  return PHASE_HEALTH;
}

void emitRecord(uint8_t type, uint32_t t, const char *phase, int seq, int axis,
                const ImuRaw *sample, uint8_t flags, int32_t jitterUs) {
  BinaryRecord record{};
  record.magic[0] = RECORD_MAGIC_0;
  record.magic[1] = RECORD_MAGIC_1;
  record.type = type;
  record.packetSeq = packetSeq++;
  record.tUs = t - tRef;
  record.phase = phaseCode(phase);
  record.seq = static_cast<int16_t>(seq);
  record.axis = static_cast<int8_t>(axis);
  record.cmdOuter = static_cast<uint16_t>(gimbalUs[OUTER_GIMBAL_AXIS]);
  record.cmdInner = static_cast<uint16_t>(gimbalUs[INNER_GIMBAL_AXIS]);
  if (sample) {
    record.ax = sample->a[0]; record.ay = sample->a[1]; record.az = sample->a[2];
    record.gx = sample->g[0]; record.gy = sample->g[1]; record.gz = sample->g[2];
  }
  record.flags = flags;
  record.jitterUs = static_cast<int16_t>(constrain(jitterUs, -32768, 32767));
  record.crc = crc16Ccitt(&record.type, sizeof(record) - 4);
  if (Serial.write(reinterpret_cast<const uint8_t *>(&record), sizeof(record)) !=
      sizeof(record))
    ++serialShortWrites;
}

void beginDataStream() {
  packetSeq = 0;
  serialShortWrites = 0;
  Serial.println(F("#COLS rec,t_us,phase,seq,axis,gimbal,cmd_outer,cmd_inner,"
                   "ax,ay,az,gx,gy,gz,flags,packet_seq,jitter_us"));
  Serial.printf("#BINARY_BEGIN format=%s record_bytes=%u stream_hz=%d\n",
                BINARY_FORMAT, static_cast<unsigned>(sizeof(BinaryRecord)),
                DATA_RATE_HZ);
  Serial.flush();
}

void endDataStream() {
  Serial.flush();
  Serial.printf("#BINARY_END packets=%lu serial_short_writes=%lu\n",
                static_cast<unsigned long>(packetSeq),
                static_cast<unsigned long>(serialShortWrites));
}

void emitEvent(uint32_t t, const char *phase, int seq, int axis) {
  emitRecord(RECORD_EVENT, t, phase, seq, axis, nullptr, 0, 0);
}

void emitSample(uint32_t t, const char *phase, int seq, int axis,
                const ImuRaw &s, uint8_t flags, int32_t jitterUs) {
  emitRecord(RECORD_SAMPLE, t, phase, seq, axis, &s, flags, jitterUs);
}

void emitMeta(const char *test, int axis, const char *extra = "") {
  selectBank(2);
  const uint8_t gyroCfg = readReg(B2_GYRO_CONFIG_1);
  const uint8_t accelCfg = readReg(B2_ACCEL_CONFIG);
  selectBank(0);
  Serial.printf("#META fw=%s test=%s axis=%d gimbal=%s gimbal_count=2 imu_count=1 "
                "imu_addr=0x%02X who_am_i=0x%02X servo_hz=%d "
                "neutral_us=%d pin_outer=%d pin_inner=%d\n",
                FW_VERSION, test, axis, gimbalName(axis), IMU_ADDR,
                readReg(B0_WHO_AM_I), SERVO_HZ, NEUTRAL_US,
                OUTER_GIMBAL_PIN, INNER_GIMBAL_PIN);
  Serial.printf("#META gyro_cfg1=0x%02X accel_cfg=0x%02X "
                "gyro_lsb_per_dps=%.4f acc_lsb_per_g=%.1f "
                "sensor_odr_hz=%d stream_hz=%d serial_baud=%lu "
                "health_pass=%d i2c_err_start=%lu %s\n",
                gyroCfg, accelCfg, GYRO_LSB_PER_DPS, ACC_LSB_PER_G,
                SENSOR_INTERNAL_ODR_HZ, DATA_RATE_HZ,
                static_cast<unsigned long>(SERIAL_BAUD), healthPass,
                static_cast<unsigned long>(i2cErrors), extra);
  if (axis >= 0 && axis < 2) {
    Serial.printf("#META limit_lo_us=%d limit_hi_us=%d center_us=%d "
                  "negative_travel_us=%d positive_travel_us=%d\n",
                  gimbalLo[axis], gimbalHi[axis], NEUTRAL_US,
                  NEUTRAL_US - gimbalLo[axis],
                  gimbalHi[axis] - NEUTRAL_US);
  }
}

bool emergencyRequested() {
  if (!Serial.available()) return false;
  const int c = Serial.peek();
  if (c != '!') return false;
  while (Serial.available()) Serial.read();
  neutral();
  detachGimbals();
  Serial.println(F("#META firmware_test_pass=0 abort=emergency_stop"));
  return true;
}

bool interruptibleWait(uint32_t durationMs) {
  const uint32_t deadline = millis() + durationMs;
  while (static_cast<int32_t>(deadline - millis()) > 0) {
    if (emergencyRequested()) return false;
    delay(10);
  }
  return true;
}

bool smokeRamp(int axis, int targetPulse) {
  const int startPulse = gimbalUs[axis];
  const int direction = targetPulse >= startPulse ? 1 : -1;
  Serial.printf("# SMOKE_RAMP gimbal=%s axis=%d from_us=%d to_us=%d "
                "step_us=%d interval_ms=%d\n",
                gimbalName(axis), axis, startPulse, targetPulse,
                SMOKE_RAMP_STEP_US, SMOKE_RAMP_INTERVAL_MS);
  int pulse = startPulse;
  while (pulse != targetPulse) {
    const int remaining = abs(targetPulse - pulse);
    pulse += direction * min(SMOKE_RAMP_STEP_US, remaining);
    writeGimbal(axis, pulse);
    if (!interruptibleWait(SMOKE_RAMP_INTERVAL_MS)) return false;
  }
  Serial.printf("# SMOKE_POSITION gimbal=%s axis=%d pwm_us=%d\n",
                gimbalName(axis), axis, targetPulse);
  return interruptibleWait(SMOKE_ENDPOINT_HOLD_MS);
}

bool smokeFast(int axis, int targetPulse) {
  writeGimbal(axis, targetPulse);
  Serial.printf("# SMOKE_POSITION gimbal=%s axis=%d pwm_us=%d mode=fast\n",
                gimbalName(axis), axis, targetPulse);
  return interruptibleWait(SMOKE_FAST_HOLD_MS);
}

void runSmokeTest(bool slowMode) {
  Serial.printf("# SMOKE_BEGIN mode=%s delta_us=%d ramp_step_us=%d "
                "ramp_interval_ms=%d source=user_pwm\n",
                slowMode ? "slow" : "fast", SMOKE_DELTA_US,
                SMOKE_RAMP_STEP_US, SMOKE_RAMP_INTERVAL_MS);
  attachGimbals();
  neutral();
  bool ok = interruptibleWait(SMOKE_NEUTRAL_SETTLE_MS);
  const int axes[] = {OUTER_GIMBAL_AXIS, INNER_GIMBAL_AXIS};
  for (int axis : axes) {
    if (ok) ok = slowMode ? smokeRamp(axis, NEUTRAL_US + SMOKE_DELTA_US)
                          : smokeFast(axis, NEUTRAL_US + SMOKE_DELTA_US);
    if (ok) ok = slowMode ? smokeRamp(axis, NEUTRAL_US)
                          : smokeFast(axis, NEUTRAL_US);
    if (ok) ok = slowMode ? smokeRamp(axis, NEUTRAL_US - SMOKE_DELTA_US)
                          : smokeFast(axis, NEUTRAL_US - SMOKE_DELTA_US);
    if (ok) ok = slowMode ? smokeRamp(axis, NEUTRAL_US)
                          : smokeFast(axis, NEUTRAL_US);
  }
  neutral();
  if (ok) ok = interruptibleWait(SMOKE_ENDPOINT_HOLD_MS);
  detachGimbals();
  Serial.printf("# SMOKE_END mode=%s pass=%d outer_us=%d inner_us=%d attached=%d\n",
                slowMode ? "slow" : "fast", ok,
                gimbalUs[OUTER_GIMBAL_AXIS],
                gimbalUs[INNER_GIMBAL_AXIS], gimbalsAttached);
}

bool logWindow(const char *phase, int seq, int axis, uint32_t durationMs) {
  const int nTarget = durationMs * DATA_RATE_HZ / 1000;
  uint32_t next = micros();
  for (int i = 0; i < nTarget; ++i) {
    while (static_cast<int32_t>(next - micros()) > 0) delayMicroseconds(10);
    const uint32_t t = micros();
    const int32_t jitter = static_cast<int32_t>(t - next);
    ImuRaw s{};
    uint8_t flags = readImu(s) ? 0 : FLAG_I2C;
    if (static_cast<int32_t>(micros() - (next + DATA_PERIOD_US)) >= 0)
      flags |= FLAG_LATE;
    emitSample(t, phase, seq, axis, s, flags, jitter);
    next += DATA_PERIOD_US;
    if (emergencyRequested()) return false;
  }
  return true;
}

void runHealth() {
  tRef = micros();
  emitMeta("HEALTH", -1);
  const int nTarget = HEALTH_MS * DATA_RATE_HZ / 1000;
  Serial.printf("#META expected_samples=%d expected_events=0\n", nTarget);
  beginDataStream();
  double sa[3] = {0, 0, 0}, sa2[3] = {0, 0, 0};
  double sg[3] = {0, 0, 0}, sg2[3] = {0, 0, 0};
  int good = 0;
  uint32_t next = micros();
  for (int i = 0; i < nTarget; ++i) {
    while (static_cast<int32_t>(next - micros()) > 0) delayMicroseconds(10);
    const uint32_t t = micros();
    const int32_t jitter = static_cast<int32_t>(t - next);
    ImuRaw s{};
    uint8_t flags = 0;
    if (!readImu(s)) flags |= FLAG_I2C;
    else {
      for (int k = 0; k < 3; ++k) {
        const double av = s.a[k] / ACC_LSB_PER_G;
        const double gv = s.g[k] / GYRO_LSB_PER_DPS;
        sa[k] += av; sa2[k] += av * av;
        sg[k] += gv; sg2[k] += gv * gv;
      }
      ++good;
    }
    if (static_cast<int32_t>(micros() - (next + DATA_PERIOD_US)) >= 0)
      flags |= FLAG_LATE;
    emitSample(t, "health", 0, -1, s, flags, jitter);
    next += DATA_PERIOD_US;
  }

  bool pass = imuReady && good == nTarget;
  double gmag2 = 0;
  double bias[3] = {0, 0, 0}, gsd[3] = {0, 0, 0}, asd[3] = {0, 0, 0};
  for (int k = 0; k < 3 && good > 0; ++k) {
    const double am = sa[k] / good;
    bias[k] = sg[k] / good;
    gsd[k] = sqrt(max(0.0, sg2[k] / good - bias[k] * bias[k]));
    asd[k] = sqrt(max(0.0, sa2[k] / good - am * am));
    gmag2 += am * am;
    pass &= fabs(bias[k]) < RAW_GYRO_BIAS_MAX_DPS &&
            gsd[k] < RAW_GYRO_NOISE_MAX_DPS &&
            asd[k] < RAW_ACCEL_NOISE_MAX_G;
  }
  const double gmag = sqrt(gmag2);
  pass &= fabs(gmag - 1.0) < RAW_GRAVITY_MAG_TOLERANCE_G;
  healthPass = pass;
  endDataStream();
  Serial.printf("#META health_pass=%d n_good=%d n_expected=%d g_mag=%.5f "
                "gyro_bias_dps=%.5f,%.5f,%.5f "
                "gyro_noise_dps=%.5f,%.5f,%.5f "
                "acc_noise_g=%.6f,%.6f,%.6f i2c_err_end=%lu\n",
                pass, good, nTarget, gmag, bias[0], bias[1], bias[2],
                gsd[0], gsd[1], gsd[2], asd[0], asd[1], asd[2],
                static_cast<unsigned long>(i2cErrors));
  Serial.println(F("# HEALTH_END"));
}

bool requireMotionReady(const char *endMarker) {
  if (!imuReady || !healthPass || !gimbalsAttached) {
    Serial.printf("#META firmware_test_pass=0 imu_ready=%d health_pass=%d "
                  "gimbals_attached=%d\n", imuReady, healthPass, gimbalsAttached);
    Serial.println(F("!! motion test refused: run HEALTH and ATTACH first"));
    Serial.println(endMarker);
    return false;
  }
  return true;
}

bool sweep(int axis, int from, int to, int step, const char *phase) {
  for (int pulse = from; (step > 0) ? pulse <= to : pulse >= to; pulse += step) {
    const uint32_t t = writeGimbal(axis, pulse);
    emitEvent(t, phase, pulse, axis);
    if (!logWindow(phase, pulse, axis, MAP_DWELL_MS)) return false;
  }
  return true;
}

void runMapping(int axis) {
  tRef = micros();
  if (!requireMotionReady("# TEST_A_END")) return;
  emitMeta("A", axis);
  const int points = (gimbalHi[axis] - gimbalLo[axis]) / MAP_STEP_US + 1;
  const int samplesPerDwell = MAP_DWELL_MS * DATA_RATE_HZ / 1000;
  Serial.printf("#META map_step_us=%d dwell_ms=%d log_hz=%d\n",
                MAP_STEP_US, MAP_DWELL_MS, DATA_RATE_HZ);
  Serial.printf("#META expected_samples=%d expected_events=%d\n",
                MAP_ZERO_MS * DATA_RATE_HZ / 1000 + 4 * points * samplesPerDwell,
                1 + 4 * points);
  beginDataStream();
  emitEvent(writeGimbal(axis, NEUTRAL_US), "zero", -1, axis);
  delay(MAP_NEUTRAL_SETTLE_MS);
  bool ok = logWindow("zero", -1, axis, MAP_ZERO_MS);
  if (ok) ok = sweep(axis, gimbalLo[axis], gimbalHi[axis], MAP_STEP_US, "up");
  if (ok) ok = sweep(axis, gimbalHi[axis], gimbalLo[axis], -MAP_STEP_US, "dn");
  if (ok) ok = sweep(axis, gimbalLo[axis], gimbalHi[axis], MAP_STEP_US, "up2");
  if (ok) ok = sweep(axis, gimbalHi[axis], gimbalLo[axis], -MAP_STEP_US, "dn2");
  neutral();
  endDataStream();
  Serial.printf("#META firmware_test_pass=%d i2c_err_end=%lu\n", ok,
                static_cast<unsigned long>(i2cErrors));
  Serial.println(F("# TEST_A_END"));
}

bool oneStep(int axis, int target, int seq) {
  neutral();
  delay(STEP_ARM_MS);
  const uint32_t t0 = micros();
  const int commandIndex = STEP_PRE_MS * DATA_RATE_HZ / 1000;
  uint32_t next = t0;
  bool ok = true;
  emitEvent(t0, "arm", seq, axis);
  for (int i = 0; i < STEP_SAMPLES; ++i) {
    while (static_cast<int32_t>(next - micros()) > 0) delayMicroseconds(10);
    if (i == commandIndex) {
      const uint32_t tCommand = writeGimbal(axis, target);
      emitEvent(tCommand, "cmd", seq, axis);
    }
    const uint32_t t = micros();
    const int32_t jitter = static_cast<int32_t>(t - next);
    ImuRaw s{};
    uint8_t flags = readImu(s) ? 0 : FLAG_I2C;
    if (flags & FLAG_I2C) ok = false;
    if (static_cast<int32_t>(micros() - (next + DATA_PERIOD_US)) >= 0)
      flags |= FLAG_LATE;
    emitSample(t, i < commandIndex ? "pre" : "post", seq, axis,
               s, flags, jitter);
    next += DATA_PERIOD_US;
    if (emergencyRequested()) return false;
  }
  neutral();
  delay(STEP_RETURN_SETTLE_MS);
  return ok;
}

void runStep(int axis) {
  tRef = micros();
  if (!requireMotionReady("# TEST_B_END")) return;
  emitMeta("B", axis);
  Serial.printf("#META log_hz=%d pre_ms=%d post_ms=%d arm_ms=%d n=%d\n",
                DATA_RATE_HZ, STEP_PRE_MS, STEP_POST_MS, STEP_ARM_MS, STEP_SAMPLES);
  Serial.printf("#META expected_samples=%d expected_events=%d\n",
                static_cast<int>(2 * STEP_PERCENT_COUNT * STEP_SAMPLES),
                static_cast<int>(4 * STEP_PERCENT_COUNT));
  beginDataStream();
  bool ok = true;
  int seq = 0;
  for (size_t i = 0; i < STEP_PERCENT_COUNT && ok; ++i) {
    const int positiveAmp = max(
        STEP_MIN_AMPLITUDE_US,
        (gimbalHi[axis] - NEUTRAL_US) * STEP_PERCENT[i] / 100);
    const int negativeAmp = max(
        STEP_MIN_AMPLITUDE_US,
        (NEUTRAL_US - gimbalLo[axis]) * STEP_PERCENT[i] / 100);
    ok &= oneStep(axis, NEUTRAL_US + positiveAmp, seq++);
    if (ok) ok &= oneStep(axis, NEUTRAL_US - negativeAmp, seq++);
    if (emergencyRequested()) ok = false;
  }
  neutral();
  endDataStream();
  Serial.printf("#META firmware_test_pass=%d i2c_err_end=%lu\n", ok,
                static_cast<unsigned long>(i2cErrors));
  Serial.println(F("# TEST_B_END"));
}

void runChirp(int axis) {
  tRef = micros();
  if (!requireMotionReady("# TEST_C_END")) return;
  emitMeta("C", axis);
  const int travel = min(gimbalHi[axis] - NEUTRAL_US, NEUTRAL_US - gimbalLo[axis]);
  const int amp = max(CHIRP_MIN_AMP_US, travel * CHIRP_AMPLITUDE_PCT / 100);
  const int preN = CHIRP_PRE_MS * DATA_RATE_HZ / 1000;
  const int sweepN = CHIRP_SWEEP_MS * DATA_RATE_HZ / 1000;
  const double T = CHIRP_SWEEP_MS / 1000.0;
  const double f0 = CHIRP_F0_HZ, f1 = CHIRP_F1_HZ;
  const double k = log(static_cast<double>(f1) / f0);
  Serial.printf("#META chirp_f0_hz=%.3f chirp_f1_hz=%.3f chirp_amp_us=%d "
                "sweep_ms=%d log_hz=%d\n", f0, f1, amp, CHIRP_SWEEP_MS, DATA_RATE_HZ);
  Serial.printf("#META expected_samples=%d expected_events=0\n", preN + sweepN);
  beginDataStream();
  neutral();
  delay(CHIRP_SETTLE_MS);
  bool ok = logWindow("pre", -1, axis, CHIRP_PRE_MS);
  uint32_t next = micros();
  for (int i = 0; i < sweepN && ok; ++i) {
    while (static_cast<int32_t>(next - micros()) > 0) delayMicroseconds(10);
    const double t = static_cast<double>(i) / DATA_RATE_HZ;
    const double phase = 2.0 * PI * f0 * T / k * (exp(k * t / T) - 1.0);
    const int pulse = NEUTRAL_US + static_cast<int>(lround(amp * sin(phase)));
    writeGimbal(axis, pulse);
    const uint32_t tt = micros();
    const int32_t jitter = static_cast<int32_t>(tt - next);
    ImuRaw s{};
    uint8_t flags = readImu(s) ? 0 : FLAG_I2C;
    if (flags & FLAG_I2C) ok = false;
    if (static_cast<int32_t>(micros() - (next + DATA_PERIOD_US)) >= 0)
      flags |= FLAG_LATE;
    emitSample(tt, "chirp", -1, axis, s, flags, jitter);
    next += DATA_PERIOD_US;
    if (emergencyRequested()) { ok = false; break; }
  }
  neutral();
  endDataStream();
  Serial.printf("#META firmware_test_pass=%d i2c_err_end=%lu\n", ok,
                static_cast<unsigned long>(i2cErrors));
  Serial.println(F("# TEST_C_END"));
}

void runDeadband(int axis) {
  tRef = micros();
  if (!requireMotionReady("# TEST_D_END")) return;
  emitMeta("D", axis);
  const int lo = NEUTRAL_US - DB_RANGE_US;
  const int hi = NEUTRAL_US + DB_RANGE_US;
  const int points = (hi - lo) / DB_STEP_US + 1;
  const int samplesPerDwell = DB_DWELL_MS * DATA_RATE_HZ / 1000;
  Serial.printf("#META db_range_us=%d db_step_us=%d dwell_ms=%d log_hz=%d\n",
                DB_RANGE_US, DB_STEP_US, DB_DWELL_MS, DATA_RATE_HZ);
  Serial.printf("#META expected_samples=%d expected_events=%d\n",
                DB_ZERO_MS * DATA_RATE_HZ / 1000 + 2 * points * samplesPerDwell,
                1 + 2 * points);
  beginDataStream();
  emitEvent(writeGimbal(axis, NEUTRAL_US), "zero", -1, axis);
  delay(DB_NEUTRAL_SETTLE_MS);
  bool ok = logWindow("zero", -1, axis, DB_ZERO_MS);
  for (int pulse = lo; pulse <= hi && ok; pulse += DB_STEP_US) {
    const uint32_t t = writeGimbal(axis, pulse);
    emitEvent(t, "up", pulse, axis);
    ok = logWindow("up", pulse, axis, DB_DWELL_MS);
  }
  for (int pulse = hi; pulse >= lo && ok; pulse -= DB_STEP_US) {
    const uint32_t t = writeGimbal(axis, pulse);
    emitEvent(t, "dn", pulse, axis);
    ok = logWindow("dn", pulse, axis, DB_DWELL_MS);
  }
  neutral();
  endDataStream();
  Serial.printf("#META firmware_test_pass=%d i2c_err_end=%lu\n", ok,
                static_cast<unsigned long>(i2cErrors));
  Serial.println(F("# TEST_D_END"));
}

void runGrid() {
  tRef = micros();
  if (!requireMotionReady("# TEST_E_END")) return;
  emitMeta("E", -1);
  const int n = GRID_POINTS_PER_AXIS;
  const int oLo = gimbalLo[OUTER_GIMBAL_AXIS], oHi = gimbalHi[OUTER_GIMBAL_AXIS];
  const int iLo = gimbalLo[INNER_GIMBAL_AXIS], iHi = gimbalHi[INNER_GIMBAL_AXIS];
  const int cells = n * n;
  const int samplesPerDwell = GRID_DWELL_MS * DATA_RATE_HZ / 1000;
  Serial.printf("#META grid_points_per_axis=%d dwell_ms=%d settle_ms=%d "
                "row_settle_ms=%d log_hz=%d\n",
                n, GRID_DWELL_MS, GRID_SETTLE_MS, GRID_ROW_SETTLE_MS, DATA_RATE_HZ);
  Serial.printf("#META outer_lo_us=%d outer_hi_us=%d inner_lo_us=%d inner_hi_us=%d\n",
                oLo, oHi, iLo, iHi);
  Serial.printf("#META expected_samples=%d expected_events=%d\n",
                GRID_ZERO_MS * DATA_RATE_HZ / 1000 + cells * samplesPerDwell,
                1 + cells);
  beginDataStream();
  neutral();
  delay(GRID_NEUTRAL_SETTLE_MS);
  emitEvent(micros(), "zero", -1, -1);
  bool ok = logWindow("zero", -1, -1, GRID_ZERO_MS);
  int seq = 0;
  for (int oi = 0; oi < n && ok; ++oi) {
    const int outer = oLo + (oHi - oLo) * oi / (n - 1);
    writeGimbal(OUTER_GIMBAL_AXIS, outer);
    // Serpentine: reverse the inner direction every other outer row so
    // consecutive cells are always one inner step apart (no long slew).
    for (int k = 0; k < n && ok; ++k) {
      const int ii = (oi & 1) ? (n - 1 - k) : k;
      const int inner = iLo + (iHi - iLo) * ii / (n - 1);
      const uint32_t t = writeGimbal(INNER_GIMBAL_AXIS, inner);
      delay(k == 0 ? GRID_ROW_SETTLE_MS : GRID_SETTLE_MS);
      emitEvent(t, "grid", seq, -1);
      ok = logWindow("grid", seq, -1, GRID_DWELL_MS);
      ++seq;
      if (emergencyRequested()) ok = false;
    }
  }
  neutral();
  endDataStream();
  Serial.printf("#META firmware_test_pass=%d i2c_err_end=%lu\n", ok,
                static_cast<unsigned long>(i2cErrors));
  Serial.println(F("# TEST_E_END"));
}

void printStatus() {
  Serial.printf("# STATUS fw=%s imu_count=1 gimbal_count=2 imu_addr=0x%02X "
                "imu_ready=%d health_pass=%d attached=%d "
                "outer_us=%d inner_us=%d "
                "sensor_odr_hz=%d stream_hz=%d serial_baud=%lu\n",
                FW_VERSION, IMU_ADDR, imuReady, healthPass, gimbalsAttached,
                gimbalUs[OUTER_GIMBAL_AXIS], gimbalUs[INNER_GIMBAL_AXIS],
                SENSOR_INTERNAL_ODR_HZ, DATA_RATE_HZ,
                static_cast<unsigned long>(SERIAL_BAUD));
  Serial.printf("# LIMITS outer=%d:%d inner=%d:%d source=user_pwm\n",
                gimbalLo[OUTER_GIMBAL_AXIS], gimbalHi[OUTER_GIMBAL_AXIS],
                gimbalLo[INNER_GIMBAL_AXIS], gimbalHi[INNER_GIMBAL_AXIS]);
}

void handleCommand(String line) {
  line.trim();
  if (!line.length()) return;
  line.toUpperCase();
  if (line == "PING") {
    Serial.printf("# PONG fw=%s imu_count=1 gimbal_count=2 ready=%d\n",
                  FW_VERSION, imuReady);
  } else if (line == "STATUS") {
    printStatus();
  } else if (line == "HEALTH") {
    runHealth();
  } else if (line == "ATTACH") {
    attachGimbals(); neutral(); Serial.println(F("# ATTACHED"));
  } else if (line == "DETACH" || line == "!") {
    neutral(); detachGimbals(); Serial.println(F("# DETACHED"));
  } else if (line == "NEUTRAL") {
    neutral(); Serial.println(F("# NEUTRAL"));
  } else if (line == "SMOKE" || line == "SMOKE SLOW") {
    runSmokeTest(true);
  } else if (line == "SMOKE FAST") {
    runSmokeTest(false);
  } else if (line == "A OUTER" || line == "A 0") {
    runMapping(OUTER_GIMBAL_AXIS);
  } else if (line == "A INNER" || line == "A 1") {
    runMapping(INNER_GIMBAL_AXIS);
  } else if (line == "B OUTER" || line == "B 0") {
    runStep(OUTER_GIMBAL_AXIS);
  } else if (line == "B INNER" || line == "B 1") {
    runStep(INNER_GIMBAL_AXIS);
  } else if (line == "C OUTER" || line == "C 0") {
    runChirp(OUTER_GIMBAL_AXIS);
  } else if (line == "C INNER" || line == "C 1") {
    runChirp(INNER_GIMBAL_AXIS);
  } else if (line == "D OUTER" || line == "D 0") {
    runDeadband(OUTER_GIMBAL_AXIS);
  } else if (line == "D INNER" || line == "D 1") {
    runDeadband(INNER_GIMBAL_AXIS);
  } else if (line == "E" || line == "GRID" || line == "E BOTH") {
    runGrid();
  } else {
    Serial.printf("!! unknown command: %s\n", line.c_str());
  }
}

void setup() {
  Serial.setTxBufferSize(SERIAL_TX_BUFFER_BYTES);
  Serial.begin(SERIAL_BAUD);
  delay(1000);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(I2C_CLOCK_HZ);
  imuReady = initImu();
  gimbalUs[OUTER_GIMBAL_AXIS] = NEUTRAL_US;
  gimbalUs[INNER_GIMBAL_AXIS] = NEUTRAL_US;
  // Stay detached until the PC completes calibration/health and explicitly
  // sends ATTACH. This prevents reset from causing uncommanded movement.
  Serial.printf("# BOOT fw=%s imu_count=1 gimbal_count=2 imu_addr=0x%02X "
                "imu_ready=%d\n", FW_VERSION, IMU_ADDR, imuReady);
  if (!imuReady)
    Serial.println(F("!! ICM-20948 not found at 0x69; motion tests are locked"));
  printStatus();
  Serial.println(F("# READY"));
}

void loop() {
  if (!Serial.available()) {
    delay(2);
    return;
  }
  handleCommand(Serial.readStringUntil('\n'));
}
