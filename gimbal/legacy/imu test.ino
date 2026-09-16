/* =====================================================================
   ICM-20948 test bench — ESP32-WROOM-32D
   IMU only.  No servo needed.

   Purpose: prove the IMU works, the axes are what you think they are,
   and the mount is rigid — before wiring it into the TVC test.

   WIRING (Adafruit ICM-20948 breakout)
     VIN -> 3V3        SDA -> GPIO 21
     GND -> GND        SCL -> GPIO 22
     I2C address 0x69 (Adafruit default).  Try 0x68 if not found.

   Raw registers, no fusion, no DMP:
     gyro  +/-2000 dps, DLPF 361 Hz (0.17 ms delay), ODR 1125 Hz
     accel +/-2 g,      DLPF 473 Hz,                 ODR 1125 Hz

   Serial 115200.
   ===================================================================== */

#include <Wire.h>

#define I2C_SDA   21
#define I2C_SCL   22
uint8_t ICM_ADDR = 0x69;

#define REG_BANK_SEL    0x7F
#define B0_WHO_AM_I     0x00
#define B0_USER_CTRL    0x03
#define B0_PWR_MGMT_1   0x06
#define B0_PWR_MGMT_2   0x07
#define B0_ACCEL_XOUT   0x2D
#define B0_GYRO_XOUT    0x33
#define B2_GYRO_SMPLRT  0x00
#define B2_GYRO_CFG1    0x01
#define B2_ACC_SMPLRT_1 0x10
#define B2_ACC_SMPLRT_2 0x11
#define B2_ACCEL_CFG    0x14

#define GYRO_LSB_PER_DPS  16.4f
#define ACC_LSB_PER_G     16384.0f

float gyroBias[3] = {0, 0, 0};
int   gravAxis    = 2;

// ---------------- low level ----------------
void wr(uint8_t r, uint8_t v) {
  Wire.beginTransmission(ICM_ADDR);
  Wire.write(r); Wire.write(v);
  Wire.endTransmission();
}
uint8_t rd(uint8_t r) {
  Wire.beginTransmission(ICM_ADDR);
  Wire.write(r);
  Wire.endTransmission(false);
  Wire.requestFrom(ICM_ADDR, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0xFF;
}
void bank(uint8_t b) { wr(REG_BANK_SEL, b << 4); }

// ICM-20948 bank 0 layout:
//   0x2D..0x32  ACCEL X,Y,Z
//   0x33..0x38  GYRO  X,Y,Z
//   0x39..0x3A  TEMP          <-- AFTER the gyro, not between (unlike MPU6050)
static inline int16_t be16() {
  uint8_t hi = Wire.read();
  uint8_t lo = Wire.read();
  return (int16_t)(((uint16_t)hi << 8) | lo);
}

bool readAll(int16_t *a, int16_t *g) {
  Wire.beginTransmission(ICM_ADDR);
  Wire.write(B0_ACCEL_XOUT);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(ICM_ADDR, (uint8_t)12) != 12) return false;  // 6+6
  for (int i = 0; i < 3; i++) a[i] = be16();
  for (int i = 0; i < 3; i++) g[i] = be16();
  return true;
}
bool readGyro(int16_t *g) {
  Wire.beginTransmission(ICM_ADDR);
  Wire.write(B0_GYRO_XOUT);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(ICM_ADDR, (uint8_t)6) != 6) return false;
  for (int i = 0; i < 3; i++) g[i] = be16();
  return true;
}

void i2cScan() {
  Serial.println(F("# I2C scan on SDA=21 SCL=22"));
  int n = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.printf("   found device at 0x%02X\n", a);
      n++;
    }
  }
  if (!n) Serial.println(F("   nothing found - check wiring / 3V3 / pull-ups"));
}

bool imuInit() {
  bank(0);
  uint8_t who = rd(B0_WHO_AM_I);
  if (who != 0xEA) {
    Serial.printf("!! WHO_AM_I = 0x%02X at addr 0x%02X (expected 0xEA)\n",
                  who, ICM_ADDR);
    return false;
  }
  wr(B0_PWR_MGMT_1, 0x80); delay(120);
  bank(0);
  wr(B0_PWR_MGMT_1, 0x01); delay(20);
  wr(B0_PWR_MGMT_2, 0x00);
  wr(B0_USER_CTRL,  0x00);

  bank(2);
  wr(B2_GYRO_CFG1,  (7 << 3) | (3 << 1) | 1);   // DLPF7, +/-2000dps
  wr(B2_GYRO_SMPLRT, 0);
  wr(B2_ACCEL_CFG,  (7 << 3) | (0 << 1) | 1);   // DLPF7, +/-2g
  wr(B2_ACC_SMPLRT_1, 0);
  wr(B2_ACC_SMPLRT_2, 0);

  bank(0); delay(50);
  Serial.printf("# ICM-20948 ok at 0x%02X\n", ICM_ADDR);
  return true;
}

void detectGravityAxis() {
  int16_t a[3], g[3];
  float s[3] = {0, 0, 0};
  for (int i = 0; i < 60; i++) {
    if (readAll(a, g)) for (int k = 0; k < 3; k++) s[k] += a[k];
    delay(4);
  }
  gravAxis = 0;
  for (int k = 1; k < 3; k++)
    if (fabsf(s[k]) > fabsf(s[gravAxis])) gravAxis = k;
  Serial.printf("# gravity is on axis %d (%c), sign %s\n",
                gravAxis, "XYZ"[gravAxis], s[gravAxis] > 0 ? "+" : "-");
}

// ---------------- tests ----------------
void calibGyro() {
  Serial.println(F("# gyro bias calibration - KEEP STILL (3 s)"));
  delay(600);
  double s[3] = {0, 0, 0}; int n = 0;
  for (int i = 0; i < 1500; i++) {
    int16_t g[3];
    if (readGyro(g)) { for (int k = 0; k < 3; k++) s[k] += g[k]; n++; }
    delayMicroseconds(1800);
  }
  for (int k = 0; k < 3; k++) gyroBias[k] = n ? (float)(s[k] / n) : 0.0f;
  Serial.printf("# bias  %.2f  %.2f  %.2f dps   (n=%d)\n",
                gyroBias[0] / GYRO_LSB_PER_DPS,
                gyroBias[1] / GYRO_LSB_PER_DPS,
                gyroBias[2] / GYRO_LSB_PER_DPS, n);
}

// live: raw values + tilt angles about each axis
void live(uint32_t ms) {
  Serial.println(F("# ax ay az [g] | gx gy gz [dps] | tiltX tiltY [deg]"));
  uint32_t t = millis();
  while (millis() - t < ms) {
    int16_t a[3], g[3];
    if (readAll(a, g)) {
      float A[3] = {a[0]/ACC_LSB_PER_G, a[1]/ACC_LSB_PER_G, a[2]/ACC_LSB_PER_G};
      float G[3] = {(g[0]-gyroBias[0])/GYRO_LSB_PER_DPS,
                    (g[1]-gyroBias[1])/GYRO_LSB_PER_DPS,
                    (g[2]-gyroBias[2])/GYRO_LSB_PER_DPS};
      float tx = atan2f(A[0], A[gravAxis]) * 57.29578f;
      float ty = atan2f(A[1], A[gravAxis]) * 57.29578f;
      float norm = sqrtf(A[0]*A[0] + A[1]*A[1] + A[2]*A[2]);
      Serial.printf("%+6.3f %+6.3f %+6.3f |%+8.2f %+8.2f %+8.2f |"
                    " %+7.2f %+7.2f  |g|=%.3f\n",
                    A[0],A[1],A[2], G[0],G[1],G[2], tx, ty, norm);
    } else Serial.println(F("read failed"));
    delay(100);
  }
}

// noise floor + effective resolution, sitting still
void noiseTest() {
  Serial.println(F("# NOISE FLOOR - keep perfectly still, 5 s"));
  delay(800);
  const int N = 2000;
  double sg[3] = {0,0,0}, sg2[3] = {0,0,0};
  double sa[3] = {0,0,0}, sa2[3] = {0,0,0};
  int n = 0;
  for (int i = 0; i < N; i++) {
    int16_t a[3], g[3];
    if (readAll(a, g)) {
      for (int k = 0; k < 3; k++) {
        double gv = (g[k] - gyroBias[k]) / GYRO_LSB_PER_DPS;
        double av = a[k] / ACC_LSB_PER_G;
        sg[k] += gv; sg2[k] += gv*gv;
        sa[k] += av; sa2[k] += av*av;
      }
      n++;
    }
    delayMicroseconds(1500);
  }
  Serial.printf("# n=%d\n", n);
  for (int k = 0; k < 3; k++) {
    double mg = sg[k]/n, sdg = sqrt(sg2[k]/n - mg*mg);
    double ma = sa[k]/n, sda = sqrt(sa2[k]/n - ma*ma);
    Serial.printf("  %c : gyro mean %+7.3f sd %6.3f dps |"
                  "  accel mean %+7.4f sd %6.4f g  (= %.3f deg)\n",
                  "XYZ"[k], mg, sdg, ma, sda, sda*57.29578f);
  }
  Serial.println(F("# gyro sd > 0.5 dps or accel sd > 0.01 g -> check"));
  Serial.println(F("# mounting, cable strain, or nearby vibration"));
}

// how fast can we actually poll over I2C
void rateTest() {
  Serial.println(F("# I2C THROUGHPUT test"));
  const int N = 2000;
  int16_t a[3], g[3];
  uint32_t t0 = micros();
  int ok = 0;
  for (int i = 0; i < N; i++) if (readAll(a, g)) ok++;
  uint32_t dt = micros() - t0;
  Serial.printf("  14-byte read : %.1f us each -> max %.0f Hz  (%d/%d ok)\n",
                (float)dt/N, 1e6f*N/dt, ok, N);

  t0 = micros(); ok = 0;
  for (int i = 0; i < N; i++) if (readGyro(g)) ok++;
  dt = micros() - t0;
  Serial.printf("  6-byte  read : %.1f us each -> max %.0f Hz  (%d/%d ok)\n",
                (float)dt/N, 1e6f*N/dt, ok, N);
  Serial.println(F("# need >= 1000 Hz for the step test. If short, drop to"));
  Serial.println(F("# gyro-only reads or move to SPI."));
}

// tap the plate: how long does it ring?  Tests mount rigidity.
void tapTest() {
  Serial.println(F("# TAP TEST - flick the plate with a finger."));
  Serial.println(F("# logging peak gyro rate for 6 s, 200 Hz"));
  uint32_t t = millis();
  float peak = 0;
  uint32_t lastPrint = 0;
  while (millis() - t < 6000) {
    int16_t g[3];
    if (readGyro(g)) {
      float m = 0;
      for (int k = 0; k < 3; k++) {
        float v = fabsf((g[k] - gyroBias[k]) / GYRO_LSB_PER_DPS);
        if (v > m) m = v;
      }
      if (m > peak) peak = m;
      if (millis() - lastPrint > 100) {
        lastPrint = millis();
        int bars = (int)(m / 5);
        Serial.printf("%7.1f dps ", m);
        for (int i = 0; i < min(bars, 60); i++) Serial.print('#');
        Serial.println();
      }
    }
    delay(5);
  }
  Serial.printf("# peak %.1f dps\n", peak);
  Serial.println(F("# it should decay to near zero within ~0.2 s."));
  Serial.println(F("# long ringing = soft mount (foam tape?) or loose bolts"));
}

// stream raw CSV for offline analysis
void streamCsv(uint32_t ms) {
  Serial.println(F("t_us,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps"));
  uint32_t t0 = micros();
  while ((micros() - t0) < ms * 1000UL) {
    int16_t a[3], g[3];
    if (readAll(a, g)) {
      Serial.printf("%lu,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f\n",
        (unsigned long)(micros() - t0),
        a[0]/ACC_LSB_PER_G, a[1]/ACC_LSB_PER_G, a[2]/ACC_LSB_PER_G,
        (g[0]-gyroBias[0])/GYRO_LSB_PER_DPS,
        (g[1]-gyroBias[1])/GYRO_LSB_PER_DPS,
        (g[2]-gyroBias[2])/GYRO_LSB_PER_DPS);
    }
    delay(2);
  }
  Serial.println(F("# STREAM_END"));
}

void help() {
  Serial.println();
  Serial.println(F("-------------------------------------------"));
  Serial.println(F(" i   I2C scan"));
  Serial.println(F(" c   gyro bias calibration (keep still)"));
  Serial.println(F(" g   re-detect gravity axis"));
  Serial.println(F(" l   live readout, 20 s"));
  Serial.println(F(" n   noise floor / resolution, 5 s"));
  Serial.println(F(" r   I2C throughput test"));
  Serial.println(F(" t   tap test (mount rigidity)"));
  Serial.println(F(" s   stream raw CSV, 10 s"));
  Serial.println(F(" ?   this help"));
  Serial.println(F("-------------------------------------------"));
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  Serial.println(F("\n=== ICM-20948 bench ==="));
  i2cScan();
  if (!imuInit()) {
    ICM_ADDR = 0x68;
    Serial.println(F("# retrying at 0x68..."));
    imuInit();
  }
  detectGravityAxis();
  calibGyro();
  help();
}

void loop() {
  if (!Serial.available()) return;
  char c = Serial.read();
  if (c == '\n' || c == '\r') return;
  switch (c) {
    case 'i': i2cScan();            break;
    case 'c': calibGyro();          break;
    case 'g': detectGravityAxis();  break;
    case 'l': live(20000);          break;
    case 'n': noiseTest();          break;
    case 'r': rateTest();           break;
    case 't': tapTest();            break;
    case 's': streamCsv(10000);     break;
    default:  help();               break;
  }
}