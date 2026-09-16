/* =====================================================================
   Simple servo jog  —  ESP32-WROOM-32D + PTK 8515MG-D
   ---------------------------------------------------------------------
   The bare minimum to find the mechanical limit angle by hand.  No IMU,
   no tests, no files.  Type a pulse, watch the ring move, note where it
   stops.  That's it.

   WIRING
     Servo signal -> GPIO 18
     Servo power  -> separate BEC 7.4V, common GND with the ESP32
     (do NOT power the servo from the board 5V pin)

   SERIAL 115200, newline ending.  Commands:
     1500     go to 1500 us
     +  -     nudge by the step (default 10 us)
     s 5      set step to 5 us
     n        neutral (1520 us)
     d        detach — servo goes LIMP (press the moment it buzzes)
     e        re-attach

   FIND THE LIMIT
     From neutral, set a small step (s 5) and tap '+' until the ring
     stops moving but the servo starts straining/buzzing.  That pulse is
     the mechanical stop.  Press 'd' immediately, write down the last
     pulse that still moved, back off ~30 us.  Holding against a hard
     stop cooks the gears.
   ===================================================================== */

#include <ESP32Servo.h>

#define SERVO_PIN   18
#define SERVO_HZ    333       // PTK 8515MG-D; use 50 for a generic SG90
#define PULSE_MIN   500
#define PULSE_MAX   2500
#define NEUTRAL     1520

Servo servo;
int  pos  = NEUTRAL;
int  step = 10;
bool attached = false;

void goTo(int us) {
  if (!attached) { Serial.println("detached - press 'e'"); return; }
  if (us < PULSE_MIN) us = PULSE_MIN;
  if (us > PULSE_MAX) us = PULSE_MAX;
  pos = us;
  servo.writeMicroseconds(pos);
  Serial.printf("-> %d us  (%+d from neutral)\n", pos, pos - NEUTRAL);
}

void handle(String s) {
  s.trim();
  if (!s.length()) return;

  if (isDigit(s.charAt(0))) { goTo(s.toInt()); return; }
  if (s == "+") { goTo(pos + step); return; }
  if (s == "-") { goTo(pos - step); return; }
  if (s == "n") { goTo(NEUTRAL);    return; }
  if (s == "d") { servo.detach(); attached = false;
                  Serial.println("DETACHED - limp"); return; }
  if (s == "e") { servo.attach(SERVO_PIN, PULSE_MIN, PULSE_MAX);
                  attached = true; servo.writeMicroseconds(pos);
                  Serial.println("attached"); return; }
  if (s.charAt(0) == 's') {
    int v = s.substring(1).toInt();
    if (v >= 1 && v <= 200) { step = v; Serial.printf("step = %d us\n", step); }
    return;
  }
  Serial.println("? type a number, + - n s<us> d e");
}

void setup() {
  Serial.begin(115200);
  delay(1500);

  ESP32PWM::allocateTimer(0);
  servo.setPeriodHertz(SERVO_HZ);
  servo.attach(SERVO_PIN, PULSE_MIN, PULSE_MAX);
  servo.writeMicroseconds(pos);
  attached = true;

  Serial.println("simple servo jog. commands: <us> | + - | s<us> | n | d e");
  Serial.printf("at neutral %d us\n", NEUTRAL);
}

void loop() {
  static String buf;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') { handle(buf); buf = ""; }
    else if (buf.length() < 16) buf += c;
  }
}
