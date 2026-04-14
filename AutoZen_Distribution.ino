#include <Arduino.h>
#include <math.h>

/*
  ZenGarden_Spiro_Distribution.ino
  --------------------------------
  Distribution-ready version of the sand-garden controller that keeps only
  the spirograph mode from the larger experimental sketch.

  What this sketch does:
    - homes both axes
    - recentres the magnet carriage in the middle of the sand pit
    - draws a single closed spirograph pattern
    - stops when the pattern is complete (no reverse leg)

  User interaction is through the Serial Monitor at 115200 baud.

  Main commands:
    R              run the current spiro pattern
    S              stop motion
    H              re-home and re-centre
    P              print status
    ?              print help

    C x y r        set pit centre and radius in mm
    T seconds      set total time for ONE outer loop of the spiro pattern
    D ms           set update interval in milliseconds
    K scale        set overall pattern scale inside the pit radius
    I frac         set spiro inner fraction, 0.00 .. 0.90
    F value        set spiro frequency (snapped to a simple fraction)

  Example:
    I 0.22
    F 2.5
    T 32
    K 0.85
    R
*/

/******** Geometry & timing ********/
#ifndef TRACK_LEN_MM
#define TRACK_LEN_MM 75
#endif

const float SOFT_MIN_MM = 0.0f;
const float SOFT_MAX_MM = (float)TRACK_LEN_MM;
const float EDGE_EPS_MM = 1.0f;
const int   STEPS_PER_REV   = 20;
const float LEAD_MM_PER_REV = 0.5f;
const int   STEPS_PER_MM    = STEPS_PER_REV / LEAD_MM_PER_REV;   // 40 steps / mm
const int   STEP_HIGH_US    = 60;
const int   MIN_DWELL_US    = 600;
const int   HOMING_DELAY_US = 1000;
const int   SAFE_START_MM   = 10;
const int   MAX_HOME_MM     = 100;
const long  MAX_HOME_STEPS  = (long)MAX_HOME_MM * STEPS_PER_MM;
const int   CENTER_DWELL_US = 1200;
const float LAND_MARGIN     = 0.3f;
const float CIRCLE_GUARD_MM = 0.4f;

static inline uint32_t max_u32(uint32_t a, uint32_t b) { return a > b ? a : b; }

/******** Motion layer copied from the working calibration sketch ********/
class StepperAxis {
public:
  StepperAxis(int stepPin, int dirPin, int enPin, int endPin, int forwardDirLevel, const char* name)
  : STEP(stepPin), DIR(dirPin), EN(enPin), ES(endPin),
    FWD(forwardDirLevel), REV(forwardDirLevel == HIGH ? LOW : HIGH), label(name) {}

  void begin() {
    pinMode(STEP, OUTPUT);
    pinMode(DIR, OUTPUT);
    pinMode(EN, OUTPUT);
    pinMode(ES, INPUT_PULLUP);
    digitalWrite(EN, HIGH);
    digitalWrite(STEP, LOW);
    delay(50);
    enable();
  }

  void enable() { digitalWrite(EN, LOW); }

  void setDirection(bool fwd) { digitalWrite(DIR, fwd ? FWD : REV); }

  float mm() const { return (float)posSteps / (float)STEPS_PER_MM; }

  void pulseFast() {
    digitalWrite(STEP, HIGH);
    delayMicroseconds(STEP_HIGH_US);
    digitalWrite(STEP, LOW);
    delayMicroseconds(STEP_HIGH_US);
  }

  bool stepForward() {
    if (posSteps >= (long)(SOFT_MAX_MM * STEPS_PER_MM)) return false;
    setDirection(true);
    pulseFast();
    posSteps++;
    return true;
  }

  bool stepReverse() {
    if (posSteps <= (long)(SOFT_MIN_MM * STEPS_PER_MM)) return false;
    setDirection(false);
    pulseFast();
    posSteps--;
    return true;
  }

  void moveMM(float dmm) {
    if (!dmm) return;
    long n = lroundf(fabsf(dmm) * STEPS_PER_MM);
    bool fwd = dmm > 0.0f;
    for (long i = 0; i < n; i++) {
      if (fwd) {
        if (!stepForward()) break;
      } else {
        if (!stepReverse()) break;
      }
      delayMicroseconds(150);
    }
  }

  void moveToMM(float t) { moveMM(t - mm()); }

  bool home() {
    if (endstopPressed()) {
      int guard = 3 * STEPS_PER_MM;
      while (endstopPressed() && guard-- > 0) {
        setDirection(true);
        pulseFast();
        posSteps++;
        delayMicroseconds(900);
      }
      delay(30);
    }

    setDirection(false);
    long cnt = 0;
    while (!endstopPressed()) {
      pulseFast();
      posSteps--;
      delayMicroseconds(HOMING_DELAY_US);
      if (++cnt > MAX_HOME_STEPS) {
        Serial.printf("[%s] Home guard hit\n", label);
        return false;
      }
    }

    setDirection(true);
    {
      int guard = 3 * STEPS_PER_MM;
      while (endstopPressed() && guard-- > 0) {
        pulseFast();
        posSteps++;
        delayMicroseconds(900);
      }
      int extra = (int)(0.5f * STEPS_PER_MM);
      for (int i = 0; i < extra; i++) {
        pulseFast();
        posSteps++;
        delayMicroseconds(900);
      }
    }

    posSteps = 0;
    Serial.printf("[%s] Home = 0 mm\n", label);
    return true;
  }

  void moveToMM_slow(float tgt, int dwell_us = CENTER_DWELL_US) {
    if (tgt < SOFT_MIN_MM) tgt = SOFT_MIN_MM;
    if (tgt > SOFT_MAX_MM) tgt = SOFT_MAX_MM;
    long T = lroundf(tgt * STEPS_PER_MM);
    bool fwd = (T > posSteps);
    setDirection(fwd);
    long N = labs(T - posSteps);
    for (long i = 0; i < N; i++) {
      pulseFast();
      posSteps += fwd ? 1 : -1;
      delayMicroseconds(dwell_us);
    }
  }

private:
  const int STEP, DIR, EN, ES;
  const int FWD, REV;
  const char* label;
  long posSteps = 0;

  bool endstopPressed() {
    const int N = 5;
    int c = 0;
    for (int i = 0; i < N; i++) {
      if (digitalRead(ES) == LOW) c++;
      delay(1);
    }
    return c > N / 2;
  }
};

StepperAxis axisX(1, 2, 3, 4, LOW, "X");
StepperAxis axisY(9, 8, 7, 44, LOW, "Y");

struct Coordinator {
  float rx_steps = 0.0f;
  float ry_steps = 0.0f;

  void moveVectorMM(float dx, float dy, uint32_t dt_ms) {
    float x_start = axisX.mm();
    float y_start = axisY.mm();
    float xt = constrain(x_start + dx, SOFT_MIN_MM, SOFT_MAX_MM);
    float yt = constrain(y_start + dy, SOFT_MIN_MM, SOFT_MAX_MM);
    dx = xt - x_start;
    dy = yt - y_start;

    float sx_f = dx * STEPS_PER_MM + rx_steps;
    float sy_f = dy * STEPS_PER_MM + ry_steps;
    long stepsX = lroundf(fabsf(sx_f));
    long stepsY = lroundf(fabsf(sy_f));

    if (stepsX == 0 && stepsY == 0) {
      rx_steps = sx_f;
      ry_steps = sy_f;
      delay(dt_ms);
      return;
    }

    long N = (stepsX > stepsY) ? stepsX : stepsY;
    bool xf = (sx_f >= 0.0f);
    bool yf = (sy_f >= 0.0f);
    axisX.setDirection(xf);
    axisY.setDirection(yf);

    float incx = (stepsX > 0) ? (float)stepsX / (float)N : 0.0f;
    float incy = (stepsY > 0) ? (float)stepsY / (float)N : 0.0f;
    float ex = 0.0f, ey = 0.0f;

    uint32_t per = (N > 0)
      ? max_u32((uint32_t)MIN_DWELL_US, (uint32_t)((dt_ms * 1000UL) / (uint32_t)N))
      : (uint32_t)MIN_DWELL_US;

    for (long i = 0; i < N; i++) {
      ex += incx;
      if (ex >= 1.0f) {
        ex -= 1.0f;
        if (xf) (void)axisX.stepForward();
        else    (void)axisX.stepReverse();
      }

      ey += incy;
      if (ey >= 1.0f) {
        ey -= 1.0f;
        if (yf) (void)axisY.stepForward();
        else    (void)axisY.stepReverse();
      }

      delayMicroseconds(per);
    }

    float x_end = axisX.mm();
    float y_end = axisY.mm();
    float ax = (x_end - x_start) * STEPS_PER_MM;
    float ay = (y_end - y_start) * STEPS_PER_MM;
    rx_steps = sx_f - ax;
    ry_steps = sy_f - ay;
  }

  void resetResiduals() {
    rx_steps = 0.0f;
    ry_steps = 0.0f;
  }
} ctrl;

/******** Slow point-to-point move helper ********/
void lineToXY_slow(float x, float y, int dwell_us = CENTER_DWELL_US) {
  x = constrain(x, SOFT_MIN_MM, SOFT_MAX_MM);
  y = constrain(y, SOFT_MIN_MM, SOFT_MAX_MM);

  float x_start = axisX.mm();
  float y_start = axisY.mm();
  float dx = x - x_start;
  float dy = y - y_start;

  long sx = lroundf(dx * STEPS_PER_MM);
  long sy = lroundf(dy * STEPS_PER_MM);
  long ax = labs(sx);
  long ay = labs(sy);
  long N  = (ax > ay) ? ax : ay;
  if (!N) return;

  bool xf = (sx >= 0);
  bool yf = (sy >= 0);
  float ex = 0.0f, ey = 0.0f;
  float incx = (ax > 0) ? (float)ax / (float)N : 0.0f;
  float incy = (ay > 0) ? (float)ay / (float)N : 0.0f;

  axisX.setDirection(xf);
  axisY.setDirection(yf);

  for (long i = 0; i < N; i++) {
    ex += incx;
    if (ex >= 1.0f) {
      ex -= 1.0f;
      if (xf) (void)axisX.stepForward();
      else    (void)axisX.stepReverse();
    }

    ey += incy;
    if (ey >= 1.0f) {
      ey -= 1.0f;
      if (yf) (void)axisY.stepForward();
      else    (void)axisY.stepReverse();
    }

    delayMicroseconds(dwell_us);
  }
}

void centerAxesReliably(float center_mm = TRACK_LEN_MM * 0.5f) {
  axisX.enable();
  axisY.enable();

  const float SEAT_MM = 5.0f;
  axisX.moveToMM_slow(min(SOFT_MAX_MM - LAND_MARGIN, axisX.mm() + SEAT_MM));
  axisY.moveToMM_slow(min(SOFT_MAX_MM - LAND_MARGIN, axisY.mm() + SEAT_MM));

  const float FAR = SOFT_MAX_MM - LAND_MARGIN;
  axisX.moveToMM_slow(FAR);
  axisY.moveToMM_slow(FAR);
  lineToXY_slow(center_mm, center_mm);
  axisX.moveToMM_slow(center_mm);
  axisY.moveToMM_slow(center_mm);
}

/******** Spirograph layer ********/
struct PathPoint {
  float x;
  float y;
};

const int MAX_PATTERN_POINTS = 1600;
PathPoint patternPath[MAX_PATTERN_POINTS];
int patternPointCount = 0;
int patternPointIndex = 0;

bool runSpiro = false;

// Calibrated sand-pit geometry.
float x_center = 39.0f;
float y_center = 37.5f;
float radius_mm = 36.0f;

// Pattern controls.
float period_s        = 28.0f;  // time for one outer loop
uint32_t motion_dt_ms = 40;
float patternScale    = 1.0f;
float spiroInnerFrac  = 0.22f;
float spiroFreq       = 5.0f;   // user-entered value

// Snapped rational used internally.
float spiroFreqUsed   = 5.0f;
int   spiroClosureNum = 5;      // p in p/q
int   spiroClosureDen = 1;      // q in p/q
bool  spiroClosureKnown = true;

/******** Utility functions ********/
static inline bool insideCircle(float x, float y, float margin = 0.0f) {
  float dx = x - x_center;
  float dy = y - y_center;
  return (dx * dx + dy * dy) <= sq(max(0.0f, radius_mm - margin));
}

void clampPointToAllowed(float &x, float &y, float margin = CIRCLE_GUARD_MM) {
  float dx = x - x_center;
  float dy = y - y_center;
  float limit = max(0.0f, radius_mm - margin);
  float d2 = dx * dx + dy * dy;

  if (d2 > limit * limit) {
    float d = sqrtf(d2);
    if (d > 1e-6f) {
      x = x_center + dx * limit / d;
      y = y_center + dy * limit / d;
    } else {
      x = x_center;
      y = y_center;
    }
  }

  x = constrain(x, SOFT_MIN_MM, SOFT_MAX_MM);
  y = constrain(y, SOFT_MIN_MM, SOFT_MAX_MM);
}

bool circleFits(float cx, float cy, float r) {
  if (r < 0.0f) return false;
  if (cx - r < SOFT_MIN_MM) return false;
  if (cx + r > SOFT_MAX_MM) return false;
  if (cy - r < SOFT_MIN_MM) return false;
  if (cy + r > SOFT_MAX_MM) return false;
  return true;
}

void approximateSimpleFraction(float x, int maxDen, float tol, int &num, int &den, bool &ok) {
  ok = false;
  num = 0;
  den = 1;

  float bestErr = 1e9f;
  int bestNum = 0;
  int bestDen = 1;

  for (int d = 1; d <= maxDen; ++d) {
    int n = (int)lroundf(x * (float)d);
    float approx = (float)n / (float)d;
    float err = fabsf(approx - x);
    if (err < bestErr) {
      bestErr = err;
      bestNum = n;
      bestDen = d;
    }
  }

  if (bestErr <= tol) {
    int a = abs(bestNum);
    int b = bestDen;
    while (b != 0) {
      int t = a % b;
      a = b;
      b = t;
    }
    int g = (a == 0) ? 1 : a;
    num = bestNum / g;
    den = bestDen / g;
    if (den < 1) den = 1;
    ok = true;
  }
}

void getSpiroClosure(float f, int &num, int &den, bool &ok) {
  // Snap to a nearby simple fraction with denominator up to 8.
  // This keeps the pattern finite and ensures it closes cleanly.
  approximateSimpleFraction(f, 8, 1.0e-3f, num, den, ok);

  // If the number is not close enough to a simple fraction, fall back to the
  // nearest integer. The pattern will still run, but with a simpler closure.
  if (!ok) {
    num = (int)lroundf(f);
    den = 1;
  }

  if (den < 1) den = 1;
}

void evalSpiroXY(float u, float rr, float &x, float &y) {
  // Simple two-frequency spirograph-like curve.
  //   a controls the main orbit.
  //   b controls the inner wobble.
  //   spiroFreqUsed controls the number of lobes / closure behaviour.
  float b = constrain(spiroInnerFrac, 0.0f, 0.90f);
  float a = 1.0f - b;

  x = x_center + rr * (a * cosf(u) + b * cosf(spiroFreqUsed * u));
  y = y_center + rr * (a * sinf(u) - b * sinf(spiroFreqUsed * u));
  clampPointToAllowed(x, y);
}

void buildSpiroPath() {
  patternPointCount = 0;
  patternPointIndex = 0;

  // Work out the rational approximation and closure length.
  int num, den;
  bool ok;
  getSpiroClosure(spiroFreq, num, den, ok);
  spiroClosureNum = num;
  spiroClosureDen = den;
  spiroClosureKnown = ok;
  spiroFreqUsed = (float)num / (float)den;

  float totalLoops = (float)spiroClosureDen;
  float uMax = 2.0f * PI * totalLoops;

  // Estimate how many sample points are needed for the requested run time.
  int est = (period_s > 0.1f && motion_dt_ms > 0)
    ? (int)lroundf((period_s * totalLoops * 1000.0f) / (float)motion_dt_ms)
    : 200;

  if (est < 80) est = 80;
  if (est > MAX_PATTERN_POINTS) est = MAX_PATTERN_POINTS;

  float rr = radius_mm * patternScale;
  for (int i = 0; i < est; ++i) {
    float frac = (est <= 1) ? 1.0f : (float)i / (float)(est - 1);
    float u = frac * uMax;
    evalSpiroXY(u, rr, patternPath[i].x, patternPath[i].y);
  }

  patternPointCount = est;

  Serial.println();
  Serial.println("=== Spiro path prepared ===");
  Serial.printf("Requested F = %.6f\n", spiroFreq);
  Serial.printf("Using      = %d/%d = %.6f\n", spiroClosureNum, spiroClosureDen, spiroFreqUsed);
  Serial.printf("Closure    = %d outer loop(s)\n", spiroClosureDen);
  Serial.printf("Points     = %d\n", patternPointCount);
  Serial.printf("Run time   = about %.1f s\n", period_s * totalLoops);
  Serial.println("===========================");
}

void printStatus() {
  Serial.println();
  Serial.println("--- Current status ---");
  Serial.printf("Position          : (%.2f, %.2f) mm\n", axisX.mm(), axisY.mm());
  Serial.printf("Pit centre/radius : (%.2f, %.2f), R = %.2f mm\n", x_center, y_center, radius_mm);
  Serial.printf("Pattern scale     : %.2f\n", patternScale);
  Serial.printf("Inner fraction I  : %.3f\n", spiroInnerFrac);
  Serial.printf("Requested F       : %.6f\n", spiroFreq);
  Serial.printf("Snapped F         : %d/%d = %.6f\n", spiroClosureNum, spiroClosureDen, spiroFreqUsed);
  Serial.printf("Loop period T     : %.2f s per outer loop\n", period_s);
  Serial.printf("Update interval D : %lu ms\n", (unsigned long)motion_dt_ms);
  Serial.printf("Running           : %s\n", runSpiro ? "yes" : "no");
  Serial.println("----------------------");
}

void stopMotion() {
  runSpiro = false;
  ctrl.resetResiduals();
  Serial.println("Motion stopped.");
  printStatus();
}

bool setCircle(float cx, float cy, float r) {
  if (!circleFits(cx, cy, r)) {
    Serial.printf("Rejected pit: centre=(%.2f, %.2f), radius=%.2f is outside %.1f..%.1f mm travel.\n",
                  cx, cy, r, SOFT_MIN_MM, SOFT_MAX_MM);
    return false;
  }

  x_center = cx;
  y_center = cy;
  radius_mm = r;
  Serial.println("Pit geometry updated.");
  printStatus();
  return true;
}

void goToXY(float x, float y) {
  lineToXY_slow(x, y);
  stopMotion();
}

void startSpiro() {
  buildSpiroPath();
  if (patternPointCount <= 0) {
    Serial.println("No path points were generated; refusing to start.");
    return;
  }

  // Move first to the first point so the pattern starts cleanly.
  lineToXY_slow(patternPath[0].x, patternPath[0].y);
  ctrl.resetResiduals();
  patternPointIndex = 0;
  runSpiro = true;

  Serial.println();
  Serial.println("Starting spiro pattern now.");
  Serial.printf("First point : (%.2f, %.2f) mm\n", patternPath[0].x, patternPath[0].y);
  Serial.printf("Last point  : (%.2f, %.2f) mm\n", patternPath[patternPointCount - 1].x, patternPath[patternPointCount - 1].y);
  Serial.printf("Closure     : %d loop(s), using %d/%d\n", spiroClosureDen, spiroClosureNum, spiroClosureDen);
}

void homeAndPrepare() {
  Serial.println("Homing both axes...");
  axisX.moveMM(+SAFE_START_MM);
  axisY.moveMM(+SAFE_START_MM);
  bool okX = axisX.home();
  bool okY = axisY.home();
  if (!(okX && okY)) {
    Serial.println("Homing failed. Check endstops, wiring, and direction settings.");
  }
  centerAxesReliably(TRACK_LEN_MM * 0.5f);
  runSpiro = false;
  ctrl.resetResiduals();
  Serial.println("Axes homed and re-centred.");
  printStatus();
}

void motionTick() {
  if (!runSpiro) return;

  if (patternPointCount <= 0) {
    Serial.println("Pattern buffer is empty; stopping.");
    stopMotion();
    return;
  }

  if (patternPointIndex >= patternPointCount) {
    runSpiro = false;
    Serial.println();
    Serial.printf("Spiro complete. Stopped at final point after %d loop(s), using %d/%d = %.6f.\n",
                  spiroClosureDen, spiroClosureNum, spiroClosureDen, spiroFreqUsed);
    printStatus();
    return;
  }

  float xt = patternPath[patternPointIndex].x;
  float yt = patternPath[patternPointIndex].y;
  patternPointIndex++;

  clampPointToAllowed(xt, yt);

  float dx = xt - axisX.mm();
  float dy = yt - axisY.mm();
  ctrl.moveVectorMM(dx, dy, motion_dt_ms);

  // Safety clamp in case cumulative motion ever drifts just outside the pit.
  float xc = axisX.mm();
  float yc = axisY.mm();
  if (!insideCircle(xc, yc, CIRCLE_GUARD_MM)) {
    float xr = xc;
    float yr = yc;
    clampPointToAllowed(xr, yr);
    lineToXY_slow(xr, yr);
    ctrl.resetResiduals();
    Serial.println("Guard: motion was nudged back inside the approved circular pit.");
  }
}

void printHelp() {
  Serial.println();
  Serial.println("Available commands:");
  Serial.println("  R              Run the current spiro pattern");
  Serial.println("  S              Stop motion immediately");
  Serial.println("  H              Re-home both axes and re-centre");
  Serial.println("  P              Print current status");
  Serial.println("  ?              Show this help message");
  Serial.println();
  Serial.println("  M x y          Move to an exact XY position in mm and stop");
  Serial.println("  C x y r        Set sand-pit centre and radius in mm");
  Serial.println("  T seconds      Set time for one outer loop of the spiro pattern");
  Serial.println("  D ms           Set update interval (10..200 ms)");
  Serial.println("  K scale        Set pattern scale (0.05 .. 1.00)");
  Serial.println("  I frac         Set spiro inner fraction (0.00 .. 0.90)");
  Serial.println("  F value        Set spiro frequency (> 0, snapped to a simple fraction)\n");

  Serial.println("Examples:");
  Serial.println("  I 0.22");
  Serial.println("  F 2.5");
  Serial.println("  T 32");
  Serial.println("  K 0.85");
  Serial.println("  R\n");

  Serial.println("More challenging examples:");
  Serial.println("  F 1.625   -> usually snaps to 13/8, so it closes after 8 loops");
  Serial.println("  F 1.1667  -> usually snaps to 7/6, so it closes after 6 loops");
}

void handleSerial() {
  if (!Serial.available()) return;

  String line = Serial.readStringUntil('\n');
  line.trim();
  if (!line.length()) return;

  if (line.equalsIgnoreCase("R")) { startSpiro(); return; }
  if (line.equalsIgnoreCase("S")) { stopMotion(); return; }
  if (line.equalsIgnoreCase("H")) { homeAndPrepare(); return; }
  if (line.equalsIgnoreCase("P")) { printStatus(); return; }
  if (line.equalsIgnoreCase("?")) { printHelp(); return; }

  float a, b, c;
  if (sscanf(line.c_str(), "M %f %f", &a, &b) == 2 ||
      sscanf(line.c_str(), "m %f %f", &a, &b) == 2) {
    goToXY(a, b);
    return;
  }

  if (sscanf(line.c_str(), "C %f %f %f", &a, &b, &c) == 3 ||
      sscanf(line.c_str(), "c %f %f %f", &a, &b, &c) == 3) {
    setCircle(a, b, c);
    return;
  }

  float t;
  if (sscanf(line.c_str(), "T %f", &t) == 1 ||
      sscanf(line.c_str(), "t %f", &t) == 1) {
    if (t > 1.0f) {
      period_s = t;
      Serial.printf("Outer-loop period set to %.2f s.\n", period_s);
    } else {
      Serial.println("T must be greater than 1 second.");
    }
    return;
  }

  int d;
  if (sscanf(line.c_str(), "D %d", &d) == 1 ||
      sscanf(line.c_str(), "d %d", &d) == 1) {
    if (d >= 10 && d <= 200) {
      motion_dt_ms = (uint32_t)d;
      Serial.printf("Update interval set to %lu ms.\n", (unsigned long)motion_dt_ms);
    } else {
      Serial.println("D must be between 10 and 200 ms.");
    }
    return;
  }

  float k;
  if (sscanf(line.c_str(), "K %f", &k) == 1 ||
      sscanf(line.c_str(), "k %f", &k) == 1) {
    if (k > 0.05f && k <= 1.0f) {
      patternScale = k;
      Serial.printf("Pattern scale set to %.2f.\n", patternScale);
    } else {
      Serial.println("K must be between 0.05 and 1.00.");
    }
    return;
  }

  float iv;
  if (sscanf(line.c_str(), "I %f", &iv) == 1 ||
      sscanf(line.c_str(), "i %f", &iv) == 1) {
    if (iv >= 0.0f && iv <= 0.90f) {
      spiroInnerFrac = iv;
      Serial.printf("Inner fraction set to %.3f.\n", spiroInnerFrac);
    } else {
      Serial.println("I must be between 0.00 and 0.90.");
    }
    return;
  }

  float fv;
  if (sscanf(line.c_str(), "F %f", &fv) == 1 ||
      sscanf(line.c_str(), "f %f", &fv) == 1) {
    if (fv > 0.0f && fv <= 20.0f) {
      spiroFreq = fv;
      int num, den;
      bool ok;
      getSpiroClosure(spiroFreq, num, den, ok);
      Serial.printf("Requested F = %.6f\n", spiroFreq);
      Serial.printf("Nearest simple fraction = %d/%d = %.6f\n", num, den, (float)num / (float)den);
      Serial.printf("This will close after %d outer loop(s).\n", den);
      if (!ok) {
        Serial.println("Note: that value was not very close to a simple fraction, so it was rounded more aggressively.");
      }
    } else {
      Serial.println("F must be greater than 0 and no more than 20.");
    }
    return;
  }

  Serial.println("Unknown command.");
  printHelp();
}

/******** Setup / loop ********/
void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println();
  Serial.println("Zen garden spirograph sketch");
  Serial.println("This simplified distribution version keeps only the spirograph mode.");

  axisX.begin();
  axisY.begin();
  homeAndPrepare();

  // Initialise the snapped frequency information for status printing.
  getSpiroClosure(spiroFreq, spiroClosureNum, spiroClosureDen, spiroClosureKnown);
  spiroFreqUsed = (float)spiroClosureNum / (float)spiroClosureDen;

  printHelp();
  Serial.println();
  Serial.println("Default pit geometry: C 39 37.5 36");
  Serial.println("Suggested gentle test: I 0.22, F 2.5, T 32, K 0.85, R");
}

void loop() {
  handleSerial();
  motionTick();
}
