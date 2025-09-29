/*  ---------------------------------------------------------
    SELF-BALANCING ROBOT (UNO + MPU6050 + L298N + 1.3" SH1106 I2C)
    - SH1106 OLED status (I2C)
    - Complementary filter for pitch angle (gyro+accel)
    - Relay auto-tune (Åström–Hägglund) → ZN PID gains
    - EEPROM store/recall of tuned gains
    - L298N dual-motor driver
    - Fixed setpoint (SP_DEG) measured separately
    --------------------------------------------------------- */

#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <math.h>

// ---------------- OLED (SH1106 128x64 I2C) ----------------
#define OLED_ADDR 0x3C
Adafruit_SH1106G display(128, 64, &Wire, -1);

// ---------------- L298N Pins ----------------
const int ENA = 5;     // PWM
const int IN1 = 7;
const int IN2 = 8;
const int ENB = 6;     // PWM
const int IN3 = 12;
const int IN4 = 13;

// ---------------- Fixed setpoint ------------
const float SP_DEG = 0.0f;  // <<< REPLACE with your measured upright angle (deg)

// ---------------- MPU6050 -------------------
const uint8_t MPU_ADDR = 0x68;   // AD0 low
const float GYRO_SENS = 131.0f;  // LSB/(°/s) @ ±250 dps
const float ACC_SENS  = 16384.0f;// LSB/g @ ±2g

float gyroX_off=0, gyroY_off=0, gyroZ_off=0;
float accX_off=0,  accY_off=0,  accZ_off=0;

// ------------- Complementary Filter ---------
float pitch_deg = 0.0f;
float alpha = 0.98f;             // gyro weight
unsigned long lastMicros = 0;
float loop_dt = 0.005f;          // seconds, updated each loop

// ---------------- PID -----------------------
volatile float setpoint = SP_DEG;     // fixed upright
float Kp = 18.0f, Ki = 0.0f, Kd = 0.6f; // defaults if no EEPROM
float i_term = 0.0f, prev_err = 0.0f;
float out_min = -255, out_max = 255;

// ------------- Auto-tune (relay) ------------
enum Mode { MODE_AUTOTUNE, MODE_RUN } mode = MODE_AUTOTUNE;
const float relay_h = 80.0f;          // PWM magnitude for relay drive (tweak 60–120)
const float tune_window = 2.0f;       // deg hysteresis around setpoint
const unsigned long settle_ms = 3000; // let oscillation stabilize
float max_angle = -1e9, min_angle = 1e9;
unsigned long last_cross_ms = 0;
unsigned long period_ms = 0;
int cross_count = 0;
bool settled = false;

// --------------- EEPROM ---------------------
struct Gains { float Kp, Ki, Kd; uint32_t magic; };
const uint32_t MAGIC = 0xBABA00F1;
const int EE_ADDR = 0;

// --------------- Utils ----------------------
float constrainf(float x, float a, float b){ return x<a? a : (x>b? b : x); }

// --------------- I2C helpers ----------------
void mpuWrite(uint8_t reg, uint8_t val){
  Wire.beginTransmission(MPU_ADDR); Wire.write(reg); Wire.write(val); Wire.endTransmission();
}
void mpuRead(uint8_t reg, uint8_t* buf, uint8_t len){
  Wire.beginTransmission(MPU_ADDR); Wire.write(reg); Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, len, true);
  for(uint8_t i=0;i<len && Wire.available();++i) buf[i]=Wire.read();
}

// --------------- MPU init -------------------
void mpuInit(){
  mpuWrite(0x6B, 0x00);   // PWR_MGMT_1: wake
  mpuWrite(0x1B, 0x00);   // GYRO_CONFIG: ±250 dps
  mpuWrite(0x1C, 0x00);   // ACCEL_CONFIG: ±2g
  delay(100);
}

// Keep robot still during this (~1s)
void mpuCalibrate(){
  const int N=1000;
  long gx=0, gy=0, gz=0, ax=0, ay=0, az=0;
  uint8_t raw[14];
  for(int i=0;i<N;i++){
    mpuRead(0x3B, raw, 14);
    int16_t axr = (raw[0]<<8)|raw[1];
    int16_t ayr = (raw[2]<<8)|raw[3];
    int16_t azr = (raw[4]<<8)|raw[5];
    int16_t gxr = (raw[8]<<8)|raw[9];
    int16_t gyr = (raw[10]<<8)|raw[11];
    int16_t gzr = (raw[12]<<8)|raw[13];
    ax += axr; ay += ayr; az += azr;
    gx += gxr; gy += gyr; gz += gzr;
    delay(1);
  }
  accX_off = (float)ax/N;
  accY_off = (float)ay/N;
  accZ_off = (float)az/N - ACC_SENS; // ~+1g on Z
  gyroX_off= (float)gx/N;
  gyroY_off= (float)gy/N;
  gyroZ_off= (float)gz/N;
}

// --------------- Sensor + angle -------------
void updateAngle(){
  uint8_t raw[14];
  mpuRead(0x3B, raw, 14);
  int16_t axr = (raw[0]<<8)|raw[1];
  int16_t ayr = (raw[2]<<8)|raw[3];
  int16_t azr = (raw[4]<<8)|raw[5];
  int16_t gyr = (raw[10]<<8)|raw[11];

  float axg = (axr - accX_off)/ACC_SENS;
  float ayg = (ayr - accY_off)/ACC_SENS;
  float azg = (azr - accZ_off)/ACC_SENS;

  // Pitch from accel (degrees). Assume X forward, Y left, Z up
  float accPitch = atan2f(axg, sqrtf(ayg*ayg + azg*azg)) * 180.0f/PI;
  float gyroY_dps = (gyr - gyroY_off)/GYRO_SENS;

  unsigned long now = micros();
  float dt = (lastMicros==0) ? 0.005f : (now - lastMicros)*1e-6f;
  if (dt < 0.0005f) dt = 0.0005f; // clamp to tame spikes
  if (dt > 0.02f)   dt = 0.02f;
  lastMicros = now;
  loop_dt = dt;

  // Complementary filter
  pitch_deg = alpha*(pitch_deg + gyroY_dps*dt) + (1.0f - alpha)*accPitch;
}

// --------------- Motors (L298N) -------------
void setMotorPWM(float u){
  // u in [-255, 255]
  int pwm = (int)fabs(u);
  pwm = constrain(pwm, 0, 255);

  if (u >= 0){
    // forward
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  } else {
    // backward
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  }

  // Overcome friction deadband
  if (pwm > 0 && pwm < 30) pwm = 30;

  analogWrite(ENA, pwm);
  analogWrite(ENB, pwm);
}

// ---------------- PID (time-based) ----------
float pidCompute(float angle){
  // continuous-time ZN gains
  static float d_filt = 0.0f;       // filtered derivative
  const float tau_d = 0.02f;         // derivative low-pass (~20 ms)
  const float dt = loop_dt;

  float err = setpoint - angle;

  // integral with anti-windup
  i_term += Ki * err * dt;
  i_term = constrainf(i_term, -300.0f, 300.0f);

  // derivative
  float derr = (err - prev_err) / (dt > 1e-6f ? dt : 1e-6f);
  float alpha_d = tau_d / (tau_d + dt);
  d_filt = alpha_d * d_filt + (1.0f - alpha_d) * derr;

  float u = Kp * err + i_term + Kd * d_filt;
  prev_err = err;
  return constrainf(u, out_min, out_max);
}

// ----- Auto-tune helpers & step function ----
void resetAutotuneState(){
  max_angle = -1e9; min_angle = 1e9;
  last_cross_ms = 0;
  period_ms = 0;
  cross_count = 0;
  settled = false;
}

bool autotuneStep(){
  float err = setpoint - pitch_deg;

  // Relay with hysteresis to avoid chatter
  static int state = 0; // -1 or +1
  if (state == 0) state = (err >= 0) ? +1 : -1;
  if (state == +1 && err < -tune_window) state = -1;
  if (state == -1 && err > +tune_window) state = +1;

  float u = (state >= 0) ? +relay_h : -relay_h;
  setMotorPWM(u);

  // Allow settling
  static unsigned long start_ms = millis();
  if (!settled && (millis() - start_ms) > settle_ms){
    settled = true;
    last_cross_ms = millis();
  }
  if (!settled) return false;

  // Track peaks within half-cycles
  if (pitch_deg > max_angle) max_angle = pitch_deg;
  if (pitch_deg < min_angle) min_angle = pitch_deg;

  // Detect state change = half-period
  static int last_state = state;
  if (state != last_state){
    unsigned long now = millis();
    unsigned long half_period = now - last_cross_ms;
    last_cross_ms = now;
    last_state = state;
    cross_count++;

    if (cross_count >= 4){ // enough samples
      period_ms = 2 * half_period; // ~full Pu
      float a = 0.5f * (max_angle - min_angle);
      if (a < 0.5f) a = 0.5f; // avoid tiny amplitude

      float Ku = (4.0f * relay_h) / (PI * a);
      float Pu = period_ms / 1000.0f;

      // Ziegler–Nichols PID (classic)
      float newKp = 0.6f * Ku;
      float newKi = 2.0f * newKp / Pu;
      float newKd = newKp * Pu / 8.0f;

      if (isfinite(newKp) && isfinite(newKi) && isfinite(newKd) &&
          newKp>0 && newKi>=0 && newKd>=0 && Pu>0.05f && Pu<5.0f){
        Kp = constrainf(newKp, 1.0f, 120.0f);
        Ki = constrainf(newKi, 0.0f, 500.0f);
        Kd = constrainf(newKd, 0.0f, 60.0f);
        Gains g{Kp,Ki,Kd,MAGIC};
        EEPROM.put(EE_ADDR, g);
        return true; // tuning done
      } else {
        // retry: reset peaks/counter
        max_angle = -1e9; min_angle = 1e9; cross_count = 0;
      }
    }
    // reset peaks for next half-cycle
    max_angle = -1e9; min_angle = 1e9;
  }
  return false;
}

// ---------------- OLED HUD ------------------
void drawOLED(const char* top){
  display.clearDisplay();
  display.setTextColor(SH110X_WHITE);

  display.setTextSize(1);
  display.setCursor(0,0);
  display.print(top);

  display.setCursor(0,14);
  display.print("Ang: "); display.print(pitch_deg,1); display.print(" deg");

  display.setCursor(0,28);
  display.print("Kp: "); display.print(Kp,1);
  display.setCursor(64,28);
  display.print("Ki: "); display.print(Ki,1);

  display.setCursor(0,42);
  display.print("Kd: "); display.print(Kd,1);

  display.setCursor(0,56);
  display.print("SP: "); display.print(setpoint,1);

  // simple balance bar
  int cx=100, cy=40, w=24, h=8;
  display.drawRect(cx, cy, w, h, SH110X_WHITE);
  float p = constrainf((pitch_deg+15.0f)/30.0f, 0.0f, 1.0f);
  int filled = (int)(p*(w-2));
  display.fillRect(cx+1, cy+1, filled, h-2, SH110X_WHITE);

  display.display();
}

// ---------------- Setup ---------------------
void setup(){
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  analogWrite(ENA, 0); analogWrite(ENB, 0);

  Wire.begin();             // UNO: SDA=A4, SCL=A5
  Wire.setClock(100000);    // keep slow & stable
  mpuInit();

  if (!display.begin(OLED_ADDR, true)) { /* if needed, try 0x3D */ }
  display.clearDisplay(); display.display();

  // Load saved gains if present
  Gains g; EEPROM.get(EE_ADDR, g);
  if (g.magic == MAGIC && isfinite(g.Kp) && isfinite(g.Ki) && isfinite(g.Kd)){
    Kp=g.Kp; Ki=g.Ki; Kd=g.Kd; mode = MODE_RUN;
  } else {
    mode = MODE_AUTOTUNE;
  }

  drawOLED("Calibrating...");
  mpuCalibrate();           // keep robot still
  lastMicros = micros();

  Serial.begin(115200);
}

// ---------------- Loop ----------------------
void loop(){
  updateAngle();

  // Tilt safety cutout (prevents thrashing when it falls)
  if (fabs(pitch_deg) > 35.0f) {
    i_term = 0; prev_err = 0;
    setMotorPWM(0);
    drawOLED("TILT SAFETY");
    delay(10);
    return;
  }

  if (mode == MODE_AUTOTUNE){
    bool done = autotuneStep();
    drawOLED("Auto-tuning...");
    if (done){
      setMotorPWM(0);
      delay(500);
      i_term=0; prev_err=0;
      mode = MODE_RUN;
      drawOLED("Tuned & saved");
      delay(400);
    }
  } else { // MODE_RUN
    float u = pidCompute(pitch_deg);
    setMotorPWM(u);
    drawOLED("BALANCING");
  }
}
