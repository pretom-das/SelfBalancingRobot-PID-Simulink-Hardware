// UPRIGHT ANGLE FINDER (UNO + MPU6050)
// Put the robot in the exact upright posture you want -> hold still for ~5s.
// This prints the averaged pitch angle in degrees to paste into your main code.
//
// Notes:
// - Uses accel-only angle (robust when static).
// - Axis convention: X forward, Y left, Z up (matches your main sketch).
// - If your board is rotated, the sign may need flipping in your main code.

#include <Wire.h>
const uint8_t MPU_ADDR = 0x68;
const float ACC_SENS = 16384.0f; // LSB/g @ ±2g

void mpuWrite(uint8_t r, uint8_t v){
  Wire.beginTransmission(MPU_ADDR); Wire.write(r); Wire.write(v); Wire.endTransmission();
}
void mpuRead(uint8_t r, uint8_t* b, uint8_t n){
  Wire.beginTransmission(MPU_ADDR); Wire.write(r); Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, n, true);
  for(uint8_t i=0;i<n && Wire.available();++i) b[i]=Wire.read();
}

void setup(){
  Serial.begin(115200);
  Wire.begin();     // UNO: SDA=A4, SCL=A5
  Wire.setClock(100000);
  delay(100);
  // Wake & set ranges
  mpuWrite(0x6B, 0x00);  // PWR_MGMT_1: wake
  mpuWrite(0x1C, 0x00);  // ACCEL_CONFIG: ±2g

  Serial.println(F("\n--- UPRIGHT ANGLE FINDER ---"));
  Serial.println(F("Place robot in desired upright position and keep it still..."));
  delay(1500);

  const int N = 1500; // ~5s at 200 Hz loop below
  double sumPitch = 0.0;

  for(int i=0;i<N;i++){
    uint8_t raw[6];
    mpuRead(0x3B, raw, 6); // ax, ay, az
    int16_t ax = (raw[0]<<8)|raw[1];
    int16_t ay = (raw[2]<<8)|raw[3];
    int16_t az = (raw[4]<<8)|raw[5];

    float axg = ax/ACC_SENS, ayg = ay/ACC_SENS, azg = az/ACC_SENS;

    // Pitch from accel (deg): atan2(X, sqrt(Y^2+Z^2))
    float accPitch = atan2f(axg, sqrtf(ayg*ayg + azg*azg)) * 180.0f/PI;

    sumPitch += accPitch;

    if (i % 100 == 0) {
      Serial.print(F("Live pitch: "));
      Serial.print(accPitch, 2);
      Serial.println(F(" deg"));
    }
    delay(5); // ~200 Hz sampling
  }

  float upright_deg = sumPitch / N;
  Serial.println(F("\n=== RESULT ==="));
  Serial.print(F("Upright pitch (deg): "));
  Serial.println(upright_deg, 3);
  Serial.println(F("\nCopy this line into your main sketch:"));
  Serial.print(F("const float SP_DEG = "));
  Serial.print(upright_deg, 3);
  Serial.println(F(";  // paste into main code"));
}

void loop(){ /* done */ }
