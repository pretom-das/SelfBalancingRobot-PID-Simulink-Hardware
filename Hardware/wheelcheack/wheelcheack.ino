/* ---------------------------------------------------------
   INTERACTIVE MOTOR JOG (UNO + L298N)
   Pins (same as your project):
     RIGHT: ENA=5 (PWM), IN1=7, IN2=8   -> OUT1/OUT2
     LEFT : ENB=6 (PWM), IN3=12, IN4=13 -> OUT3/OUT4

   Serial @ 115200. Commands (case-insensitive):
     RF [p]  -> Right  Forward  with optional PWM (0..255)
     RB [p]  -> Right  Backward
     RC      -> Right  Coast (freewheel)
     RK      -> Right  Brake (short brake)

     LF [p]  -> Left   Forward
     LB [p]  -> Left   Backward
     LC      -> Left   Coast
     LK      -> Left   Brake

     AF [p]  -> BOTH Forward
     AB [p]  -> BOTH Backward
     AC      -> BOTH Coast
     AK      -> BOTH Brake

     P [p]   -> Set default PWM (used when cmd has no [p])
     + / -   -> Increase / decrease default PWM by 10
     ?       -> Help
     X       -> All stop (brake)
   Examples:
     RF 180   (right fwd at 180)
     LB       (left back at default PWM)
     AK       (both brake)
     P 200    (set default PWM = 200)
--------------------------------------------------------- */

#include <Arduino.h>

// ---- Pins
const int ENA = 5;     // RIGHT PWM
const int IN1 = 7;
const int IN2 = 8;
const int ENB = 6;     // LEFT PWM
const int IN3 = 12;
const int IN4 = 13;

// ---- Defaults
int defaultPWM = 180;    // used if command omits [p]
int deadband   = 30;     // minimum kick to overcome friction (applied when pwm>0)

// ---- Helpers
void rightDrive(int dir, int pwm) {
  pwm = constrain(pwm, 0, 255);
  if (dir > 0)      { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);  }
  else if (dir < 0) { digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); }
  else              { digitalWrite(IN1, LOW);  digitalWrite(IN2, LOW);  } // coast
  if (pwm > 0 && pwm < deadband) pwm = deadband;
  analogWrite(ENA, pwm);
}
void leftDrive(int dir, int pwm) {
  pwm = constrain(pwm, 0, 255);
  if (dir > 0)      { digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);  }
  else if (dir < 0) { digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); }
  else              { digitalWrite(IN3, LOW);  digitalWrite(IN4, LOW);  } // coast
  if (pwm > 0 && pwm < deadband) pwm = deadband;
  analogWrite(ENB, pwm);
}
void rightBrake(){ digitalWrite(IN1, HIGH); digitalWrite(IN2, HIGH); analogWrite(ENA, 0); }
void leftBrake() { digitalWrite(IN3, HIGH); digitalWrite(IN4, HIGH); analogWrite(ENB, 0); }

void allStopBrake(){ rightBrake(); leftBrake(); }
void allCoast()    { rightDrive(0,0); leftDrive(0,0); }

void printHUD(const char* msg){
  Serial.print(F("[PWM=")); Serial.print(defaultPWM);
  Serial.print(F(", dead=")); Serial.print(deadband);
  Serial.print(F("]  "));
  Serial.println(msg);
}

void help(){
  Serial.println(F("\nCommands:"));
  Serial.println(F("  RF [p], RB [p], RC, RK   -> Right motor Fwd/Back/Coast/Brake"));
  Serial.println(F("  LF [p], LB [p], LC, LK   -> Left  motor Fwd/Back/Coast/Brake"));
  Serial.println(F("  AF [p], AB [p], AC, AK   -> BOTH  motors Fwd/Back/Coast/Brake"));
  Serial.println(F("  P [p]    -> set default PWM (0..255)"));
  Serial.println(F("  + / -    -> inc/dec default PWM by 10"));
  Serial.println(F("  D [v]    -> set deadband (0..80), applied when pwm>0"));
  Serial.println(F("  X        -> ALL STOP (brake)"));
  Serial.println(F("  ?        -> this help\n"));
}

// Trim spaces and uppercase
String cleanUpper(String s){
  s.trim();
  for (unsigned i=0;i<s.length();++i) s[i] = toupper(s[i]);
  return s;
}

// Parse optional integer after a command (e.g., 'RF 180')
bool parseIntAfter(const String& s, int& outVal){
  // find first space
  int sp = s.indexOf(' ');
  if (sp < 0) return false;
  String t = s.substring(sp+1);
  t.trim();
  if (t.length()==0) return false;
  outVal = t.toInt();
  return true;
}

void setup(){
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);

  allCoast();

  Serial.begin(115200);
  delay(300);
  Serial.println(F("\n=== INTERACTIVE MOTOR JOG ==="));
  Serial.println(F("Type '?' for help. Place robot on a stand!"));
  printHUD("Ready.");
}

void loop(){
  // quick +/- shortcuts without newline
  if (Serial.available()){
    // Peek: if single-char control, handle directly
    int c = Serial.peek();
    if (c=='+' || c=='-'){
      Serial.read();
      defaultPWM = constrain(defaultPWM + (c=='+'? +10 : -10), 0, 255);
      printHUD(c=='+' ? "PWM +10" : "PWM -10");
      return;
    }
  }

  if (Serial.available()){
    String line = Serial.readStringUntil('\n');
    line = cleanUpper(line);
    if (line.length()==0) return;

    // single-char commands
    if (line=="X"){ allStopBrake(); printHUD("ALL STOP (brake)"); return; }
    if (line=="?"){ help(); printHUD("OK"); return; }

    // deadband set: D [v]
    if (line.startsWith("D")){
      int v;
      if (parseIntAfter(line, v)){
        deadband = constrain(v, 0, 80);
        printHUD("Deadband set");
      } else {
        Serial.println(F("Usage: D 30   (0..80)"));
      }
      return;
    }

    // PWM set: P [p]
    if (line.startsWith("P")){
      int p;
      if (parseIntAfter(line, p)){
        defaultPWM = constrain(p, 0, 255);
        printHUD("Default PWM set");
      } else {
        Serial.println(F("Usage: P 200   (0..255)"));
      }
      return;
    }

    // Motor commands
    // Pattern: [R|L|A][F|B|C|K] [p]
    if (line.length() >= 2){
      char side = line[0];
      char act  = line[1];
      int p; bool hasP = parseIntAfter(line, p);
      int pwm = hasP ? constrain(p,0,255) : defaultPWM;
      if (pwm>0 && pwm<deadband) pwm = deadband;

      auto doSide = [&](char s, char a){
        if (s=='R'){
          if (a=='F') { rightDrive(+1, pwm); printHUD("Right FORWARD"); }
          else if (a=='B') { rightDrive(-1, pwm); printHUD("Right BACKWARD"); }
          else if (a=='C') { rightDrive(0, 0);    printHUD("Right COAST"); }
          else if (a=='K') { rightBrake();        printHUD("Right BRAKE"); }
          else { Serial.println(F("Unknown action. Use F/B/C/K.")); }
        } else if (s=='L'){
          if (a=='F') { leftDrive(+1, pwm); printHUD("Left FORWARD"); }
          else if (a=='B') { leftDrive(-1, pwm); printHUD("Left BACKWARD"); }
          else if (a=='C') { leftDrive(0, 0);    printHUD("Left COAST"); }
          else if (a=='K') { leftBrake();        printHUD("Left BRAKE"); }
          else { Serial.println(F("Unknown action. Use F/B/C/K.")); }
        } else if (s=='A'){
          if (a=='F') { rightDrive(+1, pwm); leftDrive(+1, pwm); printHUD("Both FORWARD"); }
          else if (a=='B') { rightDrive(-1, pwm); leftDrive(-1, pwm); printHUD("Both BACKWARD"); }
          else if (a=='C') { allCoast();   printHUD("Both COAST"); }
          else if (a=='K') { allStopBrake(); printHUD("Both BRAKE"); }
          else { Serial.println(F("Unknown action. Use F/B/C/K.")); }
        } else {
          Serial.println(F("Unknown side. Use R/L/A."));
        }
      };

      doSide(side, act);
    } else {
      Serial.println(F("Bad command. Use '?' for help."));
    }
  }
}
