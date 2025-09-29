#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "MPU6050.h"

// OLED
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// Wi-Fi
const char* ssid = "Robot_WiFi";
const char* password = "12345678";
WebServer server(80);

// MPU6050
MPU6050 mpu;
float accY, accZ, gyroX;
float accAngle, gyroRate, currentAngle, prevAngle = 0;
float elapsedTime, lastTime;

// PID
float Kp = 30, Ki = 0, Kd = 1;
float error, prevError = 0, integral = 0;
float setpoint = 0;
int motorPower;

// Movement bias (joystick)
int moveBias = 0;  
int turnBias = 0;  

// L298N pins
#define ENA 25
#define IN1 26
#define IN2 27
#define ENB 32
#define IN3 33
#define IN4 15

#define PWM_FREQ 1000
#define PWM_RES 8

// ---------------- Motor Control ----------------
void setMotors(int left, int right){
  left = constrain(left, -255, 255);
  right = constrain(right, -255, 255);

  // Left motor
  if(left >= 0){
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    ledcWrite(0, left);
  }else{
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    ledcWrite(0, -left);
  }

  // Right motor
  if(right >= 0){
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    ledcWrite(1, right);
  }else{
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    ledcWrite(1, -right);
  }
}

// ---------------- OLED Face ----------------
void updateOLED(float angle){
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);

  if(abs(angle)<5){
    display.setCursor(30,20); display.println(":)");
  }else if(angle>5){
    display.setCursor(30,20); display.println(":O");
  }else{
    display.setCursor(30,20); display.println(":/");
  }
  display.display();
}

// ---------------- Balance PID ----------------
void balanceControl(){
  accY = mpu.getAccelerationY();
  accZ = mpu.getAccelerationZ();
  gyroX = mpu.getRotationX();

  accAngle = atan2(accY, accZ)*180/PI;
  gyroRate = (float)gyroX / 131.0;

  float now = millis();
  elapsedTime = (now - lastTime)/1000.0;
  lastTime = now;

  currentAngle = 0.98*(prevAngle + gyroRate*elapsedTime) + 0.02*accAngle;

  error = currentAngle - setpoint;
  integral += error*elapsedTime;
  float derivative = (error-prevError)/elapsedTime;
  prevError = error;

  motorPower = Kp*error + Ki*integral + Kd*derivative;

  int leftMotor = motorPower + moveBias - turnBias;
  int rightMotor = motorPower + moveBias + turnBias;

  setMotors(leftMotor, rightMotor);
  updateOLED(currentAngle);

  prevAngle = currentAngle;
}

// ---------------- Web Page ----------------
String htmlPage(){
  return R"rawliteral(
<html><head><title>ESP32 Robot Joystick</title>
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<style>
body{font-family:Arial;text-align:center;background:#222;color:#fff;}
#joy{width:200px;height:200px;margin:50px auto;background:#444;border-radius:50%;position:relative;}
#stick{width:80px;height:80px;background:#0f0;border-radius:50%;position:absolute;top:60px;left:60px;}
</style></head>
<body>
<h2>ESP32 Robot Joystick</h2>
<div id="joy"><div id="stick"></div></div>
<script>
let joy=document.getElementById("joy");
let stick=document.getElementById("stick");
let joyRect=joy.getBoundingClientRect();
let joyX=joyRect.left+joyRect.width/2;
let joyY=joyRect.top+joyRect.height/2;
joy.addEventListener("touchmove",function(e){
let touch=e.touches[0];
let dx=touch.clientX-joyX;
let dy=touch.clientY-joyY;
let dist=Math.min(Math.sqrt(dx*dx+dy*dy),80);
let angle=Math.atan2(dy,dx);
let x=dist*Math.cos(angle);
let y=dist*Math.sin(angle);
stick.style.transform="translate("+x+"px,"+y+"px)";
let px=Math.round((x/80)*100);
let py=Math.round((-y/80)*100);
fetch(`/control?x=${px}&y=${py}`);
});
joy.addEventListener("touchend",function(){
stick.style.transform="translate(0px,0px)";
fetch("/control?x=0&y=0");
});
</script>
</body></html>
)rawliteral";
}

// ---------------- Web Handlers ----------------
void handleRoot(){server.send(200,"text/html",htmlPage());}

void handleControl(){
  if(server.hasArg("x")){
    int x = server.arg("x").toInt();
    int y = server.arg("y").toInt();
    moveBias = map(y,-100,100,-100,100);
    turnBias = map(x,-100,100,-80,80);
  }
  server.send(200,"text/plain","OK");
}

// ---------------- Setup ----------------
void setup(){
  Serial.begin(115200);
  Wire.begin();

  // OLED
  if(!display.begin(SSD1306_SWITCHCAPVCC,0x3C)){Serial.println("OLED failed");while(1);}
  display.clearDisplay();

  // MPU
  mpu.initialize();
  if(!mpu.testConnection()){Serial.println("MPU6050 failed");while(1);}

  // Motors
  pinMode(IN1,OUTPUT); pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT); pinMode(IN4,OUTPUT);
  ledcSetup(0,PWM_FREQ,PWM_RES); ledcAttachPin(ENA,0);
  ledcSetup(1,PWM_FREQ,PWM_RES); ledcAttachPin(ENB,1);

  // Wi-Fi
  WiFi.softAP(ssid,password);
  Serial.print("AP IP: "); Serial.println(WiFi.softAPIP());
  server.on("/",handleRoot);
  server.on("/control",handleControl);
  server.begin();

  lastTime = millis();
}

// ---------------- Loop ----------------
void loop(){
  server.handleClient();
  balanceControl();
}
