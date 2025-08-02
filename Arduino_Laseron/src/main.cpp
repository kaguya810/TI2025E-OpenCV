#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// OLED参数
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// 引脚定义
#define LASER_PIN 2
#define BUTTON1_PIN A0   // 低电平有效 -> 发送 "start1"
#define BUTTON2_PIN A1   // 高电平有效 -> 发送 "start2"

// 全局变量
bool laserState = false;       
int startState = 0;            // 0=Idle, 1=start1已触发, 2=start2已触发
bool locked = false;           // 互锁标志

void setup() {
  pinMode(LASER_PIN, OUTPUT);
  pinMode(BUTTON1_PIN, INPUT); // A0低电平有效
  pinMode(BUTTON2_PIN, INPUT); // A1高电平有效
  digitalWrite(LASER_PIN, LOW);
  Serial.begin(9600);

  // 初始化OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
    for(;;); // 初始化失败停机
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
}

void loop() {
  // 1. 串口接收命令（激光控制）
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.equalsIgnoreCase("laseron")) {
      laserState = true;
      digitalWrite(LASER_PIN, HIGH);
    } 
    else if (cmd.equalsIgnoreCase("laseroff")) {
      laserState = false;
      digitalWrite(LASER_PIN, LOW);
    }
  }

  // 2. 按键触发互锁逻辑
  if (!locked) {  // 只有未锁定时才响应
    if (digitalRead(BUTTON1_PIN) == LOW) { // A0低电平有效
      Serial.println("start1");
      startState = 1;
      locked = true; // 锁定，禁止后续触发
    } 
    else if (digitalRead(BUTTON2_PIN) == HIGH) { // A1高电平有效
      Serial.println("start2");
      startState = 2;
      locked = true; // 锁定，禁止后续触发
    }
  }

  // 3. OLED刷新显示
  display.clearDisplay();

  display.setCursor(0, 16);
  display.print("Laser: ");
  display.println(laserState ? "ON" : "OFF");

  display.setCursor(0, 32);
  display.print("Start: ");
  if (startState == 1) display.println("start1");
  else if (startState == 2) display.println("start2");
  else display.println("Idle");

  display.setCursor(0, 48);
  display.print("Locked: ");
  display.println(locked ? "YES" : "NO");

  display.display();
}
