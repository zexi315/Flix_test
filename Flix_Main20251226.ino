// Copyright (c) 2023 Oleg Kalachev <okalachev@gmail.com>
// Repository: https://github.com/okalachev/flix
// Main firmware file 主程序文件20251226
// 源代码使用Arduino IDE 2.3.x以上版本编译，不支持老版本的1.8.19！
// 必须安装ESP32开发板核心库（即esp32 by Espressif Systems）
// 必须安装MAVLINK，FlixPeriph库文件！
// 开发板可选择ESP32 Dev Module或WeMOS D1 MINI ESP32；
// 嘉立创开源项目：ESP32迷你无人机 https://oshwhub.com/malagis/esp32-mini-plane
// B站微辣火龙果 https://space.bilibili.com/544479100

#include "vector.h"
#include "quaternion.h"
#include "util.h"

#define WIFI_ENABLED 1

float t = NAN; // current step time, s
float dt; // time delta from previous step, s
float controlRoll, controlPitch, controlYaw, controlThrottle; // pilot's inputs, range [-1, 1]
float controlMode = NAN;
Vector gyro; // gyroscope data
Vector acc; // accelerometer data, m/s/s
Vector rates; // filtered angular rates, rad/s
Quaternion attitude; // estimated attitude
bool landed; // are we landed and stationary
float motors[4]; // normalized motors thrust in range [0..1]

void setup() {
	Serial.begin(115200);
	print("程序初始化！\n");
	disableBrownOut();
	setupParameters();
	setupLED();
	setupMotors();
	setLED(true);
#if WIFI_ENABLED
	setupWiFi();
#endif
	setupIMU();
	setupRC();
	setLED(false);
	print("初始化完成！\n");
}

void loop() {
	readIMU();
	step();
	readRC();
	estimate();
	control();
	sendMotors();
	handleInput();
#if WIFI_ENABLED
	processMavlink();
#endif
	logData();
	syncParameters();
}
