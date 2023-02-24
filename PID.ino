#include <Wire.h>
#include <L3G.h>
#include <Servo.h>

L3G gyro;
Servo zservo;

int pos = 0;
const int startpin = 2;
const int endpin = 3;
const int updatepin = 4;

int zStart = 0;

int zEnd = 0;

int refAngle = 90;

void setup() {
  Serial.begin(9600);
  zservo.attach(9);
  zservo.write(refAngle);
  pinMode(startpin, INPUT);
  pinMode(endpin, INPUT);
  pinMode(updatepin, INPUT);

  Wire.begin();

  if (!gyro.init()) {
    Serial.print("Failed to autodetect gyro type!");
    while (1);
  }

  gyro.enableDefault();
  zservo.write(0);
}

void loop() {
  if(digitalRead(startpin) == HIGH){
    Serial.println("Started!");
    gyro.read();
    zStart = gyro.g.z;
  }
  if(digitalRead(endpin) == HIGH){
    Serial.println("Ended!");
    gyro.read();
    zEnd = gyro.g.z;
  }
  if(digitalRead(updatepin) == HIGH){
    Serial.println("Updated!");
    int val = zEnd - zStart;
    Serial.println(val);
    Serial.println(refAngle + val);
    zservo.write(refAngle + val);
  }
  delay(200);
}
