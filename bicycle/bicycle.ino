#include <Wire.h>
#include <MPU6050.h>

#define BT_SERIAL Serial1  
#define TRIG_PIN 9
#define ECHO_PIN 10
#define CDS_SENSOR_PIN A0
#define LED_PIN 6
#define WARNING_DISTANCE 50
#define HALL_SENSOR_PIN 2

const float ACCEL_THRESHOLD = 1.1;  
const float TILT_ANGLE_THRESHOLD = 45.0;
const int LIGHT_THRESHOLD = 300;        
const unsigned long SENSOR_UPDATE_INTERVAL = 500;
const unsigned long MPU_UPDATE_INTERVAL = 500;  
const unsigned long SPEED_UPDATE_INTERVAL = 1000;
const unsigned long ACCIDENT_TIME_THRESHOLD = 5000; 

volatile unsigned long lastWheelTime = 0;
volatile float wheelRPM = 0;
const float WHEEL_CIRCUMFERENCE = 2.1; // 휠 둘레 (미터 단위)

unsigned long lastUpdateTime = 0;
unsigned long lastMpuUpdateTime = 0;
unsigned long lastSpeedUpdateTime = 0;
unsigned long accidentStartTime = 0; // 사고 감지 시작 시간

MPU6050 mpu;

// 상태 변수
bool isTilted = false;
bool isImpactDetected = false;
bool accidentDetected = false; // 사고 감지 여부

void wheelRotationDetected() {
  unsigned long currentTime = millis();
  unsigned long timeDifference = currentTime - lastWheelTime;

  // 자석이 회전하며 인식되는 속도 50ms 이하일 경우 값 무시
  if (timeDifference > 50) {
    wheelRPM = (60.0 / (timeDifference / 1000.0));
    lastWheelTime = currentTime;
  }
}

float calculateSpeed() {
  unsigned long currentTime = millis();

  // 마지막 회전 감지 이후 2초 이후 RPM 0 값으로 초기화
  if (currentTime - lastWheelTime > 2000) {
    wheelRPM = 0;
  }

  return (wheelRPM * WHEEL_CIRCUMFERENCE * 60.0) / 1000.0;
}

void setup() {
  Serial.begin(9600);
  BT_SERIAL.begin(9600);  

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(HALL_SENSOR_PIN, INPUT_PULLUP);

  Wire.begin();
  mpu.initialize();

  if (mpu.testConnection()) {
    Serial.println("자이로 센서 연결 성공");
  } else {
    Serial.println("자이로 센서 연결 실패");
  }

  attachInterrupt(digitalPinToInterrupt(HALL_SENSOR_PIN), wheelRotationDetected, FALLING);
}

void loop() {
  unsigned long currentTime = millis();

  if (currentTime - lastUpdateTime >= SENSOR_UPDATE_INTERVAL) {
    lastUpdateTime = currentTime;

    float distance = getAverageDistance();
    sendSensorData("DISTANCE", (int)distance);  
    delay(100);  

    bool warningState = (distance > 0 && distance <= WARNING_DISTANCE);
    sendSensorData("WARNING", warningState ? 1 : 0);
    delay(100);

    int lightValue = analogRead(CDS_SENSOR_PIN);
    sendSensorData("LIGHT", lightValue); 
    delay(100);

    static unsigned long lastLightChange = 0;
    if (currentTime - lastLightChange > 2000) {
      if (lightValue > LIGHT_THRESHOLD) {
        digitalWrite(LED_PIN, HIGH);  
        sendSensorData("LED", 1);
      } else {
        digitalWrite(LED_PIN, LOW);   
        sendSensorData("LED", 0);
      }
      lastLightChange = currentTime;
    }
    delay(100);
  }

  if (currentTime - lastMpuUpdateTime >= MPU_UPDATE_INTERVAL) {
    lastMpuUpdateTime = currentTime;
    analyzeMPU6050(currentTime);
  }

  if (currentTime - lastSpeedUpdateTime >= SPEED_UPDATE_INTERVAL) {
    lastSpeedUpdateTime = currentTime;
    sendSensorData("SPEED", calculateSpeed());
  }
}

void sendSensorData(String key, float value) {
  String jsonBuffer = "{\"" + key + "\":" + String(value, 1) + "}\n";  
  BT_SERIAL.print(jsonBuffer + "\n");  
  Serial.println(jsonBuffer);
  delay(200); 
}

void analyzeMPU6050(unsigned long currentTime) {
  int16_t ax, ay, az, gx, gy, gz;

  if (!mpu.testConnection()) return;
  
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float accelX = ax / 16384.0, accelY = ay / 16384.0, accelZ = az / 16384.0;
  float totalAccel = sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);

  bool impactNow = (totalAccel > ACCEL_THRESHOLD);
  if (impactNow) {
    isImpactDetected = true;  
  }

  sendSensorData("IMPACT", isImpactDetected ? 1 : 0);
  delay(100);

  float tiltAngle = sqrt(pow(atan2(accelY, accelZ) * 180 / PI, 2) + 
                         pow(atan2(accelX, accelZ) * 180 / PI, 2));

  sendSensorData("TILT", tiltAngle);
  delay(100);

  bool tiltNow = (tiltAngle > TILT_ANGLE_THRESHOLD);

  if (tiltNow) {
    isTilted = true;
  }

  if (isTilted) {
    Serial.println("넘어짐 감지");
  }

  if (isTilted && !tiltNow) {
    isTilted = false;
    accidentDetected = false;
    isImpactDetected = false;
    sendSensorData("ACCIDENT", 0);
    Serial.println("정상각도로 복귀 - 사고 감지 해제");
  }

  if (isTilted && isImpactDetected) {
    if (!accidentDetected) {
      accidentStartTime = currentTime;
      accidentDetected = true;
    } else if (currentTime - accidentStartTime >= ACCIDENT_TIME_THRESHOLD) {
      sendSensorData("ACCIDENT", 1);
      Serial.println("사고 발생 감지 (5초 이상 복귀 안됨)");
    }
  }
}

float measureDistance() {
  static float lastValidDistance = 100; 

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 50000);  

  if (duration == 0) {
    return lastValidDistance;  
  }

  float distance = (duration * 0.0343) / 2.0;
  
  if (distance > 400 || distance < 0) {
    return lastValidDistance; 
  }

  lastValidDistance = distance; 
  return distance;
}

float getAverageDistance() {
  float totalDistance = 0;
  int validSamples = 0;
  const int sampleCount = 5;

  for (int i = 0; i < sampleCount; i++) {
    float distance = measureDistance();
    if (distance != -1) {
      totalDistance += distance;
      validSamples++;
    }
    delay(10);
  }

  return (validSamples == 0) ? -1 : totalDistance / validSamples;
}