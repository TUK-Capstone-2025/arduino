#include <Wire.h>
#include <MPU6050.h>

#define BT_SERIAL Serial1  
#define TRIG_PIN 9
#define ECHO_PIN 10
#define CDS_SENSOR_PIN A0
#define LED_PIN 6
#define WARNING_DISTANCE 200 // 경고 알람 거리를 2 미터로 설정함

// 자이로 센서 임계값 설정
const float ACCEL_THRESHOLD = 2.5;     
const float GYRO_THRESHOLD = 200.0;    
const float TILT_ANGLE_THRESHOLD = 45.0;
const unsigned long RECOVERY_TIME = 10000;
const int LIGHT_THRESHOLD = 300;        

// 상태 변수
bool isTilted = false;
bool isImpactDetected = false;
unsigned long tiltStartTime = 0;
unsigned long lastUltrasonicTime = 0;
unsigned long lastNormalStatusTime = 0;
unsigned long lastLightCheckTime = 0;

MPU6050 mpu;

void checkLightSensor();
float getAverageDistance();
float measureDistance();
void sendData(String key, int value);
void analyzeMPU6050(unsigned long currentTime);  

void setup() {
  Serial.begin(9600);
  BT_SERIAL.begin(9600); 

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);

  Wire.begin();
  mpu.initialize();

  if (mpu.testConnection()) {
    Serial.println("자이로 센서 연결 성공");
  } else {
    Serial.println("자이로 센서 연결 실패");
  }
  Serial.println("블루투스 연결 대기 중...");
}

void loop() {
  unsigned long currentTime = millis();

  if (currentTime - lastUltrasonicTime >= 1000) {
    int distanceCm = (int)getAverageDistance(); // cm 단위로 변환

    if (distanceCm != -1) {
      sendData("DISTANCE", distanceCm); // cm 단위로 전송

      // 2m (200cm) 이내 접근물체 감지 시 경고
      if (distanceCm <= WARNING_DISTANCE) {
        Serial.println("🚨 경고! 후방 2m 이내에 사물이 접근 중입니다!");
        sendData("WARNING", 1);
      } else {
        sendData("WARNING", 0);
      }
    }

    lastUltrasonicTime = currentTime;
  }

  if (currentTime - lastNormalStatusTime >= 1000) {
    analyzeMPU6050(currentTime);
    lastNormalStatusTime = currentTime;
  }

  if (currentTime - lastLightCheckTime >= 500) {
    checkLightSensor();
    lastLightCheckTime = currentTime;
  }
}

// 초음파 거리 측정 (cm 단위 유지)
float measureDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 50000); // 타임아웃 50ms로 증가

  if (duration == 0) {
    return -1; // 신호 없음 -> 오류 값 반환 (메시지 출력 제거)
  }

  float distance = (duration * 0.0343) / 2.0; // 거리 변환

  if (distance > 400 || distance < 0) {
    return -1; // 초음파 센서 측정 가능 거리(400cm) 초과 시 오류 반환
  }

  return distance;
}

// 초음파 거리 측정 (노이즈 제거, 오류 값 제외)
float getAverageDistance() {
  float totalDistance = 0;
  int validSamples = 0;
  const int sampleCount = 5;

  for (int i = 0; i < sampleCount; i++) {
    float distance = measureDistance();
    if (distance != -1) { // 유효한 값만 포함
      totalDistance += distance;
      validSamples++;
    }
    delay(10);
  }

  if (validSamples == 0) {
    Serial.println("⚠️ 초음파 센서 오류: 유효한 거리 데이터를 얻지 못함");
    return -1;
  }

  return totalDistance / validSamples;
}

// JSON 형식으로 블루투스 데이터 전송 (정수 cm 단위)
void sendData(String key, int value) {
  Serial.print(key);
  Serial.print(": ");
  Serial.println(value);

  BT_SERIAL.print("{\"");
  BT_SERIAL.print(key);
  BT_SERIAL.print("\": ");
  BT_SERIAL.print(value);
  BT_SERIAL.println("}");
}

// 자이로 센서 분석 (기존 코드 유지)
void analyzeMPU6050(unsigned long currentTime) {
  int16_t ax, ay, az, gx, gy, gz;
  if (!mpu.testConnection()) return;

  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float accelX = ax / 16384.0, accelY = ay / 16384.0, accelZ = az / 16384.0;
  float totalAccel = sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);

  float tiltAngle = abs(atan2(accelY, accelZ) * 180 / PI) +  
                    abs(atan2(accelX, accelZ) * 180 / PI);

  bool impactDetected = totalAccel > ACCEL_THRESHOLD;
  bool tiltDetected = tiltAngle > TILT_ANGLE_THRESHOLD;

  static bool accidentOccurred = false;

  if (impactDetected && tiltDetected) {
      if (!accidentOccurred) {
          sendData("ACCIDENT", 1);
          Serial.println("🚨 사고 감지! (충격 + 45도 이상 기울기)");
          accidentOccurred = true; 
      }
  } else {
      accidentOccurred = false;
  }

  sendData("IMPACT", impactDetected ? 1 : 0);
  sendData("TILT", tiltDetected ? 1 : 0);
}

// 조도 센서 분석 (기존 코드 유지)
void checkLightSensor() {
  int lightValue = analogRead(CDS_SENSOR_PIN);
  sendData("LIGHT", lightValue);

  if (lightValue > LIGHT_THRESHOLD) {
    digitalWrite(LED_PIN, HIGH);
    sendData("LED", 1);
  } else {
    digitalWrite(LED_PIN, LOW);
    sendData("LED", 0);
  }
}
