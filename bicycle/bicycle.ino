#include <Wire.h>
#include <MPU6050.h>

#define TRIG_PIN 9           // 초음파 트리거
#define ECHO_PIN 10          // 초음파 에코
#define WARNING_DISTANCE 2.0 // 경고 거리 (m)

// 자이로 센서
MPU6050 mpu;

// 임계값 설정
const float ACCEL_THRESHOLD = 2.5;   // 가속도 변화량 임계값 (g)
const float GYRO_THRESHOLD = 200.0;  // 각속도 변화량 임계값 (각도/s)
const float TILT_ANGLE_THRESHOLD = 45.0; // 넘어짐 기울기 각도 임계값 (각도)
const unsigned long RECOVERY_TIME = 10000; // 정상 범주 복귀 허용 시간 (10초)

// 조도 센서와 LED 핀 설정
#define CDS_SENSOR_PIN A0 // 조도 센서 아날로그 핀
#define LED_PIN 6         // LED 핀 (디지털 핀)

// 조도 센서 임계값 (이 값 이상일 때 어둡다고 판단)
const int LIGHT_THRESHOLD = 300; // 조도 센서 임계값 (필요에 따라 조정)

// 상태 변수
bool isTilted = false;            // 현재 기울어진 상태 여부
bool isImpactDetected = false;    // 충격 발생 여부
unsigned long tiltStartTime = 0;  // 기울기 발생 시간
unsigned long lastUltrasonicTime = 0;  // 초음파 측정 시간 관리
unsigned long lastNormalStatusTime = 0; // 정상 상태 출력 시간 관리
unsigned long lastLightCheckTime = 0;   // 조도 센서 확인 시간 관리

void setup() {
  Serial.begin(9600);
  delay(1000);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT); // LED 핀을 출력으로 설정

  Wire.begin();
  mpu.initialize();

  if (mpu.testConnection()) {
    Serial.println("자이로 센서 연결 성공!!!");
  } else {
    Serial.println("자이로 센서 연결 실패...");
  }
}

void loop() {
  unsigned long currentTime = millis();

  // 1초 간격으로 초음파 거리 측정
  if (currentTime - lastUltrasonicTime >= 1000) {
    float distance = getAverageDistance();
    displayDistance(distance);
    lastUltrasonicTime = currentTime;
  }

  // 1초 간격으로 자이로 센서 데이터 분석
  if (currentTime - lastNormalStatusTime >= 1000) {
    analyzeMPU6050(currentTime);
    lastNormalStatusTime = currentTime;
  }

  // 0.5초 간격으로 조도 센서 데이터 확인
  if (currentTime - lastLightCheckTime >= 500) {
    checkLightSensor();
    lastLightCheckTime = currentTime;
  }
}

// 초음파 거리 측정 (노이즈 제거)
float getAverageDistance() {
  float totalDistance = 0;
  const int sampleCount = 5;

  for (int i = 0; i < sampleCount; i++) {
    totalDistance += measureDistance();
    delay(10);
  }

  return totalDistance / sampleCount;
}

// 초음파 거리 측정
float measureDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH);
  float distance = duration * 0.034 / 100.0;
  return distance;
}

// 거리 출력
void displayDistance(float distance) {
  Serial.print("후방 접근 물체와의 거리 : ");
  Serial.print(distance, 2);
  Serial.println(" m");

  if (distance > 0 && distance <= WARNING_DISTANCE) {
    Serial.println("경고 : 후방 2m 이내 차량 접근!");
  }
}

// 자이로 센서 데이터 분석
void analyzeMPU6050(unsigned long currentTime) {
  int16_t ax, ay, az, gx, gy, gz;

  // 센서 연결 상태 확인
  if (!mpu.testConnection()) {
    Serial.println("자이로 센서와의 연결이 끊어졌습니다...");
    return;
  }

  // 센서 데이터 읽기
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float accelX = ax / 16384.0, accelY = ay / 16384.0, accelZ = az / 16384.0;
  float gyroX = gx / 131.0, gyroY = gy / 131.0, gyroZ = gz / 131.0;

  float totalAccel = sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);

  // 충격 감지
  if (totalAccel > ACCEL_THRESHOLD || 
      abs(gyroX) > GYRO_THRESHOLD || 
      abs(gyroY) > GYRO_THRESHOLD || 
      abs(gyroZ) > GYRO_THRESHOLD) {
    isImpactDetected = true;
    Serial.println("경고 : 강한 충격 감지!!!");
  }

  // 기울기 확인
  float angleX = atan2(accelY, accelZ) * 180 / PI;
  float angleY = atan2(accelX, accelZ) * 180 / PI;

  if (abs(angleX) > TILT_ANGLE_THRESHOLD || abs(angleY) > TILT_ANGLE_THRESHOLD) {
    if (!isTilted) {
      isTilted = true;
      tiltStartTime = currentTime;
      Serial.println("경고 : 45°이상 기울임 감지...");
    } else if (currentTime - tiltStartTime > RECOVERY_TIME) {
      Serial.println("경고 : 45°이상 기울어진 각도로 주행 중입니다.");
      isTilted = false;
    }
  } else if (isTilted) {
    Serial.println("정상 각도 복귀 완료.");
    isTilted = false;
  }

  // 충격 후 기울기 지속 여부 확인
  if (isImpactDetected && isTilted) {
    Serial.println("경고 : 충격 후 기울어진 상태 지속!");
  }

  // 상태 초기화
  isImpactDetected = false;
}

// 조도 센서 값 확인 및 LED 제어
void checkLightSensor() {
  int lightValue = analogRead(CDS_SENSOR_PIN); // 조도 센서 값 읽기

  // 시리얼 모니터에 조도 센서 값 출력
  Serial.print("조도 센서 값: ");
  Serial.println(lightValue);

  // 밝을 때 LED 끄기, 어두울 때 LED 켜기
  if (lightValue > LIGHT_THRESHOLD) { // 센서 값이 높으면 어둡다고 판단
    digitalWrite(LED_PIN, HIGH); // LED 켜기
    Serial.println("LED ON (어두움)");
  } else {
    digitalWrite(LED_PIN, LOW); // LED 끄기
    Serial.println("LED OFF (밝음)");
  }
}
