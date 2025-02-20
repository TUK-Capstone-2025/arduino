#include <Wire.h>
#include <MPU6050.h>

#define BT_SERIAL Serial1  
#define TRIG_PIN 9
#define ECHO_PIN 10
#define CDS_SENSOR_PIN A0
#define LED_PIN 6
#define WARNING_DISTANCE 50

const float ACCEL_THRESHOLD = 2.5;     
const float TILT_ANGLE_THRESHOLD = 45.0;
const int LIGHT_THRESHOLD = 300;        
const unsigned long SENSOR_UPDATE_INTERVAL = 500;
const unsigned long MPU_UPDATE_INTERVAL = 500;  

unsigned long lastUpdateTime = 0;
unsigned long lastMpuUpdateTime = 0;
unsigned long lastAccidentTime = 0;

MPU6050 mpu;

// 상태 변수
bool isTilted = false;
bool isImpactDetected = false;

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

  if (currentTime - lastUpdateTime >= SENSOR_UPDATE_INTERVAL) {
    lastUpdateTime = currentTime;

    sendSensorData("DISTANCE", (int)getAverageDistance()); 
    delay(500);  

    sendSensorData("WARNING", (getAverageDistance() <= WARNING_DISTANCE) ? 1 : 0); 
    delay(500);

    int lightValue = analogRead(CDS_SENSOR_PIN);
    sendSensorData("LIGHT", lightValue); 
    delay(500);

    // led 점등/소등 변경 주기 (2초 간 변화 없을 때만 변경)
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
  }

  if (currentTime - lastMpuUpdateTime >= MPU_UPDATE_INTERVAL) {
    lastMpuUpdateTime = currentTime;
    analyzeMPU6050(currentTime);
  }
}

void sendSensorData(String key, float value) {
  String jsonBuffer = "{\"" + key + "\":" + String(value, 1) + "}";  
  BT_SERIAL.println(jsonBuffer);  
  Serial.println(jsonBuffer);  
  Serial.println();
}

void analyzeMPU6050(unsigned long currentTime) {
  int16_t ax, ay, az, gx, gy, gz;

  if (!mpu.testConnection()) return;
  
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float accelX = ax / 16384.0, accelY = ay / 16384.0, accelZ = az / 16384.0;
  float totalAccel = sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);

  // 충격 감지 (임계값 2.5G 이상 시 1 아님 0)
  bool impactDetected = (totalAccel > ACCEL_THRESHOLD);
  sendSensorData("IMPACT", impactDetected ? 1 : 0);

  float tiltAngle = sqrt(pow(atan2(accelY, accelZ) * 180 / PI, 2) + 
                         pow(atan2(accelX, accelZ) * 180 / PI, 2));

  // JSON 형식으로 기울기 출력
  sendSensorData("TILT", tiltAngle);

  delay(500);

  // 넘어짐 감지 (45도 이상 기울어짐)
  bool tiltDetected = (tiltAngle > TILT_ANGLE_THRESHOLD);

  if (tiltDetected) {
    if (!isTilted) {
      isTilted = true;
      Serial.println("넘어짐 감지 (45도 이상 기울어짐)");
    }
  } else {
    if (isTilted) {
      isTilted = false;
      Serial.println("정상 주행 각도로 복귀");
    }
  }

  // 사고 감지
  if (tiltDetected && impactDetected) {
    sendSensorData("ACCIDENT", 1);
    Serial.println("사고 발생 감지");
  } else {
    sendSensorData("ACCIDENT", 0);
  }
}

float measureDistance() {
  static float lastValidDistance = 100; // 초기 값

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 50000);  

  if (duration == 0) {
    return lastValidDistance;  
    // 측정 가능 거리 (4m) 초과 시 이전 값 반환
  }

  float distance = (duration * 0.0343) / 2.0;
  
  if (distance > 400 || distance < 0) {
    return lastValidDistance; 
    // 위랑 동일한 매커니즘
  }

  lastValidDistance = distance; 
  // 4m 이내 정상 측정 시 값 저장
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
