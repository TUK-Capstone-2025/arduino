#include <Wire.h> // I2C 통신용 헤더 (SDA/SCL 통신)
#include <MPU6050.h>  // 가속도/자이로 센서 제어용 라이브러리

#define BT_SERIAL Serial1    // 외부 장치용 추가 UART 포트

#define TRIG_PIN 9            // 초음파 센서 (신호 출력 핀)
#define ECHO_PIN 10           // 초음파 센서 (거리 입력 핀)
#define WARNING_DISTANCE 2000   // 거리 임계 값 2m 이내 감지 시 경고

#define CDS_SENSOR_PIN A0     // CDS 센서 아날로그 입력 핀

#define HALL_SENSOR_PIN 2     // 휠 회전 RPM 측정
#define BRAKE_LED_PIN 7       // 속도 감소 감지 시 점등

#define EMERGENCY_LED_PIN 4   // 사고 감지 시 점멸
#define ADDITIONAL_LED_PIN 5  // 전조등
#define LED_PIN 6             // 후미등

// 배정밀도(double)는 8byte 만큼의 메모리 공간 점유
// 따라서 4byte 크기의 float 자료형을 사용 (굳이 double 필요 없음)

MPU6050 mpu;  // MPU6050 센서의 객체(맴버가 아직 메모리에 할당된 건 아님) 선언

// 시간 관리용
const unsigned long SENSOR_UPDATE_INTERVAL = 500; // 센서 신호 갱신 주기 (0.5초)
unsigned long lastUpdateTime = 0;                 // 일반 센서 마지막 갱신 시각
unsigned long lastMpuUpdateTime = 0;              // MPU6050 마지막 분석 시각
unsigned long lastSpeedUpdateTime = 0;            // 속도 계산 마지막 수행 시각
unsigned long accidentStartTime = 0;              // 사고 조건 충족 후 경과 시간 계산용 시작 시각
unsigned long brakeLightOnTime = 0;               // 브레이크 LED 점등 시작 시각
unsigned long lastEmergencyBlinkTime = 0;         // 비상등 마지막 점멸 전환 시각

// 사고 발생 관련
const unsigned long MPU_UPDATE_INTERVAL = 500;       // 가속도/자이로 센서 데이터 분석 주기 (0.5초)
const float ACCEL_THRESHOLD = 2.0;                  // 충격 감지를 위한 가속도 임계 값 (중력 2g)
const float TILT_ANGLE_THRESHOLD = 45.0;             // 넘어짐 판단을 위한 기울기 각도 (45도)
const unsigned long ACCIDENT_TIME_THRESHOLD = 20000; // 사고 상황 판단까지 소요되는 시간 (20초)
const unsigned long EMERGENCY_BLINK_INTERVAL = 1000; // 비상등 점멸 주기 (1초)
bool emergencyBlinkState = false;                    // 비상등 ON/OFF 상태 여부
bool isTilted = false;                               // 현재 기넘어짐 상태 여부
bool isImpactDetected = false;                       // 강한 충격 감지 여부
bool accidentDetected = false;                       // 최종적으로 사고로 판단 되었는지 여부

// 속도 관련
const float WHEEL_CIRCUMFERENCE = 2.1;            // 700c 기준 자전거 바퀴 둘레 (속도 계산용)

const unsigned long SPEED_UPDATE_INTERVAL = 500;  // 속도 계산 및 전송 주기 (0.5초)
float previousSpeed = 0;                          // 이전 속도 저장 (급감속 여부 판단용)
volatile float wheelRPM = 0;                      // 현재 휠 회전 수(RPM), 인터럽트 기반 계산값
volatile unsigned long lastWheelTime = 0;         // 휠 회전을 마지막으로 감지한 시간 (ISR에서 갱신)
const float BRAKE_THRESHOLD = 2.0;               // 브레이크등 점등 판단 (주행 속도가 2km 이상 감소)
const unsigned long BRAKE_LIGHT_DURATION = 4000;  // 브레이크등 점등 유지 시간 (4초)

// 오토라이트 관련
const int LIGHT_THRESHOLD = 300;                  // 이 값을 초과 시 라이트 점등

auto getAverageDistance() -> float;
auto measureDistance() -> float;
auto calculateSpeed() -> float;
void sendSensorData(const String&, float);
void analyzeMPU6050(unsigned long);
void wheelRotationDetected();

// 전원 입력 or 리셋 시에만 딱 한 번 실행되는 함수 (하드웨어 초기화)
void setup() {
  Serial.begin(9600);    // MCU와 하드웨어 모듈 및 센서들 간 통신 속도 (초당 9600bit 데이터 전송)
  BT_SERIAL.begin(9600); // MCU와 BLE(HM-10) 간 유선 통신 속도

  // 상단에 선언된 메크로 상수에서 사용할 핀 번호의 용도를 지정 (INPUT/OUTPUT)
  pinMode(TRIG_PIN, OUTPUT);  // 자전거 후방으로 초음파 신호를 발사 (OUTPUT)
  pinMode(ECHO_PIN, INPUT);   // 사물에 반사된 신호를 수신 (INPUT)
  pinMode(LED_PIN, OUTPUT);
  pinMode(HALL_SENSOR_PIN, INPUT_PULLUP);
  pinMode(BRAKE_LED_PIN, OUTPUT);
  pinMode(EMERGENCY_LED_PIN, OUTPUT);
  pinMode(ADDITIONAL_LED_PIN, OUTPUT);

  Wire.begin();     // Wire 인스턴스 (MPU 보드를 I2C Master 보드로 설정 후 SDA/SCL 핀을 통신 모드로 전환)
  mpu.initialize(); // 전원 절약을 위해 sleep 모드에 있는 MPU6050 센서를 깨우는 작업

  Serial.println(mpu.testConnection() ? "자이로 센서 연결 성공" : "자이로 센서 연결 실패");

  attachInterrupt(digitalPinToInterrupt(HALL_SENSOR_PIN), wheelRotationDetected, FALLING);
}

// 셋업 함수 이후 전원이 차단될 때까지 무한반복되는 함수
void loop() {
  // 여러 센서들이 마지막으로 데이터를 측정한 시간을 아래의 값을 통해 확인
  // ex) 센서 신호 갱신 주기가 0.5초인데 loop() 함수 순회 후 재시작 시점에 0.5가 되지 않았다면 sleep 상태 유지
  unsigned long currentTime = millis(); // 전원 인가 후 지난의 누적합을 ms 단위로 저장

  if (currentTime - lastUpdateTime >= SENSOR_UPDATE_INTERVAL) {
    
    lastUpdateTime = currentTime;
    
    float distance = getAverageDistance();
    // 정수형으로 캐스팅하여 불필요한 소수점 이하의 값은 버림
    sendSensorData("DISTANCE", static_cast<int>(distance));
    delay(100);

    bool warningState = (distance > 0 && distance <= WARNING_DISTANCE);
    sendSensorData("WARNING", warningState ? 1 : 0);
    delay(100);

    int lightValue = analogRead(CDS_SENSOR_PIN);
    sendSensorData("LIGHT", lightValue);
    delay(100);

    // 전후방 라이트의 마지막 상태 변화 시점을 저장
    static unsigned long lastLightChange = 0;

    // 자전거를 타고 이동 중 주변 밝기의 빈번한 변화로 인해 LED의 채터링 현상을 방지하기 위해
    // 2초의 가드 타임을 설정하여 불필요한 전력 소모를 줄이고 잦은 상태 변화에 따른 LED 소자의 수명 저하를 방지
    if (currentTime - lastLightChange > 2000) {
      if (lightValue > LIGHT_THRESHOLD) {       
        digitalWrite(LED_PIN, HIGH);            // 주행 환경이 임계치 300보다 어둡다면 LED 점등
        digitalWrite(ADDITIONAL_LED_PIN, HIGH); 
        sendSensorData("LED", 1);
      } else {                  
        digitalWrite(LED_PIN, LOW);             // 300보다 밝다면 LED 소등
        digitalWrite(ADDITIONAL_LED_PIN, LOW);  
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
    
    float currentSpeed = calculateSpeed();
    sendSensorData("SPEED", currentSpeed);

    static bool brakeActive = false;
    static bool initialized = false;

    if (!initialized) {
      previousSpeed = currentSpeed;
      initialized = true;
    }

    float speedDrop = previousSpeed - currentSpeed;

    if (!brakeActive &&
      previousSpeed >= 5.0 &&
      currentSpeed >= 0.0 &&
      speedDrop >= BRAKE_THRESHOLD) {
      digitalWrite(BRAKE_LED_PIN, HIGH);
      brakeLightOnTime = currentTime;
      brakeActive = true;
      Serial.println("BRAKE ON");
    }

    if (brakeActive && (currentTime - brakeLightOnTime >= BRAKE_LIGHT_DURATION)) {
      digitalWrite(BRAKE_LED_PIN, LOW);
      brakeActive = false;
      Serial.println("BRAKE OFF");
    } 

    previousSpeed = currentSpeed;
  }
  if (accidentDetected && isTilted) {
    if (millis() - lastEmergencyBlinkTime >= EMERGENCY_BLINK_INTERVAL) {
      emergencyBlinkState = !emergencyBlinkState;
      digitalWrite(EMERGENCY_LED_PIN, emergencyBlinkState ? HIGH : LOW);
      lastEmergencyBlinkTime = millis();
    }
  } else {
    // 사고 해제되면 비상등 OFF
    digitalWrite(EMERGENCY_LED_PIN, LOW);
    emergencyBlinkState = false;
  }
}

auto calculateSpeed() -> float {
  unsigned long currentTime = millis();

  if (currentTime - lastWheelTime > 2000) { wheelRPM = 0; }

  return (wheelRPM * WHEEL_CIRCUMFERENCE * 60.0) / 1000.0;
}

void wheelRotationDetected() {
  unsigned long currentTime = millis();
  unsigned long timeDifference = currentTime - lastWheelTime;

  if (timeDifference > 50) {
    wheelRPM = (60.0 / (timeDifference / 1000.0));
    lastWheelTime = currentTime;
  }
}

void sendSensorData(const String& key, float value) {
  // JSON 규격 key는 반드시 큰따옴표("")를 사용해 감싸야 됨 (그래서 이스케이프 시퀀스를 사용함)
  String jsonBuffer = "{\"" + key + "\":" + String(value, 1) + "}\n"; // String(value, 1) 소숫점 첫째 자리까지만 표시
  BT_SERIAL.print(jsonBuffer + "\n"); // 연동된 Android 스마트폰으로 데이터 전송
  Serial.println(jsonBuffer);         // HW 개발자 실시간 모니터링을 위해 시리얼 터미널에 출력
  delay(200);                         // 모듈이 데이터를 처리 후 전송할 수 있는 버퍼(여유) 2ms 제공 (너무 빨리 보내면 데이터가 깨질 수 있음)
}

void analyzeMPU6050(unsigned long currentTime) {
  int16_t ax, ay, az, gx, gy, gz;

  if (!mpu.testConnection()) return;

  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float accelX = ax / 16384.0, accelY = ay / 16384.0, accelZ = az / 16384.0;
  float totalAccel = sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);

  bool impactNow = (totalAccel > ACCEL_THRESHOLD);
  isImpactDetected = (impactNow) ? true : false;

  sendSensorData("IMPACT", isImpactDetected ? 1 : 0);
  delay(100);

  float tiltAngle = sqrt(pow(atan2(accelY, accelZ) * 180 / PI, 2) + 
                         pow(atan2(accelX, accelZ) * 180 / PI, 2));

  sendSensorData("TILT", tiltAngle);
  delay(100);
  
  bool tiltNow = (tiltAngle > TILT_ANGLE_THRESHOLD);

  isTilted = (tiltNow) ? true : false;
  
  if (isTilted) { Serial.println("넘어짐 감지"); }

  if (isTilted && !tiltNow) {
    isTilted = false;
    accidentDetected = false;
    isImpactDetected = false;
    sendSensorData("ACCIDENT", 0);
    Serial.println("정상 주행 각도로 복귀 (사고 감지 해제)");
  }

  if (isTilted && isImpactDetected) {
    if (!accidentDetected) {
      accidentStartTime = currentTime;
      accidentDetected = true;
    } else if (currentTime - accidentStartTime >= ACCIDENT_TIME_THRESHOLD) {
      sendSensorData("ACCIDENT", 1);
      Serial.println("사고 발생 감지 (20초 이상 넘어진 상태 유지)");
    }
  }
}

auto measureDistance() -> float {
  static float lastValidDistance = 100; // 마지막 유효 거리 (초기 값은 1m)

  digitalWrite(TRIG_PIN, LOW);  // digitalWrite() -> TRIG_PIN 전압을 LOW(0V) 상태로 만들어 신호 종료
  delayMicroseconds(2);         // 전압이 완전히 0V가 되어 안정화될 때까지 대기
  digitalWrite(TRIG_PIN, HIGH); // 트리거 핀으로 전압을 가해 초음파를 발사할 준비 상태로 전환
  delayMicroseconds(10);        // 센서에서 명령을 인식할 수 있는 최소한의 시간 제공
  digitalWrite(TRIG_PIN, LOW);  // 명령 전달 종료 후 센서가 초음파를 발사했다면 핀 전압 인가 종료

  // 초음파 전송 이후 50ms 동안 echo 수신이 없을 시 후방에 사물이 없다고 판단 후 0 반환
  long duration = pulseIn(ECHO_PIN, HIGH, 50000);
  
  if (duration == 0) { return lastValidDistance; }

  float distance = (duration * 0.0343) / 2.0;

  // HC-SR04 센서는 저가형이라 최대 측정 거리가 4m (잘못된 측정 값을 필터링하기 위한 로직)
  // 이상한 값이 측정될 경우 마지막으로 측정된 사물과의 거리를 보내 앱에서 시도 때도 없이 잘못된
  // "후방 접근 감지" 경고 배너가 발생하지 않도록 하여 시스템이 죽거나 갑자기 튀지 않도록 방지
  if (distance > 400 || distance < 0) { return lastValidDistance; }

  lastValidDistance = distance; // DATA 섹션에 저장된 마지막 유효 측정 거리를 갱신
  
  return distance;  // 마지막 측정 거리를 반환
}

// 초음파 센서에서 수집된 거리 측정 데이터의 신뢰성 향상을 위한 필터링 & 평균화 작업 함수
auto getAverageDistance() -> float {
  float totalDistance = 0;
  int validSamples = 0;
  const int sampleCnt = 5;  // 표본화 작업 5번

  for (int i = 0; i < sampleCnt; i++) {
    float distance = measureDistance();
    if (distance != -1) {         // 측정된 거리가 -1이 아닐 시
      totalDistance += distance;  // 총 거리 + 현재 측정 거리
      validSamples++;             // 유효 측정 횟수 1회 증가
    }
    delay(10);                    // 0.1초 대기 후 재측정 (초음파 간 간섭 방지용)
  }
  // 유효 측정 횟수가 0번일 경우 -1 아니라면 측정된 총 거리 / 유효 측정 횟수
  return (validSamples == 0) ? -1 : totalDistance / validSamples;
}