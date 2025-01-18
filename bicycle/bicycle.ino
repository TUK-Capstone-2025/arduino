#define TRIG_PIN 9           // HC-SR04의 TRIG 핀 연결
#define ECHO_PIN 10          // HC-SR04의 ECHO 핀 연결
#define WARNING_DISTANCE 2.0 // 경고 거리 (미터 단위)
#define SHOCK_SENSOR_PIN 2   // 충격 감지 센서 핀
#define SHOCK_THRESHOLD 3    // 감지 임계값
#define TIME_WINDOW 500      // 시간 창 (ms)

int shockCount = 0;                 // 충격 횟수 저장 변수
unsigned long lastShockTime = 0;    // 마지막 충격 발생 시간 저장 변수

void setup() {
  Serial.begin(9600);         // 시리얼 통신 시작
  pinMode(TRIG_PIN, OUTPUT);  // TRIG 핀 출력 설정
  pinMode(ECHO_PIN, INPUT);   // ECHO 핀 입력 설정
  pinMode(SHOCK_SENSOR_PIN, INPUT); // 충격 감지 센서 핀 입력 설정
}

void loop() {
  long duration;  // 초음파가 되돌아오는 데 걸리는 시간
  float distance; // 계산된 거리 (단위: m)
  unsigned long currentTime = millis();

  // --- 초음파 거리 측정 ---
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  duration = pulseIn(ECHO_PIN, HIGH);
  distance = duration * 0.034 / 100.0; // 거리 계산 (미터 단위)

  // 거리 출력
  Serial.print("탐지 거리 : ");
  Serial.print(distance, 2);
  Serial.println(" m");

  if (distance > 0 && distance <= WARNING_DISTANCE) {
    Serial.println("후방 2m 이내에 차량이 접근 중...");
  }

  // --- 충격 감지 ---
  int shockState = digitalRead(SHOCK_SENSOR_PIN);
  
  // 충격이 발생하면 충격 횟수 증가 및 마지막 충격 시간 업데이트
  if (shockState == HIGH) {
    shockCount++;
    lastShockTime = currentTime;
  }

  // 시간 창 내 충격 횟수 확인
  if (currentTime - lastShockTime > TIME_WINDOW) {
    if (shockCount >= SHOCK_THRESHOLD) {
      Serial.println("강한 충격 감지!");
    }
    shockCount = 0; // 충격 횟수 초기화
  }

  delay(500); // 작은 딜레이
}
