#include <SoftwareSerial.h>

#define BT_RX 7 
#define BT_TX 8 
#define LED_PIN 6 

SoftwareSerial bluetooth(BT_RX, BT_TX);

void setup() {

  Serial.begin(9600);
  bluetooth.begin(9600);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW); 

  Serial.println("전조등 조작 보드 초기화 완료!");
  Serial.println("블루투스 수신 대기 중...");
}

void loop() {
  if (bluetooth.available()) {
    String command = bluetooth.readStringUntil('\n');
    command.trim(); // 공백 & 개행 제거
    Serial.print("수신한 명령 : ");
    Serial.println(command);

    if (command == "ON") {
      digitalWrite(LED_PIN, HIGH);
      Serial.println("전조등 점등");
    } else if (command == "OFF") {
      digitalWrite(LED_PIN, LOW); 
      Serial.println("전조등 소등");
    } else {
      Serial.println("알 수 없는 명령어 수신...");
    }
  }
}
