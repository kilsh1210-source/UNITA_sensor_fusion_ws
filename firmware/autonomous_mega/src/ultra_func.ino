float readDistanceCm(uint8_t trigPin, uint8_t echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long duration = pulseIn(echoPin, HIGH, ECHO_TIMEOUT_US);
  if (duration == 0) return -1.0f;

  return (duration * 0.0343f) / 2.0f ; // cm
}

// loop가 안 막히게: 한 번에 1개 센서만 측정(라운드로빈)
void ultrasonicStep() {
  unsigned long now = millis();
  if (now - lastUsMs < BETWEEN_SENSORS_MS) return;
  lastUsMs = now;

  us_cm[us_idx] = readDistanceCm(TRIG_PINS[us_idx], ECHO_PINS[us_idx]);
  us_idx = (us_idx + 1) % N_US;
}

// 주기적으로 한 줄 송신
// 주기적으로 한 줄 송신 (m 단위)
void ultrasonicPublishIfDue() {
  unsigned long now = millis();
  if (now - lastPubMs < ULTRA_PUB_MS) return;
  lastPubMs = now;

  for (uint8_t i = 0; i < N_US; i++) {
    float cm = us_cm[i];
    float m  = (cm < 0.0f) ? -1.0f : (cm * 0.01f);  // -1은 그대로 유지

    Serial.print("S");
    Serial.print(i + 1);
    Serial.print(":");
    Serial.print(m, 4);  // 소수점 4자리 (원하면 3으로)
    if (i < N_US - 1) Serial.print(",");
  }
  Serial.print(",FRONT:");
  Serial.print(steeringPwmOutput);
  Serial.print(",REAR:");
  Serial.print(rear_pwm_cmd);
  Serial.println();
}
