void initMotors() {
  pinMode(FRONT_EN, OUTPUT); pinMode(FRONT_IN1, OUTPUT); pinMode(FRONT_IN2, OUTPUT);
  pinMode(REAR_EN, OUTPUT); pinMode(REAR_IN1, OUTPUT); pinMode(REAR_IN2, OUTPUT);
}

// 모터 구동 기본 함수
void setMotor(int EN, int A, int B, int speed) {
  if (speed > 0) {
    digitalWrite(A, HIGH);
    digitalWrite(B, LOW);
    analogWrite(EN, speed);
  } else if (speed < 0) {
    digitalWrite(A, LOW);
    digitalWrite(B, HIGH);
    analogWrite(EN, -speed);
  } else {
    digitalWrite(A, LOW);
    digitalWrite(B, LOW);
    analogWrite(EN, 0);
  }
}

int readSteeringPosition() {
  return analogRead(STEERING_POT_PIN);
}

int normalizedToSteeringPosition(float normalizedSteering) {
  normalizedSteering = constrain(normalizedSteering, -1.0f, 1.0f);

  float target;
  if (normalizedSteering < 0.0f) {
    target = steeringPotCenter
        + normalizedSteering * (steeringPotCenter - steeringPotMin);
  } else {
    target = steeringPotCenter
        + normalizedSteering * (steeringPotMax - steeringPotCenter);
  }

  return constrain((int)(target + 0.5f), steeringPotMin, steeringPotMax);
}

void reportSteeringPosition() {
  unsigned long now = millis();
  if (now - lastSteeringReport < STEERING_REPORT_INTERVAL_MS) return;

  lastSteeringReport = now;
  int steeringValue = readSteeringPosition();

  Serial.print("S ");
  Serial.println(steeringValue);
}

void updateSteeringControl() {
  int currentSteering = readSteeringPosition();
  int error = targetSteering - currentSteering;
  int absError = abs(error);

  if (!driveCommandActive || absError <= STEERING_DEADBAND) {
    steeringPwmOutput = 0;
    setMotor(FRONT_EN, FRONT_IN1, FRONT_IN2, 0);
    return;
  }

  int steeringPwm;

  if (absError >= STEERING_SLOWDOWN_RANGE) {
    steeringPwm = STEERING_MAX_PWM;
  } else {
    steeringPwm = map(
        absError,
        STEERING_DEADBAND + 1,
        STEERING_SLOWDOWN_RANGE,
        STEERING_MIN_PWM,
        STEERING_MAX_PWM
    );
    steeringPwm = constrain(
        steeringPwm,
        STEERING_MIN_PWM,
        STEERING_MAX_PWM
    );
  }

  int motorDirection = (error > 0) ? STEERING_DIRECTION : -STEERING_DIRECTION;
  steeringPwmOutput = motorDirection * steeringPwm;
  setMotor(FRONT_EN, FRONT_IN1, FRONT_IN2, steeringPwmOutput);
}
