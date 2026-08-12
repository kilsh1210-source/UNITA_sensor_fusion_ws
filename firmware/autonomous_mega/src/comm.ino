void checkSerial() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    handleLine(line);
  }
}

void handleLine(String line) {
  line.trim();
  if (line.length() == 0) return;

  // Comma-separated command: C,<steering -1.0..1.0>,<rear_pwm>
  if (line.startsWith("C,")) {
    int idx1 = line.indexOf(',');
    int idx2 = line.indexOf(',', idx1 + 1);
    int idx3 = line.indexOf(',', idx2 + 1);

    if (idx1 == -1 || idx2 == -1 || idx3 != -1) return;

    String steerStr = line.substring(idx1 + 1, idx2);
    String rearStr = line.substring(idx2 + 1);

    rear_pwm_cmd = constrain(rearStr.toInt(), -255, 255);
    targetSteering = normalizedToSteeringPosition(steerStr.toFloat());

    lastDriveMs = millis();
    driveCommandActive = true;
    return;
  }

  // Space-separated command: C <steering -1.0..1.0> <rear_pwm>
  if (line.startsWith("C ")) {
    int firstSpace = line.indexOf(' ');
    int secondSpace = line.indexOf(' ', firstSpace + 1);
    int thirdSpace = line.indexOf(' ', secondSpace + 1);

    if (firstSpace == -1 || secondSpace == -1 || thirdSpace != -1) return;

    String steerStr = line.substring(firstSpace + 1, secondSpace);
    String rearStr = line.substring(secondSpace + 1);

    rear_pwm_cmd = constrain(rearStr.toInt(), -255, 255);
    targetSteering = normalizedToSteeringPosition(steerStr.toFloat());

    lastDriveMs = millis();
    driveCommandActive = true;
    return;
  }
}
