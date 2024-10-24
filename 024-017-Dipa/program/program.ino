#include "Header.h"

void setup() {
  Serial.begin(9600);
  DS18B20.begin();
  lcd.init();
  lcd.backlight();
  lcd.createChar(0, degreeSymbol);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(RELAY_PIN_1, OUTPUT);

  digitalWrite(RELAY_PIN_1, LOW);
}

void loop() {
  // if (Serial.available() > 0) {
  //   temperatureTest = Serial.readStringUntil('\n').toFloat();
  // }

  int state = !digitalRead(BUTTON_PIN);
  if (state) {
    systemState = !systemState;
    delay(500);
  }

  if (systemState) {
    digitalWrite(RELAY_PIN_1, LOW);
    Serial.print("| sp: ");
    Serial.print(sp);
    Serial.print("| temperature: ");
    Serial.print(temperature);
    Serial.print("| outputPID: ");
    Serial.print(outputPID);
    Serial.println();

    pid.setTunings(10.0, 0.02, 0.79); // 2.0, 0.001, 0.005 // 10.0, 0.02, 0.79 // 9.96, 0.02, 0.38
    pid.setOutputLimits(0, 100);
    pid.setMode(AUTOMATIC);
    pid.setControllerDirection(DIRECT);
    pid.setSampleTime(1);
    pid.compute(sp, temperature);
    outputPID = pid.getOutput();

    unsigned long currentMillis = millis();
    if (currentMillis - prevMillis >= interval) {
      prevMillis = currentMillis;
      DS18B20.requestTemperatures();

      temperature = DS18B20.getTempCByIndex(0);
      // temperature = temperatureTest + (temperatureTest * 0.018 * ((float)random(-100, 100) / 100.0));

      outputPID = map(outputPID, 0, 100, 0, 255);
      // analogWrite(SSR_PIN, outputPID);
      analogWrite(SSR_PIN, 255);

      lcd.setCursor(0, 0);
      lcd.print("PV =             ");
      lcd.setCursor(5, 0);
      lcd.print(temperature);
      lcd.write(byte(0));
      lcd.print("C");

      lcd.setCursor(0, 1);
      lcd.print("SP =             ");
      lcd.setCursor(5, 1);
      lcd.print(sp);
      lcd.write(byte(0));
      lcd.print("C");

      lcd.setCursor(0, 2);
      lcd.print("PID =             ");
      lcd.setCursor(5, 2);
      lcd.print(outputPID);
    }
  } else {
    Serial.print("| sistemStop: ");
    Serial.print(0);
    Serial.println();

    digitalWrite(RELAY_PIN_1, HIGH);
    analogWrite(SSR_PIN, 0);

    lcd.setCursor(0, 0);
    lcd.print("                ");

    lcd.setCursor(0, 1);
    lcd.print(" SISTEM KENDALI");

    lcd.setCursor(0, 2);
    lcd.print("  SUHU KEDELAI");
  }
}
