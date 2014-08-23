

uint32_t timer;

void setup() {
  InitSensors();
  InitValues();
  
}

double RollAngle=0;

void loop() {
  
  RollAngle += getGyroRoll();
  timer = micros();

  Serial.print(getAccRoll());
  Serial.print(" : ");
  Serial.println(RollAngle);
  delay(10);

}

