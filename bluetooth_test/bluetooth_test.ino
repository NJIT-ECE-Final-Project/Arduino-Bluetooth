const int DELAY_MS = 20; // 20 ms delay to achieve 50 Hz

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {

  // get the current time
  int begin_ms = millis();
  
  // put your main code here, to run repeatedly:
  Serial.println("Hello, world!");

  int ellapsed = millis() - begin_ms;

  if(ellapsed < DELAY_MS) {
    delay(DELAY_MS - ellapsed);
  } else {
    Serial.println("Not fast enough!");
  }
}
