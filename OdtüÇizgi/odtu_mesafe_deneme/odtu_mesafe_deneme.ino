#define sagmotor1 4
#define sagmotor2 5
#define sagmotorpwmpin 3

#define solmotor1 8
#define solmotor2 9
#define solmotorpwmpin 10

int mz80 = 11;
int sensor_durumu;
void setup() {
  pinMode(sagmotor1, OUTPUT);
  pinMode(sagmotor2, OUTPUT);
  pinMode(sagmotorpwmpin, OUTPUT);
  pinMode(solmotor1, OUTPUT);
  pinMode(solmotor2, OUTPUT);
  pinMode(solmotorpwmpin, OUTPUT);
  pinMode(mz80, INPUT);

}

void loop() {
  sensor_durumu = digitalRead(mz80);
    digitalWrite(sagmotor1, HIGH);
    digitalWrite(sagmotor2, LOW);
    digitalWrite(solmotor1, HIGH);
    digitalWrite(solmotor2, LOW);
    analogWrite(sagmotorpwmpin,150);
    analogWrite(sagmotorpwmpin, 150);
 if(sensor_durumu == 0){
  while(sensor_durumu == 0){
    digitalWrite(sagmotor1, LOW);
    digitalWrite(sagmotor2, HIGH);
    digitalWrite(solmotor1, HIGH);
    digitalWrite(solmotor2, LOW);
    analogWrite(sagmotorpwmpin,0);
    analogWrite(sagmotorpwmpin,0);
  }
  delay(1000);
  }
}
