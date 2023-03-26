#include <QTRSensors.h>
QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

int P,D,I,error,lastError;
int Rmotorf = 10;
int Rmotorb = 9;
int enRmotor = 6;
int Lmotorb = 7;
int Lmotorf = 8;
int enLmotor = 11;

int maxspeeda = 150;
int maxspeedb = 150;
int basespeeda = 60;
int basespeedb = 60;

float Kp = 0.1; 
float Ki = 0;
float Kd = 0.25;


void sol90(){
   uint16_t position = qtr.readLineBlack(sensorValues);
  if(sensorValues[0]<200 && sensorValues[1]<200  && sensorValues[6]>650){
    digitalWrite(Rmotorb,HIGH);    
    digitalWrite(Rmotorf,LOW);
    digitalWrite(Lmotorb,LOW);
    digitalWrite(Lmotorf,HIGH);
    analogWrite (enRmotor, 50);
    analogWrite (enLmotor, 50);
    delay(100);
    uint16_t position = qtr.readLineBlack(sensorValues);

    while(sensorValues[4]>200)
    {
    uint16_t position = qtr.readLineBlack(sensorValues);

    digitalWrite(Rmotorf,LOW);    
    digitalWrite(Rmotorb,HIGH);
    digitalWrite(Lmotorf,HIGH);
    digitalWrite(Lmotorb,LOW);
    analogWrite (enRmotor, 75);
    analogWrite (enLmotor, 70);
    }
    return;
  }
  else{
    pid();
  }
}


void sag90(){
   uint16_t position = qtr.readLineBlack(sensorValues);
   if(sensorValues[0]>400  && sensorValues[6]<300 && sensorValues[7]<300){
    digitalWrite(Rmotorb,HIGH);    
    digitalWrite(Rmotorf,LOW);
    digitalWrite(Lmotorb,LOW);
    digitalWrite(Lmotorf,HIGH);
    analogWrite (enRmotor, 50);
    analogWrite (enLmotor, 50);
    delay(100);
    
    uint16_t position = qtr.readLineBlack(sensorValues);

    while(sensorValues[3]>200)
    {
    uint16_t position = qtr.readLineBlack(sensorValues);
     digitalWrite(Rmotorf,HIGH);    
    digitalWrite(Rmotorb,LOW);
    digitalWrite(Lmotorf,LOW);
    digitalWrite(Lmotorb,HIGH);
    analogWrite (enRmotor, 70);
    analogWrite (enLmotor, 75);

    
    }
   }
   else{
      pid();
   }
}


void sol(){
   uint16_t position = qtr.readLineBlack(sensorValues);
  if(sensorValues[0]<200 && sensorValues[1]>400 && (sensorValues[3]<200 or sensorValues[4]<300)  && sensorValues[6]>300){
    digitalWrite(Rmotorb,LOW);    
    digitalWrite(Rmotorf,HIGH);
    digitalWrite(Lmotorb,LOW);
    digitalWrite(Lmotorf,HIGH);
    analogWrite (enRmotor, 55);
    analogWrite (enLmotor, 55);
    delay(100);
    uint16_t position = qtr.readLineBlack(sensorValues);
  }
  else{
    sag90();
    sol90();
  }
}

void setup() {
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A6,A5, A4, A3, A2, A1, A0,A7}, SensorCount);
  qtr.setEmitterPin(2);

  delay(200);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

  // analogRead() takes about 0.1 ms on an AVR.
  // 0.1 ms per sensor * 4 samples per sensor read (default) * 6 sensors
  // * 10 reads per calibrate() call = ~24 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 200; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration

  // print the calibration minimum values measured when emitters were on
  Serial.begin(9600);
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }

}


void loop() {
    sol();
    
    
  }



void pid()  {

  
  uint16_t position = qtr.readLineBlack(sensorValues);
  int error = 3500 - position;
   /*
    for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  */
  //uSerial.println(position);
   
  P = error;
  I = error + lastError;
  D = error - lastError;
  lastError = error;
  int motorspeed = P*Kp + D*Kd + I*Ki ;
  
  int motorspeeda = basespeeda + motorspeed;
  int motorspeedb = basespeedb - motorspeed;
  
  if (motorspeeda > maxspeeda) {
    motorspeeda = maxspeeda;
  }
  if (motorspeedb > maxspeedb) {
    motorspeedb = maxspeedb;
  }

  if (motorspeeda < 0) {
  motorspeeda =0;
    
  }
  if (motorspeedb < 0) {
    
    motorspeedb = 0;
    
  } 

   
    digitalWrite(Rmotorf,HIGH);
    digitalWrite(Rmotorb,LOW);
    digitalWrite(Lmotorf,HIGH);
    digitalWrite(Lmotorb,LOW);
    
    analogWrite(enRmotor,motorspeedb);
    analogWrite(enLmotor,motorspeeda);   
    Serial.print(motorspeeda);
    Serial.print("          ");
    Serial.println(motorspeedb);
    
   
    
}
