#include <QTRSensors.h>
#include <NewPing.h>

#define TRIGGER_PIN  6  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     12  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 200

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

int P,D,I,error,lastError;
int Rmotorf = 11;
int Rmotorb = 4;
int enRmotor = 3;
int Lmotorb = 8;
int standby=10;
int Lmotorf = 9;
int enLmotor = 5;

int a=1;

const int trigPin = 6; 
const int echoPin = 12; 

int maxspeeda = 255;
int maxspeedb = 255;
int basespeeda = 100;
int basespeedb = 100;

float Kp = 0.25; 
float Ki = 0;
float Kd = 2.5;

int sayac=0,doksansayac=0, park = 0;
long uzaklik=100;

QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];




void setup()
{
  delay(500);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  digitalWrite(standby, HIGH);// turn on Arduino's LED to indicate we are in calibration mode
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0,A1,A2,A3,A4,A5,A6,A7}, SensorCount);
  


  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

  // analogRead() takes about 0.1 ms on an AVR.
  // 0.1 ms per sensor * 4 samples per sensor read (default) * 6 sensors
  // * 10 reads per calibrate() call = ~24 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 400; i++)
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
  Serial.println();
  Serial.println();
  delay(1000);
  pinMode(trigPin,OUTPUT);
  pinMode(echoPin,INPUT);
}


void loop() {
  yol_ayrimi();
  sag90(); 
  sollama();
  soladonme();
  beyazizleme();
  Serial.println(uzaklik);
}

void sollama(){
  uint16_t position = qtr.readLineBlack (sensorValues);
  mesafe();
  if (uzaklik<25 && sayac == 0){
    digitalWrite(Rmotorf,HIGH);    
    digitalWrite(Rmotorb,LOW);
    digitalWrite(Lmotorf,LOW);
    digitalWrite(Lmotorb,HIGH);
    analogWrite (enRmotor, 75);
    analogWrite (enLmotor, 75);
    delay(400);
  while(sensorValues[3]>200){
    uint16_t position = qtr.readLineBlack (sensorValues);
    digitalWrite(Rmotorb,LOW);    
    digitalWrite(Rmotorf,HIGH);
    digitalWrite(Lmotorb,LOW);
    digitalWrite(Lmotorf,HIGH);
    analogWrite (enRmotor, 100);
    analogWrite (enLmotor, 100);
    }
    sayac++;
  }
  else{
    pid();
  }
}

void bekleme(){
  uint16_t position = qtr.readLineBlack (sensorValues);
  mesafe();
  if(uzaklik<10&&sayac==1){
   while (uzaklik<10 && sayac == 1){
    digitalWrite(Rmotorf,HIGH);    
    digitalWrite(Rmotorb,LOW);
    digitalWrite(Lmotorf,LOW);
    digitalWrite(Lmotorb,HIGH);
    analogWrite (enRmotor, 0);
    analogWrite (enLmotor, 0);
    sayac++;
    }
    delay(1000);
  }
  else{
      pid();
    }  
}

void soladonme(){
  uint16_t position = qtr.readLineBlack (sensorValues);
  if((sensorValues[6] <600 ||sensorValues[7]<600) && sayac ==2){
    digitalWrite(Rmotorf,HIGH);    
    digitalWrite(Rmotorb,LOW);
    digitalWrite(Lmotorf,LOW);
    digitalWrite(Lmotorb,HIGH);
    analogWrite (enRmotor, 100);
    analogWrite (enLmotor, 100);
    delay(250);
    sayac++;
  }
  else{
    pid();
  }
}

void beyazizleme(){
  uint16_t position = qtr.readLineBlack (sensorValues);
  if(sensorValues[0]>600 && sensorValues[7]>600 && sayac ==3 ){
    while(sensorValues[0]>600 && sensorValues[7]>600){
      beyazpid();
    }
    sayac++;
  }
  else{
    pid();
  }
}

void yol_ayrimi(){
 uint16_t position = qtr.readLineBlack (sensorValues);
  
  if((sensorValues[0]<300||sensorValues[1]<300) && (sensorValues[3]>600 || sensorValues[4]>600) && (sensorValues[6]<300||sensorValues[7]<300)){
    while(sensorValues[3]>200){
       uint16_t position = qtr.readLineBlack (sensorValues);
       digitalWrite(Rmotorb,HIGH);    
       digitalWrite(Rmotorf,LOW);
       digitalWrite(Lmotorb,LOW);
       digitalWrite(Lmotorf,HIGH);
       analogWrite (enRmotor, 55);
       analogWrite (enLmotor, 55);
    }
  }
  else{
    sol90();
  }
}


void sol90(){
   uint16_t position = qtr.readLineBlack (sensorValues);
  if(sensorValues[0]<600 && sensorValues[7]>650 && sayac != 2){
     digitalWrite(Rmotorf,LOW);    
    digitalWrite(Rmotorb,HIGH);
    digitalWrite(Lmotorf,LOW);
    digitalWrite(Lmotorb,HIGH);
    analogWrite (enRmotor, 75);
    analogWrite (enLmotor, 75);
    delay(50);
    uint16_t position = qtr.readLineBlack (sensorValues);

    while(sensorValues[4]>200)
    {
    uint16_t position = qtr.readLineBlack (sensorValues);

    digitalWrite(Rmotorf,LOW);    
    digitalWrite(Rmotorb,HIGH);
    digitalWrite(Lmotorf,HIGH);
    digitalWrite(Lmotorb,LOW);
    analogWrite (enRmotor, 55);
    analogWrite (enLmotor, 50);
    }
  }
  else{
    pid();
  }
}

void beyaz(){
  uint16_t position = qtr.readLineBlack (sensorValues);
  if(sayac == 4 &&sensorValues[0]<300&&sensorValues[1]<300&&sensorValues[2]<300&&sensorValues[3]<300&&sensorValues[4]<300&&sensorValues[5]<300&&sensorValues[6]<300&&sensorValues[7]<300){
    park++;
  }
  else{
    pid();
  }
}
void sag90(){
   uint16_t position = qtr.readLineBlack (sensorValues);
     
   if(sensorValues[0]>600  && sensorValues[7]<600 && park ==0){
     digitalWrite(Rmotorf,LOW);    
    digitalWrite(Rmotorb,HIGH);
    digitalWrite(Lmotorf,LOW);
    digitalWrite(Lmotorb,HIGH);
    analogWrite (enRmotor, 75);
    analogWrite (enLmotor, 75);
    delay(50);
    
    uint16_t position = qtr.readLineBlack (sensorValues);
    
    if(sayac>3&&doksansayac==0){
      doksansayac++;
      return;
    }

    else if(doksansayac>=2){
    digitalWrite(Rmotorf,HIGH);    
    digitalWrite(Rmotorb,LOW);
    digitalWrite(Lmotorf,LOW);
    digitalWrite(Lmotorb,HIGH);
    analogWrite (enRmotor, 50);
    analogWrite (enLmotor, 55);
    delay(500);
    }
    
    while(sensorValues[3]>200)
    {
    uint16_t position = qtr.readLineBlack (sensorValues);
     digitalWrite(Rmotorf,HIGH);    
    digitalWrite(Rmotorb,LOW);
    digitalWrite(Lmotorf,LOW);
    digitalWrite(Lmotorb,HIGH);
    analogWrite (enRmotor, 50);
    analogWrite (enLmotor, 55);

    
    }
   
   }
    else if(sensorValues[0]>600  && sensorValues[6]<600 && sensorValues[7]<600 && park ==1){
      digitalWrite(Rmotorb,HIGH);    
    digitalWrite(Rmotorf,LOW);
    digitalWrite(Lmotorb,HIGH);
    digitalWrite(Lmotorf,LOW);
    analogWrite (enRmotor, 50);
    analogWrite (enLmotor, 50);
    delay(50);
    
    uint16_t position = qtr.readLineBlack (sensorValues);
    
    while(sensorValues[3]>200)
    {
    uint16_t position = qtr.readLineBlack (sensorValues);
     digitalWrite(Rmotorf,HIGH);    
    digitalWrite(Rmotorb,LOW);
    digitalWrite(Lmotorf,LOW);
    digitalWrite(Lmotorb,HIGH);
    analogWrite (enRmotor, 70);
    analogWrite (enLmotor, 75);
     }
    digitalWrite(Rmotorb,LOW);    
    digitalWrite(Rmotorf,HIGH);
    digitalWrite(Lmotorb,LOW);
    digitalWrite(Lmotorf,HIGH);
    analogWrite (enRmotor, 100);
    analogWrite (enLmotor, 100);
    delay(1000);
    digitalWrite(Rmotorb,LOW);    
    digitalWrite(Rmotorf,HIGH);
    digitalWrite(Lmotorb,LOW);
    digitalWrite(Lmotorf,HIGH);
    analogWrite (enRmotor, 0);
    analogWrite (enLmotor, 0);
    delay(50000);
    }
   else{
      pid();
   }
}

void beyazpid(){
  uint16_t position = qtr.readLineWhite (sensorValues);
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
  //  Serial.print(motorspeeda);
    Serial.print("          ");
    //Serial.println(motorspeedb);
}

void pid()  {

  
  uint16_t position = qtr.readLineBlack (sensorValues);
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
   // Serial.print(motorspeedb);
 //   Serial.print("          ");
   // Serial.println(motorspeeda);
   // Serial.println("");
}

void mesafe(){
  pid();
  uzaklik=sonar.ping_cm();
}
