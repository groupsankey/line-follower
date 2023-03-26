#include <QTRSensors.h>
QTRSensors qtr;
const uint8_t SensorCount = 7;
uint16_t sensorValues[SensorCount];

int yon;
//yön 1 sol yön 2 sağ

int Rmotor1 = 3;
int Rmotor2 =4;
int enLmotor = 8;
int Lmotor1 = 5;
int Lmotor2 = 6;
int enRmotor = 7;

float Kp = 0.25; 
float Ki = 0.3;
float Kd = 0.05;
int P;
int I;
int D;

int sayac = 0;

int lastError = 0;

int maxspeeda = 175;
int maxspeedb = 175;
int basespeeda = 100;
int basespeedb = 100;


void pid1()
{
  uint16_t position = qtr.readLineBlack(sensorValues);
  int error = 3500 - position;

  P = error;
  I = I + error;
  D = error - lastError;
  lastError = error;
  int motorspeed = P*Kp + D*Kd;
  
  int motorspeeda = basespeeda + motorspeed;
  int motorspeedb = basespeedb - motorspeed;
  
  if (motorspeeda > maxspeeda) {
    motorspeeda = maxspeeda;
  }
  if (motorspeedb > maxspeedb) {
    motorspeedb = maxspeedb;
  }
  if (motorspeeda < 0) {
    motorspeeda = 0;
  }
  if (motorspeedb < 0) {
    motorspeedb = 0;
  } 
    digitalWrite(Lmotor1,LOW);
    digitalWrite(Lmotor2,HIGH);
    digitalWrite(Rmotor1,HIGH);
    digitalWrite(Rmotor2,LOW);
    analogWrite (enRmotor, motorspeedb);
    analogWrite (enLmotor, motorspeeda);
    Serial.println(position);
  }

  
void sag90(){
   uint16_t position = qtr.readLineBlack(sensorValues);
  if(sensorValues[0]<200 && sensorValues[1]<200 && sensorValues[2]<200 && sensorValues[6]>650 && sensorValues[7]>650){
    digitalWrite(Rmotor1,HIGH);    
    digitalWrite(Rmotor2,LOW);
    digitalWrite(Lmotor1,LOW);
    digitalWrite(Lmotor2,HIGH);
    analogWrite (enRmotor, 50);
    analogWrite (enLmotor, 50);
    delay(100);
    uint16_t position = qtr.readLineBlack(sensorValues);

    while(sensorValues[0]>650 or sensorValues[0]>650)
    {
    uint16_t position = qtr.readLineBlack(sensorValues);

    digitalWrite(Rmotor1,LOW);    
    digitalWrite(Rmotor2,HIGH);
    digitalWrite(Lmotor1,LOW);
    digitalWrite(Lmotor2,HIGH);
    analogWrite (enRmotor, 75);
    analogWrite (enLmotor, 75);
    }
    return;
  }
  else{
    pid1();
  }
}

void sol90(){
   uint16_t position = qtr.readLineBlack(sensorValues);
   if(sensorValues[0]>650 && sensorValues[1]>650 && sensorValues[5]<200 && sensorValues[6]<200 && sensorValues[7]<200){
    digitalWrite(Rmotor1,HIGH);    
    digitalWrite(Rmotor2,LOW);
    digitalWrite(Lmotor1,LOW);
    digitalWrite(Lmotor2,HIGH);
    analogWrite (enRmotor, 50);
    analogWrite (enLmotor, 50);
    delay(200);
    
    uint16_t position = qtr.readLineBlack(sensorValues);

    while(sensorValues[6]>650)
    {
    uint16_t position = qtr.readLineBlack(sensorValues);
    digitalWrite(Rmotor1,HIGH);    
    digitalWrite(Rmotor2,LOW);
    digitalWrite(Lmotor1,HIGH);
    digitalWrite(Lmotor2,LOW);
    analogWrite (enRmotor, 75);
    analogWrite (enLmotor, 75);

    
    }
    return;
  }
  else{
    pid1();
  }
}

void yonlusag90(){
   uint16_t position = qtr.readLineBlack(sensorValues);
  if(yon ==1){
    sag90();
  }

  if(yon ==2){
    sol90();
  }
}

void yonlusol90(){
   uint16_t position = qtr.readLineBlack(sensorValues);
   if(yon ==1){
    sol90();
  }

  if(yon ==2){
    sag90();
  }
}

void yolsecimi(){
  if(yon ==1){
   if(sensorValues[0]>650 && sensorValues[1]>650 && sensorValues[5]<200 && sensorValues[6]<200 && sensorValues[7]<200){
     digitalWrite(Lmotor1,LOW);
    digitalWrite(Lmotor2,HIGH);
    digitalWrite(Rmotor1,HIGH);
    digitalWrite(Rmotor2,LOW);
    analogWrite (enRmotor, 100);
    analogWrite (enLmotor, 100);
    delay(1500);
    return;
     }
     else{
      pid1();
     }
   }

   if(yon == 2){
    if(sensorValues[0]<200 && sensorValues[1]<200 && sensorValues[2]<200 && sensorValues[6]>650 && sensorValues[7]>650){
       digitalWrite(Lmotor1,LOW);
    digitalWrite(Lmotor2,HIGH);
    digitalWrite(Rmotor1,HIGH);
    digitalWrite(Rmotor2,LOW);
    analogWrite (enRmotor, 100);
    analogWrite (enLmotor, 100);
    delay(1500);
    return;
   }
   else{
    pid1();
   }
}
}


void setup() {
  Serial.begin(9600);
  qtr.setTypeRC();
  //Set up the sensor array pins
  qtr.setSensorPins((const uint8_t[]){5,A0, A1, A2, A3, A4,A5}, SensorCount);
  qtr.setEmitterPin(2);
 pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); 
  
  for (uint16_t i = 0; i < 400; i++)
  {
    Serial.println(i);
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); 
  
  Serial.begin(9600);
  for (uint8_t j = 0; j < SensorCount; j++)
  {
    Serial.print(qtr.calibrationOn.minimum[j]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t k = 0; k < SensorCount; k++)
  {
    Serial.print(qtr.calibrationOn.maximum[k]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);

  uint16_t position = qtr.readLineBlack(sensorValues);
while(1 ==1){
  uint16_t position = qtr.readLineBlack(sensorValues);
if(sensorValues[0]<200 && sensorValues[1]<200 && sensorValues[2]<200 && sensorValues[6]>650 && sensorValues[7]>650){
    digitalWrite(Rmotor1,HIGH);    
    digitalWrite(Rmotor2,LOW);
    digitalWrite(Lmotor1,LOW);
    digitalWrite(Lmotor2,HIGH);
    analogWrite (enRmotor, 50);
    analogWrite (enLmotor, 50);
    delay(100);
    uint16_t position = qtr.readLineBlack(sensorValues);

    while(sensorValues[0]>650 or sensorValues[0]>650)
    {
    uint16_t position = qtr.readLineBlack(sensorValues);

    digitalWrite(Rmotor1,LOW);    
    digitalWrite(Rmotor2,HIGH);
    digitalWrite(Lmotor1,LOW);
    digitalWrite(Lmotor2,HIGH);
    analogWrite (enRmotor, 75);
    analogWrite (enLmotor, 75);
    }
    yon = 1;
    //yön sol
    return;
      }
      
    if(sensorValues[0]>650 && sensorValues[1]>650 && sensorValues[5]<200 && sensorValues[6]<200 && sensorValues[7]<200){
    digitalWrite(Rmotor1,HIGH);    
    digitalWrite(Rmotor2,LOW);
    digitalWrite(Lmotor1,LOW);
    digitalWrite(Lmotor2,HIGH);
    analogWrite (enRmotor, 50);
    analogWrite (enLmotor, 50);
    delay(200);
    
    uint16_t position = qtr.readLineBlack(sensorValues);

    while(sensorValues[6]>650)
    {
    uint16_t position = qtr.readLineBlack(sensorValues);
    digitalWrite(Rmotor1,HIGH);    
    digitalWrite(Rmotor2,LOW);
    digitalWrite(Lmotor1,HIGH);
    digitalWrite(Lmotor2,LOW);
    analogWrite (enRmotor, 75);
    analogWrite (enLmotor, 75);

    
    
    yon = 2;
    //yön sağ
    return;
   }
    }
   else{
    pid1();
   }
  
  }
}

void yol_ayrimi(){
  if(yon==1){
   if(sensorValues[0]<200 and (sensorValues[3]<200 or sensorValues[4]<200) and sensorValues[6]>650 and sensorValues[7]>650){
    sag90(); 
   }
   else{
    pid1();
   }
}
  if(yon==2){
   if(sensorValues[0]>650 and sensorValues[1]>650 and(sensorValues[3]<200 or sensorValues[4]<200) and sensorValues[7]<200){
    sol90(); 
   }
   else{
    pid1();
   }

}
}


void yol_secimi(){
  uint16_t position = qtr.readLineBlack(sensorValues);
  if(sensorValues[0]>650 and sensorValues[1]<200 and ( sensorValues[3]<200 or sensorValues[4]<200 or sensorValues[5]<200 ) and sensorValues[6]<200 and sensorValues[7]>650){
    yonlusag90();
  }
  else{
    pid1();
  }
}

void rampa(){
 uint16_t position = qtr.readLineBlack(sensorValues);
   if (sayac == 0){
    basespeeda = 180;
    basespeedb = 180;
    maxspeeda = 220;
    maxspeedb = 220;
   }
   if(sayac == 1){
    basespeeda = 75;
    basespeedb = 75;
    maxspeeda = 120;
    maxspeedb = 120;
   }
   if(sayac == 2){
    basespeeda = 50;
    basespeedb = 50;
    maxspeeda = 100;
    maxspeedb = 100;
   }
      
  if(sensorValues[0]<200 && sensorValues[1]<200 && sensorValues[2]<200 && sensorValues[6]>650 && sensorValues[7]>650){
    uint16_t position = qtr.readLineBlack(sensorValues);
  
    if (sayac = 0 ){
    digitalWrite(Rmotor1,HIGH);    
    digitalWrite(Rmotor2,LOW);
    digitalWrite(Lmotor1,LOW);
    digitalWrite(Lmotor2,HIGH);
    analogWrite (enRmotor, 150);
    analogWrite (enLmotor, 150);
    delay(200);
    while(sensorValues[0]>650 or sensorValues[1]>650)
    {
    uint16_t position = qtr.readLineBlack(sensorValues);

    digitalWrite(Rmotor1,LOW);    
    digitalWrite(Rmotor2,HIGH);
    digitalWrite(Lmotor1,LOW);
    digitalWrite(Lmotor2,HIGH);
    analogWrite (enRmotor, 50);
    analogWrite (enLmotor, 50);
    }
    }

   if(sayac = 1){
    digitalWrite(Rmotor1,LOW);    
    digitalWrite(Rmotor2,HIGH);
    digitalWrite(Lmotor1,HIGH);
    digitalWrite(Lmotor2,LOW);
    analogWrite (enRmotor, 75);
    analogWrite (enLmotor, 75);
    delay(200);
   while(sensorValues[0]>650 or sensorValues[1]>650){
    uint16_t position = qtr.readLineBlack(sensorValues);
    digitalWrite(Rmotor1,LOW);    
    digitalWrite(Rmotor2,HIGH);
    digitalWrite(Lmotor1,HIGH);
    digitalWrite(Lmotor2,LOW);
    analogWrite (enRmotor, 75);
    analogWrite (enLmotor, 50);
    }
    }
    
    if (sayac = 2){
      sol90();
    }
    sayac = sayac + 1;
  }

  if(sensorValues[0]>650 && sensorValues[1]>650 && sensorValues[5]<200 && sensorValues[6]>200 && sensorValues[7]>200){
    uint16_t position = qtr.readLineBlack(sensorValues);
  
    if (sayac = 0 ){
    digitalWrite(Rmotor1,HIGH);    
    digitalWrite(Rmotor2,LOW);
    digitalWrite(Lmotor1,LOW);
    digitalWrite(Lmotor2,HIGH);
    analogWrite (enRmotor, 150);
    analogWrite (enLmotor, 150);
    delay(200);
    while(sensorValues[6]>650 or sensorValues[7]>650)
    {
    uint16_t position = qtr.readLineBlack(sensorValues);

    digitalWrite(Rmotor1,HIGH);    
    digitalWrite(Rmotor2,LOW);
    digitalWrite(Lmotor1,HIGH);
    digitalWrite(Lmotor2,LOW);
    analogWrite (enRmotor, 50);
    analogWrite (enLmotor, 50);
    }
    }

   if(sayac = 1){
    digitalWrite(Rmotor1,LOW);    
    digitalWrite(Rmotor2,HIGH);
    digitalWrite(Lmotor1,HIGH);
    digitalWrite(Lmotor2,LOW);
    analogWrite (enRmotor, 75);
    analogWrite (enLmotor, 75);
    delay(200);
   while(sensorValues[0]>650 or sensorValues[1]>650){
    uint16_t position = qtr.readLineBlack(sensorValues);
    digitalWrite(Rmotor1,LOW);    
    digitalWrite(Rmotor2,HIGH);
    digitalWrite(Lmotor1,HIGH);
    digitalWrite(Lmotor2,LOW);
    analogWrite (enRmotor, 75);
    analogWrite (enLmotor, 50);
    }
    }

    if (sayac = 2){
      sol90();
    }
    sayac = sayac + 1;
  }
  
  else{
    pid1();
  }
}



 void beyaz_pid(){
   uint16_t position = qtr.readLineWhite(sensorValues);
  int error = 3500 - position;

  P = error;
  I = I + error;
  D = error - lastError;
  lastError = error;
  int motorspeed = P*Kp + D*Kd;
  
  int motorspeeda = basespeeda + motorspeed;
  int motorspeedb = basespeedb - motorspeed;
  
  if (motorspeeda > maxspeeda) {
    motorspeeda = maxspeeda;
  }
  if (motorspeedb > maxspeedb) {
    motorspeedb = maxspeedb;
  }
  if (motorspeeda < 0) {
    motorspeeda = 0;
  }
  if (motorspeedb < 0) {
    motorspeedb = 0;
  } 
    digitalWrite(Lmotor1,LOW);
    digitalWrite(Lmotor2,HIGH);
    digitalWrite(Rmotor1,HIGH);
    digitalWrite(Rmotor2,LOW);
    analogWrite (enRmotor, motorspeedb);
    analogWrite (enLmotor, motorspeeda);
 }

void beyaz_izleme(){
  uint16_t position = qtr.readLineBlack(sensorValues);
  if(sensorValues[0]<200){
    beyaz_pid(); 
  }
  else{
   yonlusag90();  
  }
   
}


void loop(){
    uint16_t position = qtr.readLineBlack(sensorValues);
    sag90();
    sol90();

    //rampadan önce kıvrımlı dönüş yapılmadı
   
   
}
