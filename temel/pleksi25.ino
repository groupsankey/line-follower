#include <QTRSensors.h>

#define sagmotor1 4
#define sagmotor2 5
#define sagmotorpwmpin 3

#define solmotor1 8
#define solmotor2 9
#define solmotorpwmpin 10



QTRSensorsAnalog qtra((unsigned char[]) {
  A0, A1, A2, A3, A4 , A5, A6, A7
} , 8);
unsigned int sensors[8];


float kp = 0.05;    float kd = 0.7;   float ki = 0.0001;   int integral = 0; int tabanhiz = 60;
int ekhiz = 0;      int sonhata = 0;  int hata = 0; int t_say = 0;     int saniye; int sagdonus = 0;
int sagmotorpwm = 0;    int solmotorpwm = 0;     unsigned char zemin = 1; int ilksol = 0;

void setup()
{
  // Dijital olarak kullanacağımız pinlerin giriş veya çıkış olarak tanımlıyoruz
  pinMode(sagmotor1, OUTPUT);
  pinMode(sagmotor2, OUTPUT);
  pinMode(sagmotorpwmpin, OUTPUT);
  pinMode(solmotor1, OUTPUT);
  pinMode(solmotor2, OUTPUT);
  pinMode(solmotorpwmpin, OUTPUT);
  pinMode(13, OUTPUT);


  delay(2000);

  // otomatik kalibre
  for (int i = 0; i < 150; i++)
  {
    if ( 0 <= i && i < 5   )  hafifsagadon();
    if ( 5 <= i && i  < 15   )  hafifsoladon();
    if ( 15 <= i && i < 25   )  hafifsagadon();
    if ( 25 <= i && i < 35   )  hafifsoladon();
    if ( 35 <= i && i < 40   )  hafifsagadon();
    if ( 45 <= i && i < 50   )  hafifsoladon();

    if ( i >= 50  )  {
      frenle(1);
      delay(3);
    }
    qtra.calibrate();
    delay(1);
  }



  // Serial.begin(9600);
  // do{
  //   motorkontrol(0,0); delay(10);
  //  } while(digitalRead(buton)==HIGH);
  //   delay(300);

}


void loop() {
  saniye = millis() / 1000;
  //t_yol();
  sensoroku();

  if(ilksol == 0 and saniye<20) { sagcatal();}

  if (saniye > 10 and saniye<20){  soldon(); }
  
  if (saniye>28) { tabanhiz=60; solcatal(); }
  
  
  /*
  if (sagdonus == 3) {
    tabanhiz = 60;
    solcatal();
  }
  if (sagdonus == 2) {
    if (sensors[7] > 700 && sensors[6] > 700 && sensors[3] < 100 && sensors[2] < 100 && sensors[1] < 100 && sensors[0] < 100)
    {
      sagdonus = 3;
    }
  }
  */

  //sensorlerioku_yaz();




  // PID
  integral += hata; //çizgiden uzaklaştıkça hataları toplar
  if (abs(hata) < 500)integral = 0;
  int duzeltmehizi = kp * hata + kd * (hata - sonhata) + ki * integral;
  sonhata = hata;

  // Motorlara uygulanacak kesin hız ayarları
  sagmotorpwm = tabanhiz + duzeltmehizi + ekhiz  ;
  solmotorpwm = tabanhiz - duzeltmehizi + ekhiz ;


  sagmotorpwm = constrain(sagmotorpwm, -254, 254);
  solmotorpwm = constrain(solmotorpwm, -254, 254);
  motorkontrol(solmotorpwm, sagmotorpwm);
}

void sensoroku() {
  //Çizgi sensörü pozisyon hesap kodları
  unsigned int position = qtra.readLine(sensors, 1, zemin);
  hata = position - 3500;
  // zemin değiştirme kodları
  // if ( sensors[0]<100 && sensors[7]<100  ) { zemin=0;  }//beyaz
  // if ( sensors[0]>750 && sensors[7]>750  ) { zemin=1;  }//siyah
}

void sensorlerioku_yaz() {
  for (unsigned char z = 0; z < 8; z++)
  {
    Serial.print(sensors[z]);
    Serial.print('\t');
  }
  Serial.println();
  delay(250);
}

void motorkontrol(int solmotorpwm, int sagmotorpwm) {
  if (sagmotorpwm <= 0) {
    sagmotorpwm = abs(sagmotorpwm);
    digitalWrite(sagmotor1, LOW);
    digitalWrite(sagmotor2, HIGH);
    analogWrite(sagmotorpwmpin, sagmotorpwm);
  }
  else {
    digitalWrite(sagmotor1, HIGH);
    digitalWrite(sagmotor2, LOW);
    analogWrite(sagmotorpwmpin, sagmotorpwm);
  }
  if (solmotorpwm <= 0) {
    solmotorpwm = abs(solmotorpwm);
    digitalWrite(solmotor1, LOW);
    digitalWrite(solmotor2, HIGH);
    analogWrite(solmotorpwmpin, solmotorpwm);
  }
  else {
    digitalWrite(solmotor1, HIGH);
    digitalWrite(solmotor2, LOW);
    analogWrite(solmotorpwmpin, solmotorpwm);
  }
}

void frenle(int bekle) {
  motorkontrol(0, 0);
  delay(bekle);
}

void hafifsagadon() {
  motorkontrol(80, -80);
}

void hafifsoladon() {
  motorkontrol(-80, 80);
}

void soldon() {
  if (sensors[7] < 100 && sensors[6] < 100 && sensors[5] < 100 && sensors[4] < 100 && sensors[1] > 700 && sensors[0] > 700)
  {
    motorkontrol(-90, -50);  delay(10);
    while (1)
    {
      sensoroku();
      if (sensors[7] > 700 && sensors[6] > 700) {
        break;
      }
    }
    while (1)
    {
      sensoroku();
      motorkontrol(-90, 60);
      if (sensors[7] < 100 || sensors[6] < 100)
      {
        break;
      }
    }
  }
}

void sagdon() {
  if (sensors[7] > 700 && sensors[6] > 700 && sensors[2] < 100 && sensors[2] < 100 && sensors[1] < 100 && sensors[0] < 100)
  {
    motorkontrol(-50, -90);  delay(10);
    while (1)
    {
      sensoroku();
      if (sensors[0] > 700 && sensors[1] > 700) {
        break;
      }
    }
    while (1)
    {
      sensoroku();
      motorkontrol(60, -90);
      if (sensors[0] < 100 || sensors[1] < 100)
      {
       sagdonus+=1; break;
      }
    }
  }
}

void sagcatal() {
  if (sensors[7] > 700 && sensors[6] > 700 && sensors[5] < 100 && sensors[4] < 300 && sensors[3] < 100 && sensors[2] < 100 && sensors[0] > 700)
  {

    while (1)
    {
      sensoroku();
      motorkontrol(100, 0);
      if (sensors[2] > 700)
      {
        tabanhiz = 90; break;
      }


    }
  }
}

void solcatal() {
  if (sensors[0] > 700 && sensors[1] > 700 && sensors[2] < 100 && sensors[3] < 300 && sensors[4] < 100 && sensors[5] < 100 && sensors[7] > 700)
  {

    while (1)
    {
      sensoroku();
      motorkontrol(0, 100);
      if (sensors[5] > 700)
      {
        break;
      }


    }
  }
}



void t_yol() {
  if (saniye > 8 && sensors[0] < 100 && sensors[1] < 100 && sensors[2] < 100 && sensors[3] < 100 && sensors[4] < 100 && sensors[5] < 100 && sensors[6] < 100 && sensors[7] < 100)
  {
    t_say = t_say + 1;
    while (1)
    {
      sensoroku();
      motorkontrol(tabanhiz, tabanhiz);
      if  (!(sensors[0] < 100 && sensors[1] < 100 && sensors[2] < 100 && sensors[3] < 100 && sensors[4] < 100 && sensors[5] < 100 && sensors[6] < 100 && sensors[7] < 100))
      {
        break;
      }
    }

  }
}