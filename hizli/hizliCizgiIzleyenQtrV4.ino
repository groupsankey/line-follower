//www.temrinler.com
#include <QTRSensors.h>//Qtr 4.0 kütüphane komutlarıyla yazılmıştır. Önemli not!: Qtr 4.0 öncesi kütüphanesine uygun değildir.

//pinleri tanımla

#define AIN1 11 //A sağ motor
#define AIN2 12
#define PWMA 3

#define PWMB 5 //B sol motor
#define BIN1 8
#define BIN2 9
#define STBY 10

#define sensorSayisi 8
#define sensorOrnekSayisi 4
#define LED 13
#define MZ80 2

//hız ayarı
int maxhiz = 70;//motor pwm ayarı 0-255

boolean zemin = 1; //1 siyah 0 beyaz



//PD’yi kullanmak için değişkenleri bildirme (her araca göre ayar yapılmalı)
int hata = 0; float KP = 0.03; // hata (her araca göre ayar yapılmalı)
int turev = 0; float KD = 0.5; // turev

unsigned int pozisyon = 3500;

int fark = 0; // motorlara uygulanan fark
int son_hata; // Orantılı son değer (hatanın türevini hesaplamak için kullanılır)
int hedef = 3500; // Ayar noktası (8 sensör kullandığımız için pozisyon 0 ile 7000 arasında olmalıdır, bu yüzden ideal 3500)



QTRSensors qtr;
unsigned int sensor[sensorSayisi];

void setup()
{
  //QTR-8A ayarla
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]) {
    A0, A1, A2, A3, A4, A5, A6, A7
  }, 8);

  // çıkış pinleri
  pinMode(AIN1 , OUTPUT);
  pinMode(AIN2 , OUTPUT);
  pinMode(PWMA , OUTPUT);
  pinMode(BIN1 , OUTPUT);
  pinMode(BIN2 , OUTPUT);
  pinMode(PWMB , OUTPUT);
  pinMode(LED , OUTPUT);
  pinMode(STBY , OUTPUT);
  pinMode(MZ80 , INPUT);

  delay(1000);//Araca enerji verince 1sn bekle
  // Dahili Led yanıp söndüğü sürece (3sn) Elle Kalibrasyon yap
  for ( int i = 0; i < 120; i++)
  {
    digitalWrite(LED, HIGH); delay(20);
    qtr.calibrate();
    digitalWrite(LED, LOW); delay(20);
    qtr.calibrate();
  }
  delay(3000);//Kalbirasyondan sonra 3sn bekle
  //while(digitalRead(MZ80)==0){ fren(1,1,0); }//engel kalkınca başla

}

void loop()
{

  pozisyon = qtr.readLineWhite(sensor);

  hata = pozisyon - hedef;

  turev = hata - son_hata;

  son_hata = hata;

  int fark = ( hata * KP) + ( turev * KD );

  if ( fark > maxhiz ) fark = maxhiz;
  else if ( fark < -maxhiz ) fark = -maxhiz;

  ( fark < 0 ) ?
  motor(maxhiz, maxhiz + fark) : motor(maxhiz - fark, maxhiz);
}

// sağ motor sürücü fonksiyonu
void sagmotor(int deger)
{
  if ( deger >= 0 )
  {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  }
  else
  {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    deger *= -1;
  }
  analogWrite(PWMA, deger);
}

// sol motor sürücü fonksiyonu
void solmotor(int deger)
{
  if ( deger >= 0 )
  {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  }
  else
  {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    deger *= -1;
  }
  analogWrite(PWMB, deger);
}

//motor sürücü
void motor(int sol, int sag)
{
  digitalWrite(STBY, HIGH);
  solmotor(sol);
  sagmotor(sag);
}
// www.temrinler.com
