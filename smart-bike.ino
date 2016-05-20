// Defined LCD Display
#include <Adafruit_ssd1306syp.h>


#include <Wtv020sd16p.h>
#include <Servo.h>

//define OLCD
#define SDA_PIN A8
#define SCL_PIN A9
Adafruit_ssd1306syp display(SDA_PIN, SCL_PIN);

// define Servo motor 
Servo backGearServo;

// define voice module
int resetPin = 7;  // The pin number of the reset pin 7.
int clockPin = 6;  // The pin number of the clock pin 8.
int dataPin = 5;  // The pin number of the data pin 5.
int busyPin = 4;  // The pin number of the busy pin 4.
Wtv020sd16p wtv020sd16p(resetPin,clockPin,dataPin,busyPin);



// temperatura
#include "DHT.h"
#define DHTPIN 48     // к какому пину будет подключен вывод Data

//выбор используемого датчика
#define DHTTYPE DHT11   // DHT 11 
//#define DHTTYPE DHT22   // DHT 22  (AM2302)
//#define DHTTYPE DHT21   // DHT 21 (AM2301)

//инициализация датчика
DHT dht(DHTPIN, DHTTYPE);


// define Magnit
 int magnitPin = 2;
// Interapt
int interrupt = 0;
volatile int state = 0;
volatile int statePin = LOW;

// define Giro

// define temp vareble
long timer = 0;
int magnitTimeout = 0;
int localState = 0;

// VoiceComment
long prevTime3 = 0;
int IsSay = 0;

// Clock 
bool printHours = false;
long prevmicros = 0;//переменная для хранения значений таймера
int sek = 0; //значение секунд
int minu = 0; //значение минут
int chas = 0; //значение часов
boolean counter = false; // счетчик для полусекунд
boolean noSayHello = true;

// define main vareble
int currentGear = 1;
double oborot = 2.1;
double KM = 0.00;
int maxSpeed;
double distance = 0.00;
bool stopBike = false;

void magnitBlink()
{
  state++;
}

void setup() {
  Serial.begin(9600);
  Serial.println("Start");
  
  dht.begin();
  wtv020sd16p.reset();
  
  attachInterrupt(interrupt, magnitBlink, FALLING);
  
  // setup Servo motor
  backGearServo.attach(2);
  
  // OLED 
  display.initialize();
  display.clear();
  display.setTextColor(WHITE);
  display.setTextSize(2);
  display.println("Smart Bike");
  display.println("Welcome!");
  display.update();
  
  if(noSayHello == true)
    SeyHello();
    
  delay(3000);
 
  DisplayInfo();
  
  // setup Giro

}

void loop() {
  
  digitalWrite(magnitPin, statePin);
  // Calc Time and Speed
  CalcTime();
}

void SeyHello()
{
  wtv020sd16p.asyncPlayVoice(0);
  noSayHello = false;
}

void CalcTime() {

  if(!stopBike){
    if (micros() - prevmicros > 500000)
    { 
      prevmicros = micros();  //принимает значение каждые полсекунды
      counter = !counter;
      if (counter == false)
      { 
        localState = state;
    state = 0;
    
    sek++;    //переменная секунда + 1
    
    CalcSpeed();
    DisplayInfo();
      }
      else
      {
     
      }
  
      if (sek > 59) //если переменная секунда больше 59 ...
      {
        sek = 0; //сбрасываем ее на 0
        minu++;//пишем +1 в переменную минута
      }
      if (minu > 59) //если переменная минута больше 59 ...
      {
        minu = 0; //сбрасываем ее на 0
        chas++;//пишем +1 в переменную час
      }
      if (chas > 23) //если переменная час больше 23 ...
      {
        chas = 0; //сбрасываем ее на 0
      }
      //PrintTime();
    }
  }else if(state > 0){
  stopBike = false;
  magnitTimeout = 0;
  }
  
  //if 3 sec we have no signal from magnit
  if(magnitTimeout == 3){
    stopBike = true;
    KM = 0;
  magnitTimeout++;
  DisplayInfo();
  }
}

// Main Logic

void CalcSpeed() {
  
  
  if(localState > 0){
  double ms =  state * oborot / (magnitTimeout + 1);
  KM = ms * 3.6; // meter / sec and conver to km/h
  magnitTimeout = 0;
  stopBike = false;
  }else{
  magnitTimeout++;
  }
  
  // Calc Distance
  CalcDistance();
  int KMH = ceil(KM);
  CalcMaxSpeed(KMH);
  SpeedController(KMH);
  VoiceComment(KMH);
}

void CalcDistance() {
  distance += oborot / 1000;
}


void DisplayInfo() {
  display.clear();
  display.setCursor(0, 0);  
  PrintTime();
  PrintGear();
  PrintSpeed();
  PrintTermo();  
  PrintDistance();
  PrintMaxSpeed();

  display.update();
}

void PrintGear()
{
  display.setTextSize(2);
  display_print("  G");
  display_println(currentGear);
}

void PrintTermo()
{
    // температура
    int h = dht.readHumidity();
    int t = dht.readTemperature();
  
    // проверяем правильные ли данные получили
    if (isnan(t) || isnan(h)) {
      Serial.println("Error reading from DHT");
    } else {
      display.setCursor(66, 22);
      display.setTextSize(1);  
      display_print(t);
      display_print(" *C ");
      display_print(h);
      display_print(" %");
    }
}

void PrintSpeed()
{
  // Display current speed
  int KMH = ceil(KM);
  display.setTextSize(5);
  display.setCursor(0, 22);
  display_println(KMH);
}

void PrintMaxSpeed()
{
  display.setCursor(66, 53);
  display.setTextSize(1);
  display_print("Max:");
  display_println(maxSpeed);
}

void PrintDistance()
{
  display.setCursor(66, 37);
  display.setTextSize(1);
  display_print("Dist: ");
  display_println(distance);
}

void PrintTime(){
    
  if(printHours){
    display.setTextSize(2);
    if (chas>=0 && chas<10) {
      display_print("0");
      display_print(chas);
    }
    else 
    {
      display_print(chas);
    }
  }
     //display_print(":");
    display_print(" ");
    if (minu >= 0 && minu < 10) {
      display_print("0");
      display_print(minu);
    }
    else
  { 
    display_print(minu);
  }
    
    display_print(":");
    
    if (sek >= 0 && sek < 10) {
      display_print("0");
      display_print(sek);
    }
    else {
    display_print(sek);
  }
  
  // read giro
  // set angule correction;
  // active Signalization
}

void SpeedController(int KMH) {
  if (KMH > 1 && KMH < 5) {
    if (currentGear != 1){
      //GearSteps(1);
    }
  } else if (KMH >= 5 && KMH < 7) {
    if (currentGear != 2){
      //GearSteps(2);
    }
  } else if (KMH >= 7 && KMH < 10) {
    if (currentGear != 3)
      GearSteps(3);
  } else if (KM >= 10 && KMH < 15) {
    if (currentGear != 4)
      GearSteps(4);
  } else if (KM >= 15 && KMH < 20) {
    if (currentGear != 5)
      GearSteps(5);
  }else if (KM >= 20 && KMH < 25) {
    if (currentGear != 6)
      GearSteps(6);
  }else if (KM >= 25 && KMH < 30) {
    if (currentGear != 7)
      GearSteps(7);
  }else if (KM > 30) {
    if (currentGear != 8)
      GearSteps(8);
  }
}

void GearSteps(int num) {

  makeStep(num);

  // TODO: Correction don't work.
  //if (num == currentGear + 1 || num == currentGear - 1)
    //makeStep(num);
  //else
    //needCorrection(num);
}

void needCorrection(int num){
  if (num > currentGear && num != currentGear)
    int correct = num - 1;
  else if (num < currentGear && num != currentGear)
    int correct = num + 1;
}

void makeStep(int num) {
  if (num == 1)
    TurnFirstGear(currentGear);
  else if (num == 2)
    TurnSecondGear(currentGear);
  else if (num == 3)
    TurnThirdGear(currentGear);
  else if (num == 4)
    TurnFourthGear(currentGear);
  else if (num == 5)
    TurnFifthGear(currentGear);
  else if (num == 6)
    TurnSixthGear(currentGear);
  else if (num == 7)
    TurnSeventhGear(currentGear);
  else if (num == 8)
    TurnEighthGear(currentGear);

  currentGear = num;
}

void TurnFirstGear(int currentGear){
  if (currentGear > 1)
  {
    /* code */
  }else{

  }
  wtv020sd16p.asyncPlayVoice(1);
}

void TurnSecondGear(int currentGear){
  if (currentGear > 2)
  {
    /* code */
  }else{

  }
  wtv020sd16p.asyncPlayVoice(2);
}

void TurnThirdGear(int currentGear){
  if (currentGear > 3)
  {
    /* code */
  }else{

  }
  wtv020sd16p.asyncPlayVoice(3);
}

void TurnFourthGear(int currentGear){
  if (currentGear > 4)
  {
    /* code */
  }else{

  }
  wtv020sd16p.asyncPlayVoice(4);
}

void TurnFifthGear(int currentGear){
  if (currentGear > 5)
  {
    /* code */
  }else{

  }
  wtv020sd16p.asyncPlayVoice(5);
}

void TurnSixthGear(int currentGear){
  if (currentGear > 6)
  {
    /* code */
  }else{

  }
  wtv020sd16p.asyncPlayVoice(6);
}

void TurnSeventhGear(int currentGear){
  if (currentGear > 7)
  {
    /* code */
  }else{

  }
  wtv020sd16p.asyncPlayVoice(7);
}

void TurnEighthGear(int currentGear){
  if (currentGear > 8)
  {
    /* code */
  }else{

  }
  wtv020sd16p.asyncPlayVoice(8);
}

void CalcMaxSpeed(int KMH) {
  if (maxSpeed < KMH) {
    maxSpeed = KMH;
  }
}

void VoiceComment(int KMH){
  long diff = timer - prevTime3;
  if(diff > 10000){
    IsSay = 0;
  }
    if(KMH > 17 && KMH < 19 && IsSay == 0){
      prevTime3 = timer;
      wtv020sd16p.asyncPlayVoice(10);
      IsSay = 1;
    }
    if(KMH > 20 && KMH < 22 && IsSay == 0){
      prevTime3 = timer;
      wtv020sd16p.asyncPlayVoice(12);
      IsSay = 1;
    }
    if(KMH > 31 && KMH < 32 && IsSay == 0){
      wtv020sd16p.asyncPlayVoice(30);
      IsSay = 1;
      prevTime3 = timer;
    }
    if(KMH > 41 && KMH < 42){
      wtv020sd16p.asyncPlayVoice(40);
      IsSay = 1;
      prevTime3 = timer;
    }
    if(KMH > 50){
      wtv020sd16p.asyncPlayVoice(50);
      wtv020sd16p.asyncPlayVoice(11);
      IsSay = 1;
      prevTime3 = timer;
    }
}


// Additional methods

void display_println(String mess) {
  display.println(mess);
}

void display_println(int num) {
  if(num >= 0)
    display.println(num);
}

void display_print(int num) {
  if(num >= 0)
    display.print(num);
}

void display_print(double num) {
  if(num >= 0)
    display.print(num);
}

void display_println(double num) {
  if(num >= 0)
    display.println(num);
}

void display_print(String mess) {
  display.print(mess);
}



