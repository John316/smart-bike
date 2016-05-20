#include <Adafruit_ssd1306syp.h> // OLED
#include "DHT.h" // temperature
#include <Servo.h> // Servo
#include <Wtv020sd16p.h> // Audio

//define OLCD
#define SDA_PIN A8
#define SCL_PIN A9
Adafruit_ssd1306syp display(SDA_PIN, SCL_PIN);

//define temperature
#define DHTPIN 48     // к какому пину будет подключен вывод Data
#define DHTTYPE DHT11   // DHT 11 
DHT dht(DHTPIN, DHTTYPE); //init

// Audio
int resetPin = 7;
int clockPin = 6;
int dataPin = 5;
int busyPin = 4;
Wtv020sd16p wtv020sd16p(resetPin,clockPin,dataPin,busyPin);

int buttonGearHigh = 26;
int buttonGearLow = 28;

volatile int state = 0;
volatile long prevTime = 0;
volatile long timer = 0;
int localState = 0;
// Clock 
bool printHours = false;
long prevmicros = 0;//переменная для хранения значений таймера
int sek = 0; //значение секунд
int minu = 0; //значение минут
int chas = 0; //значение часов
boolean counter = false; // счетчик для полусекунд

//calc speed
double oborot = 2.1;
double KM = 0.00;
int KMH = 0;
int magnitTimeout = 0;
bool stopBike = false;

double maxSpeed = 0.00;
double countOborots = 0.00;
double distance = 0.00;

//Gear control 
long LastChangeGearTime = 0;
int currentGear = 3;
String LastChangedGear = "";
bool servoAtMiddle = true;
int servoZero = 124;

// define Servo motor 
Servo backGearServo;

void setup() {

  // buttons
  pinMode(buttonGearHigh, INPUT);
  digitalWrite(buttonGearHigh, HIGH);

  pinMode(buttonGearLow, INPUT);
  digitalWrite(buttonGearLow, HIGH);

  // Audio
  dht.begin();
  wtv020sd16p.reset();
  
  // OLED 
  display.initialize();
  display.clear();
  display.setTextColor(WHITE);
  display.setTextSize(2);
  display.println("Smart Bike");
  display.println("Welcome!");
  display.update();
  
  // put your setup code here, to run once:
  attachInterrupt(0, magnitBlink, FALLING);

  digitalWrite(2, HIGH);

  // setup Servo motor
  backGearServo.attach(3);

  Serial.begin(9600);
  Serial.println("Start");
  SetServoToZero();
  DisplayInfo();
  // audio ok sound
  wtv020sd16p.asyncPlayVoice(0);
  delay(2000);
}

void loop() {
  CalcTimeInRoad();
  TurnBackServo();
}
void magnitBlink()
{
  timer = millis();
  double diff = timer - prevTime;
  double a = diff / 1000;
  double ms =  oborot / a;
  KM = ms * 3.6; 
  prevTime = timer;
  countOborots++;
  state = 1;
}

void readButtons(){
  if (digitalRead(buttonGearHigh) == HIGH) {
    Serial.println("buttonGearHigh");
    setManualGearHigh();
    delay(50);
  }
  if (digitalRead(buttonGearLow) == HIGH) {
    Serial.println("buttonGearHigh");
    setManualGearLow();
    delay(50);
  }
}
void CalcTimeInRoad() {
    if (micros() - prevmicros > 500000)
    { 
      prevmicros = micros();  //принимает значение каждые полсекунды
      counter = !counter;
      if (counter == false)
      { 
        sek++;    //переменная секунда + 1
        
        if(state == 0){ // когда за секунду не было прерываний магнитом
          localState++;
          if(localState == 3){ // если 3 секунды нет сигнала от магнита
            KMH = 0;
            KM = 0;
            stopBike = true;
            wtv020sd16p.asyncPlayVoice(10);
          }
        }else{
          localState = 0;
          stopBike = false;
        }
        KMH = ceil(KM);
        SpeedController(KMH);
        DisplayInfo();
        state = 0;
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
    }

}

void CalcMaxSpeed(double KMH){
  if(maxSpeed < KMH)
    maxSpeed = KMH;
}

void CalcDistance() {
  distance = countOborots * oborot / 1000;
}

void SpeedController(int KMH) {
  if (KMH > 1 && KMH < 5) {
    if (currentGear != 1)
      ChangeGear(3);
  } else if (KMH >= 5 && KMH < 7) {
    if (currentGear != 2)
      ChangeGear(3);
  } else if (KMH >= 7 && KMH < 13) {
    if (currentGear != 3)
      ChangeGear(3);
  } else if (KM >= 13 && KMH < 18) {
    if (currentGear != 4)
      ChangeGear(4);
  } else if (KM >= 18 && KMH < 24) {
    if (currentGear != 5)
      ChangeGear(5);
  }else if (KM >= 24 && KMH < 29) {
    if (currentGear != 6)
      ChangeGear(6);
  }else if (KM > 29) {
    if (currentGear != 7)
      ChangeGear(7);
  }
 
  if(!servoAtMiddle)
    TurnBackServo();

  CalcMaxSpeed(KMH);
  CalcDistance();
}

void ChangeGear(int num) {
  if(servoAtMiddle){
    makeStep(num);
  }
}

void makeStep(int num) {
  if (num > currentGear){
    GearHigh();
    currentGear++;
    // Say current gear
    wtv020sd16p.asyncPlayVoice(currentGear);
  }else if (num < currentGear){
    GearLow();
    currentGear--;
    // Say current gear
    wtv020sd16p.asyncPlayVoice(currentGear);
  }
}

void GearHigh(){
    //Serial.println("GearHigh");
    LastChangeGearTime = millis();
    backGearServo.write(180);
    LastChangedGear = "high";
    servoAtMiddle = false;
}

void GearLow(){
    //Serial.println("GearLow");
    LastChangeGearTime = millis();
    backGearServo.write(0);
    LastChangedGear = "low";
    servoAtMiddle = false;
}  

void TurnBackServo(){
  if(!servoAtMiddle){
    long now = millis();
    long diff = now - LastChangeGearTime;
    
    if(LastChangedGear == "high" && diff >= 500)
      SetServoToZero();
    if(LastChangedGear == "low" && diff >= 900)
      SetServoToZero();  
  }
}

void SetServoToZero(){
    backGearServo.write(servoZero);
    servoAtMiddle = true;
}

void setManualGearHigh(){
  currentGear++;
}

void setManualGearLow(){
  currentGear--;
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

void PrintSpeed()
{
  // Display current speed
  if(KMH < 99){
    display.setTextSize(5);
    display.setCursor(0, 22);
    display_println(KMH);
  }
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

void PrintTermo()
{
    // температура
    int h = dht.readHumidity();
    int t = dht.readTemperature();
  
    // проверяем правильные ли данные получили
    if (isnan(t) || isnan(h)) {
      //Serial.println("Error reading from DHT");
    } else {
      display.setCursor(66, 22);
      display.setTextSize(1);
      int _t = t - 2;
      int _h = h - 10;
      display_print(_t);
      display_print(" *C ");
      display_print(_h);
      display_print(" %");
    }
}

void PrintTime(){
  display.setTextSize(2);
  if(printHours){
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
