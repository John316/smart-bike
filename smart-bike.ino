#include <Adafruit_ssd1306syp.h> // OLED
#include "DHT.h" // temperature
#include <Servo.h> // Servo
#include <Wtv020sd16p.h> // Audio

//define OLCD
#define SDA_PIN A8
#define SCL_PIN A9
Adafruit_ssd1306syp display(SDA_PIN, SCL_PIN);

//define temperature
#define DHTPIN 48     // Data pin
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
bool printHours = true;
bool printMin = true;
bool printSec = false;

long prevmicros = 0;
int sek = 0; 
int minu = 0; 
int chas = 0; 
boolean counter = false; 

//calc speed
double oborot = 2.1;
double KM = 0.00;
int KMH = 0;
int magnitTimeout = 0;
bool stopBike = false;

short int maxSpeed = 0;
double countOborots = 0.00;
short int distance = 0;

//Gear control 
long LastChangeGearTime = 0;
int currentGear = 3;
String LastChangedGear = "";
bool servoAtMiddle = true;
int servoZero = 124;

// Battery
int battery = 0;
int batCounter = 0;
int voltBuffer = 0;

// UltraSound
int pingPin = 40;//trig
int inPin = 38;//echo

//Alarm
bool isFirst = true;
bool alarmOn = false;
// define Servo motor 
Servo backGearServo;

void setup() {

  // buttons
  pinMode(buttonGearHigh, INPUT);
  digitalWrite(buttonGearHigh, HIGH);

  pinMode(buttonGearLow, INPUT);
  digitalWrite(buttonGearLow, HIGH);

  // UltraSound
  pinMode(inPin, INPUT);
  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);

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
  
  attachInterrupt(0, magnitBlink, FALLING);

  digitalWrite(2, HIGH);

  // setup Servo motor
  backGearServo.attach(3);

  Serial.begin(9600);
  Serial.println("Start");
  
  // Set servo to zero
  backGearServo.write(servoZero);
  
  DisplayInfo(); 
  // audio ok sound
  wtv020sd16p.asyncPlayVoice(0);
  delay(500);
}

void loop() {
  if(!alarmOn){
    CalcTimeInRoad();
    TurnBackServo();
  }else{
    UnableAlarm();
    delay(100);
  }
}

void magnitBlink()
{
  timer = millis();
  double diff = timer - prevTime;
  double a = diff / 1000;
  double ms =  oborot / a;
  KM = ms * 3.6; // in double
  KMH = ceil(KM); // in int
  prevTime = timer;
  countOborots++;
  state = 1; // for check timeout
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

void UnableAlarm(){
  int dist = getDistanceToObject();
  if(dist < 100){
      if(isFirst){
        //sey Attantion
        wtv020sd16p.asyncPlayVoice(20); // вы подошли слишком близко
        delay(5000);
      }
      
      if(dist < 100 && isFirst){
        // on Alarm for a sec
        digitalWrite(buttonGearHigh, HIGH);
        delay(1000);
        // off Alarm
        digitalWrite(buttonGearHigh, LOW);
        wtv020sd16p.asyncPlayVoice(21); // это было предупреждение
        isFirst = false;
      }else if(dist < 100 && !isFirst){
        // on Alarm for 2 min.
        wtv020sd16p.asyncPlayVoice(22); // я пердупреждал
        delay(2000);
        digitalWrite(buttonGearHigh, HIGH);
        
      }
  }
}
int getDistanceToObject(){
  int distanceToObject = 0; 
  long duration, cm,tmp1;//объявляем переменные
  //посылаем датчику сигнал начала замера (высокий уровень на 10 мкс)
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(pingPin, LOW);
  //делаем паузу чтобы датчик успел среагировать
  delayMicroseconds(500);
  //засекаем время в микросекундах
  tmp1 = micros();
  //ждем пока сигнал на выходе echo не станет низкий
  while(digitalRead(inPin)==HIGH){
    //если долго нет ответа от датчика, значит препятствий в зоне видимости нет, выходим по таймауту
    if(micros() - tmp1 > 15000)
      break;
  }
  //вычисляем время "полета" ультразвукового сигнала
  duration=micros() - tmp1;
  cm = microsecondsToCentimeters(duration);//переводим время в сантиметры
  
  Serial.print("Distance: ");
  //если помех не обнаружено сообщаем что расстояние более 1.5 метра
  
  if(duration<15000){
    distanceToObject = cm;
    Serial.println(cm);//иначе выводим расстояние до помехи в сантиметрах
  }else{
    Serial.println(">1.5m");
  }
    return distanceToObject;
}

long microsecondsToCentimeters(long microseconds)
{
  //скорость звука 340 м/с или 29,412 микросекунд/см, а поскольку звук летит до помехи и обратно, делим результат на двое
  return microseconds / 29.412 / 2;
}

void CalcTimeInRoad() {
    if (micros() - prevmicros > 500000)
    { 
      prevmicros = micros();  //It is set to every half second
      counter = !counter;
      if (counter == false)
      { 
        sek++;   //a second variable + 1
        
        OperationOncePerSecond();
      }
      
      if (sek > 59) 
      {
        sek = 0; 
        minu++;
      }
      if (minu > 59) 
      {
        minu = 0; 
        chas++;
      }
      if (chas > 23)
      {
        chas = 0; 
      }
    }
}

void OperationOncePerSecond()
{
  CheckVoltage();
  CheckIsBikeStop();
  SpeedController(KMH);
  DisplayInfo();
}

void CheckIsBikeStop()
{
  if(state == 0){ //  when a second did not interrupt magnet
    localState++;
    if(localState == 3){ // if more than 3 seconds, no signal from the magnet
      KMH = 0;
      KM = 0;
      stopBike = true;
      wtv020sd16p.asyncPlayVoice(10);
    }
  }else{
    localState = 0;
    stopBike = false;
  }
  state = 0;
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
  }else if (num < currentGear){
    GearLow();
  }
}

void GearHigh(){
    //Serial.println("GearHigh");
    LastChangeGearTime = millis();
    backGearServo.write(180);
    LastChangedGear = "high";
    servoAtMiddle = false;
    currentGear++;
    // Say current gear
    wtv020sd16p.asyncPlayVoice(currentGear);
}

void GearLow(){
    //Serial.println("GearLow");
    LastChangeGearTime = millis();
    backGearServo.write(0);
    LastChangedGear = "low";
    servoAtMiddle = false;
    currentGear--;
    // Say current gear
    wtv020sd16p.asyncPlayVoice(currentGear);
}  

void TurnBackServo(){
  if(!servoAtMiddle){
    long now = millis();
    long diff = now - LastChangeGearTime;
    
    if(LastChangedGear == "high" && diff >= 400)
      SetServoToZero(diff);
      
    if(LastChangedGear == "low" && diff >= 800)
      SetServoToZero(diff);  
  }
}

void SetServoToZero(long diff){
    backGearServo.write(servoZero);
    if(LastChangedGear == "high" && diff >= 400)
      servoAtMiddle = true;

    if(LastChangedGear == "low" && diff >= 800)
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
  PrintVoltage();
  PrintDistance();
  PrintMaxSpeed();
  display.update();
}

void CheckVoltage()
{
  if(batCounter == 5)
  {
    battery = voltBuffer / 5; // midddle value
    batCounter = 0;
    voltBuffer = 0;
  }else{
    voltBuffer += readVcc();
    batCounter++;
  }
}

int getBatteryLevel()
{
  int batLevel = 0;
  if(battery < 3300)
    batLevel = 0;
  else if(battery >= 3300 && battery < 4000)
    batLevel = 1;
  else if(battery >= 4000 && battery < 4500)
    batLevel = 2;
  else if(battery > 4500)
    batLevel = 3;

 return batLevel;
}

void PrintVoltage()
{
  if(battery > 0)
    ToDrawTheBattery();
 
  int batteryLevel = getBatteryLevel();
  
  if(batteryLevel == 3)
  {
    //As tutorial display.fillRect(x, y, w, h, color);
    display.fillRect(103, 3, 6, 10, WHITE);
    display.fillRect(111, 3, 6, 10, WHITE);
    display.fillRect(119, 3, 6, 10, WHITE);
  }else if(batteryLevel == 2)
  {
    display.fillRect(111, 3, 6, 10, WHITE);
    display.fillRect(119, 3, 6, 10, WHITE);
  }else if(batteryLevel == 1)
  {
    display.fillRect(119, 3, 6, 10, WHITE);
  }
  
}

void ToDrawTheBattery()
{
  //As tutorial display.drawFastVLine(x,y,h,color);
  display.drawFastVLine(127,0,16,WHITE);
  display.drawFastHLine(101,0,26,WHITE);
  display.drawFastHLine(101,15,26,WHITE);
  display.drawFastVLine(100,0,5,WHITE);
  display.drawFastVLine(100,11,5,WHITE);
  display.drawFastVLine(97,4,8,WHITE);
  display.drawFastHLine(98,4,2,WHITE);
  display.drawFastHLine(98,11,2,WHITE);
}

void PrintGear()
{
  display.fillRect(115, 17, 12, 16, WHITE);
  display.setCursor(116, 18);
  display.setTextSize(2);
  display.setTextColor(BLACK);
  display_print(currentGear);
  display.setTextColor(WHITE);
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
  display_print("Max: ");
  display_println(maxSpeed);
}

void PrintDistance()
{
  // Graw road
  display.drawFastHLine(66,37,13,WHITE);
  
  display.drawFastHLine(66,41,3,WHITE);
  display.drawFastHLine(71,41,3,WHITE);
  display.drawFastHLine(76,41,3,WHITE);
  
  display.drawFastHLine(66,45,13,WHITE);
  
  display.setCursor(84, 38);
  display.setTextSize(1);
  display_print(distance);
  display_println(" km");
}

void PrintTermo()
{
    int h = dht.readHumidity();
    int t = dht.readTemperature();
  
    if (isnan(t) || isnan(h)) {
      //Serial.println("Error reading from DHT");
    } else {
      display.setCursor(66, 22);
      display.setTextSize(1);
      int _t = t - 2;
      int _h = h - 10;
      display_print("t");
      display.setCursor(75, 22);
      display_print(_t);
      display.drawCircle(90, 22, 2, WHITE);
      display.setCursor(96, 22);
      display_print("C");
      //display_print(_h);
      //display_print(" %");
    }
}

void PrintTime(){

  //Draw the clock
  display.drawCircle(7, 7, 7, WHITE);
  display.drawFastVLine(7, 3, 5, WHITE);
  display.drawFastHLine(8, 7, 3, WHITE);
  
  display.setCursor(19, 1);
  display.setTextSize(2);

  if(printHours){
    if (chas>=0 && chas<10) {
      //display_print("0");
      display_print(chas);
    }
    else 
    {
      display_print(chas);
    }
    display_print(":");
  }
  if(printMin){
    if (minu >= 0 && minu < 10) {
      display_print("0");
      display_print(minu);
    }
    else
    { 
      display_print(minu);
    }
  }
  if(printSec){
    display_print(":");
    
    if (sek >= 0 && sek < 10) {
      display_print("0");
      display_print(sek);
    }
    else {
    display_print(sek);
    }
  }
}

// Additional methods

long readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif  

  delay(75); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both

  long result = (high<<8) | low;

  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}

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
