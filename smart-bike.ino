// Defined LCD Display
#include <LiquidCrystal.h>
LiquidCrystal  lcd (8, 9, 4, 5, 6, 7);

// Defined Servo motor
#include "Servo.h"
Servo servoMain;

  int reed = A11;

// Defined Magnit
int magnitPin = 22;

// Defined Giro

// Defined temp vareble
long timer = 0;
long prevTime = 0;
long prevTime2 = 0;
long prevmicros = 0;//переменная для хранения значений таймера
int sek = 0; //значение секунд
int minu = 0; //значение минут
int chas = 0; //значение часов
boolean counter = false; // счетчик для полусекунд

int magnitDetect;

// Defined main vareble
int currentGear = 1;
double oborot = 2.1;
double KM = 0.00;
double maxSpeed;
double distance = 0.00;

void setup() {
  Serial.begin(9600);
  Serial.println("Start");

  pinMode(magnitPin, INPUT);
  
  // setup LCD Display
  lcd.begin(16, 2);
  lcd.clear();
  lcd.write("Smart Bike");
  lcd.setCursor(0, 2);
  lcd.write("Welcome!");
  delay(1000);

  // setup Servo motor
  //servoMain.attach(24);

  // setup Giro

  // setup Interrupt
  //attachInterrupt(magnitPin, CalcSpeed, RISING);
}

void loop() {

  //int sign = digitalRead(magnitPin);
  //Serial.println("sign");
  //Serial.println(sign);
  //delay(500);

  int magnitDetect = digitalRead(magnitPin);
  long timer2 = millis();
  long diff = timer2 - prevTime2;
  if(magnitDetect == 0 && diff > 100){
    CalcSpeed();
    prevTime2 = timer2;
  }
  
  SpeedController();

  // Read button
  ReadButton();

  // Read sensor
  ReadSensor();

  // Calc Time
  CalcTime();
}

// Main Logic

void DisplayInfo() {
  lcd.clear();
  lcd.print("Gear:");
  lcd.setCursor(5, 0);
  lcd.print(currentGear);

  // Display current speed
  lcd.setCursor(9, 0);
  lcd.print("KM/H:");
  lcd.setCursor(14, 0);
  int KMH = ceil(KM);
  lcd.print(KMH);
}

void ReadButton() {
}

void ReadSensor() {
  // read Magnit
  //int magnitSignal = digitalRead(magnitPin);

  // call Speed calc

  // read giro
  // set angule correction;
  // active Signalization
}

void SpeedController() {
  if (KM > 1 && KM < 4) {
    if (currentGear != 5)
      SpeedSteps(5);
  } else if (KM >= 5 && KM < 10) {
    if (currentGear != 4)
      SpeedSteps(4);
  } else if (KM >= 10 && KM < 15) {
    if (currentGear != 3)
      SpeedSteps(3);
  } else if (KM >= 15 && KM < 20) {
    if (currentGear != 2)
      SpeedSteps(2);
  } else if (KM >= 20 && KM < 25) {
    if (currentGear != 1)
      SpeedSteps(1);
  }
}

void SpeedSteps(int num) {
  if (num + 1 == currentGear || num - 1 == currentGear)
    makeStep(num);
  else
    needCorrection(num);
}

void needCorrection(int num)
{
  if (num > currentGear && num != currentGear)
    int correct = num - 1;
  else if (num < currentGear && num != currentGear)
    int correct = num + 1;
}

void makeStep(int num) {
  if (num == 1)
    servoMain.write(0);
  else if (num == 2)
    servoMain.write(45);
  else if (num == 3)
    servoMain.write(90);
  else if (num == 4)
    servoMain.write(135);
  else if (num == 5)
    servoMain.write(180);

  currentGear = num;
}

void CalcSpeed() {
  timer = millis();
  double diff = timer - prevTime; // difference in second between two signal from magnit detector
  double a = diff / 1000;
  double ms =  oborot / a;
  KM = ms * 3.6; // meter / sec and conver to km/h
  prevTime = timer;

  // Calc Distance
  CalcDistance();
  CalcMaxSpeed();
}

void CalcMaxSpeed() {
  if (maxSpeed < currentGear) {
    maxSpeed = currentGear;
  }
}

void CalcTime() {
  if (micros() - prevmicros > 500000)
  { 
    prevmicros = micros();  //принимает значение каждые полсекунды
    counter = !counter;
    if (counter == false)
    { sek++;              //переменная секунда + 1
      lcd.setCursor(2, 1);
      lcd.print(":");         //выводим символ ":"между часами и минутами
      lcd.setCursor(5, 1);
      lcd.print(":");         //выводим символ ":"между  минутами и секундами
    }
    else
    {
      DisplayInfo();
      lcd.setCursor(2, 1);
      lcd.print(" ");    // мигание :
      lcd.setCursor(5, 1);
      lcd.print(" ");    // мигание :
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

    lcd.setCursor(0, 1); //выводим значение часов
    if (chas >= 0 && chas < 10) {
      lcd.print("0");
      lcd.print(chas);
    }//количество часов
    else lcd.print(chas);

    lcd.setCursor(3, 1); //выводим значение минут
    if (minu >= 0 && minu < 10) {
      lcd.print("0");
      lcd.print(minu);
    }//количество минут
    else lcd.print(minu);

    lcd.setCursor(6, 1); //выводим значение секунд
    if (sek >= 0 && sek < 10) {
      lcd.print("0");
      lcd.print(sek);
    }//количество секунд
    else lcd.print(sek);
  }
}

void CalcDistance() {
  distance += oborot / 1000;
}

// Additional Logic

void ActiveSignalization() {
}


