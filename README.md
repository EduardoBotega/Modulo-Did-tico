# Modulo-Did-tico
Codigo em linguagem C++ para fazer o controle de temperatura através de uma resistência elétrica, possuindo leituras de temperatura de uma superfície aletada (comentarios estarão com "//" no inicio)


#include <TM1637Display.h>  //Biblioteca para o funcionamento do display de sete segmentos e quatro digitos
#include <max6675.h>        //Biblioteca para o funcionamento dos Thermopares
#include <PID_v1.h>         //Biblioteca para o funcionamento do controle PID
#include <Wire.h>           //Biblioteca para a comunicação com o módulo I2C do display   

int potPin = A7;            //potenciômetro
int pinHeatBed = 11;        //Rele

int pinThermoDO = 12;       //Thermopares                           
int pinThermoCLK = 9; 
int pinThermoCS1 = 10;
int pinThermoCS2 = 8;   
int pinThermoCS3 = 7;

double temperatureFin1;    
double temperatureFin2;     //temperaturas
double temperaturePlate;
double minPWM = 0;
double maxPWM = 255;
double currentTemperature;
double currentPWM;
double setpoint;
float KP = 0.07;
float KI = 0.98;
float KD = 25.5;

long currentTime;

MAX6675 thermocouple1(pinThermoCLK, pinThermoCS1, pinThermoDO);
MAX6675 thermocouple2(pinThermoCLK, pinThermoCS2, pinThermoDO);
MAX6675 thermocouple3(pinThermoCLK, pinThermoCS3, pinThermoDO);

PID reflowPID(&temperaturePlate, &currentPWM, &setpoint, KP, KI, KD, DIRECT);
 
TM1637Display display(2, 3);

void setup() {
  display.setBrightness(0x0f);

  pinMode(pinHeatBed, OUTPUT);       
  digitalWrite(pinHeatBed, LOW);

  Serial.begin(9600);

  reflowPID.SetMode(AUTOMATIC);
  reflowPID.SetOutputLimits(minPWM, maxPWM);
}

void loop() {
  int potValue = analogRead(potPin);
  currentTemperature = map(potValue, 0, 1023, 50, 150);
  setpoint  = currentTemperature;

  // Exibe o valor mapeado no display
  display.showNumberDec(currentTemperature);
  temperaturePlate = thermocouple1.readCelsius();
  temperatureFin1 = thermocouple2.readCelsius();
  temperatureFin2 = thermocouple3.readCelsius();

  //Para plotar o gráfico e mostrar os dados no monitor serial
  currentTime = (millis()/1000);
  Serial.print(setpoint);
  Serial.print(",");
  Serial.print(currentPWM);
  Serial.print(",");
  Serial.print(temperaturePlate);
  Serial.print(",");
  Serial.print(temperatureFin1);  
  Serial.print(",");
  Serial.println(temperatureFin2);  
  delay(1000);
  reflowPID.Compute();
  
  analogWrite(pinHeatBed, currentPWM);  

  delay(100);
}
