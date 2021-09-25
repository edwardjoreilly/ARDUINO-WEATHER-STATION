#include <Wire.h>
#include <SPI.h>
#include <DHT.h>
#include <LiquidCrystal.h>

//Global variables
LiquidCrystal lcd(13,11,12,10,9,8);
DHT dhtSensor(5,DHT22);

const int ldr_pin = 7;
float temperature, humidity;
int measurePin = analogRead(A0); //Connect dust sensor to Arduino A0 pin
int ledPower = 2;   //Connect 3 led driver pins of dust sensor to Arduino D2
int samplingTime = 280;
int delta = 40;
int sleep = 9680;
float voMeasured = 0;
float calcVoltage = 0;
float dustDensity = 0;

/////////////////////////
void setup() {
  Serial.begin(9600); //Begin serial
  dhtSensor.begin(); //Begin temp/humidity sensor
  lcd.begin(16,8); //Begin LCD screen
  pinMode(ledPower,OUTPUT);
}/////////////////////// End Setup

///////////////////////
void loop() {
  //Local variables
  int sensorValue = analogRead(A1);
  float voltage = sensorValue * (5.0 / 1023.0); //voltage mapped from values 0-1023
  humidity = dhtSensor.readHumidity(); //get the humidity from the sensor
  temperature = dhtSensor.readTemperature(); //get the temperature from the sensor
  digitalWrite(ledPower,LOW);
  delayMicroseconds(samplingTime);
  voMeasured = analogRead(measurePin);
  delayMicroseconds(delta);
  digitalWrite(ledPower,HIGH);
  delayMicroseconds(sleep);
  calcVoltage = voMeasured * (5.0 / 1024.0); //voltage mapped from values 0-1024
  dustDensity = (0.17*calcVoltage-0.1)*1000; //compute dust density from voltage
  float fahTemp = (1.8*temperature)+32; //convert temperature from celsius to fahrenheit
  
  //Print the temperature to the Serial Monitor and the LCD screen
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println("°F");
  lcd.print("Temperature: ");
  lcd.setCursor(0,1);
  lcd.print(fahTemp);
  lcd.print(" degrees F");
  delay(4000);
  lcd.clear();
  lcd.setCursor(0,0);
  
  //Print the humidity to the Serial Monitor and the LCD screen
  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.println("%");
  lcd.print("Humidity: ");
  lcd.setCursor(0,1);
  lcd.print(humidity);
  lcd.println("%");
  delay(4000);
  lcd.clear();
  lcd.setCursor(0,0);

  //Print the UV Index to the Serial Monitor and the LCD screen
  Serial.print("UV index = ");
  Serial.println(voltage/.1);
  lcd.print("UV index = ");
  lcd.setCursor(0,1);
  lcd.print(voltage/.1);
  delay(4000);
  lcd.clear();
  lcd.setCursor(0,0);

  //Print the dust density (units: ug/m^3) to the Serial Monitor and the LCD screen
  Serial.print("Dust density (ug/m^3): ");
  Serial.println(dustDensity);
  lcd.print("Dust density: ");
  lcd.setCursor(0,1);
  lcd.print(dustDensity);
  lcd.print(" ug/m^3");
  delay(4000);
  lcd.clear();
  lcd.setCursor(0,0);

  //Print whether light is detected or not to the Serial Monitor and the LCD screen
  Serial.print("Light detected: ");
  lcd.print("Light detected: ");
  lcd.setCursor(0,1);
  if(digitalRead( ldr_pin )){
    Serial.println("No");
    lcd.print("No");
  }
  else{
    Serial.println("Yes\n\n\n");
    lcd.print("Yes");
  }
  delay(4000);
  lcd.clear();
  lcd.setCursor(0,0);

  //Pause for 2 seconds after all data has been printed
  delay(2000);
}/////////////// End Loop
