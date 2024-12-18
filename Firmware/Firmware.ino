#define BLYNK_TEMPLATE_ID "TMPL65aDG9afW"
#define BLYNK_TEMPLATE_NAME "Huỳnh Nhật Tiến"
#define BLYNK_AUTH_TOKEN "wNBLT1F7kVcIwdII3_uld8otJAKTFjLf"
 

 
bool fetch_blynk_state = true;  
#include <WiFiManager.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <HardwareSerial.h>
#include <Adafruit_MAX31865.h>
#include <SPI.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <AceButton.h>
#include <Arduino.h>
#include <driver/adc.h>
#include "esp_adc_cal.h"
#include <stdlib.h>
float current_mA;
float inputVoltage;
const float conversionFactor = 6.66666667; // Hệ số nhân để chuyển từ V sang mA
#define pin 36
#define pinV 35
#define DEFAULT_VREF    3340        
static esp_adc_cal_characteristics_t *adc_chars;    
static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;
using namespace ace_button;
#define RelayPin1 12  
#define RelayPin2 14  
#define RelayPin3 33  
#define RelayPin4 32
#define SwitchPin1 15 
#define SwitchPin2 2
#define SwitchPin3 4 
#define SwitchPin4 34 
#define TRIGGER_PIN 39

int timeout = 120;
#define VPIN_BUTTON_1    V3
#define VPIN_BUTTON_2    V4
#define VPIN_BUTTON_3    V2
#define VPIN_BUTTON_4    V5
#define VPIN_TEMPERATURE V0 
#define VPIN_HUMIDITY V6
bool toggleState_1 = LOW;
bool toggleState_2 = LOW;
bool toggleState_3 = LOW; //Define integer to remember the toggle state for relay 3
bool toggleState_4 = LOW;
int wifiFlag = 0;
char auth[] = BLYNK_AUTH_TOKEN;
ButtonConfig config1;
AceButton button1(&config1);
ButtonConfig config2;
AceButton button2(&config2);
ButtonConfig config3;
AceButton button3(&config3);
ButtonConfig config4;
AceButton button4(&config4);
void handleEvent1(AceButton*, uint8_t, uint8_t);
void handleEvent2(AceButton*, uint8_t, uint8_t);
void handleEvent3(AceButton*, uint8_t, uint8_t);
void handleEvent4(AceButton*, uint8_t, uint8_t);
BlynkTimer timer;
BLYNK_WRITE(VPIN_BUTTON_1)
{
  toggleState_1 = param.asInt();
  digitalWrite(RelayPin1, toggleState_1);
}
BLYNK_WRITE(VPIN_BUTTON_2)
{
  toggleState_2 = param.asInt();
  digitalWrite(RelayPin2, toggleState_2);
}
BLYNK_WRITE(VPIN_BUTTON_3)
{
  toggleState_3 = param.asInt();
  digitalWrite(RelayPin3, toggleState_3);
}
BLYNK_WRITE(VPIN_BUTTON_4)
{
  toggleState_4 = param.asInt();
  digitalWrite(RelayPin4, toggleState_4);
}
void checkBlynkStatus()
{
  bool isconnected = Blynk.connected();
  if (isconnected == false)
  {
    wifiFlag = 1;
    Serial.println("Blynk Not Connected");
  }
  if (isconnected == true)
  {
    wifiFlag = 0;
    if (!fetch_blynk_state)
    {
      Blynk.virtualWrite(VPIN_BUTTON_1, toggleState_1);
      Blynk.virtualWrite(VPIN_BUTTON_2, toggleState_2);
      Blynk.virtualWrite(VPIN_BUTTON_3, toggleState_3);
      Blynk.virtualWrite(VPIN_BUTTON_4, toggleState_4);
    }
    Serial.println("Blynk Connected");
  }
}
BLYNK_CONNECTED()
{
  if (fetch_blynk_state)
  {
    Blynk.syncVirtual(VPIN_BUTTON_1);
    Blynk.syncVirtual(VPIN_BUTTON_2);
    Blynk.syncVirtual(VPIN_BUTTON_3);
    Blynk.syncVirtual(VPIN_BUTTON_4);
  }
}
#define simSerial               Serial2
#define SET_BAUDRATE      "AT+IPREX=115200"         
#define MCU_SIM_BAUDRATE        115200
#define MCU_SIM_TX_PIN              17
#define MCU_SIM_RX_PIN              16
#define MCU_SIM_EN_PIN              15
#define PHONE_NUMBER                "+84523202797"

#define DHT_PIN 13
float humidity = 0.0;
float temp = 0.0;
#define SCK 18
#define MISO 19
#define MOSI 23
#define CS 5
Adafruit_MAX31865 thermo = Adafruit_MAX31865(CS, MOSI, MISO, SCK);
#define RREF      430.0
#define RNOMINAL  100.0


LiquidCrystal_I2C lcd(0x27, 20, 4); 
const int encoderPinA = 25;
const int encoderPinB = 26;
const int buttonPin = 27; 
volatile boolean aSetLast = false;
volatile boolean bSetLast = false;
bool tt = true;
int numberOffunction = 5;
int selectedfunction = 0;
String DefautMenu[]={"NHIET DO PT100:","DO AM DHT22:", "ADC 4-20mA:", "ADC 0-10V", "WI-FI:" };
void readPT100(float *temperature) {
  uint16_t rtd = thermo.readRTD();
  Serial.print("RTD value: "); Serial.println(rtd);
  float ratio = rtd;
  ratio /= 32768;
  Serial.print("Ratio = "); Serial.println(ratio,8);
  Serial.print("Resistance = "); Serial.println(RREF*ratio,8);
   *temperature = thermo.temperature(RNOMINAL, RREF);
}
void read_dht22(float *hum) {
  uint8_t data[5] = {0, 0, 0, 0, 0};
  // start
  pinMode(DHT_PIN, OUTPUT);
  digitalWrite(DHT_PIN, LOW);
  delay(20);
  digitalWrite(DHT_PIN, HIGH);
  delayMicroseconds(40);
  pinMode(DHT_PIN, INPUT);

  // Chờ DHT22
  while (digitalRead(DHT_PIN) == HIGH);
  while (digitalRead(DHT_PIN) == LOW);
  while (digitalRead(DHT_PIN) == HIGH);

  // Đọc giá trị
  for (int i = 0; i < 5; i++) {
    for (int j = 0; j < 8; j++) {
      while (digitalRead(DHT_PIN) == LOW);
      delayMicroseconds(30);
      if (digitalRead(DHT_PIN) == HIGH) {
        data[i] |= (1 << (7 - j));
      }
      while (digitalRead(DHT_PIN) == HIGH);
    }
  }
  *hum = ((data[0] << 8  ) + data[1]) / 10.0;
}
void sim_at_wait()
{
    delay(100);
    while (simSerial.available()) {
        Serial.write(simSerial.read());
    }
}

bool sim_at_cmd(String cmd){
    simSerial.println(cmd);
    sim_at_wait();
    return true;
}
bool sim_at_send(char c){
    simSerial.write(c);
    return true;
}
void sent_sms()
{
    sim_at_cmd("AT+CMGF=1");
    String temp = "AT+CMGS=\"";
    temp += (String)PHONE_NUMBER;
    temp += "\"";
    sim_at_cmd(temp);
    sim_at_cmd("Canh Bao Nhiet Do Cao");

    // End charactor for SMS
    sim_at_send(0x1A);
}
void call()
{
    String temp = "ATD";
    temp += PHONE_NUMBER;
    temp += ";";
    sim_at_cmd(temp); 

    delay(20000);

    // Hang up
    sim_at_cmd("ATH"); 
}

void setup()
{
  Serial.begin(115200);
  
  adc_chars = (esp_adc_cal_characteristics_t *)calloc(1, sizeof(esp_adc_cal_characteristics_t));
  esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
  pinMode(RelayPin1, OUTPUT);
  pinMode(RelayPin2, OUTPUT);
  pinMode(RelayPin3, OUTPUT);
  pinMode(RelayPin4, OUTPUT);
  pinMode(SwitchPin1, INPUT_PULLUP);
  pinMode(SwitchPin2, INPUT_PULLUP);
  pinMode(SwitchPin3, INPUT_PULLUP);
  pinMode(SwitchPin4, INPUT_PULLUP);
  WiFi.mode(WIFI_STA); 
  Serial.println("\n Starting");
  pinMode(TRIGGER_PIN, INPUT_PULLUP);
  digitalWrite(RelayPin1, toggleState_1);
  digitalWrite(RelayPin2, toggleState_2);
  digitalWrite(RelayPin3, toggleState_3);
  digitalWrite(RelayPin4, toggleState_4);
  config1.setEventHandler(button1Handler);
  config2.setEventHandler(button2Handler);
  config3.setEventHandler(button3Handler);
  config4.setEventHandler(button4Handler);
  button1.init(SwitchPin1);
  button2.init(SwitchPin2);
  button3.init(SwitchPin3);
  button4.init(SwitchPin4);
  timer.setInterval(2000L, checkBlynkStatus); // check if Blynk server is connected every 2 seconds
  Blynk.config(auth);
  delay(1000);
 
  if (!fetch_blynk_state) {
    Blynk.virtualWrite(VPIN_BUTTON_1, toggleState_1);
    Blynk.virtualWrite(VPIN_BUTTON_2, toggleState_2);
    Blynk.virtualWrite(VPIN_BUTTON_3, toggleState_3);
    Blynk.virtualWrite(VPIN_BUTTON_4, toggleState_4);
  }
  
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
  pinMode(buttonPin, INPUT_PULLUP); 

  attachInterrupt(digitalPinToInterrupt(encoderPinA), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB), updateEncoder, CHANGE);
  pinMode(MCU_SIM_EN_PIN,OUTPUT); 
    digitalWrite(MCU_SIM_EN_PIN,LOW);
    delay(20);
    Serial.begin(115200);
    Serial.println("\n\n\n\n-----------------------\nSystem started!!!!");
    // Delay 8s for power on
    delay(8000);
    simSerial.begin(MCU_SIM_BAUDRATE, SERIAL_8N1, MCU_SIM_RX_PIN, MCU_SIM_TX_PIN);
    // Check AT Command
    sim_at_cmd("AT");
    // Product infor
    sim_at_cmd("ATI");
    // Check SIM Slot
    sim_at_cmd("AT+CPIN?");
    // Check Signal Quality
    sim_at_cmd("AT+CSQ");
    sim_at_cmd("AT+CIMI");
  lcd.init();                      
  lcd.backlight();                

  lcd.setCursor(0, 0);
  lcd.print("System Started...");
  Serial.println("Setup done");

  thermo.begin(MAX31865_3WIRE);

}
 
void loop()
{
  if ( digitalRead(TRIGGER_PIN) == HIGH) {
    WiFiManager wm;    
    wm.setConfigPortalTimeout(timeout);
    if (!wm.startConfigPortal("OnDemandAP")) {
      Serial.println("failed to connect and hit timeout");
      delay(3000);
      ESP.restart();
      delay(5000);
    }
    Serial.println("connected...yeey :)");
    Serial.print("SSID: "); Serial.println(WiFi.SSID());
    Serial.print("Password: "); Serial.println(WiFi.psk());
    Serial.print("IP Address: "); Serial.println(WiFi.localIP());
  }
  Blynk.run();
  timer.run();
  button1.check();
  button2.check();
  button3.check();
  button4.check();

  read_dht22(&humidity);
  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.println(" %");
  Blynk.virtualWrite(VPIN_HUMIDITY, humidity);

  readPT100(&temp);
  Serial.print("Temp: ");
  Serial.print(temp);
  Blynk.virtualWrite(VPIN_TEMPERATURE, temp);

  delay(1000);
    if(temp > 35){
    sent_sms();
    delay(5000);   
    call();
  }

  int adc_reading ;
  adc_reading = analogRead(pin);
      //Convert adc_reading to voltage in mV
  uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
  float valuevoltage = (float)voltage/1000; 
  current_mA = valuevoltage * conversionFactor;

  // Hiển thị kết quả
  Serial.print("ADC Value: ");
  Serial.print(adc_reading);
  Serial.print(" | Voltage: ");
  Serial.print(valuevoltage, 2);
  Serial.print(" V | Current: ");
  Serial.print(current_mA, 2);
  Serial.println(" mA");

  int adc_reading1 ;
  adc_reading1 = analogRead(pinV);
  uint32_t voltage1 = esp_adc_cal_raw_to_voltage(adc_reading1, adc_chars);
  float valuevoltage1 = (float)voltage1/1000; 
  if (valuevoltage1 >= 3.11) {
    inputVoltage = valuevoltage1 * 3.205;
  } else if (valuevoltage1 <= 0.15) {
    inputVoltage = 0;
  } else {
    inputVoltage = valuevoltage1 * 3;
  }
  Serial.print("Điện áp đầu vào: ");
  Serial.print( adc_reading1     );
   Serial.println(" V");
  Serial.print( valuevoltage1      );
   Serial.println(" V");
  Serial.print( inputVoltage     );
  Serial.println(" V");
  delay(1000);  // Đợi 5 giây trước khi đọc lại
  if (Serial.available()){
        char c = Serial.read();
        simSerial.write(c);
    }
    sim_at_wait();

  if (digitalRead(buttonPin) == LOW) {
    tt = !tt;
    
    delay(100);
  }
  if(tt == false){
    displaySelectedWiFiInfo();
  delay(200);
  }
  else{
   displayWiFiMenu();
  }
  delay(300);
}

void button1Handler(AceButton* button, uint8_t eventType, uint8_t buttonState)
{
  Serial.println("EVENT1");
  switch (eventType)
  {
    case AceButton::kEventPressed:
      Serial.println("kEventPressed");
      digitalWrite(RelayPin1, !toggleState_1);
      toggleState_1 = !toggleState_1;
      Blynk.virtualWrite(VPIN_BUTTON_1, toggleState_1);
      break;
    case AceButton::kEventReleased:
      Serial.println("kEventReleased");
      digitalWrite(RelayPin1, !toggleState_1);
      toggleState_1 = !toggleState_1;
      Blynk.virtualWrite(VPIN_BUTTON_1, toggleState_1);
      break;
  }
}
void button2Handler(AceButton* button, uint8_t eventType, uint8_t buttonState)
{
  Serial.println("EVENT2");
  switch (eventType)
  {
    case AceButton::kEventPressed:
      Serial.println("kEventPressed");
      digitalWrite(RelayPin2, !toggleState_2);
      toggleState_2 = !toggleState_2;
      Blynk.virtualWrite(VPIN_BUTTON_2, toggleState_2);
      break;
    case AceButton::kEventReleased:
      Serial.println("kEventReleased");
      digitalWrite(RelayPin2, !toggleState_2);
      toggleState_2 = !toggleState_2;
      Blynk.virtualWrite(VPIN_BUTTON_2, toggleState_2);
      break;
  }
}
void button3Handler(AceButton* button, uint8_t eventType, uint8_t buttonState)
{
  Serial.println("EVENT3");
  switch (eventType)
  {
    case AceButton::kEventPressed:
      Serial.println("kEventPressed");
      digitalWrite(RelayPin3, !toggleState_3);
      toggleState_3 = !toggleState_3;
      Blynk.virtualWrite(VPIN_BUTTON_3, toggleState_3);
      break;
    case AceButton::kEventReleased:
      Serial.println("kEventReleased");
      digitalWrite(RelayPin3, !toggleState_3);
      toggleState_3 = !toggleState_3;
      Blynk.virtualWrite(VPIN_BUTTON_3, toggleState_3);
      break;
  }
}
void button4Handler(AceButton* button, uint8_t eventType, uint8_t buttonState)
{
  Serial.println("EVENT3");
  switch (eventType)
  {
    case AceButton::kEventPressed:
      Serial.println("kEventPressed");
      digitalWrite(RelayPin4, !toggleState_4);
      toggleState_4 = !toggleState_4;
      Blynk.virtualWrite(VPIN_BUTTON_4, toggleState_4);
      break;
    case AceButton::kEventReleased:
      Serial.println("kEventReleased");
      digitalWrite(RelayPin4, !toggleState_4);
      toggleState_4 = !toggleState_4;
      Blynk.virtualWrite(VPIN_BUTTON_4, toggleState_4);
      break;
  }
}
void updateEncoder() {
  
  boolean aSetNew = digitalRead(encoderPinA);
  boolean bSetNew = digitalRead(encoderPinB);

  
  if (aSetLast == LOW && bSetLast == LOW) {
    if (aSetNew == HIGH && bSetNew == LOW) {
      selectedfunction++;
    } else if (aSetNew == LOW && bSetNew == HIGH) {
      selectedfunction--;
    }
  }
  aSetLast = aSetNew;
  bSetLast = bSetNew;
} 
void displayWiFiMenu() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("He Thong Giam Sat");
  if (selectedfunction <= 0) selectedfunction = 0;
  if (selectedfunction >= numberOffunction) selectedfunction = numberOffunction - 1;
  int startIndex = selectedfunction - 1;
  if (startIndex < 0) {
    startIndex = 0;
  }
  int endIndex = min(startIndex + 3, numberOffunction);

  for (int i = startIndex; i < endIndex; ++i) {
    lcd.setCursor(0, i - startIndex + 1);
    if (i == selectedfunction) {
      lcd.print(">");
    } else {
      lcd.print(" ");
    }
    lcd.print(DefautMenu[i]);
  }

}

void displaySelectedWiFiInfo() {
    if (selectedfunction >= 0 && selectedfunction < numberOffunction) {     
    lcd.clear();
if (selectedfunction == 0){
    lcd.setCursor(0, 0);
    lcd.print("NHIET DO PT100: "); 
    lcd.setCursor(0, 1);
    lcd.print(temp); 
    }
if (selectedfunction == 1){
    lcd.setCursor(0, 0);
    lcd.print("DO AM DHT22: "); 
    lcd.setCursor(0, 1);
    lcd.print(humidity); 

    }
    if (selectedfunction == 2){
    lcd.setCursor(0, 0);
    lcd.print("ADC 4-20mA: "); 
    lcd.setCursor(0, 1);
    lcd.println(current_mA);
    }
    if (selectedfunction == 3){
    lcd.setCursor(0, 0);
    lcd.print("ADC 0-10V: "); 
    lcd.setCursor(0, 1);
    lcd.println(inputVoltage);
    }
    if (selectedfunction == 4){
    lcd.setCursor(0, 0);
    lcd.print("WI-FI: "); 
    lcd.setCursor(0, 1);
    lcd.print( WiFi.SSID()); 
    lcd.setCursor(0, 2);
    lcd.print( WiFi.psk());
    lcd.setCursor(0, 3);
    lcd.print( WiFi.localIP());
    }
    }
}