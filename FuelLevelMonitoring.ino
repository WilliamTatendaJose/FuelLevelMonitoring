#include <Wire.h>
#include <math.h>
#include "RTClib.h"    // DS1302 library
#include <avr/sleep.h> // AVR library for controlling the sleep modes
#include <GSM.h>
#include <LiquidCrystal_I2C.h>

#define PINNUMBER ""

#define alarmPin 2

// UltraSonic Sensor setup
#define SOUND_SPEED 0.034 // cm/us
#define trig 5
#define echo 18
long duration;
float distanceCm;

// initialize the library instance
GSM gsmAccess;
GSM_SMS sms;
RTC_DS1302 rtc;

// set the LCD number of columns and rows
int lcdColumns = 16;
int lcdRows = 2;
int lcdAddress = 0x27; // to run get address sketch first

// set LCD address, number of columns and rows
// if you don't know your display address, run an I2C scanner sketch
LiquidCrystal_I2C lcd(lcdAddress, lcdColumns, lcdRows);

const double pi = 3.14159265;
const double length = 3.0;
const double radius = 0.5;
double height;
double volume;
tmElements_t tm;

void setupRtcModule();
void enterSleep();
void triggerAlarm();
double calculateVolume(double i);
void setupGSM();

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);

  // initialize LCD
  lcd.init();
  // turn on LCD backlight
  lcd.backlight();

  while (!Serial)
    ; // wait for Arduino Serial Monitor
  delay(200);

  setupGSM();
  setupRtcModule();
}

void loop()
{
  // put your main code here, to run repeatedly:

  delay(10000); // Wait 10 seconds before going to sleep

  // Get current time and set alarm to a time to wake
  DateTime now = rtc.now(); // Get current time

  rtc.setAlarm1(DateTime(0, 0, 0, 8, 0, 0), DS1302_A1_Hour); // set to 8 am daily

  enterSleep(); // Go to sleep
}

void setupGSM()
{

  // connection state
  boolean notConnected = true;

  // Start GSM shield
  // If your SIM has PIN, pass it as a parameter of begin() in quotes
  while (notConnected)
  {
    if (gsmAccess.begin(PINNUMBER) == GSM_READY)
    {
      notConnected = false;
    }
    else
    {
      Serial.println("Not connected");
      delay(1000);
    }
  }

  Serial.println("GSM initialized");
}

void setupRtcModule()
{

  pinMode(alarmPin, INPUT_PULLUP); // Set alarm pin as pullup

  if (!rtc.begin())
  {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    abort();
  }

  // If required set time
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); // To compiled time
  // rtc.adjust(DateTime(2020, 7, 3, 20, 0, 0)); // Or explicitly, e.g. July 3, 2020 at 8pm

  // Disable and clear both alarms
  rtc.disableAlarm(1);
  rtc.disableAlarm(2);
  rtc.clearAlarm(1);
  rtc.clearAlarm(2);

  rtc.writeSqwPinMode(DS1302_OFF); // Place SQW pin into alarm interrupt mode

  Serial.println("Starting");
}

void enterSleep()
{
  sleep_enable();                      // Enabling sleep mode
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // Setting the sleep mode, in this case full sleep

  noInterrupts(); // Disable interrupts
  attachInterrupt(digitalPinToInterrupt(alarmPin), triggerAlarm_ISR, LOW);

  Serial.println("Going to sleep!"); // Print message to serial monitor
  Serial.flush();                    // Ensure all characters are sent to the serial monitor

  interrupts(); // Allow interrupts again
  sleep_cpu();  // Enter sleep mode

  /* The program will continue from here when it wakes */

  // Disable and clear alarm
  rtc.disableAlarm(1);
  rtc.clearAlarm(1);

  Serial.println("I'm back!"); // Print message to show we're back
}
void triggerALarm_ISR()
{
  leep_disable();                                   // Disable sleep mode
  detachInterrupt(digitalPinToInterrupt(alarmPin)); // Detach the interrupt to stop it firing
  height = getHeight();
  displayVolume(calculateVolume());
  sendMessage(calculateVolume());
}

void sendMessage(double message)
{

  char remoteNum[20]; // telephone number to send sms
  readSerial(remoteNum);
  // sms text
  txtMsg = to_string(message);
  char txtMsg[200] = "fuel level at:" + volume + "litres";
  readSerial(txtMsg);
  // send the message
  sms.beginSMS(remoteNum);
  sms.print(txtMsg);
  sms.endSMS();
}

// https://mathworld.wolfram.com/HorizontalCylindricalSegment.html
double calculateVolume(double height)
{
  double radius_height = radius - height;
  double radian_angle = acos((radius_height) / radius); //*radian;
  double sqr_root = sqrt((2 * radius * height - pow(height, 2)));

  double volume = length * (pow(radius, 2) * radian_angle - radius_height * sqr_root);

  return volume;
}

double getHeight()
{

  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);

  // Calculate the distance
  distanceCm = duration * SOUND_SPEED / 2;
  height = distanceCm / 100; // height in meters
  return height;
}

void displayVolume()
{
  lcd.setcursor(0, 0);
  lcd.print("Liters left");
  lcd.print(volume);
}

/*get lcd address




#include <Wire.h>

void setup() {
  Wire.begin();
  Serial.begin(115200);
  Serial.println("\nI2C Scanner");
}

void loop() {
  byte error, address;
  int nDevices;
  Serial.println("Scanning...");
  nDevices = 0;
  for(address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
      nDevices++;
    }
    else if (error==4) {
      Serial.print("Unknow error at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
    }
  }
  if (nDevices == 0) {
    Serial.println("No I2C devices found\n");
  }
  else {
    Serial.println("done\n");
  }
  delay(5000);
}



*/