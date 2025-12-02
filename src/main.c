#define BLYNK_PRINT Serial // comment this out to disable prints and save space

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <Stepper.h>
#include <DHT.h>
#include "../inc/custom.h"

/*
  Blynk setup.
*/

char ssid[] = WIFI_SSID;
char pass[] = WIFI_PASS; // password to "" for open networks
BlynkTimer timer;

BLYNK_CONNECTED() // called every time the device is connected to the Blynk.Cloud
{
  Blynk.setProperty(V3, "offImageUrl", "https://static-image.nyc3.cdn.digitaloceanspaces.com/general/fte/congratulations.png");
  Blynk.setProperty(V3, "onImageUrl",  "https://static-image.nyc3.cdn.digitaloceanspaces.com/general/fte/congratulations_pressed.png");
  Blynk.setProperty(V3, "url", "https://docs.blynk.io/en/getting-started/what-do-i-need-to-blynk/how-quickstart-device-was-made");
}

void myTimerEvent() // this function sends Arduino's uptime every second to Virtual Pin 2.
{
  Blynk.virtualWrite(V2, millis() / 1000); // don't send more that 10 values per second.
}

/*
  Potentiometer setup. (Voltage simulator)
*/

#define POT_PIN 32
float voltage = 12;

/*
  DHT setup.
*/

#define DHT_PIN 33

DHT dht(DHT_PIN, DHT22);

float humidity = 0;
float temperature = 23;

/*
  Sonar setup.
*/

int reqSpeed = 60; 
int seedRate = 10;

#define TRIGGER_PIN 25
#define ECHO_PIN 26
#define MAX_DIST 200

typedef enum SonarState
{
  NotDetecting,
  IsDetecting
} SonarState;

boolean triggered = false;
int startTime;
int endTime;
int echoTime = 0;
int distance = 0;
int numDetects = 0;
int numFails = 0;
int numSuccess = 0;
int numDoubles = 0;
int numFailsAndDoubles = 0;
int prev_USms = 0;
int prev_msSeed = 0;
int curr_ms = 0;
int duration = 0;
SonarState prevState = NotDetecting;
SonarState currState = NotDetecting;

enum State {
  IDLE,
  TRIGGER_SENT,
  ECHO_HIGH,
  ECHO_LOW
};

State currentState = IDLE;

#define FAIL_MARGIN 1.2f

float getSeedInterval(int rate)
{
  float seed_interval;

  switch(rate)
  {
    case 2:
      seed_interval = 50;
    case 3:
      seed_interval = 42;
    case 4:
      seed_interval = 25;
    case 5:
      seed_interval = 20;
    case 6:
      seed_interval = 17;
    case 7:
      seed_interval = 14;
    case 8:
      seed_interval = 12;
    case 9:
      seed_interval = 11;
    case 10:
      seed_interval = 10;
    case 11:
      seed_interval = 9;
    case 12:
      seed_interval = 8;
  }

  return (seed_interval*1000*FAIL_MARGIN);    
} 

void DoSonarUpdate()
{  
  digitalWrite(TRIGGER_PIN, HIGH);
  digitalWrite(TRIGGER_PIN, LOW);

  duration = endTime - startTime;
  distance = duration * 0.0343 / 2;

  prev_USms = curr_ms;

  if (distance > 200 || distance < 0) // bad measurement
  {
    return;
  }

  if (distance <= 10)
  {
    currState = IsDetecting;
  }
  else
  {
    currState = NotDetecting;
  }

  if(prevState == IsDetecting && currState == NotDetecting) // seed is in front of the sensor
  {
    numDetects++;
  }

  if((curr_ms - prev_msSeed) > getSeedInterval(seedRate))
  {
    if(numDetects == 0)
    {
      numFails++;
      Serial.print("Falhas: ");
      Serial.println(numFails);
    }
    else if(numDetects == 1)
    {
      numSuccess++;
      Serial.print("Sucessos: ");
      Serial.println(numSuccess);
    }
    else if(numDetects >= 2)
    {
      numDoubles++;
      Serial.print("Duplas: ");
      Serial.println(numDoubles);
    }

    numDetects = 0;
    prev_msSeed = curr_ms;
  }

  prevState = currState;
}

void handleEcho()
{
  if (digitalRead(ECHO_PIN) == HIGH) 
  {
    startTime = micros();
  } 
  else 
  {
    endTime = micros();
  }
}

/*
  Stepper setup.
*/

#define STEPPER_PIN_A 27
#define STEPPER_PIN_B 14
#define STEPPER_PIN_C 12
#define STEPPER_PIN_D 13

#define STEPS_PER_REV 200

Stepper stepper(STEPS_PER_REV, STEPPER_PIN_A, STEPPER_PIN_B, STEPPER_PIN_C, STEPPER_PIN_D);

bool isSystemEnabled = true;
int prev_ms = 0;
int stepNumber = 0;

void UpdateReqSpeed()
{
  stepper.setSpeed(reqSpeed);
}

void DoStepperUpdate()
{
  UpdateReqSpeed();
  stepper.step(1);
}

BLYNK_WRITE(V0) 
{
  isSystemEnabled = (boolean)param.asInt(); 
}

BLYNK_WRITE(V3) 
{
  seedRate = (param.asInt());
  reqSpeed = seedRate * 6; // multiply the seed rate by six so we get a good stepper speed
}

int prevWriteMs = 0;
int writeMs = 0;

typedef enum WriteState{
  FailsAndDoubles,
  Temperature,
  Voltage,
}WriteState;

WriteState writeState;

#define LED_PIN 2 // internal LED

void setup()
{
  //blynk
  Serial.begin(9400); // debug console
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  timer.setInterval(1000L, myTimerEvent); // setup a function to be called every second

  //dht
  dht.begin();

  //sonar
	pinMode(TRIGGER_PIN, OUTPUT);
	pinMode(ECHO_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ECHO_PIN), handleEcho, CHANGE);

  pinMode(LED_PIN, OUTPUT);
  writeState = FailsAndDoubles;
}

void loop()
{
  Blynk.run();
  timer.run();
  curr_ms = millis();

  //voltage sim update
  voltage = analogRead(POT_PIN)* 16.0f / 4096.0f;

  //dht update
  humidity = dht.readHumidity();
  temperature = dht.readTemperature();

  if (isSystemEnabled)
  {
    digitalWrite(LED_PIN, HIGH);
    DoSonarUpdate();
    DoStepperUpdate();
  }
  else
  {
    digitalWrite(LED_PIN, LOW);
    numFails = 0;
    numDoubles = 0;
  }

  // write to Blynk and display just each 500ms, don't write all values at once since it may take a while
  if (writeMs - prevWriteMs > 500)
  {
    switch(writeState)
    {
      case FailsAndDoubles:
        numFailsAndDoubles = numFails + numDoubles;
        Blynk.virtualWrite(V1, numFailsAndDoubles);
        writeState = (WriteState)(writeState + 1);
        break;
      case Temperature:
        Blynk.virtualWrite(V4, temperature);
        writeState = (WriteState)(writeState + 1);
        break;
      case Voltage:
        Blynk.virtualWrite(V5, voltage);
        writeState = FailsAndDoubles; // reset
        break;
    }
    
    prevWriteMs = millis();
  }  

  writeMs = millis();
}