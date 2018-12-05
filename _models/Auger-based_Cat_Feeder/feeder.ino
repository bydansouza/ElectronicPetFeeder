#include <LowPower.h>  // https://github.com/rocketscream/Low-Power
#include <Servo.h>

#define PIN_SERVO 9
#define PIN_BUTTON 2
#define PIN_LED 13

Servo feedServo;

unsigned long feedIntervalFullMsec = 28800000; // 8 hours
unsigned long feedIntervalMiniMsec = 3600000; // 1 hour

int feedRate = 105; // 90+15
int feedReversal = 75; // 90-20
int feedTimeFullCount = 10;
int feedTimeMiniCount = 2;

int lastButtonState = HIGH;

boolean triggerFeed = false;
boolean triggerRfid = false;

unsigned long nextFullFeedTime = 0;
unsigned long nextMiniFeedTime = 0;

void wakeUpNow()
{
  // empty. after wake-up handler
}

void setup()
{
  Serial.begin(9600);

  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_BUTTON, INPUT);
}

void loop()
{
  unsigned long timeNow = millis();
  int feedAmount = 0;

  // if we triggered micro-feeding
  // TODO
  if (triggerRfid && (long)(timeNow - nextMiniFeedTime) >= 0)
  {
    triggerRfid = false;
    triggerFeed = true;
    feedAmount = feedTimeMiniCount;
    nextMiniFeedTime = timeNow + feedIntervalMiniMsec;
  }

  // if we triggered by button
  // (makershield is wired to go LOW)
  int buttonState = digitalRead(PIN_BUTTON);
  if (buttonState == LOW && lastButtonState != buttonState)
  {
    triggerFeed = true;
    feedAmount = 2 * feedTimeMiniCount;
  }
  lastButtonState = buttonState;

  // if we met the feed time...
  if ((long)(timeNow - nextFullFeedTime) >= 0)
  {
    triggerFeed = true;
    feedAmount = feedTimeFullCount;
    nextFullFeedTime = timeNow + feedIntervalFullMsec;
  }

  // trigger a feeding (turn servo, light led)
  // since nothing else should run during feeding, delay() is fine
  if (triggerFeed)
  {
    digitalWrite(PIN_LED, HIGH);
    feedServo.attach(PIN_SERVO);
    for (int cnt = 0; cnt < feedAmount; cnt++)
    {
      feedServo.write(feedRate);
      delay(1000);
      feedServo.write(feedReversal);
      delay(500);
      feedServo.write(feedRate);
      delay(500);

    }
    digitalWrite(PIN_LED, LOW);
    feedServo.detach();
    triggerFeed = false;
  }

  // done doing stuff, lets sleep for a bit
  attachInterrupt(0, wakeUpNow, LOW); // pin-2 (button) went low
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
  detachInterrupt(0);
}

