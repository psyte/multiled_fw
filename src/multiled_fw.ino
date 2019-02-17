#include <Adafruit_TLC5947.h>
#include <JC_Button.h>
#include <Chrono.h>

#define PIN_DATA 0
#define PIN_CLK 2
#define PIN_LATCH 1
#define PIN_BUTTON 4
#define PIN_POWER 3

#define NUM_MODES 6
#define MODE_ON 0
#define MODE_BREATHE 1
#define MODE_RING1 2
#define MODE_RING2 3
#define MODE_SYMMETRIC 4
#define MODE_OFF 5

#define NUM_LEDS 24

// LEDs
Adafruit_TLC5947 leds = Adafruit_TLC5947(1, PIN_CLK, PIN_DATA, PIN_LATCH);
Chrono ledChrono;
Chrono shutdownChrono;
Chrono watchdogChrono(Chrono::SECONDS);

// Button
Button button(PIN_BUTTON, 25, true, true);

volatile uint8_t mode = MODE_BREATHE;
uint8_t force = 0;


void leds_update(void)
{
  switch(mode)
  {
  case MODE_ON:
    leds_fill(1500, 500);
    break;
  case MODE_BREATHE:
    leds_breathe(40);
    break;
  case MODE_RING1:
    leds_ring(1, 20);
    break;
  case MODE_RING2:
    leds_ring(3, 20);
    break;
  case MODE_SYMMETRIC:
    leds_symmetric(20);
    break;
  case MODE_OFF:
    leds_fill(0, 500);
    break;
  }
}

void leds_breathe(uint16_t wait)
{
  static uint8_t step = 0;
  float phase;
  float val;
  uint16_t pwm;

  if(ledChrono.hasPassed(wait, true) || force)
  {
    phase = (step/256.0f) * (PI*2.0f);
    val = cos(phase)+1;       // 0 offset cos curve
    val *= val;               // square it
    pwm = val * 980 + 80;    // scale it from 0..4 to 0..4000
    
    leds_fill(pwm, 0);        // fill and display right away
  
    step+=2;
  }
}

uint8_t leds_bootring(uint16_t pwm, uint16_t wait)
{
  static uint8_t step = 0;
    
  if(ledChrono.hasPassed(wait, true) || force)
  {
    for(int i=0; i<NUM_LEDS; i++)
    {
      if((step%NUM_LEDS) == i)
        leds.setPWM(i, pwm);
      else
        leds.setPWM(i, 0);      
    }
    leds.write();
    step++;
  }
  return step;
}

void leds_ring(uint8_t peaks, uint16_t wait)
{
  static uint8_t step = 0;
  float phase;
  float delta;
  float val;
  uint16_t pwm;
    
  if(ledChrono.hasPassed(wait, true) || force)
  {
    for(int i=0; i<NUM_LEDS; i++)
    {
      phase = (step/256.0f) * (PI*2.0f); 
      delta = i * (float)((PI*2.0f) / (NUM_LEDS/peaks));
      phase += delta;       // i * (float)((PI*2.0f) / NUM_LEDS)
      val = cos(phase)+1;   // 0 offset cos curve
      val *= val;           // square it
      pwm = val * 1000;     // scale it from 0..4 to 0..4000
      
      leds.setPWM(i, pwm); // fill and display right away
    }
    leds.write();
    step+=2;
  }
}

void leds_symmetric(uint16_t wait)
{
  static uint8_t step = 0;
  float phase;
  float delta;
  float val;
  uint16_t pwm;
    
  if(ledChrono.hasPassed(wait, true) || force)
  {
    for(int i=0; i<13; i++)
    {
      phase = (step/256.0f) * (PI*2.0f);
      delta = (i * (PI*2.0f)) / NUM_LEDS;
      phase += delta;
      val = cos(phase)+1;   // 0 offset cos curve
      val *= val;           // square it
      pwm = val * 1000;     // scale it from 0..4 to 0..4000
      
      leds.setPWM(i, pwm); // set led

      if((i != 0) && (i != 12))
      {
        leds.setPWM(24-i, pwm); // set opposite led 
      }
    }
    leds.write();
    step-=2;
  }
}

void leds_fill(uint16_t pwm, uint16_t wait)
{
  if(!wait || ledChrono.hasPassed(wait, true) || force)
  {
    for(int i=0; i<NUM_LEDS; i++)
    {
      leds.setPWM(i, pwm);
    }
    leds.write();
  }
}
  
void setup() 
{
  delay(500);

  // Keep power on
  pinMode(PIN_POWER, OUTPUT);
  digitalWrite(PIN_POWER, HIGH);

  // Button
  button.begin();

  // LEDs
  leds.begin();

  // Chronos
  ledChrono.start();
  shutdownChrono.start();
  watchdogChrono.start();

  // Bootring
  while(leds_bootring(2000, 5) < 48) {}
}

void loop()
{
  force = 0;

  // auto shutdown after 5 minutes
  if(watchdogChrono.hasPassed(5*60))
  {
    force = 1;
    mode = MODE_OFF;
  }

  button.read(); // Update the button instance
  if(button.wasPressed())
  {
    watchdogChrono.restart();
    force = 1;
    if(++mode >= NUM_MODES)
      mode = MODE_BREATHE;
    if(mode == MODE_OFF)
      shutdownChrono.restart();
  }

  leds_update();

  // turn off?
  if(mode == MODE_OFF)
  {
     // wait a few secs before shutting down entirely
    if(shutdownChrono.hasPassed(5000))
    {
      leds.setPWM(0, 4000);
      leds.write();
      digitalWrite(PIN_POWER, LOW);  // power off
      while(1) {}
    }
  }
}
