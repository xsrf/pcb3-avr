#include <avr/io.h>
#include <avr/interrupt.h>

#define LEDS 12

volatile uint8_t LED[LEDS];
volatile long animationCount = 0;
volatile long lastAnimationCount = 0;
const uint8_t isrPWRvals[3] = { _BV(1), _BV(3), _BV(4) };

void led_on (int var);
void leds_off ();

int main() {
  PORTB |= _BV(PB1); // input
  DDRB &= ~_BV(PB1);

  TCCR0A |= _BV(WGM01); // set timer counter mode to CTC
  //TCCR0B |= _BV(CS01)|_BV(CS00); // set prescaler /64
  TCCR0B |= _BV(CS02); // set prescaler /256
  OCR0A = 100; // set Timer's counter top value
  TIMSK0 |= _BV(OCIE0A); // enable Timer CTC interrupt
  TIFR0 |= _BV(OCF0A); // enable Interrupt on top
  sei(); // enable global interrupts

  LED[0] = 7;
  uint8_t ledc = 0;

  while(1) {
    if(lastAnimationCount + 500 < animationCount || lastAnimationCount > animationCount) {
      lastAnimationCount = animationCount;
      LED[ledc] = 0;
      LED[(ledc+7)%LEDS] = 0;
      ledc = ( ledc + 1 ) % LEDS;
      LED[ledc] = 7;
      LED[(ledc+7)%LEDS] = 7;
    }
  }


  while(1) {
    if(lastAnimationCount + 50 < animationCount || lastAnimationCount > animationCount) {
      lastAnimationCount = animationCount;
      for(uint8_t led=4; led<5; led++) {
        if(LED[led] <= 7) LED[led]++;
        if(LED[led] > 8) LED[led]--;
        if(LED[led] == 8) LED[led] = 0xF7;
        if(LED[led] == 0xF0) LED[led] = 0;
      }
    }
  }
  for(uint8_t led=0; led<LEDS; led++) LED[led] = led;
  while(1) {
    if(lastAnimationCount + 50 < animationCount || lastAnimationCount > animationCount) {
      lastAnimationCount = animationCount;
      for(uint8_t led=0; led<LEDS; led++) {
        if(LED[led] <= 7) LED[led]++;
        if(LED[led] > 8) LED[led]--;
        if(LED[led] == 8) LED[led] = 0xF7;
        if(LED[led] == 0xF0) LED[led] = 0;
      }
    }
  }
}





void leds_off() {
  PORTB &= ~(
            _BV(PB0) |
            _BV(PB2) |
            _BV(PB3) |
            _BV(PB4)
          );
  DDRB &= ~(
            _BV(PB0) |
            _BV(PB2) |
            _BV(PB3) |
            _BV(PB4)
          );
}

void led_on (int var) {
  switch (var) {
    case 0: // 1
      DDRB |= _BV(PB0) | _BV(PB2); // output
      PORTB |= _BV(PB0); // HIGH
      PORTB &= ~_BV(PB2); // LOW
      break;
    case 1: // 2
      DDRB |= _BV(PB0) | _BV(PB2); // output
      PORTB |= _BV(PB2); // HIGH
      PORTB &= ~_BV(PB0); // LOW
      break;
    case 2: // 3
      DDRB |= _BV(PB2) | _BV(PB3); // output
      PORTB |= _BV(PB2); // HIGH
      PORTB &= ~_BV(PB3); // LOW
      break;
    case 3: // 4
      DDRB |= _BV(PB2) | _BV(PB3); // output
      PORTB |= _BV(PB3); // HIGH
      PORTB &= ~_BV(PB2); // LOW
      break;
    case 4: // 5
      DDRB |= _BV(PB3) | _BV(PB4); // output
      PORTB |= _BV(PB3); // HIGH
      PORTB &= ~_BV(PB4); // LOW
      break;
    case 5: // 6
      DDRB |= _BV(PB3) | _BV(PB4); // output
      PORTB |= _BV(PB4); // HIGH
      PORTB &= ~_BV(PB3); // LOW
      break;
    case 6: // 7
      DDRB |= _BV(PB0) | _BV(PB3); // output
      PORTB |= _BV(PB0); // HIGH
      PORTB &= ~_BV(PB3); // LOW
      break;
    case 7: // 8
      DDRB |= _BV(PB0) | _BV(PB3); // output
      PORTB |= _BV(PB3); // HIGH
      PORTB &= ~_BV(PB0); // LOW
      break;
    case 8: // 9
      DDRB |= _BV(PB2) | _BV(PB4); // output
      PORTB |= _BV(PB2); // HIGH
      PORTB &= ~_BV(PB4); // LOW
      break;
    case 9: // 10
      DDRB |= _BV(PB2) | _BV(PB4); // output
      PORTB |= _BV(PB4); // HIGH
      PORTB &= ~_BV(PB2); // LOW
      break;
    case 10: // 11
      DDRB |= _BV(PB0) | _BV(PB4); // output
      PORTB |= _BV(PB0); // HIGH
      PORTB &= ~_BV(PB4); // LOW
      break;
    case 11: // 12
      DDRB |= _BV(PB0) | _BV(PB4); // output
      PORTB |= _BV(PB4); // HIGH
      PORTB &= ~_BV(PB0); // LOW
      break;
    default:
      ;
  }
}

ISR(TIM0_COMPA_vect) {
  static uint8_t isrLEDidx = 0; // the LED lit during this cycle
  static uint8_t isrPWRidx = 0; // the duration/power index the LED is lit ()
  uint8_t skipCnt = 0;

  OCR0A = isrPWRvals[isrPWRidx]; // set how long LED will stay on
  leds_off();
  if( LED[isrLEDidx] & ( 1 << isrPWRidx ) ) led_on(isrLEDidx);

  // Calc idx and timer duration for next interrupt
  isrLEDidx = ( isrLEDidx + 1 ) % LEDS;
  if(isrLEDidx == 0) isrPWRidx = ( isrPWRidx + 1 ) % 3;
  if(isrPWRidx == 0 && isrPWRidx == 0) animationCount++;
  // Skip to next lit LED
  while( LED[isrLEDidx] == 0 && skipCnt++ < LEDS) {
    isrLEDidx = ( isrLEDidx + 1 ) % LEDS;
    if(isrLEDidx == 0) isrPWRidx = ( isrPWRidx + 1 ) % 3;
  }
}