#include <avr/io.h>
#include <avr/interrupt.h>

#define LEDS 12 // LED count
#define CPLX_MASK 0b00011101 // LED IO mask on PORTB

#define MPLX 0b10000000 // Don't skip LED in Multiplexing if off
#define MINC 0b01000000 // Increase LED brightness (for animations)
#define MDEC 0b00100000 // Decrease LED brightness (for animations)
#define MPOW 0b00000111 // Mask for LED power values
#define POWMAX 0b00000111 // Max LED power

const uint8_t isrPWRvals[3] = { _BV(1), _BV(3), _BV(4) }; // PWM Power/Duration values (LSB to MSB)
const uint8_t CPLX_PBH[LEDS] = {
  0b00000001,
  0b00000100,
  0b00000100,
  0b00001000,
  0b00001000,
  0b00010000,
  0b00000001,
  0b00001000,
  0b00000100,
  0b00010000,
  0b00000001,
  0b00010000,
}; // Charlieplexing IOs in pairs

uint8_t LED[LEDS]; // array containing each LEDs brightness (0-7)
uint16_t globalCounter = 0;
uint8_t mode,timescale,c_led = 0;
volatile bool isrTick = true;
volatile bool btnPressed = false;

void cplx_led_on(uint8_t var);
void cplx_off();
void clear_all();
void ani_single_led(uint8_t off_state, uint8_t flash_threshold = 0xFF);
void ani_pulse(uint8_t off_state);
void tick();

// See https://blog.podkalicki.com/attiny13-pseudo-random-numbers/
uint16_t random_number = 0x123;
uint16_t lfsr16_next(uint16_t n) {
    return (n >> 0x01U) ^ (-(n & 0x01U) & 0xB400U);
}
uint16_t random(void) {
    return (random_number = lfsr16_next(random_number)); 
}

int main() {
  // Setup Mode-Button on PB1 (Push-Button to GND)
  DDRB &= ~_BV(PB1); // Input
  PORTB |= _BV(PB1); // Enable Pull-Up 
  // Setup Timer0 / Main PWM and Counter
  TCCR0A |= _BV(WGM01); // Timer0 to CTC mode (count to OCR0A)
  TCCR0B |= _BV(CS02); // Prescaler /256
  TIMSK0 |= _BV(OCIE0A); // enable Timer0 CTC interrupt
  TIFR0 |= _BV(OCF0A); // enable Interrupt on TOP
  sei(); // enable interrupts

  LED[1] = 7;

  while(1) {
    if(isrTick) {
      isrTick = false;
      globalCounter++;
      tick();
    }
  }
}

void tick() {
  static uint16_t btnPressCnt = 0;
  if(btnPressed) { 
    // Button pressed
    btnPressCnt++;
    if((btnPressCnt & 0x1F) < 7) {
      LED[1] = 7;
    } else {
      LED[1] = 1;
    }
    // long press adjusts timescale
    if((btnPressCnt & 0x1F) == 0) timescale++;
  } else {
    if(btnPressCnt > 0) {
      if(btnPressCnt < 0x1F) {
        mode++;
        timescale = 0;
      }
      btnPressCnt = 0;      
      LED[1] = 0;
      globalCounter = 0;
    }
    // modes
    switch (mode) {
      case 0:
      case 1:
        ani_single_led(mode);
        break;     
      case 2:
      case 3:
        ani_single_led(mode-2, 4);
        break;
      case 4:
      case 5:
        ani_pulse(mode-4);
        break;
      default:
        mode = 0;
        break;
    }
  }
}

void ani_single_led(uint8_t off_state, uint8_t flash_threshold = 0xFF) {
  static uint8_t ledIdx = 0;
  if(globalCounter == flash_threshold) {
    LED[ledIdx] = off_state;
  }
  if(globalCounter > 8*(timescale+1)) {
    globalCounter = 1;
    LED[ledIdx] = off_state;
    ledIdx += 1;
    if(ledIdx == LEDS) ledIdx = 0;
    LED[ledIdx] = 7;
  }
}

void ani_pulse(uint8_t off_state) {
  uint8_t val;
  if(globalCounter==0) {
    for(uint8_t i=0; i<LEDS; i++) LED[i] = (random() & MPOW) | MPLX | MDEC; // init all LEDs
  }
  if(globalCounter < timescale+1) return;
  for(uint8_t i=0; i<LEDS; i++) {
    val = LED[i];
    if((val & MINC) && (val & MPOW) < POWMAX) val++; // fade in
    if((val & MDEC) && (val & MPOW) > off_state) val--; // fade out
    if((val & MPOW) == POWMAX) val = ( val & MPOW ) | MDEC | MPLX; // LED is on, set it to fade out
    if((val & MPOW) <= off_state && (random() & 0x1F) == 0 ) val = ( val & MPOW ) | MINC | MPLX; // LED is off, set it to fade in (in case of some randomness)
    LED[i] = val;
  }
  globalCounter = 1;
}



void clear_all() {
  for(uint8_t i = 0; i < LEDS; i++) LED[i] = 0;
}


void cplx_off() {
  PORTB &= ~CPLX_MASK; // GND all (clears charge on pin)
  DDRB &= ~CPLX_MASK; // input all
}

void cplx_led_on (uint8_t var) {
  DDRB |= CPLX_PBH[var | 1];
  DDRB |= CPLX_PBH[var & ~1];
  PORTB |= CPLX_PBH[var];
}

ISR(TIM0_COMPA_vect) {
  static uint8_t isrLEDidx = 0; // the LED lit during this cycle
  static uint8_t isrPWRidx = 0; // the duration/power index the LED is lit ()
  static uint8_t isrCnt = 0; // Count interrupts before isrTick
  uint8_t skipCnt = 0;

  OCR0A = isrPWRvals[isrPWRidx]; // set how long LED will stay on
  cplx_off();
  if( LED[isrLEDidx] & ( 1 << isrPWRidx ) ) cplx_led_on(isrLEDidx);

  // Inc global counter every LEDS*3 cycles which should always be same duration
  if( isrCnt++ == LEDS*6 - 1 ) {
    btnPressed = !(PINB & _BV(PINB1));
    isrCnt = 0;
    isrTick = true;
  }

  // Calc idx and timer duration for next interrupt
  do {
    isrLEDidx += 1;
    if(isrLEDidx == LEDS) isrLEDidx = 0;
    if(isrLEDidx == 0) isrPWRidx = ( isrPWRidx + 1 ) & 3;
  } while ( (LED[isrLEDidx] & (MPLX | MPOW)) == 0 && skipCnt++ < LEDS );
}