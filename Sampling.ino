void start_timer() {
  // Don't need compare outputs
  TCCR1A = 0; 
  // Select /8 prescaler and enable CTC mode
  TCCR1B = 0b010 | (1<<WGM12);
  // Don't force any output compare matches
  TCCR1C = 0;
  // Enable interrupt
  TIMSK1 = (1<<OCIE1B);
  // Set OCR1A -- this register will be used to reset the timer every interval
  OCR1A = 334; // Count up to 334 ticks, which gives 5988Hz with the /8 prescaler
  // Set OCR1B -- this register will be used to trigger the ADC every interval
  OCR1B = 334; // Count up to 334 ticks, which gives 5988Hz with the /8 prescaler
}

void init_adc() {
  // Set the pin's data direction to input (0)
  DDRC &= ~(1<<INPUT_ADC); // DDR is a bitmask while ADMUX is a binary number
  // Sets to the correct ADC and Vcc ref
  ADMUX = INPUT_ADC | (1<<REFS0);  // ADMUX is a binary number while DDR is a bitmask
  // Enable ADC, auto trigger, and interrupts; set prescaler to 128
  ADCSRA = (1<<ADEN) | (1<<ADATE) | (1<<ADIE) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
  // Set ADC auto trigger source to Timer1 compare match B
  ADCSRB = 0b101;
  // Disable digital input on the used ADC pin
  if (INPUT_ADC < 6) DIDR0 |= (1 << (INPUT_ADC+1) );
  // Start ADC
  ADCSRA |= (1<<ADSC); 
}

ISR(ADC_vect) {
  adc_vals[current_sample] = ADCL | (ADCH << 8);
  current_sample++;
  if (current_sample == NUM_SAMPLES) {
    // Disable the ADC until the loop reenables it
    ADCSRA &= ~(1<<ADEN);
    // Set samples finished flags
    //samples_finished_flag = 1;
  }
}

// Every timer ticks almost exactly 5988 times per second. The normal target triggers every
// 299 seconds, 19 times in a row, then the compensate target ticks 307 times. That means a 
// complete cycle takes 299*19+307 = 5988 ticks and is triggered 10 times, for an average
// frequency of 100 Hz.
#define TARGET_NORMAL 299
#define TARGET_COMPENSATE 300
#define COMPENSATION_FREQUENCY_1 1
#define COMPENSATION_FREQUENCY_2 3

volatile int counter = 0;
volatile int compensator = 0;

ISR(TIMER1_COMPB_vect) {
  counter++;
  if (compensator == COMPENSATION_FREQUENCY_1 || compensator == COMPENSATION_FREQUENCY_2 && counter == TARGET_COMPENSATE) {
    flag_100Hz = 1;
    if (compensator == 4) compensator = 0;
    counter = 0;
  } else if (counter == TARGET_NORMAL) {
    flag_100Hz = 1;
    compensator++;
    counter = 0;
  }
}
