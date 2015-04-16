#include <avr/sleep.h>
#include <avr/power.h>

#include <Wire.h>

#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>

#include "Motor.h"

// These must come before the FHT include
#define LOG_OUT 1 // use the log output function
#define FHT_N 128 // set to 256 point fht

// How different two pulses' frequencies need to be in order to be considered different
#define PULSE_FREQUENCY_THRESHOLD 5 // units are FHT buckets

#define FREQUENCY_TOLERANCE 2 // Number of buckets in each direction
#define FREQUENCY_MAGNITUDE_THRESHOLD 30
#define FIRST_SIGNIFICANT_SAMPLE (FHT_N/8)
#define HIGH_LEVEL 4 // 0.2 second
#define LEVEL_TOLERANCE (HIGH_LEVEL/2) 
#define LEVEL_CUTOFF_H (HIGH_LEVEL/2) // Ignore level_tolerance
#define LEVEL_CUTOFF_L (0 + (LEVEL_TOLERANCE/2)) 

#define INPUT_ADC 0 // MUST be 0-8
#define NUM_SAMPLES FHT_N

boolean flag_100Hz = 0;

#include <FHT.h>

volatile unsigned long ms;

volatile int* adc_vals = fht_input;
volatile int current_sample = 0;
volatile unsigned long prev_t = 0;

int prev_sample_frequency = 0;
int current_level = 0;
boolean pulse_active = false; // Boolean
boolean sample_skipped = false;
int pulse_length = 0;
int pulse_drift = 0;

typedef struct {
  unsigned long time_complete;
  int length;
  int frequency;
} complete_pulse;

static complete_pulse _no_pulse = {0,0,0};

complete_pulse current_pulse = _no_pulse;
complete_pulse prev_pulse = _no_pulse;

enum {
  NO_PATTERN,
  STEADY_PATTERN,
  RISING_PATTERN,
  FALLING_PATTERN
} pattern_detected = NO_PATTERN;

Motor leftMotor = Motor(8, 11, 9, 150, 25);
Motor rightMotor = Motor(12, 13, 10, 150, 25);

MPU6050 mpu;

void setup_sleep() {
  set_sleep_mode(SLEEP_MODE_IDLE);
  sleep_enable();
  // Disable unneeded peripherals
  power_spi_disable();
  power_timer2_disable();
  power_twi_disable();
}

void setup() {
  start_timer();
  init_adc();
//  setup_sleep();
  mpu.initialize();
  sei();
  pinMode(A1, OUTPUT);
//  pinMode(6, OUTPUT);
//  pinMode(7, OUTPUT);
  leftMotor.init();
  rightMotor.init();
  Serial.begin(9600);
}

void loop() {
  sleep_mode();
    
  if (current_sample == NUM_SAMPLES) {
    current_sample = 0;

    // Perform FHT
    fht_window(); // window the data for better frequency response
    fht_reorder(); // reorder the data before doing the fht
    fht_run(); // process the data in the fht
    fht_mag_log(); // take the output of the fht
    
    if (false) {
       Serial.write(255); // send a start byte
       Serial.write(fht_log_out, FHT_N/2); // send out the data
    }
    
    // Ignore the first few samples -- they have garbage data from the fht transform
    int sample_frequency = FIRST_SIGNIFICANT_SAMPLE;
    for (int i = FIRST_SIGNIFICANT_SAMPLE+1; i < NUM_SAMPLES/2; i++) {
      if (fht_log_out[i] > fht_log_out[sample_frequency]) {
        sample_frequency = i;
      }
    }
    
    // Only consider this if it's high enough and close enough to the current frequency,
    // OR if it's high enough and it's the first pulse
    if (fht_log_out[sample_frequency] > FREQUENCY_MAGNITUDE_THRESHOLD && (pulse_length == 0 ||
        abs(sample_frequency - prev_sample_frequency) < FREQUENCY_TOLERANCE))
      {
      
      sample_skipped = false;
      
      if (current_level < HIGH_LEVEL) {
        current_level++;
      }
    } else {
      // Some code needs to know if this is a mismatch that's below the threshold
      sample_skipped = true;
      if (current_level > 0) {
        current_level--;
      }
    }
    
//    Serial.print(current_level);
//    Serial.print("  ");
//    
//    Serial.print(sample_skipped);
//    Serial.print("  ");
//    
//    Serial.print(pulse_drift);
//    Serial.print("  ");
//    
//    Serial.println();
    
    // Set pulse_active here only
    if (current_level > LEVEL_CUTOFF_H && !pulse_active) {
      pulse_active = true;
    } else if (current_level < LEVEL_CUTOFF_L && pulse_active) {
      pulse_active = false;
    }
    
    if (pulse_active) {
      digitalWrite(A1, HIGH);
      // Don't record drift or update frequency when sample_skipped is true, because
      // the data will be invalid
      if (!sample_skipped) {
        // Don't record drift on the first sample in a pulse, it throws off the data
        // because prev_sample_frequency isn't set yet
        if (pulse_length > 0) {
          // Record the drift -- this value gets more negative during a swoop down,
          // more positive during a swoop up, neutral during a constant pulse
          pulse_drift += sample_frequency - prev_sample_frequency;
        }
          
        // Update the current frequency to so it detects a pulse even when the pulse drifts
        prev_sample_frequency = sample_frequency;
      }
      
      pulse_length++;
    } else {
      digitalWrite(A1, LOW);
      // If a pulse just ended, print stats and reset them
      if (pulse_length > 0) {
        current_pulse.time_complete = millis();
        current_pulse.length = pulse_length;
        // Store prev_sample_frequency instead of sample_frequency because sample_frequency
        // is updated even on glitches, but prev_sample_frequency isn't 
        current_pulse.frequency = prev_sample_frequency;
        
//        Serial.print("Pulse: ");
//        Serial.print(current_pulse.time_complete);
//        Serial.print(", ");
//        Serial.print(current_pulse.length);
//        Serial.print(", ");
//        Serial.print(current_pulse.frequency);
//        Serial.println();
        
        
        // Reset
        pulse_length = 0;
        
        // Try to detect a pattern //TODO: Put 2000 in a #define
        // Note: _no_pulse will always fail this test.
        if (millis() - prev_pulse.time_complete < 2000) {
          if (current_pulse.frequency - prev_pulse.frequency > PULSE_FREQUENCY_THRESHOLD) {
            pattern_detected = RISING_PATTERN;
          } else if (current_pulse.frequency - prev_pulse.frequency < -PULSE_FREQUENCY_THRESHOLD) {
            pattern_detected = FALLING_PATTERN;
          } else {
            pattern_detected = STEADY_PATTERN;
          }
          // Overwrite the previous pulse so we don't double-count any pulses
          prev_pulse = _no_pulse;
        } else {
          // This pulse is the new prev_pulse
          prev_pulse = current_pulse;
        }
      }
      pulse_drift = 0; // Has to be updated outside the if
    }
  }
  
  if (pattern_detected != NO_PATTERN) {
    if (pattern_detected == RISING_PATTERN) {
      Serial.println("RISING_PATTERN");
//      digitalWrite(6, HIGH);
//      digitalWrite(7, LOW);
    } else if (pattern_detected == FALLING_PATTERN) {
//      Serial.println("FALLING_PATTERN");
//      digitalWrite(6, LOW);
//      digitalWrite(7, HIGH);
    } else if (pattern_detected == STEADY_PATTERN) {
      Serial.println("STEADY_PATTERN");
//      digitalWrite(6, HIGH);
//      digitalWrite(7, HIGH);
    }
    pattern_detected = NO_PATTERN;
  }
  
  if (flag_100Hz) {
    flag_100Hz = 0;
 
    // Reenable ADC
    ADCSRA |= (1<<ADEN);
    
    // Set motors
    int accel = -mpu.getAccelerationY() / 20;
//    Serial.println(accel);
    
    leftMotor.setMotor(accel);
    rightMotor.setMotor(accel);
  }
}
