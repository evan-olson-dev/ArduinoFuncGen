// ---------------------------------------------------------
// 8-BIT ARDUINO FUNCTION GENERATOR
// ---------------------------------------------------------
// This sketch turns an Arduino into a simple waveform generator
// outputting Sine, Triangle, and Sawtooth waves via an 8-bit R-2R DAC.
//
// DAC PINS: D2(LSB) through D9(MSB)
// BUTTON:   D10 (Connect to GND, enables internal pullup)
// POT:      A0 (Connect between 5V and GND for frequency control)
// OLED:     A4 (SDA), A5 (SCL)
// ---------------------------------------------------------

#include <Wire.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"

// OLED Address (usually 0x3C)
#define I2C_ADDRESS 0x3C
SSD1306AsciiWire oled;

// --- CONSTANTS & GLOBALS ---

// 256-point Sine Table used for efficient waveform generation
// Scaled 0-255 to match the 8-bit DAC output range.
const int TABLE_SIZE = 256; 
byte sineTable[TABLE_SIZE]; 

// Waveform Modes:
// 0 = Sawtooth
// 1 = Triangle
// 2 = Sine
int mode = 0; 
const int MAX_MODE = 2;

// Input Pins
const int BUTTON_PIN = 10;
const int POT_PIN = A0;

// Button State Tracking
int lastButtonState = HIGH; 

// Waveform Generation State
int stepIndex = 0;      // Current position in the lookup table or counter
int direction = 1;      // Direction for Triangle wave bouncing (+1 or -1)

// Optimized Timing & Control
unsigned int loopCounter = 0; // To periodically read sensor inputs
long delayTime = 0;           // Current delay in microseconds (using long for frequency math)

// USER CONFIGURABLE FREQUENCY RANGE
// Adjust these to set your desired frequency bounds
int minFreqDelay = 1000;   // Delay for 0% (in microseconds)
int maxFreqDelay = 10;     // Delay for 100% (Safety Floor for responsive control)

// OLED Refresh Timing
int displayStep = 0;               // To interleave text updates
int lastMode = -1;                 // To trigger full screen clear on mode change

// --- SETUP ---
void setup() {
  // 1. Configure DAC Pins (D2-D9) as OUTPUT
  for (int i = 2; i <= 9; i++) {
    pinMode(i, OUTPUT);
  }
  
  // 2. Configure Input Button
  pinMode(BUTTON_PIN, INPUT_PULLUP); 
  
  // 3. Pre-calculate Sine Table
  // Generates a lookup table to avoid expensive `sin()` calls in the loop.
  // The value is offset by 127.5 and scaled to fit the 0-255 byte range.
  for (int i = 0; i < TABLE_SIZE; i++) {
    float angle = (float)i / TABLE_SIZE * 2.0 * PI;
    sineTable[i] = (byte)(127.5 + 127.5 * sin(angle));
  }

  // 4. Configure Non-Blocking ADC (Analog Read)
  ADMUX = (1 << REFS0) | (0 << MUX3) | (0 << MUX2) | (0 << MUX1) | (0 << MUX0); 
  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
  ADCSRA |= (1 << ADSC);

  // 5. Configure Timer1 for Interrupt-Driven Generation (Pro Mode)
  cli(); // Disable interrupts during setup
  TCCR1A = 0; // Normal mode
  TCCR1B = 0; // Clear register
  TCNT1  = 0; // Reset counter
  
  // Set Compare Match Register (OCR1A)
  // Formula: (Clock / (Prescaler * TargetFreq)) - 1
  // We'll update this dynamically in the loop for frequency control.
  OCR1A = 16000; // Start with 1ms delay (16MHz / 1 / 1000 - 1)
  
  // Turn on CTC (Clear Timer on Compare Match) mode
  TCCR1B |= (1 << WGM12);
  // Set CS10 bit for 1 prescaler (maximum precision)
  TCCR1B |= (1 << CS10);  
  // Enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei(); // Enable interrupts

  // 6. Initialize OLED
  Wire.begin();
  Wire.setClock(100000L); // Standard 100kHz clock for stability
  oled.begin(&Adafruit128x64, I2C_ADDRESS);
  oled.displayRemap(true); // Rotate display 180 degrees
  oled.setFont(System5x7);
  oled.clear();
  oled.println("  FUNC GEN v2.0");
  oled.println("----------------");
}

// --- MAIN LOOP ---
void loop() {
  // --- 1. READ CONTROLS (Non-Blocking) ---
  
  // Check if the ADC conversion is finished (ADSC bit goes to 0)
  if (!(ADCSRA & (1 << ADSC))) {
    int potValue = ADC; // Read the full 10-bit result (0-1023)
    
    // Map full 10-bit resolution directly to the delay range.
    // This removes the "steps" and "dead zones" caused by 0-100 mapping.
    delayTime = map(potValue, 0, 1023, minFreqDelay, maxFreqDelay);
    
    // Start the NEXT conversion immediately
    ADCSRA |= (1 << ADSC);
  }

  // Periodically check the button (much faster than analog read)
  if ((loopCounter & 255) == 0) {
    checkSwitch();
  }
  loopCounter++;

  // --- 2. UPDATE SYSTEM TIMING ---
  // Update the hardware timer to match the new potentiometer reading
  // OCR1A = 16 ticks per microsecond (at 16MHz clock, prescaler 1)
  // Safety check: ensure OCR1A is at least 160 ticks (10us * 16)
  unsigned int ticks = (unsigned int)(delayTime * 16);
  if (ticks < 160) ticks = 160; 
  OCR1A = ticks;

  // --- 3. UPDATE DISPLAY (Interleaved) ---
  if ((loopCounter & 1023) == 0) {
    updateDisplayStep();
  }
}

// --- HARDWARE INTERRUPT (PRO MODE) ---
/**
 * Timer1 Interrupt Service Routine (ISR)
 * Fired by hardware based on the OCR1A value.
 * This handles the waveform generation independently of the main loop.
 */
ISR(TIMER1_COMPA_vect) {
  if (mode == 0) { // SAWTOOTH
    outputDAC(stepIndex);
    stepIndex++;
    if (stepIndex >= TABLE_SIZE) stepIndex = 0; 
  }
  
  else if (mode == 1) { // TRIANGLE
    stepIndex += direction;
    if (stepIndex >= TABLE_SIZE - 1) { 
      stepIndex = TABLE_SIZE - 1; 
      direction = -1; 
    }
    if (stepIndex <= 0) { 
      stepIndex = 0; 
      direction = 1; 
    }
    outputDAC(stepIndex);
  }
  
  else if (mode == 2) { // SINE
    outputDAC(sineTable[stepIndex]);
    stepIndex++;
    if (stepIndex >= TABLE_SIZE) stepIndex = 0;
  }
}

// --- HELPER FUNCTIONS ---

/**
 * Writes an 8-bit value to the DAC pins.
 * Optimized with Direct Port Manipulation for speed.
 * 
 * Pin mapping on ATmega328P (Arduino Uno/Nano):
 * Port D (D0-D7): Bits 0-7 but D0, D1 are RX/TX
 * Pins used: D2-D9
 * DAC Bit 0-5 (Value bits 0-5) -> Pins D2-D7 (Port D bits 2-7)
 * DAC Bit 6-7 (Value bits 6-7) -> Pins D8-D9 (Port B bits 0-1)
 */
void outputDAC(int value) {
  // Clear Port D bits 2-7 and set them from value bits 0-5
  // (value << 2) aligns value bit 0 to Port D bit 2
  PORTD = (PORTD & 0x03) | (value << 2);
  
  // Clear Port B bits 0-1 and set them from value bits 6-7
  // (value >> 6) aligns value bit 6 to Port B bit 0
  PORTB = (PORTB & 0xFC) | (value >> 6);
}

/**
 * Checks the mode button state and switches modes on a falling edge (press).
 * Includes simple logic to debounce and cycle through modes.
 */
void checkSwitch() {
  int currentState = digitalRead(BUTTON_PIN);
  
  // Detect falling edge (HIGH -> LOW)
  if (lastButtonState == HIGH && currentState == LOW) {
    // Switch to next mode
    mode++; 
    if (mode > MAX_MODE) mode = 0;
    
    // Reset waveform state for clean transition
    stepIndex = 0;
    direction = 1;
    
    // Simple debounce delay (shortened to keep loop responsive)
    delay(50); 
  }
  lastButtonState = currentState;
}

/**
 * Updates the OLED display one line at a time (Interleaved).
 * This eliminates the "Big Ripple" by keeping I2C pauses very short.
 */
void updateDisplayStep() {
  // Line 0: Header (System Info)
  if (displayStep == 0) {
    if (mode != lastMode) {
      oled.set1X();
      oled.setCursor(0, 0);
      oled.print("  FUNC GEN v2.0 "); 
      lastMode = mode;
    }
    displayStep++;
  }
  
  // Line 1: Mode Name (Blue Section - Large)
  else if (displayStep == 1) {
    oled.set2X();
    oled.setCursor(0, 2);
    if (mode == 0)      oled.print("SAW      ");
    else if (mode == 1) oled.print("TRI      ");
    else if (mode == 2) oled.print("SINE     ");
    displayStep++;
  }

  // Line 2: Frequency & Timing Value (Blue Section - Large)
  else if (displayStep == 2) {
    oled.set2X();
    oled.setCursor(0, 5);
    
    // 1. Calculate Frequency safely
    float freq = 0;
    if (delayTime > 0) {
      // Calculate microseconds per cycle (period)
      // For delayTime = 1000 and TABLE_SIZE = 256, periodUs = 256,000 (fits in unsigned long)
      unsigned long periodUs = (unsigned long)delayTime * (unsigned long)TABLE_SIZE;
      freq = 1000000.0 / (float)periodUs;
    } else {
      freq = 600.0; // Estimate for delay = 0
    }

    // 2. Display Frequency
    if (freq < 100.0) oled.print(" ");
    oled.print((int)freq); // Print as whole number for maximum stability
    oled.print(" Hz    ");
    
    displayStep = 0; // Loop back
  }
}