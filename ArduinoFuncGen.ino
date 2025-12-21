// ---------------------------------------------------------
// 8-BIT ARDUINO FUNCTION GENERATOR
// ---------------------------------------------------------
// This sketch turns an Arduino into a simple waveform generator
// outputting Sine, Triangle, and Sawtooth waves via an 8-bit R-2R DAC.
//
// DAC PINS: D2(LSB) through D9(MSB)
// BUTTON:   D10 (Connect to GND, enables internal pullup)
// POT:      A0 (Connect between 5V and GND for frequency control)
// ---------------------------------------------------------

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
int pot100 = 0;               // Potentiometer value mapped 0-100
int delayTime = 0;            // Current delay in microseconds

// USER CONFIGURABLE FREQUENCY RANGE
// Adjust these to set your desired frequency bounds
int minFreqDelay = 1000;   // Delay for 0% (in microseconds)
int maxFreqDelay = 0;      // Delay for 100% (in microseconds)

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
  // Set ADC reference to AVCC (5V) and select A0 channel
  ADMUX = (1 << REFS0); 
  // Enable ADC, set prescaler to 128 (16MHz/128 = 125kHz ADC clock)
  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
  // Start the first conversion
  ADCSRA |= (1 << ADSC);
}

// --- MAIN LOOP ---
void loop() {
  // --- 1. READ CONTROLS (Non-Blocking) ---
  
  // Check if the ADC conversion is finished (ADSC bit goes to 0)
  if (!(ADCSRA & (1 << ADSC))) {
    int potValue = ADC; // Read the 10-bit result
    
    // Convert analog read (0-1023) to 0-100
    pot100 = map(potValue, 0, 1023, 0, 100);
    
    // Map 0-100 to the user-defined delay range
    delayTime = map(pot100, 0, 100, minFreqDelay, maxFreqDelay);
    
    // Start the NEXT conversion immediately
    ADCSRA |= (1 << ADSC);
  }

  // Periodically check the button (much faster than analog read)
  if ((loopCounter & 255) == 0) {
    checkSwitch();
  }
  loopCounter++;

  // --- 2. GENERATE WAVEFORMS ---
  
  if (mode == 0) { // SAWTOOTH
    // Ramps up from 0 to 255, then instantly resets to 0.
    outputDAC(stepIndex);
    stepIndex++;
    if (stepIndex >= TABLE_SIZE) stepIndex = 0; 
  }
  
  else if (mode == 1) { // TRIANGLE
    // Ramps up to 255, then ramps down to 0.
    stepIndex += direction;
    
    // Top bounce
    if (stepIndex >= TABLE_SIZE - 1) { 
      stepIndex = TABLE_SIZE - 1; 
      direction = -1; 
    }
    // Bottom bounce
    if (stepIndex <= 0) { 
      stepIndex = 0; 
      direction = 1; 
    }
    outputDAC(stepIndex);
  }
  
  else if (mode == 2) { // SINE
    // Outputs values from the pre-calculated sine table.
    outputDAC(sineTable[stepIndex]);
    stepIndex++;
    if (stepIndex >= TABLE_SIZE) stepIndex = 0;
  }

  // --- 3. TIMING CONTROL ---
  // Controls the frequency of the waveform
  delayMicroseconds(delayTime);
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
    
    // Simple debounce delay
    delay(200); 
  }
  lastButtonState = currentState;
}