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

// PORT BUFFERS: Stores pre-calculated DAC bit patterns for ultra-fast ISR
// This moves the "bit shifting" logic out of the interrupt.
volatile byte portDBuffer[TABLE_SIZE];
volatile byte portBBuffer[TABLE_SIZE];

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

// USER CONFIGURABLE FREQUENCY RANGE (In Hz)
// Linear mapping ensures the dial feels consistent from low to high frequencies.
float minFreq = 1.0;     // Frequency at 0% (in Hz)
float maxFreq = 380.0;   // Frequency at 100% (in Hz, limited by 10us safety floor)

// OLED Refresh Timing
int displayStep = 0;               // To interleave text updates
int lastMode = -1;                 // To trigger full screen clear on mode change
int lastPotValue = -1;             // To detect significant changes
float smoothedPotValue = -1.0;     // Software filter (EMA)
float lastDisplayedFreq = -1.0;    // To detect human-sized Hz changes
bool displayNeedsUpdate = false;   // Flag to silence I2C bus when idle
unsigned long lastHeartbeat = 0;   // To force occasional refresh
unsigned long lastPotSampleTime = 0; // To throttle sampling at low frequencies

// --- SETUP ---
void setup() {
  // 1. Configure DAC Pins (D2-D9) as OUTPUT
  for (int i = 2; i <= 9; i++) {
    pinMode(i, OUTPUT);
  }
  
  // 2. Configure Input Button
  pinMode(BUTTON_PIN, INPUT_PULLUP); 
  
  // 3. Pre-calculate lookup tables
  calcSineTable();
  updatePortBuffers(); // Initialize buffers for the starting mode

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
  // --- 1. READ INPUTS (Throttled to 50Hz) ---
  // Throttling prevents the ADC from being over-sampled when the CPU is idle 
  // at low frequencies, which otherwise amplifies electrical noise.
  if (millis() - lastPotSampleTime >= 20) {
    // Check if the ADC conversion is finished (ADSC bit goes to 0)
    if (!(ADCSRA & (1 << ADSC))) {
      int potValue = ADC; // Read the full 10-bit result (0-1023)
      
      // SNAP ZONES: Force absolute extremes at the edges of the dial
      // to ensure reliable access to min/max despite the delta filter.
      if (potValue < 8) potValue = 0;
      else if (potValue > 1015) potValue = 1023;
      
      // --- ADAPTIVE RESPONSIVE SMOOTHING ---
      // We adjust the filter strength (alpha) based on how fast the pot is moving.
      // If moving fast, alpha is high (low lag). If nearly still, alpha is tiny (heavy filtering).
      float diff = abs((float)potValue - smoothedPotValue);
      float alpha = 0.05; // Heaviest smoothing for idle
      if (diff > 10) alpha = 0.3; // Responsive mode for actual tuning
      
      if (smoothedPotValue < 0) smoothedPotValue = potValue; // First run
      smoothedPotValue = (alpha * (float)potValue) + ((1.0 - alpha) * smoothedPotValue);

      // --- FREQUENCY HYSTERESIS (LOCK-IN) ---
      // Only recalculate the hardware delay and update the display if the
      // change is significant enough to "break the lock."
      float targetFreq = minFreq + (smoothedPotValue / 1023.0) * (maxFreq - minFreq);
      
      if (abs(targetFreq - lastDisplayedFreq) >= 1.0) {
        // LOCK IN: Frequency is updated and display flag is set
        displayNeedsUpdate = true; 
        lastDisplayedFreq = targetFreq;

        if (targetFreq > 0) {
          delayTime = (long)(1000000.0 / (targetFreq * (float)TABLE_SIZE));
        }
        
        lastPotValue = (int)smoothedPotValue;
      }
      
      // Start the NEXT conversion immediately
      ADCSRA |= (1 << ADSC);
      lastPotSampleTime = millis();
    }
  }

  // Periodically check the button (much faster than analog read)
  if ((loopCounter & 255) == 0) {
    int prevMode = mode;
    checkSwitch();
    if (mode != prevMode) displayNeedsUpdate = true;
  }
  loopCounter++;

  // --- 2. UPDATE SYSTEM TIMING ---
  // Atomic update: Reset timer count when changing frequency
  // to prevent the "wrap-around" lock-up.
  unsigned int ticks = (unsigned int)(delayTime * 16);
  if (ticks < 160) ticks = 160; // 10us safety floor
  
  if (ticks != OCR1A) {
    cli();
    OCR1A = ticks;
    TCNT1 = 0; // Reset counter so it doesn't have to wait for 65k wrap
    sei();
  }

  // --- 3. SILENT DISPLAY UPDATES ---
  // Only update the display if something changed, OR every 2 seconds (heartbeat)
  if (displayNeedsUpdate || (millis() - lastHeartbeat > 2000)) {
    if ((loopCounter & 1023) == 0) {
      updateDisplayStep();
      // If we finished all 3 steps of the update, clear the flag
      if (displayStep == 0) {
        displayNeedsUpdate = false;
        lastHeartbeat = millis();
      }
    }
  }
}

// --- HARDWARE INTERRUPT (PRO MODE) ---
/**
 * Timer1 Interrupt Service Routine (ISR)
 * Fired by hardware based on the OCR1A value.
 * This handles the waveform generation independently of the main loop.
 */
/**
 * Timer1 Interrupt Service Routine (ISR)
 * ULTRA-OPTIMIZED: Uses pre-calculated buffers to drive the DAC.
 */
ISR(TIMER1_COMPA_vect) {
  // Output pre-calculated port values (Fastest possible DAC drive)
  PORTD = portDBuffer[stepIndex];
  PORTB = portBBuffer[stepIndex];

  stepIndex++;
  if (stepIndex >= TABLE_SIZE) stepIndex = 0;
}

// --- HELPER FUNCTIONS ---

// --- RE-ARCHITECTED WAVEFORM ENGINE ---

/**
 * Pre-calculates the Sine table values.
 */
void calcSineTable() {
  for (int i = 0; i < TABLE_SIZE; i++) {
    float angle = (float)i / TABLE_SIZE * 2.0 * PI;
    sineTable[i] = (byte)(127.5 + 127.5 * sin(angle));
  }
}

/**
 * Pre-calculates the bit patterns for the CURRENT mode into Port Buffers.
 * This moves all the "Heavy Lifting" (shifts, masks, conditional logic)
 * out of the interrupt and into the background.
 */
void updatePortBuffers() {
  // Temporarily disable timer to prevent glitching during calculation
  TIMSK1 &= ~(1 << OCIE1A);

  for (int i = 0; i < TABLE_SIZE; i++) {
    byte val = 0;
    
    // Choose the raw 8-bit value based on mode
    if (mode == 0) { // SAWTOOTH
      val = i;
    } else if (mode == 1) { // TRIANGLE
      // Map index to a triangle wave (0-255-0)
      if (i < 128) val = i * 2;
      else         val = 255 - ((i - 128) * 2);
    } else if (mode == 2) { // SINE
      val = sineTable[i];
    }

    // MAP VALUE TO HARDWARE PORTS
    // DAC Bit 0-5 (Value bits 0-5) -> Pins D2-D7 (Port D bits 2-7)
    // DAC Bit 6-7 (Value bits 6-7) -> Pins D8-D9 (Port B bits 0-1)
    
    // Port D Buffer: Preserve RX/TX (bits 0-1) and set bits 2-7
    portDBuffer[i] = (PORTD & 0x03) | (val << 2);
    // Port B Buffer: Preserve bits 2-7 and set bits 0-1
    portBBuffer[i] = (PORTB & 0xFC) | (val >> 6);
  }

  // Re-enable timer interrupt
  TIMSK1 |= (1 << OCIE1A);
}

/**
 * Writes an 8-bit value to the DAC pins (Legacy support/Setup)
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
    
    // Pre-calculate the new waveform's bit patterns
    updatePortBuffers();
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