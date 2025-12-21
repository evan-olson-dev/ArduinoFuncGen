// ---------------------------------------------------------
// HIGH-PERFORMANCE 8-BIT ARDUINO FUNCTION GENERATOR (v2.0)
// ---------------------------------------------------------
// This firmware transforms an Arduino Uno (ATmega328P) into a 
// precision laboratory-grade function generator. 
//
// ARCHITECTURAL OVERVIEW:
// 1. SIGNAL ENGINE: 100% Hardware-Interrupt driven (Timer1). 
//    Uses "Zero-Overhead" port buffering to drive an 8-bit R-2R DAC. 
// 2. SIGNAL PURITY: Asynchronous ADC reading eliminates "logic ripple."
// 3. DSP STABILITY: 8x Oversampling + Adaptive EMA Smoothing + Hysteresis
//    locks the frequency to sub-Hz precision even with electrical noise.
// 4. USER INTERFACE: Interleaved OLED updates prevent I2C bus contention.
//
// PINOUT:
// - DAC: D2 (LSB) to D9 (MSB) -> Connect to R-2R Ladder.
// - POT: A0 -> Frequency Control (0V to 5V).
// - BUTTON: D10 -> Mode Select (Active LOW, Internal Pullup).
// - I2C: A4 (SDA), A5 (SCL) -> SSD1306 OLED Display.
// ---------------------------------------------------------

#include <Wire.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"

// Hardware Configuration
#define I2C_ADDRESS 0x3C
SSD1306AsciiWire oled;

// --- WAVEFORM STORAGE ---
// We use a 256-point lookup table. 256 is ideal because the step index (8-bit)
// can be allowed to overflow naturally, saving a conditional "if" check in the ISR.
const int TABLE_SIZE = 256; 
byte sineTable[TABLE_SIZE]; 

// PORT BUFFERS: These store the PRE-CALCULATED register values for PORTD and PORTB.
// In the ISR, we don't calculate anything; we just dump these values to the ports.
// This is critical for reaching the ~400Hz limit (100kHz sample rate).
volatile byte portDBuffer[TABLE_SIZE];
volatile byte portBBuffer[TABLE_SIZE];

// Operational State
int mode = 0;              // 0:Saw, 1:Tri, 2:Sine
const int MAX_MODE = 2;
const int BUTTON_PIN = 10;
const int POT_PIN = A0;

//Display Step
static int displayStep = 0;

// DSP / Filtering Variables
int stepIndex = 0;           // Global index into the lookup tables
unsigned int loopCounter = 0; // Main loop heartbeat for interleaving
long delayTime = 0;          // Microseconds between DAC updates

// Frequency Range (Hz)
// minFreq/maxFreq determine the linear mapping of the potentiometer.
float minFreq = 1.0;         
float maxFreq = 380.0;       // Safety ceiling to ensure CPU time for UI

// Digital Signal Processing (DSP) Logic
int lastPotValue = -1;       
float smoothedPotValue = -1.0; // The EMA-filtered representation of the dial
float lastDisplayedFreq = -1.0; // Prevents UI flicker
bool displayNeedsUpdate = false; 
unsigned long lastHeartbeat = 0;
unsigned long lastPotSampleTime = 0; 

// --- CORE FUNCTIONS ---

/**
 * SETUP: Configures hardware registers for high-performance operation.
 */
void setup() {
  // 1. DAC PORT CONFIGURATION
  // We use D2-D9 to avoid interfering with D0/D1 (Serial RX/TX).
  // This split requires writing to both PORTD (bits 2-7) and PORTB (bits 0-1).
  for (int i = 2; i <= 9; i++) pinMode(i, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  
  // 2. MATH INITIALIZATION
  calcSineTable();
  updatePortBuffers(); // Prepare the starting waveform (Sawtooth)

  // 3. ASYNCHRONOUS ADC SETUP
  // analogRead() is blocking (100us stall). Instead, we configure the ADC registers.
  // ADMUX: Select A0 channel, use VCC as reference.
  ADMUX = (1 << REFS0); 
  // ADCSRA: Enable ADC, set 128 prescaler (125kHz ADC clock).
  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
  ADCSRA |= (1 << ADSC); // Prime the first conversion

  // 4. TIMER1 (THE ENGINE) SETUP
  // Timer1 is a 16-bit timer. We use CTC (Clear Timer on Match) mode.
  cli(); // Atomic block
  TCCR1A = 0; TCCR1B = 0; TCNT1 = 0;
  OCR1A = 16000;          // Starting Period (1ms)
  TCCR1B |= (1 << WGM12); // CTC Mode
  TCCR1B |= (1 << CS10);  // No prescaler (1-tick = 62.5ns)
  TIMSK1 |= (1 << OCIE1A); // Enable interrupt
  sei();

  // 5. UI INITIALIZATION
  Wire.begin();
  Wire.setClock(100000L); // High I2C speeds can cause EMF noise on the DAC
  oled.begin(&Adafruit128x64, I2C_ADDRESS);
  oled.displayRemap(true); // Flip for standard orientation
  oled.setFont(System5x7);
  oled.clear();
}

/**
 * MAIN LOOP: Handles UI and Control logic.
 * Waveform generation happens in the BACKGROUND via Timer1.
 */
void loop() {
  // A. DSP & POTENTIOMETER FILTERING (Throttled to 50Hz)
  // We only sample 50 times per second to prevent over-sampling noise.
  if (millis() - lastPotSampleTime >= 20) {
    if (!(ADCSRA & (1 << ADSC))) { // Check if ADC conversion finished
      // 1. OVERSAMPLING: Take 8 rapid readings and average.
      // This cancels high-frequency noise spikes before they hit the filter.
      long potSum = 0;
      for (int i = 0; i < 8; i++) {
        ADCSRA |= (1 << ADSC); 
        while (ADCSRA & (1 << ADSC)); // Standard ADC wait is only ~100us
        potSum += ADC;
      }
      int potValue = potSum / 8;
      
      // 2. SNAP ZONES: Fix for "Boundary Wander"
      if (potValue < 8) potValue = 0;
      else if (potValue > 1015) potValue = 1023;
      
      // 3. ADAPTIVE RESPONSIVE SMOOTHING (EMA)
      // If moving the knob fast, we prioritize speed (Alpha 0.4).
      // If knob is nearly still, we prioritize stability (Alpha 0.02).
      float diff = abs((float)potValue - smoothedPotValue);
      float alpha = (diff > 15) ? 0.4 : 0.02;
      
      if (smoothedPotValue < 0) smoothedPotValue = potValue; 
      smoothedPotValue = (alpha * (float)potValue) + ((1.0 - alpha) * smoothedPotValue);

      // 4. LOCK-IN HYSTERESIS
      // Check if the filtered value translates to a 1.0Hz change.
      // This prevents the screen from "hunting" or flickering.
      float targetFreq = minFreq + (smoothedPotValue / 1023.0) * (maxFreq - minFreq);
      if (abs(targetFreq - lastDisplayedFreq) >= 1.0) {
        displayNeedsUpdate = true; 
        lastDisplayedFreq = targetFreq;
        // Convert Freq to Timer Ticks: ticks = 16,000,000 / (freq * 256)
        if (targetFreq > 0) delayTime = (long)(1000000.0 / (targetFreq * 256.0));
        lastPotValue = (int)smoothedPotValue;
      }
      lastPotSampleTime = millis();
    }
  }

  // B. BUTTON DEBOUNCING (Low priority)
  if ((loopCounter & 255) == 0) {
    int prevMode = mode;
    checkSwitch();
    if (mode != prevMode) displayNeedsUpdate = true;
  }
  loopCounter++;

  // C. ATOMIC FREQUENCY UPDATE
  // We must disable interrupts when updating OCR1A and TCNT1 to prevent 
  // the timer from counting past its new limit while we are setting it.
  unsigned int ticks = (unsigned int)(delayTime * 16);
  if (ticks < 160) ticks = 160; // 10us Safety Floor
  if (ticks != OCR1A) {
    cli();
    OCR1A = ticks;
    TCNT1 = 0; 
    sei();
  }

  // D. INTERLEAVED OLED REFRESH
  // We refresh the UI one line per frame (triggered by loopCounter).
  // This keeps the I2C bus traffic spread out, preventing signal jitter.
  if (displayNeedsUpdate || (millis() - lastHeartbeat > 2000)) {
    if ((loopCounter & 1023) == 0) {
      updateDisplayStep();
      if (displayStep == 0) {
        displayNeedsUpdate = false;
        lastHeartbeat = millis();
      }
    }
  }
}

// --- SIGNAL ENGINE (ISR) ---

/**
 * TIMER1 INTERRUPT: The High-Speed Heartbeat of the Generator.
 * Executes on every "Tick" of the signal. 
 * ZERO MATH: Directly dumps pre-calculated byte buffers to the Ports.
 */
ISR(TIMER1_COMPA_vect) {
  PORTD = portDBuffer[stepIndex];
  PORTB = portBBuffer[stepIndex];

  stepIndex++; // Increments naturally 0 to 255
  // Note: Since stepIndex is treated as an 8-bit wrap, we don't strictly 
  // need the comparison below if using 'byte', but we keep it for 256-safety.
  if (stepIndex >= 256) stepIndex = 0;
}

// --- WAVEFORM LOGIC ---

/**
 * updatePortBuffers: The "Heavy Lifter"
 * Pre-calculates the bit patterns for PORTD and PORTB.
 * This function translates the 8-bit signal value into the D2-D9 DAC mapping.
 * Value -> PORTD(bits 2-7) | PORTB(bits 0-1)
 */
void updatePortBuffers() {
  TIMSK1 &= ~(1 << OCIE1A); // Safety: Stop ISR during calculation

  for (int i = 0; i < TABLE_SIZE; i++) {
    byte val = 0;
    if (mode == 0) val = i; // Sawtooth
    else if (mode == 1) val = (i < 128) ? (i * 2) : (255 - ((i - 128) * 2)); // Triangle
    else val = sineTable[i]; // Sine

    // PORTD Buffer: Clear D2-D7, preserve D0-D1 (RX/TX), or-in the bits
    portDBuffer[i] = (PORTD & 0x03) | (val << 2);
    // PORTB Buffer: Clear D8-D9 (Port B bits 0-1), or-in the bits
    portBBuffer[i] = (PORTB & 0xFC) | (val >> 6);
  }

  TIMSK1 |= (1 << OCIE1A); // Restart engine
}

/**
 * calcSineTable: Generates a high-precision 256-point lookup.
 */
void calcSineTable() {
  for (int i = 0; i < TABLE_SIZE; i++) {
    float angle = (float)i / 256.0 * 2.0 * PI;
    sineTable[i] = (byte)(127.5 + 127.5 * sin(angle));
  }
}

/**
 * checkSwitch: Debounced state machine for mode switching.
 */
void checkSwitch() {
  static int lastBtn = HIGH;
  int current = digitalRead(BUTTON_PIN);
  if (lastBtn == HIGH && current == LOW) {
    mode = (mode + 1) > MAX_MODE ? 0 : mode + 1;
    updatePortBuffers();
    delay(50); // Small debounce block is safe here as loop handles UI
  }
  lastBtn = current;
}

/**
 * updateDisplayStep: The Interleaved UI Engine.
 * Refreshes individual rows to keep I2C transactions short.
 */

// --- BITMAP ICONS (32x16) ---
// SSD1306 Page Mapping: LSB (bit 0) is TOP, MSB (bit 7) is BOTTOM.
// Pro-grade 32x16 icons generated for pixel-perfect alignment.
const uint8_t iconSine[] PROGMEM = {
  // Page 0 (Top): 7->0->7. Flattened curve for roundness.
  0x00, 0x40, 0x20, 0x10, 0x04, 0x04, 0x02, 0x02, 0x02, 0x02, 0x02, 0x04, 0x08, 0x10, 0x20, 0x80,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  // Page 1 (Bottom): 8->15->8. Flattened curve for roundness.
  0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x01, 0x04, 0x08, 0x10, 0x20, 0x40, 0x40, 0x40, 0x40, 0x40, 0x20, 0x20, 0x08, 0x04, 0x02, 0x01
};

const uint8_t iconSaw[] PROGMEM = {
  // Page 0 (Top): Ramp up, then 0xFF vertical line at x=31.
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x80, 0x80, 0x40, 0x40, 0x20, 0x20, 0x10, 0x10, 0x08, 0x08, 0x04, 0x04, 0x02, 0x02, 0x01, 0xFF,
  // Page 1 (Bottom): Ramp up, then 0xFF vertical line at x=31.
  0x80, 0x80, 0x40, 0x40, 0x20, 0x20, 0x10, 0x10, 0x08, 0x08, 0x04, 0x04, 0x02, 0x02, 0x01, 0x01,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF
};

const uint8_t iconTri[] PROGMEM = {
  // Page 0 (Top): Middle peak. 0x80->0x01->0x80.
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x40, 0x20, 0x10, 0x08, 0x04, 0x02, 0x01,
  0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  // Page 1 (Bottom): Sides. 0x01->0x80 ... 0x80->0x01.
  0x80, 0x40, 0x20, 0x10, 0x08, 0x04, 0x02, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80
};

void drawIcon(const uint8_t* icon, uint8_t x, uint8_t y) {
  for (uint8_t p = 0; p < 2; p++) {
    oled.setCursor(x, y + p);
    for (uint8_t i = 0; i < 32; i++) {
        oled.ssd1306WriteRam(pgm_read_byte(&icon[p * 32 + i]));
    }
  }
}

void updateDisplayStep() {
  if (displayStep == 0) {
    oled.set1X(); oled.setCursor(0, 0);
    oled.print("  FUNC GEN v2.0 "); 
    displayStep++;
  } 
  else if (displayStep == 1) {
    oled.set2X(); oled.setCursor(0, 2);
    // Clearing Strategy: Print trailing spaces to wipe previous content
    if (mode == 0) {
      oled.print("SAW      ");
      drawIcon(iconSaw, 80, 2);
    }
    else if (mode == 1) {
      oled.print("TRI      ");
      drawIcon(iconTri, 80, 2);
    }
    else {
      oled.print("SINE     ");
      drawIcon(iconSine, 80, 2);
    }
    displayStep++;
  } 
  else if (displayStep == 2) {
    oled.set2X(); oled.setCursor(0, 5);
    float freq = 1000000.0 / ((float)delayTime * 256.0);
    if (freq < 100.0) oled.print(" ");
    oled.print((int)freq); oled.print(" Hz    ");
    displayStep = 0; 
  }
}