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
}

// --- MAIN LOOP ---
void loop() {
  // --- 1. READ CONTROLS ---
  
  // Read frequency potentiometer
  // Mapping logic: Higher pot value -> smaller delay -> higher frequency.
  // The '5000' constant sets the base frequency range.
  int potValue = analogRead(POT_PIN);
  int delayTime = 5000 / (potValue + 5); 

  // Check for mode switch button press
  checkSwitch();

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
 * Writes an 8-bit value to the DAC pins (D2-D9).
 * 
 * @param value The byte value (0-255) to output.
 * 
 * Optimization Note:
 * This function uses bitwise operations to extract each bit of the 'value'
 * and write it to the corresponding digital pin. 
 * 'value >> N' shifts the bits right by N positions.
 * '& 1' selects the least significant bit of the result.
 */
void outputDAC(int value) {
  // LSB (Bit 0) -> Pin 2
  digitalWrite(2, (value >> 0) & 1); 
  digitalWrite(3, (value >> 1) & 1);
  digitalWrite(4, (value >> 2) & 1);
  digitalWrite(5, (value >> 3) & 1);
  digitalWrite(6, (value >> 4) & 1);
  digitalWrite(7, (value >> 5) & 1);
  digitalWrite(8, (value >> 6) & 1); 
  // MSB (Bit 7) -> Pin 9
  digitalWrite(9, (value >> 7) & 1); 
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