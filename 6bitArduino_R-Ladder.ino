// ---------------------------------------------------------
// 6-BIT FUNCTION GENERATOR (v5.0)
// ---------------------------------------------------------
// DAC PINS: D2(LSB) through D7(MSB)
// BUTTON:   D9 (Connect to GND)
// POT:      A0 (Connect between 5V and GND)
// ---------------------------------------------------------

// 256-point Sine Table (Scaled 0-255)
// Corrected 256-point Sine Table (No flat spot in the middle!)
const int TABLE_SIZE = 256; 
byte sineTable[TABLE_SIZE]; 

int mode = 0; // 0=Sawtooth, 1=Triangle, 2=Sine
int lastButtonState = HIGH; 
const int BUTTON_PIN = 10;
const int potPin = A0;

// Waveform State
int stepIndex = 0;
int direction = 1;

void setup() {
   // Set D2 through D9 as OUTPUT for the 8-bit DAC
  for (int i = 2; i <= 9; i++) {
    pinMode(i, OUTPUT);
  }
  // Set the new button pin
  pinMode(BUTTON_PIN, INPUT_PULLUP); // Assuming internal pullup, change if needed
  
  // Mathematically generate the Sine Wave
  // 8-bit range is 0-255. Midpoint is 127.5.
  for (int i = 0; i < TABLE_SIZE; i++) {
    float angle = (float)i / TABLE_SIZE * 2.0 * PI;
    // Calculate and cast to byte (0-255)
    sineTable[i] = (byte)(127.5 + 127.5 * sin(angle));
  }
}

void loop() {
  // --- 1. READ CONTROLS ---
  
  // Speed Math: We reduced the divider from 20000 to 5000
  // because we now have 4x as many steps to draw.
  int potValue = analogRead(potPin);
  int delayTime = 5000 / (potValue + 5); 

  checkSwitch();

  // --- 2. GENERATE WAVEFORMS ---
  
  if (mode == 0) { // SAWTOOTH
    outputDAC(stepIndex);
    stepIndex++;
    if (stepIndex >= TABLE_SIZE) stepIndex = 0; // Reset at 256 to 0
  }
  
  else if (mode == 1) { // TRIANGLE
    stepIndex += direction;
    
    // Bounce off top (255)
    if (stepIndex >= TABLE_SIZE) { 
      stepIndex = TABLE_SIZE; 
      direction = -1; 
    }
    // Bounce off bottom (0)
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

  // --- 3. WAIT ---
  delayMicroseconds(delayTime);
}

// --- HELPER FUNCTIONS ---

// Output the 8-bit value to pins D2-D9
void outputDAC(int value) {
  // Unrolled for speed
  digitalWrite(2, (value >> 0) & 1); // LSB
  digitalWrite(3, (value >> 1) & 1);
  digitalWrite(4, (value >> 2) & 1);
  digitalWrite(5, (value >> 3) & 1);
  digitalWrite(6, (value >> 4) & 1);
  digitalWrite(7, (value >> 5) & 1);
  digitalWrite(8, (value >> 6) & 1); 
  digitalWrite(9, (value >> 7) & 1); // MSB
}

void checkSwitch() {
  int currentState = digitalRead(BUTTON_PIN);
  
  if (lastButtonState == HIGH && currentState == LOW) {
    mode++; 
    if (mode > 2) mode = 0;
    
    stepIndex = 0;
    direction = 1;
    
    delay(200); 
  }
  lastButtonState = currentState;
}