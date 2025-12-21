# Signal Engine Architecture ⚙️

This document explains the low-level hardware optimizations used to achieve high-frequency, jitter-free waveform generation on the 16MHz ATmega328P.

## 1. Timer1 Hardware Engine
The core of the generator is a **Timer1 Capture Tape Compare (CTC)** interrupt. Unlike software-timed loops, a hardware interrupt fires at the exact same clock cycle every time, ensuring a perfectly stable frequency.

*   **Mode**: CTC (Clear Timer on Compare Match).
*   **Prescaler**: 1 (No division). Every timer "tick" is exactly 62.5ns.
*   **ISR (Interrupt Service Routine)**: `ISR(TIMER1_COMPA_vect)`

## 2. Zero-Math Interrupts (Port Buffering)
To reach a 100kHz sample rate (~400Hz wave at 256 points), the interrupt cannot perform any math, shifts, or logic. 

Every time you change the mode or frequency, the system calculates a **Port Buffer** in the background. In the interrupt, we simply dump these pre-calculated bytes directly to the ports:

```cpp
ISR(TIMER1_COMPA_vect) {
  PORTD = portDBuffer[stepIndex]; // Drive D2-D7
  PORTB = portBBuffer[stepIndex]; // Drive D8-D9
  stepIndex++; // Naturally wraps at 256
}
```

## 3. Atomic Updates
Changing frequency requires updating the 16-bit `OCR1A` register. If the timer is halfway through a count when this happens, it can "miss" the new limit and count all the way to 65,535, causing a massive glitch. 

We use **Atomic Blocks** (`cli()`/`sei()`) and force a `TCNT1 = 0` reset during every frequency change to ensure the transition is instant and glitch-free.

## 4. Hardware Pin Mapping
To avoid interfering with the Arduino Serial Pins (D0/D1), we use pins D2 through D9. This spans two physical hardware ports:
*   **PORTD (Bits 2-7)**: Drives DAC Bits 0-5.
*   **PORTB (Bits 0-1)**: Drives DAC Bits 6-7.

The `updatePortBuffers()` function handles the complex binary shifting required to map an 8-bit value across these divided ports.
