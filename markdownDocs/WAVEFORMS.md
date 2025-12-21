# Waveform Mathematics 📐

This document explains how the Sine, Triangle, and Sawtooth waveforms are mathematically generated and mapped to the 8-bit (0-255) DAC range.

## 1. Table Generation
We use a **256-point lookup table**. 256 is the "Magic Number" for 8-bit microcontrollers because an 8-bit counter (`byte`) will naturally wrap from 255 back to 0, eliminating the need for an `if (index > 255)` check in high-speed code.

### Sine Wave
Generated using the standard sine function, scaled to fit a single byte:
$$Value_i = 127.5 + 127.5 \times \sin\left(\frac{i}{256} \times 2\pi\right)$$

### Triangle Wave
Generated using a linear slope that bounces at the halfway point:
*   From 0 to 127: $Value = i \times 2$
*   From 128 to 255: $Value = 255 - ((i - 128) \times 2)$

### Sawtooth Wave
The simplest waveform. The value is simply equal to the current index $i$.

## 2. Pre-Calculation (The Buffer Strategy)
To keep the output speed high, we don't calculate these values in the interrupt. 
*   **Sine** is pre-calculated once at startup into `sineTable[]`.
*   **Triangle** and **Sawtooth** are generated on-the-fly when you switch modes and stored into the **Port Buffers**.

## 3. DAC Translation (R-2R Ladder)
An R-2R ladder is a simple Digital-to-Analog converter. It expects the LSB (Least Significant Bit) on the first pin and the MSB (Most Significant Bit) on the last.

Our code ensures that the 8-bit value (0-255) is correctly "exploded" across the Port D and Port B pins to maintain the correct binary weighting for the R-2R ladder.
