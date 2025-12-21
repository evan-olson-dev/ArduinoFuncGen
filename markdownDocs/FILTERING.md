# DSP & Stability Filtering 🛡️

This guide explains the "Lab-Grade" stabilization techniques that make the frequency control feel premium and prevent electrical noise from causing frequency "hunting."

## 1. 8x Oversampling
Electrical noise on the 5V rail can cause the analog-to-digital converter (ADC) to jitter between values. To solve this, the source code takes **8 rapid-fire samples** every 20ms and averages them. 

$$Value_{avg} = \frac{\sum_{i=1}^{8} Sample_i}{8}$$

This mathematically "cancels out" random spikes before they reach the main frequency logic.

## 2. Adaptive Responsive Smoothing (EMA)
We use an **Exponential Moving Average (EMA)** filter to smooth the dial, but with a twist: the "weight" of the filter changes based on your hand movement.

*   **Idle Mode (Alpha 0.02)**: When the knob is still, the filter has "Infinite Mass." 98% of the previous value is kept, making it practically impossible for noise to move the frequency.
*   **Tuning Mode (Alpha 0.4)**: When you turn the knob quickly (>15 units of change), the filter becomes light and responsive, tracking your movement with zero perceived lag.

Formula: $Smoothed = (\alpha \times Raw) + ((1 - \alpha) \times Smoothed)$

## 3. Lock-in Hysteresis
To prevent the OLED display and the hardware timer from "flickering" between two values (e.g., 50Hz and 51Hz), we implement a **Software Lock**.

The system calculates the target frequency but refuses to update the hardware registers or the screen unless the change is **at least 1.0 Hz**. This ensures that once you let go of the knob, the output stays perfectly locked to a single value.

## 4. Interleaved UI Protection
Updating an I2C OLED display is "noisy"—it creates electrical interference and takes CPU time. To protect the signal engine:
1.  **I2C Speed**: Set to 100kHz (Standard) to reduce EMF emission.
2.  **Row-by-Row Updates**: The screen refreshes only one line of text at a time. This keeps the I2C "bursts" short and prevents the main loop from stalling.
