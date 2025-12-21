# 8-Bit High-Performance Function Generator

This project transforms a standard Arduino Uno into a laboratory-grade function generator capable of producing Sine, Triangle, and Sawtooth waves with ultra-stable frequency control and high-speed output.

## 🚀 Key Features

*   **Interrupt-Driven Hardware Engine**: Waveforms are generated using Timer1 hardware interrupts for jitter-free output up to ~400Hz (100kHz sample rate).
*   **Zero-Overhead DAC Drive**: Uses pre-calculated port buffers and direct register manipulation for maximum execution speed.
*   **Lab-Grade DSP Filtering**: 8x Oversampling, Adaptive EMA Smoothing, and Lock-in Hysteresis ensure a perfectly stationary signal.
*   **High-Visibility UI**: Interleaved SSD1306 OLED updates with `set2X` double-height text for clarity.
*   **Linear Frequency Mapping**: Intuitive 1Hz to 380Hz control across the entire dial.

## 📂 Technical Documentation

To keep the codebase easy to maintain, the documentation is broken down into specialized architectural guides in the `markdownDocs/` folder:

*   [**Signal Engine Guide** (markdownDocs/ENGINE.md)](markdownDocs/ENGINE.md) - Deep dive into Timer1 interrupts, port buffering, and hardware register configuration.
*   [**DSP & Filtering Guide** (markdownDocs/FILTERING.md)](markdownDocs/FILTERING.md) - Mathematical explanation of oversampling, adaptive alpha filters, and the frequency lock-in system.
*   [**Waveform Mathematics** (markdownDocs/WAVEFORMS.md)](markdownDocs/WAVEFORMS.md) - How lookup tables are generated and scaled for 8-bit R2R DACs.

## 🛠️ Hardware Setup

### 1. The DAC (R-2R Ladder)
Construct an 8-bit R-2R ladder and connect it to the following pins:
*   **D2 (LSB)** to **D9 (MSB)**

### 2. Control & UI
*   **Potentiometer**: Connect to **A0** (Voltage divider between 5V and GND).
*   **Mode Button**: Connect between **D10** and **GND**.
*   **OLED Display**: Connect via I2C (**A4/SDA**, **A5/SCL**).

## 📥 Installation

1.  Install the **SSD1306Ascii** library (by Bill Greiman) via the Arduino Library Manager.
2.  Open `ArduinoFuncGen.ino` in the Arduino IDE.
3.  Select **Arduino Uno** and your COM port.
4.  Upload and enjoy the clean signal!

---
*Created with 💙 for the Arduino community.*
