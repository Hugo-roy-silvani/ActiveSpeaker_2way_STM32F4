# ActiveSpeaker_2way_STM32

A two-way active speaker system powered by an STM32F407, featuring a real-time DSP crossover, gain control, and I2S audio processing.  
This project demonstrates how to implement embedded audio DSP on ARM Cortex-M architecture using block-based processing.

## Project Status
This project is currently under active development.  
Some features may be incomplete or subject to change.
---

## Overview

This prototype processes audio in real time to drive a **two-band active loudspeaker** (low and high channels).  
It uses:

- ARM Cortex-M4 FPU  
- I2S digital audio interface  
- CMSIS-DSP functions  
- 128-sample block processing  
- Real-time crossover and gain control  

The system is designed as a lightweight, modular DSP pipeline suitable for embedded audio applications.

---

## DSP Pipeline


Block Processing (128 samples)  
↓  
Two-Way Crossover (IIR / FIR depending on config)  
↓  
Gain Processing (per band)  
↓  
I2S Output → Amplifier → Speakers  


## Hardware Requirements

- **STM32F407 Discovery board (STM32F407G-DISC1)**  
- **I2S codec or I2S amplifier** (e.g. PCM5102, MAX98357A)  
- **Two loudspeaker drivers** (low + high frequency)
  
---

## Software Requirements

Before building the project, you must install:

- **STM32CubeIDE** (recommended)  
- **STM32CubeMX** (optional but useful)  
- **ARM CMSIS-DSP library**  

---

## Adding CMSIS-DSP (Required Step)

CMSIS-DSP is NOT included in the repository.  
To set it up manually:

1. Download CMSIS-DSP from Arm:  
   https://github.com/ARM-software/CMISIS-DSP  

2. Copy the following folders into your project:
CMSIS/DSP/Include/
CMSIS/DSP/Source/
CMSIS/DSP/PrivateInclude

3. In STM32CubeIDE:
- Right-click project → **Properties**  
- Go to **C/C++ General → Paths and Symbols**  
- Add the `Include` directory to the include path

## Project Goals

- Explore real-time DSP implementation on STM32  
- Build a functional two-way active speaker engine  
- Demonstrate crossover filtering using ARM CMSIS-DSP  
- Provide a base for advanced features (EQ, limiter, spatialisation, etc.) 
