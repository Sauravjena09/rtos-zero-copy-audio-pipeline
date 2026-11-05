# STM32F4 Zero-Copy Real-Time Audio DSP Pipeline

[![Zephyr RTOS](https://img.shields.io/badge/Zephyr_RTOS-v4.4-blue.svg)](https://zephyrproject.org/)
[![Hardware](https://img.shields.io/badge/Board-STM32F4--Discovery-brightgreen.svg)](https://www.st.com/en/evaluation-tools/stm32f4discovery.html)
[![Core](https://img.shields.io/badge/Core-ARM_Cortex--M4F_@_168MHz-orange.svg)]()
[![DSP](https://img.shields.io/badge/Math-CMSIS--DSP_v1.9-red.svg)]()

A high-performance, real-time embedded audio digital signal processing (DSP) pipeline implemented on the **STM32F407G-DISC1** board using **Zephyr RTOS**.

The system captures raw Pulse Density Modulation (PDM) audio from the onboard ST MP45DT02 MEMS microphone via I2S DMA, converts it into 48 kHz single-precision floating-point audio through a hardware-calibrated 3rd-order Cascaded Integrator-Comb (CIC) decimator, processes the audio through a 512-point Weighted Overlap-Add (WOLA) Short-Time Fourier Transform (STFT) Wiener noise reduction gate, dual-stage calibrated Biquad IIR filters, and a feedforward circular delay-line echo, streaming the processed audio out in real time through the onboard Cirrus Logic CS43L22 DAC into 3.5 mm stereo headphones.

---

## Architecture Overview

```
                                      STM32F407G-DISC1
+-----------------------------------------------------------------------------------------------+
|                                                                                               |
|  [ ST MP45DT02 MEMS Mic ]                                [ Cirrus Logic CS43L22 Audio DAC ]   |
|            |                                                              ^                   |
|     PDM Bitstream (3.072 MHz)                                   16-bit I2S Stereo (48 kHz)    |
|            v                                                              |                   |
|   +------------------+                                          +---------------------+       |
|   |   I2S2 Master    |                                          |     I2S3 Master     |       |
|   |  DMA1 Stream 3   |                                          |    DMA1 Stream 5    |       |
|   +------------------+                                          +---------------------+       |
|            |                                                              ^                   |
|       (2048 Bytes)                                                   (1024 Bytes)             |
|            v                                                              |                   |
|   +------------------+                                          +---------------------+       |
|   |  audio_rx_thread |                                          |   audio_tx_thread   |       |
|   |  (Priority 2)    |                                          |   (Priority 1 - Hi) |       |
|   +------------------+                                          +---------------------+       |
|            |                                                              ^                   |
|     pdm_rx_slab                                                     pcm_tx_slab               |
|            \                                                              /                   |
|             \                    +--------------------+                  /                    |
|              +------------------>|  dsp_worker_thread |-----------------+                     |
|                                  |  (Priority 3)      |                                       |
|                                  +--------------------+                                       |
|                                            |                                                  |
|                                            v                                                  |
|                   +---------------------------------------------------+                       |
|                   |  1. 3rd-Order CIC Decimator (R=64, modulo 2^32)   |                       |
|                   |  2. 1st-Order IIR DC Blocker (fc ≈ 20 Hz)         |                       |
|                   |  3. 15-Tap FIR Inverse-Sinc Droop Compensator     |                       |
|                   |  4. 512-Point WOLA STFT Analysis (50% Overlap)    |                       |
|                   |  5. Spectral Wiener Gate (Asymmetric Tracker)     |                       |
|                   |  6. 512-Point Inverse STFT + Synthesis Window     |                       |
|                   |  7. Dual-Stage Biquad (HPF 150 Hz + LPF 3800 Hz)   |                       |
|                   |  8. Circular Feedforward Stereo Echo (~85.3 ms)   |                       |
|                   |  9. Float-to-Q15 PCM Stereo Interleaver           |                       |
|                   +---------------------------------------------------+                       |
|                                                                                               |
+-----------------------------------------------------------------------------------------------+
```

---

## Key Hardware & Signal Processing Details

### 1. Decoupled 3-Thread RTOS Architecture
Audio pipelines suffer from buffer underruns if audio I/O blocks are subjected to DSP mathematical jitter. We decoupled the pipeline into three strictly prioritized Zephyr threads:
- **`audio_tx_thread` (Priority 1, preemptive)**: Highest priority. Takes pre-rendered PCM blocks from `dsp_output_q` and writes directly to the CS43L22 TX DMA stream. Ensures the DAC DMA queue never starves.
- **`audio_rx_thread` (Priority 2, preemptive)**: High priority. Reads captured 2048-byte PDM blocks from the microphone RX DMA stream without blocking and queues them for DSP.
- **`dsp_worker_thread` (Priority 3, preemptive, 8 KB Stack)**: Compute thread. Executes CIC decimation, STFT FFT transforms, spectral filtering, biquads, and echo buffering.

### 2. Audio PLL & Clock Synchronization
The STM32F4 Audio PLL (`PLLI2S`) is configured to output an exact 86 MHz clock ($8\text{ MHz} / 8 \times 258 / 3$):
- **Microphone RX (I2S2)**: Master mode, generating a **3.072 MHz bit clock** on `PB10`. Configured with `I2S_FMT_BIT_CLK_INV` so that the STM32 samples data on the **falling clock edge** when the onboard MP45DT02 microphone is actively driving the bus (avoiding the rising-edge high-impedance state).
- **DAC TX (I2S3)**: Master mode, driving `PC7` with a **12.288 MHz Master Clock (MCLK)** ($256 \times F_s$) and `PC10` with a **1.536 MHz Bit Clock** ($32 \times F_s$) for pristine 48 kHz, 16-bit stereo playback.

### 3. PDM-to-PCM Decimator
- **CIC Filter**: 3rd-order Cascaded Integrator-Comb architecture with decimation factor $R = 64$ ($3.072\text{ MHz} \rightarrow 48\text{ kHz}$). Integrators and combs utilize 32-bit unsigned arithmetic to guarantee well-defined modulo $2^{32}$ wrap-around without signed integer overflow undefined behavior.
- **DC Blocker**: 1st-order IIR filter ($y[n] = 0.995 (x[n] - x[n-1]) + 0.995 y[n-1]$) to eliminate MEMS acoustic DC bias.
- **Droop Compensator**: 15-tap linear-phase FIR filter ($h_{CIC}^{-3}$) that completely flattens passband sinc-droop up to 20 kHz.

### 4. 512-Point WOLA STFT Wiener Spectral Gate
- **Analysis Window**: Square-root Hann window $\sin(\pi n / N)$ with 50% overlap ($N = 512$, Hop $H = 256$).
- **CMSIS-DSP RFFT**: Accelerated real-valued Fast Fourier Transform (`arm_rfft_fast_f32`).
- **Wiener Noise Suppression**: Asymmetric noise-floor energy tracker ($\alpha_{up} = 0.999$, $\alpha_{down} = 0.90$) estimating ambient background noise per bin. Calculates smooth non-subtractive Wiener gain:
  $$G[k] = \frac{P_{sig}[k]}{P_{sig}[k] + 1.2 \cdot N_{floor}[k]}$$
  with a gentle -12 dB suppression floor to eliminate musical noise.
- **Synthesis & Overlap-Add**: Synthesis windowing and circular tail reconstruction reconstructs time-domain audio with zero spectral distortion.

### 5. Dual Biquad Filter & Studio Delay
- **Biquad Cascade**: Two Direct Form I IIR second-order stages:
  - **Stage 1 (HPF @ 150 Hz)**: Removes mic handling thump, breath pops, and 50/60 Hz AC hum.
  - **Stage 2 (LPF @ 3800 Hz)**: Anti-aliasing filter optimizing human vocal clarity.
- **Echo**: 4096-sample circular feedforward delay line ($85.3\text{ ms}$ delay) with 20% studio reflection balanced across both headphone channels.

---

## Hardware Pinout (Onboard STM32F4-Discovery)

All audio peripherals are hardwired onboard—**no external jumper wires are needed**:

| Peripheral | Signal | Pin | Description |
|---|---|---|---|
| **MP45DT02 MEMS Mic** | I2S2_CK | `PB10` | 3.072 MHz PDM Bit Clock |
| **MP45DT02 MEMS Mic** | I2S2_SD | `PC3`  | PDM Serial Data |
| **CS43L22 Audio DAC** | I2S3_WS | `PA4`  | Word Select (48 kHz LRCLK) |
| **CS43L22 Audio DAC** | I2S3_CK | `PC10` | Serial Clock (1.536 MHz) |
| **CS43L22 Audio DAC** | I2S3_SD | `PC12` | Serial PCM Audio Data |
| **CS43L22 Audio DAC** | I2S3_MCK | `PC7`  | Master Clock (12.288 MHz) |
| **CS43L22 Audio DAC** | I2C1_SCL | `PB6`  | I2C Clock (Control Plane) |
| **CS43L22 Audio DAC** | I2C1_SDA | `PB9`  | I2C Data (Control Plane) |
| **CS43L22 Audio DAC** | RESET | `PD4`  | Active-Low Hardware Reset |

---

## Repository Structure

```
├── CMakeLists.txt              # Root build script with compiler warning flags
├── prj.conf                    # Zephyr kernel configuration & CMSIS-DSP enablement
├── boards/
│   └── stm32f4_disco.overlay   # DeviceTree overlay for I2S2, I2S3, I2C1, DMA & PLLI2S
├── src/
│   ├── main.c                  # Bootloader, peripheral init, CS43L22 setup & DMA trigger
│   ├── audio_pipeline.h        # Buffer geometries, thread priorities, STFT parameters
│   ├── audio_pipeline.c        # 3-thread decoupled RTOS engine & zero-copy slabs
│   └── dsp_filters.c           # WOLA STFT Wiener gate, Biquad filters, Echo line
├── filter/
│   ├── CMakeLists.txt          # Decimator library build definition
│   ├── pdm_decimator.h         # CIC filter state structures
│   └── pdm_decimator.c         # 3rd-order CIC, DC blocker & 15-tap FIR droop compensator
└── README.md                   # System documentation
```

---

## Building and Flashing

### Prerequisites
- [Zephyr SDK](https://docs.zephyrproject.org/latest/develop/getting_started/index.html) and `west` installed.
- ST-LINK utility / STM32CubeProgrammer installed.

### 1. Build Firmware
From the root of your Zephyr workspace:
```bash
west build -b stm32f4_disco stm32f4_audio_pipeline
```

### 2. Flash to Board
Connect your STM32F4-Discovery board via the ST-LINK Mini-USB port (`CN1`) and run:
```bash
west flash
```

### 3. Connect Headphones & Listen
Plug standard 3.5 mm headphones into the onboard black audio jack (`CN4`). Speak into the MEMS microphone (`U5`, located between the two push buttons) to hear clean real-time audio!

---

## License
Apache License 2.0.
