# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

STM32F103C8 (Cortex-M3) based **FOC (Field-Oriented Control)** motor driver using SPWM modulation. Uses STM32CubeMX HAL, built with Keil MDK-ARM (uVision 5, ARMCC V5.06).

## Build

Open `MDK-ARM/FOC_SPWM.uvprojx` in Keil uVision 5, click Build (F7). The `.hex` output goes to `MDK-ARM/FOC_SPWM/`.

Hardware configuration is in `FOC_SPWM.ioc` — open with STM32CubeMX to modify pinout/peripheral settings, then regenerate code. User code lives between `USER CODE BEGIN/END` markers and survives regeneration.

## Architecture

### Control loop (1 kHz, TIM2 ISR → `HAL_TIM_PeriodElapsedCallback` in `Core/Src/main.c`)

```
AS5600_Update → switch(ctrl_mode) → set_Foc_{current|speed|angle}() → PID → setTorque() → Park/Clarke → setPwm() → TIM1 CCR
```

- **TIM1**: 3-channel PWM output (CH1/CH2/CH3)
- **TIM2**: 1 ms periodic interrupt drives the FOC control loop
- **ADC1+ADC2**: Injected simultaneous sampling of phase currents (2-shunt)
- **I2C1**: AS5600 magnetic encoder
- **UART3**: DMA + idle-line detection for serial commands; also printf output

### Control modes (`FOC_ControlMode` enum in `Core/Inc/FOC.h`)

| Mode | Command | Description |
|------|---------|-------------|
| `FOC_MODE_IDLE` | `stop` | Zero torque |
| `FOC_MODE_CURRENT` | `cur_<A>` | Torque/current loop |
| `FOC_MODE_VELOCITY` | `vel_<rad/s>` | Speed loop (cascaded angle→speed→current PID) |
| `FOC_MODE_ANGLE` | `ang_<rad>` | Position loop (cascaded angle→speed→current PID) |

### Key modules (`Core/Inc/` and `Core/Src/`)

- **`FOC.c/h`** — Park/Clarke transforms, PWM duty calculation, FOC init (auto-calibrates zero electrical angle), 3 control mode entry points
- **`pid.c/h`** — Three independent PID controllers: `pid_angle`, `pid_speed`, `pid_current`. Cascaded for angle/speed modes
- **`as5600.c/h`** — AS5600 I2C magnetic encoder driver: raw angle, accumulated angle (multi-turn), velocity
- **`InlineCurrent.c/h`** — 2-shunt inline phase current sensing via ADC: offset calibration, ADC→voltage→current conversion
- **`LowPassFilter.c/h`** — First-order IIR low-pass filter (time-constant based), used for speed and Iq filtering
- **`main.c`** — System init, control loop dispatch, serial command parsing (`Process_Serial_Command`), telemetry output via printf and VOFA+ protocol

### FOC transform chain

1. **Clarke** (Ia, Ib → Iα, Iβ) for current feedback
2. **Park** (Iα, Iβ → Iq, Id) using electrical angle
3. **PID** on Iq (current) / speed / angle error
4. **Inverse Park** (Uq, Ud → Uα, Uβ) using electrical angle
5. **Inverse Clarke** (Uα, Uβ → Ua, Ub, Uc) for SPWM

### Critical parameters

- `PP` — motor pole pairs
- `DIR` — direction (±1)
- `zero_electric_angle` — auto-calibrated on FOC init (averages 10 readings)
- `voltage_power_supply` — DC bus voltage (default 12.0 V)
- PWM period is 2400 counts; max duty capped at 2160 (10% dead-time)
- `CurrentSensor` configured with 0.02 Ω shunt, 50× gain

### Code generation boundary

CubeMX auto-generates files in `Core/` with `USER CODE BEGIN/END` markers. Custom code outside these markers will be **overwritten** on regeneration. All HAL peripheral init functions (`MX_*_Init`) and `SystemClock_Config` are auto-generated.

## Git

Build outputs (`.o`, `.crf`, `.axf`, `.hex`, `.map`, `.dep`, `.lnp`) are gitignored. Only commit source files under `Core/` and `Drivers/`, the `.ioc` project file, and the Keil `.uvprojx`/`.uvoptx` project files.
