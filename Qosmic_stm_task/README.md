# Qosmic STM Task (STM32H743)

This repository contains the STM32H743 firmware for the FSM control task:
- 10 kHz control loop (`TIM2`)
- QPD read via internal ADC
- PAT state machine + notch filter + PID
- DAC command output over `SPI2`
- telemetry over `UART4`

## Build

### 1) Build in STM32CubeIDE (primary workflow used)

1. Open **STM32CubeIDE**.
2. Import project from this folder (`/home/lakshya/dhurandhar/Qosmic_stm_task`) if not already in workspace.
3. Right click project **Qosmic_stm_task** -> **Build Project**.

CubeIDE uses the ARM GCC toolchain (`arm-none-eabi-gcc`) from its managed build system.

### 2) ARM GCC command-line syntax build (quick verification)

Used to validate source compilation units directly:

```bash
arm-none-eabi-gcc \
  -DSTM32H743xx -DUSE_HAL_DRIVER \
  -ICore/Inc \
  -IDrivers/STM32H7xx_HAL_Driver/Inc \
  -IDrivers/CMSIS/Device/ST/STM32H7xx/Include \
  -IDrivers/CMSIS/Include \
  -fsyntax-only Core/Src/*.c
```

> Note: this is a syntax/compile check for C files, not a full link/ELF generation flow.

## Repository Overview

### Project root

- `Qosmic_stm_task.ioc`  
  STM32CubeMX configuration (clocks, pins, peripherals).
- `.project`, `.cproject`, `.mxproject`, `.settings/`  
  STM32CubeIDE/Eclipse project metadata.
- `STM32H743ZITX_FLASH.ld`, `STM32H743ZITX_RAM.ld`  
  Linker scripts.
- `FSM_Control_Report_Updated.tex`  
  Project technical report source.
- `Debug/`  
  IDE-generated build artifacts/output directory.

### `Core/`

- `Core/Inc/`  
  Application and HAL configuration headers.
  - `fsm_control.h` - top-level control context/state machine API
  - `pid_controller.h` - PID structs/constants/API
  - `biquad_filter.h` - notch filter structs/constants/API
  - `main.h`, `stm32h7xx_it.h`, `stm32h7xx_hal_conf.h` - project/HAL headers
- `Core/Src/`  
  Application source files.
  - `main.c` - init, ISR callback entry, control loop plumbing
  - `fsm_control.c` - PAT + fade handling + notch + PID integration
  - `pid_controller.c` - discrete PID implementation
  - `biquad_filter.c` - biquad notch implementation
  - `stm32h7xx_it.c`, `stm32h7xx_hal_msp.c`, `system_stm32h7xx.c` - startup/interrupt/MSP/system clock support
  - `syscalls.c`, `sysmem.c` - runtime support stubs
- `Core/Startup/`  
  startup assembly and low-level startup files.

### `Drivers/`

- `Drivers/CMSIS/`  
  ARM CMSIS core/device headers for STM32H7.
- `Drivers/STM32H7xx_HAL_Driver/`  
  STM32 HAL driver headers and source used by the project.
