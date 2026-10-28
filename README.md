# Bio-Signal Driven Multi-Mode RISC-V Processor for Wearable Cardiac Monitoring

A precision-scalable, three-core RISC-V based processor in Verilog that adapts its arithmetic precision to the severity of the patient's ECG signal, to extend wearable battery life.

B.Tech final year project, School of Electrical & Electronics Engineering, SASTRA Deemed to be University (2025-26).
Guide: Dr. R. Balasubramanian, Associate Professor.

## Overview

Wearable ECG monitors usually run at full precision all the time, which drains the battery quickly. Most of the time the heart is in normal sinus rhythm and does not need it. This design switches between three cores depending on the ECG condition:

| | Core 1 (Sleep) | Core 2 (Monitor) | Core 3 (Emergency) |
|---|---|---|---|
| Bit width | 8-bit | 16-bit | 32-bit |
| Adder | Ripple carry | TCSA (approximate) | CLA (exact) |
| Multiplier | None | BAM (approximate) | Wallace tree (exact) |
| Relative power | ~6% | ~28% | 100% |
| Register banks | 1 (x0-x7) | 2 (x0-x15) | 4 (x0-x31) |
| Power saving | Clock gating (ICG) + power gating | Clock gating | Clock gating |

TCSA = Truncated Carry Select Adder, BAM = Broken Array Multiplier, CLA = Carry Lookahead Adder.

## Key features

- **Mode Controller FSM** with three states (SLEEP, MONITOR, EMERGENCY) and hysteresis: 3 consecutive elevated cycles to escalate, 10 consecutive normal cycles to de-escalate.
- **Non-Maskable Interrupt (NMI)** that activates Core 3 within 5 clock cycles (50 ns at 100 MHz).
- **MSB packing**: the 8-bit ECG amplitude is placed in bits [31:24] of a 32-bit operand, so all three cores read the same input word at different precisions.
- **Error Estimation Unit** in Core 2 that raises a flag when the approximate result drifts past the clinical threshold, triggering escalation to Core 3.
- **Vector Processing Unit** in Core 2 for the average and peak of four ECG samples (QRS / R-peak detection).
- **Manual force-mode** input for testing and demonstration.

## Repository structure

```
rtl/    Verilog design files
        core1_sleep.v, core2_monitor.v, core3_emergency.v,
        mode_controller_fsm.v, top_system.v
tb/     Testbench
        tb_full_system.v
docs/   Final project report (PDF)
```

## How to simulate (Vivado)

1. Create a new Vivado project (the project was built for the Zynq-7010 part `xc7z010iclg225-1L`).
2. Add all files in `rtl/` as design sources.
3. Add `tb/tb_full_system.v` as a simulation source and set it as the top simulation module.
4. Run Behavioral Simulation (Vivado 2025.2, XSim). The testbench uses a 10 ns clock (100 MHz) and prints pass/fail results in the Tcl console.

The design files contain no timescale directive, so Vivado shows the benign warning XSIM 43-4099.

## Verification

The full-system testbench is a directed, self-checking testbench with 10 scenarios: the three operating modes, NMI response, force-mode override, recovery, multi-core comparison, SUB, MUL and a rapid mode-change stress test.

## Future work

- Post-synthesis power analysis on FPGA hardware
- Integration with a real ECG front-end (AD8232 or ADS1292)
- Multi-lead ECG support
- ASIC tape-out

## Authors

- Rahul Aravind A R
- Pranav Krishna M (simulation-based evaluation and Verilog testbench development)
- Supreet Narayan V

Full details are in [`docs/FINAL_REPORT.pdf`](docs/FINAL_REPORT.pdf).
