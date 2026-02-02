# Automatic Gain Control (AGC) Module for BLE 6.0 Receiver

## Overview

This directory contains the Verilog implementation of an Automatic Gain Control (AGC) module for a Bluetooth Low Energy (BLE) 6.0 receiver. The AGC module automatically adjusts the receiver gain to maintain optimal signal levels and prevent saturation.

## Architecture

The AGC system consists of the following modules:

1. **leading_one_detector.v** - Finds the position of the most significant '1' bit
2. **mitchell_log.v** - Computes 20*log10(x) using Mitchell's logarithm approximation
3. **rssi_estimator.v** - Estimates Received Signal Strength Indicator (RSSI) from I/Q samples
4. **agc_controller.v** - Monitors RSSI and determines optimal gain control word
5. **agc.v** - Top-level module that integrates all components
6. **agc_tb.v** - Testbench for verification

## Module Descriptions

### leading_one_detector.v

Hardware-equivalent Leading One Detector (LOD) that finds the position of the MSB in a number. This is used as a building block for Mitchell's logarithm approximation.

**Parameters:**
- `DATA_WIDTH`: Bit width of input data (default: 16)

**Ports:**
- `data_in`: Input data to find leading one
- `lod_out`: Position of leading one (0 to DATA_WIDTH-1)

### mitchell_log.v

Implements Mitchell's logarithm approximation algorithm to compute 20*log10(x) in fixed-point Q8.8 format. This avoids expensive floating-point operations in hardware.

**Parameters:**
- `INPUT_WIDTH`: Bit width of input magnitude (default: 16)
- `OUTPUT_WIDTH`: Bit width of output in Q8.8 format (default: 16)
- `LOD_WIDTH`: Bit width for LOD output (default: 4)

**Ports:**
- `clk`: Clock signal
- `rst_n`: Active-low reset
- `magnitude_in`: Input magnitude (unsigned)
- `data_valid`: Input data valid signal
- `db_out`: Output in Q8.8 format (dB)
- `db_valid`: Output valid signal

**Fixed-Point Format:**
- Output is in Q8.8 format: 8 integer bits, 8 fractional bits
- Range: approximately -100*256 to +100*256 (representing -100 dB to +100 dB)

### rssi_estimator.v

Estimates RSSI from I/Q samples using:
1. Magnitude approximation: `sqrt(I² + Q²) ≈ (max - max/32) + (min/4 + min/8 + min/32)`
2. Mitchell's logarithm to convert magnitude to dB
3. EWMA (Exponentially Weighted Moving Average) filter for smoothing

**Parameters:**
- `I_Q_WIDTH`: Bit width of I and Q samples (default: 10)
- `MAGNITUDE_WIDTH`: Bit width for magnitude estimate (default: 16)
- `RSSI_WIDTH`: Bit width for RSSI output (default: 16, Q8.8 format)
- `ALPHA_SHIFT`: EWMA filter parameter, λ = 1/2^ALPHA_SHIFT (default: 3)
- `ADC_RESOLUTION`: ADC resolution in bits (default: 10)

**Ports:**
- `clk`: Clock signal
- `rst_n`: Active-low reset
- `i_sample`: I component sample
- `q_sample`: Q component sample
- `data_valid`: Input data valid signal
- `rssi_dbfs`: RSSI output in dBFS (Q8.8 format)
- `rssi_raw`: RSSI output in raw Q8.8 format (before dBFS conversion)
- `rssi_valid`: RSSI output valid signal

### agc_controller.v

Monitors RSSI values and determines the optimal gain control word. The decision is made at a specific sample index (default: 32) after RSSI has stabilized.

**Parameters:**
- `RSSI_WIDTH`: Bit width of RSSI input (default: 16, Q8.8 format)
- `CONTROL_WORD_WIDTH`: Bit width of control word output (default: 8)
- `RSSI_SAMPLE_INDEX`: Sample index at which to make AGC decision (default: 32)
- `SET_POINT_DBFS`: Desired RSSI level in Q8.8 format (default: -9 dBFS = -9*256)
- `NUM_GAIN_LEVELS`: Number of gain levels available (default: 17)

**Ports:**
- `clk`: Clock signal
- `rst_n`: Active-low reset
- `rssi_dbfs`: RSSI input in dBFS (Q8.8 format)
- `rssi_valid`: RSSI valid signal
- `control_word`: Control word output
- `control_word_valid`: Control word valid signal
- `tuning_steps`: Number of tuning steps applied (for debugging)

**Gain Levels:**
The module supports 17 gain levels from 104 dB (maximum) down to 21 dB, with corresponding control words:
- 104 dB → 0x1F
- 99 dB → 0x1E
- 94 dB → 0x1D
- ... (see code for full list)
- 21 dB → 0x60

### agc.v

Top-level module that integrates the RSSI estimator and AGC controller.

**Parameters:**
- `I_Q_WIDTH`: Bit width of I and Q samples (default: 10)
- `RSSI_WIDTH`: Bit width for RSSI (default: 16, Q8.8 format)
- `CONTROL_WORD_WIDTH`: Bit width of control word output (default: 8)
- `ALPHA_SHIFT`: EWMA filter parameter (default: 3)
- `ADC_RESOLUTION`: ADC resolution in bits (default: 10)
- `RSSI_SAMPLE_INDEX`: Sample index for AGC decision (default: 32)
- `SET_POINT_DBFS`: Desired RSSI level (default: -9*256, Q8.8 format)
- `NUM_GAIN_LEVELS`: Number of gain levels (default: 17)

**Ports:**
- `clk`: Clock signal
- `rst_n`: Active-low reset
- `i_sample`: I component sample
- `q_sample`: Q component sample
- `data_valid`: Input data valid signal
- `rssi_dbfs`: RSSI output in dBFS
- `rssi_raw`: RSSI output in raw Q8.8 format
- `rssi_valid`: RSSI output valid signal
- `control_word`: Gain control word output
- `control_word_valid`: Control word valid signal
- `tuning_steps`: Number of tuning steps (for debugging)

## Python to Verilog Mapping

### Leading One Detector
- **Python**: `x.bit_length() - 1`
- **Verilog**: Priority encoder finding MSB position

### Mitchell's Log Approximation
- **Python**: `hw_mitchell_db(amp_int)` function
- **Verilog**: Multi-stage pipeline implementing:
  1. Leading one detection
  2. Mantissa calculation with normalization
  3. log2(x) computation in Q10 format
  4. Conversion to dB using constant 1541 (20*log10(2)*2^8)

### RSSI Estimation
- **Python**: `estimateRSSI(I, Q, ...)` function
- **Verilog**: Pipeline implementing:
  1. Magnitude approximation (hardware-efficient)
  2. Mitchell's log computation
  3. EWMA filtering
  4. dBFS conversion

### AGC Control Logic
- **Python**: Main script logic comparing RSSI with SET_POINT_DBFS
- **Verilog**: State machine that:
  1. Monitors RSSI samples
  2. Makes decision at RSSI_SAMPLE_INDEX
  3. Calculates tuning steps needed
  4. Outputs appropriate control word

## Testbench

The testbench (`agc_tb.v`) includes the following test scenarios:

1. **Low Amplitude Signal (0.1)**: Tests AGC with weak input signal
2. **Medium Amplitude Signal (0.5)**: Tests moderate signal levels
3. **High Amplitude Signal (0.9)**: Tests strong signal requiring significant gain reduction
4. **Zero Input**: Edge case testing

### Running the Testbench

```bash
# Using ModelSim/QuestaSim
vlog AGC/*.v
vsim agc_tb
run -all

# Using Verilator
verilator --cc --exe --build AGC/*.v agc_tb.v

# Using Icarus Verilog
iverilog -o agc_tb AGC/*.v agc_tb.v
vvp agc_tb
```

## Test Vector Generation

To generate test vectors from the Python model:

1. Run the Python script with different amplitude values
2. Use the `saveControlledRSSIToFile` function to save I/Q samples and RSSI values
3. Compare Verilog simulation results with Python model outputs

Example Python usage:
```python
AMPLITUDE = 0.1
I, Q, t_axis = generatePreambleGFSKSignal(AMPLITUDE=AMPLITUDE)
RSSI_Values, RSSI_ValuesRaw = estimateRSSI(I, Q)
# Save to file for comparison
saveControlledRSSIToFile(I, Q, RSSI_Values, RSSI_SAMPLE_INDEX, controlLineVector)
```

## Timing Considerations

- **Pipeline Latency**: The RSSI estimator has multiple pipeline stages:
  - Magnitude estimation: 1 cycle
  - Mitchell's log: ~4-5 cycles
  - EWMA filter: 1 cycle
  - Total: ~6-7 cycles latency

- **AGC Decision Timing**: 
  - Decision is made at sample index 32 (configurable)
  - Control word is updated once and remains stable
  - This allows RSSI to stabilize before making gain adjustment

## Synthesis Notes

- All modules are synthesizable
- Fixed-point arithmetic is used throughout (no floating-point)
- Pipeline stages are balanced for timing closure
- Resource usage is optimized for hardware efficiency

## Future Enhancements

Potential improvements:
1. Adaptive RSSI_SAMPLE_INDEX based on signal characteristics
2. Hysteresis in gain control to prevent oscillation
3. Multiple set points for different signal conditions
4. Gain adjustment during reception (not just at start)

## Credits

- **Algorithm Design**: Eng. Ahmed Ibrahim (Mitchell's Log Approximation and Leading One Detector)
- **Python Model**: Omar Wael Ahmed
- **Verilog Implementation**: Generated from Python model

## License

[Specify license if applicable]

