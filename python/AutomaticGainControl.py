<<<<<<< HEAD
# =================================================================================================================
# File             : AutomaticGainControl.py
# Author           : Omar Wael Ahmed
# Created          : 2026-01-26
# Acknowledgement  : Eng. Ahmed Ibrahim (for Mitchell's Log Approximation and Leading One Detector algorithms) 
#                
# A Python implementation for the Automatic Gain Control (AGC) module of a Bluetooth Low Energy (BLE) 6.0 receiver.
# =================================================================================================================

import numpy as np
import matplotlib.pyplot as plt

def generatePreambleGFSKSignal(AMPLITUDE, F_IF = 2e6, F_DEV = 250e3, FS = 8e6, BIT_RATE = 1e6, ADC_RESOLUTION = 10):

    ADC_MAX_OUTPUT_VALUE = 2**(ADC_RESOLUTION - 1) - 1
    PREAMBLE_DURATION = 8 * (1/BIT_RATE) # Duration of the 8 bits constiuting the preamble
    samplesPerBit = int(FS / BIT_RATE)

    # t-axis with number of points equal to the number of samples of the 8-bit preamble, depending on FS and BIT_RATE 
    # (e.g. 64 points @ FS = 8 MSPS and BIT_RATE = 1 Mbps)
    t_axis = np.linspace(0, PREAMBLE_DURATION, int(FS * PREAMBLE_DURATION), endpoint=False)

    preambleBitsCount = int(len(t_axis) / samplesPerBit) # A fancy way to declare the length of the preamble :)
    preambleBits = np.tile([1, 0], preambleBitsCount // 2 + 1)[:preambleBitsCount] # 10101010
    # preambleBits = np.tile([0, 1], bitsCount // 2 + 1)[:bitsCount] # 01010101

    # Repeats each bit by samplesPerBit so that we generate an actual 
    # digital waveform (with correct number of samples) for the incoming preamble bit stream.
    preambleBitStream = np.repeat(preambleBits, samplesPerBit)[:len(t_axis)] 

    # The (preambleBitStream * 2 - 1) expression maps 0s of the preambleBitStream to -1 but keeps the 1s as is to 
    # generate a NRZ version of the bit stream, which is then used to calculate the frequency deviation properly.
    RxSignalFrequencyValues = F_IF + ((preambleBitStream * 2 - 1) * F_DEV) 

    # φ(t) = 2π ∫ f(t) dt, so in discrete time domain, we evaluate  φ[n] = 2π * Σ f[k] * T_n,
    # where the sigma variable k ranges from 0 to n, and T_n is the sample period = 1/FS.
    RxSignalPhaseValues = 2 * np.pi * np.cumsum(RxSignalFrequencyValues) * (1/FS)

    # x(t) = A * cos(φ(t)) + j * A * sin(φ(t))
    # Define the I and Q components of x(t) and feed them to the ADC.
    I = (AMPLITUDE * np.cos(RxSignalPhaseValues) * ADC_MAX_OUTPUT_VALUE).astype(np.int16)
    Q = (AMPLITUDE * np.sin(RxSignalPhaseValues) * ADC_MAX_OUTPUT_VALUE).astype(np.int16)
    
    # The timestamps may be useful for plotting the received signal.
    return I, Q, t_axis

def hw_lod(x):
    """
    Hardware-equivalent Leading One Detector.
    Credits: Eng. Ahmed Ibrahim
    """
    if x <= 0: return 0
    return x.bit_length() - 1

def hw_mitchell_db(amp_int):
    """
    Mitchell's Log Approximation (Fixed-Point Q8.8).
    Credits: Eng. Ahmed Ibrahim
    """
    if amp_int <= 0: return -100 * 256 # Return very low dB (ideally -∞) for non-positive inputs
    k = hw_lod(amp_int)
    if k >= 10:
        m_fixed = (amp_int >> (k - 10)) - 1024 # Scale down to fit Q10 format, then find the difference
    else:
        m_fixed = (amp_int << (10 - k)) - 1024 # Scale up to fit Q10 format, then find the difference
    log2_q10 = (k << 10) + m_fixed # Q10 means log_2(10) has been scaled up 10 bits to give more resolution to the result
    # Constant 1541 = 20*log10(2)*2^8
    return (log2_q10 * 1541) >> 10 # Scale 10 bits down to convert from Q8.18 to Q8.8, because our bit width is 16 bits only.

def estimateRSSI(I,Q, ALPHA_SHIFT = 3, ADC_RESOLUTION = 10):
    ADC_MAX_OUTPUT_VALUE = 2**(ADC_RESOLUTION - 1) - 1
    samplesCount = len(I) # or len(Q)

    # Inflate a vector to hold the RSSI values (floating-point dBFS values)
    RSSI_Values = np.zeros(samplesCount)

    # Inflate another vecror to hold the raw RSSI values (fixed-point Q8.8 dB values)
    # This would be useful for the Verilog testbench comparison later on.
    RSSI_ValuesRaw = np.zeros(samplesCount)

    # Initializing the accumulator for the IIR filter used to implement
    # the Exponentially Weighted Moving Average (EWMA) filter. See below for more details.
    EMWA_FilterAccumulator = 0

    # The reference for the dB values calculated below. We use it to normalize
    # the output dB values to dBFS.
    FULL_SCALE_DB_Q8 = hw_mitchell_db(ADC_MAX_OUTPUT_VALUE)

    for i in range(samplesCount): # Consider adding LATENCY cycles if needed
        # Fetch a sample from the I and Q vectors, as long as the loop is within the number of samples, else assign 0
        iSample = int(I[i]) if i < samplesCount else 0
        qSample = int(Q[i]) if i < samplesCount else 0
        
        # Estimating the magnitude of the complex sample (sqrt(I^2 + Q^2)) using cheaper hardware resources))
        absoluteI, absoluteQ = abs(iSample), abs(qSample)
        maxValue, minValue = max(absoluteI, absoluteQ), min(absoluteI, absoluteQ)
        magnitudeEstimate = (maxValue - (maxValue >> 5)) + ((minValue >> 2) + (minValue >> 3) + (minValue >> 5))
        
        # Estimate the dB value (20log_10(magnitude)) by the means of Mitchell's algo.
        RSSI = hw_mitchell_db(magnitudeEstimate)

        # Apply the EMWA filter to smooth the calculated dB values over time.
        # Exponentially Weighted Moving Average (EWMA) filter's equation:  y[n]=(1−λ)⋅y[n−1]+λ⋅x[n] 
        # where λ=1/2^ALPHA_SHIFT, which determines the "smoothness" of the filter.
        # However, to avoid multiplications, we use an approach based on shifting: 
        # EMWA_FilterAccumulator = EMWA_FilterAccumulator + RSSI - (EMWA_FilterAccumulator >> ALPHA_SHIFT)
        filterLeak = EMWA_FilterAccumulator >> ALPHA_SHIFT 
        EMWA_FilterAccumulator = EMWA_FilterAccumulator + RSSI - filterLeak

        RSSI_Filtered = EMWA_FilterAccumulator >> ALPHA_SHIFT

        RSSI_ValuesRaw[i] = RSSI_Filtered
        
        RSSI_Values[i] = (RSSI_Filtered - FULL_SCALE_DB_Q8) / 256.0

    return RSSI_Values, RSSI_ValuesRaw
def saveControlledRSSIToFile(I, Q, RSSI_values, RSSI_SAMPLE_INDEX, controlLine, filename="C:\\Users\\omarw\\OneDrive\\Documents\\CND\\BTLE6.0\\python\\AGC_Output"):
    with open(filename+f"_{RSSI_SAMPLE_INDEX}.txt", "w") as file:
        file.write("I_sample\tQ_sample\tRSSI_dB\t\t\tAGC_control\n")
        for i in range(len(RSSI_values)):
            file.write(f"{I[i]}\t\t{Q[i]}\t\t{RSSI_values[i]}\t\t{controlLine[i]}\n")


def plotControlledRSSI(amplitude, RSSI_SAMPLE_INDEX, trueRSSI, t_axis, RSSI_Values, newRxGainLevel, tuningSteps, controlLine):
    fig, axes = plt.subplots(2, 1, figsize=(15, 15), sharex=True)
    axes[0].plot(t_axis[:RSSI_SAMPLE_INDEX+1]*1e6, RSSI_Values[:RSSI_SAMPLE_INDEX+1], color='green', lw=2, label='RSSI @ Rx Gain = 104 dB (max)')
    axes[0].plot(t_axis[RSSI_SAMPLE_INDEX:]*1e6, RSSI_Values[RSSI_SAMPLE_INDEX:], color='red', lw=2, label="RSSI @ Rx Gain = {} dB".format(newRxGainLevel))
    axes[0].plot(t_axis[:RSSI_SAMPLE_INDEX+1]*1e6, trueRSSI[:RSSI_SAMPLE_INDEX+1], color='lime', ls='--', label=f"True RSSI @ Rx Gain = 104 dB (max) [{round(trueRSSI[0],3)} dB]")
    axes[0].plot(t_axis[RSSI_SAMPLE_INDEX:]*1e6, trueRSSI[RSSI_SAMPLE_INDEX:], color='crimson', ls='--', label=f"True RSSI @ Rx Gain = {newRxGainLevel} dB [{round(trueRSSI[RSSI_SAMPLE_INDEX + 1],3)} dB]")
    axes[0].set_ylabel("dBFS")
    axes[0].set_xlabel("Time (µs)")
    axes[0].set_title(r"RSSI Output for $x(t) = {} * exp(j*φ(t))$".format(amplitude)+f", with Automatic Gain Control (AGC) Enabled")
    if (RSSI_SAMPLE_INDEX >= 8):
        firstBitRSSIError = abs(RSSI_Values[8] - trueRSSI[8])
        axes[0].annotate(f'RSSI Error: {round(firstBitRSSIError,3)} dB', xy=(t_axis[8]*1e6, RSSI_Values[8]),
                    xytext=(t_axis[8]*1e6 - 0.5, RSSI_Values[8] - 5),
                    arrowprops=dict(facecolor='black', arrowstyle='->'))
    if (RSSI_SAMPLE_INDEX >= 16):
        secondBitRSSIError = abs(RSSI_Values[16] - trueRSSI[16])
        axes[0].annotate(f'RSSI Error: {round(secondBitRSSIError,3)} dB', xy=(t_axis[16]*1e6, RSSI_Values[16]),
                    xytext=(t_axis[16]*1e6 - 0.5, RSSI_Values[16] - 5),
                    arrowprops=dict(facecolor='black', arrowstyle='->'))

    if (RSSI_SAMPLE_INDEX >= 24):
        thirdBitRSSIError = abs(RSSI_Values[24] - trueRSSI[24])
        axes[0].annotate(f'RSSI Error: {round(thirdBitRSSIError,3)} dB', xy=(t_axis[24]*1e6, RSSI_Values[24]),
                    xytext=(t_axis[24]*1e6 - 0.5, RSSI_Values[24] - 5),
                    arrowprops=dict(facecolor='black', arrowstyle='->'))
    if (RSSI_SAMPLE_INDEX >= 32):
        fourthBitRSSIError = abs(RSSI_Values[32] - trueRSSI[32])
        axes[0].annotate(f'RSSI Error: {round(fourthBitRSSIError,3)} dB', xy=(t_axis[32]*1e6, RSSI_Values[32]),
                    xytext=(t_axis[32]*1e6 - 0.5, RSSI_Values[32] - 5),
                    arrowprops=dict(facecolor='black', arrowstyle='->'))
    axes[0].grid(True)
    axes[0].legend()

    axes[1].step(t_axis*1e6, controlLine, where='post', color='green', lw=2)
    axes[1].set_ylabel("Control Word (hex)")
    axes[1].set_xlabel("Time (µs)")
    axes[1].set_title("AGC Control Line")
    unique_values = np.unique(controlLine)
    axes[1].set_yticks(unique_values)
    axes[1].set_yticklabels([f'0x{int(val):02X}' for val in unique_values])

    axes[1].annotate(f'AGC Transition: {tuningSteps} steps from 0x1F', 
                    xy=(t_axis[RSSI_SAMPLE_INDEX]*1e6, controlLine[RSSI_SAMPLE_INDEX]),
                    xytext=(t_axis[RSSI_SAMPLE_INDEX]*1e6 + 0.3, controlLine[RSSI_SAMPLE_INDEX]+0.5),
                    arrowprops=dict(facecolor='black', arrowstyle='<|-|>'))
    
    axes[1].grid(True)
   
    plt.show()


RX_GAINS = [104, 99, 94, 88, 83, 78, 73, 67, 62, 57, 52, 47, 41, 36, 31, 26, 21] # in dB
CONTROL_WORDS = [0x1F, 0x1E, 0x1D, 0x16, 0x15, 0x14, 0x13, 0x2D, 0x2C, 0x2B, 0x2A, 0x4A, 0x43, 0x42, 0x62, 0x61, 0x60]

# Desired RSSI level of the received signal
SET_POINT_DBFS = -9

# The index of the RSSI sample with a reliable estimate that is close enough to the true RSSI value of x(t)
RSSI_SAMPLE_INDEX = 32

# Initialize a counter to keep track of the number of RX gain adjustments necessary to reach the desired RSSI set point.
TUNING_STEPS = 0

# Capture the preamble's GFSK signal x(t) = A * cos(φ(t)) + j * A * sin(φ(t)) and sample its I and Q components
AMPLITUDE = 0.8
I, Q, t_axis = generatePreambleGFSKSignal(AMPLITUDE=AMPLITUDE)

# Estimate the RSSI values for the captured signal
RSSI_Values, RSSI_ValuesRaw = estimateRSSI(I, Q)

# Calculate the true RSSI value of the received signal x(t) in a vector of the same length for easier plotting later.
trueRSSI = np.full_like(RSSI_Values, 20*np.log10(AMPLITUDE))

gainError = RSSI_Values[RSSI_SAMPLE_INDEX] - SET_POINT_DBFS

while (gainError > 0):
    gainError -= 5
    TUNING_STEPS += 1

# The gain level that Rx needs to switch to in order to bring the received signal in the neighborood of the SET_POINT_DBFS
newRxGainLevel = RX_GAINS[TUNING_STEPS]

# The control word that would set RX's gain to newRxGainLevel
controlWord = CONTROL_WORDS[TUNING_STEPS]

# The drop in the received signal's RSSI due to the decrease of RX's gain
deltaRxGain = RX_GAINS[0] - newRxGainLevel

# Inflate a vector with the deltaRxGain values that correspond to the RSSI samples to be affected by the RX gain change.
deltaRxGainVector = np.full(len(RSSI_Values), deltaRxGain)

# Since no change should take place in RSSI values before the gain change takes effect, we 
# zero out the first RSSI_SAMPLE_INDEX values of the deltaRxGainVector.
deltaRxGainVector[:RSSI_SAMPLE_INDEX] = 0 

# Adjust the RSSI values of x(t) to reflect the change in RX gain.
RSSI_Values -= deltaRxGainVector
trueRSSI -= deltaRxGainVector

# For the control line, we inflate a vector for the initial control word, then adjust it with
# the new control word at the correct time stamp so that we end up with a vector that can be plotted vs time.
controlLineVector = np.full(len(t_axis), CONTROL_WORDS[0])
controlLineVector[RSSI_SAMPLE_INDEX:] = controlWord

saveControlledRSSIToFile(I, Q, RSSI_Values, RSSI_SAMPLE_INDEX, controlLineVector)
=======
# =================================================================================================================
# File             : AutomaticGainControl.py
# Author           : Omar Wael Ahmed
# Created          : 2026-01-26
# Acknowledgement  : Eng. Ahmed Ibrahim (for Mitchell's Log Approximation and Leading One Detector algorithms) 
#                
# A Python implementation for the Automatic Gain Control (AGC) module of a Bluetooth Low Energy (BLE) 6.0 receiver.
# =================================================================================================================

import numpy as np
import matplotlib.pyplot as plt

def generatePreambleGFSKSignal(AMPLITUDE, F_IF = 2e6, F_DEV = 250e3, FS = 8e6, BIT_RATE = 1e6, ADC_RESOLUTION = 10):

    ADC_MAX_OUTPUT_VALUE = 2**(ADC_RESOLUTION - 1) - 1
    PREAMBLE_DURATION = 8 * (1/BIT_RATE) # Duration of the 8 bits constiuting the preamble
    samplesPerBit = int(FS / BIT_RATE)

    # t-axis with number of points equal to the number of samples of the 8-bit preamble, depending on FS and BIT_RATE 
    # (e.g. 64 points @ FS = 8 MSPS and BIT_RATE = 1 Mbps)
    t_axis = np.linspace(0, PREAMBLE_DURATION, int(FS * PREAMBLE_DURATION), endpoint=False)

    preambleBitsCount = int(len(t_axis) / samplesPerBit) # A fancy way to declare the length of the preamble :)
    preambleBits = np.tile([1, 0], preambleBitsCount // 2 + 1)[:preambleBitsCount] # 10101010
    # preambleBits = np.tile([0, 1], bitsCount // 2 + 1)[:bitsCount] # 01010101

    # Repeats each bit by samplesPerBit so that we generate an actual 
    # digital waveform (with correct number of samples) for the incoming preamble bit stream.
    preambleBitStream = np.repeat(preambleBits, samplesPerBit)[:len(t_axis)] 

    # The (preambleBitStream * 2 - 1) expression maps 0s of the preambleBitStream to -1 but keeps the 1s as is to 
    # generate a NRZ version of the bit stream, which is then used to calculate the frequency deviation properly.
    RxSignalFrequencyValues = F_IF + ((preambleBitStream * 2 - 1) * F_DEV) 

    # φ(t) = 2π ∫ f(t) dt, so in discrete time domain, we evaluate  φ[n] = 2π * Σ f[k] * T_n,
    # where the sigma variable k ranges from 0 to n, and T_n is the sample period = 1/FS.
    RxSignalPhaseValues = 2 * np.pi * np.cumsum(RxSignalFrequencyValues) * (1/FS)

    # x(t) = A * cos(φ(t)) + j * A * sin(φ(t))
    # Define the I and Q components of x(t) and feed them to the ADC.
    I = (AMPLITUDE * np.cos(RxSignalPhaseValues) * ADC_MAX_OUTPUT_VALUE).astype(np.int16)
    Q = (AMPLITUDE * np.sin(RxSignalPhaseValues) * ADC_MAX_OUTPUT_VALUE).astype(np.int16)
    
    # The timestamps may be useful for plotting the received signal.
    return I, Q, t_axis

def hw_lod(x):
    """
    Hardware-equivalent Leading One Detector.
    Credits: Eng. Ahmed Ibrahim
    """
    if x <= 0: return 0
    return x.bit_length() - 1

def hw_mitchell_db(amp_int):
    """
    Mitchell's Log Approximation (Fixed-Point Q8.8).
    Credits: Eng. Ahmed Ibrahim
    """
    if amp_int <= 0: return -100 * 256 # Return very low dB (ideally -∞) for non-positive inputs
    k = hw_lod(amp_int)
    if k >= 10:
        m_fixed = (amp_int >> (k - 10)) - 1024 # Scale down to fit Q10 format, then find the difference
    else:
        m_fixed = (amp_int << (10 - k)) - 1024 # Scale up to fit Q10 format, then find the difference
    log2_q10 = (k << 10) + m_fixed # Q10 means log_2(10) has been scaled up 10 bits to give more resolution to the result
    # Constant 1541 = 20*log10(2)*2^8
    return (log2_q10 * 1541) >> 10 # Scale 10 bits down to convert from Q8.18 to Q8.8, because our bit width is 16 bits only.

def estimateRSSI(I,Q, ALPHA_SHIFT = 3, ADC_RESOLUTION = 10):
    ADC_MAX_OUTPUT_VALUE = 2**(ADC_RESOLUTION - 1) - 1
    samplesCount = len(I) # or len(Q)

    # Inflate a vector to hold the RSSI values (floating-point dBFS values)
    RSSI_Values = np.zeros(samplesCount)

    # Inflate another vecror to hold the raw RSSI values (fixed-point Q8.8 dB values)
    # This would be useful for the Verilog testbench comparison later on.
    RSSI_ValuesRaw = np.zeros(samplesCount)

    # Initializing the accumulator for the IIR filter used to implement
    # the Exponentially Weighted Moving Average (EWMA) filter. See below for more details.
    EMWA_FilterAccumulator = 0

    # The reference for the dB values calculated below. We use it to normalize
    # the output dB values to dBFS.
    FULL_SCALE_DB_Q8 = hw_mitchell_db(ADC_MAX_OUTPUT_VALUE)

    for i in range(samplesCount): # Consider adding LATENCY cycles if needed
        # Fetch a sample from the I and Q vectors, as long as the loop is within the number of samples, else assign 0
        iSample = int(I[i]) if i < samplesCount else 0
        qSample = int(Q[i]) if i < samplesCount else 0
        
        # Estimating the magnitude of the complex sample (sqrt(I^2 + Q^2)) using cheaper hardware resources))
        absoluteI, absoluteQ = abs(iSample), abs(qSample)
        maxValue, minValue = max(absoluteI, absoluteQ), min(absoluteI, absoluteQ)
        magnitudeEstimate = (maxValue - (maxValue >> 5)) + ((minValue >> 2) + (minValue >> 3) + (minValue >> 5))
        
        # Estimate the dB value (20log_10(magnitude)) by the means of Mitchell's algo.
        RSSI = hw_mitchell_db(magnitudeEstimate)

        # Apply the EMWA filter to smooth the calculated dB values over time.
        # Exponentially Weighted Moving Average (EWMA) filter's equation:  y[n]=(1−λ)⋅y[n−1]+λ⋅x[n] 
        # where λ=1/2^ALPHA_SHIFT, which determines the "smoothness" of the filter.
        # However, to avoid multiplications, we use an approach based on shifting: 
        # EMWA_FilterAccumulator = EMWA_FilterAccumulator + RSSI - (EMWA_FilterAccumulator >> ALPHA_SHIFT)
        filterLeak = EMWA_FilterAccumulator >> ALPHA_SHIFT 
        EMWA_FilterAccumulator = EMWA_FilterAccumulator + RSSI - filterLeak

        RSSI_Filtered = EMWA_FilterAccumulator >> ALPHA_SHIFT

        RSSI_ValuesRaw[i] = RSSI_Filtered
        
        RSSI_Values[i] = (RSSI_Filtered - FULL_SCALE_DB_Q8) / 256.0

    return RSSI_Values, RSSI_ValuesRaw
def saveControlledRSSIToFile(I, Q, RSSI_values, RSSI_SAMPLE_INDEX, controlLine, filename="C:\\Users\\omarw\\OneDrive\\Documents\\CND\\BTLE6.0\\python\\AGC_Output"):
    with open(filename+f"_{RSSI_SAMPLE_INDEX}.txt", "w") as file:
        file.write("I_sample\tQ_sample\tRSSI_dB\t\t\tAGC_control\n")
        for i in range(len(RSSI_values)):
            file.write(f"{I[i]}\t\t{Q[i]}\t\t{RSSI_values[i]}\t\t{controlLine[i]}\n")


def plotControlledRSSI(amplitude, RSSI_SAMPLE_INDEX, trueRSSI, t_axis, RSSI_Values, newRxGainLevel, tuningSteps, controlLine):
    fig, axes = plt.subplots(2, 1, figsize=(15, 15), sharex=True)
    axes[0].plot(t_axis[:RSSI_SAMPLE_INDEX+1]*1e6, RSSI_Values[:RSSI_SAMPLE_INDEX+1], color='green', lw=2, label='RSSI @ Rx Gain = 104 dB (max)')
    axes[0].plot(t_axis[RSSI_SAMPLE_INDEX:]*1e6, RSSI_Values[RSSI_SAMPLE_INDEX:], color='red', lw=2, label="RSSI @ Rx Gain = {} dB".format(newRxGainLevel))
    axes[0].plot(t_axis[:RSSI_SAMPLE_INDEX+1]*1e6, trueRSSI[:RSSI_SAMPLE_INDEX+1], color='lime', ls='--', label=f"True RSSI @ Rx Gain = 104 dB (max) [{round(trueRSSI[0],3)} dB]")
    axes[0].plot(t_axis[RSSI_SAMPLE_INDEX:]*1e6, trueRSSI[RSSI_SAMPLE_INDEX:], color='crimson', ls='--', label=f"True RSSI @ Rx Gain = {newRxGainLevel} dB [{round(trueRSSI[RSSI_SAMPLE_INDEX + 1],3)} dB]")
    axes[0].set_ylabel("dBFS")
    axes[0].set_xlabel("Time (µs)")
    axes[0].set_title(r"RSSI Output for $x(t) = {} * exp(j*φ(t))$".format(amplitude)+f", with Automatic Gain Control (AGC) Enabled")
    if (RSSI_SAMPLE_INDEX >= 8):
        firstBitRSSIError = abs(RSSI_Values[8] - trueRSSI[8])
        axes[0].annotate(f'RSSI Error: {round(firstBitRSSIError,3)} dB', xy=(t_axis[8]*1e6, RSSI_Values[8]),
                    xytext=(t_axis[8]*1e6 - 0.5, RSSI_Values[8] - 5),
                    arrowprops=dict(facecolor='black', arrowstyle='->'))
    if (RSSI_SAMPLE_INDEX >= 16):
        secondBitRSSIError = abs(RSSI_Values[16] - trueRSSI[16])
        axes[0].annotate(f'RSSI Error: {round(secondBitRSSIError,3)} dB', xy=(t_axis[16]*1e6, RSSI_Values[16]),
                    xytext=(t_axis[16]*1e6 - 0.5, RSSI_Values[16] - 5),
                    arrowprops=dict(facecolor='black', arrowstyle='->'))

    if (RSSI_SAMPLE_INDEX >= 24):
        thirdBitRSSIError = abs(RSSI_Values[24] - trueRSSI[24])
        axes[0].annotate(f'RSSI Error: {round(thirdBitRSSIError,3)} dB', xy=(t_axis[24]*1e6, RSSI_Values[24]),
                    xytext=(t_axis[24]*1e6 - 0.5, RSSI_Values[24] - 5),
                    arrowprops=dict(facecolor='black', arrowstyle='->'))
    if (RSSI_SAMPLE_INDEX >= 32):
        fourthBitRSSIError = abs(RSSI_Values[32] - trueRSSI[32])
        axes[0].annotate(f'RSSI Error: {round(fourthBitRSSIError,3)} dB', xy=(t_axis[32]*1e6, RSSI_Values[32]),
                    xytext=(t_axis[32]*1e6 - 0.5, RSSI_Values[32] - 5),
                    arrowprops=dict(facecolor='black', arrowstyle='->'))
    axes[0].grid(True)
    axes[0].legend()

    axes[1].step(t_axis*1e6, controlLine, where='post', color='green', lw=2)
    axes[1].set_ylabel("Control Word (hex)")
    axes[1].set_xlabel("Time (µs)")
    axes[1].set_title("AGC Control Line")
    unique_values = np.unique(controlLine)
    axes[1].set_yticks(unique_values)
    axes[1].set_yticklabels([f'0x{int(val):02X}' for val in unique_values])

    axes[1].annotate(f'AGC Transition: {tuningSteps} steps from 0x1F', 
                    xy=(t_axis[RSSI_SAMPLE_INDEX]*1e6, controlLine[RSSI_SAMPLE_INDEX]),
                    xytext=(t_axis[RSSI_SAMPLE_INDEX]*1e6 + 0.3, controlLine[RSSI_SAMPLE_INDEX]+0.5),
                    arrowprops=dict(facecolor='black', arrowstyle='<|-|>'))
    
    axes[1].grid(True)
   
    plt.show()


RX_GAINS = [104, 99, 94, 88, 83, 78, 73, 67, 62, 57, 52, 47, 41, 36, 31, 26, 21] # in dB
CONTROL_WORDS = [0x1F, 0x1E, 0x1D, 0x16, 0x15, 0x14, 0x13, 0x2D, 0x2C, 0x2B, 0x2A, 0x4A, 0x43, 0x42, 0x62, 0x61, 0x60]

# Desired RSSI level of the received signal
SET_POINT_DBFS = -9

# The index of the RSSI sample with a reliable estimate that is close enough to the true RSSI value of x(t)
RSSI_SAMPLE_INDEX = 32

# Initialize a counter to keep track of the number of RX gain adjustments necessary to reach the desired RSSI set point.
TUNING_STEPS = 0

# Capture the preamble's GFSK signal x(t) = A * cos(φ(t)) + j * A * sin(φ(t)) and sample its I and Q components
AMPLITUDE = 0.1
I, Q, t_axis = generatePreambleGFSKSignal(AMPLITUDE=AMPLITUDE)

# Estimate the RSSI values for the captured signal
RSSI_Values, RSSI_ValuesRaw = estimateRSSI(I, Q)

# Calculate the true RSSI value of the received signal x(t) in a vector of the same length for easier plotting later.
trueRSSI = np.full_like(RSSI_Values, 20*np.log10(AMPLITUDE))

gainError = RSSI_Values[RSSI_SAMPLE_INDEX] - SET_POINT_DBFS

while (gainError > 0):
    gainError -= 5
    TUNING_STEPS += 1

# The gain level that Rx needs to switch to in order to bring the received signal in the neighborood of the SET_POINT_DBFS
newRxGainLevel = RX_GAINS[TUNING_STEPS]

# The control word that would set RX's gain to newRxGainLevel
controlWord = CONTROL_WORDS[TUNING_STEPS]

# The drop in the received signal's RSSI due to the decrease of RX's gain
deltaRxGain = RX_GAINS[0] - newRxGainLevel

# Inflate a vector with the deltaRxGain values that correspond to the RSSI samples to be affected by the RX gain change.
deltaRxGainVector = np.full(len(RSSI_Values), deltaRxGain)

# Since no change should take place in RSSI values before the gain change takes effect, we 
# zero out the first RSSI_SAMPLE_INDEX values of the deltaRxGainVector.
deltaRxGainVector[:RSSI_SAMPLE_INDEX] = 0 

# Adjust the RSSI values of x(t) to reflect the change in RX gain.
RSSI_Values -= deltaRxGainVector
trueRSSI -= deltaRxGainVector

# For the control line, we inflate a vector for the initial control word, then adjust it with
# the new control word at the correct time stamp so that we end up with a vector that can be plotted vs time.
controlLineVector = np.full(len(t_axis), CONTROL_WORDS[0])
controlLineVector[RSSI_SAMPLE_INDEX:] = controlWord

saveControlledRSSIToFile(I, Q, RSSI_Values, RSSI_SAMPLE_INDEX, controlLineVector)
>>>>>>> af868f6f8288187823a211e90a8e5617f22c4e82
plotControlledRSSI(AMPLITUDE, RSSI_SAMPLE_INDEX, trueRSSI, t_axis, RSSI_Values, newRxGainLevel, TUNING_STEPS, controlLineVector)