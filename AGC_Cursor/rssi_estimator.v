// =================================================================================================================
// File             : rssi_estimator.v
// Description      : RSSI (Received Signal Strength Indicator) Estimator Module
//                    Estimates signal strength using magnitude approximation and Mitchell's log algorithm
//                    Includes EWMA (Exponentially Weighted Moving Average) filter for smoothing
// =================================================================================================================
//
// Python Mapping:
// ---------------
// The Python function estimateRSSI(I, Q, ...) performs:
// 1. Magnitude estimation: sqrt(I^2 + Q^2) ≈ (max - max>>5) + (min>>2 + min>>3 + min>>5)
// 2. RSSI calculation: 20*log10(magnitude) using Mitchell's algorithm
// 3. EWMA filtering: y[n] = y[n-1] + RSSI - (y[n-1] >> ALPHA_SHIFT)
// 4. Normalization to dBFS: RSSI_dBFS = (RSSI_filtered - FULL_SCALE_DB_Q8) / 256.0
//
// Magnitude Approximation:
// ------------------------
// Instead of computing sqrt(I^2 + Q^2) exactly (expensive in hardware), we use:
// magnitude ≈ max(|I|, |Q|) - max(|I|, |Q|)/32 + min(|I|, |Q|)/4 + min(|I|, |Q|)/8 + min(|I|, |Q|)/32
// This is a hardware-efficient approximation that avoids square root and multiplication operations.
//
// EWMA Filter:
// ------------
// Exponentially Weighted Moving Average: y[n] = (1-λ)*y[n-1] + λ*x[n]
// where λ = 1/2^ALPHA_SHIFT
// Hardware implementation: y[n] = y[n-1] + x[n] - (y[n-1] >> ALPHA_SHIFT)
// This avoids multiplication by using bit shifts.
//
// =================================================================================================================

module rssi_estimator #(
    parameter I_Q_WIDTH = 10,           // Bit width of I and Q samples (ADC resolution)
    parameter MAGNITUDE_WIDTH = 16,     // Bit width for magnitude estimate
    parameter RSSI_WIDTH = 16,          // Bit width for RSSI output (Q8.8 format)
    parameter ALPHA_SHIFT = 3,          // EWMA filter parameter (λ = 1/2^ALPHA_SHIFT)
    parameter ADC_RESOLUTION = 10       // ADC resolution in bits
)(
    input wire clk,
    input wire rst_n,
    input wire signed [I_Q_WIDTH-1:0] i_sample,  // I component sample
    input wire signed [I_Q_WIDTH-1:0] q_sample,  // Q component sample
    input wire data_valid,                       // Input data valid signal
    output reg signed [RSSI_WIDTH-1:0] rssi_dbfs, // RSSI output in dBFS (Q8.8 format)
    output reg signed [RSSI_WIDTH-1:0] rssi_raw,   // RSSI output in raw Q8.8 format (before dBFS conversion)
    output reg rssi_valid                          // RSSI output valid signal
);

    // Constants
    localparam ADC_MAX_OUTPUT_VALUE = (1 << (ADC_RESOLUTION - 1)) - 1;  // Maximum ADC output value
    
    // Internal signals for magnitude estimation
    wire [I_Q_WIDTH-1:0] abs_i, abs_q;           // Absolute values of I and Q
    wire [I_Q_WIDTH-1:0] max_val, min_val;       // Maximum and minimum of |I| and |Q|
    reg [MAGNITUDE_WIDTH-1:0] magnitude_estimate; // Estimated magnitude
    
    // Internal signals for Mitchell's log
    wire signed [RSSI_WIDTH-1:0] rssi_unfiltered; // RSSI before EWMA filtering
    wire mitchell_valid;
    
    // EWMA filter accumulator (wider to prevent overflow)
    reg signed [RSSI_WIDTH+ALPHA_SHIFT-1:0] ewma_accumulator;
    reg signed [RSSI_WIDTH-1:0] rssi_filtered;
    
    // Full scale dB value in Q8.8 format (reference for dBFS conversion)
    reg signed [RSSI_WIDTH-1:0] full_scale_db_q8;
    
    // Pipeline registers
    reg signed [I_Q_WIDTH-1:0] i_reg, q_reg;
    reg data_valid_reg;
    
    // Compute absolute values of I and Q
    assign abs_i = (i_sample[I_Q_WIDTH-1]) ? (~i_sample + 1) : i_sample;  // Two's complement absolute
    assign abs_q = (q_sample[I_Q_WIDTH-1]) ? (~q_sample + 1) : q_sample;
    
    // Find maximum and minimum of |I| and |Q|
    assign max_val = (abs_i > abs_q) ? abs_i : abs_q;
    assign min_val = (abs_i > abs_q) ? abs_q : abs_i;
    
    // Pipeline Stage 1: Register I and Q samples
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            i_reg <= {I_Q_WIDTH{1'b0}};
            q_reg <= {I_Q_WIDTH{1'b0}};
            data_valid_reg <= 1'b0;
        end else begin
            i_reg <= i_sample;
            q_reg <= q_sample;
            data_valid_reg <= data_valid;
        end
    end
    
    // Pipeline Stage 2: Compute magnitude estimate
    // Python: magnitudeEstimate = (maxValue - (maxValue >> 5)) + ((minValue >> 2) + (minValue >> 3) + (minValue >> 5))
    // This approximates sqrt(I^2 + Q^2) without expensive operations
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            magnitude_estimate <= {MAGNITUDE_WIDTH{1'b0}};
        end else begin
            // magnitude ≈ max - max/32 + min/4 + min/8 + min/32
            //            = max - (max >> 5) + (min >> 2) + (min >> 3) + (min >> 5)
            magnitude_estimate <= (max_val - (max_val >> 5)) + 
                                 ((min_val >> 2) + (min_val >> 3) + (min_val >> 5));
        end
    end
    
    // Instantiate Mitchell's log module to compute RSSI
    mitchell_log #(
        .INPUT_WIDTH(MAGNITUDE_WIDTH),
        .OUTPUT_WIDTH(RSSI_WIDTH),
        .LOD_WIDTH($clog2(MAGNITUDE_WIDTH))
    ) u_mitchell_log (
        .clk(clk),
        .rst_n(rst_n),
        .magnitude_in(magnitude_estimate),
        .data_valid(data_valid_reg),
        .db_out(rssi_unfiltered),
        .db_valid(mitchell_valid)
    );
    
    // Pipeline Stage 3: Initialize full scale dB value
    // This is computed once using Mitchell's log on ADC_MAX_OUTPUT_VALUE
    // Python: FULL_SCALE_DB_Q8 = hw_mitchell_db(ADC_MAX_OUTPUT_VALUE)
    // For 10-bit ADC: ADC_MAX_OUTPUT_VALUE = 511
    // We'll compute this using a separate instance and latch the result
    reg init_full_scale;
    wire signed [RSSI_WIDTH-1:0] full_scale_db_temp;
    wire full_scale_db_valid;
    
    mitchell_log #(
        .INPUT_WIDTH(MAGNITUDE_WIDTH),
        .OUTPUT_WIDTH(RSSI_WIDTH),
        .LOD_WIDTH($clog2(MAGNITUDE_WIDTH))
    ) u_full_scale_log (
        .clk(clk),
        .rst_n(rst_n),
        .magnitude_in(ADC_MAX_OUTPUT_VALUE[MAGNITUDE_WIDTH-1:0]),
        .data_valid(init_full_scale),
        .db_out(full_scale_db_temp),
        .db_valid(full_scale_db_valid)
    );
    
    // Initialize full scale dB value at startup
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            full_scale_db_q8 <= {RSSI_WIDTH{1'b0}};
            init_full_scale <= 1'b0;
        end else begin
            // Trigger full scale calculation once after reset
            if (init_full_scale == 1'b0) begin
                init_full_scale <= 1'b1;
            end else begin
                init_full_scale <= 1'b0;  // Only trigger once
            end
            
            // Latch the result when valid
            if (full_scale_db_valid && full_scale_db_q8 == 0) begin
                full_scale_db_q8 <= full_scale_db_temp;
            end
        end
    end
    
    // Pipeline Stage 4: Apply EWMA filter
    // Python: filterLeak = EMWA_FilterAccumulator >> ALPHA_SHIFT
    //         EMWA_FilterAccumulator = EMWA_FilterAccumulator + RSSI - filterLeak
    //         RSSI_Filtered = EMWA_FilterAccumulator >> ALPHA_SHIFT
    // The accumulator is kept at higher precision (wider) to maintain accuracy
    // during the filtering operation, then scaled down for output
    reg signed [RSSI_WIDTH-1:0] filter_leak;
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ewma_accumulator <= {(RSSI_WIDTH+ALPHA_SHIFT){1'b0}};
            rssi_filtered <= {RSSI_WIDTH{1'b0}};
            filter_leak <= {RSSI_WIDTH{1'b0}};
        end else if (mitchell_valid) begin
            // Calculate filter leak: accumulator scaled down by ALPHA_SHIFT
            // This represents the "leakage" term in the EWMA filter
            filter_leak <= ewma_accumulator[RSSI_WIDTH+ALPHA_SHIFT-1:ALPHA_SHIFT];
            
            // EWMA filter: y[n] = y[n-1] + x[n] - (y[n-1] >> ALPHA_SHIFT)
            // The accumulator is kept at higher precision (scaled by 2^ALPHA_SHIFT)
            // So we scale RSSI up by ALPHA_SHIFT before adding
            ewma_accumulator <= ewma_accumulator + 
                               ({{ALPHA_SHIFT{rssi_unfiltered[RSSI_WIDTH-1]}}, rssi_unfiltered} << ALPHA_SHIFT) - 
                               {{ALPHA_SHIFT{filter_leak[RSSI_WIDTH-1]}}, filter_leak};
            
            // Output is the accumulator scaled down by ALPHA_SHIFT
            rssi_filtered <= ewma_accumulator[RSSI_WIDTH+ALPHA_SHIFT-1:ALPHA_SHIFT];
        end
    end
    
    // Pipeline Stage 5: Convert to dBFS and register output
    // Python: RSSI_Values[i] = (RSSI_Filtered - FULL_SCALE_DB_Q8) / 256.0
    // In Q8.8 format, division by 256 is just taking the integer part (upper 8 bits)
    // But we keep it in Q8.8 format for consistency
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rssi_raw <= {RSSI_WIDTH{1'b0}};
            rssi_dbfs <= {RSSI_WIDTH{1'b0}};
            rssi_valid <= 1'b0;
        end else begin
            rssi_raw <= rssi_filtered;
            // dBFS = RSSI_filtered - FULL_SCALE_DB_Q8 (both in Q8.8 format)
            rssi_dbfs <= rssi_filtered - full_scale_db_q8;
            rssi_valid <= mitchell_valid;
        end
    end

endmodule

