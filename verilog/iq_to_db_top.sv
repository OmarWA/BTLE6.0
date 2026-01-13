// -----------------------------------------------------------------------------
// Project      : Bluetooth PHY
// File         : iq_to_db_top.sv
// Author       : Gemini (AI)
// Reviewer     : Ahmed Ibrahim
// Created      : 2026-01-08
// Description  : Hardware-efficient IQ to dB converter.
//                Uses Alpha-Max + Beta-Min for magnitude, Mitchell's Log 
//                approximation, and a high-precision EMA filter.
// -----------------------------------------------------------------------------

module iq_to_db_top (
    input  logic         i_clk,          // System Clock
    input  logic         i_rst_n,        // Active Low Reset
    input  logic         i_en,           // Module Enable
    input  logic [15:0]  i_data_i,       // In-phase input component
    input  logic [15:0]  i_data_q,       // Quadrature input component
    output logic [15:0]  o_db_result_q8  // Output dB value in Q8.8 format
);

    // --- Local Parameters ---
    localparam int P_BIT_WIDTH   = 16;
    localparam int P_ALPHA_SHIFT = 3;

    // --- Internal Signals ---
    logic [P_BIT_WIDTH-1:0] abs_i, abs_q;    // Absolute values of I and Q
    logic [P_BIT_WIDTH:0]   mag_approx;      // Calculated magnitude (approx)
    logic [15:0]            log2_q10;        // Binary logarithm result (Q5.10)
    logic [31:0]            db_scaled;       // Internal scaling product
    logic signed [15:0]     db_val;          // Instantaneous dB value (Q8.8)

    // ---------------------------------------------------------
    // Stage 1: Absolute Value & Magnitude Approx (Alpha-Max/Beta-Min)
    // ---------------------------------------------------------
    logic [P_BIT_WIDTH-1:0] mx_val, mn_val;
    
    always_ff @(posedge i_clk or negedge i_rst_n) begin : stage1_proc
        if (!i_rst_n) begin
            abs_i <= '0;
            abs_q <= '0;
            mag_approx <= '0;
        end else if (i_en) begin
            // Calculate Absolute Values
            abs_i <= ($signed(i_data_i) < 0) ? -$signed(i_data_i) : i_data_i;
            abs_q <= ($signed(i_data_q) < 0) ? -$signed(i_data_q) : i_data_q;
            
            // Determine Max and Min for Alpha-Max/Beta-Min
            if (abs_i > abs_q) begin
                mx_val = abs_i;
                mn_val = abs_q;
            end else begin
                mx_val = abs_q;
                mn_val = abs_i;
            end // end if abs_i > abs_q
            
            // Result = (31/32)*mx + (13/32)*mn
            mag_approx <= (mx_val - (mx_val >> 5)) + ((mn_val >> 2) + (mn_val >> 3) + (mn_val >> 5));
        end // end if i_en
    end : stage1_proc

    // ---------------------------------------------------------
    // Stage 2: Mitchell's Log2 Approximation
    // ---------------------------------------------------------
    logic [4:0]  leading_bit; // Characteristic (k)
    logic [9:0]  fraction;    // Mantissa (m)
    logic        found_msb;
    
    always_comb begin : lod_comb
        leading_bit = 5'd0;
        fraction    = 10'd0;
        found_msb   = 1'b0;

        // Priority Encoder for Leading One Detection
        for (int k = 16; k >= 0; k--) begin
            if (mag_approx[k] && !found_msb) begin
                found_msb   = 1'b1;
                leading_bit = 5'(k);
                // Extract 10 bits below the leading one for the mantissa
                if (k >= 10) begin
                    fraction = mag_approx[k-1 -: 10];
                end else if (k > 0) begin
                    fraction = 10'(mag_approx << (10 - k));
                end else begin
                    fraction = 10'd0;
                end // end if k range check
            end // end if mag_approx
        end // end for loop
    end : lod_comb

    always_ff @(posedge i_clk or negedge i_rst_n) begin : stage2_proc
        if (!i_rst_n) begin
            log2_q10 <= '0;
        end else if (i_en) begin
            // Combine characteristic and mantissa: log2(x) approx k.m
            log2_q10 <= {leading_bit, fraction};
        end // end if i_en
    end : stage2_proc

    // ---------------------------------------------------------
    // Stage 3: dB Scaling (20*log10(x))
    // ---------------------------------------------------------
    // 20*log10(2) approx 6.0206. 6.0206 * 2^8 = 1541.
    assign db_scaled = (32'(log2_q10) * 32'd1541);
    
    always_ff @(posedge i_clk or negedge i_rst_n) begin : stage3_proc
        if (!i_rst_n) begin
            db_val <= '0;
        end else if (i_en) begin
            // log2_q10 is Q5.10. 1541 is Q8. Product is Q13.18.
            // Shifting right by 10 results in Q13.8 (standard dB Q8.8 result)
            db_val <= 16'(db_scaled >> 10); 
        end // end if i_en
    end : stage3_proc

    // ---------------------------------------------------------
    // Stage 4: High-Precision EMA Filter
    // ---------------------------------------------------------
    localparam int ACC_WIDTH = 16 + 3; // 16 + P_ALPHA_SHIFT
    logic signed [ACC_WIDTH-1:0] filter_acc; // Internal high-precision accumulator

    always_ff @(posedge i_clk or negedge i_rst_n) begin : stage4_proc
        if (!i_rst_n) begin
            filter_acc <= '0;
            o_db_result_q8 <= '0;
        end else if (i_en) begin
            // Unity-gain EMA structure: acc = acc + input - (acc >> shift)
            filter_acc <= filter_acc + $signed(db_val) - $signed(filter_acc >>> 3);
            
            // The filtered output is the stabilized MSBs of the accumulator
            o_db_result_q8 <= 16'(filter_acc >>> 3);
        end // end if i_en
    end : stage4_proc

endmodule : iq_to_db_top