// =================================================================================================================
// File             : agc.v
// Description      : Top-level Automatic Gain Control (AGC) Module for BLE 6.0 Receiver
//                    Integrates RSSI estimation and gain control logic
// =================================================================================================================
//
// Python Mapping:
// ---------------
// This top-level module integrates the functionality from the Python script:
// 1. estimateRSSI(I, Q, ...) -> rssi_estimator module
// 2. AGC control logic -> agc_controller module
// 3. Main flow: Process I/Q samples, estimate RSSI, make gain control decision
//
// Architecture:
// -------------
// - RSSI Estimator: Computes signal strength from I/Q samples using magnitude approximation
//                   and Mitchell's logarithm algorithm with EWMA filtering
// - AGC Controller: Monitors RSSI and determines optimal gain control word
//
// Timing:
// -------
// - I/Q samples are processed continuously
// - RSSI is estimated for each sample
// - AGC decision is made at sample index 32 (RSSI_SAMPLE_INDEX)
// - Control word is updated once and remains stable
//
// =================================================================================================================

module agc #(
    parameter I_Q_WIDTH = 10,           // Bit width of I and Q samples (ADC resolution)
    parameter RSSI_WIDTH = 16,          // Bit width for RSSI (Q8.8 format)
    parameter CONTROL_WORD_WIDTH = 8,   // Bit width of control word output
    parameter ALPHA_SHIFT = 3,          // EWMA filter parameter
    parameter ADC_RESOLUTION = 10,      // ADC resolution in bits
    parameter RSSI_SAMPLE_INDEX = 32,    // Sample index for AGC decision
    parameter SET_POINT_DBFS = -9 * 256, // Desired RSSI level in Q8.8 format (-9 dBFS)
    parameter NUM_GAIN_LEVELS = 17      // Number of gain levels available
)(
    input wire clk,
    input wire rst_n,
    input wire signed [I_Q_WIDTH-1:0] i_sample,  // I component sample
    input wire signed [I_Q_WIDTH-1:0] q_sample,  // Q component sample
    input wire data_valid,                       // Input data valid signal
    output wire signed [RSSI_WIDTH-1:0] rssi_dbfs, // RSSI output in dBFS (Q8.8 format)
    output wire signed [RSSI_WIDTH-1:0] rssi_raw,   // RSSI output in raw Q8.8 format
    output wire rssi_valid,                       // RSSI output valid signal
    output wire [CONTROL_WORD_WIDTH-1:0] control_word, // Gain control word output
    output wire control_word_valid,                 // Control word valid signal
    output wire [3:0] tuning_steps                  // Number of tuning steps (for debugging)
);

    // Internal signals
    wire signed [RSSI_WIDTH-1:0] rssi_dbfs_int;  // Internal RSSI dBFS signal
    wire rssi_valid_int;                          // Internal RSSI valid signal
    
    // Instantiate RSSI Estimator
    rssi_estimator #(
        .I_Q_WIDTH(I_Q_WIDTH),
        .MAGNITUDE_WIDTH(16),
        .RSSI_WIDTH(RSSI_WIDTH),
        .ALPHA_SHIFT(ALPHA_SHIFT),
        .ADC_RESOLUTION(ADC_RESOLUTION)
    ) u_rssi_estimator (
        .clk(clk),
        .rst_n(rst_n),
        .i_sample(i_sample),
        .q_sample(q_sample),
        .data_valid(data_valid),
        .rssi_dbfs(rssi_dbfs_int),
        .rssi_raw(rssi_raw),
        .rssi_valid(rssi_valid_int)
    );
    
    // Instantiate AGC Controller
    agc_controller #(
        .RSSI_WIDTH(RSSI_WIDTH),
        .CONTROL_WORD_WIDTH(CONTROL_WORD_WIDTH),
        .RSSI_SAMPLE_INDEX(RSSI_SAMPLE_INDEX),
        .SET_POINT_DBFS(SET_POINT_DBFS),
        .NUM_GAIN_LEVELS(NUM_GAIN_LEVELS)
    ) u_agc_controller (
        .clk(clk),
        .rst_n(rst_n),
        .rssi_dbfs(rssi_dbfs_int),
        .rssi_valid(rssi_valid_int),
        .control_word(control_word),
        .control_word_valid(control_word_valid),
        .tuning_steps(tuning_steps)
    );
    
    // Connect RSSI outputs
    assign rssi_dbfs = rssi_dbfs_int;
    assign rssi_valid = rssi_valid_int;

endmodule

