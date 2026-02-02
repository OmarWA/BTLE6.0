// =================================================================================================================
// File             : agc_controller.v
// Description      : Automatic Gain Control (AGC) Controller Module
//                    Monitors RSSI and adjusts receiver gain to maintain optimal signal level
// =================================================================================================================
//
// Python Mapping:
// ---------------
// The Python code performs:
// 1. Estimates RSSI for incoming I/Q samples
// 2. At RSSI_SAMPLE_INDEX (32), compares RSSI with SET_POINT_DBFS (-9 dBFS)
// 3. Calculates gain error: gainError = RSSI[RSSI_SAMPLE_INDEX] - SET_POINT_DBFS
// 4. Determines tuning steps: while (gainError > 0): gainError -= 5, TUNING_STEPS += 1
// 5. Selects new gain level: newRxGainLevel = RX_GAINS[TUNING_STEPS]
// 6. Outputs control word: controlWord = CONTROL_WORDS[TUNING_STEPS]
//
// Gain Control Strategy:
// ----------------------
// - Receiver starts at maximum gain (104 dB, control word 0x1F)
// - Each tuning step reduces gain by 5 dB (except some steps which reduce by different amounts)
// - Goal: Bring RSSI close to SET_POINT_DBFS (-9 dBFS) to avoid saturation
// - Control word is updated once at RSSI_SAMPLE_INDEX and remains stable
//
// Timing:
// -------
// - AGC decision is made at sample index 32 (RSSI_SAMPLE_INDEX)
// - Control word changes at this point and remains constant afterward
// - This allows RSSI to stabilize before making gain adjustment decision
//
// =================================================================================================================

module agc_controller #(
    parameter RSSI_WIDTH = 16,              // Bit width of RSSI input (Q8.8 format)
    parameter CONTROL_WORD_WIDTH = 8,       // Bit width of control word output
    parameter RSSI_SAMPLE_INDEX = 32,       // Sample index at which to make AGC decision
    parameter SET_POINT_DBFS = -9 * 256,    // Desired RSSI level in Q8.8 format (-9 dBFS)
    parameter NUM_GAIN_LEVELS = 17           // Number of gain levels available
)(
    input wire clk,
    input wire rst_n,
    input wire signed [RSSI_WIDTH-1:0] rssi_dbfs,  // RSSI input in dBFS (Q8.8 format)
    input wire rssi_valid,                          // RSSI valid signal
    output reg [CONTROL_WORD_WIDTH-1:0] control_word, // Control word output
    output reg control_word_valid,                   // Control word valid signal
    output reg [3:0] tuning_steps                    // Number of tuning steps applied (for debugging)
);

    // RX Gain levels in dB (from Python: RX_GAINS array)
    // These represent the gain levels available in the receiver
    localparam [6:0] RX_GAINS [0:NUM_GAIN_LEVELS-1] = {
        7'd104, 7'd99, 7'd94, 7'd88, 7'd83, 7'd78, 7'd73, 7'd67, 
        7'd62, 7'd57, 7'd52, 7'd47, 7'd41, 7'd36, 7'd31, 7'd26, 7'd21
    };
    
    // Control words corresponding to each gain level (from Python: CONTROL_WORDS array)
    localparam [CONTROL_WORD_WIDTH-1:0] CONTROL_WORDS [0:NUM_GAIN_LEVELS-1] = {
        8'h1F, 8'h1E, 8'h1D, 8'h16, 8'h15, 8'h14, 8'h13, 8'h2D,
        8'h2C, 8'h2B, 8'h2A, 8'h4A, 8'h43, 8'h42, 8'h62, 8'h61, 8'h60
    };
    
    // Internal signals
    reg [$clog2(RSSI_SAMPLE_INDEX+1)-1:0] sample_counter;  // Counter to track sample index
    reg signed [RSSI_WIDTH-1:0] rssi_at_decision;          // RSSI value at decision point
    reg signed [RSSI_WIDTH-1:0] gain_error;                // Gain error (RSSI - SET_POINT)
    reg [3:0] calculated_steps;                            // Calculated tuning steps
    reg decision_made;                                     // Flag indicating decision has been made
    
    // State machine states
    localparam [1:0] IDLE = 2'b00;
    localparam [1:0] MONITORING = 2'b01;
    localparam [1:0] DECISION_MADE = 2'b10;
    
    reg [1:0] state;
    
    // Gain error calculation and tuning steps determination
    // Python: gainError = RSSI_Values[RSSI_SAMPLE_INDEX] - SET_POINT_DBFS
    //         while (gainError > 0): gainError -= 5, TUNING_STEPS += 1
    // In hardware, we calculate this once when we reach RSSI_SAMPLE_INDEX
    // Each step reduces gain by approximately 5 dB
    // In Q8.8 format, 5 dB = 5 * 256 = 1280
    localparam [RSSI_WIDTH-1:0] STEP_SIZE_DB = 16'd1280;  // 5 dB in Q8.8 format
    
    always @(*) begin
        gain_error = rssi_at_decision - SET_POINT_DBFS;
        
        // Calculate tuning steps: divide gain_error by STEP_SIZE_DB (5 dB per step)
        // Use simple division: steps = gain_error / STEP_SIZE_DB
        if (gain_error <= 0) begin
            calculated_steps = 4'b0000;  // RSSI is already at or below set point
        end else begin
            // Divide gain_error by STEP_SIZE_DB (1280 = 5 dB in Q8.8)
            // This is equivalent to right-shifting by log2(1280) ≈ 10 bits, but we need exact division
            // For simplicity, we'll use a lookup table approach with thresholds
            // Since each step is 5 dB, we can approximate: steps ≈ gain_error >> 10 (roughly)
            // But for accuracy, we'll use comparison with multiples of STEP_SIZE_DB
            if (gain_error > (STEP_SIZE_DB * 16)) begin
                calculated_steps = 4'd16;  // Maximum (17 levels, index 0-16)
            end else if (gain_error > (STEP_SIZE_DB * 15)) begin
                calculated_steps = 4'd15;
            end else if (gain_error > (STEP_SIZE_DB * 14)) begin
                calculated_steps = 4'd14;
            end else if (gain_error > (STEP_SIZE_DB * 13)) begin
                calculated_steps = 4'd13;
            end else if (gain_error > (STEP_SIZE_DB * 12)) begin
                calculated_steps = 4'd12;
            end else if (gain_error > (STEP_SIZE_DB * 11)) begin
                calculated_steps = 4'd11;
            end else if (gain_error > (STEP_SIZE_DB * 10)) begin
                calculated_steps = 4'd10;
            end else if (gain_error > (STEP_SIZE_DB * 9)) begin
                calculated_steps = 4'd9;
            end else if (gain_error > (STEP_SIZE_DB * 8)) begin
                calculated_steps = 4'd8;
            end else if (gain_error > (STEP_SIZE_DB * 7)) begin
                calculated_steps = 4'd7;
            end else if (gain_error > (STEP_SIZE_DB * 6)) begin
                calculated_steps = 4'd6;
            end else if (gain_error > (STEP_SIZE_DB * 5)) begin
                calculated_steps = 4'd5;
            end else if (gain_error > (STEP_SIZE_DB * 4)) begin
                calculated_steps = 4'd4;
            end else if (gain_error > (STEP_SIZE_DB * 3)) begin
                calculated_steps = 4'd3;
            end else if (gain_error > (STEP_SIZE_DB * 2)) begin
                calculated_steps = 4'd2;
            end else if (gain_error > (STEP_SIZE_DB * 1)) begin
                calculated_steps = 4'd1;
            end else begin
                calculated_steps = 4'd0;  // Less than one step needed
            end
        end
    end
    
    // Main state machine
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sample_counter <= {($clog2(RSSI_SAMPLE_INDEX+1)){1'b0}};
            rssi_at_decision <= {RSSI_WIDTH{1'b0}};
            control_word <= CONTROL_WORDS[0];  // Start with maximum gain (0x1F)
            control_word_valid <= 1'b0;
            tuning_steps <= 4'b0000;
            decision_made <= 1'b0;
            state <= IDLE;
        end else begin
            case (state)
                IDLE: begin
                    if (rssi_valid) begin
                        sample_counter <= 1;
                        state <= MONITORING;
                        control_word_valid <= 1'b0;
                    end
                end
                
                MONITORING: begin
                    if (rssi_valid) begin
                        if (sample_counter == RSSI_SAMPLE_INDEX) begin
                            // Capture RSSI at decision point
                            rssi_at_decision <= rssi_dbfs;
                            state <= DECISION_MADE;
                            decision_made <= 1'b1;
                        end else begin
                            sample_counter <= sample_counter + 1;
                        end
                    end
                end
                
                DECISION_MADE: begin
                    // Decision has been made, update control word
                    if (!decision_made) begin
                        // This should not happen, but handle gracefully
                        state <= DECISION_MADE;
                    end else begin
                        // Limit tuning steps to available gain levels
                        if (calculated_steps >= NUM_GAIN_LEVELS) begin
                            tuning_steps <= NUM_GAIN_LEVELS - 1;
                            control_word <= CONTROL_WORDS[NUM_GAIN_LEVELS - 1];
                        end else begin
                            tuning_steps <= calculated_steps;
                            control_word <= CONTROL_WORDS[calculated_steps];
                        end
                        control_word_valid <= 1'b1;
                        // Stay in this state (control word remains constant)
                    end
                end
                
                default: begin
                    state <= IDLE;
                end
            endcase
        end
    end

endmodule

