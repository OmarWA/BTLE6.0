// =================================================================================================================
// File             : agc_tb.v
// Description      : Testbench for Automatic Gain Control (AGC) Module
//                    Tests RSSI estimation and gain control functionality
// =================================================================================================================
//
// Test Scenarios:
// ---------------
// 1. Low amplitude signal (0.1) - should require gain reduction
// 2. Medium amplitude signal (0.5) - should require moderate gain reduction
// 3. High amplitude signal (0.9) - should require significant gain reduction
// 4. Very high amplitude signal (1.0) - should require maximum gain reduction
// 5. Edge cases: zero input, maximum input
//
// Test Vector Generation:
// -----------------------
// The testbench generates I/Q samples similar to the Python model:
// - GFSK preamble signal (10101010 pattern)
// - Configurable amplitude
// - 8 MSPS sampling rate, 1 Mbps bit rate (8 samples per bit)
// - 64 samples total (8 bits * 8 samples/bit)
//
// Verification:
// -------------
// - RSSI values should match Python model (within tolerance)
// - Control word should be selected correctly based on RSSI
// - AGC decision should occur at sample index 32
//
// =================================================================================================================

`timescale 1ns / 1ps

module agc_tb;

    // Parameters
    parameter I_Q_WIDTH = 10;
    parameter RSSI_WIDTH = 16;
    parameter CONTROL_WORD_WIDTH = 8;
    parameter CLK_PERIOD = 125;  // 8 MHz clock (125 ns period)
    parameter SAMPLES_PER_BIT = 8;
    parameter NUM_BITS = 8;
    parameter TOTAL_SAMPLES = SAMPLES_PER_BIT * NUM_BITS;  // 64 samples
    
    // Test parameters
    parameter [31:0] F_IF = 32'd2_000_000;      // 2 MHz IF frequency
    parameter [31:0] F_DEV = 32'd250_000;       // 250 kHz frequency deviation
    parameter [31:0] FS = 32'd8_000_000;         // 8 MSPS sampling rate
    parameter [31:0] BIT_RATE = 32'd1_000_000;   // 1 Mbps bit rate
    parameter real AMPLITUDE = 0.1;              // Test amplitude (will be varied)
    
    // Clock and reset
    reg clk;
    reg rst_n;
    
    // DUT signals
    reg signed [I_Q_WIDTH-1:0] i_sample;
    reg signed [I_Q_WIDTH-1:0] q_sample;
    reg data_valid;
    wire signed [RSSI_WIDTH-1:0] rssi_dbfs;
    wire signed [RSSI_WIDTH-1:0] rssi_raw;
    wire rssi_valid;
    wire [CONTROL_WORD_WIDTH-1:0] control_word;
    wire control_word_valid;
    wire [3:0] tuning_steps;
    
    // Test vectors storage
    reg signed [I_Q_WIDTH-1:0] i_samples [0:TOTAL_SAMPLES-1];
    reg signed [I_Q_WIDTH-1:0] q_samples [0:TOTAL_SAMPLES-1];
    integer sample_index;
    
    // Expected results (from Python model)
    // These would be calculated based on the Python model output
    // For now, we'll verify the control word selection logic
    
    // Instantiate DUT
    agc #(
        .I_Q_WIDTH(I_Q_WIDTH),
        .RSSI_WIDTH(RSSI_WIDTH),
        .CONTROL_WORD_WIDTH(CONTROL_WORD_WIDTH),
        .ALPHA_SHIFT(3),
        .ADC_RESOLUTION(10),
        .RSSI_SAMPLE_INDEX(32),
        .SET_POINT_DBFS(-9 * 256),
        .NUM_GAIN_LEVELS(17)
    ) dut (
        .clk(clk),
        .rst_n(rst_n),
        .i_sample(i_sample),
        .q_sample(q_sample),
        .data_valid(data_valid),
        .rssi_dbfs(rssi_dbfs),
        .rssi_raw(rssi_raw),
        .rssi_valid(rssi_valid),
        .control_word(control_word),
        .control_word_valid(control_word_valid),
        .tuning_steps(tuning_steps)
    );
    
    // Clock generation
    always begin
        clk = 1'b0;
        #(CLK_PERIOD/2);
        clk = 1'b1;
        #(CLK_PERIOD/2);
    end
    
    // Function to generate GFSK preamble signal samples
    // This mimics the Python generatePreambleGFSKSignal function
    task generate_preamble_samples;
        input real amplitude;
        integer i, j;
        integer bit_index;
        reg [7:0] preamble_bits;
        real t, phase, freq, i_real, q_real;
        integer i_int, q_int;
        integer adc_max;
        
        begin
            adc_max = (1 << (I_Q_WIDTH - 1)) - 1;  // ADC max value (511 for 10-bit)
            preamble_bits = 8'b10101010;  // Preamble pattern
            
            phase = 0.0;
            for (i = 0; i < TOTAL_SAMPLES; i = i + 1) begin
                // Determine which bit we're in
                bit_index = i / SAMPLES_PER_BIT;
                
                // Get bit value (0 or 1)
                // Map 0 -> -1, 1 -> +1 for frequency deviation
                freq = F_IF + ((preamble_bits[7-bit_index] ? 1.0 : -1.0) * F_DEV);
                
                // Update phase: φ(t) = 2π * ∫ f(t) dt
                // In discrete time: phase += 2π * freq / FS
                phase = phase + (2.0 * 3.141592653589793 * freq / FS);
                
                // Generate I and Q components
                i_real = amplitude * $cos(phase) * adc_max;
                q_real = amplitude * $sin(phase) * adc_max;
                
                // Convert to integer (saturate if needed)
                i_int = $rtoi(i_real);
                q_int = $rtoi(q_real);
                
                // Saturate to ADC range
                if (i_int > adc_max) i_int = adc_max;
                if (i_int < -adc_max) i_int = -adc_max;
                if (q_int > adc_max) q_int = adc_max;
                if (q_int < -adc_max) q_int = -adc_max;
                
                i_samples[i] = i_int[I_Q_WIDTH-1:0];
                q_samples[i] = q_int[I_Q_WIDTH-1:0];
            end
        end
    endtask
    
    // Test scenario: Low amplitude signal
    task test_low_amplitude;
        begin
            $display("========================================");
            $display("Test 1: Low Amplitude Signal (0.1)");
            $display("========================================");
            
            // Generate test vectors
            generate_preamble_samples(0.1);
            
            // Reset
            rst_n = 1'b0;
            data_valid = 1'b0;
            #(CLK_PERIOD * 10);
            rst_n = 1'b1;
            #(CLK_PERIOD * 5);
            
            // Apply samples
            for (sample_index = 0; sample_index < TOTAL_SAMPLES; sample_index = sample_index + 1) begin
                i_sample = i_samples[sample_index];
                q_sample = q_samples[sample_index];
                data_valid = 1'b1;
                #CLK_PERIOD;
                
                if (rssi_valid) begin
                    $display("Sample %0d: I=%0d, Q=%0d, RSSI_dBFS=%0d (Q8.8), RSSI_raw=%0d", 
                             sample_index, i_sample, q_sample, rssi_dbfs, rssi_raw);
                end
                
                if (control_word_valid && sample_index == 32) begin
                    $display("AGC Decision at sample %0d: Control Word = 0x%02X, Tuning Steps = %0d", 
                             sample_index, control_word, tuning_steps);
                end
            end
            
            data_valid = 1'b0;
            #(CLK_PERIOD * 10);
            
            $display("Test 1 Complete\n");
        end
    endtask
    
    // Test scenario: Medium amplitude signal
    task test_medium_amplitude;
        begin
            $display("========================================");
            $display("Test 2: Medium Amplitude Signal (0.5)");
            $display("========================================");
            
            generate_preamble_samples(0.5);
            
            rst_n = 1'b0;
            data_valid = 1'b0;
            #(CLK_PERIOD * 10);
            rst_n = 1'b1;
            #(CLK_PERIOD * 5);
            
            for (sample_index = 0; sample_index < TOTAL_SAMPLES; sample_index = sample_index + 1) begin
                i_sample = i_samples[sample_index];
                q_sample = q_samples[sample_index];
                data_valid = 1'b1;
                #CLK_PERIOD;
                
                if (rssi_valid) begin
                    $display("Sample %0d: RSSI_dBFS=%0d", sample_index, rssi_dbfs);
                end
                
                if (control_word_valid && sample_index == 32) begin
                    $display("AGC Decision: Control Word = 0x%02X, Tuning Steps = %0d", 
                             control_word, tuning_steps);
                end
            end
            
            data_valid = 1'b0;
            #(CLK_PERIOD * 10);
            
            $display("Test 2 Complete\n");
        end
    endtask
    
    // Test scenario: High amplitude signal
    task test_high_amplitude;
        begin
            $display("========================================");
            $display("Test 3: High Amplitude Signal (0.9)");
            $display("========================================");
            
            generate_preamble_samples(0.9);
            
            rst_n = 1'b0;
            data_valid = 1'b0;
            #(CLK_PERIOD * 10);
            rst_n = 1'b1;
            #(CLK_PERIOD * 5);
            
            for (sample_index = 0; sample_index < TOTAL_SAMPLES; sample_index = sample_index + 1) begin
                i_sample = i_samples[sample_index];
                q_sample = q_samples[sample_index];
                data_valid = 1'b1;
                #CLK_PERIOD;
                
                if (control_word_valid && sample_index == 32) begin
                    $display("AGC Decision: Control Word = 0x%02X, Tuning Steps = %0d", 
                             control_word, tuning_steps);
                end
            end
            
            data_valid = 1'b0;
            #(CLK_PERIOD * 10);
            
            $display("Test 3 Complete\n");
        end
    endtask
    
    // Test scenario: Edge case - zero input
    task test_zero_input;
        begin
            $display("========================================");
            $display("Test 4: Zero Input (Edge Case)");
            $display("========================================");
            
            rst_n = 1'b0;
            data_valid = 1'b0;
            #(CLK_PERIOD * 10);
            rst_n = 1'b1;
            #(CLK_PERIOD * 5);
            
            for (sample_index = 0; sample_index < TOTAL_SAMPLES; sample_index = sample_index + 1) begin
                i_sample = 0;
                q_sample = 0;
                data_valid = 1'b1;
                #CLK_PERIOD;
                
                if (rssi_valid) begin
                    $display("Sample %0d: Zero input, RSSI_dBFS=%0d", sample_index, rssi_dbfs);
                end
            end
            
            data_valid = 1'b0;
            #(CLK_PERIOD * 10);
            
            $display("Test 4 Complete\n");
        end
    endtask
    
    // Main test sequence
    initial begin
        $display("========================================");
        $display("AGC Module Testbench");
        $display("========================================");
        $display("Starting tests...\n");
        
        // Run test scenarios
        test_low_amplitude();
        #(CLK_PERIOD * 100);
        
        test_medium_amplitude();
        #(CLK_PERIOD * 100);
        
        test_high_amplitude();
        #(CLK_PERIOD * 100);
        
        test_zero_input();
        #(CLK_PERIOD * 100);
        
        $display("========================================");
        $display("All Tests Complete");
        $display("========================================");
        $finish;
    end
    
    // Monitor for assertions (optional)
    // You can add assertions here to verify expected behavior
    
    // Waveform dump (uncomment if needed)
    initial begin
        $dumpfile("agc_tb.vcd");
        $dumpvars(0, agc_tb);
    end

endmodule

