// -----------------------------------------------------------------------------
// Project      : Bluetooth PHY
// File         : iq_to_db_tb.sv
// Author       : Gemini (AI)
// Reviewer     : Ahmed Ibrahim
// Created      : 2026-01-08
// Description  : Golden Model Comparison Testbench.
// -----------------------------------------------------------------------------

`timescale 1ns/1ps

module iq_to_db_tb;

  // --- Parameters ---
  localparam int CLK_PERIOD = 125; 
  localparam int ALPHA_SHIFT = 3;  
  
  // --- Signals ---
  logic        clk;
  logic        rst_n;
  logic        en;
  logic [15:0] data_i;
  logic [15:0] data_q;
  logic [15:0] db_result;

  // --- DUT ---
  iq_to_db_top dut (
    .i_clk          (clk),
    .i_rst_n        (rst_n),
    .i_en           (en),
    .i_data_i       (data_i),
    .i_data_q       (data_q),
    .o_db_result_q8 (db_result)
  );

  // --- Clock Gen ---
  initial begin : clk_gen_proc
    clk = 0;
    forever #(CLK_PERIOD/2) clk = ~clk;
  end : clk_gen_proc

  // --- Verification Logic ---
  int file_h;
  int status;
  int r_i, r_q, r_gold;
  int vector_count = 0;
  int error_count = 0;

  initial begin : test_proc
    // Init
    rst_n = 0;
    en    = 0;
    data_i = 0;
    data_q = 0;
    
    // Open the test vectors file
    file_h = $fopen("test_vectors.txt", "r");
    if (!file_h) begin
      $display("[ERROR] Could not open test_vectors.txt. Run Python script first!");
      $finish;
    end

    // Reset sequence
    repeat(10) @(posedge clk);
    rst_n = 1;
    en = 1;

    $display("--- Starting Golden Model Comparison (Alpha=%0d) ---", ALPHA_SHIFT);

    // Process vectors from file
    while (!$feof(file_h)) begin
      status = $fscanf(file_h, "%d %d %d\n", r_i, r_q, r_gold);
      
      if (status == 3) begin
        // Apply Stimulus
        data_i <= 16'(r_i);
        data_q <= 16'(r_q);
        
        // Wait for the edge where the output should be valid for the CURRENT vector
        @(posedge clk);
        #1; // Sample shortly after edge to avoid race conditions

        // Signed comparison of Q8.8 result
        if ($signed(db_result) !== $signed(r_gold[15:0])) begin
          $display("[FAIL] Vector %0d | IN:(%d, %d) | EXP:%d | GOT:%d", 
                   vector_count, r_i, r_q, $signed(r_gold[15:0]), $signed(db_result));
          error_count++;
        end else begin
          if (vector_count % 100 == 0)
            $display("[PASS] Vector %0d | Result:%0d", vector_count, $signed(db_result));
        end
        
        vector_count++;
      end
    end

    $fclose(file_h);
    
    // Final Reporting
    $display("\n-------------------------------------------");
    $display("Verification Finished");
    $display("Total Vectors: %0d", vector_count);
    $display("Total Errors : %0d", error_count);
    if (error_count == 0) begin
      $display("RESULT: BIT-EXACT SUCCESS");
    end else begin
      $display("RESULT: FAILURE");
    end
    $display("-------------------------------------------");
    
    $finish;
  end : test_proc

endmodule : iq_to_db_tb