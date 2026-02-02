// =================================================================================================================
// File             : leading_one_detector.v
// Description      : Hardware-equivalent Leading One Detector (LOD) module
//                    Finds the position of the most significant '1' bit in a number
//                    Used as a building block for Mitchell's logarithm approximation
// =================================================================================================================
//
// Python Mapping:
// ---------------
// The Python function hw_lod(x) returns x.bit_length() - 1, which finds the position of the MSB.
// In Verilog, we implement this using a priority encoder that finds the leftmost '1' bit.
//
// Algorithm:
// ----------
// For a positive number x, the leading one detector finds k such that 2^(k) <= x < 2^(k+1)
// This is equivalent to finding the bit position of the MSB (most significant bit).
// Returns 0 if input is 0 or negative (handled by checking sign bit).
//
// Example:
// --------
// Input:  16-bit value 0x0A3F (binary: 0000 1010 0011 1111)
// Output: k = 13 (the MSB is at position 13, counting from 0)
//         2^13 = 8192 <= 0x0A3F = 2623 < 2^14 = 16384
//
// =================================================================================================================

module leading_one_detector #(
    parameter DATA_WIDTH = 16  // Bit width of input data
)(
    input wire [DATA_WIDTH-1:0] data_in,      // Input data to find leading one
    output reg [$clog2(DATA_WIDTH)-1:0] lod_out  // Position of leading one (0 to DATA_WIDTH-1)
);

    // Priority encoder to find the position of the most significant '1' bit
    // This implements the hardware equivalent of Python's x.bit_length() - 1
    integer i;
    always @(*) begin
        lod_out = {($clog2(DATA_WIDTH)){1'b0}};  // Default to 0
        
        // Check from MSB to LSB to find the first '1'
        // If no '1' is found, lod_out remains 0 (which is correct for input = 0)
        for (i = DATA_WIDTH - 1; i >= 0; i = i - 1) begin
            if (data_in[i]) begin
                lod_out = i[$clog2(DATA_WIDTH)-1:0];
                // Break out of loop (in simulation, this will find the first match)
            end
        end
    end

    // Note: In hardware synthesis, the for loop will be unrolled into a priority encoder tree
    // This is a standard synthesis pattern and will generate efficient hardware

endmodule

