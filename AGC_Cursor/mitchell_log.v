// =================================================================================================================
// File             : mitchell_log.v
// Description      : Mitchell's Logarithm Approximation Module (Fixed-Point Q8.8 format)
//                    Computes 20*log10(x) using Mitchell's algorithm for hardware efficiency
//                    Credits: Eng. Ahmed Ibrahim (algorithm design)
// =================================================================================================================
//
// Python Mapping:
// ---------------
// The Python function hw_mitchell_db(amp_int) implements Mitchell's log approximation:
// 1. Uses leading one detector to find k = log2(x) approximation
// 2. Computes mantissa m = (x >> (k-10)) - 1024 or (x << (10-k)) - 1024
// 3. Combines to get log2(x) in Q10 format: log2_q10 = (k << 10) + m
// 4. Converts to dB using: dB = (log2_q10 * 1541) >> 10
//    where 1541 = 20*log10(2)*2^8 (scaled for Q8.8 output)
//
// Fixed-Point Format:
// -------------------
// Input:  Unsigned integer (magnitude estimate)
// Output: Signed Q8.8 format (8 integer bits, 8 fractional bits)
//         Range: approximately -100*256 to +100*256 (representing -100 dB to +100 dB)
//
// Algorithm Details:
// ------------------
// Mitchell's approximation: log2(x) ≈ k + (x/2^k - 1)
// where k is the position of the leading one (from LOD)
// This is then converted to dB: 20*log10(x) = 20*log10(2) * log2(x) ≈ 6.02 * log2(x)
//
// Hardware Implementation:
// ------------------------
// 1. Leading One Detector finds k (MSB position)
// 2. Normalize x to Q10 format: m = (x normalized) - 1024
// 3. log2(x) ≈ (k << 10) + m  (in Q10 format)
// 4. Convert to dB: dB = (log2_q10 * 1541) >> 10  (result in Q8.8)
//
// =================================================================================================================

module mitchell_log #(
    parameter INPUT_WIDTH = 16,      // Bit width of input magnitude
    parameter OUTPUT_WIDTH = 16,     // Bit width of output (Q8.8 format)
    parameter LOD_WIDTH = 4          // Bit width for LOD output (should be $clog2(INPUT_WIDTH))
)(
    input wire clk,
    input wire rst_n,
    input wire [INPUT_WIDTH-1:0] magnitude_in,  // Input magnitude (unsigned)
    input wire data_valid,                       // Input data valid signal
    output reg signed [OUTPUT_WIDTH-1:0] db_out, // Output in Q8.8 format (dB)
    output reg db_valid                          // Output valid signal
);

    // Constants
    localparam Q10_SCALE = 10;           // Q10 format uses 10 fractional bits
    localparam Q10_ONE = 1024;           // 1.0 in Q10 format = 2^10
    localparam DB_CONVERSION_CONST = 1541;  // 20*log10(2)*2^8 = 1541 (for Q8.8 output)
    localparam MIN_DB_VALUE = -100 * 256;   // Minimum dB value in Q8.8 format (-100 dB)
    
    // Internal signals
    wire [LOD_WIDTH-1:0] k;              // Leading one position from LOD
    reg [INPUT_WIDTH+Q10_SCALE-1:0] m_fixed;  // Mantissa in Q10 format (wider to handle shifts)
    reg [INPUT_WIDTH+Q10_SCALE-1:0] log2_q10; // log2(x) in Q10 format
    reg signed [OUTPUT_WIDTH+10-1:0] db_raw;   // Raw dB calculation (before scaling)
    
    // Pipeline registers
    reg [INPUT_WIDTH-1:0] magnitude_reg;
    reg [LOD_WIDTH-1:0] k_reg;
    reg data_valid_reg;
    
    // Instantiate Leading One Detector
    leading_one_detector #(
        .DATA_WIDTH(INPUT_WIDTH)
    ) u_lod (
        .data_in(magnitude_in),
        .lod_out(k)
    );
    
    // Pipeline Stage 1: Register inputs and LOD result
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            magnitude_reg <= {INPUT_WIDTH{1'b0}};
            k_reg <= {LOD_WIDTH{1'b0}};
            data_valid_reg <= 1'b0;
        end else begin
            magnitude_reg <= magnitude_in;
            k_reg <= k;
            data_valid_reg <= data_valid;
        end
    end
    
    // Pipeline Stage 2: Compute mantissa m_fixed
    // Python: if k >= 10: m_fixed = (amp_int >> (k - 10)) - 1024
    //         else: m_fixed = (amp_int << (10 - k)) - 1024
    always @(*) begin
        if (magnitude_reg == 0) begin
            // Special case: input is 0, return minimum dB value
            m_fixed = {INPUT_WIDTH+Q10_SCALE{1'b0}};
        end else if (k_reg >= Q10_SCALE) begin
            // Scale down: shift right by (k - 10)
            m_fixed = (magnitude_reg >> (k_reg - Q10_SCALE)) - Q10_ONE;
        end else begin
            // Scale up: shift left by (10 - k)
            m_fixed = (magnitude_reg << (Q10_SCALE - k_reg)) - Q10_ONE;
        end
    end
    
    // Pipeline Stage 3: Compute log2_q10 = (k << 10) + m_fixed
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            log2_q10 <= {(INPUT_WIDTH+Q10_SCALE){1'b0}};
            data_valid_reg <= 1'b0;
        end else begin
            if (magnitude_reg == 0) begin
                // Input was 0, set log2 to minimum
                log2_q10 <= {(INPUT_WIDTH+Q10_SCALE){1'b0}};
            end else begin
                // log2(x) ≈ k*1024 + m (in Q10 format)
                log2_q10 <= (k_reg << Q10_SCALE) + m_fixed[INPUT_WIDTH+Q10_SCALE-1:0];
            end
            data_valid_reg <= data_valid_reg;
        end
    end
    
    // Pipeline Stage 4: Convert to dB: dB = (log2_q10 * 1541) >> 10
    // This converts from Q10 format to Q8.8 format
    always @(*) begin
        if (magnitude_reg == 0) begin
            db_raw = MIN_DB_VALUE;
        end else begin
            // Multiply by conversion constant and scale down by 10 bits
            // Result is in Q8.8 format
            db_raw = (log2_q10 * DB_CONVERSION_CONST) >> Q10_SCALE;
        end
    end
    
    // Pipeline Stage 5: Register output
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            db_out <= {OUTPUT_WIDTH{1'b0}};
            db_valid <= 1'b0;
        end else begin
            // Saturate to output width
            if (db_raw > $signed({1'b0, {(OUTPUT_WIDTH-1){1'b1}}})) begin
                db_out <= $signed({1'b0, {(OUTPUT_WIDTH-1){1'b1}}});  // Positive saturation
            end else if (db_raw < $signed({1'b1, {(OUTPUT_WIDTH-1){1'b0}}})) begin
                db_out <= $signed({1'b1, {(OUTPUT_WIDTH-1){1'b0}}});  // Negative saturation
            end else begin
                db_out <= db_raw[OUTPUT_WIDTH-1:0];
            end
            db_valid <= data_valid_reg;
        end
    end

endmodule

