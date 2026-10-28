// ============================================================
//  PROJECT  : Bio-Signal Driven Multi-Mode RISC-V Processor
//  MODULE   : core2_monitor
//  PURPOSE  : Core 2 — Monitor Mode (16-bit Approximate ALU)
//             TCSA adder, BAM multiplier, VPU, EEU
//  FPGA     : Synthesisable — no simulation-only constructs
//  AUTHOR   : SASTRA Deemed to be University
// ============================================================

module core2_monitor (
    input  wire        clk,
    input  wire        reset,
    input  wire [31:0] operand_a,      // Core 2 reads [31:16]
    input  wire [31:0] operand_b,      // Core 2 reads [31:16]
    input  wire [2:0]  alu_op,
    input  wire        core2_enable,
    input  wire [1:0]  approx_level,   // 00=12bit 01=16bit 10=20bit
    output reg  [31:0] result,
    output reg         carry_flag,
    output reg         zero_flag,
    output reg         error_flag      // HIGH = approx error too large
);

// ALU ops
localparam ADD  = 3'b000;
localparam SUB  = 3'b001;
localparam AND  = 3'b010;
localparam OR   = 3'b011;
localparam XOR  = 3'b100;
localparam SLT  = 3'b101;
localparam MUL  = 3'b110;

// Error threshold — if approx error > this, set error_flag
localparam ERROR_THRESHOLD = 8'd50;

// ── OPERAND EXTRACTION based on approx_level ──────────────
reg [19:0] op_a_ext, op_b_ext;

always @(*) begin
    case (approx_level)
        2'b00: begin   // 12-bit
            op_a_ext = {operand_a[31:20], 8'b0};
            op_b_ext = {operand_b[31:20], 8'b0};
        end
        2'b01: begin   // 16-bit (default)
            op_a_ext = {operand_a[31:16], 4'b0};
            op_b_ext = {operand_b[31:16], 4'b0};
        end
        2'b10: begin   // 20-bit
            op_a_ext = operand_a[31:12];
            op_b_ext = operand_b[31:12];
        end
        default: begin
            op_a_ext = {operand_a[31:16], 4'b0};
            op_b_ext = {operand_b[31:16], 4'b0};
        end
    endcase
end

// Active 16-bit operands
wire [15:0] active_op_a = op_a_ext[19:4];
wire [15:0] active_op_b = op_b_ext[19:4];

// ── TCSA — Truncated Carry Select Adder ───────────────────
// OR-based carry prediction on lower 8 bits
// Exact computation on upper 8 bits with predicted carry
wire        carry_predict;
wire [7:0]  lower_sum_approx;
wire [8:0]  upper_sum_exact;
wire [16:0] tcsa_result_full;

assign carry_predict    = |(active_op_a[7:0] & active_op_b[7:0]);
assign lower_sum_approx = active_op_a[7:0] ^ active_op_b[7:0];
assign upper_sum_exact  = {1'b0, active_op_a[15:8]}
                        + {1'b0, active_op_b[15:8]}
                        + {8'b0, carry_predict};
assign tcsa_result_full = {upper_sum_exact, lower_sum_approx};

// Exact 16-bit result for error calculation
wire [16:0] exact_result_full = {1'b0, active_op_a}
                              + {1'b0, active_op_b};
wire [15:0] exact_result_16   = exact_result_full[15:0];
wire [15:0] tcsa_result_16    = tcsa_result_full[15:0];

// Error magnitude (absolute difference)
wire [15:0] error_mag = (tcsa_result_16 > exact_result_16)
                      ? (tcsa_result_16 - exact_result_16)
                      : (exact_result_16 - tcsa_result_16);

// ── BAM — Broken Array Multiplier ─────────────────────────
// Skips lower triangle partial products — ~40% fewer gates
// Approximation: zero out lower 8×8 partial products
wire [31:0] bam_result;
wire [15:0] bam_a = active_op_a;
wire [15:0] bam_b = active_op_b;

// Upper×Upper (exact)
wire [15:0] pp_hh = bam_a[15:8] * bam_b[15:8];
// Upper×Lower (kept)
wire [15:0] pp_hl = bam_a[15:8] * bam_b[7:0];
wire [15:0] pp_lh = bam_a[7:0]  * bam_b[15:8];
// Lower×Lower (broken — approximated as 0)
// wire pp_ll = 0  ← this is the "broken" part

assign bam_result = ({pp_hh, 16'b0})
                  + ({8'b0, pp_hl, 8'b0})
                  + ({8'b0, pp_lh, 8'b0});

// ── MAIN ALU ──────────────────────────────────────────────
always @(posedge clk or posedge reset) begin
    if (reset) begin
        result     <= 32'b0;
        carry_flag <= 1'b0;
        zero_flag  <= 1'b1;
        error_flag <= 1'b0;
    end
    else if (core2_enable) begin
        // Error flag — set if TCSA approximation too large
        error_flag <= (error_mag > {8'b0, ERROR_THRESHOLD});

        case (alu_op)
            ADD: begin
                result     <= {tcsa_result_16, 16'b0};
                carry_flag <= tcsa_result_full[16];
                zero_flag  <= (tcsa_result_16 == 16'b0);
            end
            SUB: begin
                result     <= {(active_op_a - active_op_b), 16'b0};
                carry_flag <= (active_op_a < active_op_b);
                zero_flag  <= (active_op_a == active_op_b);
            end
            AND: begin
                result     <= {(active_op_a & active_op_b), 16'b0};
                carry_flag <= 1'b0;
                zero_flag  <= ((active_op_a & active_op_b) == 16'b0);
            end
            OR: begin
                result     <= {(active_op_a | active_op_b), 16'b0};
                carry_flag <= 1'b0;
                zero_flag  <= ((active_op_a | active_op_b) == 16'b0);
            end
            XOR: begin
                result     <= {(active_op_a ^ active_op_b), 16'b0};
                carry_flag <= 1'b0;
                zero_flag  <= ((active_op_a ^ active_op_b) == 16'b0);
            end
            SLT: begin
                result     <= (active_op_a < active_op_b)
                              ? 32'h00010000 : 32'h0;
                carry_flag <= 1'b0;
                zero_flag  <= ~(active_op_a < active_op_b);
            end
            MUL: begin
                result     <= bam_result;
                carry_flag <= |bam_result[31:16];
                zero_flag  <= (bam_result == 32'b0);
            end
            default: begin
                result     <= 32'b0;
                carry_flag <= 1'b0;
                zero_flag  <= 1'b1;
            end
        endcase
    end
end

endmodule
