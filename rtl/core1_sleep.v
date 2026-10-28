// ============================================================
//  PROJECT  : Bio-Signal Driven Multi-Mode RISC-V Processor
//  MODULE   : core1_sleep
//  PURPOSE  : Core 1 — Sleep Mode (8-bit MSB-Only ALU)
//             Ultra-low power — uses only top 8 bits
//             Supports: ADD SUB AND OR XOR SLT
//  FPGA     : Synthesisable — no simulation-only constructs
//  AUTHOR   : SASTRA Deemed to be University
// ============================================================

module core1_sleep (
    input  wire        clk,
    input  wire        reset,
    input  wire [31:0] operand_a,     // Full 32-bit — core reads [31:24]
    input  wire [31:0] operand_b,     // Full 32-bit — core reads [31:24]
    input  wire [2:0]  alu_op,
    input  wire        core1_enable,
    input  wire        power_gate,    // HIGH = cut power (output zeroed)
    output reg  [31:0] result,        // result in bits [31:24], rest = 0
    output reg         carry_flag,
    output reg         zero_flag
);

// ALU operation codes
localparam ADD  = 3'b000;
localparam SUB  = 3'b001;
localparam AND  = 3'b010;
localparam OR   = 3'b011;
localparam XOR  = 3'b100;
localparam SLT  = 3'b101;

// Extract top 8 bits — MSB truncation
wire [7:0] msb_a = operand_a[31:24];
wire [7:0] msb_b = operand_b[31:24];

// 9-bit result to capture carry
reg [8:0] alu_out;

always @(posedge clk or posedge reset) begin
    if (reset || power_gate) begin
        result     <= 32'b0;
        carry_flag <= 1'b0;
        zero_flag  <= 1'b1;
    end
    else if (core1_enable) begin
        case (alu_op)
            ADD: begin
                alu_out    = {1'b0, msb_a} + {1'b0, msb_b};
                result     <= {alu_out[7:0], 24'b0};
                carry_flag <= alu_out[8];
                zero_flag  <= (alu_out[7:0] == 8'b0);
            end
            SUB: begin
                alu_out    = {1'b0, msb_a} - {1'b0, msb_b};
                result     <= {alu_out[7:0], 24'b0};
                carry_flag <= alu_out[8];
                zero_flag  <= (alu_out[7:0] == 8'b0);
            end
            AND: begin
                result     <= {(msb_a & msb_b), 24'b0};
                carry_flag <= 1'b0;
                zero_flag  <= ((msb_a & msb_b) == 8'b0);
            end
            OR: begin
                result     <= {(msb_a | msb_b), 24'b0};
                carry_flag <= 1'b0;
                zero_flag  <= ((msb_a | msb_b) == 8'b0);
            end
            XOR: begin
                result     <= {(msb_a ^ msb_b), 24'b0};
                carry_flag <= 1'b0;
                zero_flag  <= ((msb_a ^ msb_b) == 8'b0);
            end
            SLT: begin
                result     <= (msb_a < msb_b) ? 32'h01000000 : 32'h0;
                carry_flag <= 1'b0;
                zero_flag  <= ~(msb_a < msb_b);
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
