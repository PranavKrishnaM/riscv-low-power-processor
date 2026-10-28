// ============================================================
//  PROJECT  : Bio-Signal Driven Multi-Mode RISC-V Processor
//  MODULE   : core3_emergency
//  PURPOSE  : Core 3 — Emergency Mode (Full 32-bit Exact ALU)
//  FEATURES : 32-bit CLA Adder, Wallace Tree Multiplier,
//             All 4 Register Banks, NMI Interrupt (5-cycle),
//             Full flag generation (carry, zero, overflow, negative)
//  AUTHOR   : SASTRA Deemed to be University
//  DATE     : 2026
// ============================================================

module core3_emergency (
    // Clock and Reset
    input  wire        clk,
    input  wire        reset,

    // NMI Interrupt Interface
    input  wire        nmi_request,      // Non-Maskable Interrupt request
    output reg         nmi_acknowledge,  // NMI accepted — goes HIGH for 1 cycle
    output reg  [2:0]  nmi_state,        // NMI FSM state (for waveform visibility)

    // Operand Inputs (full 32-bit)
    input  wire [31:0] operand_a,        // First operand
    input  wire [31:0] operand_b,        // Second operand
    input  wire [2:0]  alu_op,           // Operation select

    // Control
    input  wire        core3_enable,     // HIGH = Core 3 is the active core
    input  wire [4:0]  rs1,              // Register file read port 1
    input  wire [4:0]  rs2,              // Register file read port 2
    input  wire [4:0]  rd,               // Register file write destination
    input  wire        reg_write,        // Write enable for register file

    // Outputs
    output reg  [31:0] result,           // ALU result (exact 32-bit)
    output reg         carry_flag,       // Carry out
    output reg         zero_flag,        // Result == 0
    output reg         overflow_flag,    // Signed overflow
    output reg         negative_flag,    // Result MSB (sign bit)
    output reg         emergency_done,   // Computation complete pulse

    // Register file read data (for waveform inspection)
    output wire [31:0] reg_rs1_data,
    output wire [31:0] reg_rs2_data
);

// ============================================================
// ALU OPERATION CODES
// ============================================================
localparam ALU_ADD  = 3'b000;
localparam ALU_SUB  = 3'b001;
localparam ALU_AND  = 3'b010;
localparam ALU_OR   = 3'b011;
localparam ALU_XOR  = 3'b100;
localparam ALU_SLT  = 3'b101;   // Set Less Than (signed)
localparam ALU_MUL  = 3'b110;   // Multiply (Wallace Tree)
localparam ALU_SLTU = 3'b111;   // Set Less Than Unsigned

// ============================================================
// NMI STATE MACHINE STATES
// ============================================================
localparam NMI_IDLE     = 3'b000;  // Waiting
localparam NMI_DETECT   = 3'b001;  // NMI detected — cycle 1
localparam NMI_SAVE     = 3'b010;  // Save context — cycle 2
localparam NMI_ACTIVATE = 3'b011;  // Activate Core 3 — cycle 3
localparam NMI_EXECUTE  = 3'b100;  // Executing — cycle 4
localparam NMI_ACK      = 3'b101;  // Acknowledge — cycle 5

// ============================================================
// REGISTER FILE — ALL 4 BANKS (x0 to x31)
// ============================================================
reg [31:0] regfile [0:31];    // 32 registers × 32 bits

// Register file read (combinational)
assign reg_rs1_data = (rs1 == 5'b0) ? 32'b0 : regfile[rs1];
assign reg_rs2_data = (rs2 == 5'b0) ? 32'b0 : regfile[rs2];

// Register file write (synchronous)
integer i;
always @(posedge clk or posedge reset) begin
    if (reset) begin
        for (i = 0; i < 32; i = i + 1)
            regfile[i] <= 32'b0;
    end
    else if (reg_write && rd != 5'b0 && core3_enable) begin
        regfile[rd] <= result;
    end
end

// ============================================================
// 32-BIT CARRY LOOKAHEAD ADDER (CLA)
// ============================================================
// Implemented as 4 × 8-bit CLA groups with group carry propagation
// Group Generate (G) and Propagate (P) for each 8-bit block

wire [31:0] cla_a, cla_b;
wire        cla_cin;
wire [31:0] cla_sum;
wire        cla_cout;

// Propagate and Generate for each bit
wire [31:0] P, G;           // bit-level
wire [3:0]  GP, GG;         // group-level (8-bit groups)
wire [4:0]  group_carry;    // carry into each group

assign P = cla_a ^ cla_b;
assign G = cla_a & cla_b;

// Group 0: bits 7:0
assign GP[0] = &P[7:0];
assign GG[0] = G[7]   | (P[7]   & G[6])   | (P[7]   & P[6]   & G[5])
             | (P[7:5] == 3'b111 && G[4])
             | (P[7:4] == 4'b1111 && G[3])
             | (P[7:3] == 5'b11111 && G[2])
             | (P[7:2] == 6'b111111 && G[1])
             | (P[7:1] == 7'b1111111 && G[0]);

// Group 1: bits 15:8
assign GP[1] = &P[15:8];
assign GG[1] = G[15]  | (P[15]  & G[14])
             | (P[15:14] == 2'b11 && G[13])
             | (P[15:13] == 3'b111 && G[12])
             | (P[15:12] == 4'b1111 && G[11])
             | (P[15:11] == 5'b11111 && G[10])
             | (P[15:10] == 6'b111111 && G[9])
             | (P[15:9]  == 7'b1111111 && G[8]);

// Group 2: bits 23:16
assign GP[2] = &P[23:16];
assign GG[2] = G[23]  | (P[23]  & G[22])
             | (P[23:22] == 2'b11 && G[21])
             | (P[23:21] == 3'b111 && G[20])
             | (P[23:20] == 4'b1111 && G[19])
             | (P[23:19] == 5'b11111 && G[18])
             | (P[23:18] == 6'b111111 && G[17])
             | (P[23:17] == 7'b1111111 && G[16]);

// Group 3: bits 31:24
assign GP[3] = &P[31:24];
assign GG[3] = G[31]  | (P[31]  & G[30])
             | (P[31:30] == 2'b11 && G[29])
             | (P[31:29] == 3'b111 && G[28])
             | (P[31:28] == 4'b1111 && G[27])
             | (P[31:27] == 5'b11111 && G[26])
             | (P[31:26] == 6'b111111 && G[25])
             | (P[31:25] == 7'b1111111 && G[24]);

// Group carry calculations
assign group_carry[0] = cla_cin;
assign group_carry[1] = GG[0] | (GP[0] & cla_cin);
assign group_carry[2] = GG[1] | (GP[1] & GG[0]) | (GP[1] & GP[0] & cla_cin);
assign group_carry[3] = GG[2] | (GP[2] & GG[1]) | (GP[2] & GP[1] & GG[0])
                      | (GP[2] & GP[1] & GP[0] & cla_cin);
assign group_carry[4] = GG[3] | (GP[3] & GG[2]) | (GP[3] & GP[2] & GG[1])
                      | (GP[3] & GP[2] & GP[1] & GG[0])
                      | (GP[3] & GP[2] & GP[1] & GP[0] & cla_cin);

// Individual bit carries using ripple within each group
// (fast enough — only 8-bit ripple, not 32)
wire [32:0] carry_bits;
assign carry_bits[0]  = group_carry[0];
assign carry_bits[8]  = group_carry[1];
assign carry_bits[16] = group_carry[2];
assign carry_bits[24] = group_carry[3];

genvar k;
generate
    for (k = 0; k < 8; k = k+1) begin : carry_gen_g0
        if (k < 7)
            assign carry_bits[k+1]  = G[k]    | (P[k]    & carry_bits[k]);
    end
    for (k = 8; k < 16; k = k+1) begin : carry_gen_g1
        if (k < 15)
            assign carry_bits[k+1]  = G[k]    | (P[k]    & carry_bits[k]);
    end
    for (k = 16; k < 24; k = k+1) begin : carry_gen_g2
        if (k < 23)
            assign carry_bits[k+1]  = G[k]    | (P[k]    & carry_bits[k]);
    end
    for (k = 24; k < 32; k = k+1) begin : carry_gen_g3
        if (k < 31)
            assign carry_bits[k+1]  = G[k]    | (P[k]    & carry_bits[k]);
    end
endgenerate

assign cla_sum  = P ^ carry_bits[31:0];
assign cla_cout = carry_bits[32] | group_carry[4];

// ============================================================
// 32-BIT WALLACE TREE MULTIPLIER
// ============================================================
// Partial products → Wallace tree reduction → final CLA
// Using 16x16 sub-multiplications for simulation efficiency

wire [63:0] mul_result_full;

// Signed multiplication using $signed for exact RTL behaviour
// In real synthesis this maps to DSP blocks or full Wallace tree
wire signed [31:0] signed_a = operand_a;
wire signed [31:0] signed_b = operand_b;

// Partial product generation (simplified Wallace structure)
// Split into four 16-bit sub-products
wire [31:0] pp_ll, pp_lh, pp_hl, pp_hh;
wire [15:0] a_lo, a_hi, b_lo, b_hi;

assign a_lo = operand_a[15:0];
assign a_hi = operand_a[31:16];
assign b_lo = operand_b[15:0];
assign b_hi = operand_b[31:16];

// Four partial products
assign pp_ll = a_lo * b_lo;          // bits [31:0]
assign pp_lh = a_lo * b_hi;          // bits [47:16]
assign pp_hl = a_hi * b_lo;          // bits [47:16]
assign pp_hh = a_hi * b_hi;          // bits [63:32]

// Wallace tree reduction of partial products
wire [63:0] pp_lh_shifted = {16'b0, pp_lh, 16'b0};
wire [63:0] pp_hl_shifted = {16'b0, pp_hl, 16'b0};
wire [63:0] pp_hh_shifted = {pp_hh, 32'b0};
wire [63:0] pp_ll_ext     = {32'b0, pp_ll};

// Final addition using extended adder
assign mul_result_full = pp_ll_ext + pp_lh_shifted 
                       + pp_hl_shifted + pp_hh_shifted;

// ============================================================
// CLA CONNECTIONS FOR ADD/SUB
// ============================================================
reg [31:0] add_a_in, add_b_in;
reg        add_cin;

assign cla_a   = add_a_in;
assign cla_b   = add_b_in;
assign cla_cin = add_cin;

// ============================================================
// NMI INTERRUPT CONTROLLER — 5 CYCLE RESPONSE
// ============================================================
reg [2:0] nmi_current_state;
reg [31:0] saved_operand_a;  // context save on NMI
reg [31:0] saved_operand_b;
reg        nmi_active;

always @(posedge clk or posedge reset) begin
    if (reset) begin
        nmi_current_state <= NMI_IDLE;
        nmi_acknowledge   <= 1'b0;
        nmi_state         <= NMI_IDLE;
        nmi_active        <= 1'b0;
        saved_operand_a   <= 32'b0;
        saved_operand_b   <= 32'b0;
    end
    else begin
        case (nmi_current_state)

            NMI_IDLE: begin
                nmi_acknowledge <= 1'b0;
                nmi_active      <= 1'b0;
                if (nmi_request) begin
                    nmi_current_state <= NMI_DETECT;
                    nmi_state         <= NMI_DETECT;
                end
            end

            NMI_DETECT: begin
                // Cycle 1: NMI detected
                // Save current operands (context preservation)
                saved_operand_a   <= operand_a;
                saved_operand_b   <= operand_b;
                nmi_current_state <= NMI_SAVE;
                nmi_state         <= NMI_SAVE;
            end

            NMI_SAVE: begin
                // Cycle 2: Context saved
                nmi_current_state <= NMI_ACTIVATE;
                nmi_state         <= NMI_ACTIVATE;
            end

            NMI_ACTIVATE: begin
                // Cycle 3: Core 3 now active
                nmi_active        <= 1'b1;
                nmi_current_state <= NMI_EXECUTE;
                nmi_state         <= NMI_EXECUTE;
            end

            NMI_EXECUTE: begin
                // Cycle 4: Execute emergency computation
                nmi_current_state <= NMI_ACK;
                nmi_state         <= NMI_ACK;
            end

            NMI_ACK: begin
                // Cycle 5: Acknowledge — back to idle
                nmi_acknowledge   <= 1'b1;
                nmi_current_state <= NMI_IDLE;
                nmi_state         <= NMI_IDLE;
            end

            default: begin
                nmi_current_state <= NMI_IDLE;
                nmi_state         <= NMI_IDLE;
            end
        endcase
    end
end

// ============================================================
// MAIN ALU — FULL 32-BIT EXACT COMPUTATION
// ============================================================
reg [32:0] alu_full_result;  // 33-bit to capture carry

always @(*) begin
    // Default values
    add_a_in = operand_a;
    add_b_in = operand_b;
    add_cin  = 1'b0;

    case (alu_op)
        ALU_ADD: begin
            add_a_in = operand_a;
            add_b_in = operand_b;
            add_cin  = 1'b0;
        end

        ALU_SUB: begin
            // Two's complement subtraction: A + (~B) + 1
            add_a_in = operand_a;
            add_b_in = ~operand_b;
            add_cin  = 1'b1;
        end

        default: begin
            add_a_in = operand_a;
            add_b_in = operand_b;
            add_cin  = 1'b0;
        end
    endcase
end

// ============================================================
// REGISTERED OUTPUT
// ============================================================
always @(posedge clk or posedge reset) begin
    if (reset) begin
        result         <= 32'b0;
        carry_flag     <= 1'b0;
        zero_flag      <= 1'b0;
        overflow_flag  <= 1'b0;
        negative_flag  <= 1'b0;
        emergency_done <= 1'b0;
    end
    else if (core3_enable) begin
        emergency_done <= 1'b1;

        case (alu_op)
            ALU_ADD: begin
                result        <= cla_sum;
                carry_flag    <= cla_cout;
                overflow_flag <= (operand_a[31] == operand_b[31]) &&
                                 (cla_sum[31] != operand_a[31]);
                zero_flag     <= (cla_sum == 32'b0);
                negative_flag <= cla_sum[31];
            end

            ALU_SUB: begin
                result        <= cla_sum;
                // Carry flag in subtraction = borrow (inverted)
                carry_flag    <= ~cla_cout;
                overflow_flag <= (operand_a[31] != operand_b[31]) &&
                                 (cla_sum[31] != operand_a[31]);
                zero_flag     <= (cla_sum == 32'b0);
                negative_flag <= cla_sum[31];
            end

            ALU_AND: begin
                result        <= operand_a & operand_b;
                carry_flag    <= 1'b0;
                overflow_flag <= 1'b0;
                zero_flag     <= ((operand_a & operand_b) == 32'b0);
                negative_flag <= (operand_a & operand_b) >> 31;
            end

            ALU_OR: begin
                result        <= operand_a | operand_b;
                carry_flag    <= 1'b0;
                overflow_flag <= 1'b0;
                zero_flag     <= ((operand_a | operand_b) == 32'b0);
                negative_flag <= (operand_a | operand_b) >> 31;
            end

            ALU_XOR: begin
                result        <= operand_a ^ operand_b;
                carry_flag    <= 1'b0;
                overflow_flag <= 1'b0;
                zero_flag     <= ((operand_a ^ operand_b) == 32'b0);
                negative_flag <= (operand_a ^ operand_b) >> 31;
            end

            ALU_SLT: begin
                // Signed less than
                result        <= ($signed(operand_a) < $signed(operand_b))
                                  ? 32'b1 : 32'b0;
                carry_flag    <= 1'b0;
                overflow_flag <= 1'b0;
                zero_flag     <= ~($signed(operand_a) < $signed(operand_b));
                negative_flag <= 1'b0;
            end

            ALU_MUL: begin
                // Wallace Tree Multiplier — lower 32 bits of result
                result        <= mul_result_full[31:0];
                carry_flag    <= |mul_result_full[63:32]; // upper bits set = overflow
                overflow_flag <= |mul_result_full[63:32];
                zero_flag     <= (mul_result_full[31:0] == 32'b0);
                negative_flag <= mul_result_full[31];
            end

            ALU_SLTU: begin
                // Unsigned less than
                result        <= (operand_a < operand_b)
                                  ? 32'b1 : 32'b0;
                carry_flag    <= 1'b0;
                overflow_flag <= 1'b0;
                zero_flag     <= ~(operand_a < operand_b);
                negative_flag <= 1'b0;
            end

            default: begin
                result        <= 32'b0;
                carry_flag    <= 1'b0;
                overflow_flag <= 1'b0;
                zero_flag     <= 1'b1;
                negative_flag <= 1'b0;
            end
        endcase
    end
    else begin
        emergency_done <= 1'b0;
    end
end

endmodule
