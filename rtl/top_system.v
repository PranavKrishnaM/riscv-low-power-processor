// ============================================================
//  PROJECT  : Bio-Signal Driven Multi-Mode RISC-V Processor
//  MODULE   : top_system
//  PURPOSE  : Top-level integration of all 3 cores + FSM
//             Supports MANUAL INPUT — user drives all inputs
//             directly from testbench or FPGA switches
//
//  MANUAL INPUTS:
//    ecg_amplitude [7:0]  — type the ECG value (0–255)
//    anomaly_level [1:0]  — 00=normal 01=elevated 10=emergency
//    alu_op        [2:0]  — operation (000=ADD 001=SUB etc.)
//    operand_b     [31:0] — second operand value
//    force_mode    [1:0]  — 00=auto 01=sleep 10=monitor 11=emerg
//    nmi_request          — pulse HIGH to trigger NMI
//    ecg_critical         — pulse HIGH for critical override
//
//  OUTPUTS (check these in waveform):
//    active_result [31:0] — result from whichever core is active
//    active_mode   [1:0]  — 00=SLEEP 01=MONITOR 10=EMERGENCY
//    active_core   [1:0]  — which core produced the result
//    carry_out            — carry/overflow flag
//    zero_out             — zero flag
//    overflow_out         — signed overflow flag
//    negative_out         — sign bit
//    mode_changed         — pulses HIGH on every mode switch
//
//  AUTHOR   : SASTRA Deemed to be University
//  DATE     : 2026
// ============================================================

module top_system (
    // ── CLOCK AND RESET ──────────────────────────────────
    input  wire        clk,
    input  wire        reset,

    // ── MANUAL USER INPUTS ───────────────────────────────
    input  wire [7:0]  ecg_amplitude,   // ECG sample value (0-255)
    input  wire [1:0]  anomaly_level,   // 00=normal 01=elevated 10=emergency
    input  wire [2:0]  alu_op,          // ALU operation select
    input  wire [31:0] operand_b,       // Second operand (full 32-bit)
    input  wire [1:0]  force_mode,      // Manual mode override
    input  wire        nmi_request,     // Trigger emergency NMI
    input  wire        ecg_critical,    // Direct critical flag

    // ── OUTPUTS ──────────────────────────────────────────
    output wire [31:0] active_result,   // Result from active core
    output wire [1:0]  active_mode,     // Current operating mode
    output wire [1:0]  active_core,     // Which core is computing
    output wire        carry_out,       // Carry flag
    output wire        zero_out,        // Zero flag
    output wire        overflow_out,    // Overflow flag
    output wire        negative_out,    // Negative flag
    output wire        mode_changed,    // Mode transition pulse
    output wire [7:0]  mode_timer,      // Cycles in current mode
    output wire        nmi_ack,         // NMI acknowledged

    // ── DEBUG OUTPUTS (visible in waveform) ──────────────
    output wire [31:0] c1_result_dbg,   // Core 1 result (always visible)
    output wire [31:0] c2_result_dbg,   // Core 2 result (always visible)
    output wire [31:0] c3_result_dbg,   // Core 3 result (always visible)
    output wire        c1_clk_gated,    // Core 1 clock gate status
    output wire        c2_clk_gated,    // Core 2 clock gate status
    output wire        c2_error_flag,   // Core 2 approximation error flag
    output wire        c3_emergency_done// Core 3 computation done
);

// ──────────────────────────────────────────────────────────
// INPUT PACKING
// ECG amplitude is packed into top 8 bits of 32-bit operand_a
// So Core 1 (8-bit) reads it directly
//    Core 2 (16-bit) reads it as amplitude × 256
//    Core 3 (32-bit) reads the full packed value
// ──────────────────────────────────────────────────────────
wire [31:0] operand_a;
assign operand_a = {ecg_amplitude, 24'b0};  // Pack into MSB

// ──────────────────────────────────────────────────────────
// INTERNAL WIRES — FSM OUTPUTS
// ──────────────────────────────────────────────────────────
wire        core1_enable;
wire        core2_enable;
wire        core3_enable;
wire        clock_en_1;
wire        clock_en_2;
wire        power_gate_1;
wire        nmi_trigger_fsm;
wire [1:0]  current_mode_wire;
wire [7:0]  mode_timer_wire;
wire        mode_changed_wire;

// ──────────────────────────────────────────────────────────
// INTERNAL WIRES — CORE OUTPUTS
// ──────────────────────────────────────────────────────────

// Core 1 outputs
wire [31:0] c1_result;
wire        c1_carry;
wire        c1_zero;

// Core 2 outputs
wire [31:0] c2_result;
wire        c2_carry;
wire        c2_zero;
wire        c2_error_flag_wire;

// Core 3 outputs
wire [31:0] c3_result;
wire        c3_carry;
wire        c3_zero;
wire        c3_overflow;
wire        c3_negative;
wire        c3_done;
wire        nmi_ack_wire;
wire [2:0]  nmi_state_wire;

// Clock gating (ICG)
assign c1_clk_gated = clock_en_1 & ~power_gate_1;
assign c2_clk_gated = clock_en_2;

// ──────────────────────────────────────────────────────────
// MODULE 1 — MODE CONTROLLER FSM
// ──────────────────────────────────────────────────────────
mode_controller_fsm FSM (
    .clk            (clk),
    .reset          (reset),
    .anomaly_level  (anomaly_level),
    .ecg_critical   (ecg_critical),
    .nmi_request    (nmi_request),
    .error_flag     (c2_error_flag_wire),
    .emergency_done (c3_done),
    .force_mode     (force_mode),
    .core1_enable   (core1_enable),
    .core2_enable   (core2_enable),
    .core3_enable   (core3_enable),
    .clock_en_1     (clock_en_1),
    .clock_en_2     (clock_en_2),
    .power_gate_1   (power_gate_1),
    .nmi_trigger    (nmi_trigger_fsm),
    .current_mode   (current_mode_wire),
    .mode_timer     (mode_timer_wire),
    .mode_changed   (mode_changed_wire)
);

// ──────────────────────────────────────────────────────────
// MODULE 2 — CORE 1 (Sleep Mode, 8-bit MSB)
// ──────────────────────────────────────────────────────────
core1_sleep CORE1 (
    .clk          (clk & c1_clk_gated),   // gated clock
    .reset        (reset),
    .operand_a    (operand_a),
    .operand_b    (operand_b),
    .alu_op       (alu_op),
    .core1_enable (core1_enable),
    .power_gate   (power_gate_1),
    .result       (c1_result),
    .carry_flag   (c1_carry),
    .zero_flag    (c1_zero)
);

// ──────────────────────────────────────────────────────────
// MODULE 3 — CORE 2 (Monitor Mode, 16-bit Approximate)
// ──────────────────────────────────────────────────────────
core2_monitor CORE2 (
    .clk          (clk & c2_clk_gated),   // gated clock
    .reset        (reset),
    .operand_a    (operand_a),
    .operand_b    (operand_b),
    .alu_op       (alu_op),
    .core2_enable (core2_enable),
    .approx_level (2'b01),                // 16-bit default
    .result       (c2_result),
    .carry_flag   (c2_carry),
    .zero_flag    (c2_zero),
    .error_flag   (c2_error_flag_wire)
);

// ──────────────────────────────────────────────────────────
// MODULE 4 — CORE 3 (Emergency Mode, 32-bit Exact)
// ──────────────────────────────────────────────────────────
core3_emergency CORE3 (
    .clk             (clk),
    .reset           (reset),
    .nmi_request     (nmi_trigger_fsm),
    .nmi_acknowledge (nmi_ack_wire),
    .nmi_state       (nmi_state_wire),
    .operand_a       (operand_a),
    .operand_b       (operand_b),
    .alu_op          (alu_op),
    .core3_enable    (core3_enable),
    .rs1             (5'b0),
    .rs2             (5'b0),
    .rd              (5'b0),
    .reg_write       (1'b0),
    .result          (c3_result),
    .carry_flag      (c3_carry),
    .zero_flag       (c3_zero),
    .overflow_flag   (c3_overflow),
    .negative_flag   (c3_negative),
    .emergency_done  (c3_done),
    .reg_rs1_data    (),
    .reg_rs2_data    ()
);

// ──────────────────────────────────────────────────────────
// OUTPUT MUX — select active core result
// ──────────────────────────────────────────────────────────
reg [31:0] mux_result;
reg        mux_carry;
reg        mux_zero;
reg        mux_overflow;
reg        mux_negative;
reg [1:0]  mux_core;

always @(*) begin
    case (current_mode_wire)
        2'b00: begin   // SLEEP — Core 1
            mux_result   = c1_result;
            mux_carry    = c1_carry;
            mux_zero     = c1_zero;
            mux_overflow = 1'b0;
            mux_negative = c1_result[7];
            mux_core     = 2'b01;
        end
        2'b01: begin   // MONITOR — Core 2
            mux_result   = c2_result;
            mux_carry    = c2_carry;
            mux_zero     = c2_zero;
            mux_overflow = 1'b0;
            mux_negative = c2_result[15];
            mux_core     = 2'b10;
        end
        2'b10: begin   // EMERGENCY — Core 3
            mux_result   = c3_result;
            mux_carry    = c3_carry;
            mux_zero     = c3_zero;
            mux_overflow = c3_overflow;
            mux_negative = c3_negative;
            mux_core     = 2'b11;
        end
        default: begin
            mux_result   = c1_result;
            mux_carry    = c1_carry;
            mux_zero     = c1_zero;
            mux_overflow = 1'b0;
            mux_negative = 1'b0;
            mux_core     = 2'b01;
        end
    endcase
end

// ──────────────────────────────────────────────────────────
// CONNECT OUTPUTS
// ──────────────────────────────────────────────────────────
assign active_result       = mux_result;
assign active_mode         = current_mode_wire;
assign active_core         = mux_core;
assign carry_out           = mux_carry;
assign zero_out            = mux_zero;
assign overflow_out        = mux_overflow;
assign negative_out        = mux_negative;
assign mode_changed        = mode_changed_wire;
assign mode_timer          = mode_timer_wire;
assign nmi_ack             = nmi_ack_wire;
assign c1_result_dbg       = c1_result;
assign c2_result_dbg       = c2_result;
assign c3_result_dbg       = c3_result;
assign c2_error_flag       = c2_error_flag_wire;
assign c3_emergency_done   = c3_done;

endmodule
