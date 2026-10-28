// ============================================================
//  PROJECT  : Bio-Signal Driven Multi-Mode RISC-V Processor
//  MODULE   : mode_controller_fsm
//  PURPOSE  : Hardware FSM that decides which core is active
//             based on ECG severity and error signals
//
//  STATES   : SLEEP    → Core 1 active  (~6%  power)
//             MONITOR  → Core 2 active  (~28% power)
//             EMERGENCY→ Core 3 active  (100% power)
//
//  INPUTS   : anomaly_level [1:0]
//               2'b00 = normal     → stay in SLEEP
//               2'b01 = elevated   → go to MONITOR
//               2'b10 = emergency  → go to EMERGENCY
//             ecg_critical  — direct emergency override
//             nmi_request   — hardware NMI (highest priority)
//             error_flag    — Core 2 approximation error too large
//             emergency_done— Core 3 finished processing
//             force_mode    — manual override from user input
//               2'b00 = auto (FSM decides)
//               2'b01 = force SLEEP
//               2'b10 = force MONITOR
//               2'b11 = force EMERGENCY
//
//  AUTHOR   : SASTRA Deemed to be University
//  DATE     : 2026
// ============================================================

module mode_controller_fsm (
    input  wire        clk,
    input  wire        reset,

    // ECG severity signals
    input  wire [1:0]  anomaly_level,   // 00=normal 01=elevated 10=emergency
    input  wire        ecg_critical,    // direct critical flag from analyser
    input  wire        nmi_request,     // Non-Maskable Interrupt
    input  wire        error_flag,      // Core 2 error too large → escalate

    // Core 3 done signal
    input  wire        emergency_done,

    // Manual override (user sets this from switches / testbench)
    input  wire [1:0]  force_mode,      // 00=auto 01=sleep 10=monitor 11=emergency

    // Core enable outputs
    output reg         core1_enable,    // HIGH = Core 1 active
    output reg         core2_enable,    // HIGH = Core 2 active
    output reg         core3_enable,    // HIGH = Core 3 active

    // Power control outputs
    output reg         clock_en_1,      // Core 1 clock enable → ICG cell
    output reg         clock_en_2,      // Core 2 clock enable
    output reg         power_gate_1,    // Core 1 power gate (PMOS header)

    // NMI control
    output reg         nmi_trigger,     // Pulse to Core 3 NMI port

    // Status outputs (visible in waveform)
    output reg  [1:0]  current_mode,    // 00=SLEEP 01=MONITOR 10=EMERGENCY
    output reg  [7:0]  mode_timer,      // cycles spent in current mode
    output reg         mode_changed     // 1-cycle pulse on any mode transition
);

// ── STATE ENCODING ─────────────────────────────────────────
localparam SLEEP     = 2'b00;
localparam MONITOR   = 2'b01;
localparam EMERGENCY = 2'b10;

// ── INTERNAL REGISTERS ─────────────────────────────────────
reg [1:0] state;
reg [1:0] next_state;
reg [7:0] stable_counter;  // counts cycles in same state
                            // must stay elevated for 3 cycles before escalating
                            // must stay normal for 10 cycles before de-escalating

// ── HYSTERESIS THRESHOLDS ──────────────────────────────────
// Prevents rapid mode bouncing on noisy ECG signals
localparam ESCALATE_THRESHOLD   = 8'd3;   // 3 cycles elevated → switch up
localparam DEESCALATE_THRESHOLD = 8'd10;  // 10 cycles normal  → switch down

// ──────────────────────────────────────────────────────────
// STATE REGISTER (sequential)
// ──────────────────────────────────────────────────────────
always @(posedge clk or posedge reset) begin
    if (reset) begin
        state           <= SLEEP;
        stable_counter  <= 8'd0;
        mode_timer      <= 8'd0;
        mode_changed    <= 1'b0;
    end
    else begin
        // Detect mode change
        if (state != next_state) begin
            mode_changed   <= 1'b1;
            mode_timer     <= 8'd0;
            stable_counter <= 8'd0;
        end
        else begin
            mode_changed <= 1'b0;
            mode_timer   <= (mode_timer == 8'hFF) ? 8'hFF : mode_timer + 1;
            stable_counter <= (stable_counter == 8'hFF) ? 8'hFF
                             : stable_counter + 1;
        end
        state <= next_state;
    end
end

// ──────────────────────────────────────────────────────────
// NEXT STATE LOGIC (combinational)
// ──────────────────────────────────────────────────────────
always @(*) begin
    next_state = state; // default: stay

    // Manual override has highest priority after NMI
    if (force_mode != 2'b00) begin
        case (force_mode)
            2'b01:   next_state = SLEEP;
            2'b10:   next_state = MONITOR;
            2'b11:   next_state = EMERGENCY;
            default: next_state = state;
        endcase
    end
    // NMI is absolute override — cannot be blocked
    else if (nmi_request || ecg_critical) begin
        next_state = EMERGENCY;
    end
    else begin
        case (state)
            SLEEP: begin
                // Escalate to MONITOR if anomaly_level elevated
                // for at least ESCALATE_THRESHOLD cycles
                if (anomaly_level == 2'b01 &&
                    stable_counter >= ESCALATE_THRESHOLD)
                    next_state = MONITOR;
                // Jump straight to EMERGENCY
                else if (anomaly_level == 2'b10)
                    next_state = EMERGENCY;
                else
                    next_state = SLEEP;
            end

            MONITOR: begin
                // Escalate to EMERGENCY
                if (anomaly_level == 2'b10 || error_flag)
                    next_state = EMERGENCY;
                // De-escalate back to SLEEP if normal for long enough
                else if (anomaly_level == 2'b00 &&
                         stable_counter >= DEESCALATE_THRESHOLD)
                    next_state = SLEEP;
                else
                    next_state = MONITOR;
            end

            EMERGENCY: begin
                // Only leave EMERGENCY when Core 3 signals done
                // AND anomaly has cleared
                if (emergency_done && anomaly_level == 2'b00 &&
                    !ecg_critical && !nmi_request)
                    next_state = SLEEP;
                else if (emergency_done && anomaly_level == 2'b01)
                    next_state = MONITOR;
                else
                    next_state = EMERGENCY;
            end

            default: next_state = SLEEP;
        endcase
    end
end

// ──────────────────────────────────────────────────────────
// OUTPUT LOGIC (registered for clean transitions)
// ──────────────────────────────────────────────────────────
always @(posedge clk or posedge reset) begin
    if (reset) begin
        core1_enable  <= 1'b1;   // Core 1 active by default
        core2_enable  <= 1'b0;
        core3_enable  <= 1'b0;
        clock_en_1    <= 1'b1;
        clock_en_2    <= 1'b0;
        power_gate_1  <= 1'b0;
        nmi_trigger   <= 1'b0;
        current_mode  <= SLEEP;
    end
    else begin
        current_mode <= next_state;
        nmi_trigger  <= (nmi_request || ecg_critical) ? 1'b1 : 1'b0;

        case (next_state)
            SLEEP: begin
                // Core 1 ON, others OFF
                core1_enable <= 1'b1;
                core2_enable <= 1'b0;
                core3_enable <= 1'b0;
                clock_en_1   <= 1'b1;
                clock_en_2   <= 1'b0;
                power_gate_1 <= 1'b0;  // Core 1 powered
            end

            MONITOR: begin
                // Core 2 ON, Core 1 clock-gated
                core1_enable <= 1'b0;
                core2_enable <= 1'b1;
                core3_enable <= 1'b0;
                clock_en_1   <= 1'b0;  // Gate Core 1 clock
                clock_en_2   <= 1'b1;
                power_gate_1 <= 1'b0;  // Keep Core 1 power for fast wake
            end

            EMERGENCY: begin
                // Core 3 ON, Core 1 power-gated, Core 2 clock-gated
                core1_enable <= 1'b0;
                core2_enable <= 1'b0;
                core3_enable <= 1'b1;
                clock_en_1   <= 1'b0;
                clock_en_2   <= 1'b0;
                power_gate_1 <= 1'b1;  // Full power gate Core 1
            end

            default: begin
                core1_enable <= 1'b1;
                core2_enable <= 1'b0;
                core3_enable <= 1'b0;
                clock_en_1   <= 1'b1;
                clock_en_2   <= 1'b0;
                power_gate_1 <= 1'b0;
            end
        endcase
    end
end

endmodule
