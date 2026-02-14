`timescale 1ns / 1ps
`default_nettype none
module VIA (
    input            phi2,     // Clock
    input            rst_n,    // Reset
    input            cs1,      // Chip Selects
    input            cs2_n,         
    input            rw,       // Read/Write (High=Read, Low=Write)
    input      [3:0] rs,       // Register Select (A0-A3)
    input      [7:0] data_in,  // Data bus from CPU
    output reg [7:0] data_out, 
    
    // Peripheral Ports
    inout      [7:0] port_a,
    inout      [7:0] port_b,
    input            ca1,
    input            cb1_in,
    input            ca2_in,
    input            cb2_in,
    output           cb1_out,
    output           ca2_out,
    output           cb2_out,
    output           irq_n         // Interrupt Request
);

    // Internal Registers
    reg  [7:0] ora;
    reg  [7:0] orb;
    reg  [7:0] ddra;
    reg  [7:0] ddrb;
    reg  [7:0] acr;
    reg  [7:0] pcr;
    reg  [7:0] ifr;
    reg  [7:0] ier;
    reg [15:0] t1_c;
    reg [15:0] t1_l;
    reg [15:0] t2_c;
    reg  [7:0] t2_l;
    reg  [7:0] sr;  // Shift Register
    reg        t1_pb7; // Timer 1 output to PB7
    
    // Edge detection for control lines
    reg ca1_r, cb1_r, ca2_r, cb2_r;
    
    // Interrupt flag control signals
    reg set_t1_ifr, clr_t1_ifr;
    reg set_t2_ifr, clr_t2_ifr;
    reg set_ca1_ifr, set_ca2_ifr, clr_ca_ifr;
    reg set_cb1_ifr, set_cb2_ifr, clr_cb_ifr;
    
    // Pulse output state
    reg ca2_pulse, cb2_pulse;
    
    wire chip_sel = cs1 && !cs2_n;

    // --- 1. Port I/O Logic ---
    // Tristate buffers: drive only if DDR bit is 1
    generate
        genvar i;
        for (i = 0; i < 8; i = i + 1) begin : port_io
            assign port_a[i] = ddra[i] ? ora[i] : 1'bz;
            // PB7 can be driven by Timer 1 when ACR bit 7 is set
            assign port_b[i] = (i == 7 && acr[7]) ? t1_pb7 : 
                               (ddrb[i] ? orb[i] : 1'bz);
        end
    endgenerate

    // --- 2. Timer 1 Logic (Free-run/One-shot) ---
    always @(posedge phi2 or negedge rst_n) begin
        if (!rst_n) begin
            t1_c <= 16'hFFFF;
            t1_pb7 <= 0;
            set_t1_ifr <= 0;
            clr_t1_ifr <= 0;
        end else begin
            set_t1_ifr <= 0;
            clr_t1_ifr <= 0;
            
            // Writing T1C-H (rs=5) loads counter
            if (chip_sel && !rw && rs == 4'h5) begin
                clr_t1_ifr <= 1;
                t1_c <= {data_in, t1_l[7:0]};
                t1_pb7 <= 0; // Reset PB7 on load
            end
            // Reading T1C-L (rs=4) clears the interrupt flag  
            else if (chip_sel && rw && rs == 4'h4) begin
                clr_t1_ifr <= 1;
            end
            // Timer countdown
            else if (t1_c == 0) begin
                set_t1_ifr <= 1; // Set T1 Interrupt Flag
                t1_c <= acr[6] ? t1_l : 16'hFFFF; // Reload if Free-run (ACR bit 6)
                t1_pb7 <= ~t1_pb7; // Toggle PB7 output
            end else begin
                t1_c <= t1_c - 1;
            end
        end
    end
    
    // --- 3. Timer 2 Logic (Count-down mode) ---
    wire t2_pulse_count = (acr[5] == 1'b1);  // ACR bit 5: 0=one-shot, 1=pulse counting
    reg pb6_r;
    wire pb6_edge = ~pb6_r & port_b[6];  // Positive edge on PB6
    
    always @(posedge phi2 or negedge rst_n) begin
        if (!rst_n) begin
            t2_c <= 16'hFFFF;
            t2_l <= 8'hFF;
            set_t2_ifr <= 0;
            clr_t2_ifr <= 0;
            pb6_r <= 0;
        end else begin
            pb6_r <= port_b[6];
            set_t2_ifr <= 0;
            clr_t2_ifr <= 0;
            
            // Writing T2C-H (rs=9) loads counter
            if (chip_sel && !rw && rs == 4'h9) begin
                clr_t2_ifr <= 1;
                t2_c <= {data_in, t2_l};
            end
            // Writing T2C-L (rs=8) stores latch value only
            else if (chip_sel && !rw && rs == 4'h8) begin
                t2_l <= data_in;
            end
            // Reading T2C-L (rs=8) clears interrupt
            else if (chip_sel && rw && rs == 4'h8) begin
                clr_t2_ifr <= 1;
            end
            // Timer 2 countdown logic
            else begin
                // Timer 2 counts down on phi2 or on PB6 pulses
                if (t2_pulse_count) begin
                    // Pulse counting mode - count on PB6 positive edges
                    if (pb6_edge && t2_c != 0) begin
                        if (t2_c == 1) begin
                            set_t2_ifr <= 1;
                        end
                        t2_c <= t2_c - 1;
                    end
                end else begin
                    // One-shot mode - count on phi2
                    if (t2_c != 0) begin
                        if (t2_c == 1) begin
                            set_t2_ifr <= 1;
                        end
                        t2_c <= t2_c - 1;
                    end
                end
            end
        end
    end
    
    // --- 4. Control Line Interrupt Logic (CA1, CA2, CB1, CB2) ---
    always @(posedge phi2 or negedge rst_n) begin
        if (!rst_n) begin
            ca1_r <= 0;
            ca2_r <= 0;
            cb1_r <= 0;
            cb2_r <= 0;
            set_ca1_ifr <= 0;
            set_ca2_ifr <= 0;
            set_cb1_ifr <= 0;
            set_cb2_ifr <= 0;
            clr_ca_ifr <= 0;
            clr_cb_ifr <= 0;
        end else begin
            // Store previous values for edge detection
            ca1_r <= ca1;
            ca2_r <= ca2_in;
            cb1_r <= cb1_in;
            cb2_r <= cb2_in;
            
            // Default: no changes
            set_ca1_ifr <= 0;
            set_ca2_ifr <= 0;
            set_cb1_ifr <= 0;
            set_cb2_ifr <= 0;
            clr_ca_ifr <= 0;
            clr_cb_ifr <= 0;
            
            // CA1 interrupt (IFR bit 1) - active edge determined by PCR bit 0
            if (pcr[0] ? (ca1 & ~ca1_r) : (~ca1 & ca1_r)) begin
                set_ca1_ifr <= 1;
            end
            
            // CA2 interrupt (IFR bit 0) - if in input mode (PCR bit 3 = 0)
            if (~pcr[3]) begin
                // PCR bit 1 controls edge: 0=negative, 1=positive
                if (pcr[1] ? (ca2_in & ~ca2_r) : (~ca2_in & ca2_r)) begin
                    set_ca2_ifr <= 1;
                end
            end
            
            // Reading or writing ORA (rs=1 or rs=15) clears CA1/CA2 flags
            if (chip_sel && (rs == 4'h1 || rs == 4'hF)) begin
                clr_ca_ifr <= 1;
            end
            
            // CB1 interrupt (IFR bit 4) - active edge determined by PCR bit 4
            if (pcr[4] ? (cb1_in & ~cb1_r) : (~cb1_in & cb1_r)) begin
                set_cb1_ifr <= 1;
            end
            
            // CB2 interrupt (IFR bit 3) - if in input mode (PCR bit 7 = 0)
            if (~pcr[7]) begin
                // PCR bit 5 controls edge: 0=negative, 1=positive
                if (pcr[5] ? (cb2_in & ~cb2_r) : (~cb2_in & cb2_r)) begin
                    set_cb2_ifr <= 1;
                end
            end
            
            // Reading or writing ORB (rs=0) clears CB1/CB2 flags
            if (chip_sel && rs == 4'h0) begin
                clr_cb_ifr <= 1;
            end
        end
    end
    
    // --- 5. Consolidated IFR Management ---
    // Single always block to manage all IFR bits to avoid multiple drivers
    always @(posedge phi2 or negedge rst_n) begin
        if (!rst_n) begin
            ifr <= 8'h00;
        end else begin
            // Handle direct writes to IFR (writing 1 clears flags)
            if (chip_sel && !rw && rs == 4'hD) begin
                ifr[6:0] <= ifr[6:0] & ~data_in[6:0];
            end else begin
                // Timer 1 interrupt
                if (clr_t1_ifr)
                    ifr[6] <= 0;
                else if (set_t1_ifr)
                    ifr[6] <= 1;
                    
                // Timer 2 interrupt
                if (clr_t2_ifr)
                    ifr[5] <= 0;
                else if (set_t2_ifr)
                    ifr[5] <= 1;
                    
                // CB1 interrupt
                if (clr_cb_ifr)
                    ifr[4] <= 0;
                else if (set_cb1_ifr)
                    ifr[4] <= 1;
                    
                // CB2 interrupt
                if (clr_cb_ifr)
                    ifr[3] <= 0;
                else if (set_cb2_ifr)
                    ifr[3] <= 1;
                    
                // Shift register - not implemented, keep cleared
                ifr[2] <= 0;
                    
                // CA1 interrupt
                if (clr_ca_ifr)
                    ifr[1] <= 0;
                else if (set_ca1_ifr)
                    ifr[1] <= 1;
                    
                // CA2 interrupt
                if (clr_ca_ifr)
                    ifr[0] <= 0;
                else if (set_ca2_ifr)
                    ifr[0] <= 1;
            end
        end
    end

    // --- 6. CPU Write Access ---
    always @(posedge phi2 or negedge rst_n) begin
        if (!rst_n) begin
            {ora, orb, ddra, ddrb, acr, pcr, ier, sr} <= 0;
            t1_l <= 16'hFFFF;
        end else if (chip_sel && !rw) begin
            case (rs)
                4'h0: orb  <= data_in;
                4'h1, 4'hF: ora  <= data_in;  // ORA (with/without handshake)
                4'h2: ddrb <= data_in;
                4'h3: ddra <= data_in;
                4'h4: t1_l[7:0] <= data_in;
                4'h5: t1_l[15:8] <= data_in;
                4'h6: t1_l[7:0] <= data_in;  // T1L-L
                4'h7: t1_l[15:8] <= data_in; // T1L-H
                4'h8: t2_l <= data_in;       // T2L-L
                4'h9: ;                       // T2C-H handled in timer logic
                4'hA: sr <= data_in;          // Shift Register
                4'hB: acr  <= data_in;
                4'hC: pcr  <= data_in;
                4'hD: ;                       // IFR handled in consolidated block
                4'hE: begin                   // IER
                    if (data_in[7])
                        ier[6:0] <= ier[6:0] | data_in[6:0];
                    else
                        ier[6:0] <= ier[6:0] & ~data_in[6:0];
                end
            endcase
        end
    end

    // --- 7. CPU Read Access ---
    always @(*) begin
        if (chip_sel && rw) begin
            case (rs)
                4'h0: begin
                    // Special handling for PB7 when Timer 1 output is enabled
                    if (acr[7])
                        data_out = {t1_pb7, (port_b[6:0] & ~ddrb[6:0]) | (orb[6:0] & ddrb[6:0])};
                    else
                        data_out = (port_b & ~ddrb) | (orb & ddrb);
                end
                4'h1, 4'hF: data_out = (port_a & ~ddra) | (ora & ddra);
                4'h2: data_out = ddrb;
                4'h3: data_out = ddra;
                4'h4: data_out = t1_c[7:0];
                4'h5: data_out = t1_c[15:8];
                4'h6: data_out = t1_l[7:0];
                4'h7: data_out = t1_l[15:8];
                4'h8: data_out = t2_c[7:0];
                4'h9: data_out = t2_c[15:8];
                4'hA: data_out = sr;
                4'hB: data_out = acr;
                4'hC: data_out = pcr;
                4'hD: data_out = {irq_active, ifr[6:0]};  // IFR bit 7 is IRQ status
                4'hE: data_out = {1'b1, ier[6:0]};
                default: data_out = 8'h00;
            endcase
        end else data_out = 8'h00;
    end

    // --- 8. Interrupt Aggregation ---
    // IFR bit 7 is set if any enabled interrupt flag is active
    wire irq_active = |(ifr[6:0] & ier[6:0]);
    assign irq_n = !irq_active;
    
    // --- 9. CA2/CB2 Output Control ---
    // PCR Register Layout per 6522 datasheet:
    // Bit 0:   CA1 interrupt control (0=neg edge, 1=pos edge)
    // Bit 1:   CA2 interrupt control when bit 3=0 (0=neg edge, 1=pos edge)
    // Bit 2:   CA2 mode bit (used with bit 1 in output mode)
    // Bit 3:   CA2 direction (0=input/interrupt, 1=output)
    // Bit 4:   CB1 interrupt control (0=neg edge, 1=pos edge)
    // Bit 5:   CB2 interrupt control when bit 7=0 (0=neg edge, 1=pos edge)
    // Bit 6:   CB2 mode bit (used with bit 5 in output mode)
    // Bit 7:   CB2 direction (0=input/interrupt, 1=output)
    //\n    // CA2 Output Modes (PCR[3:1]):\n    //   100: Handshake - goes LOW on ORA access, HIGH on CA1 active edge\n    //   101: Pulse - one phi2 cycle LOW pulse on ORA access\n    //   110: Manual LOW\n    //   111: Manual HIGH\n    //\n    // CB2 Output Modes (PCR[7:5]): Same as CA2 but for ORB and CB1\n    \n    reg ca2_out_r, cb2_out_r;
    
    always @(posedge phi2 or negedge rst_n) begin
        if (!rst_n) begin
            ca2_out_r <= 1'b1;  // Start high per datasheet
            cb2_out_r <= 1'b1;
            ca2_pulse <= 0;
            cb2_pulse <= 0;
        end else begin            
            // CA2 output control (PCR bit 3 = 1 for output mode)
            if (pcr[3]) begin
                case (pcr[2:1])
                    2'b00: begin  // Handshake mode
                        // Goes LOW on read/write of ORA
                        if (chip_sel && (rs == 4'h1 || rs == 4'hF)) begin
                            ca2_out_r <= 1'b0;
                        end
                        // Returns HIGH on active CA1 edge
                        else if (pcr[0] ? (ca1 & ~ca1_r) : (~ca1 & ca1_r)) begin
                            ca2_out_r <= 1'b1;
                        end
                    end
                    2'b01: begin  // Pulse output - one cycle pulse
                        // Pulse immediately on access, return high next cycle
                        if (chip_sel && (rs == 4'h1 || rs == 4'hF)) begin
                            ca2_out_r <= 1'b0;  // Go low immediately
                            ca2_pulse <= 1;      // Flag to return high next cycle
                        end else if (ca2_pulse) begin
                            ca2_out_r <= 1'b1;  // Return high
                            ca2_pulse <= 0;
                        end
                    end
                    2'b10: ca2_out_r <= 1'b0;  // Manual low
                    2'b11: ca2_out_r <= 1'b1;  // Manual high
                endcase
            end else begin
                ca2_out_r <= 1'b1;  // Input mode - keep high
            end
            
            // CB2 output control (PCR bit 7 = 1 for output mode)
            if (pcr[7]) begin
                case (pcr[6:5])
                    2'b00: begin  // Handshake mode
                        // Goes LOW on write of ORB
                        if (chip_sel && !rw && rs == 4'h0) begin
                            cb2_out_r <= 1'b0;
                        end
                        // Returns HIGH on active CB1 edge
                        else if (pcr[4] ? (cb1_in & ~cb1_r) : (~cb1_in & cb1_r)) begin
                            cb2_out_r <= 1'b1;
                        end
                    end
                    2'b01: begin  // Pulse output - one cycle pulse
                        // Pulse immediately on write, return high next cycle
                        if (chip_sel && !rw && rs == 4'h0) begin
                            cb2_out_r <= 1'b0;  // Go low immediately
                            cb2_pulse <= 1;      // Flag to return high next cycle
                        end else if (cb2_pulse) begin
                            cb2_out_r <= 1'b1;  // Return high
                            cb2_pulse <= 0;
                        end
                    end
                    2'b10: cb2_out_r <= 1'b0;  // Manual low
                    2'b11: cb2_out_r <= 1'b1;  // Manual high
                endcase
            end else begin
                cb2_out_r <= 1'b1;  // Input mode - keep high
            end
        end
    end
    
    assign ca2_out = ca2_out_r;
    assign cb2_out = cb2_out_r;
    
    // CB1 can be output in shift register mode, but simplified here as input
    assign cb1_out = 1'b0;

endmodule
`default_nettype wire
