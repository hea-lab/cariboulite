module lvds_rx
    (
        input               i_reset,
        input               i_ddr_clk,
        input [1:0]         i_ddr_data,

        output reg          o_clk,
        output reg          o_enable,
        output reg [3:0]    o_two_bits_cnt,
        output reg [31:0]   o_data);

    // Internal FSM States
    localparam
        state_idle   = 2'b00,
        state_i_phase = 2'b01,
        state_q_phase = 2'b11;

    // Modem sync symbols
    localparam
        modem_i_sync = 2'b10,
        modem_q_sync = 2'b01;

    // Internal Registers
    reg [1:0]   r_state_if;
    reg [2:0]   r_phase_count;
    reg [31:0]  r_data;

    // Initial conditions
    initial begin
        r_state_if = state_idle;
        r_phase_count = 3'b111;
        r_data = 0;
        o_enable = 1'b0;
    end

    // Main Process
    always @(posedge i_ddr_clk)
    begin
        if (i_reset) begin
            r_state_if <= state_idle;
            o_enable <= 1'b0;
            r_phase_count <= 3'b111;
            r_data <= 0;
            o_clk <= 0;
        end else begin

            o_clk <= !o_clk;

            case (r_state_if)
                state_idle: begin
                    if (i_ddr_data == modem_i_sync ) begin
                        r_state_if <= state_i_phase;
                        r_data <= modem_i_sync;
                        o_two_bits_cnt <= 0;
                    end else begin
                        o_two_bits_cnt <= o_two_bits_cnt + 1;
                    end

                    r_phase_count <= 3'b111;
                    o_enable <= 1'b0;
                end

                state_i_phase: begin
                    o_two_bits_cnt <= o_two_bits_cnt + 1;
                    if (r_phase_count == 3'b000) begin
                        if (i_ddr_data == modem_q_sync ) begin
                            r_phase_count <= 3'b110;
                            r_state_if <= state_q_phase;
                        end else begin
                            r_state_if <= state_idle;
                        end

                    end else begin
                        r_phase_count <= r_phase_count - 1;
                    end
                    r_data <= {r_data[29:0], i_ddr_data};
                end

                state_q_phase: begin
                    o_two_bits_cnt <= o_two_bits_cnt + 1;
                    if (r_phase_count == 3'b000) begin
                        o_enable <= 1'b1;
                        r_state_if <= state_idle;
                        o_data <= {r_data[29:0], i_ddr_data};

                    end else begin
                        r_phase_count <= r_phase_count - 1;
                    end
                    r_data <= {r_data[29:0], i_ddr_data};
                end
            endcase
        end
    end
endmodule
