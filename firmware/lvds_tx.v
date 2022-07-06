module lvds_tx
    (
		input               i_reset,
        input               i_ddr_clk,
        input               i_empty,
        input [31:0]        i_data,

        output reg          o_read,
        output reg          o_clk,
        output reg [1:0]    o_ddr_data,

        output reg          o_led0,
        output reg          o_led1,
    );

    reg [3:0]  r_count;
    reg [6:0]  r_icount;
    reg [31:0] r_data;

    initial o_clk = 0;
    initial o_read = 0;
    initial o_led0 = 0;
    initial o_led1 = 0;

    always @(negedge i_ddr_clk)
        if (i_reset) begin
            r_icount <= 0;
            r_count <= 0;
            o_read <= 0;
            r_data <= 0;
            o_led0 <= 1'b0;
            o_led1 <= 1'b0;
            o_clk <= 0; 
        end else begin

            if (r_icount > 32) begin
                o_clk <= ~o_clk;

                /* TODO check */
                if (r_count == 8'hd) begin
                    o_read <= ~i_empty;
                end else begin 
                    if (r_count == 8'hf) begin
                        if (o_read == 1'b1) begin
                            o_read <= 0;
                            r_data <= {2'b10, i_data[28:16], 1'b1, 2'b01, i_data[12:0], 1'b0};
                        end else begin
                            r_data <= 32'h80004000;
							if (o_led1) 
								o_led0 <= 1;
                        end
                    end
                end
                r_count <= r_count + 1;
            end else begin
                r_icount <= r_icount + 1;
            end
            o_led1 <= r_data[16];
            o_ddr_data <= {r_data[31-2*r_count], r_data[30-2*r_count]};
        end
endmodule
