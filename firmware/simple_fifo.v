module simple_fifo #(
    parameter ADDR_WIDTH = 9,
    parameter DATA_WIDTH = 32 
)
(
    input wire                      wr_rst_i,
    input wire                      wr_clk_i,
    input wire                      wr_en_i,
    input wire [DATA_WIDTH-1:0] wr_data_i,

    input wire                      rd_rst_i,
    input wire                      rd_clk_i,
    input wire                      rd_en_i,
    output reg [DATA_WIDTH-1:0] rd_data_o,

    output reg          full_o,
    output reg          a_full_o,
    output reg          empty_o,
    output reg          o_led
);

reg [ADDR_WIDTH-1:0]    wr_addr;
reg [ADDR_WIDTH-1:0]    rd_addr;

always @(posedge wr_clk_i) begin
    if (wr_rst_i) begin
        wr_addr <= 0;
        full_o <= 1'b0;
    end else begin
	    //if (full_o == 1'b1) begin
	    //end 
        if (wr_en_i && !full_o) begin
            wr_addr <= wr_addr + 1'b1;
            full_o <= ((wr_addr + 2) & ((2**ADDR_WIDTH)-1)) == rd_addr;
            mem[wr_addr] <= wr_data_i;
        end else begin
            full_o <= ((wr_addr + 1'b1) & ((2**ADDR_WIDTH)-1)) == rd_addr;
        end
    end
end

always @(posedge rd_clk_i) begin
    if (rd_rst_i) begin
        rd_addr <= 0;
        empty_o <= 1'b1;
        o_led <= 1'b0;
    end else begin
        if (rd_en_i && !empty_o) begin
            rd_addr <= rd_addr + 1'b1;
            rd_data_o <= mem[rd_addr];
            empty_o <= ((rd_addr + 1) & ((2**ADDR_WIDTH)-1)) == wr_addr;
	    o_led <= 1'b1;
        end else begin
            empty_o <= (rd_addr == wr_addr);
	    o_led <= 1'b0;
        end
    end
end

reg [DATA_WIDTH-1:0] mem[(1<<ADDR_WIDTH)-1:0];

endmodule
