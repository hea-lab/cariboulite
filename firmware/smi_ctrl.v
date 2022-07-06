module smi_ctrl
    (
        input               i_reset,
        input               i_sys_clk,        // FPGA Clock

        input [4:0]         i_ioc,
        input [7:0]         i_data_in,
        output reg [7:0]    o_data_out,
        input               i_cs,
        input               i_fetch_cmd,
        input               i_load_cmd,

        // FIFO INTERFACE 0.9 GHz
        output              o_fifo_09_pull,
        input [31:0]        i_fifo_09_pulled_data,
        input               i_fifo_09_full,
        input               i_fifo_09_empty,

        output reg          o_fifo_09_push,
        output reg[31:0]    o_fifo_09_pushed_data,
        output reg          o_clk,
        // FIFO INTERFACE 2.4 GHz
        output              o_fifo_24_pull,
        input [31:0]        i_fifo_24_pulled_data,
        input               i_fifo_24_full,
        input               i_fifo_24_empty,

        // SMI INTERFACE
        input [2:0]         i_smi_a,
        input               i_smi_soe_se,
        input               i_smi_swe_srw,
        output reg [7:0]    o_smi_data_out,
        input [7:0]         i_smi_data_in,
        output              o_smi_read_req,
        output              o_smi_write_req,
        output              o_smi_writing,
        input               i_smi_test,
        input               i_fifo_full,
        input[10:0]         i_fifo_fill_level,
        output              o_dreq,

        // Errors
        output reg          o_address_error );

    // MODULE SPECIFIC IOC LIST
    // ------------------------
    localparam
        ioc_module_version  = 5'b00000,     // read only
        ioc_fifo_status     = 5'b00001;     // read-only

    // MODULE SPECIFIC PARAMS
    // ----------------------
    localparam
        module_version  = 8'b00000001;

    // SMI ADDRESS DEFS
    // ----------------
    localparam
        smi_address_idle = 3'b000,
        smi_address_write_900 = 3'b001,
        smi_address_write_2400 = 3'b010,
        smi_address_write_res2 = 3'b011,
        smi_address_read_res1 = 3'b100,
        smi_address_read_900 = 3'b101,
        smi_address_read_2400 = 3'b110,
        smi_address_read_res = 3'b111;

    always @(posedge i_sys_clk)
    begin
        if (i_reset) begin
            o_address_error <= 1'b0;
            // put the initial states here
        end else begin
            if (i_cs == 1'b1) begin
                if (i_fetch_cmd == 1'b1) begin
                    case (i_ioc)
                        //----------------------------------------------
                        ioc_module_version: o_data_out <= module_version; // Module Version

                        //----------------------------------------------
                        ioc_fifo_status: begin
                            o_data_out[0] <= i_fifo_09_empty;
                            o_data_out[1] <= i_fifo_09_full;
                            o_data_out[2] <= i_fifo_24_empty;
                            o_data_out[3] <= i_fifo_24_full;
                            o_data_out[7:4] <= 4'b0000;
                        end

                    endcase
                end
            end
        end
    end

    // Tell the RPI that data is pending in either of the two fifos
    assign o_smi_read_req = 0;
    //assign o_smi_read_req = !i_fifo_09_empty;
    //assign o_smi_read_req = (!i_fifo_09_empty && (i_smi_a == smi_address_read_900)) || 
    //                        (!i_fifo_24_empty && (i_smi_a == smi_address_read_2400));
    //assign o_smi_read_req = 1'b1;

    //!i_fifo_09_empty || !i_fifo_24_empty;

    reg [4:0] int_cnt_09;
    reg [4:0] int_cnt;
    reg r_fifo_09_pull;
    reg r_fifo_09_pull_1;
    wire w_fifo_09_pull_trigger;
    reg r_fifo_24_pull;
    reg r_fifo_24_pull_1;
    wire w_fifo_24_pull_trigger;
    reg [7:0] r_smi_test_count_09;
    reg [7:0] b3;
    reg [7:0] b2;
    reg [7:0] b1;
    reg [7:0] b0;


    wire swe_and_reset;
    assign swe_and_reset = !i_reset && i_smi_swe_srw;

    initial int_cnt = 5'd31;
    initial o_clk = 1'b0;
    initial b0 = 0;
    initial b1 = 0;
    initial b2 = 0;
    initial b3 = 0;

    always @(negedge swe_and_reset)
    begin
        if (i_reset) begin
            int_cnt <= 5'd31;
            o_fifo_09_push <= 1'b1;
            o_clk <= 1'b0;
        end else begin

            case (int_cnt)
                31: b3 = i_smi_data_in;
                23: b2 = i_smi_data_in;
                15: b1 = i_smi_data_in;
                7: b0 = i_smi_data_in;
            endcase

            if (int_cnt == 7) begin
                o_clk <= 1'b1;
                /* https:
                * //raw.githubusercontent.com/cariboulabs/cariboulite/main/docs/smi/Secondary%
                * 20Memory%20Interface.pdf => figure 10*/
                o_fifo_09_pushed_data <= {b2, b3, b0, b1};
            end else begin
                o_clk <= 1'b0;
            end

            int_cnt <= int_cnt - 8;

        end
    end

    wire soe_and_reset;
    assign soe_and_reset = !i_reset && i_smi_soe_se;

    always @(negedge soe_and_reset)
    begin
        if (i_reset) begin
            int_cnt_09 <= 5'd31;
            r_smi_test_count_09 <= 8'b00000000;
        end else begin
            //w_fifo_09_pull_trigger <= !i_fifo_09_empty && (int_cnt_09 == 5'd7);
            //w_fifo_24_pull_trigger <= !i_fifo_24_empty && (int_cnt_24 == 5'd7);

            if (i_smi_a == smi_address_read_900) begin
                if ( i_smi_test ) begin
                    o_smi_data_out <= r_smi_test_count_09;
                    r_smi_test_count_09 <= r_smi_test_count_09 + 1'b1;
                end else begin
                    int_cnt_09 <= int_cnt_09 - 8;
                    o_smi_data_out <= i_fifo_09_pulled_data[int_cnt_09:int_cnt_09-7];
                end
            end

        end
    end

    always @(posedge i_sys_clk)
    begin
        if (i_reset) begin
            r_fifo_09_pull <= 1'b0;
            r_fifo_24_pull <= 1'b0;
            r_fifo_09_pull_1 <= 1'b0;
            r_fifo_24_pull_1 <= 1'b0;
        end else begin
            r_fifo_09_pull <= w_fifo_09_pull_trigger;
            r_fifo_24_pull <= w_fifo_24_pull_trigger;
            r_fifo_09_pull_1 <= r_fifo_09_pull;
            r_fifo_24_pull_1 <= r_fifo_24_pull;
        end
    end

    //assign o_smi_data_out = 8'b01011010;
    assign o_fifo_09_pull = !r_fifo_09_pull_1 && r_fifo_09_pull && !i_fifo_09_empty;
    assign o_fifo_24_pull = !r_fifo_24_pull_1 && r_fifo_24_pull && !i_fifo_24_empty;
    assign o_smi_writing = i_smi_a[2];
    assign o_smi_write_req = !i_fifo_full;

endmodule // smi_ctrl
