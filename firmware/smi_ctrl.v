module smi_ctrl
    (
        input               i_reset,
        input               i_sys_clk,        // FPGA Clock

        input [4:0]         i_ioc,
        input [7:0]         i_data_in,
        //output reg [7:0]    o_data_out,
        input               i_cs,
        input               i_fetch_cmd,
        input               i_load_cmd,

        // FIFO INTERFACE 0.9 GHz
        output              o_fifo_pull,
        input [31:0]        i_fifo_pulled_data,
        input               i_fifo_full,
        input               i_fifo_empty,

        output reg          o_fifo_push,
        output reg[31:0]    o_fifo_pushed_data,
        output reg          o_clk,

        // SMI INTERFACE
        input [2:0]         i_smi_a,
        input               i_smi_soe_se,
        input               i_smi_swe_srw,
        output reg [7:0]    o_smi_data_out,
        input [7:0]         i_smi_data_in,

        //output              o_smi_dreq,
        //input               i_smi_dreq_ack,

        );

    //// MODULE SPECIFIC IOC LIST
    //// ------------------------
    //localparam
    //    ioc_module_version  = 5'b00000,     // read only
    //    ioc_fifo_status     = 5'b00001;     // read-only

    //// MODULE SPECIFIC PARAMS
    //// ----------------------
    //localparam
    //    module_version  = 8'b00000001;

    //// SMI ADDRESS DEFS
    //// ----------------
    //localparam
    //    smi_address_idle = 3'b000,
    //    smi_address_write_900 = 3'b001,
    //    smi_address_read_900 = 3'b101;

    //always @(posedge i_sys_clk)
    //begin
    //    if (i_reset) begin
    //        o_address_error <= 1'b0;
    //    end else begin
    //        if (i_cs == 1'b1) begin
    //            if (i_fetch_cmd == 1'b1) begin
    //                case (i_ioc)
    //                    //----------------------------------------------
    //                    ioc_module_version: o_data_out <= module_version; // Module Version

    //                    //----------------------------------------------
    //                    ioc_fifo_status: begin
    //                        o_data_out[0] <= i_fifo_empty;
    //                        o_data_out[1] <= i_fifo_full;
    //                        o_data_out[7:4] <= 4'b0000;
    //                    end

    //                endcase
    //            end
    //        end
    //    end
    //end

    // Tell the RPI that data is pending
    //assign o_smi_dreq = !i_fifo_empty;

    /* TODO naming */
    reg [4:0] int_cnt_09;
    reg [4:0] int_cnt;
    reg r_fifo_pull;
    reg r_fifo_pull_1;
    reg w_fifo_pull_trigger;
    reg [7:0] b3;
    reg [7:0] b2;
    reg [7:0] b1;
    reg [7:0] b0;


    wire swe_and_reset;
    assign swe_and_reset = !i_reset && i_smi_swe_srw;

    initial begin
        int_cnt = 5'd31;
        o_clk = 1'b0;
        b0 = 0;
        b1 = 0;
        b2 = 0;
        b3 = 0;
    end

    always @(negedge swe_and_reset)
    begin
        if (i_reset) begin
            int_cnt <= 5'd31;
            o_fifo_push <= 1'b1;
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
                o_fifo_pushed_data <= {b2, b3, b0, b1};
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
        end else begin
            w_fifo_pull_trigger <= int_cnt_09 == 5'd7;
            int_cnt_09 <= int_cnt_09 - 8;

            /* If fifo is empty, trigger a CRC error to avoid data corruption */

            if (i_fifo_empty)
                o_smi_data_out <= 8'hDE;
            else
                o_smi_data_out <= i_fifo_pulled_data[int_cnt_09:int_cnt_09-7];
        end
    end

    always @(posedge i_sys_clk)
    begin
        if (i_reset) begin
            r_fifo_pull <= 1'b0;
            r_fifo_pull_1 <= 1'b0;
        end else begin
            r_fifo_pull <= w_fifo_pull_trigger;
            r_fifo_pull_1 <= r_fifo_pull;
        end
    end

    assign o_fifo_pull = !r_fifo_pull_1 && r_fifo_pull && !i_fifo_empty;

endmodule
