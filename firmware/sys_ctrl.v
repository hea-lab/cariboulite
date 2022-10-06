module sys_ctrl
    (
        input               i_sys_clk,        // FPGA Clock

        input [4:0]         i_ioc,
        input [7:0]         i_data_in,
        //output reg [7:0]    o_data_out,
        input               i_cs,
        input               i_fetch_cmd,
        input               i_load_cmd,

        output reg          o_soft_reset,
        output reg          o_trx_state_tx,
        input [7:0]         i_error_list );

    // MODULE SPECIFIC IOC LIST
    // ------------------------
    localparam
        ioc_module_version  = 5'b00000,     // read only
        ioc_system_version  = 5'b00001,     // read only
        ioc_manu_id         = 5'b00010,     // read only
        ioc_error_state     = 5'b00011,     // read only
        ioc_soft_reset      = 5'b00100,     // write only

        ioc_trx_state_rx      = 5'b00101,     // write only
        ioc_trx_state_tx      = 5'b00110;     // write only

    //// MODULE SPECIFIC PARAMS
    //// ----------------------
    //localparam
    //    module_version  = 8'b00000001,
    //    system_version  = 8'b00000001,
    //    manu_id         = 8'b00000001;

    // MODULE INTERNAL SIGNALS
    // -----------------------
    reg [3:0] reset_count;
    reg reset_cmd;

    initial o_trx_state_tx = 0;

    // MODULE MAIN PROCESS
    // -------------------
    always @(posedge i_sys_clk)
    begin
        if (i_cs == 1'b1) begin
            //=============================================
            // READ OPERATIONS
            //=============================================
            //if (i_fetch_cmd == 1'b1) begin
            //    //case (i_ioc)
            //    //    ioc_module_version: o_data_out <= module_version;
            //    //    ioc_system_version: o_data_out <= system_version;
            //    //    ioc_manu_id: o_data_out <= manu_id;
            //    //    ioc_error_state: o_data_out <= i_error_list;
            //    //endcase
            //end
            ////=============================================
            //// WRITE OPERATIONS
            ////=============================================
            //else
            if (i_load_cmd == 1'b1) begin
                case (i_ioc)
                    ioc_soft_reset: begin reset_cmd <= 1'b1; end
                    ioc_trx_state_tx: begin o_trx_state_tx <= 1'b1; end
                    ioc_trx_state_rx: begin o_trx_state_tx <= 1'b0; end
                endcase
            end
        end else begin
            reset_cmd <= 1'b0;
        end
    end

    // Reset state process
    always @(posedge i_sys_clk)
    begin
        if (reset_cmd) begin
            reset_count <= 0;
        end else begin
            if (reset_count < 4'd15) begin
                reset_count <= reset_count + 1'b1;
                o_soft_reset <= 1'b1;
            end else if (reset_count == 4'd15) begin
                reset_count <= reset_count;
                o_soft_reset <= 1'b0;
            end else begin
                reset_count <= 0; 
            end
        end
    end

endmodule // sys_ctrl
