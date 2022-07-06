`include "spi_if.v"
`include "sys_ctrl.v"
`include "smi_ctrl.v"
`include "lvds_rx.v"
`include "lvds_tx.v"
`include "simple_fifo.v"
`include "afifo.v"
`include "rx_framer.v"

module top(
      input i_glob_clock,


      // MODEM (LVDS & CLOCK)
      output o_iq_tx_p,
      output o_iq_tx_n,
      output o_iq_tx_clk_p,
      output o_iq_tx_clk_n,
      input i_iq_rx_09_p,     // Paired with i_iq_rx_09_n - only the 'B' pins need to be specified
      input i_iq_rx_clk_p,    // Paired with i_iq_rx_clk_n - only the 'B' pins need to be specified


      // DIGITAL I/F
      output o_led0,
      output o_led1,

      // SMI TO RPI
      input i_smi_a1,
      input i_smi_a2,
      input i_smi_a3,

      input i_smi_soe_se,
      input i_smi_swe_srw,
      inout [7:0] io_smi_data,
      output o_smi_write_req,
      output o_smi_read_req,

      // SPI
      input i_mosi,
      input i_sck,
      input i_ss,
      output o_miso );

   //=========================================================================
   // INNER SIGNALS
   //=========================================================================
   reg         r_counter;
   wire        w_clock_spi;
   wire        w_clock_sys;
   wire        w_clock_smi_tx;
   wire [4:0]  w_ioc;
   wire [7:0]  w_rx_data;
   reg [7:0]   r_tx_data;
   wire [3:0]  w_cs;
   wire        w_fetch;
   wire        w_load;
   reg         r_reset;
   wire        w_soft_reset;

   wire [7:0]  w_tx_data_sys;
   wire [7:0]  w_tx_data_io;
   wire [7:0]  w_tx_data_smi;

   //=========================================================================
   // INITIAL STATE
   //=========================================================================
   initial begin
      r_counter = 2'b0;
      r_reset = 1'b0;
   end

   //=========================================================================
   // INSTANCES
   //=========================================================================
   spi_if spi_if_ins
   (
      .i_rst_b (w_soft_reset),
      .i_sys_clk (w_clock_sys),
      .o_ioc (w_ioc),
      .o_data_in (w_rx_data),
      .i_data_out (r_tx_data),
      .o_cs (w_cs),
      .o_fetch_cmd (w_fetch),
      .o_load_cmd (w_load),

      // SPI Interface
      .i_spi_sck (i_sck),
      .o_spi_miso (int_miso),
      .i_spi_mosi (i_mosi),
      .i_spi_cs_b (i_ss)
   );

   wire int_miso;
   assign o_miso = (i_ss)?1'bZ:int_miso;

   sys_ctrl sys_ctrl_ins
   (
      .i_reset (r_reset),
      .i_sys_clk (w_clock_sys),
      .i_ioc (w_ioc),
      .i_data_in (w_rx_data),
      .o_data_out (w_tx_data_sys),
      .i_cs (w_cs[0]),
      .i_fetch_cmd (w_fetch),
      .i_load_cmd (w_load),
      .o_soft_reset (w_soft_reset),

      //.i_error_list (w_debug)
   );

   //=========================================================================
   // CONBINATORIAL ASSIGNMENTS
   //=========================================================================
   //assign w_clock_spi = r_counter[0];
   assign w_clock_sys = r_counter;

   /*SB_GB sys_clk_buffer (          // Improve 'lvds_clock' fanout by pushing it into
                                    // a global high-fanout buffer
      .USER_SIGNAL_TO_GLOBAL_BUFFER (r_counter),
      .GLOBAL_BUFFER_OUTPUT(w_clock_sys) );*/

   //=========================================================================
   // CLOCK AND DATA-FLOW
   //=========================================================================
   always @(posedge i_glob_clock)
   begin
      r_counter <= !r_counter;

      case (w_cs)
         4'b0001: r_tx_data <= w_tx_data_sys;
         4'b0010: r_tx_data <= w_tx_data_io;
         4'b0100: r_tx_data <= w_tx_data_smi;
         4'b1000: r_tx_data <= 8'b10100101;  // 0xA5: reserved
         4'b0000: r_tx_data <= 8'b00000000;  // no module selected
      endcase
   end

   //=========================================================================
   // I/O (SB_IO, SB_GB) DIFFERENTIAL LINES
   //=========================================================================
   // Differential clock signal
   wire lvds_clock;        // The direct clock input
   wire lvds_clock_buf;    // The clock input after global buffer (improved fanout)

   SB_IO #(
       .PIN_TYPE(6'b0000_01),        // Input only, direct mode
      .IO_STANDARD("SB_LVDS_INPUT") // LVDS input
   ) iq_rx_clk (
      .PACKAGE_PIN(i_iq_rx_clk_p),  // Physical connection to 'i_iq_rx_clk_p'
      .D_IN_0 ( lvds_clock ));      // Wire out to 'lvds_clock'

   SB_IO #(
       .PIN_TYPE(6'b0110_00),
       .IO_STANDARD("SB_LVCMOS") 
   ) iq_tx_clk_n (
       .PACKAGE_PIN(o_iq_tx_clk_n),
       .D_OUT_0 ( ~lvds_clock_buf ));

   SB_IO #(
       .PIN_TYPE(6'b0110_00),
       .IO_STANDARD("SB_LVCMOS")
   ) iq_tx_clk_p (
       .PACKAGE_PIN(o_iq_tx_clk_p),
       .D_OUT_0 ( lvds_clock_buf ));

   SB_IO #(
       .PIN_TYPE(6'b0100_00),
       .IO_STANDARD("SB_LVCMOS")
   ) iq_tx_n (
       .PACKAGE_PIN(o_iq_tx_n),
       .OUTPUT_CLK (lvds_clock_buf),
       .D_OUT_0 ( ~d1 ),
       .D_OUT_1 ( ~d0 ));

   SB_IO #(
       .PIN_TYPE(6'b0100_00),
       .IO_STANDARD("SB_LVCMOS")
   ) iq_tx_p (
       .PACKAGE_PIN(o_iq_tx_p),
       .OUTPUT_CLK (lvds_clock_buf),
       .D_OUT_0 ( d1 ),
       .D_OUT_1 ( d0 ));

   SB_GB lvds_clk_buffer (          // Improve 'lvds_clock' fanout by pushing it into
                                    // a global high-fanout buffer
      .USER_SIGNAL_TO_GLOBAL_BUFFER (lvds_clock),
      .GLOBAL_BUFFER_OUTPUT(lvds_clock_buf) );

   //assign lvds_clock_buf = lvds_clock;

   // Differential 0.9GHz I/Q DDR signal
   SB_IO #(
      .PIN_TYPE(6'b000000),         // Input only, DDR mode (sample on both pos edge and
                                    // negedge of the input clock)
      .IO_STANDARD("SB_LVDS_INPUT"),// LVDS standard
      .NEG_TRIGGER(1'b0)            // The signal is negated in hardware
   ) iq_rx_09 (
      .PACKAGE_PIN(i_iq_rx_09_p),
      .INPUT_CLK (lvds_clock_buf),  // The I/O sampling clock with DDR
      .D_IN_0 ( w_lvds_rx_09_d1 ),  // the 0 deg data output
      .D_IN_1 ( w_lvds_rx_09_d0 ) );// the 180 deg data output


   //=========================================================================
   // LVDS RX SIGNAL FROM MODEM
   //=========================================================================
   wire w_lvds_rx_09_d0;   // 0 degree
   wire w_lvds_rx_09_d1;   // 180 degree
   reg d0;   // 0 degree
   reg d1;   // 180 degree

   wire w_rx_09_fifo_full;
   wire w_rx_09_fifo_full2;
   wire w_rx_09_fifo_empty;
   wire w_rx_09_fifo_empty2;
   wire w_rx_09_fifo_empty_tx;
   wire w_rx_09_fifo_write_clk;
   wire w_rx_09_fifo_push;
   wire w_rx_09_fifo_push2;
   wire [31:0] w_rx_09_fifo_data;
   wire [31:0] w_rx_09_fifo_data2;
   wire w_rx_09_fifo_pull;
   wire w_rx_09_fifo_pull2;
   wire w_rx_09_fifo_pull_tx;
   wire w_rx_09_fifo_push_tx;
   wire w_rx_09_fifo_full_tx;
   wire [31:0] w_rx_09_fifo_data_tx;
   wire [31:0] w_rx_09_fifo_pulled_data;
   wire [31:0] w_rx_09_fifo_pulled_data2;
   wire [31:0] w_rx_09_fifo_pulled_data_tx;

   wire o_clk;
   wire [10:0] w_fill_level;

   lvds_tx lvds_tx_09_inst(
      .i_reset (w_soft_reset),
      .i_ddr_clk (lvds_clock_buf),
      
      .o_ddr_data ({d1, d0}),
      .o_read(w_rx_09_fifo_pull_tx),
      
      .i_empty(w_rx_09_fifo_empty_tx),
      .i_data(w_rx_09_fifo_pulled_data_tx),
      .o_led0 (o_led0),
      .o_led1 (o_led1),
      .o_clk(o_clk)
   );

    afifo tx_09_fifo(
       .i_wrst_n (!w_soft_reset),
       .i_wclk (w_clock_smi_tx),

       .i_wr (w_rx_09_fifo_push_tx),
       .i_wdata (w_rx_09_fifo_data_tx),

       .i_rrst_n (!w_soft_reset),
       .i_rclk (o_clk),

       .i_rd (w_rx_09_fifo_pull_tx),
       .o_rdata (w_rx_09_fifo_pulled_data_tx),
       .o_wfull (w_rx_09_fifo_full_tx),
       .o_rempty (w_rx_09_fifo_empty_tx),
       .o_wfill_level(w_fill_level)
   );

   lvds_rx lvds_rx_09_inst(
      .i_reset (w_soft_reset),
      .i_ddr_clk (lvds_clock),
      
      .i_ddr_data ({w_lvds_rx_09_d1, w_lvds_rx_09_d0}),
      .i_fifo_full (w_rx_09_fifo_full),
      .o_fifo_write_clk (w_rx_09_fifo_write_clk),
      .o_fifo_push (w_rx_09_fifo_push),
      
      .o_fifo_data (w_rx_09_fifo_data),
      
      .o_debug_state ()
   );

   simple_fifo rx_09_fifo1(
      .wr_rst_i (w_soft_reset),
      .wr_clk_i (w_rx_09_fifo_write_clk),
      .wr_en_i (w_rx_09_fifo_push),
      .wr_data_i ({3'b000, w_rx_09_fifo_data[29:17], 3'b000, w_rx_09_fifo_data[13:1]}),
      .rd_rst_i (w_soft_reset),
      .rd_clk_i (w_clock_sys),
      .rd_en_i (w_rx_09_fifo_pull),
      .rd_data_o (w_rx_09_fifo_pulled_data),
      .full_o (w_rx_09_fifo_full),
      .empty_o (w_rx_09_fifo_empty)
   );

   rx_framer myFramer(
       .i_reset(w_soft_reset),
       .i_clk(w_clock_sys),

       .o_read(w_rx_09_fifo_pull),
       .i_empty(w_rx_09_fifo_empty),
       .i_data(w_rx_09_fifo_pulled_data),

       .o_fifo_push(w_rx_09_fifo_push2),
       .o_fifo_data(w_rx_09_fifo_data2),
       .i_fifo_full(w_rx_09_fifo_full2),
   );

   simple_fifo rx_09_fifo2(
       .wr_rst_i (w_soft_reset),
       .wr_clk_i (w_clock_sys),

       .wr_en_i (w_rx_09_fifo_push2),
       .wr_data_i (w_rx_09_fifo_data2),
       .full_o (w_rx_09_fifo_full2), 

       .rd_rst_i (w_soft_reset),
       .rd_clk_i (w_clock_sys),

       .rd_en_i (w_rx_09_fifo_pull2),
       .rd_data_o (w_rx_09_fifo_pulled_data2),
       .empty_o (w_rx_09_fifo_empty2),
   );

   smi_ctrl smi_ctrl_ins
   (
      .i_reset (w_soft_reset),
      .i_sys_clk (w_clock_sys),
      .i_ioc (w_ioc),
      .i_data_in (w_rx_data),
      .o_data_out (w_tx_data_smi),
      .i_cs (w_cs[2]),
      .i_fetch_cmd (w_fetch),
      .i_load_cmd (w_load),

      .o_fifo_09_pull (w_rx_09_fifo_pull2),
      .i_fifo_09_pulled_data (w_rx_09_fifo_pulled_data2),
      .i_fifo_09_full (w_rx_09_fifo_full2),
      .i_fifo_09_empty (w_rx_09_fifo_empty2),

      .o_fifo_09_push(w_rx_09_fifo_push_tx),
      .o_fifo_09_pushed_data(w_rx_09_fifo_data_tx),
      .o_clk(w_clock_smi_tx),

      .i_smi_a (w_smi_addr),
      .i_smi_soe_se (i_smi_soe_se),
      .i_smi_swe_srw (i_smi_swe_srw),
      .o_smi_data_out (w_smi_data_output),
      .i_smi_data_in (w_smi_data_input),
      .o_smi_read_req (w_smi_read_req),
      .o_smi_write_req (w_smi_write_req),
      .o_smi_writing (w_smi_writing),
      .o_dreq (w_dreq),
      .i_smi_test (w_smi_test),
      .i_fifo_full (w_rx_09_fifo_full_tx),
      .i_fifo_fill_level (w_fill_level),
      .o_address_error ()
   );

   wire [2:0] w_smi_addr;
   wire [7:0] w_smi_data_output;
   wire [7:0] w_smi_data_input;
   wire w_smi_read_req;
   wire w_smi_write_req;
   wire w_smi_writing;
   wire w_dreq;
   wire w_smi_test;

   assign w_smi_test = 1'b0;
   assign w_smi_addr = {i_smi_a3, i_smi_a2, i_smi_a1};
   assign io_smi_data = (w_smi_writing)?w_smi_data_output:8'bZ;
   assign w_smi_data_input = io_smi_data;

   //assign o_smi_write_req = (w_smi_writing)? w_smi_write_req :1'bZ;

   //assign o_smi_read_req = (w_smi_writing)? w_smi_read_req :1'bZ;
   assign o_smi_write_req = 1'bZ;
   //assign o_smi_read_req = (w_fill_level < 984) ? 1 : 0;
   assign o_smi_read_req = (w_fill_level < 500) ? 1 : 0;

endmodule // top
