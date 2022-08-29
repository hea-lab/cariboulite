`include "spi_if.v"
`include "sys_ctrl.v"
`include "smi_ctrl.v"
`include "lvds_rx.v"
`include "lvds_tx.v"
`include "afifo.v"
`include "rx_framer.v"
`include "tx_deframer.v"

module top(
      input i_glob_clock,

      // MODEM (LVDS & CLOCK)
      output o_iq_tx_p,
      output o_iq_tx_n,
      output o_iq_tx_clk_p,
      output o_iq_tx_clk_n,

      input i_iq_rx_p,     // Paired with i_iq_rx_n - only the 'B' pins need to be specified
      input i_iq_rx_clk_p, // Paired with i_iq_rx_clk_n - only the 'B' pins need to be specified

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

      output o_smi_dreq,
      input i_smi_dack, // not used

      // SPI
      input i_mosi,
      input i_sck,
      input i_ss,
      output o_miso );

   //=========================================================================
   // INNER SIGNALS
   //=========================================================================
   reg         r_clock_sys;
   wire        w_clock_smi_tx;
   wire [4:0]  w_ioc;
   wire [7:0]  w_rx_data;
   reg [7:0]   r_tx_data;
   wire [3:0]  w_cs;
   wire        w_fetch;
   wire        w_load;
   wire        w_soft_reset;

   //=========================================================================
   // INITIAL STATE
   //=========================================================================
   initial begin
      r_clock_sys = 1'b0;
   end

   //=========================================================================
   // INSTANCES
   //=========================================================================
   spi_if spi_if_ins
   (
      .i_rst_b (w_soft_reset),
      .i_sys_clk (r_clock_sys),
      .o_ioc (w_ioc),
      .o_data_in (w_rx_data),
      .i_data_out (r_tx_data),
      .o_cs (w_cs),
      .o_fetch_cmd (w_fetch),
      .o_load_cmd (w_load),

      // SPI Interface
      .i_spi_sck (i_sck),
      .o_spi_miso (w_miso),
      .i_spi_mosi (i_mosi),
      .i_spi_cs_b (i_ss)
   );

   wire w_miso;
   assign o_miso = (i_ss)?1'bZ:w_miso;

   sys_ctrl sys_ctrl_ins
   (
      .i_sys_clk (r_clock_sys),
      .i_ioc (w_ioc),
      .i_data_in (w_rx_data),
      //.o_data_out (w_tx_data_sys),
      .i_cs (w_cs[0]),
      .i_fetch_cmd (w_fetch),
      .i_load_cmd (w_load),
      .o_soft_reset (w_soft_reset),
   );

   //=========================================================================
   // SYSTEM CLOCK
   //=========================================================================
   always @(posedge i_glob_clock)
   begin
	  r_clock_sys <= !r_clock_sys;
   end

   //=========================================================================
   // I/O (SB_IO, SB_GB) DIFFERENTIAL LINES
   //=========================================================================
   // Differential clock signal
   wire w_lvds_rx_clk;        // The direct clock input
   wire w_lvds_tx_clk;

   assign w_lvds_tx_clk = w_lvds_rx_clk;

   SB_IO #(
       .PIN_TYPE(6'b0000_01),        // Input only, direct mode
      .IO_STANDARD("SB_LVDS_INPUT")  // LVDS input
   ) iq_rx_clk (
      .PACKAGE_PIN(i_iq_rx_clk_p),  // Physical connection to 'i_iq_rx_clk_p'
      .D_IN_0 ( w_lvds_rx_clk ));   // Wire out to 'w_lvds_rx_clk'

   SB_IO #(
       .PIN_TYPE(6'b0110_00),
       .IO_STANDARD("SB_LVCMOS") 
   ) iq_tx_clk_n (
       .PACKAGE_PIN(o_iq_tx_clk_n),
       .D_OUT_0 ( ~w_lvds_tx_clk ));

   SB_IO #(
       .PIN_TYPE(6'b0110_00),
       .IO_STANDARD("SB_LVCMOS")
   ) iq_tx_clk_p (
       .PACKAGE_PIN(o_iq_tx_clk_p),
       .D_OUT_0 ( w_lvds_tx_clk ));

   SB_IO #(
       .PIN_TYPE(6'b0100_00),
       .IO_STANDARD("SB_LVCMOS")
   ) iq_tx_n (
       .PACKAGE_PIN(o_iq_tx_n),
       .OUTPUT_CLK (w_lvds_tx_clk),
       .D_OUT_0 ( ~r_lvds_tx_d1 ),
       .D_OUT_1 ( ~r_lvds_tx_d0 ));

   SB_IO #(
       .PIN_TYPE(6'b0100_00),
       .IO_STANDARD("SB_LVCMOS")
   ) iq_tx_p (
       .PACKAGE_PIN(o_iq_tx_p),
       .OUTPUT_CLK (w_lvds_tx_clk),
       .D_OUT_0 ( r_lvds_tx_d1 ),
       .D_OUT_1 ( r_lvds_tx_d0 ));

   // Differential 0.9GHz I/Q DDR signal
   SB_IO #(
      .PIN_TYPE(6'b000000),         // Input only, DDR mode (sample on both pos edge and
                                    // negedge of the input clock)
      .IO_STANDARD("SB_LVDS_INPUT"),// LVDS standard
      .NEG_TRIGGER(1'b0)            // The signal is negated in hardware
   ) iq_rx (
      .PACKAGE_PIN(i_iq_rx_p),
      .INPUT_CLK (w_lvds_tx_clk),  // The I/O sampling clock with DDR
      .D_IN_0 ( w_lvds_rx_d1 ),    // the 0 deg data output
      .D_IN_1 ( w_lvds_rx_d0 ) );  // the 180 deg data output

   //=========================================================================
   // LVDS RX SIGNAL FROM MODEM
   //=========================================================================
   wire w_lvds_rx_d0;   // 0 degree
   wire w_lvds_rx_d1;   // 180 degree
   reg r_lvds_tx_d0;    // 0 degree
   reg r_lvds_tx_d1;    // 180 degree

   wire w_lvds_rx_enable;
   wire [31:0] w_lvds_rx_data;

   wire w_rx_fifo_pull;
   wire w_rx_fifo_push;
   wire w_rx_fifo_full;
   wire w_rx_fifo_empty;
   wire [31:0] w_rx_fifo_data;
   wire [31:0] w_rx_fifo_pulled_data;

   wire w_tx_fifo_pull;
   wire w_tx_fifo_push;
   wire w_tx_fifo_full;
   wire w_tx_fifo_empty;
   wire [31:0] w_tx_fifo_data;
   wire [31:0] w_tx_fifo_pulled_data;
   wire w_tx_clk;
   wire w_rx_clk;

   wire [9:0] w_filling_level;

   lvds_tx lvds_tx_inst(
      .i_reset (w_soft_reset),
      .i_ddr_clk (w_lvds_tx_clk),
      
      .o_ddr_data ({r_lvds_tx_d1, r_lvds_tx_d0}),
      .o_read(w_tx_fifo_pull),
      
      .i_empty(w_tx_fifo_empty),
      .i_data(w_tx_fifo_pulled_data),
      .o_clk(w_tx_clk),
      .o_led0(o_led0),
      .o_led1(o_led1),
   );

    afifo #(.ENABLE_FILLING_LEVEL(1)) tx_fifo(
       .i_wrst_n (!w_soft_reset),
       .i_wclk (w_clock_smi_tx),
       .i_wr (w_tx_fifo_push),
       .i_wdata (w_tx_fifo_data),
       .i_rrst_n (!w_soft_reset),
       .i_rclk (w_tx_clk),
       .i_rd (w_tx_fifo_pull),

       .o_rdata (w_tx_fifo_pulled_data),
       .o_wfull (w_tx_fifo_full),
       .o_rempty (w_tx_fifo_empty),
       .o_wfilling_level(w_filling_level)
   );

   lvds_rx lvds_rx_inst(
      .i_reset (w_soft_reset),
      .i_ddr_clk (w_lvds_rx_clk),
      .i_ddr_data ({w_lvds_rx_d1, w_lvds_rx_d0}),

      .o_enable (w_lvds_rx_enable),
      .o_data (w_lvds_rx_data),
      .o_clk(w_rx_clk)
   );

   rx_framer rx_framer(
       .i_reset(w_soft_reset),
       .i_clk(w_rx_clk),

       .i_enable(w_lvds_rx_enable),
       .i_data ({3'b000, w_lvds_rx_data[29:17], 3'b000, w_lvds_rx_data[13:1]}),

       .o_fifo_push(w_rx_fifo_push),
       .o_fifo_data(w_rx_fifo_data),
       .i_fifo_full(w_rx_fifo_full),
   );

   afifo rx_fifo(
       .i_wrst_n (!w_soft_reset),
       .i_wclk (r_clock_sys),
       .i_wr (w_rx_fifo_push),
       .i_wdata (w_rx_fifo_data),
       .i_rrst_n (!w_soft_reset),
       .i_rclk (r_clock_sys),
       .i_rd (w_rx_fifo_pull),

       .o_rdata (w_rx_fifo_pulled_data),
       .o_wfull (w_rx_fifo_full),
       .o_rempty (w_rx_fifo_empty),
   );

   smi_ctrl smi_ctrl_ins
   (
      .i_reset (w_soft_reset),
      .i_sys_clk (r_clock_sys),
      .i_ioc (w_ioc),
      .i_data_in (w_rx_data),
      //.o_data_out (w_tx_data_smi),
      .i_cs (w_cs[2]),
      .i_fetch_cmd (w_fetch),
      .i_load_cmd (w_load),

      .o_fifo_pull (w_rx_fifo_pull),
      .i_fifo_pulled_data (w_rx_fifo_pulled_data),
      .i_fifo_full (w_rx_fifo_full),
      .i_fifo_empty (w_rx_fifo_empty),

      .o_fifo_push(w_tx_fifo_push),
      .o_fifo_pushed_data(w_tx_fifo_data),
      .o_clk(w_clock_smi_tx),

      .i_smi_a (w_smi_addr),
      .i_smi_soe_se (i_smi_soe_se),
      .i_smi_swe_srw (i_smi_swe_srw),
      .o_smi_data_out (w_smi_data_output),
      .i_smi_data_in (w_smi_data_input),
      .o_smi_writing (w_smi_writing),
      .i_fifo_full (w_tx_fifo_full),
   );

   wire [2:0] w_smi_addr;
   wire [7:0] w_smi_data_output;
   wire [7:0] w_smi_data_input;
   wire w_smi_writing;

   assign w_smi_addr = {i_smi_a3, i_smi_a2, i_smi_a1};

   assign io_smi_data = (w_smi_writing)?w_smi_data_output:8'bZ;
   assign w_smi_data_input = io_smi_data;

  /* LVDS TX and RX seem to run simultaneously, so we only need to deal TX */
  assign o_smi_dreq = (w_filling_level < 500) ? 1 : 0;

endmodule
