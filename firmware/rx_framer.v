
module rx_framer
(
	input               i_reset,
	input               i_clk,
	input               i_empty,
	input [31:0]        i_data,
	input               i_fifo_full,

	output              o_read,
	output reg          o_fifo_push,
	output reg [31:0]   o_fifo_data
);

// CRC polynomial coefficients: x^16 + x^12 + x^5 + 1
//                              0x1021 (hex)
// CRC width:                   16 bits
// CRC shift direction:         left (big endian)
// Input word width:            32 bits

function automatic [15:0] crc16;
    input [15:0] crcIn;
    input [31:0] data;
begin
    crc16[0] = (crcIn[3] ^ crcIn[4] ^ crcIn[6] ^ crcIn[10] ^ crcIn[11] ^ crcIn[12] ^ data[0] ^ data[4] ^ data[8] ^ data[11] ^ data[12] ^ data[19] ^ data[20] ^ data[22] ^ data[26] ^ data[27] ^ data[28]);
    crc16[1] = (crcIn[4] ^ crcIn[5] ^ crcIn[7] ^ crcIn[11] ^ crcIn[12] ^ crcIn[13] ^ data[1] ^ data[5] ^ data[9] ^ data[12] ^ data[13] ^ data[20] ^ data[21] ^ data[23] ^ data[27] ^ data[28] ^ data[29]);
    crc16[2] = (crcIn[5] ^ crcIn[6] ^ crcIn[8] ^ crcIn[12] ^ crcIn[13] ^ crcIn[14] ^ data[2] ^ data[6] ^ data[10] ^ data[13] ^ data[14] ^ data[21] ^ data[22] ^ data[24] ^ data[28] ^ data[29] ^ data[30]);
    crc16[3] = (crcIn[6] ^ crcIn[7] ^ crcIn[9] ^ crcIn[13] ^ crcIn[14] ^ crcIn[15] ^ data[3] ^ data[7] ^ data[11] ^ data[14] ^ data[15] ^ data[22] ^ data[23] ^ data[25] ^ data[29] ^ data[30] ^ data[31]);
    crc16[4] = (crcIn[0] ^ crcIn[7] ^ crcIn[8] ^ crcIn[10] ^ crcIn[14] ^ crcIn[15] ^ data[4] ^ data[8] ^ data[12] ^ data[15] ^ data[16] ^ data[23] ^ data[24] ^ data[26] ^ data[30] ^ data[31]);
    crc16[5] = (crcIn[0] ^ crcIn[1] ^ crcIn[3] ^ crcIn[4] ^ crcIn[6] ^ crcIn[8] ^ crcIn[9] ^ crcIn[10] ^ crcIn[12] ^ crcIn[15] ^ data[0] ^ data[4] ^ data[5] ^ data[8] ^ data[9] ^ data[11] ^ data[12] ^ data[13] ^ data[16] ^ data[17] ^ data[19] ^ data[20] ^ data[22] ^ data[24] ^ data[25] ^ data[26] ^ data[28] ^ data[31]);
    crc16[6] = (crcIn[1] ^ crcIn[2] ^ crcIn[4] ^ crcIn[5] ^ crcIn[7] ^ crcIn[9] ^ crcIn[10] ^ crcIn[11] ^ crcIn[13] ^ data[1] ^ data[5] ^ data[6] ^ data[9] ^ data[10] ^ data[12] ^ data[13] ^ data[14] ^ data[17] ^ data[18] ^ data[20] ^ data[21] ^ data[23] ^ data[25] ^ data[26] ^ data[27] ^ data[29]);
    crc16[7] = (crcIn[2] ^ crcIn[3] ^ crcIn[5] ^ crcIn[6] ^ crcIn[8] ^ crcIn[10] ^ crcIn[11] ^ crcIn[12] ^ crcIn[14] ^ data[2] ^ data[6] ^ data[7] ^ data[10] ^ data[11] ^ data[13] ^ data[14] ^ data[15] ^ data[18] ^ data[19] ^ data[21] ^ data[22] ^ data[24] ^ data[26] ^ data[27] ^ data[28] ^ data[30]);
    crc16[8] = (crcIn[0] ^ crcIn[3] ^ crcIn[4] ^ crcIn[6] ^ crcIn[7] ^ crcIn[9] ^ crcIn[11] ^ crcIn[12] ^ crcIn[13] ^ crcIn[15] ^ data[3] ^ data[7] ^ data[8] ^ data[11] ^ data[12] ^ data[14] ^ data[15] ^ data[16] ^ data[19] ^ data[20] ^ data[22] ^ data[23] ^ data[25] ^ data[27] ^ data[28] ^ data[29] ^ data[31]);
    crc16[9] = (crcIn[0] ^ crcIn[1] ^ crcIn[4] ^ crcIn[5] ^ crcIn[7] ^ crcIn[8] ^ crcIn[10] ^ crcIn[12] ^ crcIn[13] ^ crcIn[14] ^ data[4] ^ data[8] ^ data[9] ^ data[12] ^ data[13] ^ data[15] ^ data[16] ^ data[17] ^ data[20] ^ data[21] ^ data[23] ^ data[24] ^ data[26] ^ data[28] ^ data[29] ^ data[30]);
    crc16[10] = (crcIn[0] ^ crcIn[1] ^ crcIn[2] ^ crcIn[5] ^ crcIn[6] ^ crcIn[8] ^ crcIn[9] ^ crcIn[11] ^ crcIn[13] ^ crcIn[14] ^ crcIn[15] ^ data[5] ^ data[9] ^ data[10] ^ data[13] ^ data[14] ^ data[16] ^ data[17] ^ data[18] ^ data[21] ^ data[22] ^ data[24] ^ data[25] ^ data[27] ^ data[29] ^ data[30] ^ data[31]);
    crc16[11] = (crcIn[1] ^ crcIn[2] ^ crcIn[3] ^ crcIn[6] ^ crcIn[7] ^ crcIn[9] ^ crcIn[10] ^ crcIn[12] ^ crcIn[14] ^ crcIn[15] ^ data[6] ^ data[10] ^ data[11] ^ data[14] ^ data[15] ^ data[17] ^ data[18] ^ data[19] ^ data[22] ^ data[23] ^ data[25] ^ data[26] ^ data[28] ^ data[30] ^ data[31]);
    crc16[12] = (crcIn[0] ^ crcIn[2] ^ crcIn[6] ^ crcIn[7] ^ crcIn[8] ^ crcIn[12] ^ crcIn[13] ^ crcIn[15] ^ data[0] ^ data[4] ^ data[7] ^ data[8] ^ data[15] ^ data[16] ^ data[18] ^ data[22] ^ data[23] ^ data[24] ^ data[28] ^ data[29] ^ data[31]);
    crc16[13] = (crcIn[0] ^ crcIn[1] ^ crcIn[3] ^ crcIn[7] ^ crcIn[8] ^ crcIn[9] ^ crcIn[13] ^ crcIn[14] ^ data[1] ^ data[5] ^ data[8] ^ data[9] ^ data[16] ^ data[17] ^ data[19] ^ data[23] ^ data[24] ^ data[25] ^ data[29] ^ data[30]);
    crc16[14] = (crcIn[1] ^ crcIn[2] ^ crcIn[4] ^ crcIn[8] ^ crcIn[9] ^ crcIn[10] ^ crcIn[14] ^ crcIn[15] ^ data[2] ^ data[6] ^ data[9] ^ data[10] ^ data[17] ^ data[18] ^ data[20] ^ data[24] ^ data[25] ^ data[26] ^ data[30] ^ data[31]);
    crc16[15] = (crcIn[2] ^ crcIn[3] ^ crcIn[5] ^ crcIn[9] ^ crcIn[10] ^ crcIn[11] ^ crcIn[15] ^ data[3] ^ data[7] ^ data[10] ^ data[11] ^ data[18] ^ data[19] ^ data[21] ^ data[25] ^ data[26] ^ data[27] ^ data[31]);
end
endfunction

    // Internal FSM States
	localparam	STATE_HEADER  = 3'b000,
		STATE_TIMESTAMP_MSB   = 3'b001,
		STATE_TIMESTAMP_LSB   = 3'b011,
		STATE_PAYLOAD         = 3'b111,
		STATE_FCS             = 3'b110,
        //NB_SAMPLES            = 10'd512;
        //NB_SAMPLES            = 9'd252;
        NB_SAMPLES            = 10'd256;

	reg[14:0]   r_seqnum;
	reg[63:0]   r_timestamp;
	reg[15:0]   r_count;
	reg[31:0]   count;
	reg[2:0]    r_state;
	reg[15:0]   r_crc_in;
	reg[7:0]   r_empty;

	assign o_read = (r_state == STATE_PAYLOAD) && (i_fifo_full == 1'b0) && (i_empty == 1'b0);

    // Main Process
    always @(posedge i_clk)
    begin
		//o_fifo_push <= 1'b0;

        if (i_reset) begin
            r_state <= STATE_PAYLOAD;
			r_count <= (NB_SAMPLES - 1);
            r_seqnum <= 0;
            r_timestamp <= 0;
			count <= 0;
			o_fifo_push <= 1'b0;
        end else if (i_fifo_full == 1'b0) begin
            case (r_state)
                STATE_HEADER: begin
					r_empty <= 0;
                    r_seqnum <= r_seqnum + 1;
                    o_fifo_push <= 1'b1;
                    o_fifo_data <= {16'hCAFE, 1'b0, r_seqnum};
                    r_state <= STATE_TIMESTAMP_MSB;
                end
                STATE_TIMESTAMP_MSB: begin
                    o_fifo_push <= 1'b1;
                    o_fifo_data <= r_timestamp[63:32];
                    r_state <= STATE_TIMESTAMP_LSB;
                end
                STATE_TIMESTAMP_LSB: begin
                    o_fifo_push <= 1'b1;
                    o_fifo_data <= r_timestamp[31:0];

                    r_count <= (NB_SAMPLES - 1);
                    r_state <= STATE_PAYLOAD;
					r_crc_in <= 16'hffff;
                end
				STATE_PAYLOAD: begin
					if (i_empty == 1'b0) begin
						// TODO should come from LVDS 
						r_timestamp <= r_timestamp + 250;
						o_fifo_push <= 1'b1;
						o_fifo_data <= i_data;
						r_crc_in <= crc16(r_crc_in, i_data);

						if (r_count == 0) begin
							r_state <= STATE_FCS;
						end else begin
							r_count <= r_count - 1;
						end
					end else begin
						o_fifo_push <= 1'b0;
						//o_fifo_data <= 32'hdeadbeef;
					end
				end
                STATE_FCS: begin
                    o_fifo_push <= 1'b1;
					o_fifo_data <= {16'hC0DE, r_crc_in};
                    r_state <= STATE_HEADER;
                end
            endcase
		end else begin
			//o_fifo_data <= 32'hdeadbeef;
        end
    end
endmodule
