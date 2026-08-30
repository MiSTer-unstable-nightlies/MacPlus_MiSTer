// Mount-time floppy image loader.
//
// Floppies were previously a one-way ioctl_download blob into SDRAM. This
// module replaces that with a real SD
// block-device mount: on img_mounted it streams the whole image in via
// sd_rd, sector by sector, into the SAME SDRAM byte offsets the read side
// (dskReadAddrInt/Ext in addrController_top.v) already expects, so nothing
// downstream of SDRAM changes. Still read-only - no sd_wr, no write-back.
//
// Each sector is staged into a small local BRAM as it streams in (sd_buff_wr
// has no rate limit against on-chip RAM), then drained out to SDRAM one word
// at a time through the shared extra-slot-3 port in addrController_top.v -
// that slot recurs roughly every 2us, so draining is the slow half of a
// mount (a 1600-sector 800K image takes on the order of a second). Load-
// then-drain per sector, not double-buffered - correctness first; the gate
// here is booting exactly as before, not load speed.
//
// `done` (and therefore the caller's insertDisk latch) does not fire until
// the whole image is resident, so the Mac can never observe a disk that is
// only partially loaded - the SD-mount equivalent of the end-of-download
// latch the old ioctl_download path used.
module floppy_loader
(
	input         clk_sys,
	input         reset,

	input         img_mounted,  // this slot's one-shot mount pulse
	input  [63:0] img_size,     // valid at img_mounted
	input         img_readonly, // valid at img_mounted

	output reg [31:0] sd_lba,
	output reg        sd_rd,
	input              sd_ack,

	input        [7:0] sd_buff_addr,
	input       [15:0] sd_buff_dout,
	input               sd_buff_wr,

	// shared extra-slot-3 SDRAM write port (see addrController_top.v)
	output reg  [21:0] wr_addr, // byte offset within THIS image, word-aligned
	output reg  [15:0] wr_data,
	output reg          wr_req,
	input                wr_ack,

	output reg          done,          // one clk_sys pulse: image now fully resident
	output reg  [63:0]  loaded_size,   // img_size, latched at this slot's own mount
	output reg           readonly_latched,
	output              busy
);

localparam IDLE         = 3'd0,
           SD_ASSERT    = 3'd1,
           SD_WAIT_ACK  = 3'd2,
           SD_WAIT_DONE = 3'd3,
           DRAIN_FETCH  = 3'd4,
           DRAIN_ASSERT = 3'd5,
           DRAIN_WAIT   = 3'd6,
           DONE_PULSE   = 3'd7;
reg [2:0] state;

assign busy = (state != IDLE);

reg [15:0] buf_mem [0:255];
reg [15:0] buf_rd;
always @(posedge clk_sys) buf_rd <= buf_mem[word_idx];

reg  [7:0] word_idx;   // 0..255 within the current sector
reg [10:0] sector;     // sector index within the image (up to 1600 for 800K)
reg [10:0] nsect;      // total sectors, latched at mount

// Latch every mount request unconditionally, mirroring the UK101
// disk_reader.sv precedent - a mount
// arriving while a previous load is still draining must not be dropped.
reg mount_pending;
always @(posedge clk_sys) begin
	if (reset) mount_pending <= 1'b0;
	else if (img_mounted && img_size != 0) mount_pending <= 1'b1;
	else if (state == IDLE && mount_pending) mount_pending <= 1'b0;
end

always @(posedge clk_sys) begin
	done <= 1'b0; // default; pulsed explicitly below

	if (reset) begin
		state   <= IDLE;
		sd_rd   <= 1'b0;
		wr_req  <= 1'b0;
	end else begin
		case (state)
		IDLE: if (mount_pending) begin
			loaded_size      <= img_size;
			readonly_latched <= img_readonly;
			nsect            <= img_size[19:9]; // img_size / 512
			sector           <= 11'd0;
			state            <= SD_ASSERT;
		end

		SD_ASSERT: begin
			sd_lba <= {21'd0, sector};
			sd_rd  <= 1'b1;
			state  <= SD_WAIT_ACK;
		end

		SD_WAIT_ACK: if (sd_ack) state <= SD_WAIT_DONE;

		SD_WAIT_DONE: begin
			// hps_io's sd_buff_dout and the ROM-download ioctl_dout both come
			// from the same raw HPS word (io_din in hps_io.sv) - the existing
			// ROM download path byte-swaps it before writing to SDRAM
			// (`dio_data <= {ioctl_data[7:0], ioctl_data[15:8]}` below), and
			// the read side (extra_rom_data_demux in MacPlus.sv) expects that
			// same convention. Missing this swap here silently transposes
			// every byte pair in every mounted image.
			if (sd_buff_wr && sd_ack) buf_mem[sd_buff_addr] <= {sd_buff_dout[7:0], sd_buff_dout[15:8]};
			if (!sd_ack) begin
				sd_rd    <= 1'b0;
				word_idx <= 8'd0;
				state    <= DRAIN_FETCH;
			end
		end

		// One cycle for buf_rd to catch up to buf_mem[word_idx] before the
		// first DRAIN_ASSERT reads it - buf_rd is a registered (one-cycle-
		// latency) BRAM read, so reading it on the same cycle word_idx
		// changes would serve the PREVIOUS word.
		DRAIN_FETCH: state <= DRAIN_ASSERT;

		DRAIN_ASSERT: begin
			wr_addr <= {sector, 9'd0} + {word_idx, 1'b0};
			wr_data <= buf_rd;
			wr_req  <= 1'b1;
			state   <= DRAIN_WAIT;
		end

		DRAIN_WAIT: if (wr_ack) begin
			wr_req <= 1'b0;
			if (word_idx == 8'd255) begin
				if (sector == nsect - 11'd1) state <= DONE_PULSE;
				else begin
					sector <= sector + 11'd1;
					state  <= SD_ASSERT;
				end
			end else begin
				word_idx <= word_idx + 8'd1;
				state    <= DRAIN_FETCH;
			end
		end

		DONE_PULSE: begin
			done  <= 1'b1;
			state <= IDLE;
		end

		default: state <= IDLE;
		endcase
	end
end

endmodule
