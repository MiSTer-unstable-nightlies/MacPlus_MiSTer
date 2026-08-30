// floppy_write_committer.v
//
// Drain a checksum-valid sector recovered
// by floppy_track_decoder.v into SDRAM. This is the write-side twin of
// floppy_loader.v's DRAIN_FETCH/DRAIN_ASSERT/DRAIN_WAIT sequence, sharing
// the same extra-slot-3 port protocol (wr_addr/wr_data/wr_req/wr_ack) - see
// that module's header comment for why the port recurs roughly every 2us
// and why a whole-word commit sidesteps byte-granular SDRAM enables.
//
// floppy_track_decoder.v's buf_addr/buf_data is a REGISTERED (1-clk-
// latency) read port, the same idiom floppy_loader.v's own buf_rd uses:
// buf_data reflects whatever buf_addr was driven to one cycle earlier.
// Each 16-bit SDRAM word therefore needs its own one-cycle "let the read
// catch up" gap per byte, not just per word - FETCH_LO presents the even
// address and waits a cycle, FETCH_HI both captures the now-valid even
// byte AND presents the odd address, and ASSERT captures the now-valid
// odd byte and issues the word write. (An earlier version of this module
// paired with a combinational-read decoder port and captured one state
// earlier throughout - that pairing silently swapped every byte pair once
// the decoder's read was made registered instead, which is what motivated
// converting the decoder's buf_mem to a real block RAM in the first
// place. Keep this module and floppy_track_decoder.v's read latency in
// sync if either ever changes.)
//
// Byte-pair -> word packing matches the existing image convention (see
// floppy_loader.v's swap comment and MacPlus.sv's extra_rom_data_demux):
// the EVEN-addressed source byte lands in the word's high half, the ODD-
// addressed byte in the low half, so a later read via the same convention
// recovers the original byte order.
//
// This module is SDRAM-only - no sd_wr here. Persistence to the mounted
// .dsk is floppy_sd_writer.v, which taps this module's ASSERT-state word
// stream (below)
// rather than re-reading the decoder's buf_mem through a second port -
// floppy_track_decoder.v's buf_mem is a Simple Dual Port BRAM (one write,
// one read), so
// there is no second read port to give a persistence module of its own.
//
// Sector-to-sector spacing on real media (~16ms at 12 sectors/revolution)
// comfortably exceeds this module's own drain time (256 words at ~2us/slot
// grant =~ 512us), so a second sector_valid arriving mid-drain is not
// expected in practice; this module does not special-case it (IDLE only
// re-arms after DONE_PULSE).
module floppy_write_committer
(
	input         clk,
	input         rst,   // synchronous, active high - matches floppy_loader.v

	// from floppy_track_decoder
	input             sector_valid, // pulses 1 clk when a sector is ready to commit
	input      [21:0] sector_addr,  // decoder's `addr`: SDRAM byte offset of byte 0
	output reg [8:0]  buf_addr,     // drives decoder's buf_addr
	input      [7:0]  buf_data,     // decoder's registered buf_data, 1 clk after buf_addr

	// shared extra-slot-3 SDRAM write port (same protocol as floppy_loader.v)
	output reg [21:0] wr_addr,
	output reg [15:0] wr_data,
	output reg        wr_req,
	input             wr_ack,

	output            busy,
	output reg        done,        // one clk pulse: sector fully committed to SDRAM
	output     [21:0] committed_addr, // = base_addr; stable from capture through `done`

	// Persistence tap: mirrors every word as it is written to SDRAM (same
	// address/data/timing as wr_addr/wr_data/wr_req above), so a
	// persistence module can build a byte-exact shadow of the sector
	// without touching the decoder's single read port. wr_req is level-
	// true for however many cycles WAIT takes to see wr_ack - re-writing
	// the same (word_idx, wr_data) to a shadow buffer on every one of
	// those cycles is harmless, so this is a plain wire tap, not a state
	// machine of its own.
	output      [7:0] sd_buf_addr,
	output     [15:0] sd_buf_data,
	output             sd_buf_wr
);

	assign committed_addr = base_addr;
	assign sd_buf_addr    = word_idx;
	assign sd_buf_data    = wr_data;
	assign sd_buf_wr      = wr_req;

localparam IDLE       = 3'd0,
           FETCH_LO   = 3'd1,
           FETCH_HI   = 3'd2,
           ASSERT     = 3'd3,
           WAIT       = 3'd4,
           DONE_PULSE = 3'd5;
reg [2:0] state;

assign busy = (state != IDLE);

reg [21:0] base_addr;
reg [7:0]  word_idx; // 0..255 (512 bytes / 2)
reg [7:0]  byte_lo;

always @(*) begin
	case (state)
		FETCH_HI: buf_addr = {word_idx, 1'b1};
		default:  buf_addr = {word_idx, 1'b0}; // FETCH_LO and idle settle
	endcase
end

always @(posedge clk) begin
	done <= 1'b0; // default; pulsed explicitly below

	if (rst) begin
		state  <= IDLE;
		wr_req <= 1'b0;
	end else begin
		case (state)
		IDLE: if (sector_valid) begin
			base_addr <= sector_addr;
			word_idx  <= 8'd0;
			state     <= FETCH_LO;
		end

		// buf_addr (even) has been presented for this whole cycle; the
		// decoder's registered read captures it at this edge, valid from
		// next cycle - nothing to sample here yet.
		FETCH_LO: state <= FETCH_HI;

		// buf_data is now the even byte (captured at the FETCH_LO->FETCH_HI
		// edge). Capture it, while buf_addr (odd) is presented this whole
		// cycle for the decoder to capture in turn.
		FETCH_HI: begin
			byte_lo <= buf_data;
			state   <= ASSERT;
		end

		// buf_data is now the odd byte (captured at the FETCH_HI->ASSERT
		// edge). Pack the word and issue the write.
		ASSERT: begin
			wr_addr <= base_addr + {word_idx, 1'b0};
			wr_data <= {byte_lo, buf_data};
			wr_req  <= 1'b1;
			state   <= WAIT;
		end

		WAIT: if (wr_ack) begin
			wr_req <= 1'b0;
			if (word_idx == 8'd255) begin
				state <= DONE_PULSE;
			end else begin
				word_idx <= word_idx + 8'd1;
				state    <= FETCH_LO;
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
