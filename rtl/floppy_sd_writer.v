// floppy_sd_writer.v
//
// Persist a checksum-valid, SDRAM-
// committed sector (see floppy_write_committer.v) out to the mounted
// .dsk on the SD card, via hps_io's sd_wr/sd_ack/sd_buff_addr block-
// device protocol.
//
// Protocol modelled on scsi.v's io_wr handshake (the only sd_wr producer
// already proven on this core): assert sd_wr with sd_lba valid, drop
// sd_wr as soon as sd_ack rises (hps_io has accepted the request and is
// now stepping sd_buff_addr through the block), then wait for sd_ack to
// fall again before considering the sector durably handed off - mirroring
// floppy_loader.v's own SD_WAIT_ACK/SD_WAIT_DONE split for the read side.
// sd_buff_din for this slot must stay valid, addressed by the HPS-driven
// shared sd_buff_addr bus, for the whole time sd_ack is high.
//
// A checksum-valid sector never depends on rotational position, so this
// module has no notion of "gap" either
// - the queue below only ever fills on commit_done. It also reacts to
// img_mounted (eject/remount of THIS slot): the queue is dropped so a
// sector captured against the old image never lands, at the old image's
// stale LBA, in whatever gets mounted next - see the img_mounted handling
// below for why this is safe against an in-flight sd_wr.
//
// Backpressure: the CPU-facing write path can only produce a new
// commit_done roughly once per sector's worth of 16us-paced IWM bytes
// (~700 encoded bytes => >10ms), comfortably longer than a real sd_wr
// transfer. But an sd_wr can occasionally run long (SD card stalls,
// wear-levelling, etc.), so a single in-flight slot is not "never drop a
// commit that arrives while the previous one is in flight" - hence the
// 2-entry queue below (two independent shadow buffers, ping-ponged by
// `tail` on capture and drained in order via `head`). A third commit
// landing before the first of the previous two has finished draining
// would have nowhere new to go and reuses the still-in-flight buffer -
// accepted as a documented, not-expected-in-practice limit (the same
// idealization writeUnderrun and the committer's own single-sector-in-
// flight assumption already accept elsewhere in this write path), not a
// silently-corrupting one: capture and drain never touch the same buffer
// under normal (non-overflowing) operation, so the failure mode is stale
// data reaching one sd_wr, not a torn transfer.
module floppy_sd_writer #(
	parameter ACK_TIMEOUT_BITS = 24 // ~0.5s at clk_sys (~32MHz); sim overrides this narrower
) (
	input         clk,
	input         reset,

	input         img_mounted, // this slot's own mount pulse - drop the queue, see header

	// commit tap from floppy_write_committer, via floppy.v/iwm.v/
	// dataController_top.sv's dskCommit* ports
	input             commit_done,
	input      [21:0] commit_addr,     // image byte offset of sector byte 0
	input             commit_buf_wr,
	input      [7:0]  commit_buf_addr, // word index 0..255
	input      [15:0] commit_buf_data,

	input             readonly,     // this drive's latched img_readonly - refuse persistence outright
	input             loader_busy,  // don't start a new sd_wr while floppy_loader owns this slot

	// Size of the mounted image in 512-byte blocks (floppy_loader's own
	// loaded_size >> 9, latched at that slot's mount). Any commit landing
	// at or beyond this is dropped rather than written - see P_IDLE.
	input      [12:0] size_blocks,

	output reg [31:0] sd_lba,
	output reg        sd_wr,
	input             sd_ack,

	input      [7:0]  sd_buff_addr, // HPS-driven shared read address
	output     [15:0] sd_buff_din,

	output            busy
);

	// two independent 256x16 shadow sectors - see header for why two.
	reg [15:0] mem0 [0:255];
	reg [15:0] mem1 [0:255];

	reg        tail;       // buffer currently receiving commit_buf_wr taps
	reg        head;       // buffer currently queued/draining to sd_wr
	reg  [1:0] valid;      // per-buffer: queued or in-flight, not yet drained
	reg [21:0] addr_q [0:1];

	always @(posedge clk) begin
		if (commit_buf_wr) begin
			if (tail == 1'b0) mem0[commit_buf_addr] <= commit_buf_data;
			else              mem1[commit_buf_addr] <= commit_buf_data;
		end
	end

	reg [15:0] mem0_do, mem1_do;
	always @(posedge clk) mem0_do <= mem0[sd_buff_addr];
	always @(posedge clk) mem1_do <= mem1[sd_buff_addr];
	wire [15:0] mem_do = (head == 1'b0) ? mem0_do : mem1_do;
	// mem0/mem1 hold commit_buf_data, which is already in the internal
	// SDRAM word convention (even source byte in the high half - see
	// floppy_write_committer.v). hps_io's sd_buff_din/dout wire format is
	// the opposite half-order (see floppy_loader.v's matching swap on the
	// read side), so this swap must mirror that one or every written byte
	// pair comes out transposed in the .dsk on disk.
	assign sd_buff_din = {mem_do[7:0], mem_do[15:8]};

	localparam P_IDLE      = 2'd0,
	           P_WAIT_ACK  = 2'd1,
	           P_WAIT_DONE = 2'd2;
	reg [1:0] pstate;

	// P_WAIT_ACK has no bound otherwise: if this slot's sd_wr is ever
	// asserted while HPS isn't servicing it (framework quirk, a mount race
	// on the shared slot, etc.) sd_ack never rises and this module would
	// wedge in P_WAIT_ACK forever with busy stuck high (busy feeds
	// LED_USER). ACK_TIMEOUT_BITS defaults to ~0.5s at clk_sys (~32MHz) -
	// far longer than any real sd_ack latency, so it never fires in normal
	// operation. On expiry the request is dropped and re-presented, NOT
	// retired - see P_WAIT_ACK below for why abandoning the queue entry
	// here would be a data-corruption path rather than a recovery.
	reg [ACK_TIMEOUT_BITS-1:0] ackTimer;
	wire ackTimeout = &ackTimer;

	wire [12:0] lba_head     = addr_q[head][21:9];
	wire        lba_in_range = (size_blocks != 13'd0) && (lba_head < size_blocks);

	assign busy = (pstate != P_IDLE) || valid[0] || valid[1];

	always @(posedge clk) begin
		if (reset) begin
			pstate <= P_IDLE;
			sd_lba <= 32'd0;
			sd_wr  <= 1'b0;
			valid  <= 2'b00;
			head   <= 1'b0;
			tail   <= 1'b0;
			ackTimer <= 0;
		end else begin
			// capture side: independent of pstate, always ready to accept
			// the next commit (see header re: the depth-2 queue's limit).
			if (commit_done && !readonly) begin
				valid[tail]  <= 1'b1;
				addr_q[tail] <= commit_addr;
				tail         <= ~tail;
			end

			// Eject-race interlock: a fresh mount of THIS slot drops
			// whatever is still queued (not yet started) - it was captured
			// against the image that is now gone. Deliberately does NOT
			// touch pstate/sd_wr/head: an sd_wr already mid-flight
			// (P_WAIT_ACK/P_WAIT_DONE) keeps running exactly as it would
			// otherwise, since tearing down a request hps_io may already
			// be servicing is worse than letting one stale sector finish -
			// only `valid` gates entry into a NEW P_IDLE->P_WAIT_ACK
			// transition, so clearing it here can only stop sectors that
			// have not started, never interrupt one that has. `tail` is
			// rewound to `head` so the very next capture cannot land in
			// whichever buffer is still draining (the same buffer-reuse
			// idioms the depth-2 queue already documents above, not a new
			// hazard).
			if (img_mounted) begin
				valid <= 2'b00;
				tail  <= head;
			end

			case (pstate)
			P_IDLE: if (valid[head] && !loader_busy) begin
				// byte offset -> LBA (512B/sector). The decoder bounds-
				// checks the SECTOR number against this track's spt, but
				// nothing upstream checks the resulting LBA against the
				// mounted image's actual length - `track` is free to reach
				// 0x4F regardless of image size. This is the last place
				// that can refuse, and it is cheap, so refuse here rather
				// than hand hps_io an offset past the end of the file.
				if (lba_in_range) begin
					sd_lba <= {19'd0, lba_head};
					sd_wr  <= 1'b1;
					pstate <= P_WAIT_ACK;
				end else begin
					// out of range: retire without writing anything
					valid[head] <= 1'b0;
					head        <= ~head;
				end
			end

			P_WAIT_ACK: if (sd_ack) begin
				sd_wr  <= 1'b0; // mirrors scsi.v: io_wr drops as soon as io_ack rises
				pstate <= P_WAIT_DONE;
			end else if (ackTimeout) begin
				// Drop the request and RE-PRESENT it - deliberately without
				// clearing valid[head] or advancing `head`. Retiring the
				// entry here is not safe: hps_io captures sd_lba during its
				// own poll command and raises sd_ack in a LATER, separate
				// command, so there is no bound on the gap between the two.
				// If the entry were retired and `head` flipped, a late
				// sd_ack would stream the OTHER buffer out to the LBA the
				// HPS had already captured - a full sector of unrelated
				// data written at a perfectly valid offset in the .dsk.
				// Leaving head/valid/sd_lba alone makes the retry idempotent
				// instead: however late the ack arrives, and whichever
				// attempt it belongs to, it transfers the same buffer to the
				// same LBA. `busy` stays high while a write is genuinely
				// still owed, which is what the LED should show anyway.
				sd_wr  <= 1'b0;
				pstate <= P_IDLE;
			end else
				ackTimer <= ackTimer + 1'b1;

			P_WAIT_DONE: if (!sd_ack) begin
				valid[head] <= 1'b0;
				head        <= ~head;
				pstate      <= P_IDLE;
			end

			default: pstate <= P_IDLE;
			endcase

			if (pstate != P_WAIT_ACK) ackTimer <= 0;
		end
	end

endmodule
