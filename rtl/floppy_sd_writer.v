// floppy_sd_writer.v
//
// Persist every sector the guest writes out to the mounted .dsk on the SD
// card, via hps_io's sd_wr/sd_ack/sd_buff_addr block-device protocol.
//
// No copy of the data is kept here: floppy_write_committer.v has already
// landed the sector in SDRAM, so only the sector NUMBER is queued and the
// block is fetched from SDRAM at write time, through addrController_top.v's
// extra-slot-3 read port, into one 256x16 buffer that hps_io streams out.
// A sector re-committed while still queued is queued and written twice, with
// the latest data both times. A full queue refuses the push; it never
// overwrites a block in flight (a fixed-depth copy of the data did, when an
// O_SYNC card write stalled longer than two sectors).
//
// Protocol modelled on scsi.v's io_wr handshake: assert sd_wr with sd_lba
// valid, drop sd_wr as soon as sd_ack rises, then wait for sd_ack to fall
// before considering the sector handed off. sd_buff_din must stay valid,
// addressed by the HPS-driven sd_buff_addr, for the whole time sd_ack is high.
//
// img_mounted (eject/remount of THIS slot) aborts the writer: FSM to idle,
// both request lines low, queue emptied. sd_ack is shared per slot, so a
// running FSM would otherwise be walked forward by floppy_loader's acks on
// the new image.
module floppy_sd_writer #(
	parameter ACK_TIMEOUT_BITS = 24, // ~0.5s at clk_sys (~32MHz)
	parameter QDEPTH_BITS      = 10  // 1024 pending sectors
) (
	input         clk,
	input         reset,

	input         img_mounted, // this slot's own mount pulse - abort, see header

	// commit notice from floppy_write_committer, via floppy.v/iwm.v/
	// dataController_top.sv's dskCommit* ports
	input             commit_done,
	input      [21:0] commit_addr,  // image byte offset of sector byte 0

	input             readonly,     // this drive's latched img_readonly - refuse persistence outright
	input             loader_busy,  // don't touch the slot or the image while floppy_loader owns them

	// image length in 512-byte blocks; a commit at or beyond it is dropped
	input      [12:0] size_blocks,

	// extra-slot-3 read port: hold fetch_req with fetch_addr stable until
	// the one-clock fetch_ack, on which fetch_data is that word of the image
	output reg [21:0] fetch_addr,   // byte offset within THIS image, word-aligned
	output reg        fetch_req,
	input             fetch_ack,
	input      [15:0] fetch_data,

	output reg [31:0] sd_lba,
	output reg        sd_wr,
	input             sd_ack,

	input      [7:0]  sd_buff_addr, // HPS-driven shared read address
	output     [15:0] sd_buff_din,

	output            busy
);

	// the sector-number queue
	reg [12:0] q_mem [0:(1<<QDEPTH_BITS)-1];
	reg [QDEPTH_BITS:0] wr_ptr, rd_ptr;
	wire [QDEPTH_BITS:0] count = wr_ptr - rd_ptr;
	wire full  = count[QDEPTH_BITS];
	wire empty = (count == 0);
	reg  [12:0] q_head;    // registered read of q_mem[rd_ptr]
	reg         empty_d;   // q_head lags a push by one cycle; see P_IDLE

	wire accept = commit_done && !readonly;
	wire push   = accept && !full;

	always @(posedge clk) begin
		if (push) q_mem[wr_ptr[QDEPTH_BITS-1:0]] <= commit_addr[21:9];
		q_head  <= q_mem[rd_ptr[QDEPTH_BITS-1:0]];
		empty_d <= empty;
	end

	// the block buffer: filled from SDRAM, drained by hps_io
	reg [15:0] blk [0:255];
	reg        blk_we;
	reg  [7:0] blk_wa;
	reg [15:0] blk_wd;
	always @(posedge clk) if (blk_we) blk[blk_wa] <= blk_wd;

	reg [15:0] blk_do;
	always @(posedge clk) blk_do <= blk[sd_buff_addr];
	// SDRAM words carry the even source byte in the high half; hps_io's wire
	// format is the opposite (floppy_loader.v swaps the same way on the read side)
	assign sd_buff_din = {blk_do[7:0], blk_do[15:8]};

	localparam P_IDLE      = 3'd0,
	           P_SKIP      = 3'd1,  // refused entry: one cycle for q_head to move on
	           P_FILL      = 3'd2,  // fetching the block from SDRAM, word w
	           P_WR        = 3'd3,  // block in the buffer: present sd_wr
	           P_WAIT_ACK  = 3'd4,
	           P_WAIT_DONE = 3'd5;
	reg [2:0] pstate;

	reg [12:0] cur_sec;    // the block being written
	reg  [7:0] w;          // word index within it, during the fetch

	// bound on P_WAIT_ACK: an sd_wr HPS never services is re-presented, not
	// retired (see P_WAIT_ACK); ~0.5s, far beyond any real sd_ack latency
	reg [ACK_TIMEOUT_BITS-1:0] ackTimer;
	wire ackTimeout = &ackTimer;

	wire lba_in_range = (size_blocks != 13'd0) && (q_head < size_blocks);

	assign busy = (pstate != P_IDLE) || !empty;

	always @(posedge clk) begin
		blk_we <= 1'b0;
		if (reset) begin
			pstate     <= P_IDLE;
			sd_lba     <= 32'd0;
			sd_wr      <= 1'b0;
			fetch_addr <= 22'd0;
			fetch_req  <= 1'b0;
			wr_ptr     <= 0;
			rd_ptr     <= 0;
			cur_sec    <= 13'd0;
			w          <= 8'd0;
			ackTimer   <= 0;
		end else begin
			// capture side: independent of pstate
			if (push) wr_ptr <= wr_ptr + 1'd1;

			case (pstate)
			// q_head is valid once the queue has been non-empty for two cycles
			P_IDLE: if (!empty && !empty_d && !loader_busy) begin
				rd_ptr <= rd_ptr + 1'd1;
				if (!lba_in_range)
					// nothing upstream checks the LBA against the image length
					pstate <= P_SKIP;
				else begin
					cur_sec <= q_head;
					w       <= 8'd0;
					pstate  <= P_FILL;
				end
			end

			// a pop that stayed here would pop again against the stale head
			P_SKIP: pstate <= P_IDLE;

			// one word per slot-3 grant; addr and req change together and hold
			P_FILL: if (!fetch_req) begin
				fetch_addr <= {cur_sec, w, 1'b0};
				fetch_req  <= 1'b1;
			end else if (fetch_ack) begin
				blk_we    <= 1'b1;
				blk_wa    <= w;
				blk_wd    <= fetch_data;
				fetch_req <= 1'b0;
				w         <= w + 8'd1;
				if (w == 8'd255) pstate <= P_WR;
			end

			P_WR: begin
				sd_lba <= {19'd0, cur_sec};
				sd_wr  <= 1'b1;
				pstate <= P_WAIT_ACK;
			end

			P_WAIT_ACK: if (sd_ack) begin
				sd_wr  <= 1'b0; // mirrors scsi.v: io_wr drops as soon as io_ack rises
				pstate <= P_WAIT_DONE;
			end else if (ackTimeout) begin
				// re-present the SAME block: hps_io captures sd_lba in one poll
				// and acks in a later one, so a retired entry plus a late ack
				// would stream the next block to the captured LBA
				sd_wr  <= 1'b0;
				pstate <= P_WR;
			end else
				ackTimer <= ackTimer + 1'b1;

			P_WAIT_DONE: if (!sd_ack) pstate <= P_IDLE;

			default: pstate <= P_IDLE;
			endcase

			if (pstate != P_WAIT_ACK) ackTimer <= 0;

			// remount abort, after the case so it wins this cycle
			if (img_mounted) begin
				wr_ptr    <= 0;
				rd_ptr    <= 0;
				sd_wr     <= 1'b0;
				fetch_req <= 1'b0;
				blk_we    <= 1'b0;
				pstate    <= P_IDLE;
			end
		end
	end

endmodule
