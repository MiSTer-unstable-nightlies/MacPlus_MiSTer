/* verilator lint_off UNUSED */

/* based on minimigmac by Benjamin Herrenschmidt */

/* Read registers */
`define RREG_CDR        3'h0    /* Current SCSI data */
`define RREG_ICR        3'h1    /* Initiator Command */
`define RREG_MR         3'h2    /* Mode register */
`define RREG_TCR        3'h3    /* Target Command */
`define RREG_CSR        3'h4    /* SCSI bus status */
`define RREG_BSR        3'h5    /* Bus and status */
`define RREG_IDR        3'h6    /* Input data */
`define RREG_RST        3'h7    /* Reset */

/* Write registers */
`define WREG_ODR        3'h0    /* Output data */
`define WREG_ICR        3'h1    /* Initiator Command */
`define WREG_MR         3'h2    /* Mode register */
`define WREG_TCR        3'h3    /* Target Command */
`define WREG_SER        3'h4    /* Select Enable */
`define WREG_DMAS       3'h5    /* Start DMA Send */
`define WREG_DMATR      3'h6    /* Start DMA Target receive */
`define WREG_IDMAR      3'h7    /* Start DMA Initiator receive */

/* MR bit numbers */
`define MR_DMA_MODE     1
`define MR_ARB          0

/* ICR bit numbers */
`define ICR_A_RST       7
`define ICR_TEST_MODE   6
`define ICR_DIFF_ENBL   5
`define ICR_A_ACK       4
`define ICR_A_BSY       3
`define ICR_A_SEL       2
`define ICR_A_ATN       1
`define ICR_A_DATA      0

/* TCR bit numbers */
`define TCR_A_REQ       3
`define TCR_A_MSG       2
`define TCR_A_CD        1
`define TCR_A_IO        0

module ncr5380
(
	input    		clk,
	input 	     	reset,

	/* Bus interface. 3-bit address, to be wired
	 * appropriately upstream (to A4..A6) plus one
	 * more bit (A9) wired as dack.
	 */
	input         bus_cs,
	input   [2:0] bus_rs,
	input         ior,
	input         iow,
	input         dack,
	output        dreq,
	input   [7:0] wdata,
	output  [7:0] rdata,

	// connections to io controller
	input  [DEVS-1:0] img_mounted,
	input      [31:0] img_size,
	
	output reg [31:0] io_lba[DEVS],
	output [DEVS-1:0] io_rd,
	output [DEVS-1:0] io_wr,
	input  [DEVS-1:0] io_ack,

	input        [7:0] sd_buff_addr,
	input        [4:0] sd_buff_addr_hi, // hps_io addr[12:8]; CD-DA frames are
	                                   // 2352 bytes = 1176 words > 8 bits
	input       [15:0] sd_buff_dout,
	output      [15:0] sd_buff_din[DEVS],
	input              sd_buff_wr,

	// CD-ROM drive present on the bus (see scsi.v's cd_enable). Only meaningful
	// when CD_DEV is in range; low makes the bus identical to a disks-only build.
	input              cd_enable,

	// CD-DA sample pair from the CD target's audio engine. Exact zeros when
	// not playing. Summed into the Mac's own audio by rtl/cd_mix.v.
	output signed [15:0] cd_snd_l,
	output signed [15:0] cd_snd_r

	// CPU BUS HOLD-OFF -- the fix for the advisory-frontier defect.
	//
	// Everything else in this file serves a DACK access unconditionally: rdata
	// returns cur_data on any DACK read, and dma_ack is gated on the bus phase
	// alone, never on scsi_req. So REQ withdrawal is ADVISORY -- a blind
	// pseudo-DMA pump (which is what the Mac Plus SCSI Manager runs) walks
	// through the fetch frontier and is served the ring slot's previous
	// occupant. That is the 2026-08-26 CD->disk corruption.
	//
	// Rather than teach three separate places to refuse an ACK, this makes the
	// pump STOPPABLE: MacPlus.sv withholds DTACK while this is high, so the 68000
	// inserts wait states until the target can actually serve the byte. That is
	// what the SE's glue did with DRQ, and what a period drive's sustained-
	// streaming guarantee did on the Plus -- a guarantee an HPS-backed target
	// cannot make, which is why the interlock has to live here instead.
	//
	// Safe against a fetch that never completes: the target's io-stall watchdog
	// (scsi.v, ~516 ms) aborts the command, which moves the phase out of DATA and
	// so drops data_holdoff. Worst case is a bounded stall ending in CHECK
	// CONDITION, never a hang.
	,output            bus_hold
);
	parameter DEVS = 2;
	// Index of the CD-ROM target within the DEVS arrays, or DEVS for "none".
	// The disks keep indices 0..n at IDs 6,5,... exactly as before; the CD takes
	// the last index at ID 3.
	parameter CD_DEV = DEVS;
	// Target bus-watchdog period, forwarded to scsi.v. Benches shorten it so a
	// timeout is reachable in simulation; synthesis keeps the ~129 ms default.
	parameter WDOG_LOG = 22;
	// Stall timeout for an HPS fetch that never completes; see scsi.v.
	parameter IOWDOG_LOG = 24;
	// CD spin-up window; see the spin-up block in scsi.v.
	parameter SPINUP_LOG = 27;

	// The one definition of "the target is offering bytes for a data phase".
	// Declared here because dreq, dma_ack, bsr_dmarq and bsr_eodma all key off
	// it; see bsr_dmarq below for why the DMA handshake is gated on the BUS
	// phase rather than on TCR.
	wire bus_data_phase = scsi_bsy & ~scsi_cd & ~scsi_msg;

	// NOT gated on bus_data_phase, and that is deliberate. A real 5380 does
	// inhibit DRQ on a phase mismatch, and two builds that did so both hung the
	// machine. The live capture from the second (build 2187326, md5-confirmed)
	// shows the driver
	// arming pseudo-DMA in a LEGITIMATE DATA-IN phase and then performing zero
	// DACK reads until the target's bus watchdog fired at 129 ms, leaving it
	// polling BSR=0x98 -- DRQ clear -- forever. Hiding DRQ from this driver is
	// what breaks it. The confirmed defect is fixed by gating dma_ack alone.
	assign dreq = scsi_req & dma_en;

	// Hold the CPU only on a pseudo-DMA (DACK) data access that cannot be served.
	// Register accesses are NEVER held: the driver learns about the frontier by
	// polling BSR.DRQ, so stalling that poll would deadlock the very initiator
	// this is meant to protect. Qualified on dma_en to match dma_ack's own gate --
	// a DACK access with pseudo-DMA unarmed cannot consume a byte anyway.
	// An idle target is not in a data phase, so |target_holdoff is exactly the
	// selected target's indication.
	assign bus_hold = bus_cs & dack & (ior | iow) & dma_en & (|target_holdoff);

	reg  [7:0] mr;        /* Mode Register */
	reg  [7:0] icr;       /* Initiator Command Register */
	reg  [3:0] tcr;       /* Target Command Register */
	wire [7:0] csr;       /* SCSI bus status register */

	/* Data in and out latches and associated
	* control logic for DMA
	*/
	reg  [7:0] din;
	reg  [7:0] dout;
	reg        dma_en;

	/* --- Main host-side interface --- */

	/* Register & DMA accesses decodes */
	reg dma_wr;
	reg reg_wr;
	reg dma_ack;

	wire i_dma_rd = bus_cs &  dack & ior;
	wire i_dma_wr = bus_cs &  dack & iow;
	wire i_reg_wr = bus_cs & ~dack & iow;

	always @(posedge clk) begin
		reg old_dma_rd, old_dma_wr, old_reg_wr;

		old_dma_rd <= i_dma_rd;
		old_dma_wr <= i_dma_wr;
		old_reg_wr <= i_reg_wr;

		dma_wr <= 0;
		dma_ack <= 0;
		reg_wr <= 0;

		if(~old_dma_wr & i_dma_wr) dma_wr <= 1;
		if(~old_reg_wr & i_reg_wr) reg_wr <= 1;
		// Gated on the BUS PHASE. A DACK access must not ACK a byte the target
		// is offering from STATUS or MESSAGE -- that is the confirmed defect
		// (hardware 2026-08-22: ACK-in-STATUS=1, no watchdog, bus free).
		// Keyed on the phase rather than on bsr_pmatch because the Plus driver
		// does not maintain TCR across a write data phase.
		if((old_dma_wr & ~i_dma_wr) | (old_dma_rd & ~i_dma_rd)) dma_ack <= dma_en & bus_data_phase;
	end

	/* System bus reads */
	assign rdata = dack                ? cur_data         :
	               bus_rs == `RREG_CDR ? cur_data         :
	               bus_rs == `RREG_ICR ? icr_read         :
	               bus_rs == `RREG_MR  ? mr               :
	               bus_rs == `RREG_TCR ? { 4'h0, tcr }    :
	               bus_rs == `RREG_CSR ? csr              :
	               bus_rs == `RREG_BSR ? bsr              :
	               bus_rs == `RREG_IDR ? cur_data         :
	               bus_rs == `RREG_RST ? 8'hff            :
	               8'hff;

	/* Data out latch (in DMA mode, this is one cycle after we've
	* asserted ACK)
	*/
	always@(posedge clk) if((reg_wr && bus_rs == `WREG_ODR) || dma_wr) dout <= wdata;

	/* Current data register. Simplified logic: We loop back the
	* output data if we are asserting the bus, else we get the
	* input latch
    */
	wire [7:0] cur_data = out_en ? dout : din;

	/* Logic for "asserting the bus" simplified */
	wire       out_en = icr[`ICR_A_DATA] | mr[`MR_ARB];

	/* ICR read wires */
	wire [7:0] icr_read = { icr[`ICR_A_RST],
	                        icr_aip,
	                        icr_la,
	                        icr[`ICR_A_ACK],
	                        icr[`ICR_A_BSY],
	                        icr[`ICR_A_SEL],
	                        icr[`ICR_A_ATN],
	                        icr[`ICR_A_DATA] };

	/* ICR write */
	always@(posedge clk or posedge reset) begin
		if (reset) begin
			icr <= 0;
		end else if (reg_wr && (bus_rs == `WREG_ICR)) begin
			icr <= wdata;
		end
	end
   
	/* MR write */
	always@(posedge clk or posedge reset) begin
		if (reset) mr <= 8'b0;
		else if (reg_wr && (bus_rs == `WREG_MR)) mr <= wdata;
	end
   
	/* TCR write */
	always@(posedge clk or posedge reset) begin
		if (reset) tcr <= 4'b0;
		else if (reg_wr && (bus_rs == `WREG_TCR)) tcr <= wdata[3:0];
	end
   
	/* DMA start send & receive registers. We currently ignore
	* the direction.
	*/
	always@(posedge clk or posedge reset) begin
		if (reset) begin
			dma_en <= 0;
		end else begin
			if (!mr[`MR_DMA_MODE]) begin
				dma_en <= 0;
			end else if (reg_wr && (bus_rs == `WREG_DMAS)) begin
				dma_en <= 1;
			end else if (reg_wr && (bus_rs == `WREG_IDMAR)) begin
				dma_en <= 1;
			end
		end
	end

	/* CSR (read only). We don't do parity */
	assign csr = { scsi_rst, scsi_bsy, scsi_req & ~req_deferred, scsi_msg,
	               scsi_cd, scsi_io, scsi_sel, 1'b0 };	

	/* Bus and Status register */
	/* BSR (read only). We don't do a few things... */
	wire bsr_eodma = ~bus_data_phase;	/* asserted whenever NOT in a data phase */
	/* A real 5380 inhibits DRQ, and halts the DMA handshake, when REQ arrives
	 * in a phase the initiator did not arm for. That inhibition is the exit ramp
	 * for "the target CHECKed instead of entering the data phase": REQ stays
	 * visible in CSR, the driver's poll loop exits through its REQ test, and the
	 * SCSI Manager handles the phase change. Without it a blind pump loop ACKs
	 * the STATUS byte as sector data and the transaction ends invisibly
	 * (hardware confirmed: ACK-in-STATUS with no watchdog fire).
	 *
	 * Keyed on the BUS PHASE, not on bsr_pmatch: gating on TCR broke every
	 * pseudo-DMA WRITE, because the Plus driver does not maintain TCR across a
	 * write data phase. This is the same condition bsr_eodma reports,
	 * so the two cannot disagree.
	 *
	 * NOTE, 2026-08-22: this is the review's PRIMARY finding, implemented ALONE.
	 * The two earlier attempts each changed this gate AND the completion-IRQ
	 * latch (both made it level-triggered, on different conditions), and both
	 * hung the machine earlier than the bug. Attempt 1's latch (!bsr_pmatch)
	 * demonstrably fires in COMMAND phase -- BSR=0x98 mid-write on hardware.
	 * Attempt 2's (cd & io) cannot, so that capture's identical counters are
	 * still UNEXPLAINED. Hence: gate alone, latch untouched, one variable.
	 */
	wire bsr_dmarq = scsi_req & dma_en;	/* see dreq: DRQ stays visible */
	wire bsr_perr = 1'b0;	/* We don't do parity */
	wire bsr_irq = irq_latch;	/* latched completion IRQ, cleared by a reg-7 read */
	wire bsr_pmatch = 
	         tcr[`TCR_A_MSG] == scsi_msg &&
	         tcr[`TCR_A_CD ] == scsi_cd  &&
	         tcr[`TCR_A_IO ] == scsi_io;

	/* BUSY ERROR (BSR bit 2). A real 5380 LATCHES this when BSY is lost while
	 * DMA mode is armed -- it is the driver's "your target has vanished, stop
	 * waiting" exit ramp. It was hardwired to 0 here with an open question
	 * attached, and that question turned out to matter: it is why the 2026-08-28
	 * CD-at-boot hang never ended. The target aborts correctly and releases BSY
	 * (proved in sim, which also shows the target is
	 * REUSABLE straight afterwards), and the ROM was left polling a BSR in which
	 * no bit could ever change -- BSR=0x90 forever, frozen "?" diskette, no
	 * volume mounted. The freeze was never the target giving up; it was the
	 * initiator having no way to learn that it had.
	 *
	 * Cleared exactly as irq_latch is -- a reg-7 read or a bus reset -- so a
	 * driver that resets the chip between commands sees no residue. Implemented
	 * ALONE, latch untouched: see the 2026-08-22 note above, where two attempts
	 * that moved this gate AND the completion-IRQ latch together both hung the
	 * machine earlier than the bug they were chasing.
	 */
	reg  berr_latch;
	wire bsr_berr = berr_latch;
	wire [7:0] bsr = { bsr_eodma, bsr_dmarq, bsr_perr, bsr_irq,
	                   bsr_pmatch, bsr_berr, scsi_atn, scsi_ack };

   /* --- Simulated SCSI Signals --- */

   /* BSY logic (simplified arbitration, see notes) */
	wire scsi_bsy = 
	    icr[`ICR_A_BSY] |
	    |target_bsy |
	    //scsi2_bsy |
	    //scsi6_bsy |
	    mr[`MR_ARB];

	/* Remains of simplified arbitration logic */
	wire icr_aip = mr[`MR_ARB];
	wire icr_la = 0;

	/* Other ORed SCSI signals */
	wire scsi_sel = icr[`ICR_A_SEL];
	wire scsi_rst = icr[`ICR_A_RST];
	wire scsi_ack = icr[`ICR_A_ACK] | dma_ack;
	wire scsi_atn = icr[`ICR_A_ATN];

	/* Mux target signals */
	reg scsi_cd, scsi_io, scsi_msg, scsi_req;

	always @* begin
		integer i;
		scsi_cd = 0;
		scsi_io = 0;
		scsi_msg = 0;
		scsi_req = 0;
		din = 8'h55;

		for (i = 0; i < DEVS; i = i + 1) begin
			if (target_bsy[i]) begin
				scsi_cd = target_cd[i];
				scsi_io = target_io[i];
				scsi_msg = target_msg[i];
				scsi_req = target_req[i];
				din = target_dout[i];
			end
		end
	end

	/* ---- 5380 completion semantics (ported from MacLC_MiSTer) -------------
	 * Three status behaviours the SCSI Manager and Apple's CD driver depend
	 * on, all previously stubbed out here. MacLC's ncr5380.sv documents each
	 * as a fix for a System 7 "Welcome to Macintosh" wedge.
	 */

	/* Register-space read strobes (never the DACK/pseudo-DMA window) */
	wire csr_rd = bus_cs & ~dack & ~iow & (bus_rs == `RREG_CSR);
	wire rst_rd = bus_cs & ~dack & ~iow & (bus_rs == `RREG_RST);

	/* Deferred bus-visible REQ (Snow controller.rs `set_req` semantics).
	 * The SCSI Manager's between-chunk settle loop
	 *     btst #5,CSR / beq exit / btst #3,BSR / bne loop
	 * exits only when a CSR read returns REQ=0. A real 5380 driving a real
	 * drive gives it that window through the per-byte handshake; a target
	 * that asserts REQ immediately on the Data -> Status transition never
	 * does, and the loop spins forever -- the initiator hangs while the
	 * target is behaving perfectly, which is why a bus watchdog cannot see
	 * it. Hide each newly-asserted REQ from CSR until one full CSR read has
	 * completed (that read returns 0 and disarms; the next shows 1).
	 * BSR.DRQ and dreq are deliberately NOT deferred, so DRQ-polled transfer
	 * loops and DACK pacing are unaffected.
	 */
	reg req_deferred;
	reg [9:0] defer_age;   // bounds how long a REQ can stay hidden
	reg old_req_bus_d;
	reg old_csr_rd_d;
	always @(posedge clk or posedge reset) begin
		if (reset) begin
			req_deferred  <= 1'b0;
			old_req_bus_d <= 1'b0;
			old_csr_rd_d  <= 1'b0;
		end else begin
			old_req_bus_d <= scsi_req;
			old_csr_rd_d  <= csr_rd;
			if (~old_req_bus_d & scsi_req)
				req_deferred <= 1'b1;       // new REQ: hidden until a CSR read
			else if (req_deferred & old_csr_rd_d & ~csr_rd)
				req_deferred <= 1'b0;       // CSR read completed: reveal REQ
			// Self-limiting: csr_rd depends on the host asserting a particular
			// byte lane, but rdata does not, so a poll through the other lane
			// reads the right value while never clearing the deferral -- REQ
			// would stay hidden for good and a CSR-polling driver would spin
			// forever. Age it out so that cannot happen.
			if (!req_deferred) defer_age <= 10'd0;
			else if (~&defer_age) defer_age <= defer_age + 10'd1;
			if (&defer_age) req_deferred <= 1'b0;
			if (!scsi_req)
				req_deferred <= 1'b0;
		end
	end

	/* Completion-IRQ latch. `dma_armed` deliberately survives a DMA-mode
	 * clear: the driver often clears MR.DMA_MODE just before the phase
	 * change, so gating the latch on MR.DMA_MODE drops the IRQ and an async
	 * driver sleeps on a completion that never arrives. Latched on the
	 * target's DATA -> STATUS phase mismatch, cleared by a reg-7 read or a
	 * bus reset. Polled through BSR bit 4 -- the Plus routes no 5380 IRQ to
	 * the VIA, so no interrupt line is synthesised here.
	 */
	reg irq_latch;
	reg dma_armed;
	reg pmatch_d;
	reg old_rst_rd;
	always @(posedge clk or posedge reset) begin
		if (reset) begin
			irq_latch  <= 1'b0;
			dma_armed  <= 1'b0;
			pmatch_d   <= 1'b1;
			old_rst_rd <= 1'b0;
		end else begin
			old_rst_rd <= rst_rd;
			pmatch_d   <= bsr_pmatch;
			if (reg_wr && (bus_rs == `WREG_DMAS || bus_rs == `WREG_IDMAR))
				dma_armed <= 1'b1;
			if (~old_rst_rd & rst_rd)
				irq_latch <= 1'b0;
			// Edge-triggered, as before the 2026-08-22 attempts. The level
			// forms tried there (!bsr_pmatch, then cd&io) both latched a
			// completion IRQ on transactions that had not reached their data
			// phase; BSR=0x98 mid-write on hardware. Restore and re-measure.
			if (dma_armed && pmatch_d && !bsr_pmatch) begin
				irq_latch <= 1'b1;
				dma_armed <= 1'b0;
			end
			if (scsi_rst) begin
				irq_latch <= 1'b0;
				dma_armed <= 1'b0;
			end
		end
	end

	/* BUSY ERROR latch -- see bsr_berr above. Deliberately its OWN block with
	 * its own reg-7 edge detector, so that nothing in the irq_latch block is
	 * disturbed. */
	reg berr_bsy_d;
	reg berr_rst_rd_d;
	always @(posedge clk or posedge reset) begin
		if (reset) begin
			berr_latch    <= 1'b0;
			berr_bsy_d    <= 1'b0;
			berr_rst_rd_d <= 1'b0;
		end else begin
			berr_bsy_d    <= scsi_bsy;
			berr_rst_rd_d <= rst_rd;
			if (~berr_rst_rd_d & rst_rd) berr_latch <= 1'b0;
			// Unexpected loss of BSY with DMA armed. Edge-triggered on BSY so a
			// bus that is simply idle between commands cannot re-arm it.
			if (dma_en && berr_bsy_d && !scsi_bsy) berr_latch <= 1'b1;
			if (scsi_rst) berr_latch <= 1'b0;
		end
	end

	wire [DEVS-1:0] target_holdoff;

	// input signals from targets
	wire [DEVS-1:0] target_bsy;
	wire [DEVS-1:0] target_msg;
	wire [DEVS-1:0] target_io;
	wire [DEVS-1:0] target_cd;
	wire [DEVS-1:0] target_req;
	wire      [7:0] target_dout[DEVS];
	wire signed [15:0] target_snd_l[DEVS];
	wire signed [15:0] target_snd_r[DEVS];

	generate
		genvar i;
		for (i = 0; i < DEVS; i = i + 1) begin : target
			// connect a target
			// SCSI IDs: disks descend from 6 (6, 5, ...); the CD sits at 3.
			// The Plus ROM scans IDs 6 -> 0, so a bootable hard disk is always
			// found before the CD, while a bootable CD still works when no disk
			// is bootable. ID 3 is also the AppleCD SC factory default.
			scsi #(.ID((i == CD_DEV) ? 3'd3 : (3'd6 - i[2:0])),
			       .CDROM((i == CD_DEV) ? 1 : 0), .WDOG_LOG(WDOG_LOG), .IOWDOG_LOG(IOWDOG_LOG),
			       .SPINUP_LOG(SPINUP_LOG)) target
			(
				.clk    ( clk ),
				.rst    ( scsi_rst ),
				.sys_rst( reset ),
				// SCSI arbitration: a target must not answer selection while
				// another one holds the bus. Own BSY is included and is
				// harmless -- this target is in IDLE (bsy=0) whenever the test
				// is evaluated.
				.bus_busy( |target_bsy ),
				.cd_enable( (i == CD_DEV) ? cd_enable : 1'b0 ),
				.sel    ( scsi_sel ),
				.atn    ( scsi_atn ),

				.ack    ( scsi_ack ),

				.bsy    ( target_bsy[i]  ),
				.msg    ( target_msg[i]  ),
				.cd     ( target_cd[i]   ),
				.io     ( target_io[i]   ),
				.req    ( target_req[i]  ),
				.dout   ( target_dout[i] ),

				.din    ( dout ),

				// connection to io controller to read and write sectors
				// to sd card
				.img_mounted(img_mounted[i]),
				.img_blocks(img_size),
				.io_lba ( io_lba[i] ),
				.io_rd  ( io_rd[i] ),
				.io_wr  ( io_wr[i] ),
				// FRAMING: sd_ack names the slot; sd_buff_addr/dout/wr are a
				// SHARED bus every target sees. Qualifying on SCSI BSY instead
				// is a proxy that holds only while every HPS transfer belongs
				// to whichever target owns the bus -- true until the CD-audio
				// engine, which fetches while bus-IDLE. Consequences, both real
				// and both fixed here (confirmed on hardware 2026-08-26):
				//
				//   * `& target_bsy[i]` on sd_buff_wr let a BUSY DISK capture
				//     the CD slot's frames into its sector buffers, and the
				//     next flush wrote CD-DA audio onto the disk image at a
				//     legitimate LBA. It destroyed two mounted volumes.
				//   * `& target_bsy[i]` on the CD's io_ack meant the engine's
				//     fetches only completed while the CD happened to be BSY --
				//     about twice a second, from the player's status polls.
				//     That was the "2 frames/s", not ca_grant.
				//
				// io_ack[i] IS sd_ack[i], which is what hps_io publishes for
				// exactly this purpose.
				//
				// The BSY term SURVIVES on io_ack for the disks, and that is
				// deliberate: it blanks a LATE ack arriving after the target
				// left the bus. The CD target must
				// not have it -- its whole point is transferring while idle.
				// This diverges from MacLC, whose disk targets still carry
				// `& target_bsy[i]` on sd_buff_wr and so, on our reading, carry
				// the same corruption hazard.
				.io_ack ( (i == CD_DEV) ? io_ack[i] : (io_ack[i] & target_bsy[i]) ),

				.sd_buff_addr( sd_buff_addr ),
				.sd_buff_addr_hi( sd_buff_addr_hi ),
				.sd_buff_dout( sd_buff_dout ),
				.sd_buff_din( sd_buff_din[i] ),
				.sd_buff_wr( sd_buff_wr & io_ack[i] ),

				.data_holdoff( target_holdoff[i] ),
				.cd_snd_l( target_snd_l[i] ),
				.cd_snd_r( target_snd_r[i] )
			);
		end
	endgenerate

	// Only the CD target has an audio engine; every other instance ties its
	// pair to zero, so this picks the one that can be non-zero.
	assign cd_snd_l = target_snd_l[CD_DEV];
	assign cd_snd_r = target_snd_r[CD_DEV];

endmodule
