// cd_mix.v -- reclaim the Mac channel's headroom, then sum the CD-DA pair
// into it.
//
// THE PROBLEM. dataController_top.sv:171 emits +127 (8-bit signed) while sound
// is DISABLED -- deliberately, because the real hardware's PWM output idled at
// TTL '1' and an entire class of software makes its waveform by toggling the
// enable pin (PR #12 / bug #7). After volume and the x32 shift that is a
// +28,448 pedestal, 87% of positive full scale. Nobody hears it today because
// MiSTer's own DC_blocker strips it -- but that blocker sits DOWNSTREAM of
// where we now add a second source, so by the time it runs, full-scale CD audio
// has already been summed onto the pedestal and every positive half-cycle has
// been clipped off. The information is gone. The blocker has to be in here.
//
// THE ENVELOPE. Switching the correction in is itself a step of up to 28,448 --
// a click. So ramp it. The clean form needs one multiplier, because the two
// candidate outputs differ by exactly the quantity being removed:
//
//     out = mac - (g * dc_est),  g ramping 0 -> 1
//
// g=0 is raw, g=1 is fully blocked. Release matters as much as attack:
// unmounting restores the pedestal and that is just as much a click, so g
// CHASES a target and one piece of logic covers both directions.
//
// THE GATE. `cd_mounted`, deliberately not "playing" -- playing flips many
// times per disc (track gaps, pause, end) and each flip would be a transition
// to manage mid-listen, and it would need lookahead to engage before audio
// starts. Never gate on `cd_snd_* == 0`: that toggles at audio rate and would
// splice discontinuities into the waveform.
//
// NOTE cd_audio.sv's `disc_audio` looks like the obvious gate here. It is the
// wrong signal: it is defined `toc_valid && !t2_has_data`, i.e. AUDIO-ONLY
// disc, so it reads 0 for a mixed-mode disc -- a data track plus CD audio,
// which is precisely what a game with CD audio is. The gate would then never
// engage for the main use case. Gate on a disc being mounted at all.
module cd_mix (
	input                    clk,
	input                    ce,        // 48 kHz sample strobe
	input                    cd_mounted,
	input      signed [15:0] mac_in,    // {audioOut, 5'b0}, signed
	input      signed [15:0] cd_l_in,
	input      signed [15:0] cd_r_in,
	input              [1:0] cd_vol,    // 0 full, 1 three-quarter, 2 half, 3 off
	output     signed [15:0] out_l,
	output     signed [15:0] out_r
);
	// ---- the DC estimate ------------------------------------------------
	// Runs CONTINUOUSLY, mounted or not, so its state is always settled and
	// there is no filter warm-up at the moment a disc appears. Only its
	// APPLICATION is gated.
	wire signed [15:0] dc_est;
	wire signed [15:0] mac_hp_unused;
	dc_blocker_slow #(.K(12)) blk (
		.clk(clk), .ce(ce), .din(mac_in),
		.dout(mac_hp_unused), .dc_est(dc_est)
	);

	// ---- the envelope ---------------------------------------------------
	// 8 per 48 kHz sample reaches full scale in 8192 samples = 171 ms:
	// comfortably below the ~20 Hz at which a ramp stops being audible, and
	// still instant to someone who just picked a disc in the menu.
	localparam [16:0] G_FULL = 17'h10000;
	localparam [16:0] G_STEP = 17'd8;
	reg [16:0] g = 17'd0;
	always @(posedge clk) if (ce) begin
		if (cd_mounted) g <= (g < (G_FULL - G_STEP)) ? (g + G_STEP) : G_FULL;
		else            g <= (g > G_STEP)            ? (g - G_STEP) : 17'd0;
	end

	wire signed [17:0] g_s      = $signed({1'b0, g});
	wire signed [33:0] corr_mul = dc_est * g_s;
	wire signed [33:0] corr_sh  = corr_mul >>> 16;
	wire signed [17:0] corr     = corr_sh[17:0];

	// mac_ch: the Mac channel with as much of its pedestal removed as g says.
	wire signed [17:0] mac_ch = {{2{mac_in[15]}}, mac_in} - corr;

	// ---- CD volume ------------------------------------------------------
	// Independent of the Mac's own volume control, because on real hardware the
	// Mac's setting had no effect on the drive -- the drive had its own knob.
	// Multiplier-free steps. Index 0 MUST be Full: `status` defaults to zero
	// and unity is the wanted default.
	function signed [15:0] volscale(input signed [15:0] v, input [1:0] sel);
		case (sel)
			2'd0: volscale = v;
			2'd1: volscale = v - (v >>> 2);   // three quarters
			2'd2: volscale = v >>> 1;         // half
			default: volscale = 16'sd0;       // off
		endcase
	endfunction

	wire signed [15:0] cdl = volscale(cd_l_in, cd_vol);
	wire signed [15:0] cdr = volscale(cd_r_in, cd_vol);

	// ---- the sum --------------------------------------------------------
	// Full gain, per MacLC's own comment: they tried half-gain first and drew a
	// "CD sounds half as loud" report. cd_snd_* are EXACT zeros when not
	// playing, so with the mount gate the no-disc path is bit-unchanged.
	// mac_ch is mono and goes to both sides; cd is true stereo and must stay
	// so -- a mono error is a volume error, a stereo error is an image error.
	wire signed [17:0] mix_l = mac_ch + {{2{cdl[15]}}, cdl};
	wire signed [17:0] mix_r = mac_ch + {{2{cdr[15]}}, cdr};

	// NOTE `16'sh8000`, not `-16'sd32768`. A signed 16-bit literal tops out at
	// +32767, so 16'sd32768 overflows and the unary minus then negates an
	// already-wrapped value -- Quartus flags it (warning 10259). 16'sh8000 IS
	// -32768 in two's complement, with nothing to overflow. The 18-bit
	// comparison operand is fine as written: 18'sd32768 has room.
	function signed [15:0] sat(input signed [17:0] v);
		sat = (v >  18'sd32767) ? 16'sh7fff :
		      (v < -18'sd32768) ? 16'sh8000 : v[15:0];
	endfunction

	assign out_l = sat(mix_l);
	assign out_r = sat(mix_r);
endmodule
