// dc_blocker_slow.v -- local copy of sys/iir_filter.v's DC_blocker, with the
// pole opened out from K=9 to a parameter.
//
// WHY A COPY. K is hardcoded 9/10 in sys/iir_filter.v and that file is upstream
// MiSTer framework -- not ours to fork. We need K=12 here:
// MiSTer's own blocker already sits downstream of our sum, so matching its
// K would put the Mac channel through TWO identical high-passes while the CD
// channel passes one. The risk is not frequency response (the combined corner
// only moves to ~23 Hz) but the bug #7 toggle-sound titles: square-wave content
// that already droops ~37% across a 5 ms half-cycle through one blocker, and
// would tilt further with edge undershoot through two.
//
//   K=9  (MiSTer's) : 14.9 Hz, tau 11 ms,  -1.7 dB at 20 Hz
//   K=12 (ours)     : 1.87 Hz, tau 85 ms,  -0.04 dB at 20 Hz
//
// At K=12 the cascade is arithmetically negligible above 20 Hz. The whole cost
// is a few hundred ms for the pedestal to clear, which is invisible because it
// was silent to begin with. This stage's only job is reclaiming headroom before
// the adder -- DC itself is inaudible, and nothing is listening to how fast it
// settles.
//
// COPIED rather than written fresh on purpose: the {din, 23'd0} fixed point is
// what dodges the classic DC-blocker foot-gun. `>>>` on a negative number
// rounds toward -inf, so a naive implementation settles about 1 LSB BELOW zero
// and sits there forever. Working 23 bits below an output LSB puts that
// truncation far under the noise floor. Do not "simplify" it.
module dc_blocker_slow #(parameter K = 12)
(
	input                     clk,
	input                     ce,     // sample rate strobe (48 kHz here)
	input  signed      [15:0] din,
	output signed      [15:0] dout,
	// The DC estimate currently being removed, at output scale: din - dout.
	// 3C's envelope needs it, because ramping the CORRECTION in is what avoids
	// a click; switching the filter output in would itself be a step of up to
	// 28,448. Exported rather than recomputed so both use the same quantity.
	output signed      [15:0] dc_est
);
	wire signed [39:0] x  = {din[15], din, 23'd0};
	wire signed [39:0] x0 = x - {{(K+1){x[39]}}, x[39:(K+1)]};
	wire signed [39:0] y1 = y - {{K{y[39]}},     y[39:K]};
	wire signed [39:0] y0 = x0 - x1 + y1;

	reg  signed [39:0] x1 = 0, y = 0;
	always @(posedge clk) if(ce) begin
		x1 <= x0;
		// Saturate rather than wrap: an overflow here would be an audible
		// full-scale flip, and this is upstream of the mixer's own clamp.
		y  <= ^y0[39:38] ? {{2{y0[39]}},{38{y0[38]}}} : y0;
	end

	assign dout   = y[38:23];
	assign dc_est = din - y[38:23];
endmodule
