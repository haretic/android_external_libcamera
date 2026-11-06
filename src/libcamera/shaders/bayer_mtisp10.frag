/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2026, itskenny0
 *
 * bayer_mtisp10.frag - Fragment shader code for raw Bayer 10-bit in the
 * MediaTek ISP packed layout
 *
 * The samples of a line are a little-endian bit stream, four samples in five
 * bytes (see mtisp.h), so unlike MIPI CSI-2 packing no byte holds the high
 * bits of a sample on its own: every sample is put together from the two
 * bytes it straddles. The demosaic and the colour processing are those of
 * bayer_1x_packed.frag.
 */

#ifdef GL_ES
precision highp float;
#endif

varying vec2 textureOut;

uniform vec2 tex_size;
uniform vec2 tex_step;
uniform vec2 tex_bayer_first_red;

uniform sampler2D tex_y;
uniform vec3 awb;
uniform mat3 ccm;
uniform vec3 blacklevel;
/*
 * Added to the raw samples at the four positions of the Bayer pattern, top
 * left, top right, bottom left, bottom right of the frame: puts positions
 * whose black level differs from their channel's on the channel's level, so
 * columns or rows that read out with different offsets don't stripe. Zero
 * leaves the samples alone.
 */
uniform vec4 black_shift;
/*
 * The tone curve, the contrast S-curve and the gamma, 1024 steps over 0 to
 * TONE_RANGE indexed by the square root, for finer steps in the shadows
 * (DebayerEGL::updateLut() has the same range).
 */
uniform sampler2D tex_lut;
#define TONE_RANGE 16.0

/*
 * Noise variance of a sample in units of white squared at the frame's gain:
 * x times the signal (shot) plus y (read). A sample within NOISE_FLAT
 * standard deviations of its same-colour neighbours' mean is taken to be
 * noise and replaced by it, one more than NOISE_EDGE away is detail and
 * kept, in between it fades. Zero leaves the samples alone.
 */
uniform vec2 noise;
#define NOISE_FLAT 1.0
#define NOISE_EDGE 3.0

/*
 * Defective samples, hot or dead. A sample standing above all its
 * same-colour peers, or below all, by more than DEFECT_SIGMA standard
 * deviations of the noise expected at their level (at least DEFECT_FLOOR)
 * plus DEFECT_SPREAD times how far the peers differ, which texture makes
 * them, is a defect, unless a sample of another colour around it rises or
 * falls with it past DEFECT_NOISE deviations, by at least a
 * DEFECT_ISOLATION-th as much: the lens spreads real detail over the
 * neighbours, and a line however thin runs on through them, while a defect
 * stays in its sample. Compared white balanced, so grey detail rises alike
 * in every colour. A defect takes the mean of its peers. The scaled outputs
 * look harder, DEFECT_SIGMA_SCALED: they average away the detail they could
 * take, and a defect left in a small preview stands still as the scene
 * moves.
 */
#define DEFECT_SIGMA 5.0
#define DEFECT_SIGMA_SCALED 4.0
#define DEFECT_FLOOR (8.0 / 1023.0)
#define DEFECT_SPREAD 0.5
#define DEFECT_NOISE 3.0
#define DEFECT_ISOLATION 8.0

/* The noise standard deviation of raw samples at level s, in raw units */
vec3 noise_sigma(vec3 s, float black)
{
	vec3 signal = max(s - black, 0.0) / (1.0 - black);
	return sqrt(noise.x * signal + noise.y) * (1.0 - black);
}

vec4 noise_sigma(vec4 s, vec4 black)
{
	vec4 signal = max(s - black, 0.0) / (1.0 - black);
	return sqrt(noise.x * signal + noise.y) * (1.0 - black);
}

/* For each component, the largest of the other three */
vec4 others_max(vec4 v)
{
	return max(max(v.yxwz, v.zwxy), v.wzyx);
}

/*
 * The texture holds four bytes of a line in each texel, in r, g, b and a,
 * the stride a quarter as wide; tex_step.x is 4 / (stride - 1).
 */
float texels()
{
	return (4.0 / tex_step.x + 1.0) * 0.25;
}

/*
 * Lines beyond the top and bottom edges, mirrored in them: the lines of the
 * same colours as the ones they stand for.
 */
float mirror_y(float y)
{
	return tex_size.y - 1.0 - abs(tex_size.y - 1.0 - abs(y));
}

/* The black level shift of the sample at column x of line y of the frame */
float black_shift_at(float x, float y)
{
	vec2 odd = mod(vec2(x, y), 2.0);
	return odd.y < 1.0 ? (odd.x < 1.0 ? black_shift.x : black_shift.y)
			   : (odd.x < 1.0 ? black_shift.z : black_shift.w);
}

/* Byte x of line y, 0 to 255 */
float byte_at(float x, float y, float stride)
{
	float t = floor(x * 0.25);
	vec4 v = texture2D(tex_y, vec2((t + 0.5) / stride, (y + 0.5) / tex_size.y));
	return floor(dot(v, vec4(equal(vec4(x - 4.0 * t), vec4(0.0, 1.0, 2.0, 3.0)))) *
		     255.0 + 0.5);
}

/*
 * Sample k of a group of four (k = 0..3) starts at bit 2k of byte k of the
 * group's five and takes 10 bits: the byte's top 8 - 2k bits, then the low
 * 2k + 2 bits of the next byte. Without integer operations in GLSL ES 1.0
 * the shifts are divisions and the masks mod() by powers of two.
 */
float fetch(float x, float y, float stride)
{
	x = tex_size.x - 1.0 - abs(tex_size.x - 1.0 - abs(x));
	y = mirror_y(y);
	float group = floor(x * 0.25 + 0.01);
	float k = x - 4.0 * group;
	float byte_x = 5.0 * group + k;
	float lo = byte_at(byte_x, y, stride);
	float hi = byte_at(byte_x + 1.0, y, stride);
	float shift = exp2(2.0 * k);
	float value = floor(lo / shift + 0.001) + mod(hi, 4.0 * shift) * (256.0 / shift);
	return value / 1023.0 + black_shift_at(x, y);
}

/*
 * The colour of the pixel at \a center from its neighbours, as
 * bayer_1x_packed.frag names them, C in the middle:
 *
 *   | D2 | A1 | D3 |
 *   | B0 |  C | B1 |
 *   | D0 | A0 | D1 |
 *
 * \a top holds D0 A0 D1, \a mid B0 C B1 and \a bot D2 A1 D3.
 */
vec3 demosaic(vec2 center, vec3 top, vec3 mid, vec3 bot)
{
	vec2 alternate = mod(center + tex_bayer_first_red, 2.0);
	bool even_col = alternate.x < 1.0;
	bool even_row = alternate.y < 1.0;

	float C = mid.y;
	vec4 patterns = (vec4(top.y, mid.x, top.x, top.z) +
			 vec4(bot.y, mid.z, bot.z, bot.x)) * 0.5;
		/* .x = (A0 + A1) / 2.0, .y = (B0 + B1) / 2.0 */
		/* .z = (D0 + D3) / 2.0, .w = (D1 + D2) / 2.0 */
	patterns.w = (patterns.z + patterns.w) * 0.5;
	patterns.z = (patterns.x + patterns.y) * 0.5;

	return even_col ?
		(even_row ?
			vec3(C, patterns.zw) :
			vec3(patterns.x, C, patterns.y)) :
		(even_row ?
			vec3(patterns.y, C, patterns.x) :
			vec3(patterns.wz, C));
}

vec3 process(vec3 rgb)
{
	/*
	 * Saturate at the sensor's range, then apply the gains: what they
	 * push past white keeps its detail for the tone curve's shoulder.
	 * Negative values from noise around the black level go on through
	 * the colour correction rather than being cut, which would make the
	 * shadows the colour of the channels with the highest gains.
	 */
	rgb = min((rgb - blacklevel) / (1.0 - blacklevel), 1.0) * awb;

	/* Colour Correction Matrix */
	rgb = ccm * rgb;

	rgb = sqrt(clamp(rgb / TONE_RANGE, 0.0, 1.0)) * (1023.0 / 1024.0) + 0.5 / 1024.0;
	return vec3(texture2D(tex_lut, vec2(rgb.r, 0.5)).r,
		    texture2D(tex_lut, vec2(rgb.g, 0.5)).r,
		    texture2D(tex_lut, vec2(rgb.b, 0.5)).r);
}

#if defined (OUTPUT_Y) || defined (OUTPUT_UV)
/*
 * The planes of NV12 and NV21 are rendered as RGBA images a quarter as wide
 * as the image, four bytes of the plane in a fragment, so a fragment is four pixels
 * of a line: a group of the packing, out_origin being where the output starts
 * in the image, at a group. The six samples from the one before the group to
 * the one after take the nine bytes from the last two of the group before,
 * all in the three texels from the one holding the first.
 */
uniform vec2 out_origin;

/*
 * Each sample of a same-colour triplet shrunk towards the mean of the other
 * two by how far it stands from it against the noise expected at that
 * level, in raw units: the expected deviation grows with the gain, so the
 * filter strengthens as the scene darkens, the way a per-ISO table does.
 */
vec3 denoise_triplet(vec3 s, float black)
{
	if (noise.x + noise.y <= 0.0)
		return s;
	vec3 mean = (s.yzx + s.zxy) * 0.5;
	vec3 signal = max(mean - black, 0.0) / (1.0 - black);
	vec3 sigma = sqrt(noise.x * signal + noise.y) * (1.0 - black);
	vec3 d = s - mean;
	vec3 keep = smoothstep(NOISE_FLAT * sigma, NOISE_EDGE * sigma, abs(d));
	return mean + d * keep;
}

/*
 * Line y around group g, raw with the black level shifts: the samples of
 * the even columns 4g, 4g + 2 and 4g + 4 and of the odd ones 4g - 1, 4g + 1
 * and 4g + 3, one colour each, and the two colours' black levels and white
 * balance gains over the range above black.
 */
void decode_line(float g, float y, float stride, out vec3 even, out vec3 odd,
		 out vec2 black, out vec2 wb)
{
	vec4 s;
	vec2 edge;
	y = mirror_y(y);
	float v = (y + 0.5) / tex_size.y;
	float first = 5.0 * g - 2.0;
	float t = floor(first * 0.25);
	float o = first - 4.0 * t;
	vec4 ta = texture2D(tex_y, vec2((t + 0.5) / stride, v));
	vec4 tb = texture2D(tex_y, vec2((t + 1.5) / stride, v));
	vec4 tc = texture2D(tex_y, vec2((t + 2.5) / stride, v));

	/*
	 * The twelve bytes shifted down by o, a weighted sum of the four
	 * shifts: cheaper than picking each byte on its own.
	 */
	vec4 w = vec4(equal(vec4(o), vec4(0.0, 1.0, 2.0, 3.0)));
	vec4 lo = ta * w.x + vec4(ta.yzw, tb.x) * w.y +
		  vec4(ta.zw, tb.xy) * w.z + vec4(ta.w, tb.xyz) * w.w;
	vec4 mi = tb * w.x + vec4(tb.yzw, tc.x) * w.y +
		  vec4(tb.zw, tc.xy) * w.z + vec4(tb.w, tc.xyz) * w.w;
	float hi = dot(tc, w);
	lo = floor(lo * 255.0 + 0.5);
	mi = floor(mi * 255.0 + 0.5);
	hi = floor(hi * 255.0 + 0.5);

	float p3 = lo.x;
	float p4 = lo.y;
	float b0 = lo.z;
	float b1 = lo.w;
	float b2 = mi.x;
	float b3 = mi.y;
	float b4 = mi.z;
	float n0 = mi.w;
	float n1 = hi;

	s = vec4(b0 + mod(b1, 4.0) * 256.0,
		 floor(b1 / 4.0 + 0.001) + mod(b2, 16.0) * 64.0,
		 floor(b2 / 16.0 + 0.001) + mod(b3, 64.0) * 16.0,
		 floor(b3 / 64.0 + 0.001) + b4 * 4.0) / 1023.0;
	edge = vec2(floor(p3 / 64.0 + 0.001) + p4 * 4.0,
		    n0 + mod(n1, 4.0) * 256.0) / 1023.0;

	/* Columns 4g, 4g + 2 and 4g + 4 are even, 4g - 1, 4g + 1 and 4g + 3 odd */
	float shift_even = black_shift_at(0.0, y);
	float shift_odd = black_shift_at(1.0, y);
	s += vec4(shift_even, shift_odd, shift_even, shift_odd);
	edge += vec2(shift_odd, shift_even);

	/* Beyond the image, the samples of the same colour mirrored in its edge */
	if (g < 0.5)
		edge.x = s.y;
	if (4.0 * g + 4.0 > tex_size.x - 0.5)
		edge.y = s.z;

	even = vec3(s.xz, edge.y);
	odd = vec3(edge.x, s.yw);

	/* Even/odd columns' channel gains on this Bayer row */
	bool red_row = mod(y + tex_bayer_first_red.y, 2.0) < 1.0;
	vec2 gain = red_row ? awb.rg : awb.gb;
	black = red_row ? blacklevel.rg : blacklevel.gb;
	if (tex_bayer_first_red.x >= 0.5) {
		gain = gain.yx;
		black = black.yx;
	}
	wb = gain / (1.0 - black);
}

/*
 * A same-colour triplet's peers' means, raw, and white balanced how far
 * each sample stands past both its peers beyond the limit, above (hot) and
 * below (dead), and how far it rises and falls from their mean beyond the
 * noise.
 */
void triplet(vec3 s, float black, float wb, out vec3 mean, out vec3 hot,
	     out vec3 dead, out vec3 rise, out vec3 fall)
{
	vec3 a = s.yzx;
	vec3 b = s.zxy;
	mean = (a + b) * 0.5;
	vec3 sigma = noise_sigma(mean, black);
	vec3 limit = max(DEFECT_SIGMA * sigma, DEFECT_FLOOR) + DEFECT_SPREAD * abs(a - b);
	vec3 above = s - max(a, b);
	vec3 below = min(a, b) - s;
	hot = above * vec3(greaterThan(above, limit)) * wb;
	dead = below * vec3(greaterThan(below, limit)) * wb;
	rise = max(s - mean - DEFECT_NOISE * sigma, 0.0) * wb;
	fall = max(mean - s - DEFECT_NOISE * sigma, 0.0) * wb;
}

/*
 * A line's two triplets with their defect figures: beside_* are the rises
 * and falls of the samples of the other colour beside each sample on the
 * line, around_* those of the column and the two beside it, which the lines
 * above and below take as theirs.
 */
struct Line {
	vec3 even, odd;
	vec3 mean_even, mean_odd;
	vec3 hot_even, hot_odd, dead_even, dead_odd;
	vec3 beside_rise_even, beside_rise_odd, beside_fall_even, beside_fall_odd;
	vec3 around_rise_even, around_rise_odd, around_fall_even, around_fall_odd;
	vec2 black;
};

Line line_at(float g, float y, float stride)
{
	Line l;
	vec2 wb;
	vec3 rise_even, rise_odd, fall_even, fall_odd;
	decode_line(g, y, stride, l.even, l.odd, l.black, wb);
	triplet(l.even, l.black.x, wb.x, l.mean_even, l.hot_even, l.dead_even,
		rise_even, fall_even);
	triplet(l.odd, l.black.y, wb.y, l.mean_odd, l.hot_odd, l.dead_odd,
		rise_odd, fall_odd);

	/* Beside even column c are odd c - 1 and c + 1, beside odd c even c - 1 and c + 1 */
	l.beside_rise_even = max(rise_odd, rise_odd.yzz);
	l.beside_fall_even = max(fall_odd, fall_odd.yzz);
	l.beside_rise_odd = max(rise_even.xxy, rise_even);
	l.beside_fall_odd = max(fall_even.xxy, fall_even);
	l.around_rise_even = max(rise_even, l.beside_rise_even);
	l.around_fall_even = max(fall_even, l.beside_fall_even);
	l.around_rise_odd = max(rise_odd, l.beside_rise_odd);
	l.around_fall_odd = max(fall_odd, l.beside_fall_odd);
	return l;
}

/*
 * Line l with its defects replaced, the samples of the lines above and
 * below it vouching for detail, then denoised: the samples of columns
 * 4g .. 4g + 3 in s, of 4g - 1 and 4g + 4 in edge. At the window's first
 * and last line, the one line next to it stands for both.
 */
void fix_line(Line l, Line up, Line down, out vec4 s, out vec2 edge)
{
	vec3 rise = max(l.beside_rise_even, max(up.around_rise_even, down.around_rise_even));
	vec3 fall = max(l.beside_fall_even, max(up.around_fall_even, down.around_fall_even));
	vec3 bad = max(vec3(greaterThan(l.hot_even, DEFECT_ISOLATION * rise)),
		       vec3(greaterThan(l.dead_even, DEFECT_ISOLATION * fall)));
	vec3 even = denoise_triplet(mix(l.even, l.mean_even, bad), l.black.x);

	rise = max(l.beside_rise_odd, max(up.around_rise_odd, down.around_rise_odd));
	fall = max(l.beside_fall_odd, max(up.around_fall_odd, down.around_fall_odd));
	bad = max(vec3(greaterThan(l.hot_odd, DEFECT_ISOLATION * rise)),
		  vec3(greaterThan(l.dead_odd, DEFECT_ISOLATION * fall)));
	vec3 odd = denoise_triplet(mix(l.odd, l.mean_odd, bad), l.black.y);

	s = vec4(even.x, odd.y, even.y, odd.z);
	edge = vec2(odd.x, even.z);
}

/* The colours of the pixels of group \a g of line \a y, linear */
void group_colours(float g, float y, float stride,
		   out vec3 c0, out vec3 c1, out vec3 c2, out vec3 c3)
{
	Line lt = line_at(g, y - 1.0, stride);
	Line lm = line_at(g, y, stride);
	Line lb = line_at(g, y + 1.0, stride);
	vec4 t, m, b;
	vec2 te, me, be;
	fix_line(lt, lm, lm, t, te);
	fix_line(lm, lt, lb, m, me);
	fix_line(lb, lm, lm, b, be);

	float x = 4.0 * g;
	c0 = demosaic(vec2(x, y), vec3(te.x, t.xy), vec3(me.x, m.xy), vec3(be.x, b.xy));
	c1 = demosaic(vec2(x + 1.0, y), t.xyz, m.xyz, b.xyz);
	c2 = demosaic(vec2(x + 2.0, y), t.yzw, m.yzw, b.yzw);
	c3 = demosaic(vec2(x + 3.0, y), vec3(t.zw, te.y), vec3(m.zw, me.y), vec3(b.zw, be.y));
}

/* Full range BT.601, as the CPU debayer has it */
float luma(vec3 rgb)
{
	return dot(rgb, vec3(0.299, 0.587, 0.114));
}

/*
 * The colour of the darkest shadows is the noise of the channels with the
 * highest gains after colour correction, purple under warm light: it fades
 * out below a luma of 0.15, gone at 0.03.
 */
vec2 chroma(vec3 rgb)
{
	return vec2(dot(rgb, vec3(-0.168736, -0.331264, 0.5)),
		    dot(rgb, vec3(0.5, -0.418688, -0.081312))) *
	       smoothstep(0.03, 0.15, luma(rgb)) + 0.5;
}
#endif

#if defined (SCALED)
/*
 * Outputs much smaller than the image are scaled from the window at
 * out_origin, out_scale samples to a pixel: each pixel is the mean colour
 * of the QUADS x QUADS Bayer quads around where it falls, which bins the
 * noise of small previews of a large image rather than aliasing it.
 */
uniform vec2 out_scale;

/*
 * Samples x and x + 1 of line y, x even: k = 0 or 2 in their group, the
 * three bytes from byte k of the group's five on, in at most two texels.
 */
vec2 pair_at(float x, float y, float stride)
{
	x = clamp(x, 0.0, tex_size.x - 2.0);
	y = mirror_y(y);
	float g = floor(x * 0.25 + 0.01);
	float k = x - 4.0 * g;
	float first = 5.0 * g + k;
	float t = floor(first * 0.25);
	float o = first - 4.0 * t;
	float v = (y + 0.5) / tex_size.y;
	vec4 ta = texture2D(tex_y, vec2((t + 0.5) / stride, v));
	vec4 tb = texture2D(tex_y, vec2((t + 1.5) / stride, v));

	vec4 w = vec4(equal(vec4(o), vec4(0.0, 1.0, 2.0, 3.0)));
	vec3 b = ta.xyz * w.x + ta.yzw * w.y + vec3(ta.zw, tb.x) * w.z +
		 vec3(ta.w, tb.xy) * w.w;
	b = floor(b * 255.0 + 0.5);

	float s0 = exp2(2.0 * k);
	float s1 = 4.0 * s0;
	return vec2(floor(b.x / s0 + 0.001) + mod(b.y, 4.0 * s0) * (256.0 / s0),
		    floor(b.y / s1 + 0.001) + mod(b.z, 4.0 * s1) * (256.0 / s1)) / 1023.0 +
	       vec2(black_shift_at(0.0, y), black_shift_at(1.0, y));
}

/*
 * The samples of the quad at q, both even, raw: r, gr (beside red), gb
 * (beside blue) and b. Quads beyond the left and right edges are mirrored in
 * them.
 */
vec4 quad_raw(vec2 q, float stride)
{
	q.x = tex_size.x - 2.0 - abs(tex_size.x - 2.0 - abs(q.x));
	q.y = min(q.y, tex_size.y - 2.0);
	vec2 top = pair_at(q.x, q.y, stride);
	vec2 bot = pair_at(q.x, q.y + 1.0, stride);
	vec2 red_line = tex_bayer_first_red.y < 0.5 ? top : bot;
	vec2 blue_line = tex_bayer_first_red.y < 0.5 ? bot : top;
	return tex_bayer_first_red.x < 0.5 ? vec4(red_line, blue_line) :
					     vec4(red_line.yx, blue_line.yx);
}

/*
 * Quad c with its defects replaced, its peers the same samples of the quads
 * l and r beside it, each sample's evidence of detail the other three of its
 * quad, all next to it.
 */
vec4 fix_quad(vec4 l, vec4 c, vec4 r)
{
	vec4 black = blacklevel.rggb;
	vec4 wb = awb.rggb / (1.0 - black);
	vec4 mean = (l + r) * 0.5;
	vec4 sigma = noise_sigma(mean, black);
	vec4 limit = max(DEFECT_SIGMA_SCALED * sigma, DEFECT_FLOOR) + DEFECT_SPREAD * abs(l - r);
	vec4 above = c - max(l, r);
	vec4 below = min(l, r) - c;
	vec4 hot = above * vec4(greaterThan(above, limit)) * wb;
	vec4 dead = below * vec4(greaterThan(below, limit)) * wb;
	vec4 rise = max(c - mean - DEFECT_NOISE * sigma, 0.0) * wb;
	vec4 fall = max(mean - c - DEFECT_NOISE * sigma, 0.0) * wb;
	vec4 bad = max(vec4(greaterThan(hot, DEFECT_ISOLATION * others_max(rise))),
		       vec4(greaterThan(dead, DEFECT_ISOLATION * others_max(fall))));
	return mix(c, mean, bad);
}

/* The colour of a quad's samples, red where first red has it */
vec3 quad_colour(vec4 q)
{
	return vec3(q.x, (q.y + q.z) * 0.5, q.w);
}

/* The colour of the quad at q, both even, its defects replaced */
vec3 quad(vec2 q, float stride)
{
	return quad_colour(fix_quad(quad_raw(q - vec2(2.0, 0.0), stride), quad_raw(q, stride),
				    quad_raw(q + vec2(2.0, 0.0), stride)));
}

#if defined (ZOOM)
/*
 * Zoomed in until a pixel covers less than two quads: the colour where c
 * falls between the four quads around it, blended by how near it is to
 * each, which a zoomed window shows smooth rather than in blocks.
 */
vec3 area(vec2 c, float stride)
{
	vec2 f = (c - 1.0) * 0.5;
	vec2 i = floor(f);
	vec2 t = f - i;
	vec2 q = 2.0 * i;
	vec3 row[2];
	for (int j = 0; j < 2; j++) {
		vec2 r = q + vec2(0.0, 2.0 * float(j));
		vec4 q0 = quad_raw(r - vec2(2.0, 0.0), stride);
		vec4 q1 = quad_raw(r, stride);
		vec4 q2 = quad_raw(r + vec2(2.0, 0.0), stride);
		vec4 q3 = quad_raw(r + vec2(4.0, 0.0), stride);
		row[j] = mix(quad_colour(fix_quad(q0, q1, q2)), quad_colour(fix_quad(q1, q2, q3)), t.x);
	}
	return mix(row[0], row[1], t.y);
}
#elif QUADS == 2
/*
 * The four samples of group g of line y. A group's five bytes always lie in
 * two texels, so two quads take half the fetches of pair_at().
 */
vec4 group_at(float g, float y, float stride)
{
	g = clamp(g, 0.0, floor(tex_size.x * 0.25) - 1.0);
	y = mirror_y(y);
	float first = 5.0 * g;
	float t = floor(first * 0.25);
	float o = first - 4.0 * t;
	float v = (y + 0.5) / tex_size.y;
	vec4 ta = texture2D(tex_y, vec2((t + 0.5) / stride, v));
	vec4 tb = texture2D(tex_y, vec2((t + 1.5) / stride, v));

	vec4 w = vec4(equal(vec4(o), vec4(0.0, 1.0, 2.0, 3.0)));
	vec4 b = ta * w.x + vec4(ta.yzw, tb.x) * w.y + vec4(ta.zw, tb.xy) * w.z +
		 vec4(ta.w, tb.xyz) * w.w;
	float b4 = dot(vec4(tb.xyz, 0.0), w) + tb.w * w.w;
	b = floor(b * 255.0 + 0.5);
	b4 = floor(b4 * 255.0 + 0.5);

	float shift_even = black_shift_at(0.0, y);
	float shift_odd = black_shift_at(1.0, y);
	return vec4(b.x + mod(b.y, 4.0) * 256.0,
		    floor(b.y / 4.0 + 0.001) + mod(b.z, 16.0) * 64.0,
		    floor(b.z / 16.0 + 0.001) + mod(b.w, 64.0) * 16.0,
		    floor(b.w / 64.0 + 0.001) + b4 * 4.0) / 1023.0 +
	       vec4(shift_even, shift_odd, shift_even, shift_odd);
}

/*
 * One colour's samples of a 2 x 2 box of quads, a quad a component: their
 * peers' means, raw, and white balanced how far each stands past all three
 * of its peers (beside, above or below and across it) beyond the limit, and
 * how far it rises and falls from their mean beyond the noise.
 */
void box_colour(vec4 s, float black, float wb, out vec4 mean, out vec4 hot,
		out vec4 dead, out vec4 rise, out vec4 fall)
{
	vec4 a = s.yxwz;
	vec4 b = s.zwxy;
	vec4 c = s.wzyx;
	mean = (a + b + c) * (1.0 / 3.0);
	vec4 sigma = noise_sigma(mean, vec4(black));
	vec4 limit = max(DEFECT_SIGMA_SCALED * sigma, DEFECT_FLOOR) +
		     DEFECT_SPREAD * (max(max(a, b), c) - min(min(a, b), c));
	vec4 above = s - max(max(a, b), c);
	vec4 below = min(min(a, b), c) - s;
	hot = above * vec4(greaterThan(above, limit)) * wb;
	dead = below * vec4(greaterThan(below, limit)) * wb;
	rise = max(s - mean - DEFECT_NOISE * sigma, 0.0) * wb;
	fall = max(mean - s - DEFECT_NOISE * sigma, 0.0) * wb;
}

/*
 * The mean colour of the 2 x 2 quads around c, in samples: the two groups'
 * worth on four lines nearest it, up to two samples off to the side. A
 * sample of a quad is a defect when it stands out from the same samples of
 * the other three and no other sample of its quad goes with it.
 */
vec3 area(vec2 c, float stride)
{
	float g = floor((c.x - 2.0) * 0.25 + 0.5);
	float y = 2.0 * floor((c.y - 2.0) * 0.5 + 0.5);
	vec4 l0 = group_at(g, y, stride);
	vec4 l1 = group_at(g, y + 1.0, stride);
	vec4 l2 = group_at(g, y + 2.0, stride);
	vec4 l3 = group_at(g, y + 3.0, stride);

	/* Each position of a quad, over the box's quads: left, right, lower left, lower right */
	vec4 top_left = vec4(l0.xz, l2.xz);
	vec4 top_right = vec4(l0.yw, l2.yw);
	vec4 bottom_left = vec4(l1.xz, l3.xz);
	vec4 bottom_right = vec4(l1.yw, l3.yw);
	bool red_top = tex_bayer_first_red.y < 0.5;
	bool red_left = tex_bayer_first_red.x < 0.5;
	vec4 red_line_left = red_top ? top_left : bottom_left;
	vec4 red_line_right = red_top ? top_right : bottom_right;
	vec4 blue_line_left = red_top ? bottom_left : top_left;
	vec4 blue_line_right = red_top ? bottom_right : top_right;
	vec4 r = red_left ? red_line_left : red_line_right;
	vec4 gr = red_left ? red_line_right : red_line_left;
	vec4 gb = red_left ? blue_line_left : blue_line_right;
	vec4 b = red_left ? blue_line_right : blue_line_left;

	vec3 wb = awb / (1.0 - blacklevel);
	vec4 mr, mgr, mgb, mb, hr, hgr, hgb, hb, dr, dgr, dgb, db;
	vec4 rr, rgr, rgb, rb, fr, fgr, fgb, fb;
	box_colour(r, blacklevel.r, wb.r, mr, hr, dr, rr, fr);
	box_colour(gr, blacklevel.g, wb.g, mgr, hgr, dgr, rgr, fgr);
	box_colour(gb, blacklevel.g, wb.g, mgb, hgb, dgb, rgb, fgb);
	box_colour(b, blacklevel.b, wb.b, mb, hb, db, rb, fb);

	r = mix(r, mr, max(vec4(greaterThan(hr, DEFECT_ISOLATION * max(max(rgr, rgb), rb))),
			   vec4(greaterThan(dr, DEFECT_ISOLATION * max(max(fgr, fgb), fb)))));
	gr = mix(gr, mgr, max(vec4(greaterThan(hgr, DEFECT_ISOLATION * max(max(rr, rgb), rb))),
			      vec4(greaterThan(dgr, DEFECT_ISOLATION * max(max(fr, fgb), fb)))));
	gb = mix(gb, mgb, max(vec4(greaterThan(hgb, DEFECT_ISOLATION * max(max(rr, rgr), rb))),
			      vec4(greaterThan(dgb, DEFECT_ISOLATION * max(max(fr, fgr), fb)))));
	b = mix(b, mb, max(vec4(greaterThan(hb, DEFECT_ISOLATION * max(max(rr, rgr), rgb))),
			   vec4(greaterThan(db, DEFECT_ISOLATION * max(max(fr, fgr), fgb)))));
	return vec3(dot(r, vec4(0.25)), dot(gr + gb, vec4(0.125)), dot(b, vec4(0.25)));
}
#else
/* The mean colour of the QUADS x QUADS quads around c, in samples */
vec3 area(vec2 c, float stride)
{
	vec2 q0 = 2.0 * floor((c - float(QUADS)) * 0.5 + 0.5);
	vec3 sum = vec3(0.0);
	for (int j = 0; j < QUADS; j++) {
		for (int i = 0; i < QUADS; i++)
			sum += quad(q0 + 2.0 * vec2(float(i), float(j)), stride);
	}
	return sum / float(QUADS * QUADS);
}
#endif
#endif

void main(void)
{
	float stride = texels();
	vec3 c0, c1, c2, c3;

#if defined (OUTPUT_Y) && defined (SCALED)
	vec2 o = floor(gl_FragCoord.xy) * vec2(4.0, 1.0) + 0.5;
	gl_FragColor = vec4(
		luma(process(area(out_origin + o * out_scale, stride))),
		luma(process(area(out_origin + (o + vec2(1.0, 0.0)) * out_scale, stride))),
		luma(process(area(out_origin + (o + vec2(2.0, 0.0)) * out_scale, stride))),
		luma(process(area(out_origin + (o + vec2(3.0, 0.0)) * out_scale, stride))));
#elif defined (OUTPUT_UV) && defined (SCALED)
	/* Two 2x2 blocks of pixels, each about their centre */
	vec2 o = floor(gl_FragCoord.xy) * vec2(4.0, 2.0) + 1.0;
	vec2 a = chroma(process(area(out_origin + o * out_scale, stride)));
	vec2 b = chroma(process(area(out_origin + (o + vec2(2.0, 0.0)) * out_scale, stride)));
#if defined (SWAP_UV)
	gl_FragColor = vec4(a.yx, b.yx);
#else
	gl_FragColor = vec4(a, b);
#endif
#elif defined (OUTPUT_Y)
	vec2 p = floor(gl_FragCoord.xy) * vec2(4.0, 1.0) + out_origin;
	group_colours(p.x * 0.25, p.y, stride, c0, c1, c2, c3);
	gl_FragColor = vec4(luma(process(c0)), luma(process(c1)),
			    luma(process(c2)), luma(process(c3)));
#elif defined (OUTPUT_UV)
	/*
	 * The chroma of two 2x2 blocks, each from the mean colour of the 4x4
	 * samples around it: four red, eight green and four blue, which takes
	 * most of the colour noise out of dark scenes. The blocks' boxes start
	 * a column before them, at x - 1 and x + 1, and a line above.
	 */
	vec2 p = floor(gl_FragCoord.xy) * vec2(4.0, 2.0) + out_origin;
	float g = p.x * 0.25;
	Line l0 = line_at(g, p.y - 1.0, stride);
	Line l1 = line_at(g, p.y, stride);
	Line l2 = line_at(g, p.y + 1.0, stride);
	Line l3 = line_at(g, p.y + 2.0, stride);
	vec4 s0, s1, s2, s3;
	vec2 e0, e1, e2, e3;
	fix_line(l0, l1, l1, s0, e0);
	fix_line(l1, l0, l2, s1, e1);
	fix_line(l2, l1, l3, s2, e2);
	fix_line(l3, l2, l2, s3, e3);

	/* Lines 0 and 2 of a box have one colour pair, 1 and 3 the other */
	vec4 a02 = vec4(e0.x, s0.xyz) + vec4(e2.x, s2.xyz);
	vec4 a13 = vec4(e1.x, s1.xyz) + vec4(e3.x, s3.xyz);
	vec4 b02 = vec4(s0.yzw, e0.y) + vec4(s2.yzw, e2.y);
	vec4 b13 = vec4(s1.yzw, e1.y) + vec4(s3.yzw, e3.y);

	/*
	 * Red sits on even columns of even lines, blue on odd columns of odd
	 * ones, counted from the first red; the same for every fragment.
	 */
	vec2 alternate = mod(p - 1.0 + tex_bayer_first_red, 2.0);
	vec4 even = alternate.x < 1.0 ? vec4(1.0, 0.0, 1.0, 0.0) : vec4(0.0, 1.0, 0.0, 1.0);
	vec4 odd = 1.0 - even;
	if (alternate.y >= 1.0) {
		vec4 t = a02;
		a02 = a13;
		a13 = t;
		t = b02;
		b02 = b13;
		b13 = t;
	}
	vec3 boxA = vec3(dot(a02, even), dot(a02, odd) + dot(a13, even), dot(a13, odd)) *
		    vec3(0.25, 0.125, 0.25);
	vec3 boxB = vec3(dot(b02, even), dot(b02, odd) + dot(b13, even), dot(b13, odd)) *
		    vec3(0.25, 0.125, 0.25);
	vec2 a = chroma(process(boxA));
	vec2 b = chroma(process(boxB));
#if defined (SWAP_UV)
	gl_FragColor = vec4(a.yx, b.yx);
#else
	gl_FragColor = vec4(a, b);
#endif
#else
	vec2 center = floor(textureOut * tex_size);
	vec3 rgb = process(demosaic(center,
		vec3(fetch(center.x - 1.0, center.y - 1.0, stride),
		     fetch(center.x, center.y - 1.0, stride),
		     fetch(center.x + 1.0, center.y - 1.0, stride)),
		vec3(fetch(center.x - 1.0, center.y, stride),
		     fetch(center.x, center.y, stride),
		     fetch(center.x + 1.0, center.y, stride)),
		vec3(fetch(center.x - 1.0, center.y + 1.0, stride),
		     fetch(center.x, center.y + 1.0, stride),
		     fetch(center.x + 1.0, center.y + 1.0, stride))));
#if defined (SWAP_BLUE)
	gl_FragColor = vec4(rgb.bgr, 1.0);
#else
	gl_FragColor = vec4(rgb, 1.0);
#endif
#endif
}
