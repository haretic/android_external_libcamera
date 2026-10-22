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
 * The tone curve, the contrast S-curve and the gamma, 1024 steps over 0 to
 * TONE_RANGE indexed by the square root, for finer steps in the shadows
 * (DebayerEGL::updateLut() has the same range).
 */
uniform sampler2D tex_lut;
#define TONE_RANGE 16.0

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
	return value / 1023.0;
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

void group_line(float g, float y, float stride, out vec4 s, out vec2 edge)
{
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

	/* Beyond the image, the samples of the same colour mirrored in its edge */
	if (g < 0.5)
		edge.x = s.y;
	if (4.0 * g + 4.0 > tex_size.x - 0.5)
		edge.y = s.z;
}

/* The colours of the pixels of group \a g of line \a y, linear */
void group_colours(float g, float y, float stride,
		   out vec3 c0, out vec3 c1, out vec3 c2, out vec3 c3)
{
	vec4 t, m, b;
	vec2 te, me, be;
	group_line(g, y - 1.0, stride, t, te);
	group_line(g, y, stride, m, me);
	group_line(g, y + 1.0, stride, b, be);

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
		    floor(b.y / s1 + 0.001) + mod(b.z, 4.0 * s1) * (256.0 / s1)) / 1023.0;
}

/* The colour of the quad at q, both even: red where first red has it */
vec3 quad(vec2 q, float stride)
{
	q.y = min(q.y, tex_size.y - 2.0);
	vec2 top = pair_at(q.x, q.y, stride);
	vec2 bot = pair_at(q.x, q.y + 1.0, stride);
	vec2 red_line = tex_bayer_first_red.y < 0.5 ? top : bot;
	vec2 blue_line = tex_bayer_first_red.y < 0.5 ? bot : top;
	float r = tex_bayer_first_red.x < 0.5 ? red_line.x : red_line.y;
	float b = tex_bayer_first_red.x < 0.5 ? blue_line.y : blue_line.x;
	return vec3(r, (red_line.x + red_line.y + blue_line.x + blue_line.y - r - b) * 0.5, b);
}

#if QUADS == 2
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

	return vec4(b.x + mod(b.y, 4.0) * 256.0,
		    floor(b.y / 4.0 + 0.001) + mod(b.z, 16.0) * 64.0,
		    floor(b.z / 16.0 + 0.001) + mod(b.w, 64.0) * 16.0,
		    floor(b.w / 64.0 + 0.001) + b4 * 4.0) / 1023.0;
}

/* The summed colours of the two quads of group g on lines y and y + 1 */
vec3 group_quads(float g, float y, float stride)
{
	vec4 top = group_at(g, y, stride);
	vec4 bot = group_at(g, y + 1.0, stride);
	vec4 red_line = tex_bayer_first_red.y < 0.5 ? top : bot;
	vec4 blue_line = tex_bayer_first_red.y < 0.5 ? bot : top;
	vec4 even = tex_bayer_first_red.x < 0.5 ? vec4(1.0, 0.0, 1.0, 0.0) : vec4(0.0, 1.0, 0.0, 1.0);
	float r = dot(red_line, even);
	float b = dot(blue_line, 1.0 - even);
	return vec3(r, dot(top + bot, vec4(1.0)) - r - b, b);
}

/*
 * The mean colour of the 2 x 2 quads around c, in samples: the two groups'
 * worth on four lines nearest it, up to two samples off to the side.
 */
vec3 area(vec2 c, float stride)
{
	float g = floor((c.x - 2.0) * 0.25 + 0.5);
	float y = 2.0 * floor((c.y - 2.0) * 0.5 + 0.5);
	vec3 sum = group_quads(g, y, stride) + group_quads(g, y + 2.0, stride);
	return sum * vec3(0.25, 0.125, 0.25);
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
	vec4 s0, s1, s2, s3;
	vec2 e0, e1, e2, e3;
	group_line(g, p.y - 1.0, stride, s0, e0);
	group_line(g, p.y, stride, s1, e1);
	group_line(g, p.y + 1.0, stride, s2, e2);
	group_line(g, p.y + 2.0, stride, s3, e3);

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
