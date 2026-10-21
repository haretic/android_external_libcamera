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
/* The contrast S-curve and the gamma, 1024 steps */
uniform sampler2D tex_lut;

/*
 * The texture holds four bytes of a line in each texel, in r, g, b and a,
 * the stride a quarter as wide; tex_step.x is 4 / (stride - 1).
 */
float texels()
{
	return (4.0 / tex_step.x + 1.0) * 0.25;
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
	x = clamp(x, 0.0, tex_size.x - 1.0);
	y = clamp(y, 0.0, tex_size.y - 1.0);
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
	rgb = (rgb - blacklevel) / (1.0 - blacklevel);

	/* Apply AWB gains, and saturate each channel at sensor range */
	rgb = clamp(rgb * awb, vec3(0.0), vec3(1.0));

	/* Colour Correction Matrix */
	rgb = ccm * rgb;

	/* Contrast and gamma after colour correction, from their table */
	rgb = clamp(rgb, 0.0, 1.0) * (1023.0 / 1024.0) + 0.5 / 1024.0;
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
	y = clamp(y, 0.0, tex_size.y - 1.0);
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

vec2 chroma(vec3 rgb)
{
	return vec2(dot(rgb, vec3(-0.168736, -0.331264, 0.5)),
		    dot(rgb, vec3(0.5, -0.418688, -0.081312))) + 0.5;
}
#endif

void main(void)
{
	float stride = texels();
	vec3 c0, c1, c2, c3;

#if defined (OUTPUT_Y)
	vec2 p = floor(gl_FragCoord.xy) * vec2(4.0, 1.0) + out_origin;
	group_colours(p.x * 0.25, p.y, stride, c0, c1, c2, c3);
	gl_FragColor = vec4(luma(process(c0)), luma(process(c1)),
			    luma(process(c2)), luma(process(c3)));
#elif defined (OUTPUT_UV)
	/* The chroma of two 2x2 blocks, sited at their top left pixels */
	vec2 p = floor(gl_FragCoord.xy) * vec2(4.0, 2.0) + out_origin;
	group_colours(p.x * 0.25, p.y, stride, c0, c1, c2, c3);
	vec2 a = chroma(process(c0));
	vec2 b = chroma(process(c2));
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
