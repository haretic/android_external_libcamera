/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2026, itskenny0
 *
 * MediaTek ISP packed raw Bayer helpers
 */

#pragma once

#include <stdint.h>

namespace libcamera {

/*
 * Unpack \a width 10-bit samples of a line stored in the MediaTek ISP packed
 * layout to 16 bits per sample. The samples are stored as a little-endian bit
 * stream, four samples in five bytes. \a width shall be a multiple of 4 and
 * \a src shall point to the start of a group of four samples.
 */
static inline void unpackMtisp10(uint16_t *dst, const uint8_t *src,
				 unsigned int width)
{
	for (unsigned int x = 0; x < width; x += 4, src += 5, dst += 4) {
		dst[0] = src[0] | ((src[1] & 0x03) << 8);
		dst[1] = (src[1] >> 2) | ((src[2] & 0x0f) << 6);
		dst[2] = (src[2] >> 4) | ((src[3] & 0x3f) << 4);
		dst[3] = (src[3] >> 6) | (src[4] << 2);
	}
}

/*
 * Unpack \a width samples of a line as unpackMtisp10() does, scaled to 12 bits
 * (four times their value) with \a shift[k] added to the samples of columns of
 * parity k, in the same 12-bit units, the sums clamped to the 12-bit range.
 * The shifts put columns with different black levels on one level, with a
 * quarter of a 10-bit step to spare.
 */
static inline void unpackMtisp10To12(uint16_t *dst, const uint8_t *src,
				     unsigned int width, const int shift[2])
{
	auto clamp12 = [](int v) -> uint16_t {
		return static_cast<uint16_t>(v < 0 ? 0 : (v > 4095 ? 4095 : v));
	};
	const int even = shift[0];
	const int odd = shift[1];

	for (unsigned int x = 0; x < width; x += 4, src += 5, dst += 4) {
		dst[0] = clamp12(((src[0] | ((src[1] & 0x03) << 8)) << 2) + even);
		dst[1] = clamp12((((src[1] >> 2) | ((src[2] & 0x0f) << 6)) << 2) + odd);
		dst[2] = clamp12((((src[2] >> 4) | ((src[3] & 0x3f) << 4)) << 2) + even);
		dst[3] = clamp12((((src[3] >> 6) | (src[4] << 2)) << 2) + odd);
	}
}

} /* namespace libcamera */
