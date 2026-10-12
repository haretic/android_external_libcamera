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

} /* namespace libcamera */
