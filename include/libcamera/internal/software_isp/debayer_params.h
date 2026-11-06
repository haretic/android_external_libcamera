/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023-2026 Red Hat Inc.
 *
 * Authors:
 * Hans de Goede <hdegoede@redhat.com>
 *
 * DebayerParams header
 */

#pragma once

#include <array>
#include <stdint.h>

#include "libcamera/internal/matrix.h"
#include "libcamera/internal/vector.h"

namespace libcamera {

struct DebayerParams {
	Matrix<float, 3, 3> combinedMatrix = { { 1.0, 0.0, 0.0,
						 0.0, 1.0, 0.0,
						 0.0, 0.0, 1.0 } };
	RGB<float> blackLevel = RGB<float>({ 0.0, 0.0, 0.0 });
	/*
	 * The black level of each position of the Bayer pattern, R, Gr, Gb and
	 * B, where a position's black differs from its channel's in blackLevel:
	 * the samples are shifted onto the channel's level before anything
	 * else. All equal to blackLevel's channels (or all zero) is no shift.
	 */
	std::array<float, 4> blackLevels = { 0.0, 0.0, 0.0, 0.0 };
	float gamma = 1.0;
	float contrastExp = 1.0;
	RGB<float> gains = RGB<float>({ 1.0, 1.0, 1.0 });
	/*
	 * Noise variance of a raw sample in units of white squared: shot
	 * times the signal plus read, at the gain the frame was taken with.
	 * Zero leaves the noise alone.
	 */
	float noiseShot = 0.0;
	float noiseRead = 0.0;
};

} /* namespace libcamera */
