/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024-2026 Red Hat Inc.
 *
 * Auto white balance
 */

#include "awb.h"

#include <algorithm>
#include <numeric>
#include <stdint.h>

#include <libcamera/base/log.h>

#include <libcamera/control_ids.h>

#include "libipa/colours.h"
#include "simple/ipa_context.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(IPASoftAwb)

namespace ipa::soft::algorithms {

/*
 * Grey world takes every cast out, but warm light that looks grey looks
 * wrong: the tuning's warmth keeps some of it, a red over blue tint that
 * fades in from 5000 K down to 2500 K.
 */
int Awb::init([[maybe_unused]] IPAContext &context, const ValueNode &tuningData)
{
	warmth_ = tuningData["warmth"].get<float>(0.0f);
	return 0;
}

int Awb::configure(IPAContext &context,
		   [[maybe_unused]] const IPAConfigInfo &configInfo)
{
	auto &gains = context.activeState.awb.gains;
	gains = { { 1.0, 1.0, 1.0 } };
	/* Neutral, no warmth, until the first statistics */
	context.activeState.awb.temperatureK = 5000;

	return 0;
}

void Awb::prepare(IPAContext &context,
		  [[maybe_unused]] const uint32_t frame,
		  IPAFrameContext &frameContext,
		  DebayerParams *params)
{
	auto &gains = context.activeState.awb.gains;

	frameContext.gains = gains;

	const float ct = context.activeState.awb.temperatureK;
	const float warmth = warmth_ * std::clamp((5000.0f - ct) / 2500.0f, 0.0f, 1.0f);
	params->gains = gains * RGB<float>({ 1.0f + warmth, 1.0f, 1.0f - warmth });
}

void Awb::process(IPAContext &context,
		  [[maybe_unused]] const uint32_t frame,
		  IPAFrameContext &frameContext,
		  const SwIspStats *stats,
		  ControlList &metadata)
{
	const SwIspStats::Histogram &histogram = stats->yHistogram;
	const uint8_t blackLevel = context.activeState.blc.level;

	metadata.set(controls::ColourGains, { frameContext.gains.r(),
					      frameContext.gains.b() });

	if (!stats->valid)
		return;

	/*
	 * Black level must be subtracted to get the correct AWB ratios, they
	 * would be off if they were computed from the whole brightness range
	 * rather than from the sensor range.
	 */
	const uint64_t nPixels = std::accumulate(
		histogram.begin(), histogram.end(), uint64_t(0));
	const uint64_t offset = blackLevel * nPixels;
	const uint64_t minValid = 1;
	/*
	 * Make sure the sums are at least minValid, while preventing unsigned
	 * integer underflow.
	 */
	const RGB<uint64_t> sum = stats->sum_.max(offset + minValid) - offset;

	/*
	 * Calculate red and blue gains for AWB.
	 * Clamp max gain at 4.0, this also avoids 0 division.
	 */
	auto &gains = context.activeState.awb.gains;
	gains = { {
		sum.r() <= sum.g() / 4 ? 4.0f : static_cast<float>(sum.g()) / sum.r(),
		1.0,
		sum.b() <= sum.g() / 4 ? 4.0f : static_cast<float>(sum.g()) / sum.b(),
	} };

	RGB<double> rgbGains{ { 1 / gains.r(), 1 / gains.g(), 1 / gains.b() } };
	context.activeState.awb.temperatureK = estimateCCT(rgbGains);
	metadata.set(controls::ColourTemperature, context.activeState.awb.temperatureK);

	LOG(IPASoftAwb, Debug)
		<< "gain R/B: " << gains << "; temperature: "
		<< context.activeState.awb.temperatureK;
}

REGISTER_IPA_ALGORITHM(Awb, "Awb")

} /* namespace ipa::soft::algorithms */

} /* namespace libcamera */
