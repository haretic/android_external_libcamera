/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024-2025, Red Hat Inc.
 *
 * Black level handling
 */

#include "blc.h"

#include <cmath>
#include <numeric>
#include <vector>

#include <libcamera/base/log.h>

#include "control_ids.h"

namespace libcamera {

namespace ipa::soft::algorithms {

LOG_DEFINE_CATEGORY(IPASoftBL)

BlackLevel::BlackLevel()
{
}

int BlackLevel::init([[maybe_unused]] IPAContext &context,
		     const ValueNode &tuningData)
{
	auto blackLevel = tuningData["blackLevel"].get<int16_t>();
	if (blackLevel.has_value()) {
		/*
		 * Convert 16 bit values from the tuning file to 8 bit black
		 * level for the SoftISP.
		 */
		definedLevel_ = blackLevel.value() >> 8;
	}

	/*
	 * The black level of each position of the Bayer pattern, R, Gr, Gb and
	 * B as SensorBlackLevels has them, 16-bit values: for sensors whose
	 * positions read out with different offsets, which stripe once the
	 * gain is up. The debayer shifts each position onto its channel's
	 * level, and the channels' levels go in at their full precision.
	 */
	const auto blackLevels = tuningData["blackLevels"].get<std::vector<uint16_t>>();
	if (blackLevels.has_value()) {
		if (blackLevels->size() != 4) {
			LOG(IPASoftBL, Error) << "blackLevels needs 4 values, R, Gr, Gb, B";
			return -EINVAL;
		}
		std::array<float, 4> levels;
		for (unsigned int i = 0; i < 4; i++)
			levels[i] = (*blackLevels)[i] / 65535.0f;
		definedLevels_ = levels;
		if (!definedLevel_)
			definedLevel_ = std::lround(std::accumulate(levels.begin(), levels.end(), 0.0f) /
						    4.0f * 255.0f);
	}
	return 0;
}

/*
 * The black level of each channel, R, G, B, from the positions' levels, the
 * greens' the mean of the two, in 0..1 units.
 */
static RGB<float> channelLevels(const std::array<float, 4> &levels)
{
	return RGB<float>({ levels[0], (levels[1] + levels[2]) / 2.0f, levels[3] });
}

int BlackLevel::configure(IPAContext &context,
			  [[maybe_unused]] const IPAConfigInfo &configInfo)
{
	if (definedLevel_.has_value())
		context.configuration.black.level = definedLevel_;
	context.activeState.blc.level =
		context.configuration.black.level.value_or(16);
	return 0;
}

void BlackLevel::prepare(IPAContext &context,
			 [[maybe_unused]] const uint32_t frame,
			 [[maybe_unused]] IPAFrameContext &frameContext,
			 DebayerParams *params)
{
	/* Latch the blacklevel gain so GPUISP can apply. */
	if (definedLevels_) {
		params->blackLevel = channelLevels(*definedLevels_);
		params->blackLevels = *definedLevels_;
	} else {
		params->blackLevel = RGB<float>(context.activeState.blc.level / 255.0f);
		params->blackLevels = { 0.0f, 0.0f, 0.0f, 0.0f };
	}
}

void BlackLevel::process(IPAContext &context,
			 [[maybe_unused]] const uint32_t frame,
			 IPAFrameContext &frameContext,
			 const SwIspStats *stats,
			 ControlList &metadata)
{
	/* Assign each of the R G G B channels as the same black level. */
	const int32_t blackLevel = context.activeState.blc.level * 256;
	int32_t blackLevels[] = {
		blackLevel, blackLevel, blackLevel, blackLevel
	};
	if (definedLevels_) {
		for (unsigned int i = 0; i < 4; i++)
			blackLevels[i] = std::lround((*definedLevels_)[i] * 65535.0f);
	}
	metadata.set(controls::SensorBlackLevels, blackLevels);

	if (!stats->valid)
		return;

	if (context.configuration.black.level.has_value())
		return;

	if (frameContext.sensor.exposure == context.activeState.blc.lastExposure &&
	    frameContext.sensor.gain == context.activeState.blc.lastGain) {
		return;
	}

	const SwIspStats::Histogram &histogram = stats->yHistogram;

	/*
	 * The constant is selected to be "good enough", not overly
	 * conservative or aggressive. There is no magic about the given value.
	 */
	constexpr float ignoredPercentage = 0.02;
	const unsigned int total =
		std::accumulate(begin(histogram), end(histogram), 0);
	if (total == 0) {
		LOG(IPASoftBL, Debug) << "Not guessing black level, histogram is empty";
		return;
	}

	const unsigned int pixelThreshold = ignoredPercentage * total;
	const unsigned int histogramRatio = 256 / SwIspStats::kYHistogramSize;
	const unsigned int currentBlackIdx =
		context.activeState.blc.level / histogramRatio;

	for (unsigned int i = 0, seen = 0;
	     i < currentBlackIdx && i < SwIspStats::kYHistogramSize;
	     i++) {
		seen += histogram[i];
		if (seen >= pixelThreshold) {
			context.activeState.blc.level = i * histogramRatio;
			context.activeState.blc.lastExposure = frameContext.sensor.exposure;
			context.activeState.blc.lastGain = frameContext.sensor.gain;
			LOG(IPASoftBL, Debug)
				<< "Auto-set black level: "
				<< i << "/" << SwIspStats::kYHistogramSize
				<< " (" << 100 * (seen - histogram[i]) / total << "% below, "
				<< 100 * seen / total << "% at or below)";
			break;
		}
	};
}

REGISTER_IPA_ALGORITHM(BlackLevel, "BlackLevel")

} /* namespace ipa::soft::algorithms */

} /* namespace libcamera */
