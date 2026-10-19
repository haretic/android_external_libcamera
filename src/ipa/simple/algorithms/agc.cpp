/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Red Hat Inc.
 *
 * Exposure and gain
 */

#include "agc.h"

#include <algorithm>
#include <cmath>
#include <stdint.h>

#include <libcamera/base/log.h>

#include "control_ids.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(IPASoftExposure)

namespace ipa::soft::algorithms {

/*
 * The number of bins to use for the optimal exposure calculations.
 */
static constexpr unsigned int kExposureBinsCount = 5;

/*
 * The exposure is optimal when the mean sample value of the histogram is
 * in the middle of the range.
 */
static constexpr float kExposureOptimal = kExposureBinsCount / 2.0;

/*
 * This implements the hysteresis for the exposure adjustment.
 * It is small enough to have the exposure close to the optimal, and is big
 * enough to prevent the exposure from wobbling around the optimal value.
 */
static constexpr float kExposureSatisfactory = 0.2;

/*
 * Proportional gain for exposure/gain adjustment. Maps the MSV error to a
 * multiplicative correction factor:
 *
 *   factor = 1.0 + kExpProportionalGain * error
 *
 * The MSV runs from 1 to kExposureBinsCount, so a dark scene's error is at
 * most 1.5 and a saturated one's -2.5. Each correction scales the exposure
 * and gain the statistics frame was taken with, so the sensor's control
 * delay doesn't accumulate, and a large gain stays stable:
 *   - error 1.5 (dark)   -> factor 1.375
 *   - error 0.3          -> factor 1.075
 *   - error -2.5 (white) -> factor 0.6 (kExpMaxStep)
 *
 * In low light the exposure and gain then reach their limits in a few
 * seconds; with 0.04 it took half a minute.
 */
static constexpr float kExpProportionalGain = 0.25;

/*
 * Maximum multiplicative step per statistics frame, to bound the correction when the
 * scene changes dramatically.
 */
static constexpr float kExpMaxStep = 0.4;

/*
 * Gain the ISP applies once exposure and analogue gain are at their limits,
 * for scenes too dark for the sensor alone.
 */
static constexpr double kDigitalGainMax = 4.0;

Agc::Agc()
{
}

int Agc::configure(IPAContext &context,
		   [[maybe_unused]] const IPAConfigInfo &configInfo)
{
	context.activeState.agc.digitalGain = 1.0;

	return 0;
}

void Agc::prepare(IPAContext &context,
		  [[maybe_unused]] const uint32_t frame,
		  IPAFrameContext &frameContext,
		  DebayerParams *params)
{
	/* Awb, earlier in the tuning file, has set the colour gains. */
	const double digitalGain = context.activeState.agc.digitalGain;

	frameContext.digitalGain = digitalGain;
	params->gains = params->gains * static_cast<float>(digitalGain);
}

void Agc::queueRequest(IPAContext &context,
		       [[maybe_unused]] const uint32_t frame,
		       [[maybe_unused]] IPAFrameContext &frameContext,
		       const ControlList &controls)
{
	const auto &agc = context.configuration.agc;
	const auto &frameDurationLimits = controls.get(controls::FrameDurationLimits);

	if (!frameDurationLimits || !agc.vblankMax)
		return;

	/* Keep frames from getting shorter than the shortest duration. */
	const int64_t height = context.sensorInfo.outputSize.height;
	const utils::Duration minFrameDuration =
		std::chrono::microseconds((*frameDurationLimits).front());
	context.activeState.agc.vblankMin =
		std::clamp<int64_t>(minFrameDuration / agc.lineDuration - height,
				    agc.vblankMin, agc.vblankMax);

	/*
	 * Allow exposures up to the longest frame duration requested, within
	 * what the sensor blanking can stretch the frame to.
	 */
	const utils::Duration maxFrameDuration =
		std::chrono::microseconds((*frameDurationLimits).back());
	const int64_t frameLength = maxFrameDuration / agc.lineDuration;
	const int64_t exposureLimit = height + agc.vblankMax - agc.exposureMargin;

	context.activeState.agc.exposureMax =
		std::clamp<int64_t>(frameLength - agc.exposureMargin,
				    agc.exposureMax, std::max<int64_t>(agc.exposureMax, exposureLimit));
}

void Agc::updateExposure(IPAContext &context, IPAFrameContext &frameContext, double exposureMSV)
{
	int32_t &exposure = frameContext.sensor.exposure;
	double &again = frameContext.sensor.gain;
	double digitalGain = frameContext.digitalGain;

	const int32_t exposureMax = context.activeState.agc.exposureMax;
	double error = kExposureOptimal - exposureMSV;

	if (std::abs(error) <= kExposureSatisfactory && exposure <= exposureMax)
		return;

	/*
	 * Compute a proportional correction factor. The sign of the error
	 * determines the direction: positive error means too dark (increase),
	 * negative means too bright (decrease).
	 */
	float step = std::clamp(static_cast<float>(error) * kExpProportionalGain,
				-kExpMaxStep, kExpMaxStep);
	float factor = 1.0f + step;

	if (factor > 1.0f) {
		/*
		 * Scene too dark: increase exposure first, then gain, then the
		 * ISP digital gain.
		 */
		if (exposure < exposureMax) {
			int32_t next = static_cast<int32_t>(exposure * factor);
			exposure = std::max(next, exposure + 1);
		} else if (again >= context.configuration.agc.againMax) {
			digitalGain = std::min(digitalGain * factor, kDigitalGainMax);
		} else {
			double next = again * factor;
			if (next - again < context.configuration.agc.againMinStep)
				again += context.configuration.agc.againMinStep;
			else
				again = next;
		}
	} else {
		/*
		 * Scene too bright: decrease the digital gain first, then the
		 * gain, then exposure.
		 */
		if (digitalGain > 1.0) {
			digitalGain = std::max(digitalGain * factor, 1.0);
		} else if (again > context.configuration.agc.again10) {
			double next = again * factor;
			if (again - next < context.configuration.agc.againMinStep)
				again -= context.configuration.agc.againMinStep;
			else
				again = next;
		} else {
			int32_t next = static_cast<int32_t>(exposure * factor);
			exposure = std::min(next, exposure - 1);
		}
	}

	exposure = std::clamp(exposure, context.configuration.agc.exposureMin,
			      exposureMax);
	again = std::clamp(again, context.configuration.agc.againMin,
			   context.configuration.agc.againMax);

	context.activeState.agc.exposure = exposure;
	context.activeState.agc.again = again;
	context.activeState.agc.digitalGain = digitalGain;

	LOG(IPASoftExposure, Debug)
		<< "exposureMSV " << exposureMSV
		<< " error " << error << " factor " << factor
		<< " exp " << exposure << " again " << again
		<< " dgain " << digitalGain;
}

void Agc::process(IPAContext &context,
		  [[maybe_unused]] const uint32_t frame,
		  IPAFrameContext &frameContext,
		  const SwIspStats *stats,
		  ControlList &metadata)
{
	utils::Duration exposureTime =
		context.configuration.agc.lineDuration * frameContext.sensor.exposure;
	metadata.set(controls::ExposureTime, exposureTime.get<std::micro>());
	metadata.set(controls::AnalogueGain, frameContext.sensor.gain);
	metadata.set(controls::DigitalGain, frameContext.digitalGain);
	if (context.configuration.agc.vblankMax) {
		utils::Duration frameDuration = context.configuration.agc.lineDuration *
			(context.sensorInfo.outputSize.height + frameContext.sensor.vblank);
		metadata.set(controls::FrameDuration, frameDuration.get<std::micro>());
	}

	if (!context.activeState.agc.valid) {
		/*
		 * Init active-state from sensor values in case updateExposure()
		 * does not run for the first frame.
		 */
		context.activeState.agc.exposure = frameContext.sensor.exposure;
		context.activeState.agc.again = frameContext.sensor.gain;
		context.activeState.agc.valid = true;
	}

	if (!stats->valid) {
		/*
		 * Use the new exposure and gain values calculated the last time
		 * there were valid stats.
		 */
		frameContext.sensor.exposure = context.activeState.agc.exposure;
		frameContext.sensor.gain = context.activeState.agc.again;
		return;
	}

	/*
	 * Calculate Mean Sample Value (MSV) according to formula from:
	 * https://www.araa.asn.au/acra/acra2007/papers/paper84final.pdf
	 */
	const auto &histogram = stats->yHistogram;
	const unsigned int blackLevelHistIdx =
		context.activeState.blc.level / (256 / SwIspStats::kYHistogramSize);
	const unsigned int histogramSize =
		SwIspStats::kYHistogramSize - blackLevelHistIdx;
	const unsigned int yHistValsPerBin = histogramSize / kExposureBinsCount;
	const unsigned int yHistValsPerBinMod =
		histogramSize / (histogramSize % kExposureBinsCount + 1);
	int exposureBins[kExposureBinsCount] = {};
	unsigned int denom = 0;
	unsigned int num = 0;

	if (yHistValsPerBin == 0) {
		LOG(IPASoftExposure, Debug)
			<< "Not adjusting exposure due to insufficient histogram data";
		return;
	}

	/*
	 * The statistics come from the raw frame: place each value where the
	 * digital gain the frame was processed with puts it.
	 */
	for (unsigned int i = 0; i < histogramSize; i++) {
		const unsigned int j =
			std::min<unsigned int>(i * frameContext.digitalGain,
					       histogramSize - 1);
		unsigned int idx = (j - (j / yHistValsPerBinMod)) / yHistValsPerBin;
		exposureBins[idx] += histogram[blackLevelHistIdx + i];
	}

	for (unsigned int i = 0; i < kExposureBinsCount; i++) {
		LOG(IPASoftExposure, Debug) << i << ": " << exposureBins[i];
		denom += exposureBins[i];
		num += exposureBins[i] * (i + 1);
	}

	float exposureMSV = (denom == 0 ? 0 : static_cast<float>(num) / denom);
	updateExposure(context, frameContext, exposureMSV);
}

REGISTER_IPA_ALGORITHM(Agc, "Agc")

} /* namespace ipa::soft::algorithms */

} /* namespace libcamera */
