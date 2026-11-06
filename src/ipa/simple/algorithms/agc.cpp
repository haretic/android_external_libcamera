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
 * The loop meters the mean luminance of the raw frame, above the black level
 * and relative to white, and moves the exposure towards a target the way a
 * phone's AE does: the whole error at once, in a bounded step, then waits
 * for frames taken with the new settings before it looks again. Without a
 * frame start event the pipeline cannot tell which frame a control landed
 * on, so corrections are only ever computed from settled frames and applied
 * to the values the loop itself set, which the frame then had.
 *
 * The defaults are the stock MediaTek tuning's where it was readable: the
 * target of 50/255 (u4AETarget), highlights counted from 240/255
 * (u4OverExpThres), analogue gain up to 8x (u4MaxGain 8192).
 */
static constexpr double kDefaultTarget = 50.0 / 255.0;
static constexpr double kDefaultTolerance = 0.08;
static constexpr double kDefaultSpeed = 0.8;
static constexpr double kDefaultMaxStep = 2.0;
static constexpr double kDefaultHighlightLevel = 240.0 / 255.0;
static constexpr double kDefaultHighlightFraction = 0.03;
static constexpr double kDefaultAgainMax = 8.0;
static constexpr double kDefaultDigitalGainMax = 4.0;
static constexpr unsigned int kDefaultSettleFrames = 6;
static constexpr double kDefaultInitialExposureTime = 1.0 / 50.0;
/*
 * Noise variance of a sample in units of white squared, per unit of gain
 * for the shot term (full well about 4500 electrons, estimated) and per
 * unit of gain squared for the read term (4 LSB of 1023 at 16x, measured
 * on a dark frame).
 */
static constexpr double kDefaultNoiseShot = 2.2e-4;
static constexpr double kDefaultNoiseRead = 6.0e-8;

Agc::Agc()
{
}

int Agc::init([[maybe_unused]] IPAContext &context, const ValueNode &tuningData)
{
	target_ = tuningData["target"].get<double>(kDefaultTarget);
	tolerance_ = tuningData["tolerance"].get<double>(kDefaultTolerance);
	speed_ = tuningData["speed"].get<double>(kDefaultSpeed);
	maxStep_ = tuningData["maxStep"].get<double>(kDefaultMaxStep);
	highlightLevel_ = tuningData["highlightLevel"].get<double>(kDefaultHighlightLevel);
	highlightFraction_ = tuningData["highlightFraction"].get<double>(kDefaultHighlightFraction);
	againMax_ = tuningData["againMax"].get<double>(kDefaultAgainMax);
	/* The gain of one sensor control step, 0 to let the helper round */
	againStep_ = tuningData["againStep"].get<double>(0.0);
	digitalGainMax_ = tuningData["digitalGainMax"].get<double>(kDefaultDigitalGainMax);
	settleFrames_ = tuningData["settleFrames"].get<uint32_t>(kDefaultSettleFrames);
	initialExposureTime_ = tuningData["initialExposureTime"].get<double>(kDefaultInitialExposureTime);
	noiseShot_ = tuningData["noiseShot"].get<double>(kDefaultNoiseShot);
	noiseRead_ = tuningData["noiseRead"].get<double>(kDefaultNoiseRead);

	return 0;
}

int Agc::configure(IPAContext &context,
		   [[maybe_unused]] const IPAConfigInfo &configInfo)
{
	auto &agc = context.configuration.agc;
	auto &state = context.activeState.agc;

	agc.againLimit = std::clamp(againMax_, std::max(agc.againMin, 1.0), agc.againMax);
	agc.againQuantum = againStep_;
	if (agc.vblankMax)
		agc.exposureMinFrame = std::clamp<int32_t>(
			context.sensorInfo.outputSize.height + agc.vblankMin - agc.exposureMargin,
			agc.exposureMin, agc.exposureMax);
	else
		agc.exposureMinFrame = agc.exposureMax;

	/* Start a little under a fiftieth of a second at the lowest gain. */
	const double lines = initialExposureTime_ / agc.lineDuration.get<std::ratio<1>>();
	state.exposure = std::clamp<int32_t>(lines, agc.exposureMin, agc.exposureMinFrame);
	state.again = std::max(agc.againMin, 1.0);
	state.digitalGain = 1.0;
	state.lastChange = 0;
	state.valid = true;

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

	/* The noise the ISP's filter expects at the gain the frame has. */
	const double gain = context.activeState.agc.again * digitalGain;
	params->noiseShot = noiseShot_ * gain;
	params->noiseRead = noiseRead_ * gain * gain;
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

/*
 * Split a total of exposure lines times gain between the controls: exposure
 * up to the longest that keeps the frame rate, then analogue gain to the
 * tuning's limit, then exposure stretching the frame as far as the frame
 * duration limits allow, then the ISP's digital gain. The analogue gain is
 * kept to the sensor's steps; below its limit what the rounding leaves goes
 * to the digital gain, not to a longer frame, so the frame rate holds until
 * the analogue gain is used up.
 */
void Agc::allocate(IPAContext &context, double total)
{
	const auto &agc = context.configuration.agc;
	auto &state = context.activeState.agc;
	const int32_t exposureMax = std::max(state.exposureMax, agc.exposureMinFrame);
	const int32_t exposureMinFrame = std::min(agc.exposureMinFrame, exposureMax);
	const double againMin = std::max(agc.againMin, 1.0);

	double exposure = std::clamp<double>(total / againMin, agc.exposureMin, exposureMinFrame);
	double again = std::clamp(total / exposure, againMin, agc.againLimit);
	if (agc.againQuantum > 0.0)
		again = againMin + std::floor((again - againMin) / agc.againQuantum + 1e-6) *
					   agc.againQuantum;
	exposure = std::clamp<double>(total / again, agc.exposureMin,
				      again < agc.againLimit ? exposureMinFrame : exposureMax);
	double digitalGain = std::clamp(total / (exposure * again), 1.0, digitalGainMax_);

	state.exposure = std::clamp<int32_t>(std::lround(exposure), agc.exposureMin, exposureMax);
	state.again = again;
	state.digitalGain = digitalGain;
}

void Agc::process(IPAContext &context,
		  const uint32_t frame,
		  IPAFrameContext &frameContext,
		  const SwIspStats *stats,
		  ControlList &metadata)
{
	const auto &agc = context.configuration.agc;
	auto &state = context.activeState.agc;

	utils::Duration exposureTime = agc.lineDuration * frameContext.sensor.exposure;
	metadata.set(controls::ExposureTime, exposureTime.get<std::micro>());
	metadata.set(controls::AnalogueGain, frameContext.sensor.gain);
	metadata.set(controls::DigitalGain, frameContext.digitalGain);
	if (agc.vblankMax) {
		utils::Duration frameDuration = agc.lineDuration *
			(context.sensorInfo.outputSize.height + frameContext.sensor.vblank);
		metadata.set(controls::FrameDuration, frameDuration.get<std::micro>());
	}

	if (!state.valid) {
		state.exposure = frameContext.sensor.exposure;
		state.again = frameContext.sensor.gain;
		state.digitalGain = 1.0;
		state.lastChange = frame;
		state.valid = true;
	}

	/*
	 * The controls written after this frame are always the loop's
	 * current ones; the frame's own values above are only metadata.
	 */
	auto apply = [&]() {
		frameContext.sensor.exposure = state.exposure;
		frameContext.sensor.gain = state.again;
	};

	if (!stats->valid || frame - state.lastChange < settleFrames_) {
		apply();
		return;
	}

	/*
	 * The mean luminance of the frame above the black level, and the
	 * share of it at or past the highlight level, from the histogram of
	 * 8-bit luminance.
	 */
	const auto &histogram = stats->yHistogram;
	constexpr unsigned int kBins = SwIspStats::kYHistogramSize;
	constexpr double kBinWidth = 256.0 / kBins;
	const double black = context.activeState.blc.level;
	const unsigned int highlightBin = std::min<unsigned int>(
		highlightLevel_ * 255.0 / kBinWidth, kBins - 1);
	double count = 0.0, sum = 0.0, highlights = 0.0;
	for (unsigned int i = 0; i < kBins; i++) {
		count += histogram[i];
		sum += histogram[i] * (i + 0.5) * kBinWidth;
		if (i >= highlightBin)
			highlights += histogram[i];
	}
	if (count == 0.0) {
		apply();
		return;
	}

	const double mean = std::max((sum / count - black) / (255.0 - black), 1e-4);
	const double highlightShare = highlights / count;

	/*
	 * The digital gain the frame was shown with brightens it past what
	 * the sensor recorded; the loop balances what is shown.
	 */
	const double shown = mean * frameContext.digitalGain;

	/* Protect highlights: too many bright pixels pull the target down. */
	double target = target_;
	if (highlightShare > highlightFraction_)
		target *= std::max(highlightFraction_ / highlightShare, 0.5);

	const double error = std::log2(target / shown);
	if (std::abs(error) <= std::log2(1.0 + tolerance_)) {
		apply();
		return;
	}

	const double step = std::clamp(error * speed_, -maxStep_, maxStep_);
	const double total = state.exposure * state.again * state.digitalGain * std::exp2(step);

	allocate(context, total);
	state.lastChange = frame;
	apply();

	LOG(IPASoftExposure, Debug)
		<< "frame " << frame << " mean " << mean
		<< " highlights " << highlightShare << " target " << target
		<< " error " << error << " step " << step
		<< " exp " << state.exposure << " again " << state.again
		<< " dgain " << state.digitalGain;
}

REGISTER_IPA_ALGORITHM(Agc, "Agc")

} /* namespace ipa::soft::algorithms */

} /* namespace libcamera */
