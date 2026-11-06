/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Red Hat Inc.
 *
 * Exposure and gain
 */

#pragma once

#include "algorithm.h"

namespace libcamera {

namespace ipa::soft::algorithms {

class Agc : public Algorithm
{
public:
	Agc();
	~Agc() = default;

	int init(IPAContext &context, const ValueNode &tuningData) override;
	int configure(IPAContext &context, const IPAConfigInfo &configInfo) override;
	void queueRequest(IPAContext &context, const uint32_t frame,
			  IPAFrameContext &frameContext,
			  const ControlList &controls) override;
	void prepare(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     DebayerParams *params) override;
	void process(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     const SwIspStats *stats,
		     ControlList &metadata) override;

private:
	void allocate(IPAContext &context, double total);

	/* Tuning */
	double target_;
	double tolerance_;
	double speed_;
	double maxStep_;
	double highlightLevel_;
	double highlightFraction_;
	double againMax_;
	double againStep_;
	double digitalGainMax_;
	unsigned int settleFrames_;
	double initialExposureTime_;
	double noiseShot_;
	double noiseRead_;
};

} /* namespace ipa::soft::algorithms */

} /* namespace libcamera */
