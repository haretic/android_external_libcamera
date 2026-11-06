/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Red Hat Inc.
 *
 * Black level handling
 */

#pragma once

#include <array>
#include <optional>

#include <optional>
#include <stdint.h>

#include "algorithm.h"

namespace libcamera {

namespace ipa::soft::algorithms {

class BlackLevel : public Algorithm
{
public:
	BlackLevel();
	~BlackLevel() = default;

	int init(IPAContext &context, const ValueNode &tuningData) override;
	int configure(IPAContext &context, const IPAConfigInfo &configInfo) override;
	void prepare(IPAContext &context,
		     const uint32_t frame,
		     IPAFrameContext &frameContext,
		     DebayerParams *params) override;
	void process(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     const SwIspStats *stats,
		     ControlList &metadata) override;

private:
	std::optional<uint8_t> definedLevel_;
	/* The black level of each position of the pattern, R, Gr, Gb, B, 0..1 */
	std::optional<std::array<float, 4>> definedLevels_;
};

} /* namespace ipa::soft::algorithms */

} /* namespace libcamera */
