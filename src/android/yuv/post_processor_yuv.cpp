/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Google Inc.
 *
 * Post Processor using libyuv
 */

#include "post_processor_yuv.h"

#include <libyuv/planar_functions.h>
#include <libyuv/scale.h>
#include <libyuv/scale_uv.h>

#include <libcamera/base/log.h>

#include <libcamera/formats.h>
#include <libcamera/geometry.h>
#include <libcamera/pixel_format.h>

#include "libcamera/internal/formats.h"
#include "libcamera/internal/mapped_framebuffer.h"

using namespace libcamera;

LOG_DEFINE_CATEGORY(YUV)

int PostProcessorYuv::configure(const StreamConfiguration &inCfg,
				const StreamConfiguration &outCfg)
{
	if (inCfg.pixelFormat != outCfg.pixelFormat) {
		LOG(YUV, Error) << "Pixel format conversion is not supported"
				<< " (from " << inCfg.pixelFormat
				<< " to " << outCfg.pixelFormat << ")";
		return -EINVAL;
	}

	if (inCfg.size < outCfg.size) {
		LOG(YUV, Error) << "Up-scaling is not supported"
				<< " (from " << inCfg.size
				<< " to " << outCfg.size << ")";
		return -EINVAL;
	}

	if (inCfg.pixelFormat != formats::NV12) {
		LOG(YUV, Error) << "Unsupported format " << inCfg.pixelFormat
				<< " (only NV12 is supported)";
		return -EINVAL;
	}

	calculateLengths(inCfg, outCfg);
	return 0;
}

void PostProcessorYuv::process(Camera3RequestDescriptor::StreamBuffer *streamBuffer)
{
	const FrameBuffer &source = *streamBuffer->srcBuffer;
	CameraBuffer *destination = streamBuffer->dstBuffer.get();

	if (!isValidBuffers(source, *destination)) {
		processComplete.emit(streamBuffer, PostProcessor::Status::Error);
		return;
	}

	const MappedFrameBuffer sourceMapped(&source, MappedFrameBuffer::MapFlag::Read);
	if (!sourceMapped.isValid()) {
		LOG(YUV, Error) << "Failed to mmap camera frame buffer";
		processComplete.emit(streamBuffer, PostProcessor::Status::Error);
		return;
	}

	/*
	 * The destination is NV12 or NV21, or planar when gralloc picked that
	 * for an implementation defined format (CameraBuffer then reports Y,
	 * Cb, Cr).
	 */
	const Size chroma((destinationSize_.width + 1) / 2,
			  (destinationSize_.height + 1) / 2);
	int ret;

	if (destination->numPlanes() == 2) {
		ret = libyuv::NV12Scale(sourceMapped.planes()[0].data(),
					sourceStride_[0],
					sourceMapped.planes()[1].data(),
					sourceStride_[1],
					sourceSize_.width, sourceSize_.height,
					destination->plane(0).data(),
					destination->stride(0),
					destination->plane(1).data(),
					destination->stride(1),
					destinationSize_.width,
					destinationSize_.height,
					libyuv::FilterMode::kFilterBilinear);
		/* Swap the chroma in place for NV21 buffers. */
		if (!ret && destination->swapsChroma())
			libyuv::SwapUVPlane(destination->plane(1).data(),
					    destination->stride(1),
					    destination->plane(1).data(),
					    destination->stride(1),
					    chroma.width, chroma.height);
	} else {
		ret = libyuv::ScalePlane(sourceMapped.planes()[0].data(),
					 sourceStride_[0],
					 sourceSize_.width, sourceSize_.height,
					 destination->plane(0).data(),
					 destination->stride(0),
					 destinationSize_.width,
					 destinationSize_.height,
					 libyuv::FilterMode::kFilterBilinear);
		if (!ret)
			ret = libyuv::UVScale(sourceMapped.planes()[1].data(),
					      sourceStride_[1],
					      (sourceSize_.width + 1) / 2,
					      (sourceSize_.height + 1) / 2,
					      chroma_.data(), chroma.width * 2,
					      chroma.width, chroma.height,
					      libyuv::FilterMode::kFilterBilinear);
		if (!ret)
			libyuv::SplitUVPlane(chroma_.data(), chroma.width * 2,
					     destination->plane(1).data(),
					     destination->stride(1),
					     destination->plane(2).data(),
					     destination->stride(2),
					     chroma.width, chroma.height);
	}

	if (ret) {
		LOG(YUV, Error) << "Failed NV12 scaling: " << ret;
		processComplete.emit(streamBuffer, PostProcessor::Status::Error);
		return;
	}

	processComplete.emit(streamBuffer, PostProcessor::Status::Success);
}

bool PostProcessorYuv::isValidBuffers(const FrameBuffer &source,
				      const CameraBuffer &destination) const
{
	if (source.planes().size() != 2) {
		LOG(YUV, Error) << "Invalid number of source planes: "
				<< source.planes().size();
		return false;
	}
	if (destination.numPlanes() != 2 && destination.numPlanes() != 3) {
		LOG(YUV, Error) << "Invalid number of destination planes: "
				<< destination.numPlanes();
		return false;
	}

	if (source.planes()[0].length < sourceLength_[0] ||
	    source.planes()[1].length < sourceLength_[1]) {
		LOG(YUV, Error)
			<< "The source planes lengths are too small, actual size: {"
			<< source.planes()[0].length << ", "
			<< source.planes()[1].length
			<< "}, expected size: {"
			<< sourceLength_[0] << ", "
			<< sourceLength_[1] << "}";
		return false;
	}

	/* Each destination plane must hold its rows at the gralloc stride. */
	const unsigned int chromaWidth = (destinationSize_.width + 1) / 2;
	const unsigned int chromaHeight = (destinationSize_.height + 1) / 2;
	const bool planar = destination.numPlanes() == 3;

	for (unsigned int i = 0; i < destination.numPlanes(); i++) {
		const unsigned int rows = i ? chromaHeight : destinationSize_.height;
		const unsigned int rowBytes = !i ? destinationSize_.width
					    : planar ? chromaWidth : chromaWidth * 2;
		const unsigned int stride = destination.stride(i);
		const size_t needed = static_cast<size_t>(stride) * (rows - 1) + rowBytes;

		if (stride < rowBytes || destination.plane(i).size() < needed) {
			LOG(YUV, Error)
				<< "Destination plane " << i << " too small: stride "
				<< stride << ", size " << destination.plane(i).size()
				<< ", need " << needed;
			return false;
		}
	}

	return true;
}

void PostProcessorYuv::calculateLengths(const StreamConfiguration &inCfg,
					const StreamConfiguration &outCfg)
{
	sourceSize_ = inCfg.size;
	destinationSize_ = outCfg.size;

	const PixelFormatInfo &nv12Info = PixelFormatInfo::info(formats::NV12);
	for (unsigned int i = 0; i < 2; i++) {
		sourceStride_[i] = inCfg.stride;
		sourceLength_[i] = nv12Info.planeSize(sourceSize_.height, i,
						      sourceStride_[i]);
	}

	/* Interleaved chroma at the destination size, for planar outputs. */
	chroma_.resize(static_cast<size_t>((destinationSize_.width + 1) / 2) * 2 *
		       ((destinationSize_.height + 1) / 2));
}
