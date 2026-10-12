/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2026, itskenny0
 *
 * Android gralloc frame buffer backend
 */

#include "../camera_buffer.h"

#include <algorithm>
#include <sys/mman.h>
#include <unistd.h>

#include <aidl/android/hardware/graphics/common/PlaneLayoutComponentType.h>
#include <ui/GraphicBufferAllocator.h>
#include <ui/GraphicBufferMapper.h>

#include <libcamera/base/log.h>

#include <libcamera/formats.h>

#include "libcamera/internal/formats.h"
#include "libcamera/internal/mapped_framebuffer.h"

using namespace libcamera;

using aidl::android::hardware::graphics::common::PlaneLayoutComponentType;

LOG_DECLARE_CATEGORY(HAL)

class CameraBuffer::Private : public Extensible::Private,
			      public MappedBuffer
{
	LIBCAMERA_DECLARE_PUBLIC(CameraBuffer)

public:
	Private(CameraBuffer *cameraBuffer, buffer_handle_t camera3Buffer,
		PixelFormat pixelFormat, const Size &size, int flags);
	~Private();

	unsigned int numPlanes() const;

	Span<uint8_t> plane(unsigned int plane);

	int fd(unsigned int plane) const;
	unsigned int stride(unsigned int plane) const;
	unsigned int offset(unsigned int plane) const;
	unsigned int size(unsigned int plane) const;
	bool swapsChroma() const { return swapsChroma_; }

	size_t jpegBufferSize(size_t maxJpegBufferSize) const;

private:
	struct PlaneInfo {
		unsigned int stride;
		unsigned int offset;
		unsigned int size;
	};

	bool queryLayout(buffer_handle_t camera3Buffer, PixelFormat pixelFormat);
	void map();

	int fd_;
	int flags_;
	off_t bufferLength_;
	uint64_t width_;
	bool mapped_;
	bool swapsChroma_;
	std::vector<PlaneInfo> planeInfo_;
};

CameraBuffer::Private::Private([[maybe_unused]] CameraBuffer *cameraBuffer,
			       buffer_handle_t camera3Buffer,
			       PixelFormat pixelFormat,
			       const Size &size, int flags)
	: fd_(-1), flags_(flags), bufferLength_(-1), width_(0), mapped_(false),
	  swapsChroma_(false)
{
	error_ = 0;

	const auto &info = PixelFormatInfo::info(pixelFormat);
	if (!info.isValid()) {
		error_ = -EINVAL;
		LOG(HAL, Error) << "Invalid pixel format: " << pixelFormat;
		return;
	}

	/*
	 * The buffer data is a dma-buf in the handle, not necessarily the
	 * first file descriptor: MediaTek gralloc puts a gralloc_extra object
	 * first and a metadata dma-buf last. Use the largest dma-buf.
	 */
	for (int i = 0; i < camera3Buffer->numFds; ++i) {
		const int fd = camera3Buffer->data[i];
		const off_t length = fd < 0 ? -1 : lseek(fd, 0, SEEK_END);
		if (length > bufferLength_) {
			fd_ = fd;
			bufferLength_ = length;
		}
	}

	if (fd_ < 0) {
		error_ = -EINVAL;
		LOG(HAL, Error) << "No dma-buf in the buffer handle";
		return;
	}

	/* The size of a BLOB buffer, which the JPEG blob trailer ends. */
	if (android::GraphicBufferMapper::get().getWidth(camera3Buffer, &width_) != android::OK)
		width_ = 0;

	if (queryLayout(camera3Buffer, pixelFormat))
		return;

	/*
	 * Fall back to the planes being stored contiguously without padding,
	 * as for BLOB buffers.
	 */
	const unsigned int numPlanes = info.numPlanes();
	planeInfo_.resize(numPlanes);

	unsigned int offset = 0;
	for (unsigned int i = 0; i < numPlanes; ++i) {
		const unsigned int planeSize = info.planeSize(size, i);

		planeInfo_[i].stride = info.stride(size.width, i, 1u);
		planeInfo_[i].offset = offset;
		planeInfo_[i].size = planeSize;

		if (bufferLength_ < offset + planeSize) {
			LOG(HAL, Error) << "Plane " << i << " is out of buffer:"
					<< " plane offset=" << offset
					<< ", plane size=" << planeSize
					<< ", buffer length=" << bufferLength_;
			return;
		}

		offset += planeSize;
	}
}

CameraBuffer::Private::~Private()
{
}

namespace {

/*
 * Return the offset in bits of the first component of \a type of \a layout
 * from the start of the buffer, or -1 if the plane doesn't contain it.
 */
int64_t componentOffset(const android::ui::PlaneLayout &layout,
			PlaneLayoutComponentType type)
{
	for (const auto &component : layout.components) {
		if (component.type.value == static_cast<int64_t>(type))
			return layout.offsetInBytes * 8 + component.offsetInBits;
	}

	return -1;
}

/*
 * Return the size of the plane described by \a layout. MediaTek's mapper
 * under-reports chroma planes, as 81920 bytes for the 240 rows of 640 bytes of
 * a 640x480 NV21 buffer, so cover at least the rows at the stride.
 */
unsigned int planeSize(const android::ui::PlaneLayout &layout)
{
	return static_cast<unsigned int>(std::max(layout.totalSizeInBytes,
						  layout.strideInBytes *
						  layout.heightInSamples));
}

} /* namespace */

/*
 * Retrieve the plane layout from the gralloc mapper. Return false if it isn't
 * available or doesn't match the planes of \a pixelFormat.
 */
bool CameraBuffer::Private::queryLayout(buffer_handle_t camera3Buffer,
					PixelFormat pixelFormat)
{
	std::vector<android::ui::PlaneLayout> layouts;

	android::status_t ret =
		android::GraphicBufferMapper::get().getPlaneLayouts(camera3Buffer, &layouts);
	if (ret != android::OK || layouts.empty())
		return false;

	for (const auto &[i, layout] : utils::enumerate(layouts))
		LOG(HAL, Debug)
			<< "Gralloc plane " << i << ": offset " << layout.offsetInBytes
			<< ", stride " << layout.strideInBytes << ", size "
			<< layout.totalSizeInBytes << ", " << layout.widthInSamples
			<< "x" << layout.heightInSamples << ", increment "
			<< layout.sampleIncrementInBits << " bits, "
			<< layout.components.size() << " components";

	const auto &info = PixelFormatInfo::info(pixelFormat);
	std::vector<PlaneInfo> planes;

	if (layouts.size() == info.numPlanes()) {
		for (const android::ui::PlaneLayout &layout : layouts)
			planes.push_back({
				static_cast<unsigned int>(layout.strideInBytes),
				static_cast<unsigned int>(layout.offsetInBytes),
				planeSize(layout),
			});
	} else if (layouts.size() == 3 && info.numPlanes() == 2 &&
		   layouts[1].sampleIncrementInBits == 8 &&
		   layouts[2].sampleIncrementInBits == 8) {
		/*
		 * Planar YUV, which gralloc can pick for an implementation
		 * defined format (MediaTek does for GPU texture usage). Report
		 * the Y, Cb and Cr planes in that order; the YUV post-processor
		 * writes them.
		 */
		const bool crFirst =
			componentOffset(layouts[1], PlaneLayoutComponentType::CR) >= 0;

		for (unsigned int i : { 0u, crFirst ? 2u : 1u, crFirst ? 1u : 2u })
			planes.push_back({
				static_cast<unsigned int>(layouts[i].strideInBytes),
				static_cast<unsigned int>(layouts[i].offsetInBytes),
				planeSize(layouts[i]),
			});
	} else if (layouts.size() == 3 && info.numPlanes() == 2) {
		/*
		 * Semi-planar YUV described with separate, interleaved Cb and
		 * Cr planes.
		 */
		const android::ui::PlaneLayout &cb = layouts[1];
		const android::ui::PlaneLayout &cr = layouts[2];
		const int64_t first = std::min(cb.offsetInBytes, cr.offsetInBytes);
		const int64_t last = std::max(cb.offsetInBytes, cr.offsetInBytes);

		if (cb.sampleIncrementInBits != 16 || cr.sampleIncrementInBits != 16 ||
		    last - first != 1)
			return false;

		planes.push_back({
			static_cast<unsigned int>(layouts[0].strideInBytes),
			static_cast<unsigned int>(layouts[0].offsetInBytes),
			static_cast<unsigned int>(layouts[0].totalSizeInBytes),
		});
		planes.push_back({
			static_cast<unsigned int>(cb.strideInBytes),
			static_cast<unsigned int>(first),
			std::max(planeSize(cb), planeSize(cr)),
		});
	} else {
		return false;
	}

	for (const auto &[i, plane] : utils::enumerate(planes)) {
		if (bufferLength_ < plane.offset + plane.size) {
			LOG(HAL, Error) << "Gralloc plane " << i << " is out of buffer:"
					<< " plane offset=" << plane.offset
					<< ", plane size=" << plane.size
					<< ", buffer length=" << bufferLength_;
			return false;
		}
	}

	/*
	 * Note a semi-planar chroma order opposite to the pixel format's, as
	 * NV21 for an NV12 stream. The YUV post-processor swaps it.
	 */
	if (planes.size() == 2 &&
	    (pixelFormat == formats::NV12 || pixelFormat == formats::NV21)) {
		int64_t cb = -1;
		int64_t cr = -1;

		for (const android::ui::PlaneLayout &layout : layouts) {
			cb = std::max(cb, componentOffset(layout, PlaneLayoutComponentType::CB));
			cr = std::max(cr, componentOffset(layout, PlaneLayoutComponentType::CR));
		}

		swapsChroma_ = cb >= 0 && cr >= 0 &&
			       (cr < cb) != (pixelFormat == formats::NV21);
	}

	planeInfo_ = std::move(planes);

	return true;
}

unsigned int CameraBuffer::Private::numPlanes() const
{
	return planeInfo_.size();
}

Span<uint8_t> CameraBuffer::Private::plane(unsigned int plane)
{
	if (!mapped_)
		map();
	if (!mapped_)
		return {};

	return planes_[plane];
}

int CameraBuffer::Private::fd([[maybe_unused]] unsigned int plane) const
{
	return fd_;
}

unsigned int CameraBuffer::Private::stride(unsigned int plane) const
{
	if (plane >= planeInfo_.size())
		return 0;

	return planeInfo_[plane].stride;
}

unsigned int CameraBuffer::Private::offset(unsigned int plane) const
{
	if (plane >= planeInfo_.size())
		return 0;

	return planeInfo_[plane].offset;
}

unsigned int CameraBuffer::Private::size(unsigned int plane) const
{
	if (plane >= planeInfo_.size())
		return 0;

	return planeInfo_[plane].size;
}

size_t CameraBuffer::Private::jpegBufferSize(size_t maxJpegBufferSize) const
{
	ASSERT(bufferLength_ >= 0);

	/*
	 * The framework looks for the blob trailer at the end of the gralloc
	 * width, which it sizes itself and which can exceed the maximum JPEG
	 * size we report.
	 */
	if (width_)
		return std::min<uint64_t>(bufferLength_, width_);

	return std::min<unsigned int>(bufferLength_, maxJpegBufferSize);
}

void CameraBuffer::Private::map()
{
	ASSERT(fd_ != -1);
	ASSERT(bufferLength_ >= 0);

	void *address = mmap(nullptr, bufferLength_, flags_, MAP_SHARED, fd_, 0);
	if (address == MAP_FAILED) {
		error_ = -errno;
		LOG(HAL, Error) << "Failed to mmap plane";
		return;
	}
	maps_.emplace_back(static_cast<uint8_t *>(address), bufferLength_);

	planes_.reserve(planeInfo_.size());
	for (const auto &info : planeInfo_) {
		planes_.emplace_back(
			static_cast<uint8_t *>(address) + info.offset, info.size);
	}

	mapped_ = true;
}

PUBLIC_CAMERA_BUFFER_IMPLEMENTATION

/*
 * Tell whether gralloc lays out the buffers of \a stream the way libcamera
 * fills \a pixelFormat buffers of \a size, with the same planes, strides and
 * chroma order.
 * For implementation defined formats gralloc picks the layout from the usage,
 * so allocate a buffer the way the camera service will and look at it.
 */
bool CameraBuffer::matchesLayout(const camera3_stream_t *stream,
				 PixelFormat pixelFormat, const Size &size)
{
	android::GraphicBufferAllocator &allocator =
		android::GraphicBufferAllocator::get();
	buffer_handle_t handle = nullptr;
	uint32_t stride;

	android::status_t ret =
		allocator.allocate(stream->width, stream->height, stream->format,
				   1, stream->usage, &handle, &stride,
				   "libcamera layout probe");
	if (ret != android::OK) {
		LOG(HAL, Warning)
			<< "Can't allocate a " << stream->width << "x"
			<< stream->height << " buffer to check its layout";
		return true;
	}

	bool matches;
	{
		const CameraBuffer buffer(handle, pixelFormat, size, PROT_READ);
		const PixelFormatInfo &info = PixelFormatInfo::info(pixelFormat);

		matches = buffer.isValid() && !buffer.swapsChroma() &&
			  buffer.numPlanes() == info.numPlanes();
		for (unsigned int i = 0; matches && i < buffer.numPlanes(); ++i)
			matches = buffer.stride(i) == info.stride(size.width, i, 1);
	}

	allocator.free(handle);

	return matches;
}
