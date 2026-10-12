/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2026, itskenny0
 *
 * Allocate FrameBuffer from DMA heaps
 */

#include <memory>
#include <vector>

#include <libcamera/base/log.h>
#include <libcamera/base/shared_fd.h>
#include <libcamera/base/unique_fd.h>
#include <libcamera/base/utils.h>

#include "libcamera/internal/dma_buf_allocator.h"
#include "libcamera/internal/formats.h"

#include "../camera_capabilities.h"
#include "../camera_device.h"
#include "../frame_buffer_allocator.h"
#include "../hal_framebuffer.h"

using namespace libcamera;

LOG_DECLARE_CATEGORY(HAL)

/*
 * The internal buffers are only accessed by libcamera and the HAL, allocate
 * them from DMA heaps with the contiguous plane layout libcamera expects
 * instead of going through gralloc.
 */
class PlatformFrameBufferAllocator::Private : public Extensible::Private
{
	LIBCAMERA_DECLARE_PUBLIC(PlatformFrameBufferAllocator)

public:
	Private(CameraDevice *const cameraDevice)
		: cameraDevice_(cameraDevice),
		  allocator_(DmaBufAllocator::DmaBufAllocatorFlag::CmaHeap |
			     DmaBufAllocator::DmaBufAllocatorFlag::SystemHeap |
			     DmaBufAllocator::DmaBufAllocatorFlag::UDmaBuf)
	{
	}

	std::unique_ptr<HALFrameBuffer>
	allocate(int halPixelFormat, const libcamera::Size &size, uint32_t usage);

private:
	const CameraDevice *const cameraDevice_;
	DmaBufAllocator allocator_;
};

std::unique_ptr<HALFrameBuffer>
PlatformFrameBufferAllocator::Private::allocate(int halPixelFormat,
						const libcamera::Size &size,
						[[maybe_unused]] uint32_t usage)
{
	if (!allocator_.isValid()) {
		LOG(HAL, Error) << "No DMA heap available";
		return nullptr;
	}

	const PixelFormat pixelFormat =
		cameraDevice_->capabilities()->toPixelFormat(halPixelFormat);
	const PixelFormatInfo &info = PixelFormatInfo::info(pixelFormat);
	if (!info.isValid()) {
		LOG(HAL, Error) << "Invalid pixel format: " << pixelFormat;
		return nullptr;
	}

	UniqueFD fd = allocator_.alloc("libcamera-hal", info.frameSize(size));
	if (!fd.isValid()) {
		LOG(HAL, Error) << "Failed to allocate a " << size << " "
				<< pixelFormat << " buffer";
		return nullptr;
	}

	/* Store the planes contiguously, as libcamera computes their sizes. */
	SharedFD sharedFd(std::move(fd));
	std::vector<FrameBuffer::Plane> planes(info.numPlanes());
	unsigned int offset = 0;
	for (auto [i, plane] : utils::enumerate(planes)) {
		plane.fd = sharedFd;
		plane.offset = offset;
		plane.length = info.planeSize(size, i);
		offset += plane.length;
	}

	return std::make_unique<HALFrameBuffer>(planes, nullptr);
}

PUBLIC_FRAME_BUFFER_ALLOCATOR_IMPLEMENTATION
