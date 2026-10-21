/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2025, Bryan O'Donoghue.
 *
 * Authors:
 * Bryan O'Donoghue <bryan.odonoghue@linaro.org>
 *
 */

#pragma once

#include <deque>
#include <memory>
#include <stdint.h>
#include <string>
#include <tuple>
#include <vector>

#define GL_GLEXT_PROTOTYPES
#define EGL_EGLEXT_PROTOTYPES
#include <libcamera/base/object.h>

#include "libcamera/internal/egl.h"
#include "libcamera/internal/mapped_framebuffer.h"
#include "libcamera/internal/software_isp/benchmark.h"
#include "libcamera/internal/software_isp/swstats_cpu.h"

#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GLES3/gl32.h>

#include "debayer.h"

namespace libcamera {

#define DEBAYER_EGL_MIN_SIMPLE_RGB_GAIN_TEXTURE_UNITS 4
#define DEBAYER_OPENGL_COORDS 4

class CameraManager;

class DebayerEGL : public Debayer
{
public:
	DebayerEGL(std::unique_ptr<SwStatsCpu> stats, const CameraManager &cm);
	~DebayerEGL();

	int configure(const StreamConfiguration &inputCfg,
		      const std::vector<std::reference_wrapper<const StreamConfiguration>> &outputCfgs,
		      bool ccmEnabled) override;

	Size patternSize(PixelFormat inputFormat) override;

	std::vector<PixelFormat> formats(PixelFormat input) override;
	std::tuple<unsigned int, unsigned int> strideAndFrameSize(const PixelFormat &outputFormat, const Size &size) override;
	uint32_t preferredInputStride(const PixelFormat &inputFormat, const Size &size) override;

	unsigned int maxOutputs() const override { return kMaxOutputs; }
	unsigned int frameSize(unsigned int output) override;
	void process(uint32_t frame, FrameBuffer *input,
		     const std::vector<FrameBuffer *> &outputs,
		     const DebayerParams &params) override;
	int start() override;
	void stop() override;

	const SharedFD &getStatsFD() override { return stats_->getStatsFD(); }

	SizeRange sizes(PixelFormat inputFormat, const Size &inputSize) override;

private:
	/*
	 * The outputs rendered from one input: a preview and a still capture,
	 * say, from the sensor's full resolution.
	 */
	static constexpr unsigned int kMaxOutputs = 2;
	/*
	 * Outputs of NV12 and NV21 more than this much smaller than the image
	 * are scaled down to it rather than cropped from it.
	 */
	static constexpr float kMaxCropScale = 1.05f;

	struct YuvTargets {
		std::unique_ptr<eGLImage> y;
		std::unique_ptr<eGLImage> uv;
	};

	struct Output {
		PixelFormat format;
		Size size;
		DebayerOutputConfig config;
		unsigned int bufferCount;
		bool yuv;
		/*
		 * The part of the image the output shows. Cropped outputs are
		 * as large as it, scaled ones average quads Bayer quads of it
		 * to a side for a pixel of luma and chromaQuads for one of
		 * chroma.
		 */
		Rectangle window;
		unsigned int quads;
		unsigned int chromaQuads;

		/*
		 * NV12 and NV21 take two passes, each its own program: the
		 * luma plane at the full size, then the chroma plane at half
		 * of it.
		 */
		GLuint program = 0;
		GLuint programY = 0;
		GLuint programUV = 0;

		std::deque<std::pair<SharedFD, std::unique_ptr<eGLImage>>> rgbCache;
		std::deque<std::pair<SharedFD, YuvTargets>> yuvCache;
		/*
		 * The targets of the NV12 and NV21 planes when the output
		 * can't be.
		 */
		std::unique_ptr<eGLImage> yTarget;
		std::unique_ptr<eGLImage> uvTarget;
	};

	static int getInputConfig(PixelFormat inputFormat, DebayerInputConfig &config);
	static int getOutputConfig(PixelFormat outputFormat, DebayerOutputConfig &config);
	int initBayerShaders(PixelFormat inputFormat, PixelFormat outputFormat,
			     const std::vector<std::string> &defines);
	int usePass(GLuint program);
	int getShaderVariableLocations();
	void setShaderVariableValues(eGLImage &eGLImageIn, const Output &out,
				     const DebayerParams &params);
	int render(eGLImage &eglImageIn, FrameBuffer *buffer, Output &out,
		   const DebayerParams &params);
	void deletePrograms(Output &out);

	eGLImage *getCachedInputFrameBuffer(FrameBuffer *input, std::optional<MappedFrameBuffer> *inMapped, std::optional<DmaSyncer> *inDmaSyncer);
	eGLImage *getCachedOutputFrameBuffer(FrameBuffer *buffer, Output &out);
	YuvTargets *getCachedOutputYuvFrameBuffer(FrameBuffer *buffer, Output &out);
	void readPlane(eGLImage &target, const Output &out, uint8_t *dst, unsigned int rows);
	void updateLut(float gamma, float contrastExp);
	int draw(eGLImage &eglImageIn, eGLImage &eglImageOut, const Output &out,
		 const DebayerParams &params, GLsizei width, GLsizei height);

	/* Shader program identifiers */
	GLuint vertexShaderId_ = 0;
	GLuint fragmentShaderId_ = 0;
	/* The program in use */
	GLuint programId_ = 0;

	std::vector<Output> outputs_;

	/* Pointer to object representing input texture */
	std::deque<std::pair<SharedFD, std::unique_ptr<eGLImage>>> eglImageInCache_;
	/* A line copy for widths not a multiple of four */
	std::vector<uint8_t> readback_;

	static constexpr unsigned int kLutSize = 1024;
	static constexpr float kToneRange = 16.0f;
	static constexpr float kToneKnee = 0.6f;
	std::unique_ptr<eGLImage> lut_;
	float lutGamma_;
	float lutContrastExp_;
	unsigned int inputBufferCount_;

	/* Shader parameters */
	float firstRed_x_;
	float firstRed_y_;
	GLint attributeVertex_;
	GLint attributeTexture_;
	GLint textureUniformStep_;
	GLint textureUniformSize_;
	GLint textureUniformStrideFactor_;
	GLint textureUniformBayerFirstRed_;
	GLint textureUniformProjMatrix_;
	GLint textureUniformOutOrigin_;
	GLint textureUniformOutScale_;
	GLint textureUniformLut_;

	GLint textureUniformBayerDataIn_;

	/* Per-frame AWB gains */
	GLint awbUniformDataIn_;

	/* Represent per-frame CCM as a uniform vector of floats 3 x 3 */
	GLint ccmUniformDataIn_;

	/* Black Level compensation */
	GLint blackLevelUniformDataIn_;

	/* Gamma */
	GLint gammaUniformDataIn_;

	/* Contrast */
	GLint contrastExpUniformDataIn_;

	Size nativeOutputSize_;
	std::unique_ptr<SwStatsCpu> stats_;
	eGL egl_;
	uint32_t width_;
	uint32_t height_;
	GLint glFormat_;
	unsigned int bytesPerPixel_;
	uint32_t shaderStridePixels_;
};

} /* namespace libcamera */
