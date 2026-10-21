/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Linaro Ltd.
 *
 * Authors:
 * Bryan O'Donoghue <bryan.odonoghue@linaro.org>
 *
 */

#include "debayer_egl.h"

#include <algorithm>
#include <array>
#include <assert.h>
#include <cmath>
#include <memory>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <tuple>
#include <vector>

#include <linux/drm_fourcc.h>

#include <libcamera/base/span.h>
#include <libcamera/base/utils.h>

#include <libcamera/formats.h>

#include "libcamera/internal/formats.h"
#include "libcamera/internal/framebuffer.h"

#include "../shaders/glsl_shaders.h"

namespace libcamera {

/**
 * \class DebayerEGL
 * \brief Class for debayering using an EGL Shader
 *
 * Implements an EGL shader based debayering solution.
 */

/**
 * \brief Construct a DebayerEGL object
 * \param[in] stats Statistics processing object
 * \param[in] cm The camera manager
 */
DebayerEGL::DebayerEGL(std::unique_ptr<SwStatsCpu> stats, const CameraManager &cm)
	: Debayer(cm), stats_(std::move(stats))
{
}

DebayerEGL::~DebayerEGL()
{
}

int DebayerEGL::getInputConfig(PixelFormat inputFormat, DebayerInputConfig &config)
{
	BayerFormat bayerFormat =
		BayerFormat::fromPixelFormat(inputFormat);

	std::vector<PixelFormat> outputFormats = { formats::XRGB8888,
						   formats::ARGB8888,
						   formats::XBGR8888,
						   formats::ABGR8888 };

	if ((bayerFormat.bitDepth == 8 || bayerFormat.bitDepth == 10) &&
	    bayerFormat.packing == BayerFormat::Packing::None &&
	    isStandardBayerOrder(bayerFormat.order)) {
		config.bpp = (bayerFormat.bitDepth + 7) & ~7;
		config.patternSize.width = 2;
		config.patternSize.height = 2;
		config.outputFormats = outputFormats;
		return 0;
	}

	/* The shaders of packed formats also render the planes of NV12 and NV21 */
	outputFormats.push_back(formats::NV12);
	outputFormats.push_back(formats::NV21);

	if (bayerFormat.bitDepth == 10 &&
	    bayerFormat.packing == BayerFormat::Packing::CSI2 &&
	    isStandardBayerOrder(bayerFormat.order)) {
		config.bpp = 10;
		config.patternSize.width = 4; /* 5 bytes per *4* pixels */
		config.patternSize.height = 2;
		config.outputFormats = outputFormats;
		return 0;
	}

	if (bayerFormat.bitDepth == 10 &&
	    bayerFormat.packing == BayerFormat::Packing::MTISP &&
	    isStandardBayerOrder(bayerFormat.order)) {
		config.bpp = 10;
		config.patternSize.width = 4; /* 5 bytes per *4* pixels */
		config.patternSize.height = 2;
		config.outputFormats = outputFormats;
		return 0;
	}

	if (bayerFormat.bitDepth == 12 &&
	    bayerFormat.packing == BayerFormat::Packing::CSI2 &&
	    isStandardBayerOrder(bayerFormat.order)) {
		config.bpp = 12;
		config.patternSize.width = 2; /* 3 bytes per *2* pixels */
		config.patternSize.height = 2;
		config.outputFormats = outputFormats;
		return 0;
	}

	LOG(Debayer, Info)
		<< "Unsupported input format " << inputFormat;

	return -EINVAL;
}

int DebayerEGL::getOutputConfig(PixelFormat outputFormat, DebayerOutputConfig &config)
{
	if (outputFormat == formats::XRGB8888 || outputFormat == formats::ARGB8888 ||
	    outputFormat == formats::XBGR8888 || outputFormat == formats::ABGR8888) {
		config.bpp = 32;
		return 0;
	}

	if (outputFormat == formats::NV12 || outputFormat == formats::NV21) {
		config.bpp = 8;
		return 0;
	}

	LOG(Debayer, Info)
		<< "Unsupported output format " << outputFormat;

	return -EINVAL;
}

int DebayerEGL::getShaderVariableLocations(void)
{
	attributeVertex_ = glGetAttribLocation(programId_, "vertexIn");
	attributeTexture_ = glGetAttribLocation(programId_, "textureIn");

	textureUniformBayerDataIn_ = glGetUniformLocation(programId_, "tex_y");
	awbUniformDataIn_ = glGetUniformLocation(programId_, "awb");
	ccmUniformDataIn_ = glGetUniformLocation(programId_, "ccm");
	blackLevelUniformDataIn_ = glGetUniformLocation(programId_, "blacklevel");
	gammaUniformDataIn_ = glGetUniformLocation(programId_, "gamma");
	contrastExpUniformDataIn_ = glGetUniformLocation(programId_, "contrastExp");

	textureUniformStep_ = glGetUniformLocation(programId_, "tex_step");
	textureUniformSize_ = glGetUniformLocation(programId_, "tex_size");
	textureUniformStrideFactor_ = glGetUniformLocation(programId_, "stride_factor");
	textureUniformBayerFirstRed_ = glGetUniformLocation(programId_, "tex_bayer_first_red");
	textureUniformProjMatrix_ = glGetUniformLocation(programId_, "proj_matrix");
	textureUniformOutOrigin_ = glGetUniformLocation(programId_, "out_origin");
	textureUniformLut_ = glGetUniformLocation(programId_, "tex_lut");

	LOG(Debayer, Debug) << "vertexIn " << attributeVertex_ << " textureIn " << attributeTexture_
			    << " tex_y " << textureUniformBayerDataIn_
			    << " awb " << awbUniformDataIn_
			    << " ccm " << ccmUniformDataIn_
			    << " blacklevel " << blackLevelUniformDataIn_
			    << " gamma " << gammaUniformDataIn_
			    << " contrastExp " << contrastExpUniformDataIn_
			    << " tex_step " << textureUniformStep_
			    << " tex_size " << textureUniformSize_
			    << " stride_factor " << textureUniformStrideFactor_
			    << " tex_bayer_first_red " << textureUniformBayerFirstRed_
			    << " proj_matrix " << textureUniformProjMatrix_;
	return 0;
}

/*
 * Builds the program of a pass into programId_: \a pass is the define of the
 * NV12/NV21 plane it renders, null for an RGB output.
 */
int DebayerEGL::initBayerShaders(PixelFormat inputFormat, PixelFormat outputFormat,
				 const char *pass)
{
	std::vector<std::string> shaderEnv;
	Span<const unsigned char> fragmentShaderData;
	Span<const unsigned char> vertexShaderData;
	GLenum err;

	/* Target gles 100 glsl requires "#version x" as first directive in shader */
	egl_.pushEnv(shaderEnv, "#version 100");

	/* Specify GL_OES_EGL_image_external */
	egl_.pushEnv(shaderEnv, "#extension GL_OES_EGL_image_external: enable");

	/*
	 * Tell shaders how to re-order output taking account of how the pixels
	 * are actually stored by EGL.
	 */
	switch (outputFormat) {
	case formats::ARGB8888:
	case formats::XRGB8888:
		break;
	case formats::ABGR8888:
	case formats::XBGR8888:
		egl_.pushEnv(shaderEnv, "#define SWAP_BLUE");
		break;
	case formats::NV12:
		break;
	case formats::NV21:
		egl_.pushEnv(shaderEnv, "#define SWAP_UV");
		break;
	default:
		LOG(Debayer, Error) << "Unsupported output format";
		return -EINVAL;
	}
	if (pass)
		egl_.pushEnv(shaderEnv, pass);

	/* Pixel location parameters */
	glFormat_ = GL_LUMINANCE;
	bytesPerPixel_ = 1;
	shaderStridePixels_ = inputConfig_.stride;

	switch (inputFormat) {
	case libcamera::formats::SBGGR8:
	case libcamera::formats::SBGGR10_CSI2P:
	case libcamera::formats::SBGGR12_CSI2P:
		firstRed_x_ = 1.0;
		firstRed_y_ = 1.0;
		break;
	case libcamera::formats::SGBRG8:
	case libcamera::formats::SGBRG10_CSI2P:
	case libcamera::formats::SGBRG12_CSI2P:
		firstRed_x_ = 0.0;
		firstRed_y_ = 1.0;
		break;
	case libcamera::formats::SGRBG8:
	case libcamera::formats::SGRBG10_CSI2P:
	case libcamera::formats::SGRBG12_CSI2P:
		firstRed_x_ = 1.0;
		firstRed_y_ = 0.0;
		break;
	case libcamera::formats::SRGGB8:
	case libcamera::formats::SRGGB10_CSI2P:
	case libcamera::formats::SRGGB12_CSI2P:
		firstRed_x_ = 0.0;
		firstRed_y_ = 0.0;
		break;
	default:
		LOG(Debayer, Error) << "Unsupported input format";
		return -EINVAL;
	};

	/* Shader selection */
	switch (inputFormat) {
	case libcamera::formats::SBGGR8:
	case libcamera::formats::SGBRG8:
	case libcamera::formats::SGRBG8:
	case libcamera::formats::SRGGB8:
		fragmentShaderData = bayer_unpacked_frag;
		vertexShaderData = bayer_unpacked_vert;
		break;
	case libcamera::formats::SBGGR10_CSI2P:
	case libcamera::formats::SGBRG10_CSI2P:
	case libcamera::formats::SGRBG10_CSI2P:
	case libcamera::formats::SRGGB10_CSI2P:
		if (BayerFormat::fromPixelFormat(inputFormat).packing == BayerFormat::Packing::MTISP) {
			/*
			 * The fourcc of CSI-2 packing with the MediaTek modifier:
			 * its own shader puts each sample together from two bytes,
			 * four bytes to a texel for fewer fetches and uploads.
			 */
			if (inputConfig_.stride % 4) {
				LOG(Debayer, Error) << "Stride " << inputConfig_.stride
						    << " not a multiple of 4";
				return -EINVAL;
			}
			fragmentShaderData = bayer_mtisp10_frag;
			vertexShaderData = identity_vert;
			glFormat_ = GL_RGBA;
			bytesPerPixel_ = 4;
			/* textureOut spans the width in pixels */
			shaderStridePixels_ = width_ * bytesPerPixel_;
			break;
		}
		egl_.pushEnv(shaderEnv, "#define RAW10P");
		if (BayerFormat::fromPixelFormat(inputFormat).packing == BayerFormat::Packing::None) {
			fragmentShaderData = bayer_unpacked_frag;
			vertexShaderData = bayer_unpacked_vert;
			glFormat_ = GL_RG;
			bytesPerPixel_ = 2;
		} else {
			fragmentShaderData = bayer_1x_packed_frag;
			vertexShaderData = identity_vert;
			shaderStridePixels_ = width_;
		}
		break;
	case libcamera::formats::SBGGR12_CSI2P:
	case libcamera::formats::SGBRG12_CSI2P:
	case libcamera::formats::SGRBG12_CSI2P:
	case libcamera::formats::SRGGB12_CSI2P:
		egl_.pushEnv(shaderEnv, "#define RAW12P");
		if (BayerFormat::fromPixelFormat(inputFormat).packing == BayerFormat::Packing::None) {
			fragmentShaderData = bayer_unpacked_frag;
			vertexShaderData = bayer_unpacked_vert;
			glFormat_ = GL_RG;
			bytesPerPixel_ = 2;
		} else {
			fragmentShaderData = bayer_1x_packed_frag;
			vertexShaderData = identity_vert;
			shaderStridePixels_ = width_;
		}
		break;
	};

	if (egl_.compileVertexShader(vertexShaderId_, vertexShaderData,
				     shaderEnv)) {
		LOG(Debayer, Error) << "Compile vertex shader fail";
		return -ENODEV;
	}
	utils::scope_exit vShaderGuard([&] { glDeleteShader(vertexShaderId_); });

	if (egl_.compileFragmentShader(fragmentShaderId_, fragmentShaderData,
				       shaderEnv)) {
		LOG(Debayer, Error) << "Compile fragment shader fail";
		return -ENODEV;
	}
	utils::scope_exit fShaderGuard([&] { glDeleteShader(fragmentShaderId_); });

	if (egl_.linkProgram(programId_, vertexShaderId_, fragmentShaderId_)) {
		LOG(Debayer, Error) << "Linking program fail";
		return -ENODEV;
	}

	egl_.dumpShaderSource(vertexShaderId_);
	egl_.dumpShaderSource(fragmentShaderId_);

	/* Ensure we set the programId_ */
	egl_.useProgram(programId_);
	err = glGetError();
	if (err != GL_NO_ERROR) {
		LOG(Debayer, Error) << "Use program error " << err;
		return -ENODEV;
	}

	return getShaderVariableLocations();
}

int DebayerEGL::configure(const StreamConfiguration &inputCfg,
			  const std::vector<std::reference_wrapper<const StreamConfiguration>> &outputCfgs,
			  [[maybe_unused]] bool ccmEnabled)
{
	if (getInputConfig(inputCfg.pixelFormat, inputConfig_) != 0)
		return -EINVAL;

	if (stats_->configure(inputCfg) != 0)
		return -EINVAL;

	const Size &stats_pattern_size = stats_->patternSize();
	if (inputConfig_.patternSize.width != stats_pattern_size.width ||
	    inputConfig_.patternSize.height != stats_pattern_size.height) {
		LOG(Debayer, Error)
			<< "mismatching stats and debayer pattern sizes for "
			<< inputCfg.pixelFormat;
		return -EINVAL;
	}

	inputConfig_.stride = inputCfg.stride;
	inputPixelFormat_ = inputCfg.pixelFormat;
	width_ = inputCfg.size.width;
	height_ = inputCfg.size.height;

	if (outputCfgs.size() != 1) {
		LOG(Debayer, Error)
			<< "Unsupported number of output streams: "
			<< outputCfgs.size();
		return -EINVAL;
	}

	const StreamConfiguration &outputCfg = outputCfgs[0];
	SizeRange outSizeRange = sizes(inputCfg.pixelFormat, inputCfg.size);
	std::tie(outputConfig_.stride, outputConfig_.frameSize) =
		strideAndFrameSize(outputCfg.pixelFormat, outputCfg.size);

	if (!outSizeRange.contains(outputCfg.size) || outputConfig_.stride != outputCfg.stride) {
		LOG(Debayer, Error)
			<< "Invalid output size/stride: "
			<< "\n  " << outputCfg.size << " (" << outSizeRange << ")"
			<< "\n  " << outputCfg.stride << " (" << outputConfig_.stride << ")";
		return -EINVAL;
	}

	outputPixelFormat_ = outputCfg.pixelFormat;
	outputSize_ = outputCfg.size;
	nativeOutputSize_ = outSizeRange.max;

	window_.x = ((inputCfg.size.width - outputCfg.size.width) / 2) &
		    ~(inputConfig_.patternSize.width - 1);
	window_.y = ((inputCfg.size.height - outputCfg.size.height) / 2) &
		    ~(inputConfig_.patternSize.height - 1);
	window_.width = outputCfg.size.width;
	window_.height = outputCfg.size.height;

	/*
	 * Don't pass x,y from window_ since process() already adjusts for it.
	 * But crop the window to 2/3 of its width and height for speedup.
	 */
	stats_->setWindow(Rectangle(window_.size()));

	inputBufferCount_ = inputCfg.bufferCount;
	outputBufferCount_ = outputCfg.bufferCount;

	return 0;
}

Size DebayerEGL::patternSize(PixelFormat inputFormat)
{
	DebayerEGL::DebayerInputConfig config;

	if (getInputConfig(inputFormat, config) != 0)
		return {};

	return config.patternSize;
}

std::vector<PixelFormat> DebayerEGL::formats(PixelFormat inputFormat)
{
	DebayerEGL::DebayerInputConfig config;

	if (getInputConfig(inputFormat, config) != 0)
		return std::vector<PixelFormat>();

	return config.outputFormats;
}

std::tuple<unsigned int, unsigned int>
DebayerEGL::strideAndFrameSize(const PixelFormat &outputFormat, const Size &size)
{
	DebayerEGL::DebayerOutputConfig config;

	if (getOutputConfig(outputFormat, config) != 0)
		return std::make_tuple(0, 0);

	/* The chroma plane follows the luma plane, with the same stride */
	if (outputFormat == formats::NV12 || outputFormat == formats::NV21)
		return std::make_tuple(size.width, size.width * size.height * 3 / 2);

	/* Align stride to 256 bytes as a generic GPU memory access alignment */
	unsigned int stride = libcamera::utils::alignUp(size.width * config.bpp / 8, 256);

	return std::make_tuple(stride, stride * size.height);
}

uint32_t DebayerEGL::preferredInputStride(const PixelFormat &inputFormat, const Size &size)
{
	const PixelFormatInfo &info = PixelFormatInfo::info(inputFormat);
	return info.stride(size.width, 0, 256);
}

void DebayerEGL::setShaderVariableValues(eGLImage &eglImageIn, const DebayerParams &params)
{
	/*
	 * Raw Bayer 8-bit, and packed raw Bayer 10-bit/12-bit formats
	 * are stored in a GL_LUMINANCE texture. The texture width is
	 * equal to the stride.
	 */
	GLfloat firstRed[] = { firstRed_x_, firstRed_y_ };
	GLfloat imgSize[] = { (GLfloat)width_,
			      (GLfloat)height_ };
	GLfloat Step[] = { static_cast<float>(bytesPerPixel_) / (inputConfig_.stride - 1),
			   1.0f / (height_ - 1) };
	GLfloat Stride = (GLfloat)width_ / (shaderStridePixels_ / bytesPerPixel_);
	/*
	 * Scale the output size from the native size the algorithm produces for
	 * the input size. Keep the aspect ratio and prefer cropping over black
	 * bars.
	 */
	GLfloat scale = std::max((GLfloat)outputSize_.width / nativeOutputSize_.width,
				 (GLfloat)outputSize_.height / nativeOutputSize_.height);
	GLfloat trans = -(1.0f - scale);
	GLfloat projMatrix[] = {
		scale, 0, 0, 0,
		0, scale, 0, 0,
		0, 0, 1, 0,
		trans, trans, 0, 1
	};
	/*
	 * The planes of NV12 and NV21 cover their whole targets and find their
	 * pixels themselves, cropped from the image as the CPU debayer does it
	 * rather than scaled: sharper, and each fragment of a packed format
	 * gets the samples of whole groups.
	 */
	GLfloat outOrigin[] = { (GLfloat)window_.x, (GLfloat)window_.y };
	if (yuvOutput_) {
		projMatrix[0] = projMatrix[5] = 1;
		projMatrix[12] = projMatrix[13] = 0;
	}
	/* Static const coordinates */
	static const GLfloat vcoordinates[4][2] = {
		{ -1.0f, -1.0f },
		{ -1.0f, +1.0f },
		{ +1.0f, +1.0f },
		{ +1.0f, -1.0f },
	};
	static const GLfloat tcoordinates[4][2] = {
		{ 0.0f, 0.0f },
		{ 0.0f, 1.0f },
		{ 1.0f, 1.0f },
		{ 1.0f, 0.0f },
	};

	/* vertexIn - bayer_8.vert */
	glEnableVertexAttribArray(attributeVertex_);
	glVertexAttribPointer(attributeVertex_, 2, GL_FLOAT, GL_TRUE,
			      2 * sizeof(GLfloat), vcoordinates);

	/* textureIn - bayer_8.vert */
	glEnableVertexAttribArray(attributeTexture_);
	glVertexAttribPointer(attributeTexture_, 2, GL_FLOAT, GL_TRUE,
			      2 * sizeof(GLfloat), tcoordinates);

	/*
	 * Set the sampler2D to the respective texture unit for each texutre
	 * To simultaneously sample multiple textures we need to use multiple
	 * texture units
	 */
	glUniform1i(textureUniformBayerDataIn_, eglImageIn.texture_unit_uniform_id_);

	/*
	 * These values are:
	 * firstRed = tex_bayer_first_red - bayer_8.vert
	 * imgSize = tex_size - bayer_8.vert
	 * step = tex_step - bayer_8.vert
	 * Stride = stride_factor identity.vert
	 * textureUniformProjMatri = No scaling
	 */
	glUniform2fv(textureUniformBayerFirstRed_, 1, firstRed);
	glUniform2fv(textureUniformSize_, 1, imgSize);
	glUniform2fv(textureUniformStep_, 1, Step);
	glUniform1f(textureUniformStrideFactor_, Stride);
	glUniformMatrix4fv(textureUniformProjMatrix_, 1, GL_FALSE, projMatrix);
	glUniform2fv(textureUniformOutOrigin_, 1, outOrigin);

	if (textureUniformLut_ >= 0) {
		updateLut(params.gamma, params.contrastExp);
		egl_.activateBindTexture(*lut_);
		glUniform1i(textureUniformLut_, lut_->texture_unit_uniform_id_);
	}

	LOG(Debayer, Debug) << "vertexIn " << attributeVertex_ << " textureIn " << attributeTexture_
			    << " tex_y " << textureUniformBayerDataIn_
			    << " tex_step " << textureUniformStep_
			    << " tex_size " << textureUniformSize_
			    << " stride_factor " << textureUniformStrideFactor_
			    << " tex_bayer_first_red " << textureUniformBayerFirstRed_;

	LOG(Debayer, Debug) << "textureUniformY_ = 0 "
			    << " firstRed.x " << firstRed[0]
			    << " firstRed.y " << firstRed[1]
			    << " textureUniformSize_.width " << imgSize[0]
			    << " textureUniformSize_.height " << imgSize[1]
			    << " textureUniformStep_.x " << Step[0]
			    << " textureUniformStep_.y " << Step[1]
			    << " textureUniformStrideFactor_ " << Stride
			    << " textureUniformProjMatrix_ " << textureUniformProjMatrix_;

	glUniformMatrix3fv(ccmUniformDataIn_, 1, GL_TRUE, params.combinedMatrix.data().data());
	LOG(Debayer, Debug) << " ccmUniformDataIn_ " << ccmUniformDataIn_ << " data " << params.combinedMatrix;

	/*
	 * 0 = Red, 1 = Green, 2 = Blue
	 */
	glUniform3f(blackLevelUniformDataIn_, params.blackLevel[0], params.blackLevel[1], params.blackLevel[2]);
	LOG(Debayer, Debug) << " blackLevelUniformDataIn_ " << blackLevelUniformDataIn_ << " data " << params.blackLevel;

	glUniform3f(awbUniformDataIn_, params.gains[0], params.gains[1], params.gains[2]);
	LOG(Debayer, Debug) << " awbUniformDataIn_ " << awbUniformDataIn_ << " data " << params.gains;

	/*
	 * Gamma
	 */
	glUniform1f(gammaUniformDataIn_, params.gamma);
	LOG(Debayer, Debug) << " gammaUniformDataIn_ " << gammaUniformDataIn_ << " data " << params.gamma;

	/*
	 * Contrast
	 */
	glUniform1f(contrastExpUniformDataIn_, params.contrastExp);
	LOG(Debayer, Debug) << " contrastExpUniformDataIn_ " << contrastExpUniformDataIn_ << " data " << params.contrastExp;

	return;
}

/*
 * The contrast curve and the gamma of shaders that take them from a table
 * rather than computing them for every pixel: 1024 steps, interpolated.
 */
void DebayerEGL::updateLut(float gamma, float contrastExp)
{
	if (lut_ && gamma == lutGamma_ && contrastExp == lutContrastExp_)
		return;

	std::array<uint8_t, kLutSize> table;
	for (unsigned int i = 0; i < kLutSize; i++) {
		float v = static_cast<float>(i) / (kLutSize - 1);
		v = v < 0.5f ? 0.5f * std::pow(v / 0.5f, contrastExp)
			     : 1.0f - 0.5f * std::pow((1.0f - v) / 0.5f, contrastExp);
		table[i] = std::clamp(std::lround(std::pow(v, gamma) * 255.0f), 0L, 255L);
	}

	if (!lut_) {
		lut_ = std::make_unique<eGLImage>(GL_LUMINANCE, kLutSize, 1, kLutSize,
						  GL_TEXTURE2, 2);
		egl_.createTexture2D(*lut_, table.data());
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	} else {
		egl_.updateTexture2D(*lut_, table.data());
	}

	lutGamma_ = gamma;
	lutContrastExp_ = contrastExp;
}

eGLImage *DebayerEGL::getCachedInputFrameBuffer(FrameBuffer *input, std::optional<MappedFrameBuffer> *inMapped, std::optional<DmaSyncer> *inDmaSyncer)
{
	const SharedFD &fd = input->planes()[0].fd;
	bool cache_miss = true;
	eGLImage *eglImageIn;

	for (auto &[ifd, img] : eglImageInCache_) {
		if (ifd == fd) {
			eglImageIn = img.get();
			cache_miss = false;
			break;
		}
	}

	if (cache_miss) {
		if (eglImageInCache_.size() >= inputBufferCount_) {
			eglImageInCache_.pop_front();
			LOG(Debayer, Debug) << "Input cache " << inputBufferCount_ << " exceeded evicted entry";
		}

		eglImageInCache_.emplace_back(fd, std::make_unique<eGLImage>(glFormat_, inputConfig_.stride / bytesPerPixel_, height_, inputConfig_.stride, GL_TEXTURE0, 0));
		eglImageIn = eglImageInCache_.back().second.get();
		/* Raw bytes in texels keep their order: r, g, b, a */
		if (glFormat_ == GL_RGBA)
			eglImageIn->drmFormat_ = DRM_FORMAT_ABGR8888;

		if (egl_.createInputDMABufTexture2D(*eglImageIn, input->planes()[0].fd.get()) == 0)
			return eglImageIn;

	} else if (!eglImageIn->dmabuf_import_failed_) {
		egl_.activateBindTexture(*eglImageIn);
		return eglImageIn;
	}

	/* DMA mode fail create/update an existing texture using the slow path */
	inDmaSyncer->emplace(input->planes()[0].fd, DmaSyncer::SyncType::Read);
	inMapped->emplace(input, MappedFrameBuffer::MapFlag::Read);
	if (!inMapped->value().isValid()) {
		LOG(Debayer, Error) << "mmap-ing buffer(s) failed";
		if (cache_miss) {
			eglImageInCache_.pop_back();
		}
		return nullptr;
	}
	if (cache_miss)
		egl_.createTexture2D(*eglImageIn, inMapped->value().planes()[0].data());
	else
		egl_.updateTexture2D(*eglImageIn, inMapped->value().planes()[0].data());

	return eglImageIn;
}

eGLImage *DebayerEGL::getCachedOutputFrameBuffer(FrameBuffer *output)
{
	const SharedFD &fd = output->planes()[0].fd;
	bool cache_miss = true;
	eGLImage *eglImageOut;

	for (auto &[ifd, img] : eglImageOutCache_) {
		if (ifd == fd) {
			eglImageOut = img.get();
			cache_miss = false;
			break;
		}
	}

	if (cache_miss) {
		if (eglImageOutCache_.size() >= outputBufferCount_) {
			eglImageOutCache_.pop_front();
			LOG(Debayer, Debug) << "Output cache " << outputBufferCount_ << " exceeded evicted entry";
		}

		eglImageOutCache_.emplace_back(fd, std::make_unique<eGLImage>(GL_RGBA, outputSize_.width,
									      outputSize_.height, outputConfig_.stride, GL_TEXTURE1, 1));
		eglImageOut = eglImageOutCache_.back().second.get();

		if (egl_.createOutputDMABufTexture2D(*eglImageOut, output->planes()[0].fd.get())) {
			eglImageOutCache_.pop_back();
			return nullptr;
		}
	}

	return eglImageOut;
}

/*
 * Reads a plane of an NV12 or NV21 output back from its target, \a rows lines
 * of the output's width at \a dst. The target is a quarter as wide in RGBA,
 * rounded up, so a width that isn't a multiple of four goes through a copy.
 */
void DebayerEGL::readPlane(eGLImage &target, uint8_t *dst, unsigned int rows)
{
	const unsigned int width = outputSize_.width;

	glBindFramebuffer(GL_FRAMEBUFFER, target.fbo_);
	if (width == target.width_ * 4 && outputConfig_.stride == width) {
		glReadPixels(0, 0, target.width_, rows, GL_RGBA, GL_UNSIGNED_BYTE, dst);
		return;
	}

	readback_.resize(target.width_ * 4 * rows);
	glReadPixels(0, 0, target.width_, rows, GL_RGBA, GL_UNSIGNED_BYTE, readback_.data());
	for (unsigned int y = 0; y < rows; y++)
		memcpy(dst + y * outputConfig_.stride, &readback_[y * target.width_ * 4], width);
}

/*
 * The planes of an NV12 or NV21 output as targets four bytes to a texel, or
 * null if the EGL can't import them or the width isn't a multiple of four.
 */
DebayerEGL::YuvTargets *DebayerEGL::getCachedOutputYuvFrameBuffer(FrameBuffer *output)
{
	Span<const FrameBuffer::Plane> planes = output->planes();
	const SharedFD &fd = planes[0].fd;

	for (auto &[ofd, targets] : eglYuvOutCache_) {
		if (ofd == fd)
			return targets.y ? &targets : nullptr;
	}

	if (eglYuvOutCache_.size() >= outputBufferCount_)
		eglYuvOutCache_.pop_front();
	eglYuvOutCache_.emplace_back(fd, YuvTargets{});
	YuvTargets &targets = eglYuvOutCache_.back().second;

	if (outputSize_.width % 4)
		return nullptr;

	const unsigned int stride = outputConfig_.stride;
	const FrameBuffer::Plane &chroma = planes.size() > 1 ? planes[1] : planes[0];
	auto y = std::make_unique<eGLImage>(GL_RGBA, outputSize_.width / 4, outputSize_.height,
					    stride, GL_TEXTURE1, 1);
	auto uv = std::make_unique<eGLImage>(GL_RGBA, outputSize_.width / 4, outputSize_.height / 2,
					     stride, GL_TEXTURE1, 1);
	/* Bytes in texels keep their order: r, g, b, a */
	y->drmFormat_ = DRM_FORMAT_ABGR8888;
	uv->drmFormat_ = DRM_FORMAT_ABGR8888;
	y->offset_ = planes[0].offset;
	uv->offset_ = planes.size() > 1 ? chroma.offset
					: planes[0].offset + stride * outputSize_.height;

	if (egl_.createOutputDMABufTexture2D(*y, fd.get()) ||
	    egl_.createOutputDMABufTexture2D(*uv, chroma.fd.get())) {
		LOG(Debayer, Debug) << "Output planes not imported, reading them back";
		return nullptr;
	}

	targets.y = std::move(y);
	targets.uv = std::move(uv);
	return &targets;
}

/* Makes \a program the one in use and finds its variables. */
int DebayerEGL::usePass(GLuint program)
{
	programId_ = program;
	egl_.useProgram(programId_);
	return getShaderVariableLocations();
}

int DebayerEGL::draw(eGLImage &eglImageIn, eGLImage &eglImageOut, const DebayerParams &params,
		     GLsizei width, GLsizei height)
{
	if (egl_.attachTextureToFBO(eglImageOut))
		return -ENODEV;
	setShaderVariableValues(eglImageIn, params);

	glViewport(0, 0, width, height);
	glClear(GL_COLOR_BUFFER_BIT);
	glDrawArrays(GL_TRIANGLE_FAN, 0, DEBAYER_OPENGL_COORDS);

	GLenum err = glGetError();
	if (err != GL_NO_ERROR) {
		LOG(eGL, Error) << "Drawing scene fail " << err;
		return -ENODEV;
	}

	return 0;
}

int DebayerEGL::debayerGPU(FrameBuffer *input, FrameBuffer *output, const DebayerParams &params, std::optional<MappedFrameBuffer> *inMapped, std::optional<DmaSyncer> *inDmaSyncer)
{
	eGLImage *eglImageIn;
	eGLImage *eglImageOut;

	/* eGL context switch */
	egl_.makeCurrent();

	eglImageIn = getCachedInputFrameBuffer(input, inMapped, inDmaSyncer);
	if (!eglImageIn)
		return -ENOMEM;

	if (!yuvOutput_) {
		eglImageOut = getCachedOutputFrameBuffer(output);
		if (!eglImageOut)
			return -ENOMEM;
		if (draw(*eglImageIn, *eglImageOut, params, width_, height_))
			return -ENODEV;
		egl_.flushOutput();
		return 0;
	}

	/*
	 * The planes are rendered into the output buffer, or where the EGL
	 * can't import it into textures of the GPU's own and read back.
	 */
	YuvTargets *targets = getCachedOutputYuvFrameBuffer(output);
	eGLImage &targetY = targets ? *targets->y : *yTarget_;
	eGLImage &targetUV = targets ? *targets->uv : *uvTarget_;
	if (usePass(programY_) ||
	    draw(*eglImageIn, targetY, params, targetY.width_, targetY.height_) ||
	    usePass(programUV_) ||
	    draw(*eglImageIn, targetUV, params, targetUV.width_, targetUV.height_))
		return -ENODEV;
	if (targets) {
		egl_.flushOutput();
		return 0;
	}

	std::vector<DmaSyncer> outDmaSyncers;
	for (const FrameBuffer::Plane &plane : output->planes()) {
		if (outDmaSyncers.empty() || plane.fd != output->planes()[0].fd)
			outDmaSyncers.emplace_back(plane.fd, DmaSyncer::SyncType::Write);
	}
	MappedFrameBuffer outMapped(output, MappedFrameBuffer::MapFlag::Write);
	if (!outMapped.isValid()) {
		LOG(Debayer, Error) << "mmap-ing output buffer failed";
		return -ENOMEM;
	}

	uint8_t *dstY = outMapped.planes()[0].data();
	uint8_t *dstUV = outMapped.planes().size() > 1
				 ? outMapped.planes()[1].data()
				 : dstY + outputConfig_.stride * outputSize_.height;
	readPlane(*yTarget_, dstY, outputSize_.height);
	readPlane(*uvTarget_, dstUV, outputSize_.height / 2);

	return 0;
}

void DebayerEGL::process(uint32_t frame, FrameBuffer *input, FrameBuffer *output, const DebayerParams &params)
{
	bench_.startFrame();

	/* Copy metadata from the input buffer */
	FrameMetadata &metadata = output->_d()->metadata();
	metadata.status = input->metadata().status;
	metadata.sequence = input->metadata().sequence;
	metadata.timestamp = input->metadata().timestamp;

	std::optional<MappedFrameBuffer> inMapped;
	std::optional<DmaSyncer> inDmaSyncer;

	if (debayerGPU(input, output, params, &inMapped, &inDmaSyncer)) {
		LOG(Debayer, Error) << "debayerGPU failed";
		goto error;
	}

	for (unsigned int i = 0; i < metadata.planes().size(); i++)
		metadata.planes()[i].bytesused = output->planes()[i].length;

	/* Calculate stats for the whole frame */
	if (frame % SwStatsCpu::kStatPerNumFrames) {
		stats_->finishFrame(frame, 0);
	} else {
		if (!inMapped) {
			/*
			 * The buffer was directly imported into EGL and thus
			 * not mapped for texture upload. Do it now for the
			 * CPU-based stats calculation.
			 */
			assert(!inDmaSyncer);
			inDmaSyncer.emplace(input->planes()[0].fd, DmaSyncer::SyncType::Read);
			inMapped.emplace(input, MappedFrameBuffer::MapFlag::Read);
		}
		stats_->processFrame(frame, 0, inMapped.value());
	}
	inDmaSyncer.reset();

	egl_.syncOutput();
	bench_.finishFrame();

	outputBufferReady.emit(output);
	inputBufferReady.emit(input);

	return;

error:
	bench_.finishFrame();
	metadata.status = FrameMetadata::FrameError;
	return;
}

int DebayerEGL::start()
{
	GLint maxTextureImageUnits;

	if (egl_.initEGLContext())
		return -ENODEV;

	glGetIntegerv(GL_MAX_TEXTURE_IMAGE_UNITS, &maxTextureImageUnits);

	LOG(Debayer, Debug) << "Available fragment shader texture units " << maxTextureImageUnits;

	yuvOutput_ = outputPixelFormat_ == formats::NV12 ||
		     outputPixelFormat_ == formats::NV21;
	if (!yuvOutput_)
		return initBayerShaders(inputPixelFormat_, outputPixelFormat_, nullptr)
			       ? -EINVAL : 0;

	const unsigned int targetWidth = (outputSize_.width + 3) / 4;
	yTarget_ = std::make_unique<eGLImage>(GL_RGBA, targetWidth, outputSize_.height,
					      targetWidth * 4, GL_TEXTURE1, 1);
	uvTarget_ = std::make_unique<eGLImage>(GL_RGBA, targetWidth, outputSize_.height / 2,
					       targetWidth * 4, GL_TEXTURE1, 1);
	egl_.createTexture2D(*yTarget_, nullptr);
	egl_.createTexture2D(*uvTarget_, nullptr);

	if (initBayerShaders(inputPixelFormat_, outputPixelFormat_, "#define OUTPUT_UV"))
		return -EINVAL;
	programUV_ = programId_;
	programId_ = 0;
	if (initBayerShaders(inputPixelFormat_, outputPixelFormat_, "#define OUTPUT_Y"))
		return -EINVAL;
	programY_ = programId_;

	return 0;
}

void DebayerEGL::stop()
{
	eglImageOutCache_.clear();
	eglYuvOutCache_.clear();
	eglImageInCache_.clear();
	yTarget_.reset();
	uvTarget_.reset();
	lut_.reset();

	if (yuvOutput_) {
		if (programY_)
			glDeleteProgram(programY_);
		if (programUV_)
			glDeleteProgram(programUV_);
		programY_ = programUV_ = programId_ = 0;
	} else if (programId_) {
		glDeleteProgram(programId_);
		programId_ = 0;
	}
}

SizeRange DebayerEGL::sizes(PixelFormat inputFormat, const Size &inputSize)
{
	Size patternSize = this->patternSize(inputFormat);
	unsigned int borderHeight = patternSize.height;

	if (patternSize.isNull())
		return {};

	/* No need for top/bottom border with a pattern height of 2 */
	if (patternSize.height == 2)
		borderHeight = 0;

	/*
	 * For debayer interpolation a border is kept around the entire image
	 * and the minimum output size is pattern-height x pattern-width.
	 */
	if (inputSize.width < (3 * patternSize.width) ||
	    inputSize.height < (2 * borderHeight + patternSize.height)) {
		LOG(Debayer, Warning)
			<< "Input format size too small: " << inputSize;
		return {};
	}

	return SizeRange(Size(patternSize.width, patternSize.height),
			 Size((inputSize.width - 2 * patternSize.width) & ~(patternSize.width - 1),
			      (inputSize.height - 2 * borderHeight) & ~(patternSize.height - 1)),
			 patternSize.width, patternSize.height);
}

} /* namespace libcamera */
