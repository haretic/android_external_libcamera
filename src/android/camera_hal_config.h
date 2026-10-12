/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Google Inc.
 *
 * Camera HAL configuration file manager
 */

#pragma once

#include <map>
#include <string>
#include <vector>

#include <libcamera/base/class.h>

struct CameraConfigData {
	int facing = -1;
	int rotation = -1;
	/*
	 * For a camera module that turns to face several directions, each one
	 * an Android camera: the position control file, the position facing
	 * this way and the one to rest at when closed.
	 */
	std::string positionControl;
	int position = -1;
	int parkPosition = -1;
};

class CameraHalConfig final : public libcamera::Extensible
{
	LIBCAMERA_DECLARE_PRIVATE()

public:
	CameraHalConfig();

	bool exists() const { return exists_; }
	bool isValid() const { return valid_; }

	const std::vector<CameraConfigData> *cameraConfigData(const std::string &cameraId) const;

private:
	bool exists_;
	bool valid_;
	std::map<std::string, std::vector<CameraConfigData>> cameras_;

	int parseConfigurationFile();
};
