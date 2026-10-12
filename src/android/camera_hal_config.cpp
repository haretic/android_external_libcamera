/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Google Inc.
 *
 * Camera HAL configuration file manager
 */
#include "camera_hal_config.h"

#include <stdlib.h>
#include <string>

#include <libcamera/base/file.h>
#include <libcamera/base/log.h>

#include "libcamera/internal/yaml_parser.h"

#include <hardware/camera3.h>

using namespace libcamera;

LOG_DEFINE_CATEGORY(HALConfig)

class CameraHalConfig::Private : public Extensible::Private
{
	LIBCAMERA_DECLARE_PUBLIC(CameraHalConfig)

public:
	Private();

	int parseConfigFile(File &file, std::map<std::string, std::vector<CameraConfigData>> *cameras);

private:
	int parseCameraConfigData(const std::string &cameraId, const ValueNode &);
	int parseView(const ValueNode &, CameraConfigData &cameraConfigData);
	int parseLocation(const ValueNode &, CameraConfigData &cameraConfigData);
	int parseRotation(const ValueNode &, CameraConfigData &cameraConfigData);

	std::map<std::string, std::vector<CameraConfigData>> *cameras_;
};

CameraHalConfig::Private::Private()
{
}

int CameraHalConfig::Private::parseConfigFile(File &file,
					      std::map<std::string, std::vector<CameraConfigData>> *cameras)
{
	/*
	 * Parse the HAL properties.
	 *
	 * Each camera properties block is a list of properties associated
	 * with the ID (as assembled by CameraSensor::generateId()) of the
	 * camera they refer to.
	 *
	 * cameras:
	 *   "camera0 id":
	 *     location: value
	 *     rotation: value
	 *     ...
	 *
	 *   "camera1 id":
	 *     location: value
	 *     rotation: value
	 *     ...
	 *
	 * A camera module that turns to face several directions lists them as
	 * views, each an Android camera, with the position (in the units of the
	 * position control) that faces that way:
	 *
	 *   "camera2 id":
	 *     position_control: /sys/.../position
	 *     park_position: value
	 *     views:
	 *       - location: value
	 *         rotation: value
	 *         position: value
	 *       ...
	 */

	cameras_ = cameras;

	std::unique_ptr<ValueNode> root = YamlParser::parse(file);
	if (!root)
		return -EINVAL;

	if (!root->isDictionary())
		return -EINVAL;

	/* Parse property "cameras" */
	if (!root->contains("cameras"))
		return -EINVAL;

	const ValueNode &camerasNode = (*root)["cameras"];

	if (!camerasNode.isDictionary())
		return -EINVAL;

	for (const auto &[cameraId, configData] : camerasNode.asDict()) {
		if (parseCameraConfigData(cameraId, configData))
			return -EINVAL;
	}

	return 0;
}

int CameraHalConfig::Private::parseCameraConfigData(const std::string &cameraId,
						    const ValueNode &cameraObject)

{
	if (!cameraObject.isDictionary())
		return -EINVAL;

	std::vector<CameraConfigData> &views = (*cameras_)[cameraId];

	if (!cameraObject.contains("views")) {
		CameraConfigData &cameraConfigData = views.emplace_back();
		return parseView(cameraObject, cameraConfigData);
	}

	const ValueNode &viewsNode = cameraObject["views"];
	std::string positionControl =
		cameraObject["position_control"].get<std::string>("");
	int32_t parkPosition = cameraObject["park_position"].get<int32_t>(-1);

	if (!viewsNode.isList() || !viewsNode.size() || positionControl.empty()) {
		LOG(HALConfig, Error)
			<< "Camera '" << cameraId
			<< "' needs a list of views and a position_control";
		return -EINVAL;
	}

	for (const ValueNode &viewNode : viewsNode.asList()) {
		CameraConfigData &cameraConfigData = views.emplace_back();

		if (!viewNode.isDictionary() || parseView(viewNode, cameraConfigData))
			return -EINVAL;

		cameraConfigData.position = viewNode["position"].get<int32_t>(-1);
		if (cameraConfigData.position < 0) {
			LOG(HALConfig, Error)
				<< "Camera '" << cameraId << "' view has no position";
			return -EINVAL;
		}

		cameraConfigData.positionControl = positionControl;
		cameraConfigData.parkPosition = parkPosition;
	}

	return 0;
}

int CameraHalConfig::Private::parseView(const ValueNode &cameraObject,
					CameraConfigData &cameraConfigData)
{
	/* Parse property "location" */
	if (parseLocation(cameraObject, cameraConfigData))
		return -EINVAL;

	/* Parse property "rotation" */
	if (parseRotation(cameraObject, cameraConfigData))
		return -EINVAL;

	return 0;
}

int CameraHalConfig::Private::parseLocation(const ValueNode &cameraObject,
					    CameraConfigData &cameraConfigData)
{
	if (!cameraObject.contains("location"))
		return -EINVAL;

	std::string location = cameraObject["location"].get<std::string>("");

	if (location == "front")
		cameraConfigData.facing = CAMERA_FACING_FRONT;
	else if (location == "back")
		cameraConfigData.facing = CAMERA_FACING_BACK;
	else
		return -EINVAL;

	return 0;
}

int CameraHalConfig::Private::parseRotation(const ValueNode &cameraObject,
					    CameraConfigData &cameraConfigData)
{
	if (!cameraObject.contains("rotation"))
		return -EINVAL;

	int32_t rotation = cameraObject["rotation"].get<int32_t>(-1);

	if (rotation < 0 || rotation >= 360) {
		LOG(HALConfig, Error)
			<< "Unknown rotation: " << rotation;
		return -EINVAL;
	}

	cameraConfigData.rotation = rotation;
	return 0;
}

CameraHalConfig::CameraHalConfig()
	: Extensible(std::make_unique<Private>()), exists_(false), valid_(false)
{
	parseConfigurationFile();
}

/*
 * Open the HAL configuration file and validate its content.
 * Return 0 on success, a negative error code otherwise
 * retval -ENOENT The configuration file is not available
 * retval -EINVAL The configuration file is available but not valid
 */
int CameraHalConfig::parseConfigurationFile()
{
	std::string filePath = LIBCAMERA_SYSCONF_DIR "/camera_hal.yaml";

	File file(filePath);
	if (!file.exists()) {
		LOG(HALConfig, Debug)
			<< "Configuration file: \"" << filePath << "\" not found";
		return -ENOENT;
	}

	if (!file.open(File::OpenModeFlag::ReadOnly)) {
		int ret = file.error();
		LOG(HALConfig, Error) << "Failed to open configuration file "
				      << filePath << ": " << strerror(-ret);
		return ret;
	}

	exists_ = true;

	int ret = _d()->parseConfigFile(file, &cameras_);
	if (ret)
		return -EINVAL;

	valid_ = true;

	for (const auto &[cameraId, views] : cameras_) {
		for (const CameraConfigData &camera : views)
			LOG(HALConfig, Debug) << "'" << cameraId << "' "
					      << "(" << camera.facing << ")["
					      << camera.rotation << "]@"
					      << camera.position;
	}

	return 0;
}

const std::vector<CameraConfigData> *CameraHalConfig::cameraConfigData(const std::string &cameraId) const
{
	const auto &it = cameras_.find(cameraId);
	if (it == cameras_.end()) {
		LOG(HALConfig, Error)
			<< "Camera '" << cameraId
			<< "' not described in the HAL configuration file";
		return nullptr;
	}

	return &it->second;
}
