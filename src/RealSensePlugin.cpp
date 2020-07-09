/*
// Copyright (c) 2016 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
*/

#include "realsense_gazebo_plugin/RealSensePlugin.h"
#include <gazebo/physics/physics.hh>
#include <gazebo/rendering/DepthCamera.hh>
#include <gazebo/sensors/sensors.hh>

#define DEPTH_SCALE_M 0.001

#define DEPTH_CAMERA_TOPIC "depth"
#define COLOR_CAMERA_TOPIC "color"
#define IRED1_CAMERA_TOPIC "infrared"
#define IRED2_CAMERA_TOPIC "infrared2"

using namespace gazebo;

/////////////////////////////////////////////////
RealSensePlugin::RealSensePlugin() {
  this->depthCam = nullptr;
  this->ired1Cam = nullptr;
  this->ired2Cam = nullptr;
  this->colorCam = nullptr;
  this->prefix = "";
  this->pointCloudCutOffMax_ = 5.0;
}

/////////////////////////////////////////////////
RealSensePlugin::~RealSensePlugin() {}

/////////////////////////////////////////////////
void RealSensePlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  // Output the name of the model
  std::cout
      << std::endl
      << "RealSensePlugin: The realsense_camera plugin is attach to model "
      << _model->GetName() << std::endl;

  cameraParamsMap_.insert(std::make_pair(COLOR_CAMERA_NAME, CameraParams()));
  cameraParamsMap_.insert(std::make_pair(DEPTH_CAMERA_NAME, CameraParams()));
  cameraParamsMap_.insert(std::make_pair(IRED1_CAMERA_NAME, CameraParams()));
  cameraParamsMap_.insert(std::make_pair(IRED2_CAMERA_NAME, CameraParams()));

  if (_sdf->HasElement("depthUpdateRate"))
    depthUpdateRate_ = _sdf->Get<double>("depthUpdateRate");
  else
    depthUpdateRate_ = 60.0;

  if (_sdf->HasElement("colorUpdateRate"))
    colorUpdateRate_ = _sdf->Get<double>("colorUpdateRate");
  else
    colorUpdateRate_ = 60.0;

  if (_sdf->HasElement("infraredUpdateRate"))
    infraredUpdateRate_ = _sdf->Get<double>("infraredUpdateRate");
  else
    infraredUpdateRate_ = 60.0;

  if (_sdf->HasElement("depthTopicName"))
    cameraParamsMap_[DEPTH_CAMERA_NAME].topic_name = _sdf->Get<std::string>("depthTopicName");
  else
    cameraParamsMap_[DEPTH_CAMERA_NAME].topic_name = "camera/depth/image_raw";

  if (_sdf->HasElement("depthCameraInfoTopicName"))
    cameraParamsMap_[DEPTH_CAMERA_NAME].camera_info_topic_name = _sdf->Get<std::string>("depthCameraInfoTopicName");
  else
    cameraParamsMap_[DEPTH_CAMERA_NAME].camera_info_topic_name = "depth_image";

  if (_sdf->HasElement("colorTopicName"))
    cameraParamsMap_[COLOR_CAMERA_NAME].topic_name = _sdf->Get<std::string>("colorTopicName");
  else
    cameraParamsMap_[COLOR_CAMERA_NAME].topic_name = "camera/color/image_raw";

  if (_sdf->HasElement("colorCameraInfoTopicName"))
    cameraParamsMap_[COLOR_CAMERA_NAME].camera_info_topic_name = _sdf->Get<std::string>("colorCameraInfoTopicName");
  else
    cameraParamsMap_[COLOR_CAMERA_NAME].camera_info_topic_name = "color_image";

  if (_sdf->HasElement("infrared1TopicName"))
    cameraParamsMap_[IRED1_CAMERA_NAME].topic_name = _sdf->Get<std::string>("infrared1TopicName");
  else
    cameraParamsMap_[IRED1_CAMERA_NAME].topic_name = "camera/ir/image_raw";

  if (_sdf->HasElement("infrared1CameraInfoTopicName"))
    cameraParamsMap_[IRED1_CAMERA_NAME].camera_info_topic_name = _sdf->Get<std::string>("infrared1CameraInfoTopicName");
  else
    cameraParamsMap_[IRED1_CAMERA_NAME].camera_info_topic_name = "ir_image";

  if (_sdf->HasElement("infrared2TopicName"))
    cameraParamsMap_[IRED2_CAMERA_NAME].topic_name = _sdf->Get<std::string>("infrared2TopicName");
  else
    cameraParamsMap_[IRED2_CAMERA_NAME].topic_name = "camera/ir2/image_raw";

  if (_sdf->HasElement("infrared2CameraInfoTopicName"))
    cameraParamsMap_[IRED2_CAMERA_NAME].camera_info_topic_name = _sdf->Get<std::string>("infrared2CameraInfoTopicName");
  else
    cameraParamsMap_[IRED2_CAMERA_NAME].camera_info_topic_name = "ir2_image";

  if (_sdf->HasElement("colorOpticalframeName"))
    cameraParamsMap_[COLOR_CAMERA_NAME].optical_frame = _sdf->Get<std::string>("colorOpticalframeName");
  else
    cameraParamsMap_[COLOR_CAMERA_NAME].optical_frame = "color";

  if (_sdf->HasElement("depthOpticalframeName"))
    cameraParamsMap_[DEPTH_CAMERA_NAME].optical_frame = _sdf->Get<std::string>("depthOpticalframeName");
  else
    cameraParamsMap_[DEPTH_CAMERA_NAME].optical_frame = "depth";

  if (_sdf->HasElement("infrared1OpticalframeName"))
    cameraParamsMap_[IRED1_CAMERA_NAME].optical_frame = _sdf->Get<std::string>("infrared1OpticalframeName");
  else
    cameraParamsMap_[IRED1_CAMERA_NAME].optical_frame = "ired1";

  if (_sdf->HasElement("infrared2OpticalframeName"))
    cameraParamsMap_[IRED2_CAMERA_NAME].optical_frame = _sdf->Get<std::string>("infrared2OpticalframeName");
  else
    cameraParamsMap_[IRED2_CAMERA_NAME].optical_frame = "ired2";

  if (_sdf->HasElement("rangeMinDepth"))
    rangeMinDepth_ = _sdf->Get<float>("rangeMinDepth");
  else
    rangeMinDepth_ = 0.3;

  if (_sdf->HasElement("rangeMaxDepth"))
    rangeMaxDepth_ = _sdf->Get<float>("rangeMaxDepth");
  else
    rangeMaxDepth_ = 10.0;

  if (_sdf->HasElement("pointCloud"))
    pointCloud_ = _sdf->Get<bool>("pointCloud");
  else
    pointCloud_ = true;

  if (_sdf->HasElement("pointCloudTopicName"))
    pointCloudTopic_ = _sdf->Get<std::string>("pointCloudTopicName");
  else
    pointCloudTopic_ = "camera/depth_registered/points";

  if (_sdf->HasElement("pointCloudCutoff"))
    pointCloudCutOff_ = _sdf->Get<double>("pointCloudCutoff");
  else
    pointCloudCutOff_ = 0.5;

  if (_sdf->HasElement("pointCloudCutoffMax"))
    pointCloudCutOffMax_ = _sdf->Get<double>("pointCloudCutoffMax");
  else
    pointCloudCutOffMax_ = 8.0;

  if (_sdf->HasElement("prefix"))
    this->prefix = _sdf->Get<std::string>("prefix");
  else
    this->prefix = "";

  // Store a pointer to the this model
  this->rsModel = _model;

  // Store a pointer to the world
  this->world = this->rsModel->GetWorld();

  // Sensors Manager
  sensors::SensorManager *smanager = sensors::SensorManager::Instance();

  // Get Cameras Renderers
  this->depthCam = std::dynamic_pointer_cast<sensors::DepthCameraSensor>(
                       smanager->GetSensor(prefix + DEPTH_CAMERA_NAME))
                       ->DepthCamera();

  this->ired1Cam = std::dynamic_pointer_cast<sensors::CameraSensor>(
                       smanager->GetSensor(prefix + IRED1_CAMERA_NAME))
                       ->Camera();
  this->ired2Cam = std::dynamic_pointer_cast<sensors::CameraSensor>(
                       smanager->GetSensor(prefix + IRED2_CAMERA_NAME))
                       ->Camera();
  this->colorCam = std::dynamic_pointer_cast<sensors::CameraSensor>(
                       smanager->GetSensor(prefix + COLOR_CAMERA_NAME))
                       ->Camera();

  // Check if camera renderers have been found successfuly
  if (!this->depthCam) {
    std::cerr << "RealSensePlugin: Depth Camera has not been found"
              << std::endl;
    return;
  }
  if (!this->ired1Cam) {
    std::cerr << "RealSensePlugin: InfraRed Camera 1 has not been found"
              << std::endl;
    return;
  }
  if (!this->ired2Cam) {
    std::cerr << "RealSensePlugin: InfraRed Camera 2 has not been found"
              << std::endl;
    return;
  }
  if (!this->colorCam) {
    std::cerr << "RealSensePlugin: Color Camera has not been found"
              << std::endl;
    return;
  }

  // Resize Depth Map dimensions
  try {
    this->depthMap.resize(this->depthCam->ImageWidth() *
                          this->depthCam->ImageHeight());
  } catch (std::bad_alloc &e) {
    std::cerr << "RealSensePlugin: depthMap allocation failed: " << e.what()
              << std::endl;
    return;
  }

  // Setup Transport Node
  this->transportNode = transport::NodePtr(new transport::Node());
#if GAZEBO_MAJOR_VERSION >= 9
  this->transportNode->Init(this->world->Name());
#else
  this->transportNode->Init(this->world->GetName());
#endif

  // Setup Publishers
  std::string rsTopicRoot = "~/" + this->rsModel->GetName() + "/rs/stream/";

  this->depthPub = this->transportNode->Advertise<msgs::ImageStamped>(
      rsTopicRoot + DEPTH_CAMERA_TOPIC, 1, depthUpdateRate_);
  this->ired1Pub = this->transportNode->Advertise<msgs::ImageStamped>(
      rsTopicRoot + IRED1_CAMERA_TOPIC, 1, infraredUpdateRate_);
  this->ired2Pub = this->transportNode->Advertise<msgs::ImageStamped>(
      rsTopicRoot + IRED2_CAMERA_TOPIC, 1, infraredUpdateRate_);
  this->colorPub = this->transportNode->Advertise<msgs::ImageStamped>(
      rsTopicRoot + COLOR_CAMERA_TOPIC, 1, colorUpdateRate_);

  // Listen to depth camera new frame event
  this->newDepthFrameConn = this->depthCam->ConnectNewDepthFrame(
      std::bind(&RealSensePlugin::OnNewDepthFrame, this));

  this->newIred1FrameConn = this->ired1Cam->ConnectNewImageFrame(std::bind(
      &RealSensePlugin::OnNewFrame, this, this->ired1Cam, this->ired1Pub));

  this->newIred2FrameConn = this->ired2Cam->ConnectNewImageFrame(std::bind(
      &RealSensePlugin::OnNewFrame, this, this->ired2Cam, this->ired2Pub));

  this->newColorFrameConn = this->colorCam->ConnectNewImageFrame(std::bind(
      &RealSensePlugin::OnNewFrame, this, this->colorCam, this->colorPub));

  // Listen to the update event
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&RealSensePlugin::OnUpdate, this));
}

/////////////////////////////////////////////////
void RealSensePlugin::OnNewFrame(const rendering::CameraPtr cam,
                                 const transport::PublisherPtr pub) {
  msgs::ImageStamped msg;

  // Set Simulation Time
#if GAZEBO_MAJOR_VERSION >= 9
  msgs::Set(msg.mutable_time(), this->world->SimTime());
#else
  msgs::Set(msg.mutable_time(), this->world->GetSimTime());
#endif

  // Set Image Dimensions
  msg.mutable_image()->set_width(cam->ImageWidth());
  msg.mutable_image()->set_height(cam->ImageHeight());

  // Set Image Pixel Format
  msg.mutable_image()->set_pixel_format(
      common::Image::ConvertPixelFormat(cam->ImageFormat()));

  // Set Image Data
  msg.mutable_image()->set_step(cam->ImageWidth() * cam->ImageDepth());
  msg.mutable_image()->set_data(cam->ImageData(),
                                cam->ImageDepth() * cam->ImageWidth() *
                                    cam->ImageHeight());

  // Publish realsense infrared stream
  pub->Publish(msg);
}

/////////////////////////////////////////////////
void RealSensePlugin::OnNewDepthFrame() {
  // Get Depth Map dimensions
  unsigned int imageSize =
      this->depthCam->ImageWidth() * this->depthCam->ImageHeight();

  // Instantiate message
  msgs::ImageStamped msg;

  // Convert Float depth data to RealSense depth data
  const float *depthDataFloat = this->depthCam->DepthData();
  for (unsigned int i = 0; i < imageSize; ++i) {
    // Check clipping and overflow
    if (depthDataFloat[i] < rangeMinDepth_ ||
        depthDataFloat[i] > rangeMaxDepth_ ||
        depthDataFloat[i] > DEPTH_SCALE_M * UINT16_MAX ||
        depthDataFloat[i] < 0) {
      this->depthMap[i] = 0;
    } else {
      this->depthMap[i] = (uint16_t)(depthDataFloat[i] / DEPTH_SCALE_M);
    }
  }

  // Pack realsense scaled depth map
#if GAZEBO_MAJOR_VERSION >= 9
  msgs::Set(msg.mutable_time(), this->world->SimTime());
#else
  msgs::Set(msg.mutable_time(), this->world->GetSimTime());
#endif
  msg.mutable_image()->set_width(this->depthCam->ImageWidth());
  msg.mutable_image()->set_height(this->depthCam->ImageHeight());
  msg.mutable_image()->set_pixel_format(common::Image::L_INT16);
  msg.mutable_image()->set_step(this->depthCam->ImageWidth() *
                                this->depthCam->ImageDepth());
  msg.mutable_image()->set_data(this->depthMap.data(),
                                sizeof(*this->depthMap.data()) * imageSize);

  // Publish realsense scaled depth map
  this->depthPub->Publish(msg);
}

/////////////////////////////////////////////////
void RealSensePlugin::OnUpdate() {}
