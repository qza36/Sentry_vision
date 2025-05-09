// Copyright Chen Jun 2023. Licensed under the MIT License.
//
// Additional modifications and features by Chengfu Zou, Labor. Licensed under Apache License 2.0.
//
// Copyright (C) FYT Vision Group. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// std
#include <algorithm>
#include <cstddef>
#include <filesystem>
#include <functional>
#include <map>
#include <memory>
#include <numeric>
#include <string>
#include <vector>
// ros2
#include <cv_bridge/cv_bridge.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>

#include <image_transport/image_transport.hpp>
#include <rclcpp/qos.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// third party
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
// project
#include "armor_detector/armor_detector_node.hpp"
#include "armor_detector/ba_solver.hpp"
#include "armor_detector/types.hpp"
#include "rm_utils/assert.hpp"
#include "rm_utils/common.hpp"
#include "rm_utils/logger/log.hpp"
#include "rm_utils/math/pnp_solver.hpp"
#include "rm_utils/math/utils.hpp"
#include "rm_utils/url_resolver.hpp"

namespace fyt::auto_aim {
ArmorDetectorNode::ArmorDetectorNode(const rclcpp::NodeOptions &options)
: Node("armor_detector", options) {
  FYT_REGISTER_LOGGER("armor_detector", "~/fyt2024-log", INFO);
  FYT_INFO("armor_detector", "Starting ArmorDetectorNode!");
  // Detector
  detector_ = initDetector();

  // Tricks to make pose more accurate
  use_ba_ = this->declare_parameter("use_ba", true);
  pnp_solution_selection_ = this->declare_parameter("pnp_solution_selection", false);

  // Armors Publisher
  armors_pub_ = this->create_publisher<rm_interfaces::msg::Armors>("armor_detector/armors",
                                                                   rclcpp::SensorDataQoS());

  // Transform initialize
  odom_frame_ = this->declare_parameter("target_frame", "odom_vision");
  imu_to_camera_ = Eigen::Matrix3d::Identity();

  // Visualization Marker Publisher
  // See http://wiki.ros.org/rviz/DisplayTypes/Marker
  armor_marker_.ns = "armors";
  armor_marker_.action = visualization_msgs::msg::Marker::ADD;
  armor_marker_.type = visualization_msgs::msg::Marker::CUBE;
  armor_marker_.scale.x = 0.03;
  armor_marker_.scale.y = 0.15;
  armor_marker_.scale.z = 0.12;
  armor_marker_.color.a = 1.0;
  armor_marker_.color.r = 1.0;
  armor_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);

  text_marker_.ns = "classification";
  text_marker_.action = visualization_msgs::msg::Marker::ADD;
  text_marker_.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  text_marker_.scale.z = 0.1;
  text_marker_.color.a = 1.0;
  text_marker_.color.r = 1.0;
  text_marker_.color.g = 1.0;
  text_marker_.color.b = 1.0;
  text_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);

  marker_pub_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>("armor_detector/marker", 10);

  // Debug Publishers
  debug_ = this->declare_parameter("debug", true);
  if (debug_) {
    createDebugPublishers();
  }
  // Debug param change moniter
  debug_param_sub_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
  debug_cb_handle_ =
    debug_param_sub_->add_parameter_callback("debug", [this](const rclcpp::Parameter &p) {
      debug_ = p.as_bool();
      debug_ ? createDebugPublishers() : destroyDebugPublishers();
    });

  cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "camera_info",
    rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::CameraInfo::SharedPtr camera_info) {
      cam_center_ = cv::Point2f(camera_info->k[2], camera_info->k[5]);
      cam_info_ = std::make_shared<sensor_msgs::msg::CameraInfo>(*camera_info);
      // Setup pnp solver
      pnp_solver_ = std::make_unique<PnPSolver>(camera_info->k, camera_info->d);
      pnp_solver_->setObjectPoints(
        "small", Armor::buildObjectPoints<cv::Point3f>(SMALL_ARMOR_WIDTH, SMALL_ARMOR_HEIGHT));
      pnp_solver_->setObjectPoints(
        "large", Armor::buildObjectPoints<cv::Point3f>(LARGE_ARMOR_WIDTH, LARGE_ARMOR_HEIGHT));
      // BA solver
      ba_solver_ = std::make_unique<BaSolver>(camera_info->k, camera_info->d);
      cam_info_sub_.reset();
    });

  img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "image_raw",
    rclcpp::SensorDataQoS(),
    std::bind(&ArmorDetectorNode::imageCallback, this, std::placeholders::_1));

  // target_sub_ = this->create_subscription<rm_interfaces::msg::Target>(
  //   "armor_solver/target",
  //   rclcpp::SensorDataQoS(),
  //   std::bind(&ArmorDetectorNode::targetCallback, this, std::placeholders::_1));

  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(), this->get_node_timers_interface());
  tf2_buffer_->setCreateTimerInterface(timer_interface);
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

  set_mode_srv_ = this->create_service<rm_interfaces::srv::SetMode>(
    "armor_detector/set_mode",
    std::bind(
      &ArmorDetectorNode::setModeCallback, this, std::placeholders::_1, std::placeholders::_2));

  heartbeat_ = HeartBeatPublisher::create(this);
}

void ArmorDetectorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg) {
  // Get the transform from odom_vision to gimbal
  try {
    rclcpp::Time target_time = img_msg->header.stamp;
    auto odom_to_gimbal = tf2_buffer_->lookupTransform(
      odom_frame_, "camera_optical_frame", target_time, rclcpp::Duration::from_seconds(0.01));
    auto msg_q = odom_to_gimbal.transform.rotation;
    tf2::Quaternion tf_q;
    tf2::fromMsg(msg_q, tf_q);
    tf2::Matrix3x3 tf2_matrix = tf2::Matrix3x3(tf_q);
    imu_to_camera_ << tf2_matrix.getRow(0)[0], tf2_matrix.getRow(0)[1], tf2_matrix.getRow(0)[2],
      tf2_matrix.getRow(1)[0], tf2_matrix.getRow(1)[1], tf2_matrix.getRow(1)[2],
      tf2_matrix.getRow(2)[0], tf2_matrix.getRow(2)[1], tf2_matrix.getRow(2)[2];
  } catch (...) {
    FYT_ERROR("armor_detector", "Something Wrong when lookUpTransform");
    return;
  }

  // Detect armors
  auto armors = detectArmors(img_msg);

  // Init message
  armors_msg_.header = armor_marker_.header = text_marker_.header = img_msg->header;
  armors_msg_.armors.clear();
  marker_array_.markers.clear();
  armor_marker_.id = 0;
  text_marker_.id = 0;

  // Solve PnP and BA
  if (ba_solver_ != nullptr && pnp_solver_ != nullptr) {
    rm_interfaces::msg::Armor armor_msg;
    // Get the pose of each armor
    for (auto &armor : armors) {
      cv::Mat rvec(3, 1, CV_64F), tvec(3, 1, CV_64F);
      cv::Mat rotation_matrix(3, 3, CV_64F);

      // Use PnP to get the initial pose information
      if (pnp_solver_->solvePnP(
            armor.landmarks(), rvec, tvec, (armor.type == ArmorType::SMALL ? "small" : "large"))) {
        armor.roll = rvecToRPY(rvec, 0) * 180 / M_PI;
        armor.imu2camera = imu_to_camera_;

        // Select the best PnP solution according to the pitch angle
        // Optimize armor parallel to the ground only
        if (pnp_solution_selection_ && std::abs(armor.roll) < 10) {
          PnPSolutionsSelection(armor, rvec, tvec);
        }

        cv::Rodrigues(rvec, rotation_matrix);
        armor.rmat = rotation_matrix.clone();
        armor.tvec = tvec.clone();

        if (use_ba_) {
          // Optimize armor parallel to the ground only
          if (std::abs(armor.roll) < 10) {
            tracked_armors_.push_back(armor);
          }

          // Initially, We wanted to do multi-frame BA optimization and a
          // queue was used as input, but later we found that it didn't work well,
          // so we just fixed the queue size to 1.
          if (tracked_armors_.size() > 1) {
            tracked_armors_.pop_front();
          }

          // Use BA alogorithm to optimize the pose from PnP
          // solveBa() will modify the rotation_matrix
          ba_solver_->solveBa(tracked_armors_, rotation_matrix);
        }

        // Fill basic info
        armor_msg.type = armorTypeToString(armor.type);
        armor_msg.number = armor.number;

        // Fill pose
        armor_msg.pose.position.x = tvec.at<double>(0);
        armor_msg.pose.position.y = tvec.at<double>(1);
        armor_msg.pose.position.z = tvec.at<double>(2);

        // rotation matrix to quaternion
        tf2::Matrix3x3 tf2_rotation_matrix(rotation_matrix.at<double>(0, 0),
                                           rotation_matrix.at<double>(0, 1),
                                           rotation_matrix.at<double>(0, 2),
                                           rotation_matrix.at<double>(1, 0),
                                           rotation_matrix.at<double>(1, 1),
                                           rotation_matrix.at<double>(1, 2),
                                           rotation_matrix.at<double>(2, 0),
                                           rotation_matrix.at<double>(2, 1),
                                           rotation_matrix.at<double>(2, 2));
        tf2::Quaternion tf2_quaternion;
        tf2_rotation_matrix.getRotation(tf2_quaternion);
        armor_msg.pose.orientation.x = tf2_quaternion.x();
        armor_msg.pose.orientation.y = tf2_quaternion.y();
        armor_msg.pose.orientation.z = tf2_quaternion.z();
        armor_msg.pose.orientation.w = tf2_quaternion.w();

        // Fill the distance to image center
        armor_msg.distance_to_image_center = pnp_solver_->calculateDistanceToCenter(armor.center);

        // Fill the markers
        armor_marker_.pose = armor_msg.pose;
        armor_marker_.id++;
        text_marker_.pose.position = armor_msg.pose.position;
        text_marker_.id++;
        text_marker_.pose.position.y -= 0.1;
        text_marker_.text = armor.classfication_result;
        armors_msg_.armors.emplace_back(armor_msg);
        marker_array_.markers.emplace_back(armor_marker_);
        marker_array_.markers.emplace_back(text_marker_);

        // std::string path =
        //   fmt::format("/home/zcf/fyt2024-log/images/{}/{}.jpg", armor_msg.number, now().seconds());
        // cv::imwrite(path, armor.number_img);

      } else {
        FYT_WARN("armor_detector", "PnP Failed!");
      }
    }
    // Publishing detected armors
    armors_pub_->publish(armors_msg_);
    // Publishing marker
    publishMarkers();
  }
}

std::unique_ptr<Detector> ArmorDetectorNode::initDetector() {
  rcl_interfaces::msg::ParameterDescriptor param_desc;
  param_desc.integer_range.resize(1);
  param_desc.integer_range[0].step = 1;
  param_desc.integer_range[0].from_value = 0;
  param_desc.integer_range[0].to_value = 255;
  int binary_thres = declare_parameter("binary_thres", 160, param_desc);

  Detector::LightParams l_params = {
    .min_ratio = declare_parameter("light.min_ratio", 0.08),
    .max_ratio = declare_parameter("light.max_ratio", 0.4),
    .max_angle = declare_parameter("light.max_angle", 40.0),
    .color_diff_thresh = static_cast<int>(declare_parameter("light.color_diff_thresh", 25))};

  Detector::ArmorParams a_params = {
    .min_light_ratio = declare_parameter("armor.min_light_ratio", 0.6),
    .min_small_center_distance = declare_parameter("armor.min_small_center_distance", 0.8),
    .max_small_center_distance = declare_parameter("armor.max_small_center_distance", 3.2),
    .min_large_center_distance = declare_parameter("armor.min_large_center_distance", 3.2),
    .max_large_center_distance = declare_parameter("armor.max_large_center_distance", 5.0),
    .max_angle = declare_parameter("armor.max_angle", 35.0)};

  auto detector = std::make_unique<Detector>(binary_thres, EnemyColor::RED, l_params, a_params);

  // Init classifier
  namespace fs = std::filesystem;
  fs::path model_path =
    utils::URLResolver::getResolvedPath("package://armor_detector/model/lenet.onnx");
  fs::path label_path =
    utils::URLResolver::getResolvedPath("package://armor_detector/model/label.txt");
  FYT_ASSERT_MSG(fs::exists(model_path) && fs::exists(label_path),
                 model_path.string() + " Not Found!");

  double threshold = this->declare_parameter("classifier_threshold", 0.7);
  std::vector<std::string> ignore_classes =
    this->declare_parameter("ignore_classes", std::vector<std::string>{"negative"});
  detector->classifier =
    std::make_unique<NumberClassifier>(model_path, label_path, threshold, ignore_classes);

  // Init Corrector
  bool use_pca = this->declare_parameter("use_pca", true);
  if (use_pca) {
    detector->corner_corrector = std::make_unique<LightCornerCorrector>();
  }

  // Set dynamic parameter callback
  on_set_parameters_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&ArmorDetectorNode::onSetParameters, this, std::placeholders::_1));

  return detector;
}

std::vector<Armor> ArmorDetectorNode::detectArmors(
  const sensor_msgs::msg::Image::ConstSharedPtr &img_msg) {
  // Convert ROS img to cv::Mat
  auto img = cv_bridge::toCvShare(img_msg, "rgb8")->image;

  auto armors = detector_->detect(img);

  auto final_time = this->now();
  auto latency = (final_time - img_msg->header.stamp).seconds() * 1000;

  // Publish debug info
  if (debug_) {
    binary_img_pub_.publish(
      cv_bridge::CvImage(img_msg->header, "mono8", detector_->binary_img).toImageMsg());

    // Sort lights and armors data by x coordinate
    std::sort(detector_->debug_lights.data.begin(),
              detector_->debug_lights.data.end(),
              [](const auto &l1, const auto &l2) { return l1.center_x < l2.center_x; });
    std::sort(detector_->debug_armors.data.begin(),
              detector_->debug_armors.data.end(),
              [](const auto &a1, const auto &a2) { return a1.center_x < a2.center_x; });

    lights_data_pub_->publish(detector_->debug_lights);
    armors_data_pub_->publish(detector_->debug_armors);

    if (!armors.empty()) {
      auto all_num_img = detector_->getAllNumbersImage();
      number_img_pub_.publish(
        *cv_bridge::CvImage(img_msg->header, "mono8", all_num_img).toImageMsg());
    }

    detector_->drawResults(img);

    // Draw camera center
    cv::circle(img, cam_center_, 5, cv::Scalar(255, 0, 0), 2);
    // Draw latency
    std::stringstream latency_ss;
    latency_ss << "Latency: " << std::fixed << std::setprecision(2) << latency << "ms";
    auto latency_s = latency_ss.str();
    cv::putText(
      img, latency_s, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
    result_img_pub_.publish(cv_bridge::CvImage(img_msg->header, "rgb8", img).toImageMsg());
  }

  return armors;
}

void ArmorDetectorNode::PnPSolutionsSelection(const Armor &armor,
                                              cv::Mat &rvec,
                                              cv::Mat &tvec) noexcept {
  // From all possible pnp solutions, we choose the one with closest pitch to 15 degree
  auto pnp_solutions = pnp_solver_->getAllSolutions();
  auto rvecs = std::move(pnp_solutions.at(0));
  auto tvecs = std::move(pnp_solutions.at(1));

  size_t best_idx = 0;
  double prior = armor.number == "outpost" ? -FIFTTEN_DEGREE_RAD : FIFTTEN_DEGREE_RAD;
  double best_diff = std::abs(rvecToRPY(rvecs[0], 1) - prior);
  for (size_t i = 0; i < rvecs.size(); i++) {
    double diff = std::abs(rvecToRPY(rvecs[i], 1) - prior);
    if (diff < best_diff) {
      best_diff = diff;
      best_idx = i;
    }
  }

  if (best_idx != 0) {
    FYT_DEBUG("armor_detector", "PnP Solution Changed!");
    rvec = rvecs[best_idx];
    // Take average
    tvec = std::accumulate(tvecs.begin(), tvecs.end(), cv::Mat::zeros(3, 1, CV_64F)) / tvecs.size();
  }
}

double ArmorDetectorNode::rvecToRPY(const cv::Mat &rvec, int axis) const noexcept {
  cv::Mat R;
  cv::Rodrigues(rvec, R);
  Eigen::Matrix3d eigen_R = utils::cvToEigen(R);
  // Transform to imu frame
  eigen_R = imu_to_camera_ * eigen_R;
  Eigen::Quaterniond q(eigen_R);
  // Get armor yaw
  tf2::Quaternion tf_q(q.x(), q.y(), q.z(), q.w());
  std::array<double, 3> rpy;
  tf2::Matrix3x3(tf_q).getRPY(rpy[0], rpy[1], rpy[2]);
  return rpy[axis];
}

rcl_interfaces::msg::SetParametersResult ArmorDetectorNode::onSetParameters(
  std::vector<rclcpp::Parameter> parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto &param : parameters) {
    if (param.get_name() == "binary_thres") {
      detector_->binary_thres = param.as_int();
    } else if (param.get_name() == "classifier_threshold") {
      detector_->classifier->threshold = param.as_double();
    } else if (param.get_name() == "light.min_ratio") {
      detector_->light_params.min_ratio = param.as_double();
    } else if (param.get_name() == "light.max_ratio") {
      detector_->light_params.max_ratio = param.as_double();
    } else if (param.get_name() == "light.max_angle") {
      detector_->light_params.max_angle = param.as_double();
    } else if (param.get_name() == "light.color_diff_thresh") {
      detector_->light_params.color_diff_thresh = param.as_int();
    } else if (param.get_name() == "armor.min_light_ratio") {
      detector_->armor_params.min_light_ratio = param.as_double();
    } else if (param.get_name() == "armor.min_small_center_distance") {
      detector_->armor_params.min_small_center_distance = param.as_double();
    } else if (param.get_name() == "armor.max_small_center_distance") {
      detector_->armor_params.max_small_center_distance = param.as_double();
    } else if (param.get_name() == "armor.min_large_center_distance") {
      detector_->armor_params.min_large_center_distance = param.as_double();
    } else if (param.get_name() == "armor.max_large_center_distance") {
      detector_->armor_params.max_large_center_distance = param.as_double();
    } else if (param.get_name() == "armor.max_angle") {
      detector_->armor_params.max_angle = param.as_double();
    }
  }
  return result;
}

// void ArmorDetectorNode::targetCallback(const rm_interfaces::msg::Target::SharedPtr target_msg) {
//   if (target_msg->tracking) {
//     tracked_target_ = target_msg;
//   } else {
//     tracked_target_ = nullptr;
//     if (!tracked_armors_.empty()) {
//       tracked_armors_.clear();
//     }
//   }
// }

void ArmorDetectorNode::createDebugPublishers() noexcept {
  lights_data_pub_ =
    this->create_publisher<rm_interfaces::msg::DebugLights>("armor_detector/debug_lights", 10);
  armors_data_pub_ =
    this->create_publisher<rm_interfaces::msg::DebugArmors>("armor_detector/debug_armors", 10);
  this->declare_parameter("armor_detector.result_img.jpeg_quality", 50);
  this->declare_parameter("armor_detector.binary_img.jpeg_quality", 50);
  binary_img_pub_ = image_transport::create_publisher(this, "armor_detector/binary_img");
  number_img_pub_ = image_transport::create_publisher(this, "armor_detector/number_img");
  result_img_pub_ = image_transport::create_publisher(this, "armor_detector/result_img");
}

void ArmorDetectorNode::destroyDebugPublishers() noexcept {
  lights_data_pub_.reset();
  armors_data_pub_.reset();

  binary_img_pub_.shutdown();
  number_img_pub_.shutdown();
  result_img_pub_.shutdown();
}

void ArmorDetectorNode::publishMarkers() noexcept {
  using Marker = visualization_msgs::msg::Marker;
  armor_marker_.action = armors_msg_.armors.empty() ? Marker::DELETE : Marker::ADD;
  marker_array_.markers.emplace_back(armor_marker_);
  marker_pub_->publish(marker_array_);
}

void ArmorDetectorNode::setModeCallback(
  const std::shared_ptr<rm_interfaces::srv::SetMode::Request> request,
  std::shared_ptr<rm_interfaces::srv::SetMode::Response> response) {
  response->success = true;
  response->message = "0";

  VisionMode mode = static_cast<VisionMode>(request->mode);
  std::string mode_name = visionModeToString(mode);
  if (mode_name == "UNKNOWN") {
    FYT_ERROR("armor_detector", "Invalid mode: {}", request->mode);
    return;
  }

  auto createImageSub = [this]() {
    if (img_sub_ == nullptr) {
      img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "image_raw",
        rclcpp::SensorDataQoS(),
        std::bind(&ArmorDetectorNode::imageCallback, this, std::placeholders::_1));
    }
  };

  switch (mode) {
    case VisionMode::AUTO_AIM_RED: {
      detector_->detect_color = EnemyColor::RED;
      createImageSub();
      break;
    }
    case VisionMode::AUTO_AIM_BLUE: {
      detector_->detect_color = EnemyColor::BLUE;
      createImageSub();
      break;
    }
    default: {
      img_sub_.reset();
    }
  }

  FYT_WARN("armor_detector", "Set mode to {}", mode_name);
}

}  // namespace fyt::auto_aim

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(fyt::auto_aim::ArmorDetectorNode)
