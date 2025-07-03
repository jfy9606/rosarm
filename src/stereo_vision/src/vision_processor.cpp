#include "stereo_vision/vision_processor.h"

namespace stereo_vision {

VisionProcessor::VisionProcessor(ros::NodeHandle& nh) : 
    nh_(nh), 
    it_(nh),
    view_mode_(0),
    is_camera_available_(false),
    detection_enabled_(false),
    stereo_camera_error_count_(0),
    stereo_method_("SGBM") {
    
    // Initialize publishers
    current_view_pub_ = it_.advertise("stereo_vision/current_view", 1);
    depth_image_pub_ = it_.advertise("stereo_vision/depth_image", 1);
    detection_image_pub_ = it_.advertise("stereo_vision/detection_image", 1);
    detection_poses_pub_ = nh_.advertise<geometry_msgs::PoseArray>("stereo_vision/object_poses", 1);
    
    // Initialize subscribers
    stereo_merged_sub_ = it_.subscribe("stereo_vision/merged_image", 1, 
                                      &VisionProcessor::stereoMergedCallback, this);
    
    // 订阅Python节点发布的检测结果
    python_detection_image_sub_ = it_.subscribe("/detections/image", 1,
                                              &VisionProcessor::pythonDetectionImageCallback, this);
    python_detection_poses_sub_ = nh_.subscribe("/detections/poses", 1,
                                              &VisionProcessor::pythonDetectionPosesCallback, this);
    
    // Initialize stereo synchronizer
    left_image_sub_.subscribe(nh_, "stereo_vision/left/image_rect", 1);
    right_image_sub_.subscribe(nh_, "stereo_vision/right/image_rect", 1);
    stereo_sync_.reset(new StereoSynchronizer(StereoSyncPolicy(10), left_image_sub_, right_image_sub_));
    stereo_sync_->registerCallback(boost::bind(&VisionProcessor::stereoPairCallback, this, _1, _2));
    
    // Initialize services
    detection_control_service_ = nh_.advertiseService("stereo_vision/detection_control",
                                                    &VisionProcessor::handleDetectionControl, this);
    view_mode_service_ = nh_.advertiseService("stereo_vision/set_view_mode",
                                            &VisionProcessor::handleSetViewMode, this);
    
    // Load camera parameters
    loadCameraParameters();
    
    // Create placeholder image
    createPlaceholderImage();
    
    ROS_INFO("Stereo Vision Processor initialized");
}

VisionProcessor::~VisionProcessor() {
    ROS_INFO("Shutting down Stereo Vision Processor");
}

// Other methods implementation
void VisionProcessor::loadCameraParameters() {
    camera_matrix_ = cv::Mat::eye(3, 3, CV_64F);
    camera_matrix_.at<double>(0, 0) = 500.0; // fx
    camera_matrix_.at<double>(1, 1) = 500.0; // fy
    camera_matrix_.at<double>(0, 2) = 320.0; // cx
    camera_matrix_.at<double>(1, 2) = 240.0; // cy
    
    stereo_baseline_ = 0.1; // 10cm default
    
    camera_extrinsic_ = cv::Mat::eye(4, 4, CV_64F);
}

void VisionProcessor::createPlaceholderImage() {
    current_image_ = cv::Mat(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::putText(current_image_, "No Camera Feed Available", cv::Point(50, 240), 
               cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
}

void VisionProcessor::detectObjects() {
    // 清空检测对象列表
    detected_objects_.clear();
    
    // 创建空的位姿数组消息
    geometry_msgs::PoseArray pose_array;
    pose_array.header.stamp = ros::Time::now();
    pose_array.header.frame_id = "camera_link";
    
    // 注意：实际的物体检测由Python节点(yolo_detector.py或stereo_detection_node.py)完成
    // 这里仅转发检测结果，不实现物体识别逻辑
    
    // 转发位姿数组
    detection_poses_pub_.publish(pose_array);
    
    // 记录日志
    ROS_DEBUG("VisionProcessor::detectObjects - 物体检测由Python节点完成，C++端仅转发结果");
}

// 新增添加的方法实现
void VisionProcessor::stereoMergedCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        
        // Split the merged image into left and right images
        if (cv_ptr->image.cols > 0) {
            int width = cv_ptr->image.cols / 2;
            left_image_ = cv_ptr->image(cv::Rect(0, 0, width, cv_ptr->image.rows)).clone();
            right_image_ = cv_ptr->image(cv::Rect(width, 0, width, cv_ptr->image.rows)).clone();
            
            is_camera_available_ = true;
            
            // Process the images based on the view mode
            switch (view_mode_) {
                case 0: // Left image
                    current_image_ = left_image_.clone();
                    break;
                case 1: // Right image
                    current_image_ = right_image_.clone();
                    break;
                case 2: // Depth image
                    if (!left_image_.empty() && !right_image_.empty()) {
                        depth_image_ = computeDepthMap(left_image_, right_image_);
                        // Normalize depth for visualization
                        cv::normalize(depth_image_, current_image_, 0, 255, cv::NORM_MINMAX, CV_8UC1);
                        cv::applyColorMap(current_image_, current_image_, cv::COLORMAP_JET);
                    }
                    break;
            }
            
            // Perform object detection if enabled
            if (detection_enabled_) {
                detectObjects();
            }
            
            // Publish current view
            publishCurrentView();
        }
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("CV bridge exception: %s", e.what());
        handleCameraError("Error processing stereo merged image");
    }
}

void VisionProcessor::stereoPairCallback(const sensor_msgs::ImageConstPtr& left_msg, 
                                        const sensor_msgs::ImageConstPtr& right_msg) {
    try {
        cv_bridge::CvImagePtr left_cv_ptr = cv_bridge::toCvCopy(left_msg, sensor_msgs::image_encodings::BGR8);
        cv_bridge::CvImagePtr right_cv_ptr = cv_bridge::toCvCopy(right_msg, sensor_msgs::image_encodings::BGR8);
        
        if (left_cv_ptr->image.cols > 0 && right_cv_ptr->image.cols > 0) {
            left_image_ = left_cv_ptr->image.clone();
            right_image_ = right_cv_ptr->image.clone();
            
            is_camera_available_ = true;
            
            // Process the images based on the view mode
            switch (view_mode_) {
                case 0: // Left image
                    current_image_ = left_image_.clone();
                    break;
                case 1: // Right image
                    current_image_ = right_image_.clone();
                    break;
                case 2: // Depth image
                    depth_image_ = computeDepthMap(left_image_, right_image_);
                    // Normalize depth for visualization
                    cv::normalize(depth_image_, current_image_, 0, 255, cv::NORM_MINMAX, CV_8UC1);
                    cv::applyColorMap(current_image_, current_image_, cv::COLORMAP_JET);
                    break;
            }
            
            // Perform object detection if enabled
            if (detection_enabled_) {
                detectObjects();
            }
            
            // Publish current view
            publishCurrentView();
        }
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("CV bridge exception: %s", e.what());
        handleCameraError("Error processing stereo pair images");
    }
}

bool VisionProcessor::handleDetectionControl(stereo_vision::DetectionControl::Request& req, 
                                          stereo_vision::DetectionControl::Response& res) {
    detection_enabled_ = req.enable_detection;
    
    ROS_INFO("Object detection %s", detection_enabled_ ? "enabled" : "disabled");
    
    res.success = true;
    res.message = "Detection control updated";
    return true;
}

bool VisionProcessor::handleSetViewMode(stereo_vision::SetViewMode::Request& req, 
                                     stereo_vision::SetViewMode::Response& res) {
    if (req.view_mode >= 0 && req.view_mode <= 2) {
        view_mode_ = req.view_mode;
        res.success = true;
        res.message = "View mode updated";
    } else {
        res.success = false;
        res.message = "Invalid view mode. Use 0 (left), 1 (right), or 2 (depth)";
    }
    
    return true;
}

cv::Mat VisionProcessor::computeDepthMap(const cv::Mat& left_image, const cv::Mat& right_image) {
    // Convert to grayscale
    cv::Mat left_gray, right_gray;
    cv::cvtColor(left_image, left_gray, cv::COLOR_BGR2GRAY);
    cv::cvtColor(right_image, right_gray, cv::COLOR_BGR2GRAY);
    
    // Create disparity map
    cv::Mat disparity;
    
    if (stereo_method_ == "SGBM") {
        cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(
            0,     // minDisparity
            128,   // numDisparities
            9,     // blockSize
            8*9*9, // P1
            32*9*9, // P2
            1,     // disp12MaxDiff
            63,    // preFilterCap
            10,    // uniquenessRatio
            100,   // speckleWindowSize
            4,     // speckleRange
            cv::StereoSGBM::MODE_SGBM // mode
        );
        
        sgbm->compute(left_gray, right_gray, disparity);
    } else { // BM method
        cv::Ptr<cv::StereoBM> bm = cv::StereoBM::create(
            128, // numDisparities
            15   // blockSize
        );
        
        bm->compute(left_gray, right_gray, disparity);
    }
    
    // Convert to float and scale
    cv::Mat disparity_float;
    disparity.convertTo(disparity_float, CV_32F, 1.0/16.0);
    
    // Calculate depth map (Z = f * B / d)
    cv::Mat depth_map = cv::Mat::zeros(disparity.size(), CV_32F);
    
    for (int y = 0; y < depth_map.rows; y++) {
        for (int x = 0; x < depth_map.cols; x++) {
            float disp = disparity_float.at<float>(y, x);
            if (disp > 0) {
                // Compute depth: Z = f * B / d
                // f is the focal length, B is the baseline, d is the disparity
                float focal_length = camera_matrix_.at<double>(0, 0);
                depth_map.at<float>(y, x) = focal_length * stereo_baseline_ / disp;
            }
        }
    }
    
    return depth_map;
}

void VisionProcessor::publishCurrentView() {
    if (current_image_.empty()) {
        return;
    }
    
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(
        std_msgs::Header(), sensor_msgs::image_encodings::BGR8, current_image_).toImageMsg();
    
    current_view_pub_.publish(msg);
}

void VisionProcessor::handleCameraError(const std::string& error_msg) {
    ROS_ERROR("%s", error_msg.c_str());
    stereo_camera_error_count_++;
    
    // If too many errors, try to reconnect
    if (stereo_camera_error_count_ > 10) {
        is_camera_available_ = false;
        stereo_camera_error_count_ = 0;
        createPlaceholderImage();
    }
}

void VisionProcessor::enableObjectDetection(bool enable) {
    detection_enabled_ = enable;
}

void VisionProcessor::setCameraViewMode(int mode) {
    if (mode >= 0 && mode <= 2) {
        view_mode_ = mode;
    }
}

// 添加Python检测结果回调函数
void VisionProcessor::pythonDetectionImageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        // 将检测图像转发到我们的话题
        detection_image_pub_.publish(msg);
        
        // 转换为OpenCV图像用于处理
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
        
        // 可以在这里添加额外处理...
        
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("CV bridge exception in pythonDetectionImageCallback: %s", e.what());
    }
}

void VisionProcessor::pythonDetectionPosesCallback(const geometry_msgs::PoseArray::ConstPtr& msg) {
    // 转发位姿数组
    detection_poses_pub_.publish(*msg);
    
    // 清空之前的检测对象
    detected_objects_.clear();
    
    // 将位姿数组转换为检测对象
    for (size_t i = 0; i < msg->poses.size(); ++i) {
        const geometry_msgs::Pose& pose = msg->poses[i];
        
        DetectedObject obj;
        obj.id = "obj_" + std::to_string(i+1);
        obj.class_name = "unknown"; // Python端没有传递类别信息，默认为unknown
        obj.type = "unknown";
        obj.confidence = 1.0; // 默认置信度
        
        // 保存位姿
        obj.pose = pose;
        
        // 转换位置到厘米单位
        obj.x = pose.position.x * 100.0;
        obj.y = pose.position.y * 100.0;
        obj.z = pose.position.z * 100.0;
        
        // 设置默认颜色
        obj.color = cv::Scalar(0, 255, 0);
        
        // 添加到检测对象列表
        detected_objects_.push_back(obj);
    }
    
    ROS_DEBUG("Received %zu objects from Python detection node", detected_objects_.size());
}

} // namespace stereo_vision
