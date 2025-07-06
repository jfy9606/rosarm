#include "vision/view_node.h"
#include <std_msgs/Header.h>

namespace vision {

ViewNode::ViewNode(ros::NodeHandle& nh) : nh_(nh), log_interval_(5.0) {
    // Initialize parameters with default values
    initial_view_mode_ = 0;
    left_image_topic_ = "/left_camera/image_raw";
    right_image_topic_ = "/right_camera/image_raw";
    last_log_time_ = ros::Time::now();
}

ViewNode::~ViewNode() {
    // Nothing to do here
}

bool ViewNode::init() {
    // Get parameters from ROS parameter server
    nh_.param<int>("initial_view_mode", initial_view_mode_, 0);
    nh_.param<std::string>("left_image_topic", left_image_topic_, "/left_camera/image_raw");
    nh_.param<std::string>("right_image_topic", right_image_topic_, "/right_camera/image_raw");
    
    // Create view control
    view_control_ = std::make_unique<ViewControl>(initial_view_mode_);
    
    // Create publisher
    view_mode_pub_ = nh_.advertise<std_msgs::Header>("/vision/view_mode", 10);
    
    // Create services
    switch_view_service_ = nh_.advertiseService("/vision/switch_view", &ViewNode::handleSwitchView, this);
    switch_view_service_gui_ = nh_.advertiseService("/set_view_mode", &ViewNode::handleSwitchView, this);
    
    ROS_INFO("View node initialized with mode %d (0=left, 1=right)", view_control_->getCurrentMode());
    
    // Publish current mode a few times to ensure other nodes receive it
    for (int i = 0; i < 3; ++i) {
        publishCurrentMode(true);
        ros::Duration(0.5).sleep();
    }
    
    return true;
}

void ViewNode::run() {
    ros::Rate rate(2); // 2 Hz
    int last_published_mode = view_control_->getCurrentMode();
    
    while (ros::ok()) {
        // Only log when mode changes
        if (view_control_->getCurrentMode() != last_published_mode) {
            ROS_INFO("View mode changed to: %d (0=left, 1=right)", view_control_->getCurrentMode());
            last_published_mode = view_control_->getCurrentMode();
            
            // Publish multiple times when mode changes
            for (int i = 0; i < 3; ++i) {
                publishCurrentMode(true);
                ros::Duration(0.1).sleep();
            }
        } else {
            // Normal operation, silent publishing
            publishCurrentMode(false);
        }
        
        ros::spinOnce();
        rate.sleep();
    }
}

void ViewNode::publishCurrentMode(bool force) {
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    // Use frame_id to store the view mode
    header.frame_id = std::to_string(view_control_->getCurrentMode());
    view_mode_pub_.publish(header);
    
    // Control log frequency
    ros::Time current_time = ros::Time::now();
    if (force || (current_time - last_log_time_) > log_interval_) {
        ROS_DEBUG("Current view mode: %s", view_control_->getCurrentModeDescription().c_str());
        last_log_time_ = current_time;
    }
}

bool ViewNode::handleSwitchView(vision::SetViewMode::Request& req, vision::SetViewMode::Response& res) {
    int mode = req.view_mode;
    
    // Ensure mode is valid
    if (ViewControl::isValidMode(mode)) {
        // Check if we're switching to a new mode
        if (view_control_->getCurrentMode() != mode) {
            int old_mode = view_control_->getCurrentMode();
            if (view_control_->switchViewMode(mode)) {
                ROS_INFO("Switched from mode %d to mode %d (0=left, 1=right)", old_mode, view_control_->getCurrentMode());
                
                // Publish multiple times to ensure receivers get the message
                for (int i = 0; i < 5; ++i) {
                    publishCurrentMode(true);
                    ros::Duration(0.1).sleep();
                }
            }
        } else {
            ROS_INFO("Already in mode %d, no change needed", view_control_->getCurrentMode());
        }
        
        res.success = true;
        res.message = "Switched to view mode " + std::to_string(view_control_->getCurrentMode());
    } else {
        res.success = false;
        res.message = "Invalid view mode. Use 0 (left) or 1 (right)";
    }
    
    return true;
}

} // namespace vision

int main(int argc, char** argv) {
    ros::init(argc, argv, "view_node");
    ros::NodeHandle nh("~");
    
    vision::ViewNode node(nh);
    
    if (!node.init()) {
        ROS_ERROR("Failed to initialize view node");
        return 1;
    }
    
    ROS_INFO("View node running");
    node.run();
    
    return 0;
} 