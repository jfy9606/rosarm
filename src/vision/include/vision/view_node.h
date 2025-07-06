#ifndef VISION_VIEW_NODE_H
#define VISION_VIEW_NODE_H

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <memory>
#include "vision/view_control.h"
#include "vision/SetViewMode.h"

namespace vision {

/**
 * @brief View node class for handling ROS communication
 * 
 * This class implements the node part of the node+control architecture
 * for the view mode switching system.
 */
class ViewNode {
public:
    /**
     * @brief Constructor
     * 
     * @param nh ROS node handle
     */
    ViewNode(ros::NodeHandle& nh);
    
    /**
     * @brief Destructor
     */
    ~ViewNode();
    
    /**
     * @brief Initialize the node
     * 
     * @return True if initialization was successful, false otherwise
     */
    bool init();
    
    /**
     * @brief Run the node
     */
    void run();
    
    /**
     * @brief Publish the current view mode
     * 
     * @param force Force publishing even if the mode hasn't changed
     */
    void publishCurrentMode(bool force = false);
    
private:
    ros::NodeHandle nh_;
    std::unique_ptr<ViewControl> view_control_;
    ros::Publisher view_mode_pub_;
    ros::ServiceServer switch_view_service_;
    ros::ServiceServer switch_view_service_gui_;
    ros::Time last_log_time_;
    ros::Duration log_interval_;
    
    // ROS parameters
    int initial_view_mode_;
    std::string left_image_topic_;
    std::string right_image_topic_;
    
    // Service callback
    bool handleSwitchView(vision::SetViewMode::Request& req, 
                          vision::SetViewMode::Response& res);
};

} // namespace vision

#endif // VISION_VIEW_NODE_H 