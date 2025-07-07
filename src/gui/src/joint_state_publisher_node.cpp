#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <vector>
#include <string>
#include <cmath>

class JointStatePublisher {
public:
    JointStatePublisher() {
        ros::NodeHandle nh;
        
        // Initialize joint values - currently only four joints are available, others are not deployed
        // Order: 1=pitch motor, 2=linear feed motor, 3=rotation motor, 4=second pitch motor, 5&6=not deployed
        current_joint_positions_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        current_joint_velocities_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        
        // Create publisher
        joint_state_pub_ = nh.advertise<sensor_msgs::JointState>("/arm1/joint_states", 10);
        
        // Create subscribers to receive motor position information for initialization
        motor1_sub_ = nh.subscribe("/motor1/position", 10, &JointStatePublisher::motor1Callback, this);
        motor2_sub_ = nh.subscribe("/motor2/position", 10, &JointStatePublisher::motor2Callback, this);
        motor3_sub_ = nh.subscribe("/motor3/position", 10, &JointStatePublisher::motor3Callback, this);
        motor4_sub_ = nh.subscribe("/motor4/position", 10, &JointStatePublisher::motor4Callback, this);
        
        // Motors 5 and 6 are not deployed, but interfaces are reserved for future expansion
        motor5_sub_ = nh.subscribe("/motor5/position", 10, &JointStatePublisher::motor5Callback, this);
        motor6_sub_ = nh.subscribe("/motor6/position", 10, &JointStatePublisher::motor6Callback, this);
        
        // Subscribe to joint command topic
        joint_command_sub_ = nh.subscribe("/arm1/joint_command", 10, &JointStatePublisher::jointCommandCallback, this);
        
        // Start timer to publish joint states periodically
        timer_ = nh.createTimer(ros::Duration(0.1), &JointStatePublisher::timerCallback, this);  // 10Hz
        
        // Publish initial joint state immediately to ensure GUI can get data
        publishJointState();
        
        ROS_INFO("Joint State Publisher initialized, 4 joints available (1-4), joints 5-6 not deployed");
    }
    
    // Motor callback function - first pitch joint
    void motor1Callback(const std_msgs::Int32::ConstPtr& msg) {
        // Convert motor position to joint angle (radians)
        // Motor position range is typically 800-2200
        float pos = mapValue(msg->data, 800, 2200, -M_PI/2, M_PI/2);
        current_joint_positions_[0] = pos;
    }
    
    // Linear feed motor
    void motor2Callback(const std_msgs::Int32::ConstPtr& msg) {
        // Convert motor position to extension length (cm)
        float pos = mapValue(msg->data, 800, 2200, 0, 43);
        current_joint_positions_[1] = pos;
    }
    
    // Rotation motor
    void motor3Callback(const std_msgs::Int32::ConstPtr& msg) {
        float pos = mapValue(msg->data, 800, 2200, -M_PI, M_PI);
        current_joint_positions_[2] = pos;
    }
    
    // Second pitch motor
    void motor4Callback(const std_msgs::Int32::ConstPtr& msg) {
        float pos = mapValue(msg->data, 800, 2200, -M_PI/2, M_PI/2);
        current_joint_positions_[3] = pos;
    }
    
    // Undeployed joint 5
    void motor5Callback(const std_msgs::Int32::ConstPtr& msg) {
        // This joint is not deployed, keep default value
        current_joint_positions_[4] = 0;
    }
    
    // Undeployed joint 6
    void motor6Callback(const std_msgs::Int32::ConstPtr& msg) {
        // This joint is not deployed, keep default value
        current_joint_positions_[5] = 0;
    }
    
    // Process joint control commands
    void jointCommandCallback(const sensor_msgs::JointState::ConstPtr& msg) {
        // Update joint positions
        if (msg->position.size() >= 4) {  // At least need values for 4 available joints
            for (size_t i = 0; i < 4 && i < msg->position.size(); i++) {
                current_joint_positions_[i] = msg->position[i];
            }
        }
        
        // If velocity information is available, update it as well
        if (msg->velocity.size() >= 4) {
            for (size_t i = 0; i < 4 && i < msg->velocity.size(); i++) {
                current_joint_velocities_[i] = msg->velocity[i];
            }
        }
    }
    
    // Add a new function to encapsulate joint state publishing logic
    void publishJointState() {
        sensor_msgs::JointState joint_state;
        
        // Set timestamp and frame ID
        joint_state.header.stamp = ros::Time::now();
        joint_state.header.frame_id = "arm1_base_link";
        
        // Set joint names
        joint_state.name = {
            "arm1_joint1",  // Pitch motor
            "arm1_joint2",  // Linear feed motor
            "arm1_joint3",  // Rotation motor
            "arm1_joint4",  // Second pitch motor
            "arm1_joint5",  // Not deployed
            "arm1_joint6"   // Not deployed
        };
        
        // Set joint positions
        joint_state.position = current_joint_positions_;
        
        // Set joint velocities
        joint_state.velocity = current_joint_velocities_;
        
        // Publish joint state
        joint_state_pub_.publish(joint_state);
    }
    
    // Timer callback to publish current joint state
    void timerCallback(const ros::TimerEvent&) {
        publishJointState();
    }
    
private:
    ros::Publisher joint_state_pub_;
    ros::Subscriber motor1_sub_;
    ros::Subscriber motor2_sub_;
    ros::Subscriber motor3_sub_;
    ros::Subscriber motor4_sub_;
    ros::Subscriber motor5_sub_;
    ros::Subscriber motor6_sub_;
    ros::Subscriber joint_command_sub_;
    ros::Timer timer_;
    
    std::vector<double> current_joint_positions_;
    std::vector<double> current_joint_velocities_;
    
    // Mapping function to map a value from one range to another
    float mapValue(float value, float fromLow, float fromHigh, float toLow, float toHigh) {
        return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "joint_state_publisher");
    
    JointStatePublisher joint_state_publisher;
    
    ros::spin();
    return 0;
} 