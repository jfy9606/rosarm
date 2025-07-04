#!/usr/bin/env python3
import rospy
import time
from stereo_vision.srv import SetViewMode

def test_view_modes():
    """Test script to cycle through view modes"""
    rospy.init_node('view_mode_tester')
    
    # Wait for service
    rospy.loginfo("Waiting for view mode service...")
    rospy.wait_for_service('/stereo_vision/switch_view')
    
    try:
        # Get service proxy
        switch_view = rospy.ServiceProxy('/stereo_vision/switch_view', SetViewMode)
        
        # Cycle through view modes
        while not rospy.is_shutdown():
            for mode in range(3):  # 0=left, 1=right, 2=depth
                rospy.loginfo(f"Switching to view mode {mode}")
                response = switch_view(mode)
                rospy.loginfo(f"Response: {response.success}, {response.message}")
                time.sleep(3)  # Wait 3 seconds before switching to the next mode
    
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

if __name__ == '__main__':
    try:
        test_view_modes()
    except rospy.ROSInterruptException:
        pass 