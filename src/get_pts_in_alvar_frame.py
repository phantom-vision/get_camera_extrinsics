#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from ar_track_alvar_msgs.msg import AlvarMarkers

    
def alvar_pose_listener():
    rospy.init_node('get_pts_in_alvar_frame', anonymous=True)

    msg = rospy.wait_for_message("ar_pose_marker", AlvarMarkers)
    try:
        pts_in_alvar_frame_X = msg.markers[0].pose.pose.position.x
        pts_in_alvar_frame_Y = msg.markers[0].pose.pose.position.y
        pts_in_alvar_frame_Z = msg.markers[0].pose.pose.position.z
        rospy.loginfo("AR tag in alvar captured \n")
        # print(pts_in_alvar_frame_X))
        # print(pts_in_alvar_frame_Y)
        # print(pts_in_alvar_frame_Z)
        return [pts_in_alvar_frame_X,pts_in_alvar_frame_Y,pts_in_alvar_frame_Z]
    except IndexError:
        rospy.loginfo(" AR tag not captured \n")
        alvar_pose_listener()

if __name__ == '__main__':
    pts_in_alvar_frame_list = alvar_pose_listener()
