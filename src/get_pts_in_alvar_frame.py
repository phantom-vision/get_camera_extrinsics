#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import position,pose
import ar_track_alvar_msgs

pts_in_alvar_frame_X = 0.0
pts_in_alvar_frame_Y = 0.0
pts_in_alvar_frame_Z = 0.0

def callback(msg):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    global pts_in_alvar_frame_X = msg.pose.pose.position.x
    global pts_in_alvar_frame_Y = msg.pose.pose.position.y
    global pts_in_alvar_frame_Z = msg.pose.pose.position.z
    rospy.loginfo("I heard ", pts_in_alvar_frame_X)
    
def listener():
    rospy.init_node('get_pts_in_alvar_frame', anonymous=True)

    rospy.Subscriber("ar_pose_marker", AlvarMarker, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()