import sys
import rospy
import os
from ar_track_alvar_msgs.msg import AlvarMarkers
from std_msgs.msg import String
from sound_play.msg import SoundRequest
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from sound_play.libsoundplay import SoundClient

def callback(data):
    id = data.markers[0].id
    distance = data.markers[0].pose.pose.position.z
    rospy.loginfo('I heard id=%d and distance=%f', id, distance)

    soundhandle = SoundClient()
    rospy.sleep(0.5)
    if id < 18 and id > 0 and distance < 1.2:
        rospy.loginfo('Playing sound %d.', id)
        for k in range(id):
            soundhandle.play(1, 1)
            rospy.sleep(0.5)
        rospy.sleep(16)

    rospy.sleep(0.5)

def listener():
    rospy.init_node('listener', anonymous=True)

    rospy.wait_for_message('ar_pose_marker', AlvarMarkers)

    rospy.Subscriber('ar_pose_marker', AlvarMarkers, callback, queue_size=1)

    rospy.spin()

if __name__ == '__main__':
    listener()
