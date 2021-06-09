import rospy
import random
import actionlib

from actionlib_msgs.msg import *                                
from geometry_msgs.msg import Pose, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class PatrolNav():
    def __init__(self):
        
        # Node
        rospy.init_node('my_navigation', anonymous=False)

        rospy.on_shutdown(self.shutdown)

        self.mark = dict()  
        self.mark['PS1']   = Pose(Point(-0.813554650391, -0.125884258569, 0.000), Quaternion(0.000, 0.000, -0.112075025675, 0.993699747721))
        self.mark['PS12']   = Pose(Point(-0.258983309869, -1.00222148807, 0.000), Quaternion(0.000, 0.000, -0.75837232922, 0.651821609241))
        self.mark['PS2']   = Pose(Point(-1.440362, -4.16325144566, 0.00), Quaternion(0.000, 0.000, 0.651087045511, 0.759003069275))
        self.mark['PS23']   = Pose(Point(-1.50632063294, -3.09276019636, 0.00), Quaternion(0.000, 0.000, 0.701538174125, 0.712631875687))
        self.mark['SS1']   = Pose(Point(-1.18030542236, -2.12698139387, 0.00), Quaternion(0.000, 0.000, 0.642292264612, 0.766459814223))
        self.mark['PS3']   = Pose(Point(2.607, -4.697, 0.00), Quaternion(0.000, 0.000, 0.6444, 0.7646))
        self.mark['SS2']   = Pose(Point(1.76455783959, -4.46939177689, 0.00), Quaternion(0.000, 0.000, 0.700398770394, 0.713751751263))
        self.mark['PS4']   = Pose(Point(3.15, -0.720, 0.000), Quaternion(0.000, 0.000, -0.751, 0.66))
        self.mark['five']   = Pose(Point(3.15, -0.720, 0.000), Quaternion(0.000, 0.000, -0.751, 0.66))
        self.mark['six']   = Pose(Point(2.607, -4.697, 0.00), Quaternion(0.000, 0.000, 0.6444, 0.7646))

        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(30))
        rospy.loginfo("Connected to move base server")

        n_successes  = 0
        id   = 0
        location   = ""
        start_time = rospy.Time.now()
        mark_cnt = len(self.mark)
        sequeue = ['PS12', 'PS2', 'PS23', 'SS1', 'PS3', 'SS2', 'PS4']
         
        rospy.loginfo("Starting position navigation ")
        # Begin the main loop and run through a sequence of mark
        while not rospy.is_shutdown():

            location = sequeue[id]

            rospy.loginfo("id_value:"+str(id)) #

            rospy.loginfo("Going to: " + str(location))
            self.send_goal(location)
            
            finished_within_time = self.move_base.wait_for_result(rospy.Duration(120))
            if not finished_within_time:
                self.move_base.cancel_goal()
                rospy.logerr("ERROR:Timed out achieving goal")
            else:
                state = self.move_base.get_state()

            if location == 'SS1' or location == 'SS2':
                rospy.sleep(0.3)
            elif location == 'PS2' or location == 'PS3':
                rospy.sleep(3)
            id += 1
            if id > 6:
                rospy.loginfo("End")
                break
                id = 0
            

    def send_goal(self, locate):
        self.goal = MoveBaseGoal() 
        self.goal.target_pose.pose = self.mark[locate]
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.header.stamp = rospy.Time.now()
        self.move_base.send_goal(self.goal) #send goal to move_base


    def shutdown(self):
        rospy.logwarn("Stopping navigation")

if __name__ == '__main__':
    try:
        PatrolNav()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logwarn("Navigation exception finished.")