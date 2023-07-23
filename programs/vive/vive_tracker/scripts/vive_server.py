#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseWithCovarianceStamped

def talker():
    rospy.init_node('talker', anonymous=True)
    pub = rospy.Publisher('chatter', Pose, queue_size=10)

    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        tmp_pose=Pose()
        tmp_pose.orientation.w=1.0
        pub.publish(tmp_pose)
        r.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
