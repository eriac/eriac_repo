#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, Twist, Vector3, PoseWithCovarianceStamped, Transform, TransformStamped

import openvr
from srs_vive import vive_class


def talker():
    rospy.init_node('talker', anonymous=True)
    pub1 = rospy.Publisher('LHR_FFF91D43/pose', PoseStamped, queue_size=10)
    pub2 = rospy.Publisher('LHR_8ED3A411/pose', PoseStamped, queue_size=10)



    vr = openvr.init(openvr.VRApplication_Other)

 
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        poses = vr.getDeviceToAbsoluteTrackingPose(openvr.TrackingUniverseStanding, 0, openvr.k_unMaxTrackedDeviceCount)
        device_dict={}
        # Iterate through the pose list to find the active devices and determine their type
        for i in range(openvr.k_unMaxTrackedDeviceCount):
            device0=vive_class(vr, i, poses[i])
            if(device0.valid):
                #print(device0.model, device0.serial, device0.result, device0.pose)
                device_dict[device0.serial]=device0
        print(device_dict.keys())

        if "LHR-FFF91D43" in device_dict:#controller
            if(device_dict["LHR-FFF91D43"].result):
                print(device_dict["LHR-FFF91D43"].pose)
                tmp_ps=PoseStamped()
                tmp_ps.header.frame_id="map"
                tmp_ps.header.stamp=rospy.Time.now()
                tmp_ps.pose=device_dict["LHR-FFF91D43"].pose
                pub1.publish(tmp_ps)
            else:
                print("Invalid")

        if "LHR-8ED3A411"  in device_dict:#controller
            if(device_dict["LHR-8ED3A411"].result):
                print(device_dict["LHR-8ED3A411"].pose)
                tmp_ps=PoseStamped()
                tmp_ps.header.frame_id="map"
                tmp_ps.header.stamp=rospy.Time.now()
                tmp_ps.pose=device_dict["LHR-8ED3A411"].pose
                pub2.publish(tmp_ps)
            else:
                print("Invalid")


        r.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass






