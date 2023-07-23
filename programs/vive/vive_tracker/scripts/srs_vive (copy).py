import time
import sys
import openvr
import math
import numpy

from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseWithCovarianceStamped
from sensor_msgs.msg import Joy



class vr_tracked_device():
    def __init__(self,vr_obj,index,device_class):
        self.device_class = device_class
        self.index = index
        self.vr = vr_obj
        
    def get_serial(self):
        return self.vr.getStringTrackedDeviceProperty(self.index,openvr.Prop_SerialNumber_String).decode('utf-8')
    
    def get_model(self):
        return self.vr.getStringTrackedDeviceProperty(self.index,openvr.Prop_ModelNumber_String).decode('utf-8')
        
    def sample(self,num_samples,sample_rate):
        interval = 1/sample_rate
        rtn = pose_sample_buffer()
        sample_start = time.time()
        for i in range(num_samples):
            start = time.time()
            pose = self.vr.getDeviceToAbsoluteTrackingPose(openvr.TrackingUniverseStanding, 0,openvr.k_unMaxTrackedDeviceCount)
            rtn.append(pose[self.index].mDeviceToAbsoluteTracking,time.time()-sample_start)
            sleep_time = interval- (time.time()-start)
            if sleep_time>0:
                time.sleep(sleep_time)
        return rtn
        
    def get_pose_euler(self):
        pose = self.vr.getDeviceToAbsoluteTrackingPose(openvr.TrackingUniverseStanding, 0,openvr.k_unMaxTrackedDeviceCount)
        return convert_to_euler(pose[self.index].mDeviceToAbsoluteTracking)
    
    def get_pose_quaternion(self):
        pose = self.vr.getDeviceToAbsoluteTrackingPose(openvr.TrackingUniverseStanding, 0,openvr.k_unMaxTrackedDeviceCount)
        return convert_to_quaternion(pose[self.index].mDeviceToAbsoluteTracking)

    def get_pose_matrix(self):
        pose = self.vr.getDeviceToAbsoluteTrackingPose(openvr.TrackingUniverseStanding, 0,openvr.k_unMaxTrackedDeviceCount)
        return pose[self.index].mDeviceToAbsoluteTracking

    def get_velocities(self):
        pose = self.vr.getDeviceToAbsoluteTrackingPose(openvr.TrackingUniverseStanding, 0,openvr.k_unMaxTrackedDeviceCount)
        [v_x, v_y, v_z] = pose[self.index].vVelocity
        [a_x, a_y, a_z] = pose[self.index].vAngularVelocity 
        return [v_x, v_y, v_z, a_x, a_y, a_z]

    def get_state(self):
        result, state = self.vr.getControllerState(self.index)
        return state

    def is_connected(self):
        tracking = self.vr.isTrackedDeviceConnected(self.index)
        return tracking

class vive_class():
    def __init__(self):
        self.valid=False
        self.tracked=False
        self.serial=""
        self.model=""
        self.pose=Pose()
        self.velocity=Twist()
        self.joy=Joy()
    

        
class triad_openvr():
    def __init__(self):
        # Initialize OpenVR in the 
        self.vr = openvr.init(openvr.VRApplication_Other)
        
        # Initializing object to hold indexes for various tracked objects 
        self.object_names = {"Tracking Reference":[],"HMD":[],"Controller":[],"Tracker":[]}
        self.devices = {}
        poses = self.vr.getDeviceToAbsoluteTrackingPose(openvr.TrackingUniverseStanding, 0,
                                                               openvr.k_unMaxTrackedDeviceCount)
        # Iterate through the pose list to find the active devices and determine their type
        for i in range(openvr.k_unMaxTrackedDeviceCount):
            if poses[i].bPoseIsValid or True:
                device_class = self.vr.getTrackedDeviceClass(i)
                if (device_class == openvr.TrackedDeviceClass_Controller):
                    device0=vive_class()
                    device_name = "controller_"+str(len(self.object_names["Controller"])+1)
                    print(device_name)
                    #print(vr_tracked_device(self.vr,i,"Controller").get_serial())
                    print(self.vr.getStringTrackedDeviceProperty(i,openvr.Prop_SerialNumber_String).decode('utf-8'))
                    #print(vr_tracked_device(self.vr,i,"Controller").get_model())
                    print(self.vr.getStringTrackedDeviceProperty(i,openvr.Prop_ModelNumber_String).decode('utf-8'))
                    print(vr_tracked_device(self.vr,i,"Controller").get_state())
                    #print(self.vr.getControllerState(i))
                    print(poses[i].mDeviceToAbsoluteTracking[0][3])
                    print(poses[i].mDeviceToAbsoluteTracking[1][3])
                    print(poses[i].mDeviceToAbsoluteTracking[2][3])
                    print(poses[i].eTrackingResult)
                    #print(dir(poses[i]))


                    #print(dir(vr_tracked_device(self.vr,i,"Controller").get_state()))
                    print("")
                elif (device_class == openvr.TrackedDeviceClass_HMD):
                    device_name = "hmd_"+str(len(self.object_names["HMD"])+1)
                    print(device_name)
                elif (device_class == openvr.TrackedDeviceClass_GenericTracker):
                    device_name = "tracker_"+str(len(self.object_names["Tracker"])+1)
                    print(device_name)
                elif (device_class == openvr.TrackedDeviceClass_TrackingReference):
                    device_name = "tracking_reference_"+str(len(self.object_names["Tracking Reference"])+1)
                    print(device_name)
'''
        for i in range(8):
            b=self.vr.getStringTrackedDeviceProperty(i,openvr.Prop_SerialNumber_String).decode('utf-8')
            print(b)        
'''


#main
vr=triad_openvr()
print(vr)



