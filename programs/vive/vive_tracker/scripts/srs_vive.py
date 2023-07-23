import time
import sys
import openvr
import math
import numpy

from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseWithCovarianceStamped
from sensor_msgs.msg import Joy
#from tf import TransForm

###############################
# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R) :
    Rt = numpy.transpose(R)
    shouldBeIdentity = numpy.dot(Rt, R)
    I = numpy.identity(3, dtype = R.dtype)
    n = numpy.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6

def quaternion_from_matrix(matrix, isprecise=False):
    """Return quaternion from rotation matrix.

    If isprecise is True, the input matrix is assumed to be a precise rotation
    matrix and a faster algorithm is used.

    """
    M = numpy.array(matrix, dtype=numpy.float64, copy=False)[:4, :4]
    if isprecise:
        q = numpy.empty((4, ))
        t = numpy.trace(M)
        if t > M[3, 3]:
            q[0] = t
            q[3] = M[1, 0] - M[0, 1]
            q[2] = M[0, 2] - M[2, 0]
            q[1] = M[2, 1] - M[1, 2]
        else:
            i, j, k = 0, 1, 2
            if M[1, 1] > M[0, 0]:
                i, j, k = 1, 2, 0
            if M[2, 2] > M[i, i]:
                i, j, k = 2, 0, 1
            t = M[i, i] - (M[j, j] + M[k, k]) + M[3, 3]
            q[i] = t
            q[j] = M[i, j] + M[j, i]
            q[k] = M[k, i] + M[i, k]
            q[3] = M[k, j] - M[j, k]
            q = q[[3, 0, 1, 2]]
        q *= 0.5 / math.sqrt(t * M[3, 3])
    else:
        m00 = M[0, 0]
        m01 = M[0, 1]
        m02 = M[0, 2]
        m10 = M[1, 0]
        m11 = M[1, 1]
        m12 = M[1, 2]
        m20 = M[2, 0]
        m21 = M[2, 1]
        m22 = M[2, 2]
        # symmetric matrix K
        K = numpy.array([[m00-m11-m22, 0.0,         0.0,         0.0],
                         [m01+m10,     m11-m00-m22, 0.0,         0.0],
                         [m02+m20,     m12+m21,     m22-m00-m11, 0.0],
                         [m21-m12,     m02-m20,     m10-m01,     m00+m11+m22]])
        K /= 3.0
        # quaternion is eigenvector of K that corresponds to largest eigenvalue
        w, V = numpy.linalg.eigh(K)
        q = V[[3, 0, 1, 2], numpy.argmax(w)]
    if q[0] < 0.0:
        numpy.negative(q, q)
    return q

# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R) :

    #assert(isRotationMatrix(R))

    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])

    singular = sy < 1e-6

    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0

    return numpy.array([x, y, z])



#Convert the standard 3x4 position/rotation matrix to a x,y,z location and the appropriate Euler angles (in degrees)
def convert_to_euler(pose_mat):
    R = [[0 for x in range(3)] for y in range(3)]
    for i in range(0,3):
        for j in range(0,3):
            R[i][j] = pose_mat[i][j]
    [xrot,yrot,zrot] = rotationMatrixToEulerAngles(numpy.asarray(R))
    x = pose_mat[0][3]
    y = pose_mat[1][3]
    z = pose_mat[2][3]

def rotationMatrixToEulerAngles(R) :

    #assert(isRotationMatrix(R))

    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])

    singular = sy < 1e-6

    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0

    return numpy.array([x, y, z])

    return [x,y,z,xrot,yrot,zrot]

#Convert the standard 3x4 position/rotation matrix to a x,y,z location and the appropriate Quaternion
def convert_to_quaternion(pose_mat):
    R = [[0 for x in range(3)] for y in range(3)]
    for i in range(0,3):
        for j in range(0,3):
            R[i][j] = pose_mat[i][j]
    [r_x, r_y, r_z, r_w] = quaternion_from_matrix(R)
    x = pose_mat[0][3]
    y = pose_mat[1][3]
    z = pose_mat[2][3]
    # Hmm, don't know why it's this order, but it's what works for ros tfs
    return [x,y,z,r_y,r_z,r_w,r_x]
###############################


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
        self.device_class=0
        self.result=False
        self.serial=""
        self.model=""
        self.pose=Pose()
        self.velocity=Twist()
        self.joy=Joy()

    def __init__(self, vr, index, tracking_pose):
        device_class=vr.getTrackedDeviceClass(index)
        if (device_class == openvr.TrackedDeviceClass_Controller):
            self.valid=True
            self.device_class=device_class
            self.model=vr.getStringTrackedDeviceProperty(index,openvr.Prop_ModelNumber_String).decode('utf-8')
            self.serial=vr.getStringTrackedDeviceProperty(index,openvr.Prop_SerialNumber_String).decode('utf-8')

            result, joy_state =vr.getControllerState(index)

            if tracking_pose.eTrackingResult==200:#vr.ETrackingResult.Running_OK
                self.result=True
                self.pose=Pose()
                tmp_pose=convert_to_quaternion(tracking_pose.mDeviceToAbsoluteTracking)
                self.pose.position.x=tmp_pose[0]
                self.pose.position.y=tmp_pose[1]
                self.pose.position.z=tmp_pose[2]
                #self.pose.position.x=tracking_pose.mDeviceToAbsoluteTracking[0][3]
                #self.pose.position.y=tracking_pose.mDeviceToAbsoluteTracking[1][3]
                #self.pose.position.z=tracking_pose.mDeviceToAbsoluteTracking[2][3]
                self.pose.orientation.x=tmp_pose[3]
                self.pose.orientation.y=tmp_pose[4]
                self.pose.orientation.z=tmp_pose[5]
                self.pose.orientation.w=tmp_pose[6]
            else:
                self.result=False
                self.pose=False
        else:
            self.valid=False


'''
                if (device_class == openvr.TrackedDeviceClass_Controller):
                    device0=vive_class()
                    device_name = "controller_"+str(len(self.object_names["Controller"])+1)
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
    

        
class triad_openvr():
    def __init__(self):
        # Initialize OpenVR in the 
        self.vr = openvr.init(openvr.VRApplication_Other)
        
        # Initializing object to hold indexes for various tracked objects 
        self.object_names = {"Tracking Reference":[],"HMD":[],"Controller":[],"Tracker":[]}
        self.devices = {}
        poses = self.vr.getDeviceToAbsoluteTrackingPose(openvr.TrackingUniverseStanding, 0,
                                                               openvr.k_unMaxTrackedDeviceCount)
        device_dict={}
        # Iterate through the pose list to find the active devices and determine their type
        for i in range(openvr.k_unMaxTrackedDeviceCount):
            device0=vive_class(self.vr, i, poses[i])
            if(device0.valid):
                #print(device0.model, device0.serial, device0.result, device0.pose)
                device_dict[device0.serial]=device0

        if "LHR-FFF91D43" in device_dict:
    	    print(device_dict["LHR-FFF91D43"].pose)

'''
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
'''
        for i in range(8):
            b=self.vr.getStringTrackedDeviceProperty(i,openvr.Prop_SerialNumber_String).decode('utf-8')
            print(b)        
'''


#main
for i in range(100):
    vr=triad_openvr()
    time.sleep(0.2)



