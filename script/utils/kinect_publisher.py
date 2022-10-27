import rospy
from sensor_msgs.msg import Image,CameraInfo
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped, PoseStamped
from sensor_msgs.msg import PointCloud2, PointField
import std_msgs.msg as std_msgs
from cv_bridge import CvBridge
import airsim
import cv2
import numpy as np
from nav_msgs.msg import Odometry
import time
import argparse
import os


CLAHE_ENABLED = False  # when enabled, RGB image is enhanced using CLAHE

CAMERA_K1 = 0.0 # -0.000591
CAMERA_K2 = 0.0 # 0.000519
CAMERA_P1 = 0.0 # 0.000001
CAMERA_P2 = 0.0 # -0.000030
CAMERA_P3 = 0.0

# How to get intrinsic matrix in Unreal Engine
# Intrinsic Matrix:
# K = [[fx, 0, Cu],
#      [0, fy, Cv],
#      [0,  0,  1]]
#
# fx=fy=f=imageWidth /(2 * tan(CameraFOV * pi / 360))
# Cu=image horizontal center = imageWidth/2
# Cv=image vertical center = imageHight/2

class KinectPublisher:
    def __init__(self,opt):
        self.bridge_rgb = CvBridge()
        self.msg_rgb = Image()
        self.msg_seg = Image()
        self.bridge_d = CvBridge()
        self.msg_d = Image()
        self.msg_info = CameraInfo()
        self.msg_tf = TFMessage()
        self.opt = opt
        self.img_height = False
        self.img_width = False
        self.pitch_degree = self.opt.pitch_degree
        # Prepare dataset folder to be saved
        # self.dataset_path = os.path.join('segmentation',self.opt.dataset_path, self.opt.dataset_type)
        # if not os.path.exists(self.dataset_path):
        #     os.makedirs(self.dataset_path)
        # for data_type in ('rgb', 'label'):
        #     if not os.path.exists(os.path.join(self.dataset_path, data_type)):
        #         os.makedirs(os.path.join(self.dataset_path, data_type))

    def getDepthImage(self,response_d):
        img_depth = np.array(response_d.image_data_float, dtype=np.float32)
        img_depth = img_depth.reshape(response_d.height, response_d.width)
        if not self.img_height:
            self.img_height = response_d.height
            self.img_width = response_d.width
        return img_depth

    def getRGBImage(self,response_rgb):
        img1d = np.fromstring(response_rgb.image_data_uint8, dtype=np.uint8)
        img_rgb = img1d.reshape(response_rgb.height, response_rgb.width, 3)
        img_rgb = img_rgb[..., :3][..., ::-1]
        return img_rgb

    def enhanceRGB(self,img_rgb):
        lab = cv2.cvtColor(img_rgb, cv2.COLOR_BGR2LAB)
        lab_planes = cv2.split(lab)
        clahe = cv2.createCLAHE(clipLimit=2.5, tileGridSize=(10, 10))
        lab_planes[0] = clahe.apply(lab_planes[0])
        lab = cv2.merge(lab_planes)
        img_rgb = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)
        return img_rgb

    def GetCurrentTime(self):
        self.ros_time = rospy.Time.now()

    def CreateRGBMessage(self,img_rgb):
        self.msg_rgb.header.stamp = self.ros_time
        self.msg_rgb.header.frame_id = "camera_rgb_optical_frame"
        self.msg_rgb.encoding = "rgb8"
        self.msg_rgb.height = self.img_height
        self.msg_rgb.width = self.img_width
        self.msg_rgb.data = self.bridge_rgb.cv2_to_imgmsg(img_rgb, "rgb8").data
        self.msg_rgb.is_bigendian = 0
        self.msg_rgb.step = self.msg_rgb.width * 3
        return self.msg_rgb

    def CreateSegMessage(self,img_rgb):
        self.msg_seg.header.stamp = self.ros_time
        self.msg_seg.header.frame_id = "camera_rgb_optical_frame"
        self.msg_seg.encoding = "rgb8"
        self.msg_seg.height = self.img_height
        self.msg_seg.width = self.img_width
        self.msg_seg.data = self.bridge_rgb.cv2_to_imgmsg(img_rgb, "rgb8").data
        self.msg_seg.is_bigendian = 0
        self.msg_seg.step = self.msg_seg.width * 3
        return self.msg_seg

    def CreateDMessage(self,img_depth):
        self.msg_d.header.stamp = self.ros_time
        self.msg_d.header.frame_id = "camera_depth_optical_frame"
        self.msg_d.encoding = "32FC1" #"16UC1"
        self.msg_d.height = self.img_height
        self.msg_d.width = self.img_width
        self.msg_d.data = self.bridge_d.cv2_to_imgmsg(img_depth, "32FC1").data
        self.msg_d.is_bigendian = 0
        self.msg_d.step = self.msg_d.width * 4
        return self.msg_d

    def CreateInfoMessage(self):

        self.msg_info.header.frame_id = "camera_rgb_optical_frame"
        self.msg_info.height = self.msg_rgb.height
        self.msg_info.width = self.msg_rgb.width
        self.msg_info.distortion_model = "plumb_bob"

        self.msg_info.D = []
        self.msg_info.D.append(CAMERA_K1)
        self.msg_info.D.append(CAMERA_K2)
        self.msg_info.D.append(CAMERA_P1)
        self.msg_info.D.append(CAMERA_P2)
        self.msg_info.D.append(CAMERA_P3)

        CAMERA_FX = self.img_width // 2  # 320 # focal length f = W / 2 / tan(FOV/2) = W/2
        CAMERA_FY = self.img_width // 2  # 320
        CAMERA_CX = self.img_width // 2  # 320
        CAMERA_CY = self.img_height // 2  # 240
        self.msg_info.K[0] = CAMERA_FX
        self.msg_info.K[1] = 0
        self.msg_info.K[2] = CAMERA_CX
        self.msg_info.K[3] = 0
        self.msg_info.K[4] = CAMERA_FY
        self.msg_info.K[5] = CAMERA_CY
        self.msg_info.K[6] = 0
        self.msg_info.K[7] = 0
        self.msg_info.K[8] = 1

        self.msg_info.R[0] = 1
        self.msg_info.R[1] = 0
        self.msg_info.R[2] = 0
        self.msg_info.R[3] = 0
        self.msg_info.R[4] = 1
        self.msg_info.R[5] = 0
        self.msg_info.R[6] = 0
        self.msg_info.R[7] = 0
        self.msg_info.R[8] = 1

        self.msg_info.P[0] = CAMERA_FX
        self.msg_info.P[1] = 0
        self.msg_info.P[2] = CAMERA_CX
        self.msg_info.P[3] = 0
        self.msg_info.P[4] = 0
        self.msg_info.P[5] = CAMERA_FY
        self.msg_info.P[6] = CAMERA_CY
        self.msg_info.P[7] = 0
        self.msg_info.P[8] = 0
        self.msg_info.P[9] = 0
        self.msg_info.P[10] = 1
        self.msg_info.P[11] = 0

        self.msg_info.binning_x = self.msg_info.binning_y = 0
        self.msg_info.roi.x_offset = self.msg_info.roi.y_offset = self.msg_info.roi.height = self.msg_info.roi.width = 0
        self.msg_info.roi.do_rectify = False
        self.msg_info.header.stamp = self.msg_rgb.header.stamp
        return self.msg_info

    def parse_lidarData(self, data):
        # reshape array of floats to array of [X,Y,Z]
        points = np.array(data.point_cloud, dtype=np.dtype('f4'))
        points = np.reshape(points, (int(points.shape[0] / 3), 3))

        return points

    def npLidar2rosMessage(self,cloud_arr, frame_id=None):
        '''Converts a numpy record array to a sensor_msgs.msg.PointCloud2.
        '''
        cloud_arr = self.parse_lidarData(cloud_arr)
        # To match coordinate
        cloud_arr[:, 2] = - cloud_arr[:,2]
        cloud_arr[:, 1] = - cloud_arr[:, 1]

        # make it 2d (even if height will be 1)
        cloud_arr = np.atleast_2d(cloud_arr)

        ros_dtype = PointField.FLOAT32
        dtype = np.float32
        itemsize = np.dtype(dtype).itemsize

        data = cloud_arr.astype(dtype).tobytes()
        header = std_msgs.Header(frame_id=frame_id, stamp=self.ros_time)

        fields = [PointField(
            name=n, offset=i * itemsize, datatype=ros_dtype, count=1)
            for i, n in enumerate('xyz')]


        cloud_msg = PointCloud2(
                        header=header,
                        height=1,
                        width=cloud_arr.shape[0],
                        is_dense=False,
                        is_bigendian=False,
                        fields=fields,
                        point_step=(itemsize * 3),
                        row_step=(itemsize * 3 * cloud_arr.shape[0]),
                        data=data)
        #import pdb; pdb.set_trace()

        return cloud_msg

    def CreateTFMessage(self,sim_pose_msg=None):
        # if self.opt.publish_odom:
        #     self.msg_tf.transforms.append(TransformStamped())
        #     self.msg_tf.transforms[-1].header.stamp = self.ros_time
        #     self.msg_tf.transforms[-1].header.frame_id = "/map"
        #     self.msg_tf.transforms[-1].child_frame_id = "/odom"
        #     self.msg_tf.transforms[-1].transform.translation.x = 0.000
        #     self.msg_tf.transforms[-1].transform.translation.y = 0.000
        #     self.msg_tf.transforms[-1].transform.translation.z = 0.000
        #     self.msg_tf.transforms[-1].transform.rotation.x = 0.000
        #     self.msg_tf.transforms[-1].transform.rotation.y = 0.000
        #     self.msg_tf.transforms[-1].transform.rotation.z = 0.000
        #     self.msg_tf.transforms[-1].transform.rotation.w = 1.000
        #     self.msg_tf.transforms.append(TransformStamped())
        #     self.msg_tf.transforms[-1].header.stamp = self.ros_time
        #     self.msg_tf.transforms[-1].header.frame_id = "/odom"
        #     self.msg_tf.transforms[-1].child_frame_id = "/camera_link"
        #     self.msg_tf.transforms[-1].transform.translation.x = sim_pose_msg.pose.position.x
        #     self.msg_tf.transforms[-1].transform.translation.y = sim_pose_msg.pose.position.y
        #     self.msg_tf.transforms[-1].transform.translation.z = sim_pose_msg.pose.position.z
        #     self.msg_tf.transforms[-1].transform.rotation.x = sim_pose_msg.pose.orientation.x
        #     self.msg_tf.transforms[-1].transform.rotation.y = sim_pose_msg.pose.orientation.y
        #     self.msg_tf.transforms[-1].transform.rotation.z = sim_pose_msg.pose.orientation.z
        #     self.msg_tf.transforms[-1].transform.rotation.w = sim_pose_msg.pose.orientation.w

        self.msg_tf.transforms.append(TransformStamped())
        self.msg_tf.transforms[-1].header.stamp = self.ros_time
        self.msg_tf.transforms[-1].header.frame_id = "/camera_link"
        self.msg_tf.transforms[-1].child_frame_id = "/camera_rgb_frame"
        self.msg_tf.transforms[-1].transform.translation.x = 0.000
        self.msg_tf.transforms[-1].transform.translation.y = 0
        self.msg_tf.transforms[-1].transform.translation.z = 0.000
        self.msg_tf.transforms[-1].transform.rotation.x = 0.00
        self.msg_tf.transforms[-1].transform.rotation.y = 0.00
        self.msg_tf.transforms[-1].transform.rotation.z = 0.00
        self.msg_tf.transforms[-1].transform.rotation.w = 1.00

        self.msg_tf.transforms.append(TransformStamped())
        self.msg_tf.transforms[-1].header.stamp = self.ros_time
        self.msg_tf.transforms[-1].header.frame_id = "/camera_rgb_frame"
        self.msg_tf.transforms[-1].child_frame_id = "/camera_rgb_optical_frame"
        self.msg_tf.transforms[-1].transform.translation.x = 0.000
        self.msg_tf.transforms[-1].transform.translation.y = 0.000
        self.msg_tf.transforms[-1].transform.translation.z = 0.000

        if self.pitch_degree =='-45':
            self.msg_tf.transforms[-1].transform.rotation.x = 0.0
            self.msg_tf.transforms[-1].transform.rotation.y = -0.9238795325112867
            self.msg_tf.transforms[-1].transform.rotation.z = 0.3826834323650898
            self.msg_tf.transforms[-1].transform.rotation.w = 0.0
        elif self.pitch_degree == '0':
            self.msg_tf.transforms[-1].transform.rotation.x = -0.500
            self.msg_tf.transforms[-1].transform.rotation.y = 0.500
            self.msg_tf.transforms[-1].transform.rotation.z = -0.500
            self.msg_tf.transforms[-1].transform.rotation.w = 0.500
        else:
            raise ValueError

        self.msg_tf.transforms.append(TransformStamped())
        self.msg_tf.transforms[-1].header.stamp = self.ros_time
        self.msg_tf.transforms[-1].header.frame_id = "/camera_link"
        self.msg_tf.transforms[-1].child_frame_id = "/camera_depth_frame"
        self.msg_tf.transforms[-1].transform.translation.x = 0
        self.msg_tf.transforms[-1].transform.translation.y = 0
        self.msg_tf.transforms[-1].transform.translation.z = 0
        self.msg_tf.transforms[-1].transform.rotation.x = 0.00
        self.msg_tf.transforms[-1].transform.rotation.y = 0.00
        self.msg_tf.transforms[-1].transform.rotation.z = 0.00
        self.msg_tf.transforms[-1].transform.rotation.w = 1.00

        self.msg_tf.transforms.append(TransformStamped())
        self.msg_tf.transforms[-1].header.stamp = self.ros_time
        self.msg_tf.transforms[-1].header.frame_id = "/camera_depth_frame"
        self.msg_tf.transforms[-1].child_frame_id = "/camera_depth_optical_frame"
        self.msg_tf.transforms[-1].transform.translation.x = 0.000
        self.msg_tf.transforms[-1].transform.translation.y = 0.000
        self.msg_tf.transforms[-1].transform.translation.z = 0.000
        if self.pitch_degree =='-45':
            self.msg_tf.transforms[-1].transform.rotation.x = 0.0
            self.msg_tf.transforms[-1].transform.rotation.y = -0.9238795325112867
            self.msg_tf.transforms[-1].transform.rotation.z = 0.3826834323650898
            self.msg_tf.transforms[-1].transform.rotation.w = 0.0
        elif self.pitch_degree == '0':
            self.msg_tf.transforms[-1].transform.rotation.x = -0.500
            self.msg_tf.transforms[-1].transform.rotation.y = 0.500
            self.msg_tf.transforms[-1].transform.rotation.z = -0.500
            self.msg_tf.transforms[-1].transform.rotation.w = 0.500
        else:
            raise ValueError

        # if self.opt.publish_lidar:
        #     self.msg_tf.transforms.append(TransformStamped())
        #     self.msg_tf.transforms[-1].header.stamp = self.ros_time
        #     self.msg_tf.transforms[-1].header.frame_id = "/camera_link"
        #     self.msg_tf.transforms[-1].child_frame_id = "/lidar_frame"
        #     self.msg_tf.transforms[-1].transform.translation.x = 0.000
        #     self.msg_tf.transforms[-1].transform.translation.y = 0.000
        #     self.msg_tf.transforms[-1].transform.translation.z = 0.000
        #     self.msg_tf.transforms[-1].transform.rotation.x = 0.000
        #     self.msg_tf.transforms[-1].transform.rotation.y = 0.000
        #     self.msg_tf.transforms[-1].transform.rotation.z = 0.000
        #     self.msg_tf.transforms[-1].transform.rotation.w = 1.000
        # if self.opt.publish_imu:
        #     self.msg_tf.transforms.append(TransformStamped())
        #     self.msg_tf.transforms[-1].header.stamp = self.ros_time
        #     self.msg_tf.transforms[-1].header.frame_id = "/camera_link"
        #     self.msg_tf.transforms[-1].child_frame_id = "/imu_frame"
        #     self.msg_tf.transforms[-1].transform.translation.x = 0.000
        #     self.msg_tf.transforms[-1].transform.translation.y = 0.000
        #     self.msg_tf.transforms[-1].transform.translation.z = 0.000
        #     self.msg_tf.transforms[-1].transform.rotation.x = 0.000
        #     self.msg_tf.transforms[-1].transform.rotation.y = 1.000
        #     self.msg_tf.transforms[-1].transform.rotation.z = 0.000
        #     self.msg_tf.transforms[-1].transform.rotation.w = 0.000

        return self.msg_tf

    def getPose(self,state):
        # get state of the car
        pos_ned = state.kinematics_estimated.position
        orientation_ned = state.kinematics_estimated.orientation
        pos, orientation = self.convert_ned_to_enu(pos_ned, orientation_ned)

        # populate PoseStamped ros message
        sim_pose_msg = PoseStamped()
        sim_pose_msg.pose.position.x = pos.x_val
        sim_pose_msg.pose.position.y = pos.y_val
        sim_pose_msg.pose.position.z = pos.z_val
        sim_pose_msg.pose.orientation.w = orientation.w_val
        sim_pose_msg.pose.orientation.x = orientation.x_val
        sim_pose_msg.pose.orientation.y = orientation.y_val
        sim_pose_msg.pose.orientation.z = orientation.z_val
        sim_pose_msg.header.seq = 1
        sim_pose_msg.header.frame_id = "odom"

        return sim_pose_msg

    def convert_ned_to_enu(self,pos_ned, orientation_ned):
        pos_enu = airsim.Vector3r(pos_ned.x_val,
                                  - pos_ned.y_val,
                                  - pos_ned.z_val)
        orientation_enu = airsim.Quaternionr(orientation_ned.w_val,
                                             - orientation_ned.z_val,
                                             - orientation_ned.x_val,
                                             orientation_ned.y_val)
        return pos_enu, orientation_ned

    def CreateOdomMessage(self,sim_pose_msg):
        # populate odom ros message
        odom_msg = Odometry()
        odom_msg.pose.pose = sim_pose_msg.pose
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "camera_link"
        odom_msg.header.stamp = self.ros_time
        return odom_msg

