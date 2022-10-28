import rospy
from sensor_msgs.msg import Image,CameraInfo
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped, PoseStamped
from sensor_msgs.msg import PointCloud2, PointField
import std_msgs.msg as std_msgs
from cv_bridge import CvBridge
import airsim
from airsim import Vector3r
import cv2
import numpy as np
from nav_msgs.msg import Odometry
import time
import argparse
import os
from utils.Object_Labeling import ObjectLabeling, SegHelper
from utils.dict_indexing import Dict_indexing
from utils.kinect_publisher import KinectPublisher
from utils.Change_Labeling import ChangeLabeling
from utils.geo_transform import rotateMultirotor

parser = argparse.ArgumentParser(description="RGBD ROS Publish options")
parser.add_argument("--map_id",
                    type=str,
                    default="Bunker")
parser.add_argument("--noSeg",
                    action='store_true')
parser.add_argument("--client_type",
                    type=str,
                    help="computervision, multirotor, or car",
                    default="multirotor",
                    choices=["computervision", "multirotor", "car"])
parser.add_argument("--img_size",
                    type=str,
                    help="img size",
                    default=(640,480))
parser.add_argument("--publish_odom",
                    action='store_true')
parser.add_argument("--publish_lidar",
                    action='store_true')
parser.add_argument("--publish_imu",
                    action='store_true')
parser.add_argument("--idx2color_path",
                    type=str,
                    default='utils/idx2color.txt')
parser.add_argument("--debug",
                    action='store_true')
parser.add_argument("--pitch_degree",
                    type=str,
                    help="pitch of multirotor pose in degree, (-45 or 0)",
                    default='0')
parser.add_argument("--change_labeling",
                    action='store_true')
parser.add_argument("--enable_dust",
                    action='store_true')
if __name__ == "__main__":
    opt= parser.parse_args()
    # Init Client.
    if opt.client_type == 'multirotor':
        client = airsim.MultirotorClient()
        client.confirmConnection()
        client.enableApiControl(True)
        client.armDisarm(True)
        if opt.enable_dust:
            client.simEnableWeather(True)
            client.simSetWeatherParameter(airsim.WeatherParameter.Dust, 0.4)
            client.simSetWeatherParameter(airsim.WeatherParameter.Fog, 0.4)
    elif opt.client_type == 'computervision':
        client = airsim.VehicleClient()
        client.confirmConnection()
    elif opt.client_type == 'car':
        client = airsim.CarClient()
        client.confirmConnection()
        client.enableApiControl(False)
        client.armDisarm(True)
    else:
        raise ValueError

    name2idx_dict= Dict_indexing()
    ObjectLabeling(client=client,map_id=opt.map_id,name2idx=name2idx_dict)
    if opt.change_labeling: ChangeLabeling(client=client, name2idx = name2idx_dict)
    rospy.init_node('airsim_publisher', anonymous=True)
    publisher_d = rospy.Publisher('/camera/depth_registered/image_raw', Image, queue_size=1)
    publisher_rgb = rospy.Publisher('/camera/rgb/image_rect_color', Image, queue_size=1)
    publisher_info = rospy.Publisher('/camera/rgb/camera_info', CameraInfo, queue_size=1)
    publisher_tf = rospy.Publisher('/tf', TFMessage, queue_size=1)
    if not opt.noSeg:
        publisher_seg = rospy.Publisher('/camera/seg/segmentation_gt', Image, queue_size=1)
    if opt.publish_lidar:
        publisher_lidar = rospy.Publisher('/lidar', PointCloud2, queue_size=1)
    if opt.publish_odom:
        publisher_odom = rospy.Publisher('/odom', Odometry, queue_size=1)

    rate = rospy.Rate(30)  # 30hz
    pub = KinectPublisher(opt)
    seg_helper = SegHelper(opt,idx2color_path=opt.idx2color_path,num_class=32)
    prevTime = time.time()
    while not rospy.is_shutdown():
        try:
            if not opt.noSeg:
                responses = client.simGetImages([
                    airsim.ImageRequest("front_left_custom", airsim.ImageType.DepthPlanner, True, False), # DEPTH
                    airsim.ImageRequest("front_left_custom", airsim.ImageType.Scene, False, False), # RGB
                    airsim.ImageRequest("front_left_custom", airsim.ImageType.Segmentation,False, False) # SEGMENTATION
                ])
            else:
                responses = client.simGetImages([
                airsim.ImageRequest("front_left_custom", airsim.ImageType.DepthPlanner, True, False), # DEPTH
                airsim.ImageRequest("front_left_custom", airsim.ImageType.Scene, False, False) # RGB
                ])
            img_depth = pub.getDepthImage(responses[0])
            img_rgb = pub.getRGBImage(responses[1])
            if not opt.noSeg:
                img_seg = pub.getRGBImage(responses[2])
            if opt.publish_lidar:
                lidarData = client.getLidarData()

            if opt.debug and not opt.noSeg:
                classes_idx, classes_str = seg_helper.extract_class_from_seg(img_seg)
                for idx,str in zip(classes_idx,classes_str):
                    print("{}:{}".format(idx,str))

            pub.GetCurrentTime()
            msg_rgb = pub.CreateRGBMessage(img_rgb)
            msg_d = pub.CreateDMessage(img_depth)
            msg_info = pub.CreateInfoMessage()
            if not opt.noSeg:
                msg_seg = pub.CreateSegMessage(img_seg)
            if opt.publish_odom:
                sim_pose_msg = pub.getPose(client.getMultirotorState())
            else: sim_pose_msg = None
            msg_tf = pub.CreateTFMessage(sim_pose_msg)

            publisher_rgb.publish(msg_rgb)
            publisher_d.publish(msg_d)
            publisher_info.publish(msg_info)
            publisher_tf.publish(msg_tf)
            if not opt.noSeg:
                publisher_seg.publish(msg_seg)
            if opt.publish_lidar:
                msg_lidar = pub.npLidar2rosMessage(lidarData,frame_id="lidar_frame")
                publisher_lidar.publish(msg_lidar)
            if opt.publish_odom:
                msg_odom = pub.CreateOdomMessage(sim_pose_msg)
                publisher_odom.publish(msg_odom)

            del pub.msg_info.D[:]
            del pub.msg_tf.transforms[:]

            # mesure FPS
            currTime = time.time()
            elapsedTime = currTime - prevTime
            prevTime = currTime
            fps = 1/elapsedTime
            print("RGBD Publish sec:{0}, fps:{1}".format(elapsedTime,fps))

            rate.sleep()
        except ValueError as e:
            print(ValueError)
        except ZeroDivisionError as e:
            print(ZeroDivisionError)
        except KeyboardInterrupt:
            exit()








