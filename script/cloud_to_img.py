import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import CameraInfo
from image_geometry import PinholeCameraModel
from sensor_msgs.msg import Image
import tf
import ros_numpy
import numpy as np
#from utils.tf2_sensor_msgs import do_transform_cloud
from utils.msg_transform import msg_to_se3
import torch
import time
import matplotlib.pyplot as plt
import argparse
import message_filters
from PIL import Image as PIL_Image
import os
from utils.Object_Labeling import SegHelper
# from utils.visualization import VisOpen3D
#from open3d_ros_helper import open3d_ros_helper as orh
#import open3d as o3d
from utils.trajectory import load_trajectory
from shutil import copyfile

colormap = [[255,255,0],[255,0,255],[0,255,255],[0,0,255],[0,255,0]]
parser = argparse.ArgumentParser(description="cloud2img projection options")
parser.add_argument("--proj_method",
                    type=str,
                    help="projection method",
                    default="resize",
                    choices=["resize", "padding", "none"])
parser.add_argument("--resize_ratio",
                    type=int,
                    help="resize ratio",
                    default=4,
                    choices=[2, 4])
parser.add_argument("--device",
                    type=str,
                    help="device",
                    default="cpu",
                    choices=["cuda", "cpu"])
parser.add_argument("--save_path",
                    type=str,
                    help="folder path to be saved",
                    default='../changesim_dataset/Localization/Bunker/Room_0/Seq_0')
parser.add_argument("--idx2color_path",
                    type=str,
                    default='utils/idx2color.txt')
parser.add_argument("--publish_2d",
                    action='store_true')
parser.add_argument("--save_interval",
                    type=int,
                    help="save_interval",
                    default=7)
parser.add_argument("--save_cloud",
                    action='store_true')

class Cloud2ImgProjection:
    def __init__(self,opt):
        self.opt=opt
        self.path = self.opt.save_path
        self.tf_listener = tf.TransformListener()
        # Subscribe map once and keep it as a torch tensor format in GPU memory
        cloud_msg = rospy.wait_for_message("/rtabmap/cloud_map",PointCloud2,timeout=3.0)
        raw_cloud_array =  ros_numpy.point_cloud2.pointcloud2_to_array(cloud_msg,squeeze=False)
        splited_cloud_array = ros_numpy.point_cloud2.split_rgb_field(raw_cloud_array)
        self.pointcloud_dtype = splited_cloud_array.dtype
        nd_cloud_array = np.concatenate([splited_cloud_array[idx] for idx in ['x','y','z','r','g','b']],axis=0) # (6,num_points)
        # new_arr = np.core.records.fromarrays(nd_cloud_array,names=self.pointcloud_dtype.names)
        # new_arr = np.array(new_arr.astype(dtype=self.pointcloud_dtype).tolist(),dtype=self.pointcloud_dtype)
        num_points = nd_cloud_array.shape[1]
        raw_cloud_xyz, raw_cloud_rgb = torch.from_numpy(nd_cloud_array[:3]).to(opt.device),torch.from_numpy(nd_cloud_array[3:]).to(opt.device)
        raw_cloud_xyz1 = torch.cat([raw_cloud_xyz,torch.ones(1,num_points).to(opt.device)],dim=0)  # (4,num_points), pad 1 for transform

        self.raw_cloud_xyz1 = raw_cloud_xyz1
        self.raw_cloud_rgb = raw_cloud_rgb
        self.num_points = num_points
        self.depth_threshold = 15.0
        # Subscribe camera_info(intrinsics) once and keep it
        cam_info_msg = rospy.wait_for_message("/camera/rgb/camera_info",CameraInfo, timeout=3.0)

        self.camera_model = PinholeCameraModel()
        self.camera_model.fromCameraInfo(cam_info_msg)
        intr_matrix = self.camera_model.intrinsicMatrix() # (3,3)
        intr_matrix = torch.from_numpy(intr_matrix)[None,:].to(opt.device)
        self.intr_matrix = intr_matrix

        # Prepare empty tensor for img re-synthesis
        if opt.proj_method == 'resize':
            self.img_projected = torch.zeros(self.camera_model.height//opt.resize_ratio,
                                             self.camera_model.width//opt.resize_ratio, 3).long().to(opt.device)
        else:
            self.img_projected = torch.zeros(self.camera_model.height, self.camera_model.width, 3).long().to(opt.device)

        # Prepare pose (trajectory) for frame matching
        self.mapping_path = self.path.replace('Localization','Mapping')
        self.poses = load_trajectory(os.path.join(self.mapping_path,'raw','poses_odom.g2o'))

        # Prepare publishers
        self.pub_imgfied_cloud = rospy.Publisher("/imgfied_cloud", Image, queue_size=1)
        self.pub_current_rgb = rospy.Publisher("/current_rgb", Image, queue_size=1)
        self.pub_current_depth = rospy.Publisher("/current_depth", Image, queue_size=1)
        self.pub_current_sem_seg = rospy.Publisher("/current_sem_seg", Image, queue_size=1)
        self.pub_current_change_seg = rospy.Publisher("/current_change_seg", Image, queue_size=1)
        self.pub_marked_cloud = rospy.Publisher("/marked_cloud", PointCloud2, queue_size=1)
        self.pub_cloud_in_fov = rospy.Publisher("/cloud_in_fov", PointCloud2, queue_size=1)

        # To measure FPS
        self.prevTime=time.time()
        self.cnt = -1

        self.seg_helper = SegHelper(opt,idx2color_path=opt.idx2color_path)
        self.save_interval = self.opt.save_interval
        self.filter_ratio = 0.8
        print("data will be saved in:{0}".format(self.path))
        if not os.path.exists(self.path):
            os.makedirs(self.path)
        if not os.path.exists(self.path + '/rgb'):
            os.makedirs(self.path + '/rgb')
        if not os.path.exists(self.path + '/depth'):
            os.makedirs(self.path + '/depth')
        if not os.path.exists(self.path + '/semantic_segmentation'):
            os.makedirs(self.path + '/semantic_segmentation')
        if not os.path.exists(self.path + '/change_segmentation'):
            os.makedirs(self.path + '/change_segmentation')
        if not os.path.exists(self.path + '/cloud'):
            os.makedirs(self.path + '/cloud')
        if not os.path.exists(self.path + '/imgfied_cloud_rgb'):
            os.makedirs(self.path + '/imgfied_cloud_rgb')
        if not os.path.exists(self.path + '/imgfied_cloud_depth'):
            os.makedirs(self.path + '/imgfied_cloud_depth')
        if not os.path.exists(self.path + '/pose'):
            os.makedirs(self.path + '/pose')
        if not os.path.exists(self.path + '/t0/rgb'):
            os.makedirs(self.path + '/t0/rgb')
        if not os.path.exists(self.path + '/t0/depth'):
            os.makedirs(self.path + '/t0/depth')
        if not os.path.exists(self.path + '/t0/idx'):
            os.makedirs(self.path + '/t0/idx')

        # cloud_open3d = orh.rospc_to_o3dpc(cloud_msg)
        # o3d.io.write_point_cloud(os.path.join(self.path, 'cloud_map.ply'), cloud_open3d)

    def callback(self, rgb_msg, seg_msg, depth_msg):
        # increase counter variable
        self.cnt += 1
        if self.cnt % self.save_interval != 0:
            # measure fps
            currTime = time.time()
            self.elapsedTime = currTime - self.prevTime
            self.prevTime = currTime
            self.fps = 1/self.elapsedTime
            print("[SKIP FRAME] sec:{0}, fps:{1}".format(self.elapsedTime,self.fps))
        else:
            try:
                self.transform_msg = self.tf_listener._buffer.lookup_transform('camera_rgb_optical_frame','map', rgb_msg.header.stamp,
                                                                          timeout=rospy.Duration(0.5))
                self.inv_transform_msg = self.tf_listener._buffer.lookup_transform('map','camera_rgb_optical_frame', rgb_msg.header.stamp,
                                                                               timeout=rospy.Duration(0.5))

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print('maximum wait time exceeded!! use the last transform_msg instead.')
                pass
            if self.opt.save_cloud:
                merged_cloud_array = self._transform_cloud(self.transform_msg) # (num_points,6)
                projected, keep, cloud_in_fov = self._filter_out_of_view_points(merged_cloud_array) # (num_filtered_points, 6)

            # split semantic seg. and change seg.
            seg_array = self._img_msg2arr(seg_msg)
            seg_classmap = self.seg_helper.colormap2classmap(seg_array)
            seg_classmap, change_classmap = self.seg_helper.split_SemAndChange(seg_classmap)

            missing_mask = change_classmap == 2

            seg_classmap[missing_mask] = torch.zeros_like(seg_classmap[missing_mask])
            seg_colormap = self.seg_helper.classmap2colormap(seg_classmap)
            seg_colormap[missing_mask.squeeze()] = torch.zeros_like(seg_colormap[missing_mask.squeeze()])
            change_colormap = self.seg_helper.classmap2colormap(change_classmap)
            seg_semantic_msg = self._img_arr2msg(seg_colormap.cpu().numpy())
            seg_change_msg = self._img_arr2msg(change_colormap.cpu().numpy())
            # img_proj = self._project_points_to_img(projected).cpu().numpy() # depreciated

            #projected = torch.cat([projected[:,:3]*scale.squeeze(),projected[:,3:]],dim=1)
            if self.opt.save_cloud:
                cloud_in_fov = cloud_in_fov.t().cpu()
                cloud_in_FoV_msg = self._cloud_arr2pc_msg(cloud_in_fov.numpy(),frame_id="camera_rgb_optical_frame")
            # publish
            if self.opt.publish_2d:
                # img_msg = self._img_arr2msg(img_proj)
                # self.pub_imgfied_cloud.publish(img_msg)
                self.pub_current_rgb.publish(rgb_msg)
                self.pub_current_depth.publish(depth_msg)
                self.pub_current_sem_seg.publish(seg_semantic_msg)
                self.pub_current_change_seg.publish(seg_change_msg)
                self.pub_cloud_in_fov.publish(cloud_in_FoV_msg)

            '''
            merged_cloud_xyz, merged_cloud_rgb = merged_cloud_array[:,:3],merged_cloud_array[:,3:] # (num_points,3)
            projected_x,projected_y = projected[:,0],projected[:,1]
            change_masks = []
            for change_class,color in zip(range(1,5),colormap):
                keep_change = keep.clone()
                binary_mask = change_classmap == change_class
                change_mask = binary_mask[projected_y,projected_x]
                keep_change[keep_change == 1] = change_mask.view(-1)
                color_array = torch.ones(keep_change.sum(),3)*torch.LongTensor(color)
                merged_cloud_rgb[keep_change] = color_array.to(opt.device)
                change_masks.append(change_mask)
            change_mask = torch.cat(change_masks,dim=1).sum(dim=1).bool()
            keep[keep == 1] = change_mask.view(-1)
            #import pdb; pdb.set_trace()
            merged_cloud_array = torch.cat([merged_cloud_xyz,merged_cloud_rgb],dim=1)
            merged_cloud_array = merged_cloud_array[keep].t().cpu() # (6, num_points)
            #import pdb; pdb.set_trace()
            # new_arr = np.core.records.fromarrays(merged_cloud_array, names=self.pointcloud_dtype.names)
            # new_arr = np.array(new_arr.astype(dtype=self.pointcloud_dtype).tolist(),dtype=self.pointcloud_dtype)
            # new_arr = ros_numpy.point_cloud2.merge_rgb_fields(new_arr)
            # new_cloud_msg = ros_numpy.point_cloud2.array_to_pointcloud2(new_arr,frame_id="camera_rgb_optical_frame")
            new_cloud_msg = self._cloud_arr2pc_msg(merged_cloud_array,frame_id="camera_rgb_optical_frame")
            self.pub_marked_cloud.publish(new_cloud_msg)
            '''

            ''' save '''
            if self.cnt//self.save_interval < 6000:
                pth = self.path
                rgb_array=self._img_msg2arr(rgb_msg)
                rgb_png = PIL_Image.fromarray(rgb_array)
                rgb_png.save(pth+'/rgb/{}.png'.format(self.cnt//self.save_interval))
                depth_array = self._img_msg2arr(depth_msg)
                depth_array = np.clip(depth_array,0,50)/50*255 # Clip maximum depth and normalize
                depth_png = PIL_Image.fromarray(depth_array.astype('uint8'))
                depth_png.save(pth + '/depth/{}.png'.format(self.cnt//self.save_interval))
                sem_seg_png = PIL_Image.fromarray(seg_colormap.cpu().numpy().astype('uint8'))
                sem_seg_png.save(pth+'/semantic_segmentation/{}.png'.format(self.cnt//self.save_interval))
                change_seg_png = PIL_Image.fromarray(change_colormap.cpu().numpy().astype('uint8'))
                change_seg_png.save(pth+'/change_segmentation/{}.png'.format(self.cnt//self.save_interval))

                # projected rgb save (depreciated)
                # imgfied_cloud_png = PIL_Image.fromarray(img_proj.astype('uint8'))
                # imgfied_cloud_png.save(pth+'/imgfied_cloud/{}.png'.format(self.cnt//self.save_interval))

                # point-cloud in FOV save (*.npy, depreciated)
                # np.save(pth+'/cloud/{}.npy'.format(self.cnt//self.save_interval),cloud_in_fov.numpy())
                # point-cloud in FOV save (*.pth depreciated)
                # torch.save(cloud_in_fov,pth+'/cloud/{}.pth'.format(self.cnt//self.save_interval))
                if self.opt.save_cloud:
                    # point-cloud in FOV save (*.ply)
                    cloud_open3d = orh.rospc_to_o3dpc(cloud_in_FoV_msg)
                    o3d.io.write_point_cloud(pth + '/cloud/{}.ply'.format(self.cnt // self.save_interval), cloud_open3d)


                    # projected rgb save
                    vis = VisOpen3D(width=640, height=480, visible=False)
                    vis.add_geometry(cloud_open3d)
                    vis.load_view_point("utils/view_point.json")

                    vis.capture_screen_image(pth+'/imgfied_cloud_rgb/{}.png'.format(self.cnt//self.save_interval))

                    # vis.capture_depth_image(pth+'/imgfied_cloud_depth/{}.png'.format(self.cnt//self.save_interval))


                    # projected depth save
                    depth_np = np.asarray(vis.capture_depth_float_buffer(show=False))
                    depth_np2 = np.clip(depth_np, 0, 50) / 50 * 255
                    depth_png = PIL_Image.fromarray(depth_np2.astype('uint8'))
                    depth_png.save(pth + '/imgfied_cloud_depth/{}.png'.format(self.cnt // self.save_interval))
                    # import pdb; pdb.set_trace()
                    # imgfied_cloud_depth_png = PIL_Image.fromarray(depth_np)
                    # imgfied_cloud_depth_png.save(pth+'/imgfied_cloud_depth/{}.png',format(self.cnt//self.save_interval))
                    # vis.emove_geometry(cloud_open3d)

                    del vis
                self._save_pose()
                best_matching_idx = self._find_best_matching_frame_idx()
                self._copy_best_matching_frame(best_matching_idx)

            else:
                print("count larger than threshold: {}".format(self.cnt))

            # measure fps
            currTime = time.time()
            self.elapsedTime = currTime - self.prevTime
            self.prevTime = currTime
            self.fps = 1/self.elapsedTime
            print("[PROCESSING FRAME] sec:{0}, fps:{1}".format(self.elapsedTime,self.fps))

    def _find_best_matching_frame_idx(self):
        inds,poses = list(self.poses.keys()),np.array(list(self.poses.values()))
        l1_distance = poses - self.current_pose_numpy
        l1_distance = np.sum(np.abs(l1_distance[:,:3]),axis=1)
        best_matching_idx = inds[l1_distance.argmin()]
        print("current_pose",self.current_pose_numpy)
        print("best_pose",self.poses[best_matching_idx])
        print("best_matching_idx",best_matching_idx)

        return best_matching_idx

    def _copy_best_matching_frame(self,idx):
        src_pth = os.path.join(self.mapping_path,'raw')
        matching_rgb_png = os.path.join(src_pth,'rgb','{}.png'.format(idx))
        matching_depth_png = os.path.join(src_pth,'depth','{}.png'.format(idx))
        dst_pth = self.path
        copyfile(matching_rgb_png, os.path.join(dst_pth,'t0/rgb','{}.png'.format(self.cnt // self.save_interval)))
        copyfile(matching_depth_png, os.path.join(dst_pth,'t0/depth','{}.png'.format(self.cnt // self.save_interval)))
        tf = open(self.path+'/t0/idx/{}.txt'.format(self.cnt//self.save_interval),mode='w')
        tf.write('{}'.format(idx))
        tf.close()

    def _cloud_arr2pc_msg(self,cloud_array,frame_id="camera_rgb_optical_frame"):
        """
        param cloud_array: (6, num_points)
        return: PointCloud2 msg
        """
        new_arr = np.core.records.fromarrays(cloud_array, names=self.pointcloud_dtype.names)
        new_arr = np.array(new_arr.astype(dtype=self.pointcloud_dtype).tolist(),dtype=self.pointcloud_dtype)
        new_arr = ros_numpy.point_cloud2.merge_rgb_fields(new_arr)
        new_cloud_msg = ros_numpy.point_cloud2.array_to_pointcloud2(new_arr,frame_id=frame_id)
        return new_cloud_msg

    def _transform_cloud(self,transform_msg):
        # Transform ('map' coordinates --> 'camera_rgb_optical_frame' coordinates)
        startTime = time.time()
        transform_array = torch.FloatTensor(msg_to_se3(transform_msg)).to(opt.device)  # (4,4)
        transformed_cloud_xyz = torch.matmul(transform_array,self.raw_cloud_xyz1)[:3]  # (3,num_points)
        transformed_cloud = torch.cat([transformed_cloud_xyz,self.raw_cloud_rgb],dim=0)
        transformed_cloud = transformed_cloud.transpose(1,0)
        return transformed_cloud # (num_points,6)

    def _filter_out_of_view_points(self,cloud_array):
        #k keep only [20%] percent of points
        num_total_points = cloud_array.shape[0]
        # TODO: merge keep_random into merged_keep
        keep_random = torch.rand(num_total_points) > self.filter_ratio
        cloud_array = cloud_array[keep_random]
        # filter out out-of-view points (with respect to z-axis)
        keep_positive_z = cloud_array[:,2] > 0 #) & (cloud_array[:,2] < self.depth_threshold)
        filtered_cloud_array = cloud_array[keep_positive_z]
        # Transform ('camera_rgb_optical_frame' coordinates --> Image(Pixel) Coordinates
        filtered_cloud_xyz = filtered_cloud_array[:,:3][:,:,None] # (num_points,3,1)
        filtered_cloud_rgb = filtered_cloud_array[:,3:][:,:,None] # (num_points,3,1)
        intr_matrix = self.intr_matrix.repeat(len(filtered_cloud_xyz),1,1) # (num_points,3,3)
        projected = torch.bmm(intr_matrix, filtered_cloud_xyz.double()) # (num_points,3,1)
        scale = projected[:, 2].unsqueeze(1).expand_as(projected).clone()
        projected /= scale # (num_points,3,1)
        # filter out out-of-view points (with respect to width and height axis)
        projected = projected.squeeze().long() # (num_points,3)
        projected = torch.cat([projected,filtered_cloud_rgb.squeeze().long()],dim=1) # (num_points, 6)

        keep_inFOV = (0 <= projected[:,0]) & (projected[:, 0]<self.camera_model.width) & \
               (0 <= projected[:,1]) & (projected[:, 1]<self.camera_model.height)
        projected = projected[keep_inFOV] # Dim: (Num_points,6),Channel order: xyzrgb
        scale = scale[keep_inFOV]
        merged_keep = keep_positive_z.clone()
        merged_keep[keep_positive_z==1] = keep_inFOV

        return projected, merged_keep, cloud_array[merged_keep]

    def _project_points_to_img(self,projected):
        '''
        depreciated function, in favor of open3d visualizer
        :param projected: an aligned point-cloud of N points where each points is 6-dim vector (xyzrgb)
        :return: img_proj (re-synthesized rgb image from point-cloud)
        '''
        order=torch.argsort(projected[:,2],descending=False)
        projected = projected[order]
        # split xy and rgb (remove z, which is not necessary)
        projected = projected[:,[1,0,3,4,5]] # channel order [xyzrgb]-->[yxrgb] (remove z)
        projected_yx = projected[:, :2]
        projected_rgb = projected[:, 2:]
        # mark each point to zero-padded image
        markStartTime = time.time()
        pixel_y,pixel_x = projected_yx[:,0], projected_yx[:,1]
        img_proj = torch.zeros_like(self.img_projected)
        if opt.proj_method == 'resize':
            img_proj[pixel_y//opt.resize_ratio, pixel_x//opt.resize_ratio] = projected_rgb
            img_proj = torch.nn.functional.interpolate(input=img_proj.permute(2,0,1).unsqueeze(0).float(),
                                                       scale_factor=4,
                                                       mode="nearest")
            img_proj = img_proj.squeeze().permute(1,2,0).long()
        elif opt.proj_method == 'padding':
            pixel_x_left = torch.max(pixel_y -1, torch.zeros_like(pixel_y))
            pixel_x_right = torch.min(pixel_y + 1, (self.camera_model.height-1)*torch.ones_like(pixel_y))
            pixel_y_left = torch.max(pixel_x -1, torch.zeros_like(pixel_x))
            pixel_y_right = torch.min(pixel_x + 1, (self.camera_model.width-1)*torch.ones_like(pixel_x))
            img_proj[pixel_x_left, pixel_x] = projected_rgb
            img_proj[pixel_x_right, pixel_x] = projected_rgb
            img_proj[pixel_y, pixel_y_left] = projected_rgb
            img_proj[pixel_y, pixel_y_right] = projected_rgb
            img_proj[pixel_x_left, pixel_y_left] = projected_rgb
            img_proj[pixel_x_right, pixel_y_right] = projected_rgb
            img_proj[pixel_y, pixel_x] = projected_rgb
        else:
            img_proj[pixel_y, pixel_x] = projected_rgb
        return img_proj

    def _img_arr2msg(self,img):
        img_projected_msg = ros_numpy.image.numpy_to_image(img.astype('uint8'),encoding='rgb8')
        return img_projected_msg

    def _img_msg2arr(self,img_msg):
        img_numpy = ros_numpy.image.image_to_numpy(img_msg)
        return img_numpy

    def _save_pose(self):
        x = self.inv_transform_msg.transform.translation.x
        y = self.inv_transform_msg.transform.translation.y
        z = self.inv_transform_msg.transform.translation.z
        qx = self.inv_transform_msg.transform.rotation.x
        qy = self.inv_transform_msg.transform.rotation.y
        qz = self.inv_transform_msg.transform.rotation.z
        qw = self.inv_transform_msg.transform.rotation.w
        self.current_pose_numpy = np.array([x,y,z,qx,qy,qz,qw])
        tf = open(self.path+'/pose/{}.txt'.format(self.cnt//self.save_interval),mode='w')
        tf.write('{} {} {} {} {} {} {}'.format(x,y,z,qx,qy,qz,qw))
        tf.close()

opt = parser.parse_args()
rospy.init_node('cloud_to_img', anonymous=True)
image_sub = message_filters.Subscriber("/camera/rgb/image_rect_color", Image)
depth_sub = message_filters.Subscriber("/camera/depth_registered/image_raw",Image)
seg_sub = message_filters.Subscriber("/camera/seg/segmentation_gt",Image)
ts = message_filters.TimeSynchronizer([image_sub, seg_sub, depth_sub], queue_size=1)
server = Cloud2ImgProjection(opt)
ts.registerCallback(server.callback)
rate = rospy.Rate(0.5) # Not working. Why?
rospy.spin()

