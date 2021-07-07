import os
import os.path as osp
import numpy as np
import random
import matplotlib.pyplot as plt
import collections
import torch
import torchvision
import torch.nn as nn
from torch.utils import data
from PIL import Image
from utils import Object_Labeling
from torchvision.transforms import ToTensor, Resize, Compose
import torchvision.transforms as transforms
from depth2disp_example import depth2disp
import glob
import pdb
import matplotlib.pyplot as plt
import tkinter
import matplotlib
matplotlib.use('TkAgg')


class ChangeSim(data.Dataset):
    def __init__(self, crop_size=(320, 240), num_classes=5, set='train'):
        """
        ChangeSim Dataloader
        Please download ChangeSim Dataset in https://github.com/SAMMiCA/ChangeSim

        Compose the folder in this way:

        [Folder Name]
        |
        --- Mapping
            |
            --- Ref_Seq_Train
            |        |
            |        +--- Warehouse_0
            |        .         .
            |        +--- Warehouse_5
            |
            +-- Ref_Seq_Test
            |        |
            |        +--- Warehouse_6
            |        .
            |        +--- Warehouse_9

            Localization
            |
            +-- Query_Seq_Train
            +-- Query_Seq_Test

            script
            |
            --- visualization.py
            |
            --- utils
            |     |
            |     --- Object_Labeling.py
            |     --- dict_indexing.py
            |     --- idx2color.txt

        Args:
            crop_size (tuple): Image resize shape (H,W) (default: (320, 240))
            num_classes (int): Number of target change detection class
                               5 for multi-class change detection
                               2 for binary change detection (default: 5)
            set (str): 'train' or 'test' (defalut: 'train')
        """
        self.crop_size = crop_size
        self.num_classes = num_classes
        self.set = set
        self.blacklist=[]
        train_list = ['Warehouse_0', 'Warehouse_1', 'Warehouse_2', 'Warehouse_3', 'Warehouse_4', 'Warehouse_5']
        test_list = ['Warehouse_6', 'Warehouse_7', 'Warehouse_8', 'Warehouse_9']
        self.image_total_files = []
        if set == 'train':
            for map in train_list:
                self.image_total_files += glob.glob('../Localization/Query_Seq_Train/' + map + '/Seq_0/rgb/*.png')
                self.image_total_files += glob.glob('../Localization/Query_Seq_Train/' + map + '/Seq_1/rgb/*.png')
        elif set == 'test':
            for map in test_list:
                self.image_total_files += glob.glob('../Localization/Query_Seq_Test/' + map + '/Seq_0/rgb/*.png')
                self.image_total_files += glob.glob('../Localization/Query_Seq_Test/' + map + '/Seq_1/rgb/*.png')
        # if not max_iters == None:
        #     self.image_total_files = self.image_total_files * int(np.ceil(float(max_iters) / len(self.image_total_files)))
        #     self.image_total_files = self.image_total_files[:max_iters]

        self.seg = Object_Labeling.SegHelper(idx2color_path='./utils/idx2color.txt', num_class=self.num_classes)
        
        #### Color Transform ####
        self.color_transform = transforms.Compose([transforms.ColorJitter(0.4, 0.4, 0.4, 0.25),
                                                   transforms.ToTensor()])
        # self.transform = Compose([Resize(crop_size), ToTensor()])

    def __len__(self):
        return len(self.image_total_files)

    def __getitem__(self, index):
        # Train set
        if self.set == 'train':
            loss = nn.L1Loss()
            while True:
                if index in self.blacklist:
                    index=random.randint(0,self.__len__()-1)
                    continue

                test_rgb_path = self.image_total_files[index]
                file_idx = test_rgb_path.split('/')[-1].split('.')[0]  # ~~ of ~~.png

                ref_pose_find_path = test_rgb_path.replace(f'rgb/{file_idx}.png',f't0/idx/{file_idx}.txt')
                f = open(ref_pose_find_path,'r',encoding='utf8')
                ref_pose_idx = int(f.readlines()[0])
                g2o_path = test_rgb_path.replace('/Localization/Query_Seq_Train','/Mapping/Ref_Seq_Train').replace(f'rgb/{file_idx}.png',f'raw/poses.g2o')
                with open(g2o_path,'r',encoding = 'utf8') as f2:
                    while True:
                        line = f2.readline()
                        try:
                            if line.split()[0] == 'VERTEX_SE3:QUAT' and int(line.split()[1]) == ref_pose_idx:
                                ref_pose = line.split()[2:]
                        except:
                            break
                ref_pose = torch.from_numpy(np.array(ref_pose).astype(float))
                change_pose_path = test_rgb_path.replace(f'rgb/{file_idx}.png',f'pose/{file_idx}.txt')
                with open(change_pose_path,'r',encoding='utf8') as f3:
                    change_pose = f3.readline().split()
                    change_pose = torch.from_numpy(np.array(change_pose).astype(float))

                distance = loss(ref_pose.cuda(),change_pose.cuda())
                if distance.item()<0.5:
                    break
                else:
                    self.blacklist.append(index)
                    index=random.randint(0,self.__len__()-1)
        # Test set
        else:
            test_rgb_path = self.image_total_files[index]

        # Get File Paths
        test_depth_path = test_rgb_path.replace('rgb', 'depth')
        ref_rgb_path = test_rgb_path.replace('rgb', 't0/rgb')
        ref_depth_path = test_rgb_path.replace('rgb', 't0/depth')
        change_segmentation_path = test_rgb_path.replace('rgb', 'change_segmentation')

        name = '_'.join(test_rgb_path.split('/')[-5:])

        #### Color Transform ####
        test_rgb = Image.open(test_rgb_path)
        ref_rgb = Image.open(ref_rgb_path)
        test_rgb = test_rgb.resize(self.crop_size, Image.BICUBIC)
        ref_rgb = ref_rgb.resize(self.crop_size, Image.BICUBIC)

        # RGB, Color Transform for train set
        if self.set == 'train':
            test_rgb = self.color_transform(test_rgb)
            ref_rgb = self.color_transform(ref_rgb)
        else:
            test_rgb = ToTensor()(test_rgb)
            ref_rgb = ToTensor()(ref_rgb)

        # Depth
        # test_depth = Image.open(test_depth_path)
        # test_depth = np.asarray(test_depth)
        # test_depth = test_depth.astype('float32') / 255
        # test_depth = depth2disp(test_depth, 1, 50)
        # test_depth = torch.from_numpy(test_depth)

        # ref_depth = Image.open(ref_depth_path)
        # ref_depth = np.clip(ref_depth, 0, 50) / 50 * 255
        # ref_depth = np.asarray(ref_depth)
        # ref_depth = ref_depth.astype('float32') / 255
        # ref_depth = depth2disp(ref_depth, 1, 50)
        # ref_depth = torch.from_numpy(ref_depth)

        # Change Label
        change_label = Image.open(change_segmentation_path)
        change_label = change_label.resize(self.crop_size, Image.NEAREST)
        change_label_mapping = np.asarray(change_label).copy()
        change_mapping = self.seg.colormap2classmap(change_label_mapping)
        label = change_mapping.permute(2,0,1).squeeze(0).long().cpu()

        #### Binarization ####
        if self.num_classes == 2:
            label[label > 0] = 1

        # if (label > 5).sum() > 0:
        #     print(image_path)

        # Horizontal Flip
        if self.set == 'train' and np.random.rand() <= 0.5:
            test_rgb = np.asarray(test_rgb)
            test_rgb = test_rgb[:, :, ::-1]
            test_rgb = np.ascontiguousarray(test_rgb)
            test_rgb = torch.from_numpy(test_rgb)

            ref_rgb = np.asarray(ref_rgb)
            ref_rgb = ref_rgb[:, :, ::-1]
            ref_rgb = np.ascontiguousarray(ref_rgb)
            ref_rgb = torch.from_numpy(ref_rgb)

            label = np.asarray(label)
            label = label[:, ::-1]
            label = np.ascontiguousarray(label)
            label = torch.from_numpy(label)
    
        return [ref_rgb, test_rgb], label.long(), test_rgb_path


if __name__ == '__main__':
    dst = ChangeSim(crop_size=(320, 240), num_classes=5, set='train')
    dataloader = data.DataLoader(dst, batch_size=4, num_workers=2)
    for i, data in enumerate(dataloader):
        imgs, labels, path = data
