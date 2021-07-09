import os
import os.path as osp
import numpy as np
import random
import matplotlib.pyplot as plt
import torch
import torchvision
import torch.nn as nn
from torch.utils import data
from PIL import Image
from utils import Object_Labeling
from torchvision.transforms import ToTensor, Resize, Compose
import torchvision.transforms as transforms
import glob
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('TkAgg')

class ChangeSim(data.Dataset):
    def __init__(self, crop_size=(320, 240), num_classes=5, set='train'):
        """
        ChangeSim Dataloader for Visualization
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

    def __len__(self):
        return len(self.image_total_files)

    def __getitem__(self, index):
        # Get File Paths
        test_rgb_path = self.image_total_files[index]
        test_depth_path = test_rgb_path.replace('rgb', 'depth')
        test_segmentation_path = test_rgb_path.replace('rgb', 'semantic_segmentation')
        ref_rgb_path = test_rgb_path.replace('rgb', 't0/rgb')
        ref_depth_path = test_rgb_path.replace('rgb', 't0/depth')
        change_segmentation_path = test_rgb_path.replace('rgb', 'change_segmentation')
        name = '_'.join(test_rgb_path.split('/')[-5:])

        # RGB
        test_rgb = Image.open(test_rgb_path)
        ref_rgb = Image.open(ref_rgb_path)
        test_rgb = test_rgb.resize(self.crop_size, Image.BICUBIC)
        ref_rgb = ref_rgb.resize(self.crop_size, Image.BICUBIC)
        test_rgb = ToTensor()(test_rgb)
        ref_rgb = ToTensor()(ref_rgb)

        # Depth
        test_depth = Image.open(test_depth_path)
        test_depth = test_depth.resize(self.crop_size, Image.BICUBIC)
        test_depth = np.asarray(test_depth)
        test_depth = test_depth.astype('float32') / 255
        test_depth = torch.from_numpy(test_depth)

        ref_depth = Image.open(ref_depth_path)
        ref_depth = ref_depth.resize(self.crop_size, Image.BICUBIC)
        ref_depth = np.clip(ref_depth, 0, 50000) / 50000 * 255  # scaling, 50m = 500000mm
        ref_depth = np.asarray(ref_depth)
        ref_depth = ref_depth.astype('float32') / 255
        ref_depth = torch.from_numpy(ref_depth)

        # Semantic Segmentation
        test_segmentation = Image.open(test_segmentation_path)
        test_segmentation = test_segmentation.resize(self.crop_size, Image.BICUBIC)
        test_segmentation = ToTensor()(test_segmentation)

        # Change Label
        change_label = Image.open(change_segmentation_path)
        label = change_label.resize(self.crop_size, Image.NEAREST)
        label = ToTensor()(label)

        return [ref_rgb, test_rgb], [ref_depth.unsqueeze(0),test_depth.unsqueeze(0)], test_segmentation, label, test_rgb_path


if __name__ == '__main__':
    batch_size = 4
    dst = ChangeSim(crop_size=(320, 240), num_classes=5, set='train')
    dataloader = data.DataLoader(dst, batch_size=batch_size, num_workers=2, shuffle=True)
    dataiter = iter(dataloader)
    imgs, depths, segmentation, labels, path = next(dataiter)

    [ref_rgb, query_rgb] = imgs
    [ref_depth, query_depth] = depths
    change_label = labels
    query_segmentation = segmentation

    # visualization
    fig = plt.figure()
    ax1 = fig.add_subplot(3, 2, 1)
    ax2 = fig.add_subplot(3, 2, 2)
    ax3 = fig.add_subplot(3, 2, 3)
    ax4 = fig.add_subplot(3, 2, 4)
    ax5 = fig.add_subplot(3, 2, 5)
    ax6 = fig.add_subplot(3, 2, 6)

    ax1.imshow(torchvision.utils.make_grid(ref_rgb, normalize=False, nrow=batch_size).permute(1, 2, 0).numpy())
    ax1.set_title('Reference RGB')
    ax1.axis('off')
    ax2.imshow(torchvision.utils.make_grid(query_rgb, normalize=False, nrow=batch_size).permute(1, 2, 0).numpy())
    ax2.set_title('Query RGB')
    ax2.axis('off')
    ax3.imshow(torchvision.utils.make_grid(ref_depth, normalize=False, nrow=batch_size).permute(1, 2, 0).numpy())
    ax3.set_title('Reference Depth')
    ax3.axis('off')
    ax4.imshow(torchvision.utils.make_grid(query_depth, normalize=False, nrow=batch_size).permute(1, 2, 0).numpy())
    ax4.set_title('Query Depth')
    ax4.axis('off')
    ax5.imshow(torchvision.utils.make_grid(change_label, normalize=False, nrow=batch_size).permute(1, 2, 0).numpy())
    ax5.set_title('Change Segmentation')
    ax5.axis('off')
    ax6.imshow(torchvision.utils.make_grid(query_segmentation, normalize=False, nrow=batch_size).permute(1, 2, 0).numpy())
    ax6.set_title('Query Semantic Segmentation')
    ax6.axis('off')

    plt.show()
