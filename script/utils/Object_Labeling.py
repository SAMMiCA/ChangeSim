# -*- coding: utf-8 -*-
import numpy as np
from .dict_indexing import Dict_indexing
import torch

class SegHelper:
    def __init__(self,opt=None,idx2color_path='../../backup/idx2color.txt',num_class=32):
        self.opt = opt
        self.num_classes = num_class
        self.idx2color_path = idx2color_path
        f = open(self.idx2color_path, 'r')
        self.idx2color = {k:[] for k in range(self.num_classes)}
        for j in range(256):
            line = f.readline()
            line = line.strip(' \n').strip('[').strip(']').strip(' ').split()
            line = [int(l) for l in line if l.isdigit()]
            self.idx2color[j] = line # color in rgb order

        self.color2idx = {tuple(v):k for k,v in self.idx2color.items()}
        name2idx = Dict_indexing()
        self.name2idx = {k: name2idx[k]['index'] for k in name2idx.keys()}
        self.idx2name = {v:k for k,v in self.name2idx.items()}
        self.idx2name_padding = {v:'BG' for v in range(self.num_classes,256)}
        self.idx2name.update(self.idx2name_padding)

    def unique(self,array):
        uniq, index = np.unique(array, return_index=True, axis=0)
        return uniq[index.argsort()]

    def extract_color_from_seg(self,img_seg):
        colors = img_seg.reshape(-1, img_seg.shape[-1]) # (H*W,3) # color channel in rgb order
        unique_colors = self.unique(colors) # (num_class_in_img,3)
        return unique_colors

    def extract_class_from_seg(self,img_seg):
        unique_colors = self.extract_color_from_seg(img_seg) # (num_class_in_img,3) # color channel in rgb order
        classes_idx = [self.color2idx[tuple(color.tolist())]for color in unique_colors]
        classes_str = [self.idx2name[idx] for idx in classes_idx]
        return classes_idx, classes_str

    def colormap2classmap(self,seg_array):
        seg_array_flattened = torch.LongTensor(seg_array.reshape(-1,3)).cuda()
        seg_map_class_flattened = torch.zeros((seg_array.shape[0],seg_array.shape[1],1)).view(-1,1).cuda()
        for color, cls in self.color2idx.items():
            matching_indices = (seg_array_flattened == torch.LongTensor(color).cuda())
            matching_indices = (matching_indices.sum(dim=1)==3)
            seg_map_class_flattened[matching_indices] = cls
        seg_map_class = seg_map_class_flattened.view(seg_array.shape[0],seg_array.shape[1],1)
        return seg_map_class

    def classmap2colormap(self,seg_map_class):
        seg_map_class_flattened = seg_map_class.view(-1,1)
        seg_map_color_flattened = torch.zeros(seg_map_class.shape[0]*seg_map_class.shape[1],3).cuda().long()
        for cls, color in self.idx2color.items():
            matching_indices = (seg_map_class_flattened == torch.LongTensor([cls]).cuda())
            seg_map_color_flattened[matching_indices.view(-1)] = torch.LongTensor(color).cuda()
        seg_map_color_flattened = seg_map_color_flattened.view(seg_map_class.shape[0],seg_map_class.shape[1],3)
        return seg_map_color_flattened

    def split_SemAndChange(self,seg_map_class):
        seg_map_change_class = seg_map_class//50
        seg_map_semantic_class = torch.fmod(seg_map_class,50)
        return seg_map_semantic_class, seg_map_change_class

def ObjectLabeling(client,map_id,name2idx):
    found = client.simSetSegmentationObjectID("[\w]*", name2idx['background']['index'], True)
    print("set all object IDs to 0: %r" % (found))
    #####
    found = client.simSetSegmentationObjectID("[\w]*column[\w]*", 1, True)
    found = client.simSetSegmentationObjectID("[\w]*pillar[\w]*", 1, True)
    found = client.simSetSegmentationObjectID("[\w]*pilar[\w]*", 1, True)
    found = client.simSetSegmentationObjectID("[\w]*Pilar[\w]*", 1, True)

    found = client.simSetSegmentationObjectID("[\w]*pipe[\w]*", 2, True)
    found = client.simSetSegmentationObjectID("[\w]*tube[\w]*", 2, True)

    found = client.simSetSegmentationObjectID("[\w]*wall[\w]*", 3, True)
    found = client.simSetSegmentationObjectID("[\w]*tunnel[\w]*", 3, True)
    found = client.simSetSegmentationObjectID("[\w]*Wall[\w]*", 3, True)

    found = client.simSetSegmentationObjectID("[\w]*beam[\w]*", 4, True)

    found = client.simSetSegmentationObjectID("[\w]*floor[\w]*", 5, True)
    found = client.simSetSegmentationObjectID("[\w]*Floor[\w]*", 5, True)
    found = client.simSetSegmentationObjectID("[\w]*slam[\w]*", 5, True)
    found = client.simSetSegmentationObjectID("[\w]*ground[\w]*", 5, True)
    found = client.simSetSegmentationObjectID("[\w]*road[\w]*", 5, True)
    found = client.simSetSegmentationObjectID("[\w]*walk[\w]*", 5, True)
    found = client.simSetSegmentationObjectID("[\w]*floor[\w]panel[\w]*", 5, True)

    found = client.simSetSegmentationObjectID("[\w]*frame[\w]*", 6, True)
    found = client.simSetSegmentationObjectID("[\w]*scafolding[\w]*", 6, True)
    found = client.simSetSegmentationObjectID("[\w]*scaffolding[\w]*", 6, True)
    found = client.simSetSegmentationObjectID("[\w]*scaffold[\w]*", 6, True)
    found = client.simSetSegmentationObjectID("[\w]*form[\w]*work[\w]*", 6, True)
    found = client.simSetSegmentationObjectID("[\w]*pole[\w]*", 6, True)
    found = client.simSetSegmentationObjectID("[\w]*support[\w]*", 6, True)

    found = client.simSetSegmentationObjectID("[\w]*fence[\w]*", 7, True)
    found = client.simSetSegmentationObjectID("[\w]*fencing[\w]*", 7, True)

    found = client.simSetSegmentationObjectID("[\w]*wire[\w]*", 8, True)
    found = client.simSetSegmentationObjectID("[\w]*wire[\w]*cylinder[\w]*", 8, True)

    found = client.simSetSegmentationObjectID("[\w]*cable[\w]*", 9, True)

    found = client.simSetSegmentationObjectID("[\w]*window[\w]*", 10, True)
    found = client.simSetSegmentationObjectID("[\w]*glass[\w]*panel[\w]*", 10, True)

    found = client.simSetSegmentationObjectID("[\w]*railing[\w]*", 11, True)

    found = client.simSetSegmentationObjectID("[\w]*rail[\w]*", 12, True)

    found = client.simSetSegmentationObjectID("[\w]*ceiling[\w]*", 13, True)
    found = client.simSetSegmentationObjectID("[\w]*roof[\w]*", 13, True)
    found = client.simSetSegmentationObjectID("[\w]*Roof[\w]*", 13, True)

    found = client.simSetSegmentationObjectID("[\w]*stair[\w]*", 14, True)
    found = client.simSetSegmentationObjectID("[\w]*Stairs[\w]*", 14, True)

    found = client.simSetSegmentationObjectID("[\w]*duct[\w]*", 15, True)
    found = client.simSetSegmentationObjectID("[\w]*vent[\w]*", 15, True)
    found = client.simSetSegmentationObjectID("[\w]*ventilation[\w]*", 15, True)
    found = client.simSetSegmentationObjectID("[\w]*Vent[\w]*", 15, True)

    found = client.simSetSegmentationObjectID("[\w]*gril[\w]*", 16, True)
    found = client.simSetSegmentationObjectID("[\w]*grid[\w]*", 16, True)

    #####
    found = client.simSetSegmentationObjectID("[\w]*light[\w]*", 17, True)
    found = client.simSetSegmentationObjectID("[\w]*lamp[\w]*", 17, True)

    found = client.simSetSegmentationObjectID("[\w]*trash[\w]*", 18, True)
    found = client.simSetSegmentationObjectID("[\w]*debris[\w]*", 18, True)
    found = client.simSetSegmentationObjectID("[\w]*book[\w]*", 18, True)
    found = client.simSetSegmentationObjectID("[\w]*paper[\w]*", 18, True)

    found = client.simSetSegmentationObjectID("[\w]*drawer[\w]*", 19, True)
    found = client.simSetSegmentationObjectID("[\w]*shelf[\w]*", 19, True)
    found = client.simSetSegmentationObjectID("[\w]*rack[\w]*", 19, True)
    found = client.simSetSegmentationObjectID("[\w]*locker[\w]*", 19, True)
    found = client.simSetSegmentationObjectID("[\w]*cabinet[\w]*", 19, True)

    found = client.simSetSegmentationObjectID("[\w]*door[\w]*", 20, True)
    found = client.simSetSegmentationObjectID("[\w]*gate[\w]*", 20, True)

    found = client.simSetSegmentationObjectID("[\w]*barrel[\w]*", 21, True)
    found = client.simSetSegmentationObjectID("[\w]*barel[\w]*", 21, True)
    found = client.simSetSegmentationObjectID("[\w]*drum[\w]*", 21, True)
    found = client.simSetSegmentationObjectID("[\w]*tank[\w]*", 21, True)
    found = client.simSetSegmentationObjectID("[\w]*Barrel[\w]*", 21, True)

    found = client.simSetSegmentationObjectID("[\w]*sign[\w]*", 22, True)
    found = client.simSetSegmentationObjectID("[\w]*sign[\w]*cver[\w]*", 22, True)

    found = client.simSetSegmentationObjectID("[\w]*paperbox[\w]*", 23, True)
    found = client.simSetSegmentationObjectID("[\w]*box[\w]*", 23, True)
    found = client.simSetSegmentationObjectID("[\w]*bin[\w]*", 23, True)
    found = client.simSetSegmentationObjectID("[\w]*cube[\w]*", 23, True)
    found = client.simSetSegmentationObjectID("[\w]*crate[\w]*plastic[\w]*", 23, True)

    found = client.simSetSegmentationObjectID("[\w]*bag[\w]*", 24, True)

    found = client.simSetSegmentationObjectID("[\w]*power[\w]*box[\w]*", 25, True)
    found = client.simSetSegmentationObjectID("[\w]*fuse[\w]*box[\w]*", 25, True)
    found = client.simSetSegmentationObjectID("[\w]*switch[\w]*board[\w]*", 25, True)
    found = client.simSetSegmentationObjectID("[\w]*electrical[\w]*supply[\w]*", 25, True)
    found = client.simSetSegmentationObjectID("[\w]*electrical[\w]*box[\w]*", 25, True)
    found = client.simSetSegmentationObjectID("[\w]*control[\w]*panel[\w]*", 25, True)
    found = client.simSetSegmentationObjectID("[\w]*electric[\w]*panel[\w]*", 25, True)

    found = client.simSetSegmentationObjectID("[\w]*transporter[\w]*", 26, True)
    found = client.simSetSegmentationObjectID("[\w]*truck[\w]*", 26, True)
    found = client.simSetSegmentationObjectID("[\w]*trailer[\w]*", 26, True)

    found = client.simSetSegmentationObjectID("[\w]*ladder[\w]*", 27, True)
    found = client.simSetSegmentationObjectID("[\w]*Ladder[\w]*", 27, True)

    found = client.simSetSegmentationObjectID("[\w]*canister[\w]*", 28, True)
    found = client.simSetSegmentationObjectID("[\w]*can[\w]*", 28, True)
    found = client.simSetSegmentationObjectID("[\w]*bottle[\w]*", 28, True)
    found = client.simSetSegmentationObjectID("[\w]*cylinder[\w]*", 28, True)
    found = client.simSetSegmentationObjectID("[\w]*keg[\w]*", 28, True)

    found = client.simSetSegmentationObjectID("[\w]*extinguisher[\w]*", 29, True)

    # found = client.simSetSegmentationObjectID("[\w]*furniture[\w]*", 30, True)
    # found = client.simSetSegmentationObjectID("[\w]*sofa[\w]*", 30, True)
    # found = client.simSetSegmentationObjectID("[\w]*chair[\w]*", 30, True)
    # found = client.simSetSegmentationObjectID("[\w]*table[\w]*", 30, True)

    client.simSetSegmentationObjectID("[\w]*forklift[\w]*", name2idx['vehicle']['index'], True)
    client.simSetSegmentationObjectID("[\w]*pallet[\w]*", name2idx['pallet']['index'], True)
    client.simSetSegmentationObjectID("[\w]*pumptruck[\w]*", name2idx['hand_truck']['index'], True)


    if map_id == 'Bunker':
        found = client.simSetSegmentationObjectID("brick_[\w]*", 18, True)
        found = client.simSetSegmentationObjectID("door_case[\w]*", 3, True)
        found = client.simSetSegmentationObjectID("hose[\w]*", 9, True)
        found = client.simSetSegmentationObjectID("Lamp_balon[\w]*", 28, True)
        found = client.simSetSegmentationObjectID("plinth[\w]*", 3, True)
        found = client.simSetSegmentationObjectID("plate_concrete[\w]*", 13, True)
        for i in range(34, 104):
            found = client.simSetSegmentationObjectID("plate_concrete{}".format(i), 5, True)
        found = client.simSetSegmentationObjectID("plate_concrete20", 5, True)
        found = client.simSetSegmentationObjectID("plate_concrete28", 5, True)
        found = client.simSetSegmentationObjectID("plate_concrete25", 5, True)
        found = client.simSetSegmentationObjectID("plate_concrete106", 5, True)
        found = client.simSetSegmentationObjectID("plate_concrete107", 5, True)
        found = client.simSetSegmentationObjectID("plate_concrete_drain[\w]*", 5, True)
        found = client.simSetSegmentationObjectID("platform_[\w]*", 16, True)
        found = client.simSetSegmentationObjectID("crane_pipe_suport[\w]*", 0, True)

    elif map_id == 'Warehouse_01':
        print('MAPNAME: {}'.format(map_id))
        client.simSetSegmentationObjectID("Landscape[\w]*", name2idx['floor']['index'], True)
        client.simSetSegmentationObjectID("tallsteps[\w]*", name2idx['ladder']['index'], True)
        client.simSetSegmentationObjectID("Cardboard[\w]*", name2idx['trash']['index'], True)
        client.simSetSegmentationObjectID("InstancedFoliageActor", name2idx['trash']['index'], True)
        client.simSetSegmentationObjectID("truck[\w]*", name2idx['hand_truck']['index'], True)
        client.simSetSegmentationObjectID("fire_ex[\w]*", name2idx['extinguisher']['index'], True)
        client.simSetSegmentationObjectID("Board[\w]*", name2idx['background']['index'], True)
        client.simSetSegmentationObjectID("Rubbish[\w]*", name2idx['trash']['index'], True)
        client.simSetSegmentationObjectID("Rail[\w]*", name2idx['railing']['index'], True)
        client.simSetSegmentationObjectID("Support[\w]*", name2idx['column']['index'], True)
        print(client.simSetSegmentationObjectID("pallet_02a[\w]*", name2idx['box']['index'], True))
        print(client.simSetSegmentationObjectID("pallet_02b[\w]*", name2idx['box']['index'], True))

        print(client.simSetSegmentationObjectID("pallet_03_178", name2idx['box']['index'], True))
        client.simSetSegmentationObjectID("pallet_4", name2idx['box']['index'], True)
        client.simSetSegmentationObjectID("pallet_5", name2idx['box']['index'], True)
        client.simSetSegmentationObjectID("pallet_6", name2idx['box']['index'], True)
        client.simSetSegmentationObjectID("pallet_208", name2idx['box']['index'], True)
        client.simSetSegmentationObjectID("pallet_100", name2idx['box']['index'], True)
        client.simSetSegmentationObjectID("pallet_202", name2idx['box']['index'], True)
        client.simSetSegmentationObjectID("pallet_26", name2idx['box']['index'], True)
        client.simSetSegmentationObjectID("pallet_38", name2idx['box']['index'], True)
        client.simSetSegmentationObjectID("pallet_47", name2idx['box']['index'], True)
        client.simSetSegmentationObjectID("pallet_59", name2idx['box']['index'], True)
        client.simSetSegmentationObjectID("pallet_68", name2idx['box']['index'], True)
        client.simSetSegmentationObjectID("pallet_80", name2idx['box']['index'], True)
        client.simSetSegmentationObjectID("pallet_89", name2idx['box']['index'], True)
        client.simSetSegmentationObjectID("pallet_101", name2idx['box']['index'], True)
        client.simSetSegmentationObjectID("pallet_110", name2idx['box']['index'], True)
        client.simSetSegmentationObjectID("pallet_122", name2idx['box']['index'], True)
        client.simSetSegmentationObjectID("pallet_131", name2idx['box']['index'], True)
        client.simSetSegmentationObjectID("pallet_143", name2idx['box']['index'], True)
        client.simSetSegmentationObjectID("pallet_152", name2idx['box']['index'], True)
        client.simSetSegmentationObjectID("pallet_164", name2idx['box']['index'], True)
        client.simSetSegmentationObjectID("pallet_270", name2idx['box']['index'], True)
        client.simSetSegmentationObjectID("pallet_114", name2idx['box']['index'], True)
        client.simSetSegmentationObjectID("pallet_152", name2idx['box']['index'], True)
        client.simSetSegmentationObjectID("pallet_173", name2idx['box']['index'], True)
        client.simSetSegmentationObjectID("pallet_174", name2idx['box']['index'], True)
        client.simSetSegmentationObjectID("pallet_242", name2idx['box']['index'], True)
        client.simSetSegmentationObjectID("pallet_180", name2idx['box']['index'], True)
        client.simSetSegmentationObjectID("pallet_230", name2idx['box']['index'], True)
        client.simSetSegmentationObjectID("pallet_205", name2idx['box']['index'], True)

        client.simSetSegmentationObjectID("SM_Keg[\w]*", name2idx['canister']['index'], True)

        client.simSetSegmentationObjectID("Wall_Half_Window[\w]*", name2idx['wall']['index'], True)
        client.simSetSegmentationObjectID("Filing_Cabinet[\w]*", name2idx['shelf']['index'], True)
        client.simSetSegmentationObjectID("Filing_Documents[\w]*", name2idx['box']['index'], True)
        client.simSetSegmentationObjectID("Steps[\w]*", name2idx['stair']['index'], True)
        client.simSetSegmentationObjectID("Floor_4", name2idx['stair']['index'], True)
        client.simSetSegmentationObjectID("Floor_5", name2idx['stair']['index'], True)
        client.simSetSegmentationObjectID("Floor_22", name2idx['stair']['index'], True)

    print("Labeling Done")


