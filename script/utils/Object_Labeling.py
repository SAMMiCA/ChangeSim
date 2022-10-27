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

    found = client.simSetSegmentationObjectID("[\w]*column[\w]*", 1, True)
    found = client.simSetSegmentationObjectID("[\w]*pillar[\w]*", 1, True)
    found = client.simSetSegmentationObjectID("[\w]*pilar[\w]*", 1, True)
    found = client.simSetSegmentationObjectID("[\w]*Pilar[\w]*", 1, True)
    found = client.simSetSegmentationObjectID("[\w]*Pillar[\w]*", 1, True)

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
    found = client.simSetSegmentationObjectID("[\w]*Fence[\w]*", 7, True)

    found = client.simSetSegmentationObjectID("[\w]*wire[\w]*", 8, True)
    found = client.simSetSegmentationObjectID("[\w]*wire[\w]*cylinder[\w]*", 8, True)

    found = client.simSetSegmentationObjectID("[\w]*cable[\w]*", 9, True)

    found = client.simSetSegmentationObjectID("[\w]*window[\w]*", 10, True)
    found = client.simSetSegmentationObjectID("[\w]*glass[\w]*panel[\w]*", 10, True)
    found = client.simSetSegmentationObjectID("[\w]*Window[\w]*", 10, True)

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
    found = client.simSetSegmentationObjectID("[\w]*Lamp[\w]*", 17, True)

    found = client.simSetSegmentationObjectID("[\w]*trash[\w]*", 18, True)
    found = client.simSetSegmentationObjectID("[\w]*debris[\w]*", 18, True)
    found = client.simSetSegmentationObjectID("[\w]*book[\w]*", 18, True)
    found = client.simSetSegmentationObjectID("[\w]*paper[\w]*", 18, True)

    found = client.simSetSegmentationObjectID("[\w]*drawer[\w]*", 19, True)
    found = client.simSetSegmentationObjectID("[\w]*shelf[\w]*", 19, True)
    found = client.simSetSegmentationObjectID("[\w]*rack[\w]*", 19, True)
    found = client.simSetSegmentationObjectID("[\w]*locker[\w]*", 19, True)
    found = client.simSetSegmentationObjectID("[\w]*cabinet[\w]*", 19, True)
    found = client.simSetSegmentationObjectID("[\w]*Rack[\w]*", 19, True)

    found = client.simSetSegmentationObjectID("[\w]*door[\w]*", 20, True)
    found = client.simSetSegmentationObjectID("[\w]*gate[\w]*", 20, True)

    found = client.simSetSegmentationObjectID("[\w]*barrel[\w]*", 21, True)
    found = client.simSetSegmentationObjectID("[\w]*barel[\w]*", 21, True)
    found = client.simSetSegmentationObjectID("[\w]*drum[\w]*", 21, True)
    found = client.simSetSegmentationObjectID("[\w]*tank[\w]*", 21, True)
    found = client.simSetSegmentationObjectID("[\w]*Barrel[\w]*", 21, True)
    found = client.simSetSegmentationObjectID("[\w]*Barell[\w]*", 21, True)
    found = client.simSetSegmentationObjectID("[\w]*Barell[\w]*", 21, True)
    found = client.simSetSegmentationObjectID("[\w]*Barel[\w]*", 21, True)

    found = client.simSetSegmentationObjectID("[\w]*sign[\w]*", 22, True)
    found = client.simSetSegmentationObjectID("[\w]*sign[\w]*cver[\w]*", 22, True)
    found = client.simSetSegmentationObjectID("[\w]*Exit*", 22, True)
    found = client.simSetSegmentationObjectID("[\w]*exit*", 22, True)

    found = client.simSetSegmentationObjectID("[\w]*paperbox[\w]*", 23, True)
    found = client.simSetSegmentationObjectID("[\w]*box[\w]*", 23, True)
    found = client.simSetSegmentationObjectID("[\w]*bin[\w]*", 23, True)
    found = client.simSetSegmentationObjectID("[\w]*cube[\w]*", 23, True)
    found = client.simSetSegmentationObjectID("[\w]*crate[\w]*plastic[\w]*", 23, True)
    found = client.simSetSegmentationObjectID("[\w]*CardBox[\w]*", 23, True)
    found = client.simSetSegmentationObjectID("[\w]*Box[\w]*", 23, True)

    found = client.simSetSegmentationObjectID("[\w]*bag[\w]*", 24, True)

    found = client.simSetSegmentationObjectID("[\w]*power[\w]*box[\w]*", 25, True)
    found = client.simSetSegmentationObjectID("[\w]*fuse[\w]*box[\w]*", 25, True)
    found = client.simSetSegmentationObjectID("[\w]*switch[\w]*board[\w]*", 25, True)
    found = client.simSetSegmentationObjectID("[\w]*electrical[\w]*supply[\w]*", 25, True)
    found = client.simSetSegmentationObjectID("[\w]*electrical[\w]*box[\w]*", 25, True)
    found = client.simSetSegmentationObjectID("[\w]*control[\w]*panel[\w]*", 25, True)
    found = client.simSetSegmentationObjectID("[\w]*electric[\w]*panel[\w]*", 25, True)
    found = client.simSetSegmentationObjectID("[\w]*ElectricBox[\w]*", 25, True)
    found = client.simSetSegmentationObjectID("[\w]*FuseBox[\w]*", 25, True)

    found = client.simSetSegmentationObjectID("[\w]*transporter[\w]*", 26, True)
    found = client.simSetSegmentationObjectID("[\w]*truck[\w]*", 26, True)
    found = client.simSetSegmentationObjectID("[\w]*trailer[\w]*", 26, True)
    client.simSetSegmentationObjectID("[\w]*forklift[\w]*", name2idx['vehicle']['index'], True)

    found = client.simSetSegmentationObjectID("[\w]*ladder[\w]*", 27, True)
    found = client.simSetSegmentationObjectID("[\w]*Ladder[\w]*", 27, True)

    found = client.simSetSegmentationObjectID("[\w]*canister[\w]*", 28, True)
    found = client.simSetSegmentationObjectID("[\w]*can[\w]*", 28, True)
    found = client.simSetSegmentationObjectID("[\w]*bottle[\w]*", 28, True)
    found = client.simSetSegmentationObjectID("[\w]*cylinder[\w]*", 28, True)
    found = client.simSetSegmentationObjectID("[\w]*keg[\w]*", 28, True)
    found = client.simSetSegmentationObjectID("[\w]*Bottle[\w]*", 28, True)
    found = client.simSetSegmentationObjectID("[\w]*Bucket[\w]*", 28, True)

    found = client.simSetSegmentationObjectID("[\w]*extinguisher[\w]*", 29, True)
    found = client.simSetSegmentationObjectID("[\w]*Extinguisher[\w]*", 29, True)
    found = client.simSetSegmentationObjectID("[\w]*SM_fire_ex[\w]*", 29, True)
    found = client.simSetSegmentationObjectID("[\w]*fire_ex[\w]*", 29, True)

    client.simSetSegmentationObjectID("[\w]*pallet[\w]*", name2idx['pallet']['index'], True)
    client.simSetSegmentationObjectID("[\w]*Pallete[\w]*", name2idx['pallet']['index'], True)
    client.simSetSegmentationObjectID("[\w]*palette[\w]*", name2idx['pallet']['index'], True)

    client.simSetSegmentationObjectID("[\w]*pumptruck[\w]*", name2idx['hand_truck']['index'], True)
    client.simSetSegmentationObjectID("[\w]*JackElectric[\w]*", name2idx['hand_truck']['index'], True)
    client.simSetSegmentationObjectID("[\w]*jack[\w]*", name2idx['hand_truck']['index'], True)



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


    elif map_id == 'Storage':
        print('MAPNAME: {}'.format(map_id))

        client.simSetSegmentationObjectID("PaintBucket[\w]*", name2idx['canister']['index'], True)
        client.simSetSegmentationObjectID("Plankwood[\w]*", name2idx['beam']['index'], True)
        client.simSetSegmentationObjectID("SM_Log[\w]*", name2idx['beam']['index'], True)
        client.simSetSegmentationObjectID("SM_Plywood[\w]*", name2idx['beam']['index'], True)
        client.simSetSegmentationObjectID("SM_Plywood04_11[\w]*", name2idx['beam']['index'], True)
        client.simSetSegmentationObjectID("SM_Plankwood[\w]*", name2idx['beam']['index'], True)
        client.simSetSegmentationObjectID("Plankwood[\w]*", name2idx['beam']['index'], True)
        client.simSetSegmentationObjectID("Plywood[\w]*", name2idx['beam']['index'], True)

        client.simSetSegmentationObjectID("Rack81", name2idx['fence']['index'], True)
        client.simSetSegmentationObjectID("Rack80", name2idx['fence']['index'], True)
        client.simSetSegmentationObjectID("Rack79", name2idx['fence']['index'], True)
        client.simSetSegmentationObjectID("Rack78", name2idx['fence']['index'], True)
        client.simSetSegmentationObjectID("Rack77", name2idx['fence']['index'], True)
        client.simSetSegmentationObjectID("Rack76", name2idx['fence']['index'], True)
        client.simSetSegmentationObjectID("Rack75", name2idx['fence']['index'], True)
        client.simSetSegmentationObjectID("Rack74", name2idx['fence']['index'], True)
        client.simSetSegmentationObjectID("Rack133", name2idx['fence']['index'], True)
        client.simSetSegmentationObjectID("Rack132", name2idx['fence']['index'], True)
        client.simSetSegmentationObjectID("Rack131", name2idx['fence']['index'], True)
        client.simSetSegmentationObjectID("Rack130", name2idx['fence']['index'], True)


        client.simSetSegmentationObjectID("StorageDecal99", name2idx['sign']['index'], True)
        client.simSetSegmentationObjectID("StorageDecal94", name2idx['sign']['index'], True)
        client.simSetSegmentationObjectID("StorageDecal89", name2idx['sign']['index'], True)
        client.simSetSegmentationObjectID("StorageDecal84", name2idx['sign']['index'], True)
        client.simSetSegmentationObjectID("StorageDecal04_67", name2idx['sign']['index'], True)
        client.simSetSegmentationObjectID("StorageDecal18", name2idx['sign']['index'], True)
        client.simSetSegmentationObjectID("StorageDecal20", name2idx['sign']['index'], True)


        client.simSetSegmentationObjectID("Ramp11", name2idx['floor']['index'], True)
        client.simSetSegmentationObjectID("Ramp12", name2idx['floor']['index'], True)
        client.simSetSegmentationObjectID("Ramp15", name2idx['floor']['index'], True)
        client.simSetSegmentationObjectID("Ramp16", name2idx['floor']['index'], True)
        client.simSetSegmentationObjectID("Ramp23", name2idx['floor']['index'], True)
        client.simSetSegmentationObjectID("Ramp24", name2idx['floor']['index'], True)
        client.simSetSegmentationObjectID("Ramp21", name2idx['floor']['index'], True)
        client.simSetSegmentationObjectID("Ramp20", name2idx['floor']['index'], True)
        client.simSetSegmentationObjectID("Ramp19", name2idx['floor']['index'], True)

    elif map_id == 'StorageHouse':

        client.simSetSegmentationObjectID("[\w]*SM_Crate_PlasticNote[\w]*", name2idx['box']['index'], True)
        client.simSetSegmentationObjectID("[\w]*SM_Crate_Plastic[\w]*", name2idx['box']['index'], True)
        client.simSetSegmentationObjectID("[\w]*SM_CartonDrawer[\w]*", name2idx['box']['index'], True)
        client.simSetSegmentationObjectID("[\w]*SM_EmergencyBoardFull[\w]*", name2idx['background']['index'], True)
        client.simSetSegmentationObjectID("[\w]*FirstAidKit[\w]*", name2idx['box']['index'], True)
        client.simSetSegmentationObjectID("[\w]*Disifectant[\w]*", name2idx['canister']['index'], True)
        client.simSetSegmentationObjectID("[\w]*Flashlight[\w]*", name2idx['lamp']['index'], True)
        client.simSetSegmentationObjectID("[\w]*Paper_Shortcut[\w]*", name2idx['sign']['index'], True)
        client.simSetSegmentationObjectID("[\w]*PaperNote[\w]*", name2idx['sign']['index'], True)
        client.simSetSegmentationObjectID("[\w]*FloorDecal[\w]*", name2idx['background']['index'], True)
        client.simSetSegmentationObjectID("[\w]*FloorDecal_Letter[\w]*", name2idx['sign']['index'], True)
        client.simSetSegmentationObjectID("[\w]*PipeHolder[\w]*", name2idx['frame']['index'], True)
        client.simSetSegmentationObjectID("[\w]*Pushcart[\w]*", name2idx['hand_truck']['index'], True)
        client.simSetSegmentationObjectID("[\w]*Rackbeam[\w]*", name2idx['shelf']['index'], True)
        client.simSetSegmentationObjectID("[\w]*RackFrame[\w]*", name2idx['shelf']['index'], True)
        client.simSetSegmentationObjectID("[\w]*RackPile[\w]*", name2idx['beam']['index'], True)
        client.simSetSegmentationObjectID("[\w]*RackShelf[\w]*", name2idx['shelf']['index'], True)
        client.simSetSegmentationObjectID("[\w]*RackShield[\w]*", name2idx['background']['index'], True)
        client.simSetSegmentationObjectID("[\w]*WallPipe[\w]*", name2idx['pipe']['index'], True)
        client.simSetSegmentationObjectID("[\w]*WallSwitch[\w]*", name2idx['electric_box']['index'], True)
        client.simSetSegmentationObjectID("[\w]*WallWire[\w]*", name2idx['wire']['index'], True)
        client.simSetSegmentationObjectID("SM_PaletteA[\w]*", name2idx['pallet']['index'], True)
        client.simSetSegmentationObjectID("SM_Exit[\w]*", name2idx['sign']['index'], True)
        client.simSetSegmentationObjectID("SM_WallA_Doorway[\w]*", name2idx['wall']['index'], True)
        client.simSetSegmentationObjectID("SM_Book[\w]*", name2idx['background']['index'], True)
        client.simSetSegmentationObjectID("SM_BarelPlastic_A_04_280[\w]*", name2idx['background']['index'], True)
        client.simSetSegmentationObjectID("SM_BarelPlastic_A_1121[\w]*", name2idx['background']['index'], True)
        client.simSetSegmentationObjectID("SM_BarelPlastic_A_36[\w]*", name2idx['background']['index'], True)
        client.simSetSegmentationObjectID("SM_BarelPlastic_A_49[\w]*", name2idx['background']['index'], True)
        client.simSetSegmentationObjectID("SM_BarelPlastic_C_03_52[\w]*", name2idx['background']['index'], True)
        client.simSetSegmentationObjectID("SM_BarelPlastic_C_1127[\w]*", name2idx['background']['index'], True)
        client.simSetSegmentationObjectID("SM_BarelPlastic_C_26[\w]*", name2idx['background']['index'], True)
        client.simSetSegmentationObjectID("SM_BarelPlastic_D_03_58[\w]*", name2idx['background']['index'], True)
        client.simSetSegmentationObjectID("SM_BarelPlastic_D_03_58[\w]*", name2idx['background']['index'], True)
        client.simSetSegmentationObjectID("SM_BarelPlastic_D_94[\w]*", name2idx['background']['index'], True)
        client.simSetSegmentationObjectID("SM_BarelPlastic_D_97[\w]*", name2idx['background']['index'], True)
        client.simSetSegmentationObjectID("SM_BarelPlastic_D_100[\w]*", name2idx['background']['index'], True)
        client.simSetSegmentationObjectID("SM_BarelPlastic_D_7[\w]*", name2idx['background']['index'], True)
        client.simSetSegmentationObjectID("SM_BarelPlastic_D_8[\w]*", name2idx['background']['index'], True)
        client.simSetSegmentationObjectID("SM_BarelPlastic_D_1115[\w]*", name2idx['background']['index'], True)
        client.simSetSegmentationObjectID("SM_BarelPlastic_D_1131[\w]*", name2idx['background']['index'], True)
        client.simSetSegmentationObjectID("SM_BarelPlastic_D_105[\w]*", name2idx['background']['index'], True)
        client.simSetSegmentationObjectID("SM_BarelPlastic_D_39[\w]*", name2idx['background']['index'], True)
        client.simSetSegmentationObjectID("SM_BarelPlastic_D_40[\w]*", name2idx['background']['index'], True)
        client.simSetSegmentationObjectID("SM_FireExtinguisher_Part[\w]*", name2idx['background']['index'], True)
        client.simSetSegmentationObjectID("[\w]*FloorDecal_Arrow[\w]*", name2idx['background']['index'], True)
        client.simSetSegmentationObjectID("[\w]*Bucket[\w]*", name2idx['canister']['index'], True)
        client.simSetSegmentationObjectID("SM_CannedFood[\w]*", name2idx['canister']['index'], True)

    elif map_id == 'ModularWarehouse':
        print('MAPNAME: {}'.format(map_id))

        client.simSetSegmentationObjectID("Landscaped", name2idx['floor']['index'], True)
        client.simSetSegmentationObjectID("Plane[\w]*", name2idx['floor']['index'], True)
        client.simSetSegmentationObjectID("Sm_PalletJack[\w]*", name2idx['hand_truck']['index'], True)
        client.simSetSegmentationObjectID("Sm_BollardPole[\w]*", name2idx['fence']['index'], True)


        client.simSetSegmentationObjectID("SM_Warehouse_Wall01a50[\w]*", name2idx['ceiling']['index'], True)
        client.simSetSegmentationObjectID("SM_Warehouse_Wall01a288[\w]*", name2idx['ceiling']['index'], True)
        client.simSetSegmentationObjectID("SM_Warehouse_Wall01a52[\w]*", name2idx['ceiling']['index'], True)
        client.simSetSegmentationObjectID("SM_Warehouse_Wall01a53[\w]*", name2idx['ceiling']['index'], True)
        client.simSetSegmentationObjectID("SM_Warehouse_Wall01a289[\w]*", name2idx['ceiling']['index'], True)
        client.simSetSegmentationObjectID("SM_Warehouse_Wall01a51[\w]*", name2idx['ceiling']['index'], True)

        client.simSetSegmentationObjectID("[\w]*support[\w]*", name2idx['frame']['index'], True)


        client.simSetSegmentationObjectID("SM_WoodPallet01a69[\w]*", name2idx['box']['index'], True)
        client.simSetSegmentationObjectID("SM_WoodPallet01a68[\w]*", name2idx['box']['index'], True)
        client.simSetSegmentationObjectID("SM_CardboardBox04a26[\w]*", name2idx['box']['index'], True)
        client.simSetSegmentationObjectID("SM_WoodPallet01a66[\w]*", name2idx['box']['index'], True)
        client.simSetSegmentationObjectID("SM_WoodPallet01a63[\w]*", name2idx['box']['index'], True)
        client.simSetSegmentationObjectID("SM_WoodPallet01a57[\w]*", name2idx['box']['index'], True)
        client.simSetSegmentationObjectID("SM_WoodPallet01a58[\w]*", name2idx['box']['index'], True)
        client.simSetSegmentationObjectID("SM_WoodPallet01a60[\w]*", name2idx['box']['index'], True)
        client.simSetSegmentationObjectID("SM_WoodPallet01a61[\w]*", name2idx['box']['index'], True)

        client.simSetSegmentationObjectID("SM_WoodPallet01a25[\w]*", name2idx['box']['index'], True)
        client.simSetSegmentationObjectID("SM_WoodPallet01a26[\w]*", name2idx['box']['index'], True)
        client.simSetSegmentationObjectID("SM_WoodPallet01a16[\w]*", name2idx['box']['index'], True)
        client.simSetSegmentationObjectID("SM_WoodPallet01a17[\w]*", name2idx['box']['index'], True)
        client.simSetSegmentationObjectID("SM_WoodPallet01a23[\w]*", name2idx['box']['index'], True)
        client.simSetSegmentationObjectID("SM_WoodPallet01a22[\w]*", name2idx['box']['index'], True)
        client.simSetSegmentationObjectID("SM_WoodPallet01a19[\w]*", name2idx['box']['index'], True)
        client.simSetSegmentationObjectID("SM_WoodPallet01a20[\w]*", name2idx['box']['index'], True)
        client.simSetSegmentationObjectID("SM_WoodPallet01a32[\w]*", name2idx['box']['index'], True)
        client.simSetSegmentationObjectID("SM_WoodPallet01a31[\w]*", name2idx['box']['index'], True)
        client.simSetSegmentationObjectID("SM_WoodPallet01a34[\w]*", name2idx['box']['index'], True)
        client.simSetSegmentationObjectID("SM_WoodPallet01a35[\w]*", name2idx['box']['index'], True)
        client.simSetSegmentationObjectID("SM_WoodPallet01a28[\w]*", name2idx['box']['index'], True)
        client.simSetSegmentationObjectID("SM_WoodPallet01a29[\w]*", name2idx['box']['index'], True)
        client.simSetSegmentationObjectID("SM_WoodPallet01a13[\w]*", name2idx['box']['index'], True)
        client.simSetSegmentationObjectID("SM_WoodPallet01a14[\w]*", name2idx['box']['index'], True)
        client.simSetSegmentationObjectID("SM_WoodPallet01a37[\w]*", name2idx['box']['index'], True)
        client.simSetSegmentationObjectID("SM_WoodPallet01a38[\w]*", name2idx['box']['index'], True)
        client.simSetSegmentationObjectID("SM_WoodPallet01a51[\w]*", name2idx['box']['index'], True)
        client.simSetSegmentationObjectID("SM_WoodPallet01a52[\w]*", name2idx['box']['index'], True)
        client.simSetSegmentationObjectID("SM_WoodPallet01a54[\w]*", name2idx['box']['index'], True)
        client.simSetSegmentationObjectID("SM_WoodPallet01a55[\w]*", name2idx['box']['index'], True)
        client.simSetSegmentationObjectID("SM_WoodPallet01a39[\w]*", name2idx['box']['index'], True)
        client.simSetSegmentationObjectID("SM_WoodPallet01a40[\w]*", name2idx['box']['index'], True)
        client.simSetSegmentationObjectID("SM_WoodPallet01a42[\w]*", name2idx['box']['index'], True)
        client.simSetSegmentationObjectID("SM_WoodPallet01a43[\w]*", name2idx['box']['index'], True)
        client.simSetSegmentationObjectID("SM_WoodPallet01a48[\w]*", name2idx['box']['index'], True)
        client.simSetSegmentationObjectID("SM_WoodPallet01a49[\w]*", name2idx['box']['index'], True)
        client.simSetSegmentationObjectID("SM_WoodPallet01a45[\w]*", name2idx['box']['index'], True)
        client.simSetSegmentationObjectID("SM_WoodPallet01a46[\w]*", name2idx['box']['index'], True)

        client.simSetSegmentationObjectID("sm_woodpallet01aaa1_missing[\w]*", name2idx['box']['index'], True)
        client.simSetSegmentationObjectID("sm_woodpallet01aaaa23_missing[\w]*", name2idx['box']['index'], True)
        client.simSetSegmentationObjectID("sm_woodpallet01a5bb2235_missing[\w]*", name2idx['box']['index'], True)
        client.simSetSegmentationObjectID("sm_woodpallet01a541244dds2_missing[\w]*", name2idx['box']['index'], True)
        client.simSetSegmentationObjectID("sm_woodpallet01abc_553123_missing[\w]*", name2idx['box']['index'], True)

        client.simSetSegmentationObjectID("SM_Door_RollUp01a7[\w]*", name2idx['wall']['index'], True)
        client.simSetSegmentationObjectID("SM_Door_RollUp01a8[\w]*", name2idx['wall']['index'], True)
        client.simSetSegmentationObjectID("SM_Door_RollUp01a9[\w]*", name2idx['wall']['index'], True)
        client.simSetSegmentationObjectID("SM_Door_RollUp01a10[\w]*", name2idx['wall']['index'], True)
        client.simSetSegmentationObjectID("SM_Door_RollUp01a11[\w]*", name2idx['wall']['index'], True)
        client.simSetSegmentationObjectID("SM_Door_RollUp01a12[\w]*", name2idx['wall']['index'], True)


        client.simSetSegmentationObjectID("SM_Liftruck[\w]*", name2idx['vehicle']['index'], True)
        client.simSetSegmentationObjectID("pumptruck_01[\w]*", name2idx['hand_truck']['index'], True)
        client.simSetSegmentationObjectID("pumptruck_02[\w]*", name2idx['hand_truck']['index'], True)
        client.simSetSegmentationObjectID("tallsteps_01[\w]*", name2idx['ladder']['index'], True)

        client.simSetSegmentationObjectID("sm_paintbucket[\w]*", name2idx['canister']['index'], True)
        client.simSetSegmentationObjectID("truck_01[\w]*", name2idx['hand_truck']['index'], True)
        client.simSetSegmentationObjectID("truck_01b[\w]*", name2idx['hand_truck']['index'], True)


    elif map_id == 'ModularWarehouse_2':
        print('MAPNAME: {}'.format(map_id))

        client.simSetSegmentationObjectID("[\w]*rack[\w]*", name2idx['shelf']['index'], True)
        client.simSetSegmentationObjectID("[\w]*woodbox[\w]*", name2idx['pallet']['index'], True)
        client.simSetSegmentationObjectID("[\w]*floor[\w]*", name2idx['floor']['index'], True)
        client.simSetSegmentationObjectID("[\w]*corner[\w]*", name2idx['wall']['index'], True)
        client.simSetSegmentationObjectID("[\w]*warehouse_stairs[\w]*", name2idx['ladder']['index'], True)
        client.simSetSegmentationObjectID("[\w]*SM_IndustrialWarehouse_modular_Object29[\w]*", name2idx['wall']['index'], True)

        client.simSetSegmentationObjectID("[\w]*SM_IndustrialWarehouse_modular_Object30[\w]*", name2idx['column']['index'], True)
        client.simSetSegmentationObjectID("[\w]*SM_IndustrialWarehouse_modular_Object31[\w]*", name2idx['column']['index'], True)
        client.simSetSegmentationObjectID("[\w]*SM_IndustrialWarehouse_modular_Object32[\w]*", name2idx['column']['index'], True)
        client.simSetSegmentationObjectID("[\w]*SM_IndustrialWarehouse_modular_Object33[\w]*", name2idx['column']['index'], True)
        client.simSetSegmentationObjectID("[\w]*SM_IndustrialWarehouse_modular_Object34[\w]*", name2idx['column']['index'], True)
        client.simSetSegmentationObjectID("[\w]*SM_IndustrialWarehouse_modular_Object35[\w]*", name2idx['column']['index'], True)
        client.simSetSegmentationObjectID("[\w]*SM_IndustrialWarehouse_modular_Object36[\w]*", name2idx['column']['index'], True)

        client.simSetSegmentationObjectID("SM_barrels[\w]*", name2idx['canister']['index'], True)
        client.simSetSegmentationObjectID("canister[\w]*", name2idx['barrel']['index'], True)
        client.simSetSegmentationObjectID("sm_liftruck[\w]*", name2idx['vehicle']['index'], True)
        client.simSetSegmentationObjectID("pumptruck[\w]*", name2idx['hand_truck']['index'], True)
        client.simSetSegmentationObjectID("sm_palletjack[\w]*", name2idx['hand_truck']['index'], True)
        client.simSetSegmentationObjectID("sm_drum[\w]*", name2idx['barrel']['index'], True)
        client.simSetSegmentationObjectID("SM_Plywood04[\w]*", name2idx['beam']['index'], True)
        client.simSetSegmentationObjectID("SM_PaintBucket[\w]*", name2idx['canister']['index'], True)

        # client.simSetSegmentationObjectID("SM_barrels_250", name2idx['barrel']['index'], True)
        # client.simSetSegmentationObjectID("SM_barrels_231", name2idx['barrel']['index'], True)
        # client.simSetSegmentationObjectID("SM_barrels_230", name2idx['barrel']['index'], True)
        # client.simSetSegmentationObjectID("SM_barrels_742", name2idx['barrel']['index'], True)
        # client.simSetSegmentationObjectID("SM_barrels_249", name2idx['barrel']['index'], True)
        # client.simSetSegmentationObjectID("SM_barrels_248", name2idx['barrel']['index'], True)
        # client.simSetSegmentationObjectID("SM_barrels_247", name2idx['barrel']['index'], True)
        # client.simSetSegmentationObjectID("SM_barrels_9", name2idx['barrel']['index'], True)
        # client.simSetSegmentationObjectID("SM_barrels_245", name2idx['barrel']['index'], True)
        # client.simSetSegmentationObjectID("SM_barrels_246", name2idx['barrel']['index'], True)
        # client.simSetSegmentationObjectID("SM_barrels_736", name2idx['barrel']['index'], True)
        # client.simSetSegmentationObjectID("SM_barrels_732", name2idx['barrel']['index'], True)
        # client.simSetSegmentationObjectID("SM_barrels_726", name2idx['barrel']['index'], True)
        # client.simSetSegmentationObjectID("SM_barrels_729", name2idx['barrel']['index'], True)
        # client.simSetSegmentationObjectID("SM_barrels_226", name2idx['barrel']['index'], True)
        # client.simSetSegmentationObjectID("SM_barrels_723", name2idx['barrel']['index'], True)
        # client.simSetSegmentationObjectID("SM_barrels_221", name2idx['barrel']['index'], True)
        # client.simSetSegmentationObjectID("SM_barrels_220", name2idx['barrel']['index'], True)
        # client.simSetSegmentationObjectID("SM_barrels_718", name2idx['barrel']['index'], True)
        # client.simSetSegmentationObjectID("SM_barrels_213", name2idx['barrel']['index'], True)
        # client.simSetSegmentationObjectID("SM_barrels_140", name2idx['barrel']['index'], True)
        # client.simSetSegmentationObjectID("SM_barrels_141", name2idx['barrel']['index'], True)
        # client.simSetSegmentationObjectID("SM_barrels_80", name2idx['barrel']['index'], True)
        # client.simSetSegmentationObjectID("SM_barrels_79", name2idx['barrel']['index'], True)
        # client.simSetSegmentationObjectID("SM_barrels_78", name2idx['barrel']['index'], True)
        # client.simSetSegmentationObjectID("SM_barrels_77", name2idx['barrel']['index'], True)
        # client.simSetSegmentationObjectID("SM_barrels_69", name2idx['barrel']['index'], True)
        # client.simSetSegmentationObjectID("SM_barrels_130", name2idx['barrel']['index'], True)
        # client.simSetSegmentationObjectID("SM_barrels_71", name2idx['barrel']['index'], True)
        # client.simSetSegmentationObjectID("SM_barrels_72", name2idx['barrel']['index'], True)
        # client.simSetSegmentationObjectID("SM_barrels_76", name2idx['barrel']['index'], True)
        # client.simSetSegmentationObjectID("SM_barrels_131", name2idx['barrel']['index'], True)
        # client.simSetSegmentationObjectID("SM_barrels_74", name2idx['barrel']['index'], True)
        # client.simSetSegmentationObjectID("SM_barrels_73", name2idx['barrel']['index'], True)
        # client.simSetSegmentationObjectID("SM_barrels_68", name2idx['barrel']['index'], True)
        # client.simSetSegmentationObjectID("SM_barrels_67", name2idx['barrel']['index'], True)
        # client.simSetSegmentationObjectID("SM_barrels_66", name2idx['barrel']['index'], True)
        # client.simSetSegmentationObjectID("SM_barrels_65", name2idx['barrel']['index'], True)
        # client.simSetSegmentationObjectID("SM_barrels_57", name2idx['barrel']['index'], True)
        # client.simSetSegmentationObjectID("SM_barrels_107", name2idx['barrel']['index'], True)
        # client.simSetSegmentationObjectID("SM_barrels_59", name2idx['barrel']['index'], True)
        # client.simSetSegmentationObjectID("SM_barrels_60", name2idx['barrel']['index'], True)
        # client.simSetSegmentationObjectID("SM_barrels_111", name2idx['barrel']['index'], True)
        # client.simSetSegmentationObjectID("SM_barrels_110", name2idx['barrel']['index'], True)
        # client.simSetSegmentationObjectID("SM_barrels_109", name2idx['barrel']['index'], True)
        # client.simSetSegmentationObjectID("SM_barrels_108", name2idx['barrel']['index'], True)
        # client.simSetSegmentationObjectID("SM_barrels_56", name2idx['barrel']['index'], True)
        # client.simSetSegmentationObjectID("SM_barrels_55", name2idx['barrel']['index'], True)
        # client.simSetSegmentationObjectID("SM_barrels_54", name2idx['barrel']['index'], True)
        # client.simSetSegmentationObjectID("SM_barrels_53", name2idx['barrel']['index'], True)
        # client.simSetSegmentationObjectID("SM_barrels_70", name2idx['barrel']['index'], True)
        # client.simSetSegmentationObjectID("SM_barrels_2", name2idx['barrel']['index'], True)
        # client.simSetSegmentationObjectID("SM_barrels_75", name2idx['barrel']['index'], True)
        # client.simSetSegmentationObjectID("SM_barrels_4", name2idx['barrel']['index'], True)
        # client.simSetSegmentationObjectID("SM_barrels_82", name2idx['barrel']['index'], True)
        # client.simSetSegmentationObjectID("SM_barrels_51", name2idx['barrel']['index'], True)
        # client.simSetSegmentationObjectID("SM_barrels_50", name2idx['barrel']['index'], True)
        # client.simSetSegmentationObjectID("SM_barrels_49", name2idx['barrel']['index'], True)


        client.simSetSegmentationObjectID("SM_electric[\w]*", name2idx['electric_box']['index'], True)

    elif map_id == 'Warehouse':
        print('MAPNAME: {}'.format(map_id))

        client.simSetSegmentationObjectID("[\w]*piller[\w]*", name2idx['column']['index'], True)
        client.simSetSegmentationObjectID("[\w]*AngledWood[\w]*", name2idx['wall']['index'], True)
        client.simSetSegmentationObjectID("[\w]*MidWallBricks[\w]*", name2idx['beam']['index'], True)
        client.simSetSegmentationObjectID("[\w]*Woodslice[\w]*", name2idx['beam']['index'], True)
        client.simSetSegmentationObjectID("[\w]*roofwall[\w]*", name2idx['wall']['index'], True)
        client.simSetSegmentationObjectID("[\w]*multistorage[\w]*", name2idx['shelf']['index'], True)
        client.simSetSegmentationObjectID("[\w]*barrelstorage[\w]*", name2idx['shelf']['index'], True)
        client.simSetSegmentationObjectID("[\w]*liquidcan[\w]*", name2idx['box']['index'], True)
        client.simSetSegmentationObjectID("[\w]*cablesroller[\w]*", name2idx['background']['index'], True)
        client.simSetSegmentationObjectID("[\w]*cablewood[\w]*", name2idx['background']['index'], True)
        client.simSetSegmentationObjectID("SM_Lifter[\w]*", name2idx['vehicle']['index'], True)
        client.simSetSegmentationObjectID("[\w]*book[\w]*", name2idx['background']['index'], True)
        client.simSetSegmentationObjectID("[\w]*paper[\w]*", name2idx['background']['index'], True)
        client.simSetSegmentationObjectID("[\w]*metalstair[\w]*", name2idx['ladder']['index'], True)
        client.simSetSegmentationObjectID("[\w]*pumptruck[\w]*", name2idx['hand_truck']['index'], True)

        client.simSetSegmentationObjectID("[\w]*PaintBucket[\w]*", name2idx['canister']['index'], True)
        client.simSetSegmentationObjectID("SM_Liftruck[\w]*", name2idx['vehicle']['index'], True)









    print("Labeling Done")


