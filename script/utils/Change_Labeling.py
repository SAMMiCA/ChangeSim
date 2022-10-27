
def ChangeLabeling(client, name2idx):
    idx2name = {v['index']:k for k,v in name2idx.items()}
    change_types = {'_static':0*50,'_new':1*50,'_missing':2*50,'_rotated':3*50,'_replaced':4*50}
    count = {'_static':0,'_new':0,'_missing':0,'_rotated':0,'_replaced':0}
    obj_list = client.simListSceneObjects()
    for obj_name in obj_list:
        for change_label,change_idx in change_types.items():
            if change_label in obj_name:
                obj_ID_before = client.simGetSegmentationObjectID(obj_name.lower())
                found = client.simSetSegmentationObjectID(obj_name,obj_ID_before+change_idx)
                status = 'SUCCESS' if found else 'FAILED'



                try:
                    semantic_label = idx2name[obj_ID_before]
                except:
                    import pdb;pdb.set_trace()
                obj_ID_after = client.simGetSegmentationObjectID(obj_name.lower())
                print('[{}][{}][{}] {}: remapped from {} to {}'.format(status,change_label[1:],
                                                                       semantic_label, obj_name,
                                                                   obj_ID_before,obj_ID_after))
                count[change_label]+=1
    print('Change Labeling Done.')
    print('Change stats: \n{}'.format(count))
