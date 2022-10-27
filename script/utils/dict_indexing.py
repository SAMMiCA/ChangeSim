# -*- coding: utf-8 -*-


def Dict_indexing():
    '''
    # 24 semantic classes + 1 background class
    :return: Dict
    '''
    Dict = {}
    Dict['background']={'index':0,'subnames':[]}
    Dict['column']={'index':1,'subnames':['pillar','pilar']}
    Dict['pipe']={'index':2,'subnames':['tube']}
    Dict['wall']={'index':3,'subnames':['tunnel']}
    Dict['beam']={'index':4,'subnames':[]}
    Dict['floor']={'index':5,'subnames':['slam','ground','road','walk','floorpanel']}
    Dict['frame']={'index':6,'subnames':['scafolding','scaffolding','scaffold','formwork','pole','support']}
    Dict['fence']={'index':7,'subnames':['fencning']}
    Dict['wire']={'index':8,'subnames':['wirecylinder']}
    Dict['ceiling']={'index':13,'subnames':['roof']}
    Dict['duct']={'index':15,'subnames':['vent','ventilation']}
    Dict['lamp']={'index':17,'subnames':['light']}
    Dict['shelf']={'index':19,'subnames':['drawer','rack','locker','cabinet']}
    Dict['door']={'index':20,'subnames':['gate']}
    Dict['barrel']={'index':21,'subnames':['barel','drum','tank']}
    Dict['sign']={'index':22,'subnames':['signcver']}
    Dict['box']={'index':23,'subnames':['paperbox','bin','cube','crateplastic']}
    Dict['bag']={'index':24,'subnames':[]}
    Dict['electric_box']={'index':25,'subnames':['fusebox','switchboard','electricalsupply',
                                             'electric_panel','powerbox','control_panel']}
    Dict['vehicle']={'index':26,'subnames':['truck','trailer','transporter','forklift']}
    Dict['ladder']={'index':27,'subnames':[]}
    Dict['canister']={'index':28,'subnames':['can','bottle','cylinder','keg']}
    Dict['extinguisher']={'index':29,'subnames':['fire_ex']}
    Dict['pallet'] = {'index': 30, 'subnames': ['palete', 'palette']}
    Dict['hand_truck'] = {'index': 31, 'subnames': ['pumptruck','pallet_jack']}


    Dict['trash']={'index':18,'subnames':['debris','book','paper']} # Background
    Dict['gril']={'index':16,'subnames':['grid']}  # Background
    Dict['stair']={'index':14,'subnames':[]} # Background
    Dict['rail']={'index':12,'subnames':[]} # Background
    Dict['railing']={'index':11,'subnames':[]} # Background
    Dict['window']={'index':10,'subnames':['glass_panel']} # Background
    Dict['cable']={'index':9,'subnames':[]} # Background

    return Dict


Dict = Dict_indexing()



