# -*- coding: utf-8 -*-


def Dict_indexing():
    Dict = {}
    Dict['background']={'index':0,'subnames':[]}
    Dict['column']={'index':1,'subnames':['pillar','pilar']} #sero
    Dict['pipe']={'index':2,'subnames':['tube']}
    Dict['wall']={'index':3,'subnames':['tunnel']}
    Dict['beam']={'index':4,'subnames':[]} # garo
    Dict['floor']={'index':5,'subnames':['slam','ground','road','walk','floorpanel']}
    Dict['frame']={'index':6,'subnames':['scafolding','scaffolding','scaffold','formwork','pole','support']}
    Dict['fence']={'index':7,'subnames':['fencning']}

    Dict['wire']={'index':8,'subnames':['wirecylinder']}
    Dict['cable']={'index':9,'subnames':[]}
    Dict['window']={'index':10,'subnames':['glass_panel']}
    Dict['railing']={'index':11,'subnames':[]}
    Dict['rail']={'index':12,'subnames':[]}
    Dict['ceiling']={'index':13,'subnames':['roof']}
    Dict['stair']={'index':14,'subnames':[]}
    Dict['duct']={'index':15,'subnames':['vent','ventilation']}
    Dict['gril']={'index':16,'subnames':['grid']}  # bunker's platform

    Dict['lamp']={'index':17,'subnames':['light']} # GOOD
    Dict['trash']={'index':18,'subnames':['debris','book','paper']} # GOOD
    Dict['shelf']={'index':19,'subnames':['drawer','rack','locker','cabinet']}
    Dict['door']={'index':20,'subnames':['gate']} #GOOD
    Dict['barrel']={'index':21,'subnames':['barel','drum','tank']} # GOOD
    Dict['sign']={'index':22,'subnames':['signcver']} # GOOD
    Dict['box']={'index':23,'subnames':['paperbox','bin','cube','crateplastic']} # Good
    Dict['bag']={'index':24,'subnames':[]} # GOOD
    Dict['electric_box']={'index':25,'subnames':['fusebox','switchboard','electricalsupply',
                                             'electric_panel','powerbox','control_panel']} # GOOD
    Dict['vehicle']={'index':26,'subnames':['truck','trailer','transporter','forklift']}
    Dict['ladder']={'index':27,'subnames':[]} # GOOD
    Dict['canister']={'index':28,'subnames':['can','bottle','cylinder','keg']}
    Dict['extinguisher']={'index':29,'subnames':['fire_ex']} # GOOD
    Dict['pallet'] = {'index': 30, 'subnames': ['palete', 'palette']}  # GOOD
    Dict['hand_truck'] = {'index': 31, 'subnames': ['pumptruck','pallet_jack']}  # GOOD

    # Dict['holder']={'index':46,'subnames':['sprinkler',cube'suport','support','bracket']}
    #Dict['vent']={'index':25,'subnames':[]}
    #Dict['tube']={'index':5,'subnames':[]}
    # Dict['roof']={'index':11,'subnames':[]} # overlapping with ceiling
    # Dict['panel']={'index':12,'subnames':[]}
    # Dict['container']={'index':19,'subnames':[]} few asset
    #Dict['wheel']={'index':23,'subnames':[]} few asset
    #Dict['locker']={'index':31,'subnames':['cabinet']}
    # Dict['floormat']={'index':34,'subnames':['showercarpet']} too few asset
    # Dict['crane']={'index':36,'subnames':[]} # too few asset
    # Dict['camera']={'index':38,'subnames':[]}
    # Dict['book']={'index':42,'subnames':[]}
    # Dict['cylinder']={'index':41,'subnames':['keg']}
    # Dict['signcover']={'index':43,'subnames':[]}
    # Dict['airconditiongplatform']={'index':44,'subnames':[]} too few object
    # Dict['pole']={'index':45,'subnames':[]}

    return Dict


Dict = Dict_indexing()



