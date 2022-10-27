IDX2STR={
    0: "Misc",
    1: "floor", # plate,
    2: "platform",
    3: "box",
    4: "barel",
    5: "cable", # wire, hose
    6: "door",
    7: "wall", # tunnel, cornice, plinth
    8: "ceiling", #
    9: "column",
    10: "pipe",
    11: "electrical_box", # electrical box, fuse box,
    12: "lamp", # light
    13: "rail",
    14: "cylinder",
    15: "sensor", #fire alarm, fire detector, sprinkler
    16: "fire_extinguisher",
    17: "hose_box",
    19: "cart",
    20: "bottle", #can
    21: "ladder",
    22: "sign", # warning_sign
}

def LabelSetting(mapname,client):
    if mapname=="bunker":  #jmpark
        found = client.simSetSegmentationObjectID("[\w]*", 0, True)
        print("set all object IDs to 0: %r" % (found))
        found = client.simSetSegmentationObjectID("floor[\w]*", 1, True)  # 바닥
        found = client.simSetSegmentationObjectID("plate[\w]*", 1, True)
        print("set all ground object IDs to 0: %r" % (found))
        found = client.simSetSegmentationObjectID("platform[\w]*", 2, True)  #
        found = client.simSetSegmentationObjectID("box[\w]*", 3, True)
        found = client.simSetSegmentationObjectID("barel[\w]*", 4, True)
        found = client.simSetSegmentationObjectID("wires[\w]*", 5, True)  # cable
        found = client.simSetSegmentationObjectID("wire[\w]*", 5, True)
        found = client.simSetSegmentationObjectID("hose[\w]*", 5, True)
        found = client.simSetSegmentationObjectID("door[\w]*", 6, True)  # 문
        found = client.simSetSegmentationObjectID("Door[\w]*", 6, True)
        found = client.simSetSegmentationObjectID("wall[\w]*", 7, True)  # 벽
        found = client.simSetSegmentationObjectID("tunnel[\w]*", 7, True)
        found = client.simSetSegmentationObjectID("ceiling[\w]*", 8, True)  # 천장
        # found = client.simSetSegmentationObjectID("plate[\w]*", 8, True)
        found = client.simSetSegmentationObjectID("column[\w]*", 9, True)  # 기둥
        found = client.simSetSegmentationObjectID("pipe[\w]*", 10, True)  # 파이프
        found = client.simSetSegmentationObjectID("cornice[\w]*", 7, True)  # 벽
        found = client.simSetSegmentationObjectID("plinth[\w]*", 7, True)  # 벽
        found = client.simSetSegmentationObjectID("electrical[\w]*", 11, True)  # 전자기기( electric box fuse box)
        found = client.simSetSegmentationObjectID("lamp[\w]*", 12, True)  # 램프
        found = client.simSetSegmentationObjectID("rail[\w]*", 13, True)  # 레일
        found = client.simSetSegmentationObjectID("crane_crane[\w]*", 13, True)
        found = client.simSetSegmentationObjectID("crane_circle[\w]*", 13, True)
        found = client.simSetSegmentationObjectID("cylinder[\w]*", 14, True)  # 긴 통 가스통

    elif mapname == "0002": #jhjang
        found = client.simSetSegmentationObjectID("[\w]*", 0, True)
        print("set all object IDs to 0: %r" % (found))
        found = client.simSetSegmentationObjectID("floor[\w]*", 1, True)  # 바닥
        found = client.simSetSegmentationObjectID("plate[\w]*", 1, True)
        print("set all ground object IDs to 0: %r" % (found))
        found = client.simSetSegmentationObjectID("platform[\w]*", 2, True)  #

        found = client.simSetSegmentationObjectID("box[\w]*", 3, True)
        found = client.simSetSegmentationObjectID("SM_wood_box[\w]*", 3, True)

        found = client.simSetSegmentationObjectID("barel[\w]*", 4, True)

        found = client.simSetSegmentationObjectID("SM_Cable_Triple_Str_a[\w]*", 5, True)  # cable
        found = client.simSetSegmentationObjectID("wire[\w]*", 5, True)
        found = client.simSetSegmentationObjectID("hose[\w]*", 5, True)

        found = client.simSetSegmentationObjectID("door[\w]*", 6, True)  # 문
        found = client.simSetSegmentationObjectID("SM_Single_Door[\w]*", 6, True)  # 문
        found = client.simSetSegmentationObjectID("Door[\w]*", 6, True)

        found = client.simSetSegmentationObjectID("SM_TunnelBlock[\w]*", 7, True)   # 벽
        found = client.simSetSegmentationObjectID("SM_DemoRoomBackWall[\w]*", 7, True)
        found = client.simSetSegmentationObjectID("tunnel[\w]*", 7, True)
        found = client.simSetSegmentationObjectID("cornice[\w]*", 7, True)  # 벽
        found = client.simSetSegmentationObjectID("plinth[\w]*", 7, True)  # 벽
        found = client.simSetSegmentationObjectID("BP_Tunnel[\w]*", 7, True)  # 벽

        found = client.simSetSegmentationObjectID("ceiling[\w]*", 8, True)  # 천장
        found = client.simSetSegmentationObjectID("plate[\w]*", 8, True)

        found = client.simSetSegmentationObjectID("column[\w]*", 9, True)  # 기둥
        found = client.simSetSegmentationObjectID("SM_Pillar[\w]*", 9, True)  # 기둥
        found = client.simSetSegmentationObjectID("sm_combined[\w]*", 10, True)  # 파이프
        found = client.simSetSegmentationObjectID("SM_Pipe05[\w]*", 10, True)  # 파이프

        found = client.simSetSegmentationObjectID("SM_electrical_box[\w]*", 11, True)  # 전자기기( electric box fuse box)
        found = client.simSetSegmentationObjectID("sm_fusebox-[\w]*", 11, True)  # 전자기기( electric box fuse box)
        found = client.simSetSegmentationObjectID("sm_fusebox_[\w]*", 11, True)  # 전자기기( electric box fuse box)

        found = client.simSetSegmentationObjectID("lamp[\w]*", 12, True)  # 램프

        found = client.simSetSegmentationObjectID("rail[\w]*", 13, True)  # 레일
        found = client.simSetSegmentationObjectID("crane_crane[\w]*", 13, True)
        found = client.simSetSegmentationObjectID("crane_circle[\w]*", 13, True)

        found = client.simSetSegmentationObjectID("cylinder[\w]*", 14, True)  # 긴 통 가스통

        found = client.simSetSegmentationObjectID("SM_FireAlarm[\w]*", 15, True)  # fire alarm
        found = client.simSetSegmentationObjectID("SM_Smoke_Detector[\w]*", 15, True)  # smoke detector

        found = client.simSetSegmentationObjectID("SM_Fire_Extinguisher[\w]*", 16, True)  # 소화기
        found = client.simSetSegmentationObjectID("SM_Fire_Extinguisher_Mount[\w]*", 0, True)  # 소화기 받침대

        found = client.simSetSegmentationObjectID("SM_Fire_Sprinkler[\w]*", 15, True)  # 스프링클러

        found = client.simSetSegmentationObjectID("SM_train[\w]*", 19, True)  # 수레 카트

        found = client.simSetSegmentationObjectID("SM_trash_bottle[\w]*", 20, True)  # 페트병
        found = client.simSetSegmentationObjectID("SM_trash_can[\w]*", 20, True)  # 캔

        #found = client.simSetSegmentationObjectID("SM_Fire_Axe[\w]*", 21, True)  # 도끼

    elif mapname == "0003": #sklee
        pass
    elif mapname == "0004": #VI1
        found = client.simSetSegmentationObjectID("[\w]*", 0, True)
        print("set all object IDs to 0: %r" % (found))
        found = client.simSetSegmentationObjectID("floor[\w]*", 1, True)  # 바닥
        found = client.simSetSegmentationObjectID("plate[\w]*", 1, True)
        found = client.simSetSegmentationObjectID("SM_TunnelGround[\w]*", 1, True)
        found = client.simSetSegmentationObjectID("SM_floor_concrete[\w]*", 1, True)

        print("set all ground object IDs to 0: %r" % (found))
        found = client.simSetSegmentationObjectID("platform[\w]*", 2, True)  #플랫폼
        found = client.simSetSegmentationObjectID("SM_PlatformCeiling[\w]*", 2, True)  #플랫폼

        found = client.simSetSegmentationObjectID("box[\w]*", 3, True)   #박스
        found = client.simSetSegmentationObjectID("SM_wood_box[\w]*", 3, True)
        found = client.simSetSegmentationObjectID("SM_brick_a[\w]*", 3, True)

        found = client.simSetSegmentationObjectID("barel[\w]*", 4, True)

        found = client.simSetSegmentationObjectID("SM_Cable_Triple_Str_a[\w]*", 5, True)  # cable, wire
        found = client.simSetSegmentationObjectID("SM_tunnel_wires_setB_[\w]*", 5, True)  # cable, wire
        found = client.simSetSegmentationObjectID("SM_wire_[\w]*", 5, True)  # cable, wire
        found = client.simSetSegmentationObjectID("SM_Cable_Bundled_2m_Str_[\w]*", 5, True)  # cable, wire
        found = client.simSetSegmentationObjectID("sm_cables_port-[\w]*", 5, True)  # cable, wire
        found = client.simSetSegmentationObjectID("SM_Several_Cable_25cm_[\w]*", 5, True)  # cable, wire
        found = client.simSetSegmentationObjectID("wire[\w]*", 5, True)
        found = client.simSetSegmentationObjectID("hose[\w]*", 5, True)

        found = client.simSetSegmentationObjectID("door[\w]*", 6, True)  # 문
        found = client.simSetSegmentationObjectID("SM_Single_Door[\w]*", 6, True)  # 문
        found = client.simSetSegmentationObjectID("Door[\w]*", 6, True)

        found = client.simSetSegmentationObjectID("SM_TunnelBlock[\w]*", 7, True)  # 벽
        found = client.simSetSegmentationObjectID("SM_TunnelWall[\w]*", 7, True)
        found = client.simSetSegmentationObjectID("SM_DemoRoomBackWall[\w]*", 7, True)
        found = client.simSetSegmentationObjectID("tunnel[\w]*", 7, True)
        found = client.simSetSegmentationObjectID("cornice[\w]*", 7, True)  # 벽
        found = client.simSetSegmentationObjectID("plinth[\w]*", 7, True)  # 벽
        found = client.simSetSegmentationObjectID("BP_Tunnel[\w]*", 7, True)  # 벽

        found = client.simSetSegmentationObjectID("ceiling[\w]*", 8, True)  # 천장

        found = client.simSetSegmentationObjectID("column[\w]*", 9, True)  # 기둥
        found = client.simSetSegmentationObjectID("SM_Pillar04_[\w]*", 9, True)  # 기둥

        found = client.simSetSegmentationObjectID("sm_combined[\w]*", 10, True)  # 파이프
        found = client.simSetSegmentationObjectID("big_pipe_[\w]*", 10, True)  # 파이프
        found = client.simSetSegmentationObjectID("SM_TunnelLargePipes[\w]*", 10, True)  # 파이프
        found = client.simSetSegmentationObjectID("SM_tunnel_pipes_[\w]*", 10, True)  # 파이프
        found = client.simSetSegmentationObjectID("sm_pipe_round_large_long_[\w]*", 10, True)  # 파이프
        found = client.simSetSegmentationObjectID("SM_Pipe_Thick_Stand_Str_[\w]*", 10, True)  # 파이프
        found = client.simSetSegmentationObjectID("SM_Pipe_Thin_Triple_Str_[\w]*", 10, True)  # 파이프
        found = client.simSetSegmentationObjectID("SM_tunnel_pipes_[\w]*", 10, True)  # 파이프


        found = client.simSetSegmentationObjectID("SM_electrical_box[\w]*", 11, True)  # 전자기기( electric box fuse box)
        found = client.simSetSegmentationObjectID("BP_Fusebox-[\w]*", 11, True)  # 전자기기( electric box fuse box)
        found = client.simSetSegmentationObjectID("SM_Fuse_Box_[\w]*", 11, True)  # 전자기기( electric box fuse box)

        found = client.simSetSegmentationObjectID("lamp[\w]*", 12, True)  # 램프
        found = client.simSetSegmentationObjectID("BP_Light_Ceiling[\w]*", 12, True)  # 천장 조명

        found = client.simSetSegmentationObjectID("rail[\w]*", 13, True)  # 레일
        found = client.simSetSegmentationObjectID("crane_crane[\w]*", 13, True)
        found = client.simSetSegmentationObjectID("crane_circle[\w]*", 13, True)

        found = client.simSetSegmentationObjectID("cylinder[\w]*", 14, True)  # 긴 통 가스통

        found = client.simSetSegmentationObjectID("SM_FireAlarm[\w]*", 15, True)  # fire alarm
        found = client.simSetSegmentationObjectID("SM_Smoke_Detector[\w]*", 15, True)  # smoke detector

        found = client.simSetSegmentationObjectID("SM_Fire_Extinguisher[\w]*", 16, True)  # 소화기
        found = client.simSetSegmentationObjectID("SM_fire_ex_[\w]*", 16, True)  # 소화기

        found = client.simSetSegmentationObjectID("SM_Fire_Extinguisher_Mount[\w]*", 17, True)  # 소화기 받침대

        found = client.simSetSegmentationObjectID("SM_Fire_Sprinkler[\w]*", 18, True)  # 스프링클러

        found = client.simSetSegmentationObjectID("SM_train[\w]*", 19, True)  # 수레 카트

        found = client.simSetSegmentationObjectID("SM_trash_bottle[\w]*", 20, True)  # 페트병
        found = client.simSetSegmentationObjectID("SM_trash_can[\w]*", 20, True)  # 캔

        found = client.simSetSegmentationObjectID("SM_Fire_Axe[\w]*", 21, True)  # 도끼

        found = client.simSetSegmentationObjectID("SM_warning_sign_[\w]*", 22, True)  # 표지판

    elif mapname == "0005": #VI2
        found = client.simSetSegmentationObjectID("[\w]*", 0, True)
        print("set all object IDs to 0: %r" % (found))
        found = client.simSetSegmentationObjectID("floor[\w]*", 1, True)  # 바닥
        found = client.simSetSegmentationObjectID("plate[\w]*", 1, True)
        found = client.simSetSegmentationObjectID("SM_TunnelGround[\w]*", 1, True)
        found = client.simSetSegmentationObjectID("SM_floor_concrete[\w]*", 1, True)

        print("set all ground object IDs to 0: %r" % (found))
        found = client.simSetSegmentationObjectID("platform[\w]*", 2, True)  # 플랫폼
        found = client.simSetSegmentationObjectID("SM_PlatformCeiling[\w]*", 2, True)  # 플랫폼

        found = client.simSetSegmentationObjectID("box[\w]*", 3, True)  # 박스
        found = client.simSetSegmentationObjectID("SM_wood_box[\w]*", 3, True)
        found = client.simSetSegmentationObjectID("SM_brick_a[\w]*", 3, True)
        found = client.simSetSegmentationObjectID("SM_storage_box_[\w]*", 3, True)
        found = client.simSetSegmentationObjectID("SM_tool_box_[\w]*", 3, True)


        found = client.simSetSegmentationObjectID("barel[\w]*", 4, True)

        found = client.simSetSegmentationObjectID("SM_Cable_Triple_Str_a[\w]*", 5, True)  # cable, wire
        found = client.simSetSegmentationObjectID("SM_tunnel_wires_setB_[\w]*", 5, True)  # cable, wire
        found = client.simSetSegmentationObjectID("SM_wire_[\w]*", 5, True)  # cable, wire
        found = client.simSetSegmentationObjectID("SM_Cable_Bundled_2m_Str_[\w]*", 5, True)  # cable, wire
        found = client.simSetSegmentationObjectID("sm_cables_port-[\w]*", 5, True)  # cable, wire
        found = client.simSetSegmentationObjectID("SM_Several_Cable_25cm_[\w]*", 5, True)  # cable, wire
        found = client.simSetSegmentationObjectID("wire[\w]*", 5, True)
        found = client.simSetSegmentationObjectID("hose[\w]*", 5, True)

        found = client.simSetSegmentationObjectID("door[\w]*", 6, True)  # 문
        found = client.simSetSegmentationObjectID("SM_Single_Door[\w]*", 6, True)  # 문
        found = client.simSetSegmentationObjectID("Door[\w]*", 6, True)

        found = client.simSetSegmentationObjectID("SM_TunnelBlock[\w]*", 7, True)  # 벽
        found = client.simSetSegmentationObjectID("SM_TunnelWall[\w]*", 7, True)
        found = client.simSetSegmentationObjectID("SM_DemoRoomBackWall[\w]*", 7, True)
        found = client.simSetSegmentationObjectID("tunnel[\w]*", 7, True)
        found = client.simSetSegmentationObjectID("cornice[\w]*", 7, True)  # 벽
        found = client.simSetSegmentationObjectID("plinth[\w]*", 7, True)  # 벽
        found = client.simSetSegmentationObjectID("BP_Tunnel[\w]*", 7, True)  # 벽

        found = client.simSetSegmentationObjectID("ceiling[\w]*", 8, True)  # 천장

        found = client.simSetSegmentationObjectID("column[\w]*", 9, True)  # 기둥
        found = client.simSetSegmentationObjectID("SM_Pillar04_[\w]*", 9, True)  # 기둥

        found = client.simSetSegmentationObjectID("sm_combined[\w]*", 10, True)  # 파이프
        found = client.simSetSegmentationObjectID("big_pipe_[\w]*", 10, True)  # 파이프
        found = client.simSetSegmentationObjectID("SM_TunnelLargePipes[\w]*", 10, True)  # 파이프
        found = client.simSetSegmentationObjectID("SM_tunnel_pipes_[\w]*", 10, True)  # 파이프
        found = client.simSetSegmentationObjectID("sm_pipe_round_large_long_[\w]*", 10, True)  # 파이프
        found = client.simSetSegmentationObjectID("SM_Pipe_Thick_Stand_Str_[\w]*", 10, True)  # 파이프
        found = client.simSetSegmentationObjectID("SM_Pipe_Thin_Triple_Str_[\w]*", 10, True)  # 파이프
        found = client.simSetSegmentationObjectID("SM_tunnel_pipes_[\w]*", 10, True)  # 파이프

        found = client.simSetSegmentationObjectID("SM_electrical_box[\w]*", 11, True)  # 전자기기( electric box fuse box)
        found = client.simSetSegmentationObjectID("BP_Fusebox-[\w]*", 11, True)  # 전자기기( electric box fuse box)
        found = client.simSetSegmentationObjectID("SM_Fuse_Box_[\w]*", 11, True)  # 전자기기( electric box fuse box)

        found = client.simSetSegmentationObjectID("lamp[\w]*", 12, True)  # 램프
        found = client.simSetSegmentationObjectID("BP_Light_Ceiling[\w]*", 12, True)  # 천장 조명

        found = client.simSetSegmentationObjectID("rail[\w]*", 13, True)  # 레일
        found = client.simSetSegmentationObjectID("crane_crane[\w]*", 13, True)
        found = client.simSetSegmentationObjectID("crane_circle[\w]*", 13, True)

        found = client.simSetSegmentationObjectID("cylinder[\w]*", 14, True)  # 긴 통 가스통

        found = client.simSetSegmentationObjectID("SM_FireAlarm[\w]*", 15, True)  # fire alarm
        found = client.simSetSegmentationObjectID("SM_Smoke_Detector[\w]*", 15, True)  # smoke detector

        found = client.simSetSegmentationObjectID("SM_Fire_Extinguisher[\w]*", 16, True)  # 소화기
        found = client.simSetSegmentationObjectID("SM_fire_ex_[\w]*", 16, True)  # 소화기

        found = client.simSetSegmentationObjectID("SM_Fire_Extinguisher_Mount[\w]*", 17, True)  # 소화기 받침대

        found = client.simSetSegmentationObjectID("SM_Fire_Sprinkler[\w]*", 15, True)  # 스프링클러

        found = client.simSetSegmentationObjectID("SM_train[\w]*", 19, True)  # 수레 카트

        found = client.simSetSegmentationObjectID("SM_trash_bottle[\w]*", 20, True)  # 페트병
        found = client.simSetSegmentationObjectID("SM_trash_can[\w]*", 20, True)  # 캔

        #found = client.simSetSegmentationObjectID("SM_Fire_Axe[\w]*", 21, True)  # 도끼
        #found = client.simSetSegmentationObjectID("SM_Fire_Shovel[\w]*", 21, True)  # 도끼

        found = client.simSetSegmentationObjectID("SM_warning_sign_[\w]*", 22, True)  # 표지판
        found = client.simSetSegmentationObjectID("SM_sign_[\w]*", 22, True)  # 표지판


        found = client.simSetSegmentationObjectID("SM_Hose_Box[\w]*", 16, True)  # 소화기 호스 박스

        found = client.simSetSegmentationObjectID("SM_ladder_[\w]*", 21, True)  # 사다리