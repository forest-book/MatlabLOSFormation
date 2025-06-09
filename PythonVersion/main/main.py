import time

import numpy as np

# CoppeliaSimとの連携
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

client = RemoteAPIClient()
sim = client.getObject('sim')

sim.setStepping(True)

# シミュレーション開始
sim.startSimulation()
time.sleep(0.1)

try:
    print("Connected to remote API server")

    quadcopter_names:list = [f"Quadcopter[{i}]" for i in range(0,5)]
    quadcopter_handles:list = []
    quadcopter_indicator_handles:list = []
    # LiDARセンサ情報取得のためのhandleを2次元リストで管理
    lidar_handles:list = []
    for i in range(5):
        lidar_row = []
        for j in range(2):
            handle = sim.getObject(f"/Quadcopter[{i}]/fastHokuyo[{j}]")
            lidar_row.append(handle)
        lidar_handles.append(lidar_row)
    # 例: lidar_handles[0][0] で Quadcopter[0]/fastHokuyo[0] のhandle
    for i in range(0, len(quadcopter_names)):
        # オブジェクトのhandleを取得
        # [/{オブジェクト名}のように、"/"をつけないとうまくいかない]
        quadcopter_handles.append(sim.getObject(f"/{quadcopter_names[i]}"))
        # targetはcoppeliasimのドローン位置を示す色付きの球体
        quadcopter_indicator_handles.append(sim.getObject(f"/target[{i}]"))
        if quadcopter_handles[i] == -1:
            print(f"not found {quadcopter_names[i]}")
        else:
            # printで確認
            print(f"Quadcopter[{i}] handle; {quadcopter_handles[i]}")
            print(f"target[{i}] handle; {quadcopter_indicator_handles[i]}")
    
    # リーダーの目標点(円筒形)のhandleを取得
    cylinder_handle = sim.getObject("/Cylinder")
    if cylinder_handle == -1:
        print("not found Cylinder")
    else:
        print(f"Cylinder handle; {cylinder_handle}")

    # positionとorientationの取得
    # 一台目
    quadcopter0_position = sim.getObjectPosition(quadcopter_handles[0], -1)
    quadcopter0_orientation = sim.getObjectOrientation(quadcopter_handles[0], -1)
    time.sleep(0.1)
    init_data = sim.sim.readCustomDataBlock(lidar_handles[0][0], 'scan_ranges11')
    init_data = sim.sim.readCustomDataBlock(lidar_handles[0][1], 'scan_ranges12')
    time.sleep(0.1)
    
    # 二台目
    quadcopter1_position = sim.getObjectPosition(quadcopter_handles[1], -1)
    quadcopter1_orientation = sim.getObjectOrientation(quadcopter_handles[1], -1)
    time.sleep(0.1)
    init_data = sim.sim.readCustomDataBlock(lidar_handles[1][0], 'scan_ranges21')
    init_data = sim.sim.readCustomDataBlock(lidar_handles[1][1], 'scan_ranges22')
    time.sleep(0.1)

    # 三台目
    quadcopter2_position = sim.getObjectPosition(quadcopter_handles[2], -1)
    quadcopter2_orientation = sim.getObjectOrientation(quadcopter_handles[2], -1)
    time.sleep(0.1)
    init_data = sim.sim.readCustomDataBlock(lidar_handles[2][0], 'scan_ranges31')
    init_data = sim.sim.readCustomDataBlock(lidar_handles[2][1], 'scan_ranges32')
    time.sleep(0.1)

    # 四台目
    quadcopter3_position = sim.getObjectPosition(quadcopter_handles[3], -1)
    quadcopter3_orientation = sim.getObjectOrientation(quadcopter_handles[3], -1)
    time.sleep(0.1)
    init_data = sim.sim.readCustomDataBlock(lidar_handles[3][0], 'scan_ranges41')
    init_data = sim.sim.readCustomDataBlock(lidar_handles[3][1], 'scan_ranges42')
    time.sleep(0.1)

    # 五台目
    quadcopter4_position = sim.getObjectPosition(quadcopter_handles[4], -1)
    quadcopter4_orientation = sim.getObjectOrientation(quadcopter_handles[4], -1)
    time.sleep(0.1)
    init_data = sim.sim.readCustomDataBlock(lidar_handles[4][0], 'scan_ranges51')
    init_data = sim.sim.readCustomDataBlock(lidar_handles[4][1], 'scan_ranges52')
    time.sleep(0.1)

    # フォーメーション制御(以下の部分が処理のメイン)
    # 変数の規定(シミュレーション状況により変更する)

    # ステップ数(シミュレーション時間)
    simulation_time = 20000

    # リーダ機の機体番号を格納(値は簡易的に1としている)
    quad_leader_num = 1

    # リーダを含めたクワッドローターの数
    quadcopter_counts = 5

    # 距離の閾値(侵入禁止領域の設定)
    distance_threshold = 80

    # 更新ステップ幅,Δt
    dt = 1

    # k0lはフォロワの目標速度(制御入力)の方向を計算する際のゲイン(k0,kl)
    k01 = np.zeros((quadcopter_counts - 1, 2))
    k01[0, :] = [5, 200]
    k01[1, :] = [5, 200]
    k01[2, :] = [5, 200]
    k01[3, :] = [5, 200]

    # kpsはフォロワの目標速度(制御入力)の大きさを計算する際のゲイン(kp,ks)
    kps = np.zeros((quadcopter_counts - 1, 2))
    kps[0, :] = [1, 1]
    kps[1, :] = [1, 1]
    kps[2, :] = [1, 1]
    kps[3, :] = [1, 1]
    
    # フォーメーションの数
    formation_num = 2

    # フォーメーションを指定(todo enumに書き換え)
    current_formation = 2

    # リーダと各フォロワの距離を規定
    range_with_leader = np.zeros(quadcopter_counts - 1, 2)
    range_with_leader[0, :] = [320, 240, 168, 80] # 直線フォーメーション
    range_with_leader[1, :] = [200, 200, 100, 100] # V字フォーメーション

    # リーダの目標点数
    goal_num = 2

    # リーダの速さ設定
    leader_speed = 5

    # リーダの目標到達地点
    goal_for_leader = np.zeros(3, goal_num)

    #goal_for_leader[:, 0] = [500, -15, 250]
    goal_for_leader[:, 0] = [1100, -15, 250]
    goal_for_leader[:, 1] = [-650, -15, 300]


except Exception as e:
    print("error", e)

sim.stopSimulation()
