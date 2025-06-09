import time

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
    

except Exception as e:
    print("error", e)

sim.stopSimulation()
