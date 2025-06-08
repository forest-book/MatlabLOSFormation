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
    quadcopter0_position = sim.getObjectPosition(quadcopter_handles[0], -1)
    quadcopter0_orientation = sim.getObjectOrientation(quadcopter_handles[0], -1)
    time.sleep(0.1)
    

except Exception as e:
    print("error", e)

sim.stopSimulation()
