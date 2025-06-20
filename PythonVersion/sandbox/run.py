# import time

# import numpy as np

# # CoppeliaSimとの連携
# from coppeliasim_zmqremoteapi_client import RemoteAPIClient
# from losMethods import HelperMethod

# from quadrotor import Quadrotor

# client = RemoteAPIClient()
# sim = client.getObject('sim')

# sim.setStepping(True)

# # シミュレーション開始
# sim.startSimulation()
# time.sleep(0.1)

# # try:
# print("Connected to remote API server")

# quadcopter_names:list = [f"Quadcopter[{i}]" for i in range(0,5)]
# quadcopter_handles:list = []
# quadcopter_indicator_handles:list = []
# # LiDARセンサ情報取得のためのhandleを2次元リストで管理
# lidar_handles:list = []
# for i in range(5):
#     lidar_row = []
#     for j in range(2):
#         handle = sim.getObject(f"/Quadcopter[{i}]/fastHokuyo[{j}]")
#         lidar_row.append(handle)
#     lidar_handles.append(lidar_row)
# # 例: lidar_handles[0][0] で Quadcopter[0]/fastHokuyo[0] のhandle
# for i in range(0, len(quadcopter_names)):
#     # オブジェクトのhandleを取得
#     # [/{オブジェクト名}のように、"/"をつけないとうまくいかない]
#     quadcopter_handles.append(sim.getObject(f"/{quadcopter_names[i]}"))
#     # targetはcoppeliasimのドローン位置を示す色付きの球体
#     quadcopter_indicator_handles.append(sim.getObject(f"/target[{i}]"))
#     if quadcopter_handles[i] == -1:
#         print(f"not found {quadcopter_names[i]}")
#     else:
#         # printで確認
#         print(f"Quadcopter[{i}] handle; {quadcopter_handles[i]}")
#         print(f"target[{i}] handle; {quadcopter_indicator_handles[i]}")

# # リーダーの目標点(円筒形)のhandleを取得
# cylinder_handle = sim.getObject("/Cylinder")
# if cylinder_handle == -1:
#     print("not found Cylinder")
# else:
#     print(f"Cylinder handle; {cylinder_handle}")

import numpy as np

# 初期位置（例: x, y, z）
start_pos = np.array([-400.0, 0.0, 220.0])
# 目標地点（ここを変数で設定）
goal_pos = np.array([1100.0, -15.0, 250.0])

# パラメータ
dt = 1.0
leader_speed = 5.0
max_steps = 1000

# 現在位置
pos = start_pos.copy()

for step in range(max_steps):
    direction = goal_pos - pos
    dist = np.linalg.norm(direction)
    if dist < 1.0:
        print(f"到達: step={step}, pos={pos}")
        break
    direction = direction / dist
    speed_vec = direction * leader_speed
    pos = pos + dt * speed_vec
    print(f"step={step}, pos={pos}")

# 最終位置出力
print(f"最終位置: {pos}")
