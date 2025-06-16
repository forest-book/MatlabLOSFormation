import time

import numpy as np

# CoppeliaSimとの連携
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from losMethods import HelperMethod

from quadrotor import Quadrotor

client = RemoteAPIClient()
sim = client.getObject('sim')

sim.setStepping(True)

# シミュレーション開始
sim.startSimulation()
time.sleep(0.1)

# try:
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
init_data = sim.readCustomDataBlock(lidar_handles[0][0], 'scan_ranges11')
init_data = sim.readCustomDataBlock(lidar_handles[0][1], 'scan_ranges12')
time.sleep(0.1)

# 二台目
quadcopter1_position = sim.getObjectPosition(quadcopter_handles[1], -1)
quadcopter1_orientation = sim.getObjectOrientation(quadcopter_handles[1], -1)
time.sleep(0.1)
init_data = sim.readCustomDataBlock(lidar_handles[1][0], 'scan_ranges21')
init_data = sim.readCustomDataBlock(lidar_handles[1][1], 'scan_ranges22')
time.sleep(0.1)

# 三台目
quadcopter2_position = sim.getObjectPosition(quadcopter_handles[2], -1)
quadcopter2_orientation = sim.getObjectOrientation(quadcopter_handles[2], -1)
time.sleep(0.1)
init_data = sim.readCustomDataBlock(lidar_handles[2][0], 'scan_ranges31')
init_data = sim.readCustomDataBlock(lidar_handles[2][1], 'scan_ranges32')
time.sleep(0.1)

# 四台目
quadcopter3_position = sim.getObjectPosition(quadcopter_handles[3], -1)
quadcopter3_orientation = sim.getObjectOrientation(quadcopter_handles[3], -1)
time.sleep(0.1)
init_data = sim.readCustomDataBlock(lidar_handles[3][0], 'scan_ranges41')
init_data = sim.readCustomDataBlock(lidar_handles[3][1], 'scan_ranges42')
time.sleep(0.1)

# 五台目
quadcopter4_position = sim.getObjectPosition(quadcopter_handles[4], -1)
quadcopter4_orientation = sim.getObjectOrientation(quadcopter_handles[4], -1)
time.sleep(0.1)
init_data = sim.readCustomDataBlock(lidar_handles[4][0], 'scan_ranges51')
init_data = sim.readCustomDataBlock(lidar_handles[4][1], 'scan_ranges52')
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
range_with_leader = np.zeros((quadcopter_counts - 1, formation_num))
range_with_leader[:, 0] = [320, 240, 168, 80] # 直線フォーメーション
range_with_leader[:, 1] = [200, 200, 100, 100] # V字フォーメーション

# リーダの目標点数
goal_num = 2

# リーダの速さ設定
leader_speed = 5

# リーダの目標到達地点
goal_for_leader = np.zeros((3, goal_num))

#goal_for_leader[:, 0] = [500, -15, 250]
goal_for_leader[:, 0] = [1100, -15, 250]
goal_for_leader[:, 1] = [-650, -15, 300]

# クワッドロータのインスタンス
quadrotor = Quadrotor(quadcopter_counts, simulation_time)

# フォロワ間のベクトルを格納する配列
# フォロワ番号(属性番号の2以降に紐づく情報)
# VBF(vector between followers):フォロワ間のベクトル
# VBF_dir:VBFの単位ベクトル
VBF = np.zeros((3, simulation_time, quadcopter_counts - 1, quadcopter_counts - 1))
VBF_dir = np.zeros((3, simulation_time, quadcopter_counts - 1, quadcopter_counts - 1))

# 目標点に到達した時間を格納する配列
# arrive_timeに格納された時間を用いて目標点を表示する
arrive_time = np.zeros((goal_num, 1))

# 初期設定

# 属性番号の初期化
quadrotor.attribute_num = np.array([1, 2, 3, 4, 5])

# AOR(Angle of rotation):フォロワの追従位置を決定する角度
# 一列目がΨ,二列目がΦ
# ψがローカルでのz軸中心の回転 θがローカルでのx軸周りの回転
AOR = np.zeros((quadcopter_counts - 1, 2, formation_num))

# フォーメーション1
AOR[0, :, 0] = [np.deg2rad(180), np.deg2rad(0)]   # フォロワ1の回転角ΨとΦ
AOR[1, :, 0] = [np.deg2rad(180), np.deg2rad(0)]   # フォロワ2の回転角ΨとΦ
AOR[2, :, 0] = [np.deg2rad(180), np.deg2rad(0)]   # フォロワ3の回転角ΨとΦ
AOR[3, :, 0] = [np.deg2rad(180), np.deg2rad(0)]   # フォロワ4の回転角ΨとΦ

# フォーメーション2
AOR[0, :, 1] = [np.deg2rad(-140), np.deg2rad(0)]  # フォロワ1の回転角ΨとΦ
AOR[1, :, 1] = [np.deg2rad(140),  np.deg2rad(0)]  # フォロワ2の回転角ΨとΦ
AOR[2, :, 1] = [np.deg2rad(140),  np.deg2rad(0)]  # フォロワ3の回転角ΨとΦ
AOR[3, :, 1] = [np.deg2rad(-140), np.deg2rad(0)]  # フォロワ4の回転角ΨとΦ

# リーダーの初期座標
quadrotor.coordinate[0, 0, 0] = -400  # x
quadrotor.coordinate[1, 0, 0] = 0     # y
quadrotor.coordinate[2, 0, 0] = 220   # z

# フォロワー1
quadrotor.coordinate[0, 0, 1] = -420
quadrotor.coordinate[1, 0, 1] = -110
quadrotor.coordinate[2, 0, 1] = 250

# フォロワー2
quadrotor.coordinate[0, 0, 2] = -500
quadrotor.coordinate[1, 0, 2] = -60
quadrotor.coordinate[2, 0, 2] = 250

# フォロワー3
quadrotor.coordinate[0, 0, 3] = -520
quadrotor.coordinate[1, 0, 3] = 45
quadrotor.coordinate[2, 0, 3] = 250

# フォロワー4
quadrotor.coordinate[0, 0, 4] = -600
quadrotor.coordinate[1, 0, 4] = -110
quadrotor.coordinate[2, 0, 4] = 250

# リーダの目標地点の変更回数
change_num = 1

# 全目標地点に到達した時点で停滞させるための座標
completed_coordinate = np.zeros((3, quadcopter_counts))

# 上記座標を取得するためのフラグ
is_completed = False

# 例外に到達したことを示すフラグ
is_exception_raised = False

# 例外時に停滞させるための座標
exception_coordinate = np.zeros((3, quadcopter_counts))

# 障害物センサ配列の要素数
stepnum = 684

# 障害物があるかの判定
is_obstacle = False

# ワールド座標系でみたローカル軸
local_axis = np.zeros((3, 3))

# クワッドロータの初期座標をCoppeliaSimに反映している
for i in range(0, quadcopter_counts):
    sim.setObjectPosition(quadcopter_handles[i], (quadrotor.coordinate[:, 0, i] / 100).tolist(), -1)
sim.setObjectPosition(cylinder_handle, (goal_for_leader[:, 0] / 100).tolist(), -1)
sim.step()
time.sleep(1)
# ステップ分ループを回す
for loop in range(0, simulation_time):
    
    # 実座標の取得
    quadcopter0_position = sim.getObjectPosition(quadcopter_handles[0], -1)
    quadcopter1_position = sim.getObjectPosition(quadcopter_handles[1], -1)
    quadcopter2_position = sim.getObjectPosition(quadcopter_handles[2], -1)
    quadcopter3_position = sim.getObjectPosition(quadcopter_handles[3], -1)
    quadcopter4_position = sim.getObjectPosition(quadcopter_handles[4], -1)

    quadrotor.coordinate[:, loop, 0] = np.array(quadcopter0_position) * 100
    quadrotor.coordinate[:, loop, 1] = np.array(quadcopter1_position) * 100
    quadrotor.coordinate[:, loop, 2] = np.array(quadcopter2_position) * 100
    quadrotor.coordinate[:, loop, 3] = np.array(quadcopter3_position) * 100
    quadrotor.coordinate[:, loop, 4] = np.array(quadcopter4_position) * 100

    # 初期状態でのリーダとフォロワの決定
    if loop == 1:
        quadrotor.attribute_num = HelperMethod.ChangeLeader(quadrotor, loop, quadcopter_counts, goal_for_leader, change_num - 1)

    # リーダ属性のクワッドロータの機体番号を取得
    quad_leader_num = np.where(quadrotor.attribute_num == 1)[0][0]

    # 障害物センサの処理
    # {
    #    ・基本的にはリーダの障害物センサの値のみを利用する
    #    ・フォーメーションが1でリーダが障害物を検知しなければ最後尾の
    #     クワッドロータの障害物センサの値を利用する
    #    ・センサの最大値は4であり,4のデータは障害物が存在しない
    #    ・フォロワと目標点も障害物センサに反応するため除外する
    # }
    # 通常の障害物検知

    # リーダ機の障害物センサの値を取得する
    packed_data1 = sim.readCustomDataBlock(lidar_handles[quad_leader_num][0], f'scan_ranges{quad_leader_num}1')
    packed_data2 = sim.readCustomDataBlock(lidar_handles[quad_leader_num][1], f'scan_ranges{quad_leader_num}2')
    # 数値データに変換
    lidar_data1 = client.unpackFloatTable(packed_data1) if packed_data1 else []
    lidar_data2 = client.unpackFloatTable(packed_data2) if packed_data2 else []
    # リーダ機の姿勢の取得
    leader_angle = sim.getObjectOrientation(quadcopter_handles[quad_leader_num], -1)
    time.sleep(0.05)
    # 障害物があるかの判定
    is_obstacle = HelperMethod.ObstacleDetection(lidar_data1, lidar_data2, leader_angle, stepnum, quadrotor, simulation_time, quad_leader_num ,quadcopter_counts, is_obstacle, goal_for_leader[0:2, change_num])

    # チョークポイントを抜けたかの判定
    if not is_obstacle and current_formation == 1:
        # 最後尾のクワッドロータの障害物センサの値を取得
        tmp_packed_data1 = sim.readCustomDataBlock(lidar_handles[np.where(quadrotor.attribute_num == 2)[0][0]][0], f'scan_ranges{np.where(quadrotor.attribute_num == 2)[0][0]}1')
        tmp_packed_data2 = sim.readCustomDataBlock(lidar_handles[np.where(quadrotor.attribute_num == 2)[0][0]][1], f'scan_ranges{np.where(quadrotor.attribute_num == 2)[0][0]}2')
        # 数値データに変換
        behind_lidar_data1 = client.unpackFloatTable(tmp_packed_data1) if tmp_packed_data1 else []
        behind_lidar_data2 = client.unpackFloatTable(tmp_packed_data2) if tmp_packed_data2 else []
        # 最後尾のクワッドロータの姿勢を取得
        behind_angle = sim.getObjectOrientation(quadcopter_handles[np.where(quadrotor.attribute_num == 2)[0][0]], -1)
        time.sleep(0.05)
        # 障害物があるかの判定
        is_obstacle = HelperMethod.ObstacleDetection(behind_lidar_data1, behind_lidar_data2, behind_angle, stepnum, quadrotor, simulation_time, np.where(quadrotor.attribute_num == 2)[0][0] ,quadcopter_counts, is_obstacle, goal_for_leader[0:2, change_num])
    
    # 障害物の有無によりフォーメーションを指定
    if is_obstacle:
        # フォーメーションを指定(一直線の形)
        current_formation = 1
    else:
        current_formation = 2
    
    # リーダの処理
    # {   
    #    リーダ機の速さ及び単位ベクトルの格納
    #    リーダ機の次ステップにおける座標の計算を行う
    # }

    # リーダから見た目標地点のベクトル
    goal_distance = goal_for_leader[:, change_num] - quadrotor.coordinate[:, loop, quad_leader_num]
    # リーダの速度を算出,計算の仕方は単位ベクトル×スカラーの速さ
    quadrotor.speed[:, loop, quad_leader_num] = goal_distance / np.linalg.norm(goal_distance) * leader_speed
    quadrotor.speed_dir[:, loop, quad_leader_num] = quadrotor.speed[:, loop, quad_leader_num] / np.linalg.norm(quadrotor.speed[:, loop, quad_leader_num])

    # リーダの次ステップでの座標を算出
    quadrotor.coordinate[:, loop + 1, quad_leader_num] = quadrotor.coordinate[:, loop, quad_leader_num] + dt * quadrotor.speed[:, loop, quad_leader_num]

    # ゼロ除算回避で0を直接代入している
    quadrotor.relative_distance[:, loop, quad_leader_num] = 0
    quadrotor.unit_relative_distance[0:2, loop, quad_leader_num] = 0

    # フォロワの処理

    # 通常動作の処理
    if np.linalg.norm(goal_distance) > 30:
        # {
        #   フォロワから見たリーダのベクトル及び単位ベクトルを算出
        #   属性番号がリーダである機体はゼロ除算を回避するために分岐させる
        # }
        for i in range(0, quadcopter_counts - 1):
             # 属性番号がi+1の機体のインデックスを取得
            follower_idx = np.where(quadrotor.attribute_num == i + 1 + 1)[0][0]  # i+1+1: MATLABのi+1, Pythonは0始まり
            quadrotor.relative_distance[:, loop, follower_idx] =  quadrotor.coordinate[:, loop, quad_leader_num] \
                                                                    - quadrotor.coordinate[:, loop, follower_idx]
            quadrotor.unit_relative_distance[0:2, loop, follower_idx] = quadrotor.relative_distance[0:2, loop, follower_idx] \
                                                                            / np.linalg.norm(quadrotor.relative_distance[0:2, loop, follower_idx])       
        
        # {
        #   フォロワ間のベクトルを算出(接近禁止領域の規定)
        #   属性番号に紐づける
        # }
        for i in range(0, quadcopter_counts - 1):
            for j in range(i + 1, quadcopter_counts - 1):
                VBF[:, loop, j, i] = quadrotor.coordinate[:, loop, np.where(quadrotor.attribute_num == i + 1)[0][0]] \
                - quadrotor.coordinate[:, loop, np.where(quadrotor.attribute_num == j + 1)[0][0]]
                VBF_dir[0:2, loop, j, i] = VBF[0:2, loop, j, i] / np.linalg.norm(VBF[0:2, loop, j, i])

        # フォロワ番号により条件確認の範囲が異なる処理

        # {
        #   フォロワ番号でループを回す
        #   フォロワが条件を満たすかの判定はflagで行う
        # }
        for i in range(0, quadcopter_counts - 1):
            flag = True
            # フォロワ単体に関する処理は以下のforループと同じインデントで行う

            # 他のフォロワとの距離が適切かの判定
            for j in range(i + 1, quadcopter_counts - 1):
                if np.linalg.norm(VBF[0:2, loop, j, i]) >= distance_threshold:
                    flag = flag and True
                else:
                    flag = flag and False
                    break
            
            # リーダとの距離が適切かの判定
            if np.linalg.norm(quadrotor.relative_distance[0:2, loop, np.where(quadrotor.attribute_num == i + 1)[0][0]]) >= distance_threshold:
                flag = flag and True
            else:
                flag = flag and False
            
            local_axis = HelperMethod.axis_transform(quadrotor.speed_dir[:, loop, quad_leader_num])

            if flag:
                # 回転ベクトル
                R1 = HelperMethod.rot(local_axis[0,0], local_axis[1,0], local_axis[2,0], AOR[i, 1, current_formation - 1])
                R2 = HelperMethod.rot(local_axis[0,2], local_axis[1,2], local_axis[2,2], AOR[i, 0, current_formation - 1])
                v = quadrotor.speed_dir[:, loop, quad_leader_num]
                D = range_with_leader[i, current_formation - 1] * (R1 @ R2 @ v)
                # フォロワからみた目標地点へのベクトル
                lt = quadrotor.coordinate[:, loop, quad_leader_num] - quadrotor.coordinate[:, loop, np.where(quadrotor.attribute_num == i + 2)[0][0]] + D
                print(np.shape(quadrotor.coordinate[:, loop, np.where(quadrotor.attribute_num == i + 2)[0][0]]))
                print(np.shape(quadrotor.coordinate[:, loop, quad_leader_num]))
                print(np.shape(lt))
                print(np.shape(quadrotor.speed_dir[:, loop, quad_leader_num]))
                print(np.shape(quadrotor.control_entry_dir[:, loop, np.where(quadrotor.attribute_num == i + 2)[0][0]]))
                # 式(3.13)
                h = k01[i, 0] + k01[i, 1] / (1 + np.linalg.norm(lt))
                # 式(3.11)
                quadrotor.control_entry_dir[:, loop, np.where(quadrotor.attribute_num == i + 2)[0][0]] = (lt + h * quadrotor.speed_dir[:, loop, quad_leader_num]) \
                                                                                                    / np.linalg.norm((lt + h * quadrotor.speed_dir[:, loop, quad_leader_num]))
                # 式(3.14) なぜabsをarctanでとっているのかを要調査
                quadrotor.control_entry[loop, np.where(quadrotor.attribute_num == i + 1)[0][0]] = np.linalg.norm(quadrotor.speed[:, loop, quad_leader_num]) \
                                                                                            * (1 + (2/np.pi) * kps[i, 0] * np.arctan(np.abs(np.dot(lt, quadrotor.speed_dir[:, loop, quad_leader_num]) / kps[i, 1])))
                # 最終的な速度の計算
                quadrotor.speed[:, loop, np.where(quadrotor.attribute_num == i + 1)[0][0]] = quadrotor.control_entry[loop, np.where(quadrotor.attribute_num == i + 1)[0][0]] \
                                                                                        * quadrotor.control_entry_dir[:, loop, np.where(quadrotor.attribute_num == i + 1)[0][0]]
                
            # リーダとフォロワの回避関係
            idx = np.where(quadrotor.attribute_num == i + 1)[0][0]
            vec = quadrotor.relative_distance[0:2, loop, idx]
            norm = np.linalg.norm(vec)
            print(norm)
            if norm == 0:
                unit_vec = np.zeros_like(vec)
            else:
                unit_vec = vec / norm
            quadrotor.unit_relative_distance[0:2, loop, idx] = unit_vec
            # quadrotor.unit_relative_distance[0:2, loop, idx] = quadrotor.relative_distance[0:2, loop, idx] \
            #                                                     / np.linalg.norm(quadrotor.relative_distance[0:2, loop, idx])
            
            # リーダに近づきすぎた時の処理
            if np.linalg.norm(quadrotor.relative_distance[0:2, loop, np.where(quadrotor.attribute_num == i + 1)[0][0]]) < distance_threshold:
                # normの前の-1はdistanceをどちらを起点にするかによって必要か変わる
                quadrotor.speed[0:2, loop, np.where(quadrotor.attribute_num == i + 1)[0][0]] = -1 * np.linalg.norm(quadrotor.speed[0:2, loop, quad_leader_num]) \
                                                                                                * quadrotor.unit_relative_distance[0:2, loop, np.where(quadrotor.attribute_num == i + 1)[0][0]]
                quadrotor.speed[2, loop, np.where(quadrotor.attribute_num == i + 1)[0][0]] = 0
            
        # フォロワ同士の回避アルゴリズム(若い番号が回避するとする)
        # 他のフォロワとの距離が適正であるかの判別,フォロワ番号でループを回す
        for j in range(0, quadcopter_counts - 1):
            for k in range(j + 1, quadcopter_counts - 1):
                if np.linalg.norm(VBF[0:2, loop, k, j]) < distance_threshold:
                    quadrotor.speed[0:2, loop, np.where(quadrotor.attribute_num == j + 1)[0][0]] = np.linalg.norm(quadrotor.speed[0:2, loop, np.where(quadrotor.attribute_num == k + 1)[0][0]]) \
                                                                                                    * VBF_dir[0:2, loop, k, j]
                    quadrotor.speed[2, loop, np.where(quadrotor.attribute_num == j + 1)[0][0]] = 0
        
        # 各機体の次の座標の計算と各機体が一直線に並んでいる状態でリーダ時の処理(折り返しのアルゴリズム)
        # フォロワ番号でループ
        for k in range(0, quadcopter_counts - 1):
            # 全フォロワの次の座標を算出
            quadrotor.coordinate[:, loop + 1, np.where(quadrotor.attribute_num == k + 1)[0][0]] = quadrotor.coordinate[:, loop, np.where(quadrotor.attribute_num == k + 1)[0][0]] \
                                                                                                    + dt * quadrotor.speed[:, loop, np.where(quadrotor.attribute_num == k + 1)[0][0]]
        
    elif np.linalg.norm(goal_distance) <= 30:
        # 目標地点到達した時間を記録(目標地点を表示する際に使用)
        arrive_time[change_num, 0] = loop
        # 次の目標地点移る
        change_num = change_num + 1
        # フォーメーションの指定
        current_formation = 2

        # {
        #   分岐1:全目標地点に到達
        #       最初に全目標地点に到達した時の座標を保存する(停留のため)
        #   分岐2:全目標地点に到達していない
        #       属性を再定義し,1ステップ停留させる
        #       次の目標地点に向かう
        # }
        if change_num >= goal_for_leader + 1:
            # 停留座標の取得
            if not is_completed:
                for i in range(0, quadcopter_counts):
                    completed_coordinate[:, i] = quadrotor.coordinate[:, loop, i]
            
            is_completed = True
            change_num = change_num - 1

        else:
            quadrotor.attribute_num[:] = HelperMethod.ChangeLeader(quadrotor, loop, quadcopter_counts, goal_for_leader, change_num)
            for i in range(0, quadcopter_counts):
                completed_coordinate[:, i] = quadrotor.coordinate[:, loop, i]
        
        # 次のステップでの座標
        for i in range(0, quadcopter_counts):
            quadrotor.coordinate[:, loop + 1, i] = completed_coordinate[:, i]
        
    # 例外が発生した場合には全クワッドロータの動きを止める
    else:
        # 最初に例外になった時の座標を保存する(停留のため)
        if not is_exception_raised:
            for k in range(0, quadcopter_counts):
                exception_coordinate[:, k] = quadrotor.coordinate[:, k]
            is_exception_raised = True
        
        # 全クワッドローダーを停留させる
        for k in range(0, quadcopter_counts):
            quadrotor.speed[:, loop:simulation_time + 1, k] = np.zeros((3, simulation_time - loop + 1))
            quadrotor.coordinate[:, loop + 1, k] = exception_coordinate[:, k]
    
    # CoppeliaSimへの反映
    for i in range(0, quadcopter_counts):
        print(quadrotor.coordinate[:, loop + 1, i])
        sim.setObjectPosition(quadcopter_indicator_handles[i], (quadrotor.coordinate[:, loop + 1, i] / 100).tolist(), -1)
    sim.setObjectPosition(cylinder_handle, (goal_for_leader[:, change_num - 1] / 100).tolist(), -1)
    sim.step()
    time.sleep(0.05)



# except Exception as e:
#     print("error", e)
#     print(e.args)
#     print(e.add_note)

sim.stopSimulation()
