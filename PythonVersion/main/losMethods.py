import numpy as np
from quadrotor import Quadrotor

class HelperMethod:

    @staticmethod
    # Python版 ChangeLeader関数
    def ChangeLeader(Q :Quadrotor, loop: int, Quad_num: int, lg: np.ndarray, Change_num: int) -> np.ndarray:
        """
        Q: 機体情報を格納したdictまたはクラス（Q['Coord'], Q['Att']などでアクセス）
        loop: 現在のステップ
        Quad_num: クワッドローターの数
        lg: 目標点座標 (3, LG_num) 配列
        Change_num: 現在の目標点インデックス（0始まりに注意）
        """
        gd = np.zeros((Quad_num, 2))

        # 各機体と目標点の距離と属性番号を格納
        for t in range(Quad_num):
            gd[t, 0] = np.linalg.norm(lg[:, Change_num] - Q.coordinate[:, loop, t])
            gd[t, 1] = Q.attribute_num[t]

        # 距離が大きい順にソート
        # argsortは小さい順なので[::-1]で逆順
        sort_idx = np.argsort(gd[:, 0])[::-1]
        gd_sorted = gd[sort_idx]

        new_attribute = np.zeros(Quad_num, dtype=int)
        # 属性番号を再決定
        for s in range(Quad_num - 1):
            # Q['Att'] == gd_sorted[s, 1] のインデックスに s+2 を割り当て
            idx = np.where(Q.attribute_num == gd_sorted[s, 1])[0][0]
            new_attribute[idx] = s + 2
        # 一番遠い機体をリーダー（属性番号1）に
        idx = np.where(Q.attribute_num == gd_sorted[Quad_num - 1, 1])[0][0]
        new_attribute[idx] = 1

        return new_attribute
    
    @staticmethod
    def ObstacleDetection(ranges1, ranges2, QuadAng, stepnum, Q:Quadrotor, loop, Center_num, Quad_num, obstacle_flag, lg) -> bool:
        # ranges1, ranges2: 1D np.array
        # QuadAng: [roll, pitch, yaw]
        # Q: dict with 'Coord' key, shape (3, loop_num+1, Quad_num)
        # lg: 1D array, shape (2,) or (2,)

        if ranges1 is None or len(ranges1) == 0:
            ranges1 = np.zeros(stepnum)
        else:
            ranges1 = np.array(ranges1)  # 追加: list→np.ndarray変換

        if ranges2 is None or len(ranges2) == 0:
            ranges2 = np.zeros(stepnum)
        else:
            ranges2 = np.array(ranges2)  # 追加: list→np.ndarray変換

        # 障害物が検知されたインデックス
        index1 = np.where(ranges1 < 4)[0]
        index2 = np.where(ranges2 < 4)[0]
        # 検知値のみ抽出
        valid_ranges1 = ranges1[ranges1 < 4]
        valid_ranges2 = ranges2[ranges2 < 4]

        # 判定対象の他ロボット番号
        Quadindex = [i for i in range(Quad_num) if i != Center_num]

        # センサ1
        if valid_ranges1.size > 0:
            obs_flags = []
            for j, r in enumerate(valid_ranges1):
                theta1 = 4/3 * np.pi / stepnum * (index1[j]) - np.pi/6 - np.pi/2 + QuadAng[2]
                x = r * np.cos(theta1) + Q.coordinate[0, loop, Center_num] / 100 + 0.1
                y = r * np.sin(theta1) + Q.coordinate[1, loop, Center_num] / 100
                obs_Coord = np.array([x*100, y*100])
                obsflag = 1
                for k in Quadindex:
                    if np.linalg.norm(obs_Coord - Q.coordinate[0:2, loop, k]) > 100:
                        obsflag = obsflag & 1
                    else:
                        obsflag = obsflag & 0
                if np.linalg.norm(obs_Coord - lg[0:2]) > 100:
                    obsflag = obsflag & 1
                else:
                    obsflag = obsflag & 0
                obs_flags.append(obsflag)
            if np.sum(obs_flags) > 0:
                obstacle_flag = True
            else:
                obstacle_flag = False
        else:
            obstacle_flag = 0

        # センサ2
        if valid_ranges2.size > 0:
            obs_flags = []
            for j, r in enumerate(valid_ranges2):
                theta2 = 4/3 * np.pi / stepnum * (index2[j]) - np.pi/6 + np.pi/2 + QuadAng[2]
                x = r * np.cos(theta2) + Q.coordinate[0, loop, Center_num] / 100 - 0.1
                y = r * np.sin(theta2) + Q.coordinate[1, loop, Center_num] / 100
                obs_Coord = np.array([x*100, y*100])
                obsflag = 1
                for k in Quadindex:
                    if np.linalg.norm(obs_Coord - Q.coordinate[0:2, loop, k]) > 100:
                        obsflag = obsflag & 1
                    else:
                        obsflag = obsflag & 0
                if np.linalg.norm(obs_Coord - lg[0:2]) > 100:
                    obsflag = obsflag & 1
                else:
                    obsflag = obsflag & 0
                obs_flags.append(obsflag)
            if np.sum(obs_flags) > 0:
                obstacle_flag = obstacle_flag or True
            else:
                obstacle_flag = obstacle_flag or False
        else:
            obstacle_flag = False

        return obstacle_flag
    
    @staticmethod 
    def axis_transform(speed_dir):
        """
        ワールド座標系でみたローカル座標軸を取得する関数
        引数: 正規化されたリーダの速度 (np.array, shape=(3,))
        戻り値: ワールド座標系でみたローカル座標軸 (np.array, shape=(3,3))
        """
        v = np.asarray(speed_dir).reshape(3)  # 速度ベクトル
        x_axis = np.array([1, 0, 0])
        y_axis = np.array([0, 1, 0])
        z_axis = np.array([0, 0, 1])

        # 回転軸（元のy軸と新しいy'軸の外積）
        rotation_axis = np.cross(y_axis, v)
        sin_theta = np.linalg.norm(rotation_axis)
        cos_theta = np.dot(y_axis, v)
        theta = np.arctan2(sin_theta, cos_theta)

        # ゼロ除算回避
        if sin_theta < 1e-8:
            rotation_axis = np.array([0, 0, 1])  # 任意の軸
        else:
            rotation_axis = rotation_axis / sin_theta

        # Rodriguesの回転公式
        K = np.array([
            [0, -rotation_axis[2], rotation_axis[1]],
            [rotation_axis[2], 0, -rotation_axis[0]],
            [-rotation_axis[1], rotation_axis[0], 0]
        ])
        R = np.eye(3) + np.sin(theta) * K + (1 - np.cos(theta)) * (K @ K)

        new_axis = np.zeros((3, 3))
        new_axis[:, 0] = R @ x_axis
        new_axis[:, 1] = R @ y_axis
        new_axis[:, 2] = R @ z_axis
        return new_axis
    
    @staticmethod
    def rot(x, y, z, th):
        """
        ロドリゲスの回転公式による回転行列生成
        x, y, z: 回転軸ベクトルの成分
        th: 回転角（ラジアン）
        """
        # 軸ベクトルを正規化
        axis = np.array([x, y, z], dtype=float)
        axis = axis / np.linalg.norm(axis)
        x, y, z = axis

        I = np.eye(3)
        # 直積行列
        kron = np.array([
            [x*x, x*y, x*z],
            [x*y, y*y, y*z],
            [x*z, y*z, z*z]
        ])
        # クロス積行列
        Crossprd = np.array([
            [0, -z, y],
            [z, 0, -x],
            [-y, x, 0]
        ])
        r = np.cos(th) * I + np.sin(th) * Crossprd + (1 - np.cos(th)) * kron
        return r

