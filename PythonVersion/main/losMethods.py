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

        # 距離が小さい順にソート
        sort_idx = np.argsort(gd[:, 0])
        gd_sorted = gd[sort_idx]

        new_attribute = np.zeros(Quad_num, dtype=int)

        # 一番近い機体をリーダー（属性番号1）に
        leader_original_attr = gd_sorted[0, 1]
        leader_idx = np.where(Q.attribute_num == leader_original_attr)[0][0]
        new_attribute[leader_idx] = 1

        # 残りの機体を距離が遠い順にフォロワーとして割り当て (属性2, 3, ...)
        # gd_sortedは昇順(近い順)なので、後ろからループする
        for s in range(Quad_num - 1):
            # s=0: 最も遠い機体 (gd_sortedの末尾)
            # s=1: 2番目に遠い機体
            follower_original_attr = gd_sorted[Quad_num - 1 - s, 1]
            follower_idx = np.where(Q.attribute_num == follower_original_attr)[0][0]
            new_attribute[follower_idx] = s + 2 # 属性2, 3, 4, ...

        return new_attribute
    
    @staticmethod
    def ObstacleDetection(ranges1, ranges2, QuadAng, stepnum, Q:Quadrotor, loop, Center_num, Quad_num, lg) -> bool:
        """
        LiDARセンサー情報に基づき、障害物を検知する。
        他のドローンや目標点は障害物から除外する。

        Args:
            ranges1 (list or np.array): センサー1の距離データ.
            ranges2 (list or np.array): センサー2の距離データ.
            QuadAng (list or np.array): 現在注目している機体の姿勢 [roll, pitch, yaw].
            stepnum (int): センサーデータのステップ数.
            Q (Quadrotor): 全機体の情報を持つクラスインスタンス.
            loop (int): 現在のシミュレーションステップ.
            Center_num (int): 現在注目している機体の機体番号 (0-indexed).
            Quad_num (int): 全機体の数.
            obstacle_flag (bool): (この関数内では未使用だが、呼び出し元の互換性のために残している).
            lg (np.array): 現在の目標点の座標 (x, y).

        Returns:
            bool: 障害物が検知された場合はTrue、そうでなければFalse.
        """
        if ranges1 is None or len(ranges1) == 0:
            ranges1 = np.array([])
        else:
            ranges1 = np.array(ranges1)

        if ranges2 is None or len(ranges2) == 0:
            ranges2 = np.array([])
        else:
            ranges2 = np.array(ranges2)

        # 障害物が検知されたインデックス (距離4m未満)
        index1 = np.where(ranges1 < 4)[0]
        index2 = np.where(ranges2 < 4)[0]
        # 検知値のみ抽出
        valid_ranges1 = ranges1[index1]
        valid_ranges2 = ranges2[index2]

        # 判定対象から自分自身を除いた他ロボットの番号
        Quadindex = [i for i in range(Quad_num) if i != Center_num]

        # --- センサー1の障害物判定 ---
        obstacle_flag1 = False
        if valid_ranges1.size > 0:
            obs_flags = []
            for j, r in enumerate(valid_ranges1):
                # 検知点のワールド座標を計算
                theta1 = (4/3 * np.pi / stepnum) * index1[j] - (np.pi / 6) - (np.pi / 2) + QuadAng[2]
                x = r * np.cos(theta1) + Q.coordinate[0, loop, Center_num] / 100 + 0.1
                y = r * np.sin(theta1) + Q.coordinate[1, loop, Center_num] / 100
                obs_Coord = np.array([x * 100, y * 100])
                
                is_real_obstacle = True
                # 他のドローンでないか確認
                for k in Quadindex:
                    if np.linalg.norm(obs_Coord - Q.coordinate[0:2, loop, k]) <= 100:
                        is_real_obstacle = False
                        break
                if not is_real_obstacle:
                    obs_flags.append(0)
                    continue

                # 目標点でないか確認
                if np.linalg.norm(obs_Coord - lg[0:2]) <= 100:
                    is_real_obstacle = False
                
                obs_flags.append(1 if is_real_obstacle else 0)

            # １つでも真の障害物があればフラグを立てる
            if np.sum(obs_flags) > 0:
                obstacle_flag1 = True

        # --- センサー2の障害物判定 ---
        obstacle_flag2 = False
        if valid_ranges2.size > 0:
            obs_flags = []
            for j, r in enumerate(valid_ranges2):
                # 検知点のワールド座標を計算
                theta2 = (4/3 * np.pi / stepnum) * index2[j] - (np.pi / 6) + (np.pi / 2) + QuadAng[2]
                x = r * np.cos(theta2) + Q.coordinate[0, loop, Center_num] / 100 - 0.1
                y = r * np.sin(theta2) + Q.coordinate[1, loop, Center_num] / 100
                obs_Coord = np.array([x * 100, y * 100])

                is_real_obstacle = True
                # 他のドローンでないか確認
                for k in Quadindex:
                    if np.linalg.norm(obs_Coord - Q.coordinate[0:2, loop, k]) <= 100:
                        is_real_obstacle = False
                        break
                if not is_real_obstacle:
                    obs_flags.append(0)
                    continue

                # 目標点でないか確認
                if np.linalg.norm(obs_Coord - lg[0:2]) <= 100:
                    is_real_obstacle = False

                obs_flags.append(1 if is_real_obstacle else 0)

            # １つでも真の障害物があればフラグを立てる
            if np.sum(obs_flags) > 0:
                obstacle_flag2 = True

        return obstacle_flag1 or obstacle_flag2
    
    # @staticmethod 
    # def axis_transform(speed_dir):
    #     """
    #     ワールド座標系でみたローカル座標軸を取得する関数
    #     引数: 正規化されたリーダの速度 (np.array, shape=(3,))
    #     戻り値: ワールド座標系でみたローカル座標軸 (np.array, shape=(3,3))
    #     """
    #     v = np.asarray(speed_dir).reshape(3)  # 速度ベクトル
    #     x_axis = np.array([1, 0, 0])
    #     y_axis = np.array([0, 1, 0])
    #     z_axis = np.array([0, 0, 1])

    #     # 回転軸（元のy軸と新しいy'軸の外積）
    #     rotation_axis = np.cross(y_axis, v)
    #     sin_theta = np.linalg.norm(rotation_axis)
    #     cos_theta = np.dot(y_axis, v)
    #     theta = np.arctan2(sin_theta, cos_theta)

    #     # ゼロ除算回避
    #     if sin_theta < 1e-8:
    #         rotation_axis = np.array([0, 0, 1])  # 任意の軸
    #     else:
    #         rotation_axis = rotation_axis / sin_theta

    #     # Rodriguesの回転公式
    #     K = np.array([
    #         [0, -rotation_axis[2], rotation_axis[1]],
    #         [rotation_axis[2], 0, -rotation_axis[0]],
    #         [-rotation_axis[1], rotation_axis[0], 0]
    #     ])
    #     R = np.eye(3) + np.sin(theta) * K + (1 - np.cos(theta)) * (K @ K)

    #     new_axis = np.zeros((3, 3))
    #     new_axis[:, 0] = R @ x_axis
    #     new_axis[:, 1] = R @ y_axis
    #     new_axis[:, 2] = R @ z_axis
    #     return new_axis

    # @staticmethod
    # def axis_transform(speed_dir):
    #     """
    #     ワールド座標系でみたローカル座標軸を取得する関数（新実装）。
    #     リーダーの進行方向をローカルy軸とし、ワールドのZ軸を基準に安定した右手座標系を構築する。
    #     引数: 正規化されたリーダの速度 (np.array, shape=(3,))
    #     戻り値: ワールド座標系でみたローカル座標軸 (np.array, shape=(3,3))
    #     """
    #     # 1. ローカルのy軸をリーダーの進行方向ベクトルとする
    #     y_new = np.asarray(speed_dir).reshape(3)
        
    #     # 進行方向ベクトルが極端に小さい場合は、デフォルトの座標系を返す
    #     if np.linalg.norm(y_new) < 1e-8:
    #         return np.eye(3)

    #     # 2. ワールド座標系のZ軸（天頂方向）を基準に利用する
    #     world_z = np.array([0, 0, 1])

    #     # 3. ローカルx軸を計算 (y_new と world_z の外積)
    #     # これにより、x_newは常に水平面（ワールドxy平面）に平行、またはそれに近いベクトルになる
    #     x_new = np.cross(y_new, world_z)

    #     # もし進行方向がワールドZ軸と平行（真上/真下）な場合、外積がゼロベクトルになるため、
    #     # ローカルx軸をワールドx軸とするなど、代替ルールを適用する
    #     if np.linalg.norm(x_new) < 1e-8:
    #         x_new = np.array([1, 0, 0])
    #     else:
    #         # 正規化して単位ベクトルにする
    #         x_new = x_new / np.linalg.norm(x_new)

    #     # 4. ローカルz軸を計算 (x_new と y_new の外積)
    #     # これで x_new, y_new, z_new が互いに直交する右手系を形成する
    #     z_new = np.cross(x_new, y_new)
    #     # y_newとx_newは単位ベクトルかつ直交なので、z_newも自動的に単位ベクトルになる

    #     # 5. 計算したローカル軸を列ベクトルとして格納して返す
    #     new_axis = np.zeros((3, 3))
    #     new_axis[:, 0] = x_new
    #     new_axis[:, 1] = y_new
    #     new_axis[:, 2] = z_new
        
    #     return new_axis

    @staticmethod
    def axis_transform(speed_dir: np.ndarray) -> np.ndarray:
        """
        ワールド座標系でみたローカル座標軸を取得する関数（新・安定版）。
        リーダーの進行方向をローカルx軸とし、ワールドのZ軸（天頂）を基準に
        安定した右手座標系を構築する。

        Args:
            speed_dir: 正規化されたリーダーの速度ベクトル (shape=(3,))

        Returns:
            ワールド座標系で表現されたローカル座標軸の3x3行列
            (列ベクトルがそれぞれローカルのx, y, z軸)
        """
        # --- 1. ローカルx軸を、正規化された進行方向ベクトルとする ---
        x_new = np.asarray(speed_dir).flatten()
        if np.linalg.norm(x_new) < 1e-8:
            # 速度がゼロの場合、デフォルトのワールド座標系を返す
            return np.eye(3)
        x_new /= np.linalg.norm(x_new)

        # --- 2. ワールドZ軸を基準に、ローカルy軸を計算する ---
        world_z = np.array([0.0, 0.0, 1.0])
        
        # x_new と world_z の外積からローカルy軸を求める
        y_new = np.cross(world_z, x_new)

        # もし進行方向がワールドZ軸とほぼ平行（真上/真下を向いている）場合、
        # 外積がゼロベクトルに近くなり不安定になる。
        if np.linalg.norm(y_new) < 1e-8:
            # この例外的なケースでは、ワールドy軸をローカルy軸の基準とする
            world_y = np.array([0.0, 1.0, 0.0])
            z_new = np.cross(x_new, world_y)
            z_new /= np.linalg.norm(z_new)
            # 再度、外積から厳密なローカルy軸を計算する
            y_new = np.cross(z_new, x_new)
        else:
            y_new /= np.linalg.norm(y_new)
            # --- 3. ローカルz軸を計算し、厳密な右手座標系を完成させる ---
            z_new = np.cross(x_new, y_new)

        # --- 4. 計算したローカル軸を列ベクトルとして格納して返す ---
        new_axis = np.column_stack([x_new, y_new, z_new])
        
        return new_axis
    
    # @staticmethod
    # def rot(x, y, z, th):
    #     """
    #     ロドリゲスの回転公式による回転行列生成
    #     x, y, z: 回転軸ベクトルの成分
    #     th: 回転角（ラジアン）
    #     """
    #     # 軸ベクトルを正規化
    #     axis = np.array([x, y, z], dtype=float)
    #     axis = axis / np.linalg.norm(axis)
    #     x, y, z = axis

    #     I = np.eye(3)
    #     # 直積行列
    #     kron = np.array([
    #         [x*x, x*y, x*z],
    #         [x*y, y*y, y*z],
    #         [x*z, y*z, z*z]
    #     ])
    #     # クロス積行列
    #     Crossprd = np.array([
    #         [0, -z, y],
    #         [z, 0, -x],
    #         [-y, x, 0]
    #     ])
    #     r = np.cos(th) * I + np.sin(th) * Crossprd + (1 - np.cos(th)) * kron
    #     return r
    
    @staticmethod
    def rot(x, y, z, th):
        """
        ロドリゲスの回転公式による回転行列生成（MATLAB版の挙動を再現）
        x, y, z: 回転軸ベクトルの成分（正規化されていることを期待）
        th: 回転角（ラジアン）
        """
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

