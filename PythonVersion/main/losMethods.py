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

