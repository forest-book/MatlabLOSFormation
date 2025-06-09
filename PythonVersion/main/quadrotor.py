import numpy as np

class Quadrotor:
    """
    クワッドローター群の情報を保持するクラス。
    MATLABの構造体Qに相当します。

    このクラスでは、シミュレーション対象となる全クワッドローターの
    時系列データをまとめて管理します。
    """
    def __init__(self, quad_num: int, loop_num: int):
        """
        クラスのインスタンスを初期化し、データを格納するNumpy配列を準備します。

        Args:
            quad_num (int): クワッドローターの総数。
            loop_num (int): シミュレーションのループ（ステップ）数。
        """
        
        # 属性番号 (リーダー:1, フォロワー:2以降)
        # MATLAB: Q.Att = zeros(Quad_num,1);
        self.attribute_num = np.zeros(quad_num)

        # 座標 (3, ステップ数+1, 機体数)
        # MATLAB: Q.Coord = zeros(3,loop_num+1,Quad_num);
        self.coordinate = np.zeros((3, loop_num + 1, quad_num))

        # 自身から見たリーダーへの相対位置ベクトル
        # MATLAB: Q.l_distance = zeros(3,loop_num+1,Quad_num);
        self.relative_distance = np.zeros((3, loop_num + 1, quad_num))

        # リーダーへの単位方向ベクトル
        # MATLAB: Q.unit_l_distance = zeros(3,loop_num+1,Quad_num);
        self.unit_relative_distance = np.zeros((3, loop_num + 1, quad_num))

        # 速度ベクトル (原文コメントの「速さ」は次元数からベクトルと解釈)
        # MATLAB: Q.speed = zeros(3,loop_num+1,Quad_num);
        self.speed = np.zeros((3, loop_num + 1, quad_num))
        
        # 速度の単位方向ベクトル
        # MATLAB: Q.speed_dir = zeros(3,loop_num+1,Quad_num);
        self.speed_dir = np.zeros((3, loop_num + 1, quad_num))

        # 制御入力の大きさ (スカラー)
        # MATLAB: Q.Cin = zeros(loop_num+1,Quad_num);
        self.control_entry = np.zeros((loop_num + 1, quad_num))

        # 制御入力の単位方向ベクトル
        # MATLAB: Q.Cin_dir = zeros(3,loop_num+1,Quad_num);
        self.control_entry_dir = np.zeros((3, loop_num + 1, quad_num))
        