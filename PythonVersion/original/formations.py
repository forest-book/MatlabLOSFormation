import numpy as np

class Formation:
    """
    フォーメーションパターンを定義するデータクラス。
    フォロワーごとの目標相対距離と角度を保持する。
    """
    def __init__(self, name: str, distances: np.ndarray, angles_rad: np.ndarray):
        """
        Args:
            name (str): フォーメーション名
            distances (np.ndarray): 各フォロワーのリーダーからの目標距離 [cm]
            angles_rad (np.ndarray): 各フォロワーの目標角度 [Ψ(z-rot), Φ(x-rot)] (ラジアン)
        """
        self.name = name
        self.distances = distances
        self.angles = angles_rad

# --- 事前定義されたフォーメーション ---

V_SHAPE_FORMATION = Formation(
    name="V-Shape",
    distances=np.array([100, 100, 200, 200]),
    angles_rad=np.array([
        [np.deg2rad(-140), np.deg2rad(0)],
        [np.deg2rad(140),  np.deg2rad(0)],
        [np.deg2rad(140),  np.deg2rad(0)],
        [np.deg2rad(-140), np.deg2rad(0)]
    ])
)

LINE_FORMATION = Formation(
    name="Line",
    distances=np.array([80, 160, 240, 320]),
    angles_rad=np.array([
        [np.deg2rad(180), np.deg2rad(0)],
        [np.deg2rad(180), np.deg2rad(0)],
        [np.deg2rad(180), np.deg2rad(0)],
        [np.deg2rad(180), np.deg2rad(0)]
    ])
)
