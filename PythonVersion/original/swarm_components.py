# swarm_components.py

import numpy as np
from enum import Enum, auto

class Role(Enum):
    """ドローンの役割を定義する列挙型"""
    LEADER = auto()
    FOLLOWER = auto()

class Quadcopter:
    """個々のドローンの状態を保持するクラス"""
    def __init__(self, q_id: int, handle: int, initial_pos: np.ndarray, role: Role):
        self.id: int = q_id
        self.handle: int = handle
        self.role: Role = role
        
        # 状態変数（現在のステップ）
        self.position: np.ndarray = initial_pos # 単位: cm
        self.velocity: np.ndarray = np.zeros(3) # 単位: cm/step
        self.speed_dir: np.ndarray = np.zeros(3) # 速度の単位ベクトル
        
        # 次のステップの計算結果を保持する変数
        self.next_velocity: np.ndarray = np.zeros(3)

    def update_state(self, new_pos: np.ndarray, new_vel: np.ndarray):
        """シミュレータから取得した値で状態を更新する"""
        self.position = new_pos * 100.0 # m -> cm
        # シミュレータの速度は m/s だが、制御周期dt=1と仮定し、cm/stepとして扱う
        self.velocity = new_vel * 100.0 
        
        norm_vel = np.linalg.norm(self.velocity)
        if norm_vel > 1e-8:
            self.speed_dir = self.velocity / norm_vel
        else:
            self.speed_dir = np.zeros(3)
            
    def update_position(self, dt: float):
        """計算された次の速度に基づき、位置を更新する"""
        self.position += self.next_velocity * dt
        