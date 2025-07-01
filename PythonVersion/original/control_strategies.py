import numpy as np
from abc import ABC, abstractmethod
from typing import List, Tuple
from swarm_components import Quadcopter, Role
from formations import Formation

# --- ユーティリティ関数 ---
# 状態を持たない計算は、クラス外の関数または静的メソッドとして定義
def axis_transform(speed_dir: np.ndarray) -> np.ndarray:
    """安定版: 進行方向をローカルx軸とし、安定した座標系を構築する"""
    x_new = np.asarray(speed_dir).flatten()
    if np.linalg.norm(x_new) < 1e-8: return np.eye(3)
    x_new /= np.linalg.norm(x_new)

    world_z = np.array([0.0, 0.0, 1.0])
    y_new = np.cross(world_z, x_new)

    if np.linalg.norm(y_new) < 1e-8:
        world_y = np.array([0.0, 1.0, 0.0])
        z_new = np.cross(x_new, world_y)
        z_new /= np.linalg.norm(z_new)
        y_new = np.cross(z_new, x_new)
    else:
        y_new /= np.linalg.norm(y_new)
        z_new = np.cross(x_new, y_new)

    return np.column_stack([x_new, y_new, z_new])

def rot(axis: np.ndarray, th: float) -> np.ndarray:
    """ロドリゲスの回転公式（正規化あり）"""
    norm_axis = np.linalg.norm(axis)
    if norm_axis < 1e-8: return np.eye(3)
    axis = axis / norm_axis
    
    x, y, z = axis
    I = np.eye(3)
    K = np.array([[0,-z,y], [z,0,-x], [-y,x,0]])
    
    return I * np.cos(th) + K * np.sin(th) + (1 - np.cos(th)) * np.outer(axis, axis)


# --- ストラテジーの定義 ---
class Strategy(ABC):
    """制御戦略のインターフェース"""
    @abstractmethod
    def calculate_velocity(self, self_quad: Quadcopter, all_quads: List[Quadcopter], **kwargs) -> np.ndarray:
        pass

class LeaderStrategy(Strategy):
    """リーダーの制御戦略"""
    def calculate_velocity(self, self_quad: Quadcopter, all_quads: List[Quadcopter], **kwargs) -> np.ndarray:
        goal_pos = kwargs['goal']
        max_speed = kwargs['max_speed']
        
        goal_vector = goal_pos - self_quad.position
        dist_to_goal = np.linalg.norm(goal_vector)
        
        if dist_to_goal < 30: # 到達判定
            return np.zeros(3)
        
        return (goal_vector / dist_to_goal) * max_speed

class FollowerStrategy(Strategy):
    """フォロワーの制御戦略（LOS追従 + 衝突回避）"""
    def __init__(self, k0l: np.ndarray, kps: np.ndarray, threshold: float):
        self.k0l = k0l
        self.kps = kps
        self.threshold = threshold

    def calculate_velocity(self, self_quad: Quadcopter, all_quads: List[Quadcopter], **kwargs) -> np.ndarray:
        leader = kwargs['leader']
        formation = kwargs['formation']
        follower_list_idx = kwargs['follower_idx'] # このフォロワーがリストの何番目か

        # リーダーの速度がほぼゼロ（停止している）場合、フォロワーも停止する
        # 物理シミュレーションでは完全に0にならない場合を考慮し、微小な閾値を設ける
        if np.linalg.norm(leader.velocity) < 0.1:
            return np.zeros(3)

        # 1. LOS追従速度の計算
        los_velocity = self._calculate_los_velocity(self_quad, leader, formation, follower_list_idx)
        
        # 2. 衝突回避の計算
        avoidance_velocity, is_avoiding = self._calculate_avoidance_velocity(self_quad, leader, all_quads)

        # 3. 回避行動中であれば回避速度を優先、そうでなければLOS追従速度を使用
        return avoidance_velocity if is_avoiding else los_velocity

    def _calculate_los_velocity(self, self_quad: Quadcopter, leader: Quadcopter, formation: Formation, idx: int) -> np.ndarray:
        """論文3.3節のロジック"""
        local_axis = axis_transform(leader.speed_dir)
        
        # 式(3.4) 目標相対位置Dの計算 (x,y,z軸を列ベクトルとして取得)
        # 進行方向=x軸, y軸=左, z軸=上 となるよう再定義
        ax, ay, az = local_axis[:, 0], local_axis[:, 1], local_axis[:, 2]
        # 論文のΦ(x-rot), Ψ(z-rot)の定義を尊重する
        psi, phi = formation.angles[idx, 0], formation.angles[idx, 1]
        R_x = rot(ax, phi)
        R_z = rot(az, psi)
        D = formation.distances[idx] * (R_x @ R_z @ leader.speed_dir)
        
        # 式(3.9) フォロワから目標点へのベクトルl
        lt = (leader.position - self_quad.position) + D
        
        # 式(3.13) hの計算
        h = self.k0l[idx, 0] + self.k0l[idx, 1] / (1 + np.linalg.norm(lt))
        
        # 式(3.11) 制御入力の方向 Cin_dir の計算
        cin_dir_vec = lt + h * leader.speed_dir
        cin_dir = cin_dir_vec / np.linalg.norm(cin_dir_vec)
        
        # 式(3.14) 制御入力の大きさ Cin の計算
        dot_val = np.dot(lt, leader.speed_dir)
        cin_mag = np.linalg.norm(leader.velocity) * \
                  (1 + (2/np.pi) * self.kps[idx, 0] * np.arctan(dot_val / self.kps[idx, 1]))
        
        return cin_mag * cin_dir

    def _calculate_avoidance_velocity(self, self_quad: Quadcopter, leader: Quadcopter, all_quads: List[Quadcopter]) -> Tuple[np.ndarray, bool]:
        """論文3.4節のロジック"""
        # リーダーとの回避
        l_dist_vec = leader.position - self_quad.position
        if np.linalg.norm(l_dist_vec[:2]) < self.threshold:
            unit_l_dist = l_dist_vec / np.linalg.norm(l_dist_vec)
            avoid_vel = np.zeros(3)
            avoid_vel[:2] = -1.0 * np.linalg.norm(leader.velocity[:2]) * unit_l_dist[:2]
            return avoid_vel, True

        # 他のフォロワーとの回避
        my_id = self_quad.id
        for other_quad in all_quads:
            if other_quad.id == my_id or other_quad.role == Role.LEADER:
                continue
            
            # 自分よりIDの小さい（優先度の高い）フォロワーからのみ回避する
            if other_quad.id < my_id:
                between_vec = self_quad.position - other_quad.position
                if np.linalg.norm(between_vec[:2]) < self.threshold:
                    vbf_dir_xy = between_vec[:2] / np.linalg.norm(between_vec[:2])
                    avoid_vel = np.zeros(3)
                    avoid_vel[:2] = np.linalg.norm(other_quad.velocity[:2]) * vbf_dir_xy
                    return avoid_vel, True

        return np.zeros(3), False
    