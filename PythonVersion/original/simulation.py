# simulation.py

import time
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from typing import List
import numpy as np
# swarm_components.pyからQuadcopterクラスをインポート
from swarm_components import Quadcopter 

class SimulatorInterface:
    """
    CoppeliaSimとの通信を管理し、シミュレータへの依存をこのクラスに集約する。
    """
    def __init__(self, host='127.0.0.1', port=23000):
        self.client = RemoteAPIClient(host, port)
        self.sim = self.client.getObject('sim')
        self.quad_handles = []
        self.target_handles = []
        self.goal_cylinder_handle = -1

    def connect(self):
        """シミュレータへの接続を試みる"""
        print("Connecting to CoppeliaSim...")
        # 必要に応じてポート番号をシーンに合わせて調整してください
        # self.client.setStepping(True) # 必要に応じて同期モードを有効にする
        print("Connected.")

    def start_simulation(self):
        """シミュレーションを開始する"""
        self.sim.stopSimulation()
        time.sleep(1) # 確実に停止するのを待つ
        self.sim.startSimulation()
        print("Simulation started.")

    def stop_simulation(self):
        """シミュレーションを停止する"""
        self.sim.stopSimulation()
        print("Simulation stopped.")
        
    def step_simulation(self):
        """シミュレーションを1ステップ進める（同期モード用）"""
        self.client.step()

    def setup_handles(self, quad_names: List[str], goal_cylinder_name: str):
        """シミュレーション内のオブジェクトハンドルを取得・保持する"""
        print("Setting up object handles...")
        self.quad_handles = [self.sim.getObject(f'/{name}') for name in quad_names]
        # targetはドローンの位置を示すインジケータ
        self.target_handles = [self.sim.getObject(f'/target{name.count()}') for name in quad_names]
        self.goal_cylinder_handle = self.sim.getObject(f'/{goal_cylinder_name}')
        print(f"Handles loaded for {len(self.quad_handles)} quadcopters.")

    def get_all_quad_states(self) -> List[dict]:
        """全ドローンの現在の状態（位置、速度）を取得する"""
        states = []
        for handle in self.quad_handles:
            pos = self.sim.getObjectPosition(handle, -1)
            vel, _ = self.sim.getObjectVelocity(handle)
            states.append({'position': np.array(pos), 'velocity': np.array(vel)})
        return states

    def set_all_quad_positions(self, quads: List[Quadcopter]):
        """全ドローンの次の目標位置をインジケータに設定する"""
        for i, quad in enumerate(quads):
            # 単位を[cm]から[m]に変換して設定
            sim_pos = (quad.position / 100.0).tolist()
            self.sim.setObjectPosition(self.target_handles[i], sim_pos, -1)
            
    def set_goal_position(self, goal_pos: np.ndarray):
        """目標点のインジケータ位置を設定する"""
        sim_pos = (goal_pos / 100.0).tolist()
        self.sim.setObjectPosition(self.goal_cylinder_handle, sim_pos, -1)
