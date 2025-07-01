import numpy as np
import time
from typing import List

from simulation import SimulatorInterface
from swarm_components import Quadcopter, Role
from formations import V_SHAPE_FORMATION, LINE_FORMATION
from control_strategies import Strategy, LeaderStrategy, FollowerStrategy

class MainController:
    """アプリケーション全体を管理し、メインループを実行する"""
    def __init__(self, params: dict):
        self.params = params
        self.sim = SimulatorInterface()
        self.quads: List[Quadcopter] = []
        self.leader: Quadcopter = None
        self.followers: List[Quadcopter] = []
        self.goal_reached = False
        
        # ストラテジーのインスタンス化
        self.leader_strategy = LeaderStrategy()
        self.follower_strategy = FollowerStrategy(
            params['k0l'], params['kps'], params['distance_threshold']
        )
        self.strategies = {
            Role.LEADER: self.leader_strategy,
            Role.FOLLOWER: self.follower_strategy
        }

    def initialize(self):
        """システムの初期化"""
        quad_names = [f'Quadcopter[{i}]' for i in range(self.params['quad_num'])]
        self.sim.setup_handles(quad_names, 'Cylinder')

        # ドローンオブジェクトの生成
        leader_idx = self.params['leader_idx']
        initial_positions = self.params['initial_positions']
        for i in range(self.params['quad_num']):
            role = Role.LEADER if i == leader_idx else Role.FOLLOWER
            quad = Quadcopter(i, self.sim.quad_handles[i], initial_positions[i], role)
            self.quads.append(quad)
        
        self.leader = self.quads[leader_idx]
        self.followers = [q for q in self.quads if q.role == Role.FOLLOWER]
        
        # シミュレータの初期位置を設定
        self.sim.set_all_quad_positions(self.quads)
        self.sim.set_goal_position(self.params['leader_goal'])

    def run(self):
        """メインループの実行"""
        self.sim.connect()
        self.sim.start_simulation()
        self.initialize()
        
        try:
            for loop in range(self.params['loop_num']):
                # 1. ゴール到達前のみ、シミュレータから全機体の状態を取得・更新
                if not self.goal_reached:
                    sim_states = self.sim.get_all_quad_states()
                    for i, quad in enumerate(self.quads):
                        quad.update_state(sim_states[i]['position'], sim_states[i]['velocity'])

                # 2. ゴール到達を判定し、フラグを立てる
                if not self.goal_reached:
                    dist_to_goal = np.linalg.norm(self.leader.position - self.params['leader_goal'])
                    # リーダーが目標から10cm以内に入ったらゴール到達とみなす
                    if dist_to_goal < 10.0:
                        print(f"--- Goal reached at loop {loop+1}. Holding positions. ---")
                        self.goal_reached = True

                # 3. 速度を計算する（ゴール到達後はゼロを設定）
                if self.goal_reached:
                    print("finish")
                    # ゴール到達後は、全機の速度をゼロにしてその場に停留させる
                    for quad in self.quads:
                        quad.next_velocity = np.zeros(3)
                else:
                    for i, quad in enumerate(self.quads):
                        strategy = self.strategies[quad.role]
                        
                        # ストラテジーが必要とする情報をkwargsで渡す
                        strategy_kwargs = {
                            'all_quads': self.quads,
                            'goal': self.params['leader_goal'],
                            'max_speed': self.params['leader_speed'],
                            'leader': self.leader,
                            'formation': V_SHAPE_FORMATION,
                            'follower_idx': self.followers.index(quad) if quad.role == Role.FOLLOWER else -1
                        }
                        quad.next_velocity = strategy.calculate_velocity(quad, **strategy_kwargs)

                # 4. 内部の位置情報を更新（ゴール到達後は、速度ゼロで更新されるため位置は変わらない）
                for quad in self.quads:
                    quad.update_position(self.params['dt'])

                # 5. 新しい位置をシミュレータに反映（ゴール到達後は、固定された位置が送られ続ける）
                self.sim.set_all_quad_positions(self.quads)
                self.sim.step_simulation()
                
                print(f"Loop {loop+1}/{self.params['loop_num']}")
                time.sleep(0.05)

        except KeyboardInterrupt:
            print("\nSimulation interrupted by user.")
        finally:
            self.sim.stop_simulation()

    def debug(self):
        """デバッグ用のメソッド"""
        for quad in self.quads:
            print(f"Quad {quad.id} - Position: {quad.position}, Velocity: {quad.velocity}, Role: {quad.role}")
        self.sim.stop_simulation()


if __name__ == '__main__':
    # パラメータを一元管理
    sim_params = {
        'quad_num': 5,
        'loop_num': 5000,
        'leader_idx': 0,
        'dt': 0.5,
        'distance_threshold': 80.0,
        'leader_speed': 10.0,
        'leader_goal': np.array([500.0, -15.0, 250.0]),
        'k0l': np.array([[5, 200], [5, 200], [5, 200], [5, 200]]),
        'kps': np.array([[1, 1], [1, 1], [1, 1], [1, 1]]),
        'initial_positions': np.array([
            [-400, 0, 220],    # リーダー
            [-420, -110, 250], # フォロワ1
            [-500, -60, 250],  # フォロワ2
            [-520, 45, 250],   # フォロワ3
            [-600, -110, 250]  # フォロワ4
        ])
    }
    
    controller = MainController(sim_params)
    controller.run()
