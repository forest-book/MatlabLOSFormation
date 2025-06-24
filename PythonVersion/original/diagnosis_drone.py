import time
import numpy as np
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# --- 設定 ---
# 診断対象のドローン名（CoppeliaSimのシーンに合わせてください）
DRONE_NAME = '/target[1]' # リーダーではない、いずれかのフォロワー機で試す
TARGET_POSITION_M = np.array([2.0, 2.0, 1.0]) # 移動先の目標座標 [m]

# --- メイン処理 ---
client = RemoteAPIClient()
sim = client.getObject('sim')

# シミュレーションを開始
sim.stopSimulation()
time.sleep(0.5)
sim.startSimulation()
time.sleep(0.5)

try:
    # ドローンのハンドルを取得
    drone_handle = sim.getObject(DRONE_NAME)
    if drone_handle == -1:
        raise Exception(f'Could not get handle for {DRONE_NAME}')

    # 目標位置を示すダミーオブジェクトを作成（任意）
    target_dummy = sim.createDummy(0.1, [255,0,0])
    sim.setObjectPosition(target_dummy, TARGET_POSITION_M.tolist(), -1)
    
    print(f"--- 診断テスト開始：{DRONE_NAME} ---")
    print(f"目標座標: {TARGET_POSITION_M}")
    
    # ドローンに直接、目標速度を設定してみる
    # この速度で目標に向かって安定して飛ぶかを観察する
    # この方法は、ドローンモデルに速度制御の口がある場合に有効です
    initial_pos = np.array(sim.getObjectPosition(drone_handle, -1))
    direction = TARGET_POSITION_M - initial_pos
    direction_norm = np.linalg.norm(direction)
    if direction_norm > 0:
        # 時速 1m/s 程度の速度ベクトル
        target_velocity = (direction / direction_norm) * 1.0 
        
        # 5秒間、同じ速度指令を出し続ける
        print(f"目標速度 {target_velocity.round(2)} を5秒間与えます...")
        start_time = time.time()
        while time.time() - start_time < 5.0:
            # sim.setObjectTargetVelocity APIは存在しないため、
            # ドローンのカスタムUIやカスタムデータブロック経由で速度指令を送るのが一般的
            # ここでは代替として、ドローンを直接動かすのではなく、
            # ドローンが自律的に動くかを見るために、何もしないループを回す
            # もしドローンが全く動かない場合、それは正常です。
            # もしドローンがその場で振動・回転を始めたら、それが問題の兆候です。
            
            # 代わりに、位置を少しずつ動かしてみるテスト
            current_pos = np.array(sim.getObjectPosition(drone_handle, -1))
            step_vec = target_velocity * 0.05 # 50msごとに進む距離
            next_pos = current_pos + step_vec
            
            # ドローンの低レベルコントローラをバイパスして、直接位置を設定
            sim.setObjectPosition(drone_handle, next_pos.tolist(), -1)
            
            time.sleep(0.05)
            
    print("--- 診断テスト終了 ---")

except Exception as e:
    print(f"エラーが発生しました: {e}")
finally:
    sim.stopSimulation()