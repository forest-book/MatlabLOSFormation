import time
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# CoppeliaSimに接続
client = RemoteAPIClient()
sim = client.getObject('sim')

print('CoppeliaSimに接続しました。')

try:
    # シミュレーションを開始
    sim.startSimulation()

    handle = sim.getObject("/Quadcopter[0]/fastHokuyo[0]")

    # 10秒間データを取得し続けるループ
    start_time = time.time()
    packed_ranges11 = sim.readCustomDataBlock(handle,'scan_ranges11')
    packed_ranges12 = sim.readCustomDataBlock(handle,'scan_ranges12')
    while time.time() - start_time < 10:
        # LiDARデータのシグナルをバイト列として取得
        packed_ranges11 = sim.readCustomDataBlock(handle,'scan_ranges11')
        packed_ranges12 = sim.readCustomDataBlock(handle,'scan_ranges12')

        # 取得したデータをアンパックして、Pythonのfloat型リストに変換
        dist11 = client.unpackFloatTable(packed_ranges11) if packed_ranges11 else []
        dist12 = client.unpackFloatTable(packed_ranges12) if packed_ranges12 else []

        # 取得したデータを表示（最初の5点のみ）
        print(f"Time: {time.time() - start_time:.2f}s | LiDAR 11 points: {len(dist11)} | LiDAR 12 points: {len(dist12)}")
        if dist11:
            print(f"  - dist11 (最初の5点): {[f'{d:.3f}' for d in dist11[:5]]}")
        if dist12:
            print(f"  - dist12 (最初の5点): {[f'{d:.3f}' for d in dist12[:5]]}")

        # 0.1秒待機
        time.sleep(0.1)

finally:
    # シミュレーションを停止
    sim.stopSimulation()
    print('シミュレーションを停止しました。')