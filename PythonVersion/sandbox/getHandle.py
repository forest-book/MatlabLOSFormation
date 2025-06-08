import time

# CoppeliaSimとの連携
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

client = RemoteAPIClient()
sim = client.getObject('sim')

sim.setStepping(True)

# シミュレーション開始
sim.startSimulation()
time.sleep(0.1)

try:
    object_name = "Quadcopter[1]"
    # [/{オブジェクト名}のように、"/"をつけないとうまくいかない]
    object_handle = sim.getObject(f"/{object_name}")

    if object_handle == -1:
        print(f"not found {object_name}")
    else:
        print(f"object handle about {object_name}: {object_handle}")

        position = sim.getObjectPosition(object_handle, -1)
        if position:
            print(type(position))
            print(f"{object_name} position: x={position[0]:.4f}, y={position[1]:.4f}, z={position[2]:.4f}")
        else:
            print(f"faild get position about {object_name}")

except Exception as e:
    print("error:", e)

sim.stopSimulation()
