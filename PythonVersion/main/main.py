import time

# CoppeliaSimとの連携
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

client = RemoteAPIClient()
sim = client.getObject('sim')

# シミュレーション開始
sim.startSimulation()
time.sleep(0.1)




