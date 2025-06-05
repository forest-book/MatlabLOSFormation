from coppeliasim_zmqremoteapi_client import RemoteAPIClient

client = RemoteAPIClient()
sim = client.require("sim")

sim.setStepping(True)

sim.startSimulation()
# while (t := sim.getSimulationTime()) < 10:
#     print(f'Simulation time: {t:.2f} [s]')
#     sim.step()


try:
    object_name = "SphereTest"
    object_handle = sim.getObject(f"/{object_name}")

    if object_handle == -1:
        print(f"not found {object_name}")
    else:
        print(f"object handle about {object_name}: {object_handle}")

        position = sim.getObjectPosition(object_handle, -1)
        if position:
            print(f"{object_name} position: x={position[0]:.4f}, y={position[1]:.4f}, z={position[2]:.4f}")
        else:
            print(f"faild get position about {object_name}")

except Exception as e:
    print("error:", e)

sim.stopSimulation()