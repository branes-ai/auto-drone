from projectairsim import ProjectAirSimClient, Drone, World

client = ProjectAirSimClient()
client.connect()
world = World(client, 'scene_basic_drone.jsonc', delay_after_load_sec=2)
drone = Drone(client, world, 'Drone1')

def on_rgb(topic, data):
    print(f'RGB Type: {type(data)}')
    print(f'RGB Dir: {[x for x in dir(data) if not x.startswith("_")]}')
    if hasattr(data, 'shape'):
        print(f'Shape: {data.shape}')
    if hasattr(data, '__len__'):
        print(f'Len: {len(data)}')
    if hasattr(data, '__dict__'):
        print(f'Dict keys: {data.__dict__.keys()}')
    print(f'Repr (first 200 chars): {repr(data)[:200]}')
    import sys; sys.exit(0)

client.subscribe(drone.sensors['DownCamera']['scene_camera'], on_rgb)
drone.enable_api_control()

import time
time.sleep(3)
print('No image received')
client.disconnect()
