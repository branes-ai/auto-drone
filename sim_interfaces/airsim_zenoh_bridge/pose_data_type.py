from projectairsim import ProjectAirSimClient, Drone, World

client = ProjectAirSimClient()
client.connect()
world = World(client, 'scene_basic_drone.jsonc', delay_after_load_sec=2)
drone = Drone(client, world, 'Drone1')

def on_pose(topic, data):
  print(f'Type: {type(data)}')
  print(f'Dir: {dir(data)}')
  if hasattr(data, '__dict__'):
    print(f'Dict: {data.__dict__}')
  print(f'Repr: {repr(data)}')
  import sys; sys.exit(0)

client.subscribe(drone.robot_info['actual_pose'], on_pose)
drone.enable_api_control()

import time
time.sleep(2)
client.disconnect()
