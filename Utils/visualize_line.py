# Real-time visualization of the track data
# tracks the position of the car on the track
# and plots it on a graph

# Usage: python visualize_line.py [--save output.csv]

# the data will be plotted in real-time
# If --save is provided, the data will be saved to the output file
# Press Ctrl+C to stop the application

# Used to get the track limits by driving the car around the track on each border, and recording the racing line too


import os
import sys
import time
import matplotlib.pyplot as plt
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from F1Env.track_data_thread import Listener, find_intersection

save_to_csv = False
output_file = 'output.csv'


def update_plot(x, z):
  plt.clf()
  plt.plot(x, z, 'bo')
  plt.xlim(min(x) - 10, max(x) + 10)
  plt.ylim(min(z) - 10, max(z) + 10)
  plt.xlabel('X Position')
  plt.ylabel('Z Position')
  plt.title('Real-time Track Plot')
  plt.draw()
  plt.pause(0.1)


def main(listener, x_data, z_data, save_to_csv, output_file):
  observation_space = listener.get_info()
  world_x, world_z, _ = listener.get_car_position()

  x_data.append(world_x)
  z_data.append(world_z)

  if save_to_csv:
    with open(output_file, 'a') as f:
      f.write(f"{world_x},{world_z}\n")

  update_plot(x_data, z_data)


if __name__ == "__main__":
  save_to_csv = ("--save" in sys.argv)
  if save_to_csv:
    try:
      output_file = sys.argv[sys.argv.index("--save") + 1]
    except IndexError:
      print("Please provide an output file name after '--save'")
      sys.exit(1)

  try:
    listener = Listener()
    x_data = []
    z_data = []

    plt.ion()
    plt.figure(figsize=(10, 8))

    while True:
      main(listener, x_data, z_data, save_to_csv, output_file)

  except KeyboardInterrupt:
    print('Stopping the application...')
    plt.ioff()
    plt.show()

  except Exception as e:
    print(e)
