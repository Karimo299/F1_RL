# Description: This script is used to plot the track and the car position in real-time.
import os
import sys
import time
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from F1Env.track_data_thread import Listener, find_intersection


def update_plot(listener, x_left_new, z_left_new, x_right_new, z_right_new, car_x=None, car_z=None, vectors=None):
  plt.clf()
  plt.plot(x_left_new, -z_left_new, '-', label='Smooth Left Border', color='red')
  plt.plot(x_right_new, -z_right_new, '-', label='Smooth Right Border', color='blue')

  if car_x is not None and car_z is not None:
    plt.plot([car_x], [-car_z], 'o', label='Car Position')

    off_track = listener.is_car_off_track()
    plt.title('Car is OFF TRACK!' if off_track else 'Car is ON TRACK!', color='red' if off_track else 'green')

    if vectors is not None:
      colors = ['r--', 'g--', 'b--', 'c--', 'm--']
      labels = ['Straight', '45° Left', '45° Right', '90° Left', '90° Right']

      for i, vector in enumerate(vectors):
        end_x = car_x + vector[0]
        end_z = car_z + vector[1]
        plt.plot([car_x, end_x], [-car_z, -end_z], colors[i], label=labels[i])

  plt.legend()
  plt.xlabel('X')
  plt.ylabel('Y')
  plt.grid()
  plt.draw()
  plt.pause(0.05)


if __name__ == "__main__":

  track_csv = "./csv/austria.csv"

  track_data = pd.read_csv(track_csv)
  x_left_new = track_data['x_left'].values
  z_left_new = track_data['z_left'].values
  x_right_new = track_data['x_right'].values
  z_right_new = track_data['z_right'].values

  listener = Listener()
  plt.ion()
  plt.figure(figsize=(10, 8))

  try:
    time.sleep(1)
    while True:
      if "--track_car" in sys.argv:
        car_x, car_z, vectors = listener.get_car_position()
      else:
        car_x, car_z, vectors = None, None, None
      update_plot(listener, x_left_new, z_left_new, x_right_new, z_right_new, car_x, car_z, vectors)
  except KeyboardInterrupt:
    print('Stopping the application...')
    plt.ioff()
    plt.show()
  except Exception as e:
    print(e)
