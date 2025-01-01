import socket
import time
from dataclasses import dataclass
import numpy as np
import pandas as pd

from data_types import PacketHeader, PacketLapData, PacketMotionData, PacketCarTelemetryData


@dataclass
class KEYS:
  current_lap_time_in_ms = 0
  lap_distance = 1
  current_lap_invalid = 2
  speed = 3
  distance_to_left = 4
  distance_to_right = 5
  distance_to_line = 6


PACKET_LAP_DATA = (2024, 1, 2)
PACKET_MOTION_DATA = (2024, 1, 0)
PACKET_CAR_TELEMETRY = (2024, 1, 6)


class Listener:
  def __init__(self):
    self.UDP_IP = "127.0.0.1"
    self.UDP_PORT = 20777

    self.left_track = pd.read_csv("csv/left_track.csv")
    self.right_track = pd.read_csv("csv/right_track.csv")
    self.racing_line = pd.read_csv("csv/racing_line.csv")

    self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    self.sock.bind((self.UDP_IP, self.UDP_PORT))

    print(f"Listening for telemetry data on {self.UDP_IP}:{self.UDP_PORT}...")

  def get_info(self):
    packets_recieved = [0, 0, 0]
    output = np.zeros(7, dtype=np.float32)
    while True:
      try:
        packet = self.sock.recvfrom(2048)  # Buffer size
        data, addr = packet

        # Unpack the packet
        header = PacketHeader.from_buffer_copy(data)
        key = (header.packet_format, header.packet_version, header.packet_id)

        if key == PACKET_LAP_DATA:
          packets_recieved[0] += 1
          packet_lap_data = PacketLapData.from_buffer_copy(data)
          curr_lap_data = packet_lap_data.lap_data[0]

          output[KEYS.current_lap_time_in_ms] = curr_lap_data.current_lap_time_in_ms
          output[KEYS.lap_distance] = curr_lap_data.lap_distance
          output[KEYS.current_lap_invalid] = curr_lap_data.current_lap_invalid

        elif key == PACKET_CAR_TELEMETRY:
          packets_recieved[2] += 1
          packet_car_telemetry_data = PacketCarTelemetryData.from_buffer_copy(data)
          telemetry = packet_car_telemetry_data.car_telemetry_data[0]

          output[KEYS.speed] = telemetry.speed

        elif key == PACKET_MOTION_DATA:
          packets_recieved[1] += 1
          packet_motion_data = PacketMotionData.from_buffer_copy(data)
          motion_data = packet_motion_data.car_motion_data[0]

          left_distances = np.sqrt((self.left_track['x'] - motion_data.world_position_x) ** 2 + (self.left_track['z'] - motion_data.world_position_z) ** 2)
          distance_to_left = left_distances.min()

          # Compute the minimum distance from the point to the right track boundary
          right_distances = np.sqrt((self.right_track['x'] - motion_data.world_position_x) ** 2 + (self.right_track['z'] - motion_data.world_position_z) ** 2)
          distance_to_right = right_distances.min()

          distance_to_line = self.calculate_distance_to_line(motion_data.world_position_x, motion_data.world_position_z)

          output[KEYS.distance_to_left] = distance_to_left
          output[KEYS.distance_to_right] = distance_to_right
          output[KEYS.distance_to_line] = distance_to_line
        # Make sure all packets have been received before returning the data
        # In-game lag when the game reacts to input data can cause packets to be missed
        if min(packets_recieved) == 6:
          packets_recieved = [0, 0, 0]
          return output

      except Exception as e:
        print(f"Error receiving or processing data: {e}")

  def calculate_distance_to_line(self, x, z):
    # Compute the minimum distance from the point to the racing line
    racing_line_distances = np.sqrt((self.racing_line['x'] - x) ** 2 + (self.racing_line['z'] - z) ** 2)
    distance_to_line = racing_line_distances.min()

    # Determine if the car is to the left or right of the racing line
    closest_point_index = racing_line_distances.argmin()
    closest_point = self.racing_line.iloc[closest_point_index]

    # Handle edge case for the first point
    if closest_point_index == 0:
      previous_point = self.racing_line.iloc[closest_point_index]
    else:
      previous_point = self.racing_line.iloc[closest_point_index - 1]

    # Compute direction of the racing line
    racing_line_direction = np.array([closest_point['x'] - previous_point['x'],
                                      closest_point['z'] - previous_point['z']])

    # Vector from the racing line to the car
    vector_to_car = np.array([x - closest_point['x'], z - closest_point['z']])

    # Cross product to determine left or right
    cross_product = np.cross(racing_line_direction, vector_to_car)

    # Modify the distance based on the position relative to the racing line
    if cross_product > 0:
      distance_to_line = -distance_to_line

    return distance_to_line


if __name__ == '__main__':
  listener = Listener()
  while True:
    listener.get_info()
