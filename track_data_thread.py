import socket
import time
from threading import Thread, Lock
from dataclasses import dataclass
import numpy as np
import pandas as pd
import threading

from data_types import PacketHeader, PacketLapData, PacketMotionData, PacketCarTelemetryData


@dataclass
class TelemetryData:
  current_lap_time_in_ms: float = 0
  lap_distance: float = 0.0
  current_lap_invalid: int = 0
  speed: float = 0
  steer: float = 0.0
  throttle: float = 0.0
  distance_to_left: float = 0.0
  distance_to_right: float = 0.0
  distance_to_line: float = 0.0


@dataclass
class KEYS:
  current_lap_time_in_ms = 0
  lap_distance = 1
  current_lap_invalid = 2
  speed = 3
  steer = 4
  throttle = 5
  distance_to_left = 6
  distance_to_right = 7
  distance_to_line = 8


PACKET_LAP_DATA = (2024, 1, 2)
PACKET_MOTION_DATA = (2024, 1, 0)
PACKET_CAR_TELEMETRY = (2024, 1, 6)


class Listener:
  def __init__(self):
    self.UDP_IP = "127.0.0.1"
    self.UDP_PORT = 20777
    self.telemetry_data = TelemetryData()
    self.lock = Lock()

    self.data_event = threading.Event()

    self.left_track = pd.read_csv("csv/left_track.csv")
    self.right_track = pd.read_csv("csv/right_track.csv")
    self.racing_line = pd.read_csv("csv/racing_line.csv")

    self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    self.sock.bind((self.UDP_IP, self.UDP_PORT))

    print(f"Listening for telemetry data on {self.UDP_IP}:{self.UDP_PORT}...")
    self.thread = Thread(target=self.track_data)
    self.thread.daemon = True
    self.thread.start()

  def track_data(self):
    packets_recieved = [False, False, False]
    while True:
      try:
        # Receive data from the game
        packet = self.sock.recvfrom(2048)
        data, addr = packet

        # Process the header to determine the packet type
        header = PacketHeader.from_buffer_copy(data)
        key = (header.packet_format, header.packet_version, header.packet_id)

        with self.lock:
          if key == PACKET_LAP_DATA:
            packets_recieved[0] = True
            packet_lap_data = PacketLapData.from_buffer_copy(data)
            curr_lap_data = packet_lap_data.lap_data[0]

            self.telemetry_data.current_lap_time_in_ms = curr_lap_data.current_lap_time_in_ms / 1800000
            self.telemetry_data.lap_distance = (curr_lap_data.lap_distance + 750) / 50000
            self.telemetry_data.current_lap_invalid = curr_lap_data.current_lap_invalid

          elif key == PACKET_MOTION_DATA:
            packets_recieved[1] = True
            packet_motion_data = PacketMotionData.from_buffer_copy(data)
            motion_data = packet_motion_data.car_motion_data[0]

            self.telemetry_data.world_x = motion_data.world_position_x
            self.telemetry_data.world_y = motion_data.world_position_y
            self.telemetry_data.world_z = motion_data.world_position_z

          elif key == PACKET_CAR_TELEMETRY:
            packets_recieved[2] = True
            packet_car_telemetry_data = PacketCarTelemetryData.from_buffer_copy(data)
            telemetry = packet_car_telemetry_data.car_telemetry_data[0]

            self.telemetry_data.steer = telemetry.steer
            self.telemetry_data.speed = telemetry.speed / 350

            if telemetry.brake > 0:
              self.telemetry_data.throttle = -telemetry.brake
            else:
              self.telemetry_data.throttle = telemetry.throttle

            if telemetry.gear < 0:
              self.telemetry_data.speed = -self.telemetry_data.speed

        if all(packets_recieved):
          self.data_event.set()
          packets_recieved = [False, False, False]
      except Exception as e:
        print(f"Error receiving or processing data: {e}")

  def get_info(self):
    self.data_event.wait()

    # Compute the minimum distance from the point to the left track boundary
    left_distances = np.sqrt((self.left_track['x'] - self.telemetry_data.world_x) ** 2 + (self.left_track['z'] - self.telemetry_data.world_z) ** 2)
    distance_to_left = left_distances.min() / 15

    # Compute the minimum distance from the point to the right track boundary
    right_distances = np.sqrt((self.right_track['x'] - self.telemetry_data.world_x) ** 2 + (self.right_track['z'] - self.telemetry_data.world_z) ** 2)
    distance_to_right = right_distances.min() / 15

    distance_to_line = self.calculate_distance_to_line() / 15
    with self.lock:
      temp = [
          self.telemetry_data.current_lap_time_in_ms,
          self.telemetry_data.lap_distance,
          self.telemetry_data.current_lap_invalid,
          self.telemetry_data.speed,
          self.telemetry_data.steer,
          self.telemetry_data.throttle,
          distance_to_left,
          distance_to_right,
          distance_to_line
      ]

    try:
      return np.array(temp, dtype=np.float32)
    except ValueError as e:
      print(f"Error converting telemetry data to array: {e}")
      return np.zeros(len(temp), dtype=np.float32)  # Fallback

  def calculate_distance_to_line(self):
    # Compute the minimum distance from the point to the racing line
    racing_line_distances = np.sqrt((self.racing_line['x'] - self.telemetry_data.world_x) ** 2 + (self.racing_line['z'] - self.telemetry_data.world_z) ** 2)
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
    vector_to_car = np.array([self.telemetry_data.world_x - closest_point['x'], self.telemetry_data.world_z - closest_point['z']])

    # Cross product to determine left or right
    cross_product = np.cross(racing_line_direction, vector_to_car)

    # Modify the distance based on the position relative to the racing line
    if cross_product > 0:
      distance_to_line = -distance_to_line

    return distance_to_line
