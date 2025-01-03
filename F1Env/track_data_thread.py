import socket
import time
from threading import Thread, Lock
from dataclasses import dataclass
import numpy as np
import pandas as pd
import threading

from F1Env.data_types import PacketHeader, PacketLapData, PacketMotionData, PacketCarTelemetryData
from shapely.geometry import Point, Polygon


@dataclass
class TelemetryData:
  current_lap_time_in_ms: float = 0
  lap_distance: float = 0.0
  current_lap_invalid: int = 0
  speed: float = 0
  steer: float = 0.0
  throttle: float = 0.0
  distance_to_line: float = 0.0
  world_x: float = 0.0
  world_y: float = 0.0
  world_z: float = 0.0
  heading: float = 0.0


@dataclass
class KEYS:
  current_lap_time_in_ms = 0
  lap_distance = 1
  current_lap_invalid = 2
  speed = 3
  steer = 4
  throttle = 5
  distance_to_line = 6
  straight_distance = 7
  left_45_distance = 8
  right_45_distance = 9
  left_90_distance = 10
  right_90_distance = 11


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

    self.track_data = pd.read_csv("./csv/austria.csv")
    self.racing_line = pd.read_csv("./csv/racing_line.csv")

    self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    self.sock.bind((self.UDP_IP, self.UDP_PORT))

    print(f"Listening for telemetry data on {self.UDP_IP}:{self.UDP_PORT}...")
    self.thread = Thread(target=self.process_track_data)
    self.thread.daemon = True
    self.thread.start()

  def process_track_data(self):
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

            forward_x = motion_data.world_forward_dir_x / 32767.0
            forward_z = motion_data.world_forward_dir_z / 32767.0

            heading = np.arctan2(forward_z, forward_x)
            self.telemetry_data.heading = heading

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

    distance_to_line = self.calculate_distance_to_line() / 15
    _, _, vectors = self.get_car_position()

    distances = np.array([calculate_vector_distance(vector) for vector in vectors], dtype=np.float32) / 735
    with self.lock:
      temp = [
          self.telemetry_data.current_lap_time_in_ms,
          self.telemetry_data.lap_distance,
          self.telemetry_data.current_lap_invalid,
          self.telemetry_data.speed,
          self.telemetry_data.steer,
          self.telemetry_data.throttle,
          distance_to_line,
          distances[0],
          distances[1],
          distances[2],
          distances[3],
          distances[4]
      ]

    try:
      return np.array(temp, dtype=np.float32)
    except ValueError as e:
      print(f"Error converting telemetry data to array: {e}")
      return np.zeros(len(temp), dtype=np.float32)  # Fallback

  def get_car_position(self):
    angles = [0, -45, 45, -90, 90]
    vectors = []

    with self.lock:
      car_x = self.telemetry_data.world_x
      car_z = self.telemetry_data.world_z
      car_heading = self.telemetry_data.heading

    for angle in angles:
      rad_angle = np.radians(angle)
      direction_x = np.cos(car_heading + rad_angle)
      direction_z = np.sin(car_heading + rad_angle)

      intersection_left = find_intersection(car_x, car_z, car_x + direction_x * 1000, car_z + direction_z * 1000, self.track_data['x_left'], self.track_data['z_left'])
      intersection_right = find_intersection(car_x, car_z, car_x + direction_x * 1000, car_z + direction_z * 1000, self.track_data['x_right'], self.track_data['z_right'])

      if intersection_left and intersection_right:
        dist_left = np.sqrt((intersection_left[0] - car_x) ** 2 + (intersection_left[1] - car_z) ** 2)
        dist_right = np.sqrt((intersection_right[0] - car_x) ** 2 + (intersection_right[1] - car_z) ** 2)
        if dist_left < dist_right:
          vector = (intersection_left[0] - car_x, intersection_left[1] - car_z)
        else:
          vector = (intersection_right[0] - car_x, intersection_right[1] - car_z)
      elif intersection_left:
        vector = (intersection_left[0] - car_x, intersection_left[1] - car_z)
      elif intersection_right:
        vector = (intersection_right[0] - car_x, intersection_right[1] - car_z)
      else:
        vector = (0, 27.11)  # No intersection found

      vectors.append(vector)

    return car_x, car_z, vectors

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
    racing_line_direction = np.array([closest_point['x'] - previous_point['x'], closest_point['z'] - previous_point['z']])

    # Vector from the racing line to the car
    vector_to_car = np.array([self.telemetry_data.world_x - closest_point['x'], self.telemetry_data.world_z - closest_point['z']])

    # Cross product to determine left or right
    cross_product = np.cross(racing_line_direction, vector_to_car)

    # Modify the distance based on the position relative to the racing line
    if cross_product > 0:
      distance_to_line = -distance_to_line

    return distance_to_line

  def is_car_off_track(self):
    """
    Check if the car is off-track using Shapely.
    """
    car_position = Point(self.telemetry_data.world_x, self.telemetry_data.world_z)

    left_border = list(zip(self.track_data['x_left'], self.track_data['z_left']))
    right_border = list(zip(self.track_data['x_right'], self.track_data['z_right']))
    track_polygon = Polygon(left_border + right_border[::-1])

    return not track_polygon.contains(car_position)


def calculate_vector_distance(vector):
  return f"{float(np.sqrt(vector[0] ** 2 + vector[1] ** 2)):.2f}"


def find_intersection(x1, z1, x2, z2, border_x, border_z):
  """
  Find the intersection of a line segment with a polyline.
  """
  heading_line_dx = x2 - x1
  heading_line_dz = z2 - z1

  closest_dist = 10000
  intersection_point = None

  for i in range(len(border_x) - 1):
    x3, z3 = border_x[i], border_z[i]
    x4, z4 = border_x[i + 1], border_z[i + 1]

    border_dx = x4 - x3
    border_dz = z4 - z3

    denominator = (heading_line_dx * border_dz - heading_line_dz * border_dx)
    
    if abs(denominator) < 1e-6:
      continue

    t = ((x3 - x1) * border_dz - (z3 - z1) * border_dx) / denominator
    u = ((x3 - x1) * heading_line_dz - (z3 - z1) * heading_line_dx) / denominator

    if 0 <= t <= 1 and 0 <= u <= 1:
      intersect_x = x1 + t * heading_line_dx
      intersect_z = z1 + t * heading_line_dz
      dist = np.sqrt((intersect_x - x1) ** 2 + (intersect_z - z1) ** 2)
      if dist < closest_dist:
        closest_dist = dist
        intersection_point = (intersect_x, intersect_z)

  return intersection_point
