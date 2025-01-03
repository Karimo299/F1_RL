import pandas as pd
import numpy as np
from shapely.geometry import Polygon, LineString
import matplotlib.pyplot as plt


def find_longest_line(csv_file):
  # Load the track data
  data = pd.read_csv(csv_file)

  # Extract boundaries
  right_boundary = list(zip(data['x_right'], data['z_right']))
  left_boundary = list(zip(data['x_left'], data['z_left']))

  # Create a closed track polygon
  track_boundary = right_boundary + left_boundary[::-1]
  track_polygon = Polygon(track_boundary).buffer(0)  # Fix invalid polygons

  # Find the longest line entirely within the polygon
  points = np.array(track_polygon.exterior.coords)
  max_length = 0
  best_line = None

  for i in range(len(points)):
    for j in range(i + 1, len(points)):
      line = LineString([points[i], points[j]])
      if line.within(track_polygon):
        length = line.length
        if length > max_length:
          max_length = length
          best_line = line

  # Extract the best line's coordinates
  if best_line:
    x_line = [point[0] for point in best_line.coords]
    z_line = [point[1] for point in best_line.coords]
  else:
    x_line, z_line = [], []

  # Plot the track and the longest line
  track_x = [p[0] for p in track_boundary] + [track_boundary[0][0]]
  track_z = [p[1] for p in track_boundary] + [track_boundary[0][1]]

  plt.figure(figsize=(10, 8))
  plt.plot(track_x, track_z, label="Track Boundary", color="blue")
  plt.plot(x_line, z_line, label="Longest Straight Line", color="red", linewidth=2)
  plt.xlabel("X Coordinate")
  plt.ylabel("Z Coordinate")
  plt.title("Track with Longest Straight Line")
  plt.legend()
  plt.grid(True)
  plt.axis("equal")
  plt.show()

  return best_line, max_length


# Example Usage
if __name__ == "__main__":
  file_path = "../csv/austria.csv"  # Replace with your file name
  longest_line, length = find_longest_line(file_path)
  print("Longest Line Coordinates:", list(longest_line.coords) if longest_line else None)
  print("Length of Longest Line:", length)
