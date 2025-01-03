# Description: This script reads the left and right track borders from CSV files, performs spline interpolation to smooth the borders,
# and saves the smoothed borders to a new CSV file.

import pandas as pd
import numpy as np
from scipy.interpolate import splprep, splev


def generate_smooth_track_border(left_csv, right_csv, output_csv=None, num_points=1000, smoothing_factor=0):
  # Load the CSVs
  left_data = pd.read_csv(left_csv)
  right_data = pd.read_csv(right_csv)

  # Extract x and y coordinates for both borders
  x_left = left_data['x'].values
  z_left = left_data['z'].values
  x_right = right_data['x'].values
  z_right = right_data['z'].values

  # Perform spline interpolation for the left border
  tck_left, u_left = splprep([x_left, z_left], s=smoothing_factor)
  u_left_new = np.linspace(0, 1, num_points)
  x_left_new, z_left_new = splev(u_left_new, tck_left)

  # Perform spline interpolation for the right border
  tck_right, u_right = splprep([x_right, z_right], s=smoothing_factor)
  u_right_new = np.linspace(0, 1, num_points)
  x_right_new, z_right_new = splev(u_right_new, tck_right)

  # Smooth the transition between the start and end of the track
  num_transition_points = 20

  # Right border transition points
  x_right_transition = np.linspace(x_right_new[-1], x_right_new[0], num_transition_points)
  z_right_transition = np.linspace(z_right_new[-1], z_right_new[0], num_transition_points)

  # Left border transition points
  x_left_transition = np.linspace(x_left_new[-1], x_left_new[0], num_transition_points)
  z_left_transition = np.linspace(z_left_new[-1], z_left_new[0], num_transition_points)

  # Combine smoothed and transition points
  x_right_combined = np.concatenate([x_right_new, x_right_transition])
  z_right_combined = np.concatenate([z_right_new, z_right_transition])
  x_left_combined = np.concatenate([x_left_new, x_left_transition])
  z_left_combined = np.concatenate([z_left_new, z_left_transition])

  # Combine into a DataFrame
  combined_data = pd.DataFrame({
      'x_right': x_right_combined,
      'z_right': z_right_combined,
      'x_left': x_left_combined,
      'z_left': z_left_combined
  })

  # Optionally save the combined smoothed points to a new CSV
  if output_csv:
    combined_data.to_csv(output_csv, index=False)
    print(f"Smoothed track borders saved to {output_csv}")


if __name__ == "__main__":
  # Input and output file paths
  left_csv = "../csv/left_track.csv"
  right_csv = "../csv/right_track.csv"
  output_csv = "../csv/track.csv"

  # Generate and save the smooth track borders
  generate_smooth_track_border(left_csv, right_csv, output_csv)
