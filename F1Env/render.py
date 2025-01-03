import pygame
import csv

# Load track data from CSV

pygame.init()
width, height = 1200, 800
screen = pygame.display.set_mode((width, height))
pygame.display.set_caption('Pygame Window')

# Global variables
track = None
offset_x = 0
offset_y = 0
scale = 1
mouse_dragging = False
last_mouse_pos = None

def load_track(filename):
  track = []
  with open(filename, newline='') as csvfile:
    track_reader = csv.reader(csvfile)
    next(track_reader)  # Skip the header row
    for row in track_reader:
      track.append([float(value) for value in row])
  return track

# Draw track on the screen


def draw_track(screen, track, offset_x=0, offset_y=0, scale=1):
  for i in range(len(track) - 1):
    x_right1, y_right1, x_left1, y_left1 = track[i]
    x_right2, y_right2, x_left2, y_left2 = track[i + 1]
    pygame.draw.line(screen, (255, 255, 255), (x_right1 * scale + offset_x, y_right1 * scale + offset_y), (x_right2 * scale + offset_x, y_right2 * scale + offset_y), 2)
    pygame.draw.line(screen, (255, 255, 255), (x_left1 * scale + offset_x, y_left1 * scale + offset_y), (x_left2 * scale + offset_x, y_left2 * scale + offset_y), 2)

# Draw car position and vectors


def draw_car(screen, car_pos, car_vectors, offset_x=0, offset_y=0, scale=1):
  car_x, car_y = car_pos
  car_screen_pos = (car_x * scale + offset_x, car_y * scale + offset_y)
  pygame.draw.circle(screen, (255, 0, 0), car_screen_pos, 5)  # Draw car as a red circle
  for vector in car_vectors:
    vector_x, vector_y = vector
    vector_screen_pos = (car_screen_pos[0] + vector_x * scale, car_screen_pos[1] + vector_y * scale)
    pygame.draw.line(screen, (0, 255, 0), car_screen_pos, vector_screen_pos, 2)  # Draw each vector as a green line


def render(listener):
  global track, offset_x, offset_y, scale, mouse_dragging, last_mouse_pos

  if track is None:
    # Load the track
    track = load_track('csv/austria.csv')
    # Calculate initial offset to center the track
    track_x_min = min(min(point[0], point[2]) for point in track)
    track_x_max = max(max(point[0], point[2]) for point in track)
    track_y_min = min(min(point[1], point[3]) for point in track)
    track_y_max = max(max(point[1], point[3]) for point in track)
    track_width = track_x_max - track_x_min
    track_height = track_y_max - track_y_min
    offset_x = (width - track_width) // 2 - track_x_min
    offset_y = (height - track_height) // 2 - track_y_min

  # Main game loop
  scroll_speed = 10

  for event in pygame.event.get():
    if event.type == pygame.QUIT:
      running = False
    elif event.type == pygame.MOUSEBUTTONDOWN:
      if event.button == 1:  # Left mouse button
        mouse_dragging = True
        last_mouse_pos = event.pos
    elif event.type == pygame.MOUSEBUTTONUP:
      if event.button == 1:  # Left mouse button
        mouse_dragging = False
    elif event.type == pygame.MOUSEMOTION:
      if mouse_dragging:
        mouse_x, mouse_y = event.pos
        dx = mouse_x - last_mouse_pos[0]
        dy = mouse_y - last_mouse_pos[1]
        offset_x += dx
        offset_y += dy
        last_mouse_pos = event.pos
    elif event.type == pygame.MOUSEWHEEL:
      if event.y > 0:  # Scroll up
        scale *= 1.1
      elif event.y < 0:  # Scroll down
        scale /= 1.1

  # Fill the screen with a background color (RGB)
  screen.fill((0, 0, 0))

  # Get car position and vectors
  car_x, car_y, car_vectors = listener.get_car_position()

  car_pos = (car_x, car_y)

  # Adjust offset to track the car
  offset_x = width // 2 - car_x * scale
  offset_y = height // 2 - car_y * scale

  # Draw the track with offset and scale
  draw_track(screen, track, offset_x, offset_y, scale)

  # Draw car position and vectors
  draw_car(screen, car_pos, car_vectors, offset_x, offset_y, scale)

  # Update the display
  pygame.display.flip()
