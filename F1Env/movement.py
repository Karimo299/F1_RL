import pyvjoy
import time


class Simulate_Input:
  def __init__(self):
    self.j = pyvjoy.VJoyDevice(1)
    self.MAX_VALUE = 32767

  def set_steering(self, normalised_steer):
    self.j.set_axis(pyvjoy.HID_USAGE_X, int(normalised_steer * (self.MAX_VALUE / 2) + (self.MAX_VALUE / 2)))

  def set_throttle(self, normalised_throttle):
    self.j.set_axis(pyvjoy.HID_USAGE_Y, int(normalised_throttle * self.MAX_VALUE))

  def set_brake(self, normalised_brake):
    self.j.set_axis(pyvjoy.HID_USAGE_Z, int(normalised_brake * self.MAX_VALUE))

  def toggle_pause(self):
    self.j.set_button(1, 1)
    time.sleep(0.2)
    self.j.set_button(1, 0)

  def replay(self):
    self.j.set_button(7, 1)
    time.sleep(0.2)
    self.j.set_button(7, 0)

  def extra_action(self):
    self.j.set_button(8, 1)
    time.sleep(0.2)
    self.j.set_button(8, 0)

  def accept(self):
    self.j.set_button(4, 1)
    time.sleep(0.2)
    self.j.set_button(4, 0)

  def upshift(self):
    self.j.set_button(2, 1)
    time.sleep(0.2)
    self.j.set_button(2, 0)

  def downshift(self):
    self.j.set_button(3, 1)
    time.sleep(0.2)
    self.j.set_button(3, 0)

  def toggle_drs(self):
    self.j.set_button(5, 1)
    time.sleep(0.2)
    self.j.set_button(5, 0)

  def toggle_ers(self):
    self.j.set_button(6, 1)
    time.sleep(0.2)
    self.j.set_button(6, 0)


# Test Controls
if __name__ == '__main__':
  controls = Simulate_Input()
  time.sleep(3)

  # controls.extra_action()
  # controls.set_brake(0)
  # # Set it left
  # controls.set_steering(0)
  # time.sleep(1)
  # controls.set_steering(1)

  controls.toggle_ers()
