import gymnasium as gym
import numpy as np
from gymnasium import spaces
import time
from track_data_thread import Listener, KEYS
# from track_data import Listener, KEYS
from movement import Simulate_Input


class F1Env(gym.Env):
    def __init__(self):
        super().__init__()
        self.movement = Simulate_Input()
        self.listener = Listener()
        self.low_speed_steps = 0

        self.total_reward = 0

        self.steering_input = 0
        self.prev_steering = 0

        self.throttle_input = 0
        self.prev_throttle = 0

        self.prev_state = np.array([0.0] * 7, dtype=np.float32)
        self.prev_state[KEYS.lap_distance] = -710

        # Simplified observation space
        self.observation_space = spaces.Box(
            low=np.array([0, 0, 0, 0, -1, -1, -1, -1, -1], dtype=np.float32),
            high=np.array([1, 1, 1, 1, 1, 1, 1, 1, 1], dtype=np.float32),
        )

        # Lap_time
        # Lap_distance
        # Current_lap_invalid
        # Speed
        # Steering
        # Throttle
        # Distance_to_left
        # Distance_to_right
        # Distance_to_line

        # Action space: Steering and Throttle
        self.action_space = spaces.Box(
            low=np.array([-1, -1], dtype=np.float32),
            high=np.array([1, 1], dtype=np.float32),
            shape=(2,),
            dtype=np.float32
        )

    def reset(self, seed=None, options=None):
        print(f"\nResetting environment. Total Reward: {self.total_reward:.2f}\n")
        self.total_reward = 0
        self.prev_state = np.array([0.0] * 9, dtype=np.float32)
        self.prev_state[KEYS.lap_distance] = 0

        self.movement.set_steering(0)
        self.movement.set_throttle(0)
        self.movement.set_brake(0)

        # DO NOT REMOVE

        # self.movement.replay()
        # time.sleep(1)
        # self.movement.extra_action()
        # time.sleep(2)

        self.movement.toggle_pause()
        time.sleep(0.2)
        self.movement.accept()
        time.sleep(6)

        # DO NOT REMOVE

        curr_state = self.listener.get_info()
        if curr_state is None:
            raise ValueError("Listener returned None. Check external system integration.")
        return curr_state, {"position": curr_state[KEYS.lap_distance] * 5000}

    def step(self, action):
        steering_action, throttle_action = action

        print(f"\nSteering: {steering_action:.2f}, Throttle: {throttle_action:.2f}")
        self.steering_input = steering_action
        self.throttle_input = throttle_action

        brake = np.clip(-throttle_action, 0, 1)
        throttle = np.clip(throttle_action, 0, 1)

        self.movement.set_steering(steering_action)
        self.movement.set_throttle(throttle)
        self.movement.set_brake(brake)

        next_state = self.listener.get_info()
        if next_state is None:
            raise ValueError("Listener returned None. Check external system integration.")

        reward = self.calculate_reward(next_state)
        done = self.is_done(next_state)

        self.prev_steering = steering_action
        self.prev_throttle = throttle_action
        self.prev_state = next_state

        return next_state, reward, done, False, {"position": next_state[KEYS.lap_distance] * 5000}

    def off_track(self, state):
        distance_to_left = state[KEYS.distance_to_left] * 15
        distance_to_right = state[KEYS.distance_to_right] * 15

        return max(distance_to_left, distance_to_right) > 15

    def calculate_reward(self, state):
        # Convert normalized values back to original units where needed
        total_track_length = 5000  # Example track length in meters
        lap_distance = state[KEYS.lap_distance] * total_track_length
        prev_lap_distance = self.prev_state[KEYS.lap_distance] * total_track_length
        max_speed = 350  # Example max speed in km/h
        speed = state[KEYS.speed] * max_speed

        distance_to_line = state[KEYS.distance_to_line]  # Remains normalized

        invalid_lap = state[KEYS.current_lap_invalid]
        off_track = self.off_track(state)

        progress = lap_distance - prev_lap_distance
        progress_reward = progress * 0.01 * (speed / max_speed)

        # Base rewards
        speed_reward = speed * 0.02  # Adjust scale for km/h

        # Reward for staying close to the centerline
        line_adherence_reward = max(0, 1 - abs(distance_to_line)) * 7 * (speed / 350)

        low_speed_penalty = -20 if speed < 10 else 0

        invalid_lap_penalty = -100 if invalid_lap or off_track else 0
        # Total reward calculation
        reward = (
            progress_reward +
            speed_reward +
            line_adherence_reward +
            low_speed_penalty +
            invalid_lap_penalty
        )

        print(f"Reward: {reward:.2f} (Progress: {progress_reward:.2f}, Speed: {speed_reward:.2f}, Line: {line_adherence_reward:.2f}, Invalid Lap: {invalid_lap_penalty:.2f}, Low Speed: {low_speed_penalty:.2f})")
        self.total_reward += reward
        return reward

    def is_done(self, state):
        # Terminate if off track
        if self.off_track(state):
            return True

        # Terminate if lap time exceeds the limit
        if state[KEYS.current_lap_time_in_ms] > 1:
            return True

        # Grace period for low speed
        if state[KEYS.speed] * 350 < 5:
            return True

        # Terminate if lap completed
        if self.lap_completed(state):
            return True

        # Invalid lap: Penalize instead of terminating (optional)
        if state[KEYS.current_lap_invalid]:
            return True  # Allow recovery despite invalidity

        return False

    def lap_completed(self, state):

        lap_completed = self.prev_state[KEYS.current_lap_time_in_ms] > state[KEYS.current_lap_time_in_ms]

        if lap_completed:
            print("restart_session")
        return lap_completed
