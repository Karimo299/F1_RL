# from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env
from F1Env import F1Env
import time

time.sleep(1)
env = F1Env()
env.reset()


check_env(env, warn=True)

# time.sleep(2)
# env.step([0.25, -1])
