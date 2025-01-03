import os
import sys
import time
from stable_baselines3.common.env_checker import check_env

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from F1Env import F1Env

time.sleep(1)
env = F1Env()
env.reset()


check_env(env, warn=True)

# time.sleep(2)
# env.step([0.25, -1])
