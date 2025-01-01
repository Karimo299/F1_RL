import os
import sys
import numpy as np
from stable_baselines3 import PPO, SAC
from stable_baselines3.common.callbacks import CheckpointCallback
from F1Env import F1Env

# Configurations
model_name = "SAC_continued"
total_timesteps = 1_000_000
models_path = f"models/{model_name}_{total_timesteps}"
log_path = f"logs/{model_name}_{total_timesteps}"

os.makedirs(models_path, exist_ok=True)
os.makedirs(log_path, exist_ok=True)

env = F1Env()

policy_kwargs = dict(net_arch=dict(pi=[256, 256], qf=[256, 256]))


def train_model(model_path=None):
  if model_path:
    model = SAC.load(model_path, env=env, tensorboard_log=log_path)
  else:
    model = SAC(
        policy="MlpPolicy",
        env=env,
        learning_rate=3e-4,
        buffer_size=1_000_000,
        # learning_starts=10_000,
        batch_size=256,
        tau=0.005,
        gamma=0.99,
        train_freq=(1, "step"),
        gradient_steps=-1,
        action_noise=None,
        replay_buffer_class=None,
        replay_buffer_kwargs=None,
        ent_coef="auto",
        target_update_interval=1,
        target_entropy="auto",
        use_sde=False,
        sde_sample_freq=-1,
        tensorboard_log=log_path,
        policy_kwargs=policy_kwargs,
        verbose=1,
        device="auto",
    )

  checkpoint_callback = CheckpointCallback(save_freq=10000, save_path=models_path, name_prefix=f"{model_name}_ckpt_copy")

  model.learn(total_timesteps=total_timesteps, tb_log_name=model_name, callback=checkpoint_callback)

  # Save the final model
  model.save(os.path.join(models_path, f"{model_name}_{total_timesteps}"))


def use_model(model_path):
  model = SAC.load(model_path, env=env, tensorboard_log=log_path)

  obs, _ = env.reset()
  for i in range(10):  # Run for 10 episodes as an example
    while True:
      action, _states = model.predict(obs, deterministic=True)
      obs, rewards, dones, _, info = env.step(action)

      if dones:
        obs, _ = env.reset()
        break


if __name__ == "__main__":
  if len(sys.argv) < 2:
    print("Usage: python script.py <train|use> [model_path]")
    sys.exit(1)

  mode = sys.argv[1].lower()

if mode == "train":
  model_path = sys.argv[2] if len(sys.argv) > 2 else None
  train_model(model_path)
elif mode == "use":
  if len(sys.argv) < 3:
    print("Usage: python script.py use <model_path>")
    sys.exit(1)
  model_path = sys.argv[2]
  use_model(model_path)
else:
  print("Invalid mode. Use 'train' or 'use'.")
  sys.exit(1)
