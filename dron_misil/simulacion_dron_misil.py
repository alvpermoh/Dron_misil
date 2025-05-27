import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback
import time
import torch
import numpy as np
import matplotlib.pyplot as plt
from scipy.io import savemat

import os
import sys

current_dir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(current_dir)

from entorno import DroneEvadeSim


class RewardCallback(BaseCallback):
    def __init__(self, schedule):
        super().__init__()
        self.episode_rewards = []
        self.current_reward = 0
        self.schedule = schedule

    def _on_step(self) -> bool:
        self.current_reward += self.locals['rewards'][0]
        if self.locals['dones'][0]:
            self.episode_rewards.append(self.current_reward)
            self.current_reward = 0

        num_timesteps = self.model.num_timesteps
        new_lr = self.schedule(num_timesteps)
        for param_group in self.model.policy.optimizer.param_groups:
            param_group['lr'] = new_lr
        return True


def lr_schedule(num_timesteps):
    return 7e-4 * (0.99 ** (num_timesteps / 10_000))


def train_model():
    env = DroneEvadeSim(render_mode=None)
    reward_callback = RewardCallback(schedule=lr_schedule)

    policy_kwargs = dict(
        net_arch=[dict(pi=[256, 256], vf=[256, 256])],
        activation_fn=torch.nn.ReLU
    )

    model = PPO(
        "MlpPolicy", env, verbose=1, batch_size=64, n_steps=2048,
        learning_rate=1e-3, gamma=0.99, ent_coef=0.01, gae_lambda=0.95,
        clip_range=0.2, policy_kwargs=policy_kwargs, device='cpu'
    )

    model.learn(total_timesteps=10_000, callback=reward_callback)
    model.save("ppo_dron_evade")

    savemat("rewards.mat", {"rewards": reward_callback.episode_rewards})

    plt.figure(figsize=(10, 5))
    plt.plot(reward_callback.episode_rewards, label="Reward per episode")
    plt.xlabel("Episode")
    plt.ylabel("Reward")
    plt.title("Training Reward Evolution")
    plt.grid()
    plt.legend()
    plt.tight_layout()
    plt.show()


def simulate_model():
    env = DroneEvadeSim(render_mode="prueba")
    model_path = "ppo_dron_evade.zip"

    if not os.path.exists(model_path):
        print("Model not found. Training first...")
        train_model()

    model = PPO.load(model_path)

    for _ in range(1):
        obs, _ = env.reset()
        pruebas = [0, 0]

        while pruebas[0] + pruebas[1] < 10:
            action, _ = model.predict(obs)
            obs, reward, terminated, truncated, _ = env.step(action)
            if terminated:
                obs, _ = env.reset()
                pruebas[0] += 1
                print("Me pilla")
                time.sleep(0.2)
            if truncated:
                obs, _ = env.reset()
                pruebas[1] += 1
                print("Esquivo")
                time.sleep(0.2)



            

        porcentaje = 100 * pruebas[1] / (pruebas[0] + pruebas[1])
        print(f"Simulation result: {porcentaje}%")

    env.close()


def main():
    rclpy.init()

    try:
        simulate_model()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
