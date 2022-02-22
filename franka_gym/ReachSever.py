import torch
import gym
import panda_gym
import numpy as np
import time
import matplotlib.pyplot as plt

from Agent import Agent


class ReachSever:
    def __init__(self):
        # Default parameters
        self.device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        self.test_model = 1000
        self.training_eps = 1000
        self.evaluation_eps = 10
        self.max_step = 200

        # Set env
        env_mode = 'PandaReachDense-v2'
        self.env = gym.make(env_mode, render=True)
        print("Env_Name------>", env_mode)
        # Env configuration
        self.state_dim = int(self.env.observation_space['observation'].shape[0] / 2)
        self.action_dim = self.env.action_space.shape[0]
        self.action_limit = 1
        print("State_dim------>", self.state_dim)
        print("Action_dim------>", self.action_dim)

        # Set a random seed
        seed = 0
        self.env.seed(seed)
        np.random.seed(seed)
        torch.manual_seed(seed)

        # Create an agent
        self.agent = Agent(self.env, self.state_dim, self.action_dim, self.action_limit, device=self.device,
                           automatic_entropy_tuning=True)

    def run(self, mode):
        start_time = time.time()

        if mode == 'train':
            self.agent.train()
            print("--->Start training!!!<---")
        else:
            self.agent.load_model(self.test_model)
            print("#The model:", self.test_model, 'is loaded!!!')

        train_num_steps = 0
        train_sum_returns = 0.
        train_num_episodes = 0
        train_sum_returns_list = []
        episodes_list = []

        # Runs a full experiment, spread over multiple training episodes
        for episode in range(1, self.training_eps + 1):
            # Run one episode
            step_number = 0
            total_reward = 0.

            obs = self.agent.env.reset()
            done = False

            # Keep interacting until agent reaches a terminal state.
            while not (done or step_number == self.max_step):
                self.agent.steps += 1

                if mode == 'train':
                    # Collect experience (s, a, r, s') using some policy
                    relative_dis = obs['desired_goal'] - obs['observation'][0:3]
                    action, _, _ = self.agent.select_action(relative_dis)
                    next_obs, reward, done, _ = self.agent.env.step(action)
                    next_relative_dis = next_obs['desired_goal'] - next_obs['observation'][0:3]

                    # Add experience to replay buffer
                    self.agent.replay_buffer.add(relative_dis, action, reward, next_relative_dis, done)

                    # Start training when the number of experience is greater than batch size
                    if self.agent.steps > self.agent.batch_size:
                        self.agent.train_model()
                else:
                    relative_dis = obs['desired_goal'] - obs['observation'][0:3]
                    mu, _ = self.agent.actor(torch.FloatTensor(relative_dis).to(self.device))
                    action = self.action_limit * torch.tanh(mu).detach().cpu().numpy()
                    next_obs, reward, done, _ = self.agent.env.step(action)

                total_reward += reward
                step_number += 1
                obs = next_obs

            train_num_steps += step_number
            train_sum_returns += total_reward
            train_num_episodes += 1

            if episode % self.evaluation_eps == 0:
                train_average_return = train_sum_returns / train_num_episodes if train_num_episodes > 0 else 0.0
                train_sum_returns_list.append(train_average_return)
                episodes_list.append(episode)

                print('---------------------------------------')
                print('Steps:', train_num_steps)
                print('Episodes:', episode)
                print('TestSteps:', train_num_episodes)
                print('AverageReturn:', round(train_average_return, 2))
                print('Time:', int(time.time() - start_time))
                print('---------------------------------------')

                train_sum_returns = 0
                train_num_episodes = 0

            if episode % 200 == 0:
                self.agent.save_model(episode)
                print("The models are saved!")

        plt.plot(episodes_list, train_sum_returns_list)
        plt.xlabel('Episodes')
        plt.ylabel('AverageReturns')
        plt.show()

    def motion_generate(self, q_current, state_goal):
        pass
