import gym
import panda_gym

env = gym.make('PandaReachJointsDense-v2', render=True)

obs = env.reset()
done = False
for i in range(100000):
    action = env.action_space.sample() # random action
    obs, reward, done, info = env.step(action)
    print("======>observation<======", obs['observation'])
    print("------>achieved_goal<------", obs['achieved_goal'])
    print("******>desired_goal<******", obs['desired_goal'])
    print("++++++>reward<++++++", reward)
    print(">>>done<<<", done, ">>>action<<<", action)

env.close()
