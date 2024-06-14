import gym
import sofagym.envs
import time
import numpy as np

env = gym.make('finger-v0')
env.seed(42)
observation = env.reset()
print(observation)
dt = env.config['dt']

done = False
for i in range(60):
    print("step")
    print(i+1)
    
    
    # print("buffer 1")
    # print(temp_obs_1)
    # temp_obs_2, reward, done, info = env.step(2)
    # print("buffer 2")
    # print(temp_obs_2)
    # j = 3
    # temp_obs_1, reward, done, info = env.step(2)
    # last_v = temp_obs_1[1] / dt
    # while not np.isclose(temp_obs_1[0][1], 0):
    #     temp_obs_1, reward, done, info = env.step(2)
    #     current_v = temp_obs_1[1] / dt
    #     if np.isclose(current_v, last_v) and np.isclose(current_v, 0):
    #         break
    #     last_v = current_v
    #     print(temp_obs_1)
    
    temp_obs_1, reward, done, info = env.step(2)
    
    print("buffer 1")
    print(temp_obs_1)
    
    temp_obs_2, reward, done, info = env.step(2)
    print("buffer 2")
    print(temp_obs_2)
    j = 3
    while not np.isclose(temp_obs_1[0][1], temp_obs_2[0][1]):
         temp_obs_1 = temp_obs_2
         temp_obs_2, reward, done, info = env.step(2)
         print("buffer "+str(j))
         print(temp_obs_2)
         j += 1


    # observation, reward, done, info = env.step(0)
    # print(observation)
    
    
    env.render()
   
env.close()
