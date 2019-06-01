#From https://github.com/the-deep-learners/TensorFlow-LiveLessons/
#modified by Albin

import os
import gym
import numpy as np

from DQNAgent import DQNAgent
from Airplane import Airplane_Env

def train():
    agent = DQNAgent(state_size, action_size) # initialise agent
    #choice = raw_input("Name weight: ")
    #filename = output_dir + "weights_" + choice + ".hdf5"
    #agent.load(filename)
    batch_size = 32
    n_episodes = 10001 # n games we want agent to play (default 1001)
    
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    done = False
    for e in range(n_episodes): # iterate over new episodes of the game
        state = env.reset() # reset state at start of each new episode of the game
        state = np.reshape(state, [1, state_size])

        score = 0

        for time in range(100):  # time represents a frame of the game; goal is to keep pole upright as long as possible up to range, e.g., 500 or 5000 timesteps
        #         env.render()
            action = agent.act(state) # action is either 0 or 1 (move cart left or right); decide on one or other here
            next_state, reward, done, _ = env.step(action) # agent interacts with env, gets feedback; 4 state data points, e.g., pole angle, cart position   
            #reward = reward if not done else -1000 # reward +1 for each additional frame with pole upright        
            next_state = np.reshape(next_state, [1, state_size])
            agent.remember(state, action, reward, next_state, done) # remember the previous timestep's state, actions, reward, etc.        
            state = next_state # set "current state" for upcoming iteration to the current next state 
            score = score + reward       
            if done: # episode ends if agent drops pole or we reach timestep 5000
                print("episode: {}/{}, score: {}, e: {:.2}, time: {}, x: {:.2}" # print the episode's score and agent's epsilon
                        .format(e, n_episodes, score, agent.epsilon, time, state[0,0]))
                break # exit loop
        if len(agent.memory) > batch_size:
            agent.replay(batch_size) # train the agent by replaying the experiences of the episode
        if e % 500 == 0:
            agent.save(output_dir + "weights_" + '{:04d}'.format(e) + ".hdf5")
            #print(env.get_traj())
            #env.plot_traj()

    env.plot_traj()


env = Airplane_Env()
state_size = 6
action_size = 26*5
output_dir = 'model_output/cartpole/'

choice = raw_input("Train? Y/N: ")

if choice == "Y" :
    train()
else:
    agent = DQNAgent(state_size, action_size,0) # initialise agent
    choice = raw_input("Name weight: ")
    filename = output_dir + "weights_" + choice + ".hdf5"
    if not os.path.isfile(filename):
        exit()
    print("Load: " + filename)
    agent.load(filename)
    state = env.reset() # reset state at start of each new episode of the game
    state = np.reshape(state, [1, state_size])

    done = False

    while(True):

        state = env.reset() # reset state at start of each new episode of the game
        state = np.reshape(state, [1, state_size])

        for time in range(5000):  # time represents a frame of the game; goal is to keep pole upright as long as possible up to range, e.g., 500 or 5000 timesteps

            action = agent.act(state) # action is either 0 or 1 (move cart left or right); decide on one or other here
            next_state, reward, done, _ = env.step(action) # agent interacts with env, gets feedback; 4 state data points, e.g., pole angle, cart position        
            if done: # episode ends if agent drops pole or we reach timestep 5000
                break # exit loop
        print(env.get_traj())
        print(env.get_controlls())
        env.plot_traj()


