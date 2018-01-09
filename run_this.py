
import time
from maze_env import Maze
from RL_brain import QLearningTable


def update():

    for episode in range(10000):
        # initial observation
        total_steps = 0
        ep_r = 0
        observation = env.reset()  #状态

        while True:
            # fresh env
            env.render()

            # RL choose action based on observation
            action = RL.choose_action(str(observation))

            # RL take action and get next observation and reward
            observation_, reward, done = env.step(action)

            ep_r += reward
            total_steps += 1

            # RL learn from this transition
            RL.learn(str(observation), action, reward, str(observation_))

            # swap observation
            observation = observation_

            # break while loop when end of this episode
            if done in [1,-1]:
                break
        if done == 1:
            print('The success episode %d reward: %d' %(episode,ep_r))
            print('The number of steps per success episode is: %d' %(total_steps))
        else:
            print('The failed episode %d reward: %d' %(episode,ep_r))
            print('The number of steps per failed episode is: %d' %(total_steps))

    print('game over')
    env.destroy()

if __name__ == "__main__":

    env = Maze()
    start = time.clock()
    RL = QLearningTable(actions=list(range(env.n_actions)))  #RL初始化
    env.after(100, update)
    env.mainloop()
    end = time.clock()
    print('The program runtime: %f s' %(end-start))