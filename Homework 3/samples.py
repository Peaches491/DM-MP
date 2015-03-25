
import os

if __name__ == '__main__':

    for goal_freq in range(1, 100, 5):
        os.system("python HW3.py " + str(goal_freq/100.0))