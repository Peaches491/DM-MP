
import os

if __name__ == '__main__':

    # try:
    #     for i in range(10):
    #         for goal_freq in range(1, 100, 5):
    #             os.system("python HW3.py " + str(goal_freq/100.0))
    # except KeyboardInterrupt:
    #     pass

    try:
        for i in range(30):
            os.system("python HW3.py " + str(0.51))
    except KeyboardInterrupt:
        pass