import numpy as np
import matplotlib.pyplot as plt
import sys
import random
import os

colors = ['red', 'green', 'cyan', 'magenta', 'black', 'purple']
paths_colors = ['blue', 'brown', 'yellow']

def main():
    p = np.loadtxt('main.csv', delimiter=', ').T
    plt.plot(p[0], p[1], color='blue')

    i = 0
    while os.path.exists(f'{i}.csv'):
        p = np.loadtxt(f'{i}.csv', delimiter=', ').T
        i += 1
        plt.plot(p[0], p[1], color=random.choice(colors))

    i = 0
    while os.path.exists(f'uav{i}.csv'):
        p = np.loadtxt(f'uav{i}.csv', delimiter=', ').T
        plt.plot(p[0], p[1], color=paths_colors[i])
        i += 1

    plt.show()




if __name__ == '__main__':
    main()
