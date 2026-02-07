from environment import Environment as En
import numpy as np
import matplotlib.pyplot as plt
import random
import math


env = En(is_discrete=True)
env.add_random_obstacles(45)
env.plot()

plt.show()