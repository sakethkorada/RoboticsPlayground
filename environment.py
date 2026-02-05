import numpy as np
import matplotlib.pyplot as plt
import random

class Environment:
    def __init__(self, width=100, height=100):
        self.width = width
        self.height = height
        self.obstacles = [] # List of (x, y, w, h)

    def add_random_obstacles(self, count=10):
        for _ in range(count):
            w, h = random.randint(1, 3), random.randint(1, 13)
            x = random.randint(0, self.width - w)
            y = random.randint(0, self.height - h)
            self.obstacles.append((x, y, w, h))
    
    def get_obstacles(self):
        return self.obstacles   

    def generate_random_pt(self):
        
        x = random.randint(0, self.width )
        y = random.randint(0, self.height)
        return (x,y)

    def plot(self):
        plt.xlim(0, self.width)
        plt.ylim(0, self.height)
        for (x, y, w, h) in self.obstacles:
            rect = plt.Rectangle((x, y), w, h, color='green')
            plt.gca().add_patch(rect)