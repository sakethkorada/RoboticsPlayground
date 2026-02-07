import numpy as np
import matplotlib.pyplot as plt
import random

class Environment:
    def __init__(self, width=100, height=100, is_discrete=False):
        self.width = width
        self.height = height
        self.obstacles = [] # List of (x, y, w, h)
        self.is_discrete = is_discrete
        if(self.is_discrete):
            self.map = [[0 for _ in range(width)] for _ in range(height)]


    def add_random_obstacles(self, count=10):
        if(self.is_discrete == False):
            for _ in range(count):
                w, h = random.randint(2, 5), random.randint(2, 5)
                x = random.randint(0, self.width - w)
                y = random.randint(0, self.height - h)
                self.obstacles.append((x, y, w, h))
        else:
            for _ in range(count):
                w,h = random.randint(1,3), random.randint(2,5)
                x = random.randint(0, len(self.map[0]) -w)
                y = random.randint(0, len(self.map)- h)
                for i in range(y, y+h):
                    for j in range(x, x+w):
                        self.map[i][j] = 1
    
    def get_obstacles(self):
        return self.obstacles   
    
    def get_map(self):
        return self.map

    def generate_random_pt(self):
        
        x = random.randint(0, self.width )
        y = random.randint(0, self.height)
        return (x,y)

    def plot(self):
        plt.clf() # Clear previous frame if animating
        plt.xlim(0, self.width)
        plt.ylim(0, self.height)
        plt.gca().set_aspect('equal', adjustable='box')
        if self.is_discrete:
            plt.imshow(self.map, cmap='binary', origin='lower', extent=[0, self.width, 0, self.height])
            #plt.grid(which='major', color='gray', linestyle='-', linewidth=0.5, alpha=0.3)
            
        else:

           
            for (x, y, w, h) in self.obstacles:
                rect = plt.Rectangle((x, y), w, h, color='green')
                plt.gca().add_patch(rect)