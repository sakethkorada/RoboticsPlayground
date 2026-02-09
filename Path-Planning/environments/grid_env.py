import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
import random

class GridEnvironment:
    def __init__(self, width=100, height=100, weighted=True):
        self.width = width
        self.height = height
        self.obstacles = [] # List of (x, y, w, h)
    
        self.map = np.ones((height, width))
        self.weights = [3,5,10,15]


    def add_random_obstacles(self, count=10):  
        for _ in range(count * 2): 
            cost = random.randint(2, 5) # Cost 2 to 5
            w, h = random.randint(3, 8), random.randint(3, 8)
            x = random.randint(0, self.width - w)
            y = random.randint(0, self.height - h)
            self.map[y:y+h, x:x+w] = cost 

    
        for _ in range(count):
            w, h = random.randint(2, 5), random.randint(2, 5)
            x = random.randint(0, self.width - w)
            y = random.randint(0, self.height - h)
            self.map[y:y+h, x:x+w] = 0 # 0 means cant' pass

        
    def get_obstacles(self):
        return self.obstacles   
    
    def get_map(self):
        return self.map

    def generate_random_pt(self):
        x = random.randint(0, self.width )
        y = random.randint(0, self.height)
        return (x,y)
    
    def _create_blob(self, start_x, start_y, target_val, max_size):
        """
        Grows a 'blob' of terrain using a randomized flood fill.
        """
        # Queue stores potential candidates: (x, y)
        active_cells = [(start_x, start_y)]
        self.map[start_y, start_x] = target_val
        current_size = 1

        while current_size < max_size and len(active_cells) > 0:
            
            idx = random.randint(0, len(active_cells) - 1)
            curr_x, curr_y = active_cells[idx]

            
            moves = [(0, 1), (0, -1), (1, 0), (-1, 0)]
            random.shuffle(moves) # Randomize direction order

            grown = False
            for dx, dy in moves:
                nx, ny = curr_x + dx, curr_y + dy

                # Check Bounds and make sure we don't overwrite existing terrain of same type
                if (0 <= nx < self.width and 0 <= ny < self.height and 
                    self.map[ny, nx] != target_val):
                    
                    self.map[ny, nx] = target_val
                    active_cells.append((nx, ny))
                    current_size += 1
                    grown = True
                    break # Only grow once per loop iteration to keep shape irregular
            
            
            if not grown:
                active_cells.pop(idx)

    def generate_natural_terrain(self, num_blobs=15):
        """
        Generates obstacles and terrain costs using organic blobs.
        """
        
        for _ in range(num_blobs):
            # Random center
            cx = random.randint(0, self.width - 1)
            cy = random.randint(0, self.height - 1)
            
            
            # Random Cost (2=Grass, 5=Mud)
            cost = self.weights[random.randint(0,len(self.weights)-1)]
            
            # Random Size (Small puddle vs Big Forest)
            area = self.width * self.height
            blob_size = int((random.randrange(0,5) / 100.0) *area)
            
            self._create_blob(cx, cy, cost, blob_size)

        
      
        for _ in range(int(num_blobs * 0.7)):
            cx = random.randint(0, self.width - 1)
            cy = random.randint(0, self.height - 1)
            
            # Obstacles usually smaller/tighter
            area = self.width * self.height
            blob_size = int((random.randrange(0,3) / 100.0) *area)
            self._create_blob(cx, cy, 0, blob_size)

    def plot(self):
        plt.clf() 
        plt.xlim(0, self.width)
        plt.ylim(0, self.height)
        plt.gca().set_aspect('equal', adjustable='box')
     
        unique_costs = [0, 1] + sorted(self.weights) # e.g. [0, 1, 3, 5, 10, 15]
        
        
        visual_map = np.searchsorted(unique_costs, self.map)
        
        
        num_categories = len(unique_costs)
        bounds = np.arange(num_categories + 1) # [0, 1, 2, 3, 4, 5, 6]
        
        #map colors
        colors_list = ['black', "#09e52d", "#053e0e", "#EF70CB", "#adecf5d3", "#11d5efd4"]
        cmap = mcolors.ListedColormap(colors_list)
        norm = mcolors.BoundaryNorm(bounds, cmap.N)

        #plot visual map
        im = plt.imshow(visual_map, cmap=cmap, norm=norm, origin='lower', 
                        extent=[0, self.width, 0, self.height])
        
        #center ticks
        tick_locs = bounds[:-1] + 0.5
        cbar = plt.colorbar(im, ticks=tick_locs)
        
        #original labels
        labels = ['Obstacle'] + [f'Cost {c}' for c in unique_costs[1:]]
        cbar.ax.set_yticklabels(labels)
        
       