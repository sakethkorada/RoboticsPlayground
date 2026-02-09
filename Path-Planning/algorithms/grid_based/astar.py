import sys
from environments.grid_env import GridEnvironment
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import random
import math
import heapq
import pprint
class astar:
    class Node:
        def __init__(self, x, y, parent=None, weight=0):
            self.x = x
            self.y = y
            self.parent = parent
            self.weight = weight
        
    def __init__ (self, startCoord, endCoord, map, weight=1.5,huerestic="Manhattan"):
        self.startCoord = startCoord #indice of goal
        self.endCoord = endCoord
        self.map = map
        self.parent_map = {}
        self.weight = weight
        self.heurestic = huerestic


    def huerestic_function(self, curCoord):
        x_cur, y_cur = curCoord
        x_end, y_end = self.endCoord
        if(self.heurestic == "Manhattan"):
            return abs(x_cur-x_end) + abs(y_cur-y_end)

        elif(self.heurestic == "Euclidean"):
            return math.sqrt( (x_cur-x_end)**2 + (y_cur-y_end)**2)
        else:
            return 0

    
    def planning(self):
        heap = []
        #initialize Start Coord
        start_x, start_y = self.startCoord
        end_x, end_y = self.endCoord

        if(self.map[start_y][start_x] == 0 or self.map[end_y][end_x] == 0):
            sys.exit('Start or End Point in Obstacle')
        
        #Heap structure (curWeight, curCoord, parentCoord)
        h_start = self.huerestic_function(self.startCoord)
        heapq.heappush(heap, (h_start, 0, self.startCoord, (-1,-1)))
       

        #plot start/end point
        plt.plot(start_x +0.5, start_y+0.5, marker='o',color='red', markersize=5, linestyle='None')
        plt.plot(end_x +0.5, end_y+0.5, marker='o',color='red', markersize= 5, linestyle='None')
        
        directions = [(-1,0), (1,0), (0,1), (0,-1)]

        while(heap):
        
            totalCost, cost, coord, parentCoord = heapq.heappop(heap)
            if(coord in self.parent_map):
                continue
            x, y = coord
            
            self.parent_map[coord] = parentCoord 
            plt.pause(0.01)
            
            if(coord == self.endCoord):
                break

            if(coord != self.startCoord):
                plt.plot(x +0.5, y+0.5, marker='o',color='white', markersize=3, linestyle='None')
            
            for dx, dy in directions:
            
                x_new = x+ dx
                y_new = y + dy
                
                #out of bounds check
                
                if(x_new < 0 or y_new < 0 or x_new >= len(self.map[0]) or y_new >= len(self.map)):
                    continue
                elif(self.map[y_new][x_new] == 0 or (x_new, y_new) in self.parent_map):
                    continue
                else:
                    step_cost = self.map[y_new][x_new]
                    g_cost = step_cost + cost
                    h_cost = self.huerestic_function((x_new,y_new))
                    f_cost = g_cost + h_cost
                    
                    heapq.heappush(heap, (f_cost, g_cost, (x_new,y_new), coord))
                    
        



        return


    def draw_shortest_path(self):
        curCoord = self.endCoord
        while(self.parent_map[curCoord] != (-1, -1)):
            x,y = curCoord
            plt.plot(x +0.5, y+0.5, marker='o',color='orange', markersize=4, linestyle='None')
            curCoord = self.parent_map[curCoord]
        
            
    
#Constants
start_Coord = (10, 5)
end_Coord = (20,15)        

env = GridEnvironment(width=30, height=30)
env.generate_natural_terrain(num_blobs=13)
env.plot()
map = env.get_map()

algo = astar(start_Coord, end_Coord, map,weight=3)
algo.planning()
algo.draw_shortest_path()

#np.set_printoptions(threshold=sys.maxsize)
#pprint.pprint(map)

plt.show()