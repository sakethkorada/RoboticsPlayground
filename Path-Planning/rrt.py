import numpy as np
import matplotlib.pyplot as plt
import random
from environment import Environment as En
import math





#2d RTT
class RTT:

    
    class Node:

        def __init__(self, x, y, parent=None):
            self.x = x
            self.y = y
            self.parent = parent
    
    def __init__(self, start, goal, obstacles, expand_dist = 2, iterations=1000):
        self.start = start
        self.goal = goal
        self.obstacles = obstacles
        self.expand_dist = expand_dist
        self.iterations = iterations
        self.node_list = []

        
    def find_nearest_node(self,x_new,y_new):
        """
        Finds the closest node based on euclidean distance to new point
        Returns the Node.
        
        :param self: 
        :param x_new: x coordinate of new point
        :param y_new: y coordinate of new point
        """
        return min(self.node_list, key=lambda n: (n.x - x_new)**2 + (n.y - y_new)**2)

    def calculate_new_node_location(self, near_node_location, random_pt):
        """
        Calculates the location of the new node based on how much distance to expand
        Returns (x,y) tuple of new location
        :param self: 
        :param near_node_location: tuple of (x,y) coordinats of near vertice
        :param random_pt: tuple of (x,y) coordinates of random pt
        """
        x_near, y_near = near_node_location
        x_rand, y_rand = random_pt

        deltaX = x_rand - x_near 
        deltaY = y_rand - y_near
        #slope = (y_rand -y_near)/(x_rand - x_near)
        dist = math.sqrt(((deltaY)**2 + (deltaX)**2))
        if(dist == 0):
            return (x_near, y_near)
        ratio = self.expand_dist/ dist

        

        x_new = x_near + ratio* deltaX
        y_new = y_near + ratio * deltaY
        return (x_new, y_new)
    
    def is_node_collision_free(self, near_node_location, new_node_location):
        """
        Checks if there are any objects that intersect the line between near_node and new_node
        return boolean
        
        :param self: 
        :param near_node_location: tuple of (x,y) coordinates of near node location
        :param new_node_location: tuple of (x,y) coordinates of new node location
        """

        def ccw(A, B, C):
            
            # Returns True if points are in Counter-Clockwise order
            return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])

            
        def check_intersect(A, B, C, D):
            # Returns True if line segment AB intersects CD
            return ccw(A, C, D) != ccw(B, C, D) and ccw(A, B, C) != ccw(A, B, D)
        
        for obstacle in self.obstacles:
            # obstacle is a list of (x, y, w, h)
            x0, y0 = obstacle[0], obstacle[1]
            x1, y1 = x0 + obstacle[2], y0 + obstacle[3]
            
            #if new point is inside obstacle
            if (x0 <= new_node_location[0] <= x1) and (y0 <= new_node_location[1] <= y1):
                return False
            if(check_intersect(near_node_location, new_node_location, (x0,y0), (x0,y1))):
                return False
            if(check_intersect(near_node_location, new_node_location, (x0,y0), (x1,y0))):
                return False
            if(check_intersect(near_node_location, new_node_location, (x0,y1), (x1,y1))):
                return False
            if(check_intersect(near_node_location, new_node_location, (x1,y1), (x1,y0))):
                return False
                
        return True

    #implements RRT algo 
    def planning(self):
        start_node = RTT.Node(self.start[0], self.start[1])
        self.node_list.append(start_node)
        plt.plot(start_node.x,start_node.y, 'bo')
        plt.plot(self.goal[0], self.goal[1], 'ro')
        for _ in range(self.iterations):
            #generate random point
            random_pt = env.generate_random_pt()

            #find nearest vertice
            nearest_node = self.find_nearest_node(random_pt[0], random_pt[1])
            x_near, y_near = nearest_node.x, nearest_node.y

            #calculate new vertice 
            new_node_location = self.calculate_new_node_location((x_near, y_near), random_pt)

            #check if joining new vertice and nearest vertice causes collision
            #if no collision, add it
            #else continue
            is_node_valid = self.is_node_collision_free((x_near,y_near), new_node_location)
            if is_node_valid:
                new_node = RTT.Node(new_node_location[0], new_node_location[1], nearest_node)
                self.node_list.append(new_node)
                plt.plot(new_node.x,new_node.y, 'ko', markersize=1)
                plt.plot((new_node.x, x_near), (new_node.y, y_near), 'k')
                plt.pause(0.01)                
            else:
                continue 
            
            
        return

    #finds a path
    def find_path(self):

        closest_node = self.find_nearest_node(self.goal[0], self.goal[1])
        while(closest_node.parent != None):
            x_cords = (closest_node.x, closest_node.parent.x)
            y_cords = (closest_node.y, closest_node.parent.y)

            plt.plot(x_cords, y_cords, 'b')
            closest_node = closest_node.parent

        return

# Usage
env = En()
env.add_random_obstacles(30)
env.plot()

obstacles = env.get_obstacles()
    

rtt = RTT((30,30),(50,60), obstacles, expand_dist=4, iterations = 500)
rtt.planning()
rtt.find_path()

plt.show()
