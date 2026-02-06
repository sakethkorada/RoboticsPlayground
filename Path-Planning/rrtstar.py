import numpy as np
import matplotlib.pyplot as plt
import random
from environment import Environment as En
import math


class RRTStar:

    class Node: 
        def __init__(self, x, y, parent=None, cost=0):
            self.x = x
            self.y = y
            self.parent = parent
            self.cost = cost
            self.line_vis = None
            
    
    def __init__(self, startCoord, goalCoord, obstacles, expand_dist =5,interations=500):
        self.startCoord = startCoord
        self.goalCoord = goalCoord 
        self.obstacles = obstacles
        self.expand_dist = expand_dist
        self.iterations = interations
        self.node_list = []


    


    def find_nearest_node(self,x_new,y_new):
        return min(self.node_list, key=lambda n: (n.x - x_new)**2 + (n.y - y_new)**2)

    def calculate_new_node_location(self, near_node_location, random_pt):
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
    
    def find_near_neighbours(self,new_node_location):
        """
        Based on variable radius, finds all vertices that are contained in the radius of new_node
        
        :param new_node_location: Tuple of (x,y) coordinates of new node location
        """
        near_nodes = []
        def generate_min_radius():
            return 1.5 * self.expand_dist
        
        x_new, y_new = new_node_location
        radius = generate_min_radius()
        for node in self.node_list:
            x_node, y_node = node.x, node.y
            if (x_new -x_node)**2 + (y_new -y_node)**2 <= radius**2:
                near_nodes.append(node)


        return near_nodes

    def planning(self):
        #add first point
        start_node = RRTStar.Node(self.startCoord[0], self.startCoord[1])
        self.node_list.append(start_node)

        
        plt.plot(start_node.x,start_node.y, 'bo')
        plt.plot(self.goalCoord[0], self.goalCoord[1], 'ro')


        for _ in range(self.iterations):
            #find random point in environment
            random_pt = env.generate_random_pt()
            #find nearest node to random point 
            nearest_node = self.find_nearest_node(random_pt[0], random_pt[1])
            x_near, y_near = nearest_node.x, nearest_node.y
            #calcualte new node based on expand_dist
            new_node_location = self.calculate_new_node_location((x_near, y_near), random_pt)

            #check if line from nearest node and new node has collision
            #if yes continue
            #else
            is_node_valid = self.is_node_collision_free((x_near,y_near), new_node_location)
            if is_node_valid:
                #find a subset of nodes that are close enough to new point
                #find cheapest node
                #check if rewiring vertices is cheaper
                x_new, y_new = new_node_location
                near_nodes = self.find_near_neighbours(new_node_location)
                min_cost = -1
                cheapest_node = None
                for node in near_nodes:
                    #check if there is a collision
                    x_node, y_node = node.x, node.y
                    if(self.is_node_collision_free( (x_node,y_node), (x_new,y_new))):
                        node_cost = node.cost
                        new_cost =  math.sqrt( (x_node-x_new)**2 + (y_node-y_new)**2)
                        if(min_cost == -1):
                            min_cost = new_cost + node_cost
                            cheapest_node = node
                        elif(node_cost + new_cost < min_cost):
                            min_cost = new_cost + node_cost
                            cheapest_node = node
                
                newNode = RRTStar.Node(x_new,y_new, cheapest_node, min_cost)
                
                
                
                
                plt.plot(x_new, y_new, 'ko', markersize=2)
                lines = plt.plot([x_new, cheapest_node.x], [y_new, cheapest_node.y], 'k-', linewidth=0.5)
                newNode.line_vis = lines[0]
                self.node_list.append(newNode)

                for node in near_nodes:
                    #check if 
                    x_node, y_node = node.x, node.y
                    if(self.is_node_collision_free( (x_node,y_node), (x_new,y_new))):
                        node_cost = node.cost
                        new_cost =  math.sqrt( (x_node-x_new)**2 + (y_node-y_new)**2)
                        if(new_cost + min_cost < node_cost):
                            if node.line_vis is not None:
                                node.line_vis.remove()
                            node.parent = newNode
                            node.cost = new_cost + min_cost

                            new_lines = plt.plot([x_node, x_new], [y_node, y_new], 'k-', linewidth=0.5)
                            node.line_vis = new_lines[0] 

                plt.pause(0.01)
            else:
                continue 
            
            continue
        return
    


# Usage
env = En()
env.add_random_obstacles(30)
env.plot()

obstacles = env.get_obstacles()

rttstar = RRTStar( (30,30), (50,50), obstacles, expand_dist=4, interations=500)
rttstar.planning()

plt.show()